-- ------------------------------------------------------------------------------------------------------------
-- IP Name: 			onewire_temp_sense
-- Author: 				Yifeng Wang (yifenwan@phys.ethz.ch)
-- Revision: 			2.1
-- Date: 				June 8, 2023
-- Description: 		periodically read onewire sensor. Can be halted with control regs. 
-- 		Write 0x0/1 to CSR to disable/enable reading. 
-- 		Default is disable.
-- ------------------------------------------------------------------------------------------------------------

-- ==== do NOT delete me! ====
-- altera vhdl_input_version vhdl_2008 
-- ==== do NOT delete me! ====
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity onewire_temp_sense is
generic(
	SENSOR_TYPE 					: string 	:= "DS18B20";
	PARACITIC_POWERING 				: boolean 	:= true;
	BAUD_RATE 						: integer 	:= 10000;
	CONVERSION_WAIT_T_MS	 		: natural 	:= 800;
	REF_CLOCK_RATE					: natural 	:= 156250000;
	TEMPERATURE_RESOLUTION			: integer 	:= 12
	
);
port (
	-- to temp sensor module signal
	io_dq					: inout std_logic; -- output to 18B20 DQ pin 2v5.  Will be converted by DAB to 3.3V.
	
	-- avalon memory-mapped slave port
	-- sensor data
	avs_sense_write			: in  std_logic;
	avs_sense_writedata		: in  std_logic_vector(31 downto 0);
	avs_sense_read			: in  std_logic;
	avs_sense_readdata		: out std_logic_vector(31 downto 0);
	avs_sense_waitrequest	: out std_logic;
	
	-- clock and reset interface
	i_clk					: in	std_logic; -- input clock
	i_rst_n					: in	std_logic  -- input reset signal reset on low
	
);
end entity;

architecture rtl of onewire_temp_sense is
	-- command codes of DS18B20
	record DS18B20_CMD is 
		ROM_SKIP						: std_logic_vector := std_logic_vector(to_unsigned(16#CC#,8));
		CONVERT							: std_logic_vector := std_logic_vector(to_unsigned(16#44#,8));
		READ_TEMP						: std_logic_vector := std_logic_vector(to_unsigned(16#BE#,8));
	end record;

	-- clock and counters
	signal pseudo_clk_1m				: std_logic;
	signal tick_1m						: std_logic;
	
	signal clk_1us						: std_logic;
	signal cnt_1us_en					: std_logic;
	signal cnt_tmp						: std_logic_vector(7 downto 0);
	signal cnt_1us						: unsigned(19 downto 0);
	signal bit_cnt						: integer := 0; -- from 0 to 2^4-1(15)
	signal byte_cnt						: integer := 0;
	signal cmd_cnt						: integer := 0;

	-- state machine flags
	type state_t is (init,ROM_SKIP,wr_byte,temp_convert,delay,rd_temp,rd_byte,idle);--, ROM_SKIP, wr_byte, temp_convert, delay, rd_temp, rd_byte);
	signal curr_st						: state_t;--std_logic_vector(2 downto 0);
	signal next_st						: state_t;--std_logic_vector(2 downto 0);
	type flow_t is (s0, s1, s2, s3 ,s4, s5, s6);
	signal flow_step 					: flow_t := s0;

	signal init_done_flg				: std_logic := '0';
	signal conv_done_flg				: std_logic := '0';
	signal wr_done_flg					: std_logic := '0';
	signal rd_done_flg					: std_logic := '0';
	signal timeout_flg					: std_logic := '0';
	signal present_flg					: std_logic := '0';

	signal FSM_err						: std_logic := '0';

	-- registers
	signal wr_data						: std_logic_vector (7 downto 0);

	type schpad_DS18B20_t is array (0 to 8) of std_logic_vector (7 downto 0);
	signal rd_data						: schpad_DS18B20_t;
	signal rd_data_bit					: std_logic;
	signal CRC_lfsr						: std_logic_vector(7 downto 0);
	signal CRC_en						: std_logic := '1';
	signal CRC_rst_n					: std_logic := '1';
	signal crc_err						: std_logic := '0';
	signal halt_reading_ctrl			: std_logic := '0';
	signal wait_for_new_reading			: std_logic := '0';

	-- tri-state I/O to DS18B20
	signal dq_z							: std_logic;
	
	-- for onewire-avm
	signal driver_status				: std_logic_vector (7 downto 0);
	signal driver_control				: std_logic_vector (7 downto 0);
	signal temp_reading_f32				: std_logic_vector(31 downto 0);
	signal temp_reading_f32_c			: std_logic_vector (31 downto 0);
	signal temp_bitshift				: integer := 0;
	signal temp_sign_bitfield			: std_logic_vector (4 downto 0);
	signal temp_data_bitfield			: std_logic_vector (10 downto 0);
	
	--signal avs_ow_address_int			: std_logic_vector(2 downto 0);
	signal avs_ow_write_int				: std_logic;
	signal avs_ow_writedata_int			: std_logic_vector(31 downto 0);
	signal avs_ow_read_int				: std_logic;
	signal avs_ow_readdata_int			: std_logic_vector(31 downto 0);
	signal avs_ow_waitrequest_int		: std_logic;
	--signal avs_ow_begintransfer_int		: std_logic;
	
	-- comb wire (lagency, for debug only)
	signal o_reading_c					: std_logic_vector(31 downto 0);
		
begin

	-- Assignment
	io_dq <= dq_z;
	avs_ow_write_int <= avs_ow_write;
	avs_ow_writedata_int <= avs_ow_writedata;
	avs_ow_read_int <= avs_ow_read;
	avs_ow_readdata <= avs_ow_readdata_int;
	avs_ow_waitrequest <= avs_ow_waitrequest_int;
	
	
	-- Register Mapping
	driver_status(0) <= present_flg;
	driver_status(1) <= crc_err;  
	driver_status(6) <= not (init_done_flg and conv_done_flg and wr_done_flg and rd_done_flg); -- trans not finished
	driver_status(7) <= FSM_err;
	
	halt_reading_ctrl 		<= not driver_control(0);
	wait_for_new_reading 	<= '0';
	
	-- ---------------------------------
	-- slow_clock_convertor
	-- ---------------------------------
	-- some checks
	assert REF_CLOCK_RATE >= 1000_000 report "reference clock rate must be larger than 1MHz!" severity error;
	
	-- instantiation
	e_sensor_slow_clock : entity work.pseudo_clock_down_convertor
	-- down convert ref clock to sensor link layer clock rate (1 MHz)
	generic map ( 
		CLK_DOWN_FACTOR => REF_CLOCK_RATE / 1000000; -- NOTE: ref clock must be larger than 1MHz
	)
	port map ( 
		-- input fast clock and reset interface
		i_clk 			=> i_clk, -- input clock
		i_reset_n 		=> i_rst_n,	-- input reset
		-- pseudo slow clock
		o_clk		 	=> pseudo_clk_slow,
		-- slow tick (active for one fast cycle at the rising edge of the slow clock)
		o_tick			=> slow_tick
	);
	
	
	-- -----------------------------------------------------
	-- crc_checker
	-- -----------------------------------------------------
	-- instantiation
	e_crc_checker : entity work.crc_checker_maxim_crc8
	-- 8-bit, poly x^8 + x^5 + x^4 + 1
	port map (
			-- control interface
			i_shift_en		=> crc_checker_shift_en,
			-- data interface
			i_din			=> rd_data(byte_cnt)(bit_cnt),
			o_lfsr			=> open,
			o_crc_error		=> crc_checker_error,-- it is your responsibility to check error at the right cycle, otherwise it is invalid. 
			-- clock and reset interface
			i_clk			=> i_clk,
			i_rst_n			=> i_rst_n
	);
	
	-- ------------------------------------
	-- slow_counter
	-- ------------------------------------
	signal slow_counter_cnt_unsigned		: unsigned(31 downto 0);
	signal slow_counter_cnt					: integer;
	signal slow_counter_on					: std_logic;
	slow_counter_cnt		<= to_integer(slow_counter_cnt_unsigned);
	proc_slow_counter : process(i_rst_n,i_clk)
	-- runs at 1MHz
	begin
		if rising_edge(i_clk) then
			if (i_rst_n /= '1' or slow_counter_on = '0') then
				slow_counter_cnt_unsigned 	<= (others => '0'); -- reset the count when off
			else
				if (slow_tick = '1') then -- count up at 1 MHz rate
					slow_counter_cnt_unsigned 	<= slow_counter_cnt_unsigned + 1;
				end if;
			end if;
		end if;
	end process;

	
	
	-- -----------------------------------------
	-- avalon_slave_interface
	-- -----------------------------------------
	type sensor_exception_t is record
		crc_err					: std_logic;
		lost_connection			: std_logic;
	end record;
	type csr_t is record
		read_enable				: std_logic;
		error					: sensor_exception_t;
		show_hidden_page		: std_logic;
		temperature				: std_logic_vector(31 downto 0);
	end record;
	signal csr					: csr_t;
	
	proc_avalon_slave_interface: process(i_clk, i_rst_n)
	begin
		if rising_edge(i_clk) then
			if (i_rst_n /= '1') then 
				avs_sense_waitrequest		<= '0';
			else 
				-- default
				avs_sense_readdata			<= (others => '0');
				-- main interface logic 
				if (avs_sense_read = '1') then 
					avs_sense_waitrequest		<= '0';
					case (csr.show_hidden_page) is  -- normal op
						when 0 =>
							avs_sense_readdata			<= csr.temperature;
						when 1 =>
							avs_sense_readdata(4)		<= csr.error.crc_err;
							avs_sense_readdata(0)		<= csr.error.lost_connection;
						when others =>
					end case;
				elsif (avs_sense_write = '1') then 
					avs_sense_waitrequest			<= '0';
					csr.show_hidden_page			<= avs_sense_writedata(8); -- you can turn on the hidden csr page for debugging 
				else 
				-- routine to update
					avs_sense_waitrequest		<= '1';		
				end if;
			end if;
		end if;
	end process;
	
	type link_msg_t is record
		sensor_present		: std_logic;
		init_timeout		: std_logic;
		init_ok				: std_logic;
		write_done			: std_logic;
		progress_bar		: integer;
	end record;
	signal link_msg			: link_msg_t;
	
	-- -------------------------------------
	-- pipeline_fsm
	-- -------------------------------------
	proc_pipeline_fsm : process(i_clk,i_rst_n)
	-- update the *_slow state at the slow tick
	-- need to sync these slow state, so the counter will also be sync, otherwise for short counts, it could be undefined. 
	begin
		if (rising_edge(i_clk) then
			if (i_rst_n /= '1') then 
				sensor_link_state			<= INIT;
				init_flow					<= RESET;
			else 
				if (slow_tick = '1') then 
					sensor_link_state			<= sensor_link_state_next;
					init_flow					<= init_flow_next;
				end if;
			end if;
		end if;
	end process;
	
	type init_flow_t is (RESET,MASTER_INIT,MASTER_WAIT,DETECTING_SLAVE,EVAL);
	signal init_flow			: init_flow_t;
	signal init_flow_next		: init_flow_t;
	
	type sensor_link_state_t is (INIT,LOAD_ROM_SKIP,
	signal sensor_link_state		: sensor_link_state_t;
	signal sensor_link_state_next	: sensor_link_state_t;
	
	-- ---------------------------------
	-- sensor_link_fsm
	-- ---------------------------------
	proc_sensor_link_fsm : process(i_clk,i_rst_n)
	-- 1-Wire protocol and sensor readout communication link layer handler, perform fully automatic initialization and readout.
	begin
		if rising_edge(i_clk) then 
			if (i_rst_n /= '1') then
			
			else 
				case sensor_link_state is 
					when INIT => 
						case init_flow is 
							when RESET => -- start here
								slow_counter_on 		<= '0';
								init_flow_next			<= MASTER_INIT;
							when MASTER_INIT => -- master pull low for 500us, then release the line
								if (slow_counter_cnt < 500) then 
									dq_z 					<= '0';
									slow_counter_on 		<= '1';
								else 
									dq_z 					<= 'Z';
									slow_counter_on 		<= '0';
									init_flow_next			<= MASTER_WAIT;
								end if;
							when MASTER_WAIT => -- master release the line, then wait for 45us
								if (slow_counter_cnt < 45) then 
									dq_z					<= 'Z';
									slow_counter_on			<= '1';
								else
									slow_counter_on			<= '0';
									init_flow_next			<= DETECTING_SLAVE;
								end if;
							when DETECTING_SLAVE => -- master detect the slave response
								slow_counter_on			<= '1';
								dq_z 					<= 'Z';
								if (io_dq = '0') then -- detected DS18B20 presense pulse
									link_msg.sensor_present	<= '1'; -- detected flag
									link_msg.init_timeout	<= '0'; -- clear init-timeout flag
									slow_counter_on			<= '0';
									init_flow_next			<= EVAL;
								else -- wait until pulse of timeout at 5000us
									if (slow_counter_cnt >= 5000) then 
										link_msg.sensor_present	<= '0'; 
										link_msg.init_timeout	<= '1';
										slow_counter_on			<= '0';
										init_flow_next			<= EVAL;
									end if;
								end if;
							when EVAL => -- check if init was successful
								slow_counter_on			<= '1';
								if (link_msg.init_timeout = '1') then 
									if (slow_counter_cnt = 1000_000) then -- wait for 1s to re-init
										init_flow_next			<= RESET;
									end if;
								else
									if (slow_counter_cnt > 500) then 
										link_msg.init_ok		<= '1';
										slow_counter_on			<= '0';
										sensor_link_state_next	<= LOAD_ROM_SKIP;
										init_flow_next			<= RESET;
									end if;
								end if;
							when others =>
						end case;
					when LOAD_ROM_SKIP => 
						master_tx_data 				<= DS18B20_CMD.ROM_SKIP;
						dq_z 						<= 'Z';
						slow_counter_on				<= '0';
						link_msg.write_done			<= '0';
						sensor_link_state_next		<= MASTER_TX_BYTE; 
					when MASTER_TX_BYTE => 
						case master_tx_flow is 
							when RESET =>
								dq_z					<= '0';
								slow_counter_on			<= '0';
								master_tx_flow			<= TX_BIT;
							when TX_BIT =>
								slow_counter_on			<= '1';
								if (slow_counter_cnt < 60+20) then -- manual: tx bit between 60-120 us
									dq_z					<= master_tx_data(master_tx_bit_cnt);
								elsif (slow_counter_cnt < 60+20+10) then -- manual: release for another 10us, between bits
									dq_z					<= 'Z';
								else 
									master_tx_bit_cnt		<= master_tx_bit_cnt + 1;
									slow_counter_on			<= '0';
									master_tx_flow			<= RESET;
									if (master_tx_bit_cnt = 7) then -- this byte tx is done
										link_msg.progress_bar			<= link_msg.progress_bar + 1; -- incr the command count
										case link_msg.progress_bar is -- decide which link state should go next
											when 0 => 
												sensor_link_state_next		<= LOAD_CONVERT;
											when 1 =>
												sensor_link_state_next		<= HARD_PULL_HIGH;
											when 2 =>
												sensor_link_state_next		<= LOAD_READ_TEMP;
											when 3 =>
												sensor_link_state_next		<= MASTER_RX_BYTE;
											when others =>
										end case;
									end if;
								end if;
							when others =>
						end case;
										
										
						
									
								
			
			end if;
		end if;
		
		

	
		elsif rising_edge(clk_1us) then
			case curr_st is
	
				when wr_byte => -- master tx: starting from LSB
					if (bit_cnt <= 7) then
						wr_done_flg <= '0';
						case flow_step is
							when s0 =>
								dq_z <= '0';
								cnt_1us_en <= '1';
								flow_step <=s1;
							when s1 =>
								if (cnt_1us < to_unsigned(60+20,cnt_1us'length)) then -- tx bit between 60-120 us
									dq_z <= wr_data(bit_cnt);
								elsif (cnt_1us < to_unsigned(60+20+10,cnt_1us'length)) then -- release for another 10us
									dq_z <= 'Z';
								else -- write another bit
									bit_cnt <= bit_cnt + 1;
									flow_step <= s0;
									cnt_1us_en <= '0';
										if (bit_cnt = 7) then -- if last bit, increase cmd_cnt
											cmd_cnt <= cmd_cnt + 1;
										end if;
								end if;
							when others =>
								flow_step <=s0;
						end case;
					else -- finish write byte
						case flow_step is
							when s0 =>
								cnt_1us_en <= '1';
								if (cnt_1us > to_unsigned(100,cnt_1us'length)) then -- wait for 100us after each command byte write
									flow_step <= s1;
									cnt_1us_en <= '0';
								end if;
							when s1 =>
								flow_step <= s0;
								bit_cnt <= 0;
								cnt_1us_en <= '0';
								wr_done_flg <= '1';
								if (cmd_cnt = 1) then
									next_st <= temp_convert;
								elsif (cmd_cnt = 2) then
									next_st <= delay;
								elsif (cmd_cnt = 3) then
									next_st <= rd_temp;
								else
									next_st <= rd_byte;
								end if;
							when others =>
								flow_step <= s0;
						end case;
					end if;
	
				when temp_convert =>
					wr_data 			<= DS18B20_CMD.CONVERT;
					wr_done_flg 		<= '0';
					next_st 			<= wr_byte;
	
				when delay =>
					cnt_1us_en <= '1';
					dq_z <= '1'; -- pull high release during temp conversion
					if (cnt_1us = to_unsigned(CONVERSION_WAIT_T_MS*1000,cnt_1us'length)) then -- delay 800ms
						cnt_1us_en <= '0';
						conv_done_flg <= '1';
						init_done_flg <= '0';
						next_st <= init; -- goto rd_temp state
					end if;
	
				when rd_temp =>
					wr_data 			<= DS18B20_CMD.READ_TEMP
					wr_done_flg 		<= '0';
					next_st 			<= wr_byte;
	
				when rd_byte => -- master rx: LSB first from byte 0 to 8
					case flow_step is
						when s0 => -- wait 500 us after sending READ_TEMP command
							if (wr_done_flg = '1') then
								cnt_1us_en <= '1';
								if (cnt_1us >= to_unsigned(500,cnt_1us'length)) then
									flow_step <= s1;
									bit_cnt <= 0; -- reset its counters and flags
									byte_cnt <= 0;
									wr_done_flg <= '0';
								end if;
							elsif (bit_cnt = 0) then -- only wait for 100us between bytes
								cnt_1us_en <= '1';
								if (cnt_1us >= to_unsigned(100,cnt_1us'length)) then
									flow_step <= s1;
								end if;
							else -- only wait for 20us between bits
								cnt_1us_en <= '1';
								if (cnt_1us >= to_unsigned(20,cnt_1us'length)) then
									flow_step <= s1;
								end if;
							end if;
						when s1 => -- reset counter
							cnt_1us_en <= '0';
							flow_step <= s2;
						when s2 => -- pull low 5us: master start read time slot
							cnt_1us_en <= '1';
							dq_z <= '0';
							if (cnt_1us >= to_unsigned(5,cnt_1us'length)) then
								flow_step <= s3;
							end if;
						when s3 =>
							dq_z <= 'Z';
							if (cnt_1us = to_unsigned(15,cnt_1us'length)) then -- master rx samples at 15us after read time slot started
								rd_data(byte_cnt)(bit_cnt) <= io_dq;
								flow_step <= s4;
							end if;
						when s4 =>
							if (cnt_1us > to_unsigned(60,cnt_1us'length)) then -- wait until read time slot ends
								flow_step <= s0;
								cnt_1us_en <= '0';
								if (bit_cnt < 7) then
									bit_cnt <= bit_cnt + 1;
								elsif (bit_cnt = 7) then
									byte_cnt <= byte_cnt + 1;
									bit_cnt <= 0;
									if (byte_cnt = 8) then					 -- byte 8 - bit 7
										rd_done_flg <= '1';
										next_st <= idle;
										flow_step <= s0;
										cnt_1us_en <= '0';
										dq_z <= 'Z';
									end if;
								end if;
							end if;
						when others =>
							flow_step <= s0;
					end case;
	
				when idle =>
					case flow_step is
						when s0 => -- export reading to conduit-end
							o_reading_c(7 downto 0) <= rd_data(0)(7 downto 0);
							o_reading_c(15 downto 8) <= rd_data(1)(7 downto 0);
							init_done_flg <= '0';
							wr_done_flg <= '0';
							dq_z <= 'Z';
							bit_cnt <= 0;
							byte_cnt <= 0;
							cmd_cnt <= 0;
							cnt_1us_en <= '0';
							CRC_rst_n <= '0';
							flow_step <= s1;
							temp_reading_f32	<= temp_reading_f32_c; -- register the comb out while it is stable.
						when s1 =>		-- reset things
							CRC_en <= '0';
							flow_step <= s2;
							CRC_rst_n <= '0'; -- reset the CRC check entity's reg
							bit_cnt <= 0;
							byte_cnt <= 0;
						when s2 => -- crc check stage
							CRC_rst_n <= '1';
							CRC_en <= '1';
							if (byte_cnt = 9) then -- end at byte 7 last bit, before CRC byt
								flow_step <= s3;
								-- CRC pass flag: 0=pass, 1=fail
								crc_err <= 	CRC_lfsr(7) or CRC_lfsr(6) or CRC_lfsr(5) or
												CRC_lfsr(4) or CRC_lfsr(3) or CRC_lfsr(2) or
												CRC_lfsr(1) or CRC_lfsr(0) ;
							else
								rd_data_bit <= rd_data(byte_cnt)(bit_cnt); -- pass into the shift_reg
								if (bit_cnt < 7) then		-- update counters
									bit_cnt <= bit_cnt + 1;
								elsif (bit_cnt = 7) then
									byte_cnt <= byte_cnt + 1;
									bit_cnt <= 0;
								end if;
							end if;
						when s3 => -- toggle for av-mm read transaction
							cnt_1us_en <= '1';
							if (cnt_1us >= to_unsigned(100,cnt_1us'length)) then
								flow_step <= s4;
							end if;
						when s4 => -- clearing flags for next around of reading
							init_done_flg <= '0';
							conv_done_flg <= '0';
							wr_done_flg <= '0';
							rd_done_flg <= '0';
							present_flg <= '0';
							FSM_err <= '0';
							dq_z <= '1';
							
							bit_cnt <= 0;
							byte_cnt <= 0;
							cmd_cnt <= 0;
							cnt_1us_en <= '0';
							rd_data <=  (OTHERS => (OTHERS => '0'));
							CRC_en <= '1';
							CRC_rst_n  <= '1';
							if (halt_reading_ctrl='1') then
								next_st <= idle;
								flow_step <= s4;
							else 
								next_st <= init;
								flow_step <= s0;
							end if;
						when others =>
							flow_step <= s0;
					end case;
	
				when others =>
					next_st <= init;
			end case;
		end if;
	end process proc_main;
	-- === Onewire Driver PHY and TRANS Layer END === 
	
	-- === Converting BEGIN ===
	-- converting from DS18B20 reading to float-32 (IEEE754-Single Precision Binary32 Floating Point)
	-- Ref: https://en.wikipedia.org/wiki/Single-precision_floating-point_format
	process(temp_data_bitfield,rd_data,temp_sign_bitfield)
	begin
		temp_sign_bitfield <= rd_data(1)(7 downto 3);
		temp_data_bitfield <= rd_data(1)(2 downto 0) & rd_data(0);
		temp_reading_f32_c(31) <= temp_sign_bitfield(0); -- sign bit
		if (temp_data_bitfield(10)='1') then
			temp_bitshift <= 6;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(9 downto 0) & std_logic_vector(to_unsigned(0,13));
		elsif (temp_data_bitfield(9)='1') then
			temp_bitshift <= 5;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(8 downto 0) & std_logic_vector(to_unsigned(0,14));
		elsif (temp_data_bitfield(8)='1') then
			temp_bitshift <= 4;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(7 downto 0) & std_logic_vector(to_unsigned(0,15));
		elsif (temp_data_bitfield(7)='1') then
			temp_bitshift <= 3;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(6 downto 0) & std_logic_vector(to_unsigned(0,16));
		elsif (temp_data_bitfield(6)='1') then
			temp_bitshift <= 2;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(5 downto 0) & std_logic_vector(to_unsigned(0,17));
		elsif (temp_data_bitfield(5)='1') then
			temp_bitshift <= 1;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(4 downto 0) & std_logic_vector(to_unsigned(0,18));
		elsif (temp_data_bitfield(4)='1') then
			temp_bitshift <= 0;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(3 downto 0) & std_logic_vector(to_unsigned(0,19));
		elsif (temp_data_bitfield(3)='1') then
			temp_bitshift <= -1;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(2 downto 0) & std_logic_vector(to_unsigned(0,20));
		elsif (temp_data_bitfield(2)='1') then
			temp_bitshift <= -2;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(1 downto 0) & std_logic_vector(to_unsigned(0,21));
		elsif (temp_data_bitfield(1)='1') then
			temp_bitshift <= -3;
			temp_reading_f32_c(22 downto 0) <= temp_data_bitfield(0) & std_logic_vector(to_unsigned(0,22));
		else
			temp_bitshift <= 0;
			temp_reading_f32_c(22 downto 0) <= (others=> '0');
		end if;	
			
	end process;
	
	process (temp_bitshift)
	begin
		temp_reading_f32_c(30 downto 23) <=  std_logic_vector(to_unsigned(temp_bitshift+127,8));-- exponent
	end process;
	-- === Converting END ===

end architecture;
