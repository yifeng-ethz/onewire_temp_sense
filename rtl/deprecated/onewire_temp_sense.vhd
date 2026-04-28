-- ------------------------------------------------------------------------------------------------------------
-- IP Name:             onewire_temp_sense
-- Author:              Yifeng Wang (yifenwan@phys.ethz.ch)
-- Revision:            3.0
-- Date:                June 8, 2023
-- Date:                Aug 28, 2024 (re-design the whole link layer)
-- Description:         periodically read onewire sensor. Can be halted with control regs.
-- Usage:                
-- 		                Write 0x0/1 to CSR to disable/enable reading. address 0 is the temperature in float-32.
--                      Otherwise, this IP is fully automatic. 
-- 		                Default is reading disabled.
-- ------------------------------------------------------------------------------------------------------------

-- ================ synthsizer configuration =================== 	
-- altera vhdl_input_version vhdl_2008 
-- ============================================================= 	

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity onewire_temp_sense is
generic(
	SENSOR_TYPE 					: string 	:= "DS18B20"; -- currently control procedure is optimized for this type of sensor, changing to other requires a thorough review
	PARACITIC_POWERING 				: boolean 	:= false; -- depending on your periphial circuitry. refer to inline code for details.  
	BAUD_RATE 						: integer 	:= 10000; -- 10kbps is norminal, user can slow down it by modifying the master tx/rx timing
	CONVERSION_WAIT_T_MS	 		: natural 	:= 800; -- can be reduced if user set the conversion resolution to lower
	REF_CLOCK_RATE					: natural 	:= 156250000; -- can be tuned, must be greater than 1 MHz. all parts are synchronized/constraint under this clock. 
	TEMPERATURE_RESOLUTION			: integer 	:= 12 -- not supported to tune yet, needs an additional sensor register access
);
port (
	-- to temp sensor module signal
	io_dq					: inout std_logic; -- output to 18B20 DQ pin 2v5 (fpga).  Will be converted by DAB to 3.3V.
	
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
	-- -----------------
	-- generic
	-- -----------------
	-- command codes of DS18B20
	type DS18B20_CMD_T is record
		ROM_SKIP							: std_logic_vector(7 downto 0);
		CONVERT								: std_logic_vector(7 downto 0);
		READ_TEMP							: std_logic_vector(7 downto 0);
	end record;
	constant DS18B20_CMD					: DS18B20_CMD_T := (ROM_SKIP => std_logic_vector(to_unsigned(16#CC#,8)), CONVERT => std_logic_vector(to_unsigned(16#44#,8)), READ_TEMP => std_logic_vector(to_unsigned(16#BE#,8)));
	-- global constants
	constant SCRATCH_PAD_BYTES				: natural := 9;

	-- ---------------------------------
	-- slow_clock_convertor
	-- ---------------------------------
	signal pseudo_clk_slow					: std_logic;
	signal slow_tick						: std_logic;
		
	-- -----------------------------------------------------
	-- crc_checker
	-- -----------------------------------------------------
	type crc_checker_t is record
		bit_idx						: natural range 0 to 7;
		byte_idx					: natural range 0 to SCRATCH_PAD_BYTES-1;
		shift_en					: std_logic;
		error						: std_logic;
		sclr						: std_logic;
	end record;
	signal crc_checker				: crc_checker_t;
	
	-- ------------------------------------
	-- slow_timer
	-- ------------------------------------
	signal slow_timer_cnt_unsigned			: unsigned(31 downto 0);
	signal slow_timer_cnt					: integer range 0 to 2**32-1;
	signal slow_timer_on					: std_logic; -- turning it off, will clear its value
	
	-- -----------------------------------------
	-- avalon_slave_interface
	-- -----------------------------------------
	-- types 
	type sensor_exception_t is record
		crc_err								: std_logic;
		lost_connection						: std_logic;
	end record;
	type csr_t is record
		read_enable							: std_logic;
		temperature							: std_logic_vector(31 downto 0);
		show_hidden_page					: std_logic;
		error								: sensor_exception_t;
	end record;
	
	-- signals 
	signal csr								: csr_t;
	
	-- ---------------------------------
	-- sensor_link_fsm
	-- ---------------------------------
	-- types
	type link_msg_t is record -- these done flags are used within this state machine 
		read_go						: std_logic; -- updated by csr, allow one read after power-up
		init_done					: std_logic; -- when init_done is high, the sensor_present is valid.
		sensor_present				: std_logic; 
		init_timeout				: std_logic; -- it is complimentary to init_done
		write_done					: std_logic; 
		progress_bar				: integer range 0 to 7;
		read_done					: std_logic;
		crc_err						: std_logic; -- valid at REPORTING state
		reading_f32					: std_logic_vector(31 downto 0); -- valid at REPORTING state
	end record;
	type init_flow_t 				is (RESET,MASTER_INIT,MASTER_WAIT,DETECTING_SLAVE,EVAL);
	type master_tx_flow_t 			is (RESET,TX_BIT);
	type master_rx_flow_t 			is (RX_EVAL,RX_BIT);
	type sensor_link_state_t 		is (INIT,LOAD_ROM_SKIP,MASTER_TX_BYTE,LOAD_CONVERT,WAIT_FOR_SENSOR_CONV,LOAD_READ_TEMP,MASTER_RX_BYTE,CRC_CHECKING,REPORTING,RESET);
	type master_tx_t is record
		data						: std_logic_vector(7 downto 0);
		bit_idx						: natural range 0 to 7;
	end record;
	type ram_8b_t					is array (integer range <>) of std_logic_vector(7 downto 0);
	subtype ds18b20_scratch_pad_t	is ram_8b_t(0 to SCRATCH_PAD_BYTES-1);
	type master_rx_t is record
		data						: ds18b20_scratch_pad_t;
		bit_idx						: natural range 0 to 7; 
		byte_idx					: natural range 0 to SCRATCH_PAD_BYTES-1;
	end record;
	
	-- signals
	signal link_msg					: link_msg_t;
	signal init_flow				: init_flow_t;
	signal init_flow_next			: init_flow_t;
	signal master_tx_flow			: master_tx_flow_t;
	signal master_tx_flow_next		: master_tx_flow_t;
	signal master_rx_flow			: master_rx_flow_t;
	signal master_rx_flow_next		: master_rx_flow_t;
	signal sensor_link_state		: sensor_link_state_t;
	signal sensor_link_state_next	: sensor_link_state_t;
	signal master_tx				: master_tx_t;
	signal master_rx				: master_rx_t;


	-- ---------------------------
	-- ds18b20_to_float
	-- ---------------------------
	signal temp_reading_f32					: std_logic_vector(31 downto 0);
	
	-- ----------------
	-- pipes
	-- ----------------
	type link2avs_t is record 
		start				: std_logic;
		done				: std_logic;
	end record;
	type pipe_t is record
		link2avs			: link2avs_t;
	end record;
	signal pipe				: pipe_t;
	
	-- -----------------------------
	-- tri_state
	-- -----------------------------
	signal dq_z							: std_logic;
	
begin

	-- ---------------------------------
	-- slow_clock_convertor
	-- ---------------------------------
	-- checks
	assert REF_CLOCK_RATE >= 1000_000 report "reference clock rate must be larger than 1MHz!" severity error;
	
	-- instantiation
	e_sensor_slow_clock : entity work.pseudo_clock_down_convertor
		-- down convert ref clock to sensor link layer clock rate (1 MHz)
	generic map ( 
		CLK_DOWN_FACTOR => REF_CLOCK_RATE / 1000000 -- NOTE: ref clock must be larger than 1MHz
	)
	port map ( 
		-- input fast clock and reset interface
		i_clk 			=> i_clk, -- input clock
		i_reset_n 		=> i_rst_n,	-- input reset
		-- pseudo slow clock
		o_clk		 	=> pseudo_clk_slow, -- so far not used, can be slow down the overall state machine to release more timing slack
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
			i_shift_en		=> crc_checker.shift_en,
			-- data interface
			i_din			=> master_rx.data(crc_checker.byte_idx)(crc_checker.bit_idx),
			o_lfsr			=> open,
			o_crc_error		=> crc_checker.error,-- it is your responsibility to check error at the right cycle, otherwise it is invalid. 
			-- clock and reset interface
			i_clk			=> i_clk,
			i_rst			=> not(i_rst_n) or crc_checker.sclr
	);
	
	
	
	-- ------------------------------------
	-- slow_timer
	-- ------------------------------------
	proc_slow_timer : process(i_rst_n,i_clk)
	-- runs at 1MHz
	begin
		if rising_edge(i_clk) then
			if (i_rst_n /= '1' or slow_timer_on = '0') then
				slow_timer_cnt_unsigned 	<= (others => '0'); -- reset the count when off
			else
				if (slow_tick = '1') then -- count up at 1 MHz rate
					slow_timer_cnt_unsigned 	<= slow_timer_cnt_unsigned + 1;
				end if;
			end if;
		end if;
	end process;
	
	proc_slow_timer_comb : process (all)
	begin
		slow_timer_cnt		<= to_integer(slow_timer_cnt_unsigned);
	end process;
	
	
	
	-- -----------------------------------------
	-- avalon_slave_interface
	-- -----------------------------------------
	proc_avalon_slave_interface: process(i_clk, i_rst_n)
		-- +-----------------------------------------------------------
		-- | memory map
		-- | ==========================================================
		-- | Address | Byte 3 | Byte 2 | Byte 1 | Byte 0 | Note
		-- |                   **READ**
		-- |-----------------------------------------------------------
		-- |    0    |           temperature             | Temperature of the sensor in IEEE754 32-bit format.
		-- |0(hidden)|  n/a   |  n/a   | crc_err| lost_cn| crc_err(error): crc error is possible as temperature is chaning rapidly (e.g. during asic power up/down).
		-- |                                           ... lost_cn(connection): might due to cable loose or sensor not powered correctly. 
		-- |-----------------------------------------------------------
		-- |                   **WRITE**
		-- |   0     |  n/a   |  n/a   | show_hd| read_en| read_en(enable): can halt the sensor reading procedure, useful to elliminate EMI/ground crosstalk
		-- |                                           ... show_hd(hidden_page): set to '1' to read the hidden csr page for debugging (e.g. crc_err and lost_cn). 
		-- +-----------------------------------------------------------
	begin
		if rising_edge(i_clk) then
			if (i_rst_n /= '1') then 
				avs_sense_waitrequest		<= '0';
				pipe.link2avs.done			<= '0';
				-- reset csr
				csr.read_enable				<= '0';
				csr.temperature				<= (others => '0');
				csr.show_hidden_page		<= '0';
				csr.error.crc_err			<= '0';
				csr.error.lost_connection	<= '0';
				
			else 
				-- default
				avs_sense_readdata			<= (others => '0');
				-- main interface logic 
				if (avs_sense_read = '1') then 
					avs_sense_waitrequest		<= '0';
					case csr.show_hidden_page is  -- normal op
						when '0' =>
							avs_sense_readdata			<= csr.temperature;
						when '1' =>
							avs_sense_readdata(4)		<= csr.error.crc_err;
							avs_sense_readdata(0)		<= csr.error.lost_connection;
						when others =>
					end case;
				elsif (avs_sense_write = '1') then 
					avs_sense_waitrequest			<= '0';
					csr.show_hidden_page			<= avs_sense_writedata(8); -- you can turn on the hidden csr page for debugging 
					csr.read_enable					<= avs_sense_writedata(0);					
				else 
					avs_sense_waitrequest		<= '1';	
					-- routine to update
					if (link_msg.init_done = '1') then -- latch the present flag
						csr.error.lost_connection	<= not link_msg.sensor_present;
					end if;
					-- inter-fsm communication (ISC)
					if (pipe.link2avs.start = '1' and pipe.link2avs.done = '0') then -- start ISC
						csr.error.crc_err			<= link_msg.crc_err;
						csr.temperature				<= link_msg.reading_f32;
						pipe.link2avs.done 			<= '1';
					elsif (pipe.link2avs.start = '1' and pipe.link2avs.done = '1') then -- wait for master to ack
						-- idle
					elsif (pipe.link2avs.start = '0' and pipe.link2avs.done = '1') then
						pipe.link2avs.done			<= '0';
					else
						-- pure idle
					end if;
				end if;
			end if;
		end if;
	end process;
	
	
	
	-- -------------------------------------
	-- sync_fsm
	-- -------------------------------------
	proc_sync_fsm : process(i_clk,i_rst_n)
	-- update the *_slow state at the slow tick
	-- need to sync these slow state, so the counter will also be sync, otherwise for short counts, it could be undefined. 
	begin
		if (rising_edge(i_clk)) then
			if (i_rst_n /= '1') then 
				sensor_link_state				<= RESET;
			else
				if (slow_tick = '1') then -- sync all states transitions to slow tick
					sensor_link_state			<= sensor_link_state_next;
					init_flow					<= init_flow_next;
					master_tx_flow				<= master_tx_flow_next;
					master_rx_flow				<= master_rx_flow_next;
				end if;
			end if;
		end if;
	end process;
	
	
	
	-- ---------------------------------
	-- sensor_link_fsm
	-- ---------------------------------
	proc_sensor_link_fsm : process(i_clk)
	-- 1-Wire protocol and sensor readout communication link layer handler, perform fully automatic initialization and readout.
	begin
		if rising_edge(i_clk) then 
			case sensor_link_state is 
				when INIT => 
					case init_flow is 
						when RESET => -- start here
							slow_timer_on 			<= '0';
							link_msg.init_done 		<= '0';
							if (link_msg.init_done = '0' and link_msg.read_go = '1') then -- may go
								init_flow_next			<= MASTER_INIT;
							else 
								init_flow_next			<= RESET;
							end if;
						when MASTER_INIT => -- master pull low for 500us, then release the line
							if (slow_timer_cnt < 500) then 
								dq_z 					<= '0';
								slow_timer_on 			<= '1';
							else 
								dq_z 					<= 'Z';
								slow_timer_on 			<= '0';
								init_flow_next			<= MASTER_WAIT;
							end if;
						when MASTER_WAIT => -- master release the line, then wait for 45us
							if (slow_timer_cnt < 45) then 
								dq_z					<= 'Z';
								slow_timer_on			<= '1';
							else
								slow_timer_on			<= '0';
								init_flow_next			<= DETECTING_SLAVE;
							end if;
						when DETECTING_SLAVE => -- master detect the slave response
							slow_timer_on			<= '1';
							dq_z 					<= 'Z';
							if (io_dq = '0') then -- detected DS18B20 presense pulse
								link_msg.sensor_present	<= '1'; -- detected flag
								link_msg.init_timeout	<= '0'; -- clear init-timeout flag
								slow_timer_on			<= '0';
								init_flow_next			<= EVAL;
							else -- wait until pulse of timeout at 5000us
								if (slow_timer_cnt >= 5000) then 
									link_msg.sensor_present	<= '0'; 
									link_msg.init_timeout	<= '1';
									slow_timer_on			<= '0';
									init_flow_next			<= EVAL;
								end if;
							end if;
						when EVAL => -- check if init was successful
							slow_timer_on			<= '1';
							if (link_msg.init_timeout = '1') then 
								if (slow_timer_cnt = 1000_000) then -- wait for 1s to re-init
									link_msg.init_done		<= '1';
									slow_timer_on			<= '0';
									init_flow_next			<= RESET;
									link_msg.progress_bar	<= 0; -- also clear the progress bar to start 
								end if;
							else
								if (slow_timer_cnt = 500) then 
									link_msg.init_done		<= '1'; -- NOTE: clear this flag before entering sensor_link_state.INIT state
									slow_timer_on			<= '0';
									sensor_link_state_next	<= LOAD_ROM_SKIP;
									init_flow_next			<= RESET;
								end if;
							end if;
						when others =>
					end case;
				when LOAD_ROM_SKIP => 
					master_tx.data 				<= DS18B20_CMD.ROM_SKIP;
					dq_z 						<= 'Z';
					slow_timer_on				<= '0';
					link_msg.write_done			<= '0';
					sensor_link_state_next		<= MASTER_TX_BYTE; 
				when MASTER_TX_BYTE => 
					case master_tx_flow is 
						when RESET =>
							dq_z					<= '0';
							slow_timer_on			<= '0';
							master_tx_flow_next		<= TX_BIT;
						when TX_BIT =>
							slow_timer_on			<= '1';
							if (slow_timer_cnt < 60+20) then -- manual: tx bit between 60-120 us
								dq_z					<= master_tx.data(master_tx.bit_idx);
							elsif (slow_timer_cnt < 60+20+10) then -- manual: release for another 10us, between bits
								dq_z					<= 'Z';
							else 
								if (master_tx.bit_idx < 7) then
									master_tx.bit_idx				<= master_tx.bit_idx + 1;
									slow_timer_on					<= '0';
									master_tx_flow_next				<= RESET;
								else -- this byte tx is done
									slow_timer_on					<= '0';
									master_tx_flow_next				<= RESET;
									link_msg.progress_bar			<= link_msg.progress_bar + 1; -- incr the command count
									link_msg.write_done				<= '1';
									case link_msg.progress_bar is -- decide which link state should go next
										when 1 => -- this is the check point of the link state machine
											sensor_link_state_next		<= LOAD_CONVERT;
										when 2 =>
											sensor_link_state_next		<= WAIT_FOR_SENSOR_CONV;
										when 3 =>
											sensor_link_state_next		<= LOAD_READ_TEMP;
										when 4 =>
											sensor_link_state_next		<= MASTER_RX_BYTE;
										when others => -- null
									end case;
								end if;
							end if;
						when others =>
					end case;
				when LOAD_CONVERT => -- progress_bar = 1
					master_tx.data 				<= DS18B20_CMD.CONVERT;
					dq_z 						<= 'Z';
					slow_timer_on				<= '0';				
					link_msg.write_done			<= '0';	
					sensor_link_state_next		<= MASTER_TX_BYTE; 
				when WAIT_FOR_SENSOR_CONV => -- progress_bar = 2
					slow_timer_on				<= '1';
					if (PARACITIC_POWERING = false) then -- release, so current will be draw from master side, so best for smallest ground crosstalk.
						dq_z 						<= 'Z';
					else -- force to pull high, the tmpr sensor will draw current from master side pull-up resistor. Need speical circuitry, but induce higher ground crosstalk.
						dq_z 						<= '1';
					end if;
					if (slow_timer_cnt >= CONVERSION_WAIT_T_MS*1000) then -- ok
						slow_timer_on				<= '0';
						sensor_link_state_next		<= INIT; -- do init again
						init_flow_next				<= RESET;
						link_msg.init_done			<= '0'; -- clear it here, as conversion has been completed
					end if;
				when LOAD_READ_TEMP => -- progress_bar = 3 
					master_tx.data 				<= DS18B20_CMD.READ_TEMP;
					dq_z 						<= 'Z';
					slow_timer_on				<= '0';	
					link_msg.write_done			<= '0';	
					sensor_link_state_next		<= MASTER_TX_BYTE;
				when MASTER_RX_BYTE => -- progress_bar = 4
					case master_rx_flow is -- master rx bytes from 0 to 8
						when RX_EVAL =>
							if (link_msg.write_done = '1') then -- state boundary, last state is tx (read_temp)
								-- wait 500 us after sending READ_TEMP command
								slow_timer_on			<= '1';
								if (slow_timer_cnt >= 500) then
									master_rx_flow_next		<= RX_BIT;
									master_rx.bit_idx		<= 0;
									master_rx.byte_idx		<= 0;
									link_msg.write_done		<= '0';
									slow_timer_on			<= '0';
								end if;
							elsif (master_rx.bit_idx = 0) then -- byte boundary
								-- only wait for 100us between bytes
								slow_timer_on			<= '1';
								if (slow_timer_cnt >= 100) then
									master_rx_flow_next		<= RX_BIT;
									slow_timer_on			<= '0';
								end if;
							else -- bit boundary
								slow_timer_on			<= '1';
								if (slow_timer_cnt >= 20) then
									master_rx_flow_next		<= RX_BIT;
									slow_timer_on			<= '0';
								end if;
							end if;
						when RX_BIT =>
							slow_timer_on			<= '1';
							if (link_msg.write_done = '1') then -- wait state
								-- idle
							elsif (slow_timer_cnt < 5) then -- pull low 5us: master start read time slot
								dq_z					<= '0';
							elsif (slow_timer_cnt < 15) then -- master release
								dq_z					<= 'Z';
							elsif (slow_timer_cnt = 15) then -- master rx samples at 15us after read time slot started
								master_rx.data(master_rx.byte_idx)(master_rx.bit_idx)		<= io_dq;
							elsif (slow_timer_cnt = 60) then -- wait until read time slot ends
								slow_timer_on			<= '0';
								link_msg.write_done		<= '1';
								-- incr bit and byte counters
								if (master_rx.bit_idx < 7) then 
									master_rx.bit_idx		<= master_rx.bit_idx + 1; -- incr bit
									master_rx_flow_next		<= RX_EVAL;
								else -- read byte boundary
									if (master_rx.byte_idx = SCRATCH_PAD_BYTES-1 and master_rx.bit_idx = 7) then -- if end of scratch pad, EXIT here 
										sensor_link_state_next		<= CRC_CHECKING;
										link_msg.read_done			<= '1';
										link_msg.reading_f32		<= temp_reading_f32;
										-- reset counters as we reached the end
										master_rx.bit_idx			<= 0;
										master_rx.byte_idx			<= 0;	
									else -- incr byte
										master_rx_flow_next			<= RX_EVAL; 
										master_rx.bit_idx			<= 0;
										master_rx.byte_idx			<= master_rx.byte_idx + 1;
									end if;
								end if;
							end if;
						when others =>
					end case;
				when CRC_CHECKING =>
					crc_checker.sclr		<= '0';
					crc_checker.shift_en	<= '1'; -- start the checking by shifting in bits
					if (crc_checker.bit_idx < 7) then -- incr bit
						crc_checker.bit_idx		<= crc_checker.bit_idx + 1;
					else
						if (crc_checker.byte_idx = SCRATCH_PAD_BYTES-1 and crc_checker.bit_idx = 7) then -- exit here
							crc_checker.bit_idx			<= 0;
							crc_checker.byte_idx		<= 0;
							crc_checker.shift_en		<= '0';
							crc_checker.sclr			<= '1';
							if (crc_checker.sclr = '0') then -- only latch once
								link_msg.crc_err			<= crc_checker.error;
							end if;
							sensor_link_state_next		<= RESET;
						else -- incr byte
							crc_checker.bit_idx			<= 0;
							crc_checker.byte_idx		<= crc_checker.byte_idx + 1;
						end if;
					end if;
				when REPORTING =>
					if (pipe.link2avs.start = '0' and pipe.link2avs.done = '0') then -- start the ISC 
						pipe.link2avs.start 	<= '1';
					elsif (pipe.link2avs.start = '1' and pipe.link2avs.done = '0') then -- wait for slave to ack
						-- idle
					elsif (pipe.link2avs.start = '1' and pipe.link2avs.done = '1') then -- slave ack
						pipe.link2avs.start 	<= '0';
					else -- reporting done
						sensor_link_state_next	<= RESET;
					end if;
				when RESET => -- reset everything here
					-- init part
					init_flow_next					<= RESET;
					link_msg.init_done				<= '0';
					link_msg.sensor_present			<= '0';
					link_msg.init_timeout			<= '0';
					link_msg.write_done				<= '0';
					link_msg.progress_bar			<= 0;
					link_msg.read_done				<= '0';
					link_msg.crc_err				<= '0';
					link_msg.reading_f32				<= (others => '0');
					-- master tx part
					master_tx_flow_next				<= RESET;
					master_tx.data					<= (others => '0');
					master_tx.bit_idx				<= 0;
					-- master rx part
					master_rx_flow_next				<= RX_EVAL;
					master_rx.data					<= (others => (others => '0'));
					master_rx.bit_idx				<= 0;
					master_rx.byte_idx				<= 0;
					-- crc checking part
					crc_checker.bit_idx				<= 0;
					crc_checker.byte_idx			<= 0;
					crc_checker.sclr				<= '0';
					crc_checker.shift_en			<= '0';
					-- misc signals
					dq_z							<= 'Z';
					slow_timer_on					<= '0';
					-- pipes (remember to fully reset the pipe at the other end)
					pipe.link2avs.start				<= '0';
					if (link_msg.read_go = '1') then 
						sensor_link_state_next			<= INIT;
					else
						sensor_link_state_next			<= RESET;
					end if;
				when others =>
			end case;
			
			-- sync reset part
			if (i_rst_n /= '1') then 
				link_msg.read_go		<= '1'; -- reset to 1 to allow one skid read after power up
			elsif (sensor_link_state = REPORTING) then 
				-- check if csr command me to stop
				link_msg.read_go				<= csr.read_enable;
			end if;
		end if;
	end process;
	
	
	-- ---------------------------
	-- ds18b20_to_float
	-- ---------------------------
	proc_ds18b20_to_float : process(all)
		-- +--------------------------------------------+
		-- |input: 		temp_sign_bitfield				|
		-- |			temp_data_bitfield				|
		-- |output: 	temp_reading_f32				|
		-- +--------------------------------------------+
		-- converting from DS18B20 reading to float-32 (IEEE754-Single Precision Binary32 Floating Point)
		-- Ref: https://en.wikipedia.org/wiki/Single-precision_floating-point_format
		variable temp_sign_bitfield			: std_logic_vector(4 downto 0); 
		variable temp_data_bitfield			: std_logic_vector(10 downto 0);
		variable temp_bitshift				: integer; -- temporary
	begin
		-- some bit twiddlings 
		-- hook up, refer to Maxim manual
		temp_sign_bitfield := master_rx.data(1)(7 downto 3);
		temp_data_bitfield := master_rx.data(1)(2 downto 0) & master_rx.data(0);
		
		-- sign part
		temp_reading_f32(31) 			<= temp_sign_bitfield(0); 
		
		-- exponent part
		temp_reading_f32(30 downto 23) 	<=  std_logic_vector(to_unsigned(temp_bitshift+127,8));
		
		-- fraction part
		-- default
		temp_bitshift					:= 0;
		temp_reading_f32(22 downto 0) 	<= (others=> '0');
		for i in 1 to 10 loop
			if (temp_data_bitfield(i) = '1') then
				temp_bitshift						:= i-4;
				temp_reading_f32(22 downto 0)		<= temp_data_bitfield(i-1 downto 0) & std_logic_vector(to_unsigned(0,23-i));
			end if;
		end loop;
	end process;
	
	-- -------------
	-- tri_state
	-- -------------
	io_dq <= dq_z; -- Master read: setting 'dq_z' to 'Z' and read io_dq. Master write: drive 'dq_z' to '0' or '1'. 
	


end architecture;
