library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity scfifo is
    generic (
        add_ram_output_register : string := "OFF";
        allow_rwcycle_when_full : string := "OFF";
        almost_empty_value      : natural := 0;
        almost_full_value       : natural := 0;
        intended_device_family  : string := "unused";
        enable_ecc              : string := "FALSE";
        lpm_numwords            : natural;
        lpm_showahead           : string := "ON";
        lpm_width               : natural;
        lpm_widthu              : natural := 1;
        overflow_checking       : string := "ON";
        ram_block_type          : string := "AUTO";
        underflow_checking      : string := "ON";
        use_eab                 : string := "ON";
        lpm_hint                : string := "UNUSED";
        lpm_type                : string := "scfifo"
    );
    port (
        clock : in  std_logic;
        data  : in  std_logic_vector(lpm_width - 1 downto 0);
        empty : out std_logic;
        full  : out std_logic;
        q     : out std_logic_vector(lpm_width - 1 downto 0);
        rdreq : in  std_logic;
        sclr  : in  std_logic := '0';
        usedw : out std_logic_vector(lpm_widthu - 1 downto 0);
        wrreq : in  std_logic
    );
end entity scfifo;

architecture sim of scfifo is
    type mem_t is array (0 to lpm_numwords - 1) of std_logic_vector(lpm_width - 1 downto 0);

    signal mem        : mem_t := (others => (others => '0'));
    signal rd_ptr     : natural range 0 to lpm_numwords - 1 := 0;
    signal wr_ptr     : natural range 0 to lpm_numwords - 1 := 0;
    signal fill_count : natural range 0 to lpm_numwords := 0;
    signal q_r        : std_logic_vector(lpm_width - 1 downto 0) := (others => '0');
begin
    empty <= '1' when fill_count = 0 else '0';
    full  <= '1' when fill_count = lpm_numwords else '0';
    q     <= mem(rd_ptr) when (lpm_showahead = "ON" and fill_count /= 0) else q_r;
    usedw <= std_logic_vector(to_unsigned(fill_count, usedw'length));

    proc_fifo : process (clock)
        variable next_rd_ptr_v : natural range 0 to lpm_numwords - 1;
        variable next_wr_ptr_v : natural range 0 to lpm_numwords - 1;
        variable fill_delta_v  : integer range -1 to 1;
    begin
        if rising_edge(clock) then
            if sclr = '1' then
                rd_ptr     <= 0;
                wr_ptr     <= 0;
                fill_count <= 0;
                q_r        <= (others => '0');
            else
                next_rd_ptr_v := rd_ptr;
                next_wr_ptr_v := wr_ptr;
                fill_delta_v  := 0;

                if rdreq = '1' and fill_count /= 0 then
                    q_r <= mem(rd_ptr);
                    if rd_ptr = lpm_numwords - 1 then
                        next_rd_ptr_v := 0;
                    else
                        next_rd_ptr_v := rd_ptr + 1;
                    end if;
                    fill_delta_v := fill_delta_v - 1;
                end if;

                if wrreq = '1' and fill_count /= lpm_numwords then
                    mem(wr_ptr) <= data;
                    if wr_ptr = lpm_numwords - 1 then
                        next_wr_ptr_v := 0;
                    else
                        next_wr_ptr_v := wr_ptr + 1;
                    end if;
                    fill_delta_v := fill_delta_v + 1;
                end if;

                rd_ptr     <= next_rd_ptr_v;
                wr_ptr     <= next_wr_ptr_v;
                fill_count <= fill_count + fill_delta_v;
            end if;
        end if;
    end process proc_fifo;
end architecture sim;
