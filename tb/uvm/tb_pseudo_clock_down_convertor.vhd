-- ------------------------------------------------------------------------------------------------------------
-- IP Name: pseudo_clock_down_convertor
-- Testbench: odd/even divider regression
-- Version : 26.2.1
-- Date    : 20260428
-- Change  : Add directed check for the 125:1 odd divider used by FEB OneWire builds.
-- Description:
--   Proves the common 1 us divider emits periodic ticks for both odd and even REF_CLOCK_RATE / 1 MHz ratios.
-- ------------------------------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

library std;
use std.env.all;

entity tb_pseudo_clock_down_convertor is
end entity;

architecture tb of tb_pseudo_clock_down_convertor is
    constant CLK_PERIOD_CONST : time := 8 ns;
    constant ODD_FACTOR_CONST : positive := 125;
    constant EVEN_FACTOR_CONST : positive := 124;

    signal clk          : std_logic := '0';
    signal reset_n      : std_logic := '0';
    signal odd_clk      : std_logic;
    signal odd_tick     : std_logic;
    signal even_clk     : std_logic;
    signal even_tick    : std_logic;
begin

    clk <= not clk after CLK_PERIOD_CONST / 2;

    odd_dut : entity work.pseudo_clock_down_convertor
        generic map (
            CLK_DOWN_FACTOR => ODD_FACTOR_CONST
        )
        port map (
            clk        => clk,
            reset_n    => reset_n,
            pseudo_clk => odd_clk,
            tick       => odd_tick
        );

    even_dut : entity work.pseudo_clock_down_convertor
        generic map (
            CLK_DOWN_FACTOR => EVEN_FACTOR_CONST
        )
        port map (
            clk        => clk,
            reset_n    => reset_n,
            pseudo_clk => even_clk,
            tick       => even_tick
        );

    stim_proc : process
        variable odd_tick_count     : natural := 0;
        variable even_tick_count    : natural := 0;
        variable odd_last_cycle     : integer := -1;
        variable even_last_cycle    : integer := -1;
    begin
        reset_n <= '0';
        wait for 5 * CLK_PERIOD_CONST;
        reset_n <= '1';

        for cycle_idx in 0 to 420 loop
            wait until rising_edge(clk);
            wait for 0 ns;

            if odd_tick = '1' then
                if odd_tick_count = 0 then
                    assert cycle_idx >= (ODD_FACTOR_CONST / 2) - 1 and
                           cycle_idx <= (ODD_FACTOR_CONST / 2) + 1
                        report "first odd-divider tick is not near the expected pseudo-clock rising edge"
                        severity failure;
                else
                    assert cycle_idx - odd_last_cycle = ODD_FACTOR_CONST
                        report "odd-divider tick period is not CLK_DOWN_FACTOR input cycles"
                        severity failure;
                end if;
                odd_last_cycle := cycle_idx;
                odd_tick_count := odd_tick_count + 1;
            end if;

            if even_tick = '1' then
                if even_tick_count = 0 then
                    assert cycle_idx >= (EVEN_FACTOR_CONST / 2) - 1 and
                           cycle_idx <= (EVEN_FACTOR_CONST / 2) + 1
                        report "first even-divider tick is not near the expected pseudo-clock rising edge"
                        severity failure;
                else
                    assert cycle_idx - even_last_cycle = EVEN_FACTOR_CONST
                        report "even-divider tick period is not CLK_DOWN_FACTOR input cycles"
                        severity failure;
                end if;
                even_last_cycle := cycle_idx;
                even_tick_count := even_tick_count + 1;
            end if;
        end loop;

        assert odd_tick_count >= 3
            report "odd divider produced fewer than three ticks; this catches the stalled-counter regression"
            severity failure;
        assert even_tick_count >= 3
            report "even divider produced fewer than three ticks"
            severity failure;

        report "PSEUDO_CLOCK_DOWN_CONVERTOR_PASS" severity note;
        finish;
    end process;

end architecture tb;
