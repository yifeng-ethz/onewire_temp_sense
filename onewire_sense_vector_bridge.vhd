-- onewire_sense_vector_bridge.vhd
-- Version : 26.0.330
-- Date    : 20260330
-- Change  : Add a source bridge that packs six split one-wire sensor lines into
--           the shared 6-bit FEB top-level sense_dq bus.

library ieee;
use ieee.std_logic_1164.all;

entity onewire_sense_vector_bridge is
    port (
        i_clk_125           : in  std_logic := '0';
        i_rst_n             : in  std_logic := '0';
        temp_mutrig0_dq_in    : out std_logic;
        temp_mutrig0_dq_out   : in  std_logic := '0';
        temp_mutrig0_dq_oe    : in  std_logic := '0';
        temp_mutrig1_dq_in    : out std_logic;
        temp_mutrig1_dq_out   : in  std_logic := '0';
        temp_mutrig1_dq_oe    : in  std_logic := '0';
        temp_sipm0_dq_in      : out std_logic;
        temp_sipm0_dq_out     : in  std_logic := '0';
        temp_sipm0_dq_oe      : in  std_logic := '0';
        temp_sipm1_dq_in      : out std_logic;
        temp_sipm1_dq_out     : in  std_logic := '0';
        temp_sipm1_dq_oe      : in  std_logic := '0';
        temp_dab0_dq_in       : out std_logic;
        temp_dab0_dq_out      : in  std_logic := '0';
        temp_dab0_dq_oe       : in  std_logic := '0';
        temp_dab1_dq_in       : out std_logic;
        temp_dab1_dq_out      : in  std_logic := '0';
        temp_dab1_dq_oe       : in  std_logic := '0';
        sense_dq_in           : in  std_logic_vector(5 downto 0) := (others => '0');
        sense_dq_out          : out std_logic_vector(5 downto 0);
        sense_dq_oe           : out std_logic_vector(5 downto 0)
    );
end entity onewire_sense_vector_bridge;

architecture rtl of onewire_sense_vector_bridge is
begin
    -- These ports keep the bridge in the same declared Platform Designer
    -- clock/reset domain as the onewire_sense conduit interfaces.
    -- Preserve the board-level sensor ordering already used in top.vhd:
    -- 0/1 = MuTRiG, 2/3 = SiPM, 4/5 = DAB.
    temp_mutrig0_dq_in    <= sense_dq_in(0);
    temp_mutrig1_dq_in    <= sense_dq_in(1);
    temp_sipm0_dq_in      <= sense_dq_in(2);
    temp_sipm1_dq_in      <= sense_dq_in(3);
    temp_dab0_dq_in       <= sense_dq_in(4);
    temp_dab1_dq_in       <= sense_dq_in(5);

    sense_dq_out(0)       <= temp_mutrig0_dq_out;
    sense_dq_out(1)       <= temp_mutrig1_dq_out;
    sense_dq_out(2)       <= temp_sipm0_dq_out;
    sense_dq_out(3)       <= temp_sipm1_dq_out;
    sense_dq_out(4)       <= temp_dab0_dq_out;
    sense_dq_out(5)       <= temp_dab1_dq_out;

    sense_dq_oe(0)        <= temp_mutrig0_dq_oe;
    sense_dq_oe(1)        <= temp_mutrig1_dq_oe;
    sense_dq_oe(2)        <= temp_sipm0_dq_oe;
    sense_dq_oe(3)        <= temp_sipm1_dq_oe;
    sense_dq_oe(4)        <= temp_dab0_dq_oe;
    sense_dq_oe(5)        <= temp_dab1_dq_oe;
end architecture rtl;
