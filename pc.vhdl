
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pc_live is
    Port (
        clk     : in  STD_LOGIC;
        reset   : in  STD_LOGIC;
        pc_in   : in  STD_LOGIC_VECTOR(31 downto 0);
        pc_out  : out STD_LOGIC_VECTOR(31 downto 0)
    );
end pc_live;

architecture Behavioral of pc_live is
begin
    process(clk, reset)
    begin
        if reset = '1' then
            pc_out <= (others => '0');
        elsif rising_edge(clk) then
            pc_out <= pc_in;
        end if;
    end process;
end Behavioral;
