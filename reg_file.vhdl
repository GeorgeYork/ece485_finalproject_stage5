
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity reg_file is
    Port (
        clk       : in  STD_LOGIC;
        reg_write : in  STD_LOGIC;
        rs1       : in  STD_LOGIC_VECTOR(4 downto 0);
        rs2       : in  STD_LOGIC_VECTOR(4 downto 0);
        rd        : in  STD_LOGIC_VECTOR(4 downto 0);
        data_in   : in  STD_LOGIC_VECTOR(31 downto 0);
        data_out1 : out STD_LOGIC_VECTOR(31 downto 0);
        data_out2 : out STD_LOGIC_VECTOR(31 downto 0)
    );
end reg_file;

architecture Behavioral of reg_file is
    type reg_array is array (0 to 31) of STD_LOGIC_VECTOR(31 downto 0);
    signal registers : reg_array := (others => (others => '0'));
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if reg_write = '1' then
                registers(to_integer(unsigned(rd))) <= data_in;
            end if;
        end if;
    end process;

    data_out1 <= registers(to_integer(unsigned(rs1)));
    data_out2 <= registers(to_integer(unsigned(rs2)));
end Behavioral;
