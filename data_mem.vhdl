
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity data_mem is
    Port (
        addr     : in  STD_LOGIC_VECTOR(31 downto 0);
        data_in  : in  STD_LOGIC_VECTOR(31 downto 0);
        data_out : out STD_LOGIC_VECTOR(31 downto 0);
        mem_read : in  STD_LOGIC;
        mem_write: in  STD_LOGIC
    );
end data_mem;

architecture Behavioral of data_mem is
    type memory_array is array (0 to 255) of STD_LOGIC_VECTOR(31 downto 0);
    signal memory : memory_array := (
        x"00000005", x"00000004", x"00000010", x"00000003",
        x"00000012", x"00000001", x"00000007", x"00000004",
        x"00000008", x"00000002",
        others => x"00000000"
    );
begin
    process(addr, mem_read, mem_write, data_in)
    begin
        if mem_read = '1' then
            data_out <= memory(to_integer(unsigned(addr(7 downto 0))));
        elsif mem_write = '1' then
            memory(to_integer(unsigned(addr(7 downto 0)))) <= data_in;
        end if;
    end process;
end Behavioral;
