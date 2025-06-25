library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity instr_mem is
    Port (
        addr    : in  STD_LOGIC_VECTOR(31 downto 0);
        instr   : out STD_LOGIC_VECTOR(31 downto 0)
    );
end instr_mem;

-- Reordering instuctions to fill stalls
architecture Behavioral of instr_mem is
    type memory_array is array (0 to 255) of STD_LOGIC_VECTOR(31 downto 0);
    signal memory : memory_array := (
        0 => x"10000317", -- load_addr x6, array (custom instruction), where array is 0x10000000
        1 => x"00900293", -- addi x5, x0, 9         000000001001 00000 000 00101 0010011
        2 => x"00032383", -- lw x7, 0(x6)           000000000000 00110 010 00111 0000011
        3 => x"00430313", -- loop: addi x6, x6, 4   000000000100 00110 000 00110 0010011
        4 => x"00032503", --       lw x10, 0(x6)    000000000000 00110 010 01010 0000011
        5 => x"FFF28293", --       subi x5, x5, 1 (really addi x5, x5, -1)  111111111111 00101 000 00101 0010011
        6 => x"007503B3", --       add x7, x10, x7 0000000 00111 01010 000 00111 0110011
        7 => x"FA0298E3", --     bne x5, x0, loop   1 111101 00000 00101 001 100 0 1 1100011 [jump -20; note: assumes PC is incremented by 4]
                                                   -- 1111 1010 0000 0010 1001 1000 1110 0011
        8 => x"FE7FF06F", -- done: j done            11111110011111111111 00000 1101111  [jump -4; note: assumes PC is incremented by 4]
                                                   -- 1111 1110 0111 1111 1111 0000 0110 1111  
        others => (others => '0')
    );
begin
    process(addr)
    begin
        instr <= memory(to_integer(unsigned(addr(7 downto 0))));
    end process;
end Behavioral;
