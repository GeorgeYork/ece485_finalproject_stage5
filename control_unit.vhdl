
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity control_unit is
    Port (
        opcode      : in  STD_LOGIC_VECTOR(6 downto 0);
        reg_write   : out STD_LOGIC;
        mem_read    : out STD_LOGIC;
        mem_write   : out STD_LOGIC;
        alu_src     : out STD_LOGIC;
        branch      : out STD_LOGIC;
        load_addr   : out STD_LOGIC;
        jump        : out STD_LOGIC
    );
end control_unit;

architecture Behavioral of control_unit is
begin
    process(opcode)
    begin
        case opcode is
            when "0110011" => -- ADD
                reg_write <= '1'; alu_src <= '0'; mem_read <= '0'; mem_write <= '0'; branch <= '0'; jump <= '0'; load_addr <= '0';
            when "0010011" => -- ADDI, SUBI
                reg_write <= '1'; alu_src <= '1'; mem_read <= '0'; mem_write <= '0'; branch <= '0'; jump <= '0'; load_addr <= '0';
            when "0010111" => -- Load_Addr (custom instruction)
                reg_write <= '1'; alu_src <= '0'; mem_read <= '0'; mem_write <= '0'; branch <= '0'; jump <= '0'; load_addr <= '1';
            when "0000011" => -- LW
                reg_write <= '1'; alu_src <= '1'; mem_read <= '1'; mem_write <= '0'; branch <= '0'; jump <= '0'; load_addr <= '0';
            when "0100011" => -- SW
                reg_write <= '0'; alu_src <= '1'; mem_read <= '0'; mem_write <= '1'; branch <= '0'; jump <= '0'; load_addr <= '0';
            when "1100011" => -- BNE
                reg_write <= '0'; alu_src <= '0'; mem_read <= '0'; mem_write <= '0'; branch <= '1'; jump <= '0'; load_addr <= '0';
            when "1101111" => -- J
                reg_write <= '0'; alu_src <= '0'; mem_read <= '0'; mem_write <= '0'; branch <= '0'; jump <= '1'; load_addr <= '0';
            when others => -- NOP [real RISC-V uses ADDI for the NOP]
                reg_write <= '0'; alu_src <= '0'; mem_read <= '0'; mem_write <= '0'; branch <= '0'; jump <= '0'; load_addr <= '0';
        end case;
    end process;
    -- NOTES:  mem_write should not be done until MEM stage
    --         reg_write should not be done until WB stage
    --         PC should be updated between WB/FETCH
end Behavioral;
