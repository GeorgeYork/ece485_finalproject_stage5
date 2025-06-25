
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity immediate_generator is
    Port (
        instr : in  STD_LOGIC_VECTOR(31 downto 0);
        imm   : out STD_LOGIC_VECTOR(31 downto 0)
    );
end immediate_generator;

architecture Behavioral of immediate_generator is
begin
    process(instr)
    begin
        case instr(6 downto 0) is
            when "0010011" => -- ADDI, SUBI
                if instr(31) = '0' then  -- positive
                    imm <= "00000000000000000000" & instr(31 downto 20);
                else -- negative
                    imm <= "11111111111111111111" & instr(31 downto 20);
                end if;                   
            when "0000011" => -- LW
                if instr(31) = '0' then  -- positive
                    imm <= "00000000000000000000" & instr(31 downto 20);
                else -- negative
                    imm <= "11111111111111111111" & instr(31 downto 20);
                end if;  
            when "0100011" => -- SW
                if instr(31) = '0' then  -- positive
                    imm <= "00000000000000000000" & instr(31 downto 25) & instr(11 downto 7);
                else -- negative
                    imm <= "11111111111111111111" & instr(31 downto 25) & instr(11 downto 7);
                end if;                   
            when "1100011" => -- BNE
                if instr(31) = '0' then  -- positive
                    imm <= "0000000000000000000000" & instr(7) &  instr(30 downto 25) & instr(11 downto 9); -- div by 4, or shift left 2
                else -- negative
                    imm <= "1111111111111111111111" & instr(7) &  instr(30 downto 25) & instr(11 downto 9); -- div by 4, or shift left 2
                end if;                
            when "1101111" => -- J
            
                 if instr(31) = '0' then  -- positive
                    imm <= "000000000000" & instr(31) & instr(21 downto 12) & instr(22) & instr(30 downto 23);
                else -- negative
                    imm <= "111111111111" & instr(31) & instr(21 downto 12) & instr(22) & instr(30 downto 23);
                end if;              
            when others =>
                imm <= (others => '0');
        end case;
    end process;
end Behavioral;
