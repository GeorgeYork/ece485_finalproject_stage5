library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity forwarding_unit is
    Port (
        ex_mem_reg_write : in STD_LOGIC;
        mem_wb_mem_read  : in STD_LOGIC;
        mem_wb_load_addr : in STD_LOGIC;
        ex_mem_rd        : in STD_LOGIC_VECTOR(4 downto 0);
        mem_wb_rd        : in STD_LOGIC_VECTOR(4 downto 0);
        id_ex_rs1        : in STD_LOGIC_VECTOR(4 downto 0);
        mux_select_A     : out STD_LOGIC_VECTOR(1 downto 0)
    );
end forwarding_unit;

architecture Behavioral of forwarding_unit is

begin
    process(ex_mem_reg_write, mem_wb_mem_read, mem_wb_load_addr, ex_mem_rd, mem_wb_rd, id_ex_rs1)
begin
  -- Default: no forwarding
  mux_select_A <= "00";

  -- EX hazard
  if ex_mem_reg_write = '1' and ex_mem_rd /= "00000" and ex_mem_rd = id_ex_rs1 then
    mux_select_A <= "01";
  elsif mem_wb_mem_read = '1' and mem_wb_rd /= "00000" and mem_wb_rd = id_ex_rs1 then
    mux_select_A <= "10";
  elsif mem_wb_load_addr = '1' and mem_wb_rd /= "00000" and mem_wb_rd = id_ex_rs1 then
    mux_select_A <= "11";
  end if;
    end process;
end Behavioral;