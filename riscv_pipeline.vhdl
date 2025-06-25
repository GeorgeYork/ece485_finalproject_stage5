
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity riscv_pipeline is
    Port (
        clk     : in  STD_LOGIC;
        reset   : in  STD_LOGIC
    );
end riscv_pipeline;

architecture Behavioral of riscv_pipeline is

    -- States no longer needed since with pipeline, each instruction is in a different state
	-- States flow through pipelined registers
    --type state_type is (FETCH, DECODE, EXECUTE, MEMORY, WRITEBACK);
    --signal state : state_type := FETCH;

    -- Basic Registers
    signal pc, pc_byte_not_word, NPC, next_pc           : STD_LOGIC_VECTOR(31 downto 0);
    signal instr                                        : STD_LOGIC_VECTOR(31 downto 0);
    signal opcode                                       : STD_LOGIC_VECTOR(6 downto 0);
    signal reg1_data, reg2_data                         : STD_LOGIC_VECTOR(31 downto 0);
    signal alu_input_a, alu_input_b, alu_result  : STD_LOGIC_VECTOR(31 downto 0);
    signal wb_data                                      : STD_LOGIC_VECTOR(31 downto 0);
    signal mem_data, data_memory_byte_not_word, mem_wb_mem_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal clock_counter : integer := 1;
     -- control signals
    signal mem_read   : STD_LOGIC;
    signal mem_write, mem_write_chip  : STD_LOGIC;
    signal alu_src    : STD_LOGIC;
    signal branch     : STD_LOGIC;
    signal jump       : STD_LOGIC;
    signal load_addr  : STD_LOGIC;
    signal reg_write, reg_write_chip  : STD_LOGIC;
    signal rs1, rs2, rd : STD_LOGIC_VECTOR(4 downto 0);
    signal wb_rd        : STD_LOGIC_VECTOR(4 downto 0);
       
    -- Registers for pipeline stages
    signal if_id_npc, id_ex_npc, ex_mem_npc, mem_wb_npc             : STD_LOGIC_VECTOR(31 downto 0) := (others => '0');
    signal id_ex_alu_result, ex_mem_alu_result, mem_wb_alu_result   : STD_LOGIC_VECTOR(31 downto 0);
    signal if_id_alu_op, id_ex_alu_op, ex_mem_alu_op, mem_wb_alu_op : STD_LOGIC_VECTOR(3 downto 0);
    signal if_id_imm, id_ex_imm, ex_mem_imm, mem_wb_imm             : STD_LOGIC_VECTOR(31 downto 0);
    signal if_id_instr, id_ex_instr, ex_mem_instr, mem_wb_instr     : STD_LOGIC_VECTOR(31 downto 0); 
    signal if_id_reg1_data, id_ex_reg1_data, ex_mem_reg1_data, mem_wb_reg1_data : STD_LOGIC_VECTOR(31 downto 0);
    signal if_id_reg2_data, id_ex_reg2_data, ex_mem_reg2_data, mem_wb_reg2_data : STD_LOGIC_VECTOR(31 downto 0);
    signal if_id_rs1, id_ex_rs1, ex_mem_rs1, mem_wb_rs1 : STD_LOGIC_VECTOR(4 downto 0);
    signal if_id_rs2, id_ex_rs2, ex_mem_rs2, mem_wb_rs2 : STD_LOGIC_VECTOR(4 downto 0);
    signal if_id_rd, id_ex_rd, ex_mem_rd, mem_wb_rd     : STD_LOGIC_VECTOR(4 downto 0);

    -- control signals for pipeline stages
    signal if_id_reg_write, id_ex_reg_write, ex_mem_reg_write, mem_wb_reg_write  : STD_LOGIC;
    signal if_id_alu_src, id_ex_alu_src, ex_mem_alu_src, mem_wb_alu_src          : STD_LOGIC;
    signal if_id_mem_read, id_ex_mem_read, ex_mem_mem_read, mem_wb_mem_read      : STD_LOGIC;
    signal if_id_mem_write, id_ex_mem_write, ex_mem_mem_write, mem_wb_mem_write  : STD_LOGIC;
    signal if_id_branch, id_ex_branch, ex_mem_branch, mem_wb_branch              : STD_LOGIC;
    signal if_id_jump, id_ex_jump, ex_mem_jump, mem_wb_jump                      : STD_LOGIC;
    signal if_id_load_addr, id_ex_load_addr, ex_mem_load_addr, mem_wb_load_addr  : STD_LOGIC;
    
     -- Additional signals
    signal stall, start_stall, double_stall        : STD_LOGIC;
    signal stall_counter : integer range 0 to 3 := 0;
    signal mux_select_A  : STD_LOGIC_VECTOR(1 downto 0) := (others => '0');
    signal mux_select_B  : STD_LOGIC_VECTOR(1 downto 0) := (others => '0');
 
    component pc_live 
    Port (
        clk     : in  STD_LOGIC;
        reset   : in  STD_LOGIC;
        pc_in   : in  STD_LOGIC_VECTOR(31 downto 0);
        pc_out  : out STD_LOGIC_VECTOR(31 downto 0)
    );
    end component;
            
    component instr_mem
        Port (
            addr : in  STD_LOGIC_VECTOR(31 downto 0);
            instr   : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;

    component reg_file
        Port (
            clk     : in  STD_LOGIC;
            rs1     : in  STD_LOGIC_VECTOR(4 downto 0);
            rs2     : in  STD_LOGIC_VECTOR(4 downto 0);
            rd      : in  STD_LOGIC_VECTOR(4 downto 0);
            data_in : in  STD_LOGIC_VECTOR(31 downto 0);
            reg_write : in  STD_LOGIC;
            data_out1     : out STD_LOGIC_VECTOR(31 downto 0);
            data_out2     : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;


    component control_unit 
        Port (
        opcode      : in  STD_LOGIC_VECTOR(6 downto 0);
        reg_write   : out STD_LOGIC;
        mem_read    : out STD_LOGIC;
        mem_write   : out STD_LOGIC;
        alu_src     : out STD_LOGIC;
        branch      : out STD_LOGIC;
        load_addr   : out STD_LOGIC;  -- Custom signal for load_addr instruction
        jump        : out STD_LOGIC
        );
    end component;

    component immediate_generator is
    Port (
        instr : in  STD_LOGIC_VECTOR(31 downto 0);
        imm   : out STD_LOGIC_VECTOR(31 downto 0)
    );
    end component;

   component alu_control is
    Port (
        funct7 : in  STD_LOGIC_VECTOR(6 downto 0);
        funct3 : in  STD_LOGIC_VECTOR(2 downto 0);
        alu_op : out STD_LOGIC_VECTOR(3 downto 0)
    );
    end component;

    component alu
        Port (
            a       : in  STD_LOGIC_VECTOR(31 downto 0);
            b       : in  STD_LOGIC_VECTOR(31 downto 0);
            op      : in  STD_LOGIC_VECTOR(3 downto 0);
            result  : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;

    component data_mem
        Port (
            addr    : in  STD_LOGIC_VECTOR(31 downto 0);
            data_in : in  STD_LOGIC_VECTOR(31 downto 0);
            mem_read  : in  STD_LOGIC;
            mem_write : in  STD_LOGIC;
            data_out  : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;
    
    component pipeline_registers 
        Port (
            clk         : in  STD_LOGIC;
            reset       : in  STD_LOGIC;
            start_stall : in  STD_LOGIC;
            stall_counter : in integer;
            
            -- IF/ID pipeline registers
            if_id_reg_write : in STD_LOGIC;
            if_id_alu_src : in STD_LOGIC;
            if_id_mem_read : in STD_LOGIC;
            if_id_mem_write : in STD_LOGIC;
            if_id_branch : in STD_LOGIC;
            if_id_jump : in STD_LOGIC;
            if_id_load_addr : in STD_LOGIC;
            if_id_rd    : inout STD_LOGIC_VECTOR(4 downto 0);
            if_id_instr : in  STD_LOGIC_VECTOR(31 downto 0);
            if_id_npc    : in  STD_LOGIC_VECTOR(31 downto 0);
            if_id_rs1    : in  STD_LOGIC_VECTOR(4 downto 0);
            if_id_reg1_data  : in  STD_LOGIC_VECTOR(31 downto 0);
            if_id_reg2_data  : in  STD_LOGIC_VECTOR(31 downto 0);
            if_id_imm        : in  STD_LOGIC_VECTOR(31 downto 0);
            if_id_alu_op : in STD_LOGIC_VECTOR(3 downto 0);
            -- ID/EX pipeline registers
            id_ex_reg_write : inout STD_LOGIC;
            id_ex_alu_src : inout STD_LOGIC;
            id_ex_mem_read : inout STD_LOGIC;
            id_ex_mem_write : inout STD_LOGIC;
            id_ex_branch : inout STD_LOGIC;
            id_ex_jump : inout STD_LOGIC;
            id_ex_load_addr : inout STD_LOGIC;
            id_ex_rd    : inout STD_LOGIC_VECTOR(4 downto 0);
            id_ex_instr : out STD_LOGIC_VECTOR(31 downto 0);
            id_ex_npc    : inout STD_LOGIC_VECTOR(31 downto 0);
            id_ex_rs1    : out STD_LOGIC_VECTOR(4 downto 0);
            id_ex_reg1_data  : inout  STD_LOGIC_VECTOR(31 downto 0);
            id_ex_reg2_data  : inout  STD_LOGIC_VECTOR(31 downto 0);
            id_ex_imm   : inout  STD_LOGIC_VECTOR(31 downto 0);
            id_ex_alu_result : in STD_LOGIC_VECTOR(31 downto 0);
            id_ex_alu_op : out STD_LOGIC_VECTOR(3 downto 0);
            -- EX/MEM pipeline registers        
            ex_mem_reg_write : inout STD_LOGIC;
            ex_mem_alu_src : inout STD_LOGIC;
            ex_mem_mem_read : inout STD_LOGIC;
            ex_mem_mem_write : inout STD_LOGIC;
            ex_mem_branch : out STD_LOGIC;
            ex_mem_jump : out STD_LOGIC;
            ex_mem_load_addr : inout STD_LOGIC;
            ex_mem_npc    : out STD_LOGIC_VECTOR(31 downto 0);
            ex_mem_rd   : inout STD_LOGIC_VECTOR(4 downto 0);
            ex_mem_reg1_data : out STD_LOGIC_VECTOR(31 downto 0);
            ex_mem_reg2_data : out STD_LOGIC_VECTOR(31 downto 0);
            ex_mem_imm  : out STD_LOGIC_VECTOR(31 downto 0);
            ex_mem_alu_result  : inout STD_LOGIC_VECTOR(31 downto 0);
            -- MEM/WB pipeline registers
            mem_wb_reg_write : out STD_LOGIC;
            mem_wb_alu_src : out STD_LOGIC;
            mem_wb_mem_read : out STD_LOGIC;
            mem_wb_mem_write : out STD_LOGIC;
            mem_wb_load_addr : out STD_LOGIC;
            mem_wb_rd   : out STD_LOGIC_VECTOR(4 downto 0);
            mem_wb_alu_result  : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;

    -- Hazard detection unit
    component hazard_detection_unit is
        Port (
            reset :          in STD_LOGIC;
            id_ex_mem_read : in STD_LOGIC;
            id_ex_load_addr : in STD_LOGIC;
            if_id_instr    : in STD_LOGIC_VECTOR(31 downto 0);
            id_ex_instr    : in STD_LOGIC_VECTOR(31 downto 0);
            id_ex_rd       : in STD_LOGIC_VECTOR(4 downto 0);
            if_id_rs1      : in STD_LOGIC_VECTOR(4 downto 0);
            if_id_rs2      : in STD_LOGIC_VECTOR(4 downto 0);
            stall_counter  : in integer range 0 to 3 := 0;
            start_stall    : out STD_LOGIC;
            double_stall   : out STD_LOGIC
        );
    end component;

    component forwarding_unit is
      Port (
          ex_mem_reg_write : in STD_LOGIC;
          mem_wb_mem_read  : in STD_LOGIC;
          mem_wb_load_addr : in STD_LOGIC;
          ex_mem_rd        : in STD_LOGIC_VECTOR(4 downto 0);
          mem_wb_rd        : in STD_LOGIC_VECTOR(4 downto 0);
          id_ex_rs1        : in STD_LOGIC_VECTOR(4 downto 0);
          mux_select_A     : out STD_LOGIC_VECTOR(1 downto 0)
      );
    end component;

    
begin

    -- clock counter process
    process(clk, reset)
    begin
        if reset = '1' then
            clock_counter <= 1;
        elsif rising_edge(clk) then
            clock_counter <= clock_counter + 1;
        end if;
    end process;

    
    -- IF units and Control units ------------------------------------------------------
    
    -- PC logic
    pc_inst: pc_live
        port map (
            clk    => clk,
            reset  => reset,
            pc_in  => next_pc,
            pc_out => pc
        );
                
    NPC <= std_logic_vector(signed(pc) + 4);
              
    -- update temporary registers to support pipelining (state machine no longer needed... as each instruction is at a different state)
    pipe_reg: pipeline_registers
        port map (
            clk    => clk,
            reset  => reset,
            start_stall => start_stall,
            stall_counter => stall_counter,
            -- IF/ID pipeline registers
            if_id_reg_write => if_id_reg_write,
            if_id_alu_src => if_id_alu_src,
            if_id_mem_read => if_id_mem_read,
            if_id_mem_write => if_id_mem_write,
            if_id_branch => if_id_branch,
            if_id_jump => if_id_jump,
            if_id_load_addr => if_id_load_addr,
            if_id_instr => if_id_instr,
            if_id_npc    => if_id_npc,
            if_id_rd    => if_id_rd,
            if_id_rs1   => if_id_rs1,
            if_id_reg1_data  => if_id_reg1_data,
            if_id_reg2_data  => if_id_reg2_data,
            if_id_imm => if_id_imm,
            if_id_alu_op => if_id_alu_op,

            -- ID/EX pipeline registers
            id_ex_reg_write => id_ex_reg_write,
            id_ex_alu_src => id_ex_alu_src,
            id_ex_mem_read => id_ex_mem_read,
            id_ex_mem_write => id_ex_mem_write,
            id_ex_branch => id_ex_branch,
            id_ex_jump => id_ex_jump,
            id_ex_load_addr => id_ex_load_addr,
            id_ex_rd    => id_ex_rd,
            id_ex_instr => id_ex_instr,
            id_ex_npc    => id_ex_npc,
            id_ex_rs1    => id_ex_rs1,
            id_ex_reg1_data  => id_ex_reg1_data,
            id_ex_reg2_data  => id_ex_reg2_data,
            id_ex_imm   => id_ex_imm,
            id_ex_alu_result => id_ex_alu_result,
            id_ex_alu_op => id_ex_alu_op,
            -- EX/MEM pipeline register
            ex_mem_reg_write => ex_mem_reg_write,
            ex_mem_alu_src => ex_mem_alu_src,
            ex_mem_mem_read => ex_mem_mem_read,
            ex_mem_mem_write => ex_mem_mem_write,
            ex_mem_branch => ex_mem_branch,
            ex_mem_jump => ex_mem_jump,
            ex_mem_load_addr => ex_mem_load_addr,
            ex_mem_npc => ex_mem_npc,
            ex_mem_rd   => ex_mem_rd,
            ex_mem_reg1_data => ex_mem_reg1_data,
            ex_mem_reg2_data => ex_mem_reg2_data,
            ex_mem_imm  => ex_mem_imm,
            ex_mem_alu_result  => ex_mem_alu_result,
            -- MEM/WB pipeline registers
            mem_wb_reg_write => mem_wb_reg_write,
            mem_wb_alu_src => mem_wb_alu_src,
            mem_wb_mem_read => mem_wb_mem_read,
            mem_wb_mem_write => mem_wb_mem_write,
            mem_wb_load_addr => mem_wb_load_addr,
            mem_wb_rd   => mem_wb_rd,
            mem_wb_alu_result  => mem_wb_alu_result
        );

    -- Instruction memory
    pc_byte_not_word <= "00" & pc(31 downto 2);  -- divide by 4 by shifting left 2, since byte addressable, not word addressable
    instr_mem_inst: instr_mem
        port map (
            addr  => pc_byte_not_word,
            instr => instr
        );  
         
    -- IF/ID pipeline registers
    if_id_instr  <= instr;
    if_id_npc    <= NPC;

    -- Decode instruction fields
    if_id_rs1 <= if_id_instr(19 downto 15);
    if_id_rs2 <= if_id_instr(24 downto 20);
    if_id_rd  <= if_id_instr(11 downto 7);
    opcode <= if_id_instr(6 downto 0);

    -- Control unit
    control_unit_inst: control_unit
        port map (
            opcode    => opcode,
            reg_write => reg_write,
            mem_read  => mem_read,
            mem_write => mem_write,
            alu_src   => alu_src,
            branch    => branch,
            load_addr => load_addr,
            jump      => jump
        );
    if_id_reg_write <= reg_write;
	if_id_mem_read <= mem_read;
	if_id_mem_write <= mem_write;
	if_id_alu_src <= alu_src;
	if_id_branch <= branch;
	if_id_load_addr <= load_addr;
	if_id_jump	<= jump;
        
    -- hazard detection unit
    hazard_unit: hazard_detection_unit
        port map (
            reset => reset,
            id_ex_mem_read => id_ex_mem_read,
            id_ex_load_addr => id_ex_load_addr,
            if_id_instr    => if_id_instr,
            id_ex_instr    => id_ex_instr,
            id_ex_rd       => id_ex_rd,
            if_id_rs1      => if_id_rs1,
            if_id_rs2      => if_id_rs2,
            stall_counter  => stall_counter,
            start_stall    => start_stall,
            double_stall   => double_stall
        );
        
    -- Stall counter process
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                stall_counter <= 0;
             elsif stall_counter > 0 then
                    stall_counter <= stall_counter - 1;
             elsif start_stall = '1' then
                --stall_counter <= 3;
                stall_counter <= 2;  -- to handle double stall case (sub then BNE)
            end if;
        end if;
    end process;

    -- Stall signal
    stall <= '1' when stall_counter > 0 else '0';        
        
    -- forwarding unit
    forward_unit: forwarding_unit
        port map (
            ex_mem_reg_write => ex_mem_reg_write,
            mem_wb_mem_read  => mem_wb_mem_read,
            mem_wb_load_addr => mem_wb_load_addr,
            ex_mem_rd        => ex_mem_rd,
            mem_wb_rd        => mem_wb_rd,
            id_ex_rs1        => id_ex_rs1,
            mux_select_A     => mux_select_A
        );

--------------------------------------------------------------------------------
    -- ID units
    -- Register file [used in ID and WB stages]
	reg_write_chip <= mem_wb_reg_write;
    reg_file_inst: reg_file
        port map (
            clk       => clk,
            reg_write => reg_write_chip,
            rs1       => if_id_rs1,
            rs2       => if_id_rs2,
            rd        => mem_wb_rd,
            data_in   => wb_data,
            data_out1 => reg1_data,
            data_out2 => reg2_data
        );    
    if_id_reg1_data <= reg1_data;  
	if_id_reg2_data <= reg2_data;
	
    -- Immediate generator
        immediate_generator_inst: immediate_generator
            port map (
                instr => if_id_instr,
                imm   => if_id_imm
            );
            
    -- ALU control unit
        alu_control_inst: alu_control
                port map (
                    funct3 => if_id_instr(14 downto 12),
                    funct7 => if_id_instr(31 downto 25),
                    alu_op => if_id_alu_op
                );
                            
    next_pc <=  pc when ((start_stall = '1' and stall_counter = 0) or (stall_counter = 2 and if_id_branch = '1' and double_stall = '1')) else   -- stall case, single and double
                std_logic_vector(signed(if_id_npc) + signed(if_id_imm)) when (if_id_branch = '1' and stall_counter = 2 and double_stall = '0' and mem_wb_alu_result /= id_ex_reg2_data) else -- branch case, single stall
                std_logic_vector(signed(if_id_npc) + signed(if_id_imm)) when (if_id_branch = '1' and stall_counter = 1 and double_stall = '1' and mem_wb_alu_result /= id_ex_reg2_data) else -- branch case, double stall
                std_logic_vector(signed(id_ex_npc) + signed(id_ex_imm)) when (id_ex_jump = '1') else  -- jump case
                NPC;    
         
    -- ID/EX pipeline registers

-----------------------------------------------------------
    -- EX units
    
    -- mux to select alu input A (with forwarding)
    --    mux_select_A
    --       00 normal
    --       01 forward from alu output
    --       10 forward from memory output
    --       11 forward from custom LoadAddr
    
    alu_input_a <= id_ex_reg1_data   when mux_select_A = "00" else
                   ex_mem_alu_result when mux_select_A = "01" else
                   mem_wb_mem_data   when mux_select_A = "10" else
                   x"10000000";            
            
    -- mux to select alu input B (not used for forwarding for this program)
    alu_input_b <= id_ex_imm when id_ex_alu_src = '1' else
                   id_ex_reg2_data;
                   
    -- ALU
    alu_inst: alu
        port map (
            a      => alu_input_a,
            b      => alu_input_b,
            op     => id_ex_alu_op,
            result => alu_result
        );
    id_ex_alu_result <= alu_result;
    -- EX/MEM pipeline register

----------------------------------------------------------------------------------------
    --  MEM units
    
    -- Data memory
    data_memory_byte_not_word <= "00" & ex_mem_alu_result(31 downto 2);  -- divide by 4 by shifting left 2, since byte addressable, not word addressable
    data_mem_inst: data_mem
        port map (
            addr      => data_memory_byte_not_word,
            data_in   => ex_mem_reg2_data,
            data_out  => mem_data,
            mem_read  => ex_mem_mem_read,
            mem_write => ex_mem_mem_write
        );
    mem_wb_mem_data <= mem_data;    
    -- MEM/WB pipeline register
    
------------------------------------------------------------------------------------------
    -- WB Units
    
    -- MUX to write back to register file
    wb_data <= mem_wb_mem_data when mem_wb_mem_read = '1' 
               else x"10000000" when mem_wb_load_addr = '1'  -- hack for custom load_addr instruction
               else mem_wb_alu_result;      
   
end Behavioral;
