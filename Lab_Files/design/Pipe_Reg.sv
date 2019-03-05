package  Pipe_Reg_PKG;
    // Reg IF_ID
    typedef struct packed{
        logic [31:0]    PC;
        logic [31:0]    PCPlus4;
        logic [31:0]    Instr;
    } if_id_reg;
    
    // Reg ID_EX
    typedef struct packed{
        logic ALUSrc; 
        logic MemtoReg;
        logic RegWrite; 
        logic MemRead;
        logic MemWrite;
        logic [3:0] ALU_CC;
        logic Branch;
        logic AUIPC;
        logic [31:0] PC;
        logic [31:0] PCBranch;
        logic [31:0] PCPlus4;
        logic [31:0] Reg1;
        logic [31:0] Reg2;
        logic [4:0] rd;
        logic [31:0] ExtImm;
        logic [2:0] func3;
        logic [6:0] func7;
        logic [31:0] Instr;
    } id_ex_reg;

    // Reg EX_MEM
    typedef struct packed{
        logic RegWrite;
        logic MemtoReg;
        logic MemRead;
        logic MemWrite;
        logic Branch;
        logic [31:0] PCBranch;
        logic [31:0] PCPlus4;
        logic [31:0] ExtImm;
        logic [31:0] ALUResult;
        logic [31:0] Reg2;
        logic [4:0] rd;
        logic [2:0] func3;
        logic [6:0] func7;
        logic [31:0] Instr;
    } ex_mem_reg;
    
    // Reg MEM_WB
    typedef struct packed{
        logic RegWrite;
        logic MemtoReg;
        logic Branch;
        logic [31:0] PCPlus4;
        logic [31:0] ExtImm;
        logic [31:0] ALUResult;
        logic [31:0] ReadData;
        logic [4:0] rd;
        logic [31:0] Instr;
    } mem_wb_reg;
endpackage
