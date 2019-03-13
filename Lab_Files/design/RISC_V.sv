`timescale 1ns / 1ps

module riscv #(
    parameter DATA_W = 32)
    (input logic clk, reset, // clock and reset signals
    output logic [31:0] WB_Data// The ALU_Result
    );

logic [6:0] opcode;
logic ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ld_hazard;

logic [2:0] ALUop;
logic [6:0] Funct7;
logic [2:0] Funct3;
logic [3:0] Operation;

//My new control signals
logic [1:0] Forward_ControlA, Forward_ControlB;
logic [4:0] Reg1_id, Reg2_id, ex_mem_rd, mem_wb_rd,ID_EX_rd,IF_ID_rs1,IF_ID_rs2;
logic ID_EX_MemRead;




    Controller c(opcode, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, ALUop, Branch, AUIPC );
    
    ALUController ac(ALUop, Funct7, Funct3, Operation);
    
    Load_Use_Detection lud(ID_EX_MemRead,ID_EX_rd,IF_ID_rs1,IF_ID_rs2,ld_hazard);

    Forward_Control fc(Reg1_id, Reg2_id, ex_mem_rd, mem_wb_rd, Forward_ControlA, Forward_ControlB);

    Datapath dp(clk, reset, RegWrite , MemtoReg, ALUSrc , MemWrite, MemRead, Branch, AUIPC,
                Forward_ControlA, Forward_ControlB, ld_hazard, Operation, opcode, Funct7, Funct3, WB_Data,
                Reg1_id, Reg2_id, ex_mem_rd, mem_wb_rd, ID_EX_MemRead, ID_EX_rd, IF_ID_rs1, IF_ID_rs2);
        
endmodule
