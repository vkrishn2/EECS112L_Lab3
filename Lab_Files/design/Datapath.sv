`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/07/2018 10:10:33 PM
// Design Name: 
// Module Name: Datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "Pipe_Reg.sv"
import Pipe_Reg_PKG::*;

module Datapath #(
    parameter PC_W = 9, // Program Counter
    parameter INS_W = 32, // Instruction Width
    parameter RF_ADDRESS = 5, // Register File Address
    parameter DATA_W = 32, // Data WriteData
    parameter DM_ADDRESS = 9, // Data Memory Address
    parameter ALU_CC_W = 4 // ALU Control Code Width
    )(
    input logic clk , reset , // global clock
                              // reset , sets the PC to zero
    RegWrite , MemtoReg ,     // Register file writing enable   // Memory or ALU MUX
    ALUsrc , MemWrite ,       // Register file or Immediate MUX // Memroy Writing Enable
    MemRead ,                 // Memroy Reading Enable
    Branch ,  AUIPC,
    input logic [1:0] Forward_ControlA,
    input logic [1:0] Forward_ControlB,
    input logic ld_hazard,
    input logic [ ALU_CC_W -1:0] ALU_CC, // ALU Control Code ( input of the ALU )
    output logic [6:0] opcode,
    output logic [6:0] Funct7,
    output logic [2:0] Funct3,
    output logic [DATA_W-1:0] WB_Data, //ALU_Result
    output logic [4:0] Reg1_id,
    output logic [4:0] Reg2_id,
    output logic [4:0] ex_mem_rd,
    output logic [4:0] mem_wb_rd,
    output logic ID_EX_MemRead,
    output logic [4:0] ID_EX_rd,
    output logic [4:0] IF_ID_rs1,
    output logic [4:0] IF_ID_rs2
    );

logic [PC_W-1:0] PC, PCPlus4;
logic [INS_W-1:0] Instr;
logic [DATA_W-1:0] Result, Result2;
logic [DATA_W-1:0] Reg1, Reg2;
logic [DATA_W-1:0] ByteOutput, HalfOutput, NotWordOutput, LoadOutput, StoreNotWordOutput, StoreOutput, RfInput, DataMemInput;
logic [DATA_W-1:0] ReadData;
logic [DATA_W-1:0] SrcB, SrcA, ALUResult, SrcA2, SrcB2;
logic [DATA_W-1:0] ExtImm;
logic [PC_W-1:0] PCNext3;
logic [PC_W-1:0] PCBranch, PCNext, PCNext2;

if_id_reg A;
id_ex_reg B;
ex_mem_reg C;
mem_wb_reg D;

// next PC
    adder #(PC_W) pcadd (PC, 9'b100, PCPlus4);
    mux2  #(PC_W) hazard_mux(PCNext2, PC, ld_hazard, PCNext3);
    flopr #(PC_W) pcreg(clk, reset, PCNext3, PC);
    mux2  #(PC_W) jumpmux2(PCNext, {ALUResult[8:1], 1'b0}, (B.Branch & B.Instr[3:2] == 2'b01), PCNext2);

 //Instruction memory
    instructionmemory instr_mem (PC, Instr);

/*---------------------------------------------IF/ID------------------------------------------*/
always @(posedge clk) 
    begin
    if(ld_hazard)
      begin
        //Nothing happens
      end
    else if(reset || ((B.Branch & ALUResult[0]) || (B.Branch & B.Instr[2])) || (B.Branch & B.Instr[3:2] == 2'b01))
        //if (reset == 1)   // initialization or flush
        begin
            A.PC <= 0;
            A.PCPlus4 <= 0;
            A.Instr <= 0;
        end
        else    // stall
        begin
            A.PC <= PC;
            A.PCPlus4 <= PCPlus4;
            A.Instr <= Instr;
        end
    end
/*--------------------------------------------------------------------------------------------*/
    
    assign opcode = A.Instr[6:0];
    assign Funct7 = A.Instr[31:25];
    assign Funct3 = A.Instr[14:12];
    assign IF_ID_rs1 = A.Instr[19:15];
    assign IF_ID_rs2 = A.Instr[24:20];
      
// //Register File
    RegFile rf(clk, reset, D.RegWrite & (D.Instr[11]|D.Instr[10]|D.Instr[9]|D.Instr[8]|D.Instr[7]), D.rd, A.Instr[19:15], A.Instr[24:20],
            Result2, Reg1, Reg2);

           
//// sign extend
    imm_Gen Ext_Imm (A.Instr,ExtImm);
    adder #(PC_W) branchadd (A.PC, ExtImm[8:0], PCBranch);

/*-------------------------------------------------ID/EX------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset || ((B.Branch & ALUResult[0]) || (B.Branch & B.Instr[2])) || (B.Branch & B.Instr[3:2] == 2'b01) || ld_hazard)// initialization or flush or generate a NOP if hazard
        begin
            B.ALUSrc <= 0; 
            B.MemtoReg <= 0;
            B.RegWrite <= 0; 
            B.MemRead <= 0;
            B.MemWrite <= 0;
            B.ALU_CC <= 0;
            B.Branch <= 0;
            B.AUIPC <= 0;
            B.PC <= 0;
            B.PCBranch <= 0;
            B.PCPlus4 <= 0;
            B.Reg1_id <= 0;
            B.Reg2_id <= 0;
            B.Reg1 <= 0;
            B.Reg2 <= 0;
            B.rd <= 0;
            B.ExtImm <= 0;
            B.func3 <= 0;
            B.func7 <= 0;
             B.Instr <= 0;
        end
        else
        begin
            B.ALUSrc <= ALUsrc;
            B.MemtoReg <= MemtoReg;
            B.RegWrite <= RegWrite;
            B.MemRead <= MemRead;
            B.MemWrite <= MemWrite;
            B.ALU_CC <= ALU_CC;
            B.Branch <= Branch;
            B.AUIPC <= AUIPC;
            B.PC <= A.PC;
            B.PCBranch <= PCBranch;
            B.PCPlus4 <= A.PCPlus4;
            B.Reg1_id <= A.Instr[19:15];
            B.Reg2_id <= A.Instr[24:20];
            B.Reg1 <= Reg1;
            B.Reg2 <= Reg2;
            B.rd <= A.Instr[11:7];
            B.ExtImm <= ExtImm;
            B.func3 <= A.Instr[14:12];
            B.func7 <= A.Instr[31:25];
            B.Instr <= A.Instr;   //debug tmp
        end
    end
/*------------------------------------------------------------------------------------------------------*/

//// ALU
    mux3 #(32) srca2mux(B.Reg1, C.ALUResult, Result2, Forward_ControlA, SrcA);
    mux3 #(32) srcb2mux(B.Reg2, C.ALUResult, Result2, Forward_ControlB, SrcB);
    
    mux2 #(32) srcamux(SrcA, {23'b0,B.PC}, B.AUIPC, SrcA2);
    mux2 #(32) srcbmux(SrcB, B.ExtImm, B.ALUSrc, SrcB2);
    

    alu alu_module(SrcA2, SrcB2, B.ALU_CC, ALUResult);

    mux2 #(PC_W) branchmux(PCPlus4, B.PCBranch, ((B.Branch & ALUResult[0]) || (B.Branch & B.Instr[2])), PCNext);

    
    assign WB_Data = Result;
    assign Reg1_id = B.Reg1_id;
    assign Reg2_id = B.Reg2_id;
    assign ID_EX_MemRead = B.MemRead;
    assign ID_EX_rd = B.rd;

/*-----------------------------------------------EX/MEM--------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization
        begin
            C.RegWrite <= 0;
            C.MemtoReg <= 0;
            C.MemRead <= 0;
            C.MemWrite <= 0;
            C.Branch <= 0;
            C.PCBranch <= 0;
            C.PCPlus4 <= 0;
            C.ExtImm <= 0;
            C.ALUResult <= 0;
            C.Reg2 <= 0;
            C.rd <= 0;
            C.func3 <= 0;
            C.func7 <= 0;
            C.Instr <= 0;
        end
        else
        begin
            C.RegWrite <= B.RegWrite;
            C.MemtoReg <= B.MemtoReg;
            C.MemRead <= B.MemRead;
            C.MemWrite <= B.MemWrite;
            C.Branch <= B.Branch;
            C.PCBranch <= PCBranch;
            C.PCPlus4 <= B.PCPlus4;;
            C.ExtImm <= B.ExtImm;
            C.ALUResult <= ALUResult;
            C.Reg2 <= SrcB;
            C.rd <= B.rd;
            C.func3 <= B.func3;
            C.func7 <= B.func7;
            C.Instr <= B.Instr;   // debug tmp
        end
    end
/*-------------------------------------------------------------------------------------------------------*/
    
////// Data memory 
  assign ex_mem_rd = C.rd;

  //mux2 #(PC_W) branchmux(PCPlus4, C.PCBranch, ((C.Branch & C.ALUResult[0]) || (C.Branch & C.Instr[2])), PCNext);

	datamemory data_mem (clk, C.MemRead, C.MemWrite, C.ALUResult[DM_ADDRESS-1:0], C.Reg2, C.func3, ReadData);

	  //mux2 #(32) st3mux(Reg2, StoreOutput,(Instr[6:0]== 7'b0100011), DataMemInput);
    //mux2 #(32) st1mux(StoreNotWordOutput, C.Reg2 , C.Instr[13], StoreOutput);
    //mux2 #(32) st2mux({C.Reg2[7] ? 24'b111111111111111111111111:24'b0, C.Reg2[7:0]}, {C.Reg2[15] ? 16'b1111111111111111:16'b0, C.Reg2[15:0]}, C.Instr[12], StoreNotWordOutput);

/*----------------------------------------------MEM/WB---------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization
        begin
            D.RegWrite <= 0;
            D.MemtoReg <= 0;
            D.Branch <= 0;
            D.PCPlus4 <= 0;
            D.ExtImm <= 0;
            D.ALUResult <= 0;
            D.ReadData <= 0;
            D.rd <= 0;
            D.Instr <= 0;
        end
        else
        begin
            D.RegWrite <= C.RegWrite;
            D.MemtoReg <= C.MemtoReg;
            D.Branch <= C.Branch;
            D.PCPlus4 <= C.PCPlus4;
            D.ExtImm <= C.ExtImm;
            D.ALUResult <= C.ALUResult;
            D.ReadData <= ReadData;
            D.rd <= C.rd;
             D.Instr <= C.Instr;   //Debug Tmp
        end
    end
/*-------------------------------------------------------------------------------------------------------*/
   assign mem_wb_rd = D.rd;

   mux2 #(32) resmux(D.ALUResult, D.ReadData, D.MemtoReg, Result);
   mux2 #(32) jumpmux(Result, {23'b0,D.PCPlus4}, ( D.Branch & D.Instr[2]), Result2);

   /*mux2 #(32) ld4mux(Result2, LoadOutput, (D.Instr[6:0] == 7'b0000011), RfInput);
   mux2 #(32) ld1mux(NotWordOutput, Result2, D.Instr[13], LoadOutput);
   mux2 #(32) ld2mux(ByteOutput,HalfOutput, D.Instr[12], NotWordOutput);
   mux2 #(32) ld3_1mux({Result2[15] ? 16'b1111111111111111:16'b0, Result2[15:0]}, {16'b0, Result2[15:0]}, (D.Instr[14:12] == 3'b101), HalfOutput);
   mux2 #(32) ld3_2mux({Result2[7] ?  24'b111111111111111111111111:24'b0, Result2[7:0]}, {24'b0, Result2[7:0]}, (D.Instr[14:12] == 3'b100), ByteOutput);*/


endmodule

