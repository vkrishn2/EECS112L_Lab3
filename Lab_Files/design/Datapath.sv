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
`include "RegPack.sv"
import Pipe_Reg_PKG::*;

module Datapath #(
    parameter PC_W = 32, // Program Counter
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
    input logic [ ALU_CC_W -1:0] ALU_CC, // ALU Control Code ( input of the ALU )
    output logic [6:0] opcode,
    output logic [6:0] Funct7,
    output logic [2:0] Funct3,
    output logic [DATA_W-1:0] WB_Data //ALU_Result
    );

logic [PC_W-1:0] PC, PCPlus4;
logic [INS_W-1:0] Instr;
logic [DATA_W-1:0] Result, Result2;
logic [DATA_W-1:0] Reg1, Reg2;
logic [DATA_W-1:0] ByteOutput, HalfOutput, NotWordOutput, LoadOutput, StoreNotWordOutput, StoreOutput, RfInput, DataMemInput;
logic [DATA_W-1:0] ReadData;
logic [DATA_W-1:0] SrcB, SrcA, ALUResult;
logic [DATA_W-1:0] ExtImm;
logic [PC_W-1:0] PCBranch, PCNext, PCNext2;

if_id_reg A;
id_ex_reg B;
ex_mem_reg C;
mem_wb_reg D

// next PC
    adder #(32) pcadd (PC, 32'b100, PCPlus4);
    flopr #(32) pcreg(clk, reset, PCNext2, PC);
    mux2  #(32) jumpmux2(PCNext, {ALUResult[31:1], 1'b0}, (Branch & Instr[3:2] == 2'b01), PCNext2);

 //Instruction memory
    instructionmemory instr_mem (PC, Instr);

/*---------------------------------------------IF/ID------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization or flush
        begin
            A.Curr_Pc <= 0;
            A.Curr_Instr <= 0;
        end
        else if (!Reg_Stall)    // stall
        begin
            A.Curr_Pc <= PC;
            A.Curr_Instr <= Instr;
        end
    end
/*--------------------------------------------------------------------------------------------*/
    
    assign opcode = Instr[6:0];
    assign Funct7 = Instr[31:25];
    assign Funct3 = Instr[14:12];
      
// //Register File
    RegFile rf(clk, reset, RegWrite & (Instr[11]|Instr[10]|Instr[9]|Instr[8]|Instr[7]), Instr[11:7], Instr[19:15], Instr[24:20],
            RfInput, Reg1, Reg2);
            
    mux2 #(32) resmux(ALUResult, ReadData, MemtoReg, Result);

    //-----------------------------------ALUResult[0]---
    mux2 #(32) jumpmux(Result, PCPlus4, ( Branch & Instr[2]), Result2);

    mux2 #(32) ld4mux(Result2, LoadOutput, (Instr[6:0] == 7'b0000011), RfInput);
  	mux2 #(32) ld1mux(NotWordOutput, Result2, Instr[13], LoadOutput);
  	mux2 #(32) ld2mux(ByteOutput,HalfOutput, Instr[12], NotWordOutput);
    mux2 #(32) ld3_1mux({Result2[15] ? 16'b1111111111111111:16'b0, Result2[15:0]}, {16'b0, Result2[15:0]}, (Instr[14:12] == 3'b101), HalfOutput);
    mux2 #(32) ld3_2mux({Result2[7] ?  24'b111111111111111111111111:24'b0, Result2[7:0]}, {24'b0, Result2[7:0]}, (Instr[14:12] == 3'b100), ByteOutput);
           
//// sign extend
    imm_Gen Ext_Imm (Instr,ExtImm);
    adder #(32) branchadd (PC, ExtImm, PCBranch);

/*-------------------------------------------------ID/EX------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization or flush or generate a NOP if hazard
        begin
            B.ALUSrc <= 0; 
            B.MemtoReg <= 0;
            B.RegWrite <= 0; 
            B.MemRead <= 0;
            B.MemWrite <= 0;
            B.ALUOp <= 0;
            B.Branch <= 0;
            B.AUIPC <= 0;
            B.Curr_Pc <= 0;
            //B.RD_One <= 0;
            //B.RD_Two <= 0;
            B.RS_One <= 0;
            B.RS_Two <= 0;
            B.rd <= 0;
            B.ImmG <= 0;
            B.func3 <= 0;
            B.func7 <= 0;
             B.Curr_Instr <= A.Curr_Instr;   //debug tmp
        end
        else
        begin
            B.ALUSrc <= ALUsrc;
            B.MemtoReg <= MemtoReg;
            B.RegWrite <= RegWrite;
            B.MemRead <= MemRead;
            B.MemWrite <= MemWrite;
            B.ALUOp <= ALUOp;
            B.Branch <= Branch;
            B.AUIPC <= AUIPC;
            B.Curr_Pc <= A.Curr_Pc;
            //B.RD_One <= Reg1;
            //B.RD_Two <= Reg2;
            B.RS_One <= A.Curr_Instr[19:15];
            B.RS_Two <= A.Curr_Instr[24:20];
            B.rd <= A.Curr_Instr[11:7];
            B.ImmG <= ExtImm;
            B.func3 <= A.Curr_Instr[14:12];
            B.func7 <= A.Curr_Instr[31:25];
             B.Curr_Instr <= A.Curr_Instr;   //debug tmp
        end
    end
/*------------------------------------------------------------------------------------------------------*/

//// ALU
    mux2 #(32) srcamux(Reg1, PC, AUIPC, SrcA);
    mux2 #(32) srcbmux(Reg2, ExtImm, ALUsrc, SrcB);
    alu alu_module(SrcA, SrcB, ALU_CC, ALUResult);

    mux2 #(32) branchmux(PCPlus4, PCBranch, ((Branch & ALUResult[0]) || (Branch & Instr[2])), PCNext);
    
    assign WB_Data = Result;

/*-----------------------------------------------EX/MEM--------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization
        begin
            C.RegWrite <= 0;
            C.MemtoReg <= 0;
            C.MemRead <= 0;
            C.MemWrite <= 0;
            //C.RWSel <= 0;
            C.Curr_Pc <= 0;
            C.Pc_Four <= 0;
            C.Imm_Out <= 0;
            C.Alu_Result <= 0;
            //C.RD_Two <= 0;
            C.rd <= 0;
            C.func3 <= 0;
            C.func7 <= 0;
        end
        else
        begin
            C.RegWrite <= B.RegWrite;
            C.MemtoReg <= B.MemtoReg;
            C.MemRead <= B.MemRead;
            C.MemWrite <= B.MemWrite;
            //C.RWSel <= B.RWSel;
            C.Curr_Pc <= PCBranch;
            C.Pc_Four <= Old_PC_Four;
            C.Imm_Out <= B.ImmG;
            C.Alu_Result <= ALUResult;
            //C.RD_Two <= FBmux_Result;
            C.rd <= B.rd;
            C.func3 <= B.func3;
            C.func7 <= B.func7;
             C.Curr_Instr <= B.Curr_Instr;   // debug tmp
        end
    end
/*-------------------------------------------------------------------------------------------------------*/
    
////// Data memory 
	datamemory data_mem (clk, MemRead, MemWrite, ALUResult[DM_ADDRESS-1:0], StoreOutput, ReadData);

	  //mux2 #(32) st3mux(Reg2, StoreOutput,(Instr[6:0]== 7'b0100011), DataMemInput);
    mux2 #(32) st1mux(StoreNotWordOutput, Reg2 , Instr[13], StoreOutput);
    mux2 #(32) st2mux({Reg2[7] ? 24'b111111111111111111111111:24'b0, Reg2[7:0]}, {Reg2[15] ? 16'b1111111111111111:16'b0, Reg2[15:0]}, Instr[12], StoreNotWordOutput);

/*----------------------------------------------MEM/WB---------------------------------------------------*/
always @(posedge clk) 
    begin
        if (reset)   // initialization
        begin
            D.RegWrite <= 0;
            D.MemtoReg <= 0;
            //D.RWSel <= 0;
            D.Pc_Imm <= 0;
            D.Pc_Four <= 0;
            D.Imm_Out <= 0;
            D.Alu_Result <= 0;
            D.MemReadData <= 0;
            D.rd <= 0;
        end
        else
        begin
            D.RegWrite <= C.RegWrite;
            D.MemtoReg <= C.MemtoReg;
            //D.RWSel <= C.RWSel;
            D.Curr_Pc <=C.Curr_Pc;
            D.Pc_Four <= C.Pc_Four;
            D.Imm_Out <= C.Imm_Out;
            D.Alu_Result <= C.Alu_Result;
            D.MemReadData <= ReadData;
            D.rd <= C.rd;
             D.Curr_Instr <= C.Curr_Instr;   //Debug Tmp
        end
    end
/*-------------------------------------------------------------------------------------------------------*/
     
endmodule
