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

// next PC
    adder #(32) pcadd (PC, 32'b100, PCPlus4);
    flopr #(32) pcreg(clk, reset, PCNext2, PC);
    mux2  #(32) jumpmux2(PCNext, {ALUResult[31:1], 1'b0}, (Branch & Instr[3:2] == 2'b01), PCNext2);

 //Instruction memory
    instructionmemory instr_mem (PC, Instr);
    
    assign opcode = Instr[6:0];
    assign Funct7 = Instr[31:25];
    assign Funct3 = Instr[14:12];
      
// //Register File
    RegFile rf(clk, reset, RegWrite & (Instr[11] | Instr[10] | Instr[9] | Instr[8] | Instr[7]), Instr[11:7], Instr[19:15], Instr[24:20],
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
    mux2 #(32) branchmux(PCPlus4, PCBranch, ((Branch & ALUResult[0]) || (Branch & Instr[2])), PCNext);

//// ALU
    mux2 #(32) srcamux(Reg1, PC, AUIPC, SrcA);
    mux2 #(32) srcbmux(Reg2, ExtImm, ALUsrc, SrcB);
    alu alu_module(SrcA, SrcB, ALU_CC, ALUResult);
    
    assign WB_Data = Result;
    
////// Data memory 
	datamemory data_mem (clk, MemRead, MemWrite, ALUResult[DM_ADDRESS-1:0], StoreOutput, ReadData);

	  //mux2 #(32) st3mux(Reg2, StoreOutput,(Instr[6:0]== 7'b0100011), DataMemInput);
    mux2 #(32) st1mux(StoreNotWordOutput, Reg2 , Instr[13], StoreOutput);
    mux2 #(32) st2mux({Reg2[7] ? 24'b111111111111111111111111:24'b0, Reg2[7:0]}, {Reg2[15] ? 16'b1111111111111111:16'b0, Reg2[15:0]}, Instr[12], StoreNotWordOutput);

     
endmodule
