module EX_MEM#
  (parameter WIDTH = 32)
  (input logic clk, reset,
   input logic [WIDTH-1:0] instr,
   input logic [WIDTH-1:0] PC,
   input logic MemtoReg,
   input logic RegWrite
   input logic MemRead,
   input logic MemWrite,
   input logic Branch,
   input logic [WIDTH-1:0] ALU_Result,
   output logic MemtoReg_o,
   output logic RegWrite_o,
   output logic MemRead_o,
   output logic MemWrite_o,
   output logic Branch_o,
   output logic [WIDTH-1:0] ALU_Result_o,
   output logic [WIDTH-1:0] instr_o,
   output logic [WIDTH-1:0] PC_o);

   always_ff @(posedge clk, posedge reset)
   if (reset)
    begin
      MemtoReg_o <= 0;
      RegWrite_o <= 0;
      MemRead_o <= 0;
      MemWrite_o <= 0;
      Branch_o <= 0;
      ALU_Result_o <= 32'b0;
      instr_o <= 32'b0;
      PC_o <= 32'b0;
    end
   else
    begin
      MemtoReg_o <= MemtoReg;
      RegWrite_o <= RegWrite;
      MemRead_o <= MemRead;
      MemWrite_o <= MemWrite;
      Branch_o <= Branch;
      ALU_Result_o <= ALU_Result;
      instr_o <= instr;
      PC_o <= PC;
    end

endmodule
