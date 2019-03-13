module ID_EX#
  (parameter WIDTH = 32)
  (input logic clk, reset,
   input logic [WIDTH-1:0] instr,
   input logic [WIDTH-1:0] PC,
   input logic ALUSrc,
   input logic MemtoReg,
   input logic RegWrite,
   input logic MemRead,
   input logic MemWrite,
   input logic [2:0] ALUOp,
   input logic Branch,
   input logic AUIPC,
   output logic ALUSrc_o,
   output logic MemtoReg_o,
   output logic RegWrite_o,
   output logic MemRead_o,
   output logic MemWrite_o,
   output logic [2:0] ALUOp_o,
   output logic Branch_o,
   output logic AUIPC_o,
   output logic [WIDTH-1:0] instr_o,
   output logic [WIDTH-1:0] PC_o);

   always_ff @(posedge clk, posedge reset)
   if (reset)
    begin
      ALUSrc_o <= 0;
      MemtoReg_o <= 0;
      RegWrite_o <= 0;
      MemRead_o <= 0;
      MemWrite_o <= 0;
      ALUOp_o <= 2'b00;
      Branch_o <= 0;
      AUIPC_o <= 0;
      instr_o <= 32'b0;
      PC_o <= 32'b0;
    end
   else
    begin
      ALUSrc_o <= ALUSrc;
      MemtoReg_o <= MemtoReg;
      RegWrite_o <= RegWrite;
      MemRead_o <= MemRead;
      MemWrite_o <= MemWrite;
      ALUOp_o <= ALUOp;
      Branch_o <= Branch;
      AUIPC_o <= AUIPC;
      instr_o <= instr;
      PC_o <= PC;
    end

endmodule
