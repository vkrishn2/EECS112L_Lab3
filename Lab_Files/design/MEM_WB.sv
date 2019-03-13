module MEM_WB#
  (parameter WIDTH = 32)
  (input logic clk, reset,
   input logic [WIDTH-1:0] instr,
   input logic MemtoReg,
   input logic RegWrite
   output logic MemtoReg_o,
   output logic RegWrite_o,
   output logic [WIDTH-1:0] instr_o);

   always_ff @(posedge clk, posedge reset)
   if (reset)
    begin
      MemtoReg_o <= 0;
      RegWrite_o <= 0;
      instr_o <= 32'b0;
    end
   else
    begin
      MemtoReg_o <= MemtoReg;
      RegWrite_o <= RegWrite;
      instr_o <= instr;
    end

endmodule
