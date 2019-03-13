module IF_ID#
  (parameter WIDTH = 32)
  (input logic clk, reset, Reg_Stall
   input logic [WIDTH-1:0] Instr,
   input logic [WIDTH-1:0] PC,
   output logic [WIDTH-1:0] Curr_Instr,
   output logic [WIDTH-1:0] Curr_PC);

   always @(posedge clk) 
    begin
        if (reset)   // initialization or flush
        begin
            Curr_PC <= 0;
            Curr_Instr <= 0;
        end
        else if (!Reg_Stall)    // stall
        begin
            Curr_PC <= PC;
            Curr_Instr <= Instr;
        end
    end

endmodule
