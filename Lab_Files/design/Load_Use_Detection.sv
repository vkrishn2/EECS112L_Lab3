`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/07/2018 10:10:33 PM
// Design Name: 
// Module Name: Load_Use_Detection
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

module Load_Use_Detection(
    
    //Input
    input logic ID_EX_MemRead,
    input logic [4:0] ID_EX_rd,
    input logic [4:0] IF_ID_rs1,
    input logic [4:0] IF_ID_rs2,
    
    //Outputs
    output logic ld_stall 
);

always_comb
  begin
  
  if((ID_EX_MemRead & ((ID_EX_rd == IF_ID_rs1))) | (ID_EX_MemRead &(ID_EX_rd == IF_ID_rs2)))
    ld_stall = 1;
  else
    ld_stall = 0;

  end

endmodule
