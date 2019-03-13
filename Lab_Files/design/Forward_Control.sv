`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/07/2018 10:10:33 PM
// Design Name: 
// Module Name: Controller
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

module Forward_Control(
    
    //Input
    input logic [4:0] Reg1_id,
    input logic [4:0] Reg2_id,
    input logic [4:0] ex_mem_rd,
    input logic [4:0] mem_wb_rd,
    
    //Outputs
    output logic [1:0] ControlA, 
    output logic [1:0] ControlB
);

always_comb
  begin

  if(Reg1_id == ex_mem_rd)
    ControlA = 2'b01;
  else if(Reg1_id == mem_wb_rd)
    ControlA = 2'b10;
  else
    ControlA = 2'b00;
  
  if(Reg2_id == ex_mem_rd)
    ControlB = 2'b01;
  else if(Reg2_id == mem_wb_rd)
    ControlB = 2'b10;
  else
    ControlB = 2'b00;

  end

endmodule
