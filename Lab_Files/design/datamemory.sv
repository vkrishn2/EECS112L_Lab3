`timescale 1ns / 1ps

module datamemory#(
    parameter DM_ADDRESS = 9 ,
    parameter DATA_W = 32
    )(
    input logic clk,
	input logic MemRead , // comes from control unit
    input logic MemWrite , // Comes from control unit
    input logic [ DM_ADDRESS -1:0] a , // Read / Write address - 9 LSB bits of the ALU output
    input logic [ DATA_W -1:0] wd , // Write Data
    output logic [ DATA_W -1:0] rd // Read Data
    );
    
    //ifdef _SIM_
    logic [DATA_W-1:0] mem [(2**DM_ADDRESS)-1:0];

    always_comb 
    begin
       if(MemRead)
            rd = mem[a];
	end
    
    always @(posedge clk) begin
       if (MemWrite)
            mem[a] = wd;
    end
   
    /*else
      logic we;
      assign we = MemWrite;

      SRAM1RW512x32 RAM (
        .A       ( a[8:0] ),
        .CE      ( 1'b1   ),
        .WEB     ( ~we    ),
        .OEB     ( we     ),
        .CSB     ( 1'b0   ),
        .I       ( wd     ),
        .O       ( rd     )
        );
     endif*/

endmodule

