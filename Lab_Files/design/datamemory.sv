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
    input logic [2:0] Funct3,
    output logic [ DATA_W -1:0] rd // Read Data
    );
    
    //ifdef _SIM_
    logic [DATA_W-1:0] mem [(2**DM_ADDRESS)-1:0];

    always_comb 
    begin
      rd = 32'b0;
       if(MemRead)
        begin
            case(Funct3)
            3'b100: //LBU
                rd = {24'b0, mem[a][7:0]};           
            3'b000: //LB
                rd = {mem[a][7]? 24'b111111111111111111111111:24'b0, mem[a][7:0]};
            3'b101: //LHU
                rd = {16'b0, mem[a][15:0]};                 
            3'b001: //LH
                rd = {mem[a][15]? 16'b1111111111111111:16'b0, mem[a][15:0]};
            3'b010: //LW
                rd = mem[a];
            default:
                rd = mem[a];
            endcase
        end
	end
    
    always @(posedge clk) 
    begin
       if (MemWrite)
        begin
            case(Funct3)
            3'b000: //SB
                mem[a][7:0] =  wd[7:0];
            3'b001: //SH
                mem[a][15:0] = wd[15:0];
            3'b010: //SW
                mem[a] = wd;
            default:
                mem[a] = wd;
            endcase
        end
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

