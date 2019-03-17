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
    
`ifdef __SIM__
    logic [DATA_W-1:0] mem [(2**DM_ADDRESS)-1:0];

    always_comb 
    begin
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
   
    `else
      logic we;
      assign we = MemWrite;
      
      logic [7:0] rd1, rd2, rd3, rd4;
      logic re1, re2, re3, re4;
      
      always_comb 
      begin
        if(MemRead)
        begin
            case(Funct3) 
             3'b100: //LBU
              begin
                rd = {24'b0, rd1}; 
                re1 = 1'b1;
                re2 = 1'b0;
                re3 = 1'b0;
                re4 = 1'b0;
              end
            3'b000: //LB
              begin
                rd = {rd1[7]? 24'b111111111111111111111111:24'b0, rd1};
                re1 = 1'b1;
                re2 = 1'b0;
                re3 = 1'b0;
                re4 = 1'b0;
              end
            3'b101: //LHU
              begin
                rd = {16'b0, rd2, rd1};  
                re1 = 1'b1;
                re2 = 1'b1;
                re3 = 1'b0;
                re4 = 1'b0; 
              end
            3'b001: //LH
              begin
                rd = { rd2[7]? 16'b1111111111111111:16'b0, rd2, rd1};
                re1 = 1'b1;
                re2 = 1'b1;
                re3 = 1'b0;
                re4 = 1'b0;
              end
            3'b010: //LW
              begin
                rd = {rd4,rd3,rd2,rd1};
                re1 = 1'b1;
                re2 = 1'b1;
                re3 = 1'b1;
                re4 = 1'b1;
              end
            default:
              begin
                rd = {rd4,rd3,rd2,rd1};
                re1 = 1'b1;
                re2 = 1'b1;
                re3 = 1'b1;
                re4 = 1'b1;
              end
            endcase
        end
        else
        begin
            rd = {rd4,rd3,rd2,rd1};
            re1 = 1'b1;
            re2 = 1'b1;
            re3 = 1'b1;
            re4 = 1'b1;
        end
      end
      


      SRAM1RW512x8 RAM (
        .A       ( a[8:0] ),
        .CE      ( clk    ),
        .WEB     ( ~we    ),
        .OEB     ( we     ),
        .CSB     ( re1    ),
        .I       ( wd[7:0]),
        .O       ( rd1    )
        );
        
        SRAM1RW512x8 RAM1 (
        .A       ( a[8:0] ),
        .CE      ( clk    ),
        .WEB     ( ~we    ),
        .OEB     ( we     ),
        .CSB     ( re2    ),
        .I       ( wd[15:8]),
        .O       ( rd2    )
        );
        
        SRAM1RW512x8 RAM2 (
        .A       ( a[8:0] ),
        .CE      ( clk    ),
        .WEB     ( ~we    ),
        .OEB     ( we     ),
        .CSB     ( re3    ),
        .I       ( wd[23:16]),
        .O       ( rd3    )
        );
        
        SRAM1RW512x8 RAM3 (
        .A       ( a[8:0] ),
        .CE      ( clk    ),
        .WEB     ( ~we    ),
        .OEB     ( we     ),
        .CSB     ( re4    ),
        .I       ( wd[31:24]),
        .O       ( rd4    )
        );
        
    `endif

endmodule
