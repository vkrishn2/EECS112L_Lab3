`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/07/2018 10:23:43 PM
// Design Name: 
// Module Name: alu
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


module alu#(
        parameter DATA_WIDTH = 32,
        parameter OPCODE_LENGTH = 4
        )(
        input logic [DATA_WIDTH-1:0]    SrcA,
        input logic [DATA_WIDTH-1:0]    SrcB,

        input logic [OPCODE_LENGTH-1:0]    Operation,
        output logic[DATA_WIDTH-1:0] ALUResult
        );
    
        always_comb
        begin
            ALUResult = 'd0;
            case(Operation)
            4'b0000:        // AND
                    ALUResult = SrcA & SrcB;
            4'b0001:        //OR
                    ALUResult = SrcA | SrcB;
            4'b0010:        //ADD
                    ALUResult = SrcA + SrcB;
	          4'b0011:        //XOR
	                  ALUResult=SrcA^SrcB;
            4'b0110:        //Subtract
                    ALUResult = $signed(SrcA) - $signed(SrcB);
            /*Missing Operations*/
            4'b0100:
                    //Shift Left
                    ALUResult = SrcA << $signed(SrcB);
            4'b0101:
                    //Shift Right
                    ALUResult = SrcA >> $signed(SrcB);
            4'b0111:
                    //Shift Right with sign
                    ALUResult = SrcA >>>$signed(SrcB);
            /*Branch equality check*/
            4'b1000:
                    //branch if equal
                    ALUResult = ($signed(SrcA) == $signed(SrcB));
            4'b1001:
                    //branch if not equal
                    ALUResult = ($signed(SrcA) != $signed(SrcB));
            4'b1010:
                    //branch if less than
                    ALUResult = $signed(SrcA) < $signed(SrcB);
            4'b1011:
                    //branch if Greater than or equal to
                    ALUResult = $signed(SrcA) >= $signed(SrcB);
            4'b1100:
                    //branch if less than unsigned
                    ALUResult = $unsigned(SrcA) < $unsigned(SrcB);
            4'b1101:
                    //branch if greater than or equal unsigned
                    ALUResult = $unsigned(SrcA) >= $unsigned(SrcB);
            4'b1110:
                    //LUI
                    ALUResult = $signed(SrcB);
            default:
                    ALUResult = 'b0;
            endcase
        end
endmodule

