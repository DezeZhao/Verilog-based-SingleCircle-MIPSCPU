`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:15:32
// Design Name: 
// Module Name: mux32_1
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

module mux32_1(//deselect 32bits data
    input [31:0]a,
    input [31:0]b,
    input [31:0]c,
	input [31:0]d,//hi
	input [31:0]e,//lo
	input [31:0]f,
	input [31:0]g,
    input [6:0]opt,
    output reg[31:0]h
    );
    always@(*)begin
    case(opt)
    7'b0000000:h<=a;
    7'b0000001,7'b0100000:h<=b;
    7'b0000010:h<=c;
	7'b0001000:h<=d;
	7'b0000100:h<=e;
	7'b0010000:h<=f;
 	7'b1000000:h<=g;
    endcase
  end
endmodule
