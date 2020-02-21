`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:14:49
// Design Name: 
// Module Name: mux32
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


module mux32(//deselect 32bits data
    input [31:0]a,
    input [31:0]b,
    input opt,
    output reg[31:0]c
    );
    always @(*) begin
    case(opt)
    1'b0:c<=a;
    1'b1:c<=b;
    endcase
    end
endmodule
