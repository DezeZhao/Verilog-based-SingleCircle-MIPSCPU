`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:23:33
// Design Name: 
// Module Name: MULTU
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


module MULTU(
    input clk,
    input reset,
    input start,
    input [31:0] a,
    input [31:0] b,
    output [63:0] z
    );
    
    wire [65:0] temp;
    assign temp={1'b0,a}*{1'b0,b};
    assign z=temp[63:0];
endmodule
