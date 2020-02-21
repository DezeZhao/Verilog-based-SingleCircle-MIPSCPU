`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:20:21
// Design Name: 
// Module Name: mux32_4
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

module mux32_4(//deselect 32bits data
    input [31:0]a,
    input [31:0]b,
    input [31:0]c,
    input [31:0]d,
    input [3:0]opt,
    output reg[31:0]e
    );
    always @(*) begin
    case(opt)
    4'b0000:e<=a;
    4'b0001:e<=b;
    4'b0010:e<=c;
    4'b1000,4'b0100:e<=d;
    endcase
    end
endmodule
