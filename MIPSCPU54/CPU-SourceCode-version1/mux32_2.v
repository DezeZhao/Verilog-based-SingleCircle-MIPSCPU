`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:18:17
// Design Name: 
// Module Name: mux32_2
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


module mux32_2(//deselect 32bits data
    input [31:0]a,
    input [31:0]b,
    input [6:0]opt,
    output reg[31:0]c
    );
    always @(*) begin
    case(opt)
    7'b0000000:c<=a;
    7'b0000001,7'b0000010,7'b0000100,7'b0001000,7'b0010000,7'b1000000,7'b0100000:c<=b;
    endcase
    end
endmodule
