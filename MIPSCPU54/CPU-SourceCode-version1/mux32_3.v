`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:19:41
// Design Name: 
// Module Name: mux32_3
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


module mux32_3(//deselect 32bits data
    input [31:0]a,
    input [31:0]b,
    input [4:0]opt,
    output reg[31:0]c
    );
    always @(*) begin
    case(opt)
    5'b00000,5'b00100,5'b00010,5'b10000,5'b01000:c<=a;
    5'b00001:c<=b;
    endcase
    end
endmodule
