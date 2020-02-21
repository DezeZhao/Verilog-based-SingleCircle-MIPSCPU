`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:21:00
// Design Name: 
// Module Name: mux5_1
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



module mux5_1(//de-select 5bits address
    input [4:0]a,
    input [4:0]b,
    input [7:0]opt,
    output reg[4:0]c
    );
    always @(*)begin
    case (opt)
       8'b00000000,8'b00010000,8'b00001000,8'b00100000,8'b01000000,8'b10000000:c<=a;//rd
       8'b00000001,8'b00000100:c<=b;//rt
       8'b00000010,8'b00000011:c<=5'b11111;//store address in 31th reg 
    endcase
    end
endmodule
