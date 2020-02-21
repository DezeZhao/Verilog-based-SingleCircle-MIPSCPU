`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:21:38
// Design Name: 
// Module Name: mux5_2
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


module mux5_2(//de-select 5bits address
    input [4:0]a,
    input [4:0]b,
    input opt,
    output reg[4:0]c
    );
    always @(*)begin
    case (opt)
       1'b0:c<=a;
       1'b1:c<=b;
    endcase
    end
endmodule
