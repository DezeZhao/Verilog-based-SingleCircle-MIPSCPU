`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:16:15
// Design Name: 
// Module Name: pcreg
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


 module pcreg(
    input clk,
    input rst,
    input ena,
    input wena,
    input [31:0]data_in,
    output[31:0]data_out
    );
    reg  [31:0]data;
    always @(negedge clk or posedge rst)
        if (rst)
            data <= 32'h00400000;
        else if (ena & wena)
            data <= data_in;
    assign data_out = data;
endmodule
