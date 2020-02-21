`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 14:58:29
// Design Name: 
// Module Name: Divider
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
/////////////////////////////////////////////////////////////////////////////////
module Divider(
    input I_CLK,
    input rst,
    output reg O_CLK
);
    reg [31:0]  i;
    parameter N=2;
    always@(posedge I_CLK or posedge rst)
    begin
      if(rst) begin
          i=0;
          O_CLK<=0;
      end
      else if(i==N-1) begin
          O_CLK<=~O_CLK;
          i=0;
      end
      else begin
          i=i+1;
      end 
   end
 endmodule
