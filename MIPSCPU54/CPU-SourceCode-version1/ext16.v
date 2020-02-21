`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:18:58
// Design Name: 
// Module Name: ext16
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

module ext16#(parameter WIDTH =16)(
   input [WIDTH - 1:0] a,
   input s_ext,
   output [31:0] b
   );
   assign b=s_ext?{ { (32-WIDTH){a[WIDTH-1]}},a}:{16'b0,a};
endmodule
