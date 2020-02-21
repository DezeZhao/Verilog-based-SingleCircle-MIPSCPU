`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:22:46
// Design Name: 
// Module Name: join_instr
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


module join_instr(
   input [3:0]part1,
   input [25:0]part2,
   output [31:0]r
);
   assign r={part1,part2,2'b0};
endmodule
