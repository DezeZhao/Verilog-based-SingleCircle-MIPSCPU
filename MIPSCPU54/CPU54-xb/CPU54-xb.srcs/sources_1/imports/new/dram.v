`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:06:48
// Design Name: 
// Module Name: dram
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
module dram(
      input clk,
      input ena,// DM_CS enable
      input wena,//DM_W write enable
      inout rena,//DM_R read enable
	  input Lw,Sw,Sb,Lb,Lbu,Sh,Lh,Lhu,
      input [31:0] addr,
      input [31:0] datain,
      output reg[31:0] dataout
    );

  reg [31:0]mem[2047:0];
  wire [10:0]Addr=addr[10:0];
   always @(*)begin
      if(ena&&rena)begin
         if(Lb) dataout<={{24{mem[Addr][7]}},mem[Addr][7:0]};
         if(Lbu)dataout<={24'b0,mem[Addr][7:0]};
         if(Lw) dataout<=mem[Addr];
         if(Lh) dataout<={{16{mem[Addr][15]}},mem[Addr][15:0]};
         if(Lhu) dataout<={16'b0,mem[Addr][15:0]};
      end
      else
         dataout<=32'bx;
     end
    always@(posedge clk)begin
    if(ena)begin
       if(wena&&ena)begin
       if(Sw)mem[Addr]<=datain;
       if(Sb)mem[Addr][7:0]<=datain[7:0];
       if(Sh)mem[Addr][15:0]<=datain[15:0];
     end
    end
   end
endmodule
