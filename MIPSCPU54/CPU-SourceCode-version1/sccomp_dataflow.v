`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:26:11
// Design Name: 
// Module Name: sccomp_dataflow
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
module  sccomp_dataflow(
  input clk_in,
  input reset,
// output [31:0]inst,//instruction from imem
//  output [31:0]pc,//program counter
//  output [31:0]addr
 output [7:0]o_seg,
  output [7:0]o_sel
   );
  wire [31:0]pc;
  wire [31:0]inst;
   wire IM_R;//imem read ->1
   wire DM_W;//dmem write ena -wena
   wire DM_R;//dmem read ena  -rena
   wire DM_CS;//dram- dmem ena -ena
   wire [31:0]rdata;//data read from dram
   wire [31:0]addr;//dmem write/read address
   wire [31:0]wdata;//data write in dram

  seg7x16  seg7(.clk(clk_in),.reset(reset),.cs(1'b1),.i_data(pc),.o_seg(o_seg),.o_sel(o_sel));
   
 Divider div(
    .I_CLK(clk_in),
    .rst(reset),
    .O_CLK(clk_out)
    );
   

   wire Lbu,Lb,Lhu,Lh,Sw,Lw,Sb,Sh;
   cpu sccpu(
   .clk(clk_out),
   .rst(reset),
   .IM_R(IM_R),
   .DM_W(DM_W),
   .DM_R(DM_R),
   .DM_CS(DM_CS),
   .rdata(rdata),
   .addr(addr),
   .wdata(wdata),
   .instr(inst),
   .pc(pc),
   .Lbu(Lbu),.Lhu(Lhu),.Lh(Lh),.Lb(Lb),.Sb(Sb),.Sh(Sh),.Sw(Sw),.Lw(Lw)
    );
  
   dist_mem_gen_0 imem(//get instruction from instr_mem  ->  IMEM
   .a(pc[12:2]),
   .spo(inst)
   );
   
   dram dmem(
   .clk(clk_out),
   .ena(DM_CS),
   .rena(DM_R),
   .wena(DM_W),
   .addr(addr),
   .datain(wdata),
   .dataout(rdata),
   .Lbu(Lbu),.Lhu(Lhu),.Lh(Lh),.Lb(Lb),.Sb(Sb),.Sh(Sh),.Sw(Sw),.Lw(Lw)
   );
 
endmodule 
