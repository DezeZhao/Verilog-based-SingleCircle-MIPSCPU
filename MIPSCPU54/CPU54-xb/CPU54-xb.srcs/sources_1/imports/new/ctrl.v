`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:02:34
// Design Name: 
// Module Name: ctrl
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
module ctrl(
    input clk,
    input rst,
    input [63:0]dec,
    input z,//zero
    input n,//neg
    output M1,
    output M2,
    output M3,
    output M4,
    output M5,
    output M6,
    output M7,
    output M8,
    output M9,
    output PC_CLK,
    output IM_R,//imem(ram1) read
    output [3:0]ALUC,
    output RF_CLK,
    output RF_W,//regfile write
    output DM_W,//dmem(ram2) 
    output DM_R,//dmem read
    output DM_CS,//dmem ena 
    output C_ext16,
    //cp0
    input [31:0]status,
    output mtc0,
    output mfc0,
    output eret,
    output exc,
    output [4:0]cause,
	output Lbu,
	output Lhu,
	output Lb,
	output Lh,
	output Sb,
	output Sh,
	output Sw,
	output Lw
     );
    assign PC_CLK=clk;
    assign IM_R=1;
    assign M1=~(dec[29]|dec[30]|dec[16]|dec[36]|dec[45]|exc|dec[43]|dec[44]|dec[53]);
    assign M2=dec[22]|dec[37]|dec[38]|dec[39]|dec[40];
    assign M3=~(dec[10]|dec[11]|dec[12]|dec[13]|dec[14]|dec[15]|dec[31]|dec[32]|dec[33]|dec[34]|dec[52]|dec[36]);
    assign M4=dec[17]|dec[18]|dec[19]|dec[20]|dec[21]|dec[22]|dec[23]|dec[26]|dec[27]|dec[28]|dec[37]|dec[38]|dec[39]|dec[40]|dec[41]|dec[42];
    assign M5=(dec[24]&z)|(dec[25]&~z)|(dec[35]&~n);
    assign M6=dec[17]|dec[18]|dec[19]|dec[20]|dec[21]|dec[22]|dec[23]|dec[26]|dec[27]|dec[28]|dec[37]|dec[38]|dec[39]|dec[40]|dec[41]|dec[42];
    assign M7=dec[30];
    assign M8=dec[16];//~(dec[29]|dec[30])
    assign M9=dec[13]|dec[14]|dec[15];
    assign ALUC[3]=dec[8]|dec[9]|dec[10]|dec[11]|dec[12]|dec[13]|dec[14]|dec[15]|dec[26]|dec[27]|dec[28]|dec[35];
    assign ALUC[2]=dec[4]|dec[5]|dec[6]|dec[7]|dec[10]|dec[11]|dec[12]|dec[13]|dec[14]|dec[15]|dec[19]|dec[20]|dec[21];
    assign ALUC[1]=dec[0]|dec[2]|dec[6]|dec[7]|dec[8]|dec[9]|dec[10]|dec[13]|dec[17]|dec[21]|dec[22]|dec[23]|dec[24]|dec[25]|dec[26]|dec[27]|dec[53]|dec[37]|dec[38]|dec[39]|dec[40]|dec[41]|dec[42];
    assign ALUC[0]=dec[2]|dec[3]|dec[5]|dec[7]|dec[8]|dec[11]|dec[14]|dec[20]|dec[24]|dec[25]|dec[26]|dec[53]|dec[35];
    assign RF_W=~(dec[16]|dec[23]|dec[24]|dec[25]|dec[29]|dec[51]|dec[41]|dec[42]);
    assign RF_CLK=clk;
    assign DM_R=dec[22]|dec[37]|dec[38]|dec[39]|dec[40];
    assign DM_W=dec[23]|dec[41]|dec[42];
    assign DM_CS=dec[22]|dec[23]|dec[37]|dec[38]|dec[39]|dec[40]|dec[41]|dec[42];
    assign C_ext16=~(dec[19]|dec[20]|dec[21]);
	assign Lbu=dec[37];
	assign Lhu=dec[38];
	assign Lb=dec[39];
	assign Lh=dec[40];
	assign Sb=dec[41];
	assign Sh=dec[42];
    assign Sw=dec[23];
	assign Lw=dec[22];
    //cp0 ctrl
    parameter Break=5'b01001;
    parameter Syscall=5'b01000;
    parameter Teq=5'b01101;
    
     assign cause = dec[43]?Break:(dec[44]?Syscall:(dec[53]?Teq:5'b00000));
     assign eret=dec[45];
     assign mfc0=dec[50];
     assign mtc0=dec[51];
     assign exc=(status[0]&&((status[3]&&cause==Teq&&dec[53]&z)||(status[1]&&cause==Syscall)||(status[2]&&cause==Break)))?1'b1:1'b0; //1表示中断允许，0表示中断禁止
    
endmodule  
