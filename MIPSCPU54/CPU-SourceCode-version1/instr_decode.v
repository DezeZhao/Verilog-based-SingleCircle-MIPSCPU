`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:04:35
// Design Name: 
// Module Name: instr_decode
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

`define ADD  12'b000000100000
`define ADDU 12'b000000100001
`define SUB  12'b000000100010
`define SUBU 12'b000000100011
`define AND  12'b000000100100
`define OR   12'b000000100101
`define XOR  12'b000000100110
`define NOR  12'b000000100111
`define SLT  12'b000000101010
`define SLTU 12'b000000101011
`define SLL  12'b000000000000
`define SRL  12'b000000000010
`define SRA  12'b000000000011
`define SLLV 12'b000000000100
`define SRLV 12'b000000000110
`define SRAV 12'b000000000111
`define JR   12'b000000001000

`define ADDI  12'b001000??????
`define ADDIU 12'b001001??????
`define ANDI 12'b001100??????
`define ORI  12'b001101??????
`define XORI 12'b001110??????
`define LW   12'b100011??????
`define SW   12'b101011??????
`define BEQ  12'b000100??????
`define BNE  12'b000101??????
`define SLTI 12'b001010??????
`define SLTIU 12'b001011??????
`define LUI  12'b001111??????

`define J    12'b000010??????
`define JAL  12'b000011??????

`define DIV   12'b000000011010
`define DIVU  12'b000000011011
`define MUL   12'b011100000010
`define MULTU 12'b000000011001

`define BGEZ  12'b000001??????

`define JALR  12'b000000001001
`define LBU   12'b100100??????
`define LHU   12'b100101??????
`define LB    12'b100000??????
`define LH    12'b100001??????
`define SB    12'b101000??????
`define SH    12'b101001??????
`define BREAK 12'b000000001101
`define SYSCALL 12'b000000001100
`define ERET  12'b010000011000
`define MFHI  12'b000000010000
`define MFLO  12'b000000010010
`define MTHI  12'b000000010001
`define MTLO  12'b000000010011
`define MFC0  12'b010000000000
`define MTC0  12'b010000100000
`define CLZ   12'b011100100000 
`define TEQ   12'b000000110100
module instr_decode(
    input [31:0]instr,
    output reg [63:0] dec
  );
always @(*)begin
    if(instr[31:26]!=6'b010000)
     casez ({instr[31:26],instr[5:0]})
        `ADD:dec<=64'h00000000_00000001;
        `ADDU:dec<=64'h00000000_00000002;
        `SUB:dec<=64'h00000000_00000004;
        `SUBU:dec<=64'h00000000_00000008;

        `AND:dec<=64'h00000000_00000010;
        `OR:dec<=64'h00000000_00000020;
        `XOR:dec<=64'h00000000_00000040;
        `NOR:dec<=64'h00000000_00000080;

        `SLT:dec<=64'h00000000_00000100;
        `SLTU:dec<=64'h00000000_00000200;
        `SLL:dec<=64'h00000000_00000400;
        `SRL:dec<=64'h00000000_00000800;

        `SRA:dec<=64'h00000000_00001000;
        `SLLV:dec<=64'h00000000_00002000;
        `SRLV:dec<=64'h00000000_00004000;
        `SRAV:dec<=64'h00000000_00008000;

        `JR:dec<=64'h00000000_00010000;

        `ADDI:dec<=64'h00000000_00020000;
        `ADDIU:dec<=64'h00000000_00040000;
        `ANDI:dec<=64'h00000000_00080000;

        `ORI:dec<=64'h00000000_00100000;
        `XORI:dec<=64'h00000000_00200000;
        `LW:dec<=64'h00000000_00400000;
        `SW:dec<=64'h00000000_00800000;

        `BEQ:dec<=64'h00000000_01000000;
        `BNE:dec<=64'h00000000_02000000;
        `SLTI:dec<=64'h00000000_04000000;
        `SLTIU:dec<=64'h00000000_08000000;
        `LUI:dec<=64'h00000000_10000000;


        `J:dec<=64'h00000000_20000000;
        `JAL:dec<=64'h00000000_40000000;

        `DIV:dec<=64'h00000000_80000000;
        `DIVU:dec<=64'h00000001_00000000;
        `MUL:dec<=64'h00000002_00000000;
		`MULTU:dec<=64'h00000004_00000000;
		`BGEZ:dec<=64'h00000008_00000000;
		
		`JALR:dec<=64'h00000010_00000000;
		`LBU:dec<=64'h00000020_00000000;
		`LHU:dec<=64'h00000040_00000000;
		`LB:dec<=64'h00000080_00000000;
		`LH:dec<=64'h00000100_00000000;
		`SB:dec<=64'h00000200_00000000;
		`SH:dec<=64'h00000400_00000000;
		`BREAK:dec<=64'h00000800_00000000;
		`SYSCALL:dec<=64'h00001000_00000000;
		`MFHI:dec<=64'h00004000_00000000;
		`MFLO:dec<=64'h00008000_00000000;
		`MTHI:dec<=64'h00010000_00000000;
		`MTLO:dec<=64'h00020000_00000000;
		`CLZ:dec<=64'h00100000_00000000;
		`TEQ:dec<=64'h00200000_00000000;
        default:dec<=64'bx;
       endcase
     else begin
        if({instr[31:26],instr[5:0]}==`ERET)
           dec<=64'h00002000_00000000;
        else
        casez({instr[31:26],instr[23],instr[4:0]})
        `MFC0:dec<=64'h00040000_00000000;
		`MTC0:dec<=64'h00080000_00000000;
         default:dec<=64'bx;
        endcase
     end
   end           
   
endmodule
