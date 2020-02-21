`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/05/17 18:17:46
// Design Name: 
// Module Name: cpu
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
module cpu(
   input clk,
   input rst,
   input [31:0]instr,
   input [31:0]rdata,//data read from dram
   output IM_R,
   output DM_W,
   output DM_R,
   output DM_CS,
   output [31:0]pc,
   output [31:0]addr,//dram write address
   output [31:0]wdata,   //data write in dram
   output Sw,
   output Lw,
   output Lbu,
   output Lhu,
   output Lh,
   output Lb,
   output Sh,
   output Sb
);

   wire [31:0]D_M1;
   wire [31:0]D_M2;
   wire [31:0]D_M3;
   wire [31:0]D_M4;
   wire [31:0]D_M5;//rd
   wire [4:0]D_M6;//rdc <=rdc(instr[15:11])or rtc(instr[20:16])
   wire [31:0]D_M7;
   wire [31:0]D_M8;
   wire [4:0]D_M9;//instr[10:6] or rs[4:0]
   
   wire [31:0] D_ext16;//output of ext16
   wire [31:0] D_ext18;//output of ext18
   wire [31:0] D_ext5;//output of ext5
   wire [31:0] D_pc;//output of pc
   wire [31:0] D_npc;//output of npc
   wire [31:0] D_add8;//output of add8
   wire [31:0] D_add;//output of add
   wire [31:0] D_rs;
   wire [31:0] D_rt;
   wire [31:0] D_alu;//output of alu
   wire [31:0] D_join;
   wire [63:0] D_mul;
   wire [63:0] D_multu;
   wire add_ov;
   //cp0
   wire [4:0]Rd;
   wire [31:0]cp0_rdata;
   wire [4:0]cause;
   wire [31:0]exc_addr;   
   
   //control instruction
   wire M1;
   wire M2;
   wire M3;
   wire M4;
   wire M5;
   wire M6;
   wire M7;
   wire M8;
   wire M9;
   wire PC_CLK;
   wire PC_ENA;
   wire RF_CLK;
   wire RF_W;
   wire C_ext16;
   wire [3:0]ALUC;
   wire Z;//zero
   wire C;//carry
   wire N;//negative
   wire O;//overflow
   wire [31:0]status;
   wire exc;
   wire eret;
   wire mtc0;
   wire mfc0;
   wire div_busy;
   wire divu_busy;
   wire [31:0] D_divr;
   wire [31:0] D_divr1;
   wire [31:0] D_divq;
   wire [31:0] D_divq1;
   wire [31:0] D_divur;
   wire [31:0] D_divur1;
   wire [31:0] D_divuq;
   wire [31:0] D_divuq1;
   
   assign PC_ENA=1;
   assign pc=D_pc;
   assign addr=D_alu;  //dram read/write address
   assign wdata=D_rt;
   //cp0
   assign Rd=instr[15:11];//¿ØÖÆcp0¼Ä´æÆ÷Ñ¡ÔñÊä³ö mfc0 rt,rd (rt<-rd) [20:16]<-[15:11];
   wire [63:0] dec;//decoded instruction 
   
   reg [31:0] hi;
   reg [31:0] lo;
   wire [31:0]D_hi;
   wire[31:0]D_lo;
   wire [31:0] D_clz;
   wire wena_hi=(dec[31]|dec[32]|dec[48]|dec[34]|dec[33])?1'b1:1'b0;
   wire wena_lo=(dec[31]|dec[32]|dec[49]|dec[34]|dec[33])?1'b1:1'b0;
   wire[5:0]opt={dec[48],dec[49],dec[33],dec[34],dec[31],dec[32]};
   assign divu_q=divu_busy?32'bz:D_divuq1;
   assign divu_r=divu_busy?32'bz:D_divur1; 
   assign div_q=div_busy?32'bz:D_divq1;
   assign div_r=div_busy?32'bz:D_divr1;  
     
   always@(*)begin
   case(opt)
   6'b100000:hi<=D_rs;
   6'b010000:lo<=D_rs;
   6'b001000: begin  hi<=D_mul[63:32];
                     lo<=D_mul[31:0];
              end
   6'b000100: begin  hi<=D_multu[63:32]; 
                     lo<=D_multu[31:0]; 
              end
   6'b000010: begin  hi<=D_divr;
                     lo<=D_divq; 
              end
   6'b000001: begin  hi<=D_divur;
                     lo<=D_divuq; 
              end
    endcase
   end
   
/****************************instruction decode***********************************/
    instr_decode cpu_dec(
        .instr(instr),
        .dec(dec)
    );
/****************************CP0 module*******************************************/
    cp0 cpu_cp0(
        .clk(clk),
        .rst(rst),
        .mfc0(mfc0),
        .mtc0(mtc0),
        .pc(pc),
        .Rd(Rd),
        .wdata(D_rt),
        .exception(exc),
        .eret(eret),
        .cause(cause),
        .rdata(cp0_rdata),
        .status(status),
        .exc_addr(exc_addr)
    );
/****************************control module***************************************/  
    ctrl cpu_ctrl(// control module
        .clk(clk),
        .rst(rst),
        .z(Z),
        .n(N),
        .dec(dec),
        .M1(M1),
        .M2(M2),
        .M3(M3),
        .M4(M4),
        .M5(M5),
        .M6(M6),
        .M7(M7),
        .M8(M8),
        .M9(M9),
        .PC_CLK(PC_CLK),
        .IM_R(IM_R),
        .ALUC(ALUC),
        .RF_CLK(RF_CLK),
        .RF_W(RF_W),
        .DM_W(DM_W),
        .DM_R(DM_R),
        .DM_CS(DM_CS),
        .C_ext16(C_ext16),
        .status(status),
        .exc(exc),
        .eret(eret),
        .mtc0(mtc0),
        .mfc0(mfc0),
        .cause(cause),
		.Lbu(Lbu),
		.Lhu(Lhu),
		.Lh(Lh),
		.Lb(Lb),
		.Sb(Sb),
		.Sh(Sh),
		.Sw(Sw),
		.Lw(Lw)
    );
/****************************pcreg module***************************************/  
    wire busy=divu_busy|div_busy?1'b1:1'b0;
    pcreg  cpu_pc(
        .clk(clk),
        .rst(rst),
        .ena(PC_ENA),
        .data_in(D_M1),
        .wena(!busy),
        .data_out(D_pc)
    ); 
/****************************regfile module***************************************/  
    regfile cpu_ref(
        .clk(clk),
        .rst(rst),
        .ena(1'b1),
        .we(RF_W),
        .raddr1(instr[25:21]),
        .raddr2(instr[20:16]),
        .waddr(D_M6),
        .wdata(D_M7),
        .rdata1(D_rs),
        .rdata2(D_rt)	
    );
/****************************ALU module***************************************/  
    alu  cpu_alu(
        .a(D_M3),
        .b(D_M4),
        .aluc(ALUC),
        .r(D_alu),
        .zero(Z),
        .carry(C),
        .negative(N),
        .overflow(O)
    );
/****************************HIreg module***************************************/  
   HILOreg cpu_hi(
          .clk(clk),
          .rst(rst),
          .data_in(hi),
          .data_out(D_hi),
          .wena(wena_hi)
   );
/****************************LOreg module***************************************/  
   HILOreg cpu_lo(
          .clk(clk),
          .rst(rst),
          .data_in(lo),
          .data_out(D_lo),
          .wena(wena_lo)
   );
/****************************5bits ext to 32bits module**************************/  
    ext5    cpu_ext5(
        .a(D_M9),
        .b(D_ext5)
    );
/****************************16bits ext to 32bits module**************************/  
    ext16   cpu_ext16(
        .a(instr[15:0]),
        .b(D_ext16),
        .s_ext(C_ext16)
    );
/****************************18bits ext to 32bits module**************************/  
    ext18   cpu_ext18(
        .a(instr[15:0]),
        .b(D_ext18)
    );
/****************************multiple selectors***********************************/  
    mux32_4   cpu_mux1(
        .a(D_M8),
        .b(D_M5),
        .c(D_rs),
        .d(exc_addr),
        .opt({dec[45],exc,dec[36],M1}),
        .e(D_M1)
    );
    mux32   cpu_mux2(
        .a(D_alu),
        .b(rdata),
        .opt(M2),
        .c(D_M2)
    );
    mux32_2   cpu_mux3(
        .a(D_ext5),
        .b(D_rs),
        .opt({dec[31],dec[32],dec[36],dec[52],dec[33],dec[34],M3}),
        .c(D_M3)
    );
    mux32_3   cpu_mux4(
        .a(D_rt),
        .b(D_ext16),
        .opt({dec[31],dec[32],dec[33],dec[34],M4}),
        .c(D_M4)
    );
    mux32   cpu_mux5(
        .a(D_npc),
        .b(D_add),
        .opt(M5),
        .c(D_M5)
    );
    mux5_1   cpu_mux6(//dec[30]=M7 : jal 
        .a(instr[15:11]),
        .b(instr[20:16]),
        .opt({dec[33],dec[36],dec[52],dec[46],dec[47],mfc0,M7,M6}),
        .c(D_M6)
    );//rd   ,rt
    mux32_1   cpu_mux7(
        .a(D_M2),
        .b(D_add8),
        .c(cp0_rdata),
        .d(D_hi),
        .e(D_lo),
        .f(D_clz),
        .g(D_mul[31:0]),
        .opt({dec[33],dec[36],dec[52],dec[46],dec[47],mfc0,M7}),
        .h(D_M7)
    );
    mux32   cpu_mux8(
        .a(D_join),
        .b(D_rs),
        .opt(M8),
        .c(D_M8)
    );
    mux5_2   cpu_mux9(
        .a(instr[10:6]),
        .b(D_rs[4:0]),
        .opt(M9),
        .c(D_M9)
    );
/****************************add module****************************************/  
    add      cpu_add(
        .a(D_ext18),
        .b(D_npc),
        .c(D_add),
        .ov(add_ov)
    );
    add8    cpu_add8(
        .a(D_pc),
        .c(D_add8)
    );
/*******************join part1 and part2 to be an instruction******************/  
    join_instr  cpu_join(
        .part1(D_pc[31:28]),
        .part2(instr[25:0]),
        .r(D_join)
    );
/****************************npc module****************************************/  
    npc   cpu_npc(
        .a(D_pc),
        .rst(rst),
        .c(D_npc)
    ); 
 /***************signed and unsigned  mul/multu/div/divu module*****************/  
    MUL  cpu_mul(
        .clk(clk),
        .reset(rst),
        .a(D_M3),
        .b(D_M4),
        .z(D_mul)
    );
    
    MULTU  cpu_multu(
        .clk(clk),
        .reset(rst),
        .a(D_M3),
        .b(D_M4),
        .z(D_multu) 
    );
    
    DIVU   cpu_divu(
        .dividend(D_M3),
        .divisor(D_M4),
        .start(dec[32]),
        .clk(clk),
        .reset(rst),
        .q(D_divuq1),
        .r(D_divur1),
        .busy(divu_busy)
    );
    
    DIV   cpu_div(
        .dividend(D_M3),
        .divisor(D_M4),
        .start(dec[31]),
        .clk(clk),
        .reset(rst),
        .q(D_divq1),
        .r(D_divr1),
        .busy(div_busy)
    );
/**************************count 0s brefore the first 1****************************/        
    clz  cpu_clz(
        .a(D_M3),
        .opt(dec[52]),
        .b(D_clz)
    );                                             
endmodule