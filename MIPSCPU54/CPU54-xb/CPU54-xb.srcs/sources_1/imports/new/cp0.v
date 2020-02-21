`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:03:21
// Design Name: 
// Module Name: cp0
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
module cp0(
    input clk,
    input rst,
    input mfc0,//mfc0 rt,rd (rt<-rd) [20:16]<-[15:11]
    input mtc0,
    input [31:0]pc,//�쳣����ʱ�쳣ָ���pc
    input [4:0]Rd,//����cp0�Ĵ���ѡ����� //mfc0 rt,rd (rt<-rd) [20:16]<-[15:11]
    input [31:0]wdata,//дcp0����,D_rt
    input exception,//�쳣�����ź�,��cpu�б�����status�Ĵ�������
    //exception=status[0]&status[3/2/1]&cause[4:0](=syscall or teq or break)
    input eret,//�쳣�����ź�
    input [4:0]cause,//�쳣����ԭ��
    output[31:0]rdata,//��cp0�Ĵ�������=>cp0_out
    output[31:0]status,//�쳣�������ı���status����,��������[3:1]0�����жϣ�1��ֹ�����ж�
    output reg [31:0]exc_addr //�����쳣����ʱ��pc��ַ=epc_out
    );
    reg [31:0] cp0_reg [0:31]; //32 cp0 registers
    assign  status = cp0_reg [12];
   
    always @(posedge clk or posedge rst)begin
    if(rst)begin
            cp0_reg [0] <=0 ;
            cp0_reg [1] <=0 ;
            cp0_reg [2] <=0 ;
            cp0_reg [3] <=0 ;
            cp0_reg [4] <=0 ;
            cp0_reg [5] <=0 ;
            cp0_reg [6] <=0 ;
            cp0_reg [7] <=0 ;
            cp0_reg [8] <=0 ;
            cp0_reg [9] <=0 ;
            cp0_reg [10] <=0 ;
            cp0_reg [11] <=0 ;
            cp0_reg [12] <=0 ;
            cp0_reg [13] <=0 ;
            cp0_reg [14] <=0 ;
            cp0_reg [15] <=0 ;
            cp0_reg [16] <=0 ;
            cp0_reg [17] <=0 ;
            cp0_reg [18] <=0 ;
            cp0_reg [19] <=0 ;
            cp0_reg [20] <=0 ;
            cp0_reg [21] <=0 ;
            cp0_reg [22] <=0 ;
            cp0_reg [23] <=0 ;
            cp0_reg [24] <=0 ;
            cp0_reg [25] <=0 ;
            cp0_reg [26] <=0 ;
            cp0_reg [27] <=0 ;
            cp0_reg [28] <=0 ;
            cp0_reg [29] <=0 ;
            cp0_reg [30] <=0 ;
            cp0_reg [31] <=0 ;        
          
    end
    else begin
    if(mtc0)begin
      cp0_reg [Rd] <= wdata;
    end
    else if(exception)begin
       cp0_reg [14] <=   pc; //epc;
       exc_addr<=32'h00400004;
       cp0_reg [13] <={24'b0,cause,2'b0}; //cause 
    end
    else if(eret)begin//�����쳣���ҷ���ʱ
       exc_addr  <=cp0_reg [14];
    end
  end
 end
  assign rdata = mfc0? cp0_reg [Rd]:32'hz; 
endmodule 
