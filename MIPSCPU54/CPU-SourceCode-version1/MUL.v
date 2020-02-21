`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/07/06 15:24:29
// Design Name: 
// Module Name: MUL
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

module MUL(
  input clk,
  input reset,
  input   [31:0] a,//rs
  input   [31:0] b,//rt
  output  [63:0] z);
  assign z=a*b;
endmodule

/*module  MUL(
    input clk,
	input reset,
	input   [31:0] a,//rs
	input   [31:0] b,//rt
	output  [63:0] z
); 
    reg  [31:0]a1;
	reg  [31:0]b1;
    reg   [63:0] temp;
    reg   [63:0] s0;
    reg   [63:0] s1;	
	reg   [63:0] s2;	
	reg   [63:0] s3;	
	reg   [63:0] s4;	
	reg   [63:0] s5;	
	reg   [63:0] s6;	
	reg   [63:0] s7;	
	reg   [63:0] s8;	
	reg   [63:0] s9;	
	reg   [63:0] s10;	
	reg   [63:0] s11;
    reg   [63:0] s12;	
	reg   [63:0] s13;	
	reg   [63:0] s14;	
	reg   [63:0] s15;
	reg   [63:0] s16;
    reg   [63:0] s17;	
	reg   [63:0] s18;	
	reg   [63:0] s19;	
	reg   [63:0] s20;	
	reg   [63:0] s21;	
	reg   [63:0] s22;	
	reg   [63:0] s23;	
	reg   [63:0] s24;	
	reg   [63:0] s25;	
	reg   [63:0] s26;	
	reg   [63:0] s27;
    reg   [63:0] s28;	
	reg   [63:0] s29;	
	reg   [63:0] s30;	
	reg   [63:0] s31;
	
	reg   [63:0] add01;	
	reg   [63:0] add23;	
	reg   [63:0] add45;	
	reg   [63:0] add67;	
	reg   [63:0] add89;	
	reg   [63:0] add1011;
	reg   [63:0] add1213;
	reg   [63:0] add1415;
    reg   [63:0] add1617;	
	reg   [63:0] add1819;	
	reg   [63:0] add2021;	
	reg   [63:0] add2223;	
	reg   [63:0] add2425;	
	reg   [63:0] add2627;
	reg   [63:0] add2829;
	reg   [63:0] add3031;
	
	reg   [63:0] add01_23;
	reg   [63:0] add45_67;
    reg   [63:0] add89_1011;
	reg   [63:0] add1213_1415;
	reg   [63:0] add1617_1819;
	reg   [63:0] add2021_2223;
    reg   [63:0] add2425_2627;
	reg   [63:0] add2829_3031;
	
    reg   [63:0] add0t3_4t7;
	reg   [63:0] add8t11_12t15;
	reg   [63:0] add16t19_20t23;
	reg   [63:0] add24t27_28t31;
	
	reg   [63:0] add0t7_8t15;
	reg   [63:0] add16t23_24t31;
	reg   sign;
	integer i;
	always@(*)
	begin
	if(reset)begin
	 temp<=0;
	 s0<=0;
	 s1<=0;
	 s2<=0;
	 s3<=0;
	 s4<=0;
	 s5<=0;
	 s6<=0;
	 s7<=0;
	 s8<=0;
	 s9<=0;
	 s10<=0;
	 s11<=0;
	 s12<=0;
	 s13<=0;
	 s14<=0;
	 s15<=0;
	 s16<=0;
	 s17<=0;
	 s18<=0;
	 s19<=0;
	 s20<=0;
	 s21<=0;
	 s22<=0;
	 s23<=0;
	 s24<=0;
	 s25<=0;
	 s26<=0;
	 s27<=0;
	 s28<=0;
	 s29<=0;
	 s30<=0;
	 s31<=0;
	 
	 add01<=0;	
	 add23<=0;	
	 add45<=0;	
	 add67<=0;	
	 add89<=0;	
	 add1011<=0;
	 add1213<=0;
	 add1415<=0;
     add1617<=0;	
	 add1819<=0;	
	 add2021<=0;	
	 add2223<=0;	
	 add2425<=0;	
	 add2627<=0;
	 add2829<=0;
	 add3031<=0;
	 add01_23<=0;
     add45_67<=0;
     add89_1011<=0;
	 add1213_1415<=0;
	 add1617_1819<=0;
	 add2021_2223<=0;
     add2425_2627<=0;
	 add2829_3031<=0;
	 
     add0t3_4t7<=0;
	 add8t11_12t15<=0;
	 add16t19_20t23<=0;
	 add24t27_28t31<=0;
	 
	 add0t7_8t15<=0;
	 add16t23_24t31<=0;
	 end
	 else begin
	  sign<=a[31]^b[31];
      if(a[31]==1'b0)
        a1<=a;
      else 
         a1=~a+1'b1;
      if(b[31]==1'b0)
        b1<=b;
       else 
        b1<=~b+1'b1;
     for(i=1;i<=32;i=i+1)begin	   
	  s0=b1[0]?{32'b0,a1}:64'b0;
	 s1=b1[1]?{31'b0,a1,1'b0}:64'b0;
	 s2=b1[2]?{30'b0,a1,2'b0}:64'b0;
	 s3=b1[3]?{29'b0,a1,3'b0}:64'b0;
	 s4=b1[4]?{28'b0,a1,4'b0}:64'b0;
	 s5=b1[5]?{27'b0,a1,5'b0}:64'b0;
	 s6=b1[6]?{26'b0,a1,6'b0}:64'b0;
	 s7=b1[7]?{25'b0,a1,7'b0}:64'b0;
	 s8=b1[8]?{24'b0,a1,8'b0}:64'b0;
     s9=b1[9]?{23'b0,a1,9'b0}:64'b0; 
     s10=b1[10]?{22'b0,a1,10'b0}:64'b0; 
     s11=b1[11]?{21'b0,a1,11'b0}:64'b0;
	 s12<=b1[12]?{20'b0,a1,12'b0}:64'b0;
	 s13<=b1[13]?{19'b0,a1,13'b0}:64'b0;
	 s14<=b1[14]?{18'b0,a1,14'b0}:64'b0;
	 s15<=b1[15]?{17'b0,a1,15'b0}:64'b0;
	 s16<=b1[16]?{16'b0,a1,16'b0}:64'b0;
	 s17<=b1[17]?{15'b0,a1,17'b0}:64'b0;
	 s18<=b1[18]?{14'b0,a1,18'b0}:64'b0;
	 s19<=b1[19]?{13'b0,a1,19'b0}:64'b0;
	 s20<=b1[20]?{12'b0,a1,20'b0}:64'b0;
	 s21<=b1[21]?{11'b0,a1,21'b0}:64'b0;
	 s22<=b1[22]?{10'b0,a1,22'b0}:64'b0; 
     s23<=b1[23]?{9'b0,a1,23'b0}:64'b0;
	 s24<=b1[24]?{8'b0,a1,24'b0}:64'b0;
	 s25<=b1[25]?{7'b0,a1,25'b0}:64'b0;
	 s26<=b1[26]?{6'b0,a1,26'b0}:64'b0;
	 s27<=b1[27]?{5'b0,a1,27'b0}:64'b0;
	 s28<=b1[28]?{4'b0,a1,28'b0}:64'b0;
	 s29<=b1[29]?{3'b0,a1,29'b0}:64'b0;
	 s30<=b1[30]?{2'b0,a1,30'b0}:64'b0;
	 s31<=b1[31]?{1'b0,a1,31'b0}:64'b0;
	 add01<=s0+s1;	
	 add23<=s3+s2;	
	 add45<=s4+s5;
	 add67<=s6+s7;
	 add89<=s8+s9;
	 add1011<=s10+s11;
	 add1213<=s12+s13;
	 add1415<=s14+s15;
     add1617<=s16+s17;	
	 add1819<=s18+s19;	
	 add2021<=s20+s21;	
	 add2223<=s22+s23;	
	 add2425<=s24+s25;	
	 add2627<=s26+s27;
	 add2829<=s28+s29;
	 add3031<=s30+s31;
	 
	 add01_23<=add01+add23;
     add45_67<=add45+add67;
     add89_1011<=add89+add1011;
	 add1213_1415<=add1213+add1415;
	 add1617_1819<=add1617+add1819;
	 add2021_2223<=add2021+add2223;
     add2425_2627<=add2425+add2627;
	 add2829_3031<=add2829+add3031;
	 
     add0t3_4t7<=add01_23+add45_67;
	 add8t11_12t15<=add89_1011+add1213_1415;
	 add16t19_20t23<=add1617_1819+add2021_2223;
	 add24t27_28t31<=add2425_2627+add2829_3031;
	 
	 add0t7_8t15<=add0t3_4t7+add8t11_12t15;
	 add16t23_24t31<=add16t19_20t23+add24t27_28t31;
	 if(sign==1'b1)
	 temp<=~(add0t7_8t15+add16t23_24t31)+1'b1;
	 else 
	 temp<=add0t7_8t15+add16t23_24t31;
	 end
	 end 
	end
	assign z=temp;
endmodule*/
