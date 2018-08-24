`timescale 1ns/1ps 

module filter_tb;
// inputs
reg pdm_clk, reset;
reg signed [31:0] raw_pdm_data;
wire signed [31:0] filt_pdm_data_1;
wire signed [31:0] filt_pdm_data_2;
reg signed [31:0] filt_pdm_data_3;
reg signed [31:0] filt_pdm_data_4;
reg signed [31:0] filt_pdm_data_5;
reg signed [31:0] filt_pdm_data_6;
reg signed [31:0] filt_pdm_data_7;
wire signed [31:0] x_1;
wire signed [31:0] x_2;
wire signed [31:0] y_1;
wire signed [31:0] y_2;


// ---- new scale mode 2^23 ---
//621		1243	621	8388608	-16739949	8353827
//619		1238	619	8388608	-16672830	8286698
//616		1233	616	8388608	-16612088	8225947
//615		1230	615	8388608	-16561084	8174936
//613		1227	613	8388608	-16522585	8136430
//612		1225	612	8388608	-16498643	8112485
//71674	71674	0		8388608	-8245260		0

// instantiate DUTs
//621		1243	621	8388608	-16739949	8353827
filter #(
	.b0(621),
	.b1(1243),
	.b2(621), 
	.a0(8388608),
	.a1(-16739949),
	.a2(8353827)
	) layer_1 (
	.clk(pdm_clk), 
	.reset(reset), 
	.x(raw_pdm_data), 
	.y(filt_pdm_data_1)
	);

//619		1238	619	8388608	-16672830	8286698
filter #(
	.b0(621),
	.b1(1238),
	.b2(619), 
	.a0(8388608),
	.a1(-16672830),
	.a2(8286698)
	) layer_2 (
	.clk(pdm_clk), 
	.reset(reset), 
	.x(filt_pdm_data_1), 
	.y(filt_pdm_data_2),
	.x_1_out(x_1),
	.x_2_out(x_2), 
	.y_1_out(y_1),
	.y_2_out(y_2)
	);
//	
////616		1233	616	8388608	-16612088	8225947
//filter #(
//	.b0(616),
//	.b1(1233),
//	.b2(616), 
//	.a0(8388608),
//	.a1(-16612088),
//	.a2(8225947)
//	) layer_3 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_2), 
//	.y(filt_pdm_data_3)
////	.x_1_out(x_1),
////	.x_2_out(x_2), 
////	.y_1_out(y_1),
////	.y_2_out(y_2)
//	);
//	
////615		1230	615	8388608	-16561084	8174936
//filter #(
//	.b0(615),
//	.b1(1230),
//	.b2(615), 
//	.a0(8388608),
//	.a1(-16561084),
//	.a2(8174936)
//	) layer_4 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_3), 
//	.y(filt_pdm_data_4)
////	.x_1_out(x_1),
////	.x_2_out(x_2), 
////	.y_1_out(y_1),
////	.y_2_out(y_2)
//	);
//	
////613		1227	613	8388608	-16522585	8136430
//filter #(
//	.b0(613),
//	.b1(1227),
//	.b2(613), 
//	.a0(8388608),
//	.a1(-16522585),
//	.a2(8136430)
//	) layer_5 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_4), 
//	.y(filt_pdm_data_5)
////	.x_1_out(x_1),
////	.x_2_out(x_2), 
////	.y_1_out(y_1),
////	.y_2_out(y_2)
//	);
//	
////612		1225	612	8388608	-16498643	8112485
//filter #(
//	.b0(612),
//	.b1(1225),
//	.b2(612), 
//	.a0(8388608),
//	.a1(-16498643),
//	.a2(8112485)
//	) layer_6 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_5), 
//	.y(filt_pdm_data_6)
////	.x_1_out(x_1),
////	.x_2_out(x_2), 
////	.y_1_out(y_1),
////	.y_2_out(y_2)
//	);
//	
////71674	71674	0		8388608	-8245260		0
//filter #(
//	.b0(71674),
//	.b1(71674),
//	.b2(0), 
//	.a0(8388608),
//	.a1(-8245260),
//	.a2(0)
//	) layer_7 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_6), 
//	.y(filt_pdm_data_7)
////	.x_1_out(x_1),
////	.x_2_out(x_2), 
////	.y_1_out(y_1),
////	.y_2_out(y_2)
//	);
	
	initial pdm_clk = 0;
	always #330 pdm_clk = ! pdm_clk;
	
	initial begin
		pdm_clk = 0;
		reset = 1;
		# 10 
		reset = 0;
		raw_pdm_data = 0;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data = - 8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		#660 raw_pdm_data =  8388607;
		$stop;
	end	
endmodule
