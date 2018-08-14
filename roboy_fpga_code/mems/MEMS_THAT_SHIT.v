module MEMS_THAT_SHIT(
	input clock,
	input reset,
	input pdm,
	input pdm_clk,
	input dec_clk,
	output pdm_clk_out,
	output dec_clk_out,
	output reg [31:0] address,
	output reg write,
	output reg [255:0] write_data,
	input waitrequest
);


assign pdm_clk_out = pdm_clk;
assign dec_clk_out = dec_clk;
reg [31:0] addr_cntr;
parameter IDLE  = 1'b0, WAIT_FOR_TRANSMIT = 1'b1;
parameter MEM_SIZE = 23'd4096; // 4k
reg onchip_state;
reg dec_clk_prev;

reg signed [31:0] raw_pdm_data;
reg signed [31:0] filt_pdm_data_1;
reg signed [31:0] filt_pdm_data_2;
reg signed [31:0] filt_pdm_data_3;
reg signed [31:0] filt_pdm_data_4;
reg signed [31:0] filt_pdm_data_5;
reg signed [31:0] filt_pdm_data_6;
reg signed [31:0] filt_pdm_data_7;

reg signed [31:0] x_1;
reg signed [31:0] x_2;
reg signed [31:0] y_1;
reg signed [31:0] y_2;

always @(posedge pdm_clk) begin
	if (pdm) 
		raw_pdm_data <= 2147483647;
	else
		raw_pdm_data <= 0;
end

iir git_filter(
	.clk(pdm_clk), 
	.rst(reset), 
	.x(raw_pdm_data), 
	.y(filt_pdm_data_1)
	);

always @(posedge clock, posedge reset) begin: AVALON_WRITE_ONCHIP_INTERFACE
	if (reset == 1) begin
		addr_cntr <= 0;
	end else begin
		write <= 0;
		dec_clk_prev <= dec_clk;
		filt_pdm_data_2<= filt_pdm_data_1;
		filt_pdm_data_3<= filt_pdm_data_2;
		filt_pdm_data_4<= filt_pdm_data_3;
		filt_pdm_data_5<= filt_pdm_data_4;
		filt_pdm_data_6<= filt_pdm_data_5;
		filt_pdm_data_7<= filt_pdm_data_6;
		case(onchip_state)
			IDLE: begin
				if (dec_clk_prev == 0 &&  dec_clk == 1) begin 
					if (addr_cntr < 300) begin
						addr_cntr <= 0; //addr_cntr + 1;
						address <= addr_cntr;
//						write_data[23:0] <= filt_pdm_data_1;
//						write_data[47:24] <=  filt_pdm_data_2;
//						write_data[71:48] <=  filt_pdm_data_3;
//						write_data[95:72] <=  filt_pdm_data_4;
//						write_data[119:96] <=  filt_pdm_data_5;
//						write_data[143:120] <=  filt_pdm_data_6;
//						write_data[167:144] <=  filt_pdm_data_7;
//						write_data[255:168] <= 0;
						
//						write_data[31:0] <= filt_pdm_data_1;
//						write_data[63:32] <=  x_1;
//						write_data[95:64] <=  x_2;
//						write_data[127:96] <=  filt_pdm_data_2;
//						write_data[159:128] <=  y_1;
//						write_data[191:160] <=  y_2;
//						write_data[255:192] <= 0;

						write_data[31:0] <= filt_pdm_data_1;
						write_data[63:32] <=  filt_pdm_data_2;
						write_data[95:64] <=  filt_pdm_data_3;
						write_data[127:96] <=  filt_pdm_data_4;
						write_data[159:128] <=  filt_pdm_data_5;
						write_data[191:160] <=  filt_pdm_data_6;
						write_data[223:192] <= filt_pdm_data_7;
						write_data[255:224] <= 0;
						onchip_state <= WAIT_FOR_TRANSMIT;
						write <= 1;
					end else begin
						addr_cntr <= 0;
					end
				end
			end
			WAIT_FOR_TRANSMIT: begin		
				if(waitrequest==0) begin
					onchip_state <= IDLE;
				end	
			end
			default: onchip_state <= IDLE;
		endcase
	end
end
	
// ----- 32 bit range ----
//	79749	159499	79749	1076132222	-2147483648	1071670424
//79749	159499	79749	1080464368,00000	-2147483648,00000	1067338278,00000
//79749	159499	79749	1084415094,00000	-2147483648,00000	1063387552,00000
//79749	159499	79749	1087754797,00000	-2147483648,00000	1060047849,00000
//79749	159499	79749	1090289387,00000	-2147483648,00000	1057513259,00000
//79749	159499	79749	1091871563,00000	-2147483648,00000	1055931083,00000
//18348550	18348550	0	2147483648,00000	-2110786547,00000	0
	
// ---- 24 bit range ----	
//311	623	311	4203641	-8388608	4186212
//311	623	311	4220563	-8388608	4169290
//311	623	311	4235996	-8388608	4153857
//311	623	311	4249042	-8388608	4140811
//311	623	311	4258942	-8388608	4130911
//311	623	311	4265123	-8388608	4124730
//71674	71674	0	8388608	-8245260	0

// ---- new scale mode 2^23 ---
//621		1243	621	8388608	-16739949	8353827
//619		1238	619	8388608	-16672830	8286698
//616		1233	616	8388608	-16612088	8225947
//615		1230	615	8388608	-16561084	8174936
//613		1227	613	8388608	-16522585	8136430
//612		1225	612	8388608	-16498643	8112485
//71674	71674	0		8388608	-8245260		0


//621		1243	621	8388608	-16739949	8353827
//filter #(
//	.b0(621),
//	.b1(1243),
//	.b2(621), 
//	.a0(8388608),
//	.a1(-16739949),
//	.a2(8353827)
//	) layer_1 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(raw_pdm_data), 
//	.y(filt_pdm_data_1)
//	);
//
////619		1238	619	8388608	-16672830	8286698
//filter #(
//	.b0(621),
//	.b1(1238),
//	.b2(619), 
//	.a0(8388608),
//	.a1(-16672830),
//	.a2(8286698)
//	) layer_2 (
//	.clk(pdm_clk), 
//	.reset(reset), 
//	.x(filt_pdm_data_1), 
//	.y(filt_pdm_data_2),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
//	);
	
//616		1233	616	8388608	-16612088	8225947
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
//	.y(filt_pdm_data_3),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
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
//	.y(filt_pdm_data_4),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
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
//	.y(filt_pdm_data_5),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
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
//	.y(filt_pdm_data_6),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
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
//	.y(filt_pdm_data_7),
//	.x_1_out(x_1),
//	.x_2_out(x_2), 
//	.y_1_out(y_1),
//	.y_2_out(y_2)
//	);

endmodule

////reg slow_clk_prev;
//reg [8:0] pdm_bit_counter;
////reg [7:0] raw_pdm_data;

// ---- filter coefficients ----
//	79749	159499	79749	107613222200000	-214748364800000	107167042400000
//	79749	159499	79749	108046436800000	-214748364800000	106733827800000
//	79749	159499	79749	108441509400000	-214748364800000	106338755200000
//	79749	159499	79749	108775479700000	-214748364800000	106004784900000
//	79749	159499	79749	109028938700000	-214748364800000	105751325900000
//	79749	159499	79749	109187156300000	-214748364800000	105593108300000
//	18348550	18348550	0	214748364800000	-211078654700000	0

//
//always @(posedge reset) begin : RESET_FILTER_VALUES
//	if (reset == 1) begin
//		filt_pdm_data_1 <= 8'd0;
//		filt_pdm_data_2 <= 49'd0;
//		filt_pdm_data_3 <= 49'd0;
//		filt_pdm_data_4 <= 49'd0;
//		filt_pdm_data_5 <= 49'd0;
//		filt_pdm_data_6 <= 49'd0;
//		filt_pdm_data_7 <= 49'd0;
//	end 
//end

	// not so smart array filter decleration
//	b0[0] <= 32'd79749;
//	b0[1] <= 32'd79749;
//	b0[2] <= 32'd79749;
//	b0[3] <= 32'd79749;
//	b0[4] <= 32'd79749;
//	b0[5] <= 32'd79749;
//	b0[6] <= 32'd18348550;
//	
//	b1[0] <= 32'd159499;
//	b1[1] <= 32'd159499;
//	b1[2] <= 32'd159499;
//	b1[3] <= 32'd159499;
//	b1[4] <= 32'd159499;
//	b1[5] <= 32'd159499;
//	b1[6] <= 32'd18348550;
//	
//	b2[0] <= 32'd79749;
//	b2[1] <= 32'd79749;
//	b2[2] <= 32'd79749;
//	b2[3] <= 32'd79749;
//	b2[4] <= 32'd79749;
//	b2[5] <= 32'd79749;
//	b2[6] <= 32'd0;
//
//	a0[0] <= 48'd107613222200000;
//	a0[1] <= 48'd108046436800000;
//	a0[2] <= 48'd108441509400000;
//	a0[3] <= 48'd108775479700000;
//	a0[4] <= 48'd109028938700000;
//	a0[5] <= 48'd109187156300000;
//	a0[6] <= 48'd214748364800000;
//
//	a1[0] <= -48'd214748364800000;
//	a1[1] <= -48'd214748364800000;
//	a1[2] <= -48'd214748364800000;
//	a1[3] <= -48'd214748364800000;
//	a1[4] <= -48'd214748364800000;
//	a1[5] <= -48'd214748364800000;
//	a1[6] <= -48'd211078654700000;
//
//	a2[0] <= 48'd107167042400000;
//	a2[1] <= 48'd106733827800000;
//	a2[2] <= 48'd106338755200000;
//	a2[3] <= 48'd106004784900000;
//	a2[4] <= 48'd105751325900000;
//	a2[5] <= 48'd105593108300000;
//	a2[6] <= 48'd0;
//	

////		if(slow_clk_prev == 0 && pdm_clk == 1)begin // rising edge jof pdm_clk
//			case(pdm_state)
//			
//			default: begin
//				pdm_state <= DATA_TO_REG;
//			end
//			endcase
////			end
				
//		case(onchip_state)
//			IDLE: begin
////				if(slow_clk_prev == 0 && pdm_clk == 1)begin 
////						onchip_state <= DATA_TO_REG;
////					end
//			end
			
//			DATA_TO_REG: begin
//				raw_pdm_data[2] <= raw_pdm_data[1];
//				raw_pdm_data[1] <= raw_pdm_data[0];
//				raw_pdm_data[0] <= pdm * 8'hff;
//				onchip_state <= FILTER;
//			end
			
//			FILTER: begin
//			// row 1
//				filt_pdm_data_1[2] = filt_pdm_data_1[1];
//				filt_pdm_data_1[1] = filt_pdm_data_1[0];
//				filt_pdm_data_1[0] = (b0[0] * raw_pdm_data[0] + b1[0] * raw_pdm_data[1] + b2[0] * raw_pdm_data[2]
//										- a1[0] * filt_pdm_data_1[1] - a2[0] * filt_pdm_data_1[2]) / a0[0];
//				
//			
//				// row 2
//				filt_pdm_data_1[2] = filt_pdm_data_2[1];
//				filt_pdm_data_2[1] = filt_pdm_data_2[0];
//				filt_pdm_data_2[0] = (b0[1] * filt_pdm_data_1[0] + b1[1] * filt_pdm_data_1[1] + b2[1] * filt_pdm_data_1[2]
//										- a1[1] * filt_pdm_data_2[1] - a2[1] * filt_pdm_data_2[2]) / a0[1];
//				
//				
//			
//				// row 3
//				filt_pdm_data_1[2] = filt_pdm_data_3[1];
//				filt_pdm_data_3[1] = filt_pdm_data_3[0];
//				filt_pdm_data_3[0] = (b0[4] * filt_pdm_data_2[0] + b1[2] * filt_pdm_data_2[1] + b2[2] * filt_pdm_data_2[2]
//										- a1[2] * filt_pdm_data_3[1] - a2[2] * filt_pdm_data_3[2]) / a0[2];
//										
//				
//			
//				// row 4
//				filt_pdm_data_1[2] = filt_pdm_data_4[1];
//				filt_pdm_data_4[1] = filt_pdm_data_4[0];
//				filt_pdm_data_4[0] = (b0[3] * filt_pdm_data_3[0] + b1[3] * filt_pdm_data_3[1] + b2[3] * filt_pdm_data_3[2]
//										- a1[3] * filt_pdm_data_4[1] - a2[3] * filt_pdm_data_4[2]) / a0[3];
//										
//										
//				
//			
//				// row 5
//				filt_pdm_data_1[2] = filt_pdm_data_5[1];
//				filt_pdm_data_5[1] = filt_pdm_data_5[0];
//				filt_pdm_data_5[0] = (b0[4] * filt_pdm_data_4[0] + b1[4] * filt_pdm_data_4[1] + b2[4] * filt_pdm_data_4[2]
//										- a1[4] * filt_pdm_data_5[1] - a2[4] * filt_pdm_data_5[2]) / a0[4];
//										
//				
//			
//				// row 6
//				filt_pdm_data_1[2] = filt_pdm_data_1[1];
//				filt_pdm_data_1[1] = filt_pdm_data_1[0];
//				filt_pdm_data_6[0] = (b0[5] * filt_pdm_data_5[0] + b1[5] * filt_pdm_data_5[1] + b2[5] * filt_pdm_data_5[2]
//										- a1[5] * filt_pdm_data_6[1] - a2[5] * filt_pdm_data_6[2]) / a0[5];
//										
//				
//			
//				// row 7
//				filt_pdm_data_1[2] = filt_pdm_data_7[1];
//				filt_pdm_data_7[1] = filt_pdm_data_7[0];
//				filt_pdm_data_7[0] = (b0[6] * filt_pdm_data_6[0] + b1[6] * filt_pdm_data_6[1] + b2[6] * filt_pdm_data_6[2]
//										- a1[6] * filt_pdm_data_7[1] - a2[6] * filt_pdm_data_7[2]) / a0[6];
//				onchip_state = DECIMATION;
//			end
			
//			DECIMATION: begin
//				if (dec_cnt == 0) begin
//					write_data <= filt_pdm_data_7[0];
//					onchip_state <= TRANSMIT;
//					address <= addr_cntr;
//					dec_cnt <= dec_cnt + 1;
//				end else if(dec_cnt == 191) begin
//					dec_cnt <= 0;
//					onchip_state <= IDLE;
//				end else begin
//					dec_cnt <= dec_cnt + 1;
//					onchip_state <= IDLE;
//				end
//			end
//       ---- old version ----		
//			SUM_BITS: begin
//			// low pass filter summation
////				pdm_sum = 0;
////				for(i = 0; i < 256; i = i + 1) begin
////					pdm_sum = pdm_sum + raw_pdm_data[i];
////					end
//				write_data = raw_pdm_data;
//				onchip_state = TRANSMIT;
//				address = addr_cntr;
//			end

//reg pdm_state;

//reg [31:0] b0 [6:0];
//
//reg [31:0] b1 [6:0]; 
//reg [31:0] b2 [6:0];
//reg [47:0] a0 [6:0];
//reg [47:0] a1 [6:0];
//reg [47:0] a2 [6:0];
//reg [7:0] dec_cnt;


//always @(posedge dec_clk, posedge reset) begin : DECIMATION
//	if (reset ==1) begin
//		write_data <= 0;
//	end else begin
//		write_data <= 8'hc5;//filt_pdm_data_7[7:0];
////		onchip_state <= TRANSMIT;
//	end
//end 
