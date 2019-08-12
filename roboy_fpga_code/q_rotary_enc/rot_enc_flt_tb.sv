timeunit 1ns;
timeprecision 100ps;

module rot_enc_flt_tb;

	bit clock = 0, sclr = 1;
	wire A, B;
	bit dir = 0;
	wire signed [31:0] bidir_counter;
	wire error;
	wire ready;
	
	always #10ns clock++;
	
	bit rot_dir = 0, run = 0;
	
	initial begin
		repeat(10) @(posedge clock);
		
		sclr = 1'b0;
		
		wait(ready);
		
		repeat(10_000) @(posedge clock);
		
		rot_dir = 0;
		run = 1;
		
		#1ms
		
		run = 0;
		
		#500us
		
		rot_dir = 1;
		run = 1;
		
		#1ms
		
		run = 0;
		
		repeat(10_000) @(posedge clock);
		
		$stop(2);
		
	end
	
	rot_enc_flt dut(.*);	
	rotary_sensor sensor(.clock, .run, .rot_dir, .A, .B);
	
endmodule :rot_enc_flt_tb

module rotary_sensor(input clock, run, rot_dir, output A, B);

	bit[1:0] gray[4] = {2'b00, 2'b01, 2'b11, 2'b10};
	
	localparam BPS = 10_000 * 256 * 4 / 60;
	localparam CLOCK_MHZ = 50_000_000;
	localparam CNT_MAX = CLOCK_MHZ / BPS - 1;
	
	int bit_cnt = 0;
	
	always_ff @(posedge clock)
		if (bit_cnt == CNT_MAX)
			bit_cnt <= 0;
		else
			bit_cnt <= bit_cnt + 1;
	
	wire bit_clk = bit_cnt == CNT_MAX;
	
	bit [1:0] gray_cnt = 0;
	always_ff @(posedge clock)
		if (bit_clk && run)
			if (rot_dir == 0)
				gray_cnt <= gray_cnt + 1'b1;
			else
				gray_cnt <= gray_cnt - 1'b1;
	
	assign {B, A} = gray[gray_cnt];
	
endmodule :rotary_sensor
