module filter_tb;

reg pdm_clk, reset;
reg [63:0] raw_pdm_data;
reg [63:0] filt_pdm_data_1;


filter #(
	.b0(24'd311),
	.b1(24'd623),
	.b2(24'd311), 
	.a0(24'd4203641),
	.a1(24'd8388608),
	.a2(24'd4186212)
	) layer_1 (
	.clk(pdm_clk), 
	.reset(reset), 
	.x(raw_pdm_data), 
	.y(filt_pdm_data_1)
	);
	
	
initial begin
	pdm_clk = 0;
	reset = 0;
	end
	
	always 
		#5 pdm_clk = ! pdm_clk;
		
		
endmodule