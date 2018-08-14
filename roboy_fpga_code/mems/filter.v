module filter 
#( 
parameter b0 = 5,
parameter b1 = 5,
parameter b2 = 5,
parameter a0 = 5,
parameter a1 = 5,
parameter a2 = 5
)(
input clk,
input reset,
input signed [31:0] x,
output reg signed [31:0] y,
output reg signed [31:0] x_1_out,
output reg signed [31:0] x_2_out,
output reg signed [31:0] y_1_out,
output reg signed [31:0] y_2_out
);
  
reg signed [31:0] x_1;
reg signed [31:0] x_2;
reg signed [31:0] y_1;
reg signed [31:0] y_2;
 
always @(posedge clk or posedge reset) begin
	if (reset) begin
		x_1 <= 24'd0;
		x_2 <= 24'd0;
		y_1 <= 24'd0;
		y_2 <= 24'd0;
		y <= 24'd0;
		x_1_out <= 24'd0;
		x_2_out <= 24'd0;
		y_1_out <= 24'd0;
		y_2_out <= 24'd0;

	end else begin 
		y <= ((b0 * x) + (b1 * x_1) + (b2 * x_2) - (a1 * y_1) - (a2 * y_2)) / a0;// / (a0 >> 31);
		y_1 <= y;
		y_2 <= y_1;
		x_2 <= x_1;
		x_1 <= x;
		x_1_out <= x; 
		x_2_out <= x_1;
		y_1_out <= y;
		y_2_out <= y_1;
	end
end
endmodule


module filter_testbench;

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
	raw_pdm_data = 0;
	end
	
	always 
		#5 pdm_clk = ! pdm_clk;
		
		
endmodule
