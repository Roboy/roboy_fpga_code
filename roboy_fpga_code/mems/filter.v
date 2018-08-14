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
		x_1 <= 32'd0;
		x_2 <= 32'd0;
		y_1 <= 32'd0;
		y_2 <= 32'd0;
		y <= 32'd0;
		x_1_out <= 32'd0;
		x_2_out <= 32'd0;
		y_1_out <= 32'd0;
		y_2_out <= 32'd0;

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


