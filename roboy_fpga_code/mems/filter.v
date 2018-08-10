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
input x,
output reg [48:0] y
);
  
reg [48:0] x_1;
reg [48:0] x_2;
reg [48:0] y_1;
reg [48:0] y_2;
 
always @(posedge clk or posedge reset) begin
	if (reset) begin
		x_1 <= 49'd0;
		x_2 <= 49'd0;
		y_1 <= 49'd0;
		y_2 <= 49'd0;
		y <= 49'd0;
	end else begin 
		y <= (b0 * x + b1 * x_1 + b2 * x_2 - a1 * y_1 - a2 * y_2) / a0;
		y_1 <= y;
		y_2 <= y_1;
		x_2 <= x_1;
		x_1 <= x;
	end
end
endmodule