`timescale 1ns/10ps

module Counter (
	input clk,
	input reset,
	output reg [31:0] counter
);

always @(posedge clk, posedge reset) begin: COUNTER_COUNTERLOGIC
    if (reset == 1) begin
			counter <= 0;
    end else begin
			counter <= counter + 1;
	 end
end

endmodule
