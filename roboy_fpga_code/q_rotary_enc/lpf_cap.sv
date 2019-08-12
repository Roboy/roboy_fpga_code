/*
Simple low-pass filter. Digital capacity
*/

`ifndef _lpf_cap_
`define _lpf_cap_

module lpf_cap #(parameter FILTER_WIDTH = 7)( // about half bit
	input clock, sclr,
	input in,
	output out,
	output init
);

reg [FILTER_WIDTH-1:0] cnt = {1'b1, {(FILTER_WIDTH-2){1'b0}}};
always_ff @(posedge clock)
	if (sclr)
		cnt <= {1'b1, {(FILTER_WIDTH-2){1'b0}}}; // middle value
	else if (in == 1'b1 && cnt != '1)
		cnt <= cnt + 1'b1; // charging
	else if (in == 1'b0 && cnt != '0)
		cnt <= cnt - 1'b1; // discharging

reg out_reg = 1'b0;
always_ff @(posedge clock)
	if (sclr)
		out_reg <= 1'b0;
	else if (cnt == '1) // full
		out_reg <= 1'b1;
	else if (cnt == '0) // empty
		out_reg <= 1'b0;

assign out = out_reg;

reg init_reg = 1'b1;
always_ff @(posedge clock)
	if (sclr)
		init_reg <= 1'b1;
	else if (cnt == '1 || cnt == '0)
		init_reg <= 1'b0;

assign init = init_reg;

endmodule :lpf_cap

`endif
