/*
	Counter for quadrature rotary encoder.
	Module with low-pass filter for A, B signals is rot_enc_flt.sv
*/

`ifndef _q_rotary_enc_
`define _q_rotary_enc_

module q_rotary_enc(
	input clock, sclr, ena,
	input dir, // main count direction. 0 - direct count, 1 - reverse
	input A, B, // signals from sensor. Signals /A, /B/, Z, /Z don't use. A, /A, B, /B more good connect to optocouplers
	output reg signed [31:0] bidir_counter, // current relative position
	output reg error // A, B error. For to clear error use sclr signal
);

	localparam bit[1:0] grey_inc2[4] = '{2'b01, 2'b11, 2'b00, 2'b10};
	localparam bit[1:0] grey_dec2[4] = '{2'b10, 2'b00, 2'b11, 2'b01};

	wire [1:0] cur_code;
	reg [1:0] old_code = '0;
	
	assign cur_code = (dir) ? {A, B} : {B, A};
	
	always_ff @(posedge clock)
		old_code <= cur_code;
	
	wire cnt_ena = old_code != cur_code;	
	wire inc = cnt_ena && cur_code == grey_inc2[old_code];
	wire dec = cnt_ena && cur_code == grey_dec2[old_code];
	wire err = cnt_ena && !(inc || dec);
	
	always_ff @(posedge clock)
		if (sclr)			bidir_counter <= 'sh0;
		else if (ena)
			if (inc)			bidir_counter <= bidir_counter + 1'b1;
			else if (dec)	bidir_counter <= bidir_counter - 1'b1;
	
	always_ff @(posedge clock)
		if (sclr)				error <= 1'b0;
		else if (ena && err)	error <= 1'b1;

endmodule :q_rotary_enc

`endif
