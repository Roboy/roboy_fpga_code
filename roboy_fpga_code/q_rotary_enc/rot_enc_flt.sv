/*
	Counter for quadrature rotary encoder with low-pass filter
*/

`ifndef _rot_enc_flt_
`define _rot_enc_flt_
`include "lpf_cap.sv"
`include "q_rotary_enc.sv"

module rot_enc_flt(
	input clock,
	input sclr,
	input dir, // main direction
	input A, B,
	output signed [31:0] bidir_counter,
	output error,
	output ready // counter enabled
);
	reg A_reg = 1'b0, B_reg = 1'b0;
	wire A_flt, B_flt;
	reg [1:0] dir_reg = '0;
	wire dir_changed;
	wire [1:0] init;	
	
	// to trigger A, B signals on input pins
	always_ff @(posedge clock) begin
		A_reg <= A;
		B_reg <= B;
		dir_reg <= {dir_reg[0], dir};
	end		
		
	assign dir_changed = dir_reg[1] ^ dir_reg[0];
	
	// debouncing
	lpf_cap #(7) // 2^7 - it's about 1/8 of min encoder period
		f0(.clock, .sclr(dir_changed), .in(A_reg), .out(A_flt), .init(init[0])),
		f1(.clock, .sclr(dir_changed), .in(B_reg), .out(B_flt), .init(init[1]));
	
	reg ready_reg = 1'b0;
	always_ff @(posedge clock)
		ready_reg <= (dir_changed) ? 1'b0 : init == '0;
	
	assign ready = ready_reg;
	
	q_rotary_enc enc_inst(
		.clock, .sclr(sclr || dir_changed), .ena(init == '0), .dir(dir_reg[0]),
		.A(A_flt), .B(B_flt),
		.bidir_counter,
		.error
	);

endmodule :rot_enc_flt

`endif
