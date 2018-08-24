// Discrete-Time IIR Filter (lowpass - real)
// -------------------------------
//
// Developer: Ânderson I. Silva
// Repository: https://github.com/aignacio/iir_filter
//
// Parameters:
//
// Fs : 48000 Samples/s
// Fc : 1000 Hz
// Fs : 1500 Hz
// Apass: 1dB
// Astop: -15dB
// Filter Structure    : Direct-Form I, Second-Order Sections
// Architecture: Cascade
// Number of Sections  : 3
// Stable              : Yes
// Linear Phase        : No
// Arithmetic          : fixed
//
// Design Method Information
// Design Algorithm : butter
//
// Design Options
// Match Exactly : stopband
// Scale Norm    : no scaling
// SystemObject  : false
//
// Design Specifications
// Sample Rate     : N/A (normalized frequency)
// Response        : Lowpass
// Specification   : Fp,Fst,Ap,Ast
// Stopband Atten. : 15 dB
// Passband Ripple : 1 dB
// Stopband Edge   : 0.0625
// Passband Edge   : 0.041667
//
// Measurements
// Sample Rate      : N/A (normalized frequency)
// Passband Edge    : 0.041667
// 3-dB Point       : 0.047064
// 6-dB Point       : 0.051568
// Stopband Edge    : 0.0625
// Passband Ripple  : 0.89022 dB
// Stopband Atten.  : 14.9855 dB
// Transition Width : 0.020833
//
// Ex arc.:
// 3x --->  X -------b0---> + ------> + -----------> Y
//               |          |         |          |
//               Z1         |         |          Z1
//               |          |         |          |
//               ----b1-----|         |----a1-----
//               |          |         |          |
//               Z2         |         |          Z2
//               |          |         |          |
//               ----b3-----|         |----a2-----

module my_iir (
	input clk,
	input rst,
	input signed [31:0] x,
	output signed [31:0] y
);
  wire signed [31:0] s1_s2, s2_s3;

  reg signed [63:0] z1_a_s1, z2_a_s1,
  								  z1_b_s1, z2_b_s1;
  wire signed [31:0] z1_a_next_s1, z2_a_next_s1,
  								   z1_b_next_s1, z2_b_next_s1;
  wire signed [63:0] b_out_s1, a_out_s1;
  wire signed [31:0] a1_s1, a2_s1,
  					         b0_s1, b1_s1, b2_s1;

  reg signed [63:0] z1_a_s2, z2_a_s2,
                    z1_b_s2, z2_b_s2;
  wire signed [31:0] z1_a_next_s2, z2_a_next_s2,
                     z1_b_next_s2, z2_b_next_s2;
  wire signed [63:0] b_out_s2, a_out_s2;
  wire signed [31:0] a1_s2, a2_s2,
                     b0_s2, b1_s2, b2_s2;

  reg signed [63:0] z1_a_s3, z2_a_s3,
                    z1_b_s3, z2_b_s3;
  wire signed [31:0] z1_a_next_s3, z2_a_next_s3,
                     z1_b_next_s3, z2_b_next_s3;
  wire signed [63:0] b_out_s3, a_out_s3;
  wire signed [31:0] a1_s3, a2_s3,
                     b0_s3, b1_s3, b2_s3;

  assign z1_b_next_s1 = x;
  assign z2_b_next_s1 = z1_b_s1;
  assign z1_a_next_s1 = s1_s2;
  assign z2_a_next_s1 = z1_a_s1;
  assign b_out_s1 = x*b0_s1 + z1_b_s1*b1_s1 + z2_b_s1*b2_s1;
  assign a_out_s1 = b_out_s1 - z1_a_s1*a1_s1 - z2_a_s1*a2_s1;
  assign s1_s2 = a_out_s1 >> 23;

  assign z1_b_next_s2 = s1_s2;
  assign z2_b_next_s2 = z1_b_s2;
  assign z1_a_next_s2 = s2_s3;
  assign z2_a_next_s2 = z1_a_s2;
  assign b_out_s2 = s1_s2*b0_s2 + z1_b_s2*b1_s2 + z2_b_s2*b2_s2;
  assign a_out_s2 = b_out_s2 - z1_a_s2*a1_s2 - z2_a_s2*a2_s2;
  assign s2_s3 = a_out_s2 >> 23;

  assign z1_b_next_s3 = s2_s3;
  assign z2_b_next_s3 = z1_b_s3;
  assign z1_a_next_s3 = y;
  assign z2_a_next_s3 = z1_a_s3;
  assign b_out_s3 = s2_s3*b0_s3 + z1_b_s3*b1_s3 + z2_b_s3*b2_s3;
  assign a_out_s3 = b_out_s3 - z1_a_s3*a1_s3 - z2_a_s3*a2_s3;
  assign y = a_out_s3 >> 23;

  // Q20 Format Coed = Floating Point * 2^23
  // -------------- SECTION 1 --------------
	assign b0_s1 = 621;
	assign b1_s1 = 1243;
	assign b2_s1 = 621;
	assign a1_s1 = -16739949;
	assign a2_s1 = 8353827;
  // -------------- SECTION 2 --------------
  assign b0_s2 = 619;
  assign b1_s2 = 1238;
  assign b2_s2 = 619;
  assign a1_s2 = -16672830;
  assign a2_s2 = 8286698;
  // -------------- SECTION 3 --------------
  assign b0_s3 = 616;
  assign b1_s3 = 1233;
  assign b2_s3 = 616;
  assign a1_s3 = -16612088;
  assign a2_s3 = 8225947;
  
	always @ (posedge clk or negedge rst) begin
		if (rst == 1'b0) begin
			z1_a_s1 <= 32'd0;
			z2_a_s1 <= 32'd0;
			z1_b_s1 <= 32'd0;
			z2_b_s1 <= 32'd0;

      z1_a_s2 <= 32'd0;
      z2_a_s2 <= 32'd0;
      z1_b_s2 <= 32'd0;
      z2_b_s2 <= 32'd0;

      z1_a_s3 <= 32'd0;
      z2_a_s3 <= 32'd0;
      z1_b_s3 <= 32'd0;
      z2_b_s3 <= 32'd0;
		end else begin
			z1_a_s1 <= z1_a_next_s1;
			z2_a_s1 <= z2_a_next_s1;
			z1_b_s1 <= z1_b_next_s1;
			z2_b_s1 <= z2_b_next_s1;

      z1_a_s2 <= z1_a_next_s2;
      z2_a_s2 <= z2_a_next_s2;
      z1_b_s2 <= z1_b_next_s2;
      z2_b_s2 <= z2_b_next_s2;

      z1_a_s3 <= z1_a_next_s3;
      z2_a_s3 <= z2_a_next_s3;
      z1_b_s3 <= z1_b_next_s3;
      z2_b_s3 <= z2_b_next_s3;
		end
	end
endmodule
