// -------------------------------------------------------------
//
// Module: hdlbutter
// Generated by MATLAB(R) 9.3 and Filter Design HDL Coder 3.1.2.
// Generated on: 2018-11-21 19:29:48
// -------------------------------------------------------------

// -------------------------------------------------------------
// HDL Code Generation Options:
//
// TargetDirectory: /tmp/tpcc6e8ac0_4bd3_4ff5_bda0_94e7d2eb1d50
// Name: hdlbutter
// InputDataType: numerictype(1,12,0)
// TargetLanguage: Verilog
// TestBenchName: hdlbutter_tb
// TestBenchUserStimulus:  User data, length 1818

// -------------------------------------------------------------
// -------------------------------------------------------------
// Filter Settings:
//

// -------------------------------------------------------------
// Multipliers           : 6



`timescale 1 ns / 1 ns

module lowpass_filter
               (
                clk,
                clk_enable,
                reset,
                filter_in,
                filter_out
                );

  input   clk; 
  input   clk_enable; 
  input   reset; 
  input   signed [11:0] filter_in; //sfix12
  output  signed [11:0] filter_out; //sfix12

////////////////////////////////////////////////////////////////
//Module Architecture: hdlbutter
////////////////////////////////////////////////////////////////
  // Local Functions
  // Type Definitions
  // Constants
  parameter signed [15:0] coeff_b1_section1 = 16'b0010010101011001; //sfix16_En27
  parameter signed [15:0] coeff_b2_section1 = 16'b0100101010110010; //sfix16_En27
  parameter signed [15:0] coeff_b3_section1 = 16'b0010010101011001; //sfix16_En27
  parameter signed [15:0] coeff_a2_section1 = 16'b1000000110001001; //sfix16_En14
  parameter signed [15:0] coeff_a3_section1 = 16'b0011111001111011; //sfix16_En14
  parameter signed [15:0] scaleconst2 = 16'b1000000000000000; //sfix16_En15
  // Signals
  reg  signed [11:0] input_register; // sfix12
  // Section 1 Signals 
  wire signed [19:0] numtypeconvert1; // sfix20_En15
  wire signed [31:0] a1sum1; // sfix32_En25
  wire signed [19:0] dentypeconvert1; // sfix20_En15
  reg  signed [19:0] numdelay_section1 [0:1] ; // sfix20_En15
  reg  signed [19:0] dendelay_section1 [0:1] ; // sfix20_En15
  wire signed [35:0] a2mul1; // sfix36_En29
  wire signed [35:0] a3mul1; // sfix36_En29
  wire signed [35:0] b1mul1; // sfix36_En42
  wire signed [35:0] b2mul1; // sfix36_En42
  wire signed [35:0] b3mul1; // sfix36_En42
  wire signed [31:0] b1sum1; // sfix32_En24
  wire signed [31:0] b2sum1; // sfix32_En24
  wire signed [31:0] b1multypeconvert1; // sfix32_En24
  wire signed [31:0] add_cast; // sfix32_En24
  wire signed [31:0] add_cast_1; // sfix32_En24
  wire signed [32:0] add_temp; // sfix33_En24
  wire signed [31:0] add_cast_2; // sfix32_En24
  wire signed [31:0] add_cast_3; // sfix32_En24
  wire signed [32:0] add_temp_1; // sfix33_En24
  wire signed [31:0] midtypeconvert1; // sfix32_En25
  wire signed [31:0] a2sum1; // sfix32_En25
  wire signed [31:0] sub_cast; // sfix32_En25
  wire signed [31:0] sub_cast_1; // sfix32_En25
  wire signed [32:0] sub_temp; // sfix33_En25
  wire signed [31:0] sub_cast_2; // sfix32_En25
  wire signed [31:0] sub_cast_3; // sfix32_En25
  wire signed [32:0] sub_temp_1; // sfix33_En25
  wire signed [42:0] scale2; // sfix43_En30
  wire signed [35:0] mul_temp; // sfix36_En30
  wire signed [11:0] output_typeconvert; // sfix12
  reg  signed [11:0] output_register; // sfix12

  // Block Statements
  always @ (posedge clk or posedge reset)
    begin: input_reg_process
      if (reset == 1'b1) begin
        input_register <= 0;
      end
      else begin
        if (clk_enable == 1'b1) begin
          input_register <= filter_in;
        end
      end
    end // input_reg_process

  // ------------------ Section 1 ------------------

  assign numtypeconvert1 = $signed({input_register[4:0], 15'b000000000000000});

  assign dentypeconvert1 = (a1sum1[29:9] + 1)>>>1;

  always @ (posedge clk or posedge reset)
    begin: numdelay_process_section1
      if (reset == 1'b1) begin
        numdelay_section1[0] <= 20'b00000000000000000000;
        numdelay_section1[1] <= 20'b00000000000000000000;
      end
      else begin
        if (clk_enable == 1'b1) begin
          numdelay_section1[1] <= numdelay_section1[0];
          numdelay_section1[0] <= numtypeconvert1;
        end
      end
    end // numdelay_process_section1

  always @ (posedge clk or posedge reset)
    begin: dendelay_process_section1
      if (reset == 1'b1) begin
        dendelay_section1[0] <= 20'b00000000000000000000;
        dendelay_section1[1] <= 20'b00000000000000000000;
      end
      else begin
        if (clk_enable == 1'b1) begin
          dendelay_section1[1] <= dendelay_section1[0];
          dendelay_section1[0] <= dentypeconvert1;
        end
      end
    end // dendelay_process_section1

  assign a2mul1 = dendelay_section1[0] * coeff_a2_section1;

  assign a3mul1 = dendelay_section1[1] * coeff_a3_section1;

  assign b1mul1 = numtypeconvert1 * coeff_b1_section1;

  assign b2mul1 = numdelay_section1[0] * coeff_b2_section1;

  assign b3mul1 = numdelay_section1[1] * coeff_b3_section1;

  assign b1multypeconvert1 = ({{14{b1mul1[35]}}, b1mul1[35:17]} + 1)>>>1;

  assign add_cast = b1multypeconvert1;
  assign add_cast_1 = ({{14{b2mul1[35]}}, b2mul1[35:17]} + 1)>>>1;
  assign add_temp = add_cast + add_cast_1;
  assign b1sum1 = add_temp[31:0];

  assign add_cast_2 = b1sum1;
  assign add_cast_3 = ({{14{b3mul1[35]}}, b3mul1[35:17]} + 1)>>>1;
  assign add_temp_1 = add_cast_2 + add_cast_3;
  assign b2sum1 = add_temp_1[31:0];

  assign midtypeconvert1 = $signed({b2sum1[30:0], 1'b0});

  assign sub_cast = midtypeconvert1;
  assign sub_cast_1 = (a2mul1[35:3] + 1)>>>1;
  assign sub_temp = sub_cast - sub_cast_1;
  assign a2sum1 = sub_temp[31:0];

  assign sub_cast_2 = a2sum1;
  assign sub_cast_3 = (a3mul1[35:3] + 1)>>>1;
  assign sub_temp_1 = sub_cast_2 - sub_cast_3;
  assign a1sum1 = sub_temp_1[31:0];

  assign mul_temp = dentypeconvert1 * scaleconst2;
  assign scale2 = $signed({{7{mul_temp[35]}}, mul_temp});

  assign output_typeconvert = (scale2[41:29] + 1)>>>1;

  always @ (posedge clk or posedge reset)
    begin: Output_Register_process
      if (reset == 1'b1) begin
        output_register <= 0;
      end
      else begin
        if (clk_enable == 1'b1) begin
          output_register <= output_typeconvert;
        end
      end
    end // Output_Register_process

  // Assignment Statements
  assign filter_out = output_register;
endmodule  // hdlbutter