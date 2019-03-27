// PID controller 
// This module implements a PID controller. It has 3 control modes (position, velocity, displacement).
// It uses single precision floating point
//
//	BSD 3-Clause License
//
//	Copyright (c) 2019, Roboy
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without
//	modification, are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice, this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright notice,
//	  this list of conditions and the following disclaimer in the documentation
//	  and/or other materials provided with the distribution.
//
//	* Neither the name of the copyright holder nor the names of its
//	  contributors may be used to endorse or promote products derived from
//	  this software without specific prior written permission.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// author: Simon Trendel, simon.trendel@tum.de, 2019

`timescale 1ns/10ps

module PID_controller (
	input clock,
	input reset,
	input [31:0] Kp,
	input [31:0] Ki,
	input [31:0] Kd,
	input [31:0] state,
	input [31:0] setpoint,
	input [31:0] outputPosMax,
	input [31:0] outputNegMax,
	input [31:0] integralPosMax,
	input [31:0] integralNegMax,
	input [31:0] deadBandPosMax,
	input [31:0] deadBandNegMax,
	input update_controller,
	output reg [31:0] result
	);
	
localparam ADD = 0, SUB = 1, MUL = 2, DIV = 3, I2F = 4, F2I = 5;
	
wire [31:0] s;
wire [31:0] err;
wire [31:0] d_err;
reg [31:0] err_prev;
wire [31:0] p_term;
wire [31:0] d_term;
wire [31:0] i_term;
reg [31:0] integral;
wire [31:0] res;
wire [31:0] res_with_upper_limit;
wire [31:0] res_with_limits;
wire [31:0] res_integer;
wire [31:0] p_term_res;
wire [31:0] pd_term_res;
wire [31:0] pid_term_res;
wire [31:0] one;

// output limits
wire res_lt_outputPosMax, res_is_nan, res_is_inf, blta0, aeqb0, zero0;
fcmp cmp0(pid_term_res, outputPosMax, res_is_nan, res_lt_outputPosMax, blta0, aeqb0, res_is_inf, zero0);

wire outputNegMax_lt_res_with_upper_limit, res_with_upper_limit_is_nan, res_with_upper_limit_is_inf, blta1, aeqb1, zero1;
fcmp cmp1(outputNegMax, res_with_upper_limit, res_with_upper_limit_is_nan, outputNegMax_lt_res_with_upper_limit, blta1, aeqb1, res_with_upper_limit_is_inf, zero1);

assign res_with_upper_limit = ((res_is_nan||res_is_inf||aeqb0)?0:(res_lt_outputPosMax?outputPosMax:pid_term_res));
assign res_with_limits = ((res_with_upper_limit_is_nan||res_with_upper_limit_is_inf||aeqb1)?0:(outputNegMax_lt_res_with_upper_limit?outputPosMax:res_with_upper_limit));

// dead_band
wire deadBandPosMax_lt_err, err_is_nan0, err_is_inf0, blta2, aeqb2, zero2;
fcmp cmp2(deadBandPosMax, err, err_is_nan0, deadBandPosMax_lt_err, blta2, aeqb2, err_is_inf0, zero2);

wire err_lt_deadBandNegMax, err_is_nan1, err_is_inf1, blta3, aeqb3, zero3;
fcmp cmp3(err, deadBandNegMax, err_is_nan1, err_lt_deadBandNegMax, blta3, aeqb3, err_is_inf1, zero3);

wire deadband_active;
assign deadband_active = ((deadBandPosMax_lt_err||aeqb2) && (err_lt_deadBandNegMax||aeqb3));

//fpu convert_to_integer(clock, 0, F2I, res_with_limits, 0, res_integer);
	
fpu f1(clock, 0, SUB, setpoint, state, err);
fpu f2(clock, 0, SUB, err, err_prev, d_err);
fpu f3(clock, 0, MUL, Kp, err, p_term);
fpu f4(clock, 0, MUL, Ki, err, i_term);
fpu f5(clock, 0, MUL, Kd, d_err, d_term);
fpu f6(clock, 0, ADD, integral, i_term, integral);
fpu f7(clock, 0, ADD, result, p_term, p_term_res);
fpu f8(clock, 0, ADD, p_term_res, d_term, pd_term_res);
fpu f9(clock, 0, ADD, pd_term_res, integral, pid_term_res);

always @(posedge clock, posedge reset) begin: PD_CONTROLLER_PD_CONTROLLERLOGIC
	if(reset) begin
		result <= 0;
		err_prev <= 0;
		integral <= 0;
	end else begin
		if(update_controller) begin
			err_prev <= err;
			result <= (deadband_active?0:res_with_limits);
		end
	end
end

//
//always @(posedge clock, posedge reset) begin: PD_CONTROLLER_PD_CONTROLLERLOGIC
//	reg [31:0] lastError;
//	reg [31:0] err;
//	reg [31:0] integral;
//	reg [31:0] pterm;
//	reg [31:0] dterm;
//	reg [31:0] ffterm;
//	reg update_controller_prev;
//	reg [31:0] result;
//	
//	if (reset == 1) begin
//		lastError <= 0;
//		err <=0;
//		result <= 0;
//		update_controller_prev <= 0;
//		integral <= 0;
//	end else begin
//		update_controller_prev <= update_controller;
//		if(update_controller_prev==0 && update_controller==1) begin
//			case(control_mode) 
//				2'b00: err = (sp - position)/outputDivider; 
//				2'b01: err = (sp - velocity)/outputDivider;
//				2'b10: duty = sp; // direct feed through
//				default: err = 0;
//			endcase;
//			if(control_mode!=2'b10) begin
//				if (((err >= deadBand) || (err <= ((-1) * deadBand)))) begin
//					pterm = (Kp * err);
//					if ((pterm < (zero_speed + outputPosMax)) || (pterm > (zero_speed + outputNegMax))) begin  //if the proportional term is not maxed
//						integral = integral + (Ki * err); //add to the integral
//						if (integral > integralPosMax) begin
//							integral = integralPosMax;
//						end else if (integral < integralNegMax) begin
//							integral = integralNegMax;
//						end
//					end
//					dterm = ((err - lastError) * Kd);
//					result = zero_speed + (pterm + dterm + integral);
//					if ((result < outputNegMax)) begin
//						 result = outputNegMax;
//					end else if ((result > outputPosMax)) begin
//						 result = outputPosMax;
//					end
//				end else begin
//					result = zero_speed;
//				end
//				duty = result;
//				lastError = err;
//			end
//		end
//	end 
//end


endmodule
