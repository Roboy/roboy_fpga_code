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
	input [31:0] integralNegMax,
	input [31:0] integralPosMax,
	input [31:0] deadBand,
	input update_controller,
	output [31:0] result
	);
	
localparam ADD = 0, SUB = 1, MUL = 2, DIV = 3, I2F = 4, F2I = 5;
	
wire [31:0] pos;
wire [31:0] vel;
wire [31:0] err;
wire [31:0] d_err;
reg [31:0] err_prev;
wire [31:0] p_term;
wire [31:0] d_term;
wire [31:0] i_term;
wire [31:0] res;
wire [31:0] res_with_upper_limit;
wire [31:0] res_with_limits;
wire [31:0] res_integer;


wire res_lt_outputPosMax, res_is_nan, res_is_inf;
fcmp(res, outputPosMax, res_is_nan, res_lt_outputPosMax, 0, 0, res_is_inf, 0);

wire outputNegMax_lt_res_with_upper_limit, res_with_upper_limit_is_nan, res_with_upper_limit_is_inf;
fcmp(outputNegMax, res_with_upper_limit, res_with_upper_limit_is_nan, outputNegMax_lt_res_with_upper_limit, 0, 0, res_with_upper_limit_is_nan, 0);

assign res_with_upper_limit = ((res_is_nan||res_is_inf)?0:(res_lt_outputPosMax?outputPosMax:res));
assign res_with_limits = ((res_with_upper_limit_is_nan||res_with_upper_limit_is_inf)?0:(outputNegMax_lt_res_with_upper_limit?outputNegMax:res_with_upper_limit));

fpu(clock, 0, F2I, res_with_limits, 0, result);

	
fpu(clock, 0, I2F, position, 0, pos);
fpu(clock, 0, I2F, velocity, 0, vel);
fpu(clock, 0, SUB, sp, pos, err);
fpu(clock, 0, SUB, err, err_prev, d_err);
fpu(clock, 0, MUL, Kp, err, p_term);
fpu(clock, 0, MUL, Kd, d_err, d_term);
fpu(clock, 0, ADD, p_term, d_term, res);

always @(posedge clock, posedge reset) begin: PD_CONTROLLER_PD_CONTROLLERLOGIC
	if(reset) begin
		
	end else begin
		if(update_controller) begin
			err_prev <= err;
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
