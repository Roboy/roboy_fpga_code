// PD controller for the msj platform
// This module implements a PID controller. It has 3 control modes (position, velocity, direct feed through).
// It uses integers only.
//
//	BSD 3-Clause License
//
//	Copyright (c) 2018, Roboy
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

// author: Simon Trendel, simon.trendel@tum.de, 2018

`timescale 1ns/10ps

module MSJPlatformPIDController (
	input clock,
	input reset,
	input signed [31:0] Kp,
	input signed [31:0] Ki,
	input signed [31:0] Kd,
	input signed [31:0] sp,
	input signed [31:0] outputPosMax,
	input signed [31:0] outputNegMax,
	input signed [31:0] integralNegMax,
	input signed [31:0] integralPosMax,
	input signed [31:0] deadBand,
	input signed [31:0] zero_speed,
	input [1:0] control_mode, // position velocity direct_duty
	input signed [31:0] position,
	input signed [31:0] velocity,
	input signed [31:0] outputDivider,
	input update_controller,
	output reg signed [31:0] duty
	);

always @(posedge clock, posedge reset) begin: PD_CONTROLLER_PD_CONTROLLERLOGIC
	reg signed [31:0] lastError;
	reg signed [31:0] err;
	reg signed [31:0] integral;
	reg signed [31:0] pterm;
	reg signed [31:0] dterm;
	reg signed [31:0] ffterm;
	reg update_controller_prev;
	reg signed [31:0] result;
	
	if (reset == 1) begin
		lastError <= 0;
		err <=0;
		result <= 0;
		update_controller_prev <= 0;
		integral <= 0;
	end else begin
		update_controller_prev <= update_controller;
		if(update_controller_prev==0 && update_controller==1) begin
			case(control_mode) 
				2'b00: err = (sp - position)/outputDivider; 
				2'b01: err = (sp - velocity)/outputDivider;
				2'b10: duty = sp; // direct feed through
				default: err = 0;
			endcase;
			if(control_mode!=2'b10) begin
				if (((err >= deadBand) || (err <= ((-1) * deadBand)))) begin
					pterm = (Kp * err);
					if ((pterm < (zero_speed + outputPosMax)) || (pterm > (zero_speed + outputNegMax))) begin  //if the proportional term is not maxed
						integral = integral + (Ki * err); //add to the integral
						if (integral > integralPosMax) begin
							integral = integralPosMax;
						end else if (integral < integralNegMax) begin
							integral = integralNegMax;
						end
					end
					dterm = ((err - lastError) * Kd);
					result = zero_speed + (pterm + dterm + integral);
					if ((result < outputNegMax)) begin
						 result = outputNegMax;
					end else if ((result > outputPosMax)) begin
						 result = outputPosMax;
					end
				end else begin
					result = zero_speed;
				end
				duty = result;
				lastError = err;
			end
		end
	end 
end


endmodule

