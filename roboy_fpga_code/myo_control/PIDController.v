// PID controller myoRobotics style
// This module implements a PID controller. It has 3 control modes (position, velocity,displacement).
// It uses integers only.
//
//	BSD 3-Clause License
//
//	Copyright (c) 2017, Roboy
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

module PIDController (
	input clock,
	input reset,
	input signed [15:0] Kp,
	input signed [15:0] Kd,
	input signed [15:0] Ki,
	input signed [31:0] sp,
	input signed [15:0] forwardGain,
	input signed [15:0] outputPosMax,
	input signed [15:0] outputNegMax,
	input signed [15:0] IntegralNegMax,
	input signed [15:0] IntegralPosMax,
	input signed [15:0] deadBand,
	input [1:0] control_mode, // position velocity displacement
	input signed [31:0] position,
	input signed [15:0] velocity,
	input wire [15:0] displacement,
	input signed [31:0] outputDivider,
	input update_controller,
	output reg signed [15:0] pwmRef
	);

always @(posedge clock, posedge reset) begin: PID_CONTROLLER_PID_CONTROLLERLOGIC
	reg signed [31:0] integral;
	reg signed [31:0] lastError;
	reg signed [31:0] err;
	reg signed [31:0] pterm;
	reg signed [31:0] dterm;
	reg signed [31:0] ffterm;
	reg signed [14:0] displacement_for_real;
	reg signed [14:0] displacement_offset;
	reg update_controller_prev;
	reg signed [31:0] result;
	
	if (reset == 1) begin
		integral <= 0;
		lastError <= 0;
		err <=0;
		result <= 0;
		update_controller_prev <= 0;
		displacement_for_real = 0;
	end else begin
		update_controller_prev <= update_controller;
		if(update_controller_prev==0 && update_controller==1) begin
			case(control_mode) 
				2'b00: err = (sp - position); 
				2'b01: err = (sp - velocity);
				2'b10: begin
							displacement_for_real = $signed(displacement[14:0]);
							if(displacement_for_real<0) begin
								displacement_offset = displacement_for_real;
							end else begin
								displacement_offset = 0;
							end
							if (sp>0) begin
								err = (sp - (displacement_for_real-displacement_offset));
							end else begin
								err = 0;
							end
						end
				default: err = 0;
			endcase;
			
			if (((err >= deadBand) || (err <= ((-1) * deadBand)))) begin
				pterm = (Kp * err);
				if ((pterm < outputPosMax) || (pterm > outputNegMax)) begin  //if the proportional term is not maxed
					integral = integral + (Ki * err); //add to the integral
					if (integral > IntegralPosMax) begin
						integral = IntegralPosMax;
					end else if (integral < IntegralNegMax) begin
						integral = IntegralNegMax;
					end
				end
				dterm = ((err - lastError) * Kd);
//				ffterm = (forwardGain * sp);
//				result = (((ffterm + pterm) + integral) + dterm)>>>outputDivider;
				result = (pterm + dterm + integral)>>>outputDivider;
				if ((result < outputNegMax)) begin
					 result = outputNegMax;
				end else if ((result > outputPosMax)) begin
					 result = outputPosMax;
				end
			end else begin
				result = integral;
			end
			pwmRef = result;
			lastError = err;
		end
	end 
end


endmodule

