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

module MSJPlatformControl (
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	output [NUMBER_OF_MOTORS-1:0] angle_ss_n_o,
	input angle_miso,
	output angle_mosi,
	output angle_sck,
	input emergency_off,
	output [NUMBER_OF_MOTORS-1:0]PWM
);

parameter NUMBER_OF_MOTORS = 6;
parameter CLOCK_SPEED_HZ = 50_000_000;
parameter SAMPLES_TO_AVERAGE = 256;

// gains and shit
// p gains
reg signed [31:0] Kp[NUMBER_OF_MOTORS-1:0];
// d gains
reg signed [31:0] Kd[NUMBER_OF_MOTORS-1:0];
// setpoints
reg signed [31:0] sp[NUMBER_OF_MOTORS-1:0];
// positions
reg signed [31:0] position[NUMBER_OF_MOTORS-1:0];
// velocities
reg signed [31:0] velocity[NUMBER_OF_MOTORS-1:0];
// dutys
reg signed [31:0] dutys[NUMBER_OF_MOTORS-1:0];
// dutys
reg signed [31:0] outputPosMax[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] outputNegMax[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] deadBand[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] zero_speed[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] outputDivider[NUMBER_OF_MOTORS-1:0];
// control mode
reg [1:0] control_mode[NUMBER_OF_MOTORS-1:0];

assign readdata = returnvalue;
assign waitrequest = (waitFlag && read);
reg [31:0] returnvalue;
reg waitFlag;

A1339Interface #(.NUMBER_OF_SENSORS(NUMBER_OF_MOTORS)) a1339_interface();
assign angle_sck = a1339_interface.sck_o;
assign angle_ss_n_o = a1339_interface.ss_n_o;
assign angle_mosi = a1339_interface.mosi_o;
assign a1339_interface.miso_i = angle_miso;
assign a1339_interface.zero_offset = emergency_off||reset_control;
			
A1339Control#(CLOCK_SPEED_HZ,NUMBER_OF_MOTORS,SAMPLES_TO_AVERAGE) a1339(
	.clock(clock),
	.reset_n(~reset), 
	.interf(a1339_interface.child)
);

// the following iterface handles read requests via lightweight axi bridge
// the upper 8 bit of the read address define which value we want to read
// the lower 8 bit of the read address define for which motor
always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	if (reset == 1) begin
		waitFlag <= 1;
	end else begin
		waitFlag <= 1;
		if(read) begin
			case(address>>8)
				8'h00: returnvalue <= Kp[address[7:0]][31:0];
				8'h01: returnvalue <= Kd[address[7:0]][31:0];
				8'h02: returnvalue <= sp[address[7:0]][31:0];
				8'h03: returnvalue <= control_mode[address[7:0]][1:0];
				8'h04: returnvalue <= outputPosMax[address[7:0]][31:0];
				8'h05: returnvalue <= outputNegMax[address[7:0]][31:0];
				8'h06: returnvalue <= deadBand[address[7:0]][31:0];
				8'h07: returnvalue <= outputDivider[address[7:0]][31:0];
				8'h08: returnvalue <= a1339_interface.sensor_angle[address[7:0]][31:0];
				8'h09: returnvalue <= a1339_interface.sensor_angle_absolute[address[7:0]][31:0];
				8'h0A: returnvalue <= a1339_interface.sensor_angle_offset[address[7:0]][31:0];
				8'h0B: returnvalue <= a1339_interface.sensor_angle_relative[address[7:0]][31:0];
				8'h0C: returnvalue <= a1339_interface.sensor_angle_velocity[address[7:0]][31:0];
				8'h0D: returnvalue <= a1339_interface.sensor_revolution_counter[address[7:0]][31:0];
				8'h0E: returnvalue <= dutys[address[7:0]][31:0];
				8'h0F: returnvalue <= zero_speed[address[7:0]][31:0];
				default: returnvalue <= 32'hDEADBEEF;
			endcase
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end
	
reg reset_control;
reg mute;
reg update_controller;
	
always @(posedge clock, posedge reset) begin: WRITE_CONTROL_LOGIC
	reg [7:0]i;
	if (reset == 1) begin
		reset_control <= 0;
		mute <= 0;
		for(i=0;i<NUMBER_OF_MOTORS;i=i+1)begin
			Kp[i] <= 20;
			Kd[i] <= 10;
			outputDivider[i] <= 6;
			outputPosMax[i] <= 330;
			outputNegMax[i] <= 300;
			zero_speed[i] <= 315;
			deadBand [i] <= 0;
			control_mode[i] <= 2;
			sp[i] <= 0;
		end
	end else begin
		// toggle registers need to be set to zero at every clock cycle
		update_controller <= 0;
		reset_control <= 0;
	
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if((address>>8)<=8'h0A && address[7:0]<NUMBER_OF_MOTORS) begin
				case(address>>8)
					8'h00: Kp[address[7:0]][31:0] <= writedata[31:0];
					8'h01: Kd[address[7:0]][31:0] <= writedata[31:0];
					8'h02: sp[address[7:0]][31:0] <= writedata[31:0];
					8'h03: control_mode[address[7:0]][1:0] <= writedata[1:0];
					8'h04: reset_control<= (writedata!=0);
					8'h05: outputDivider[address[7:0]][31:0]<= writedata;
					8'h06: outputPosMax[address[7:0]][31:0]<= writedata;
					8'h07: outputNegMax[address[7:0]][31:0]<= writedata;
					8'h08: deadBand[address[7:0]][31:0]<= writedata;
					8'h09: zero_speed[address[7:0]][31:0]<= writedata;
					8'h0A: mute <= (writedata!=0);
				endcase
			end
		end
	end 
end
	
genvar j;
generate
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : instantiate_pid_controllers
	  MSJPlatformPDController pd_controller(
			.clock(clock),
			.reset(reset_control),
			.Kp(Kp[j]),
			.Kd(Kd[j]),
			.sp(sp[j]),
			.outputPosMax(outputPosMax[j]),
			.outputNegMax(outputNegMax[j]),
			.deadBand(deadBand[j]),
			.zero_speed(zero_speed[j]),
			.control_mode(control_mode[j]), // position velocity 
			.position(a1339_interface.sensor_angle_absolute[j]),
			.velocity(a1339_interface.sensor_angle_velocity[j]),
			.outputDivider(outputDivider[j]),
			.update_controller(a1339_interface.cycle[j]),
			.duty(dutys[j])
		);
	end
	
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : instantiate_pwm_controllers
	  pwm #(CLOCK_SPEED_HZ, 50, CLOCK_SPEED_HZ, 12, 1)  motor(
			.clk(clock), 					//system clock
			.reset_n(~reset),				//asynchronous reset
			.ena(a1339_interface.cycle[j]),					//latches in new duty cycle
			.duty(dutys[j]),					//duty cycle
			.pwm_out(PWM[j]&(!mute))				//pwm outputs
		);
	end
	
endgenerate 

endmodule

