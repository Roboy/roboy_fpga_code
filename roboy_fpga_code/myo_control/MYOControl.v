// MyoControl 
// This module handles the communication and implements PID controller for each myo motor board.
// Communication with the motors is via SPI. The module is accessible via lightweight axi bridge.
// On axi read/write request, the upper 8 bit of the address define which value is accessed, while
// the lower 8 bit define for which motor (if applicable).
// Through the axi bridge, the following values can be READ
//	address            -----   [type] value
// [8'h00 8'h(motor)]         [int16] Kp - gain of PID controller
// [8'h01 8'h(motor)]         [int16] Ki - gain of PID controller
// [8'h02 8'h(motor)]         [int16] Kd - gain of PID controller
// [8'h03 8'h(motor)]         [int32] sp - setpoint of PID controller
// [8'h04 8'h(motor)]         [int16] forwardGain - gain of PID controller
// [8'h05 8'h(motor)]         [int16] outputPosMax - maximal output of PID controller
// [8'h06 8'h(motor)]         [int16] outputNegMax - minimal output of PID controller
// [8'h07 8'h(motor)]         [int16] IntegralPosMax - maximal integral of PID controller
// [8'h08 8'h(motor)]         [int16] IntegralNegMax - minimal integral of PID controller
// [8'h09 8'h(motor)]         [int16] deadBand - deadBand of PID controller
// [8'h0A 8'h(motor)]         [uint8] control_mode - control_mode of PID controller
// [8'h0B 8'h(motor)]         [int32] position - motor position
// [8'h0C 8'h(motor)]         [int16] velocity - motor velocity
// [8'h0D 8'h(motor)]         [int16] current - motor current
// [8'h0E 8'h(motor)]         [int16] displacement - spring displacement
// [8'h0F 8'h(motor)]         [int16] pwmRef - output of PID controller
// [8'h10 8'hz]               [uint32] update_frequency - update frequency between pid an motor board
// [8'h11 8'hz]               [uint32] power_sense_n - power sense pin
// [8'h12 8'hz]               [bool ]gpio_enable - gpio status
// [8'h13 8'h(motor)]         [uin16] angle - myo brick motor angle
// [8'h14 8'hz]               [uint32] myo_brick - myo_brick enable mask
// [8'h15 8'h(motor)]         [uint8] myo_brick_device_id - myo brick i2c device id
// [8'h16 8'h(motor)]         [int32] myo_brick_gear_box_ratio - myo brick gear box ratio
// [8'h17 8'h(motor)]         [int32] myo_brick_encoder_multiplier - myo brick encoder mulitiplier
// [8'h18 8'h(motor)]		   [int32] outputDivider- PID output divider
//
// Through the axi bridge, the following values can be WRITTEN
//	address            -----   [type] value
// [8'h00 8'h(motor)]         [int16] Kp - gain of PID controller
// [8'h01 8'h(motor)]         [int16] Ki - gain of PID controller
// [8'h02 8'h(motor)]         [int16] Kd - gain of PID controller
// [8'h03 8'h(motor)]         [int32] sp - setpoint of PID controller
// [8'h04 8'h(motor)]         [int16] forwardGain - gain of PID controller
// [8'h05 8'h(motor)]         [int16] outputPosMax - maximal output of PID controller
// [8'h06 8'h(motor)]         [int16] outputNegMax - minimal output of PID controller
// [8'h07 8'h(motor)]         [int16] IntegralPosMax - maximal integral of PID controller
// [8'h08 8'h(motor)]         [int16] IntegralNegMax - minimal integral of PID controller
// [8'h09 8'h(motor)]         [int16] deadBand - deadBand of PID controller
// [8'h0A 8'h(motor)]         [uint8] control_mode - control_mode of PID controller
// [8'h0B 8'hz]               [bool] reset_myo_control - reset
// [8'h0C 8'hz]               [bool] spi_activated - toggles spi communication
// [8'h0D 8'h(motor)]         [bool] reset_controller - resets individual PID controller
// [8'h0E 8'hz]               [bool] update_frequency - motor pid update frequency
// [8'h0F 8'hz]               [bool] gpio_enable - controls the gpio 
// [8'h10 8'hz]               [uint32] myo_brick - bit mask for indicating which muscle is a myoBrick
// [8'h11 8'h(motor)]         [uint8] myo_brick_device_id - i2c device id for reading the motor angle
// [8'h12 8'h(motor)]         [int32] myo_brick_gear_box_ratio - myo brick gear box ratio
// [8'h13 8'h(motor)]         [int32] myo_brick_encoder_multiplier - myo brick encoder mulitiplier
// [8'h14 8'h(motor)]         [int32] outputDivider- PID output divider

// Features: 
// * use the NUMBER_OF_MOTORS parameter to define how many motors are connected on one SPI bus (maximum 254)
// * use the update_frequency to define at what rate the motors should be controlled
//   NOTE: The maximal update_frequency is limited by the amount of motors per SPI bus. For 7 motors
//			  on one bus this is for example ~2.8kHz. Setting a higher frequency has no effect.

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

// author: Simon Trendel, simon.trendel@tum.de, 2018/19

`timescale 1ns/10ps

module MYOControl (
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// these are the spi ports
	output [NUMBER_OF_MOTORS-1:0] ss_n_o,
	input miso,
	output mosi,
	output sck,
	input mirrored_muscle_unit,
	input power_sense_n,
	output gpio_n,
	output [NUMBER_OF_MOTORS-1:0] angle_ss_n_o,
	input angle_miso,
	output angle_mosi,
	output angle_sck
);

parameter NUMBER_OF_MOTORS = 6;
parameter CLOCK_SPEED_HZ = 50_000_000;
parameter ENABLE_MYOBRICK_CONTROL = 0;

// gains and shit
// p gains
reg signed [15:0] Kp[NUMBER_OF_MOTORS-1:0];
// i gains
reg signed [15:0] Ki[NUMBER_OF_MOTORS-1:0];
// d gains
reg signed [15:0] Kd[NUMBER_OF_MOTORS-1:0];
// setpoints
reg signed [31:0] sp[NUMBER_OF_MOTORS-1:0];
// forward gains
reg signed [15:0] forwardGain[NUMBER_OF_MOTORS-1:0];
// output positive limits
reg signed [15:0] outputPosMax[NUMBER_OF_MOTORS-1:0];
// output negative limits
reg signed [15:0] outputNegMax[NUMBER_OF_MOTORS-1:0];
// integral negative limits
reg signed [15:0] IntegralNegMax[NUMBER_OF_MOTORS-1:0];
// integral positive limits
reg signed [15:0] IntegralPosMax[NUMBER_OF_MOTORS-1:0];
// deadband
reg signed [15:0] deadBand[NUMBER_OF_MOTORS-1:0];
// control mode
reg [1:0] control_mode[NUMBER_OF_MOTORS-1:0];
// reset pid_controller
reg reset_controller[NUMBER_OF_MOTORS-1:0];
// output divider
reg signed [31:0] outputDivider[NUMBER_OF_MOTORS-1:0];

// pwm output to motors 
wire signed [0:15] pwmRefs[NUMBER_OF_MOTORS-1:0];

// the following is stuff we receive from the motors via spi
// positions of the motors
reg signed [31:0] positions[NUMBER_OF_MOTORS-1:0];
// velocitys of the motors
reg signed [15:0] velocitys[NUMBER_OF_MOTORS-1:0];
// currents of the motors
reg signed [15:0] currents[NUMBER_OF_MOTORS-1:0];
// displacements of the springs
reg [15:0] displacements[NUMBER_OF_MOTORS-1:0];
reg [15:0] displacement_offsets[NUMBER_OF_MOTORS-1:0];

assign readdata = returnvalue;
assign waitrequest = (waitFlag && read) || update_controller;
reg [31:0] returnvalue;
reg waitFlag;

reg [31:0] update_frequency;
reg [31:0] actual_update_frequency;
reg [31:0] delay_counter;

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
				8'h00: returnvalue <= Kp[address[7:0]][15:0];
				8'h01: returnvalue <= Ki[address[7:0]][15:0];
				8'h02: returnvalue <= Kd[address[7:0]][15:0];
				8'h03: returnvalue <= sp[address[7:0]][31:0];
				8'h04: returnvalue <= forwardGain[address[7:0]][15:0];
				8'h05: returnvalue <= outputPosMax[address[7:0]][15:0];
				8'h06: returnvalue <= outputNegMax[address[7:0]][15:0];
				8'h07: returnvalue <= IntegralPosMax[address[7:0]][15:0];
				8'h08: returnvalue <= IntegralNegMax[address[7:0]][15:0];
				8'h09: returnvalue <= deadBand[address[7:0]][15:0];
				8'h0A: returnvalue <= control_mode[address[7:0]][1:0];
				8'h0B: returnvalue <= positions[address[7:0]][31:0];
				8'h0C: returnvalue <= velocitys[address[7:0]][15:0];
				8'h0D: returnvalue <= currents[address[7:0]][15:0];
				8'h0E: returnvalue <= displacements[address[7:0]][15:0];
				8'h0F: returnvalue <= pwmRefs[address[7:0]][0:15];
				8'h10: returnvalue <= actual_update_frequency;
				8'h11: returnvalue <= (power_sense_n==0); // active low
				8'h12: returnvalue <= gpio_enable;
				8'h13: returnvalue <= motor_angle[address[7:0]][31:0];
				8'h14: returnvalue <= myo_brick;
				8'h15: returnvalue <= myo_brick_device_id[address[7:0]][6:0];
				8'h16: returnvalue <= myo_brick_gear_box_ratio[address[7:0]][7:0];
				8'h17: returnvalue <= myo_brick_encoder_multiplier[address[7:0]][31:0];
				8'h18: returnvalue <= outputDivider[address[7:0]][31:0];
				default: returnvalue <= 32'hDEADBEEF;
			endcase
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end
	
reg reset_myo_control;
reg spi_activated;
reg update_controller;
reg start_spi_transmission;
reg gpio_enable;
assign gpio_n = !gpio_enable;
	
reg [7:0] motor;
reg [7:0] pid_update;
reg [31:0] spi_enable_counter;
	
always @(posedge clock, posedge reset) begin: MYO_CONTROL_LOGIC
	reg spi_done_prev; 
	reg [7:0]i;
	reg [31:0] counter;
	reg spi_enable;
	if (reset == 1) begin
		reset_myo_control <= 0;
		spi_activated <= 0;
		motor <= 0;
		spi_done_prev <= 0;
		delay_counter <= 0;
		update_frequency <= 0;
		counter <= 0;
		spi_enable_counter <= 0;
		spi_enable <= 0;
		myo_brick <= 0;
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_reset_controller
			myo_brick_gear_box_ratio[i] <= 62;
			myo_brick_encoder_multiplier[i] <= 1;
			outputDivider[i] <= 1;
		end
	end else begin
		// toggle registers need to be set to zero at every clock cycle
		update_controller <= 0;
		start_spi_transmission <= 0;
		reset_myo_control <= 0;
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_reset_controller
			reset_controller[i] <= 0;
		end
		// for rising edge detection of spi done
		spi_done_prev <= spi_done;
		
		// increment counter, will be used to calculate actual update frequency
		counter <= counter + 1;
		
		// when spi is done, latch the received values for the current motor and toggle PID controller update of previous motor
		if(spi_done_prev==0 && spi_done) begin
			positions[motor][31:0] <= position[0:31];
			velocitys[motor][15:0] <= velocity[0:15];
			currents[motor][15:0] <= current[0:15];
			if(~myo_brick[motor]) begin
				if(mirrored_muscle_unit && motor<9) begin 
					displacements[motor][15:0] <= (-1)*displacement[0:15]; 
				end else begin
					displacements[motor][15:0] <= displacement[0:15];
				end
			end else begin
				displacements[motor][15:0] <= motor_spring_angle[motor];
			end
			if(motor==0) begin // lazy update (we are updating the controller following the current spi transmission)
				pid_update <= NUMBER_OF_MOTORS-1; 
			end else begin
				pid_update <= motor-1;
			end
			update_controller <= 1; 
		end
		
		// if a frequency is requested, a delay counter makes sure the next motor cycle will be delayed accordingly
		if(update_frequency>0) begin
			if(spi_done_prev==0 && spi_done) begin
				if(motor<(NUMBER_OF_MOTORS-1)) begin
					motor <= motor + 1;
					start_spi_transmission <= 1;
				end
			end			
			if(delay_counter>0) begin
				delay_counter <= delay_counter-1;
			end else begin
				if(spi_done && motor>=(NUMBER_OF_MOTORS-1)) begin
					motor <= 0;
					delay_counter <= CLOCK_SPEED_HZ/update_frequency;
					actual_update_frequency <= CLOCK_SPEED_HZ/counter;
					counter <= 0;
					start_spi_transmission <= 1; 
				end
			end
		end else begin
			// update as fast as possible
			if(spi_done_prev==0 && spi_done) begin
				start_spi_transmission <= 1;
				if(motor<NUMBER_OF_MOTORS-1) begin
					motor <= motor + 1;
				end else begin
					motor <= 0;
					actual_update_frequency <= CLOCK_SPEED_HZ/counter;
					counter <= 0;
				end
			end
		end
	
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if((address>>8)<=8'h14 && address[7:0]<NUMBER_OF_MOTORS) begin
				case(address>>8)
					8'h00: Kp[address[7:0]][15:0] <= writedata[15:0];
					8'h01: Ki[address[7:0]][15:0] <= writedata[15:0];
					8'h02: Kd[address[7:0]][15:0] <= writedata[15:0];
					8'h03: sp[address[7:0]][31:0] <= writedata[31:0];
					8'h04: forwardGain[address[7:0]][15:0] <= writedata[15:0];
					8'h05: outputPosMax[address[7:0]][15:0] <= writedata[15:0];
					8'h06: outputNegMax[address[7:0]][15:0] <= writedata[15:0];
					8'h07: IntegralPosMax[address[7:0]][15:0] <= writedata[15:0];
					8'h08: IntegralNegMax[address[7:0]][15:0] <= writedata[15:0];
					8'h09: deadBand[address[7:0]][15:0] <= writedata[15:0];
					8'h0A: control_mode[address[7:0]][1:0] <= writedata[1:0];
					8'h0B: reset_myo_control <= (writedata!=0);
					8'h0C: spi_activated <= (writedata!=0);
					8'h0D: reset_controller[address[7:0]] <= (writedata!=0);
					8'h0E: update_frequency <= writedata;
					8'h0F: gpio_enable <= (writedata!=0);
					8'h10: myo_brick <= writedata;
					8'h11: myo_brick_device_id[address[7:0]][6:0] <= writedata[6:0];
					8'h12: myo_brick_gear_box_ratio[address[7:0]][31:0] <= writedata[31:0];
					8'h13: myo_brick_encoder_multiplier[address[7:0]][31:0] <= writedata[31:0];
					8'h14: outputDivider[address[7:0]][31:0] <= writedata[31:0];
				endcase
			end
		end
		
		if(power_sense_n==0) begin // if power on and delay not reached yet, we count
			if(spi_enable_counter<150000000) begin 
				spi_enable_counter <= spi_enable_counter + 1;
				reset_myo_control <= 1;
			end else begin
				reset_myo_control <= 0;
				spi_activated <= 1;
			end
		end else begin 
			spi_enable_counter <= 0;
			reset_myo_control <= 1;
		end
	end 
end

reg [NUMBER_OF_MOTORS-1:0] myo_brick;
reg [6:0] myo_brick_device_id[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] myo_brick_gear_box_ratio[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] myo_brick_encoder_multiplier[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] motor_spring_angle[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] motor_angle[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] motor_angle_offset[NUMBER_OF_MOTORS-1:0];
reg [31:0] status[NUMBER_OF_MOTORS-1:0];
reg [NUMBER_OF_MOTORS-1:0] myo_brick_ack_error;

generate
	if(ENABLE_MYOBRICK_CONTROL!=0) begin
		integer angle_motor_index;
		wire [11:0] angle;
		wire signed [31:0] angle_signed;
		assign angle_signed = angle;
		reg ack_error;

		always @(posedge clock, posedge reset) begin: MYOBRICK_ANGLE_CONTROL_LOGIC
			reg [11:0] motor_angle_prev[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] motor_angle_counter[NUMBER_OF_MOTORS-1:0];
			reg [7:0]i;
			if (reset == 1) begin
				angle_motor_index <= 0;
				for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_angle_counter
					motor_angle_counter[i] <= 0;
				end
			end else begin
				// the angle sensor has no internal rotation counter, therefore we gotta count over-/underflow on ower own
				if(power_sense_n) begin // if power sense is off (high), reset the overflow counters
					motor_angle_counter[angle_motor_index] <= 0;
					motor_angle_offset[angle_motor_index] <= angle_signed;
				end else begin
					if(motor_angle_prev[angle_motor_index]>3500 && angle_signed < 500) begin
						motor_angle_counter[angle_motor_index] <= motor_angle_counter[angle_motor_index] + 1;
					end
					if(motor_angle_prev[angle_motor_index]<500 && angle_signed > 3500) begin
						motor_angle_counter[angle_motor_index] <= motor_angle_counter[angle_motor_index] - 1;
					end
				end
				// motor_angle_offset is set to the angle after power on of the motor boards
				motor_angle[angle_motor_index] <= (angle_signed - motor_angle_offset[angle_motor_index] + motor_angle_counter[angle_motor_index]*4096); 
				// division by gearbox ration gives encoder ticks, angle sensor divided by 4 gives the same range
				motor_spring_angle[angle_motor_index] <= (positions[angle_motor_index]/myo_brick_gear_box_ratio[angle_motor_index])*myo_brick_encoder_multiplier[angle_motor_index] 
																		- ((angle_signed - motor_angle_offset[angle_motor_index] + motor_angle_counter[angle_motor_index]*4096)/4);
				motor_angle_prev[angle_motor_index] <= angle_signed;
				if(angle_motor_index<NUMBER_OF_MOTORS-1) begin
					angle_motor_index <= angle_motor_index + 1;
				end else begin
					angle_motor_index <= 0;
				end
			end 
		end
		
		A1339Control#(NUMBER_OF_MOTORS) a1339(
			.clock(clock),
			.reset_n(~reset),
			.sensor_angle(angle),
			.sensor(angle_motor_index),
			// SPI
			.sck_o(angle_sck), // clock
			.ss_n_o(angle_ss_n_o), // slave select line for each sensor
			.mosi_o(angle_mosi),	// mosi
			.miso_i(angle_miso)	// miso
		);
	end

endgenerate 

wire di_req, wr_ack, do_valid, wren, spi_done, ss_n;
wire [0:15] Word;
wire [15:0] data_out;
wire signed [0:15] pwmRef;
wire signed [0:31] position; 
wire signed [0:15] velocity;
wire signed [0:15] current;
wire [0:15] displacement;
wire signed [0:15] sensor1;
wire signed [0:15] sensor2;

// the pwmRef signal is wired to the active motor pid controller output
assign pwmRef = pwmRefs[motor];

// control logic for handling myocontrol frame
SpiControl spi_control(
	.clock(clock),
	.reset(reset_myo_control),
	.di_req(di_req),
	.write_ack(wr_ack),
	.data_read_valid(do_valid),
	.data_read(data_out[15:0]),
	.start(spi_activated && start_spi_transmission),
	.Word(Word[0:15]),
	.wren(wren),
	.spi_done(spi_done),
	.pwmRef(pwmRef),
	.position(position),
	.velocity(velocity),
	.current(current),
	.displacement(displacement),
	.sensor1(sensor1),
	.sensor2(sensor2),
	.ss_n(ss_n)
);

// SPI specs: 2MHz, 16bit MSB, clock phase of 1
spi_master #(16, 1'b0, 1'b1, 2, 5) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(reset_myo_control),
	.spi_miso_i(miso),
	.di_i(Word[0:15]),
	.wren_i(wren),
	.spi_ssel_o(ss_n),
	.spi_sck_o(sck),
	.spi_mosi_o(mosi),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_out[15:0])
);

// PID controller for NUMBER_OF_MOTORS
genvar j;
generate 
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : instantiate_pid_controllers
	  PIDController pid_controller(
			.clock(clock),
			.reset(reset_myo_control||reset_controller[j]),
			.Kp(Kp[j]),
			.Kd(Kd[j]),
			.Ki(Ki[j]),
			.sp(sp[j]),
			.forwardGain(forwardGain[j]),
			.outputPosMax(outputPosMax[j]),
			.outputNegMax(outputNegMax[j]),
			.IntegralNegMax(IntegralNegMax[j]),
			.IntegralPosMax(IntegralPosMax[j]),
			.deadBand(deadBand[j]),
			.control_mode(control_mode[j]), // position velocity displacement
			.position(positions[j]),
			.velocity(velocitys[j]),
			.displacement(displacements[j]),
			.outputDivider(outputDivider[j]),
			.update_controller(pid_update==j && update_controller),
			.pwmRef(pwmRefs[j])
		);
		assign ss_n_o[j] = (motor==j?ss_n:1);
	end
endgenerate 


endmodule

