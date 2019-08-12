// MyoControl 
// This module handles the communication and implements PID controller for each myo motor board.
// Communication with the motors is via SPI. The module is accessible via lightweight axi bridge.
// On axi read/write request, the upper 8 bit of the address define which value is accessed, while
// the lower 8 bit define for which motor (if applicable).
// Through the axi bridge, the following values can be READ
//	address            -----   [type] value
// [8'h00 8'h(motor)]         [int32] Kp - gain of PID controller
// [8'h01 8'h(motor)]         [int32] Ki - gain of PID controller
// [8'h02 8'h(motor)]         [int32] Kd - gain of PID controller
// [8'h03 8'h(motor)]         [int32] sp - setpoint of PID controller
// [8'h04 8'h(motor)]         [int32] forwardGain - gain of PID controller
// [8'h05 8'h(motor)]         [int32] outputPosMax - maximal output of PID controller
// [8'h06 8'h(motor)]         [int32] outputNegMax - minimal output of PID controller
// [8'h07 8'h(motor)]         [int32] IntegralPosMax - maximal integral of PID controller
// [8'h08 8'h(motor)]         [int32] IntegralNegMax - minimal integral of PID controller
// [8'h09 8'h(motor)]         [int32] deadBand - deadBand of PID controller
// [8'h0A 8'h(motor)]         [uint8] control_mode - control_mode of PID controller
// [8'h0B 8'h(motor)]         [int32] position - motor position
// [8'h0C 8'h(motor)]         [int16] velocity - motor velocity
// [8'h0D 8'h(motor)]         [int16] current - motor current
// [8'h0E 8'h(motor)]         [int32] displacement - spring displacement
// [8'h0F 8'h(motor)]         [int16] pwmRef - output of PID controller
// [8'h10 8'hz]               [uint32] update_frequency - update frequency between pid an motor board
// [8'h11 8'hz]               [uint32] power_sense_n - power sense pin
// [8'h12 8'hz]               [bool ]gpio_enable - gpio status
// [8'h13 8'h(motor)]         [uin16] angle - myo brick motor angle
// [8'h14 8'hz]               [uint32] myo_brick - myo_brick enable mask
// [8'h15 8'h(motor)]         [int32] myo_brick_gear_box_ratio - myo brick gear box ratio
// [8'h16 8'h(motor)]         [int32] myo_brick_encoder_multiplier - myo brick encoder mulitiplier
// [8'h17 8'h(motor)]		   [int32] outputShifter- PID output shifter, to scale PID output values (division by multiples of 2)
//
// Through the axi bridge, the following values can be WRITTEN
//	address            -----   [type] value
// [8'h00 8'h(motor)]         [int32] Kp - gain of PID controller
// [8'h01 8'h(motor)]         [int32] Ki - gain of PID controller
// [8'h02 8'h(motor)]         [int32] Kd - gain of PID controller
// [8'h03 8'h(motor)]         [int32] sp - setpoint of PID controller
// [8'h04 8'h(motor)]         [int32] forwardGain - gain of PID controller
// [8'h05 8'h(motor)]         [int32] outputPosMax - maximal output of PID controller
// [8'h06 8'h(motor)]         [int32] outputNegMax - minimal output of PID controller
// [8'h07 8'h(motor)]         [int32] IntegralPosMax - maximal integral of PID controller
// [8'h08 8'h(motor)]         [int32] IntegralNegMax - minimal integral of PID controller
// [8'h09 8'h(motor)]         [int32] deadBand - deadBand of PID controller
// [8'h0A 8'h(motor)]         [uint8] control_mode - control_mode of PID controller
// [8'h0B 8'hz]               [bool] reset_myo_control - reset
// [8'h0C 8'hz]               [bool] spi_activated - toggles spi communication
// [8'h0D 8'h(motor)]         [bool] reset_controller - resets individual PID controller
// [8'h0E 8'hz]               [bool] update_frequency - motor pid update frequency
// [8'h0F 8'hz]               [bool] gpio_enable - controls the gpio 
// [8'h10 8'hz]               [uint32] myo_brick - bit mask for indicating which muscle is a myoBrick
// [8'h12 8'h(motor)]         [int32] myo_brick_gear_box_ratio - myo brick gear box ratio
// [8'h13 8'h(motor)]         [int32] myo_brick_encoder_multiplier - myo brick encoder mulitiplier
// [8'h14 8'h(motor)]         [int32] outputShifter- PID output shifter, to scale PID output values (division by multiples of 2)

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
	MyoControlInterface interf
)/* synthesis noprune */;

parameter NUMBER_OF_MOTORS = 6;
parameter CLOCK_SPEED_HZ = 50_000_000;
parameter ENABLE_MYOBRICK_CONTROL = 0;

// gains and shit
// p gains
reg [31:0] Kp_f[NUMBER_OF_MOTORS-1:0];
// i gains
reg [31:0] Ki_f[NUMBER_OF_MOTORS-1:0];
// d gains
reg [31:0] Kd_f[NUMBER_OF_MOTORS-1:0];
// setpoints
reg [31:0] sp_f[NUMBER_OF_MOTORS-1:0];
// output limits
reg signed [31:0] outputLimit[NUMBER_OF_MOTORS-1:0];
// control mode
reg [2:0] control_mode[NUMBER_OF_MOTORS-1:0];

// pwm output to motors 
reg signed [15:0] pwmRefs[NUMBER_OF_MOTORS-1:0];

// the following is stuff we receive from the motors via spi
// positions of the motors
reg signed [31:0] positions[NUMBER_OF_MOTORS-1:0];
// velocitys of the motors
reg signed [15:0] velocities[NUMBER_OF_MOTORS-1:0];
// currents of the motors
reg signed [15:0] currents[NUMBER_OF_MOTORS-1:0];
// displacements of the springs
reg signed [31:0] displacements[NUMBER_OF_MOTORS-1:0];
// controlflags for each motor
reg [15:0] controlFlags[NUMBER_OF_MOTORS-1:0];

// raw motor states in float
reg [31:0] positions_raw_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] velocities_raw_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_raw_f[NUMBER_OF_MOTORS-1:0];

// converted motor states in float
reg [31:0] positions_conv_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] velocities_conv_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_conv_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_myo_brick_conv_f[NUMBER_OF_MOTORS-1:0];

// setpoint errors of the motors in float
reg [31:0] positions_err_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] velocities_err_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_err_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_myo_brick_err_f[NUMBER_OF_MOTORS-1:0];

// results of the PD controllers of the motors in float
reg [31:0] positions_res_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] velocities_res_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_res_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] displacements_myo_brick_res_f[NUMBER_OF_MOTORS-1:0];

// results of the PD controllers of the motors in int
reg signed [31:0] positions_res[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] velocities_res[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] displacements_res[NUMBER_OF_MOTORS-1:0];
reg signed [31:0] displacements_myo_brick_res[NUMBER_OF_MOTORS-1:0];

// encoder multipliers in float
reg [31:0] pos_encoder_multiplier_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] dis_encoder_multiplier_f[NUMBER_OF_MOTORS-1:0];

// freaky freaky
reg [31:0] update_frequency;
reg [31:0] actual_update_frequency;
reg [31:0] delay_counter;

// interface connections
genvar k;
generate
for(k=0;k<NUMBER_OF_MOTORS;k=k+1) begin : assign_interfaces
	assign interf.Kp_f[k] = Kp_f[k];
	assign interf.Ki_f[k] = Ki_f[k];
	assign interf.Kd_f[k] = Kd_f[k];
	assign interf.sp_f[k] = sp_f[k];
	assign interf.outputLimit[k] = outputLimit[k];
	assign interf.control_mode[k] = control_mode[k];
	assign interf.controlFlags[k] = controlFlags[k];
	assign interf.pwmRefs[k] = pwmRefs[k];
	assign interf.positions[k] = positions[k];
	assign interf.velocities[k] = velocities[k];
	assign interf.currents[k] = currents[k];
	assign interf.displacements[k] = displacements[k];
	assign interf.positions_raw_f[k] = positions_raw_f[k];
	assign interf.velocities_raw_f[k] = velocities_raw_f[k];
	assign interf.displacements_raw_f[k] = displacements_raw_f[k];
	assign interf.positions_conv_f[k] = positions_conv_f[k];
	assign interf.velocities_conv_f[k] = velocities_conv_f[k];
	assign interf.displacements_conv_f[k] = displacements_conv_f[k];
	assign interf.displacements_myo_brick_conv_f[k] = displacements_myo_brick_conv_f[k];
	assign interf.positions_err_f[k] = positions_err_f[k];
	assign interf.velocities_err_f[k] = velocities_err_f[k];
	assign interf.displacements_err_f[k] = displacements_err_f[k];
	assign interf.displacements_myo_brick_conv_f[k] = displacements_myo_brick_conv_f[k];
	assign interf.positions_res_f[k] = positions_res_f[k];
	assign interf.velocities_res_f[k] = velocities_res_f[k];
	assign interf.displacements_res_f[k] = displacements_res_f[k];
	assign interf.displacements_myo_brick_res_f[k] = displacements_myo_brick_res_f[k];
	assign interf.positions_res[k] = positions_res[k];
	assign interf.velocities_res[k] = velocities_res[k];
	assign interf.displacements_res[k] = displacements_res[k];
	assign interf.displacements_myo_brick_res[k] = displacements_myo_brick_res[k];
	assign interf.positions_res[k] = positions_res[k];
	assign interf.velocities_res[k] = velocities_res[k];
	assign interf.displacements_res[k] = displacements_res[k];
	assign interf.displacements_myo_brick_res[k] = displacements_myo_brick_res[k];
	assign interf.pos_encoder_multiplier_f[k] = pos_encoder_multiplier_f[k];
	assign interf.dis_encoder_multiplier_f[k] = dis_encoder_multiplier_f[k];
	assign interf.update_frequency[k] = update_frequency[k];
	assign interf.actual_update_frequency[k] = actual_update_frequency[k];
end
endgenerate

assign readdata = returnvalue;
assign waitrequest = (waitFlag && read);
reg [31:0] returnvalue;
reg waitFlag;

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
				8'h00: returnvalue <= Kp_f[address[7:0]];
				8'h01: returnvalue <= Ki_f[address[7:0]];
				8'h02: returnvalue <= Kd_f[address[7:0]];
				
				8'h03: returnvalue <= sp_f[address[7:0]];
				8'h04: returnvalue <= outputLimit[address[7:0]];
				8'h05: returnvalue <= control_mode[address[7:0]][2:0];
				
				8'h06: returnvalue <= positions[address[7:0]];
				8'h07: returnvalue <= velocities[address[7:0]];
				8'h08: returnvalue <= currents[address[7:0]];
				8'h09: returnvalue <= displacements[address[7:0]];
				
				8'h0A: returnvalue <= pwmRefs[address[7:0]];
				8'h0B: returnvalue <= actual_update_frequency;
				8'h0C: returnvalue <= (power_sense_n==0); // active low
				
				8'h0D: returnvalue <= positions_raw_f[address[7:0]];
				8'h0E: returnvalue <= velocities_raw_f[address[7:0]];
				8'h0F: returnvalue <= displacements_raw_f[address[7:0]];
				
				8'h10: returnvalue <= positions_conv_f[address[7:0]];
				8'h11: returnvalue <= velocities_conv_f[address[7:0]];
				8'h12: returnvalue <= displacements_conv_f[address[7:0]];
				8'h13: returnvalue <= displacements_myo_brick_conv_f[address[7:0]];
				
				8'h14: returnvalue <= positions_err_f[address[7:0]];
				8'h15: returnvalue <= velocities_err_f[address[7:0]];
				8'h16: returnvalue <= displacements_err_f[address[7:0]];
				8'h17: returnvalue <= displacements_myo_brick_err_f[address[7:0]];
				
				8'h18: returnvalue <= positions_res_f[address[7:0]];
				8'h19: returnvalue <= velocities_res_f[address[7:0]];
				8'h1A: returnvalue <= displacements_res_f[address[7:0]];
				8'h1B: returnvalue <= displacements_myo_brick_res_f[address[7:0]];
				
				8'h1C: returnvalue <= positions_res[address[7:0]];
				8'h1D: returnvalue <= velocities_res[address[7:0]];
				8'h1E: returnvalue <= displacements_res[address[7:0]];
				8'h1F: returnvalue <= displacements_myo_brick_res[address[7:0]];
				
				8'h20: returnvalue <= pos_encoder_multiplier_f[address[7:0]];
				8'h21: returnvalue <= dis_encoder_multiplier_f[address[7:0]];
				8'h22: returnvalue <= controlFlags[address[7:0]];
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
reg start_spi_transmission;
	
reg [7:0] motor;
reg [31:0] spi_enable_counter;

localparam POSITION=0, VELOCITY=1, DISPLACEMENT=2, MYODIS=3;
	
always @(posedge clock, posedge reset) begin: MYO_CONTROL_LOGIC
	reg spi_done_prev; 
	reg [7:0]i;
	reg [31:0] counter;
	reg spi_enable;
	reg next_motor;
	reg pos_result2ready_prev, vel_result2ready_prev, dis_result2ready_prev, myo_result2ready_prev;
	if (reset == 1) begin
		reset_myo_control <= 0;
		spi_activated <= 1;
		motor <= 0;
		spi_done_prev <= 0;
		delay_counter <= 0;
		update_frequency <= 0;
		counter <= 0;
		spi_enable_counter <= 0;
		spi_enable <= 0;
	end else begin
		// toggle registers need to be set to zero at every clock cycle
		start_spi_transmission <= 0;
		next_motor <= 0;
		reset_myo_control <= 0;
		// for rising edge detection of spi done
		spi_done_prev <= spi_done;
		
		// increment counter, will be used to calculate actual update frequency
		counter <= counter + 1;
		
		// when spi is done, latch the received values for the current motor and toggle PID controller update of the motor
		if(spi_done_prev==0 && spi_done) begin
			positions[motor] <= position[31:0];
			velocities[motor] <= velocity[15:0];
			currents[motor] <= current[15:0];
			
			if(mirrored_muscle_unit) begin 
				displacements[motor] <= -displacement;
			end else begin
				displacements[motor] <= displacement;
			end
		end
		
		pos_result2ready_prev <= pos_result2ready;
		vel_result2ready_prev <= vel_result2ready;
		dis_result2ready_prev <= dis_result2ready;
		myo_result2ready_prev <= myo_result2ready;
		
		case (control_mode[motor])
			POSITION: begin
				if(pos_result2ready_prev==0 && pos_result2ready==1) begin // pos result is ready
					positions_raw_f[motor] <= pos_raw_f;
					positions_conv_f[motor] <= pos_conv_f;
					positions_err_f[motor] <= p_pos_err_f;
					positions_res_f[motor] <= pos_res_f;
					positions_res[motor] <= pos_res;
					pos_err_prev_f[motor] <= p_pos_err_f;
					pwmRefs[motor] <= positions_res[motor];
					next_motor <= 1;
				end
			end
			VELOCITY: begin
				if(vel_result2ready_prev==0 && vel_result2ready==1) begin // vel result is ready
					velocities_raw_f[motor] <= vel_raw_f;
					velocities_conv_f[motor] <= vel_conv_f;
					velocities_err_f[motor] <= p_vel_err_f;
					velocities_res_f[motor] <= vel_res_f;
					velocities_res[motor] <= vel_res;
					vel_err_prev_f[motor] <= p_vel_err_f;
					pwmRefs[motor] <= velocities_res[motor];
					next_motor <= 1;
				end
			end
			DISPLACEMENT: begin
				if(dis_result2ready_prev==0 && dis_result2ready==1) begin // vel result is ready
					displacements_raw_f[motor] <= dis_raw_f;
					displacements_conv_f[motor] <= dis_conv_f;
					displacements_err_f[motor] <= p_dis_err_f;
					displacements_res_f[motor] <= dis_res_f;
					displacements_res[motor] <= dis_res;
					dis_err_prev_f[motor] <= p_dis_err_f;
					pwmRefs[motor] <= positions_res[motor];
					next_motor <= 1;
				end
			end
			MYODIS: begin
				if(myo_result2ready_prev==0 && myo_result2ready==1) begin // myo result is ready
					displacements_myo_brick_conv_f[motor] <= myo_brick_dis_f;
					displacements_myo_brick_err_f[motor] <= p_dis_myo_brick_err_f;
					displacements_myo_brick_res_f[motor] <= dis_myo_brick_res_f;
					displacements_myo_brick_res[motor] <= dis_myo_brick_res;
					dis_myo_brick_err_prev_f[motor] <= p_dis_myo_brick_err_f;
					pwmRefs[motor] <= displacements_myo_brick_res[motor];
					next_motor <= 1;
				end
			end
		endcase;
		
		
		// if a frequency is requested, a delay counter makes sure the next motor cycle will be delayed accordingly
		if(update_frequency>0) begin
			if(next_motor) begin
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
			if(next_motor) begin
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
			if((address>>8)<=8'h13 && address[7:0]<NUMBER_OF_MOTORS) begin
				case(address>>8)
					8'h00: Kp_f[address[7:0]] <= writedata;
					8'h01: Ki_f[address[7:0]] <= writedata;
					8'h02: Kd_f[address[7:0]] <= writedata;
					8'h03: sp_f[address[7:0]] <= writedata;
					8'h04: outputLimit[address[7:0]] <= writedata;
					8'h05: control_mode[address[7:0]][2:0] <= writedata[2:0];
					8'h06: reset_myo_control <= (writedata!=0);
					8'h07: spi_activated <= (writedata!=0);
					8'h09: update_frequency <= writedata;
					8'h1A: pos_encoder_multiplier_f[address[7:0]] <= writedata;
					8'h1B: dis_encoder_multiplier_f[address[7:0]] <= writedata;
					9'h1C: controlFlags[address[7:0]] <= writedata;
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

wire di_req, wr_ack, do_valid, wren, spi_done, ss_n;
wire [15:0] Word;
wire [15:0] data_out;
wire [15:0] controlFlag;
wire signed [15:0] pwmRef;
wire signed [31:0] position; 
wire signed [15:0] velocity;
wire signed [15:0] current;
wire signed [31:0] displacement;
wire signed [15:0] sensor1;

// the pwmRef signal is wired to the active motor pid controller output
assign controlFlag = controlFlags[motor];
assign pwmRef = pwmRefs[motor];
genvar j;
generate 
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : slave_select_lines
		assign ss_n_o[j] = (motor==j?ss_n:1);
	end
endgenerate 

// control logic for handling myocontrol frame
SpiControl spi_control(
	.clock(clock),
	.reset(reset_myo_control),
	.di_req(di_req),
	.write_ack(wr_ack),
	.data_read_valid(do_valid),
	.data_read(data_out[15:0]),
	.start(spi_activated && start_spi_transmission),
	.Word(Word[15:0]),
	.wren(wren),
	.spi_done(spi_done),
	.pwmRef(pwmRef),
	.controlFlag(controlFlag),
	.position(position),
	.velocity(velocity),
	.current(current),
	.displacement(displacement),
	.sensor1(sensor1),
	.ss_n(ss_n)
);

// SPI specs: 2MHz, 16bit MSB, clock phase of 1
spi_master #(16, 1'b0, 1'b1, 2, 5) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(reset_myo_control),
	.spi_miso_i(miso),
	.di_i(Word[15:0]),
	.wren_i(wren),
	.spi_ssel_o(ss_n),
	.spi_sck_o(sck),
	.spi_mosi_o(mosi),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_out[15:0])
);

localparam ADD = 0;
localparam SUB = 1;
localparam MUL = 2;
localparam DIV = 3;
localparam INT2FLO = 4;
localparam FLO2INT = 5;

wire [31:0] pos_raw_f;
wire [31:0] vel_raw_f;
wire [31:0] dis_raw_f;

wire [31:0] pos_conv_f;
wire [31:0] vel_conv_f;
wire [31:0] dis_conv_f;
wire [31:0] myo_brick_dis_f;

wire [31:0] p_pos_err_f;
wire [31:0] p_vel_err_f;
wire [31:0] p_dis_err_f;
wire [31:0] p_dis_myo_brick_err_f;

wire [31:0] p_pos_res_f;
wire [31:0] p_vel_res_f;
wire [31:0] p_dis_res_f;
wire [31:0] p_dis_myo_brick_res_f;

wire [31:0] d_pos_err_f;
wire [31:0] d_vel_err_f;
wire [31:0] d_dis_err_f;
wire [31:0] d_dis_myo_brick_err_f;

wire [31:0] d_pos_res_f;
wire [31:0] d_vel_res_f;
wire [31:0] d_dis_res_f;
wire [31:0] d_dis_myo_brick_res_f;

reg [31:0] pos_err_prev_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] vel_err_prev_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] dis_err_prev_f[NUMBER_OF_MOTORS-1:0];
reg [31:0] dis_myo_brick_err_prev_f[NUMBER_OF_MOTORS-1:0];

wire [31:0] pos_res_f;
wire [31:0] vel_res_f;
wire [31:0] dis_res_f;
wire [31:0] dis_myo_brick_res_f;

wire [31:0] pos_res;
wire [31:0] vel_res;
wire [31:0] dis_res;
wire [31:0] dis_myo_brick_res;

reg [7:0] pid_motor;

wire pos_conv, vel_conv, dis_conv, myo_conv;
wire pos_conv2err, vel_conv2err, dis_conv2err, myo_conv2err;
wire pos_err2preverr, vel_err2preverr, dis_err2preverr, myo_err2preverr;
wire pos_preverr2pres, vel_preverr2pres, dis_preverr2pres, myo_preverr2pres;
wire pos_pposres2dres, vel_pposres2dres, dis_pposres2dres, myo_pposres2dres;
wire pos_dres2result, vel_dres2result, dis_dres2result, myo_dres2result;
wire pos_result2ready, vel_result2ready, dis_result2ready, myo_result2ready;

//fpu pos2flo( clock, 0, INT2FLO, positions[motor], 0, pos_raw_f);
//fpu vel2flo( clock, 0, INT2FLO, velocities[motor], 0, vel_raw_f);
//fpu dis2flo( clock, 0, INT2FLO, displacements[motor], 0, dis_raw_f);
wire overflow;
qmult pos_conv_fpu( pos_raw_f, pos_encoder_multiplier_f[motor], pos_conv_f, overflow);
//fpu100 vel_conv_fpu( vel_raw_f, pos_encoder_multiplier_f[motor], MUL, 0, vel_conv_f);
//fpu100 dis_conv_fpu( dis_raw_f, dis_encoder_multiplier_f[motor], MUL, 0, dis_conv_f);
//fpu100 myo_brick_dis_fpu (clock, pos_conv_f, dis_conv_f, SUB, 0, myo_brick_dis_f, 														myo_conv, myo_conv2err);
//
//fpu100 pos_err( clock, sp_f[motor], pos_conv_f, SUB, 0, p_pos_err_f, 																		pos_conv2err, pos_err2preverr);
//fpu100 vel_err( clock, sp_f[motor], vel_conv_f, SUB, 0, p_vel_err_f, 																		vel_conv2err, vel_err2preverr);
//fpu100 dis_err( clock, sp_f[motor], dis_conv_f, SUB, 0, p_dis_err_f, 																		dis_conv2err, dis_err2preverr);
//fpu100 dis_myo_err( clock, sp_f[motor], myo_brick_dis_f, SUB, 0,p_dis_myo_brick_err_f,													myo_conv2err, myo_err2preverr);
//
//fpu100 pos_err_prev( clock, p_pos_err_f, pos_err_prev_f[motor], SUB, 0, d_pos_err_f,														pos_err2preverr, pos_preverr2pres);
//fpu100 vel_err_prev( clock, p_vel_err_f, vel_err_prev_f[motor], SUB, 0, d_vel_err_f,														vel_err2preverr, vel_preverr2pres);
//fpu100 dis_err_prev( clock, p_dis_err_f, dis_err_prev_f[motor], SUB, 0, d_dis_err_f,														dis_err2preverr, dis_preverr2pres);
//fpu100 dis_myo_brick_err_prev( clock, p_dis_myo_brick_err_f, dis_myo_brick_err_prev_f[motor], SUB, 0, d_dis_myo_brick_err_f,	myo_err2preverr, myo_preverr2pres);
//
//fpu100 p_pos_res( clock, p_pos_err_f, Kp_f[motor], MUL, 0, p_pos_res_f,																		pos_preverr2pres, pos_pposres2dres);
//fpu100 p_vel_res( clock, p_vel_err_f, Kp_f[motor], MUL, 0, p_vel_res_f,																		vel_preverr2pres, vel_pposres2dres);
//fpu100 p_dis_res( clock, p_dis_err_f, Kp_f[motor], MUL, 0, p_dis_res_f,																		dis_preverr2pres, dis_pposres2dres);
//fpu100 p_dis_myo_brick_res( clock, p_dis_myo_brick_err_f, Kp_f[motor], MUL, 0, p_dis_myo_brick_res_f,								myo_preverr2pres, myo_pposres2dres);
//
//fpu100 d_pos_res( clock, d_pos_err_f, Kd_f[motor], MUL, 0, d_pos_res_f,																		pos_pposres2dres, pos_dres2result);
//fpu100 d_vel_res( clock, d_vel_err_f, Kd_f[motor], MUL, 0, d_vel_res_f,																		vel_pposres2dres, vel_dres2result);
//fpu100 d_dis_res( clock, d_dis_err_f, Kd_f[motor], MUL, 0, d_dis_res_f,																		dis_pposres2dres, dis_dres2result);
//fpu100 d_dis_myo_brick_res( clock, d_dis_myo_brick_err_f, Kd_f[motor], MUL, 0, d_dis_myo_brick_res_f,								myo_pposres2dres, myo_dres2result);
//
//fpu100 pos_result( clock, p_pos_res_f, d_pos_res_f, ADD, 0, pos_res_f,																		pos_dres2result, pos_result2ready);
//fpu100 vel_result( clock, p_vel_res_f, d_pos_res_f, ADD, 0, vel_res_f,																		vel_dres2result, vel_result2ready);
//fpu100 dis_result( clock, p_dis_res_f, d_pos_res_f, ADD, 0, dis_res_f,																		dis_dres2result, dis_result2ready);
//fpu100 dis_myo_brick_result( clock, p_dis_myo_brick_res_f, d_dis_myo_brick_res_f, ADD, 0, dis_myo_brick_res_f,					myo_dres2result, myo_result2ready);

//fpu pos_res2int( clock, 0, FLO2INT, pos_res_f, 0, pos_res);
//fpu vel_res2int( clock, 0, FLO2INT, vel_res_f, 0, vel_res);
//fpu dis_res2int( clock, 0, FLO2INT, dis_res_f, 0, dis_res);
//fpu dis_myo_brick_res2int( clock, 0, FLO2INT, dis_myo_brick_res_f, 0, dis_myo_brick_res);

//fpu pos2flo( clock, 0, INT2FLO, positions[motor], 0, pos_raw_f);
//fpu vel2flo( clock, 0, INT2FLO, velocities[motor], 0, vel_raw_f);
//fpu dis2flo( clock, 0, INT2FLO, displacements[motor], 0, dis_raw_f);
//
//fpu100 pos_conv_fpu( clock, pos_raw_f, pos_encoder_multiplier_f[motor], MUL, 0, pos_conv_f, 											pos_conv, pos_conv2err);
//fpu100 vel_conv_fpu( clock, vel_raw_f, pos_encoder_multiplier_f[motor], MUL, 0, vel_conv_f, 									vel_conv, vel_conv2err);
//fpu100 dis_conv_fpu( clock, dis_raw_f, dis_encoder_multiplier_f[motor], MUL, 0, dis_conv_f, 									dis_conv, dis_conv2err);
//fpu100 myo_brick_dis_fpu (clock, pos_conv_f, dis_conv_f, SUB, 0, myo_brick_dis_f, 														myo_conv, myo_conv2err);
//
//fpu100 pos_err( clock, sp_f[motor], pos_conv_f, SUB, 0, p_pos_err_f, 																		pos_conv2err, pos_err2preverr);
//fpu100 vel_err( clock, sp_f[motor], vel_conv_f, SUB, 0, p_vel_err_f, 																		vel_conv2err, vel_err2preverr);
//fpu100 dis_err( clock, sp_f[motor], dis_conv_f, SUB, 0, p_dis_err_f, 																		dis_conv2err, dis_err2preverr);
//fpu100 dis_myo_err( clock, sp_f[motor], myo_brick_dis_f, SUB, 0,p_dis_myo_brick_err_f,													myo_conv2err, myo_err2preverr);
//
//fpu100 pos_err_prev( clock, p_pos_err_f, pos_err_prev_f[motor], SUB, 0, d_pos_err_f,														pos_err2preverr, pos_preverr2pres);
//fpu100 vel_err_prev( clock, p_vel_err_f, vel_err_prev_f[motor], SUB, 0, d_vel_err_f,														vel_err2preverr, vel_preverr2pres);
//fpu100 dis_err_prev( clock, p_dis_err_f, dis_err_prev_f[motor], SUB, 0, d_dis_err_f,														dis_err2preverr, dis_preverr2pres);
//fpu100 dis_myo_brick_err_prev( clock, p_dis_myo_brick_err_f, dis_myo_brick_err_prev_f[motor], SUB, 0, d_dis_myo_brick_err_f,	myo_err2preverr, myo_preverr2pres);
//
//fpu100 p_pos_res( clock, p_pos_err_f, Kp_f[motor], MUL, 0, p_pos_res_f,																		pos_preverr2pres, pos_pposres2dres);
//fpu100 p_vel_res( clock, p_vel_err_f, Kp_f[motor], MUL, 0, p_vel_res_f,																		vel_preverr2pres, vel_pposres2dres);
//fpu100 p_dis_res( clock, p_dis_err_f, Kp_f[motor], MUL, 0, p_dis_res_f,																		dis_preverr2pres, dis_pposres2dres);
//fpu100 p_dis_myo_brick_res( clock, p_dis_myo_brick_err_f, Kp_f[motor], MUL, 0, p_dis_myo_brick_res_f,								myo_preverr2pres, myo_pposres2dres);
//
//fpu100 d_pos_res( clock, d_pos_err_f, Kd_f[motor], MUL, 0, d_pos_res_f,																		pos_pposres2dres, pos_dres2result);
//fpu100 d_vel_res( clock, d_vel_err_f, Kd_f[motor], MUL, 0, d_vel_res_f,																		vel_pposres2dres, vel_dres2result);
//fpu100 d_dis_res( clock, d_dis_err_f, Kd_f[motor], MUL, 0, d_dis_res_f,																		dis_pposres2dres, dis_dres2result);
//fpu100 d_dis_myo_brick_res( clock, d_dis_myo_brick_err_f, Kd_f[motor], MUL, 0, d_dis_myo_brick_res_f,								myo_pposres2dres, myo_dres2result);
//
//fpu100 pos_result( clock, p_pos_res_f, d_pos_res_f, ADD, 0, pos_res_f,																		pos_dres2result, pos_result2ready);
//fpu100 vel_result( clock, p_vel_res_f, d_pos_res_f, ADD, 0, vel_res_f,																		vel_dres2result, vel_result2ready);
//fpu100 dis_result( clock, p_dis_res_f, d_pos_res_f, ADD, 0, dis_res_f,																		dis_dres2result, dis_result2ready);
//fpu100 dis_myo_brick_result( clock, p_dis_myo_brick_res_f, d_dis_myo_brick_res_f, ADD, 0, dis_myo_brick_res_f,					myo_dres2result, myo_result2ready);
//
//fpu pos_res2int( clock, 0, FLO2INT, pos_res_f, 0, pos_res);
//fpu vel_res2int( clock, 0, FLO2INT, vel_res_f, 0, vel_res);
//fpu dis_res2int( clock, 0, FLO2INT, dis_res_f, 0, dis_res);
//fpu dis_myo_brick_res2int( clock, 0, FLO2INT, dis_myo_brick_res_f, 0, dis_myo_brick_res);

//fpu pos2flo( clock, 0, INT2FLO, positions[motor], 0, pos_raw_f);
//fpu vel2flo( clock, 0, INT2FLO, velocities[motor], 0, vel_raw_f);
//fpu dis2flo( clock, 0, INT2FLO, displacements[motor], 0, dis_raw_f);
//
//fpu pos_conv( clock, 0, MUL, pos_raw_f, pos_encoder_multiplier_f[motor], pos_conv_f);
//fpu vel_conv( clock, 0, MUL, vel_raw_f, pos_encoder_multiplier_f[motor], vel_conv_f);
//fpu dis_conv( clock, 0, MUL, dis_raw_f, dis_encoder_multiplier_f[motor], dis_conv_f);
//fpu myo_brick_dis (clock, 0, SUB, pos_conv_f, dis_conv_f, myo_brick_dis_f);
//
//fpu pos_err( clock, 0, SUB, sp_f[motor], pos_conv_f, p_pos_err_f);
//fpu vel_err( clock, 0, SUB, sp_f[motor], vel_conv_f, p_vel_err_f);
//fpu dis_err( clock, 0, SUB, sp_f[motor], dis_conv_f, p_dis_err_f);
//fpu dis_myo_err( clock, 0, SUB, sp_f[motor], myo_brick_dis_f, p_dis_myo_brick_err_f);
//
//fpu pos_err_prev( clock, 0, SUB, p_pos_err_f, pos_err_prev_f[motor], d_pos_err_f);
//fpu vel_err_prev( clock, 0, SUB, p_vel_err_f, vel_err_prev_f[motor], d_vel_err_f);
//fpu dis_err_prev( clock, 0, SUB, p_dis_err_f, dis_err_prev_f[motor], d_dis_err_f);
//fpu dis_myo_brick_err_prev( clock, 0, SUB, p_dis_myo_brick_err_f, dis_myo_brick_err_prev_f[motor], d_dis_myo_brick_err_f);
//
//fpu p_pos_res( clock, 0, MUL, p_pos_err_f, Kp_f[motor], p_pos_res_f);
//fpu p_vel_res( clock, 0, MUL, p_vel_err_f, Kp_f[motor], p_vel_res_f);
//fpu p_dis_res( clock, 0, MUL, p_dis_err_f, Kp_f[motor], p_dis_res_f);
//fpu p_dis_myo_brick_res( clock, 0, MUL, p_dis_myo_brick_err_f, Kp_f[motor], p_dis_myo_brick_res_f);
//
//fpu d_pos_res( clock, 0, MUL, d_pos_err_f, Kd_f[motor], d_pos_res_f);
//fpu d_vel_res( clock, 0, MUL, d_vel_err_f, Kd_f[motor], d_vel_res_f);
//fpu d_dis_res( clock, 0, MUL, d_dis_err_f, Kd_f[motor], d_dis_res_f);
//fpu d_dis_myo_brick_res( clock, 0, MUL, d_dis_myo_brick_err_f, Kd_f[motor], d_dis_myo_brick_res_f);
//
//fpu pos_result( clock, 0, ADD, p_pos_res_f, d_pos_res_f, pos_res_f);
//fpu vel_result( clock, 0, ADD, p_vel_res_f, d_pos_res_f, vel_res_f);
//fpu dis_result( clock, 0, ADD, p_dis_res_f, d_pos_res_f, dis_res_f);
//fpu dis_myo_brick_result( clock, 0, ADD, p_dis_myo_brick_res_f, d_dis_myo_brick_res_f, dis_myo_brick_res_f);
//
//fpu pos_res2int( clock, 0, FLO2INT, pos_res_f, 0, pos_res);
//fpu vel_res2int( clock, 0, FLO2INT, vel_res_f, 0, vel_res);
//fpu dis_res2int( clock, 0, FLO2INT, dis_res_f, 0, dis_res);
//fpu dis_myo_brick_res2int( clock, 0, FLO2INT, dis_myo_brick_res_f, 0, dis_myo_brick_res);


endmodule