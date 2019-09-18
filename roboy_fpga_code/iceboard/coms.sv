module coms #(parameter NUMBER_OF_MOTORS = 6, parameter CLK_FREQ_HZ = 50_000_000, parameter BAUDRATE = 115200)(
	input clk,
	input reset,
	output tx_o,
	output tx_enable,
	input rx_i,
	input wire [31:0] status_update_frequency_Hz,
	input wire trigger_control_mode_update,
	input wire trigger_setpoint_update,
	input wire [7:0] motor_to_update,
	output wire signed [31:0] encoder0_position[NUMBER_OF_MOTORS-1:0],
	output wire signed [31:0] encoder1_position[NUMBER_OF_MOTORS-1:0],
	output wire signed [31:0] encoder0_velocity[NUMBER_OF_MOTORS-1:0],
	output wire signed [31:0] encoder1_velocity[NUMBER_OF_MOTORS-1:0],
	output wire [15:0] current_phase1[NUMBER_OF_MOTORS-1:0],
	output wire [15:0] current_phase2[NUMBER_OF_MOTORS-1:0],
	output wire [15:0] current_phase3[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] setpoint[NUMBER_OF_MOTORS-1:0],
	input wire [7:0] control_mode[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] Kp[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] Ki[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] Kd[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] PWMLimit[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] IntegralLimit[NUMBER_OF_MOTORS-1:0],
	input wire signed [31:0] deadband[NUMBER_OF_MOTORS-1:0],
	output reg [31:0] error_code[NUMBER_OF_MOTORS-1:0],
	output rx_receive,
	output reg [3:0] debug_signals
);

////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1999-2008 Easics NV.
// This source file may be used and distributed without restriction
// provided that this copyright statement is not removed from the file
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose : synthesizable CRC function
//   * polynomial: x^16 + x^15 + x^2 + 1
//   * data width: 8
//
// Info : tools@easics.be
//        http://www.easics.com
////////////////////////////////////////////////////////////////////////////////

// polynomial: x^16 + x^15 + x^2 + 1
// data width: 8
// convention: the first serial bit is D[7]
function [15:0] nextCRC16_D8;

	input [7:0] Data;
	input [15:0] crc;
	reg [7:0] d;
	reg [15:0] c;
	reg [15:0] newcrc;
	begin
		d = Data;
		c = crc;

		newcrc[0] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[3] ^ d[2] ^ d[1] ^ d[0] ^ c[8] ^ c[9] ^ c[10] ^ c[11] ^ c[12] ^ c[13] ^ c[14] ^ c[15];
		newcrc[1] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[3] ^ d[2] ^ d[1] ^ c[9] ^ c[10] ^ c[11] ^ c[12] ^ c[13] ^ c[14] ^ c[15];
		newcrc[2] = d[1] ^ d[0] ^ c[8] ^ c[9];
		newcrc[3] = d[2] ^ d[1] ^ c[9] ^ c[10];
		newcrc[4] = d[3] ^ d[2] ^ c[10] ^ c[11];
		newcrc[5] = d[4] ^ d[3] ^ c[11] ^ c[12];
		newcrc[6] = d[5] ^ d[4] ^ c[12] ^ c[13];
		newcrc[7] = d[6] ^ d[5] ^ c[13] ^ c[14];
		newcrc[8] = d[7] ^ d[6] ^ c[0] ^ c[14] ^ c[15];
		newcrc[9] = d[7] ^ c[1] ^ c[15];
		newcrc[10] = c[2];
		newcrc[11] = c[3];
		newcrc[12] = c[4];
		newcrc[13] = c[5];
		newcrc[14] = c[6];
		newcrc[15] = d[7] ^ d[6] ^ d[5] ^ d[4] ^ d[3] ^ d[2] ^ d[1] ^ d[0] ^ c[7] ^ c[8] ^ c[9] ^ c[10] ^ c[11] ^ c[12] ^ c[13] ^ c[14] ^ c[15];
		nextCRC16_D8 = newcrc;
	end
endfunction

	localparam  MAGIC_NUMBER_LENGTH = 4;
	localparam  STATUS_REQUEST_FRAME_MAGICNUMBER = 32'h1CE1CEBB;
	localparam	STATUS_REQUEST_FRAME_LENGTH = 7;
	localparam 	STATUS_FRAME_MAGICNUMBER = 32'h1CEB00DA;
	localparam  STATUS_FRAME_LENGTH = 30;
	localparam 	SETPOINT_FRAME_MAGICNUMBER = 32'hD0D0D0D0;
	localparam  SETPOINT_FRAME_LENGTH = 11;
	localparam 	CONTROL_MODE_FRAME_MAGICNUMBER = 32'hBAADA555;
	localparam  CONTROL_MODE_FRAME_LENGTH = 34;
	localparam  MAX_FRAME_LENGTH = CONTROL_MODE_FRAME_LENGTH;

	reg[7:0] byte_transmit_counter ;
	reg [15:0] data ;
	reg[7:0] data_out[MAX_FRAME_LENGTH-1:0];
	wire [7:0] tx_data ;
	wire tx_active ;
	wire tx_done ;
	reg tx_transmit ;
	wire rx_data_ready;
	
	assign tx_data = data_out[byte_transmit_counter];

	uart_tx #(CLK_FREQ_HZ,BAUDRATE) tx(clk,tx_transmit,tx_data,tx_active,tx_o,tx_enable,tx_done);

	reg [15:0] tx_crc ;
	integer receive_byte_counter;
	reg [31:0]delay_counter;
	reg tx_active_prev;
	reg [7:0] motor;
	always @(posedge clk, posedge reset) begin: UART_TRANSMITTER
		localparam IDLE=8'h0, PREPARE_CONTROL_MODE = 8'h1, SEND_CONTROL_MODE = 8'h2, PREPARE_SETPOINT  = 8'h3, SEND_SETPOINT = 8'h4,
				PREPARE_STATUS_REQUEST = 8'h5, SEND_STATUS_REQUEST = 8'h6, WAIT_UNTIL_BUS_FREE = 8'h7;
		reg [7:0] state;
		reg done;
		reg [31:0] status_update_delay_counter;
		integer i;
		if(reset) begin
			state = IDLE;
			done <= 1;
			status_update_delay_counter <= 0;
		end else begin
			tx_active_prev <= tx_active;
			tx_transmit <= 0;
			
			if(trigger_control_mode_update)begin
				if(motor_to_update==8'hFF) begin
					done <= 0;
					motor <= 0;
				end else begin
					motor <= motor_to_update;
				end
				state = PREPARE_CONTROL_MODE;
			end
			
			if(trigger_setpoint_update)begin
				if(motor_to_update==8'hFF) begin
					done <= 0;
					motor <= 0;
				end else begin
					motor <= motor_to_update;
				end
				state = PREPARE_SETPOINT;
			end
			
			if(status_update_delay_counter!=0)begin
				status_update_delay_counter <= status_update_delay_counter - 1;
			end
			
			if(rx_receive)begin
				byte_transmit_counter = receive_byte_counter;
				data_out[receive_byte_counter] <= data_in_frame[receive_byte_counter];
				tx_transmit <= 1;
			end
			
			case(state)
				IDLE: begin
					if(!done && motor_to_update==8'hFF)begin // if we are not done and all motors should be updated
						if(motor<NUMBER_OF_MOTORS-1) begin
							motor <= motor + 1;
						end else begin
							done <= 1;
							motor <= 0; 
						end
					end else begin
						if(status_update_delay_counter==0) begin
							status_update_delay_counter <= (CLK_FREQ_HZ/status_update_frequency_Hz/NUMBER_OF_MOTORS);
							state = PREPARE_STATUS_REQUEST;
							if(motor<NUMBER_OF_MOTORS-1) begin
								motor <= motor + 1;
							end else begin
								motor <= 0;
							end
						end
					end
				end
				PREPARE_CONTROL_MODE: begin
					data_out[0] = CONTROL_MODE_FRAME_MAGICNUMBER[31:24];
					data_out[1] = CONTROL_MODE_FRAME_MAGICNUMBER[23:16];
					data_out[2] = CONTROL_MODE_FRAME_MAGICNUMBER[15:8];
					data_out[3] = CONTROL_MODE_FRAME_MAGICNUMBER[7:0];
					data_out[4] = motor; // motor id
					data_out[5] = control_mode[motor]; // control_mode
					data_out[6] = Kp[motor][31:24];
					data_out[7] = Kp[motor][23:16];
					data_out[8] = Kp[motor][15:8];
					data_out[9] = Kp[motor][7:0];
					data_out[10] = Ki[motor][31:24];
					data_out[11] = Ki[motor][23:16];
					data_out[12] = Ki[motor][15:8];
					data_out[13] = Ki[motor][7:0];
					data_out[14] = Kd[motor][31:24];
					data_out[15] = Kd[motor][23:16];
					data_out[16] = Kd[motor][15:8];
					data_out[17] = Kd[motor][7:0];
					data_out[18] = PWMLimit[motor][31:24];
					data_out[19] = PWMLimit[motor][23:16];
					data_out[20] = PWMLimit[motor][15:8];
					data_out[21] = PWMLimit[motor][7:0];
					data_out[22] = IntegralLimit[motor][31:24];
					data_out[23] = IntegralLimit[motor][23:16];
					data_out[24] = IntegralLimit[motor][15:8];
					data_out[25] = IntegralLimit[motor][7:0];
					data_out[26] = deadband[motor][31:24];
					data_out[27] = deadband[motor][23:16];
					data_out[28] = deadband[motor][15:8];
					data_out[29] = deadband[motor][7:0];
					data_out[30] = setpoint[motor][31:24];
					data_out[31] = setpoint[motor][23:16];
					data_out[32] = setpoint[motor][15:8];
					data_out[33] = setpoint[motor][7:0];
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<CONTROL_MODE_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[CONTROL_MODE_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[CONTROL_MODE_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					state = SEND_CONTROL_MODE;
				end
				SEND_CONTROL_MODE: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<CONTROL_MODE_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state = IDLE;
						end
					end
				end
				PREPARE_SETPOINT: begin
					data_out[0] = SETPOINT_FRAME_MAGICNUMBER[31:24];
					data_out[1] = SETPOINT_FRAME_MAGICNUMBER[23:16];
					data_out[2] = SETPOINT_FRAME_MAGICNUMBER[15:8];
					data_out[3] = SETPOINT_FRAME_MAGICNUMBER[7:0];
					data_out[4] = motor; // motor id
					data_out[5] = setpoint[motor][31:24];
					data_out[6] = setpoint[motor][23:16];
					data_out[7] = setpoint[motor][15:8];
					data_out[8] = setpoint[motor][7:0];
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<SETPOINT_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[SETPOINT_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[SETPOINT_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					state = SEND_SETPOINT;
				end
				SEND_SETPOINT: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<SETPOINT_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state = IDLE;
						end
					end
				end
				PREPARE_STATUS_REQUEST: begin
					data_out[0] = STATUS_REQUEST_FRAME_MAGICNUMBER[31:24];
					data_out[1] = STATUS_REQUEST_FRAME_MAGICNUMBER[23:16];
					data_out[2] = STATUS_REQUEST_FRAME_MAGICNUMBER[15:8];
					data_out[3] = STATUS_REQUEST_FRAME_MAGICNUMBER[7:0];
					data_out[4] = motor; // motor id
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<STATUS_REQUEST_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[STATUS_REQUEST_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[STATUS_REQUEST_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					delay_counter = CLK_FREQ_HZ/BAUDRATE*(MAX_FRAME_LENGTH*8+MAX_FRAME_LENGTH*2);
					state = SEND_STATUS_REQUEST;
				end
				SEND_STATUS_REQUEST: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<STATUS_REQUEST_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state = WAIT_UNTIL_BUS_FREE;
						end
					end
				end
				WAIT_UNTIL_BUS_FREE: begin
					if(delay_counter==0) begin // we have to wait until the bus is free
						state = IDLE;
					end else begin
						delay_counter = delay_counter - 1;
					end
				end
			endcase
		end
	end

	assign rx_receive = rx_data_ready;
	
	wire [7:0] rx_data ;

	uart_rx #(CLK_FREQ_HZ,BAUDRATE) rx(clk,rx_i,rx_data_ready,rx_data);

	reg [7:0] data_in[MAGIC_NUMBER_LENGTH-1:0];
	reg [7:0] data_in_frame[MAX_FRAME_LENGTH-1:0];

	reg [15:0] rx_crc;
	
	always @(posedge clk, posedge reset) begin: FRAME_MATCHER
		localparam IDLE = 8'h0, RECEIVE_STATUS = 8'h1, CHECK_CRC_STATUS = 8'h2;
		reg [7:0] state;
		reg rx_data_ready_prev;
		integer j, k;
		integer motor_id;
		if(reset) begin
			state = IDLE;
			receive_byte_counter <= 0;
		end else begin
			rx_data_ready_prev <= rx_data_ready;
			if(rx_data_ready)begin
			  data_in[MAGIC_NUMBER_LENGTH-1] <= rx_data;
			  for(j=MAGIC_NUMBER_LENGTH-2;j>=0;j=j-1)begin
					data_in[j] <= data_in[j+1];
			  end
			end
			case(state)
				IDLE: begin
					if({data_in[0],data_in[1],data_in[2],data_in[3]}==STATUS_FRAME_MAGICNUMBER)begin
						receive_byte_counter <= 0;
						state = RECEIVE_STATUS;
						debug_signals[0] <= 1;
					end
				end
				RECEIVE_STATUS: begin
					if(rx_data_ready==1 && rx_data_ready_prev==0)begin
						data_in_frame[receive_byte_counter] = rx_data;
						receive_byte_counter <= receive_byte_counter + 1;
						debug_signals[1] <= 1;
					end
					if(receive_byte_counter>STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2) begin
						state = CHECK_CRC_STATUS;
						motor_id <= data_in_frame[0];
						error_code[motor] <= 32'h1CE1CEBB;
						debug_signals[2] <= 1;
					end
				end
				CHECK_CRC_STATUS: begin
//					rx_crc = 16'hFFFF;
//					for(k=0;k<STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2;k=k+1) begin
//						rx_crc = nextCRC16_D8(data_in_frame[k],rx_crc);
//					end
//					if(rx_crc[15:8]==data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2]
//						  && rx_crc[7:0]==data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-1]
//						  && (motor_id==motor)) begin // MATCH! and from the motor we requested
//						if(data_in_frame[1]!=control_mode[data_in_frame[0]]) begin
//							error_code[data_in_frame[0]] <= 8'h1; // control mode error
//						end else begin
//							error_code[data_in_frame[0]] <= 8'h0;
//						end
						encoder0_position[motor_id][31:24] <= data_in_frame[2];
						encoder0_position[motor_id][23:16] <= data_in_frame[3];
						encoder0_position[motor_id][15:8] <= data_in_frame[4];
						encoder0_position[motor_id][7:0] <= data_in_frame[5];
						encoder1_position[motor_id][31:24] <= data_in_frame[6];
						encoder1_position[motor_id][23:16] <= data_in_frame[7];
						encoder1_position[motor_id][15:8] <= data_in_frame[8];
						encoder1_position[motor_id][7:0] <= data_in_frame[9];
						encoder0_velocity[motor_id][31:24] <= data_in_frame[10];
						encoder0_velocity[motor_id][23:16] <= data_in_frame[11];
						encoder0_velocity[motor_id][15:8] <= data_in_frame[12];
						encoder0_velocity[motor_id][7:0] <= data_in_frame[13];
						encoder1_velocity[motor_id][31:24] <= data_in_frame[14];
						encoder1_velocity[motor_id][23:16] <= data_in_frame[15];
						encoder1_velocity[motor_id][15:8] <= data_in_frame[16];
						encoder1_velocity[motor_id][7:0] <= data_in_frame[17];
						current_phase1[motor_id][15:8] <= data_in_frame[18];
						current_phase1[motor_id][7:0] <= data_in_frame[19];
						current_phase2[motor_id][15:8] <= data_in_frame[20];
						current_phase2[motor_id][7:0] <= data_in_frame[21];
						current_phase3[motor_id][15:8] <= data_in_frame[22];
						current_phase3[motor_id][7:0] <= data_in_frame[23];
						error_code[motor_id] <= {receive_byte_counter,rx_crc}; // crc error
						state = IDLE;
						debug_signals[3] <= 1;
//					end else begin
//						error_code[motor] <= {16'hFFFF,data_in_frame[2],data_in_frame[3]}; // crc error
//						state <= IDLE;
//					end
				end
			endcase
		end
	end

endmodule
