module coms #(parameter NUMBER_OF_MOTORS = 8, parameter CLK_FREQ_HZ = 50_000_000, parameter BAUDRATE = 2_000_000)(
	input clk,
	input reset,
	output tx_o,
	output tx_enable,
	input rx_i,
	input wire [31:0] update_frequency_Hz,
	input wire [7:0] id[NUMBER_OF_MOTORS-1:0],
	output wire signed [23:0] duty[NUMBER_OF_MOTORS-1:0],
	output wire signed [23:0] encoder0_position[NUMBER_OF_MOTORS-1:0],
	output wire signed [23:0] encoder1_position[NUMBER_OF_MOTORS-1:0],
	output wire signed [15:0] current[NUMBER_OF_MOTORS-1:0],
	output wire signed [23:0] displacement[NUMBER_OF_MOTORS-1:0],
	input wire signed [23:0] setpoint[NUMBER_OF_MOTORS-1:0],
	input wire [23:0] neopxl_color[NUMBER_OF_MOTORS-1:0],
	input wire [7:0] control_mode[NUMBER_OF_MOTORS-1:0],
	input wire signed [15:0] Kp[NUMBER_OF_MOTORS-1:0],
	input wire signed [15:0] Ki[NUMBER_OF_MOTORS-1:0],
	input wire signed [15:0] Kd[NUMBER_OF_MOTORS-1:0],
	input wire signed [23:0] PWMLimit[NUMBER_OF_MOTORS-1:0],
	input wire signed [23:0] IntegralLimit[NUMBER_OF_MOTORS-1:0],
	input wire signed [23:0] deadband[NUMBER_OF_MOTORS-1:0],
	input wire signed [15:0] current_limit[NUMBER_OF_MOTORS-1:0],
	output wire [31:0] error_code[NUMBER_OF_MOTORS-1:0],
	output wire [31:0] crc_checksum[NUMBER_OF_MOTORS-1:0],
	output wire [31:0] communication_quality[NUMBER_OF_MOTORS-1:0],
	output wire [7:0] current_motor
);

//	`define DEBUG

	localparam  MAGIC_NUMBER_LENGTH = 4;
	localparam  STATUS_REQUEST_FRAME_MAGICNUMBER = 32'h1CE1CEBB;
	localparam	STATUS_REQUEST_FRAME_LENGTH = 7;
	localparam 	STATUS_FRAME_MAGICNUMBER = 32'h1CEB00DA;
	localparam  STATUS_FRAME_LENGTH = 28;
	localparam 	SETPOINT_FRAME_MAGICNUMBER = 32'hD0D0D0D0;
	localparam  SETPOINT_FRAME_LENGTH = 13;
	localparam 	CONTROL_MODE_FRAME_MAGICNUMBER = 32'hBAADA555;
	localparam  CONTROL_MODE_FRAME_LENGTH = 28;
	localparam  MAX_FRAME_LENGTH = STATUS_FRAME_LENGTH;

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
	assign current_motor = motor;
	reg timeout;
	reg [31:0] status_requests[NUMBER_OF_MOTORS-1:0];
	reg [31:0] status_received[NUMBER_OF_MOTORS-1:0];
	reg trigger_control_mode_update[NUMBER_OF_MOTORS-1:0];
	reg trigger_setpoint_update[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] setpoint_actual[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] neopxl_color_actual[NUMBER_OF_MOTORS-1:0];
	
	always @(posedge clk, posedge reset) begin: UART_TRANSMITTER
		localparam IDLE=8'h0, 
				PREPARE_CONTROL_MODE = 8'h1, GENERATE_CONTROL_MODE_CRC = 8'h2, SEND_CONTROL_MODE = 8'h3, 
				PREPARE_SETPOINT  = 8'h4, GENERATE_SETPOINT_CRC = 8'h5, SEND_SETPOINT = 8'h6,
				PREPARE_STATUS_REQUEST = 8'h7, GENERATE_STATUS_REQUEST_CRC = 8'h8, SEND_STATUS_REQUEST = 8'h9, 
				WAIT_UNTIL_BUS_FREE = 8'hA;
		reg [7:0] state;
		reg done;
		reg [31:0] update_delay_counter;
		integer i;
		if(reset) begin
			state = IDLE;
			done <= 1;
			update_delay_counter <= 0;
			for(i=0;i<NUMBER_OF_MOTORS;i=i+1)begin
				status_requests[i] <= 0;
			end
		end else begin
			tx_active_prev <= tx_active;
			tx_transmit <= 0;
			timeout <= 0;
			
			if(update_delay_counter!=0)begin
				update_delay_counter <= update_delay_counter - 1;
			end
			
			`ifdef DEBUG
			if(status_byte_received)begin
				byte_transmit_counter = receive_byte_counter-1;
				data_out[receive_byte_counter-1] <= data_in_frame[receive_byte_counter-1];
				tx_transmit <= 1;
			end
			`endif
			
			case(state)
				IDLE: begin
					if(update_delay_counter==0) begin
						update_delay_counter <= (CLK_FREQ_HZ/update_frequency_Hz/NUMBER_OF_MOTORS);
						state <= PREPARE_STATUS_REQUEST;
						if(motor<NUMBER_OF_MOTORS-1) begin
							motor <= motor + 1;
						end else begin
							motor <= 0;
						end
					end 
				end
				PREPARE_CONTROL_MODE: begin
					data_out[0] <= CONTROL_MODE_FRAME_MAGICNUMBER[31:24];
					data_out[1] <= CONTROL_MODE_FRAME_MAGICNUMBER[23:16];
					data_out[2] <= CONTROL_MODE_FRAME_MAGICNUMBER[15:8];
					data_out[3] <= CONTROL_MODE_FRAME_MAGICNUMBER[7:0];
					data_out[4] <= id[motor]; // motor id
					data_out[5] <= control_mode[motor]; // control_mode
					data_out[6] <= Kp[motor][15:8];
					data_out[7] <= Kp[motor][7:0];
					data_out[8] <= Ki[motor][15:8];
					data_out[9] <= Ki[motor][7:0];
					data_out[10] <= Kd[motor][15:8];
					data_out[11] <= Kd[motor][7:0];
					data_out[12] <= PWMLimit[motor][23:16];
					data_out[13] <= PWMLimit[motor][15:8];
					data_out[14] <= PWMLimit[motor][7:0];
					data_out[15] <= IntegralLimit[motor][23:16];
					data_out[16] <= IntegralLimit[motor][15:8];
					data_out[17] <= IntegralLimit[motor][7:0];
					data_out[18] <= deadband[motor][23:16];
					data_out[19] <= deadband[motor][15:8];
					data_out[20] <= deadband[motor][7:0];
					data_out[21] <= setpoint[motor][23:16];
					data_out[22] <= setpoint[motor][15:8];
					data_out[23] <= setpoint[motor][7:0];
					data_out[24] <= current_limit[motor][15:8];
					data_out[25] <= current_limit[motor][7:0];
					state <= GENERATE_CONTROL_MODE_CRC;
				end
				GENERATE_CONTROL_MODE_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<CONTROL_MODE_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[CONTROL_MODE_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[CONTROL_MODE_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					state <= SEND_CONTROL_MODE;
					tx_transmit <= 1;
				end
				SEND_CONTROL_MODE: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<CONTROL_MODE_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state <= IDLE;
						end
					end
				end
				PREPARE_SETPOINT: begin
					data_out[0] <= SETPOINT_FRAME_MAGICNUMBER[31:24];
					data_out[1] <= SETPOINT_FRAME_MAGICNUMBER[23:16];
					data_out[2] <= SETPOINT_FRAME_MAGICNUMBER[15:8];
					data_out[3] <= SETPOINT_FRAME_MAGICNUMBER[7:0];
					data_out[4] <= id[motor]; // motor id
					data_out[5] <= setpoint[motor][23:16];
					data_out[6] <= setpoint[motor][15:8];
					data_out[7] <= setpoint[motor][7:0];
					data_out[8] <= neopxl_color[motor][23:16];
					data_out[9] <= neopxl_color[motor][15:8];
					data_out[10] <= neopxl_color[motor][7:0];
					state <= GENERATE_SETPOINT_CRC;
				end
				GENERATE_SETPOINT_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<SETPOINT_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[SETPOINT_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[SETPOINT_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					tx_transmit <= 1;
					state <= SEND_SETPOINT;
				end
				SEND_SETPOINT: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<SETPOINT_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state <= IDLE;
						end
					end
				end
				PREPARE_STATUS_REQUEST: begin
					data_out[0] <= STATUS_REQUEST_FRAME_MAGICNUMBER[31:24];
					data_out[1] <= STATUS_REQUEST_FRAME_MAGICNUMBER[23:16];
					data_out[2] <= STATUS_REQUEST_FRAME_MAGICNUMBER[15:8];
					data_out[3] <= STATUS_REQUEST_FRAME_MAGICNUMBER[7:0];
					data_out[4] <= id[motor]; // motor id
					state <= GENERATE_STATUS_REQUEST_CRC;
				end
				GENERATE_STATUS_REQUEST_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<STATUS_REQUEST_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					data_out[STATUS_REQUEST_FRAME_LENGTH-2] = tx_crc[15:8];
					data_out[STATUS_REQUEST_FRAME_LENGTH-1] = tx_crc[7:0];
					byte_transmit_counter = 0;
					delay_counter = CLK_FREQ_HZ/BAUDRATE*(MAX_FRAME_LENGTH*8+1);
					status_requests[motor] <= status_requests[motor] + 1;
					state <= SEND_STATUS_REQUEST;
					tx_transmit <= 1;
				end
				SEND_STATUS_REQUEST: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<STATUS_REQUEST_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state <= WAIT_UNTIL_BUS_FREE;
						end
					end
				end
				WAIT_UNTIL_BUS_FREE: begin
					// either timeout or we know the bus is free now
					if(delay_counter==0 || trigger_control_mode_update[motor] || trigger_setpoint_update[motor]) begin 
						// control_mode update has higher priority because it also sends a new setpoint
						if(trigger_control_mode_update[motor])begin 
							state <= PREPARE_CONTROL_MODE;
						end else if (trigger_setpoint_update[motor]) begin
							state <= PREPARE_SETPOINT;
						end else begin
							state <= IDLE;
						end
						timeout <= 1;
						if(status_requests[motor]>update_frequency_Hz)begin
							status_requests[motor] <= 0;
						end else begin
							communication_quality[motor] <= (status_received[motor]*100)/status_requests[motor];
						end
					end else begin
						delay_counter = delay_counter - 1;
					end
				end
			endcase
		end
	end
	
	wire [7:0] rx_data ;

	uart_rx #(CLK_FREQ_HZ,BAUDRATE) rx(clk,rx_i,rx_data_ready,rx_data);

	reg [7:0] data_in[MAGIC_NUMBER_LENGTH-1:0];
	reg [7:0] data_in_frame[MAX_FRAME_LENGTH-1:0];

	reg [15:0] rx_crc;
	reg status_byte_received;
	
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
			status_byte_received <= 0;
			for(j=0;j<NUMBER_OF_MOTORS;j=j+1)begin
				trigger_control_mode_update[motor] <= 0;
				trigger_setpoint_update[motor] <= 0;
			end
			if(status_requests[motor]==0)begin // reset for communication_quality measurement
				status_received[motor] <= 0;
			end
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
						error_code[motor] <= 32'h1;
						state = RECEIVE_STATUS;
					end
				end
				RECEIVE_STATUS: begin
					if(!timeout) begin
						if(rx_data_ready==1 && rx_data_ready_prev==0)begin
							status_byte_received <= 1;
							data_in_frame[receive_byte_counter] <= rx_data;
							receive_byte_counter <= receive_byte_counter + 1;
						end
						if(receive_byte_counter>(STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-1)) begin
							state = CHECK_CRC_STATUS;
							motor_id <= data_in_frame[0];
							error_code[motor] <= 32'h2;
						end
					end else begin
						state <= IDLE;
						error_code[motor] <= 32'hDEADBEAF;
						crc_checksum[motor] = 0;
					end
				end
				CHECK_CRC_STATUS: begin
					rx_crc = 16'hFFFF;
					for(k=0;k<(STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2);k=k+1) begin
						rx_crc = nextCRC16_D8(data_in_frame[k],rx_crc);
					end
					crc_checksum[motor] = {rx_crc,
									data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2],
									data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-1]};
					if(rx_crc[15:8]==data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-2]
						  && rx_crc[7:0]==data_in_frame[STATUS_FRAME_LENGTH-MAGIC_NUMBER_LENGTH-1]
						  && (motor_id==id[motor])) begin // MATCH! and from the motor we requested
						// if the control_mode of the motor does not match the one we want or the motor lost connection we trigger an update
						if(data_in_frame[1]!=control_mode[data_in_frame[0]] || status_received[motor_id]==0) begin
							trigger_control_mode_update[motor] <= 1;
						end else begin
							error_code[data_in_frame[0]] <= 8'h0;
						end
						encoder0_position[motor][23:16] <= data_in_frame[2];
						encoder0_position[motor][15:8] <= data_in_frame[3];
						encoder0_position[motor][7:0] <= data_in_frame[4];
						encoder1_position[motor][23:16] <= data_in_frame[5];
						encoder1_position[motor][15:8] <= data_in_frame[6];
						encoder1_position[motor][7:0] <= data_in_frame[7];
						setpoint_actual[motor][23:16] <= data_in_frame[8];
						setpoint_actual[motor][15:8] <= data_in_frame[9];
						setpoint_actual[motor][7:0] <= data_in_frame[10];
						duty[motor][23:16] <= data_in_frame[11];
						duty[motor][15:8] <= data_in_frame[12];
						duty[motor][7:0] <= data_in_frame[13];
						displacement[motor][23:16] <= data_in_frame[14];
						displacement[motor][15:8] <= data_in_frame[15];
						displacement[motor][7:0] <= data_in_frame[16];
						current[motor][15:8] <= data_in_frame[17];
						current[motor][7:0] <= data_in_frame[18];
						neopxl_color_actual[motor][23:16] <= data_in_frame[19];
						neopxl_color_actual[motor][15:8] <= data_in_frame[20];
						neopxl_color_actual[motor][7:0] <= data_in_frame[21];
						status_received[motor] <= status_received[motor_id] + 1;
						if(setpoint_actual[motor]!=setpoint[motor] || 
							neopxl_color_actual[motor]!=neopxl_color[motor] )begin
							trigger_setpoint_update[motor] <= 1;
						end
						state = IDLE;
					end else begin
						error_code[motor] <= 32'hBAADC0DE; // crc error
						state <= IDLE;
					end
				end
			endcase
		end
	end

endmodule
