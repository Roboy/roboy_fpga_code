module ArmBusComs #(parameter NUMBER_OF_MOTORS = 8, parameter CLK_FREQ_HZ = 50_000_000)(
	input clk,
	input reset,
	output tx_o,
	output tx_enable,
	input rx_i,
	input wire [31:0] update_frequency_Hz,
	input wire [31:0] baudrate[NUMBER_OF_MOTORS-1:0],
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
	output wire [31:0] communication_quality[NUMBER_OF_MOTORS-1:0]
);

// `define DEBUG

typedef reg [31:0] uint32_t;
typedef reg [15:0] uint16_t;
typedef reg [7:0] uint8_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint16_t crc;
}status_request_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint8_t control_mode;
  uint16_t setpoint0;
  uint16_t setpoint1;
  uint16_t setpoint2;
  uint16_t setpoint3;
  uint16_t position0;
  uint16_t position1;
  uint16_t position2;
  uint16_t position3;
  uint16_t current0;
  uint16_t current1;
  uint16_t current2;
  uint16_t current3;
  uint16_t crc;
}hand_status_response_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint16_t setpoint0;
  uint16_t setpoint1;
  uint16_t setpoint2;
  uint16_t setpoint3;
  uint16_t crc;
}hand_command_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint8_t motor;
  uint8_t control_mode;
  uint16_t setpoint;
  uint32_t Kp;
  uint32_t Ki;
  uint32_t Kd;
  uint32_t deadband;
  uint32_t IntegralLimit;
  uint32_t PWMLimit;
  uint16_t crc;
}hand_control_mode_t;

localparam  MAGIC_NUMBER_LENGTH = 4;
localparam	STATUS_REQUEST_FRAME_LENGTH = $bits(status_request_t)/8;
localparam	HAND_STATUS_RESPONSE_FRAME_LENGTH = $bits(hand_status_response_t)/8;
localparam	HAND_COMMAND_FRAME_LENGTH = $bits(hand_command_t)/8;
localparam	HAND_CONTROL_MODE_FRAME_LENGTH = $bits(hand_control_mode_t)/8;

localparam  MAX_FRAME_LENGTH = HAND_CONTROL_MODE_FRAME_LENGTH;

status_request_t status_request = '{32'hABADBABE,0,0};
hand_status_response_t hand_status_response = '{32'hB000B135,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
hand_command_t hand_command = '{32'hB105F00D,0,0,0,0,0,0};
hand_control_mode_t hand_control_mode = '{32'hB15B00B5,0,0,0,0,0,0,0,0,0,0,0};

// arrrggghhh, just because quartus doesn't support unions...
wire [7:0] status_request_data [MAX_FRAME_LENGTH-1:0];
reg [7:0] hand_status_response_data [MAX_FRAME_LENGTH-1:0];
wire [7:0] hand_command_data [MAX_FRAME_LENGTH-1:0];
wire [7:0] hand_control_mode_data [MAX_FRAME_LENGTH-1:0];

assign status_request_data[0] = status_request.header[31:24];
assign status_request_data[1] = status_request.header[23:16];
assign status_request_data[2] = status_request.header[15:8];
assign status_request_data[3] = status_request.header[7:0];
assign status_request_data[4] = status_request.id;
assign status_request_data[5] = status_request.crc[15:8];
assign status_request_data[6] = status_request.crc[7:0];

assign hand_command_data[0] = hand_command.header[31:24];
assign hand_command_data[1] = hand_command.header[23:16];
assign hand_command_data[2] = hand_command.header[15:8];
assign hand_command_data[3] = hand_command.header[7:0];
assign hand_command_data[4] = hand_command.id;
assign hand_command_data[5] = hand_command.setpoint0[15:8];
assign hand_command_data[6] = hand_command.setpoint0[7:0];
assign hand_command_data[7] = hand_command.setpoint1[15:8];
assign hand_command_data[8] = hand_command.setpoint1[7:0];
assign hand_command_data[9] = hand_command.setpoint2[15:8];
assign hand_command_data[10] = hand_command.setpoint2[7:0];
assign hand_command_data[11] = hand_command.setpoint3[15:8];
assign hand_command_data[12] = hand_command.setpoint3[7:0];
assign hand_command_data[13] = hand_command.crc[15:8];
assign hand_command_data[14] = hand_command.crc[7:0];

assign hand_control_mode_data[0] = hand_control_mode.header[31:24];
assign hand_control_mode_data[1] = hand_control_mode.header[23:16];
assign hand_control_mode_data[2] = hand_control_mode.header[15:8];
assign hand_control_mode_data[3] = hand_control_mode.header[7:0];
assign hand_control_mode_data[4] = hand_control_mode.id;
assign hand_control_mode_data[5] = hand_control_mode.crc[15:8];
assign hand_control_mode_data[6] = hand_control_mode.crc[7:0];

`include "crc16.sv"

	reg [7:0]byte_transmit_counter ;
	reg [15:0] data ;
	wire[7:0] data_out[MAX_FRAME_LENGTH-1:0];
	wire [7:0] tx_data ;
	wire tx_active ;
	wire tx_done ;
	reg tx_transmit ;
	wire rx_data_ready;

	assign tx_data = data_out[byte_transmit_counter];

	uart_tx #(CLK_FREQ_HZ) tx(clk,baudrate[motor],tx_transmit,tx_data,tx_active,tx_o,tx_enable,tx_done);

	reg [15:0] tx_crc ;
	integer receive_byte_counter;
	reg [31:0]delay_counter;
	reg tx_active_prev;
	reg [7:0] motor;
	reg timeout;
	reg [31:0] status_requests[NUMBER_OF_MOTORS-1:0];
	reg [31:0] status_received[NUMBER_OF_MOTORS-1:0];
	reg trigger_control_mode_update[NUMBER_OF_MOTORS-1:0];
	reg trigger_hand_command_update[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] setpoint_actual[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] neopxl_color_actual[NUMBER_OF_MOTORS-1:0];

	localparam IDLE=8'h0,
			PREPARE_HAND_CONTROL_MODE = 8'h1, GENERATE_HAND_CONTROL_MODE_CRC = 8'h2, SEND_HAND_CONTROL_MODE = 8'h3,
			PREPARE_HAND_COMMAND  = 8'h4, GENERATE_HAND_COMMAND_CRC = 8'h5, SEND_HAND_COMMAND = 8'h6,
			PREPARE_STATUS_REQUEST = 8'h7, GENERATE_STATUS_REQUEST_CRC = 8'h8, SEND_STATUS_REQUEST = 8'h9,
			WAIT_UNTIL_BUS_FREE = 8'hA;
	reg [7:0] state;

	assign data_out =
	(state==GENERATE_HAND_CONTROL_MODE_CRC||state==SEND_HAND_CONTROL_MODE)?hand_control_mode_data:
	(state==GENERATE_HAND_COMMAND_CRC||state==SEND_HAND_COMMAND)?hand_command_data:
	(state==GENERATE_STATUS_REQUEST_CRC||state==SEND_STATUS_REQUEST)?status_request_data:data_in_frame;

	always @(posedge clk, posedge reset) begin: UART_TRANSMITTER
		reg done;
		integer update_delay_counter;
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

			if(update_delay_counter>0)begin
				update_delay_counter <= update_delay_counter - 1;
			end

			`ifdef DEBUG // echo the received bytes
			if(status_byte_received)begin
				byte_transmit_counter = receive_byte_counter-1;
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
				PREPARE_HAND_CONTROL_MODE: begin
					hand_control_mode.id <= id[motor]; // motor id
					hand_control_mode.control_mode <= control_mode[motor];
					hand_control_mode.setpoint <= setpoint[motor];
					hand_control_mode.Kp <= Kp[motor];
					hand_control_mode.Ki <= Ki[motor];
					hand_control_mode.Kd <= Kd[motor];
					hand_control_mode.deadband <= deadband[motor];
					hand_control_mode.IntegralLimit <= IntegralLimit[motor];
					hand_control_mode.PWMLimit <= PWMLimit[motor];
					state <= GENERATE_HAND_CONTROL_MODE_CRC;
				end
				GENERATE_HAND_CONTROL_MODE_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<HAND_CONTROL_MODE_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					hand_control_mode.crc = tx_crc;
					byte_transmit_counter = 0;
					state <= SEND_HAND_CONTROL_MODE;
					tx_transmit <= 1;
				end
				SEND_HAND_CONTROL_MODE: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<HAND_CONTROL_MODE_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state <= IDLE;
						end
					end
				end
				PREPARE_HAND_COMMAND: begin
					hand_command.id <= id[motor]; // motor id
					hand_command.setpoint0 <= setpoint[6];
					hand_command.setpoint1 <= setpoint[7];
					hand_command.setpoint2 <= setpoint[8];
					hand_command.setpoint3 <= setpoint[9];
					state <= GENERATE_HAND_COMMAND_CRC;
				end
				GENERATE_HAND_COMMAND_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<HAND_COMMAND_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					hand_command.crc = tx_crc;
					byte_transmit_counter = 0;
					tx_transmit <= 1;
					state <= SEND_HAND_COMMAND;
				end
				SEND_HAND_COMMAND: begin
					if(!tx_active && tx_active_prev)begin
						byte_transmit_counter = byte_transmit_counter+1;
					end
					if(!tx_active && !tx_transmit)begin
						if(byte_transmit_counter<HAND_COMMAND_FRAME_LENGTH)begin
							tx_transmit <= 1;
						end else begin
							state <= IDLE;
						end
					end
				end
				PREPARE_STATUS_REQUEST: begin
					status_request.id <= id[motor]; // motor id
					state <= GENERATE_STATUS_REQUEST_CRC;
				end
				GENERATE_STATUS_REQUEST_CRC: begin
					tx_crc = 16'hFFFF;
					for(i=MAGIC_NUMBER_LENGTH;i<STATUS_REQUEST_FRAME_LENGTH-2;i=i+1) begin
						tx_crc = nextCRC16_D8(data_out[i],tx_crc);
					end
					status_request.crc = tx_crc;
					byte_transmit_counter = 0;
					// timeout counter: (8bit + 1 start-bit + 1 stop-bit) * 2 because we are not talking to fpgas here
					delay_counter = CLK_FREQ_HZ/baudrate[motor]*((MAX_FRAME_LENGTH+1)*8+2*(MAX_FRAME_LENGTH+1))*2;
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
					if(delay_counter==0 || trigger_control_mode_update[motor] || trigger_hand_command_update[motor]) begin
						// control_mode update has higher priority because it also sends a new setpoint
						if(trigger_control_mode_update[motor])begin
							state <= PREPARE_HAND_CONTROL_MODE;
						end else if (trigger_hand_command_update[motor]) begin
							state <= PREPARE_HAND_COMMAND;
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

	uart_rx #(CLK_FREQ_HZ) rx(clk,baudrate[motor],rx_i,rx_data_ready,rx_data);

	reg [7:0] data_in[MAGIC_NUMBER_LENGTH-1:0];
	reg [7:0] data_in_frame[MAX_FRAME_LENGTH-1:0];

	reg [15:0] rx_crc;
	reg status_byte_received;

	always @(posedge clk, posedge reset) begin: FRAME_MATCHER
		localparam IDLE = 8'h0, RECEIVE_HAND_STATUS = 8'h1, GENERATE_CRC_HAND_STATUS = 8'h2, CHECK_CRC_HAND_STATUS = 8'h3;
		reg [7:0] state;
		reg rx_data_ready_prev;
		integer j, k;
		if(reset) begin
			state = IDLE;
			receive_byte_counter <= 0;
		end else begin
			rx_data_ready_prev <= rx_data_ready;
			status_byte_received <= 0;
			for(j=0;j<NUMBER_OF_MOTORS;j=j+1)begin
				trigger_control_mode_update[motor] <= 0;
				trigger_hand_command_update[motor] <= 0;
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
					if({data_in[0],data_in[1],data_in[2],data_in[3]}==hand_status_response.header)begin
						receive_byte_counter <= 4;
						state = RECEIVE_HAND_STATUS;
						error_code[motor] <= 1;
					end
				end
				RECEIVE_HAND_STATUS: begin
					if(!timeout) begin
						if(rx_data_ready==1 && rx_data_ready_prev==0)begin
							status_byte_received <= 1;
							data_in_frame[receive_byte_counter] <= rx_data;
							receive_byte_counter <= receive_byte_counter + 1;
							error_code[motor] <= receive_byte_counter+1;
						end
						if(receive_byte_counter>=HAND_STATUS_RESPONSE_FRAME_LENGTH) begin
							state = GENERATE_CRC_HAND_STATUS;
							error_code[motor] <= 2;
						end
					end else begin
						state <= IDLE;
						error_code[motor] <= 32'hDEADBEAF;
						crc_checksum[motor] = 0;
					end
				end
				GENERATE_CRC_HAND_STATUS: begin
					rx_crc = 16'hFFFF;
					for(k=MAGIC_NUMBER_LENGTH;k<(HAND_STATUS_RESPONSE_FRAME_LENGTH-2);k=k+1) begin
						rx_crc = nextCRC16_D8(data_in_frame[k],rx_crc);
					end
					crc_checksum[motor] = {rx_crc,data_in_frame[30],data_in_frame[31]};
					state = CHECK_CRC_HAND_STATUS;
				end
				CHECK_CRC_HAND_STATUS: begin
					if(rx_crc=={data_in_frame[30],data_in_frame[31]}) begin // MATCH!
						if(data_in_frame[4]==id[motor])begin // and from the motor we requested
							encoder0_position[6] <= {data_in_frame[14],data_in_frame[15]};
							encoder0_position[7] <= {data_in_frame[16],data_in_frame[17]};
							encoder0_position[8] <= {data_in_frame[18],data_in_frame[19]};
							encoder0_position[9] <= {data_in_frame[20],data_in_frame[21]};
							current[6] <= {data_in_frame[22],data_in_frame[23]};
							current[7] <= {data_in_frame[24],data_in_frame[25]};
							current[8] <= {data_in_frame[26],data_in_frame[27]};
							current[9] <= {data_in_frame[28],data_in_frame[29]};
							status_received[motor] <= status_received[hand_status_response.id] + 1;
							if({data_in_frame[6],data_in_frame[7]}!=setpoint[6] ||
								{data_in_frame[8],data_in_frame[9]}!=setpoint[7] ||
								{data_in_frame[10],data_in_frame[11]}!=setpoint[8] ||
								{data_in_frame[12],data_in_frame[13]}!=setpoint[9] )begin
								trigger_hand_command_update[motor] <= 1;
							end
							error_code[motor] <= 0;
						end else begin
							error_code[motor] <= 3;
						end
						state <= IDLE;
					end else begin
						error_code[motor] <= 32'hBAADC0DE; // crc error
						state <= IDLE;
					end
				end
			endcase
		end
	end

endmodule
