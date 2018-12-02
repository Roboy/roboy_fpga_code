


module A1339Control(
	input clock,
	input reset_n,
	A1339Interface interf
);

parameter CLOCK_SPEED = 50_000_000;
localparam CLOCK_SPEED_MILLIHZ = CLOCK_SPEED/1000;
parameter NUMBER_OF_SENSORS = 1;
parameter SAMPLES_TO_AVERAGE = 512;

wire di_req, wr_ack, do_valid, wren;
reg [19:0] data_send;
wire [19:0] turns_register = 20'h2C001;
wire [19:0] angle_register = 20'h20009;
wire [19:0] data_received;
wire ss_n;

wire signed [31:0] val;
assign val = ((data_received>>4)&12'hFFF);
wire signed [11:0] rev_val;
assign rev_val = ((data_received>>4)&12'hFFF); 

reg trigger;
reg data_valid;

reg [7:0] current_sensor;

assign interf.sensor_angle = angle;
assign interf.sensor_angle_absolute = angle_absolute;
assign interf.sensor_angle_offset = angle_offset;
assign interf.sensor_angle_relative = angle_relative;
assign interf.sensor_angle_velocity = angle_velocity;
assign interf.sensor_revolution_counter = revolution_counter;

reg signed [31:0] angle [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_absolute [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_absolute_prev [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_relative [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_prev [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_filtered [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_offset [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] angle_velocity [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] revolution_counter [NUMBER_OF_SENSORS-1:0];
reg signed [31:0] revolution_counter_offset [NUMBER_OF_SENSORS-1:0];
reg [16:0] sample_counter[NUMBER_OF_SENSORS-1:0];
reg [16:0] revolution_sample_counter[NUMBER_OF_SENSORS-1:0];
integer freq_counter;
reg signed [31:0] update_time;

reg [3:0] state;
reg [3:0] next_state;
always @(posedge clock, negedge reset_n) begin: SPI_DATA_PROCESS
	parameter IDLE  = 0, TRIGGER_READ_TURNS = 1, WAIT_FOR_REQUEST_TURNS = 2, WAIT_FOR_RECEIVE_TURNS = 3, 
			TRIGGER_READ_ANGLE = 4, WAIT_FOR_RECEIVE_ANGLE = 5, WAIT_FOR_REQUEST_ANGLE = 6, CALCULATE_ANGLES = 7, DELAY = 8;
	reg CRC0;
	reg CRC1;
	reg CRC2;
	reg CRC3;
	reg DoInvert;
	reg [15:0] mask;
	reg [3:0] crc; 
	integer j;
	reg [31:0] delay_counter;
	reg interleaved;
	if (reset_n==0) begin
		state <= IDLE;
	end else begin
		trigger <= 0;
		interf.cycle <= 0;
		freq_counter <= freq_counter + 1;
		if(interf.zero_offset) begin
			for(j=0;j<NUMBER_OF_SENSORS;j=j+1) begin
				angle_offset[j] <= angle_relative[j];
				revolution_counter_offset[j]<=revolution_counter[j];
			end
		end
		case(state)
			IDLE: begin
				current_sensor <= current_sensor+1;
				if(current_sensor>=NUMBER_OF_SENSORS) begin
					current_sensor <= 0;
				end
				if(sample_counter[current_sensor]==0) begin
					state <= TRIGGER_READ_TURNS;
				end else begin 
					state <= TRIGGER_READ_ANGLE;
				end
			end
			TRIGGER_READ_TURNS: begin 
				data_send <= turns_register;
				state <= WAIT_FOR_REQUEST_TURNS;
				trigger <= 1;
			end
			WAIT_FOR_REQUEST_TURNS: begin
				if(do_valid) begin
					delay_counter <= 50;
					state<=DELAY;
					next_state<=WAIT_FOR_RECEIVE_TURNS;
				end
			end
			WAIT_FOR_RECEIVE_TURNS: begin
				if(do_valid) begin
					CRC0 = 1'b1;
					CRC1 = 1'b1;
					CRC2 = 1'b1;
					CRC3 = 1'b1;
					mask = 16'h8000;
					for (j = 0; j < 16; j=j+1) begin
						  DoInvert = (((data_received>>4) & mask) != 0) ^ CRC3;         // XOR required?
						  CRC3 = CRC2;
						  CRC2 = CRC1;
						  CRC1 = CRC0 ^ DoInvert;
						  CRC0 = DoInvert; 
						  mask = mask >> 1;
					end
					crc = (CRC3 ? 4'd8 : 4'd0) + (CRC2 ? 4'd4 : 4'd0) + (CRC1 ? 4'd2 : 4'd0) + (CRC0 ? 4'd1 : 4'd0);
					if(crc == data_received[3:0]) begin
						data_valid <= 1;
						revolution_counter[current_sensor] <= rev_val;
					end else begin
						revolution_counter[current_sensor] <= 0;
						data_valid <= 0;
					end
					state<=TRIGGER_READ_ANGLE;
				end
			end
			TRIGGER_READ_ANGLE: begin 
				data_send <= angle_register;
				if(sample_counter[current_sensor]==0) begin
					state <= WAIT_FOR_REQUEST_ANGLE; // because of the interleaved queries, we need to wait for the last sample
				end else begin
					state <= WAIT_FOR_RECEIVE_ANGLE;
				end
				trigger <= 1;
			end
			WAIT_FOR_REQUEST_ANGLE: begin
				if(do_valid) begin
					delay_counter <= 50;
					state<=DELAY;
					next_state<=WAIT_FOR_RECEIVE_ANGLE;
				end
			end
			WAIT_FOR_RECEIVE_ANGLE: begin
				if(do_valid) begin
					CRC0 = 1'b1;
					CRC1 = 1'b1;
					CRC2 = 1'b1;
					CRC3 = 1'b1;
					mask = 16'h8000;
					for (j = 0; j < 16; j=j+1) begin
						  DoInvert = (((data_received>>4) & mask) != 0) ^ CRC3;         // XOR required?
						  CRC3 = CRC2;
						  CRC2 = CRC1;
						  CRC1 = CRC0 ^ DoInvert;
						  CRC0 = DoInvert; 
						  mask = mask >> 1; 
					end
					crc = (CRC3 ? 4'd8 : 4'd0) + (CRC2 ? 4'd4 : 4'd0) + (CRC1 ? 4'd2 : 4'd0) + (CRC0 ? 4'd1 : 4'd0);
					if(crc == data_received[3:0]) begin
						data_valid <= 1;
						if(SAMPLES_TO_AVERAGE!=0) begin
							if(sample_counter[current_sensor]<SAMPLES_TO_AVERAGE)begin
								angle_filtered[current_sensor] <= angle_filtered[current_sensor]+ val;
								state<=IDLE;
								sample_counter[current_sensor] <= sample_counter[current_sensor]+1;
							end else begin
								angle[current_sensor] <= (angle_filtered[current_sensor]>>>$clog2(SAMPLES_TO_AVERAGE));
								angle_filtered[current_sensor] <= 0;
								state <= CALCULATE_ANGLES;
								sample_counter[current_sensor] <= 0;
							end
						end else begin // no average
							angle[current_sensor] <= val;
							state <= CALCULATE_ANGLES;
						end
					end else begin
						data_valid <= 0;
						state<=IDLE;
					end
				end
			end
			CALCULATE_ANGLES: begin
				angle_relative[current_sensor] = angle[current_sensor]%512;
				angle_absolute_prev[current_sensor] = angle_absolute[current_sensor];
				angle_absolute[current_sensor] = angle_relative[current_sensor] + 
												(revolution_counter[current_sensor]-revolution_counter_offset[current_sensor])*$signed(512) -
												angle_offset[current_sensor];
				interf.cycle[current_sensor] <= 1'b1;
				if(update_time!=0) begin
					angle_velocity[current_sensor] = (angle_absolute[current_sensor]-angle_absolute_prev[current_sensor])/update_time;
				end
				if(current_sensor==0) begin
					update_time <= freq_counter/CLOCK_SPEED_MILLIHZ;
					freq_counter <= 0;
				end
				state<=IDLE;
			end
			DELAY: begin
				delay_counter <= delay_counter -1;
				if(delay_counter==0) begin
					trigger <= 1;
					state <= next_state;
				end
			end
			default: state = IDLE;
		endcase
	end
end

genvar k;
generate 
	for(k=0; k<NUMBER_OF_SENSORS; k = k+1) begin : assign_slave_select
		assign interf.ss_n_o[k] = (current_sensor==k?ss_n:1);
	end
endgenerate 

spi_master #(20, 1'b1, 1'b1, 2, 3) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(~reset_n),
	.spi_miso_i(interf.miso_i),
	.di_i(data_send),
	.wren_i(trigger),
	.spi_ssel_o(ss_n),
	.spi_sck_o(interf.sck_o), 
	.spi_mosi_o(interf.mosi_o),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_received)
);

endmodule