module A1339Control(
	input clock,
	input reset_n,
	output wire [31:0] sensor_angle,
	input wire [7:0] sensor,
	// SPI
	output sck_o, // clock
	output [NUMBER_OF_SENSORS-1:0] ss_n_o, // slave select line for each sensor
	output mosi_o,	// mosi
	input miso_i,	// miso
	output reg [NUMBER_OF_SENSORS-1:0]cycle,
	output [2:0] LED
);

parameter NUMBER_OF_SENSORS = 1;
parameter SAMPLES_TO_AVERAGE = 512;

assign sensor_angle = angle[sensor];

wire di_req, wr_ack, do_valid, wren;
reg [19:0] data_send;
wire [15:0] data = 16'h2000;
wire [19:0] data_received;
wire ss_n;

reg trigger;
reg data_valid;

assign LED[2] = data_valid;
assign LED[0] = trigger;

reg [7:0] current_sensor;

reg [31:0] angle [NUMBER_OF_SENSORS-1:0];
reg [31:0] angle_filtered [NUMBER_OF_SENSORS-1:0];
reg [16:0] sample_counter[NUMBER_OF_SENSORS-1:0];

reg [2:0] state;
always @(posedge clock, negedge reset_n) begin: SPI_DATA_PROCESS
	parameter IDLE  = 0, CALCULATE_CRC = 1, TRIGGER_SEND = 2, WAIT_FOR_DATA = 4, DELAY = 5;
	reg CRC0;
	reg CRC1;
	reg CRC2;
	reg CRC3;
	reg DoInvert;
	reg [15:0] mask;
	reg [3:0] crc;
	integer j;
	reg [31:0] delay_counter;
	if (reset_n==0) begin
		state <= IDLE;
		trigger <= 0;
	end else begin
		trigger <= 0;
		cycle <= 0;
		case(state)
			IDLE: begin
				state <= CALCULATE_CRC;
			end
			CALCULATE_CRC: begin 
				CRC0 = 1'b1;
				CRC1 = 1'b1;
				CRC2 = 1'b1;
				CRC3 = 1'b1;
				mask = 16'h8000;
				for (j = 0; j < 16; j=j+1) begin
					  DoInvert = ((data & mask) != 0) ^ CRC3;         // XOR required?
					  CRC3 = CRC2;
					  CRC2 = CRC1;
					  CRC1 = CRC0 ^ DoInvert;
					  CRC0 = DoInvert; 
					  mask = mask >> 1;
				end
				crc = (CRC3 ? 4'd8 : 4'd0) + (CRC2 ? 4'd4 : 4'd0) + (CRC1 ? 4'd2 : 4'd0) + (CRC0 ? 4'd1 : 4'd0);
				data_send = {data,crc};
				state = TRIGGER_SEND;
				current_sensor = current_sensor+1;
				if(current_sensor>=NUMBER_OF_SENSORS) begin
					current_sensor = 0;
				end
			end
			TRIGGER_SEND: begin		
				trigger <= 1;
				state <= WAIT_FOR_DATA;
			end
			WAIT_FOR_DATA: begin
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
						data_valid = 1;
						if(sample_counter[current_sensor]<SAMPLES_TO_AVERAGE)begin
							angle_filtered[current_sensor] = angle_filtered[current_sensor]+(data_received>>4);
							sample_counter <= sample_counter+1;
						else
							sample_counter[current_sensor] <= 0;
							angle[current_sensor] = angle_filtered[current_sensor]>>>$clog2(SAMPLES_TO_AVERAGE);
							angle_filtered[current_sensor] = 0;
						end
					end else begin
						angle[current_sensor] = 0;
						data_valid = 0;
					end
					
					if(NUMBER_OF_SENSORS==1) begin
						state = DELAY;
						delay_counter <= 100;
					end else begin
						state = IDLE;
					end
					
				end
			end
			DELAY: begin
				delay_counter <= delay_counter -1;
				if(delay_counter==0) begin
					state <= IDLE;
				end
			end
			default: state = IDLE;
		endcase
	end
end

genvar k;
generate 
	for(k=0; k<NUMBER_OF_SENSORS; k = k+1) begin : assign_slave_select
		assign ss_n_o[k] = (current_sensor==k?ss_n:1);
	end
endgenerate 

spi_master #(20, 1'b1, 1'b1, 2, 3) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(~reset_n),
	.spi_miso_i(miso_i),
	.di_i(data_send),
	.wren_i(trigger),
	.spi_ssel_o(ss_n),
	.spi_sck_o(sck_o), 
	.spi_mosi_o(mosi_o),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_received)
);

endmodule