// This module instantiates NUMBER_OF_SENSORS lighthouse sensors

`timescale 1ns/10ps

module DarkRoom(
	input clock,
	input reset_n,
	// those are the sensor signals
	input [NUMBER_OF_SENSORS-1:0]sensor_signals_i,
	// this is a debug connection(triggers SPI transmission when there are no sensors connected)
	input trigger_me,
	// SPI
	output sck_o, // clock
	output ss_n_o, // slave select
	output mosi_o	// mosi
);

parameter NUMBER_OF_SENSORS = 8 ;
localparam NUMBER_OF_SPI_FRAMES = (NUMBER_OF_SENSORS+8-1)/8; // ceil division to get eg 2 frames when using 15 sensors

reg [255:0]sensor_data[NUMBER_OF_SPI_FRAMES-1:0] ;
reg [NUMBER_OF_SENSORS-1:0] sync;

genvar i,sensor_frame,sensor_counter;
generate 
	for(i=0; i<NUMBER_OF_SENSORS; i = i+1) begin : instantiate_lighthouse_sensors
	  localparam integer sensor_frame = i/8;
	  localparam integer sensor_counter = i%8;
	  localparam unsigned [9:0]sensor_id = i;
	  lighthouse_sensor #(sensor_id) lighthouse_sensors(
		.clk(clock),
		.sensor(sensor_signals_i[i]),
		.combined_data(sensor_data[sensor_frame][32*(sensor_counter+1)-1:32*sensor_counter]),
		.sync(sync[i])
	  );
	end
endgenerate 

wire wr_ack, wren, di_req;
reg trigger;
wire [255:0] spi_frame;
wire [7:0]data;

SpiControl_esp8266 spi_control_esp8266(
	.clock(clock),
	.data(spi_frame),
	.dataReady(trigger), // triggers transmission if any sensor sees a non-skipping lighthouse
	.reset_n(reset_n),
	.write_ack(wr_ack),
	.di_req(di_req),
	.data_byte(data),
	.wren(wren)
);

spi_master #(8, 1'b0, 1'b0, 2, 5) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(~reset_n),
	.wren_i(wren),
	.di_req_o(di_req),
	.spi_ssel_o(ss_n_o),
	.spi_sck_o(sck_o),
	.spi_mosi_o(mosi_o),
	.wr_ack_o(wr_ack),
	.di_i(data)
);


reg [3:0] sensor_frame_counter;
assign spi_frame = sensor_data[sensor_frame_counter];
reg [9:0] delay_counter;

always @(posedge clock, negedge reset_n) begin: SPI_DATA_MUX
	parameter IDLE  = 2'b00, TRIGGER_SEND = 2'b01, WAIT_FOR_NEXT_FRAME = 2'b10, DELAY = 2'b11;
	reg [1:0] mux_state;
	reg ss_n_prev;
	if (reset_n == 0) begin
		mux_state <= IDLE;
	end else begin
		trigger <= 0; 
		ss_n_prev <= ss_n_o;
		case(mux_state)
			IDLE: begin
						if(trigger_me|| (|sync)) begin // if trigger me or if any sensor detects a non-skipping sweep 
							mux_state <= TRIGGER_SEND;
							sensor_frame_counter <= 0;
						end
					end
			TRIGGER_SEND: begin		
						if(sensor_frame_counter< NUMBER_OF_SPI_FRAMES) begin // each SPI frame contains 8 sensors
							trigger <= 1;
							mux_state <= WAIT_FOR_NEXT_FRAME;
						end else begin
							mux_state <= IDLE;
						end
					end
			
			WAIT_FOR_NEXT_FRAME: begin
						if(ss_n_prev==0 && ss_n_o==1) begin // if the frame is done, go to next frame
							delay_counter<= 1;
							mux_state <= DELAY;
						end
						
					end
			DELAY: begin
						delay_counter <= delay_counter+1;
						if(delay_counter==0) begin
							sensor_frame_counter <= sensor_frame_counter+1;
							mux_state <= TRIGGER_SEND;
						end
			end
			
			default: mux_state <= IDLE;
		endcase
		
	end
end

endmodule

