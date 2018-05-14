// DarkRoom 
// This module implements signal decoders for HTC Vive lighthouse tracking. 
// Optionally it adds a SPI core which triggers transmission if any sensor sees a non-skipping lighthouse.
// The SPI frame is compatible with ESP8266 spi and transmits a frame of 256 bits containing 8 decoded
// sensor values

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

module DarkRoom(
	input clock,
	input reset_n,
	// this is for the avalon interface
	input [8:0] address,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// those are the sensor data lines
	inout [NUMBER_OF_SENSORS-1:0]D_io,
	// those are the sensor envelope lines
	inout [NUMBER_OF_SENSORS-1:0]E_io,
	// this is a debug connection(triggers SPI transmission when there are no sensors connected)
	input trigger_me,
	output [NUMBER_OF_SENSORS-1:0]sync_o,
	// SPI
	output sck_o, // clock
	output ss_n_o, // slave select
	output mosi_o	// mosi
);

parameter ENABLE_AVALON_INTERFACE = 1;
parameter ENABLE_SPI_TRANSMITTER = 0;
parameter NUMBER_OF_SENSORS = 8 ;
parameter CLK_SPEED = 50_000_000 ;

generate 
if(ENABLE_AVALON_INTERFACE!=0) begin
	assign readdata = sensor_data_avalon;
	assign waitrequest = waitFlag;
	reg [31:0] sensor_data_avalon;
	reg waitFlag;

	always @(posedge clock, negedge reset_n) begin: AVALON_READ_INTERFACE
		if (reset_n == 0) begin
			waitFlag <= 0;
		end else begin
			waitFlag <= 1;
			if(read) begin	
				if(address[8]==0) begin // return sensor data
					case(address%8)
						0: sensor_data_avalon <= sensor_data[address/8][31:0];
						1: sensor_data_avalon <= sensor_data[address/8][63:32];
						2: sensor_data_avalon <= sensor_data[address/8][95:64];
						3: sensor_data_avalon <= sensor_data[address/8][127:96];
						4: sensor_data_avalon <= sensor_data[address/8][159:128];
						5: sensor_data_avalon <= sensor_data[address/8][191:160];
						6: sensor_data_avalon <= sensor_data[address/8][223:192];
						7: sensor_data_avalon <= sensor_data[address/8][255:224];
						default: sensor_data_avalon <= 0;
					endcase
				end else begin // return sensor state
					sensor_data_avalon <= sensor_states[address[7:0]];
				end
				if(waitFlag==1) begin // after one clock cycle the sensor_data_avalon should be stable
					waitFlag <= 0;
				end
			end
		end
	end
end
endgenerate 

localparam NUMBER_OF_SPI_FRAMES = (NUMBER_OF_SENSORS+8-1)/8; // ceil division to get eg 2 frames when using 15 sensors

reg [255:0]sensor_data[NUMBER_OF_SPI_FRAMES-1:0] ;
reg [NUMBER_OF_SENSORS-1:0] sync;
//wire [2:0] sensor_states[NUMBER_OF_SENSORS-1:0];
//wire [3:0] current_states[NUMBER_OF_SENSORS-1:0];
assign sync_o = sync;

genvar i,sensor_frame,sensor_counter;
generate 
	for(i=0; i<NUMBER_OF_SENSORS; i = i+1) begin : instantiate_lighthouse_sensors
		localparam integer sensor_frame = i/8;
		localparam integer sensor_counter = i%8;
		localparam unsigned [9:0]sensor_id = i;
		lighthouse_sensor #(sensor_id) lighthouse_sensors(
			.clk(clock),
			.sensor((~E_io[i]) && sensor_states[i]==3'b001), // activate envelope line when sensor is in watch state
			.combined_data(sensor_data[sensor_frame][32*(sensor_counter+1)-1:32*sensor_counter]),
			.sync(sync[i])
		);
	end
endgenerate 

reg [2:0] sensor_states[NUMBER_OF_SENSORS-1:0];
reg [3:0] current_states[NUMBER_OF_SENSORS-1:0];
wire D;
wire E;
assign D = (sensor_to_configure<NUMBER_OF_SENSORS)?D_io[sensor_to_configure]:1'bz;
assign E = (sensor_to_configure<NUMBER_OF_SENSORS)?E_io[sensor_to_configure]:1'bz;

wire [2:0] sensor_state;
wire [3:0] current_state;

ts4231 #(CLK_SPEED) sensor(
	.clk(clock),
	.rst(reset_ts4231_config),
	.D(D),
	.E(E),
	.sensor_STATE(sensor_state),
	.current_STATE(current_state)
);

reg reset_ts4231_config;
reg [7:0] sensor_to_configure;

always @(posedge clock, negedge reset_n) begin: TS4231_INIT_INTERFACE
	reg [31:0] timeout;
	if (reset_n == 0) begin
		timeout <= 0;
		sensor_to_configure<=0;
		reset_ts4231_config <= 1; 
	end else begin
		reset_ts4231_config <= 0;
		if(timeout>0) begin
			timeout <= timeout - 1;
		end else begin
			sensor_states[sensor_to_configure] <= sensor_state;
			current_states[sensor_to_configure] <= current_state;
			if(sensor_to_configure<NUMBER_OF_SENSORS-1) begin
				sensor_to_configure <= sensor_to_configure + 1;
			end else begin
				sensor_to_configure <= 0;
			end
			reset_ts4231_config <= 1;
			timeout <= CLK_SPEED/10; // 100ms timeout
		end
	end
end

generate
	if(ENABLE_SPI_TRANSMITTER!=0) begin
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
								if(trigger_me||(|sync)) begin // if trigger me or if any sensor detects a non-skipping sweep 
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
	end

endgenerate 


endmodule

