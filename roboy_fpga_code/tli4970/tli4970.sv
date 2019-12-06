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

// author: Simon Trendel, simon.trendel@tum.de, 19

`timescale 1ns/10ps

module TLI4970 (
	input clock,
	input reset,
	// this is for the avalon interface
	input [7:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// these are the spi ports
	output [NUMBER_OF_SENSORS-1:0] ss_n_o,
	input miso,
	output sck
);

parameter NUMBER_OF_SENSORS = 2;
parameter CLOCK_SPEED_HZ = 50_000_000;
parameter UPDATE_FREQUENCY = 1_000;

assign readdata = returnvalue;
assign waitrequest = (waitFlag && read);
reg [31:0] returnvalue;
reg waitFlag;

reg [31:0] update_frequency;
reg [31:0] actual_update_frequency;
reg [31:0] delay_counter;

reg signed [31:0] current[NUMBER_OF_SENSORS-1:0];

always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	if (reset == 1) begin
		waitFlag <= 1;
	end else begin
		waitFlag <= 1;
		if(read) begin
			if(address<NUMBER_OF_SENSORS) begin
				returnvalue <= current[address];
			end else begin
				returnvalue <= 32'hdeadbeef;
			end
			
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end
	
integer current_sensor;
	
always @(posedge clock, posedge reset) begin: TLI4970_READOUT_LOGIC
	if(reset)begin
		current_sensor<=0;
		delay_counter<=0;
	end else begin
		wren <= 0;
		if(delay_counter==0)begin
			if(ss_n)begin
				if(data_out[15]==0)begin
					current[current_sensor] <= data_out[12:0];
				end
				if(current_sensor<NUMBER_OF_SENSORS-1)begin
					current_sensor <= current_sensor+1;
				end else begin
					current_sensor <= 0;
				end
				delay_counter <= CLOCK_SPEED_HZ/UPDATE_FREQUENCY/NUMBER_OF_SENSORS/2;
				wren <= 1;
			end 
		end else begin
			delay_counter <= delay_counter-1;
		end
	end
end

wire di_req, wr_ack, do_valid, wren, ss_n;
wire [15:0] Word;
wire [15:0] data_out;

// SPI specs: 1MHz, 16bit MSB, pol 0 phase 0
spi_master #(16, 1'b0, 1'b1, 2, 25) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(1'b0),
	.spi_miso_i(miso),
	.di_i(Word),
	.wren_i(wren),
	.spi_ssel_o(ss_n),
	.spi_sck_o(sck),
	.spi_mosi_o(mosi),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_out)
);

genvar j;
generate 
	for(j=0; j<NUMBER_OF_SENSORS; j = j+1) begin : connect_ss_n
		assign ss_n_o[j] = (current_sensor==j?ss_n:1);
	end
endgenerate 

endmodule