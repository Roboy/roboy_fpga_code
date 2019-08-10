// SpiControl 
// This module handles the myorobotics specific frame structure by orchestrating the 
// incoming spi data. Myorobotics motorboards are sending the data big-endian.
//
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

module SpiControl (
   input clock,
	input reset,
	input di_req,
	input write_ack,
	input data_read_valid,
	input [15:0] data_read,
	input start,
	input signed [15:0] pwmRef,
	input [15:0] controlFlag,
	input ss_n,
	output reg [15:0] Word,
	output reg wren,
	output reg spi_done,
	output reg signed[31:0] position,
	output reg signed[15:0] velocity,
	output reg signed[15:0] current,
	output reg signed[31:0] displacement,
	output reg signed[15:0] sensor1
);

reg [7:0] numberOfWordsTransmitted;
reg [7:0] numberOfWordsReceived;
reg write_ack_prev;
reg next_value;
reg start_frame;
reg data_read_valid_prev;
reg [7:0] delay_counter;

`define ENABLE_DELAY

localparam SPI_FRAME_WORDS = 10;

always @(posedge clock, posedge reset) begin: SPICONTROL_SPILOGIC
	if (reset == 1) begin
		numberOfWordsTransmitted <= SPI_FRAME_WORDS;
		wren <= 0;
		write_ack_prev <= 0;
		start_frame <= 0;
		spi_done <= 0;
	end else begin
		write_ack_prev <= write_ack;
		// if the write is acknowledged we increment our counter and trigger the sending of the next word by asserting next_value
		if( write_ack_prev==0 && write_ack == 1) begin
			wren <= 0;
			numberOfWordsTransmitted <= numberOfWordsTransmitted + 1;
			next_value <= 1;
		end
		
		// if it's the start of a frame or we are trigger by di_reg to send the next value.
		// of course we stop if everything was sent
		if( (di_req || start_frame) && numberOfWordsTransmitted<SPI_FRAME_WORDS && next_value==1) begin
			case(numberOfWordsTransmitted)
				0: Word <= 16'h8000;
				1: Word <= pwmRef;
				2: Word <= controlFlag;
				default: Word <= numberOfWordsTransmitted;
			endcase
			// reset next_value
			next_value <= 0;
			
`ifdef ENABLE_DELAY
			// start delay counter
			delay_counter <= 1;
`else
			// trigger transmission with wren
			wren <= 1;
`endif
			// reset start_frame
			if(start_frame)
				start_frame <= 0;
		end
		
`ifdef ENABLE_DELAY
		if(wren==0 && next_value==0) begin // this adds a delay of 64/50 approx. 1.28us
			if(delay_counter==0)
				wren <= 1;
			else if (delay_counter>0)
				delay_counter <= delay_counter + 1;
		end
`endif /*ENABLE_DELAY*/
		
		data_read_valid_prev <= data_read_valid;
		// if data_read_valid goes high, we can put the received data into correspondig register
		if( data_read_valid_prev==1 && data_read_valid==0 ) begin
			// the first received word we are interested in is word 3, but since we increment afterwards it is 2
			case(numberOfWordsReceived)
				3: position[15:0] <= data_read;
				4: position[31:16] <= data_read;
				5: velocity <= data_read;
				6: current <= data_read;
				7: displacement[15:0] <= data_read;
				8: displacement[31:16] <= data_read;
				9: sensor1 <= data_read;
			endcase
			numberOfWordsReceived <= numberOfWordsReceived + 1;
		end
		
		// if all data was transmitted and slaveslect is high...again
		if ( numberOfWordsTransmitted>=SPI_FRAME_WORDS && ss_n==1 ) begin 		
			spi_done <= 1;
			// start transmission of next spi frame
			if ( start ) begin
				// reset the amount of data transmitted an received
				numberOfWordsTransmitted<= 0;
				numberOfWordsReceived <= 0;
				start_frame <= 1;
				next_value <= 1;
				spi_done <= 0;
			end
		end 
	end
end

endmodule
