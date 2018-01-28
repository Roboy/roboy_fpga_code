// SpiControl_esp8266
// this is a helper module for sending data via SPI to the ESP8266

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

module SpiControl_esp8266 (
	input clock, // clock -- input clock
	input [(8*32)-1:0] data, // data -- input data, 256 bit
	input dataReady, // dataReady -- signal data is ready for transmission, starts transmission of 256 bit field
	input reset_n, // reset 
	input di_req, // di_req signal from spi core, signals when next data byte can be written
	input write_ack, // write acknowledge
	output reg [7:0] data_byte, // this will be send next
	output reg wren // triggers send via spi
);

reg [7:0] numberOfBytesTransmitted;
reg write_ack_prev;

always @(posedge clock, negedge reset_n) begin: SPICONTROL_ESP8266_SPILOGIC
    if (reset_n == 0) begin
			numberOfBytesTransmitted <= 0;
			wren <= 0;
			write_ack_prev <= 0;
    end
	 
    else begin
			
			write_ack_prev <= write_ack;
			if( write_ack_prev==0 && write_ack == 1 ) begin
				wren <= 0;
				numberOfBytesTransmitted <= numberOfBytesTransmitted + 1;
			end
			
			if(di_req==1 && numberOfBytesTransmitted<34) begin
				case(numberOfBytesTransmitted)
					1: data_byte <= 0; // this is the address we want to write to on the esp(we start at 0)
					// sensor0
					2: data_byte <= data[7:0];
					3: data_byte <= data[15:8];
					4: data_byte <= data[23:16];
					5: data_byte <= data[31:24];
					// sensor1
					6: data_byte <= data[39:32];
					7: data_byte <= data[47:40];
					8: data_byte <= data[55:48];
					9: data_byte <= data[63:56];
					// sensor3
					10: data_byte <= data[71:64];
					11: data_byte <= data[79:72];
					12: data_byte <= data[87:80];
					13: data_byte <= data[95:88];
					// sensor4
					14: data_byte <= data[103:96];
					15: data_byte <= data[111:104];
					16: data_byte <= data[119:112];
					17: data_byte <= data[127:120];
					// sensor5
					18: data_byte <= data[135:128];
					19: data_byte <= data[143:136];
					20: data_byte <= data[153:144];
					21: data_byte <= data[159:152];
					// sensor6
					22: data_byte <= data[167:160];
					23: data_byte <= data[175:168];
					24: data_byte <= data[183:176];
					25: data_byte <= data[191:184];
					// sensor7
					26: data_byte <= data[199:192];
					27: data_byte <= data[209:200];
					28: data_byte <= data[215:208];
					29: data_byte <= data[223:216];
					// sensor8
					30: data_byte <= data[231:224];
					31: data_byte <= data[239:232];
					32: data_byte <= data[247:240];
					33: data_byte <= data[255:248];
					default: data_byte <= numberOfBytesTransmitted-2;
				endcase
				wren <= 1;
			end
			
			if (dataReady==1) begin
				numberOfBytesTransmitted<= 0;
				data_byte <= 2;
				wren <= 1;
			end
    end
end

endmodule
