// I2C_avalon_bridge node
// This module implements a i2c communication node.
// Through the lightweight axi bridge, the following values can be READ
//	address            -----   [type] value
// [3'h00]                    [uint32] addr - address of i2c device slave to be accessed
// [3'h01]                    [uint32] data_read_fifo - 32-bit chunks of data read from device
// [3'h02]                    [bool] rw - read =1 write = 0
// [3'h03]                    [bool] ena - enable, starts transmission
// [3'h04]                    [bool] busy - true is i2c transmission is not finished
// [3'h05]                    [bool] ack_error - true if slave did not acknowledge
// [3'h06]                    [uint8] usedw - amount of 32-bit chunks stored in fifo
//
// Through the lightweight axi bridge, the following values can be WRITTEN
//	address            -----   [type] value
// [3'h00]                    [uint32] addr - address of i2c device to be accessed
// [3'h01]                    [uint32] data_wd - data to be written [31:24 register, 23:0 data]
// [3'h02]                    [bool] rw - read =1 write = 0
// [3'h03]                    [bool] ena - enable, starts transmission
// [3'h04]                    [uint8] number_of_bytes - that many bytes shall be transmitted
// [3'h05]                    [uint8] gpio_set - lowest 3 bit control gpios
// 															if gpio_set[3] is set, we pull sda low
//											 					if gpio_set[4] is set, we pull sda high
// [3'h06]                    [bool] read_only - if true, the core will simply start reading from slave,
//																without transmitting a register address
// Features:
//  * complete control via lightweight axi bridge
//  * continuous read using a fifo
//  * specialized sda, scl, gpio control for Infineon 3d magnetic sensor configuration

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

module I2C_avalon_bridge (
	input clock,
	input reset,
	// this is for the avalon interface
	input [3:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// these are the i2c ports
	inout scl,
	inout sda
);

parameter CLOCK_SPEED_HZ = 50_000_000;
parameter BUS_SPEED_HZ = 100_000;

reg [6:0] addr;
reg rw;
reg busy;
reg ack_error;
reg ena;
reg [7:0] number_of_bytes;
wire [7:0] byte_counter;
reg busy_prev;
reg [31:0] data_rd;
reg [31:0] data_read_fifo;
reg [31:0] data_wd;

reg [4:0] gpio_set;
reg read_only;

reg [7:0] read_counter;

assign readdata = 
	((address == 0))? addr :
	((address == 1))? data_read_fifo :
	((address == 2))? rw :
	((address == 3))? ena :
	((address == 4))? busy :
	((address == 5))? ack_error :
	((address == 6))? usedw :
	32'hDEAD_BEEF;
	
always @(posedge clock, posedge reset) begin: I2C_CONTROL_LOGIC
	reg ena_prev;
	reg [7:0] i;
	reg [15:0] delay_counter;
	if (reset == 1) begin 
		data_wd <= 0;
		ena <= 0;
		read_only <= 0;
		gpio_set <= 0;
		read_only <= 0;
		number_of_bytes<= 0;
		i<=0;
	end else begin
		ena_prev <= ena;
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			case(address)
				0: addr <= writedata; 
				1: data_wd <= writedata; 
				2: rw <= writedata; 
				3: ena <= writedata;
				4: number_of_bytes <= writedata;
				5: gpio_set <= writedata[4:0];
				6: read_only <= (writedata!=0); 
			endcase 
		end
		if(read && ~waitrequest && address==1 && ~fifo_empty) begin
			fifo_read_ack <= 1;
		end
		
		if(byte_counter>=number_of_bytes) begin
			ena <= 0;
		end
		
		if(fifo_read_ack==1) begin
			fifo_read_ack <= 0;
		end
		
		if(ena_prev == 0 && ena == 1 && ~fifo_empty) begin
			fifo_clear <= 1;
		end
		
		if(fifo_clear == 1) begin
			fifo_clear <= 0;
		end
	end 
end

//assign sda = (gpio_set[3]==1)?0: // if gpio_set[3] is set, we pull sda low
//				 (gpio_set[4]==1)?1: // if gpio_set[4] is set, we pull sda high
//				 1'bz;					// else we leave it the fuck alone

// if i2c node is busy we have to wait
assign waitrequest = ena|fifo_read_ack ;

wire fifo_write;
reg read_fifo;
reg write_fifo;
wire fifo_write_ack;
reg fifo_read_ack;
reg fifo_clear;
wire fifo_empty;
wire fifo_full;
reg [7:0] usedw;

fifo fifo(
	.clock(clock),
	.data(data_rd),
	.rdreq(fifo_read_ack),
	.sclr(reset||fifo_clear),
	.wrreq(fifo_write),
	.q(data_read_fifo),
	.empty(fifo_empty),
	.full(fifo_full),
	.usedw(usedw)
);

oneshot oneshot(
	.clk(clock),
   .edge_sig(fifo_write_ack),
   .level_sig(fifo_write)
);

i2c_master #(CLOCK_SPEED_HZ, BUS_SPEED_HZ) i2c(
	.clk(clock),
	.reset_n(~reset),
	.ena(ena),
	.addr(addr),
	.rw(rw),
	.data_wr(data_wd),
	.busy(busy),
	.data_rd(data_rd),
	.ack_error(ack_error),
	.sda(sda),
	.scl(scl),
	.byte_counter(byte_counter),
	.read_only(read_only),
	.number_of_bytes(number_of_bytes),
	.fifo_write_ack(fifo_write_ack)
);

endmodule