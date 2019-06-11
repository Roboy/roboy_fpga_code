// neopixel transmitter

`timescale 1ns/10ps

module neopixel (
	input clock,
	input reset,
	// this is for the avalon interface
	input [7:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	output reg one_wire
);

assign readdata = 
	((address == 0))? send_to_neopixels:
	32'hDEAD_BEEF;

parameter CLOCK_SPEED_HZ = 50_000_000;
parameter NUMBER_OF_NEOPIXEL = 35;
parameter RGBW = 1;

reg send_to_neopixels;

always @(posedge clock, posedge reset) begin: NEOPIXEL_CONTROL_LOGIC
	reg spi_done_prev; 
	if (reset == 1) begin
		send_to_neopixels <= 0;
	end else begin
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if(address==0) begin
				send_to_neopixels <= (writedata[31:0]!=0); 
			end else begin
				if(address<NUMBER_OF_NEOPIXEL+1) begin
					color[address-1] <= writedata[31:0];
				end
			end
		end
		if(send_to_neopixels && bit_ctr<(24*NUMBER_OF_NEOPIXEL)) begin
			send_to_neopixels <= 0;
		end
	end 
end

wire [31:0] timer;

Counter counter(
	.clk(clock),
	.reset(reset),
	.counter(timer)
);


reg unsigned [3:0] state;
wire unsigned [3:0] next_state;
reg [31:0] bit_ctr;
wire color_bit;
reg start;

reg [23:0] color[NUMBER_OF_NEOPIXEL-1:0];
assign color_bit = color[bit_ctr/24][bit_ctr%24];
assign waitrequest = bit_ctr<(24*NUMBER_OF_NEOPIXEL);

generate
	if(RGBW==0) begin		
		always @(posedge clock, posedge reset) begin: neo_pixel_transmitter
			parameter SIZE = 4;
			parameter SEND_0  = 0, SEND_1 = 1,LATCH = 2, IDLE = 3;
			reg done;
			reg [31:0] t0;
			reg [31:0] t1;
			if(reset==1) begin
				start <= 1;
				done <= 0;
				state <= 3;
				bit_ctr <= 24*NUMBER_OF_NEOPIXEL;
			end else begin
				case(state)
					IDLE : 	begin
									if(bit_ctr<(24*NUMBER_OF_NEOPIXEL)) begin
										state <= (color_bit?SEND_1:SEND_0);
									end else begin
										state <= LATCH;
										if(send_to_neopixels) begin
											bit_ctr <= 0;
										end
									end
								end
					SEND_1 : begin 
									if (start == 1) begin
										t0 <= timer;
										start <= 0;
										done <= 0;
									end else begin
										t1 = timer - t0;
										if( done == 0 ) begin 
											if( t1 < CLOCK_SPEED_HZ/1111111 ) begin // 0.9 us approx 1111111 Hz
												one_wire <= 1;
											end else begin
												done <= 1;
												t0 <= timer;
											end
										end else begin
											if( t1 < CLOCK_SPEED_HZ/3333333 ) begin // 0.3 us approx 3333333 Hz
												one_wire <= 0;
											end else begin
												done <= 0;
												start <= 1;
												t0 <= timer;
												state <= IDLE;
												bit_ctr <= bit_ctr+1;
											end
										end
									end
								end
					SEND_0 : begin
									if (start == 1) begin
										t0 <= timer;
										start <= 0;
										done <= 0;
									end else begin
										t1 = timer - t0;
										if( done == 0 ) begin 
											if( t1 < CLOCK_SPEED_HZ/3333333 ) begin // 0.3 us approx 3333333 Hz
												one_wire <= 1;
											end else begin
												done <= 1;
												t0 <= timer;
											end
										end else begin
											if( t1 < CLOCK_SPEED_HZ/1111111 ) begin // 0.9 us approx 1111111 Hz
												one_wire <= 0;
											end else begin
												done <= 0;
												start <= 1;
												t0 <= timer;
												state <= IDLE;
												bit_ctr <= bit_ctr+1;
											end
										end
									end
								end
					LATCH : begin
									t1 = timer - t0;
									if( done == 0 ) begin 
										if( t1 < CLOCK_SPEED_HZ/12500 ) begin // 80 us approx 12500 Hz
											one_wire <= 0;
										end else begin
											done <= 0;
											start <= 1;
											state <= IDLE;
										end
									end
								end
				  default : begin
									state <= IDLE;
								end
				endcase
			end
		end
	end else begin
	// TODO
	end
endgenerate
endmodule