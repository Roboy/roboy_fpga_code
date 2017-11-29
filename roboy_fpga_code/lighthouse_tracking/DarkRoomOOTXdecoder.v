// DarkRoom ootx decoder
// author: Simon Trendel (simon.trendel@tum.de)
module DarkRoomOOTXdecoder (
	input clock,
	input reset,
	// this is for the avalon interface
	input [5:0] address,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// uart tx port
	output uart_tx,
	input sensor,
	output [7:0] led
);

assign readdata = 
	((address == 0))? ootx_payload_o[0][15:0] : // fw_version
	((address == 1))? ootx_payload_o[0][47:16] : // ID
	((address == 2))? ootx_payload_o[0][63:48] : // fcal.0.phase
	((address == 3))? ootx_payload_o[0][79:64] : // fcal.1.phase
	((address == 4))? ootx_payload_o[0][95:80] : // fcal.0.tilt
	((address == 5))? ootx_payload_o[0][95:80] : // fcal.1.tilt
	((address == 6))? ootx_payload_o[0][119:112] : // sys.unlock_count
	((address == 7))? ootx_payload_o[0][127:120] : // hw_version
	((address == 8))? ootx_payload_o[0][143:128] : // fcal.0.curve
	((address == 9))? ootx_payload_o[0][159:144] : // fcal.1.curve
	((address == 10))? ootx_payload_o[0][183:160] : // accel.dir_xyz
	((address == 11))? ootx_payload_o[0][199:184] : // fcal.0.gibphase
	((address == 12))? ootx_payload_o[0][215:200] : // fcal.1.gibphase
	((address == 13))? ootx_payload_o[0][231:216] : // fcal.0.gibmag
	((address == 14))? ootx_payload_o[0][247:232] : // fcal.1.gibmag
	((address == 15))? ootx_payload_o[0][255:248] : // mode.current
	((address == 16))? ootx_crc32_o[0][31:0] : // crc32
	((address == 17))? ootx_payload_o[1][15:0] : // fw_version
	((address == 18))? ootx_payload_o[1][47:16] : // ID
	((address == 19))? ootx_payload_o[1][63:48] : // fcal.0.phase
	((address == 20))? ootx_payload_o[1][79:64] : // fcal.1.phase
	((address == 21))? ootx_payload_o[1][95:80] : // fcal.0.tilt
	((address == 22))? ootx_payload_o[1][95:80] : // fcal.1.tilt
	((address == 23))? ootx_payload_o[1][119:112] : // sys.unlock_count
	((address == 24))? ootx_payload_o[1][127:120] : // hw_version
	((address == 25))? ootx_payload_o[1][143:128] : // fcal.0.curve
	((address == 26))? ootx_payload_o[1][159:144] : // fcal.1.curve
	((address == 27))? ootx_payload_o[1][183:160] : // accel.dir_xyz
	((address == 28))? ootx_payload_o[1][199:184] : // fcal.0.gibphase
	((address == 29))? ootx_payload_o[1][215:200] : // fcal.1.gibphase
	((address == 30))? ootx_payload_o[1][231:216] : // fcal.0.gibmag
	((address == 31))? ootx_payload_o[1][247:232] : // fcal.1.gibmag
	((address == 32))? ootx_payload_o[1][255:248] : // mode.current
	((address == 33))? ootx_crc32_o[1][31:0] : // crc32
	32'hDEAD_BEEF;

assign waitrequest = |sync; // if we are syncing, the frame is updated, so we should wait for sync to go low
	
wire [1:0] sync;
wire uart_done;
reg [264:0] ootx_payload_o[1:0];
reg [31:0] ootx_crc32_o[1:0];
reg [15:0] ootx_payload_length_o[1:0];

lighthouse_ootx_decoder ootx_decoder(
	.clk(clock),
	.reset(reset),
	.sensor(sensor),
	.led(led),
	.sync(sync),
	.ootx_crc32_1_o(ootx_crc32_o[0]),
	.ootx_crc32_2_o(ootx_crc32_o[1]),
	.ootx_payload_length_1_o(ootx_payload_length_o[0]),
	.ootx_payload_length_2_o(ootx_payload_length_o[1]),
	.ootx_payload_1_o(ootx_payload_o[0]),
	.ootx_payload_2_o(ootx_payload_o[1])
);

reg trigger_send;
reg [0:7] data_byte;
reg lighthouse;

reg [8:0] byte_counter;
always @(posedge clock) begin: UART_TRANSMISSION
	
	reg transmit;
	reg [1:0] uart_state;
	parameter IDLE  = 2'b00, TRIGGER_SEND = 2'b01, WAIT_FOR_NEXT_BYTE = 2'b10;
	
	trigger_send <= 0;
	case(uart_state) 
		IDLE: begin
					if(sync) begin // if trigger me or successfull ootx decoding 
						uart_state <= TRIGGER_SEND;
						byte_counter <= 0;
						if(sync[0]==1)
							lighthouse<=0;
						if(sync[1]==1)
							lighthouse<=1;
					end
				end
		TRIGGER_SEND: begin		
					if(byte_counter< 37) begin
						case(byte_counter)
							// fw_version
							0: data_byte <= ootx_payload_o[lighthouse][7:0];
							1: data_byte <= ootx_payload_o[lighthouse][15:8];
							// ID
							2: data_byte <= ootx_payload_o[lighthouse][23:16];
							3: data_byte <= ootx_payload_o[lighthouse][31:24];
							4: data_byte <= ootx_payload_o[lighthouse][39:32];
							5: data_byte <= ootx_payload_o[lighthouse][47:40];
							// fcal.0.phase
							6: data_byte <= ootx_payload_o[lighthouse][55:48];
							7: data_byte <= ootx_payload_o[lighthouse][63:56];
							// fcal.1.phase
							8: data_byte <= ootx_payload_o[lighthouse][71:64];
							9: data_byte <= ootx_payload_o[lighthouse][79:72];
							// fcal.0.tilt
							10: data_byte <= ootx_payload_o[lighthouse][87:80];
							11: data_byte <= ootx_payload_o[lighthouse][95:88];
							// fcal.1.tilt
							12: data_byte <= ootx_payload_o[lighthouse][103:96];
							13: data_byte <= ootx_payload_o[lighthouse][111:104];
							// sys.unlock_count
							14: data_byte <= ootx_payload_o[lighthouse][119:112];
							// hw_version
							15: data_byte <= ootx_payload_o[lighthouse][127:120];
							// fcal.0.curve
							16: data_byte <= ootx_payload_o[lighthouse][135:128];
							17: data_byte <= ootx_payload_o[lighthouse][143:136];
							// fcal.1.curve
							18: data_byte <= ootx_payload_o[lighthouse][151:144];
							19: data_byte <= ootx_payload_o[lighthouse][159:152];
							// accel.dir_x
							20: data_byte <= ootx_payload_o[lighthouse][167:160];
							// accel.dir_y
							21: data_byte <= ootx_payload_o[lighthouse][175:168];
							// accel.dir_z
							22: data_byte <= ootx_payload_o[lighthouse][183:176];
							// fcal.0.gibphase
							23: data_byte <= ootx_payload_o[lighthouse][191:184];
							24: data_byte <= ootx_payload_o[lighthouse][199:192];
							// fcal.1.gibphase
							25: data_byte <= ootx_payload_o[lighthouse][207:200];
							26: data_byte <= ootx_payload_o[lighthouse][215:208];
							// fcal.0.gibmag
							27: data_byte <= ootx_payload_o[lighthouse][223:216];
							28: data_byte <= ootx_payload_o[lighthouse][231:224];
							// fcal.1.gibmag
							29: data_byte <= ootx_payload_o[lighthouse][239:232];
							30: data_byte <= ootx_payload_o[lighthouse][247:240];
							// mode.current
							31: data_byte <= ootx_payload_o[lighthouse][255:248];
							// sys.faults
							32: data_byte <= ootx_payload_o[lighthouse][263:256];
							// crc32
							33: data_byte <= ootx_crc32_o[lighthouse][7:0];
							34: data_byte <= ootx_crc32_o[lighthouse][15:8];
							35: data_byte <= ootx_crc32_o[lighthouse][23:16];
							36: data_byte <= ootx_crc32_o[lighthouse][31:24];
							default: data_byte <= 0; 
						endcase
						trigger_send <= 1;
						uart_state <= WAIT_FOR_NEXT_BYTE;
					end else begin
						uart_state <= IDLE; 
					end
				end
		
		WAIT_FOR_NEXT_BYTE: begin
					if(uart_done) begin // if the byte is done, go to next byte
						uart_state <= TRIGGER_SEND;
						byte_counter <= byte_counter+1;
					end
				end
		default: uart_state <= IDLE;
	endcase
end 

// big endian uart transmitter
uart_tx #(434) uart(
	.i_Clock(clock),
	.i_Tx_DV(trigger_send),
	.i_Tx_Byte(data_byte),
	.o_Tx_Done(uart_done),
	.o_Tx_Serial(uart_tx)
);

endmodule