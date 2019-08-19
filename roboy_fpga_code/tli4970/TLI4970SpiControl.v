`timescale 1ns/10ps

module TLI4970SpiControl (
   input clock,
	input reset,
	input di_req,
	input write_ack,
	input data_read_valid,
	input [15:0] data_read,
	input start,
	input ss_n,
	output reg wren,
	output reg spi_done,
	output reg signed[15:0] current
);

reg [7:0] numberOfWordsTransmitted;
reg [7:0] numberOfWordsReceived;
reg write_ack_prev;
reg next_value;
reg start_frame;
reg data_read_valid_prev;
reg [7:0] delay_counter;

`define ENABLE_DELAY

localparam SPI_FRAME_WORDS = 1;

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
			current <= data_read;
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
