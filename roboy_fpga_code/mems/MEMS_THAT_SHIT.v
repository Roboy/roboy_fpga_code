module MEMS_THAT_SHIT(
	input clock,
	input reset,
	output reg [31:0] address,
	output reg write,
	output reg [7:0] write_data,
	input waitrequest
);



always @(posedge clock, posedge reset) begin: AVALON_WRITE_ONCHIP_INTERFACE
	reg [31:0] counter;
	parameter IDLE  = 2'b00, WAIT_FOR_TRANSMIT = 2'b01, WAIT_FOR_NEXT_FRAME = 2'b10, DELAY = 2'b11;
	reg [1:0] onchip_state;
	if (reset == 1) begin
		counter <= 0;
	end else begin
		write <= 0;
		case(onchip_state)
			IDLE: begin
						if(counter<4095) begin // our onchip memory 
							counter <= counter + 1;
							address <= counter;
							write_data <= counter;
							onchip_state <= WAIT_FOR_TRANSMIT;
							write <= 1;
						end else begin
							counter <= 0;
						end
					end
			WAIT_FOR_TRANSMIT: begin		
						if(waitrequest==0) begin
							onchip_state <= IDLE;
						end	
					end
			default: onchip_state <= IDLE;
		endcase
	end
end

endmodule