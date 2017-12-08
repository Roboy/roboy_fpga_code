module tca9546a (
    input clock,
	 input reset,
    input [6:0] address, // i2c address
	 input [2:0] channel,
	 input update,
	 inout sda,
	 output scl,
	 output [2:0] LED,
	 input read,
	 output waitrequest
);


reg rw;
reg busy;
reg ack_error;
reg ena;
wire [7:0] byte_counter;
reg busy_prev;
reg [31:0] data_rd;
wire [31:0] data_read_fifo;
reg [31:0] data_wd;

reg [4:0] gpio_set;
reg read_only;

reg [7:0] read_counter;
	
always @(posedge clock, posedge reset) begin: I2C_CONTROL_LOGIC
	reg ena_prev;
	if (reset == 1) begin 
		data_wd <= 0;
		ena <= 0;
		read_only <= 0;
	end else begin
		if(update == 1) begin
			ena <= 1;
			case(channel)
				0: data_wd <= {8'h1, 8'h0, 8'h0, 8'h0};
				1: data_wd <= {8'h2, 8'h0, 8'h0, 8'h0};
				2: data_wd <= {8'h4, 8'h0, 8'h0, 8'h0}; 
				3: data_wd <= {8'h8, 8'h0, 8'h0, 8'h0};
				default: data_wd <= {8'h0, 8'h0, 8'h0, 8'h0};
			endcase 
		end
		ena_prev <= ena;
		
		if(read && ~waitrequest && address==1 && ~fifo_empty) begin
			fifo_read_ack <= 1;
		end
		
		if(byte_counter>=1) begin
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
wire [7:0] usedw;

assign LED[0] = fifo_empty;
assign LED[1] = fifo_full;
assign LED[2] = ena;


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

i2c_master i2c(
	.clk(clock),
	.reset_n(~reset),
	.ena(ena),
	.addr(address),
	.rw(rw),
	.data_wr(data_wd),
	.busy(busy),
	.data_rd(data_rd),
	.ack_error(ack_error),
	.sda(sda),
	.scl(scl),
	.byte_counter(byte_counter),
	.read_only(read_only),
	.number_of_bytes(1),
	.fifo_write_ack(fifo_write_ack)
);

endmodule