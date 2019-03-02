module TLV493(
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input read,
	output signed [31:0] readdata,
	input write,
	output signed [31:0] writedata,
	output waitrequest,
	// I2C
	output [NUMBER_OF_SENSORS-1:0] scl, // clock
	inout [NUMBER_OF_SENSORS-1:0] sda
);

parameter CLOCK_SPEED_HZ = 50_000_000;
parameter BUS_SPEED_HZ = 400_000;
parameter NUMBER_OF_SENSORS = 1;

assign waitrequest = (waitFlag || state==4); // if waiting for data
reg waitFlag;
reg reset_sensors;
reg [31:0]update_frequency;
reg [11:0]mag_x[NUMBER_OF_SENSORS-1:0];
reg [11:0]mag_y[NUMBER_OF_SENSORS-1:0];
reg [11:0]mag_z[NUMBER_OF_SENSORS-1:0];

always @(posedge clock, posedge reset) begin: AVALON_INTERFACE
	if (reset == 1) begin
		waitFlag <= 0;
	end else begin
		waitFlag <= 1;
		if(read) begin	
			if((address>>8)<=8'h14 && address[7:0]<NUMBER_OF_SENSORS) begin
				case(address>>8)
					8'h00: readdata <= mag_x[address[7:0]];
					8'h01: readdata <= mag_y[address[7:0]];
					8'h02: readdata <= mag_z[address[7:0]];
					8'h03: readdata <= ack_error[address[7:0]];
				endcase
			end
			if(waitFlag==1) begin // after one clock cycle the sensor_data_avalon should be stable
				waitFlag <= 0;
			end
		end
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if((address>>8)<=8'h14 && address[7:0]<NUMBER_OF_SENSORS) begin
				case(address>>8)
					8'h00: reset_sensors <= (writedata!=0);
					8'h01: update_frequency <= writedata;
				endcase
			end
		end
		if(reset_sensors==1) begin
			reset_sensors<=0;
		end
	end
end

reg [31:0] config_data[NUMBER_OF_SENSORS-1:0];
wire [NUMBER_OF_SENSORS-1:0] parity;
generate
	for(j=0;k<NUMBER_OF_SENSORS;k=k+1) begin
		 assign parity[k]=(~^config_data[k]);
	end
endgenerate


reg [2:0] state;
always @(posedge clock, posedge reset) begin: TLV_FSM
	parameter IDLE  = 0, RESET = 1, READCONFIG = 2, CONFIGURE = 3, READMAGDATA = 4, WAIT_FOR_DATA = 5, DELAY = 6;
	reg [31:0] delay_counter;
	reg [8:0] i;
	if (reset_sensors==1) begin
		state <= RESET;
	end else begin
		for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
			if(fifo_clear[i]==1) begin
				fifo_clear[i] <= 0;
			end
		end
		case(state)
			IDLE: begin
				state <= READMAGDATA;
			end
			RESET: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					number_of_bytes[i] <= 3;
					addr[i] <= 7'h5e;
					data_wd[i] <= 7; // we need registers 7-9 for configuration
					rw[i] <= 1;
					read_only[i] <= 0;
					ena[i] <= 1;
				end
				state <= READCONFIG;
			end
			READCONFIG: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes || ack_error[k] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
						byte_counter[i] <= 0;
						config_data[i] = 0;
						config_data[i] |= (0b011|((data_read_fifo[i]>>24)&0b11000))<<8;
						config_data[i] |= ((data_read_fifo[i]>>16)<<16);
						config_data[i] |= (0b01000000|(0b11111&(data_read_fifo[i]>>8)))<<24;
						config_data[i] |= parity[i];
					end
				end
				if(~&ena==0) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						fifo_clear[i] <= 1;
						number_of_bytes[i] <= 4;
						addr[i] <= 7'h5e;
						data_wd[i] <= config_data; 
						rw[i] <= 0;
						read_only[i] <= 0;
						ena[i] <= 1;
					end
					state <= CONFIGURE;
				end
			end
			CONFIGURE:begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes || ack_error[k] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
						byte_counter[i] <= 0;
						state<=READMAGDATA;
					end
				end
			end
			READMAGDATA: begin 
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					number_of_bytes[i] <= 7;
					addr[i] <= 7'h5e;
					rw[i] <= 1;
					read_only[i] <= 1;
					ena[i] <= 1;
					state <= WAIT_FOR_DATA;
				end
			end
			WAIT_FOR_DATA: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes || ack_error[k] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
						byte_counter[i] <= 0;
						state<=DELAY;
						delay_counter <= CLOCK_SPEED_HZ/update_frequency;
					end
				end
			end
			DELAY: begin
				delay_counter <= delay_counter -1;
				if(delay_counter==0) begin
					state <= IDLE;
				end
			end
			default: state = IDLE;
		endcase
	end
end

reg [NUMBER_OF_SENSORS-1:0]ena;
reg [7:0] number_of_bytes[NUMBER_OF_SENSORS-1:0];
reg [6:0] addr[NUMBER_OF_SENSORS-1:0];
reg [NUMBER_OF_SENSORS-1:0] rw;
reg [NUMBER_OF_SENSORS-1:0] busy;
reg [NUMBER_OF_SENSORS-1:0] ack_error;
reg [NUMBER_OF_SENSORS-1:0] ena;
reg [7:0] number_of_bytes[NUMBER_OF_SENSORS-1:0] ;
wire [7:0] byte_counter[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_rd[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_read_fifo[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_wd[NUMBER_OF_SENSORS-1:0] ;

reg read_only[NUMBER_OF_SENSORS-1:0];

reg ena[NUMBER_OF_SENSORS-1:0];
wire fifo_write[NUMBER_OF_SENSORS-1:0];
reg read_fifo[NUMBER_OF_SENSORS-1:0];
reg write_fifo[NUMBER_OF_SENSORS-1:0];
wire fifo_write_ack[NUMBER_OF_SENSORS-1:0];
reg fifo_read_ack[NUMBER_OF_SENSORS-1:0];
reg fifo_clear[NUMBER_OF_SENSORS-1:0];
wire fifo_empty[NUMBER_OF_SENSORS-1:0];
wire fifo_full[NUMBER_OF_SENSORS-1:0];
reg [7:0] usedw[NUMBER_OF_SENSORS-1:0];

genvar k;
generate 
	for(k=0; k<NUMBER_OF_SENSORS; k = k+1) begin : i2c_cores
		fifo fifo(
			.clock(clock),
			.data(data_rd[k]),
			.rdreq(fifo_read_ack[k]),
			.sclr(reset||fifo_clear[k]),
			.wrreq(fifo_write[k]),
			.q(data_read_fifo[k]),
			.empty(fifo_empty[k]),
			.full(fifo_full[k]),
			.usedw(usedw[k])
		);

		oneshot oneshot(
			.clk(clock),
			.edge_sig(fifo_write_ack[k]),
			.level_sig(fifo_write[k])
		);

		i2c_master #(CLOCK_SPEED_HZ, BUS_SPEED_HZ) i2c(
			.clk(clock),
			.reset_n(~reset_sensors),
			.ena(ena[k]),
			.addr(addr[k]),
			.rw(rw[k]),
			.data_wr(data_wd[k]),
			.busy(busy[k]),
			.data_rd(data_rd[k]),
			.ack_error(ack_error[k]),
			.sda(sda[k]),
			.scl(scl[k]),
			.byte_counter(byte_counter[k]),
			.read_only(read_only[k]),
			.number_of_bytes(number_of_bytes[k]),
			.fifo_write_ack(fifo_write_ack[k])
		);
	end
endgenerate 



endmodule