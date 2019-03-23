module TLV493(
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input read,
	output signed [31:0] readdata,
	input write,
	input signed [31:0] writedata,
	output waitrequest,
	// I2C
	output [NUMBER_OF_SENSORS-1:0] scl, // clock
	inout [NUMBER_OF_SENSORS-1:0] sda
);

parameter CLOCK_SPEED_HZ = 50_000_000;
parameter BUS_SPEED_HZ = 400_000;
parameter NUMBER_OF_SENSORS = 1;

assign waitrequest = (waitFlag && read); // if waiting for data
reg waitFlag;
reg reset_sensors;
reg [31:0]update_frequency;
reg [11:0]mag_x[NUMBER_OF_SENSORS-1:0];
reg [11:0]mag_y[NUMBER_OF_SENSORS-1:0];
reg [11:0]mag_z[NUMBER_OF_SENSORS-1:0];
reg [11:0]temp[NUMBER_OF_SENSORS-1:0];
reg [1:0]frm[NUMBER_OF_SENSORS-1:0];
reg [1:0]ch[NUMBER_OF_SENSORS-1:0];
reg t[NUMBER_OF_SENSORS-1:0];
reg ff[NUMBER_OF_SENSORS-1:0];
reg pd[NUMBER_OF_SENSORS-1:0];

reg read_magnetic_data;

always @(posedge clock, posedge reset) begin: AVALON_INTERFACE
	if (reset == 1) begin
		waitFlag <= 0;
		update_frequency <= 100;
	end else begin
		waitFlag <= 1;
		if(read) begin	
			if((address>>8)<=8'h14 && address[7:0]<NUMBER_OF_SENSORS) begin
				case(address>>8)
					8'h00: readdata <= mag_x[address[7:0]];
					8'h01: readdata <= mag_y[address[7:0]];
					8'h02: readdata <= mag_z[address[7:0]];
					8'h03: readdata <= temp[address[7:0]];
					8'h04: readdata <= {t[address[7:0]],ff[address[7:0]],pd[address[7:0]],frm[address[7:0]],ch[address[7:0]]};
					8'h05: readdata <= ack_error[address[7:0]];
					8'h06: readdata <= state;
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
					8'h02: read_magnetic_data <=  (writedata!=0);
				endcase
			end
		end
		if(reset_sensors==1) begin
			reset_sensors<=0;
		end
		if(read_magnetic_data) begin
			read_magnetic_data<=0;
		end
	end
end

reg [31:0] config_data[NUMBER_OF_SENSORS-1:0];
wire [NUMBER_OF_SENSORS-1:0] parity;
generate
	for(k=0;k<NUMBER_OF_SENSORS;k=k+1) begin: parity_assignement
		 assign parity[k]=(~^config_data[k]);
	end
endgenerate

reg [NUMBER_OF_SENSORS-1:0]tlv_reset_sda;
reg [NUMBER_OF_SENSORS-1:0]tlv_reset_scl;
reg reset_done;
reg [1:0] frame_counter[NUMBER_OF_SENSORS-1:0];

reg [3:0] state;
always @(posedge clock, posedge reset) begin: TLV_FSM
	parameter IDLE  = 0, RESET = 1, READCONFIG = 2, CONFIGURE = 3, READMAGDATA = 4, WAIT_FOR_DATA = 5, DELAY = 6, GENERAL_RESET = 7, PARSE_CONFIG=8, READCONFIG2 = 9, PARSE_CONFIG2=10, CONFIGURE2=11, RESET2 = 12;
	reg [31:0] delay_counter;
	reg [8:0] i;
	reg [8:0] j;
	reg config_upper_half;
	if (reset==1) begin
		state <= IDLE;
	end else begin
		for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
			if(fifo_clear[i]==1) begin
				fifo_clear[i] <= 0;
			end
			if(fifo_read_ack[i]==1) begin
				fifo_read_ack[i] <= 0;
			end
		end
		case(state)
			IDLE: begin
				if(reset_sensors) begin
					state <= RESET;
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						fifo_clear[i] <= 1;
					end
				end
				if(read_magnetic_data) begin
					state <= READMAGDATA;
				end
			end
			GENERAL_RESET: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					case(j)
						0: begin 
							tlv_reset_sda[i] <= 1;
							tlv_reset_scl[i] <= 1;
							delay_counter <= 5000;
							j <= j+1;
						end
						1: begin
							if(delay_counter==0) begin
								tlv_reset_sda[i] <= 0;
								delay_counter <= 500;
								j <= j+1;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						2: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						3: begin //1
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						4: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						5: begin //2
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						6: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						7: begin //3
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						8: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						9: begin //4
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						10: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						11: begin //5
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						12: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						13: begin //6
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						14: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						15: begin //7
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						16: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						17: begin // 8
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						18: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						19: begin // 9
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						20: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 0;
								tlv_reset_sda[i] <= 1'bz; // floating
								j <= j+1;
								delay_counter <= 150;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						21: begin
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1; //will set the tlv i2c address to 0x5e
								tlv_reset_sda[i] <= 1; 
								j <= j+1;
								delay_counter <= 1000;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						22: begin 
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								j <= j+1;
								delay_counter <= 250;
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
						23: begin 
							if(delay_counter==0) begin
								tlv_reset_scl[i] <= 1;
								tlv_reset_sda[i] <= 1;
								j<=0;
								state <= RESET;
								for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
									fifo_clear[i] <= 1;
								end
							end else begin
								delay_counter <= delay_counter-1;
							end
						end
					endcase
				end
			end
			RESET: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					number_of_bytes[i] <= 10;
					addr[i] <= 7'h5e;
					data_wd[i] <= 0; 
					rw[i] <= 1;
					read_only[i] <= 1;
					ena[i] <= 1;
					delay_counter <= 500;
					config_data[i] = 0;
				end
				state <= READCONFIG;
			end
			READCONFIG: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes[i] || ack_error[i] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
					end
				end
				if(ena==0) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] <= 0;
					end
					config_upper_half <= 1;
					state <= PARSE_CONFIG;
				end
			end
			PARSE_CONFIG: begin
				if(config_upper_half) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] |= (((data_read_fifo[i]>>24)&8'hff)<<16);
						config_data[i] |= (8'b01000000|(5'b11111&((data_read_fifo[i]>>16)&8'hff)))<<24;
						fifo_read_ack[i] <= 1;
					end
					config_upper_half <= 0;
				end else begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] |= (3'b011|((data_read_fifo[i])&5'b11000))<<8;
						config_data[i] |= (parity[i]<<7);
					end
					delay_counter <= delay_counter -1;
					if(delay_counter==0) begin
						for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
							fifo_clear[i] <= 1;
							number_of_bytes[i] <= 4;
							addr[i] <= 7'h5e;
							data_wd[i] <= config_data[i]; 
							rw[i] <= 0;
							read_only[i] <= 0;
							ena[i] <= 1;
						end
						state <= CONFIGURE;
					end
				end
			end
			CONFIGURE:begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes[i] || ack_error[i] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
					end
				end
				if(ena==0) begin
					delay_counter <= 500;
					state<=DELAY;
				end
			end
			RESET2: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					number_of_bytes[i] <= 10;
					addr[i] <= 7'h5e;
					data_wd[i] <= 0; 
					rw[i] <= 1;
					read_only[i] <= 1;
					ena[i] <= 1;
					delay_counter <= 500;
					config_data[i] = 0;
				end
				state <= READCONFIG2;
			end
			READCONFIG2: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes[i] || ack_error[i] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
					end
				end
				if(ena==0) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] <= 0;
					end
					config_upper_half <= 1;
					state <= PARSE_CONFIG2;
				end
			end
			PARSE_CONFIG2: begin
				if(config_upper_half) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] |= (((data_read_fifo[i]>>24)&8'hff)<<16);
						config_data[i] |= (8'b01000000|(5'b11111&((data_read_fifo[i]>>16)&8'hff)))<<24;
						fifo_read_ack[i] <= 1;
					end
					config_upper_half <= 0;
				end else begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						config_data[i] |= (3'b011|((data_read_fifo[i])&5'b11000))<<8;
						config_data[i] |= (parity[i]<<7);
					end
					delay_counter <= delay_counter -1;
					if(delay_counter==0) begin
						for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
							fifo_clear[i] <= 1;
							number_of_bytes[i] <= 4;
							addr[i] <= 7'h5e;
							data_wd[i] <= config_data[i]; 
							rw[i] <= 0;
							read_only[i] <= 0;
							ena[i] <= 1;
						end
						state <= CONFIGURE2;
					end
				end
			end
			CONFIGURE2:begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes[i] || ack_error[i] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
					end
				end
				if(ena==0) begin
					delay_counter <= 500;
					state<=DELAY;
					reset_done <= 1;
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
					fifo_clear[i]<=1;
				end
			end
			WAIT_FOR_DATA: begin
				for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
					if((byte_counter[i]>=number_of_bytes[i] || ack_error[i] == 1) && ena[i] == 1) begin
						ena[i] <= 0;
						temp[i][7:0] = (data_read_fifo[i]>>16);
						mag_z[i][3:0] = (data_read_fifo[i]>>8)&8'b00001111;
						pd[i] = (data_read_fifo[i]>>12)&1'b1;
						ff[i] = (data_read_fifo[i]>>13)&1'b1;
						t[i] = (data_read_fifo[i]>>14)&1'b1;
						mag_y[i][3:0] = data_read_fifo[i]&8'b00001111;
						mag_x[i][3:0] = (data_read_fifo[i]>>4)&8'b00001111;
						fifo_read_ack[i] = 1;
						fifo_read_ack[i] = 0;
						temp[i][11:8] = (data_read_fifo[i]>>28)&8'b00001111;
						if(reset_done) begin
							reset_done <= 0;
							frame_counter[i] = (data_read_fifo[i]>>26)&8'b00000011 + 1;
						end else begin
							frame_counter[i] = frame_counter[i]+1;
							frm[i][1:0] = (data_read_fifo[i]>>26)&8'b00000011;
							if(frame_counter[i]!=frm[i]) begin
								state <= GENERAL_RESET;
							end
						end
						ch[i][1:0] = (data_read_fifo[i]>>24)&8'b00000011;
						mag_z[i][11:4] = (data_read_fifo[i]>>16)&8'hff;
						mag_y[i][11:4] = (data_read_fifo[i]>>8)&8'hff;
						mag_x[i][11:4] = (data_read_fifo[i])&8'hff;
						state<=DELAY;
						delay_counter <= CLOCK_SPEED_HZ/update_frequency;
					end
				end
			end
			DELAY: begin
				delay_counter <= delay_counter -1;
				if(delay_counter==0) begin
					for(i=0;i<NUMBER_OF_SENSORS;i=i+1) begin
						fifo_clear[i] <= 1;
					end
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
wire [7:0] byte_counter[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_rd[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_read_fifo[NUMBER_OF_SENSORS-1:0] ;
reg [31:0] data_wd[NUMBER_OF_SENSORS-1:0] ;

reg read_only[NUMBER_OF_SENSORS-1:0];

wire fifo_write[NUMBER_OF_SENSORS-1:0];
reg read_fifo[NUMBER_OF_SENSORS-1:0];
reg write_fifo[NUMBER_OF_SENSORS-1:0];
wire fifo_write_ack[NUMBER_OF_SENSORS-1:0];
reg fifo_read_ack[NUMBER_OF_SENSORS-1:0];
reg fifo_clear[NUMBER_OF_SENSORS-1:0];
wire fifo_empty[NUMBER_OF_SENSORS-1:0];
wire fifo_full[NUMBER_OF_SENSORS-1:0];
reg [7:0] usedw[NUMBER_OF_SENSORS-1:0];

reg [NUMBER_OF_SENSORS-1:0]i2c_select;

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
			.fifo_write_ack(fifo_write_ack[k]),
			.tlv_scl(1'b1),
			.tlv_sda(1'b1)
		);
	end
endgenerate 



endmodule