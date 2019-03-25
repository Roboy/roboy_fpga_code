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
	output  scl, // clock
	inout  sda,
	input trigger_reset,
	input trigger_read,
	output fifo_write_ack_out
);

parameter CLOCK_SPEED_HZ = 50_000_000;
parameter BUS_SPEED_HZ = 400_000;

assign fifo_write_ack_out = fifo_write;

assign waitrequest = (waitFlag && read); // if waiting for data
reg waitFlag;
reg reset_sensor;
reg [31:0]update_frequency;
reg [11:0]mag_x;
reg [11:0]mag_y;
reg [11:0]mag_z;
reg [11:0]temp;
reg [1:0]frm;
reg [1:0]ch;
reg t;
reg ff;
reg pd;

reg read_magnetic_data;

always @(posedge clock, posedge reset) begin: AVALON_INTERFACE
	if (reset == 1) begin
		waitFlag <= 0;
		update_frequency <= 100;
	end else begin
		waitFlag <= 1;
		if(read) begin	
			if((address>>8)<=8'h14) begin
				case(address>>8)
					8'h00: readdata <= mag_x;
					8'h01: readdata <= mag_y;
					8'h02: readdata <= mag_z;
					8'h03: readdata <= temp;
					8'h04: readdata <= {t,ff,pd,frm,ch};
					8'h05: readdata <= ack_error;
					8'h06: readdata <= tlv_state;
					8'h07: readdata <= config_data;
				endcase
			end
			if(waitFlag==1) begin // after one clock cycle the sensor_data_avalon should be stable
				waitFlag <= 0;
			end
		end
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if((address>>8)<=8'h14) begin
				case(address>>8)
					8'h00: reset_sensor <= ((writedata&8'hff)!=0);
					8'h01: update_frequency <= writedata;
					8'h02: read_magnetic_data <=  ((writedata&8'hff)!=0);
				endcase
			end
		end
		if(reset_sensor!=0) begin
			reset_sensor<=0;
		end
		if(read_magnetic_data!=0) begin
			read_magnetic_data<=0;
		end
	end
end

reg [31:0] config_data;
wire  parity;
assign parity=(~^config_data);

reg tlv_reset_sda;
reg tlv_reset_scl;
reg reset_done;
reg [1:0] frame_counter;
reg reset_first_time;
reg [4:0] tlv_state;
reg [4:0] tlv_state_next;
always @(posedge clock, posedge reset) begin: TLV_FSM
	parameter IDLE  = 0, RESET = 1, READCONFIG = 2, CONFIGURE = 3, READMAGDATA = 4, WAIT_FOR_DATA = 5, DELAY = 6, GENERAL_RESET = 7, BYTE7=8, BYTE8AND9 = 9, PARITY=10, WAITFORCONFIGURE=11, THROWAWAYFIRSTBYTE = 12;
	reg [31:0] delay_counter;
	reg [8:0] general_reset_state;
	reg init;
	if (reset==1) begin
			tlv_state <= IDLE;
			tlv_reset_sda <= 1;
			tlv_reset_scl <= 1;
			init <= 1;
	end else begin
		if(fifo_clear==1) begin
			fifo_clear <= 0;
		end
		if(fifo_read_ack) begin
			fifo_read_ack <= 0;
		end
		case(tlv_state)
			IDLE: begin
//				if(reset_sensor || trigger_reset) begin
				if(init) begin
					init <= 0;
					tlv_state <= GENERAL_RESET;
					fifo_clear <= 1;
					fifo_read_ack <= 0;
					general_reset_state <= 0;
				end else begin
//				if(read_magnetic_data || trigger_read) begin
					tlv_state <= READMAGDATA;
				end
			end
			GENERAL_RESET: begin
				case(general_reset_state)
					0: begin 
						tlv_reset_sda <= 1;
						tlv_reset_scl <= 1;
						delay_counter <= 5000;
						general_reset_state <= general_reset_state+1;
					end
					1: begin
						if(delay_counter==0) begin
							tlv_reset_sda <= 0;
							delay_counter <= 500;
							general_reset_state <= general_reset_state+1;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					2: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					3: begin //1
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					4: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					5: begin //2
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					6: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					7: begin //3
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					8: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					9: begin //4
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					10: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					11: begin //5
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					12: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					13: begin //6
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					14: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					15: begin //7
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					16: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					17: begin // 8
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					18: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					19: begin // 9
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					20: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 0;
							tlv_reset_sda <= 1'bz; // floating
							general_reset_state <= general_reset_state+1;
							delay_counter <= 150;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					21: begin
						if(delay_counter==0) begin
							tlv_reset_scl <= 1; //will set the tlv i2c address to 0x5e
							tlv_reset_sda <= 1; 
							general_reset_state <= general_reset_state+1;
							delay_counter <= 1000;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					22: begin 
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							general_reset_state <= general_reset_state+1;
							delay_counter <= 250;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
					23: begin 
						if(delay_counter==0) begin
							tlv_reset_scl <= 1;
							tlv_reset_sda <= 1;
							general_reset_state <=0;
							delay_counter <= 5000;
							tlv_state <= DELAY;
							tlv_state_next <= RESET;
							reset_first_time <= 1;
						end else begin
							delay_counter <= delay_counter-1;
						end
					end
				endcase
			end
			RESET: begin
				number_of_bytes <= 10;
				addr <= 7'h5e;
				data_wd <= 0; 
				rw <= 1;
				read_only <= 1;
				ena <= 1;
				config_data <= 0;
				tlv_state <= READCONFIG;
				fifo_clear <= 1;
			end
			READCONFIG: begin
				if((byte_counter>=number_of_bytes) && ena == 1) begin
					ena <= 0;
					fifo_read_ack <= 1;
					delay_counter <= 500;
					tlv_state <= DELAY;
					tlv_state_next <= THROWAWAYFIRSTBYTE;
				end 
			end
			THROWAWAYFIRSTBYTE: begin
				if(fifo_read_ack==0) begin
					delay_counter <= 500;
					tlv_state <= DELAY;
					tlv_state_next <= BYTE8AND9;
					config_data |= ((3'b011|((data_read_fifo)&5'b11000))<<8);
					fifo_read_ack <= 1;
					tlv_state <= BYTE8AND9;
				end
			end
			BYTE8AND9: begin
				if(fifo_read_ack==0) begin
					config_data |= (((data_read_fifo>>24)&8'hff)<<16);
					config_data |= ((8'b01000000|(5'b11111&((data_read_fifo>>16)&8'hff)))<<24);
					tlv_state <= PARITY;
				end
			end
			PARITY: begin
				config_data |= (parity<<15);
				delay_counter <= 500;
				tlv_state <= DELAY;
				tlv_state_next <= CONFIGURE;
			end
			CONFIGURE:begin
				number_of_bytes <= 4;
				addr <= 7'h5e;
				data_wd <= config_data;  
				rw <= 0;
				read_only <= 0;
				ena <= 1;
				tlv_state <= WAITFORCONFIGURE;
			end
			WAITFORCONFIGURE: begin
				if((byte_counter>=number_of_bytes) && ena == 1) begin
					ena <= 0;
					if(reset_first_time) begin
						reset_first_time <= 0;
						delay_counter <= 5000;
						tlv_state <= DELAY;
						tlv_state_next <= RESET;
					end else begin
						delay_counter <= 5000;
						tlv_state <= DELAY;
						tlv_state_next <= READMAGDATA;
						reset_done <= 1;
					end
				end
			end
			READMAGDATA: begin 
				number_of_bytes <= 7;
				addr <= 7'h5e;
				rw <= 1;
				read_only <= 1;
				ena <= 1;
				tlv_state <= WAIT_FOR_DATA;
			end
			WAIT_FOR_DATA: begin
				if((byte_counter>=number_of_bytes || ack_error == 1) && ena == 1) begin
					ena <= 0;
					temp[7:0] = (data_read_fifo>>16);
					mag_z[3:0] = (data_read_fifo>>8)&8'b00001111;
					pd = (data_read_fifo>>12)&1'b1;
					ff = (data_read_fifo>>13)&1'b1;
					t = (data_read_fifo>>14)&1'b1;
					mag_y[3:0] = data_read_fifo&8'b00001111;
					mag_x[3:0] = (data_read_fifo>>4)&8'b00001111;
					fifo_read_ack <= 1;
					temp[11:8] = (data_read_fifo>>28)&8'b00001111;
					if(reset_done) begin
						reset_done <= 0;
						frame_counter = (data_read_fifo>>26)&8'b00000011 + 1;
					end else begin
						frame_counter = frame_counter+1;
						frm[1:0] = (data_read_fifo>>26)&8'b00000011;
//						if(frame_counter!=frm) begin
//							tlv_state <= GENERAL_RESET;
//							fifo_clear <= 1;
//							fifo_read_ack <= 0;
//							general_reset_state <= 0;
//						end
					end
					ch[1:0] = (data_read_fifo>>24)&8'b00000011;
					mag_z[11:4] = (data_read_fifo>>16)&8'hff;
					mag_y[11:4] = (data_read_fifo>>8)&8'hff;
					mag_x[11:4] = (data_read_fifo)&8'hff;
					if(update_frequency!=0) begin
						delay_counter <= CLOCK_SPEED_HZ/update_frequency;
						tlv_state <= DELAY;
						tlv_state_next <= IDLE;
					end else begin
						tlv_state <= IDLE;
					end
				end
			end
			DELAY: begin
				delay_counter <= delay_counter -1;
				if(delay_counter==0) begin
					tlv_state <= tlv_state_next;
				end
			end
			default: tlv_state <= IDLE;
		endcase
	end
end

reg ena;
reg [7:0] number_of_bytes;
reg [6:0] addr;
reg  rw;
reg  busy;
reg  ack_error;
wire [7:0] byte_counter ;
reg [31:0] data_rd ;
reg [31:0] data_read_fifo ;
reg [31:0] data_wd ;

reg read_only;

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
	.sclr(fifo_clear),
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
	.reset_n(~reset_sensor),
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
	.fifo_write_ack(fifo_write_ack),
	.tlv_scl(tlv_reset_scl),
	.tlv_sda(tlv_reset_sda)
);



endmodule