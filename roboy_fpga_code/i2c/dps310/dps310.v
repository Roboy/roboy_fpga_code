module dps310 (
    input clock,
	 input reset,
	 input update,
	 inout sda,
	 output scl,
	 output uart_out,
	 output [2:0] LED,
	 input read,
	 output waitrequest
);

//parameter pressureMeasurementRate = 127;
//parameter pressureOversamplingRate = 0;

reg rw;
reg busy;
reg [6:0] address;
reg ack_error;
reg ena;
reg ena_prev;
wire [7:0] byte_counter;
reg [31:0] data_rd;
wire [31:0] data_read_fifo;
reg [31:0] data_wd;
reg [7:0] number_of_bytes;

reg [4:0] gpio_set;
reg read_only;

reg [7:0] read_counter;
reg i2c_reset;
reg [4:0] dps310_state;
reg [4:0] dps310_next_state;
wire i2c_done;
assign i2c_done = (ena==0);

reg [31:0] uart_data_out;
reg [143:0] coefficient_buffer[1:0]; // for two sensors

always @(posedge clock, posedge reset) begin: DPS310_CONTROL_LOGIC
	parameter IDLE  = 0, TRIGGER_SEND = 1, WAIT_FOR_NEXT_COMMAND = 2, 
				INITIALIZE_SENSOR0 = 3,  INITIALIZE_SENSOR1 = 4, 
				READ_COEFFICIENTS_SENSOR0 = 5,  READ_COEFFICIENTS_SENSOR1 = 6, 
				SEND_COEFFICIENTS_SENSOR0 = 7,  SEND_COEFFICIENTS_SENSOR1 = 8, 
				CONFIG_PRESSURE_SENSOR0 = 9, CONFIG_PRESSURE_SENSOR1 = 10, 
				CONFIG_TEMPERATURE_SENSOR0 = 11, CONFIG_TEMPERATURE_SENSOR1 = 12, 
				CONFIG_OP_MODE_SENSOR0 = 13, CONFIG_OP_MODE_SENSOR1 = 14, 
				READ_PRESSURE_SENSOR0 = 15, READ_PRESSURE_SENSOR1 = 16, 
				SEND_DATA_SENSOR0 = 17, SEND_DATA_SENSOR1 = 18;
	
	reg [7:0] command_counter;
	reg [3:0] counter;
	if (reset == 1) begin 
		data_wd <= 0;
		ena <= 0;
		read_only <= 0;
		dps310_state <= IDLE;
		i2c_reset <= 1;
	end else begin
		ena_prev <= ena;
		i2c_reset <= 0;
		uart_send <= 0;
		case(dps310_state) 
			IDLE: begin
						if(update) begin // start
							dps310_state <= INITIALIZE_SENSOR0;
							command_counter <= 0;
						end
					end
			INITIALIZE_SENSOR0: begin		
						if(command_counter< 5) begin
							address <= 7'h76;
							case(command_counter)
								// correct temperature
								0: data_wd <= {8'h96, 8'hA5, 16'h0000};
								1: data_wd <= {8'h0F, 8'h96, 16'h0000};
								2: data_wd <= {8'h62, 8'h02, 16'h0000};
								3: data_wd <= {8'h0E, 24'h000000};
								4: data_wd <= {8'h0F, 24'h000000};
								default: data_wd <= 0; 
							endcase
							ena <= 1;
							number_of_bytes <= 2;
							dps310_state <= WAIT_FOR_NEXT_COMMAND;
							dps310_next_state <= INITIALIZE_SENSOR0;
						end else begin
							command_counter <= 0;
							dps310_state <= INITIALIZE_SENSOR1;  
						end
					end
			INITIALIZE_SENSOR1: begin		
						if(command_counter< 5) begin
							address <= 7'h77;
							case(command_counter)
								// correct temperature
								0: data_wd <= {8'h96, 8'hA5, 16'h0000};
								1: data_wd <= {8'h0F, 8'h96, 16'h0000};
								2: data_wd <= {8'h62, 8'h02, 16'h0000};
								3: data_wd <= {8'h0E, 24'h000000};
								4: data_wd <= {8'h0F, 24'h000000};
								default: data_wd <= 0; 
							endcase
							ena <= 1;
							number_of_bytes <= 2;
							dps310_state <= WAIT_FOR_NEXT_COMMAND;
							dps310_next_state <= INITIALIZE_SENSOR1;
						end else begin
							command_counter <= 0;
							dps310_state <= READ_COEFFICIENTS_SENSOR0;  
						end
					end
			READ_COEFFICIENTS_SENSOR0: begin		
						if(command_counter< 5) begin
							address <= 7'h76;
							case(command_counter)
								0: begin
										data_wd <= {8'h10, 24'h000000};
										number_of_bytes <= 5;
									end
								1: begin
										data_wd <= {8'h14, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[0][0*32+7:0*32+0] = data_rd[31:24];
										coefficient_buffer[0][0*32+15:0*32+8] = data_rd[23:16];
										coefficient_buffer[0][0*32+23:0*32+16] = data_rd[15:8];
										coefficient_buffer[0][0*32+31:0*32+24] = data_rd[7:0];
									end
								2: begin
										data_wd <= {8'h18, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[0][1*32+7:1*32+0] = data_rd[31:24];
										coefficient_buffer[0][1*32+15:1*32+8] = data_rd[23:16];
										coefficient_buffer[0][1*32+23:1*32+16] = data_rd[15:8];
										coefficient_buffer[0][1*32+31:1*32+24] = data_rd[7:0];
									end
								3: begin
										data_wd <= {8'h1C, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[0][2*32+7:2*32+0] = data_rd[31:24];
										coefficient_buffer[0][2*32+15:2*32+8] = data_rd[23:16];
										coefficient_buffer[0][2*32+23:2*32+16] = data_rd[15:8];
										coefficient_buffer[0][2*32+31:2*32+24] = data_rd[7:0];
									end
								4: begin
										data_wd <= {8'h20, 24'h000000};
										number_of_bytes <= 3;
										coefficient_buffer[0][3*32+7:3*32+0] = data_rd[31:24];
										coefficient_buffer[0][3*32+15:3*32+8] = data_rd[23:16];
										coefficient_buffer[0][3*32+23:3*32+16] = data_rd[15:8];
										coefficient_buffer[0][3*32+31:3*32+24] = data_rd[7:0];
									end
								default: data_wd <= 0; 
							endcase
							ena <= 1;
							rw <= 1;
							dps310_state <= WAIT_FOR_NEXT_COMMAND;
							dps310_next_state <= READ_COEFFICIENTS_SENSOR0;
						end else begin
							dps310_state <= SEND_COEFFICIENTS_SENSOR0;  
							coefficient_buffer[0][4*32+7:4*32+0] = data_rd[31:24];
							coefficient_buffer[0][4*32+15:4*32+8] = data_rd[23:16];
							uart_send_type <= COEFFICIENTS_SENSOR0;
							uart_send <= 1;
						end
					end
			SEND_COEFFICIENTS_SENSOR0: begin
				if(uart_state==IDLE) begin
					command_counter <= 0;
					dps310_state <= READ_COEFFICIENTS_SENSOR1;  
				end
			end
			READ_COEFFICIENTS_SENSOR1: begin		
						if(command_counter< 5) begin
							address <= 7'h77;
							case(command_counter)
								0: begin
										data_wd <= {8'h10, 24'h000000};
										number_of_bytes <= 5;
									end
								1: begin
										data_wd <= {8'h14, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[1][0*32+7:0*32+0] = data_rd[31:24];
										coefficient_buffer[1][0*32+15:0*32+8] = data_rd[23:16];
										coefficient_buffer[1][0*32+23:0*32+16] = data_rd[15:8];
										coefficient_buffer[1][0*32+31:0*32+24] = data_rd[7:0];
									end
								2: begin
										data_wd <= {8'h18, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[1][1*32+7:1*32+0] = data_rd[31:24];
										coefficient_buffer[1][1*32+15:1*32+8] = data_rd[23:16];
										coefficient_buffer[1][1*32+23:1*32+16] = data_rd[15:8];
										coefficient_buffer[1][1*32+31:1*32+24] = data_rd[7:0];
									end
								3: begin
										data_wd <= {8'h1C, 24'h000000};
										number_of_bytes <= 5;
										coefficient_buffer[1][2*32+7:2*32+0] = data_rd[31:24];
										coefficient_buffer[1][2*32+15:2*32+8] = data_rd[23:16];
										coefficient_buffer[1][2*32+23:2*32+16] = data_rd[15:8];
										coefficient_buffer[1][2*32+31:2*32+24] = data_rd[7:0];
									end
								4: begin
										data_wd <= {8'h20, 24'h000000};
										number_of_bytes <= 3;
										coefficient_buffer[1][3*32+7:3*32+0] = data_rd[31:24];
										coefficient_buffer[1][3*32+15:3*32+8] = data_rd[23:16];
										coefficient_buffer[1][3*32+23:3*32+16] = data_rd[15:8];
										coefficient_buffer[1][3*32+31:3*32+24] = data_rd[7:0];
									end
								default: data_wd <= 0; 
							endcase
							ena <= 1;
							rw <= 1;
							dps310_state <= WAIT_FOR_NEXT_COMMAND;
							dps310_next_state <= READ_COEFFICIENTS_SENSOR1;
						end else begin
							dps310_state <= SEND_COEFFICIENTS_SENSOR1;  
							coefficient_buffer[1][4*32+7:4*32+0] = data_rd[31:24];
							coefficient_buffer[1][4*32+15:4*32+8] = data_rd[23:16];
							uart_send_type <= COEFFICIENTS_SENSOR1;
							uart_send <= 1;
						end
					end
			SEND_COEFFICIENTS_SENSOR1: begin
				if(uart_state==IDLE) begin
					dps310_state <= CONFIG_PRESSURE_SENSOR0;  
				end
			end
			CONFIG_PRESSURE_SENSOR0: begin
					address <= 7'h76;
//					data_wd <= {8'h06, ((pressureMeasurementRate<<4)|pressureOversamplingRate&4'b1111), 16'hDEAD};
					data_wd <= {8'h06, 8'h70, 16'h0000}; // max speed, lowest precision
					ena <= 1;
					rw <= 0;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= CONFIG_PRESSURE_SENSOR1;
			end
			CONFIG_PRESSURE_SENSOR1: begin
					address <= 7'h77;
//					data_wd <= {8'h06, ((pressureMeasurementRate<<4)|pressureOversamplingRate&4'b1111), 16'hDEAD};
					data_wd <= {8'h06, 8'h70, 16'h0000}; // max speed, lowest precision
					ena <= 1;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= CONFIG_TEMPERATURE_SENSOR0;
			end
			CONFIG_TEMPERATURE_SENSOR0: begin
					address <= 7'h76;
					data_wd <= {8'h07, 8'h07, 16'h0000}; // max speed, lowest precision
					ena <= 1;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= CONFIG_TEMPERATURE_SENSOR1;
			end
			CONFIG_TEMPERATURE_SENSOR1: begin
					address <= 7'h77;
					data_wd <= {8'h07, 8'h07, 16'h0000}; // max speed, lowest precision
					ena <= 1;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= CONFIG_OP_MODE_SENSOR0;
			end
			CONFIG_OP_MODE_SENSOR0: begin
					address <= 7'h76;
					data_wd <= {8'h08, 8'h07, 16'h0000}; // background, continuous
					ena <= 1;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= CONFIG_OP_MODE_SENSOR1;
			end
			CONFIG_OP_MODE_SENSOR1: begin
					address <= 7'h77;
					data_wd <= {8'h08, 8'h07, 16'h0000}; // background, continuous
					ena <= 1;
					number_of_bytes <= 2;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= READ_PRESSURE_SENSOR0;
			end
			READ_PRESSURE_SENSOR0: begin
					address <= 7'h76;
					data_wd <= {8'h00, 24'h000000}; 
					rw <= 1;
					ena <= 1;
					number_of_bytes <= 4;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= SEND_DATA_SENSOR0;
			end
			SEND_DATA_SENSOR0: begin
					address <= 7'h76;
					data_wd <= data_rd; 
					uart_data_out <= {8'b00000000,data_rd[31:8]};
					uart_send <= 1;
					rw <= 0;
					ena <= 1;
					number_of_bytes <= 3;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= READ_PRESSURE_SENSOR1;
			end
			READ_PRESSURE_SENSOR1: begin
					address <= 7'h77;
					data_wd <= {8'h00, 24'h000000}; 
					rw <= 1;
					ena <= 1;
					number_of_bytes <= 4;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= SEND_DATA_SENSOR1;
			end
			SEND_DATA_SENSOR1: begin
					address <= 7'h77;
					data_wd <= data_rd; 
					uart_data_out <= {8'b00000001,data_rd[31:8]};
					uart_send <= 1;
					rw <= 0;
					ena <= 1;
					number_of_bytes <= 3;
					dps310_state <= WAIT_FOR_NEXT_COMMAND;
					dps310_next_state <= IDLE;
			end
			WAIT_FOR_NEXT_COMMAND: begin
					if(busy==0 && ena==0) begin
						dps310_state <= dps310_next_state;
						command_counter <= command_counter+1;
					end
			end
			default: dps310_state <= IDLE;
		endcase
		
//		if(read && ~waitrequest && address==1 && ~fifo_empty) begin
//			fifo_read_ack <= 1;
//		end

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

assign LED = dps310_state;

//assign LED[0] = fifo_empty;
//assign LED[1] = busy;
//assign LED[2] = ena;


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
	.reset_n(~i2c_reset),
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
	.number_of_bytes(number_of_bytes),
	.fifo_write_ack(fifo_write_ack)
);

reg uart_send;
wire uart_done;
reg trigger_send;
reg [7:0] data_byte;
reg [8:0] uart_byte_counter;

reg [1:0] uart_state;
parameter IDLE  = 2'b00, TRIGGER_SEND = 2'b01, WAIT_FOR_NEXT_BYTE = 2'b10;
reg [1:0] uart_send_type;
parameter COEFFICIENTS_SENSOR0  = 0, COEFFICIENTS_SENSOR1 = 1, PRESSURE_VALUE = 2;

always @(posedge clock) begin: UART_TRANSMISSION
	reg transmit;
	trigger_send <= 0;
	case(uart_state) 
		IDLE: begin
					if(uart_send) begin // if send
						uart_state <= TRIGGER_SEND;
						uart_byte_counter <= 0;
					end
				end
		TRIGGER_SEND: begin		
					case(uart_send_type)
							PRESSURE_VALUE: begin 
								if(uart_byte_counter< 4) begin
									case(uart_byte_counter)
										0: data_byte <= uart_data_out[31:24];
										1: data_byte <= uart_data_out[23:16];
										2: data_byte <= uart_data_out[15:8];
										3: data_byte <= uart_data_out[7:0];
										default: data_byte <= 0; 
									endcase
									trigger_send <= 1;
									uart_state <= WAIT_FOR_NEXT_BYTE;
								end else begin
									uart_state <= IDLE; 
								end
							end
							COEFFICIENTS_SENSOR0: begin 
								if(uart_byte_counter< 18) begin
									case(uart_byte_counter)
										0: data_byte <= coefficient_buffer[0][7:0];
										1: data_byte <= coefficient_buffer[0][15:8];
										2: data_byte <= coefficient_buffer[0][23:16];
										3: data_byte <= coefficient_buffer[0][31:24];
										4: data_byte <= coefficient_buffer[0][39:32];
										5: data_byte <= coefficient_buffer[0][47:40];
										6: data_byte <= coefficient_buffer[0][55:48];
										7: data_byte <= coefficient_buffer[0][63:56];
										8: data_byte <= coefficient_buffer[0][71:64];
										9: data_byte <= coefficient_buffer[0][79:72];
										10: data_byte <= coefficient_buffer[0][87:80];
										11: data_byte <= coefficient_buffer[0][95:88];
										12: data_byte <= coefficient_buffer[0][103:96];
										13: data_byte <= coefficient_buffer[0][111:104];
										14: data_byte <= coefficient_buffer[0][119:112];
										15: data_byte <= coefficient_buffer[0][127:120];
										16: data_byte <= coefficient_buffer[0][135:128];
										17: data_byte <= coefficient_buffer[0][143:136];
										default: data_byte <= 0; 
									endcase
									trigger_send <= 1;
									uart_state <= WAIT_FOR_NEXT_BYTE;
								end else begin
									uart_state <= IDLE; 
								end
							end
							COEFFICIENTS_SENSOR1: begin 
								if(uart_byte_counter< 18) begin
									case(uart_byte_counter)
										0: data_byte <= coefficient_buffer[1][7:0];
										1: data_byte <= coefficient_buffer[1][15:8];
										2: data_byte <= coefficient_buffer[1][23:16];
										3: data_byte <= coefficient_buffer[1][31:24];
										4: data_byte <= coefficient_buffer[1][39:32];
										5: data_byte <= coefficient_buffer[1][47:40];
										6: data_byte <= coefficient_buffer[1][55:48];
										7: data_byte <= coefficient_buffer[1][63:56];
										8: data_byte <= coefficient_buffer[1][71:64];
										9: data_byte <= coefficient_buffer[1][79:72];
										10: data_byte <= coefficient_buffer[1][87:80];
										11: data_byte <= coefficient_buffer[1][95:88];
										12: data_byte <= coefficient_buffer[1][103:96];
										13: data_byte <= coefficient_buffer[1][111:104];
										14: data_byte <= coefficient_buffer[1][119:112];
										15: data_byte <= coefficient_buffer[1][127:120];
										16: data_byte <= coefficient_buffer[1][135:128];
										17: data_byte <= coefficient_buffer[1][143:136];
										default: data_byte <= 0; 
									endcase
									trigger_send <= 1;
									uart_state <= WAIT_FOR_NEXT_BYTE;
								end else begin
									uart_state <= IDLE; 
								end
							end
						endcase
					end
		
		WAIT_FOR_NEXT_BYTE: begin
					if(uart_done) begin // if the byte is done, go to next byte
						uart_state <= TRIGGER_SEND;
						uart_byte_counter <= uart_byte_counter+1;
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
	.o_Tx_Serial(uart_out)
);

endmodule