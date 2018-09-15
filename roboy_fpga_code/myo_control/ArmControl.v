module ArmControl (
    input clock,
	 input reset,
	 input wire elbow_read_joint_angle,
	 input wire read_status,
	 input wire write_hand,
	 input wire read_hand,
	 inout wire sda,
	 output wire scl,
	 output [2:0] LED,
	 input wire [6:0] elbow_device_id,
	 input wire [6:0] arm_board_device_id_0,
	 input wire [6:0] arm_board_device_id_1,
	 input wire [6:0] arm_board_device_id_2,
	 input wire [6:0] arm_board_device_id_3,
	 input wire [87:0] arm_board_commandFrame_0,
	 input wire [87:0] arm_board_commandFrame_1,
	 input wire [87:0] arm_board_commandFrame_2,
	 input wire [87:0] arm_board_commandFrame_3,
	 output reg done,
	 output reg [11:0] angle,
	 output reg [31:0] status,
	 output reg ack_error
);

reg rw;
reg busy;
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
reg [7:0] a1335_state;
reg [7:0] a1335_next_state;
reg [6:0] device_id;

reg [87:0] arm_board_commandFrame_prev[3:0];

always @(posedge clock, posedge reset) begin: A1335_CONTROL_LOGIC
	parameter IDLE  = 0, WAIT_FOR_I2C_TRANSMISSION = 1, DONE = 2, READ_ANGLE = 3, WRITE_HAND = 4, DELAY = 5;
	reg [7:0] command_counter;
	reg [7:0] delay_counter;
	if (reset == 1) begin 
		data_wd <= 0;
		ena <= 0;
		read_only <= 0;
		a1335_state <= IDLE;
		done <= 1;
		angle <= 7;
	end else begin
		ena_prev <= ena;
		case(a1335_state) 
			IDLE: begin
						ena <= 0;
						if(elbow_read_joint_angle) begin // read that shit
							a1335_state <= READ_ANGLE;
							command_counter <= 0;
							done <= 0;
						end else if (write_hand) begin // write that shit
							a1335_state <= WRITE_HAND;
							command_counter <= 0;
							done <= 0;
						end
					end 
			READ_ANGLE: begin
				if(command_counter< 2) begin
					device_id <= elbow_device_id;
					a1335_state <= WAIT_FOR_I2C_TRANSMISSION;
					a1335_next_state <= READ_ANGLE;
					rw <= 1; 
					case(command_counter)
						0: begin data_wd <= {8'h20, 8'h00, 16'h0000}; ena <= 1; number_of_bytes <= 3; end
						1: begin 
								angle <= data_read_fifo[27:16];
								fifo_read_ack <= 1;
							end	
						default: data_wd <= 0; 
					endcase
				end else begin
					command_counter <= 0;
					a1335_state <= DONE;
				end
			end
			WRITE_HAND: begin
				if(command_counter<=18) begin
					a1335_state <= WAIT_FOR_I2C_TRANSMISSION;
					a1335_next_state <= WRITE_HAND;
					rw <= 0; 
					case(command_counter)
						0: begin 
							if(arm_board_commandFrame_0!=arm_board_commandFrame_prev[0]) begin
								data_wd <= {8'h01, arm_board_commandFrame_0[23:0]}; ena <= 1; number_of_bytes <= 4; 
								device_id <= arm_board_device_id_0;
								arm_board_commandFrame_prev[0] <= arm_board_commandFrame_0;
							end else begin
								command_counter <= 5;
								a1335_state <= WRITE_HAND;
							end
						end
						1: begin data_wd <= {8'h02, arm_board_commandFrame_0[47:24]}; ena <= 1; number_of_bytes <= 4; end
						2: begin data_wd <= {8'h03, arm_board_commandFrame_0[71:48]}; ena <= 1; number_of_bytes <= 4; end
						3: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_0[87:72]}; ena <= 1; number_of_bytes <= 4; end
						4: begin delay_counter <= 100; a1335_state <= DELAY; end
						5: begin 
							if(arm_board_commandFrame_1!=arm_board_commandFrame_prev[1]) begin
								data_wd <= {8'h01, arm_board_commandFrame_1[23:0]}; ena <= 1; number_of_bytes <= 4; 
								device_id <= arm_board_device_id_1;
								arm_board_commandFrame_prev[1] <= arm_board_commandFrame_1;
							end else begin
								command_counter <= 10;
								a1335_state <= WRITE_HAND;
							end
						end
						6: begin data_wd <= {8'h02, arm_board_commandFrame_1[47:24]}; ena <= 1; number_of_bytes <= 4; end
						7: begin data_wd <= {8'h03, arm_board_commandFrame_1[71:48]}; ena <= 1; number_of_bytes <= 4; end
						8: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_1[87:72]}; ena <= 1; number_of_bytes <= 4; end
						9: begin delay_counter <= 100; a1335_state <= DELAY; end
						10: begin 
							if(arm_board_commandFrame_2!=arm_board_commandFrame_prev[2]) begin
								data_wd <= {8'h01, arm_board_commandFrame_1[23:0]}; ena <= 1; number_of_bytes <= 4; 
								device_id <= arm_board_device_id_2;
								arm_board_commandFrame_prev[2] <= arm_board_commandFrame_2;
							end else begin
								command_counter <= 15;
								a1335_state <= WRITE_HAND;
							end
						end
						11: begin data_wd <= {8'h02, arm_board_commandFrame_2[47:24]}; ena <= 1; number_of_bytes <= 4; end
						12: begin data_wd <= {8'h03, arm_board_commandFrame_2[71:48]}; ena <= 1; number_of_bytes <= 4; end
						13: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_2[87:72]}; ena <= 1; number_of_bytes <= 4; end
						14: begin delay_counter <= 100; a1335_state <= DELAY; end
						15: begin 
							if(arm_board_commandFrame_3!=arm_board_commandFrame_prev[3]) begin
								data_wd <= {8'h01, arm_board_commandFrame_3[23:0]}; ena <= 1; number_of_bytes <= 4; 
								device_id <= arm_board_device_id_3;
								arm_board_commandFrame_prev[3] <= arm_board_commandFrame_3;
							end else begin
								command_counter <= 0;
								a1335_state <= DONE;
							end
						end
						16: begin data_wd <= {8'h02, arm_board_commandFrame_3[47:24]}; ena <= 1; number_of_bytes <= 4; end
						17: begin data_wd <= {8'h03, arm_board_commandFrame_3[71:48]}; ena <= 1; number_of_bytes <= 4; end
						18: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_3[87:72]}; ena <= 1; number_of_bytes <= 4; end
						default: data_wd <= 0; 
					endcase
				end else begin
					command_counter <= 0;
					a1335_state <= DONE;
				end
			end
			DELAY: begin
				if(delay_counter!=0) begin
					delay_counter <= delay_counter-1;
				end else begin
					command_counter <= command_counter+1;
					a1335_state <= WRITE_HAND;
				end
			end 
			DONE: begin 
				a1335_state <= IDLE;
				done <= 1;
			end
			WAIT_FOR_I2C_TRANSMISSION: begin
				if(busy==0 && ena==0) begin
					a1335_state <= a1335_next_state;
					command_counter <= command_counter+1;
				end
			end
			default: a1335_state <= IDLE;
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

wire fifo_write;
reg read_fifo;
reg write_fifo;
wire fifo_write_ack;
reg fifo_read_ack;
reg fifo_clear;
wire fifo_empty;
wire fifo_full;
wire [7:0] usedw;

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

i2c_master #(50000000, 400000) i2c(
	.clk(clock),
	.reset_n(~reset),
	.ena(ena),
	.addr(device_id),
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