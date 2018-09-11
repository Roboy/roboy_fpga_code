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


always @(posedge clock, posedge reset) begin: A1335_CONTROL_LOGIC
	parameter IDLE  = 0, WAIT_FOR_I2C_TRANSMISSION = 1, DONE = 2, READ_ANGLE = 3, WRITE_HAND = 4;
	reg [7:0] command_counter;
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
						end
						if(write_hand) begin // write that shit
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
				if(command_counter<16) begin
					a1335_state <= WAIT_FOR_I2C_TRANSMISSION;
					a1335_next_state <= WRITE_HAND;
					rw <= 0; 
					case(command_counter)
						0: begin data_wd <= {8'h01, arm_board_commandFrame_0[23:0]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_0;
						end
						1: begin data_wd <= {8'h02, arm_board_commandFrame_0[47:24]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_1;
						end
						2: begin data_wd <= {8'h03, arm_board_commandFrame_0[71:48]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_2;
						end
						3: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_0[87:72]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_3;
						end
						4: begin data_wd <= {8'h01, arm_board_commandFrame_1[23:0]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_0;
						end
						5: begin data_wd <= {8'h02, arm_board_commandFrame_1[47:24]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_1;
						end
						6: begin data_wd <= {8'h03, arm_board_commandFrame_1[71:48]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_2;
						end
						7: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_1[87:72]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_3;
						end
						8: begin data_wd <= {8'h01, arm_board_commandFrame_2[23:0]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_0;
						end
						9: begin data_wd <= {8'h02, arm_board_commandFrame_2[47:24]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_1;
						end
						10: begin data_wd <= {8'h03, arm_board_commandFrame_2[71:48]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_2;
						end
						11: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_2[87:72]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_3;
						end
						12: begin data_wd <= {8'h01, arm_board_commandFrame_3[23:0]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_0;
						end
						13: begin data_wd <= {8'h02, arm_board_commandFrame_3[47:24]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_1;
						end
						14: begin data_wd <= {8'h03, arm_board_commandFrame_3[71:48]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_2;
						end
						15: begin data_wd <= {8'h04, 8'h00, arm_board_commandFrame_3[87:72]}; ena <= 1; number_of_bytes <= 3; 
							device_id <= arm_board_device_id_3;
						end
						default: data_wd <= 0; 
					endcase
				end else begin
					command_counter <= 0;
					a1335_state <= DONE;
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