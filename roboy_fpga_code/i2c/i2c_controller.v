module i2c_controller(
	input wire clk,
	input wire rst,
	input wire [6:0] addr,
	input wire [7:0] data_in,
	input wire enable,
	input wire rw,

	output reg [7:0] data_out,
	output wire ready,

	output wire i2c_sda_out,
	input wire i2c_sda_in,
	inout wire i2c_scl,
  output sda_enable,
  output scl_enable
	)/* synthesis syn_noprune = 1*/;

	localparam IDLE = 0;
	localparam START = 1;
	localparam ADDRESS = 2;
	localparam READ_ACK = 3;
	localparam WRITE_DATA = 4;
	localparam WRITE_ACK = 5;
	localparam READ_DATA = 6;
	localparam READ_ACK2 = 7;
	localparam STOP = 8;
  localparam DELAY = 9;
  localparam DELAY2 = 10;

	localparam DIVIDE_BY = 128;

	reg [7:0] state;
	reg [7:0] saved_addr;
	reg [7:0] saved_data;
	reg [7:0] counter;
	reg [7:0] counter2 = 0;
	reg write_enable;
	reg sda_out;
	reg i2c_scl_enable = 0;
	reg i2c_clk = 1;
  // wire i2c_clk;

	assign ready = ((rst == 0) && (state == IDLE)) ? 1'b1 : 1'b0;
	assign i2c_scl = (i2c_scl_enable == 1'b0 ) ? 1'b1 : i2c_clk;
	assign i2c_sda_out = (write_enable == 1'b1) ? sda_out : 'bz;
  assign sda_enable = write_enable;
  assign scl_enable = i2c_scl_enable;

  // TinyFPGA_B_pll_1MHz pll(
  //   .REFERENCECLK(clk),
  //   .PLLOUTGLOBAL(i2c_clk),
  //   .RESET(1'b1)
  //   );

  reg enable_slow;

	always @(posedge clk) begin
    if(enable)begin
      enable_slow <= 1;
    end
		if(enable_slow && state!=IDLE)begin
			enable_slow <= 0;
		end
		if (counter2 == (DIVIDE_BY/2) - 1) begin
			i2c_clk <= !i2c_clk;
			counter2 <= 0;
		end
		else counter2 <= counter2 + 8'd1;

	end

	always @(negedge i2c_clk, posedge rst) begin
		if(rst == 1) begin
			i2c_scl_enable <= 0;
		end else begin
			if ((state == IDLE) || (state == START) || (state == STOP)) begin
				i2c_scl_enable <= 0;
			end else begin
				i2c_scl_enable <= 1;
			end
		end

	end

  reg [3:0]delay_counter;
	always @(posedge i2c_clk, posedge rst) begin
		if(rst == 1) begin
			state <= IDLE;
		end
		else begin
			case(state)

				IDLE: begin
					if (enable_slow) begin
						state <= START;
						saved_addr <= {addr, rw};
						saved_data <= data_in;
					end else begin
						state <= IDLE;
					end
				end

				START: begin
					counter <= 7;
					state <= ADDRESS;
				end

				ADDRESS: begin
					if (counter == 0) begin
						state <= READ_ACK;
					end else begin
						counter <= counter - 8'd1;
					end
				end

				READ_ACK: begin
					if (i2c_sda_in == 0) begin
						counter <= 7;
						if(saved_addr[0] == 0)begin
							state <= WRITE_DATA;
						end else begin
							state <= READ_DATA;
						end
					end else begin
						state <= STOP;
					end
				end

				WRITE_DATA: begin
					if(counter == 0) begin
						state <= DELAY;
            delay_counter <= 0;
					end else begin
						counter <= counter - 1;
					end
				end

        DELAY: begin
          state <= READ_ACK2;
        end

				READ_ACK2: begin
          if ((i2c_sda_in == 0) && (enable == 1)) begin
            state <= IDLE;
          end else begin
						state <= STOP;
					end
				end

				READ_DATA: begin
					//data_out[counter] <= i2c_sda;
					data_out[counter] <= i2c_sda_in;
					if (counter == 0) begin
						state <= WRITE_ACK;
					end else begin
						counter <= counter - 8'd1;
					end
				end

				WRITE_ACK: begin
					state <= DELAY2;
				end

        DELAY2: begin
          state <= STOP;
        end

				STOP: begin
					state <= IDLE;
				end
			endcase
		end
	end

	always @(negedge i2c_clk, posedge rst) begin
		if(rst == 1) begin
			write_enable <= 1;
			sda_out <= 1;
		end else begin
			case(state)
				IDLE: begin
					write_enable <= 0;
				end

				START: begin
					write_enable <= 1;
					sda_out <= 0;
				end

				ADDRESS: begin
					sda_out <= saved_addr[counter];
				end

				READ_ACK: begin
					write_enable <= 0;
				end

				WRITE_DATA: begin
					write_enable <= 1;
					sda_out <= saved_data[counter];
				end

				WRITE_ACK: begin
					write_enable <= 1;
					sda_out <= 0;
				end

				READ_DATA: begin
					write_enable <= 0;
				end

				STOP: begin
					write_enable <= 1;
					sda_out <= 1;
				end
			endcase
		end
	end

endmodule
