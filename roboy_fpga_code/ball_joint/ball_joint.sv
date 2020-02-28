module BallJoint (
    input clk,
    input reset,
    // this is for the avalon interface
    input [15:0] address,
    input write,
    input signed [31:0] writedata,
    input read,
    output signed [31:0] readdata,
    output waitrequest,
    // i2c
    inout scl,
    output sda
  );

  parameter CLOCK_SPEED_HZ = 50_000_000;
  parameter BUS_SPEED_HZ = 100_000;
  parameter NUMBER_OF_SENSORS = 4;

  localparam  RESET = 0, CONTINUOUS_READOUT = 1, READOUT = 2, POSTPROCESS = 3,
    SWITCH_SENSOR = 4, CHANGE_PROTOCOL = 5, WAIT_FOR_TRANSMISSION = 6, READOUT2 = 7;
  reg i2c_reset;
  reg [7:0] state;
  reg [7:0] state_next;
  reg [7:0] current_sensor;
  reg [11:0] temperature[NUMBER_OF_SENSORS-1:0];
  reg [11:0] mag_x[NUMBER_OF_SENSORS-1:0];
  reg [11:0] mag_y[NUMBER_OF_SENSORS-1:0];
  reg [11:0] mag_z[NUMBER_OF_SENSORS-1:0];
  integer update_frequency;
  reg waitFlag;
  reg [31:0] returnvalue;
  assign waitrequest = (state==READOUT || state==READOUT2 || state == POSTPROCESS) || (waitFlag && read);

  wire [7:0] sensor;
	wire [7:0] register;
	assign register = (address>>8);
	assign sensor = (address&8'hFF);

  always @(posedge clk, posedge reset) begin: I2C_CONTROL_LOGIC
  	reg ena_prev;
  	reg [7:0] i;
  	reg [15:0] delay_counter;
  	if (reset == 1) begin
  		data_wd <= 0;
  		ena <= 0;
  		read_only <= 0;
  		number_of_bytes<= 0;
  		i<=0;
      update_frequency <= 100;
      waitFlag <= 1;
  	end else begin
      if(write && ~waitrequest) begin
        case(register)
          8'h00: update_frequency <= writedata;
          8'h01: begin
            current_sensor <= 0;
            state <= SWITCH_SENSOR;
            state_next <= CHANGE_PROTOCOL;
          end
        endcase
      end

      waitFlag <= 1;
			if(read) begin
				case(register)
					8'h00: returnvalue <= mag_x[sensor];
					8'h01: returnvalue <= mag_y[sensor];
					8'h02: returnvalue <= mag_z[sensor];
					8'h03: returnvalue <= temperature[sensor];
					default: returnvalue <= 32'hDEADBEEF;
				endcase
				if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
					waitFlag <= 0;
				end
			end

  		ena_prev <= ena;

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
      if(i2c_reset)begin
        i2c_reset <= 0;
      end

      case(state)
        RESET: begin
          if(current_sensor<NUMBER_OF_SENSORS)begin
            current_sensor <= current_sensor +1;
            state <= SWITCH_SENSOR;
            state_next <= CHANGE_PROTOCOL;
          end else begin
            current_sensor <= 0;
            state <= SWITCH_SENSOR;
            state_next <= CONTINUOUS_READOUT;
          end
        end
        CHANGE_PROTOCOL: begin
          addr <= 7'h35;
          data_wd[31:24] <= 8'h11;
          data_wd[23:16] <= 8'h10;
          number_of_bytes <= 2;
          read_only <= 0;
          rw <= 0;
          ena <= 1;
          state <= WAIT_FOR_TRANSMISSION;
          state_next <= RESET;
        end
        CONTINUOUS_READOUT: begin
          delay_counter<=delay_counter+1;
          if(delay_counter>(CLOCK_SPEED_HZ/update_frequency/NUMBER_OF_SENSORS))begin
            if(~fifo_empty)begin
              fifo_clear <= 1;
            end
            if(current_sensor>NUMBER_OF_SENSORS)begin
              current_sensor <= 0;
            end
            number_of_bytes <= 7;
            read_only <= 1;
            rw <= 1;
            addr <= 7'h35;
            ena <= 1;
            state <= WAIT_FOR_TRANSMISSION;
            state_next <= READOUT;
          end
        end
        READOUT: begin
          mag_x[current_sensor][11:4] <= data_rd[31:24];
          mag_y[current_sensor][11:4] <= data_rd[23:16];
          mag_z[current_sensor][11:4] <= data_rd[15:8];
          temperature[current_sensor][11:4] <= data_rd[7:0];
          fifo_read_ack <= 1;
          state <= READOUT2;
        end
        READOUT2: begin
          mag_x[current_sensor][3:0] <= data_rd[31:28];
          mag_y[current_sensor][3:0] <= data_rd[27:24];
          mag_z[current_sensor][3:0] <= data_rd[19:16];
          temperature[current_sensor][3:2] <= data_rd[23:22];
          fifo_read_ack <= 1;
          state <= POSTPROCESS;
        end
        POSTPROCESS: begin
          current_sensor <= current_sensor + 1;
          state <= CONTINUOUS_READOUT;
        end
        SWITCH_SENSOR: begin
          if(ena==0)begin
            addr <= 7'b1110000;
            data_wd <= 0;
            data_wd[24+current_sensor] <= 1;
            read_only <= 0;
            rw <= 0;
            ena <= 1;
            number_of_bytes <= 1;
            state <= state_next;
          end
        end
        WAIT_FOR_TRANSMISSION: begin
          if(ena==0)begin
            state <= state_next;
          end
        end
      endcase

  	end
  end

  reg [6:0] addr;
  reg rw;
  reg busy;
  reg ack_error;
  reg ena;
  reg [7:0] number_of_bytes;
  wire [7:0] byte_counter;
  reg busy_prev;
  reg [31:0] data_rd;
  reg [31:0] data_read_fifo;
  reg [31:0] data_wd;
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
  	.clock(clk),
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
   .clk(clk),
   .edge_sig(fifo_write_ack),
   .level_sig(fifo_write)
  );

  i2c_master #(CLOCK_SPEED_HZ, BUS_SPEED_HZ) i2c(
  	.clk(clk),
  	.reset_n(~reset || i2c_reset),
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
  	.fifo_write_ack(fifo_write_ack)
  );

endmodule //BallJoint
