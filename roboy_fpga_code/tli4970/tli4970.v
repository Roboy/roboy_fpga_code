module TLI4970(
		input clk,
		input reset,
		input read,
		input address,
		output signed [31:0] readdata,
		output waitrequest,
		input spi_miso,
		output [NUMBER_OF_SENSORS-1:0] spi_cs,
		output spi_clk
  );
  
	assign readdata = current[address];

	parameter NUMBER_OF_SENSORS = 2;
	parameter CLOCK_FREQ = 50_000_000;
	parameter CLOCK_DIVIDER = 13;
	parameter UPDATE_FREQ = 100;

	reg clk_slow;
	reg [7:0] counter;
	always @ ( posedge clk ) begin: CLK_GENERATOR
	 counter <= counter + 1;
	 if(counter>CLOCK_DIVIDER)begin
		clk_slow <= !clk_slow;
		counter <= 0;
	 end
	end

	reg clk_out;
	reg slave_select;
	assign spi_clk = (slave_select?1:clk_slow)&clk_out;
	
	genvar j;
	generate 
		for(j=0; j<NUMBER_OF_SENSORS; j = j+1) begin : connect_slave_selects
			assign spi_cs[j] = (current_sensor==j?slave_select:1);
		end
	endgenerate 

	reg [7:0] bit_counter;
	reg [15:0] data;
	reg [15:0] delay_counter;
	reg [7:0] state;
	reg [7:0] current_sensor;
	reg signed [15:0] current[NUMBER_OF_SENSORS-1:0];
	wire signed [12:0] current_raw;
   assign current_raw = data[12:0];

	localparam  IDLE = 0, SLAVE_SELECT = 1, CLOCK_DATA = 2, STOP = 3;

	always @(negedge clk_slow) begin: TLI4970_READOUT_LOGIC
	 if(delay_counter>(CLOCK_FREQ/CLOCK_DIVIDER/UPDATE_FREQ/NUMBER_OF_SENSORS))begin
		state <= SLAVE_SELECT;
		delay_counter <= 0;
	 end else begin
		delay_counter <= delay_counter+1;
	 end

	 case(state)
		IDLE: begin
		  slave_select <= 1;
		end
		SLAVE_SELECT: begin
		  if(current_sensor<(NUMBER_OF_SENSORS-1))begin
			current_sensor <= current_sensor+1;
		  end else begin
			current_sensor <= 0;
		  end
		  slave_select <= 0;
		  clk_out <= 1;
		  state <= CLOCK_DATA;
		  bit_counter <= 15;
		end
		CLOCK_DATA: begin
		  data[bit_counter] <= spi_miso;
		  bit_counter <= bit_counter - 1;
		  if(bit_counter==0)begin
			 state <= STOP;
			 clk_out <= 0;
		  end
		end
		STOP: begin
		  if(data[15]==0)begin
			 current[current_sensor] <= current_raw-16'd4096;
		  end
		  state <= IDLE;
		end
	 endcase
	end

  endmodule
