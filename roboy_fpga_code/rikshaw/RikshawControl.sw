module RikshawControl (
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	output [NUMBER_OF_SENSORS-1:0] angle_ss_n_o,
	input angle_miso,
	output angle_mosi,
	output angle_sck,
	input emergency_off,
	output throttle,
	input hall_sensor1,
	input hall_sensor2,
	input hall_sensor3
);

parameter CLOCK_SPEED_HZ = 50_000_000;
localparam CLOCK_DIV_500HZ = 100000;
localparam CLOCK_SPEED_MILLIHZ = CLOCK_SPEED_HZ/1000;
parameter NUMBER_OF_SENSORS = 1;
parameter SAMPLES_TO_AVERAGE = 512;

reg waitFlag;
assign readdata = returnvalue;
assign waitrequest = (waitFlag && read);
reg [31:0] returnvalue;


A1339Interface #(.NUMBER_OF_SENSORS(NUMBER_OF_SENSORS)) a1339_interface();
assign angle_sck = a1339_interface.sck_o;
assign angle_ss_n_o = a1339_interface.ss_n_o;
assign angle_mosi = a1339_interface.mosi_o;
assign a1339_interface.miso_i = angle_miso;
assign a1339_interface.zero_offset = emergency_off;

A1339Control#(CLOCK_SPEED_HZ,NUMBER_OF_SENSORS,SAMPLES_TO_AVERAGE) a1339(
	.clock(clock),
	.reset_n(~reset), 
	.interf(a1339_interface.child)
);


//clk is 50000000  divid it o 500
wire clk_out;
wire [31:0] speed_cnt;

assign counterData = speed_cnt;
assign divclk = clk_out;

reg hall_reset = 0;
reg [31:0] speed;	
 
clock_divider cd1(clock, reset, clk_out);
speed_sub ss1(clk_out,reset,hall_reset, speed_cnt);

always @(posedge hall_sensor1, posedge hall_sensor2, posedge hall_sensor3) begin: SPEED_CALC
	speed = speed_cnt;
	for (int i = 0; i < 2; i = i +1) begin
		wait(!clk_out); 
		wait(clk_out);
	end
	hall_reset = 1;
	wait(!clk_out); 
	wait(clk_out);
	hall_reset = 0;
end

// the following iterface handles read requests via lightweight axi bridge
// the upper 8 bit of the read address define which value we want to read
// the lower 8 bit of the read address define for which motor
always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	reg waitFlag;
	if (reset == 1) begin
		waitFlag <= 1;
	end else begin
		waitFlag <= 1;
		if(read) begin
			case(address>>8)
				8'h00: returnvalue <= a1339_interface.sensor_angle[address[7:0]];
				8'h01: returnvalue <= toggle;
				8'h03: returnvalue <= a1339_interface.sensor_angle_absolute[address[7:0]];
				8'h04: returnvalue <= a1339_interface.sensor_angle_offset[address[7:0]];
				8'h05: returnvalue <= a1339_interface.sensor_angle_relative[address[7:0]];
				8'h06: returnvalue <= a1339_interface.sensor_angle_velocity[address[7:0]];
				8'h07: returnvalue <= a1339_interface.sensor_revolution_counter[address[7:0]];
				8'h08: returnvalue <= speed;
				default: returnvalue <= 32'hDEADBEEF;
			endcase
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end

assign throttle=toggle;
reg toggle;
	
always @(posedge clock, posedge reset) begin: MYO_CONTROL_LOGIC
	reg spi_done_prev; 
	reg [7:0]i;
	reg [31:0] counter;
	if (reset == 1) begin
		toggle <=0 ;
	end else begin
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if((address>>8)<=8'h31 && address[7:0]<NUMBER_OF_SENSORS) begin
				case(address>>8)
					8'h00: toggle <= writedata!=0;
				endcase
			end
		end
	end 
end

endmodule


 module speed_sub(
	input clk, 
	input reset, 
	input reset_hall,
	output[32:0] counter
 );
reg [32:0] counter_up;

always @(posedge clk or posedge reset or posedge reset_hall) begin
		if(reset)
			counter_up <= 32'd0;
		else if(reset_hall)
			counter_up <= 32'd0;
		else if(counter_up > 7000)
			counter_up <= 32'd0;
		else 
			counter_up <= counter_up + 32'd1;
	end 
	assign counter = counter_up;
endmodule


module clock_divider ( 
	input clock,
	input reset,
	output out_clk 
);
	reg out_clk_reg;
	reg [32:0] counter_up;

	always @(posedge clock)
		if (reset) begin
			  out_clk_reg <= 1'b0;
			  counter_up <= 32'd0;
		end else begin
			counter_up <= counter_up + 32'd1;
			if(counter_up == 1000000) begin
				counter_up <= 32'd0;
				out_clk_reg <= ~out_clk_reg;
			end
		end
	assign out_clk = out_clk_reg;
endmodule


