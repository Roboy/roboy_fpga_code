// MyoControl logic
// author: Simon Trendel, simon.trendel@tum.de, 2018

`timescale 1ns/10ps

module MYOControl (
	input clock,
	input reset,
	// this is for the avalon interface
	input [7:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
	// these are the spi ports
	output [NUMBER_OF_MOTORS-1:0] ss_n_o,
	input miso,
	output mosi,
	output sck
);

parameter NUMBER_OF_MOTORS = 6 ;

// gains and shit
// p gains
reg signed [15:0] Kp[NUMBER_OF_MOTORS-1:0];
// i gains
reg signed [15:0] Ki[NUMBER_OF_MOTORS-1:0];
// d gains
reg signed [15:0] Kd[NUMBER_OF_MOTORS-1:0];
// setpoints
reg signed [31:0] sp[NUMBER_OF_MOTORS-1:0];
// forward gains
reg signed [15:0] forwardGain[NUMBER_OF_MOTORS-1:0];
// output positive limits
reg signed [15:0] outputPosMax[NUMBER_OF_MOTORS-1:0];
// output negative limits
reg signed [15:0] outputNegMax[NUMBER_OF_MOTORS-1:0];
// integral negative limits
reg signed [15:0] IntegralNegMax[NUMBER_OF_MOTORS-1:0];
// integral positive limits
reg signed [15:0] IntegralPosMax[NUMBER_OF_MOTORS-1:0];
// deadband
reg signed [15:0] deadBand[NUMBER_OF_MOTORS-1:0];
// control mode
reg unsigned [1:0] controller[NUMBER_OF_MOTORS-1:0];
// reset pid_controller
reg reset_controller[NUMBER_OF_MOTORS-1:0];

// pwm output to motors 
wire signed [0:15] pwmRefs[NUMBER_OF_MOTORS-1:0];

// the following is stuff we receive from the motors via spi
// positions of the motors
reg signed [31:0] positions[NUMBER_OF_MOTORS-1:0];
// velocitys of the motors
reg signed [15:0] velocitys[NUMBER_OF_MOTORS-1:0];
// currents of the motors
reg signed [15:0] currents[NUMBER_OF_MOTORS-1:0];
// displacements of the springs
reg [15:0] displacements[NUMBER_OF_MOTORS-1:0];


assign readdata = returnvalue;
assign waitrequest = waitFlag || update_controller;
reg [31:0] returnvalue;
reg waitFlag;

// the following iterface handles read requests via lightweight axi bridge
// the upper 16 bit off the read register define which value we want to read
// the lower 16 bit off the read register define for which motor
always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	if (reset == 1) begin
		waitFlag <= 0;
	end else begin
		if(read) begin
			waitFlag <= 1;
			case(address[31:16])
				0: returnvalue <= Kp[address[15:0]][15:0];
				1: returnvalue <= Ki[address[15:0]][15:0];
				2: returnvalue <= Kd[address[15:0]][15:0];
				3: returnvalue <= sp[address[15:0]][31:0];
				4: returnvalue <= forwardGain[address[15:0]][15:0];
				5: returnvalue <= outputPosMax[address[15:0]][15:0];
				6: returnvalue <= outputNegMax[address[15:0]][15:0];
				7: returnvalue <= IntegralNegMax[address[15:0]][15:0];
				8: returnvalue <= IntegralPosMax[address[15:0]][15:0];
				9: returnvalue <= deadBand[address[15:0]][15:0];
				10: returnvalue <= controller[address[15:0]][1:0];
				default: returnvalue <= 31'hDEADBEEF;
			endcase
		end
		if(waitFlag==1) begin // after one clock cycle the returnvalue should be ready
			waitFlag <= 0;
		end
	end
end
	
reg reset_myo_control;
reg spi_activated;
reg update_controller;

// when spi is done we transfer the results, which would be a bad time to read the values.
assign waitrequest = update_controller;
	
reg [2:0] motor;
reg [2:0] pid_update;
	
always @(posedge clock, posedge reset) begin: MYO_CONTROL_LOGIC
	reg spi_done_prev; 
	reg [3:0]i;
	if (reset == 1) begin
		reset_myo_control <= 0;
		spi_activated <= 0;
		motor <= 0;
		spi_done_prev <= 0;
	end else begin
		update_controller <= 0;
		spi_done_prev <= spi_done;
		if(spi_done_prev==0 && spi_done) begin
			positions[motor][31:0] <= position[0:31];
			velocitys[motor][15:0] <= velocity[0:15];
			currents[motor][15:0] <= current[0:15];
			displacements[motor][15:0] <= displacement[0:15];
			update_controller <= 1;
			pid_update <= motor;
			if(motor==NUMBER_OF_MOTORS) begin
				motor <= 0;
			end else begin
				motor <= motor + 1;
			end
		end
	
		reset_myo_control <= 0;
		
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_reset_controller
			reset_controller[i] <= 0;
		end
	
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if(address[31:16]<=12 && address[15:0]<NUMBER_OF_MOTORS) begin
				case(address[31:16])
					0: Kp[address[15:0]][15:0] <= writedata[15:0];
					1: Ki[address[15:0]][15:0] <= writedata[15:0];
					2: Kd[address[15:0]][15:0] <= writedata[15:0];
					3: sp[address[15:0]][31:0] <= writedata[31:0];
					4: forwardGain[address[15:0]][15:0] <= writedata[15:0];
					5: outputPosMax[address[15:0]][15:0] <= writedata[15:0];
					6: outputNegMax[address[15:0]][15:0] <= writedata[15:0];
					7: IntegralNegMax[address[15:0]][15:0] <= writedata[15:0];
					8: IntegralPosMax[address[15:0]][15:0] <= writedata[15:0];
					9: deadBand[address[15:0]][15:0] <= writedata[15:0];
					10: controller[address[15:0]][1:0] <= writedata[1:0];
					11: reset_myo_control <= (writedata!=0);
					12: spi_activated <= (writedata!=0);
				endcase
			end
		end
	end 
end

wire di_req, wr_ack, do_valid, wren, spi_done, ss_n;
wire [0:15] Word;
wire [15:0] data_out;
wire signed [0:15] pwmRef;
wire signed [0:31] position; 
wire signed [0:15] velocity;
wire signed [0:15] current;
wire [0:15] displacement;
wire signed [0:15] sensor1;
wire signed [0:15] sensor2;

wire motor_line;
assign motor_line = motor;
// the pwmRef signal will be wired to the corresponding pid controller output
assign pwmRef = pwmRefs[motor_line];
assign ss_n_o = ss_n;

SpiControl spi_control(
	.clock(clock),
	.reset(reset_myo_control),
	.di_req(di_req),
	.write_ack(wr_ack),
	.data_read_valid(do_valid),
	.data_read(data_out[15:0]),
	// if spi is activated and we update the previous controller, we start transmission with the next motor
	.start(spi_activated && update_controller),
	.Word(Word[0:15]),
	.wren(wren),
	.spi_done(spi_done),
	.pwmRef(pwmRef),
	.position(position),
	.velocity(velocity),
	.current(current),
	.displacement(displacement),
	.sensor1(sensor1),
	.sensor2(sensor2),
	.ss_n(ss_n)
);

spi_master #(16, 1'b0, 1'b1, 2, 5) spi(
	.sclk_i(clock),
	.pclk_i(clock),
	.rst_i(reset_myo_control),
	.spi_miso_i(miso),
	.di_i(Word[0:15]),
	.wren_i(wren),
	.spi_ssel_o(ss_n),
	.spi_sck_o(sck),
	.spi_mosi_o(mosi),
	.di_req_o(di_req),
	.wr_ack_o(wr_ack),
	.do_valid_o(do_valid),
	.do_o(data_out[15:0])
);

genvar j;
generate 
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : instantiate_pid_controllers
	  PIDController pid_controller(
			.clock(clock),
			.reset(reset_myo_control||reset_controller[j]),
			.Kp(Kp[j]),
			.Kd(Kd[j]),
			.Ki(Ki[j]),
			.sp(sp[j]),
			.forwardGain(forwardGain[j]),
			.outputPosMax(outputPosMax[j]),
			.outputNegMax(outputNegMax[j]),
			.IntegralNegMax(IntegralNegMax[j]),
			.IntegralPosMax(IntegralPosMax[j]),
			.deadBand(deadBand[j]),
			.controller(controller[j]), // position velocity force
			.position(position[j]),
			.velocity(velocity[j]),
			.displacement(displacement[j]),
			.update_controller(pid_update==j && update_controller),
			.pwmRef(pwmRef[j])
		);
	end
endgenerate 


endmodule

