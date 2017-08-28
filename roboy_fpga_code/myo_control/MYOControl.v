// myo central control node
// you can read out the registers via avalon bus in the following way:
// #define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
// #define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
// where reg corresponds to the address of the avalon slave

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
	output [6:0] ss_n_o,
	input miso,
	output mosi,
	output sck
);

// gains and shit
// p gains
reg signed [15:0] Kp0;
reg signed [15:0] Kp1;
reg signed [15:0] Kp2;
reg signed [15:0] Kp3;
reg signed [15:0] Kp4;
reg signed [15:0] Kp5;
reg signed [15:0] Kp6;
// d gains
reg signed [15:0] Kd0;
reg signed [15:0] Kd1;
reg signed [15:0] Kd2;
reg signed [15:0] Kd3;
reg signed [15:0] Kd4;
reg signed [15:0] Kd5;
reg signed [15:0] Kd6;
// i gains
reg signed [15:0] Ki0;
reg signed [15:0] Ki1;
reg signed [15:0] Ki2;
reg signed [15:0] Ki3;
reg signed [15:0] Ki4;
reg signed [15:0] Ki5;
reg signed [15:0] Ki6;
// setpoints
reg signed [31:0] sp0;
reg signed [31:0] sp1;
reg signed [31:0] sp2;
reg signed [31:0] sp3;
reg signed [31:0] sp4;
reg signed [31:0] sp5;
reg signed [31:0] sp6;
// forward gains
reg signed [15:0] forwardGain0;
reg signed [15:0] forwardGain1;
reg signed [15:0] forwardGain2;
reg signed [15:0] forwardGain3;
reg signed [15:0] forwardGain4;
reg signed [15:0] forwardGain5;
reg signed [15:0] forwardGain6;
// output positive limits
reg signed [15:0] outputPosMax0;
reg signed [15:0] outputPosMax1;
reg signed [15:0] outputPosMax2;
reg signed [15:0] outputPosMax3;
reg signed [15:0] outputPosMax4;
reg signed [15:0] outputPosMax5;
reg signed [15:0] outputPosMax6;
// output negative limits
reg signed [15:0] outputNegMax0;
reg signed [15:0] outputNegMax1;
reg signed [15:0] outputNegMax2;
reg signed [15:0] outputNegMax3;
reg signed [15:0] outputNegMax4;
reg signed [15:0] outputNegMax5;
reg signed [15:0] outputNegMax6;
// integral negative limits
reg signed [15:0] IntegralNegMax0;
reg signed [15:0] IntegralNegMax1;
reg signed [15:0] IntegralNegMax2;
reg signed [15:0] IntegralNegMax3;
reg signed [15:0] IntegralNegMax4;
reg signed [15:0] IntegralNegMax5;
reg signed [15:0] IntegralNegMax6;
// integral positive limits
reg signed [15:0] IntegralPosMax0;
reg signed [15:0] IntegralPosMax1;
reg signed [15:0] IntegralPosMax2;
reg signed [15:0] IntegralPosMax3;
reg signed [15:0] IntegralPosMax4;
reg signed [15:0] IntegralPosMax5;
reg signed [15:0] IntegralPosMax6;
// deadband
reg signed [15:0] deadBand0;
reg signed [15:0] deadBand1;
reg signed [15:0] deadBand2;
reg signed [15:0] deadBand3;
reg signed [15:0] deadBand4;
reg signed [15:0] deadBand5;
reg signed [15:0] deadBand6;
// control mode
reg unsigned [1:0] controller0;
reg unsigned [1:0] controller1;
reg unsigned [1:0] controller2;
reg unsigned [1:0] controller3;
reg unsigned [1:0] controller4;
reg unsigned [1:0] controller5;
reg unsigned [1:0] controller6;
// reset pid_controller
reg reset_controller0;
reg reset_controller1;
reg reset_controller2;
reg reset_controller3;
reg reset_controller4;
reg reset_controller5;
reg reset_controller6;

assign readdata = 
	((address == 0))? reset_myo_control :
	((address == 1))? spi_activated :
	((address == 2))? position0 :
	((address == 3))? position1 :
	((address == 4))? position2 :
	((address == 5))? position3 :
	((address == 6))? position4 :
	((address == 7))? position5 :
	((address == 8))? position6 :
	((address == 10))? velocity0 :
	((address == 11))? velocity1 :
	((address == 12))? velocity2 :
	((address == 13))? velocity3 :
	((address == 14))? velocity4 :
	((address == 15))? velocity5 :
	((address == 16))? velocity6 :
	((address == 18))? current0 :
	((address == 19))? current1 :
	((address == 20))? current2 :
	((address == 21))? current3 :
	((address == 22))? current4 :
	((address == 23))? current5 :
	((address == 24))? current6 :
	((address == 26))? displacement0 :
	((address == 27))? displacement1 :
	((address == 28))? displacement2 :
	((address == 29))? displacement3 :
	((address == 30))? displacement4 :
	((address == 31))? displacement5 :
	((address == 32))? displacement6 :
	((address == 34))? Kp0:
	((address == 35))? Kp1:
	((address == 36))? Kp2:
	((address == 37))? Kp3:
	((address == 38))? Kp4:
	((address == 39))? Kp5:
	((address == 40))? Kp6:
	((address == 42))? Kd0:
	((address == 43))? Kd1:
	((address == 44))? Kd2:
	((address == 45))? Kd3:
	((address == 46))? Kd4:
	((address == 47))? Kd5:
	((address == 48))? Kd6:
	((address == 50))? Ki0:
	((address == 51))? Ki1:
	((address == 52))? Ki2:
	((address == 53))? Ki3:
	((address == 54))? Ki4:
	((address == 55))? Ki5:
	((address == 56))? Ki6:
	((address == 58))? sp0:
	((address == 59))? sp1:
	((address == 60))? sp2:
	((address == 61))? sp3:
	((address == 62))? sp4:
	((address == 63))? sp5:
	((address == 64))? sp6:
	((address == 66))? forwardGain0:
	((address == 67))? forwardGain1:
	((address == 68))? forwardGain2:
	((address == 69))? forwardGain3:
	((address == 70))? forwardGain4:
	((address == 71))? forwardGain5:
	((address == 72))? forwardGain6:
	((address == 74))? outputPosMax0:
	((address == 75))? outputPosMax1:
	((address == 76))? outputPosMax2:
	((address == 77))? outputPosMax3:
	((address == 78))? outputPosMax4:
	((address == 79))? outputPosMax5:
	((address == 80))? outputPosMax6:
	((address == 82))? outputNegMax0:
	((address == 83))? outputNegMax1:
	((address == 84))? outputNegMax2:
	((address == 85))? outputNegMax3:
	((address == 86))? outputNegMax4:
	((address == 87))? outputNegMax5:
	((address == 88))? outputNegMax6:
	((address == 90))? IntegralNegMax0:
	((address == 91))? IntegralNegMax1:
	((address == 92))? IntegralNegMax2:
	((address == 93))? IntegralNegMax3:
	((address == 94))? IntegralNegMax4:
	((address == 95))? IntegralNegMax5:
	((address == 96))? IntegralNegMax6:
	((address == 98))? IntegralPosMax0:
	((address == 99))? IntegralPosMax1:
	((address == 100))? IntegralPosMax2:
	((address == 101))? IntegralPosMax3:
	((address == 102))? IntegralPosMax4:
	((address == 103))? IntegralPosMax5:
	((address == 104))? IntegralPosMax6:
	((address == 106))? deadBand0:
	((address == 107))? deadBand1:
	((address == 108))? deadBand2:
	((address == 109))? deadBand3:
	((address == 110))? deadBand4:
	((address == 111))? deadBand5:
	((address == 112))? deadBand6:
	((address == 114))? controller0:
	((address == 115))? controller1:
	((address == 116))? controller2:
	((address == 117))? controller3:
	((address == 118))? controller4:
	((address == 119))? controller5:
	((address == 120))? controller6:
	((address == 122))? pwmRef0:
	((address == 123))? pwmRef1:
	((address == 124))? pwmRef2:
	((address == 125))? pwmRef3:
	((address == 126))? pwmRef4:
	((address == 127))? pwmRef5:
	((address == 128))? pwmRef6:
	32'hDEAD_BEEF;

wire signed [0:15] pwmRef0;
wire signed [0:15] pwmRef1;
wire signed [0:15] pwmRef2;
wire signed [0:15] pwmRef3;
wire signed [0:15] pwmRef4;
wire signed [0:15] pwmRef5;
wire signed [0:15] pwmRef6;

// positions for the eight motors
reg signed [31:0] position0;
reg signed [31:0] position1;
reg signed [31:0] position2;
reg signed [31:0] position3;
reg signed [31:0] position4;
reg signed [31:0] position5;
reg signed [31:0] position6;
// velocitys for the eight motors
reg signed [15:0] velocity0;
reg signed [15:0] velocity1;
reg signed [15:0] velocity2;
reg signed [15:0] velocity3;
reg signed [15:0] velocity4;
reg signed [15:0] velocity5;
reg signed [15:0] velocity6;
// currents for the eight motors
reg signed [15:0] current0;
reg signed [15:0] current1;
reg signed [15:0] current2;
reg signed [15:0] current3;
reg signed [15:0] current4;
reg signed [15:0] current5;
reg signed [15:0] current6;
// displacements for the eight motors
reg [15:0] displacement0;
reg [15:0] displacement1;
reg [15:0] displacement2;
reg [15:0] displacement3;
reg [15:0] displacement4;
reg [15:0] displacement5;
reg [15:0] displacement6;
	
reg reset_myo_control;
reg spi_activated;
reg update_controller;

// when spi is done we transfer the results, which would be a bad time to read the values.
assign waitrequest = update_controller;
	
reg [2:0] motor;
reg [2:0] pid_update;
	
always @(posedge clock, posedge reset) begin: MYO_CONTROL_LOGIC
	reg spi_done_prev; 
	if (reset == 1) begin
		reset_myo_control <= 0;
		spi_activated <= 0;
		motor <= 0;
		spi_done_prev <= 0;
	end else begin
		update_controller <= 0;
		spi_done_prev <= spi_done;
		if(spi_done_prev==0 && spi_done) begin
			case(motor)
				0: position0[31:0] <= position[0:31];
				1: position1[31:0] <= position[0:31];
				2: position2[31:0] <= position[0:31];
				3: position3[31:0] <= position[0:31];
				4: position4[31:0] <= position[0:31];
				5: position5[31:0] <= position[0:31];
				6: position6[31:0] <= position[0:31];
			endcase
			case(motor)
				0: velocity0[15:0] <= velocity[0:15];
				1: velocity1[15:0] <= velocity[0:15];
				2: velocity2[15:0] <= velocity[0:15];
				3: velocity3[15:0] <= velocity[0:15];
				4: velocity4[15:0] <= velocity[0:15];
				5: velocity5[15:0] <= velocity[0:15];
				6: velocity6[15:0] <= velocity[0:15];
			endcase
			case(motor)
				0: current0[15:0] <= current[0:15];
				1: current1[15:0] <= current[0:15];
				2: current2[15:0] <= current[0:15];
				3: current3[15:0] <= current[0:15];
				4: current4[15:0] <= current[0:15];
				5: current5[15:0] <= current[0:15];
				6: current6[15:0] <= current[0:15];
			endcase
			case(motor)
				0: displacement0[15:0] <= displacement[0:15];
				1: displacement1[15:0] <= displacement[0:15];
				2: displacement2[15:0] <= displacement[0:15];
				3: displacement3[15:0] <= displacement[0:15];
				4: displacement4[15:0] <= displacement[0:15];
				5: displacement5[15:0] <= displacement[0:15];
				6: displacement6[15:0] <= displacement[0:15];
			endcase
			update_controller <= 1;
			pid_update <= motor;
			if(motor==6)
				motor <= 0;
			else
				motor <= motor + 1;
		end
	
		reset_myo_control <= 0;
		reset_controller0 <= 0;
		reset_controller1 <= 0;
		reset_controller2 <= 0;
		reset_controller3 <= 0;
		reset_controller4 <= 0;
		reset_controller5 <= 0;
		reset_controller6 <= 0;
	
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			case(address)
				0: reset_myo_control <= 1; 
				1: spi_activated		<= (writedata[31:0]!=0); // activate spi if not zero
				34: Kp0 <= writedata[15:0];
				35: Kp1 <= writedata[15:0];
				36: Kp2 <= writedata[15:0];
				37: Kp3 <= writedata[15:0];
				38: Kp4 <= writedata[15:0];
				39: Kp5 <= writedata[15:0];
				40: Kp6 <= writedata[15:0];
				42: Kd0 <= writedata[15:0];
				43: Kd1 <= writedata[15:0];
				44: Kd2 <= writedata[15:0];
				45: Kd3 <= writedata[15:0];
				46: Kd4 <= writedata[15:0];
				47: Kd5 <= writedata[15:0];
				48: Kd6 <= writedata[15:0];
				50: Ki0 <= writedata[15:0];
				51: Ki1 <= writedata[15:0];
				52: Ki2 <= writedata[15:0];
				53: Ki3 <= writedata[15:0];
				54: Ki4 <= writedata[15:0];
				55: Ki5 <= writedata[15:0];
				56: Ki6 <= writedata[15:0];
				58: sp0 <= writedata[31:0];
				59: sp1 <= writedata[31:0];
				60: sp2 <= writedata[31:0];
				61: sp3 <= writedata[31:0];
				62: sp4 <= writedata[31:0];
				63: sp5 <= writedata[31:0];
				64: sp6 <= writedata[31:0];
				66: forwardGain0 <= writedata[15:0];
				67: forwardGain1 <= writedata[15:0];
				68: forwardGain2 <= writedata[15:0];
				69: forwardGain3 <= writedata[15:0];
				70: forwardGain4 <= writedata[15:0];
				71: forwardGain5 <= writedata[15:0];
				72: forwardGain6 <= writedata[15:0];
				74: outputPosMax0 <= writedata[15:0];
				75: outputPosMax1 <= writedata[15:0];
				76: outputPosMax2 <= writedata[15:0];
				77: outputPosMax3 <= writedata[15:0];
				78: outputPosMax4 <= writedata[15:0];
				79: outputPosMax5 <= writedata[15:0];
				80: outputPosMax6 <= writedata[15:0];
				82: outputNegMax0 <= writedata[15:0];
				83: outputNegMax1 <= writedata[15:0];
				84: outputNegMax2 <= writedata[15:0];
				85: outputNegMax3 <= writedata[15:0];
				86: outputNegMax4 <= writedata[15:0];
				87: outputNegMax5 <= writedata[15:0];
				88: outputNegMax6 <= writedata[15:0];
				90: IntegralNegMax0 <= writedata[15:0];
				91: IntegralNegMax1 <= writedata[15:0];
				92: IntegralNegMax2 <= writedata[15:0];
				93: IntegralNegMax3 <= writedata[15:0];
				94: IntegralNegMax4 <= writedata[15:0];
				95: IntegralNegMax5 <= writedata[15:0];
				96: IntegralNegMax6 <= writedata[15:0];
				98: IntegralPosMax0 <= writedata[15:0];
				99: IntegralPosMax1 <= writedata[15:0];
				100: IntegralPosMax2 <= writedata[15:0];
				101: IntegralPosMax3 <= writedata[15:0];
				102: IntegralPosMax4 <= writedata[15:0];
				103: IntegralPosMax5 <= writedata[15:0];
				104: IntegralPosMax6 <= writedata[15:0];
				106: deadBand0 <= writedata[15:0];
				107: deadBand1 <= writedata[15:0];
				108: deadBand2 <= writedata[15:0];
				109: deadBand3 <= writedata[15:0];
				110: deadBand4 <= writedata[15:0];
				111: deadBand5 <= writedata[15:0];
				112: deadBand6 <= writedata[15:0];
				114: controller0 <= writedata[1:0];
				115: controller1 <= writedata[1:0];
				116: controller2 <= writedata[1:0];
				117: controller3 <= writedata[1:0];
				118: controller4 <= writedata[1:0];
				119: controller5 <= writedata[1:0];
				120: controller6 <= writedata[1:0];
				130: reset_controller0 <= 1;
				131: reset_controller1 <= 1;
				132: reset_controller2 <= 1;
				133: reset_controller3 <= 1;
				134: reset_controller4 <= 1;
				135: reset_controller5 <= 1;
				136: reset_controller6 <= 1;
			endcase 
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

// the pwmRef signal will be wired to the corresponding pid controller output
assign pwmRef = 
(motor==0)?pwmRef0:
(motor==1)?pwmRef1:
(motor==2)?pwmRef2:
(motor==3)?pwmRef3:
(motor==4)?pwmRef4:
(motor==5)?pwmRef5:
(motor==6)?pwmRef6:
0;

assign ss_n_o[0] = (motor==0?ss_n:1);
assign ss_n_o[1] = (motor==1?ss_n:1);
assign ss_n_o[2] = (motor==2?ss_n:1);
assign ss_n_o[3] = (motor==3?ss_n:1);
assign ss_n_o[4] = (motor==4?ss_n:1);
assign ss_n_o[5] = (motor==5?ss_n:1);
assign ss_n_o[6] = (motor==6?ss_n:1);

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

PIDController pid_controller0(
	.clock(clock),
	.reset(reset_myo_control||reset_controller0),
	.Kp(Kp0),
	.Kd(Kd0),
	.Ki(Ki0),
	.sp(sp0),
	.forwardGain(forwardGain0),
	.outputPosMax(outputPosMax0),
	.outputNegMax(outputNegMax0),
	.IntegralNegMax(IntegralNegMax0),
	.IntegralPosMax(IntegralPosMax0),
	.deadBand(deadBand0),
	.controller(controller0), // position velocity force
	.position(position0),
	.velocity(velocity0),
	.displacement(displacement0),
	.update_controller(pid_update==0 && update_controller),
	.pwmRef(pwmRef0)
);

PIDController pid_controller1(
	.clock(clock),
	.reset(reset_myo_control||reset_controller1),
	.Kp(Kp1),
	.Kd(Kd1),
	.Ki(Ki1),
	.sp(sp1),
	.forwardGain(forwardGain1),
	.outputPosMax(outputPosMax1),
	.outputNegMax(outputNegMax1),
	.IntegralNegMax(IntegralNegMax1),
	.IntegralPosMax(IntegralPosMax1),
	.deadBand(deadBand1),
	.controller(controller1), // position velocity force
	.position(position1),
	.velocity(velocity1),
	.displacement(displacement1),
	.update_controller(pid_update==1 && update_controller),
	.pwmRef(pwmRef1)
);

PIDController pid_controller2(
	.clock(clock),
	.reset(reset_myo_control||reset_controller2),
	.Kp(Kp2),
	.Kd(Kd2),
	.Ki(Ki2),
	.sp(sp2),
	.forwardGain(forwardGain2),
	.outputPosMax(outputPosMax2),
	.outputNegMax(outputNegMax2),
	.IntegralNegMax(IntegralNegMax2),
	.IntegralPosMax(IntegralPosMax2),
	.deadBand(deadBand2),
	.controller(controller2), // position velocity force
	.position(position2),
	.velocity(velocity2),
	.displacement(displacement2),
	.update_controller(pid_update==2 && update_controller),
	.pwmRef(pwmRef2)
);

PIDController pid_controller3(
	.clock(clock),
	.reset(reset_myo_control||reset_controller3),
	.Kp(Kp3),
	.Kd(Kd3),
	.Ki(Ki3),
	.sp(sp3),
	.forwardGain(forwardGain3),
	.outputPosMax(outputPosMax3),
	.outputNegMax(outputNegMax3),
	.IntegralNegMax(IntegralNegMax3),
	.IntegralPosMax(IntegralPosMax3),
	.deadBand(deadBand3),
	.controller(controller3), // position velocity force
	.position(position3),
	.velocity(velocity3),
	.displacement(displacement3),
	.update_controller(pid_update==3 && update_controller),
	.pwmRef(pwmRef3)
);

PIDController pid_controller4(
	.clock(clock),
	.reset(reset_myo_control||reset_controller4),
	.Kp(Kp4),
	.Kd(Kd4),
	.Ki(Ki4),
	.sp(sp4),
	.forwardGain(forwardGain4),
	.outputPosMax(outputPosMax4),
	.outputNegMax(outputNegMax4),
	.IntegralNegMax(IntegralNegMax4),
	.IntegralPosMax(IntegralPosMax4),
	.deadBand(deadBand4),
	.controller(controller4), // position velocity force
	.position(position4),
	.velocity(velocity4),
	.displacement(displacement4),
	.update_controller(pid_update==4 && update_controller),
	.pwmRef(pwmRef4)
);

PIDController pid_controller5(
	.clock(clock),
	.reset(reset_myo_control||reset_controller5),
	.Kp(Kp5),
	.Kd(Kd5),
	.Ki(Ki5),
	.sp(sp5),
	.forwardGain(forwardGain5),
	.outputPosMax(outputPosMax5),
	.outputNegMax(outputNegMax5),
	.IntegralNegMax(IntegralNegMax5),
	.IntegralPosMax(IntegralPosMax5),
	.deadBand(deadBand5),
	.controller(controller5), // position velocity force
	.position(position5),
	.velocity(velocity5),
	.displacement(displacement5),
	.update_controller(pid_update==5 && update_controller),
	.pwmRef(pwmRef5)
);

PIDController pid_controller6(
	.clock(clock),
	.reset(reset_myo_control||reset_controller6),
	.Kp(Kp6),
	.Kd(Kd6),
	.Ki(Ki6),
	.sp(sp6),
	.forwardGain(forwardGain6),
	.outputPosMax(outputPosMax6),
	.outputNegMax(outputNegMax6),
	.IntegralNegMax(IntegralNegMax6),
	.IntegralPosMax(IntegralPosMax6),
	.deadBand(deadBand6),
	.controller(controller6), // position velocity force
	.position(position6),
	.velocity(velocity6),
	.displacement(displacement6),
	.update_controller(pid_update==6 && update_controller),
	.pwmRef(pwmRef6)
);

endmodule

