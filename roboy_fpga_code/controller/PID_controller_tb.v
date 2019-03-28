`timescale 1 ns/1 ns
module PID_controller_tb;

reg clock, reset;
reg [31:0] Kp;
reg [31:0] Ki;
reg [31:0] Kd;
reg [31:0] state;
reg [31:0] setpoint;
reg [31:0] outputPosMax;
reg [31:0] outputNegMax;
reg [31:0] integralPosMax;
reg [31:0] integralNegMax;
reg [31:0] deadBandPosMax;
reg [31:0] deadBandNegMax;
reg update_controller;
wire [31:0] result;

PID_controller UUT(
	.clock(clock),
	.reset(reset),
	.Kp(Kp),
	.Kd(Kd),
	.Ki(Ki),
	.state(state),
	.setpoint(setpoint),
	.outputPosMax(outputPosMax),
	.outputNegMax(outputNegMax),
	.integralPosMax(integralPosMax),
	.integralNegMax(integralNegMax),
	.deadBandPosMax(deadBandPosMax),
	.deadBandNegMax(deadBandNegMax),
	.update_controller(update_controller),
	.result(result)
);
	
localparam ADD = 0, SUB = 1, MUL = 2, DIV = 3, I2F = 4, F2I = 5;
	
// setup clock
initial begin 
	clock = 1'b0;
	forever clock = #1 ~clock;
end
	
initial begin
	update_controller = 0;
	reset = 1'b1;
	repeat (2) @(posedge clock);   
	reset = 1'b0;
	Kp = 32'h3dcccccd; // 0.1
	Kd = 0;
	Ki = 0;
	Kd = 32'h3dcccccd; // 0.1
	Ki = 32'h3dcccccd; // 0.1
	state = 0;
	setpoint = 32'h41200000; // 10
//	setpoint = 32'h41100000; // 9
	outputPosMax = 32'h457a0000; // 4000
	outputNegMax = 32'hc57a0000; // -4000
	integralNegMax = 0;
	integralPosMax = 0;
//	deadBandPosMax = 32'h41200000; // 10
	deadBandPosMax = 32'h41100000; // 9
	deadBandNegMax = 32'hc1200000; // -10
	repeat (20) @(posedge clock);
	update_controller = 1;
	repeat (1) @(posedge clock);   
	update_controller = 0;
	repeat (20) @(posedge clock);
	update_controller = 1;
	repeat (1) @(posedge clock);   
	update_controller = 0;
	repeat (20) @(posedge clock);
	update_controller = 1;
	repeat (1) @(posedge clock);   
	update_controller = 0;
	
	$stop;
end
		
		
endmodule