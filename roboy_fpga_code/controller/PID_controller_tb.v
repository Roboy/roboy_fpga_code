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
reg [31:0] integralNegMax;
reg [31:0] integralPosMax;
reg [31:0] deadBand;
reg update_controller;
reg [31:0] result;

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
	.integralNegMax(integralNegMax),
	.integralPosMax(integralPosMax),
	.deadBand(deadBand),
	.update_controller(update_controller),
	.result(result)
);
	
// setup clock
initial begin 
	clock = 1'b0;
	forever clock = #1 ~clock;
end
	
initial begin
	reset = 1'b1;
	repeat (2) @(posedge clock);   
	reset = 1'b0;
	repeat (1000) @(posedge clock);
	$stop;
end
		
		
endmodule