// pwm_avalon_bridge node
// you can read out the registers via avalon bus in the following way:
// #define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
// #define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
// where reg corresponds to the address of the avalon slave

`timescale 1ns/10ps

module pwm_avalon_bridge (
	input clock,
	input reset,
	// this is for the avalon interface
	input [15:0] address,
	input write,
	input signed [31:0] writedata,
	// these are the pwm ports
	output [NUMBER_OF_MOTORS-1:0] PWM
);

parameter NUMBER_OF_MOTORS = 6;
parameter CLOCK_SPEED_HZ = 50_000_000;
parameter PWM_FREQ = 60;
parameter PWM_PAUSE_FREQ = 50;
parameter PWM_RESOLUTION = 12;
parameter PWM_PHASES = 1;

reg [31:0] i;
reg [31:0] duty[NUMBER_OF_MOTORS-1:0];
reg [NUMBER_OF_MOTORS-1:0] ena;
	
always @(posedge clock, posedge reset) begin: PWM_CONTROL_LOGIC
	if (reset == 1) begin 
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_pwm_controller
			duty[i] <= 0;
			ena[i] <= 0;
		end
	end else begin
		if(write) begin
			if(address[7:0]<NUMBER_OF_MOTORS) begin
				duty[address[7:0]] <= writedata; 
				ena[address[7:0]] <= 1;
			end
		end
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_pwm_controller_after_enable
			if(ena[i]==1) begin
				ena[i] <= 0;
			end
		end
	end 
end

genvar j;
generate 
	for(j=0; j<NUMBER_OF_MOTORS; j = j+1) begin : instantiate_pwm_controllers
	  pwm #(CLOCK_SPEED_HZ, PWM_FREQ, PWM_PAUSE_FREQ, PWM_RESOLUTION, PWM_PHASES)  motor(
			.clk(clock), 					//system clock
			.reset_n(~reset),				//asynchronous reset
			.ena(ena[j]),					//latches in new duty cycle
			.duty(duty[j]),					//duty cycle
			.pwm_out(PWM[j])				//pwm outputs
		);
	end
endgenerate 



endmodule