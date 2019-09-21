module ICEboardControl (
		input clk,
		input reset,
		// this is for the avalon interface
		input [15:0] address,
		input write,
		input signed [31:0] writedata,
		input read,
		output signed [31:0] readdata,
		output waitrequest,
		input rx,
		output tx
);
	
	parameter NUMBER_OF_MOTORS = 8;
	parameter CLOCK_FREQ_HZ = 50_000_000;
	parameter BAUDRATE = 1_000_000;
		
	reg signed [31:0] pwm[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] Kp[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] Ki[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] Kd[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] sp[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] PWMLimit[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] IntegralLimit[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] deadband[NUMBER_OF_MOTORS-1:0];
	reg [7:0] control_mode[NUMBER_OF_MOTORS-1:0];

	// encoder 
	reg signed [31:0] encoder0_position[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] encoder1_position[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] encoder0_velocity[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] encoder1_velocity[NUMBER_OF_MOTORS-1:0];
	
	// current
	reg signed [31:0] current_phase1[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] current_phase2[NUMBER_OF_MOTORS-1:0];
	reg signed [31:0] current_phase3[NUMBER_OF_MOTORS-1:0];
	
	reg [31:0] error_code[NUMBER_OF_MOTORS-1:0];
	reg [31:0] crc_checksum[NUMBER_OF_MOTORS-1:0];
	reg [31:0] communication_quality[NUMBER_OF_MOTORS-1:0];
	reg [31:0] update_frequency_Hz;

	assign readdata = returnvalue;
	assign waitrequest = (waitFlag && read);
	reg [31:0] returnvalue;
	reg waitFlag;
	
	wire [7:0] motor;
	wire [7:0] addr;
	assign addr = (address>>8);
	assign motor = (address&8'hFF);

	always @(posedge clk, posedge reset) begin: AVALON_READ_INTERFACE
		if (reset == 1) begin
			waitFlag <= 1;
		end else begin
			waitFlag <= 1;
			if(read) begin
				case(addr)
					8'h00: returnvalue <= 32'hB15B00B5;
					8'h01: returnvalue <= Kp[motor];
					8'h02: returnvalue <= Ki[motor];
					8'h03: returnvalue <= Kd[motor];
					8'h04: returnvalue <= encoder0_position[motor];
					8'h05: returnvalue <= encoder1_position[motor];
					8'h06: returnvalue <= encoder0_velocity[motor];
					8'h07: returnvalue <= encoder1_velocity[motor];
					8'h08: returnvalue <= PWMLimit[motor];
					8'h09: returnvalue <= IntegralLimit[motor];
					8'h0A: returnvalue <= deadband[motor];
					8'h0B: returnvalue <= control_mode[motor];
					8'h0C: returnvalue <= sp[motor];
					8'h0D: returnvalue <= error_code[motor];
					8'h11: returnvalue <= update_frequency_Hz;
					8'h12: returnvalue <= current_phase1[motor];
					8'h13: returnvalue <= current_phase2[motor];
					8'h14: returnvalue <= current_phase3[motor];
					8'h15: returnvalue <= crc_checksum[motor];
					8'h16: returnvalue <= communication_quality[motor];
					8'h17: returnvalue <= pwm[motor];
					default: returnvalue <= 32'hDEADBEEF;
				endcase
				if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
					waitFlag <= 0;
				end
			end
		end
	end
		
	always @(posedge clk, posedge reset) begin: MYO_CONTROL_LOGIC
		integer i;
		if (reset == 1) begin
			for(i=0;i<NUMBER_OF_MOTORS;i=i+1) begin
				Kp[i] <= 1;
				Ki[i] <= 0;
				Kd[i] <= 0;
				sp[i] <= 0;
				deadband[i] <= 0;
				control_mode[i] <= 0;
				PWMLimit[i] <= 127;
				IntegralLimit[i] <= 50;
			end
			update_frequency_Hz <= 100;
		end else begin
			if(write && ~waitrequest) begin
				case(addr)
					8'h01: Kp[motor] <= writedata;
					8'h02: Ki[motor] <= writedata;
					8'h03: Kd[motor] <= writedata;
					8'h08: PWMLimit[motor] <= writedata;
					8'h09: IntegralLimit[motor] <= writedata;
					8'h0A: deadband[motor] <= writedata;
					8'h0B: control_mode[motor] <= writedata;
					8'h0C: sp[motor] <= writedata;
					8'h11: update_frequency_Hz <= writedata;
				endcase
			end
		end 
	end
	
	coms #(NUMBER_OF_MOTORS,CLOCK_FREQ_HZ,BAUDRATE)com(
		.clk(clk),
		.reset(reset),
		.tx_o(tx),
		.rx_i(rx),
		.update_frequency_Hz(update_frequency_Hz),
		.pwm(pwm),
		.encoder0_position(encoder0_position),
		.encoder1_position(encoder1_position),
		.encoder0_velocity(encoder0_velocity),
		.encoder1_velocity(encoder1_velocity),
		.current_phase1(current_phase1),
		.current_phase2(current_phase2),
		.current_phase3(current_phase3),
		.setpoint(sp),
		.control_mode(control_mode),
		.Kp(Kp),
		.Ki(Ki),
		.Kd(Kd),
		.PWMLimit(PWMLimit),
		.IntegralLimit(IntegralLimit),
		.deadband(deadband),
		.error_code(error_code),
		.crc_checksum(crc_checksum),
		.communication_quality(communication_quality)
	);
	
endmodule