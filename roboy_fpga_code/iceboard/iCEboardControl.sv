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
		output tx,
		// power out for fans
		output signed [31:0] current_average
);

	parameter NUMBER_OF_MOTORS = 8;
	parameter CLOCK_FREQ_HZ = 50_000_000;

	reg [31:0] baudrate;
	reg [7:0] id[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] duty[NUMBER_OF_MOTORS-1:0];
	reg signed [15:0] Kp[NUMBER_OF_MOTORS-1:0];
	reg signed [15:0] Ki[NUMBER_OF_MOTORS-1:0];
	reg signed [15:0] Kd[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] sp[NUMBER_OF_MOTORS-1:0];
	reg [23:0] neopxl_color[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] PWMLimit[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] IntegralLimit[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] deadband[NUMBER_OF_MOTORS-1:0];
	reg signed [15:0] current[NUMBER_OF_MOTORS-1:0];
	reg signed [15:0] current_limit[NUMBER_OF_MOTORS-1:0];
	reg [7:0] control_mode[NUMBER_OF_MOTORS-1:0];

	// encoder
	reg signed [23:0] encoder0_position[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] encoder1_position[NUMBER_OF_MOTORS-1:0];
	reg signed [23:0] displacement[NUMBER_OF_MOTORS-1:0];

	reg [31:0] error_code[NUMBER_OF_MOTORS-1:0];
	reg [31:0] crc_checksum[NUMBER_OF_MOTORS-1:0];
	reg [31:0] communication_quality[NUMBER_OF_MOTORS-1:0];
	reg [31:0] update_frequency_Hz;

	assign readdata = returnvalue;
	assign waitrequest = (waitFlag && read);
	reg [31:0] returnvalue;
	reg waitFlag;

	wire [7:0] current_motor;
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
					8'h00: returnvalue <= id[motor];
					8'h01: returnvalue <= Kp[motor];
					8'h02: returnvalue <= Ki[motor];
					8'h03: returnvalue <= Kd[motor];
					8'h04: returnvalue <= encoder0_position[motor];
					8'h05: returnvalue <= encoder1_position[motor];
					8'h08: returnvalue <= PWMLimit[motor];
					8'h09: returnvalue <= IntegralLimit[motor];
					8'h0A: returnvalue <= deadband[motor];
					8'h0B: returnvalue <= control_mode[motor];
					8'h0C: returnvalue <= sp[motor];
					8'h0D: returnvalue <= error_code[motor];
					8'h11: returnvalue <= update_frequency_Hz;
					8'h15: returnvalue <= crc_checksum[motor];
					8'h16: returnvalue <= communication_quality[motor];
					8'h17: returnvalue <= duty[motor];
					8'h18: returnvalue <= displacement[motor];
					8'h19: returnvalue <= current[motor];
					8'h1A: returnvalue <= neopxl_color[motor];
					8'h1B: returnvalue <= current_limit[motor];
					8'h1C: returnvalue <= current_average;
					8'h1D: returnvalue <= baudrate;
					default: returnvalue <= 32'hDEADBEEF;
				endcase
				if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
					waitFlag <= 0;
				end
			end
		end
	end

	always @(posedge clk, posedge reset) begin: AVALON_WRITE_INTERFACE
		integer i;
		if (reset == 1) begin
			for(i=0;i<NUMBER_OF_MOTORS;i=i+1) begin
				Kp[i] <= 1;
				Ki[i] <= 0;
				Kd[i] <= 0;
				sp[i] <= 0;
				deadband[i] <= 0;
				control_mode[i] <= 3;
				PWMLimit[i] <= 500;
				IntegralLimit[i] <= 100;
				id[i] <= i+128;
			end
			baudrate <= 1_000_000;
			update_frequency_Hz <= 100;
		end else begin
			if(write && ~waitrequest) begin
				case(addr)
					8'h00: id[motor] <= writedata;
					8'h01: Kp[motor] <= writedata;
					8'h02: Ki[motor] <= writedata;
					8'h03: Kd[motor] <= writedata;
					8'h08: PWMLimit[motor] <= writedata;
					8'h09: IntegralLimit[motor] <= writedata;
					8'h0A: deadband[motor] <= writedata;
					8'h0B: control_mode[motor] <= writedata;
					8'h0C: sp[motor] <= writedata;
					8'h11: update_frequency_Hz <= writedata;
					8'h12: neopxl_color[motor] <= writedata;
					8'h13: current_limit[motor] <= writedata;
					8'h14: baudrate <= writedata;
				endcase
			end
		end
	end

	coms #(NUMBER_OF_MOTORS,CLOCK_FREQ_HZ)com(
		.clk(clk),
		.reset(reset),
		.tx_o(tx),
		.rx_i(~rx),
		.update_frequency_Hz(update_frequency_Hz),
		.baudrate(baudrate),
		.id(id),
		.duty(duty),
		.encoder0_position(encoder0_position),
		.encoder1_position(encoder1_position),
		.displacement(displacement),
		.current(current),
		.current_limit(current_limit),
		.setpoint(sp),
		.neopxl_color(neopxl_color),
		.control_mode(control_mode),
		.Kp(Kp),
		.Ki(Ki),
		.Kd(Kd),
		.PWMLimit(PWMLimit),
		.IntegralLimit(IntegralLimit),
		.deadband(deadband),
		.error_code(error_code),
		.crc_checksum(crc_checksum),
		.communication_quality(communication_quality),
		.current_motor(current_motor),
		.current_average(current_average)
	);

endmodule
