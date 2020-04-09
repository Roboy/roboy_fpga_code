module iCEbusControl (
		input clk,
		input reset,
		// this is for the avalon interface
		input [9:0] address,
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

	parameter iceboard_coms = 0;
	parameter arm_coms = 0;
	parameter NUMBER_OF_MOTORS = 10;
	parameter CLOCK_FREQ_HZ = 50_000_000;

	reg [31:0] baudrate[NUMBER_OF_MOTORS-1:0];
	reg [7:0] id[NUMBER_OF_MOTORS-1:0];
	reg [7:0] control_mode[NUMBER_OF_MOTORS-1:0];
	reg [23:0] neopxl_color[NUMBER_OF_MOTORS-1:0];
	
	generate
		if(iceboard_coms)begin
			reg signed [23:0] duty[NUMBER_OF_MOTORS-1:0];
			reg signed [15:0] Kp[NUMBER_OF_MOTORS-1:0];
			reg signed [15:0] Ki[NUMBER_OF_MOTORS-1:0];
			reg signed [15:0] Kd[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] sp[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] PWMLimit[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] IntegralLimit[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] deadband[NUMBER_OF_MOTORS-1:0];
			reg signed [15:0] current[NUMBER_OF_MOTORS-1:0];
			reg signed [15:0] current_limit[NUMBER_OF_MOTORS-1:0];
			// encoder
			reg signed [23:0] encoder0_position[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] encoder1_position[NUMBER_OF_MOTORS-1:0];
			reg signed [23:0] displacement[NUMBER_OF_MOTORS-1:0];
		end else begin
			reg signed [31:0] duty[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] Kp[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] Ki[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] Kd[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] sp[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] PWMLimit[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] IntegralLimit[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] deadband[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] current[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] current_limit[NUMBER_OF_MOTORS-1:0];
			// encoder
			reg signed [31:0] encoder0_position[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] encoder1_position[NUMBER_OF_MOTORS-1:0];
			reg signed [31:0] displacement[NUMBER_OF_MOTORS-1:0];
		end
	endgenerate

	reg [31:0] error_code[NUMBER_OF_MOTORS-1:0];
	reg [31:0] crc_checksum[NUMBER_OF_MOTORS-1:0];
	reg [31:0] communication_quality[NUMBER_OF_MOTORS-1:0];
	reg [31:0] update_frequency_Hz;

	assign readdata = returnvalue;
	assign waitrequest = (waitFlag && read);
	reg [31:0] returnvalue;
	reg waitFlag;

	wire [4:0] motor;
	wire [4:0] addr;
	assign addr = (address>>5);
	assign motor = (address&5'hF);

	always @(posedge clk, posedge reset) begin: AVALON_READ_INTERFACE
		if (reset == 1) begin
			waitFlag <= 1;
		end else begin
			waitFlag <= 1;
			if(read) begin
				case(addr)
					5'h0: returnvalue <= id[motor];
					5'h1: returnvalue <= Kp[motor];
					5'h2: returnvalue <= Ki[motor];
					5'h3: returnvalue <= Kd[motor];
					5'h4: returnvalue <= encoder0_position[motor];
					5'h5: returnvalue <= encoder1_position[motor];
					5'h6: returnvalue <= PWMLimit[motor];
					5'h7: returnvalue <= IntegralLimit[motor];
					5'h8: returnvalue <= deadband[motor];
					5'h9: returnvalue <= control_mode[motor];
					5'hA: returnvalue <= sp[motor];
					5'hB: returnvalue <= error_code[motor];
					5'hC: returnvalue <= update_frequency_Hz;
					5'hD: returnvalue <= crc_checksum[motor];
					5'hE: returnvalue <= communication_quality[motor];
					5'hF: returnvalue <= duty[motor];
					5'h10: returnvalue <= displacement[motor];
					5'h11: returnvalue <= current[motor];
					5'h12: returnvalue <= neopxl_color[motor];
					5'h13: returnvalue <= current_limit[motor];
					5'h14: returnvalue <= current_average;
					5'h15: returnvalue <= baudrate[motor];
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
				control_mode[i] <= 0;
				PWMLimit[i] <= 500;
				IntegralLimit[i] <= 100;
				id[i] <= i+128;
				if(iceboard_coms)begin
					baudrate[i] <= 2_000_000;
				end else if(arm_coms)begin
					if(i<6)begin // m3
						baudrate[i] <= 57600;
					end else begin // openbionics
						baudrate[i] <= 57600;
					end
				end
			end

			update_frequency_Hz <= 100;
		end else begin
			if(write && ~waitrequest) begin
				case(addr)
					5'h0: id[motor] <= writedata;
					5'h1: Kp[motor] <= writedata;
					5'h2: Ki[motor] <= writedata;
					5'h3: Kd[motor] <= writedata;
					5'h4: PWMLimit[motor] <= writedata;
					5'h5: IntegralLimit[motor] <= writedata;
					5'h6: deadband[motor] <= writedata;
					5'h7: control_mode[motor] <= writedata;
					5'h8: sp[motor] <= writedata;
					5'h9: update_frequency_Hz <= writedata;
					5'hA: neopxl_color[motor] <= writedata;
					5'hB: current_limit[motor] <= writedata;
					5'hC: baudrate[motor] <= writedata;
				endcase
			end
		end
	end

generate
	if(iceboard_coms)begin
		iCEboardComs #(NUMBER_OF_MOTORS,CLOCK_FREQ_HZ)com(
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
			.current_average(current_average)
		);
	end else if(arm_coms) begin
		ArmBusComs #(NUMBER_OF_MOTORS,CLOCK_FREQ_HZ)com(
			.clk(clk),
			.reset(reset),
			.tx_o(tx),
			.rx_i(rx),
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
	end
endgenerate

endmodule
