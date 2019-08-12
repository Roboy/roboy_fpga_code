// Interface definition
interface MyoControlInterface #(parameter NUMBER_OF_MOTORS = 1) ();
	// gains and shit
	// p gains
	wire [31:0] Kp_f[NUMBER_OF_MOTORS-1:0];
	// i gains
	wire [31:0] Ki_f[NUMBER_OF_MOTORS-1:0];
	// d gains
	wire [31:0] Kd_f[NUMBER_OF_MOTORS-1:0];
	// setpoints
	wire [31:0] sp_f[NUMBER_OF_MOTORS-1:0];
	// output limits
	wire signed [31:0] outputLimit[NUMBER_OF_MOTORS-1:0];
	// control mode
	wire [2:0] control_mode[NUMBER_OF_MOTORS-1:0];
		// controlflags for each motor
	wire [15:0] controlFlags[NUMBER_OF_MOTORS-1:0];

	// pwm output to motors 
	wire signed [15:0] pwmRefs[NUMBER_OF_MOTORS-1:0];
	

	// the following is stuff we receive from the motors via spi
	// positions of the motors
	wire signed [31:0] positions[NUMBER_OF_MOTORS-1:0];
	// velocitys of the motors
	wire signed [15:0] velocities[NUMBER_OF_MOTORS-1:0];
	// currents of the motors
	wire signed [15:0] currents[NUMBER_OF_MOTORS-1:0];
	// displacements of the springs
	wire signed [31:0] displacements[NUMBER_OF_MOTORS-1:0];

	// raw motor states in float
	wire [31:0] positions_raw_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] velocities_raw_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_raw_f[NUMBER_OF_MOTORS-1:0];

	// converted motor states in float
	wire [31:0] positions_conv_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] velocities_conv_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_conv_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_myo_brick_conv_f[NUMBER_OF_MOTORS-1:0];

	// setpoint errors of the motors in float
	wire [31:0] positions_err_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] velocities_err_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_err_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_myo_brick_err_f[NUMBER_OF_MOTORS-1:0];

	// results of the PD controllers of the motors in float
	wire [31:0] positions_res_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] velocities_res_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_res_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] displacements_myo_brick_res_f[NUMBER_OF_MOTORS-1:0];

	// results of the PD controllers of the motors in int
	wire signed [31:0] positions_res[NUMBER_OF_MOTORS-1:0];
	wire signed [31:0] velocities_res[NUMBER_OF_MOTORS-1:0];
	wire signed [31:0] displacements_res[NUMBER_OF_MOTORS-1:0];
	wire signed [31:0] displacements_myo_brick_res[NUMBER_OF_MOTORS-1:0];

	// encoder multipliers in float
	wire [31:0] pos_encoder_multiplier_f[NUMBER_OF_MOTORS-1:0];
	wire [31:0] dis_encoder_multiplier_f[NUMBER_OF_MOTORS-1:0];

	// freaky freaky
	wire [31:0] update_frequency;
	wire [31:0] actual_update_frequency;
	// From child module perspective				
	modport child(
				inout Kp_f, inout Ki_f, inout Kd_f, inout sp_f, inout outputLimit, inout control_mode, inout controlFlags, 
				inout pwmRefs, inout positions, inout velocities, inout currents, inout displacements, 
				inout positions_raw_f, inout velocities_raw_f, inout displacements_raw_f, 
				inout positions_conv_f, inout velocities_conv_f, inout displacements_conv_f, inout displacements_myo_brick_conv_f,
				inout positions_err_f, inout velocities_err_f, inout displacements_err_f, inout displacements_myo_brick_err_f,
				inout positions_res_f, inout velocities_res_f, inout displacements_res_f, inout displacements_myo_brick_res_f,
				inout positions_res, inout velocities_res, inout displacements_res, inout displacements_myo_brick_res,
				inout pos_encoder_multiplier_f, inout dis_encoder_multiplier_f, inout update_frequency, inout actual_update_frequency
				);
endinterface