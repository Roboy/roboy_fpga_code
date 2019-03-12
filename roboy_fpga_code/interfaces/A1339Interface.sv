// Interface definition
interface A1339Interface #(parameter NUMBER_OF_SENSORS = 1) ();
	wire sck_o;
	wire [NUMBER_OF_SENSORS-1:0]ss_n_o;
	wire mosi_o;
	wire miso_i;
	wire zero_offset;
	wire signed [31:0] sensor_angle[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] sensor_angle_absolute[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] sensor_angle_offset[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] sensor_angle_relative[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] sensor_angle_velocity[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] sensor_revolution_counter[NUMBER_OF_SENSORS-1:0];
	wire signed [31:0] update_freq;
	reg [NUMBER_OF_SENSORS-1:0]cycle;
	// From child module perspective				
	modport child(
				output sensor_angle, output sensor_angle_absolute, output sensor_angle_offset,
				output sensor_angle_relative, output sensor_angle_velocity, output sensor_revolution_counter, 
				output cycle, input zero_offset, output sck_o, output ss_n_o, output mosi_o, input miso_i, input update_freq
				);
endinterface