// This module instantiates NUMBER_OF_SENSORS lighthouse sensors

`timescale 1ns/10ps

module DarkRoom #(parameter NUMBER_OF_SENSORS)(
	input clock,
	input reset_n,
	// this is for the avalon interface
	input [NUMBER_OF_SENSORS-1:0] sensor_signals_i,
	// SPI
	output sck_o, // clock
	output ss_n_o, // slave select
	output mosi_o,	// mosi
);

reg [NUMBER_OF_SENSORS-1:0] sensor_data [31:0];

genvar i;
generate
	for(i=0; i<NUMBER_OF_SENSORS; i++) begin
	  lighthouse_sensor lighthouse_sensors(
		.clk(clock),
		.sensor(sensor_signal_i[i]),
	    	.combined_data(sensor_data[i])
	  );
	end
endgenerate 

endmodule

