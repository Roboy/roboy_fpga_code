// DarkRoom node
// you can read out the registers via avalon bus in the following way:
// #define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
// #define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
// where reg corresponds to the address of the avalon slave

`timescale 1ns/10ps

module DarkRoom (
	input clock,
	input reset_n,
	// this is for the avalon interface
	input [5:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest,
//	output interrupt_sender_irq,
	// these are the spi ports
	input [31:0] sensor_signal_i,
	output [6:0] LED
);

// Arrays would be nice ...

// 32 SENSOR RESULTS
wire [31:0] sensor_combined_data_00;
wire [31:0] sensor_combined_data_01;
wire [31:0] sensor_combined_data_02;
wire [31:0] sensor_combined_data_03;
wire [31:0] sensor_combined_data_04;
wire [31:0] sensor_combined_data_05;
wire [31:0] sensor_combined_data_06;
wire [31:0] sensor_combined_data_07;
wire [31:0] sensor_combined_data_08;
wire [31:0] sensor_combined_data_09;
wire [31:0] sensor_combined_data_10;
wire [31:0] sensor_combined_data_11;
wire [31:0] sensor_combined_data_12;
wire [31:0] sensor_combined_data_13;
wire [31:0] sensor_combined_data_14;
wire [31:0] sensor_combined_data_15;
wire [31:0] sensor_combined_data_16;
wire [31:0] sensor_combined_data_17;
wire [31:0] sensor_combined_data_18;
wire [31:0] sensor_combined_data_19;
wire [31:0] sensor_combined_data_20;
wire [31:0] sensor_combined_data_21;
wire [31:0] sensor_combined_data_22;
wire [31:0] sensor_combined_data_23;
wire [31:0] sensor_combined_data_24;
wire [31:0] sensor_combined_data_25;
wire [31:0] sensor_combined_data_26;
wire [31:0] sensor_combined_data_27;
wire [31:0] sensor_combined_data_28;
wire [31:0] sensor_combined_data_29;
wire [31:0] sensor_combined_data_30;
wire [31:0] sensor_combined_data_31;


// 32 OUTPUTS TO THE ARM CORE
assign readdata = 
	(address == 0)  ? sensor_combined_data_00 :
	(address == 1)  ? sensor_combined_data_01 :
	(address == 2)  ? sensor_combined_data_02 :
	(address == 3)  ? sensor_combined_data_03 :
	(address == 4)  ? sensor_combined_data_04 :
	(address == 5)  ? sensor_combined_data_05 :
	(address == 6)  ? sensor_combined_data_06 :
	(address == 7)  ? sensor_combined_data_07 :
	(address == 8)  ? sensor_combined_data_08 :
	(address == 9)  ? sensor_combined_data_09 :
	(address == 10) ? sensor_combined_data_10 :
	(address == 11) ? sensor_combined_data_11 :
	(address == 12) ? sensor_combined_data_12 :
	(address == 13) ? sensor_combined_data_13 :
	(address == 14) ? sensor_combined_data_14 :
	(address == 15) ? sensor_combined_data_15 :
	(address == 16) ? sensor_combined_data_16 :
	(address == 17) ? sensor_combined_data_17 :
	(address == 18) ? sensor_combined_data_18 :
	(address == 19) ? sensor_combined_data_19 :
	(address == 20) ? sensor_combined_data_20 :
	(address == 21) ? sensor_combined_data_21 :
	(address == 22) ? sensor_combined_data_22 :
	(address == 23) ? sensor_combined_data_23 :
	(address == 24) ? sensor_combined_data_24 :
	(address == 25) ? sensor_combined_data_25 :
	(address == 26) ? sensor_combined_data_26 :
	(address == 27) ? sensor_combined_data_27 :
	(address == 28) ? sensor_combined_data_28 :
	(address == 29) ? sensor_combined_data_29 :
	(address == 30) ? sensor_combined_data_30 :
	(address == 31) ? sensor_combined_data_31 :
						  32'hDEAD_BEEF;

assign waitrequest = 0;

// 32 SENSOR CORES
lighthouse_sensor awesome_lighthouse00 (
	.clk(clock),
	.sensor(sensor_signal_i[0]),
	.combined_data(sensor_combined_data_00)
);

lighthouse_sensor awesome_lighthouse01 (
	.clk(clock),
	.sensor(sensor_signal_i[1]),
	.combined_data(sensor_combined_data_01)
);

lighthouse_sensor awesome_lighthouse02 (
	.clk(clock),
	.sensor(sensor_signal_i[2]),
	.combined_data(sensor_combined_data_02)
);


lighthouse_sensor awesome_lighthouse03 (
	.clk(clock),
	.sensor(sensor_signal_i[3]),
	.combined_data(sensor_combined_data_03)
);

lighthouse_sensor awesome_lighthouse04 (
	.clk(clock),
	.sensor(sensor_signal_i[4]),
	.combined_data(sensor_combined_data_04)
);

lighthouse_sensor awesome_lighthouse05 (
	.clk(clock),
	.sensor(sensor_signal_i[5]),
	.combined_data(sensor_combined_data_05)
);

lighthouse_sensor awesome_lighthouse06 (
	.clk(clock),
	.sensor(sensor_signal_i[6]),
	.combined_data(sensor_combined_data_06)
);

lighthouse_sensor awesome_lighthouse07 (
	.clk(clock),
	.sensor(sensor_signal_i[7]),
	.combined_data(sensor_combined_data_07)
);

lighthouse_sensor awesome_lighthouse08 (
	.clk(clock),
	.sensor(sensor_signal_i[8]),
	.combined_data(sensor_combined_data_08)
);

lighthouse_sensor awesome_lighthouse09 (
	.clk(clock),
	.sensor(sensor_signal_i[9]),
	.combined_data(sensor_combined_data_09)
);

lighthouse_sensor awesome_lighthouse10 (
	.clk(clock),
	.sensor(sensor_signal_i[10]),
	.combined_data(sensor_combined_data_10)
);

lighthouse_sensor awesome_lighthouse11 (
	.clk(clock),
	.sensor(sensor_signal_i[11]),
	.combined_data(sensor_combined_data_11)
);

lighthouse_sensor awesome_lighthouse12 (
	.clk(clock),
	.sensor(sensor_signal_i[12]),
	.combined_data(sensor_combined_data_12)
);

lighthouse_sensor awesome_lighthouse13 (
	.clk(clock),
	.sensor(sensor_signal_i[13]),
	.combined_data(sensor_combined_data_13)
);

lighthouse_sensor awesome_lighthouse14 (
	.clk(clock),
	.sensor(sensor_signal_i[14]),
	.combined_data(sensor_combined_data_14)
);

lighthouse_sensor awesome_lighthouse15 (
	.clk(clock),
	.sensor(sensor_signal_i[15]),
	.combined_data(sensor_combined_data_15)
);

lighthouse_sensor awesome_lighthouse16 (
	.clk(clock),
	.sensor(sensor_signal_i[16]),
	.combined_data(sensor_combined_data_16)
);

lighthouse_sensor awesome_lighthouse17 (
	.clk(clock),
	.sensor(sensor_signal_i[17]),
	.combined_data(sensor_combined_data_17)
);

lighthouse_sensor awesome_lighthouse18 (
	.clk(clock),
	.sensor(sensor_signal_i[18]),
	.combined_data(sensor_combined_data_18)
);

lighthouse_sensor awesome_lighthouse19 (
	.clk(clock),
	.sensor(sensor_signal_i[19]),
	.combined_data(sensor_combined_data_19)
);

lighthouse_sensor awesome_lighthouse20 (
	.clk(clock),
	.sensor(sensor_signal_i[20]),
	.combined_data(sensor_combined_data_20)
);

lighthouse_sensor awesome_lighthouse21 (
	.clk(clock),
	.sensor(sensor_signal_i[21]),
	.combined_data(sensor_combined_data_21)
);

lighthouse_sensor awesome_lighthouse22 (
	.clk(clock),
	.sensor(sensor_signal_i[22]),
	.combined_data(sensor_combined_data_22)
);

lighthouse_sensor awesome_lighthouse23 (
	.clk(clock),
	.sensor(sensor_signal_i[23]),
	.combined_data(sensor_combined_data_23)
);

lighthouse_sensor awesome_lighthouse24 (
	.clk(clock),
	.sensor(sensor_signal_i[24]),
	.combined_data(sensor_combined_data_24)
);

lighthouse_sensor awesome_lighthouse25 (
	.clk(clock),
	.sensor(sensor_signal_i[25]),
	.combined_data(sensor_combined_data_25)
);

lighthouse_sensor awesome_lighthouse26 (
	.clk(clock),
	.sensor(sensor_signal_i[26]),
	.combined_data(sensor_combined_data_26)
);

lighthouse_sensor awesome_lighthouse27 (
	.clk(clock),
	.sensor(sensor_signal_i[27]),
	.combined_data(sensor_combined_data_27)
);

lighthouse_sensor awesome_lighthouse28 (
	.clk(clock),
	.sensor(sensor_signal_i[28]),
	.combined_data(sensor_combined_data_28)
);

lighthouse_sensor awesome_lighthouse29 (
	.clk(clock),
	.sensor(sensor_signal_i[29]),
	.combined_data(sensor_combined_data_29)
);

lighthouse_sensor awesome_lighthouse30 (
	.clk(clock),
	.sensor(sensor_signal_i[30]),
	.combined_data(sensor_combined_data_30)
);

lighthouse_sensor awesome_lighthouse31 (
	.clk(clock),
	.sensor(sensor_signal_i[31]),
	.combined_data(sensor_combined_data_31)
);



endmodule

