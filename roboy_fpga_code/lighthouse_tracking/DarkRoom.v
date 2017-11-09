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
wire [31:0] crc32_2;
wire [263:0] payload;


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
	(address == 32) ? crc32_2 :
	(address == 33) ? payload[15:0] : // fw_version
	(address == 34) ? payload[47:16] : // ID
	(address == 35) ? payload[63:48] : // fcal.0.phase
	(address == 36) ? payload[79:64] : // fcal.1.phase
	(address == 37) ? payload[95:80] : // fcal.0.tilt
	(address == 38) ? payload[111:96] : // fcal.1.tilt
	(address == 39) ? payload[119:112] : // sys.unlock_count
	(address == 40) ? payload[127:120] : // hw_version
	(address == 41) ? payload[143:128] : // fcal.0.curve
	(address == 42) ? payload[159:144] : // fcal.1.curve
	(address == 43) ? payload[167:160] : // accel.dir_x
	(address == 44) ? payload[175:168] : // accel.dir_y
	(address == 45) ? payload[183:176] : // accel.dir_z
	(address == 46) ? payload[199:184] : // fcal.0.gibphase
	(address == 47) ? payload[215:200] : // fcal.1.gibphase
	(address == 48) ? payload[231:216] : // fcal.0.gibmag
	(address == 49) ? payload[247:232] : // fcal.1.gibmag
	(address == 50) ? payload[255:248] : // mode.current
	(address == 51) ? payload[263:256] : // sys.faults
						  32'hDEAD_BEEF;

assign waitrequest = 0;

// 32 SENSOR CORES
lighthouse_sensor awesome_lighthouse00 (
	.clk(clock),
	.sensor(sensor_signal_i[0]),
	.combined_data(sensor_combined_data_00),
	.led(LED[0])
);

lighthouse_sensor awesome_lighthouse01 (
	.clk(clock),
	.sensor(sensor_signal_i[1]),
	.combined_data(sensor_combined_data_01),
	.led(LED[1])
);

lighthouse_sensor awesome_lighthouse02 (
	.clk(clock),
	.sensor(sensor_signal_i[2]),
	.combined_data(sensor_combined_data_02),
	.led(LED[2]),
	.crc32(crc32_2),
	.payload(payload)
);


lighthouse_sensor awesome_lighthouse03 (
	.clk(clock),
	.sensor(sensor_signal_i[3]),
	.combined_data(sensor_combined_data_03),
	.led(LED[3])
);

lighthouse_sensor awesome_lighthouse04 (
	.clk(clock),
	.sensor(sensor_signal_i[4]),
	.combined_data(sensor_combined_data_04),
	.led(LED[4])
);

lighthouse_sensor awesome_lighthouse05 (
	.clk(clock),
	.sensor(sensor_signal_i[5]),
	.combined_data(sensor_combined_data_05),
	.led(LED[5])
);

lighthouse_sensor awesome_lighthouse06 (
	.clk(clock),
	.sensor(sensor_signal_i[6]),
	.combined_data(sensor_combined_data_06),
	.led(LED[6])
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

