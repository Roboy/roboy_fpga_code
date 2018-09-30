`timescale 1 ns/1 ns
module XL320_tb;

reg clock, reset;
wire serial_io;
reg [15:0] address;
reg write;
reg signed [31:0] writedata;
reg read;
wire signed [31:0] readdata;
wire waitrequest; 

localparam DXL_INST_PING          = 8'h01; /**< checks if ID is associated to a Device */
localparam DXL_INST_READ          = 8'h02; /**< read data from the Device */
localparam DXL_INST_WRITE         = 8'h03; /**< write data on the Device */
localparam DXL_INST_REG_WRITE     = 8'h04; /**< registers the write instruction to a standby status */
localparam DXL_INST_ACTION        = 8'h05; /**< executes the write instruction previously registered */
localparam DXL_INST_FACTORY_RESET = 8'h06; /**< resets the Control Table to its initial factory default settings */
localparam DXL_INST_REBOOT        = 8'h08; /**< reboot the Device */
localparam DXL_INST_STATUS        = 8'h55; /**< Return Instruction for the Instruction Packet */
localparam DXL_INST_SYNC_READ     = 8'h82; /**< (Multiple devices) read data with same Address and length at once */
localparam DXL_INST_SYNC_WRITE    = 8'h83; /**< (Multiple devices) write data on the same Address and length at once */
localparam DXL_INST_BULK_READ     = 8'h92; /**< (Multiple devices) read data from different Addresses and lengths at once */
localparam DXL_INST_BULK_WRITE    = 8'h93; /**< (Multiple devices) write data on different Addresses and lengths at once */

	/*EEPROM Area*/
localparam MODEL_NUMBER             = 0; /**< Model number [R] (default=350) */
localparam VERSION                  = 2; /**< Information on the version of firmware [R] */
localparam ID                       = 3; /**< ID of Dynamixel [RW] (default=1 ; min=0 ; max=252) */
localparam BAUD_RATE                = 4; /**< Baud Rate of Dynamixel [RW] (default=3 ; min=0 ; max=3) 0: 9600; 1:57600; 2:115200; 3:1Mbps*/
localparam RETURN_DELAY_TIME        = 5; /**< Return Delay Time [RW] (default=250 ; min=0 ; max=254) */
localparam CW_ANGLE_LIMIT           = 6; /**< clockwise Angle Limit [RW] (default=0 ; min=0 ; max=1023) */
localparam CCW_ANGLE_LIMIT          = 8; /**< counterclockwise Angle Limit [RW] (default=1023 ; min=0 ; max=1023) */
localparam CONTROL_MODE             = 11; /**< Control Mode [RW] (default=2 ; min=1 ; max=2) */
localparam LIMIT_TEMPERATURE        = 12; /**< Internal Limit Temperature [RW] (default=65 ; min=0 ; max=150) */
localparam LOWER_LIMIT_VOLTAGE      = 13; /**< Lowest Limit Voltage [RW] (default=60 ; min=50 ; max=250) */
localparam UPPPER_LIMIT_VOLTAGE     = 14; /**< Upper Limit Voltage [RW] (default=90 ; min=50 ; max=250) */
localparam MAX_TORQUE               = 15; /**< Lowest byte of Max. Torque [RW] (default=1023 ; min=0 ; max=1023) */
localparam RETURN_LEVEL             = 17; /**< Return Level [RW] (default=2 ; min=0 ; max=2) */
localparam ALARM_SHUTDOWN           = 18; /**< Shutdown for Alarm [RW] (default=3 ; min=0 ; max=7) */
	/*RAM Area*/
localparam TORQUE_ENABLE            = 24; /**< Torque On/Off [RW] (default=0 ; min=0 ; max=1) */
localparam LED                      = 25; /**< LED On/Off [RW] (default=0 ; min=0 ; max=7) */
localparam D_GAIN     				  = 27; /**< D Gain [RW] (default=0 ; min=0 ; max=254) */
localparam I_GAIN                   = 28; /**< I Gain [RW] (default=0 ; min=0 ; max=254) */
localparam P_GAIN                   = 29; /**< P Gain [RW] (default=32 ; min=0 ; max=254) */
localparam GOAL_POSITION            = 30; /**< Goal Position [RW] (min=0 ; max=1023) */
localparam GOAL_SPEED               = 32; /**< Goal Speed [RW] (min=0 ; max=2047) */
localparam GOAL_TORQUE 			     = 35; /**< Goal Torque [RW] (min=0 ; max=1023) */
localparam PRESENT_POSITION         = 37; /**< Current Position [R] */
localparam PRESENT_SPEED            = 39; /**< Current Speed [R] */
localparam PRESENT_LOAD             = 41; /**< Current Load [R] */
localparam PRESENT_VOLTAGE          = 45; /**< Current Voltage [R] */
localparam PRESENT_TEMPERATURE      = 46; /**< Present temperature [R] */
localparam REGISTERED_INSTRUCTION   = 47; /**< Registered Instruction [R] (default=0) */
localparam MOVING                   = 49; /**< Moving [R] (default=0) */
localparam HARDWARE_ERROR           = 50; /**< Hardware error status [R] (default=0) */
localparam PUNCH                    = 51;  /**< Punch [RW] (default=32 ; min=0 ; max=1023) */

localparam READ = 69;
localparam WRITE = 70;

XL320 UUT(
	.clock(clock),
	.reset(reset),
	.serial_io(serial_io),
	.address(address),
	.write(write),
	.writedata(writedata),
	.read(read),
	.readdata(readdata),
	.waitrequest(waitrequest)
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
	read = 0;
	repeat (2) @(posedge clock);
	// set goal position of 1023 for motor 0
	address = {8'd70,8'd0};
	writedata = {16'd30,16'd1023};
	write = 1;
	repeat (1) @(posedge clock); 
	write = 0;
	repeat (1) @(posedge clock); 
	 
	repeat (10000) @(posedge clock);
	$stop;
end
		
		
endmodule