module XL320 (
   input       clock,
   input       reset,
	inout 		serial_io,
	// this is for the avalon interface
	input [15:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest
   );

parameter NUMBER_OF_MOTORS = 8;
reg send;
reg getValue;
	
reg [7:0] motor_id;
reg [7:0] instruction;
reg [15:0] motor_address;
reg [15:0] value;

reg [15:0] model_number[NUMBER_OF_MOTORS-1:0];
reg [15:0] version[NUMBER_OF_MOTORS-1:0];
reg [15:0] id[NUMBER_OF_MOTORS-1:0];
reg [1:0] baud_rate[NUMBER_OF_MOTORS-1:0];
reg [7:0] return_delay_time[NUMBER_OF_MOTORS-1:0];
reg [15:0] cw_angle_limit[NUMBER_OF_MOTORS-1:0];
reg [15:0] ccw_angle_limit[NUMBER_OF_MOTORS-1:0];
reg [1:0] control_mode[NUMBER_OF_MOTORS-1:0];
reg [7:0] limit_temperature[NUMBER_OF_MOTORS-1:0];
reg [7:0] lower_limit_voltage[NUMBER_OF_MOTORS-1:0];
reg [7:0] upper_limit_voltage[NUMBER_OF_MOTORS-1:0];
reg [15:0] max_torque[NUMBER_OF_MOTORS-1:0];
reg [1:0] return_level[NUMBER_OF_MOTORS-1:0];
reg [3:0] alarm_shutdown[NUMBER_OF_MOTORS-1:0]; 
reg [1:0] torque_enable[NUMBER_OF_MOTORS-1:0];
reg [15:0] led[NUMBER_OF_MOTORS-1:0];
reg [15:0] d_gain[NUMBER_OF_MOTORS-1:0];
reg [15:0] i_gain[NUMBER_OF_MOTORS-1:0];
reg [15:0] p_gain[NUMBER_OF_MOTORS-1:0];
reg [15:0] goal_position[NUMBER_OF_MOTORS-1:0];
reg [15:0] goal_speed[NUMBER_OF_MOTORS-1:0];
reg [15:0] goal_torque[NUMBER_OF_MOTORS-1:0];
reg [15:0] present_position[NUMBER_OF_MOTORS-1:0];
reg [15:0] present_speed[NUMBER_OF_MOTORS-1:0];
reg [15:0] present_load[NUMBER_OF_MOTORS-1:0];
reg [15:0] present_voltage[NUMBER_OF_MOTORS-1:0];
reg [15:0] present_temperature[NUMBER_OF_MOTORS-1:0];
reg [15:0] registered_instruction[NUMBER_OF_MOTORS-1:0];
reg [15:0] moving[NUMBER_OF_MOTORS-1:0];
reg [15:0] hardware_error[NUMBER_OF_MOTORS-1:0];
reg [15:0] punch[NUMBER_OF_MOTORS-1:0];

reg [31:0] returnvalue; 
reg waitFlag;	
assign readdata = returnvalue;

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

integer i, j;
// the following iterface handles read requests via lightweight axi bridge
// the upper 8 bit of the read address define which value we want to read
// the lower 8 bit of the read address define for which motor
always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	if (reset == 1) begin
		waitFlag <= 1;
	end else begin
		waitFlag <= 1;
		if(read) begin
			case(address>>8)
				MODEL_NUMBER: 				returnvalue <= model_number[address[7:0]];
				VERSION: 					returnvalue <= version[address[7:0]];
				ID: 							returnvalue <= id[address[7:0]];
				BAUD_RATE: 					returnvalue <= baud_rate[address[7:0]];
				RETURN_DELAY_TIME: 		returnvalue <= return_delay_time[address[7:0]];
				CW_ANGLE_LIMIT: 			returnvalue <= cw_angle_limit[address[7:0]];
				CCW_ANGLE_LIMIT: 			returnvalue <= ccw_angle_limit[address[7:0]];
				CONTROL_MODE: 				returnvalue <= control_mode[address[7:0]];
				LIMIT_TEMPERATURE: 		returnvalue <= limit_temperature[address[7:0]];
				LOWER_LIMIT_VOLTAGE: 	returnvalue <= lower_limit_voltage[address[7:0]];
				UPPPER_LIMIT_VOLTAGE: 	returnvalue <= upper_limit_voltage[address[7:0]];
				MAX_TORQUE: 				returnvalue <= max_torque[address[7:0]];
				RETURN_LEVEL: 				returnvalue <= return_level[address[7:0]];
				ALARM_SHUTDOWN: 			returnvalue <= alarm_shutdown[address[7:0]];
				TORQUE_ENABLE: 			returnvalue <= torque_enable[address[7:0]];
				LED: 							returnvalue <= led[address[7:0]];
				D_GAIN: 						returnvalue <= d_gain[address[7:0]];
				I_GAIN: 						returnvalue <= i_gain[address[7:0]];
				P_GAIN: 						returnvalue <= p_gain[address[7:0]];
				GOAL_POSITION: 			returnvalue <= goal_position[address[7:0]];
				GOAL_SPEED: 				returnvalue <= goal_speed[address[7:0]];
				GOAL_TORQUE: 				returnvalue <= goal_torque[address[7:0]];
				PRESENT_POSITION: 		returnvalue <= present_position[address[7:0]];
				PRESENT_LOAD: 				returnvalue <= present_load[address[7:0]];
				PRESENT_SPEED: 			returnvalue <= present_speed[address[7:0]];
				PRESENT_VOLTAGE: 			returnvalue <= present_voltage[address[7:0]];
				PRESENT_TEMPERATURE: 	returnvalue <= present_temperature[address[7:0]];
				REGISTERED_INSTRUCTION: returnvalue <= registered_instruction[address[7:0]];
				MOVING: 						returnvalue <= moving[address[7:0]];
				HARDWARE_ERROR: 			returnvalue <= hardware_error[address[7:0]];
				PUNCH: 						returnvalue <= punch[address[7:0]];
				default: returnvalue <= 32'hDEADBEEF;
			endcase
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end

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

always @(posedge clock, posedge reset) begin: AVALON_WRITE_INTERFACE
	if (reset == 1) begin
		for(i=0; i<NUMBER_OF_MOTORS; i = i+1) begin : reset_default_values
			model_number[i] <= 350;
			version[i] <= 0;
			id[i] <= i;
			baud_rate[i] <= 3;
			return_delay_time[i] <= 250;
			cw_angle_limit[i] <= 0;
			ccw_angle_limit[i] <= 1023;
			control_mode[i] <= 2;
			limit_temperature[i] <= 65;
			lower_limit_voltage[i] <= 60;
			upper_limit_voltage[i] <= 90;
			max_torque[i] <= 1023;
			return_level[i] <= 2;
			alarm_shutdown[i] <= 3;
			torque_enable[i] <= 0;
			led[i] <= 0;
			d_gain[i] <= 0;
			i_gain[i] <= 0;
			p_gain[i] <= 32;
			goal_position[i] <= 0;
			goal_speed[i] <= 0;
			goal_torque[i] <= 0;
			present_position[i] <= 0;
			present_load[i] <= 0;
			present_speed[i] <= 0;
			present_voltage[i] <= 0;
			present_temperature[i] <= 0;
			registered_instruction[i] <= 0;
			moving[i] <= 0;
			hardware_error[i] <= 0;
			punch[i] <= 0;
		end
	end else begin
		send<=0;
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			if(address[7:0]<NUMBER_OF_MOTORS) begin
				case((writedata>>16))
					MODEL_NUMBER: 				model_number[address[7:0]] <= (writedata&16'hFF);
					VERSION: 					version[address[7:0]] <= (writedata&16'hFF);
					ID: 							id[address[7:0]] <= (writedata&16'hFF);
					BAUD_RATE: 					baud_rate[address[7:0]] <= (writedata&16'hFF);
					RETURN_DELAY_TIME: 		return_delay_time[address[7:0]] <= (writedata&16'hFF);
					CW_ANGLE_LIMIT: 			cw_angle_limit[address[7:0]] <= (writedata&16'hFF);
					CCW_ANGLE_LIMIT: 			ccw_angle_limit[address[7:0]] <= (writedata&16'hFF);
					CONTROL_MODE: 				control_mode[address[7:0]] <= (writedata&16'hFF);
					LIMIT_TEMPERATURE: 		limit_temperature[address[7:0]] <= (writedata&16'hFF);
					LOWER_LIMIT_VOLTAGE: 	lower_limit_voltage[address[7:0]] <= (writedata&16'hFF);
					UPPPER_LIMIT_VOLTAGE: 	upper_limit_voltage[address[7:0]] <= (writedata&16'hFF);
					MAX_TORQUE: 				max_torque[address[7:0]] <= (writedata&16'hFF);
					RETURN_LEVEL: 				return_level[address[7:0]] <= (writedata&16'hFF);
					ALARM_SHUTDOWN: 			alarm_shutdown[address[7:0]] <= (writedata&16'hFF);
					TORQUE_ENABLE: 			torque_enable[address[7:0]] <= (writedata&16'hFF);
					LED: 							led[address[7:0]] <= (writedata&16'hFF);
					D_GAIN: 						d_gain[address[7:0]] <= (writedata&16'hFF);
					I_GAIN: 						i_gain[address[7:0]] <= (writedata&16'hFF);
					P_GAIN: 						p_gain[address[7:0]] <= (writedata&16'hFF);
					GOAL_POSITION: 			goal_position[address[7:0]] <= (writedata&16'hFF);
					GOAL_SPEED: 				goal_speed[address[7:0]] <= (writedata&16'hFF);
					GOAL_TORQUE: 				goal_torque[address[7:0]] <= (writedata&16'hFF);
					PRESENT_POSITION: 		present_position[address[7:0]] <= (writedata&16'hFF);
					PRESENT_LOAD: 				present_load[address[7:0]] <= (writedata&16'hFF);
					PRESENT_SPEED: 			present_speed[address[7:0]] <= (writedata&16'hFF);
					PRESENT_VOLTAGE: 			present_voltage[address[7:0]] <= (writedata&16'hFF);
					PRESENT_TEMPERATURE: 	present_temperature[address[7:0]] <= (writedata&16'hFF);
					REGISTERED_INSTRUCTION: registered_instruction[address[7:0]] <= (writedata&16'hFF);
					MOVING: 						moving[address[7:0]] <= (writedata&16'hFF);
					HARDWARE_ERROR: 			hardware_error[address[7:0]] <= (writedata&16'hFF);
					PUNCH: 						punch[address[7:0]] <= (writedata&16'hFF);
				endcase
				case(address>>8)
					READ: begin
						motor_id <= id[address[7:0]];
						motor_address <= (writedata>>16);
						value <= (writedata&16'hFF);
						instruction <= DXL_INST_READ;
						getValue <= 1;
						send <= 1;
					end
					WRITE: begin
						motor_id <= id[address[7:0]];
						motor_address <= (writedata>>16);
						value <= (writedata&16'hFF);
						instruction <= DXL_INST_WRITE;
						getValue <= 0;
						send <= 1;
					end
				endcase
			end
		end
	end 
end

reg crc_calculate, crc_calculated, crc_calculate_done;

localparam bit [15:0] crc_table [0:255] = '{16'h0000,
										  16'h8005, 16'h800F, 16'h000A, 16'h801B, 16'h001E, 16'h0014, 16'h8011,
										  16'h8033, 16'h0036, 16'h003C, 16'h8039, 16'h0028, 16'h802D, 16'h8027,
										  16'h0022, 16'h8063, 16'h0066, 16'h006C, 16'h8069, 16'h0078, 16'h807D,
										  16'h8077, 16'h0072, 16'h0050, 16'h8055, 16'h805F, 16'h005A, 16'h804B,
										  16'h004E, 16'h0044, 16'h8041, 16'h80C3, 16'h00C6, 16'h00CC, 16'h80C9,
										  16'h00D8, 16'h80DD, 16'h80D7, 16'h00D2, 16'h00F0, 16'h80F5, 16'h80FF,
										  16'h00FA, 16'h80EB, 16'h00EE, 16'h00E4, 16'h80E1, 16'h00A0, 16'h80A5,
										  16'h80AF, 16'h00AA, 16'h80BB, 16'h00BE, 16'h00B4, 16'h80B1, 16'h8093,
										  16'h0096, 16'h009C, 16'h8099, 16'h0088, 16'h808D, 16'h8087, 16'h0082,
										  16'h8183, 16'h0186, 16'h018C, 16'h8189, 16'h0198, 16'h819D, 16'h8197,
										  16'h0192, 16'h01B0, 16'h81B5, 16'h81BF, 16'h01BA, 16'h81AB, 16'h01AE,
										  16'h01A4, 16'h81A1, 16'h01E0, 16'h81E5, 16'h81EF, 16'h01EA, 16'h81FB,
										  16'h01FE, 16'h01F4, 16'h81F1, 16'h81D3, 16'h01D6, 16'h01DC, 16'h81D9,
										  16'h01C8, 16'h81CD, 16'h81C7, 16'h01C2, 16'h0140, 16'h8145, 16'h814F,
										  16'h014A, 16'h815B, 16'h015E, 16'h0154, 16'h8151, 16'h8173, 16'h0176,
										  16'h017C, 16'h8179, 16'h0168, 16'h816D, 16'h8167, 16'h0162, 16'h8123,
										  16'h0126, 16'h012C, 16'h8129, 16'h0138, 16'h813D, 16'h8137, 16'h0132,
										  16'h0110, 16'h8115, 16'h811F, 16'h011A, 16'h810B, 16'h010E, 16'h0104,
										  16'h8101, 16'h8303, 16'h0306, 16'h030C, 16'h8309, 16'h0318, 16'h831D,
										  16'h8317, 16'h0312, 16'h0330, 16'h8335, 16'h833F, 16'h033A, 16'h832B,
										  16'h032E, 16'h0324, 16'h8321, 16'h0360, 16'h8365, 16'h836F, 16'h036A,
										  16'h837B, 16'h037E, 16'h0374, 16'h8371, 16'h8353, 16'h0356, 16'h035C,
										  16'h8359, 16'h0348, 16'h834D, 16'h8347, 16'h0342, 16'h03C0, 16'h83C5,
										  16'h83CF, 16'h03CA, 16'h83DB, 16'h03DE, 16'h03D4, 16'h83D1, 16'h83F3,
										  16'h03F6, 16'h03FC, 16'h83F9, 16'h03E8, 16'h83ED, 16'h83E7, 16'h03E2,
										  16'h83A3, 16'h03A6, 16'h03AC, 16'h83A9, 16'h03B8, 16'h83BD, 16'h83B7,
										  16'h03B2, 16'h0390, 16'h8395, 16'h839F, 16'h039A, 16'h838B, 16'h038E,
										  16'h0384, 16'h8381, 16'h0280, 16'h8285, 16'h828F, 16'h028A, 16'h829B,
										  16'h029E, 16'h0294, 16'h8291, 16'h82B3, 16'h02B6, 16'h02BC, 16'h82B9,
										  16'h02A8, 16'h82AD, 16'h82A7, 16'h02A2, 16'h82E3, 16'h02E6, 16'h02EC,
										  16'h82E9, 16'h02F8, 16'h82FD, 16'h82F7, 16'h02F2, 16'h02D0, 16'h82D5,
										  16'h82DF, 16'h02DA, 16'h82CB, 16'h02CE, 16'h02C4, 16'h82C1, 16'h8243,
										  16'h0246, 16'h024C, 16'h8249, 16'h0258, 16'h825D, 16'h8257, 16'h0252,
										  16'h0270, 16'h8275, 16'h827F, 16'h027A, 16'h826B, 16'h026E, 16'h0264,
										  16'h8261, 16'h0220, 16'h8225, 16'h822F, 16'h022A, 16'h823B, 16'h023E,
										  16'h0234, 16'h8231, 16'h8213, 16'h0216, 16'h021C, 16'h8219, 16'h0208,
										  16'h820D, 16'h8207, 16'h0202 };
reg [7:0] packet[13:0];
reg [15:0] length;
reg [15:0] crc;

always @(posedge clock)
begin
	if(crc_calculate||crc_calculate_done==0) begin
		crc_calculate_done=0;
		crc = 0;
		for(j = 0; j < 10; j=j+1) begin
			i = ((crc >> 8) ^ packet[j]) & 8'hFF;
			crc = (crc << 8) ^ crc_table[i];
		end
		crc_calculate_done = 1;
	end
end

integer timeout_counter;
reg transmit, receive, direction;
reg [7:0] tx_byte; 
wire [7:0] rx_byte;
wire tx_active;
wire tx_done, rx_done;
reg package_valid;
reg [7:0] byte_counter;
reg [7:0] xl320_state;
localparam IDLE = 0, SENDPACKET = 1, RECEIVE = 2, TRANSMIT = 3, CALCULATE_CRC_AND_TRANSMIT = 4, WAIT_FOR_TRANSMIT_DONE = 5, CHECK_PACKAGE_VALID = 6;
reg [7:0] receive_state;
localparam HEADER_FF_1 = 0, HEADER_FF_2 = 1, HEADER_FD = 2, HEADER_RESERVED = 3, HEADER_ID = 4, LENGTH_1 = 5, LENGTH_2 = 6, INSTR = 7, PARAMS = 8, CRC_1 = 9, CRC_2 = 10;
always @(posedge clock, posedge reset)
begin
	if(reset) begin
		xl320_state <= IDLE;
		transmit<=0; 
	end else begin 
		case (xl320_state) 
			IDLE: begin
				if(send)begin
					xl320_state <= SENDPACKET; 
				end
			end
			SENDPACKET: begin
				if(!tx_active) begin
					if (motor_address==MODEL_NUMBER||motor_address==CW_ANGLE_LIMIT||motor_address==CCW_ANGLE_LIMIT||motor_address==MAX_TORQUE||
					motor_address==GOAL_POSITION||motor_address==GOAL_SPEED||motor_address==GOAL_TORQUE||motor_address==PRESENT_POSITION||motor_address==PRESENT_SPEED||
					motor_address==PRESENT_LOAD||motor_address==PUNCH ) begin
						length = 2; 
					end else begin
						length = 1;
					end
					packet[0] = 8'hFF;
					packet[1] = 8'hFF;
					packet[2] = 8'hFD;
					packet[3] = 8'h00;
					packet[4] = motor_id; 
					packet[5] = (length & 8'hff);
					packet[6] = ((length >> 8) & 8'hff);
					packet[7] = instruction;
					packet[8] = (motor_address & 8'hff);
					packet[9] = ((motor_address >> 8) & 8'hff);
					packet[10] = (value & 8'hff);
					if(length==2) begin
						packet[11] = ((value >> 8) & 8'hff);
					end
					crc_calculate=1;
					xl320_state= CALCULATE_CRC_AND_TRANSMIT;
				end
			end
			CALCULATE_CRC_AND_TRANSMIT: begin
				crc_calculate=0;
				if(crc_calculate_done) begin
					if(length==1) begin
						packet[11] = crc[7:0];
						packet[12] = crc[15:8];
					end else begin
						packet[12] = crc[7:0];
						packet[13] = crc[15:8];
					end
					byte_counter=0;
					direction=1;
					xl320_state=TRANSMIT;
				end
			end
			TRANSMIT: begin
				if(byte_counter<=(11+length)) begin
					case(byte_counter)
						0: tx_byte=packet[0];
						1: tx_byte=packet[1];
						2: tx_byte=packet[2];
						3: tx_byte=packet[3];
						4: tx_byte=packet[4];
						5: tx_byte=packet[5];
						6: tx_byte=packet[6];
						7: tx_byte=packet[7];
						8: tx_byte=packet[8];
						9: tx_byte=packet[9];
						10: tx_byte=packet[10];
						11: tx_byte=packet[11];
						12: tx_byte=packet[12];
						13: tx_byte=packet[13];
					endcase
					xl320_state=WAIT_FOR_TRANSMIT_DONE;
					transmit=1;
				end else begin 
					if(getValue==0) begin
						xl320_state=IDLE;
					end else begin
						xl320_state=RECEIVE;
						byte_counter=0;
						direction=0;
						package_valid=1;
						receive_state<=HEADER_FF_1;
						timeout_counter<=50000; //1ms timeout
					end
				end
			end
			RECEIVE: begin
				if(timeout_counter>0) begin
					timeout_counter <= timeout_counter -1;
					if(rx_done) begin
						case(receive_state)
							HEADER_FF_1: begin 
								if(rx_byte==8'hFF) begin
									packet[0]<=rx_byte;
									receive_state<=HEADER_FF_2;
									byte_counter<=byte_counter+1;
								end else begin
									package_valid<=0;
								end
							end
							HEADER_FF_2: begin 
								if(rx_byte==8'hFF) begin
									packet[1]<=rx_byte;
									receive_state<=HEADER_FD;
									byte_counter<=byte_counter+1;
								end else begin
									package_valid<=0;
								end
							end
							HEADER_FD: begin 
								if(rx_byte==8'hFD) begin
									packet[2]<=rx_byte;
									receive_state<=HEADER_RESERVED;
									byte_counter<=byte_counter+1;
								end else begin
									package_valid<=0;
								end
							end
							HEADER_RESERVED: begin 
								packet[3]<=0;
								receive_state<=ID;
								byte_counter<=byte_counter+1;
							end
							ID: begin 
								if(rx_byte==motor_id) begin
									packet[4]<=motor_id;
									receive_state<=LENGTH_1;
									byte_counter<=byte_counter+1;
								end else begin
									package_valid<=0;
								end
							end
							LENGTH_1: begin 
								packet[5]<=rx_byte;
								receive_state<=LENGTH_2;
								byte_counter<=byte_counter+1;
							end
							LENGTH_2: begin 
								packet[6]<=rx_byte;
								receive_state<=INSTR;
								byte_counter<=byte_counter+1;
							end
							INSTR: begin 
								packet[7]<=rx_byte;
								if(rx_byte==DXL_INST_ACTION||rx_byte==DXL_INST_REBOOT) begin
									receive_state<=CRC_1;
								end else begin
									receive_state<=PARAMS;
								end
								byte_counter<=byte_counter+1;
							end
							PARAMS: begin 
								packet[byte_counter] <= rx_byte;
								byte_counter<=byte_counter+1;
								receive_state<=CRC_1;
								// TODO check length and go to crc accordingly
							end
							CRC_1: begin 
								packet[byte_counter]<=rx_byte;
								byte_counter<=byte_counter+1; 
								receive_state<=CRC_2;
							end
							CRC_2: begin 
								packet[byte_counter]<=rx_byte;
								byte_counter<=byte_counter+1;
								xl320_state<=CHECK_PACKAGE_VALID;
								length <= {packet[6],packet[5]};
								crc_calculate<=1;
							end
						endcase
					end
					end else begin 
						package_valid<=0;
						xl320_state <= IDLE;
					end
			end
			WAIT_FOR_TRANSMIT_DONE: begin
				if(tx_done) begin
					transmit<=0; 
					byte_counter <= byte_counter + 1;
					xl320_state<=TRANSMIT; 
				end
			end
			CHECK_PACKAGE_VALID: begin
				crc_calculate<=0;
				if(crc_calculate_done) begin
					if(length==1) begin
						if((packet[11] != crc[7:0]) || (packet[12] != crc[15:8])) begin 
							package_valid <= 0;
						end
					end else begin
						if((packet[12] != crc[7:0]) || (packet[13] != crc[15:8])) begin 
							package_valid <= 0;
						end
					end
					xl320_state<=IDLE;
				end
			end
		endcase
	end
end

wire serial_i, serial_o;

assign serial_io=(direction?serial_o:serial_i);
assign waitrequest = (waitFlag && read)||(xl320_state!=IDLE);

// 1Mbit uart transmitter
uart_tx #(.CLKS_PER_BIT(50)) uart_tx(
	.i_Clock(clock),
   .i_Tx_DV(transmit),
   .i_Tx_Byte(tx_byte), 
   .o_Tx_Active(tx_active),
   .o_Tx_Serial(serial_o),
   .o_Tx_Done(tx_done)
);

// 1Mbit uart receiver
uart_rx #(.CLKS_PER_BIT(50)) uart_rx(
	.i_Clock(clock),
   .o_Rx_DV(rx_done),
   .o_Rx_Byte(rx_byte),
	.i_Rx_Serial(serial_i)
);
	
endmodule