localparam  MAGIC_NUMBER_LENGTH = 4;

typedef reg [31:0] uint32_t;
typedef reg [16:0] uint16_t;
typedef reg [7:0] uint8_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint16_t crc;
}status_request_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint8_t control_mode;
  uint16_t setpoint0;
  uint16_t setpoint1;
  uint16_t setpoint2;
  uint16_t setpoint3;
  uint16_t position0;
  uint16_t position1;
  uint16_t position2;
  uint16_t position3;
  uint16_t current0;
  uint16_t current1;
  uint16_t current2;
  uint16_t current3;
  uint16_t crc;
}hand_status_response_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint16_t setpoint0;
  uint16_t setpoint1;
  uint16_t setpoint2;
  uint16_t setpoint3;
  uint16_t crc;
}hand_command_t;

typedef struct packed{
  uint32_t header;
  uint8_t id;
  uint8_t control_mode0;
  uint8_t control_mode1;
  uint8_t control_mode2;
  uint8_t control_mode3;
  uint16_t crc;
}hand_control_mode_t;

localparam	STATUS_REQUEST_FRAME_LENGTH = $bits(status_request_t)/8;
localparam	HAND_STATUS_RESPONSE_FRAME_LENGTH = $bits(hand_status_response_t)/8;
localparam	HAND_COMMAND_FRAME_LENGTH = $bits(hand_command_t)/8;
localparam	HAND_CONTROL_MODE_FRAME_LENGTH = $bits(hand_control_mode_t)/8;

localparam  MAX_FRAME_LENGTH = HAND_STATUS_RESPONSE_FRAME_LENGTH;

status_request_t status_request = '{32'hABADBABE,0,0};
hand_status_response_t hand_status_response = '{32'hB000B135,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
hand_command_t hand_command = '{32'hB105F00D,0,0,0,0,0,0};
hand_control_mode_t hand_control_mode = '{32'hB15B00B5,0,0,0,0,0,0};

// arrrggghhh, just because quartus doesn't support unions...
wire [7:0] status_request_data [MAX_FRAME_LENGTH-1:0];
assign status_request_data[0] = status_request.header[31:24];
assign status_request_data[1] = status_request.header[23:16];
assign status_request_data[2] = status_request.header[15:8];
assign status_request_data[3] = status_request.header[7:0];
assign status_request_data[4] = status_request.id;
assign status_request_data[5] = status_request.crc[15:8];
assign status_request_data[6] = status_request.crc[7:0];

wire [7:0]hand_status_response_data [MAX_FRAME_LENGTH-1:0];
assign hand_status_response_data[0] = hand_status_response.header[31:24];
assign hand_status_response_data[1] = hand_status_response.header[23:16];
assign hand_status_response_data[2] = hand_status_response.header[15:8];
assign hand_status_response_data[3] = hand_status_response.header[7:0];
assign hand_status_response_data[4] = hand_status_response.id;
assign hand_status_response_data[5] = hand_status_response.control_mode;
assign hand_status_response_data[6] = hand_status_response.setpoint0[15:8];
assign hand_status_response_data[7] = hand_status_response.setpoint0[7:0];
assign hand_status_response_data[8] = hand_status_response.setpoint1[15:8];
assign hand_status_response_data[9] = hand_status_response.setpoint1[7:0];
assign hand_status_response_data[10] = hand_status_response.setpoint2[15:8];
assign hand_status_response_data[11] = hand_status_response.setpoint2[7:0];
assign hand_status_response_data[12] = hand_status_response.setpoint3[15:8];
assign hand_status_response_data[13] = hand_status_response.setpoint3[7:0];
assign hand_status_response_data[14] = hand_status_response.position0[15:8];
assign hand_status_response_data[15] = hand_status_response.position0[7:0];
assign hand_status_response_data[16] = hand_status_response.position1[15:8];
assign hand_status_response_data[17] = hand_status_response.position1[7:0];
assign hand_status_response_data[18] = hand_status_response.position2[15:8];
assign hand_status_response_data[19] = hand_status_response.position2[7:0];
assign hand_status_response_data[20] = hand_status_response.position3[15:8];
assign hand_status_response_data[21] = hand_status_response.position3[7:0];
assign hand_status_response_data[22] = hand_status_response.current0[15:8];
assign hand_status_response_data[23] = hand_status_response.current0[7:0];
assign hand_status_response_data[24] = hand_status_response.current1[15:8];
assign hand_status_response_data[25] = hand_status_response.current1[7:0];
assign hand_status_response_data[26] = hand_status_response.current2[15:8];
assign hand_status_response_data[27] = hand_status_response.current2[7:0];
assign hand_status_response_data[28] = hand_status_response.current3[15:8];
assign hand_status_response_data[29] = hand_status_response.current3[7:0];
assign hand_status_response_data[30] = hand_status_response.crc[15:8];
assign hand_status_response_data[31] = hand_status_response.crc[7:0];

// arrrggghhh, just because quartus doesn't support unions...
wire [7:0] hand_command_data [MAX_FRAME_LENGTH-1:0];
assign hand_command_data[0] = hand_command.header[31:24];
assign hand_command_data[1] = hand_command.header[23:16];
assign hand_command_data[2] = hand_command.header[15:8];
assign hand_command_data[3] = hand_command.header[7:0];
assign hand_command_data[4] = hand_command.id;
assign hand_command_data[5] = hand_command.setpoint0[15:8];
assign hand_command_data[6] = hand_command.setpoint0[7:0];
assign hand_command_data[7] = hand_command.setpoint1[15:8];
assign hand_command_data[8] = hand_command.setpoint1[7:0];
assign hand_command_data[9] = hand_command.setpoint2[15:8];
assign hand_command_data[10] = hand_command.setpoint2[7:0];
assign hand_command_data[11] = hand_command.setpoint3[15:8];
assign hand_command_data[12] = hand_command.setpoint3[7:0];
assign hand_command_data[13] = hand_command.crc[15:8];
assign hand_command_data[14] = hand_command.crc[7:0];

// arrrggghhh, just because quartus doesn't support unions...
wire [7:0] hand_control_mode_data [MAX_FRAME_LENGTH-1:0];
assign hand_control_mode_data[0] = hand_control_mode.header[31:24];
assign hand_control_mode_data[1] = hand_control_mode.header[23:16];
assign hand_control_mode_data[2] = hand_control_mode.header[15:8];
assign hand_control_mode_data[3] = hand_control_mode.header[7:0];
assign hand_control_mode_data[4] = hand_control_mode.id;
assign hand_control_mode_data[5] = hand_control_mode.crc[15:8];
assign hand_control_mode_data[6] = hand_control_mode.crc[7:0];
