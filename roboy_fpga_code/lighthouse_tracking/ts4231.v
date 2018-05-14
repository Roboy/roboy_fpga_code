module ts4231 (
    input clk,  // clock
    input rst,  // reset
    inout D,
    inout E,
    output wire [2:0] sensor_STATE,
    output wire [3:0] current_STATE
  );
  
  parameter CLK_SPEED = 50_000_000;
  
  reg D_out;
  reg E_out;
  
  reg D_control;
  reg E_control;
  
  assign D = D_control?D_out:1'bz;
  assign E = E_control?E_out:1'bz;
  assign current_STATE = state[0];
  assign sensor_STATE = sensor_state;
  
  reg [3:0] state[3:0];
  reg [2:0] sensor_state;
  always @(posedge clk, posedge rst) begin: TS4231_CONTROL_LOGIC
    reg [31:0] delay_counter;
    reg [7:0] command_counter;
    reg [7:0] config_index;
    reg [1:0] config_state;
    reg [7:0] votes;
    reg [1:0] S0_count;
    reg [1:0] SLEEP_count;
    reg [1:0] WATCH_count;
    reg [1:0] S3_count;
    reg [15:0] config_value;
    parameter IDLE  = 4'b0000, WAIT_FOR_LIGHT  = 4'b0001, CHECK_BUS = 4'b0010, RESET_COUNTERS = 4'b0011,
      DELAY = 4'b0100, READ_CONFIG = 4'b0101, CONFIG_DEVICE = 4'b0110, GO_TO_WATCH = 4'b0111,
      WRITE_CONFIG = 4'b1000, WRITE_CONFIG_VALUE = 4'b1001, READ_CONFIG_VALUE = 4'b1010;
    parameter SLEEP_STATE = 3'b000, WATCH_STATE = 3'b001, S3_STATE = 3'b010, S0_STATE = 3'b011, UNKNOWN = 3'b100;
    parameter DATA = 2'b00, CLK_HIGH = 2'b01, CLK_LOW = 2'b10;
    if (rst) begin
      D_control <= 0;
      E_control <= 0;
      state[0] <= IDLE;
      command_counter <= 0;
      votes <= 0;
      sensor_state <= UNKNOWN;
    end else begin
      case(state[0])
        IDLE: begin
          state[0] <= RESET_COUNTERS;
			 state[1] <= CHECK_BUS;
			 state[2] <= WAIT_FOR_LIGHT;
        end
        WAIT_FOR_LIGHT: begin
			 if(sensor_state==SLEEP_STATE) begin
    					state[0] <= GO_TO_WATCH;
    		 end  
          if(sensor_state==WATCH_STATE) begin
    					state[0] <= IDLE;
    			end  
      		if(sensor_state==S0_STATE) begin
      				state[0] <= CONFIG_DEVICE;
      		end  
      		if(sensor_state==S3_STATE) begin
      				state[0] <= GO_TO_WATCH;
      		end
          if(sensor_state==UNKNOWN) begin
              state[0] <= IDLE;
          end
        end
        RESET_COUNTERS: begin
          S0_count <= 0;
          SLEEP_count <= 0;
          WATCH_count <= 0;
          S3_count <= 0;
          state[0] <= state[1];
          votes <= 0;
          command_counter <= 0;
        end
        CHECK_BUS: begin
          if(votes<3) begin
            if(D) begin
              if(E) begin
                S3_count <= S3_count + 1;
              end else begin
                SLEEP_count <= SLEEP_count + 1;
              end
            end else begin
              if(E) begin
                WATCH_count <= WATCH_count + 1;
              end else begin
                S0_count <= S0_count + 1;
              end
            end
            delay_counter <= CLK_SPEED/2000; // 500 us
            state[0] <= DELAY;
            state[1] <= CHECK_BUS;
            votes <= votes + 1;
          end else begin
            sensor_state <= SLEEP_STATE;
            if(SLEEP_count >= 2) begin
              sensor_state <= SLEEP_STATE;
            end else if(WATCH_count) begin
              sensor_state <= WATCH_STATE;
            end else if(S3_count) begin
              sensor_state <= S3_STATE;
            end else if(S0_count) begin
              sensor_state <= S0_STATE;
            end else begin
              sensor_state <= UNKNOWN;
            end
            state[0] <= state[2];
          end
        end
        DELAY: begin
          if(delay_counter>0) begin
            delay_counter <= delay_counter - 1;
          end else begin
            state[0] <= state[1];
          end
        end
        CONFIG_DEVICE: begin 
          delay_counter <= CLK_SPEED/1000000; // 1 us
          state[0] <= DELAY;
          state[1] <= CONFIG_DEVICE;
          case(command_counter) 
            0: begin
              E_control <= 1;
              E_out <= 0;
            end
            1: begin
              E_control <= 1;
              E_out <= 1;
            end
            2: begin
              E_control <= 1;
              E_out <= 0;
            end
            3: begin
              E_control <= 1;
              E_out <= 1;
            end
            4: begin
              D_control <= 1;
              D_out <= 0;
            end
            5: begin
              D_control <= 1;
              D_out <= 1;
            end
            6: begin
              D_control <= 0;
              E_control <= 0;
              state[0] <= RESET_COUNTERS; 
              state[1] <= CHECK_BUS;
              state[2] <= WRITE_CONFIG;
            end
            default: state[0] <= IDLE;
          endcase
          command_counter <= command_counter + 1;
        end
        WRITE_CONFIG: begin
          delay_counter <= CLK_SPEED/1000000; // 1 us
          state[0] <= DELAY;
          state[1] <= WRITE_CONFIG;
          case(command_counter) 
            0: begin
              D_control <= 1;
              E_control <= 1;
              D_out <= 1;
              E_out <= 1;
            end
            1: begin
              D_out <= 0;
            end
            2: begin
              E_out <= 0;
            end
            3: begin
              config_value <= 16'h392B;
              config_index <= 15;
              state[0] <= WRITE_CONFIG_VALUE;
              config_state <= 0;
            end
            4: begin
               D_out <= 0;
            end
            5: begin
               E_out <= 1;
            end
            6: begin
               D_out <= 1;
            end
            7: begin
              D_control <= 0;
              E_control <= 0;
              state[0] <= RESET_COUNTERS;
              state[1] <= READ_CONFIG;
            end
            default: state[0] <= IDLE;
          endcase
          command_counter <= command_counter + 1;    
        end  
        WRITE_CONFIG_VALUE: begin
          delay_counter <= CLK_SPEED/1000000; // 1 us
          state[0] <= DELAY;
          state[1] <= WRITE_CONFIG_VALUE;  
          case(config_state)
            0: begin
              if(config_index!=0) begin
                D_out <= config_value[config_index-1];
                config_index <= config_index -1;
                config_state <= config_state + 1;
              end else begin 
                command_counter <= 4;
                state[0] <= WRITE_CONFIG;
              end
            end
            1: begin
              E_out <= 1;
              config_state <= config_state + 1;
            end
            2: begin
              E_out <= 0;
              config_state <= 0;
            end
          endcase
        end
        READ_CONFIG: begin
          delay_counter <= CLK_SPEED/1000000; // 1 us
          state[0] <= DELAY;
          state[1] <= READ_CONFIG;
          case(command_counter) 
            0: begin
              D_control <= 1;
              E_control <= 1;
              D_out <= 1;
              E_out <= 1;
            end
            1: begin
              D_out <= 0;
            end
            2: begin
              E_out <= 0;
            end
            3: begin
              D_out <= 1;
            end
            4: begin
              E_out <= 1;
            end
            5: begin
              D_control <= 0;
            end
            6: begin
              E_out <= 0;
            end
            7: begin
              config_value <= 0;
              config_index <= 14;
              state[0] <= READ_CONFIG_VALUE;
              config_state <= 0;
            end
            8: begin
               D_control <= 1;
               D_out <= 0;
            end
            9: begin
               E_out <= 1;
            end
            10: begin
               D_out <= 1;
            end
            11: begin
               D_control <= 0;
               E_control <= 0;
            end
            12: begin
              if(config_value!=16'h392B) begin
                state[0] <= IDLE;
              end else begin
                D_control <= 0;
                E_control <= 0;
                state[0] <= RESET_COUNTERS;
                state[1] <= GO_TO_WATCH;
              end
            end
            default: state[0] <= IDLE;
          endcase
          if(command_counter<12) begin
            command_counter <= command_counter + 1; 
          end
        end
        READ_CONFIG_VALUE: begin
          delay_counter <= CLK_SPEED/1000000; // 1 us
          case(config_state)
            0: begin
              E_out <= 1;
              config_state <= config_state + 1;
              state[0] <= DELAY;
              state[1] <= READ_CONFIG_VALUE;  
            end
            1: begin
              if(config_index!=0) begin
                config_value[config_index-1] <= D;
                config_index <= config_index - 1;
                config_state <= config_state + 1;
                state[0] <= READ_CONFIG_VALUE; 
              end else begin 
                command_counter <= 8;
                state[0] <= READ_CONFIG;
              end
            end
            2: begin
              E_out <= 0;
              config_state <= 0;
              state[0] <= DELAY;
              state[1] <= READ_CONFIG_VALUE;
            end
          endcase
        end
        GO_TO_WATCH: begin
          case(sensor_state) 
            S0_STATE: begin
              state[0] <= IDLE;
            end
            SLEEP_STATE: begin
              case(command_counter)
                0: begin 
                  D_control <= 1;
                  D_out <= 1;
                end
                1: begin 
                  E_control <= 1;
                  E_out <= 0;
                end
                2: begin 
                  D_out <= 0;
                end
                3: begin 
                  D_control <= 0;
                end  
                4: begin 
                  E_out <= 0;
                end
                5: begin 
                  E_control <= 0;
                end  
                6: begin
                  delay_counter <= CLK_SPEED/10000; // 100 us
                  state[0] <= DELAY;
                  state[1] <= CHECK_BUS;
                  state[2] <= GO_TO_WATCH;  
                  command_counter <= 0;
                  S0_count <= 0;
                  SLEEP_count <= 0;
                  WATCH_count <= 0;
                  S3_count <= 0;
                  votes <= 0;
                end
              endcase
              if(command_counter<6) begin
                command_counter <= command_counter + 1;
              end
            end
            WATCH_STATE: begin
              state[0] <= IDLE;
            end
            S3_STATE: begin
              case(command_counter)
                0: begin 
                  E_control <= 1;
                  E_out <= 1;
                end
                1: begin 
                  D_control <= 1;
                  D_out <= 1;
                end
                2: begin 
                  E_out <= 0;
                end
                3: begin 
                  D_out <= 0;
                end  
                4: begin 
                  E_out <= 0;
                end
                5: begin 
                  D_control <= 0;
                end  
                6: begin 
                  E_out <= 1;
                end
                7: begin 
                  E_control <= 0;
                end  
                8: begin
                  //if(continue) begin
                    delay_counter <= CLK_SPEED/10000; // 100 us
                    state[0] <= DELAY;
                    state[1] <= CHECK_BUS;
                    state[2] <= IDLE;  
                    command_counter <= 0;
                    S0_count <= 0;
                    SLEEP_count <= 0;
                    WATCH_count <= 0;
                    S3_count <= 0;
                    votes <= 0;
                  //  waiting <= 0;
                  //end else begin
                  //  waiting <= 1;
                  //end
                end
              endcase
              if(command_counter<8) begin
                command_counter <= command_counter + 1;
              end
            end
            default: state[1] <= IDLE;
          endcase
        end
        default: state[0] <= IDLE;
      endcase
    end
  end
  
endmodule