module FanControl (
    input clk,
    input reset,
    // this is for the avalon interface
    input [2:0] address,
    input write,
    input signed [31:0] writedata,
    input read,
    output signed [31:0] readdata,
    output waitrequest,
    input signed [31:0] current_average,
    output wire pwm
  );

  parameter CLOCK_SPEED_HZ = 50_000_000;

  reg auto_fan;
  integer pwm_freq,duty_ticks,counter,sensitivity;

  reg waitflag;
  reg signed [31:0] readvalue;
  assign readdata = readvalue;
  assign waitrequest = (waitflag&&read);
  
  assign pwm = counter<duty_ticks;

  always @ ( posedge clk , posedge reset) begin
    if(reset)begin
      pwm_freq <= 1000;
      duty_ticks <= 0;
		sensitivity <= 1;
    end else begin
      waitflag <= 1;

      if(read)begin
        waitflag <= 0;
        case(address)
          0: readvalue<=pwm_freq;
          1: readvalue<=duty_ticks;
          2: readvalue<=auto_fan;
			 3: readvalue<=sensitivity;
			 4: readvalue<=current_average;
        endcase
      end

      if(write)begin
        case(address)
          0: pwm_freq<=writedata;
          1: duty_ticks <= writedata;
          2: auto_fan<=(writedata!=0);
		    3: sensitivity <= writedata;
        endcase
      end
        counter <= counter + 1;
        if(counter>(CLOCK_SPEED_HZ/pwm_freq))begin
          counter <= 0;
        end
        if(auto_fan)begin
          duty_ticks <= current_average*sensitivity;
        end
    end
  end


endmodule // FanControl
