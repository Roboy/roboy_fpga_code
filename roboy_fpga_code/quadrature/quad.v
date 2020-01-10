/*From https://www.fpga4fun.com/QuadratureDecoder.html*/

module quad #(parameter DEBOUNCE_TICKS = 5, parameter CLK_FREQ_HZ = 32_000_000)(
    input clk,
    input quadA,
    input quadB,
    output reg [23:0] count
    // output reg [23:0] count_per_millisecond,
    // output A_filtered,
    // output B_filtered
  );

  wire quadA_debounced, quadB_debounced;

  grp_debouncer #(2,DEBOUNCE_TICKS) debounce(
    .clk_i(clk),
    .data_i({quadA,quadB}),
    .data_o({quadA_debounced,quadB_debounced})
  );

  reg A_delayed, B_delayed;
  always @(posedge clk) A_delayed <= quadA_debounced;
  always @(posedge clk) B_delayed <= quadB_debounced;

  wire count_enable = quadA_debounced ^ A_delayed ^ quadB_debounced ^ B_delayed;
  wire count_direction = quadA_debounced ^ B_delayed;

  // reg A_delayed,B_delayed;
  // always @(posedge clk) A_delayed <= quadA;
  // always @(posedge clk) B_delayed <= quadB;
  //
  // wire count_enable = quadA ^ A_delayed ^ quadB ^ B_delayed;
  // wire count_direction = quadA ^ B_delayed;

  always @(posedge clk)
  begin
    if(count_enable)
    begin
      if(count_direction) count<=count+1; else count<=count-1;
    end
  end

  // reg [31:0] millisecond_counter;
  // reg [31:0] count_prev;
  // always @(posedge clk) begin : VELOCITY_CALCULATION
  //   millisecond_counter <= millisecond_counter + 1;
  //   if((millisecond_counter%(CLK_FREQ_HZ/1000)==0)) begin
  //     count_per_millisecond <= (count_prev-count);
  //     count_prev <= count;
  //   end
  // end

endmodule
