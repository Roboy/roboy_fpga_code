/*From https://www.fpga4fun.com/QuadratureDecoder.html*/

module quad #(parameter DEBOUNCE_TICKS = 5, parameter CLK_FREQ_HZ = 32_000_000)(
    input clk,
    input quadA,
    input quadB,
    output reg [31:0] count
  );

  wire quadA_debounced, quadB_debounced;
  
//  // by-passing debounce
//  assign quadA_debounced = quadA;
//  assign quadB_debounced = quadB;

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

  always @(posedge clk)
  begin
    if(count_enable)
    begin
      if(count_direction) count<=count+1; else count<=count-1;
    end
  end

endmodule
