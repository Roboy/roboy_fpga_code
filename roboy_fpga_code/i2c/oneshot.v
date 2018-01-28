// oneshot
// helper module which generates a pulse on changing signals
// author: Simon Trendel, 2017

module oneshot (
    input clk,
    input edge_sig,
    output level_sig
);

reg cur_value;
reg last_value;

assign level_sig = cur_value & ~last_value;

always @(posedge clk) begin
    cur_value <= edge_sig;
    last_value <= cur_value;
end

endmodule
