`timescale 1 ns/1 ns
module XL320_tb;

reg clock, reset;
wire serial_io;

XL320 UUT(
	.clock(clock),
	.reset(reset),
	.serial_io(serial_io)
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
	 
	repeat (100000) @(posedge clock);
	$stop;
end
		
		
endmodule