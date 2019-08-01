`timescale 1ns/10ps

module FpuTest (
	input clock,
	input reset,
	input [7:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	output signed [31:0] readdata,
	output waitrequest
);

assign readdata = returnvalue;
assign waitrequest = (waitFlag && read);
reg [31:0] returnvalue;
reg waitFlag;

reg [31:0] a;
reg [31:0] b;
reg [31:0] result[5:0];

always @(posedge clock, posedge reset) begin: AVALON_READ_INTERFACE
	if (reset == 1) begin
		waitFlag <= 1;
	end else begin
		waitFlag <= 1;
		if(read) begin
			case(address)
				8'h00: returnvalue <= result[ADD];
				8'h01: returnvalue <= result[SUB];
				8'h02: returnvalue <= result[MUL];
				8'h03: returnvalue <= result[DIV];
				8'h04: returnvalue <= result[INT2FLO];
				8'h05: returnvalue <= result[FLO2INT];
				default: returnvalue <= 32'hDEADBEEF;
			endcase
			if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
				waitFlag <= 0;
			end
		end
	end
end
	
always @(posedge clock, posedge reset) begin: FPU_LOGIC
	if (reset == 1) begin
		
	end else begin
		// if we are writing via avalon bus and waitrequest is deasserted, write the respective register
		if(write && ~waitrequest) begin
			case(address)
				8'h00: a <= writedata[31:0];
				8'h01: b <= writedata[31:0];
			endcase
		end
		
	end 
end

localparam ADD = 0;
localparam SUB = 1;
localparam MUL = 2;
localparam DIV = 3;
localparam INT2FLO = 4;
localparam FLO2INT = 5;

fpu add( clock, 0, ADD, a, b, result[ADD]);
fpu sub( clock, 0, SUB, a, b, result[SUB]);
fpu mul( clock, 0, MUL, a, b, result[MUL]);
fpu div( clock, 0, DIV, a, b, result[DIV]);
fpu int2flo( clock, 0, INT2FLO, a, b, result[INT2FLO]);
fpu flo2int( clock, 0, FLO2INT, a, b, result[FLO2INT]);

endmodule