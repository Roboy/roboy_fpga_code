module MyoQuad (
		input clk,
		input reset,
		// this is for the avalon interface
		input [3:0] address,
		input write,
		input signed [31:0] writedata,
		input read,
		output signed [31:0] readdata,
		output waitrequest,
		input quad0_Aneg,
		input quad0_Apos,
		input quad0_Bneg,
		input quad0_Bpos,
		input quad1_Aneg,
		input quad1_Apos,
		input quad1_Bneg,
		input quad1_Bpos
);
	
	parameter CLOCK_FREQ_HZ = 50_000_000;
	
	reg quad0_A, quad0_B, quad1_A, quad1_B;
	
	always @(posedge clk) begin: DIFFERENTIAL_RS422
		quad0_A <= quad0_Aneg && !quad0_Apos;
		quad0_B <= quad0_Bneg && !quad0_Bpos;
		quad1_A <= quad1_Aneg && !quad1_Apos;
		quad1_B <= quad1_Bneg && !quad1_Bpos;
	end
	
	reg signed [31:0] pos_0;
	reg signed [31:0] pos_0_offset;
	reg signed [31:0] pos_1;
	reg signed [31:0] pos_1_offset;
	reg signed [31:0] displacement;
	
	// encoder0
	quad #(5) quad_counter0 (
		.clk(clk),
		.quadA(quad0_A),
		.quadB(quad0_B),
		.count(pos_0)
	);

	// encoder1
	quad #(5) quad_counter1 (
		.clk(clk),
		.quadA(quad1_A),
		.quadB(quad1_B),
		.count(pos_1)
	);
	
	always @(posedge clk) begin: DISPLACEMENT_CALCULATION
		displacement <= 75 * (pos_0-pos_0_offset) - 106 * (pos_1-pos_1_offset);
	end

	assign readdata = returnvalue;
	assign waitrequest = (waitFlag && read);
	reg [31:0] returnvalue;
	reg waitFlag;

	always @(posedge clk, posedge reset) begin: AVALON_READ_INTERFACE
		if (reset == 1) begin
			waitFlag <= 1;
		end else begin
			waitFlag <= 1;
			if(read) begin
				case(address)
					4'h0: returnvalue <= pos_0;
					4'h1: returnvalue <= pos_1;
					4'h2: returnvalue <= displacement;
					4'h3: returnvalue <= pos_0_offset;
					4'h4: returnvalue <= pos_1_offset;
					default: returnvalue <= 32'hDEADBEEF;
				endcase
				if(waitFlag==1) begin // next clock cycle the returnvalue should be ready
					waitFlag <= 0;
				end
			end
		end
	end
		
	always @(posedge clk, posedge reset) begin: AVALON_WRITE_INTERFACE
		integer i;
		if (reset == 1) begin
			pos_0_offset <= 0;
			pos_1_offset <= 0;
		end else begin
			if(write && ~waitrequest) begin
				case(address)
					4'h0: begin
						pos_0_offset <= pos_0;
						pos_1_offset <= pos_1;
					end
				endcase
			end
		end 
	end
	
	
endmodule