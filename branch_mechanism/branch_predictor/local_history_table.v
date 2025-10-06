module logical_history_table #(parameter INDEX_LEN = 7, 
parameter HISTORY_LEN = 10)
(
	input clk, taken_not_taken, reset, write_enabled, 
	input [INDEX_LEN-1:0] pc_bits_read, 
	input [INDEX_LEN-1:0] pc_bits_write, 
	output [HISTORY_LEN-1:0] history_read, 
	output [HISTORY_LEN-1:0] history_write
); 
	 


	localparam LOCATIONS = 2**INDEX_LEN, 
	localparam VALUE_WIRES_LENGTH = HISTORY_LEN*LOCATIONS; 
	
	wire [LOCATIONS-1:0] out_bits; //not used 
	wire [LOCATIONS-1:0] history_select_write; 
	wire [VALUES_WIRES_LENGTH-1:0] values; 
	

	//decodes pc bits into 1 hot addresses, this 1 hot address will be used to select the history register for writing 
	register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER(
		.in(pc_bits_write),
		.out(history_select_write)
	);  
	
	//multiplex values from all the history registers to get the read history and the write history to be passed into the PHT. 
	reg_mux #(.REG_N(LOCATIONS), .WIDTH(HISTORY_LEN))  OUTPUT_MUX_READ(
		.in(values), 
		.select(pc_bits_read), 
		.out(history_read)
	);


	reg_mux #(.REG_N(LOCATIONS), .WIDTH(HISTORY_LEN))  OUTPUT_MUX_WRITE(
		.in(values), 
		.select(pc_bits_write), 
		.out(history_write)
	);


	genvar i; 
	generate 
		for(i=0; i<LOCATIONS; i = i+1)
			begin:LOCATION
				shift_reg_n#(.WIDTH(HISTORY_LEN))  HISTORY_REG(
					.clk(clk),
					.in(taken_not_taken), 
					.out(out_bits[i]), 
					.value(values[i*HISTORY_LEN +: HISTORY_LEN]), 
					.reset(reset), 
					.enabled(history_select_write[i] & write_enabled)
				);
			end
	endgenerate 


endmodule 
 


module shift_reg_n #(parameter WIDTH = 10)
(
	input in, reset, clk, enabled, 
	output out, 
	output wire [WIDTH-1:0]  value
); 


	wire [WIDTH-1:0] ff_out; 
	
	assign value = ff_out; 
	assign out = ff_out[WIDTH-1]; 
	genvar i; 
	generate
	for(i=0; i < WIDTH; i = i+1) 
		begin
			
			d_ff  D_FF(
				.clk(clk),
				.d(enabled?(i==0 ? in: ff_out[i-1]):ff_out[i]),
				.q(ff_out[i]),
				.reset(reset)
			);
		end
	endgenerate 

endmodule 




	