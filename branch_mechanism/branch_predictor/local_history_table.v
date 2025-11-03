///////////////////////////////////////////////////////////////////////////////
// File:        local_history_table.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: Local History Table
///////////////////////////////////////////////////////////////////////////////


module local_history_table #(parameter INDEX_LEN = 7, HISTORY_LEN = 10)
(
	input clk, prediction, reset, rollback_enabled, predict_enable, 
	input [INDEX_LEN-1:0] pc_bits_read, 
	input [INDEX_LEN-1:0] pc_bits_write,
	input [HISTORY_LEN-1:0] history_write 
	output [HISTORY_LEN-1:0] history_read, 
); 
	 


	localparam LOCATIONS = 2**INDEX_LEN; 
	localparam VALUE_WIRES_LENGTH = HISTORY_LEN*LOCATIONS; 
	
	wire [LOCATIONS-1:0] out_bits; //not used 
	wire [LOCATIONS-1:0] history_select_write;
	wire [LOCATIONS-1:0] history_select_read; 
	wire [VALUES_WIRES_LENGTH-1:0] values; 
	

	//decodes pc bits into 1 hot addresses, this 1 hot address will be used to select the history register for writing 
	register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER_WRITE(
		.in(pc_bits_write),
		.out(history_select_write)
	);  

	register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER_READ(
		.in(pc_bits_read),
		.out(history_select_read)
	);
	
	//multiplex values from all the history registers to get the read history and the write history to be passed into the PHT. 
	reg_mux #(.REG_N(LOCATIONS), .WIDTH(HISTORY_LEN))  OUTPUT_MUX_READ(
		.in(values), 
		.select(pc_bits_read), 
		.out(history_read)
	);




	genvar i; 
	generate 
		for(i=0; i<LOCATIONS; i = i+1)
			begin:LOCATION
				shift_reg_n#(.WIDTH(HISTORY_LEN))  HISTORY_REG(
					.clk(clk),
					.in(prediction), 
					.out(out_bits[i]), 
					.value(values[i*HISTORY_LEN +: HISTORY_LEN]), 
					.reset(reset), 
					.p_load_enabled(history_select_write[i] && rollback_enabled), 
					.load(history_write), 
					.shift_enabled(history_select_read[i] && ~rollback_enabled && predict_enable)
				);
			end
	endgenerate 


endmodule 
 


module shift_reg_n #(parameter WIDTH = 10)
(
	input in, reset, clk, shift_enabled, p_load_enabled, 
	input [WIDTH-1:0] load, 
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
				.d(shift_enabled?	(i==0 ? in: ff_out[i-1]):
					p_load_enabled? load[i]:
									f_out[i]),
				.q(ff_out[i]),
				.reset(reset)
			);
		end
	endgenerate 

endmodule 




	