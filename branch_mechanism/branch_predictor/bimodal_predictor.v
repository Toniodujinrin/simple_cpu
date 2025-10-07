module bimodal_predictor #(parameter INDEX_LEN =7, parameter TAG_LEN = 7 )( 
	input clk, reset, write_enabled, outcome, 
	input [15:0] pc_bits_read, pc_bits_write,
	output wire prediction 
); 


	wire [1:0] count; 
	wire tag_not_added; 
	wire [INDEX_LEN-1:0] index_read  = pc_bits_read[8:2];
	wire [INDEX_LEN-1:0] index_write = pc_bits_write[8:2]; 
	wire [TAG_LEN-1:0] tag_read = pc_bits_read[15:9];
	wire [TAG_LEN-1:0] tag_write = pc_bits_write[15:9];
	
	assign prediction = tag_not_added? 1'b0 : count[1]; 
	
	branch_history_table   BHT(
		.clk(clk), 
		.increment_decrement(outcome), 
		.tag_bits_read(tag_read), 
		.tag_bits_write(tag_write), 
		.index_read(index_read), 
		.index_write(index_write), 
		.reset(reset), 
		.count(count), 
		.tag_not_added(tag_not_added), 
		.write_enabled(write_enabled)
	);

endmodule
