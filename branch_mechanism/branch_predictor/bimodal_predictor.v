module bimodal_predictor( 
	input clk, reset, write_enabled, outcome, 
	input [15:0] pc_bits_read, pc_bits_write,
	output wire prediction 
); 


	wire [1:0] count; 
	wire tag_not_added; 
	
	assign prediction = tag_not_added? 1'b0 : count[1]; 
	
	branch_history_table   BHT(
		.clk(clk), 
		.increment_decrement(outcome), 
		.pc_bits_read(pc_bits_read),
		.pc_bits_write(pc_bits_write),  
		.reset(reset), 
		.count(count), 
		.tag_not_added(tag_not_added), 
		.write_enabled(write_enabled)
	);

endmodule