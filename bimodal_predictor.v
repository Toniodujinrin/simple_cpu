module bimodal_predictor(clk, reset, update_en, pc_bits, outcome, prediction); 
	input [15:0] pc_bits; 
	wire [1:0] count; 
	wire tag_not_added; 
	
	assign prediction = tag_not_added? 0 : count[1]; 
	
	branch_history_table   BHT(.clk(clk), .increment_decrement(outcome), .pc_bits(pc_bits), .reset(reset), .count(count), .tag_not_added(tag_not_added), .read_only(~update_en));

endmodule