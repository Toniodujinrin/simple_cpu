module local_predictor(clk, reset, pc_bits, update_en, outcome, prediction); 
	input clk, reset, update_en, outcome; 
	input [6:0] pc_bits; 
	output prediction; 

	wire [9:0] history;
	wire [1:0] count; 
	
	assign prediction = count[1]; 
	
	pattern_history_table  PHT(.clk(clk), .index(history), .count(count), .increment_decrement(outcome), .reset(reset), .read_only(~update_en));
	logical_history_table  LHT(.clk(clk), .taken_not_taken(outcome), .pc_bits(pc_bits), .reset(reset), .history(history), .read_only(~update_en));

endmodule 