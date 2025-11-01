module local_predictor #(parameter HISTORY_LEN = 10) (
	input clk, reset, write_enabled, outcome, rollback_enabled
	input [15:0] pc_bits_read, pc_bits_write, 
	input [HISTORY_LEN-1:0] history_write, 
	output prediction
); 

	wire [9:0] history_read;
	 
	wire [1:0] count; 
	
	assign prediction = count[1]; 
	 
	pattern_history_table  PHT(
		.clk(clk),
		.index_read(history_read), 
		.index_write(history_write), 
		.count(count), 
		.increment_decrement(outcome), 
		.reset(reset), 
		.write_enabled(write_enabled)
	);


	local_history_table  LHT(
		.clk(clk), 
		.prediction(prediction), 
		.pc_bits_read(pc_bits_read[8:2]), 
		.pc_bits_write(pc_bits_write[8:2]), 
		.reset(reset), 
		.history_read(history_read), 
		.history_write(history_write), 

		.write_enabled(write_enabled), 
		.rollback_enabled(rollback_enabled), 
	);

endmodule 