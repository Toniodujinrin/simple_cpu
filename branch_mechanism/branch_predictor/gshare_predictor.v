module gshare_predictor(clk, reset, update_en, pc_bits_read, pc_bits_write, history_write, outcome, prediction); 
	parameter HISTORY_LEN = 8 ; 
	input [HISTORY_LEN-1:0] pc_bits_read;
	input [HISTORY_LEN-1:0] pc_bits_write; 
	input [HISTORY_LEN-1:0] history_write;
	input clk, reset, update_en, outcome; 
	output prediction; 
	
	wire [HISTORY_LEN-1:0] history_read;
	wire [HISTORY_LEN-1:0] index_read = history_read ^ pc_bits_read; 
	wire [HISTORY_LEN-1:0] index_write = history_write ^ pc_bits_write; 
	wire [1:0] count;
	wire out_bit; //not used
	assign prediction = count[1]; 
	
	shift_reg_n#(.WIDTH(HISTORY_LEN)) HISTORY_REG(
		.clk(clk),
		.in(outcome), 
		.out(out_bit), 
		.value(history_read), 
		.reset(reset), 
		.enabled(update_en)
		);

	pattern_history_table#(.INDEX_LEN(HISTORY_LEN))  COUNTER_FILE(
		.clk(clk) ,
		.index_read(index_read), 
		.index_write(index_write), 
		.count(count), 
		.increment_decrement(outcome), 
		.reset(reset), 
		.write_enabled(update_en)
	); 
endmodule 