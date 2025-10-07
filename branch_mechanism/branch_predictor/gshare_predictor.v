module gshare_predictor #(parameter HISTORY_LEN = 8) (
	input [15:0] pc_bits_read, pc_bits_write, 
	input [HISTORY_LEN-1:0] history_write,
	input clk, reset, write_enabled, outcome,
	output prediction,
	output [HISTORY_LEN-1:0] history_read_out
); 
	localparam COUNT_LEN = 2; 
	
	wire [HISTORY_LEN-1:0] history_read;
	wire [HISTORY_LEN-1:0] index_read = history_read ^ pc_bits_read[9:2]; 
	wire [HISTORY_LEN-1:0] index_write = history_write ^ pc_bits_write[9:2]; 
	wire [COUNT_LEN-1:0] count;
	wire out_bit; //not used
	assign prediction = count[1]; 
	assign history_read_out = history_read; 
	
	shift_reg_n#(.WIDTH(HISTORY_LEN)) HISTORY_REG(
		.clk(clk),
		.in(outcome), 
		.out(out_bit), 
		.value(history_read), 
		.reset(reset), 
		.enabled(write_enabled)
		);

	pattern_history_table#(.INDEX_LEN(HISTORY_LEN))  COUNTER_FILE(
		.clk(clk) ,
		.index_read(index_read), 
		.index_write(index_write), 
		.count(count), 
		.increment_decrement(outcome), 
		.reset(reset), 
		.write_enabled(write_enabled)
	); 
endmodule 