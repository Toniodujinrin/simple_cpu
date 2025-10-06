module pattern_history_table #(parameter INDEX_LEN = 10)  (
	input [INDEX_LEN-1:0] index_read, 
	input [INDEX_LEN-1:0] index_write, 
	input increment_decrement, reset, write_enabled, clk, 
	output [1:0] count
); 


	localparam TOTAL_COUNT_LEN = INDEX_LEN*2;
	localparam LOCATIONS = 2**INDEX_LEN; 

	
	
	 
	wire [LOCATIONS-1:0] count_select_write; 
	wire [TOTAL_COUNT_LEN-1:0] total_count_wires; 
	
	reg_mux#(.REG_N(LOCATIONS), .WIDTH(2)) COUNTER_MUX(.in(total_count_wires), .select(index_read), .out(count));
	
	
	register_address_decoder#(.INPUT_WIDTH(INDEX_LEN))  WRITE_DECODER(.in(index_write), .out(count_select_write));
	
	
	genvar i; 
	generate 
		for(i=0; i<LOCATIONS; i = i+1)
			begin: LOCATION
				sat_counter_2bit COUNTER(.clk(clk), .reset(reset), .enabled(write_enabled & count_select_write[i]), .in(increment_decrement), .count(total_count_wires[i*2 +: 2]));
			end 
	endgenerate 


endmodule 

