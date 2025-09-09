module logical_history_table(clk, taken_not_taken, pc_bits, reset, history, read_only); 
	parameter INDEX_LEN = 7; 
	localparam LOCATIONS = 2**INDEX_LEN; 
	parameter HISTORY_LEN = 10; 
	
	localparam VALUE_WIRES_LENGTH = HISTORY_LEN*LOCATIONS; 
	
	input clk, taken_not_taken, reset, read_only; 
	input [INDEX_LEN-1:0] pc_bits; 
	output [HISTORY_LEN-1:0] history;	
	wire [LOCATIONS-1:0] out_bits; //not used 
	wire [LOCATIONS-1:0] history_select; 
	wire [VALUES_WIRES_LENGTH-1:0] values; 
	
	
	
	//decodes pc bits into 1 hot addresses 
	register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER(.in(pc_bits),.out(history_select));  
	
	reg_mux(.REG_N(LOCATIONS), .WIDTH(HISTORY_LEN))  OUTPUT_MUX(.in(values), .select(pc_bits), .out(history));
	genvar i; 
	generate 
		for(i=0; i<LOCATIONS; i = i+1)
			begin:LOCATION
				shift_reg_n#(.WIDTH(HISTORY_LEN))  HISTORY_REG(.clk(clk),.in(taken_not_taken), .out(out_bits[i]), .value(values[i*HISTORY_LEN +: HISTORY_LEN]), .reset(reset), .enabled(history_select[i] & ~read_only));
			end
	endgenerate 


endmodule 
 


module shift_reg_n(clk,in, out, value, reset, enabled);
	parameter WIDTH = 10; 
	input in, reset, clk, enabled; 
	output out; 
	output wire [WIDTH-1:0]  value; 
	wire [WIDTH-1:0] ff_out; 
	
	assign value = ff_out; 
	assign out = ff_out[WIDTH-1]; 
	genvar i; 
	generate
	for(i=0; i < WIDTH; i = i+1) 
		begin
			
			d_ff  D_FF(.clk(clk),.d(enabled?(i==0 ? in: ff_out[i-1]):ff_out[i]),.q(ff_out[i]),.reset(reset));
		end
	endgenerate 

endmodule 




	