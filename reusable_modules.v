//extensible register decoder
module register_address_decoder(in,out);  
	parameter INPUT_WIDTH = 3;
	localparam OUTPUT_WIDTH = 2**INPUT_WIDTH; 
	input [INPUT_WIDTH-1:0] in;  
	output reg [OUTPUT_WIDTH-1:0] out; 
	
	always@(*)
		begin
			out = '0; 
			out[in] = 1; 
		end 
endmodule 

module d_ff(clk,d,q,reset); 
	input clk,d,reset; 
	output reg q; 
	
	always @(posedge clk or posedge reset)
		begin 
			if(reset) 
				q <= 0; 
			else 
				q <= d; 
		end
endmodule 

//extensible reg_mux  
module reg_mux (in, select, out);
	parameter REG_N = 8; 
    parameter WIDTH = 16; 
    input  [(REG_N*WIDTH)-1:0] in; 
    input  [$clog2(REG_N)-1:0] select;   
    output reg [WIDTH-1:0] out; 

    always @(*) 
	 begin
        out = in[ (select*WIDTH) +: WIDTH ];  
    end

endmodule

module register(clk,in, out, write_selected, write_enabled, reset); 
	parameter WIDTH = 8; 
	input reset, write_selected, write_enabled; 
	input [WIDTH-1:0] in; 
	input clk; 
	output wire [WIDTH-1:0] out; 
	
	//generate flip flops 
	genvar i; 
	generate 
	for(i = 0; i < WIDTH; i = i +1)
		begin:REG_BITS
			d_ff D_FF(.clk(clk),.d(write_selected & write_enabled? in[i] : out[i]),.q(out[i]),.reset(reset)); 
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
				.d(p_load_enabled? load[i]:
				shift_enabled?	(i==0 ? in: ff_out[i-1]):
									ff_out[i]),
				.q(ff_out[i]),
				.reset(reset)
			);
		end
	endgenerate 

endmodule 