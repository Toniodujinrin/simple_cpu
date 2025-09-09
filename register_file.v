module register_file(clk,read_1_addr,read_2_addr, write_addr, read_bus_1, read_bus_2, write_bus, write_enabled,reset);
	parameter ADDR_WIDTH= 3;
   parameter REG_N = 2**ADDR_WIDTH; 
	parameter REG_WIDTH = 16; 
	output [REG_WIDTH-1:0] read_bus_1, read_bus_2;
	input [REG_WIDTH-1:0] write_bus; 
	input [ADDR_WIDTH-1:0] read_1_addr, read_2_addr, write_addr;
			
	input reset,clk, write_enabled ; 
	wire [REG_N-1:0] write_reg_select; 
	wire [(REG_N*REG_WIDTH)-1:0] register_output; 

	register_address_decoder#(.INPUT_WIDTH(ADDR_WIDTH)) WRITE_REG_DECODE(.in(write_addr),.out(write_reg_select)); 
	
	genvar i; 
	generate
	for (i=0; i < REG_N; i = i +1)
		begin:register_file 
			register REG(.clk(clk),.in(write_bus),.out(register_output[(((i+1)*REG_WIDTH)-1):(i*REG_WIDTH)]), .write_selected(write_reg_select[i]), .write_enabled(write_enabled), .reset(reset)); 
		end 
	endgenerate 
	
	reg_mux#(.REG_N(REG_N), .WIDTH(REG_WIDTH)) READ_BUS_1(.select(read_1_addr), .in(register_output), .out(read_bus_1)); 
	reg_mux#(.REG_N(REG_N), .WIDTH(REG_WIDTH)) READ_BUS_2(.select(read_2_addr), .in(register_output), .out(read_bus_2)); 
	
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


	


	
