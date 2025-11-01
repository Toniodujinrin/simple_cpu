///////////////////////////////////////////////////////////////////////////////
// File:        register_file.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: Extensible Register File
///////////////////////////////////////////////////////////////////////////////

module register_file #(parameter ADDR_WIDTH= 3, REG_N = 2**ADDR_WIDTH, REG_WIDTH = 16;)(
	input clk, write_enabled, reset, 
	output [REG_WIDTH-1:0] read_bus_1, read_bus_2;
	input [REG_WIDTH-1:0] write_bus; 
	input [ADDR_WIDTH-1:0] read_1_addr, read_2_addr, write_addr;
)		

	wire [REG_N-1:0] write_reg_select; 
	wire [(REG_N*REG_WIDTH)-1:0] register_output; 

	register_address_decoder#(.INPUT_WIDTH(ADDR_WIDTH)) WRITE_REG_DECODE(.in(write_addr),.out(write_reg_select)); 
	
	genvar i; 
	generate
	for (i=0; i < REG_N; i = i +1)
		begin:register_file 
			register REG(
				.clk(clk),
				.in(write_bus),
				.out(register_output[(((i+1)*REG_WIDTH)-1):(i*REG_WIDTH)]), 
				.write_selected(write_reg_select[i]), 
				.write_enabled(write_enabled), 
				.reset(reset)
			); 
		end 
	endgenerate 
	
	reg_mux#(.REG_N(REG_N), .WIDTH(REG_WIDTH)) READ_BUS_1(.select(read_1_addr), .in(register_output), .out(read_bus_1)); 
	reg_mux#(.REG_N(REG_N), .WIDTH(REG_WIDTH)) READ_BUS_2(.select(read_2_addr), .in(register_output), .out(read_bus_2)); 
	
endmodule 






	




	


	
