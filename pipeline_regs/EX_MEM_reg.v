///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: EX-MEM pipeline register 
///////////////////////////////////////////////////////////////////////////////

module EX_MEM_reg(
    input clk, reset, write_enable
    input [15:0] alu_result_in, 
    input ctrl_MEM_read_in, ctrl_MEM_write_in, 
    input [1:0] ctrl_WB_in, 
    input [2:0] rs2_addr_in, rd_addr_in,
    input [15:0] rs2_data_in, 
    input [6:0] alu_cpsr_in 

    output reg [15:0] alu_result_out, 
    output reg ctrl_MEM_read_out, ctrl_MEM_write_out, 
    output reg [1:0] ctrl_WB_out, 
    output reg [2:0] rs2_addr_out, rd_addr_out,
    output reg [15:0] rs2_data_out, 
    output reg [6:0] alu_cpsr_out
); 

always @(posedge clk or posedge reset) begin 
    if (reset) begin 
        rs2_addr_out       <= 3'b0; 
        rd_addr_out        <= 3'b0;  
        rs2_data_out       <= 16'b0;  
        ALU_OP_out         <= 8'b0; 
        ALU_options_out    <= 5'b0; 
        ctrl_MEM_read_out  <= 1'b0; 
        ctrl_MEM_write_out <= 1'b0; 
        ctrl_WB_out        <= 2'b0; 
    end
    else if (write_enable) begin 
        rs2_addr_out       <= rs2_addr_in; 
        rd_addr_out        <= rd_addr_in;  
        rs2_data_out       <= rs2_data_in; 
        ALU_OP_out         <= ALU_OP_in; 
        ALU_options_out    <= ALU_options_in; 
        ctrl_MEM_read_out  <= ctrl_MEM_read_in; 
        ctrl_MEM_write_out <= ctrl_MEM_write_in; 
        ctrl_WB_out        <= ctrl_WB_in; 
    end 
end


endmodule 