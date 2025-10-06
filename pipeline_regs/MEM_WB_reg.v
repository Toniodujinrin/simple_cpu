///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: MEM_WB pipeline register 
///////////////////////////////////////////////////////////////////////////////


module MEM_WB_REG(
    input clk, reset, write_enable, 
    input [1:0] ctrl_WB_in; 
    input [2:0] rd_addr_in,  
    output [2:0] rd_addr_out, 
    output [1:0] ctrl_WB_out
); 

always @(posedge clk or posedge reset) 
begin 
    if(reset)
    begin 
        ctrl_WB_out <= 0; 
        rd_addr_out <= 0;  
    end 

    else if (write_enable)
    begin 
        ctrl_WB_out <= ctrl_WB_in;
        rd_addr_out <= rd_addr_in; 
    end 
end 


endmodule 