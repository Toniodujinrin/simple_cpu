///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: IF-ID pipeline register 
///////////////////////////////////////////////////////////////////////////////

module IF_ID_reg(
    input clk, reset, write_enable, 
    input wire [15:0] instruction, 
    output reg [15:0] instruction)
 

    always @(posedge clk or posedge reset)
    begin 
        if(reset)
        begin 
            instruction_out = 16'b0; 
        end 
        else if(write_enable)
        begin
            instruction_out <= instruction_in; 
        end 
    end 
endmodule 