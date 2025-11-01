///////////////////////////////////////////////////////////////////////////////
// File:        immediate_generator.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: Immediate Value Generator
///////////////////////////////////////////////////////////////////////////////

module immediate_generator (
    input  wire [15:0] instruction, 
    output reg  [15:0] immediate_16bit
);
    // Instruction class
    wire [1:0] cls = instruction[15:14];
    wire [1:0] cls00_opcode = instruction[13:12];  // for LDR/STR vs ADDI/SUBI
    wire [2:0] cls10_opcode = instruction[13:11];  // for branch vs compare
	wire cls10_opt = instruction[0]; //opt bit for branch/compare

    // Immediate slices
    wire [5:0]  imm6  = instruction[5:0];
    wire [10:0] imm11 = instruction[10:0];
    wire [6:0]  imm7  = instruction[6:0];
    wire [3:0]  imm4  = instruction[4:1]; // bits [0] unused

    // Sign extensions
    function [15:0] signext6(input [5:0] val);
        signext6 = {{10{val[5]}}, val};
    endfunction

    function [15:0] signext7(input [6:0] val);
        signext7 = {{9{val[6]}}, val};
    endfunction

    function [15:0] signext11(input [10:0] val);
        signext11 = {{5{val[10]}}, val};
    endfunction

    function [15:0] zeroext4(input [3:0] val);
        zeroext4 = {12'b0, val};
    endfunction

    function [15:0] zeroext7(input [6:0] val);
        zeroext7 = {9'b0, val};
    endfunction

    always @(*) begin
        case (cls)
            // Class 00: LDR/STR/ADDI/SUBI
            2'b00: begin
                immediate_16bit = signext6(imm6);
                if (cls00_opcode == 2'b00 || cls00_opcode == 2'b01) begin
                    // LDR or STR — shift for halfword alignment
                    immediate_16bit = immediate_16bit <<< 1;
                end
                // ADDI/SUBI: no shift needed
            end

            // Class 10: Branch & Compare
            2'b10: begin
                if (cls10_opcode <= 3'b100) 
						begin
							  // Branch instructions — IMM11
							  immediate_16bit = signext11(imm11) <<< 1;
						end 
					else 
						begin
                    // Compare instructions — IMM7
                    if (cls10_opcode == 3'b101) 
								begin
									// CMP (signed)
									immediate_16bit = signext7(imm7);
								end 
						  else if (cls10_opcode == 3'b110) 
								begin
									// UCMP (unsigned)
									immediate_16bit = zeroext7(imm7);
								end 
						  else 
								begin
									immediate_16bit = 16'b0;
								end
                end
            end

            // Class 11: Shift with immediate
            2'b11: 
					begin
						immediate_16bit = zeroext4(imm4);
					end

            // Class 01: R-type, no immediate
            default: 
					begin
                immediate_16bit = 16'b0;
					end
        endcase
    end
endmodule