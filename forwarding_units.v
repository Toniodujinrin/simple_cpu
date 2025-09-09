//forwarding unit for the execution stage of the pipeline
module forwarding_unit_a(ID_EX_RegS1, ID_EX_RegS2, EX_MEM_RegD, MEM_WB_RegD, EX_MEM_wb_enabled, MEM_WB_wb_enabled, forward_rs1, forward_rs2);
	input wire [2:0] ID_EX_RegS1, ID_EX_RegS2, EX_MEM_RegD, MEM_WB_RegD; 
	output reg [1:0] forward_rs1, forward_rs2; 
	input EX_WB_wb_enabled, MEM_WB_wb_enabled; 
	
	always @(*)
		begin 
		
			//forward rs1 
			if(EX_MEM_wb_enabled && (EX_MEM_RegD == ID_EX_RegS1)) //if data will be written back to a register destination and current instruction depends on previous instruction, 
				forward_rs1 = 2'b10; 											//forward data from memory stage (prev instruction) to ex stage (current instruction)
			else if(MEM_WB_wb_enabled && (MEM_WB_RegD == ID_EX_RegS1)) //current instruction depends on instruction before last. 
				forward_rs1 = 2'b01; 										  			//forward data from instruction before last to current instruction execution stage
			else 
				forward_rs1= 2'b00; 														//continue with regular flow
				
			
			//forward rs2 
			if(EX_MEM_wb_enabled && (EX_MEM_RegD == ID_EX_RegS2))
				forward_rs2 = 2'b10; 
			else if (MEM_WB_wb_enabled && (MEM_WB_RegD == ID_EX_RegS2))
				forward_rs2 = 2'b01;
			else
				forward_rs2 = 2'b00; 
		
		end

endmodule 


//forwarding unit for the mem stage of the pipeline
//used for store after load instructions  
module forwarding_unit_b(MEM_WB_RegD, MEM_WB_MemRead, EX_MEM_RegS2, EX_MEM_MemWrite, forward_data_in); 

	input MEM_WB_MemRead, EX_MEM_MemWrite; 
	input [2:0] MEM_WB_RegD, EX_MEM_RegS2; 
	output reg forward_data_in; 
	
	
	always @(*) 
		begin 
			if(MEM_WB_MemRead && EX_MEM_MemWrite && (EX_MEM_RegS2 == MEM_WB_RegD)) //if current instruction is a store and previous instruction is a load, and store depends on load 
				forward_data_in = 1'b1; //forward the data from load into the store
			else 
				forward_data_in = 1'b0; //continue with regular flow
		end 

endmodule 