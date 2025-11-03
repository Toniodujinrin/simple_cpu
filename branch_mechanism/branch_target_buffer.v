///////////////////////////////////////////////////////////////////////////////
// File:        branch_target_buffer.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: Branch target Buffer 
///////////////////////////////////////////////////////////////////////////////


module branch_target_buffer(clk, pc_bits_read, pc_bits_write, target_address_out, target_address_in, miss, write_enabled, reset); 
	parameter INDEX_LEN = 7; 
	localparam LOCATIONS = 2**INDEX_LEN;
	parameter TAG_LEN = 8; 
	parameter ADDRESS_LEN = 16; 
	parameter TOTAL_ADDRESS_WIRES_LEN = LOCATIONS*ADDRESS_LEN; 
	
	input clk, write_enabled, reset; 
	input [15:0]  pc_bits_read; 
	input [15:0]  pc_bits_write;
	
	input [15:0] target_address_in; 
	output [15:0] target_address_out; 
	output miss; 
	
	wire [LOCATIONS-1:0] tag_not_added_array; 
	wire [TAG_LEN-1:0] tag_read =  pc_bits_read[14:7]; 
	wire [TAG_LEN-1:0] tag_write = pc_bits_write[14:7]; 
	wire [INDEX_LEN-1:0] index_read = pc_bits_read[6:0]; 
	wire [INDEX_LEN-1:0] index_write = pc_bits_write[6:0]; 
	
	
	wire [LOCATIONS-1:0] read_set_selecter; 
	wire [LOCATIONS-1:0] write_set_selecter; 
	
	
	wire [TOTAL_ADDRESS_WIRES_LEN-1:0] target_address_wires; 
	
	
	
	assign miss = |(tag_not_added_array & read_set_selecter); //use a mask to select the right address and the reduction OR
	
	// Decodes lower 7 pc bits into 128 one hot addresses 
	register_address_decoder#(.INPUT_WIDTH(7)) READ_DECODER (.in(index_read),.out(set_selecter_read));
	register_address_decoder#(.INPUT_WIDTH(7)) WRITE_DECODER(.in(index_write),.out(set_selecter_write));
	
	// Multiplexes target addresses from a flat array
	reg_mux(.REG_N(LOCATIONS), .WIDTH(ADDRESS_LEN))  OUTPUT_MUX(.in(target_address_wires), .select(index_read), .out(target_address_out));
	
	// Generates 128 2-way sets 
	genvar i; 
	generate 
		for(i=0; i< LOCATIONS; i= i +1)
			begin:location 
				set_2_way#(.WAY_N(2),.TAG_LEN(TAG_LEN), .ADDRESS_LEN(ADDRESS_LEN))  SET(
					.clk(clk), 
					.set_selected_read(set_selecter_read[i]), 
					.set_selected_write(set_selecter_write[i]), 
					.tag_in_read(tag_read), 
					.tag_in_write(tag_write),
					.tag_not_added(tag_not_added_array[i]), 
					.reset(reset), 
					.target_address(target_address_wires[i*ADDRESS_LEN +:ADDRESS_LEN]), 
					.address_in(target_address_in), 
					.write_enabled(write_enabled)
				); 
			end 
	endgenerate
	
endmodule 


module set_2_way(clk, set_selected_read, set_selected_write, tag_in_read, tag_in_write, tag_not_added, reset, target_address, address_in, write_enabled);
	 // PARAMS 
    parameter WAY_N = 2; // must be 2 in this module
	parameter ADDRESS_LEN = 16; 
	parameter TAG_LEN = 8; 
	localparam PLRU_DEFAULT = 1'b0; 
	 
	 // IO 
    input  clk, set_selected_read, set_selected_write, reset; 
    input  [TAG_LEN-1:0] tag_in_read, tag_in_write;
	input  [ADDRESS_LEN-1:0] address_in; 
	output [ADDRESS_LEN-1:0] target_address; 
    output tag_not_added; 
	 
	 wire tag_not_added_read; 
	 wire tag_not_added_write; 
	  
    // Per-way signals
    wire [TAG_LEN-1:0] tag_out [0:WAY_N-1];
	 
	 
	wire [ADDRESS_LEN-1:0] address_out [0:WAY_N-1]; 
    wire       valid   [0:WAY_N-1];
	wire 		hit_read    [0:WAY_N-1]; 
	wire       hit_write   [0:WAY_N-1]; 
	wire       update_way [0:WAY_N-1]; 

	///////////////////////////////
    // Hit detect (only if valid)
	//////////////////////////////
    // Hit detect for read
    assign hit_read[0] = set_selected_read && valid[0] && (tag_out[0] == tag_in_read);
    assign hit_read[1] = set_selected_read && valid[1] && (tag_out[1] == tag_in_read);
	 
	//Hit detect for write 
	assign hit_write[0] = set_selected_write && valid[0] && (tag_out[0] == tag_in_write); 
	assign hit_write[1] = set_selected_write && valid[1] && (tag_out[1] == tag_in_write); 

	
	/////////////////////////////
    // Miss / not-added (tag is said to be not found if the set was selected and both ways do not have the tag (miss))
	////////////////////////////
	// Target not added for read request 
    assign tag_not_added_read = set_selected_read && !(hit_read[0] || hit_read[1]);   
	  
	// Target not added for write request 
	assign tag_not_added_write = set_selected_write && !(hit_write[0] || hit_write[1]);
	 
	 
	 /////////////////////////////
	 // Replacement Policy
	 ////////////////////////////
	 
	reg plru_bit;  // 0 = way0 was LRU, replace way0; 1 = way1 was LRU

    // Check if the way is valid , only use write signals because replacement is only relevant when writing, not reading 
	 wire choose_way0_invalid = tag_not_added_write && set_selected_write && !valid[0];
    wire choose_way1_invalid = tag_not_added_write && set_selected_write && !valid[1];

	 // Prefer an invalid way; else use PLRU.
    wire replace_way0 = tag_not_added_write && (                   //only replace when the tag is not found in the set 
                          choose_way0_invalid ||
                          (!choose_way0_invalid && !choose_way1_invalid && (plru_bit == 1'b0))
                        );
    wire replace_way1 = tag_not_added_write && (
                          choose_way1_invalid ||
                          (!choose_way0_invalid && !choose_way1_invalid && (plru_bit == 1'b1))
                        );
	////////////////////////////
	// PLRU update:
	////////////////////////////
	//  - PLRU is updated on either read or write hit. It should check for general access (read or write) when deciding the MRU or LRU. 
	//  - If one "way" is written to and the other "way" is read from, PLRU state is unchanged 
    //  - On a hit, point PLRU to the other(LRU) way (the hit way becomes MRU).
    //  - On a replacement, set PLRU away from the just-filled way (filled way becomes MRU).
    always @(posedge clk or posedge reset) begin
        if (reset) 
			  begin
					plru_bit <= PLRU_DEFAULT;     // arbitrary reset value
			  end 
		  else if (set_selected_read & !set_selected_write) //if set is only being read 
			  begin
					if (hit_read[0]) plru_bit <= 1'b1;  // sets way0 to MRU and sets way1 to LRU
					else if (hit_read[1]) plru_bit <= 1'b0; // sets way1 to MRU and sets way0 to LRU
			  end
			else if (set_selected_write & !set_selected_read)  //if set is only being written to 
			  begin 
					if (hit_write[0] | replace_way0) plru_bit <= 1'b1; 
					else if (hit_write[1] | replace_way1) plru_bit <= 1'b0; 
			  end 
			else if (set_selected_write & set_selected_read)  //if set is both being written to and read from 
				begin 
					if((hit_write[0] | replace_way0) & hit_read[0]) plru_bit <= 1'b1;  //if way0 is being both written to and read from 
					else if ((hit_write[1] | replace_way1) & hit_write[1]) plru_bit <= 1'b0; //if way1 is being written to and read from
					else plru_bit <= plru_bit;  //plru bit remains unchanged if both ways are being accessed. 
				end
    end
	 
	 ///////////////////////////////////////
	 // Final read and write Assignments
	 //////////////////////////////////////
    // Assign count. default to 2'b01 if not tag is found, and also raise tag_not_found flag
    assign target_address = hit_read[0] ? address_out[0] : hit_read[1]? address_out[1]: 16'b0;
	assign tag_not_added = tag_not_added_read; 
	assign update_way[0] =  hit_write[0] && write_enabled && (address_in != address_out[0]); 
	assign update_way[1] =  hit_write[1] && write_enabled && (address_in != address_out[1]); 

    // Ways
    way#(.ADDRESS_LEN(ADDRESS_LEN), .TAG_LEN(TAG_LEN)) WAY0 (
        .clk(clk),
        .global_reset(reset),
        .replacement_mode((replace_way0 & write_enabled) | update_way[0]),  //condition explanation: if write is enabled AND the tag is not yet recorded (and this is the appropriate "way" to be replaced) OR you want to update the value of the tag
        .tag_in(tag_in_write),
        .tag_out(tag_out[0]),
		  .address_in(address_in), 
		  .address_out(address_out[0]),
        .valid(valid[0]), 
    );

    way#(.ADDRESS_LEN(ADDRESS_LEN), .TAG_LEN(TAG_LEN)) WAY1 (
        .clk(clk),
        .global_reset(reset),
        .replacement_mode((replace_way1 & write_enabled) | update_way[1]), //condition explanation: if write is enabled AND the tag is not yet recorded (and this is the appropriate "way" to be replaced) OR you want to update the value of the tag
        .tag_in(tag_in_write),
        .tag_out(tag_out[1]),
		.address_in(address_in),
		.address_out(address_out[1]), 
        .valid(valid[1]),
    );

endmodule


module way(clk, replacement_mode, global_reset, address_in, address_out, tag_in, tag_out, valid);
	parameter TAG_LEN = 8; 
	parameter ADDRESS_LEN = 16; 
	input clk, replacement_mode, global_reset; //signal bits and clk 
	input [ADDRESS_LEN-1:0] address_in; 
	output [ADDRESS_LEN-1:0] address_out; 
	input  [TAG_LEN-1:0] tag_in; 
    output [TAG_LEN-1:0] tag_out; 
    output reg   valid; //valid bit to indicate if the contents of the way is valid. Default set to 0 when module is initialized
	 
    // Tag storage: written only on replacement
    register #(.WIDTH(TAG_LEN)) TAG_REG (
        .clk(clk),
        .in(tag_in),
        .out(tag_out),
        .write_selected(replacement_mode),
        .write_enabled(replacement_mode),
        .reset(global_reset)
    );

    // Address register:
    // Written only on replacement 

	register #(.WIDTH(ADDRESS_LEN)) ADD_REG (
        .clk(clk),
        .in(address_in),
        .out(address_out),
        .write_selected(replacement_mode),
        .write_enabled(replacement_mode),
        .reset(global_reset)
    );
    // Valid bit
    always @(posedge clk or posedge global_reset) begin
        if (global_reset)       valid <= 1'b0;
        else if (replacement_mode) valid <= 1'b1;
    end
endmodule



