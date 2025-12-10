///////////////////////////////////////////////////////////////////////////////
// File:        branch_history_table.v 
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: Branch History Table for bimodal predictor
///////////////////////////////////////////////////////////////////////////////


module branch_history_table #(parameter INDEX_LEN = 7, parameter TAG_LEN = 7) (
    input [TAG_LEN-1:0] tag_bits_read,
    input [TAG_LEN-1:0] tag_bits_write,  
    input [INDEX_LEN-1:0] index_read,
    input [INDEX_LEN-1:0] index_write,  
    input increment_decrement, clk, reset, write_enabled,
    output [1:0] count,
    output tag_not_added
); 
    
    
    localparam COUNT_LEN = 2; 
    localparam LOCATIONS = 2**INDEX_LEN; 
    
    
 
    wire [LOCATIONS-1:0] tag_not_added_array; 
    wire [LOCATIONS-1:0] set_selecter_read; 
    wire [LOCATIONS-1:0] set_selecter_write; 
    wire [LOCATIONS*COUNT_LEN-1:0] total_counts; 
    
    

    // mask the selected read set and reduce OR to get single tag_not_added
    assign tag_not_added = |(tag_not_added_array & set_selecter_read); //use a mask to select the right address and the reduction OR
    
    //decodes lower INDEX_LEN pc bits into one-hot addresses 
    register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER_READ(
        .in(index_read),
        .out(set_selecter_read)
    );

    register_address_decoder#(.INPUT_WIDTH(INDEX_LEN)) DECODER_WRITE(
        .in(index_write),
        .out(set_selecter_write)
    );
    
    // read the selected 2-bit count for the read index
    reg_mux #(.WIDTH(COUNT_LEN), .REG_N(LOCATIONS)) COUNT_MUX (
        .in(total_counts), 
        .select(index_read), 
        .out(count)
    ); 
    
    
    //generates LOCATIONS 2-way sets 
    genvar i; 
    generate 
        for(i=0; i< LOCATIONS; i= i +1)
            begin:location 
                set_2_way#(.TAG_LEN(TAG_LEN))  SET(
                    .clk(clk), 
                    .set_selected_read(set_selecter_read[i]), 
                    .set_selected_write(set_selecter_write[i]), 
                    .reset(reset), 
                    .increment_decrement(increment_decrement), 
                    .target_count(total_counts[i*COUNT_LEN +: COUNT_LEN]), 
                    .tag_in_read(tag_bits_read),
                    .tag_in_write(tag_bits_write),  
                    .tag_not_added(tag_not_added_array[i]), 
                    .write_enabled(write_enabled)
                ); 
            end 
    endgenerate 
    
    

endmodule 


module set_2_way #(parameter TAG_LEN = 7) 
   (
    input  clk, set_selected_read, set_selected_write, reset, write_enabled, increment_decrement, 
    input  [TAG_LEN-1:0] tag_in_read, tag_in_write,
    output [1:0] target_count, 
    output tag_not_added
   ); 

    // PARAMS 
    localparam WAY_N = 2; // must be 2 in this module
    localparam PLRU_DEFAULT = 1'b0; 

    wire tag_not_added_read; 
    wire tag_not_added_write; 
    // Per-way signals
    wire [TAG_LEN-1:0] tag_out [0:WAY_N-1];
    wire [SAT_COUNT_SIZE-1:0] count_out [0:WAY_N-1]; 
    wire valid       [0:WAY_N-1];
    wire hit_read    [0:WAY_N-1]; 
    wire hit_write   [0:WAY_N-1]; 
    // update_way not used in this simplified design; kept for clarity
    wire update_way  [0:WAY_N-1]; 

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
    //  - On a hit, point PLRU to the *other* way (the hit way becomes MRU).
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
                    else if ((hit_write[1] | replace_way1) & hit_read[1]) plru_bit <= 1'b0; //if way1 is being written to and read from
                    else plru_bit <= plru_bit;  //plru bit remains unchanged if ambiguous  
                end
    end
     
    ///////////////////////////////////////
    // Final read and write Assignments
    //////////////////////////////////////
    // Assign count. default to 2'b01 if no tag is found on read, and also raise tag_not_added flag
    assign target_count = hit_read[0] ? count_out[0] : hit_read[1] ? count_out[1] : 2'b01;
    assign tag_not_added = tag_not_added_read;  

    // Ways
    way #(.TAG_WIDTH(TAG_LEN)) WAY0 (
        .clk(clk),
        .global_reset(reset),
        .write_enabled(hit_write[0] & write_enabled),
        .replacement_mode(replace_way0),
        .increment_decrement(increment_decrement),
        .tag_in(tag_in_write),
        .tag_out(tag_out[0]),
        .sat_count(count_out[0]),
        .valid(valid[0])
    );

    way #(.TAG_WIDTH(TAG_LEN)) WAY1 (
        .clk(clk),
        .global_reset(reset),
        .write_enabled(hit_write[1] & write_enabled),
        .replacement_mode(replace_way1),
        .increment_decrement(increment_decrement),
        .tag_in(tag_in_write),
        .tag_out(tag_out[1]),
        .sat_count(count_out[1]),
        .valid(valid[1])
    );

endmodule





module way #(parameter TAG_WIDTH = 7)(
    input clk, write_enabled, replacement_mode, increment_decrement, global_reset, //signal bits and clk 
    input  [TAG_WIDTH-1:0] tag_in, 
    output [TAG_WIDTH-1:0] tag_out,
    output [1:0] sat_count,   //saturated counter count 
    output reg  valid //valid bit to indicate if the contents of the way is valid. Default set to 0 when module is initialized
); 
     
    // Tag storage: written only on replacement
    register #(.WIDTH(TAG_WIDTH)) TAG_REG (
        .clk(clk),
        .in(tag_in),
        .out(tag_out),
        .write_selected(replacement_mode),
        .write_enabled(replacement_mode),
        .reset(global_reset)
    );

    // 2-bit saturating counter:
    //  - reset on global reset or replacement (fresh entry → default bias)
    //  - count only when this way is selected (hit) and when the branch is being updated 
    sat_counter_2bit SAT_COUNT (
        .clk(clk),
        .reset(global_reset | replacement_mode),
        .enabled(write_enabled), //only increment counter when way is being updated
        .in(increment_decrement),
        .count(sat_count)
    );

    // Valid bit
    always @(posedge clk or posedge global_reset) begin
        if (global_reset)       valid <= 1'b0;
        else if (replacement_mode) valid <= 1'b1;
    end
endmodule




