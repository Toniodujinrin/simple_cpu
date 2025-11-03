///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: 4-way set associative cache with write-back, write-allocate policy  
///////////////////////////////////////////////////////////////////////////////

module cache_controller(); 

endmodule 


module cache  #(parameter BLOCK_SIZE = 8, parameter INDEX_WIDTH, parameter TAG_WIDTH, parameter DATA_WIDTH) (
    input clk, reset, 
    input [TAG_WIDTH-1:0] input_tag, 
    input [DATA_WIDTH-1:0] write_data, 
    output [DATA_WIDTH-1:0] read_data, 
    output [DATA_WIDTH-1:0] evicted_data, 
    output evict_valid, evict_dirty, 
    output data_ready
    
); 
    

    localparam BLOCK_OFFSET_WIDTH = clog2(BLOCK_SIZE); 
    
    


endmodule


module #(parameter SET_WIDTH, DATA_WIDTH, TAG_WIDTH) set(
    input clk, reset, replace_enabled, write_enabled 
    input tag_in 
); 
    localparam SET_DATA_LEN = SET_WIDTH * DATA_WIDTH;
    localparam SET_TAG_LEN = SET_WIDTH * TAG_WIDTH; 
    localparam PLRU_BIT_SIZE = 3; 


    reg [SET_DATA_LEN-1:0] set_data; 
    reg [SET_WIDTH-1:0] set_dirty_bits; //grouped dirty bits from all the ways 
    reg [SET_WIDTH-1:0] set_valid_bits  //grouped valid bits from all the ways 
    reg [SET_TAG_LEN-1:0] set_tag_bits; //grouped tag bits from all the ways


    reg []
    
    wire [SET_WIDTH-1:0] set_select; //chooses the way whose tag matches the input tag 
    wire tag_not_found; 

    set_selector #(.SET_WIDTH(SET_WIDTH), .TAG_WIDTH(TAG_WIDTH), .SET_TAG_LEN(SET_TAG_LEN)) SELECTOR(
        .tag_in(tag_in), 
        .tags(set_tag_bits), 
        .set_select(set_select), 
        .tag_not_found(tag_not_found)
    )

    //plru update logic
    always


    generate
        genvar i; 
        for(i = 0; i < SET_WIDTH; i = i +1)
            begin: way_instantiation 
                way #(.DATA_WIDTH(DATA_WIDTH), .TAG_WIDTH(TAG_WIDTH)) WAY_N()
            end 
    endgenerate

endmodule 

module #(parameter SET_WIDTH, TAG_WIDTH, SET_TAG_LEN) set_selector( 
    input [TAG_WIDTH-1:0] tag_in
    input [SET_TAG_LEN-1:0] tags; 
    output wire [SET_WIDTH-1:0] set_select //one hot ouput to select the actual way being refered to 
    output reg tag_not_found; //asserted when the tag in 
); 

    generate 
        genvar i; 
        for (i =0; i < SET_WIDTH; i = i +1)
            begin 
              assign set_select[i] =  tag_in == tags[TAG_WIDTH*i +: TAG_WIDTH];  //set_select[i] is set to 1 when the tag_in matches the i-th tag in the packed tags array 
            end
    endgenerate 

    assign tag_not_found = ~(|set_select); 

endmodule 




module #(parameter DATA_WIDTH, parameter TAG_WIDTH) way( 
    input reset, clk, write_enabled, replace_enabled; 
    output reg valid_bit, dirty_bit; 
    input [DATA_WIDTH-1:0] data_in; 
    input wire [TAG_WIDTH-1:0] tag_in
    output reg [DATA_WIDTH-1:0] data_out; 
    output reg [TAG_WIDTH-1:0] tag_out; 
);

    always @(posedge clk, posedge reset)
    begin 
        if(reset)
            tag_out <= 0; 
            data_out <= 0; 
            valid_bit <= 0; 
        else if (write_enabled)
            data_out <= data_in;
        else if (replace_enabled)
            data_out <= data_in; 
            tag_out <= tag_in;
            valid_bit <= 1;   
        
    end 
    
endmodule