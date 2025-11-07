///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: 4-way set associative cache with write-back, write-allocate policy  
///////////////////////////////////////////////////////////////////////////////

module block_buffer #(
    parameter BLOCK_SIZE = 8,
    parameter WORD_SIZE  = 16
) (
    input  wire                       clk,
    input  wire                       flush_buffer,
    input  wire                       write_request, //signal from the main memory indicating that the data is available
    input  wire [WORD_SIZE-1:0]       word_in,
    output reg  [(BLOCK_SIZE*WORD_SIZE)-1:0] block_out,
    output reg                        buffer_full,
    output reg                        buffer_empty, 
    output reg                        data_ready
);


    reg [$clog2(BLOCK_SIZE)-1:0] pointer;

    // On flush_buffer (async) clear state; on clock collect words into block_out
    always @(posedge clk or posedge flush_buffer) begin
        if (flush_buffer) begin
            block_out    <= {(BLOCK_SIZE*WORD_SIZE){1'b0}};
            pointer      <= {($clog2(BLOCK_SIZE)){1'b0}};
            buffer_empty <= 1'b1;
            buffer_full  <= 1'b0;
        end else begin
            if (!buffer_full & write_request) begin
                block_out[pointer*WORD_SIZE +: WORD_SIZE] <= word_in;
                pointer <= pointer + 1'b1;
                buffer_empty <= 1'b0;

                
                if (pointer == (BLOCK_SIZE - 1)) 
                begin
                    buffer_full <= 1'b1;
                    data_ready <= 1; 
                end
            end
        end
    end

endmodule 


module write_back_buffer #(
    parameter integer BLOCK_SIZE  = 8,   
    parameter integer WORD_SIZE   = 16,  
    parameter integer BUFFER_SIZE = 4,
    parameter integer ADDR_SIZE   = 16,  
    parameter integer INDEX_SIZE  = 8,
    parameter integer TAG_SIZE    = 4
)(
    input  wire  clk,reset,

    // enqueue entire block from cache
    input  wire                         write_request,
    input  wire [BLOCK_SIZE*WORD_SIZE-1:0] block_in,
    input  wire                         block_dirty_in,
    input  wire [INDEX_SIZE-1:0]        block_index_in,
    input  wire [TAG_SIZE-1:0]          block_tag_in,

    output reg  [7:0]                   byte_out,      
    output reg  [ADDR_SIZE-1:0]         addr_out,
    output reg                          evict_valid,
    input  wire                         evict_ready,

    output wire                         buffer_empty,
    output wire                         buffer_full
);

    // pointer widths
    localparam PTR_BITS   = $clog2(BUFFER_SIZE);
    localparam COUNT_BITS = $clog2(BUFFER_SIZE + 1);

    // Storage arrays
    reg [BLOCK_SIZE*WORD_SIZE-1:0] data_buffer [0:BUFFER_SIZE-1];
    reg [INDEX_SIZE-1:0]          index_buffer[0:BUFFER_SIZE-1];
    reg [TAG_SIZE-1:0]            tag_buffer  [0:BUFFER_SIZE-1];
    reg                           dirty_buffer[0:BUFFER_SIZE-1];

    // FIFO control
    reg [PTR_BITS-1:0] head_ptr;
    reg [PTR_BITS-1:0] tail_ptr;
    reg [COUNT_BITS-1:0] count;

    // Draining state
    reg draining;

    // Word and byte pointers
    reg [$clog2(BLOCK_SIZE)-1:0] word_pointer;
    reg                          byte_pointer;   // 1 bit → selects high/low byte

    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            head_ptr     <= 0;
            tail_ptr     <= 0;
            count        <= 0;
            draining     <= 0;
            word_pointer <= 0;
            byte_pointer <= 0;
            evict_valid  <= 0;

            for (i = 0; i < BUFFER_SIZE; i = i + 1) begin
                data_buffer[i]  <= 0;
                index_buffer[i] <= 0;
                tag_buffer[i]   <= 0;
                dirty_buffer[i] <= 0;
            end
        end else begin
            evict_valid <= 0;

            // -------------------------
            // ENQUEUE NEW BLOCK
            // -------------------------
            if (write_request && (count != BUFFER_SIZE)) begin
                data_buffer[head_ptr]  <= block_in;
                index_buffer[head_ptr] <= block_index_in;
                tag_buffer[head_ptr]   <= block_tag_in;
                dirty_buffer[head_ptr] <= block_dirty_in;

                head_ptr <= head_ptr + 1'b1;
                count    <= count + 1'b1;
            end

            // -------------------------
            // START DRAINING IF POSSIBLE
            // -------------------------
            if (!draining && ~buffer_empty) begin
                draining     <= 1'b1;
                word_pointer <= 0;
                byte_pointer <= 0;
            end

            //draining logic 
            if (draining) begin
                reg [15:0] selected_word;
                selected_word = data_buffer[tail_ptr][ (word_pointer*WORD_SIZE) +: WORD_SIZE ];

                byte_out <= (byte_pointer == 1'b0)
                            ? selected_word[7:0]    // low byte// high byte
                            : selected_word[15:8];  // high byte

                // Address = {tag, index, word_pointer, byte_pointer}
                addr_out <= {
                    tag_buffer[tail_ptr],
                    index_buffer[tail_ptr],
                    word_pointer,
                    byte_pointer
                };

                evict_valid <= 1'b1;

                if (evict_ready) begin //memory is ready to recieve another byte 
                    // -------------------------
                    // ADVANCE POINTERS
                    // -------------------------
                    if (byte_pointer == 1'b0) 
                        begin
                            byte_pointer <= 1'b1;  // next byte in same word
                        end 
                    else 
                        begin
                            // finished both bytes in this word
                            byte_pointer <= 1'b0;
                            if (word_pointer == (BLOCK_SIZE - 1))
                                begin
                                    word_pointer <= 0;
                                    draining     <= 1'b0;
                                    tail_ptr     <= tail_ptr + 1'b1;
                                    count        <= count - 1'b1;
                                end 
                            else 
                                begin
                                    word_pointer <= word_pointer + 1'b1;
                                end
                        end
                end
            end
        end
    end
endmodule




module cache_controller #( 
    parameter
            BLOCK_SIZE = 8,  
            WORD_SIZE = 16, 
            ADDR_SIZE = 16, 
            BLOCK_OFFSET_SIZE = 3, 
            WORD_OFFSET_SIZE = 1,  
            TAG_SIZE = 4, 
            SET_INDEX_SIZE = 8, 
            WB_BUFFER_SIZE = 4 
)
(
    input clk, reset, 


    //controller <=> cpu  
    input [ADDR_SIZE-1:0] cpu_addr_in, 
    input cpu_req_type, //read = 0, write = 1 
    input [WORD_SIZE-1:0] cpu_data_in, //write data from CPU
    input cpu_req_valid, //request is valid 
    output [WORD_SIZE-1:0] cpu_data_out,  //read data to cpu
    output cpu_data_ready,  //indicates data going to the cpu is ready


    //controller <=> main store
    input  [WORD_SIZE-1:0] mem_data_in, //read data from memory
    output mem_write_enable, mem_read_enable,  
    output [WORD_SIZE-1:0] mem_data_out

); 
    //controller <=> cache
    wire [BLOCK_OFFSET_SIZE-1:0] block_offset; 
    wire [WORD_OFFSET_SIZE-1:0] word_offset;  
    wire [SET_INDEX_SIZE-1:0] set_index;
    wire [TAG_SIZE-1:0] tag_bits; 

    //controller <=> memory-block-buffer
    wire [(BLOCK_SIZE*WORD_SIZE)-1:0] mem_block_data; 
    reg flush_buffer; 
    wire buffer_full, buffer_empty; 


    //assignments 
    assign block_offset = cpu_addr_in[3:1]; 
    assign word_offset = cpu_addr_in[0]; 
    assign set_index = cpu_addr_in[11:4]; 
    assign tag_bits = cpu_addr_in[15:12]; 


    //module instantiation (buffers and cache)
    block_buffer #(.BLOCK_SIZE(BLOCK_SIZE), .WORD_SIZE(WORD_SIZE))  BUFFER(
        .clk(clk), .flush_buffer(flush_buffer), 
        .word_in(mem_data_in), 
        .block_out(mem_block_data), 
        .buffer_full(buffer_full), 
        .buffer_empty(buffer_empty)
    ); 

    write_back_buffer #(.BLOCK_SIZE(BLOCK_SIZE), .WORD_SIZE(WORD_SIZE), .BUFFER_SIZE(WB_BUFFER_SIZE)) WB_BUFFER(

    ); 

    cache #(.BLOCK_SIZE(BLOCK_SIZE), .INDEX_WIDTH(SET_INDEX_SIZE), .TAG_WIDTH(TAG_SIZE), .DATA_WIDTH(WORD_SIZE), .BLOCK_OFFSET_WIDTH(BLOCK_OFFSET_SIZE), .WORD_OFFSET_WIDTH(WORD_OFFSET_SIZE))
    CACHE(

    ); 

    //control logic 

    always @(posedge clk, posedge reset)
    begin 


    end 

endmodule 




module cache  #(parameter BLOCK_SIZE = 8, INDEX_WIDTH, TAG_WIDTH, DATA_WIDTH, BLOCK_OFFSET_WIDTH, WORD_OFFSET_WIDTH) (
    input clk, reset, write_enabled, read_enabled, replace_request, 
    input [INDEX_WIDTH-1:0] set_index, 
    input [TAG_WIDTH-1:0] input_tag, 
    input [DATA_WIDTH-1:0] write_data_word,
    input [(DATA_WIDTH*BLOCK_SIZE)-1:0] write_data_block, 
    input [WORD_OFFSET_WIDTH-1:0] write_byte_offset, read_byte_offset; 
    output [DATA_WIDTH-1:0] read_data_word,
    output [(DATA_WIDTH*BLOCK_SIZE)-1:0] read_data_block, evicted_data_block, 
    output evict_flag, evict_dirty, evict_tag, data_ready
); 
    
    
    


endmodule


module #(parameter SET_WIDTH, DATA_WIDTH, TAG_WIDTH) set(
    input clk, reset, replace_request, write_enabled 
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
    input reset, clk, write_enabled, replace_request; 
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
            dirty_bit <= 0; 
        else if (write_enabled)
            data_out <= data_in;
            dirty_bit <= 1; 
        else if (replace_request)
            data_out <= data_in; 
            tag_out <= tag_in;
            valid_bit <= 1;
            dirty_bit <= 0;    
        
    end 
    
endmodule