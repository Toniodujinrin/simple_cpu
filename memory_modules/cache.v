///////////////////////////////////////////////////////////////////////////////
// File:        ID_EX_reg.v
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: 4-way set associative cache + controller with write-back, write-allocate policy  
///////////////////////////////////////////////////////////////////////////////


module cache_controller #( 
    parameter
        BLOCK_SIZE = 8,  
        WORD_WIDTH = 16, 
        ADDR_WIDTH = 16, 
        WORD_OFFSET_WIDTH = 3,  
        TAG_WIDTH = 4, 
        SET_INDEX_SIZE = 8, 
        WB_BUFFER_SIZE = 4 
)
(
    
    input clk, reset,  

    //controller <=> cpu  
    input [ADDR_WIDTH-1:0] cpu_addr_in, 
    input cpu_req_read, 
    input cpu_req_write, 
    input cpu_req_ready, //indicates the cpu request data is ready and valid
    input [WORD_WIDTH-1:0] cpu_data_in, //write data from CPU
    output reg [WORD_WIDTH-1:0] cpu_data_out,  //read data to cpu
    output reg cpu_data_ready,  //indicates data going to the cpu is ready
    output reg cpu_stall_req, //requests that the cpu should stall 
    output reg cpu_req_invalid, // request made by the CPU is invalid 

    //controller <=> main store
    input  [7:0] mem_data_in, //read data from memory
    output mem_write_enable, mem_read_enable,  
    output [ADDR_WIDTH-1:0] mem_addr, 
    output [7:0] mem_data_out,  //write data to memory 
    output mem_data_ready, //indicates that data going into the memory is ready. 
    input mem_ack //memory has acknowledged data transfer
); 
    localparam DATA_WIDTH = BLOCK_SIZE*WORD_WIDTH; 
    localparam IDLE_STATE = 4'b0000; 
    localparam COMPARE_TAG_STATE = 4'b0001
    localparam WRITE_ALLOCATE_STATE = 4'b0010; 
    localparam WRITE_BACK_STATE = 4'b0011; 
    


    //controller <=> cache
    reg [WORD_OFFSET_WIDTH-1:0] cache_word_offset;   
    reg [SET_INDEX_SIZE-1:0] cache_set_index;
    reg [TAG_WIDTH-1:0] cache_tag_bits; 
    reg cache_write_enabled; 
    reg [WORD_WIDTH-1:0] cache_write_data_word; 
    reg [DATA_WIDTH-1:0] cache_write_data_block; 
    reg cache_replace_request; 
    wire [DATA_WIDTH-1:0] cache_evict_data
    wire [TAG_WIDTH-1:0] cache_evict_tag; 
    wire cache_evict_dirty, cache_evict_flag; 
    wire [WORD_WIDTH-1:0] cache_read_data_word; 
    wire cache_miss; 
     

    //controller <=> memory-block-buffer
    wire [DATA_WIDTH-1:0] mbb_block_out;
    reg mbb_reset; 
    reg mbb_byte_offset; 
    reg mbb_byte_in; 
    reg mbb_mem_data_ready; 
    wire mbb_buffer_full, mbb_buffer_empty; 
    wire mbb_data_ready; 


    //controller <=> write-back-buffer 
    reg wbb_write_request; 
    reg [DATA_WIDTH-1:0] wbb_block_in; 
    reg wbb_block_dirty; 
    reg [INDEX_WIDTH-1:0] wbb_block_index; 
    reg [TAG_WIDTH-1:0] wbb_tag_in; 
    reg wbb_write_request; 
    wire wbb_buffer_full, wbb_buffer_empty;  
    reg wbb_evict_ready; //indicates that main store is ready to recieve data
    wire wbb_evict_valid; //indicates that the data from the WBB is ready to be put into the main store 
    wire [7:0] wbb_byte_out; 
    wire [ADDR_WIDTH-1:0] wbb_addr_out; 


    //module instantiation (buffers and cache)
    block_buffer #(.BLOCK_SIZE(BLOCK_SIZE), .WORD_WIDTH(WORD_WIDTH))  
    MB_BUFFER(
        .clk(clk), 
        .reset(mbb_reset), 
        .mem_data_ready(mbb_mem_data_ready), 
        .byte_offset(mbb_byte_offset), 
        .byte_in(mbb_byte_in), 
        .block_out(mbb_block_out), 
        .buffer_full(mbb_buffer_full), 
        .buffer_empty(mbb_buffer_empty), 
        .data_ready(mbb_data_ready)
    ); 


    write_back_buffer #(.BLOCK_SIZE(BLOCK_SIZE), .WORD_WIDTH(WORD_WIDTH), .BUFFER_SIZE(WB_BUFFER_SIZE), .ADDR_WIDTH(ADDR_WIDTH), .INDEX_SIZE(SET_INDEX_SIZE), .TAG_WIDTH(TAG_WIDTH)) 
    WB_BUFFER(
        .clk(clk), 
        .reset(reset),
        .write_request(wbb_write_request), 
        .block_in(wbb_block_in), 
        .block_dirty_in(wbb_block_dirty), 
        .block_index_in(wbb_block_index), 
        .block_tag_in(wbb_tag_in), 
        .byte_out(wbb_byte_out), 
        .addr_out(wbb_addr_out), 
        .evict_valid(wbb_evict_valid), 
        .evict_ready(wbb_evict_ready), 
        .buffer_empty(wbb_buffer_empty), 
        .buffer_full(wbb_buffer_full)
    ); 

    cache #(.BLOCK_SIZE(BLOCK_SIZE), .INDEX_WIDTH(SET_INDEX_SIZE), .TAG_WIDTH(TAG_WIDTH), .WORD_WIDTH(WORD_WIDTH), .WORD_OFFSET_WIDTH(WORD_OFFSET_WIDTH))
    CACHE(
        //inputs 
        .clk(clk), 
        .reset(reset), 
        .write_enabled(cache_write_enabled), 
        .replace_request(cache_replace_request), 
        .set_index(cache_set_index), 
        .input_tag(cache_tag_bits), 
        .write_data_word(cache_write_data_word), 
        .write_data_block(cache_write_data_block), 
        .word_offset(cache_word_offset), 
        //outputs 
        .read_data_word(cache_read_data_word), 
        .evicted_data_block(cache_evict_data), 
        .evict_tag(cache_evict_data), 
        .evict_flag(cache_evict_flag), 
        .evict_dirty(cache_evict_dirty), 
        .miss(cache_miss)
    ); 

    //assignments 
    assign word_offset = cpu_addr_in[3:1]; 
    assign byte_offset = cpu_addr_in[0]; 
    assign set_index = cpu_addr_in[11:4]; 
    assign tag_bits = cpu_addr_in[15:12]; 
    

    //control logic 
    reg current_state; 

    always @(posedge clk, posedge reset)
    begin 
        case current_state
            IDLE_STATE: 
                begin 
                    if(cpu_req_ready)
                        begin
                            //pull data ready low
                            cpu_data_ready <= 0; 
                            //send request to the cache
                            cache_tag_bits <= tag_bits; 
                            cache_set_index <= set_index; 
                            cache_word_offset <= word_offset; 
                            cache_replace_request <= 0; 
                            cache_write_data_block <= 0; 
                            if(cpu_req_read)
                                begin 
                                    cache_write_enabled <= 1'b0;
                                    cache_write_data_word <= 0;   
                                    current_state <= COMPARE_TAG_STATE; 
                                end 
                            else if(cpu_req_write)
                                begin 
                                    cache_write_enabled <= 1'b1; 
                                    cache_write_data_word <= cpu_data_in; 
                                    current_state <= COMPARE_TAG_STATE; 
                                end
                            else if(cpu_req_read & cpu_req_write) //invalid state
                                begin 
                                    current_state <= IDLE_STATE; 
                                    cpu_req_invalid <= 1'b1; 
                                end 
                            //if both cpu_req_read and cpu_req_write are both low, this indicates a no-op, cache remains idle 
                        end 
                end

            COMPARE_TAG_STATE:
                begin
                    //handle cache miss
                    if(cache_miss)
                        begin 
                            cpu_stall_req <= 1'b1; 
                            cpu_data_ready <= 1'b0; 
                            //handle write miss 
                            if(cpu_req_write)
                                begin 
                                    //issue a replace request to the cache
                                    cache_replace_request <= 1'b1;
                                    current_state <= WRITE_BACK_STATE; 
                                end 
                            else //handle read miss
                                //request data from memory 
                                current_state <= WRITE_ALLOCATE_STATE;
                        end
                    else //handle cache hit 
                        begin 
                            cpu_data_out <= cache_read_data_word; 
                            cpu_data_ready <= 1'b1; 
                            cpu_stall_req <= 1'b0; 
                            cpu_req_invalid <= 1'b0; 
                            current_state <= IDLE_STATE; 
                        end 
                end   

            WRITE_ALLOCATE_STATE: 
                begin 

                end 

            WRITE_BACK_STATE: 
                begin 

                end
        endcase
    end 


    always @(posedge reset)
    begin 
        if(reset)
            begin
                current_state <= IDLE_STATE;  
            end 
    end 
endmodule


    
module memory_arbiter(); 

endmodule

module block_buffer #(
    parameter BLOCK_SIZE = 8,       // number of 16-bit words
    parameter WORD_WIDTH = 16
)(
    input  wire clk,
    input  wire reset,
    input  wire mem_data_ready,     // byte is available
    input  wire byte_offset,        // 0 = low byte, 1 = high byte
    input  wire [7:0] byte_in,      // incoming byte
    output reg  [BLOCK_SIZE*WORD_WIDTH-1:0] block_out,
    output reg  buffer_full,
    output reg  buffer_empty,
    output reg  data_ready //block is ready for the cache 
);
    reg [$clog2(BLOCK_SIZE)-1:0] word_ptr;  
    reg byte_ptr; 

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            block_out    <= {(BLOCK_SIZE*WORD_WIDTH){1'b0}};
            word_ptr     <= 0;
            byte_ptr     <= 0;
            buffer_empty <= 1;
            buffer_full  <= 0;
            data_ready   <= 0;
        end else begin
            data_ready <= 0;

            if (!buffer_full && mem_data_ready) begin
                integer base;
                base = word_ptr * WORD_WIDTH;

                // Write the byte into the correct half of the word
                if (byte_offset == 1'b0) begin
                    block_out[base + 7 : base + 0] <= byte_in;
                end else begin
                    block_out[base + 15 : base + 8] <= byte_in;
                end

                // Update byte pointer
                if (byte_ptr == 1'b0) begin
                    byte_ptr <= 1'b1;
                end else begin
                    byte_ptr <= 1'b0;
                    word_ptr <= word_ptr + 1'b1;

                    if (word_ptr == BLOCK_SIZE-1) begin
                        buffer_full <= 1'b1;
                        data_ready  <= 1'b1;
                    end
                    buffer_empty <= 1'b0;
                end
            end
        end
    end
endmodule

module write_back_buffer #(
    parameter integer BLOCK_SIZE  = 8,   
    parameter integer WORD_WIDTH  = 16,  
    parameter integer BUFFER_SIZE = 4,
    parameter integer ADDR_WIDTH   = 16,  
    parameter integer INDEX_SIZE  = 8,
    parameter integer TAG_WIDTH    = 4
)(
    input  wire  clk,reset,

    // enqueue entire block from cache
    input  wire write_request,
    input  wire [BLOCK_SIZE*WORD_WIDTH-1:0] block_in,
    input  wire block_dirty_in,
    input  wire [INDEX_SIZE-1:0] block_index_in,
    input  wire [TAG_WIDTH-1:0] block_tag_in,
    input  wire evict_ready,

    output reg  [7:0] byte_out,      
    output reg  [ADDR_WIDTH-1:0] addr_out,
    output reg  evict_valid, //used to indicate to the RAM that the evicted data is ready to be written to the 
    output wire buffer_empty,
    output wire buffer_full
);

    // pointer widths
    localparam PTR_BITS   = $clog2(BUFFER_SIZE);
    localparam COUNT_BITS = $clog2(BUFFER_SIZE + 1);

    // Storage arrays
    reg [BLOCK_SIZE*WORD_WIDTH-1:0] data_buffer [0:BUFFER_SIZE-1];
    reg [INDEX_SIZE-1:0]          index_buffer[0:BUFFER_SIZE-1];
    reg [TAG_WIDTH-1:0]            tag_buffer  [0:BUFFER_SIZE-1];
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
            if (write_request && (count != BUFFER_SIZE) && block_dirty_in) begin
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
                selected_word = data_buffer[tail_ptr][ (word_pointer*WORD_WIDTH) +: WORD_WIDTH ];

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

                if (evict_ready) begin //memory has acknowledged byte and is ready to recieve another byte 
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





module cache  #(parameter BLOCK_SIZE = 8, INDEX_WIDTH = 8, TAG_WIDTH =4, WORD_WIDTH =16, WORD_OFFSET_WIDTH = 3) (
    input clk, reset, write_enabled, replace_request, 
    input [INDEX_WIDTH-1:0] set_index, 
    input [TAG_WIDTH-1:0] input_tag, 
    input [WORD_WIDTH-1:0] write_data_word,
    input [(WORD_WIDTH*BLOCK_SIZE)-1:0] write_data_block, 
    input [WORD_OFFSET_WIDTH-1:0] word_offset, //selects particular word in block 
    output [WORD_WIDTH-1:0] read_data_word,
    output [(WORD_WIDTH*BLOCK_SIZE)-1:0] evicted_data_block, 
    output [TAG_WIDTH-1:0] evict_tag, 
    output evict_flag, evict_dirty, miss
); 
    localparam DATA_WIDTH = WORD_WIDTH*BLOCK_SIZE; 
    localparam SET_N = 2**INDEX_WIDTH;
    localparam SET_WIDTH = 4; //number of ways in set

    wire [(SET_N*DATA_WIDTH)-1:0] read_data_block_set; //data from all the blocks into one packed array 
    wire [(SET_N*DATA_WIDTH)-1:0] evicted_data_block_set;
    wire [(SET_N*TAG_WIDTH)-1:0] evicted_tag_set;
    wire [SET_N-1:0] evict_flag_set; 
    wire [SET_N-1:0] evict_dirty_set; 
    wire [SET_N-1:0] miss_set; 
    wire [SET_N-1:0] set_select; 
    wire [DATA_WIDTH-1:0] read_data_block; 

    //output data muxes
    reg_mux #(.REG_N(SET_N), .WIDTH(DATA_WIDTH))  READ_DATA_MUX(.in(read_data_block_set), .select(set_index), .out(read_data_block));
    reg_mux #(.REG_N(SET_N), .WIDTH(DATA_WIDTH))  EVICTED_DATA_MUX(.in(evicted_data_block_set), .select(set_index), .out(evicted_data_block)); 
    reg_mux #(.REG_N(SET_N), .WIDTH(TAG_WIDTH))  EVICTED_TAG_MUX(.in(evicted_tag_set), .select(set_index), .out(evict_tag)); 
	register_address_decoder #(.INPUT_WIDTH(INDEX_WIDTH)) (.in(set_index),.out(set_select));  
    
    
    //assignments
    assign read_data_word = read_data_block[word_offset*WORD_WIDTH+:WORD_WIDTH]; 
    assign miss = |miss_set; 
    assign evict_flag = |evict_flag_set; 
    assign evict_dirty = |evict_dirty_set; 

    generate 
        genvar i; 

        for(i = 0; i < SET_N; i = i+1)
            begin 
                set #(.SET_WIDTH(SET_WIDTH), .DATA_WIDTH(DATA_WIDTH), .TAG_WIDTH(TAG_WIDTH), .WORD_WIDTH(WORD_WIDTH), WORD_OFFSET_WIDTH(WORD_OFFSET_WIDTH)) SET(
                    .clk(clk), 
                    .reset(reset), 
                    .replace_request(replace_request), 
                    .write_enabled(write_enabled), 
                    .set_selected(set_select[i]),
                    .word_offset(word_offset), 
                    .word_in(write_data_word),  
                    .tag_in(input_tag), 
                    .data_in(write_data_block), 
                    .data(read_data_block_set[i*DATA_WIDTH+:DATA_WIDTH]), 
                    .evict_data(evicted_data_block_set[i*DATA_WIDTH+:DATA_WIDTH]), 
                    .evict_tag(evicted_tag_set[i*TAG_WIDTH+:TAG_WIDTH]), 
                    .evict_flag(evict_flag_set[i]),
                    .evict_dirty(evict_dirty_set[i]), 
                    .miss(miss_set[i])
                ); 
            end 
    endgenerate 

endmodule


module set #(
    parameter SET_WIDTH        = 4,
    parameter DATA_WIDTH       = 128,
    parameter TAG_WIDTH        = 4,
    parameter WORD_WIDTH       = 16,
    parameter WORD_OFFSET_WIDTH= 3
) (
    input  wire clk, reset, replace_request, write_enabled, set_selected,
    input  wire [TAG_WIDTH-1:0] tag_in,
    input  wire [DATA_WIDTH-1:0] data_in,     // block
    input  wire [WORD_WIDTH-1:0] word_in,     // word
    input  wire [WORD_OFFSET_WIDTH-1:0] word_offset,
    output reg [DATA_WIDTH-1:0] data, evict_data,
    output reg [TAG_WIDTH-1:0] evict_tag,
    output reg evict_dirty,evict_flag,
    output wire miss
);

    localparam LRU_COUNT_SIZE = $clog2(SET_WIDTH);

    // per-way arrays 
    wire [DATA_WIDTH-1:0]set_data      [0:SET_WIDTH-1];
    wire set_valid_bits [0:SET_WIDTH-1];
    wire set_evict_dirty_bits [0:SET_WIDTH-1];
    wire set_evict_flag [0:SET_WIDTH-1];
    wire [TAG_WIDTH-1:0] set_tag_bits   [0:SET_WIDTH-1];
    wire [TAG_WIDTH-1:0] set_evict_tag  [0:SET_WIDTH-1];
    wire [DATA_WIDTH-1:0] set_evict_data[0:SET_WIDTH-1];
    wire [LRU_COUNT_SIZE-1:0] lru_count [0:SET_WIDTH-1];
    wire [SET_WIDTH-1:0] way_select_one_hot;
    reg  [LRU_COUNT_SIZE-1:0] lru_way;    
    wire tag_not_found;



    assign miss = set_selected & tag_not_found;


    way_selector #(.SET_WIDTH(SET_WIDTH), .TAG_WIDTH(TAG_WIDTH)) selector_i (
        .tag_in(tag_in),
        .tags(set_tag_bits),
        .valids(set_valid_bits),
        .way_select(way_select_one_hot),
        .tag_not_found(tag_not_found)
    );

    //lru update logic (specific to 4 ways) 
    always @(*) 
    begin 
        if ((lru_count[0] < lru_count[1]) && 
            (lru_count[0] < lru_count[2]) && 
            (lru_count[0] < lru_count[3]) ) 
            lru_way = 2'b00; 
        else if( lru_count[1] < lru_count[2] && 
                lru_count[1] < lru_count[3] ) 
            lru_way = 2'b01; 
        else if( lru_count[2] < lru_count[3] ) 
            lru_way = 2'b10; 
        else 
            lru_way = 2'b11; 
    end

    // --- combinational muxes: select current data/evict_data/tag from the per-way arrays
    always @(*) begin
        data = {DATA_WIDTH{1'b0}};
        evict_data = {DATA_WIDTH{1'b0}};
        evict_tag  = {TAG_WIDTH{1'b0}};
        evict_dirty = 1'b0;
        evict_flag = 1'b0;
        integer w; 
        for (w = 0; w < SET_WIDTH; w = w + 1) begin
            if (way_select_one_hot[w]) begin
                data = set_data[w];
            end
            
            if (w == lru_way) begin
                evict_data  = set_evict_data[w];
                evict_tag   = set_evict_tag[w];
                evict_dirty = set_evict_dirty_bits[w];
                evict_flag  = set_evict_flag[w];
            end
        end
    end


    genvar i;
    generate
        for (i = 0; i < SET_WIDTH; i = i + 1) begin : way_instantiation
            way #(
                .DATA_WIDTH(DATA_WIDTH),
                .TAG_WIDTH(TAG_WIDTH),
                .SET_WIDTH(SET_WIDTH),
                .LRU_COUNT_SIZE(LRU_COUNT_SIZE)
            ) WAY_N (
                .clk(clk),
                .reset(reset),
                .write_enabled(write_enabled),
                .replace_request(replace_request),
                .way_selected( set_selected & ( replace_request ? (lru_way == i) : way_select_one_hot[i] ) ),
                .data_in(data_in),
                .word_in(word_in),
                .word_offset(word_offset),
                .tag_in(tag_in),
                .valid_bit(set_valid_bits[i]),
                .evict_dirty(set_evict_dirty_bits[i]),
                .data(set_data[i]),
                .evict_data(set_evict_data[i]),
                .evict_flag(set_evict_flag[i]),
                .tag(set_tag_bits[i]),
                .evict_tag(set_evict_tag[i]),
                .lru_count(lru_count[i])
            );
        end
    endgenerate

endmodule


module way_selector #(
    parameter SET_WIDTH = 4,
    parameter TAG_WIDTH = 4
) (
    input  wire [TAG_WIDTH-1:0] tag_in,
    input  wire [TAG_WIDTH-1:0] tags [0:SET_WIDTH-1],
    input  wire valids [0:SET_WIDTH-1],
    output wire [SET_WIDTH-1:0] way_select,    // one-hot
    output wire tag_not_found
);

    genvar i;
    generate
        for (i = 0; i < SET_WIDTH; i = i + 1) 
        begin : tag_match
            assign way_select[i] = valids[i] & (tag_in == tags[i]);
        end
    endgenerate

    assign tag_not_found = ~(|way_select);

endmodule



module way #(parameter DATA_WIDTH = 128, TAG_WIDTH = 4, SET_WIDTH = 4, LRU_COUNT_SIZE = 2, WORD_OFFSET_WIDTH = 3, WORD_WIDTH = 16) ( 
    input reset, clk, write_enabled, replace_request, way_selected, 
    output reg valid_bit, evict_dirty, evict_flag, 
    input [DATA_WIDTH-1:0] data_in,
    input [WORD_WIDTH-1:0] word_in, 
    input [WORD_OFFSET_WIDTH-1:0] word_offset,
    input wire [TAG_WIDTH-1:0] tag_in, 
    output reg [DATA_WIDTH-1:0] data, evict_data,
    output reg [TAG_WIDTH-1:0] tag, evict_tag,
    output wire [LRU_COUNT_SIZE-1:0] lru_count
); 
    
    reg dirty_bit; 
    //Counter
    lru_counter #(.COUNTER_WIDTH(LRU_COUNT_SIZE)) LRU(
        .clk(clk), 
        .reset(reset), 
        .enabled(way_selected), 
        .sync_reset(replace_request & way_selected),
        .count(lru_count)
    ); 
    //IO policy 
    always @(posedge clk or posedge reset) begin
        if (reset) 
            begin
                tag       <= {TAG_WIDTH{1'b0}};
                data      <= {DATA_WIDTH{1'b0}};
                valid_bit <= 1'b0;
                dirty_bit <= 1'b0;
                evict_flag<= 1'b0;
                evict_dirty<= 1'b0;
                evict_data <= {DATA_WIDTH{1'b0}};
                evict_tag  <= {TAG_WIDTH{1'b0}};
            end 
        else 
            begin
                evict_flag <= 1'b0;
                evict_dirty <= 1'b0;
                evict_data <= evict_data; 
                evict_tag <= evict_tag;

        
                if (replace_request && way_selected) 
                    begin
                        //evict old 
                        evict_data  <= data;
                        evict_tag   <= tag;
                        evict_dirty <= dirty_bit;
                        // install new block
                        data        <= data_in;
                        tag         <= tag_in;
                        valid_bit   <= 1'b1;
                        dirty_bit   <= 1'b0;
                        // evict flag indicates we have evicted a valid line
                        evict_flag  <= valid_bit ? 1'b1 : 1'b0;
                    end 
                else if (write_enabled && way_selected) 
                    begin
                        // Write to specific word within block
                        data[word_offset*WORD_WIDTH +: WORD_WIDTH] <= word_in;
                        dirty_bit <= 1'b1;
                    end
        end
    end
endmodule


module lru_counter #(parameter COUNTER_WIDTH = 2)(
    input clk, reset, enabled, sync_reset, 
    output reg [COUNTER_WIDTH-1:0] count
)
    always @(posedge clk, posedge reset)
    begin 
        if(reset)
            count <= {COUNTER_WIDTH{1'b0}};
        else if (sync_reset)
            count <= {COUNTER_WIDTH{1'b0}};
        else if (enabled)
            count <= count+1;  
    end 

endmodule 