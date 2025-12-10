///////////////////////////////////////////////////////////////////////////////
// File:        cache.v
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: 4-way set associative cache + controller with write-back, write-allocate policy  
///////////////////////////////////////////////////////////////////////////////


//@Todo: remove the early replace in the COMPARE_TAG state in cache controller
//modify the cache base to allow combinational read of the evicted data using the LRU  


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
    input  [7:0] mem_data_read, //read data from memory
    output mem_write_enable, mem_read_enable,  
    output [ADDR_WIDTH-1:0] mem_addr, 
    output [7:0] mem_data_write,  //write data to memory 
    output mem_req_valid, //indicates that data going into the memory is ready. 
    input mem_ack //memory has acknowledged data transfer
); 
    ////////////////////////////////////////////
    //CONTANTS
    ///////////////////////////////////////////
    localparam DATA_WIDTH = BLOCK_SIZE*WORD_WIDTH; 
    localparam IDLE_STATE = 4'b0000; 
    localparam COMPARE_TAG_STATE = 4'b0001;
    localparam WRITE_ALLOCATE_STATE = 4'b0010; 
    localparam WRITE_BACK_STATE = 4'b0011; 

    /////////////////////////////////////////////
    //CONTROLER SIGNALS
    ////////////////////////////////////////////
    wire [WORD_OFFSET_WIDTH-1:0] word_offset; 
    wire byte_offset; 
    wire [SET_INDEX_SIZE-1:0] set_index; 
    wire [TAG_WIDTH-1:0] tag_bits; 
    reg evict_hold_valid;
    reg [DATA_WIDTH-1:0]   evict_hold_data;
    reg [TAG_WIDTH-1:0]    evict_hold_tag;
    reg [SET_INDEX_SIZE-1:0] evict_hold_index;

    ///////////////////////////////////////
    //CACHE SIGNALS
    //////////////////////////////////////
    //controller <=> cache
    reg [WORD_OFFSET_WIDTH-1:0] cache_word_offset;   
    reg [SET_INDEX_SIZE-1:0] cache_set_index;
    reg [TAG_WIDTH-1:0] cache_tag_bits; 
    reg cache_write_enabled; 
    reg [WORD_WIDTH-1:0] cache_write_data_word; 
    reg [DATA_WIDTH-1:0] cache_write_data_block; 
    reg cache_replace_request; 
    wire [DATA_WIDTH-1:0] cache_evict_data;
    wire [TAG_WIDTH-1:0] cache_evict_tag; 
    wire cache_evict_dirty; 
    wire [WORD_WIDTH-1:0] cache_read_data_word; 
    wire cache_miss; 
     
    ///////////////////////////////////////
    //MBB SIGNALS
    //////////////////////////////////////
        //controller <=> memory-block-buffer
        wire [DATA_WIDTH-1:0] mbb_block_out;
        reg [ADDR_WIDTH-1:0] mbb_block_addr; 
        reg mbb_write_alloc_req;
        wire mbb_data_ready; 
        //memory-block-buffer <=> arbiter 
        wire mbb_data_req; 
        wire [ADDR_WIDTH-1:0] mbb_data_addr; 
        wire mbb_grant; 


    /////////////////////////////////////
    //ARBITER SIGNALS
    ////////////////////////////////////
        wire [7:0] arbiter_mem_data_out; 
        wire arbiter_mem_ack; 
        

    ///////////////////////////////////
    //WBB SIGNALS
    //////////////////////////////////
        //controller <=> write-back-buffer 
        reg [DATA_WIDTH-1:0] wbb_block_in; 
        reg [SET_INDEX_SIZE-1:0] wbb_block_index; 
        reg [TAG_WIDTH-1:0] wbb_tag_in; 
        reg wbb_write_request; 
        wire wbb_buffer_full, wbb_buffer_empty;   
        //wbb <=> arbiter 
        wire wbb_grant; 
        wire wbb_data_req; 
        wire [7:0] wbb_data; 
        wire [ADDR_WIDTH-1:0] wbb_data_addr; 


    //module instantiation (buffers and cache)
    block_buffer #(
        .BLOCK_SIZE(BLOCK_SIZE), 
        .WORD_WIDTH(WORD_WIDTH), 
        .ADDR_WIDTH(ADDR_WIDTH)
    )  
    MB_BUFFER(
        .clk(clk), 
        .reset(reset), 
        .mem_data_ack(arbiter_mem_ack),   
        .mem_access_grant(mbb_grant),   
        .byte_in(arbiter_mem_data_out),     
        .wrt_alloc_req(mbb_write_alloc_req),      
        .block_address(mbb_block_addr),
        .mem_data_req(mbb_data_req),        // request a memory read to memory arbiter
        .mem_data_addr(mbb_data_addr), // data request address 
        .block_out(mbb_block_out),
        .block_data_ready(mbb_data_ready)
    ); 


    write_back_buffer #(
        .BLOCK_SIZE(BLOCK_SIZE), 
        .WORD_WIDTH(WORD_WIDTH), 
        .BUFFER_SIZE(WB_BUFFER_SIZE), 
        .ADDR_WIDTH(ADDR_WIDTH), 
        .INDEX_SIZE(SET_INDEX_SIZE), 
        .TAG_WIDTH(TAG_WIDTH)
    ) 
    WB_BUFFER(
        .clk(clk), 
        .reset(reset),
        .write_request(wbb_write_request), 
        .block_in(wbb_block_in), 
        .block_index_in(wbb_block_index), 
        .block_tag_in(wbb_tag_in), 
        .byte_out(wbb_data), 
        .addr_out(wbb_data_addr), 
        .mem_data_req(wbb_data_req),
        .mem_access_grant(wbb_grant), 
        .mem_data_ack(arbiter_mem_ack), 
        .buffer_empty(wbb_buffer_empty), 
        .buffer_full(wbb_buffer_full)
    ); 
    
    memory_arbiter #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) 
    ARBITER(
        .clk(clk),
        .reset(reset),
        .mbb_req(mbb_data_req),
        .wbb_req(wbb_data_req),
        .mbb_addr(mbb_data_addr),
        .wbb_addr(wbb_data_addr),
        .wbb_data(wbb_data),
        .mem_data_in(mem_data_read),
        .mem_ack_in(mem_ack),
        .mem_addr(mem_addr),
        .mem_data_write(mem_data_write),
        .mem_write_enable(mem_write_enable),
        .mem_read_enable(mem_read_enable),
        .mem_req_valid(mem_req_valid),
        .mem_data_out(arbiter_mem_data_out),
        .mem_ack_out(arbiter_mem_ack),
        .mbb_grant(mbb_grant),
        .wbb_grant(wbb_grant)
    ); 

    cache #(
        .BLOCK_SIZE(BLOCK_SIZE), 
        .INDEX_WIDTH(SET_INDEX_SIZE), 
        .TAG_WIDTH(TAG_WIDTH), 
        .WORD_WIDTH(WORD_WIDTH), 
        .WORD_OFFSET_WIDTH(WORD_OFFSET_WIDTH)
    )
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
        .evict_tag(cache_evict_tag), 
        .evict_dirty(cache_evict_dirty), 
        .miss(cache_miss)
    ); 

    //assignments 
    assign word_offset = cpu_addr_in[3:1]; 
    assign byte_offset = cpu_addr_in[0]; 
    assign set_index = cpu_addr_in[11:4]; 
    assign tag_bits = cpu_addr_in[15:12]; 
    

    //control logic 
    reg [3:0] current_state; 

    always @(posedge clk, posedge reset)
    begin 
        cache_write_enabled   <= 0;
        cache_replace_request <= 0;
        mbb_write_alloc_req   <= 0;
        wbb_write_request     <= 0;
        cpu_data_ready        <= 0;
        cpu_req_invalid       <= 0;
        if(reset)
            begin
                //reset state 
                current_state <= IDLE_STATE;  
                //reset registers
                cache_replace_request <= 1'b0; 
                cache_write_data_block <= 1'b0; 
                cache_write_data_word <= 1'b0;
                cache_write_enabled <= 1'b0; 
                mbb_write_alloc_req <= 1'b0; 
                cpu_stall_req <= 1'b0; 
                cpu_req_invalid <= 1'b0; 
                cpu_data_ready <= 1'b0; 
                wbb_write_request <= 1'b0; 
                wbb_block_in <= '0;
                wbb_tag_in <= '0; 
                wbb_block_index <= '0; 
                mbb_block_addr <= '0; 
                evict_hold_valid <= 1'b0; 
                evict_hold_data <= '0; 
                evict_hold_tag <= '0; 
                evict_hold_index <= '0; 
            end 
        else
            begin 
                case(current_state)
                    IDLE_STATE: 
                        begin 
                            if(cpu_req_ready)
                                begin
                                    //reset registers
                                    cpu_data_ready <= 1'b0;  
                                    //send request to the cache
                                    cache_tag_bits <= tag_bits; 
                                    cache_set_index <= set_index; 
                                    cache_word_offset <= word_offset; 
                                    current_state <= COMPARE_TAG_STATE; 
                                end 
                        end

                    COMPARE_TAG_STATE:
                        begin
                            if(cache_miss)
                                begin 
                                    cpu_stall_req <= 1'b1;  
                                    current_state <= WRITE_BACK_STATE; 
                                end
                            else
                                begin 
                                    if(cpu_req_read & cpu_req_write) //invalid state
                                    begin 
                                        cpu_req_invalid <= 1'b1; 
                                        cache_write_enabled <= 1'b0; 
                                    end 
                                    else if(cpu_req_read)
                                        begin 
                                            cpu_req_invalid <= 1'b0; 
                                            cache_write_enabled <= 1'b0; 
                                            cpu_data_out <= cache_read_data_word; 
                                        end 
                                    else if(cpu_req_write)
                                        begin 
                                            cache_write_enabled <= 1'b1; 
                                            cache_write_data_word <= cpu_data_in; 
                                            cpu_req_invalid <= 1'b0; 
                                        end
                                 
                                    cpu_data_ready <= 1'b1; 
                                    cpu_stall_req <= 1'b0; 
                                    cache_replace_request <= 1'b0; 
                                    current_state <= IDLE_STATE; 
                                end 
                        end   

                    WRITE_ALLOCATE_STATE: 
                        begin 
                            if(mbb_data_ready)
                                begin 
                                    mbb_write_alloc_req <= 1'b0; 
                                    cache_replace_request <= 1'b1; 
                                    cache_write_data_block <= mbb_block_out; 
                                    cache_tag_bits <= tag_bits; 
                                    current_state <= COMPARE_TAG_STATE; 
                                end 
                            else
                                begin 
                                    mbb_write_alloc_req <= 1'b1; 
                                    mbb_block_addr <= {tag_bits, set_index,{WORD_OFFSET_WIDTH{1'b0}},1'b0}; 
                                end
                        end 

                    WRITE_BACK_STATE: 
                        begin
                            if (!evict_hold_valid) //no evicted data in register 
                                begin
                                    if (cache_evict_dirty) 
                                        begin
                                            evict_hold_valid <= 1'b1;
                                            evict_hold_data  <= cache_evict_data;
                                            evict_hold_tag   <= cache_evict_tag;
                                            evict_hold_index <= cache_set_index;
                                            current_state <= WRITE_BACK_STATE;
                                        end
                                    else 
                                        begin
                                            current_state <= WRITE_ALLOCATE_STATE;
                                        end
                                end
                            else begin
                                if (!wbb_buffer_full) 
                                    begin
                                        wbb_block_in     <= evict_hold_data;
                                        wbb_tag_in       <= evict_hold_tag;
                                        wbb_block_index  <= evict_hold_index;
                                        wbb_write_request<= 1'b1;

                                        // Clear hold slot
                                        evict_hold_valid <= 1'b0;

                                        // Proceed to allocate
                                        current_state <= WRITE_ALLOCATE_STATE;
                                    end
                                else 
                                    begin
                                        current_state <= WRITE_BACK_STATE; //if wbb full stay in state 
                                    end
                            end
                        end

                    default: current_state <= IDLE_STATE; 
                endcase
            end 
    end  
endmodule


//arbiter circuit to manage contention of memory between WBB and MBB 
module memory_arbiter #(parameter ADDR_WIDTH = 16)(
    input  wire clk,
    input  wire reset,
    input  wire mbb_req,
    input  wire wbb_req,
    input  wire [ADDR_WIDTH-1:0] mbb_addr,
    input  wire [ADDR_WIDTH-1:0] wbb_addr,
    input  wire [7:0] wbb_data,
    input  wire [7:0] mem_data_in,
    input  wire mem_ack_in,
    
    output wire [ADDR_WIDTH-1:0] mem_addr,
    output wire [7:0] mem_data_write, //data going into the memory 
    output wire mem_write_enable,
    output wire mem_read_enable,
    output wire mem_req_valid,
    output wire [7:0] mem_data_out, //data output from arbiter to either mbb or wbb 
    output wire mem_ack_out,
    output wire mbb_grant,
    output wire wbb_grant
);

    localparam IDLE    = 2'b00;
    localparam GRANT_M = 2'b10; 
    localparam GRANT_W = 2'b01;

    reg [1:0] state, next_state;

    // ARbiter logic
    always @(*) begin
        case(state)
            IDLE: 
                begin
                    if (mbb_req)          
                        next_state = GRANT_M;
                    else if (wbb_req)     
                        next_state = GRANT_W;
                    else                  
                        next_state = IDLE;
                end

            GRANT_M: 
                begin
                    if (!mbb_req) 
                        begin
                            if (wbb_req)      
                                next_state = GRANT_W;
                            else              
                                next_state = IDLE;
                        end 
                    else             
                        next_state = GRANT_M;
                end

            GRANT_W: 
                begin
                    if (!wbb_req) //only update if wbb req is de-asserted 
                        begin
                            if (mbb_req)      
                                next_state = GRANT_M;
                            else              
                                next_state = IDLE;
                        end 
                    else              
                        next_state = GRANT_W;
                end
        endcase
    end

    always @(posedge clk or posedge reset) 
        begin
            if (reset)
                state <= IDLE;
            else
                state <= next_state;
        end

    assign mbb_grant = (state == GRANT_M);
    assign wbb_grant = (state == GRANT_W);

    // Memory outputs
    assign mem_addr = mbb_grant ? mbb_addr : wbb_grant ? wbb_addr : {ADDR_WIDTH{1'b0}};
    assign mem_data_write  = wbb_grant ? wbb_data : 8'b0;
    assign mem_write_enable = wbb_grant;
    assign mem_read_enable  = mbb_grant;
    assign mem_req_valid = mbb_grant ? mbb_req : wbb_grant ? wbb_req : 1'b0;

    //arbiter outputs 
    assign mem_data_out = mem_data_in;
    assign mem_ack_out  = mem_ack_in;
endmodule



module block_buffer #(
    parameter BLOCK_SIZE = 8, 
    parameter WORD_WIDTH = 16,
    parameter ADDR_WIDTH = 16
)(
    input  wire clk,
    input  wire reset,

    // Interface from memory system
    input  wire mem_data_ack,     // memory has acknowledeg request and returned the byte 
    input  wire mem_access_grant,   // arbiter gives permission
    input  wire [7:0] byte_in,      // data coming from memory

    // Interface from cache controller
    input  wire wrt_alloc_req,      // start block fill
    input  wire [ADDR_WIDTH-1:0] block_address,

    // Outputs
    output reg mem_data_req,        // request a memory read
    output reg [ADDR_WIDTH-1:0] mem_data_addr, // data request address 
    output reg [BLOCK_SIZE*WORD_WIDTH-1:0] block_out,
    output reg block_data_ready
);

    // FSM States
    localparam IDLE_STATE = 2'b00;
    localparam STATE_A    = 2'b01;
    localparam STATE_B    = 2'b10;

    reg [1:0] current_state;
    reg [$clog2(BLOCK_SIZE)-1:0] word_ptr;
    reg byte_ptr;

    integer base;  // used for indexing block_out
    always @(posedge clk or posedge reset) 
        begin
            if (reset) 
                begin
                    current_state <= IDLE_STATE;
                    word_ptr <= 0;
                    byte_ptr <= 0;
                    mem_data_req <= 0;
                    mem_data_addr <= 0;
                    block_out <= 0;
                    block_data_ready <= 0;
                end
            else 
                begin
                    case (current_state)
                        IDLE_STATE: 
                            begin
                                mem_data_req     <= 0;
                                block_data_ready <= 0;

                                if (wrt_alloc_req) 
                                    begin
                                        word_ptr      <= 0;
                                        byte_ptr      <= 0;
                                        current_state <= STATE_A;
                                    end
                            end

                        STATE_A: //issue memory request to arbiter
                            begin
                                if (mem_access_grant) 
                                    begin
                                        current_state <= STATE_B;
                                    end
                                else 
                                    begin
                                        mem_data_req  <= 1'b1;
                                        mem_data_addr <= block_address + (word_ptr << 1) + byte_ptr;
                                    end
                            end

                        STATE_B: 
                            begin
                                if (mem_data_ack) 
                                    begin
                                        mem_data_req  <= 1'b0; //de-assert arbiter req after memory has acknowledged
                                        base = word_ptr * WORD_WIDTH;

                                        // Write correct byte of the word
                                        if (byte_ptr == 1'b0)
                                            block_out[base +: 8] <= byte_in;
                                        else
                                            block_out[base+8 +: 8] <= byte_in;

                                        // Update pointers
                                        if (byte_ptr == 1'b0) 
                                            begin
                                                byte_ptr <= 1'b1;
                                            end
                                        else 
                                            begin
                                                byte_ptr <= 1'b0;
                                                word_ptr <= word_ptr + 1;
                                                if (word_ptr == BLOCK_SIZE - 1) 
                                                    begin
                                                        block_data_ready <= 1'b1;
                                                        current_state    <= IDLE_STATE;
                                                    end
                                                else 
                                                    begin
                                                        current_state <= STATE_A;
                                                    end
                                            end
                                    end 
                            end
                    endcase
                end
        end
endmodule


module write_back_buffer #(
    parameter integer BLOCK_SIZE  = 8,
    parameter integer WORD_WIDTH  = 16,
    parameter integer BUFFER_SIZE = 4,
    parameter integer ADDR_WIDTH  = 16,
    parameter integer INDEX_SIZE  = 8,
    parameter integer TAG_WIDTH   = 4
)(
    input  wire clk, reset,
    input  wire write_request,
    input  wire [BLOCK_SIZE*WORD_WIDTH-1:0] block_in,
    input  wire [INDEX_SIZE-1:0] block_index_in,
    input  wire [TAG_WIDTH-1:0] block_tag_in,
    input  wire mem_access_grant,
    input  wire mem_data_ack,

    // Outputs
    output reg  [7:0] byte_out,
    output reg  [ADDR_WIDTH-1:0] addr_out,
    output reg  mem_data_req, 
    output reg buffer_empty,
    output reg buffer_full
);

    // FIFO pointer widths
    localparam PTR_BITS   = $clog2(BUFFER_SIZE);
    localparam COUNT_BITS = $clog2(BUFFER_SIZE+1);

    // BUffers
    reg [BLOCK_SIZE*WORD_WIDTH-1:0] data_buffer [0:BUFFER_SIZE-1];
    reg [INDEX_SIZE-1:0] index_buffer[0:BUFFER_SIZE-1];
    reg [TAG_WIDTH-1:0]  tag_buffer  [0:BUFFER_SIZE-1];

    // FIFO controls 
    reg [PTR_BITS-1:0] head_ptr;
    reg [PTR_BITS-1:0] tail_ptr;
    reg [COUNT_BITS-1:0] count;


    reg draining;
    reg [$clog2(BLOCK_SIZE)-1:0] word_pointer;
    reg byte_pointer;
    reg [WORD_WIDTH-1:0] selected_word;


    integer i;
    always @(posedge clk or posedge reset) 
        begin
            if (reset) 
                begin
                    head_ptr       <= 0;
                    tail_ptr       <= 0;
                    count          <= 0;
                    draining       <= 0;
                    word_pointer   <= 0;
                    byte_pointer   <= 0;
                    mem_data_req   <= 0;

                    for (i = 0; i < BUFFER_SIZE; i = i + 1) 
                        begin
                            data_buffer[i]  <= 0;
                            index_buffer[i] <= 0;
                            tag_buffer[i]   <= 0;
                        end
                end
            else begin
                // -------------------------
                // ENQUEUE NEW BLOCK
                // -------------------------
                if (write_request && ~buffer_full) 
                    begin
                        data_buffer[head_ptr]  <= block_in;
                        index_buffer[head_ptr] <= block_index_in;
                        tag_buffer[head_ptr]   <= block_tag_in;

                        head_ptr <= head_ptr + 1'b1;
                        count    <= count + 1'b1;
                    end

                // -------------------------
                // START DRAINING DIRTY BLOCK
                // -------------------------
                if (!draining && (count > 0)) 
                    begin
                        draining     <= 1'b1;
                        word_pointer <= 0;
                        byte_pointer <= 0;
                    end

                // -------------------------
                // DRAINING LOGIC
                // -------------------------
                if (draining) 
                    begin
                        selected_word = data_buffer[tail_ptr][word_pointer*WORD_WIDTH +: WORD_WIDTH];

                        byte_out <= (byte_pointer == 1'b0)
                                    ? selected_word[7:0]
                                    : selected_word[15:8];

                        addr_out <= {
                            tag_buffer[tail_ptr],
                            index_buffer[tail_ptr],
                            word_pointer,
                            byte_pointer
                        };


                        // Asser memory request 
                        if (!mem_access_grant)
                            mem_data_req <= 1'b1; 
                        
                            

                        // memory accepted this byte
                        if (mem_access_grant && mem_data_ack) 
                            begin
                                mem_data_req <= 1'b0;
                                if (byte_pointer == 0)
                                    byte_pointer <= 1;
                                else 
                                    begin
                                        byte_pointer <= 0;
                                        if (word_pointer == BLOCK_SIZE-1) 
                                            begin
                                                draining <= 0;
                                                tail_ptr <= tail_ptr + 1'b1;
                                                count <= count - 1'b1;
                                            end
                                        else
                                            word_pointer <= word_pointer + 1'b1;
                                    end
                            end
                    end
            end
        end
    always @(*) begin
        buffer_empty = (count == 0);
        buffer_full  = (count == BUFFER_SIZE);
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
    output evict_dirty, miss
); 
    localparam DATA_WIDTH = WORD_WIDTH*BLOCK_SIZE; 
    localparam SET_N = 2**INDEX_WIDTH;
    localparam SET_WIDTH = 4; //number of ways in set

    wire [(SET_N*DATA_WIDTH)-1:0] read_data_block_set; //data from all the blocks into one packed array 
    wire [(SET_N*DATA_WIDTH)-1:0] evicted_data_block_set;
    wire [(SET_N*TAG_WIDTH)-1:0] evicted_tag_set;
    wire [SET_N-1:0] evict_dirty_set; 
    wire [SET_N-1:0] miss_set; 
    wire [SET_N-1:0] set_select; 
    wire [DATA_WIDTH-1:0] read_data_block; 

    //output data muxes
    reg_mux #(.REG_N(SET_N), .WIDTH(DATA_WIDTH))  READ_DATA_MUX(.in(read_data_block_set), .select(set_index), .out(read_data_block));
    reg_mux #(.REG_N(SET_N), .WIDTH(DATA_WIDTH))  EVICTED_DATA_MUX(.in(evicted_data_block_set), .select(set_index), .out(evicted_data_block)); 
    reg_mux #(.REG_N(SET_N), .WIDTH(TAG_WIDTH))  EVICTED_TAG_MUX(.in(evicted_tag_set), .select(set_index), .out(evict_tag)); 
    reg_mux #(.REG_N(SET_N), .WIDTH(1))  EVICTED_DIRTY_MUX(.in(evict_dirty_set), .select(set_index), .out(evict_dirty)); 
	register_address_decoder #(.INPUT_WIDTH(INDEX_WIDTH)) DECODER(.in(set_index),.out(set_select));  
    
    //assignments
    assign read_data_word = read_data_block[word_offset*WORD_WIDTH+:WORD_WIDTH]; 
    assign miss = |miss_set; 

    generate 
        genvar i; 
        for(i = 0; i < SET_N; i = i+1)
            begin 
                set #(.SET_WIDTH(SET_WIDTH), .DATA_WIDTH(DATA_WIDTH), .TAG_WIDTH(TAG_WIDTH), .WORD_WIDTH(WORD_WIDTH), .WORD_OFFSET_WIDTH(WORD_OFFSET_WIDTH)) SET(
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
                    .evict_dirty(evict_dirty_set[i]), 
                    .miss(miss_set[i])
                ); 
            end 
    endgenerate 
endmodule


module set #(
    parameter SET_WIDTH = 4,
    parameter DATA_WIDTH = 128,
    parameter TAG_WIDTH = 4,
    parameter WORD_WIDTH = 16,
    parameter WORD_OFFSET_WIDTH = 3
) (
    input  wire clk, reset, replace_request, write_enabled, set_selected,
    input  wire [TAG_WIDTH-1:0] tag_in,
    input  wire [DATA_WIDTH-1:0] data_in,     // block
    input  wire [WORD_WIDTH-1:0] word_in,     // word
    input  wire [WORD_OFFSET_WIDTH-1:0] word_offset,
    output reg [DATA_WIDTH-1:0] data, evict_data,
    output reg [TAG_WIDTH-1:0] evict_tag,
    output reg evict_dirty,
    output wire miss
);

    localparam LRU_COUNT_SIZE = $clog2(SET_WIDTH);

    // per-way arrays 
    wire [DATA_WIDTH-1:0]set_data      [0:SET_WIDTH-1];
    wire set_valid_bits [0:SET_WIDTH-1];
    wire set_dirty_bits [0:SET_WIDTH-1]; 
    wire [TAG_WIDTH-1:0] set_tag_bits   [0:SET_WIDTH-1];
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
        if(~set_valid_bits[0]) lru_way = 2'b00; 
        else if(~set_valid_bits[1]) lru_way = 2'b01; 
        else if(~set_valid_bits[2]) lru_way = 2'b10; 
        else if(~set_valid_bits[3]) lru_way = 2'b11; 
        else
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
    end

    //combinational muxes: select current data/evict_data/tag from the per-way arrays
    always @(*) begin
        data = {DATA_WIDTH{1'b0}};
        evict_data = {DATA_WIDTH{1'b0}};
        evict_tag  = {TAG_WIDTH{1'b0}};
        evict_dirty = 1'b0;
        integer w; 
        for (w = 0; w < SET_WIDTH; w = w + 1) begin
            if (way_select_one_hot[w]) begin
                data = set_data[w];
            end
            
            if (w == lru_way) begin
                evict_data  = set_data[w];
                evict_tag   = set_tag_bits[w];
                evict_dirty = set_dirty_bits[w];
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
                .dirty_bit(set_dirty_bits[i]),
                .word_offset(word_offset),
                .tag_in(tag_in),
                .valid_bit(set_valid_bits[i]),
                .data(set_data[i]),
                .tag(set_tag_bits[i]),
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
    output reg valid_bit,
    input [DATA_WIDTH-1:0] data_in,
    input [WORD_WIDTH-1:0] word_in, 
    input [WORD_OFFSET_WIDTH-1:0] word_offset,
    input wire [TAG_WIDTH-1:0] tag_in, 
    output reg [DATA_WIDTH-1:0] data,
    output reg dirty_bit,
    output reg [TAG_WIDTH-1:0] tag,
    output wire [LRU_COUNT_SIZE-1:0] lru_count
); 
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
            end 
        else 
            begin        
                if (replace_request && way_selected) 
                    begin
                        // install new block
                        data        <= data_in;
                        tag         <= tag_in;
                        valid_bit   <= 1'b1;
                        dirty_bit   <= 1'b0;
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
);
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