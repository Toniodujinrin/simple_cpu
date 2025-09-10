module branch_history_table(clk, increment_decrement, pc_bits, reset, count, tag_not_added, read_only);
	parameter INDEX_LEN = 7; 
	localparam LOCATIONS = 2**INDEX_LEN; 
	
	input [15:0] pc_bits; 
	input increment_decrement, clk, reset, read_only; 
	output [1:0] count; 
	output tag_not_added; 
	
	
	wire [LOCATIONS-1:0] tag_not_added_array; 
	wire [INDEX_LEN-1:0] index = pc_bits[8:2]; 
	wire [INDEX_LEN-1:0] tag = pc_bits[15:9]; 
	wire [LOCATIONS-1:0] set_selecter; 
	
	

	assign tag_not_added = |(tag_not_added_array & set_selecter); //use a mask to select the right address and the reduction OR
	
	//decodes lower 7 pc bits into 128 one hot addresses 
	register_address_decoder#(.INPUT_WIDTH(7)) DECODER(.in(index),.out(set_selecter));
	
	
	//generates 128 2-way sets 
	genvar i; 
	generate 
		for(i=0; i< LOCATIONS; i= i +1)
			begin:location 
				set_2_way#(.WAY_N(2))  SET(.clk(clk), .set_selected(set_selecter[i]), .reset(reset), .increment_decrement(increment_decrement), .count(count), .tag_in(tag), .tag_not_added(tag_not_added_array[i]), .read_only(read_only)); 
			end 
	endgenerate 
	
	

endmodule 



module set_2_way(increment_decrement, read_only, reset, clk, set_selected, tag_in, count, tag_not_added);
    input  increment_decrement, read_only, reset, clk, set_selected; 
    input  [6:0] tag_in; 
    output [1:0] count; 
    output tag_not_added; 
	  
    parameter WAY_N = 2; // must be 2 in this module

    // Per-way signals
    wire [6:0] tag_out [0:WAY_N-1];
    wire [1:0] cnt     [0:WAY_N-1];
    wire       valid   [0:WAY_N-1];

    // Hit detect (only if valid)
    wire hit0 = set_selected && valid[0] && (tag_out[0] == tag_in);
    wire hit1 = set_selected && valid[1] && (tag_out[1] == tag_in);

    // Miss / not-added
    assign tag_not_added = set_selected && !(hit0 || hit1);

    // --- Replacement choice ---
    // Prefer an invalid way; else use PLRU.
    reg plru_bit;  // 0 = way0 was LRU, replace way0; 1 = way1 was LRU
    wire choose_way0_invalid = tag_not_added && set_selected && !valid[0];
    wire choose_way1_invalid = tag_not_added && set_selected && !valid[1];

    wire replace_way0 = tag_not_added && (
                          choose_way0_invalid ||
                          (!choose_way0_invalid && !choose_way1_invalid && (plru_bit == 1'b0))
                        );
    wire replace_way1 = tag_not_added && (
                          choose_way1_invalid ||
                          (!choose_way0_invalid && !choose_way1_invalid && (plru_bit == 1'b1))
                        );

    // PLRU update:
    //  - On a hit, point PLRU to the *other* way (the hit way becomes MRU).
    //  - On a replacement, set PLRU away from the just-filled way (filled way becomes MRU).
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            plru_bit <= 1'b0;     // arbitrary reset value
        end else if (set_selected) begin
            if (hit0 || replace_way0) plru_bit <= 1'b1;  // way0 is MRU → next replace way1
            else if (hit1 || replace_way1) plru_bit <= 1'b0; // way1 is MRU → next replace way0
        end
    end

    //assign count. default to 2'b01 if not tag is found, and also raise tag_not_found flag
    assign count = !set_selected ? 2'bz :hit0 ? cnt[0] :cnt[1]; 

    // Ways
    way WAY0 (
        .clk(clk),
        .global_reset(reset),
        .way_selected(hit0),
        .replacement_mode(replace_way0),
        .increment_decrement(increment_decrement),
        .tag_in(tag_in),
        .tag_out(tag_out[0]),
        .sat_count(cnt[0]),
        .valid(valid[0]), 
		  .read_only(read_only)
    );

    way WAY1 (
        .clk(clk),
        .global_reset(reset),
        .way_selected(hit1),
        .replacement_mode(replace_way1),
        .increment_decrement(increment_decrement),
        .tag_in(tag_in),
        .tag_out(tag_out[1]),
        .sat_count(cnt[1]),
        .valid(valid[1]),
		  .read_only(read_only)
    );

endmodule


module way(clk, way_selected, replacement_mode, increment_decrement, global_reset, tag_in, tag_out, sat_count, valid, read_only);
	 input clk, way_selected, replacement_mode, increment_decrement, global_reset, read_only; //signal bits and clk 
    input  [6:0] tag_in; 
    output [6:0] tag_out; 
    output [1:0] sat_count;   //saturated counter count 
    output reg   valid; //valid bit to indicate if the contents of the way is valid. Default set to 0 when module is initialized
	 
    // Tag storage: written only on replacement
    register #(.WIDTH(7)) TAG_REG (
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
        .enabled(way_selected & ~read_only), //only increment counter when way is being updated
        .in(increment_decrement),
        .count(sat_count)
    );

    // Valid bit
    always @(posedge clk or posedge global_reset) begin
        if (global_reset)       valid <= 1'b0;
        else if (replacement_mode) valid <= 1'b1;
    end
endmodule



module sat_counter_2bit (clk, reset, enabled, in, count);
	 parameter DEFAULT_VALUE = 2'b01; 
    input  wire clk;
    input  wire reset;  // reset all counters (system reset)
    input  wire enabled;
    input  wire in;             // 1 = increment, 0 = decrement
    output reg  [1:0] count; 

    always @(posedge clk or posedge reset) begin
        if (reset) 
			  begin
					count <= DEFAULT_VALUE; //"usualy not taken" at startup
			  end 
		  else if (enabled) 
			  begin
					if (in) 
						begin
							 if (count != 2'b11)
								  count <= count + 1;
							 else 
									count <= count; 
						end 
					else 
						begin
							 if (count != 2'b00)
								  count <= count - 1;
							  else 
									count <= count; 
						end
			  end
		  else 
					count <= count; 
    end
endmodule

