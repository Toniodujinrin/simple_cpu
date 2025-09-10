module tournament_predictor_3 (clk, reset, pc_bits, update_en, outcome, prediction); 

	input clk, reset, update_en, outcome; 
	wire gshare_prediction, bimodal_prediction, local_prediction; 

	gshare_predictor  GSHARE(.clk(clk), .reset(reset), .update_en(update_en), .pc_bits(pc_bits[9:2]), .outcome(outcome), .prediction(gshare_prediction)); 
	bimodal_predictor BIMODAL(.clk(clk), .reset(reset), .update_en(update_en), .pc_bits(pc_bits), .outcome(outcome), .prediction(bimodal_prediction)); 
	local_predictor   LOCAL(.clk(clk), .reset(reset), .update_en(update_en), .pc_bits(pc_bits[8:2]), .outcome(outcome), .prediction(local_prediction)); 
	chooser           CHOOSER(.clk(clk), .gshare_prediction(gshare_prediction), .bimodal_prediction(bimodal_prediction), .local_prediction(local_prediction), .choice(prediction), .outcome(outcome), .update_en(update_en)); 

endmodule


module chooser(
    input  wire clk,
    input  wire reset,
    input  wire gshare_prediction,
    input  wire bimodal_prediction,
    input  wire local_prediction,
    input  wire outcome,
    input  wire update_en,
    output reg  choice
);

    // Rank counters (2-bit saturating)
    wire [1:0] gshare_rank;
    wire [1:0] bimodal_rank;
    wire [1:0] local_rank;

    // Static priority values (higher = better)
    localparam GSHARE_PRIORITY = 2'd3;
    localparam LOCAL_PRIORITY  = 2'd2;
    localparam BIMODAL_PRIORITY= 2'd1;

    // Correct → increment; wrong → decrement
    wire gshare_rank_update  = (outcome == gshare_prediction);
    wire bimodal_rank_update = (outcome == bimodal_prediction);
    wire local_rank_update   = (outcome == local_prediction);

    sat_counter_2bit GSHARE_RANKER(
        .clk(clk), .reset(reset),
        .enabled(update_en), .in(gshare_rank_update),
        .count(gshare_rank)
    );

    sat_counter_2bit BIMODAL_RANKER(
        .clk(clk), .reset(reset),
        .enabled(update_en), .in(bimodal_rank_update),
        .count(bimodal_rank)
    );

    sat_counter_2bit LOCAL_RANKER(
        .clk(clk), .reset(reset),
        .enabled(update_en), .in(local_rank_update),
        .count(local_rank)
    );

    // Choose predictor
    always @(*) begin
        if ((gshare_rank > bimodal_rank) && (gshare_rank > local_rank))
            choice = gshare_prediction;
        else if ((local_rank > gshare_rank) && (local_rank > bimodal_rank))
            choice = local_prediction;
        else if ((bimodal_rank > gshare_rank) && (bimodal_rank > local_rank))
            choice = bimodal_prediction;
        else begin
            // Tie-breaking by static priority
            if ((gshare_rank == local_rank) && (gshare_rank > bimodal_rank))
                choice = (GSHARE_PRIORITY > LOCAL_PRIORITY) ? gshare_prediction : local_prediction;
            else if ((gshare_rank == bimodal_rank) && (gshare_rank > local_rank))
                choice = (GSHARE_PRIORITY > BIMODAL_PRIORITY) ? gshare_prediction : bimodal_prediction;
            else if ((local_rank == bimodal_rank) && (local_rank > gshare_rank))
                choice = (LOCAL_PRIORITY > BIMODAL_PRIORITY) ? local_prediction : bimodal_prediction;
            else begin
                // Full tie (all equal) → fall back to static order
                if (GSHARE_PRIORITY >= LOCAL_PRIORITY && GSHARE_PRIORITY >= BIMODAL_PRIORITY)
                    choice = gshare_prediction;
                else if (LOCAL_PRIORITY >= BIMODAL_PRIORITY)
                    choice = local_prediction;
                else
                    choice = bimodal_prediction;
            end
        end
    end

endmodule