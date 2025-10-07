module chooser(
    input  wire clk,
    input  wire reset,
    input  wire gshare_prediction,
    input  wire bimodal_prediction,
    input  wire local_prediction,
    input  wire outcome,
    input  wire write_enabled,
    output reg  choice
);

    // Rank counters (2-bit saturating)
    wire [1:0] gshare_rank;
    wire [1:0] bimodal_rank;
    wire [1:0] local_rank;

    // Correct → increment; wrong → decrement
    wire gshare_rank_update;
    wire bimodal_rank_update;
    wire local_rank_update; 

    // Static priority values for tie in ranks (higher = better)
    localparam GSHARE_PRIORITY = 2'd3;
    localparam LOCAL_PRIORITY  = 2'd2;
    localparam BIMODAL_PRIORITY= 2'd1;

   

    assign gshare_rank_update = outcome == gshare_prediction; 
    assign bimodal_rank_update = outcome == bimodal_prediction; 
    assign local_rank_update = outcome == local_predictor; 


    //saturated counter used for ranking system, MSB of saturated counter as effective rank 
    sat_counter_2bit GSHARE_RANKER(
        .clk(clk), .reset(reset),
        .enabled(write_enabled), .in(gshare_rank_update),
        .count(gshare_rank)
    );

    sat_counter_2bit BIMODAL_RANKER(
        .clk(clk), .reset(reset),
        .enabled(write_enabled), .in(bimodal_rank_update),
        .count(bimodal_rank)
    );

    sat_counter_2bit LOCAL_RANKER(
        .clk(clk), .reset(reset),
        .enabled(write_enabled), .in(local_rank_update),
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