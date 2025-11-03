///////////////////////////////////////////////////////////////////////////////
// File:        local_predictor.v
// Author:      Toni Odujinrin
// Date:        2025-11-02 
// Description: Local Predictor Module
///////////////////////////////////////////////////////////////////////////////


module local_predictor #(
    parameter HISTORY_LEN = 10,
    parameter INDEX_LEN   = 7
) (
    input  wire clk,
    input  wire reset,
    input  wire write_enabled,
    input  wire outcome,
    input  wire rollback_enabled,
    input  wire predict_enable,
    input  wire [15:0] pc_bits_read,
    input  wire [15:0] pc_bits_write,
    input  wire [HISTORY_LEN-1:0] history_write,
    output wire [HISTORY_LEN-1:0] history_read_out,
    output wire prediction
);

    wire [HISTORY_LEN-1:0] history_read;
    wire [1:0] count;

    assign prediction = count[1];
    assign history_read_out = history_read;


    pattern_history_table #(.INDEX_LEN(HISTORY_LEN)) PHT (
        .clk(clk),
        .index_read(history_read),
        .index_write(history_write),
        .count(count),
        .increment_decrement(outcome),
        .reset(reset),
        .write_enabled(write_enabled)
    );

    
    local_history_table #(.HISTORY_LEN(HISTORY_LEN), .INDEX_LEN(INDEX_LEN)) LHT (
        .clk(clk),
        .prediction(prediction),
        .reset(reset),
        .rollback_enabled(rollback_enabled),
        .predict_enable(predict_enable),
        .pc_bits_read(pc_bits_read[8:2]),
        .pc_bits_write(pc_bits_write[8:2]),
        .history_write(history_write),
        .history_read(history_read)
    );

endmodule
