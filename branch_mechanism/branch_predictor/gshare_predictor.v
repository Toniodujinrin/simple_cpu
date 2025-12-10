///////////////////////////////////////////////////////////////////////////////
// File:        gshare_predictor.v
// Author:      Toni Odujinrin
// Date:        2025-10-04 
// Description: G-share predictor module
///////////////////////////////////////////////////////////////////////////////

module gshare_predictor #(
    parameter HISTORY_LEN = 8
) (
    input  wire [15:0] pc_bits_read,
    input  wire [15:0] pc_bits_write,
    input  wire [HISTORY_LEN-1:0] history_write,
    input  wire clk,
    input  wire reset,
    input  wire predict_enable,     // asserted when making a prediction
    input  wire write_enabled,      // asserted during branch resolution
    input  wire outcome,            // actual branch outcome (1 = taken)
    input  wire rollback_enabled,   // asserted on mispredict to restore history
    output wire prediction,         // current prediction
    output wire [HISTORY_LEN-1:0] history_read_out
);

    localparam COUNT_LEN = 2;

    // Internal signals
    wire [HISTORY_LEN-1:0] history_read;
    wire [HISTORY_LEN-1:0] index_read;
    wire [HISTORY_LEN-1:0] index_write;
    wire [COUNT_LEN-1:0] count;
    wire out_bit;

    // Masked index generation (PC XOR GHR)
    assign index_read  = (history_read  ^ pc_bits_read[2 +: HISTORY_LEN]);
    assign index_write = (history_write ^ pc_bits_write[2 +: HISTORY_LEN]);

    // Prediction = MSB of 2-bit counter
    assign prediction = count[1];
    assign history_read_out = history_read;

    // Global History Register (GHR)
    shift_reg_n #(.WIDTH(HISTORY_LEN)) HISTORY_REG (
        .clk(clk),
        .in(prediction),
        .out(out_bit),
        .value(history_read),
        .reset(reset),
        .shift_enabled(predict_enable & ~rollback_enabled),
        .p_load_enabled(rollback_enabled),
        .load(history_write)
    );

    // Pattern History Table (PHT)
    pattern_history_table #(.INDEX_LEN(HISTORY_LEN)) COUNTER_FILE (
        .clk(clk),
        .index_read(index_read),
        .index_write(index_write),
        .count(count),
        .increment_decrement(outcome),
        .reset(reset),
        .write_enabled(write_enabled)
    );

endmodule
