module tournament_predictor_3 #(
    parameter GLOBAL_HISTORY_LEN = 8,
    parameter LOCAL_HISTORY_LEN  = 10,
    parameter LP_INDEX_LEN      = 7,
    parameter BP_INDEX_LEN      = 7,
    parameter BP_TAG_LEN        = 7
) (
    input  wire clk,
    input  wire reset,
    input  wire write_enabled,
    input  wire outcome,
    input  wire predict_enable,
    input  wire branch_miss,
    input  wire [15:0] pc_bits_read,
    input  wire [15:0] pc_bits_write,
    input  wire [GLOBAL_HISTORY_LEN-1:0] global_history_write,
    output wire [GLOBAL_HISTORY_LEN-1:0] global_history_read,
    input  wire [LOCAL_HISTORY_LEN-1:0]  local_history_write,
    output wire [LOCAL_HISTORY_LEN-1:0]  local_history_read,
    output wire prediction
);

    // local signals
    wire gshare_prediction;
    wire bimodal_prediction;
    wire local_prediction;

    reg gshare_write_enabled;
    reg bimodal_write_enabled;
    reg local_write_enabled;
    reg gshare_rollback_enabled;
    reg local_rollback_enabled;

    // update logic (simple always block setting enables/rollback)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gshare_rollback_enabled <= 1'b0;
            local_rollback_enabled  <= 1'b0;
            gshare_write_enabled    <= 1'b0;
            bimodal_write_enabled   <= 1'b0;
            local_write_enabled     <= 1'b0;
        end else begin
            if (branch_miss) begin
                gshare_rollback_enabled <= 1'b1;
                local_rollback_enabled  <= 1'b1;
            end else begin
                gshare_rollback_enabled <= 1'b0;
                local_rollback_enabled  <= 1'b0;
            end
            // normally update predictors at branch resolution
            gshare_write_enabled  <= 1'b1;
            bimodal_write_enabled <= 1'b1;
            local_write_enabled   <= 1'b1;
        end
    end

    // GSHARE instantiation (named ports to match gshare_predictor)
    gshare_predictor #(.HISTORY_LEN(GLOBAL_HISTORY_LEN)) GSHARE (
        .pc_bits_read(pc_bits_read),
        .pc_bits_write(pc_bits_write),
        .history_write(global_history_write),
        .clk(clk),
        .reset(reset),
        .predict_enable(predict_enable),
        .write_enabled(gshare_write_enabled),
        .outcome(outcome),
        .rollback_enabled(gshare_rollback_enabled),
        .prediction(gshare_prediction),
        .history_read_out(global_history_read)
    );

    // BIMODAL instantiation
    bimodal_predictor #(.INDEX_LEN(BP_INDEX_LEN), .TAG_LEN(BP_TAG_LEN)) BIMODAL (
        .clk(clk),
        .reset(reset),
        .write_enabled(bimodal_write_enabled),
        .outcome(outcome),
        .pc_bits_read(pc_bits_read),
        .pc_bits_write(pc_bits_write),
        .prediction(bimodal_prediction)
    );

    // LOCAL predictor instantiation (use correct parameter names and port separators)
    local_predictor #(.HISTORY_LEN(LOCAL_HISTORY_LEN), .INDEX_LEN(LP_INDEX_LEN)) LOCAL (
        .clk(clk),
        .reset(reset),
        .write_enabled(local_write_enabled),
        .outcome(outcome),
        .rollback_enabled(local_rollback_enabled),
        .predict_enable(predict_enable),
        .pc_bits_read(pc_bits_read),
        .pc_bits_write(pc_bits_write),
        .history_write(local_history_write),
        .history_read_out(local_history_read),
        .prediction(local_prediction)
    );

    // chooser selects final prediction
    chooser CHOOSER (
        .clk(clk),
        .reset(reset),
        .gshare_prediction(gshare_prediction),
        .bimodal_prediction(bimodal_prediction),
        .local_prediction(local_prediction),
        .outcome(outcome),
        .write_enabled(write_enabled),
        .choice(prediction)
    );

endmodule

