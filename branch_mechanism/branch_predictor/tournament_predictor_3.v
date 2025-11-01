module tournament_predictor_3 (clk, reset, pc_bits, update_en, outcome, prediction); 
    parameter HISTORY_LEN = 8; 


	input clk, reset, write_enabled, outcome, 
    input [15:0] pc_bits_read, pc_bits_write; 
    input [HISTORY_LEN-1:0] history_write, 
    output [HISTORY_LEN-1:0] history_read; //current history to be propagted through the pipeline, gshare update at brnach resolution
    output prediction; 
     


	wire gshare_prediction, bimodal_prediction, local_prediction; 
    reg gshare_write_enabled, bimodal_write_enabled, local_write_enabled;

    always@(posedge clk, posedge reset)
    begin
        
    end  


	gshare_predictor  GSHARE(
        .clk(clk), 
        .reset(reset), 
        .write_enabled(write_enabled), 
        .pc_bits_read(pc_bits_read), 
        .pc_bits_wite(pc_bits_write),
        .history_write(history_write),
        .history_read(history_read), 
        .outcome(outcome), 
        .prediction(gshare_prediction)
    ); 
	
    bimodal_predictor BIMODAL(
        .clk(clk), 
        .reset(reset), 
        .write_enabled(write_enabled), 
        .pc_bits_read(pc_bits_read), 
        .pc_bits_write(pc_bits_write),
        .outcome(outcome), 
        .prediction(bimodal_prediction)
    ); 

	local_predictor   LOCAL(
        .clk(clk), 
        .reset(reset), 
        .write_enabled(write_enabled), 
        .pc_bits_read(pc_bits_read), 
        .pc_bits_read(pc_bits_write), 
        .outcome(outcome), 
        .prediction(local_prediction)
    ); 

	chooser           CHOOSER(
        .clk(clk), 
        .gshare_prediction(gshare_prediction), 
        .bimodal_prediction(bimodal_prediction), 
        .local_prediction(local_prediction), 
        .choice(prediction), 
        .outcome(outcome), 
        .write_enabled(write_enabled)
    ); 

   

endmodule
 


