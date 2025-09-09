module tb_branch_history_table;
    reg clk, reset, increment_decrement;
    reg [15:0] pc_bits;
    wire [1:0] count;
    wire tag_not_added;

    branch_history_table dut (
        .clk(clk),
        .increment_decrement(increment_decrement),
        .pc_bits(pc_bits),
        .reset(reset),
        .count(count),
        .tag_not_added(tag_not_added)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns clock period
    end

    initial begin
        // Reset
        reset = 1;
        pc_bits = 16'h0000;
        increment_decrement = 0;
        #15;  // Hold reset across a clock edge
        reset = 0;
        #10;  // Wait after reset deassert

        // Test new tag (e.g., pc_bits such that tag=7'h05, index selects a set)
        pc_bits = 16'h0A14;  // tag=pc[15:9]=7'h05, index=pc[8:2]=7'h0A
        #5;  // Delay for combo propagation
        if (tag_not_added !== 1'b1) $display("Error: tag_not_added not asserted for new tag!");
        else $display("Success: tag_not_added asserted (value: %b)", tag_not_added);

        // Clock to perform replacement
        @(posedge clk);
        #5;  // Check after edge, with same pc_bits
        if (tag_not_added !== 1'b0) $display("Error: tag_not_added still asserted after add!");
        else $display("Success: tag_not_added deasserted (value: %b)", tag_not_added);

        // Test another new tag in same set (different tag, same index)
        pc_bits = 16'h0C14;  // tag=7'h06, same index=7'h0A
        #5;
        if (tag_not_added !== 1'b1) $display("Error: tag_not_added not asserted for second new tag!");
        else $display("Success: tag_not_added asserted (value: %b)", tag_not_added);

        // Clock to add
        @(posedge clk);
        #5;
        if (tag_not_added !== 1'b0) $display("Error: tag_not_added still asserted after second add!");
        else $display("Success: tag_not_added deasserted (value: %b)", tag_not_added);

        $finish;
    end
endmodule