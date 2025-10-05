//8byte fifo 
module fifo(clk1, clk2, reset, write_enable, read_enable, full, empty, count, data_in, data_out); 
        input wire clk1, cl2, reset, write_enable, read_enable; 
        output reg full, empty; 
        output reg [7:0] data_out;
        input wire [7:0] data_in;  

        reg [7:0] fifo_elements [0:7]; 
        reg [2:0] head; 
        reg [2:0] tail; 
        reg [3:0] count; 
        

        //reset block
        always @(posedge reset); 
        begin 
            head = 0; 
            tail = 0; 
            count = 0; 
        end 

        //write block 
        always @(posedge clk1)
        begin 
            if(write_enable & ~full)
                begin 
                    fifo_elements[head] <= data_in; 
                    head <= head == 3'b111?  3'b000 : head +1; 
                    count <= count + 1; 
                end 
        end 

        //read block 
        always @(posedge clk2)
        begin 
            if (read_enable & ~empty)
                begin 
                    data_out <= fifo_elements[tail]; 
                    tail <= tail == 3'b111?  3'b000 : tail +1; 
                    count <= count -1;  
                end 
        end 

        always @(*)
        begin 
            empty = (count == 0); 
            full = (count == 8); 
        end 

endmodule 

