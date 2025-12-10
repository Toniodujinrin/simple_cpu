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
                    count <= DEFAULT_VALUE; //"usually not taken" at startup
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