module trigger (
    input wire input_btn,
    input wire clk,
    output reg signal
);
    logic last_state;

    always_ff @(posedge clk) begin
        if (input_btn) 
            signal <= ~last_state;
        else 
            signal <= 1'b0;
            
        last_state <= input_btn;
    end
endmodule

