module counter (
    // Clock and reset signals
    input wire clk,
    input wire reset,

    // Trigger for counting
    input wire trigger,

    // Current count value
    output wire [3:0] count
);

reg [3:0] count_reg;

always_ff @(posedge clk) begin
    if(reset) begin
        count_reg <= 4'd0;  // Reset the counter register
    end else if (trigger && count_reg != 4'd15) begin
        count_reg <= count_reg + 4'd1;  // Increment the counter if trigger is high and not at max value
    end
end

assign count = count_reg;  // Output the count value

endmodule
