module register_file_pipeline #(
    parameter REG_DATA_WIDTH = 32,          // Bitwidth of data
    parameter REG_ADDR_WIDTH = 5,           // Bitwidth of address, supports 2^N registers by default
    parameter NUM_READ_PORTS = 2,       // Number of read ports
    parameter NUM_WRITE_PORTS = 1       // Number of write ports
) (
    input wire clk,
    input wire reset,
    input reg [NUM_WRITE_PORTS-1:0] wr_en,
    input reg [NUM_WRITE_PORTS-1:0][REG_ADDR_WIDTH-1:0] wr_addr,
    input reg [NUM_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data,
    input wire [NUM_READ_PORTS-1:0][REG_ADDR_WIDTH-1:0] rd_addr,
    output reg [NUM_READ_PORTS-1:0][REG_DATA_WIDTH-1:0] rd_data
    // output reg [REG_DATA_WIDTH-1:0] debug
);

    localparam NUM_REGISTERS = 1 << REG_ADDR_WIDTH;
    reg [REG_DATA_WIDTH-1:0] registers [0:NUM_REGISTERS-1];

    // Write logic
    always_ff @(posedge clk) begin
        if (reset) begin
            registers <= '{default: 0};  // Reset all registers
        end else begin
            for (int i = 0; i < NUM_WRITE_PORTS; i = i + 1) begin
                if (wr_en[i] && wr_addr[i] != 0) begin  // Ensure not writing to address 0
                    registers[wr_addr[i]] <= wr_data[i];
                end
            end
            // debug <= wr_data[0];
        end
    end

    // Read logic
    always_comb begin
        for (int j = 0; j < NUM_READ_PORTS; j = j + 1) begin
            if (rd_addr[j][REG_ADDR_WIDTH-1:0] == 0) 
                rd_data[j][REG_DATA_WIDTH-1:0] = 0;           // Address 0 always reads as 0
            else 
                rd_data[j][REG_DATA_WIDTH-1:0] = registers[rd_addr[j]];
        end
    end

endmodule
