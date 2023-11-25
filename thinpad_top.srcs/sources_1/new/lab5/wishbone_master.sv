`timescale 1ns / 1ps

module wishbone_master #(
    parameter ADDR_WIDTH = 32, // Address width
    parameter DATA_WIDTH = 32  // Data width
) (
    input wire clk,               // System clock
    input wire rst,               // System reset, active high
    input wire enable,            // Enable signal to start transactions
    input wire write,             // Write control (1 for write, 0 for read)
    input wire [ADDR_WIDTH-1:0] address, // Input address for transactions
    input wire [DATA_WIDTH-1:0] write_data, // Input data for write transactions
    input wire [3:0] sel,
    output reg [DATA_WIDTH-1:0] read_data, // Output data for read transactions
    output reg finished,          // Transaction finished indicator
    // Wishbone interface
    output reg wb_cyc_o,          // Wishbone cycle valid output
    output reg wb_stb_o,          // Wishbone strobe output
    input wire wb_ack_i,          // Wishbone acknowledge input
    output reg [ADDR_WIDTH-1:0] wb_adr_o, // Wishbone address output
    output reg [DATA_WIDTH-1:0] wb_dat_o, // Wishbone data output (write)
    input wire [DATA_WIDTH-1:0] wb_dat_i, // Wishbone data input (read)
    output reg [DATA_WIDTH/8-1:0] wb_sel_o, // Wishbone byte select outputs
    output reg wb_we_o            // Wishbone write enable output
);

// Define states using enum
typedef enum logic [1:0] {
    IDLE,
    READ_WAIT,
    WRITE_WAIT,
    DONE
} state_t;

// State register
state_t state = IDLE;

// Wishbone interface control
always @(posedge clk) begin
    if (rst) begin
        // Reset all outputs and state
        wb_cyc_o <= 1'b0;
        wb_stb_o <= 1'b0;
        wb_adr_o <= 0;
        wb_dat_o <= 0;
        wb_sel_o <= 0;
        wb_we_o  <= 1'b0;
        read_data <= 0;
        finished <= 1'b0;
        state <= IDLE;
    end else begin
        case (state)
            IDLE: begin
                if (enable) begin
                    wb_adr_o <= address;
                    wb_sel_o <= sel; // Select all bytes
                    finished <= 1'b0;
                    if (write) begin
                        // Initiate a write operation
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b1;
                        wb_dat_o <= write_data;
                        state <= WRITE_WAIT;
                    end else begin
                        // Initiate a read operation
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b0;
                        state <= READ_WAIT;
                    end
                end
            end
            READ_WAIT: begin
                // Wait for acknowledgment of read
                if (wb_ack_i) begin
                    // Read operation complete, get data
                    read_data <= wb_dat_i;
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished <= 1'b1;
                    state <= DONE;
                end
            end
            WRITE_WAIT: begin
                // Wait for acknowledgment of write
                if (wb_ack_i) begin
                    // Write operation complete
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished <= 1'b1;
                    state <= DONE;
                end
            end
            DONE: begin
                finished <= 1'b0;
                state <= IDLE;
            end
            default: begin
                // If an unknown state is reached, reset to IDLE
                state <= IDLE;
            end
        endcase
    end
end

endmodule
