`timescale 1ns / 1ps

module wishbone_controller #(
    parameter ADDR_WIDTH = 32, // Address width
    parameter DATA_WIDTH = 32  // Data width
) (
    input wire clk, // System clock
    input wire rst, // System reset, active high

    // IF stage input signals
    input wire enable_IF,
    input wire write_IF,
    input wire [ADDR_WIDTH-1:0] address_IF,
    input wire [DATA_WIDTH-1:0] write_data_IF,
    input wire [3:0] sel_IF,
    output reg [DATA_WIDTH-1:0] read_data_IF,
    output reg finished_IF,

    // MEM stage input signals
    input wire enable_MEM,
    input wire write_MEM,
    input wire [ADDR_WIDTH-1:0] address_MEM,
    input wire [DATA_WIDTH-1:0] write_data_MEM,
    input wire [3:0] sel_MEM,
    output reg [DATA_WIDTH-1:0] read_data_MEM,
    output reg finished_MEM,

    // Wishbone interface
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    output reg [ADDR_WIDTH-1:0] wb_adr_o,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH/8-1:0] wb_sel_o,
    output reg wb_we_o
);

// Define states using enum
typedef enum logic [2:0] {
    IDLE,
    READ_WAIT_IF,
    WRITE_WAIT_IF,
    READ_WAIT_MEM,
    WRITE_WAIT_MEM,
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
        read_data_IF <= 0;
        read_data_MEM <= 0;
        finished_IF <= 1'b0;
        finished_MEM <= 1'b0;
        state <= IDLE;
    end else begin
        case (state)
            IDLE: begin
                finished_IF <= 1'b0;
                finished_MEM <= 1'b0;
                if (enable_MEM) begin
                    wb_adr_o <= address_MEM;
                    wb_sel_o <= sel_MEM;
                    finished_MEM <= 1'b0;
                    if (write_MEM) begin
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b1;
                        wb_dat_o <= write_data_MEM;
                        state <= WRITE_WAIT_MEM;
                    end else begin
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b0;
                        state <= READ_WAIT_MEM;
                    end
                end else if (enable_IF) begin
                    wb_adr_o <= address_IF;
                    wb_sel_o <= sel_IF;
                    finished_IF <= 1'b0;
                    if (write_IF) begin
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b1;
                        wb_dat_o <= write_data_IF;
                        state <= WRITE_WAIT_IF;
                    end else begin
                        wb_cyc_o <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_we_o  <= 1'b0;
                        state <= READ_WAIT_IF;
                    end
                end
            end
            READ_WAIT_IF: begin
                // $display("done?? %h %h", wb_adr_o,  wb_dat_i);
                if (wb_ack_i) begin
                    read_data_IF <= wb_dat_i;
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished_IF <= 1'b1;
                    state <= DONE;
                    // $display("done %h %h", wb_adr_o,  wb_dat_i);
                end
            end
            WRITE_WAIT_IF: begin
                if (wb_ack_i) begin
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished_IF <= 1'b1;
                    state <= DONE;
                end
            end
            READ_WAIT_MEM: begin
                if (wb_ack_i) begin
                    read_data_MEM <= wb_dat_i;
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished_MEM <= 1'b1;
                    state <= DONE;
                end
            end
            WRITE_WAIT_MEM: begin
                if (wb_ack_i) begin
                    wb_cyc_o <= 1'b0;
                    wb_stb_o <= 1'b0;
                    finished_MEM <= 1'b1;
                    state <= DONE;
                end
            end
            DONE: begin
                finished_IF <= 1'b0;
                finished_MEM <= 1'b0;
                state <= IDLE;
            end
            default: begin
                state <= IDLE;
            end
        endcase
    end
end

endmodule
