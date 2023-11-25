`timescale 1ns / 1ps

module lab5_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire clk_i,
    input wire rst_i,
    input wire [ADDR_WIDTH-1:0] base_address_in,

    // wishbone master interface
    output wire wb_cyc_o,
    output wire wb_stb_o,
    input wire wb_ack_i,
    output wire [ADDR_WIDTH-1:0] wb_adr_o,
    output wire [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output wire [DATA_WIDTH/8-1:0] wb_sel_o,
    output wire wb_we_o
);

    localparam SERIAL_ADDR = 32'h10000000;
    localparam SERIAL_STATUS_ADDR = 32'h10000005;
    localparam NUM_CYCLES = 10;

    // Signals for wishbone master
    reg enable, write;
    reg [ADDR_WIDTH-1:0] address;
    reg [DATA_WIDTH-1:0] write_data;
    wire [DATA_WIDTH-1:0] read_data;
    wire finished;
    reg [3:0] sel;

    // Instantiate the wishbone master
    wishbone_master #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) wb_master (
        .clk(clk_i),
        .rst(rst_i),
        .enable(enable),
        .write(write),
        .address(address),
        .write_data(write_data),
        .sel(sel),
        .read_data(read_data),
        .finished(finished),
        .wb_cyc_o(wb_cyc_o),
        .wb_stb_o(wb_stb_o),
        .wb_ack_i(wb_ack_i),
        .wb_adr_o(wb_adr_o),
        .wb_dat_o(wb_dat_o),
        .wb_dat_i(wb_dat_i),
        .wb_sel_o(wb_sel_o),
        .wb_we_o(wb_we_o)
    );

    // Internal variables
    integer i;
    reg [7:0] byte_to_write;
    reg [7:0] status_byte;
    reg [DATA_WIDTH-1:0] memory_data;

    // FSM states
    typedef enum logic [3:0]{
        IDLE,
        CHECK_READ_ENABLE,
        READ_BYTE,
        // WAIT_FOR_READ,
        CHECK_WRITE_ENABLE,
        WRITE_TO_MEMORY,
        WRITE_TO_SERIAL,
        // WAIT_FOR_WRITE,
        INCREMENT,
        DONE
    } fsm_state_t;

    fsm_state_t fsm_state = IDLE;

    // Control FSM
    always @(posedge clk_i) begin
        if (rst_i) begin
            // Reset all control signals
            fsm_state <= IDLE;
            enable <= 0;
            write <= 0;
            address <= 0;
            write_data <= 0;
            sel <= 4'b0000;
            i <= 0;
        end else begin
            case (fsm_state)
                IDLE: begin
                    if (i < NUM_CYCLES) begin
                        fsm_state <= CHECK_READ_ENABLE;
                    end else begin
                        fsm_state <= DONE;
                    end
                end
                CHECK_READ_ENABLE: begin
                    enable <= 1;
                    write <= 0;
                    address <= SERIAL_STATUS_ADDR;
                    sel <= 4'b0010; // Select the second byte
                    fsm_state <= READ_BYTE;
                end
                READ_BYTE: begin
                    if (finished) begin
                        status_byte <= read_data[15:8]; // Assuming data is aligned
                        enable <= 0;
                        if (status_byte[0]) begin // Read enable check
                            fsm_state <= CHECK_WRITE_ENABLE;
                        end else begin
                            fsm_state <= CHECK_READ_ENABLE; // Polling until read is enabled
                        end
                    end
                end
                CHECK_WRITE_ENABLE: begin
                    if (status_byte[5]) begin // Write enable check
                        address <= SERIAL_ADDR;
                        sel <= 4'b0001; // Select the first byte
                        enable <= 1;

                        fsm_state <= WRITE_TO_MEMORY;
                    end else begin
                        fsm_state <= CHECK_READ_ENABLE; // Go back to check if we can read
                    end
                end
                // WAIT_FOR_READ: begin
                //     enable <= 1;
                //     fsm_state <= WRITE_TO_MEMORY;
                // end
                WRITE_TO_MEMORY: begin
                  enable <= 0;
                    if (finished) begin
                        byte_to_write = read_data[7:0]; // Read the byte
                        enable <= 1;
                        write <= 1;
                        address <= base_address_in + (i * 4);
                        memory_data[7:0] = byte_to_write;
                        write_data <= memory_data;
                        sel <= 4'b0001; // Select byte for writing to memory
                        fsm_state <= WRITE_TO_SERIAL;
                    end
                end
                WRITE_TO_SERIAL: begin
                  enable <= 0;
                    if (finished) begin
                        address <= SERIAL_ADDR;
                        write_data <= {24'b0, byte_to_write};
                        sel <= 4'b0001; // Select the first byte to write back to serial
                        fsm_state <= INCREMENT;
                        enable <= 1;
                    end
                end

                INCREMENT: begin
                    if (finished) begin
                        i <= i + 1;
                        enable <= 0;
                        write <= 0;
                        fsm_state <= IDLE;
                    end
                end
                DONE: begin
                    // Finished operation
                end
                default: fsm_state <= IDLE;
            endcase
        end
    end

endmodule
