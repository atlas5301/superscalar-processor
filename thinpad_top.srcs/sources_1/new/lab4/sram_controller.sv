module sram_controller #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,

    parameter SRAM_ADDR_WIDTH = 20,
    parameter SRAM_DATA_WIDTH = 32,

    localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
    localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
) (
    // clk and reset
    input wire clk_i,
    input wire rst_i,

    // wishbone slave interface
    input wire wb_cyc_i,
    input wire wb_stb_i,
    output reg wb_ack_o,
    input wire [ADDR_WIDTH-1:0] wb_adr_i,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input wire wb_we_i,

    // sram interface
    output reg [SRAM_ADDR_WIDTH-1:0] sram_addr,
    inout wire [SRAM_DATA_WIDTH-1:0] sram_data,
    output reg sram_ce_n,
    output reg sram_oe_n,
    output reg sram_we_n,
    output reg [SRAM_BYTES-1:0] sram_be_n
);

  localparam BYTE_WIDTH = DATA_WIDTH/8;

  wire clock, reset;
  assign clock = clk_i;
  assign reset = rst_i;

  wire CYC_I, STB_I, WE_I;
  assign CYC_I = wb_cyc_i;
  assign STB_I = wb_stb_i;
  assign WE_I = wb_we_i;


  reg ram_ce_n_reg,ram_oe_n_reg,ram_we_n_reg;
  // initial begin
  //     ram_ce_n_reg = 1'b1;
  //     ram_oe_n_reg = 1'b1;
  //     ram_we_n_reg = 1'b1;
  // end

  assign sram_ce_n = ram_ce_n_reg;
  assign sram_oe_n = ram_oe_n_reg;
  assign sram_we_n = ram_we_n_reg;

  // Define the enum states
  typedef enum {
      READ,
      WRITE
  } sram_state_t;

  reg [SRAM_DATA_WIDTH-1:0] sram_data_i_comb;
  reg [SRAM_DATA_WIDTH-1:0] sram_data_o_comb;
  reg sram_data_t_comb;
  sram_state_t sram_state;

  // Logic for setting sram_data_t_comb based on state
  always_comb begin
      case(sram_state)
          READ: sram_data_t_comb = 1'b1;
          WRITE: sram_data_t_comb = 1'b0;
          default: sram_data_t_comb = 1'b1;
      endcase
  end

  // Other assignment statements remain the same
  assign sram_data = sram_data_t_comb ? 32'bz : sram_data_o_comb;
  assign sram_data_i_comb = sram_data;


  // TODO: 实现 SRAM 控制器

  //DEFINE STATES
  typedef enum logic [2:0] {
    STATE_IDLE,
    STATE_READ_1,
    STATE_READ_2,
    STATE_WRITE_1,
    STATE_WRITE_2,
    STATE_WRITE_3
  } state_t;

  state_t state;

  always_ff @ (posedge clock) begin
      if (reset) begin
          ram_ce_n_reg <= 1'b1;
          ram_oe_n_reg <= 1'b1;
          ram_we_n_reg <= 1'b1;
          state <= STATE_IDLE;
          wb_ack_o <= 0;
      end else begin
          case (state)
              STATE_IDLE: begin
                  wb_ack_o <= 1'b0;
                  if (STB_I && CYC_I) begin
                      if (WE_I) begin
                          state <= STATE_WRITE_1;
                          sram_state <= WRITE;
                          ram_ce_n_reg <= 1'b0;
                          ram_oe_n_reg <= 1'b1;
                          ram_we_n_reg <= 1'b1;
                          sram_addr <= wb_adr_i[SRAM_ADDR_WIDTH+1:2];
                          sram_data_o_comb <= wb_dat_i;
                          sram_be_n <= ~wb_sel_i;
                      end else begin
                          state <= STATE_READ_1;
                          sram_state <= READ;
                          ram_ce_n_reg <= 1'b0;
                          ram_oe_n_reg <= 1'b0;
                          ram_we_n_reg <= 1'b1;
                          sram_addr <= wb_adr_i[SRAM_ADDR_WIDTH+1:2];
                          sram_be_n <= {SRAM_BYTES{1'b0}};
                          // wb_dat_o <= {SRAM_DATA_WIDTH{1'b0}};
                      end
                  end else begin
                    ram_ce_n_reg <= 1'b1;
                  end
              end
              STATE_READ_1: begin
                // $display("read1");
                  wb_dat_o <= sram_data_i_comb; 
                  wb_ack_o <= 1'b1;
                  ram_oe_n_reg <= 1'b1;
                  state <= STATE_READ_2;
              end

              STATE_READ_2: begin
                // $display("read2");
                  wb_ack_o <= 1'b0;
                  state <= STATE_IDLE;
              end
              

              STATE_WRITE_1: begin
                // $display("write1");
                  ram_we_n_reg <= 1'b0;
                  state <= STATE_WRITE_2;
              end

              STATE_WRITE_2: begin
                // $display("write2");
                  ram_we_n_reg <= 1'b1;
                  wb_ack_o <= 1'b1;
                  state <= STATE_WRITE_3;
              end

              STATE_WRITE_3: begin
                // $display("write3");
                  ram_we_n_reg <= 1'b1;
                  wb_ack_o <= 1'b0;
                  state <= STATE_IDLE;
              end

          endcase
      end
  end

endmodule
