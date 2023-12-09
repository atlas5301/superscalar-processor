//four ways set associative cache
import signals::*;
module inst_cache_blocks #(
    parameter PC_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter IF_PORT = 2,
    parameter CACHE_WAYS = 4,
    parameter CACHE_WIDTH = 32,
    parameter bit CACHE_ENABLE = 1
) (
    input wire clk,
    input wire reset,

    input wire [IF_PORT-1:0][PC_WIDTH-1:0] read_pc,
    output reg [IF_PORT-1:0][CACHE_WAYS-1:0] check_valid,
    output reg [IF_PORT-1:0][DATA_WIDTH-1:0] o_inst,

    input wire we,
    input wire [PC_WIDTH-1:0] wr_pc,
    input wire [DATA_WIDTH-1:0] wr_inst
);  

    localparam int CACHE_BITWIDTH = $clog2(CACHE_WIDTH);
    localparam int WAY_BITWIDTH = $clog2(CACHE_WAYS);

    logic [CACHE_WIDTH-1:0][CACHE_WAYS-1:0] valid;
    logic [CACHE_WIDTH-1:0][CACHE_WAYS-1:0][PC_WIDTH-1:0] pc_entry;
    logic [CACHE_WIDTH-1:0][CACHE_WAYS-1:0][DATA_WIDTH-1:0] inst;
    logic [CACHE_WIDTH-1:0][WAY_BITWIDTH-1:0] way2write;

    always_comb begin
        if (CACHE_ENABLE) begin
            for (int j=0; j< IF_PORT; j++) begin
                logic [CACHE_BITWIDTH-1:0] tmp_addr = read_pc[j][CACHE_BITWIDTH+1:2];
                check_valid[j] = 'b0;
                o_inst[j] = 'b0;

                for (int i=0; i< CACHE_WAYS; i++) begin
                    if (valid[tmp_addr][i]) begin
                        if (pc_entry[tmp_addr][i] == read_pc[j]) begin
                            check_valid[j][i] = 1'b1;
                        end
                    end
                end

                for (int i=0; i< CACHE_WAYS; i++) begin
                    if (check_valid[j][i]) begin
                        o_inst[j] = inst[tmp_addr][i];
                        break;
                    end
                end
            end
        end
        else begin
            for (int j=0; j< IF_PORT; j++) begin
                check_valid[j] = 'b0;
                o_inst[j] = 'b0;        
            end
        end
    end

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i=0; i< CACHE_WIDTH; i++) begin
                for (int j=0; j<CACHE_WAYS; j++) begin
                    valid[i][j] <= 1'b0;
                end
                way2write[i] <= 'b0;
            end
        end else begin
            if (we) begin
                logic [CACHE_BITWIDTH-1:0] tmp_addr = wr_pc[CACHE_BITWIDTH+1:2];
                logic [WAY_BITWIDTH-1:0] tmp_way = way2write[tmp_addr];
                logic flag = 1'b1;
                for (int i=0; i< CACHE_WAYS; i++) begin
                    if (valid[tmp_addr][i]) begin
                        if (pc_entry[tmp_addr][i] == wr_pc) begin
                            tmp_way = i;
                            flag = 1'b0;
                            break;
                        end
                    end
                end

                valid[tmp_addr][tmp_way] <= 1'b1;
                pc_entry[tmp_addr][tmp_way] <= wr_pc;
                inst[tmp_addr][tmp_way] <= wr_inst;
                if (flag) begin
                    // $display("new_entry %h %h",wr_pc, wr_inst);
                    way2write[tmp_addr] <= (way2write[tmp_addr] + 1) % CACHE_WAYS;
                end
            end
        end

    end

endmodule