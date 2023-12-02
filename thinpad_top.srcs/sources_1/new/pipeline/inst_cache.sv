//four ways set associative cache
import signals::*;
module inst_cache_blocks #(
    parameter PC_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter DEPTH = 64,
    parameter IF_PORT = 2,
    parameter CACHE_WAYS = 4,
    parameter CACHE_WIDTH = 32,
    parameter CACHE_BITWIDTH = 6
) (
    input wire clk,
    input wire reset,
    input wire we,
    input wire [IF_PORT-1:0][PC_WIDTH-1:0] in_pc,
    input wire [ADDR_WIDTH-1:0] in_inst,
    output reg [IF_PORT-1:0] o_enable,
    output reg [IF_PORT-1:0][ADDR_WIDTH-1:0] o_inst
);  
    typedef struct packed {
        logic [PC_WIDTH-1:0] pc;
        logic [ADDR_WIDTH-1:0] inst;
    } inst_cache_t;

    inst_cache_t [CACHE_WIDTH-1:0][CACHE_WAYS-1:0] cache;
    logic [CACHE_WIDTH-1:0][CACHE_BITWIDTH-1:0] head;
    logic [CACHE_WAYS-1:0] way;
    logic [CACHE_WIDTH-1:0] index;
    int x;

    function logic [3:0] get_cache_ways(
            input logic [CACHE_WIDTH-1:0][CACHE_BITWIDTH-1:0] head,
            input logic [CACHE_BITWIDTH-1:0] index, 
            input logic [PC_WIDTH-1:0] pc, 
            input logic we
    );
        if(we == 0) begin
            for(int i=0;i<CACHE_WAYS;i++) 
                if(cache[index][i].pc == pc) return i;
            return CACHE_WAYS;
        end else begin
            for(int i=0;i<CACHE_WAYS;i++) 
                if(cache[index][i].pc == 0) return i;
            x = head[index];
            head[index] = (head[index] + 1)% 4;
            return x;
        end
    endfunction

    initial begin
        for(int i=0;i<CACHE_WIDTH;i++) head[i] = 0;
    end
    always_comb begin
        index = 0;
        way = 0 ;    
        for(int i=0;i<IF_PORT;i++) begin 
            o_enable[i] = 0;
            o_inst[i] = 0;
        end
        if(we) begin
            index = in_pc[0][31:2] % CACHE_WIDTH;
            way = get_cache_ways(head, index, in_pc[0], we);
        end else begin
            for(int i=0;i<IF_PORT;i++) begin
                index = in_pc[i][31:2] % CACHE_WIDTH;
                way = get_cache_ways(head, index, in_pc[i], we);
                if(way != CACHE_WAYS) begin
                    o_enable[i] = 1;
                    o_inst[i] = cache[index][way];
                end else begin
                    o_enable[i] = 0;
                    o_inst[i] = 0;
                end
            end
        end
    end
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i=0;i<CACHE_WIDTH;i++)
                for(int j=0;j<CACHE_WAYS;j++)
                    cache[i][j] <= 0;
        end else begin
            if(we) begin
                cache[index][way].pc <= in_pc[0];
                cache[index][way].inst <= in_inst;
            end else ;
            end
    end
endmodule