import signals::*;
module if_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter IF_PORT = 2,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
) (
    input wire clk,
    input wire reset,


    input wire [IF_PORT-1:0][ROB_ADDR_WIDTH-1:0] if_ports_available,    //delivered ports for IF stage
    input wire [IF_PORT-1:0] if_enable,   //status of the delivered ports for IF stage

    output logic if_clear_signal,     //whether instructions should be emptied from buffer.
    output logic [DEPTH-1:0] if_clear_mask,    //the address of instructions in ROB to be emptied.
    output logic [ROB_ADDR_WIDTH-1:0] if_set_pt,   //the entry to set next_PC
    output logic [PC_WIDTH-1:0] if_next_pc,    //value of next_PC

    output logic is_if_ready,     // is IF ready for restart after pipeline reset

    output if_signals_t if_entries_i [DEPTH-1:0],   //IF generated signals
    output stage_t [DEPTH-1:0] current_status_if,   //IF status updates
    output logic [DEPTH-1:0] current_status_if_enable,   //IF status updates enable

    input wire [DEPTH-1:0][PC_WIDTH-1:0] PC_IF,
    input wire [DEPTH-1:0] PC_ready,
    input wire [DEPTH-1:0] is_branch,

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    input wire is_ready,
    input wire is_pipeline_stall,


    output logic enable_IF,
    output logic write_IF,
    output logic [ADDR_WIDTH-1:0] address_IF,
    output logic [DATA_WIDTH-1:0] write_data_IF,
    output logic [3:0] sel_IF,
    input wire [DATA_WIDTH-1:0] read_data_IF,
    input wire finished_IF

);
    import signals::*;
    logic cache_we;
    logic [IF_PORT-1:0][PC_WIDTH-1:0] cache_in_pc;
    logic [ADDR_WIDTH-1:0] cache_in_inst;
    logic [IF_PORT-1:0] cache_o_enable;
    logic [IF_PORT-1:0][ADDR_WIDTH-1:0] cache_o_inst;
    inst_cache_blocks #(
        .PC_WIDTH(PC_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DEPTH(DEPTH),
        .IF_PORT(IF_PORT)
    ) inst_cache(
        .clk(clk),
        .reset(reset),
        .we(cache_we),
        .in_pc(cache_in_pc),
        .in_inst(cache_in_inst),
        .o_enable(cache_o_enable),
        .o_inst(cache_o_inst)
    );
    typedef enum logic [1:0] {
        IDLE,
        WORKING
    } state_IF_t;

    logic [31:0] debug_PC;
    logic [3:0] debug_entry;

    assign sel_IF = 4'b1;
    assign write_IF = 1'b0;
    assign write_data_IF = 'b0;
    
    logic [IF_PORT-1:0][ROB_ADDR_WIDTH-1:0] addresses;
    logic [IF_PORT-1:0] is_enable;

    logic [IF_PORT-1:0][ROB_ADDR_WIDTH-1:0] tmp_if_ports_available;
    logic [IF_PORT-1:0] tmp_if_enable;
    state_IF_t state_IF;
    int debug_cnt;
    always_comb begin
        cache_we = 0;
        cache_in_inst = 0;
        for(int i=0;i<IF_PORT;i++) cache_in_pc[i] = 1;
        case(state_IF)  
            IDLE: begin
                if(!is_pipeline_stall) begin
                    // TODO: finish 2 if-port
                    for(int i=0;i<IF_PORT;i++) begin
                            cache_in_pc[i] = PC_IF[if_ports_available[i]];
                    end
                end
            end
            WORKING: begin
                if(finished_IF) begin
                    cache_we = 1;
                    cache_in_inst = read_data_IF;
                    cache_in_pc[0] = PC_IF[addresses[0]];
                end
            end
            default : ;
        endcase
    end
    always_ff @(posedge clk) begin
        if (reset) begin
            if_clear_signal <= 1'b0;
            is_if_ready <= 1'b0;
            current_status_if_enable <= 'b0;
            enable_IF <= 1'b0;
            state_IF <= IDLE;
            if_clear_mask <= 'b0;
            if_set_pt <= 'b0;
            if_next_pc <= 'b0;
            current_status_if <= '{DEPTH{IF}};
            debug_cnt <= 0;
        end else begin
            debug_cnt <= debug_cnt + 1;
            case(state_IF) 
            IDLE: begin
                current_status_if_enable = 'b0;
                if (is_pipeline_stall) begin
                    is_if_ready <= 1'b1;
                    if (is_ready) begin
                        if_clear_signal <= 1'b0;
                        if_clear_mask <= 'b0;
                    end
                end else begin
                    is_if_ready <= 1'b0;
                    addresses = if_ports_available;
                    is_enable = if_enable;
                    // TODO: can only read One Inst, finish 2 ,fill IF_PORT
                    for(int i=0;i<IF_PORT;i++) begin 
                        if (is_enable[i]&& PC_ready[addresses[i]]) begin
                            //cache hit
                            // $display("cache_ic_pc : %h ; ADR : %h ; PC_IF[addresses[i]] : %h",cache_in_pc[i],addresses[i], PC_IF[addresses[i]]);
                            if(cache_o_enable[i]) begin
                                $display("hello : %h",cache_in_pc[i]);
                                state_IF <= IDLE;
                                debug_entry <= addresses[i];
                                debug_PC <= PC_IF[addresses[i]];
                                if_entries_i[addresses[i]].PC <= PC_IF[addresses[i]];
                                if_entries_i[addresses[i]].inst <= cache_o_inst[i];
                                if_entries_i[addresses[i]].branch_taken <= is_branch[addresses[i]];
                                current_status_if[addresses[i]] <= ID;
                                current_status_if_enable[addresses[i]] <= 1'b1;
                            end
                            //cache miss
                            else begin
                                $display("miss : %h",cache_in_pc[i]);
                                state_IF <= WORKING; 
                                enable_IF <= 1'b1;
                                address_IF <= PC_IF[addresses[i]];
                            end
                        end
                        break;
                    end
                end
            end
            WORKING: begin
                if (finished_IF) begin
                    state_IF <= IDLE;
                    debug_entry <= addresses[0];
                    debug_PC <= PC_IF[addresses[0]];
                    if_entries_i[addresses[0]].PC <= PC_IF[addresses[0]];
                    if_entries_i[addresses[0]].inst <= read_data_IF;
                    if_entries_i[addresses[0]].branch_taken <= is_branch[addresses[0]];
                    enable_IF <= 1'b0;
                    current_status_if[addresses[0]] <= ID;
                    current_status_if_enable[addresses[0]] <= 1'b1;
                end
            end
            default: begin
                state_IF <= IDLE;
            end
            
            endcase
        end
    end


endmodule


module mem_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter MEM_PORT = 2,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
) (
    input wire clk,
    input wire reset,


    input wire [MEM_PORT-1:0][ROB_ADDR_WIDTH-1:0] mem_ports_available,    //delivered ports for IF stage
    input wire [MEM_PORT-1:0] mem_enable,   //status of the delivered ports for IF stage
    output logic mem_clear_signal,     //whether instructions should be emptied from buffer.
    output logic [DEPTH-1:0] mem_clear_mask,    //the address of instructions in ROB to be emptied.
    output logic [ROB_ADDR_WIDTH-1:0] mem_set_pt,   //the entry to set next_PC
    output logic [PC_WIDTH-1:0] mem_next_pc,    //value of next_PC
    output logic is_mem_ready,     // is MEM ready for restart after pipeline reset
    output mem_signals_t mem_entries_i [DEPTH-1:0],   //MEM generated signals
    output stage_t [DEPTH-1:0] current_status_mem,   //MEM status updates
    output logic [DEPTH-1:0] current_status_mem_enable,   //MEM status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    input wire is_ready,
    input wire is_pipeline_stall,


    output logic enable_MEM,
    output logic write_MEM,
    output logic [ADDR_WIDTH-1:0] address_MEM,
    output logic [DATA_WIDTH-1:0] write_data_MEM,
    output logic [3:0] sel_MEM,
    input wire [DATA_WIDTH-1:0] read_data_MEM,
    input wire finished_MEM
);
    import signals::*;
    function logic [3:0] generate_byte_mask(input logic [31:0] address, input logic [2:0] read_length);
        logic [1:0] address_last_2_bits;
        logic [3:0] byte_mask;

        address_last_2_bits = address[1:0];

        case (read_length)
            1: begin
                byte_mask = (1 << address_last_2_bits);
            end
            2: begin
                byte_mask = (2'b11 << address_last_2_bits);
            end
            4: begin
                byte_mask = 4'b1111;
            end
        endcase

        return byte_mask;
    endfunction


    function logic [31:0] extend_to_32bit(
        input logic        signed_input,
        input logic [3:0]  sel_MEM,
        input logic [31:0] in_32bit
    );
        logic [31:0] operand;
        if (signed_input) begin
            // Signed extension based on the selected bytes using the mask
            case (sel_MEM)
                4'b0001: operand = {{24{in_32bit[7]}}, in_32bit[7:0]};
                4'b0010: operand = {{24{in_32bit[15]}}, in_32bit[15:8]};
                4'b0011: operand = {{16{in_32bit[15]}}, in_32bit[15:0]};
                4'b0100: operand = {{24{in_32bit[23]}}, in_32bit[23:16]};
                4'b1000: operand = {{24{in_32bit[31]}}, in_32bit[31:24]};                
                4'b1100: operand = {{16{in_32bit[31]}}, in_32bit[31:16]};
                4'b1111: operand = in_32bit;
                default: operand = 32'h0;
            endcase
        end else begin
            // Zero extension based on the selected bytes using the mask
            case (sel_MEM)
                4'b0001: operand = {24'h0, in_32bit[7:0]};
                4'b0010: operand = {24'h0, in_32bit[15:8]};
                4'b0011: operand = {16'h0, in_32bit[15:0]};
                4'b0100: operand = {24'h0, in_32bit[23:16]};
                4'b1000: operand = {24'h0, in_32bit[31:24]};  
                4'b1100: operand = {16'h0, in_32bit[31:16]};
                4'b1111: operand = in_32bit;
                default: operand = 32'h0;
            endcase
        end
        return operand;
    endfunction


    logic [3:0] sel_MEM_DEBUG;
    logic [31:0] result_DEBUG;

    typedef enum logic [1:0] {
        IDLE,
        WORKING
    } state_MEM_t;
    
    logic [MEM_PORT-1:0][ROB_ADDR_WIDTH-1:0] addresses;
    logic [MEM_PORT-1:0] is_enable;
    state_MEM_t state_MEM;
    logic access_mem;
    logic bypass_mem;
    logic [DATA_WIDTH-1:0] tmpdata;

    always_ff @(posedge clk) begin
        if (reset) begin
            sel_MEM = 4'b1;
            mem_clear_signal <= 1'b0;
            is_mem_ready <= 1'b0;
            current_status_mem_enable <= 'b0;
            enable_MEM <= 1'b0;
            state_MEM <= IDLE;
            mem_clear_mask <= 'b0;
            mem_set_pt <= 'b0;
            mem_next_pc <= 'b0;
            current_status_mem <= '{DEPTH{IF}};
        end else begin
            case(state_MEM) 
            IDLE: begin
                if (is_pipeline_stall) begin
                    is_mem_ready <= 1'b1;
                    current_status_mem_enable = 'b0;
                    if (is_ready) begin
                        mem_clear_signal <= 1'b0;
                        mem_clear_mask <= 'b0;
                    end

                end else begin
                    is_mem_ready <= 1'b0;
                    current_status_mem_enable = 'b0;
                    addresses = mem_ports_available;
                    is_enable = mem_enable;
                    access_mem = 1'b0;
                    bypass_mem = 1'b0;

                    //left empty for cache hit


                    for (int i=0;i<MEM_PORT;i++) begin
                        if (is_enable[i]) begin
                            if (entries_o[addresses[i]].id_signals.mem_en) begin
                                if (~bypass_mem) begin
                                    access_mem = 1'b1;
                                end
                                break;
                            end else begin
                                bypass_mem = 1'b1;
                                mem_entries_i[addresses[i]].rf_wdata_mem <= entries_o[addresses[i]].exe_signals.rf_wdata_exe;
                                current_status_mem_enable[addresses[i]] <= 1'b1;
                                current_status_mem[addresses[i]] = WB;
                            end
                        end
                    end

                    if (access_mem) begin
                        enable_MEM <= 1'b1;
                        write_MEM <= entries_o[addresses[0]].id_signals.mem_write;
                        address_MEM <= entries_o[addresses[0]].exe_signals.rf_wdata_exe;
                        write_data_MEM <= entries_o[addresses[0]].of_signals.rf_rdata_b;
                        sel_MEM = generate_byte_mask(entries_o[addresses[0]].exe_signals.rf_wdata_exe, entries_o[addresses[0]].id_signals.mem_len);
                        sel_MEM_DEBUG = sel_MEM;
                        state_MEM <= WORKING;
                        mem_entries_i[addresses[0]].rf_wdata_mem <= entries_o[addresses[0]].exe_signals.rf_wdata_exe;
                    end
                end
            end

            WORKING: begin
                if (finished_MEM) begin
                    state_MEM <= IDLE;
                    if (~write_MEM) begin
                        tmpdata = read_data_MEM;
                        mem_entries_i[addresses[0]].rf_wdata_mem <= extend_to_32bit(entries_o[addresses[0]].id_signals.mem_is_signed, sel_MEM, tmpdata);
                        result_DEBUG <= read_data_MEM;
                        //extend_to_32bit(entries_o[addresses[0]].id_signals.mem_is_signed, sel_MEM, tmpdata);
                    end
                    enable_MEM <= 1'b0;
                    current_status_mem[addresses[0]] = WB;
                    current_status_mem_enable[addresses[0]] = 1'b1;
                end
            end
            
            default: begin
                state_MEM <= IDLE;
            end

            endcase
        end
    end


endmodule



