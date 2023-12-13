import signals::*;

module exe_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter EXE_PORT = 2,
    parameter REG_DATA_WIDTH = 32,
    parameter int EXE_WRITE_PORTS = 4, 
    parameter int PHYSICAL_REGISTERS_ADDR_LEN = 6,
    parameter int NUM_PHYSICAL_REGISTERS = 64,
    parameter int CACHE_CYCLES = 3
) (
    input wire clk,
    input wire reset,

    input wire [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] exe_ports_available,    //delivered ports for EXE stage
    input wire [EXE_PORT-1:0] exe_enable,   //status of the delivered ports for EXE stage
    input wire [EXE_PORT-1:0] exe_is_ready_a,
    input wire [EXE_PORT-1:0] exe_is_ready_b,

    output logic exe_clear_signal,     //whether instructions should be emptied from buffer.
    output logic [DEPTH-1:0] exe_clear_mask,    //the address of instructions in ROB to be emptied.
    output logic [ROB_ADDR_WIDTH-1:0] exe_set_pt,   //the entry to set next_PC
    output logic [PC_WIDTH-1:0] exe_next_pc,    //value of next_PC

    output logic is_exe_ready,     // is EXE ready for restart after pipeline reset

    output exe_signals_t exe_entries_i [DEPTH-1:0],   //EXE generated signals
    output stage_t [DEPTH-1:0] current_status_exe,   //EXE status updates
    output logic [DEPTH-1:0] current_status_exe_enable,   //EXE status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    output logic i_cache_reset,

    // Status signals
    input wire is_ready,
    input wire is_pipeline_stall,
    input wire [ROB_ADDR_WIDTH-1:0] head,

    output logic [EXE_WRITE_PORTS-1:0] exe_wr_enable,
    output logic [EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] exe_wr_physical_addr,

    output reg [EXE_WRITE_PORTS-1:0] wr_en_exe,
    output reg [EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] wr_addr_exe,
    output reg [EXE_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data_exe,

    output logic [NUM_PHYSICAL_REGISTERS-1:0] is_cached_exe,

    output logic branch_prediction_en,
    output logic [PC_WIDTH-1:0] branch_prediction_pc,
    output logic [PC_WIDTH-1:0] branch_prediction_bias,
    output logic branch_prediction_taken,
    output logic clear_btb

);
    import signals::*;
    function logic [DEPTH-1:0] generate_clear_mask(
        input logic [ROB_ADDR_WIDTH-1:0] addr,
        input logic [ROB_ADDR_WIDTH-1:0] head
    );
        logic [ROB_ADDR_WIDTH-1:0] tmpaddr;
        logic [DEPTH-1:0] exe_clear_mask;
        logic [DEPTH-1:0] tmpmask;
        int shift;

        tmpaddr = (addr + 1) %DEPTH;

        // Calculate the shift
        shift = (tmpaddr + DEPTH - head) % DEPTH;

        // Create the temporary mask
        tmpmask = {DEPTH{1'b1}} >> shift;

        // Rotate the mask by 'addr' positions
        exe_clear_mask = (tmpmask << tmpaddr) | (tmpmask >> (DEPTH - tmpaddr));

        if (tmpaddr == head) begin
            exe_clear_mask = 'b0;
        end

        return exe_clear_mask;
    endfunction

    logic [DEPTH-1:0] mask_exe;
    logic [EXE_PORT-1:0] enable_addr_exe;
    logic [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_exe;
    logic unpredicted_jump;
    logic [ROB_ADDR_WIDTH-1:0] unpredicted_jump_entry_addr;
    logic [ROB_ADDR_WIDTH-1:0] unpredicted_jump_entry_addr_tmp;
    logic [ROB_ADDR_WIDTH-1:0] unpredicted_jump_entry_head;
    logic [PC_WIDTH-1:0] unpredicted_jump_pc_base;
    logic [PC_WIDTH-1:0] unpredicted_jump_pc_additional;

    logic is_branch;
    branch_t branch_op;
    
    logic [PC_WIDTH-1:0] next_pc;
    logic branch_taken;

    logic [REG_DATA_WIDTH-1:0] a,b,result, tmpb;
    logic [31:0] debug_PC;

    logic unpredicted_jump_release;


    logic [CACHE_CYCLES-1:0][EXE_WRITE_PORTS-1:0] wr_en_exe_cache;
    logic [CACHE_CYCLES-1:0][EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] wr_addr_exe_cache;
    logic [CACHE_CYCLES-1:0][EXE_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data_exe_cache;

    logic [NUM_PHYSICAL_REGISTERS-1:0] is_cached;
    logic [NUM_PHYSICAL_REGISTERS-1:0][REG_DATA_WIDTH-1:0] cached_value;

    assign clear_btb = i_cache_reset;

    // logic [EXE_WRITE_PORTS-1:0] 
    always_comb begin
        is_cached = 'b0;
        for (int i=0; i< NUM_PHYSICAL_REGISTERS; i++) begin
            cached_value[i] = 'b0;
        end

        for (int i=0; i< CACHE_CYCLES; i++) begin
            for (int j=0; j< EXE_WRITE_PORTS; j++) begin
                if (wr_en_exe_cache[i][j]) begin
                    cached_value[wr_addr_exe_cache[i][j]] = wr_data_exe_cache[i][j];
                end
            end
        end

        is_cached[0] = 1'b1;
        cached_value[0] = 'b0;

        // is_cached = 'b0;

        is_cached_exe = is_cached;

    end


    // assign current_status_exe_enable = mask_exe;

    always_comb begin
        mask_exe = 'b0;
        for (int j = 0; j < EXE_PORT; j = j + 1) begin
            if (enable_addr_exe[j]) begin
                mask_exe[addr_exe[j]] = 1'b1;
            end
        end
        if (unpredicted_jump_release && is_ready) begin
            mask_exe[unpredicted_jump_entry_addr] = 1'b1;
        end
        current_status_exe_enable = mask_exe;

    end


    always_ff @(posedge clk) begin
        branch_prediction_en <= 1'b0;
        // current_status_exe_enable <= mask_exe;
        unpredicted_jump_entry_head <= head;
        unpredicted_jump_entry_addr_tmp <= exe_ports_available[0];

        for (int i=0; i< CACHE_CYCLES-1; i++) begin
            for (int j=0; j< EXE_WRITE_PORTS; j++) begin
                wr_en_exe_cache[i+1][j] <= wr_en_exe_cache[i][j];
                wr_addr_exe_cache[i+1][j] <= wr_addr_exe_cache[i][j];
                wr_data_exe_cache[i+1][j] <= wr_data_exe_cache[i][j];
            end
        end
        if (reset) begin
            // unpredicted_jump_entry_head <= head;
            branch_prediction_pc <= 'b0;
            branch_prediction_bias <= 'b0;
            branch_prediction_taken <= 1'b0;
            // clear_btb <= 1'b0;

            i_cache_reset <= 1'b0;
            is_exe_ready <= 1'b0;
            enable_addr_exe <= 'b0;
            exe_clear_signal <= 1'b0;
            exe_clear_mask <= 'b0;
            exe_set_pt <= 'b0;
            exe_next_pc <= 'b0;
            current_status_exe <= '{DEPTH{IF}};
            unpredicted_jump <= 'b0;
            unpredicted_jump_release <= 1'b0;

            wr_en_exe <= 'b0;
            exe_wr_enable <= 'b0;

            for (int i=0; i< CACHE_CYCLES; i++) begin
                for (int j=0; j< EXE_WRITE_PORTS; j++) begin
                    wr_en_exe_cache[i][j] <= 1'b0;
                end
            end

        end else begin

            wr_en_exe <= 'b0;
            exe_wr_enable <= 'b0;  // ensure no additional writes
            for (int j=0; j<EXE_WRITE_PORTS;j++) begin
                wr_en_exe_cache[0][j] <= 1'b0;
            end

            if (unpredicted_jump) begin
                unpredicted_jump_entry_addr <= unpredicted_jump_entry_addr_tmp;
                exe_clear_signal <= 1'b1;
                exe_set_pt <= unpredicted_jump_entry_addr_tmp;
                next_pc = unpredicted_jump_pc_base + unpredicted_jump_pc_additional;
                exe_next_pc <= next_pc;
                exe_clear_mask <= generate_clear_mask(unpredicted_jump_entry_addr_tmp, head);
                //$display("exe_clear_mask:", generate_clear_mask(unpredicted_jump_entry_addr, unpredicted_jump_entry_head));
                // $display("jumped %h %h", unpredicted_jump_entry_addr, entries_o[unpredicted_jump_entry_addr].if_signals.PC);
                current_status_exe[unpredicted_jump_entry_addr_tmp] <= MEM;
                unpredicted_jump <= 1'b0;
                unpredicted_jump_release <= 1'b1;
            end else begin
                if (is_pipeline_stall) begin
                    for (int i=0; i< CACHE_CYCLES; i++) begin
                        for (int j=0; j< EXE_WRITE_PORTS; j++) begin
                            wr_en_exe_cache[i][j] <= 1'b0;
                        end
                    end
                    // $display("stall_exe");
                    is_exe_ready <= 1'b1;
                    enable_addr_exe <= 'b0;
                    if (is_ready) begin
                        exe_clear_signal <= 1'b0;
                        exe_clear_mask <= 'b0;
                        unpredicted_jump_release <= 1'b0;
                    end
                end else begin


                    unpredicted_jump_release <= 1'b1;
                    addr_exe = exe_ports_available;
                    enable_addr_exe = exe_enable;
                    is_exe_ready <= 1'b0;

                    for (int i=0;i<EXE_PORT;i++) begin
                        if (enable_addr_exe[i]) begin
                            current_status_exe[addr_exe[i]] <= EXE;
                        end
                    end

                    for (int i=0;i<EXE_PORT;i++) begin
                        if (enable_addr_exe[i]) begin
                            is_branch = entries_o[addr_exe[i]].id_signals.is_branch;

                            current_status_exe[addr_exe[i]] <= MEM;
                            if (exe_is_ready_a[i]) begin
                                a = entries_o[addr_exe[i]].of_signals.rf_rdata_a;
                            end else begin
                                a = cached_value[entries_o[addr_exe[i]].id_signals.src_rf_tag_a];
                            end

                            if (exe_is_ready_b[i]) begin
                                tmpb = entries_o[addr_exe[i]].of_signals.rf_rdata_b;
                            end else begin
                                tmpb = cached_value[entries_o[addr_exe[i]].id_signals.src_rf_tag_b];
                            end

                            exe_entries_i[addr_exe[i]].final_rf_rdata_b <= tmpb;

                            if (entries_o[addr_exe[i]].id_signals.use_rs2) begin
                                b = tmpb;
                            end else begin
                                b = entries_o[addr_exe[i]].id_signals.immediate;
                            end

                            case (entries_o[addr_exe[i]].id_signals.alu_op)
                                ADD: result = a + b;
                                SUB: result = a - b;
                                AND: result = a & b;
                                OR:  result = a | b;
                                NOT: result = ~a;
                                XOR: result = a ^ b;
                                default: result = 0;  // Default case for unrecognized opcodes
                            endcase

                            exe_entries_i[addr_exe[i]].rf_wdata_exe <= result;   // here, the result should be final, otherwise may cause problems
                            exe_wr_physical_addr[i] <= entries_o[addr_exe[i]].id_signals.dst_rf_tag;
                            wr_addr_exe[i] <= entries_o[addr_exe[i]].id_signals.dst_rf_tag;
                            wr_data_exe[i] <= result;
                            wr_addr_exe_cache[0][i] <= entries_o[addr_exe[i]].id_signals.dst_rf_tag;
                            wr_data_exe_cache[0][i] <= result;
                            if (~entries_o[addr_exe[i]].id_signals.mem_en) begin
                                wr_en_exe[i] <= 1'b1;
                                exe_wr_enable[i] <= 1'b1;
                                wr_en_exe_cache[0][i] <= 1'b1;
                            end
                        // if ((entries_o[addr_exe[i]].if_signals.PC == 32'h80000074)) begin
                            //$display("flag, %h %h %h %h", a, b, entries_o[addr_exe[i]].if_signals.PC, result);
                        // end
                            debug_PC <= entries_o[addr_exe[i]].if_signals.PC; 
                            branch_op = entries_o[addr_exe[i]].id_signals.branch_op;

                            if (is_branch && i == 0) begin
                                // next_pc = entries_o[addr_exe[i]].if_signals.PC;
                                branch_taken = 1'b0;
                                case(branch_op)
                                    BEQ: begin
                                        if (a == b) begin
                                            branch_taken = 1'b1;
                                            // next_pc = next_pc + entries_o[addr_exe[i]].id_signals.immediate;                                   
                                        end else begin
                                            // next_pc = next_pc + 4;
                                        end
                                    end
                                    default: begin
                                        //next_pc = entries_o[addr_exe[i]].if_signals.PC+4;
                                    end
                                endcase
                                if (~entries_o[addr_exe[i]].id_signals.is_pc_op) begin
                                    branch_prediction_en <= 1'b1;
                                    //left for branch prediction
                                end
                                // unpredicted_jump_entry_head <= head;
                                branch_prediction_pc <= entries_o[addr_exe[0]].if_signals.PC;
                                branch_prediction_bias <= entries_o[addr_exe[0]].id_signals.immediate;
                                branch_prediction_taken <= branch_taken;

                                unpredicted_jump_pc_base <= entries_o[addr_exe[0]].if_signals.PC;
                                unpredicted_jump_pc_additional = branch_taken ? entries_o[addr_exe[0]].id_signals.immediate : 4;


                                if (entries_o[addr_exe[i]].id_signals.is_pc_op | branch_taken != entries_o[addr_exe[i]].if_signals.branch_taken) begin
                                    unpredicted_jump <= 1'b1;
                                    // unpredicted_jump_entry_addr <= addr_exe[i];
                                    // unpredicted_jump_entry_head <= head;
                                    // unpredicted_jump_pc_base = entries_o[addr_exe[i]].if_signals.PC;
                                    // unpredicted_jump_pc_additional = branch_taken ? entries_o[addr_exe[i]].id_signals.immediate : 4;
                                    current_status_exe[addr_exe[i]] <= EXE;
                                    //$display("unpredicted branch: %d %h %h %h", i, entries_o[addr_exe[i]].if_signals.PC,  entries_o[addr_exe[i]].if_signals.inst, unpredicted_jump_pc_additional);
                                end else begin
                                    //$display("predicted branch: %d %h %h", i, entries_o[addr_exe[i]].if_signals.PC,  entries_o[addr_exe[i]].if_signals.inst);
                                end
                            end
                                
                        end
                    end
                end
            end

        end
    end


endmodule