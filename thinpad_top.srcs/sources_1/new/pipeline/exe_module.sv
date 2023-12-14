import signals::*;

module exe_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter EXE_PORT = 2,
    parameter REG_DATA_WIDTH = 32,
    parameter CSR_ADDR_LEN = 5,
    parameter int EXE_WRITE_PORTS = 4, 
    parameter int PHYSICAL_REGISTERS_ADDR_LEN = 6,
    parameter int NUM_PHYSICAL_REGISTERS = 64,
    parameter int CACHE_CYCLES = 2,
    parameter int SHIFT_WIDTH = 5,
    parameter int TICK_PER_CLK = 2048
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


    privilege_mode_t privilege_mode; 

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
                    is_cached[wr_addr_exe_cache[i][j]] = 1'b1;
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

    logic is_csr;
    csr_t csr_op;
    logic [CSR_ADDR_LEN: 0] csr_addr;

    logic [REG_DATA_WIDTH-1:0] mtime_low;// 0X100, Machine Time.
    logic [REG_DATA_WIDTH-1:0] mtime_high;
                                         // 0X101, Machine Time.
    logic [REG_DATA_WIDTH-1:0] mtimecmp_low;
                                        // 0X120, Machine Time Cmp.
    logic [REG_DATA_WIDTH-1:0] mtimecmp_high;
                                        // 0X121, Machine Time Cmp.

    logic [REG_DATA_WIDTH-1:0] new_mtime_low;
    logic [REG_DATA_WIDTH-1:0] new_mtime_high;

    logic [REG_DATA_WIDTH-1:0] mstatus;  // 0X300, Machine status register.
    logic [REG_DATA_WIDTH-1:0] mie;      // 0X304, Machine interrupt-enable register.
    logic [REG_DATA_WIDTH-1:0] mtvec;    // 0X305, Machine trap-handler base address.
    logic [REG_DATA_WIDTH-1:0] mscratch; // 0X340, Scratch register for machine trap handlers.
    logic [REG_DATA_WIDTH-1:0] mepc;     // 0X341, Machine exception program counter.
    logic [REG_DATA_WIDTH-1:0] mcause;   // 0X342, Machine trap cause.
    logic [REG_DATA_WIDTH-1:0] mip;      // 0X344, Machine interrupt pending.

    logic [REG_DATA_WIDTH-1:0] new_csr_reg;

    logic [REG_DATA_WIDTH-1:0] trap_pc;

    logic [16:0] cnt;

    logic mtie;
    logic mtip;

    always_ff @(posedge clk) begin
        i_cache_reset <= 1'b0;
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
            
            mstatus <= 32'b0;
            mie <= 32'b0;
            mtvec <= 32'b0;
            mscratch <= 32'b0;
            mepc <= 32'b0;
            mcause <= 32'b0;
            mip <= 32'b0;
            new_csr_reg <= 32'b0;
            trap_pc <= 'b0;
            cnt <= 16'b0;

            wr_en_exe <= 'b0;
            exe_wr_enable <= 'b0;

            for (int i=0; i< CACHE_CYCLES; i++) begin
                for (int j=0; j< EXE_WRITE_PORTS; j++) begin
                    wr_en_exe_cache[i][j] <= 1'b0;
                end
            end

        end else begin

            if (cnt == TICK_PER_CLK) begin
                cnt <= 0;
                new_mtime_low = mtime_low + 1;
                if (new_mtime_low == 0) begin
                    new_mtime_high = mtime_high + 1;
                end else begin
                    new_mtime_high = mtime_high;
                end
                mtime_low <= new_mtime_low;
                mtime_high <= new_mtime_high;
            end
            else begin
                cnt <= cnt + 1;
            end

            if (mtime_high > mtimecmp_high || (mtime_high == mtimecmp_high && mtime_low > mtimecmp_low)) begin
                mip[7] <= 1;
                mtip = 1;
            end else begin
                mip[7] <= 0;
                mtip = 0;
            end
            mtie = mie[7];

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
                                XOR: result = a ^ b;
                                NOT: result = ~a;
                                SLL: result = a << b[SHIFT_WIDTH-1:0];
                                SRL: result = a >> b[SHIFT_WIDTH-1:0];  // logical right shift
                                SRA: result = signed'(a) >>> b[SHIFT_WIDTH-1:0];   // arithmetic right shift
                                SLT: result = $signed(a) < $signed(b);
                                SLTU: result = a < b;
                                CLZ: begin
                                    result = REG_DATA_WIDTH;
                                    for(int i = 0; i < REG_DATA_WIDTH; i++) 
                                        if((a << i) >> (REG_DATA_WIDTH - 1)) begin
                                            result = i;
                                            break;
                                        end
                                end
                                MIN: result = ($signed(a) < $signed(b))? a:b;
                                PCNT: begin
                                    result = 0;
                                    for (int i = 0; i < REG_DATA_WIDTH; i++)
                                        result += (a >> i) & 1;
                                end         
                                AUIPC: begin
                                    result = entries_o[addr_exe[i]].if_signals.PC + b;
                                end                    
                                NEXT_PC: begin
                                    result = entries_o[addr_exe[i]].if_signals.PC + 4;
                                end
                                default: result = 0;  // Default case for unrecognized opcodes
                            endcase

                            is_csr = entries_o[addr_exe[i]].id_signals.is_csr;
                            csr_op = entries_o[addr_exe[i]].id_signals.csr_op;
                            csr_addr = entries_o[addr_exe[i]].id_signals.csr_addr;

                            if (mtip && mtie && privilege_mode == mode_u) begin
                                csr_addr = 0;
                                is_csr = 1;
                                csr_op = TIME_OUT;
                            end

                            if (is_csr) begin 
                                case(csr_addr)
                                    12'h300: b = mstatus;
                                    12'h304: b = mie;
                                    12'h305: b = mtvec;
                                    12'h340: b = mscratch;
                                    12'h341: b = mepc;
                                    12'h342: b = mcause;
                                    12'h344: b = mip;
                                    12'h100: b = mtime_low;
                                    12'h101: b = mtime_high;
                                    12'h120: b = mtimecmp_low;
                                    12'h121: b = mtimecmp_high;
                                    default: b = 0;
                                endcase

                                case(csr_op)
                                    NO_CSR: begin
                                    end
                                    CSRRC: begin
                                        new_csr_reg = b & (~a);
                                        result = b;
                                    end
                                    CSRRS: begin
                                        new_csr_reg = b | a;
                                        result = b;
                                    end
                                    CSRRW: begin
                                        new_csr_reg = a;
                                        result = b;
                                    end
                                    SETI: begin
                                        new_csr_reg = entries_o[addr_exe[i]].id_signals.immediate;
                                        result = 0;
                                    end
                                    EBREAK: begin
                                        mcause <= 32'd3; // supervisor-rv/kernel/include/exception.h:11
                                        mepc <= entries_o[addr_exe[i]].if_signals.PC + 4;
                                        trap_pc = {mtvec[31: 2], 2'b0};
                                        privilege_mode <= mode_m;
                                        mstatus <= {mstatus[31: 13], privilege_mode, mstatus[10: 0]};
                                    end
                                    ECALL: begin
                                        mcause <= 32'd8;
                                        mepc <= entries_o[addr_exe[i]].if_signals.PC + 4;
                                        trap_pc = {mtvec[31: 2], 2'b0};
                                        privilege_mode <= mode_m;
                                        mstatus <= {mstatus[31: 13], privilege_mode, mstatus[10: 0]};
                                    end
                                    MRET: begin 
                                        trap_pc = mepc;
                                        privilege_mode <= privilege_mode_t'(mstatus[12:11]);
                                        mstatus <= {mstatus[31: 13], 2'b0, mstatus[10: 0]};
                                    end
                                    TIME_OUT: begin
                                        mcause <= 32'h80000007;
                                        mepc <= entries_o[addr_exe[i]].if_signals.PC + 4;
                                        trap_pc = {mtvec[31: 2], 2'b0};
                                        privilege_mode <= mode_m;
                                        mstatus <= {mstatus[31: 13], privilege_mode, mstatus[10: 0]};
                                    end
                                endcase

                                case(csr_addr)
                                    12'h300: mstatus <= new_csr_reg;
                                    12'h304: mie <= new_csr_reg;
                                    12'h305: mtvec <= new_csr_reg;
                                    12'h340: mscratch <= new_csr_reg;
                                    12'h341: mepc <= new_csr_reg;
                                    12'h342: mcause <= new_csr_reg;
                                    12'h344: mip <= new_csr_reg;
                                    12'h100: mtime_low <= new_csr_reg;
                                    12'h101: mtime_high <= new_csr_reg;
                                    12'h120: mtimecmp_low <= new_csr_reg;
                                    12'h121: mtimecmp_high <= new_csr_reg;
                                    default: ;
                                endcase
                            end

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
                                current_status_exe[addr_exe[i]] <= WB;
                            end
                            //$display("exe_module: %h %h %h %h", a, b, entries_o[addr_exe[i]].if_signals.PC, result);
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
                                        end
                                    end
                                    BNE: begin
                                        if (a != b) branch_taken = 1'b1;
                                    end
                                    BGE: begin
                                        if ($signed(a) >= $signed(b)) branch_taken = 1'b1;
                                    end
                                    BGEU: begin
                                        if (a >= b) branch_taken = 1'b1;
                                    end
                                    BLT: begin
                                        if ($signed(a) < $signed(b)) branch_taken = 1'b1;
                                    end
                                    BLTU: begin
                                        if (a < b) branch_taken = 1'b1;
                                    end
                                    JAL: begin
                                        branch_taken = 1'b1;
                                    end
                                    JALR: begin
                                        branch_taken = 1'b1;
                                    end
                                    FENCE: begin
                                        branch_taken = 1'b0;
                                        i_cache_reset <= 1'b1;
                                    end
                                    TRAP: branch_taken = 1'b1;
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

                                if(branch_op == JALR) begin
                                    unpredicted_jump_pc_base = a;
                                    unpredicted_jump_pc_additional = entries_o[addr_exe[i]].id_signals.immediate;
                                end else if(branch_op == TRAP) begin
                                    unpredicted_jump_pc_base = trap_pc;
                                    unpredicted_jump_pc_additional = 0;
                                end else begin
                                    unpredicted_jump_pc_base = entries_o[addr_exe[i]].if_signals.PC;
                                    unpredicted_jump_pc_additional = branch_taken ? entries_o[addr_exe[i]].id_signals.immediate : 4;
                                end

                                if (entries_o[addr_exe[i]].id_signals.is_pc_op
                                    | branch_taken != entries_o[addr_exe[i]].if_signals.branch_taken
                                    | branch_op == TRAP
                                    ) begin
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