import signals::*;

module exe_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter EXE_PORT = 2,
    parameter REG_DATA_WIDTH = 32  
) (
    input wire clk,
    input wire reset,

    input wire [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] exe_ports_available,    //delivered ports for EXE stage
    input wire [EXE_PORT-1:0] exe_enable,   //status of the delivered ports for EXE stage

    output logic exe_clear_signal,     //whether instructions should be emptied from buffer.
    output logic [DEPTH-1:0] exe_clear_mask,    //the address of instructions in ROB to be emptied.
    output logic [ROB_ADDR_WIDTH-1:0] exe_set_pt,   //the entry to set next_PC
    output logic [PC_WIDTH-1:0] exe_next_pc,    //value of next_PC

    output logic is_exe_ready,     // is EXE ready for restart after pipeline reset

    output exe_signals_t exe_entries_i [DEPTH-1:0],   //EXE generated signals
    output stage_t [DEPTH-1:0] current_status_exe,   //EXE status updates
    output logic [DEPTH-1:0] current_status_exe_enable,   //EXE status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    // Status signals
    input wire is_ready,
    input wire is_pipeline_stall,
    input wire [ROB_ADDR_WIDTH-1:0] head
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

        tmpaddr = (addr + 2) %DEPTH;

        // Calculate the shift
        shift = (tmpaddr + DEPTH - head) % DEPTH;

        // Create the temporary mask
        tmpmask = {DEPTH{1'b1}} >> shift;

        // Rotate the mask by 'addr' positions
        exe_clear_mask = (tmpmask << tmpaddr) | (tmpmask >> (DEPTH - tmpaddr));

        return exe_clear_mask;
    endfunction



    logic [DEPTH-1:0] mask_exe;
    logic [EXE_PORT-1:0] enable_addr_exe;
    logic [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_exe;
    logic branch_flag;

    logic is_branch;
    branch_t branch_op;
    
    logic [PC_WIDTH-1:0] next_pc;
    logic branch_taken;

    logic [REG_DATA_WIDTH-1:0] a,b,result;
    logic [31:0] debug_PC;

    assign current_status_exe_enable = mask_exe;

    always_comb begin
        mask_exe = 'b0;
        for (int j = 0; j < EXE_PORT; j = j + 1) begin
            if (enable_addr_exe[j]) begin
                mask_exe[addr_exe[j]] = 1'b1;
            end
        end
    end


    always_ff @(posedge clk) begin
        if (reset) begin
            is_exe_ready <= 1'b0;
            enable_addr_exe <= 'b0;
            exe_clear_signal <= 1'b0;
            exe_clear_mask <= 'b0;
            branch_flag <= 1'b0;
            exe_set_pt <= 'b0;
            exe_next_pc <= 'b0;
            current_status_exe <= '{DEPTH{IF}};

        end else begin
            if (is_pipeline_stall) begin
                is_exe_ready <= 1'b1;
                enable_addr_exe <= 'b0;
                branch_flag = 1'b0;
                if (is_ready) begin
                    exe_clear_signal <= 1'b0;
                    exe_clear_mask <= 'b0;
                end
            end else begin
                addr_exe = exe_ports_available;
                enable_addr_exe = exe_enable;
                branch_flag = 1'b0;
                is_exe_ready <= 1'b0;

                for (int i=0;i<EXE_PORT;i++) begin
                    if (branch_flag) begin
                        break;
                    end

                    if (enable_addr_exe[i]) begin

                        a = entries_o[addr_exe[i]].of_signals.rf_rdata_a;
                        if (entries_o[addr_exe[i]].id_signals.use_rs2) begin
                            b = entries_o[addr_exe[i]].of_signals.rf_rdata_b;
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

                        exe_entries_i[addr_exe[i]].rf_wdata_exe <= result;

                        debug_PC <= entries_o[addr_exe[i]].if_signals.PC; 


                        is_branch = entries_o[addr_exe[i]].id_signals.is_branch;
                        branch_op = entries_o[addr_exe[i]].id_signals.branch_op;

                        if (is_branch) begin
                            next_pc = entries_o[addr_exe[i]].if_signals.PC;
                            branch_taken = 1'b0;
                            case(branch_op)
                                BEQ: begin
                                    if (a == b) begin
                                        branch_taken = 1'b1;
                                        next_pc = next_pc + entries_o[addr_exe[i]].id_signals.immediate;                                   
                                    end else begin
                                        next_pc = next_pc + 4;
                                    end
                                end
                                default: begin
                                    next_pc = entries_o[addr_exe[i]].if_signals.PC+4;
                                end
                            endcase
                            if (branch_taken != entries_o[addr_exe[i]].if_signals.branch_taken) begin
                                branch_flag = 1'b1;
                                exe_clear_signal <= 1'b1;
                                exe_set_pt <= addr_exe[i];
                                exe_next_pc <= next_pc;
                                exe_clear_mask <= generate_clear_mask(addr_exe[i], head);
                            end
                        end

                        current_status_exe[addr_exe[i]] <= MEM;
                        

                            
                    end
                end

            end

        end
    end


endmodule