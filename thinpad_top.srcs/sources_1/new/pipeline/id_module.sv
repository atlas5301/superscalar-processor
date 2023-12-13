import signals::*;
module id_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter ID_PORT = 2
) (
    input wire clk,
    input wire reset,

    input wire [ID_PORT-1:0][ROB_ADDR_WIDTH-1:0] id_ports_available,    //delivered ports for ID stage
    input wire [ID_PORT-1:0] id_enable,   //status of the delivered ports for ID stage

    output logic id_clear_signal,     //whether instructions should be emptied from buffer.
    output logic [DEPTH-1:0] id_clear_mask,    //the address of instructions in ROB to be emptied.
    output logic [ROB_ADDR_WIDTH-1:0] id_set_pt,   //the entry to set next_PC
    output logic [PC_WIDTH-1:0] id_next_pc,    //value of next_PC

    output logic is_id_ready,     // is ID ready for restart after pipeline reset

    output id_signals_t id_entries_i [DEPTH-1:0],   //ID generated signals
    output stage_t [DEPTH-1:0] current_status_id,   //ID status updates
    output logic [DEPTH-1:0] current_status_id_enable,   //ID status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    // Status signals
    input wire is_ready,
    input wire is_pipeline_stall

);
    import signals::*;
    function id_signals_t rv32i_decoder_func (input logic [31:0] inst);
        id_signals_t decoded;
        
        // Extract fields from instruction
        logic [6:0] opcode = inst[6:0];
        logic [4:0] rd = inst[11:7];
        logic [3:0] funct3 = inst[14:12];
        logic [4:0] rs1 = inst[19:15];
        logic [4:0] rs2 = inst[24:20];
        logic [6:0] funct7 = inst[31:25];
        logic [31:0] imm;

        decoded.immediate = 0;
        decoded.alu_op = ADD;
        decoded.use_rs2 = 0;
        decoded.mem_en = 0;
        decoded.mem_write = 0;
        decoded.mem_is_signed = 0;
        decoded.mem_len = 0;
        decoded.is_pc_op = 0;
        decoded.is_branch = 0;
        decoded.branch_op = BEQ;
        decoded.rr_a = 0;
        decoded.rr_b = 0;
        decoded.rr_dst = 0;

        // Process fields for immediate values
        case(opcode)
            7'b0110111: imm = {inst[31:12], 12'b0};   //LUI
            7'b0110011: imm = 'b0; // ADD
            7'b0010011: imm = {{20{inst[31]}}, inst[31:20]};  // ADDI, ANDI
            7'b1100011: imm = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0}; // BEQ
            7'b0000011: imm = {{20{inst[31]}}, inst[31:20]};   // LB
            7'b0100011: imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};       // SB, SW
            default: imm = 32'b0;
        endcase

        decoded.immediate = imm;


        case (opcode)
            7'b0110111: begin// LUI
                decoded.rr_dst = rd;
                decoded.rr_a = 0;
            end

            7'b0010011: begin // ADDI, ANDI
                case(funct3)
                    3'b000: begin
                        decoded.alu_op = ADD;
                    end
                    3'b111: begin
                        decoded.alu_op = AND;
                    end
                endcase
                decoded.rr_a = rs1;
                decoded.rr_dst = rd;
            end
            7'b1100011: begin // BEQ
                decoded.is_branch = 1;
                decoded.is_pc_op = 1;
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
                decoded.use_rs2 = 1;
            end
            7'b0000011: begin // LB
                decoded.mem_en = 1;
                decoded.rr_a = rs1;
                decoded.rr_dst = rd;
                decoded.mem_is_signed = 1;
                decoded.mem_len = 1;
            end
            7'b0100011: begin // SB, SW
                case(funct3)
                    3'b000: begin
                        decoded.mem_len = 1;
                    end
                    3'b010: begin
                        decoded.mem_len = 4;
                    end
                endcase
                decoded.mem_en = 1;
                decoded.mem_write = 1;
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
            end
            7'b0110011: begin // ADD
                decoded.alu_op = ADD;
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
                decoded.rr_dst = rd;
                decoded.use_rs2 = 1;
            end
            default: begin

            end

        endcase

        return decoded;
    endfunction


    logic [DEPTH-1:0] mask_id;
    logic [ID_PORT-1:0] enable_addr_id;
    logic [ID_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_id;
    

    // assign current_status_id_enable = mask_id;

    always_comb begin
        mask_id = 'b0;
        for (int j = 0; j < ID_PORT; j = j + 1) begin
            if (enable_addr_id[j]) begin
                mask_id[addr_id[j]] = 1'b1;
            end
        end
    end


    always_ff @(posedge clk) begin
        current_status_id_enable <= mask_id;
        if (reset) begin
            is_id_ready <= 1'b0;
            enable_addr_id <= 'b0;
            id_clear_signal <= 1'b0;
            id_clear_mask <= 'b0;
            id_set_pt <= 'b0;
            id_next_pc <= 'b0;
            current_status_id <= '{DEPTH{IF}};

        end else begin
            if (is_pipeline_stall) begin
                is_id_ready <= 1'b1;
                enable_addr_id <= 'b0;
                if (is_ready) begin
                    id_clear_signal <= 1'b0;
                    id_clear_mask <= 'b0;
                end
            end else begin
                is_id_ready <= 1'b0;
                addr_id = id_ports_available;
                enable_addr_id = id_enable;

                for (int i=0;i<ID_PORT;i++) begin
                    if (enable_addr_id[i]) begin
                        id_entries_i[addr_id[i]] <= rv32i_decoder_func(entries_o[addr_id[i]].if_signals.inst);
                        current_status_id[addr_id[i]] <= OF;
                    end
                end

            end

        end
    end



endmodule
