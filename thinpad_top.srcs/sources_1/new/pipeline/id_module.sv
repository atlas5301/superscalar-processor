import signals::*;
module id_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter ID_PORT = 2,
    parameter int NUM_LOGICAL_REGISTERS = 32,
    parameter int NUM_PHYSICAL_REGISTERS = 64,
    parameter int LOGICAL_REGISTERS_ADDR_LEN = 5,
    parameter int PHYSICAL_REGISTERS_ADDR_LEN = 6,
    parameter int ASSIGN_PORTS = 4
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
    input wire is_pipeline_stall,

    input wire [NUM_LOGICAL_REGISTERS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] latest_table_out,

    input wire [ASSIGN_PORTS-1:0] available_regs_enable,
    input wire [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] available_physical_regs,

    output logic [ASSIGN_PORTS-1:0] assign_regs_enable,
    // output logic [ASSIGN_PORTS-1:0][LOGICAL_REGISTERS_ADDR_LEN-1:0] assign_logical_regs,
    output logic [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] assign_physical_regs

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
        decoded.src_rf_tag_a = 0;
        decoded.src_rf_tag_b = 0;
        decoded.dst_rf_tag = 0;

        // Process fields for immediate values
        case(opcode)
            7'b0110111: imm = {inst[31:12], 12'b0};   //LUI
            7'b0110011: imm = 'b0; // ADD
            7'b0010011: imm = {{20{inst[31]}}, inst[31:20]};  // ADDI, ANDI
            7'b1100011: imm = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0}; // BEQ
            7'b0000011: begin
                if (funct3 == 3'b100 || funct3 == 3'b101) begin
                    imm = {20'b0, inst[31:20]};   // LBU,LHU
                end else begin
                    imm = {{20{inst[31]}}, inst[31:20]};   // LB
                end
            end
            7'b0100011: imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};       // SB, SW
            7'b1101111: imm = {{20{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}; //jal
            7'b1100111: imm = {{20{inst[31]}}, inst[31:20]}; //jalr
            7'b0010111: imm = {inst[31:12], 12'b0};//luipc
            default: imm = 32'b0;
        endcase

        decoded.immediate = imm;


        case (opcode)
            7'b0110111: begin// LUI
                decoded.rr_dst = rd;
            end
            7'b0010111: begin//AUIPC
                decoded.alu_op = AUIPC;
                decoded.rr_dst = rd;
                // $display("AUIPC %h %h",imm,inst);
            end
            7'b0010011: begin // ADDI, ANDI, XORI, ORI
                case(funct3)
                    3'b000: decoded.alu_op = ADD;
                    3'b001: begin
                        case(funct7) 
                            7'b0000000: decoded.alu_op = SLL;
                            7'b0110000: begin
                                case(rs2) 
                                    5'b00000: decoded.alu_op = CLZ;
                                    5'b00010: decoded.alu_op = PCNT;
                                endcase
                            end
                        endcase
                    end
                    3'b010: decoded.alu_op = SLT;
                    3'b011: decoded.alu_op = SLTU;
                    3'b100: decoded.alu_op = XOR;
                    3'b101: begin
                        case(funct7)
                            7'b0000000: decoded.alu_op = SRL;
                            7'b0100000: decoded.alu_op = SRA;
                            default: decoded.alu_op = SRL;
                        endcase
                    end
                    3'b110: decoded.alu_op = OR;
                    3'b111: decoded.alu_op = AND;
                    default: decoded.alu_op = ADD;
                endcase
                decoded.rr_a = rs1;
                decoded.rr_dst = rd;
            end
            7'b1100011: begin // BEQ, BNE, BGE, BGEU, BLT, BLTU
                case(funct3)
                    3'b000: decoded.branch_op = BEQ;
                    3'b001: decoded.branch_op = BNE;
                    3'b101: decoded.branch_op = BGE;
                    3'b111: decoded.branch_op = BGEU;
                    3'b100: decoded.branch_op = BLT;
                    3'b110: decoded.branch_op = BLTU;
                    default: decoded.branch_op = BEQ;
                endcase
                decoded.is_branch = 1;
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
                decoded.use_rs2 = 1;
            end
            7'b0000011: begin // LB, LH, LW, LBU, LHU
                case(funct3)
                    3'b000: decoded.mem_len = 1;
                    3'b001: decoded.mem_len = 2;
                    3'b010: decoded.mem_len = 4;
                    3'b100: decoded.mem_len = 1;
                    3'b101: decoded.mem_len = 2;
                endcase
                case(funct3)
                    3'b100: decoded.mem_is_signed = 0;
                    3'b101: decoded.mem_is_signed = 0;
                    default: decoded.mem_is_signed = 1;
                endcase
                decoded.mem_en = 1;
                decoded.rr_a = rs1;
                decoded.rr_dst = rd;
            end
            7'b0100011: begin // SB, SH, SW
                case(funct3)
                    3'b000: decoded.mem_len = 1;
                    3'b001: decoded.mem_len = 2;
                    3'b010: decoded.mem_len = 4;
                endcase
                decoded.mem_is_signed = 1;
                decoded.mem_en = 1;
                decoded.mem_write = 1;
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
            end
            7'b0110011: begin // ADD, SUB, XOR, OR, XOR, SLL, SRA, SRL, SLTU
                case(funct7) 
                    7'b0000000: begin
                        case(funct3)
                            3'b000: decoded.alu_op = ADD;
                            3'b001: decoded.alu_op = SLL;
                            3'b010: decoded.alu_op = SLT;
                            3'b011: decoded.alu_op = SLTU;
                            3'b100: decoded.alu_op = XOR;
                            3'b101: decoded.alu_op = SRL;
                            3'b110: decoded.alu_op = OR;
                            3'b111: decoded.alu_op = AND;
                            default: decoded.alu_op = ADD;
                        endcase
                    end
                    7'b0100000: begin
                        case(funct3)
                            3'b000: decoded.alu_op = SUB;
                            3'b101: decoded.alu_op = SRA;
                            default: decoded.alu_op = SUB;
                        endcase
                    end
                    7'b0000101: decoded.alu_op = MIN;
                endcase
                decoded.rr_a = rs1;
                decoded.rr_b = rs2;
                decoded.rr_dst = rd;
                decoded.use_rs2 = 1;
            end
            7'b1101111: begin //jal
                decoded.alu_op = NEXT_PC;
                decoded.branch_op = JAL;
                if (imm == 4) begin
                    decoded.is_branch = 0;
                end else begin
                    decoded.is_branch = 1;
                end
                decoded.rr_dst = rd;
            end
            7'b1100111: begin //jalr
                decoded.alu_op = NEXT_PC;
                decoded.branch_op = JALR;
                decoded.is_branch = 1;
                decoded.is_pc_op = 1;
                decoded.rr_a = rs1;
                decoded.rr_dst = rd;
            end
            default: begin
            end

        endcase

        return decoded;
    endfunction


    logic [NUM_LOGICAL_REGISTERS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] latest_table;

    logic [DEPTH-1:0] mask_id;
    logic [ID_PORT-1:0] enable_addr_id;
    logic [ID_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_id;

    id_signals_t tmp_id_signals [ID_PORT-1:0];
    

    // assign current_status_id_enable = mask_id;

    always_comb begin
        mask_id = 'b0;
        for (int j = 0; j < ID_PORT; j = j + 1) begin
            if (enable_addr_id[j]) begin
                mask_id[addr_id[j]] = 1'b1;
            end
        end
        current_status_id_enable = mask_id;
    end


    always_ff @(posedge clk) begin
        for (int i=0; i < ASSIGN_PORTS; i++) begin
            assign_physical_regs[i] <= available_physical_regs[i];
        end
        // current_status_id_enable <= mask_id;
        if (reset) begin
            is_id_ready <= 1'b0;
            enable_addr_id <= 'b0;
            id_clear_signal <= 1'b0;
            id_clear_mask <= 'b0;
            id_set_pt <= 'b0;
            id_next_pc <= 'b0;
            current_status_id <= '{DEPTH{IF}};

            for (int i = 0; i < NUM_LOGICAL_REGISTERS; i++) begin
                latest_table[i] = '0;
            end
            assign_regs_enable <= 'b0;

        end else begin
            assign_regs_enable <= 'b0;
            if (is_pipeline_stall) begin
                // $display("stall_id");
                for (int i = 0; i < NUM_LOGICAL_REGISTERS; i++) begin
                    latest_table[i] = latest_table_out[i];
                end        
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
                    //$display("ID: %d %d %d", i, available_regs_enable[i], available_physical_regs[i]);
                    if (enable_addr_id[i]) begin
                        if (available_regs_enable[i]) begin
                            tmp_id_signals[i] = rv32i_decoder_func(entries_o[addr_id[i]].if_signals.inst);

                            tmp_id_signals[i].src_rf_tag_a = latest_table[tmp_id_signals[i].rr_a];
                            tmp_id_signals[i].src_rf_tag_b = latest_table[tmp_id_signals[i].rr_b];                            

                            if (tmp_id_signals[i].rr_dst != 0) begin
                                tmp_id_signals[i].dst_rf_tag = available_physical_regs[i];
                                latest_table[tmp_id_signals[i].rr_dst] = available_physical_regs[i];
                                assign_regs_enable[i] <= 1'b1;
                                // $display("ID: %d %d", i, available_physical_regs[i]);

                            end else begin
                                tmp_id_signals[i].dst_rf_tag = 'b0;
                            end

                            id_entries_i[addr_id[i]] <= tmp_id_signals[i];
                            //$display("IDPC: %h",entries_o[addr_id[i]].if_signals.PC);
                            current_status_id[addr_id[i]] <= EXE;
                        end else begin
                            current_status_id[addr_id[i]] <= ID;
                        end
                    end
                end

            end

        end
    end



endmodule
