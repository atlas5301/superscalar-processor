import signals::*;

module rename_register_mapping_table #(
    parameter int NUM_LOGICAL_REGISTERS = 32,
    parameter int NUM_PHYSICAL_REGISTERS = 64,
    parameter int LOGICAL_REGISTERS_ADDR_LEN = 5,
    parameter int PHYSICAL_REGISTERS_ADDR_LEN = 6,
    parameter int ASSIGN_PORTS = 4,
    parameter int SUBMIT_PORTS = 4,
    parameter int EXE_WRITE_PORTS = 4,
    parameter int MEM_WRITE_PORTS = 4,
    parameter int DEPTH = 64,
    parameter int ROB_ADDR_WIDTH = 6,
    parameter int OF_PORT = 4
) (
    input wire clk,
    input wire reset,
    input wire re_map,

    output logic [NUM_LOGICAL_REGISTERS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] latest_table_out,

    output logic [ASSIGN_PORTS-1:0] available_regs_enable,
    output logic [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] available_physical_regs,

    input wire [ASSIGN_PORTS-1:0] assign_regs_enable,
    // input wire [ASSIGN_PORTS-1:0][LOGICAL_REGISTERS_ADDR_LEN-1:0] assign_logical_regs,
    input wire [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] assign_physical_regs,

    input wire [SUBMIT_PORTS-1:0] submit_regs_enable,
    input wire [SUBMIT_PORTS-1:0][LOGICAL_REGISTERS_ADDR_LEN-1:0] submit_logical_regs,
    input wire [SUBMIT_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] submit_physical_regs,

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],
    input wire stage_t [DEPTH-1:0] current_status,
    input wire [ROB_ADDR_WIDTH-1:0] head,

    output logic [NUM_PHYSICAL_REGISTERS-1:0] reg_valid,

    input wire [EXE_WRITE_PORTS-1:0] exe_wr_enable,
    input wire [EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] exe_wr_physical_addr,

    input wire [MEM_WRITE_PORTS-1:0] mem_wr_enable,
    input wire [MEM_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] mem_wr_physical_addr,  

    output logic [OF_PORT-1:0][ROB_ADDR_WIDTH-1:0] of_ports_available,    //delivered ports for OF stage
    output logic [OF_PORT-1:0] of_enable   //status of the delivered ports for OF stage   
);
    import signals::*;

    logic [NUM_PHYSICAL_REGISTERS-1:0] reg_in_use;
    logic [NUM_PHYSICAL_REGISTERS-1:0] reg_in_use_input;

    logic [NUM_PHYSICAL_REGISTERS-1:0] reg_in_use_input_ahead;

    logic [NUM_PHYSICAL_REGISTERS-1:0] reg_valid_input;

    logic [NUM_PHYSICAL_REGISTERS-1:0] reg_valid_input_ahead;

    logic [NUM_PHYSICAL_REGISTERS-1:0] submitted_reg_in_use;
    logic [NUM_PHYSICAL_REGISTERS-1:0] submitted_reg_in_use_input;

    
    assign reg_in_use[0] = 1'b1;
    assign reg_valid[0] = 1'b1;
    assign submitted_reg_in_use[0] = 1'b1;

    genvar i;
    generate
        for (i = 1; i < NUM_PHYSICAL_REGISTERS; i++) begin: gen_assign
            assign reg_in_use[i] = reg_in_use_input[i] | reg_in_use_input_ahead[i];
            assign reg_valid[i] = reg_valid_input[i] | reg_valid_input_ahead[i];
            assign submitted_reg_in_use[i] = submitted_reg_in_use_input[i];
        end
    endgenerate

    always_comb begin
        reg_in_use_input_ahead = 'b0;
        for (int i=0; i< ASSIGN_PORTS; i++) begin
            if (assign_regs_enable[i]) begin
                reg_in_use_input_ahead[assign_physical_regs[i]] = 1'b1;
            end
        end
    end


    always_comb begin
        reg_valid_input_ahead = 'b0;
        for (int i = 0; i< EXE_WRITE_PORTS; i++) begin
            if (exe_wr_enable[i]) begin
                reg_valid_input_ahead[exe_wr_physical_addr[i]] = 1'b1;
            end
        end        

        for (int i = 0; i< MEM_WRITE_PORTS; i++) begin
            if (mem_wr_enable[i]) begin
                reg_valid_input_ahead[mem_wr_physical_addr[i]] = 1'b1;
            end
        end  
    end

    int current_port;

    always_comb begin  //block for output available ports
        // Set output_enable and output_ports
        for (int i = 0; i < ASSIGN_PORTS; i++) begin
            available_regs_enable[i] = 1'b0;
            available_physical_regs[i] = 'b0;
        end

        current_port = 0;

        // Check from head to DEPTH
        for (int i = 0; i < NUM_PHYSICAL_REGISTERS; i++) begin
            if (!reg_in_use[i]) begin
                if (current_port == ASSIGN_PORTS)
                    break;

                available_regs_enable[current_port] = 1'b1;
                available_physical_regs[current_port] = i;
                current_port += 1;
            end
        end
    end


    logic [DEPTH-1:0] is_at_of;
    logic [DEPTH-1:0] is_at_exe; 
    logic [DEPTH-1:0] is_at_mem;
    logic [DEPTH-1:0] is_at_wb;

    logic [DEPTH-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] read1_conflict_control;
    logic [DEPTH-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] read2_conflict_control;
    logic [DEPTH-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] write1_conflict_control;


    generate
        for (genvar i3 = 0; i3 < DEPTH; i3++) begin : gen_control_signals
            assign is_at_of[i3]=(current_status[i3] == OF);
            assign is_at_exe[i3]=(current_status[i3] == EXE);
            assign is_at_mem[i3]=(current_status[i3] == MEM);
            assign is_at_wb[i3]=(current_status[i3] == WB);
            assign read1_conflict_control[i3]=entries_o[i3].id_signals.rr_a;
            assign read2_conflict_control[i3]=entries_o[i3].id_signals.rr_b;
            assign write1_conflict_control[i3]=entries_o[i3].id_signals.rr_dst;
        end
    endgenerate

    int current_of_output_port;

    always_comb begin   //let go OF:
        logic [DEPTH-1:0] output_readenable;

        logic [DEPTH-1:0] output_readenable_a;
        logic [DEPTH-1:0] output_readenable_b;

        current_of_output_port = 0;

        for (int i = 0; i < DEPTH; i++) begin
            output_readenable[i] = 1'b0;
            output_readenable_a[i] = 1'b0;
            output_readenable_b[i] = 1'b0;
        end

        for (int i = 0; i < DEPTH; i++) begin
            if (is_at_of[i]) begin
                output_readenable_a[i] = reg_valid[entries_o[i].id_signals.src_rf_tag_a];
                output_readenable_b[i] = reg_valid[entries_o[i].id_signals.src_rf_tag_b];
                output_readenable[i] = output_readenable_a[i] && output_readenable_b[i];
            end
        end

        // Set output_enable and output_ports
        for (int i = 0; i < OF_PORT; i++) begin
            of_enable[i] = 0;
            of_ports_available[i] = 0;
        end

        for (int i = 0; i < DEPTH; i++) begin
            int new_index = (i + head) % DEPTH; // Calculate new index
            if (output_readenable[new_index]) begin
                if (current_of_output_port == OF_PORT)
                    break;

                of_enable[current_of_output_port] = 1;
                of_ports_available[current_of_output_port] = new_index;
                current_of_output_port += 1;
            end
        end
    end


    logic [NUM_LOGICAL_REGISTERS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] submit_table;
    // assign latest_table_out = latest_table_in;
    // assign submitted_table_in = submitted_table_out;

    // Write operation to the mapping table
    always_ff @(posedge clk) begin
        if (reset) begin
            // Reset the mapping table
            for (int i = 0; i < NUM_LOGICAL_REGISTERS; i++) begin
                latest_table_out[i] = '0;
                submit_table[i] = '0;
            end

            for (int i = 0; i < NUM_PHYSICAL_REGISTERS; i++) begin
                reg_in_use_input[i] = 1'b0;
                reg_valid_input[i] = 1'b0;
                submitted_reg_in_use_input[i] = 1'b0;
            end
        end else begin
            reg_valid_input = reg_valid;
            reg_in_use_input = reg_in_use;
            //first of all, simply submit used reg.
            for (int i=0; i< SUBMIT_PORTS; i++) begin
                if (submit_regs_enable[i]) begin
                    //$display("%d %d %d", i,submit_table[submit_logical_regs[i]], submit_physical_regs[i]);
                    reg_in_use_input[submit_table[submit_logical_regs[i]]] = 1'b0;

                    submitted_reg_in_use_input[submit_table[submit_logical_regs[i]]] = 1'b0;

                    reg_valid_input[submit_table[submit_logical_regs[i]]] = 1'b0;

                    submit_table[submit_logical_regs[i]] = submit_physical_regs[i];

                    submitted_reg_in_use_input[submit_physical_regs[i]] = 1'b1;

                    reg_valid_input[submit_physical_regs[i]] = 1'b1;
                end
            end

            // for (int i=0; i< ASSIGN_PORTS; i++) begin
            //     if (assign_regs_enable[i]) begin
            //         // reg_in_use_input_ahead[assign_physical_regs[i]] = 1'b1;
            //         //$display("%d %d", i, assign_physical_regs[i]);
            //     end
            // end
            // $display("assign end");
            // re map all existing valid regs.
            if (re_map) begin
                // $display("REMAPPPPPPPPPPPPPPPPPPPP");
                reg_in_use_input = submitted_reg_in_use_input;
                reg_valid_input = submitted_reg_in_use_input;

                for (int i = 0; i < NUM_LOGICAL_REGISTERS; i++) begin
                    latest_table_out[i] = submit_table[i];
                end
                for (int i=0; i<DEPTH; i++) begin
                    int tmp_idx = (i+head) % DEPTH;
                    if ((current_status[tmp_idx] != IF) && (current_status[tmp_idx] != ID) && (current_status[tmp_idx] != IF2)) begin
                        logic [LOGICAL_REGISTERS_ADDR_LEN-1:0] logical_addr;
                        logic [PHYSICAL_REGISTERS_ADDR_LEN-1:0] physical_addr;
                        logical_addr = entries_o[tmp_idx].id_signals.rr_dst;
                        physical_addr = entries_o[tmp_idx].id_signals.dst_rf_tag;

                        reg_in_use_input[latest_table_out[logical_addr]] = 1'b0;
                        latest_table_out[logical_addr] = physical_addr;
                        reg_in_use_input[physical_addr] = 1'b1;
                    end
                end
            end

        end
    end




endmodule