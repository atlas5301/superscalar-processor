module pc_gen_pipeline #(
    parameter DEPTH = 64,
    parameter ADDR_WIDTH = 6,
    parameter PC_WIDTH = 32,
    parameter IF_PORT = 2,
    parameter int BTB_SIZE = 8
)(
    input wire clk,
    input wire reset,
    input wire [DEPTH-1:0] mask,
    input wire [DEPTH-1:0] new_if_mask,
    input wire [ADDR_WIDTH-1:0] head,
    input wire refill,

    output logic [DEPTH-1:0][PC_WIDTH-1:0] pc,
    output logic [DEPTH-1:0] pc_ready,
    output logic [DEPTH-1:0] is_branch,
    

    input wire if_clear_signal, 
    input wire [ADDR_WIDTH-1:0] if_set_pt,
    input wire [PC_WIDTH-1:0] if_next_pc, 

    input wire id_clear_signal, 
    input wire [ADDR_WIDTH-1:0] id_set_pt,
    input wire [PC_WIDTH-1:0] id_next_pc,  

    input wire exe_clear_signal, 
    input wire [ADDR_WIDTH-1:0] exe_set_pt,
    input wire [PC_WIDTH-1:0] exe_next_pc,    

    input wire mem_clear_signal, 
    input wire [ADDR_WIDTH-1:0] mem_set_pt,
    input wire [PC_WIDTH-1:0] mem_next_pc,


    input wire branch_prediction_en,
    input wire [PC_WIDTH-1:0] branch_prediction_pc,
    input wire [PC_WIDTH-1:0] branch_prediction_bias,
    input wire branch_prediction_taken,
    input wire clear_btb
);

    // function logic [PC_WIDTH-1:0] next_pc_gen(
    //     input logic [PC_WIDTH-1:0] tmppc
    // );
    //     logic next_pc = tmppc + 4;
    //     return next_pc;
    // endfunction

    typedef enum logic[1:0] {STRONG_UNTAKEN, WEAK_UNTAKEN, WEAK_TAKEN, STRONG_TAKEN} branch_predict_t;

    typedef struct packed {
        logic enable;
        logic [PC_WIDTH-1:0] pc;
        logic [PC_WIDTH-1:0] target;
        branch_predict_t prediction;
    } btb_entry_t;

    btb_entry_t [BTB_SIZE-1:0] btb_table;

    // logic [ADDR_WIDTH-1:0] tmphead;
    logic [DEPTH-1:0][PC_WIDTH-1:0] next_pc;
    logic [DEPTH-1:0] inside_mask;

    // logic [DEPTH-1:0] output_mask;

    logic [DEPTH-1:0] head_mask;
    logic new_if_mask_tmp;
    logic is_exist_in_btb;

    logic [IF_PORT-1:0] BTB_query_enable;
    logic [IF_PORT-1:0][ADDR_WIDTH-1:0] BTB_query_entry;
    logic [IF_PORT-1:0][PC_WIDTH-1:0] BTB_query_PC;

    
    logic [IF_PORT-1:0] BTB_query_output_enable;
    logic [IF_PORT-1:0][PC_WIDTH-1:0] BTB_query_output_next_pc;
    logic [IF_PORT-1:0][PC_WIDTH-1:0] BTB_query_output_pc;
    logic [IF_PORT-1:0] BTB_query_output_is_branch;
    logic [IF_PORT-1:0][ADDR_WIDTH-1:0] BTB_query_output_query_entry;

    // logic [2*IF_PORT-1:0] candidates_BTB_enable;
    // logic [2*IF_PORT-1:0][ADDR_WIDTH-1:0] candidates_BTB_query_entry;
    // logic [2*IF_PORT-1:0][PC_WIDTH-1:0] candidates_BTB_query_PC;


    assign pc_ready = ~inside_mask & ~new_if_mask_tmp;

    always_comb begin
        for (int i=0; i<IF_PORT;i++) begin
            BTB_query_output_enable[i] = BTB_query_enable[i];
            BTB_query_output_query_entry[i] = BTB_query_entry[i];
            BTB_query_output_next_pc[i] = BTB_query_PC[i] + 4;
            BTB_query_output_pc[i] = BTB_query_PC;
            BTB_query_output_is_branch[i] = 1'b0;

            for (int j=0; j<BTB_SIZE; j++) begin
                if (btb_table[j].enable) begin
                    if (btb_table[j].pc == BTB_query_PC[i]) begin
                        if ((btb_table[j].prediction == WEAK_TAKEN) || (btb_table[j].prediction == STRONG_TAKEN)) begin
                            BTB_query_output_next_pc[i] = btb_table[j].target;
                            BTB_query_output_is_branch[i] = 1'b1;
                        end else begin
                            // next_info[i].PC = tmppc + i*4 + 4;
                            // next_info[i].is_branch = 1'b0;
                        end
                        // next_info[i].mask = 1'b0;
                    end
                end
            end
        end
    end


    always_comb begin
        for (int j=0; j<IF_PORT;j++) begin
            BTB_query_enable[j] = 0;
            BTB_query_entry[j] = 0;
            BTB_query_PC[j] = 0;
        end

        for (int i=0; i<DEPTH; i++) begin
            if (inside_mask[i] & ~inside_mask[(i+DEPTH-1)%DEPTH]) begin
                for (int j=0;j<IF_PORT;j++) begin
                    BTB_query_enable[j] = inside_mask[(i+j)%DEPTH];
                    BTB_query_entry[j] = (i+j)%DEPTH;
                    BTB_query_PC[j] = next_pc[(i-1 + DEPTH)%DEPTH] + j*4;
                end
                break;
            end                     
        end
    end


    logic [BTB_SIZE-1:0] new_btb_update_mask;
    always_ff @(posedge clk) begin
        if (reset | clear_btb) begin
            for (int i=0; i< BTB_SIZE; i++) begin
                btb_table[i].enable = 1'b0;
                btb_table[i].pc = 'b0;
                btb_table[i].target = 'b0;
                btb_table[i].prediction = WEAK_UNTAKEN;
            end
            new_btb_update_mask = 1;
        end else begin
            if (branch_prediction_en) begin
                is_exist_in_btb = 1'b0;
                for (int j=0; j<BTB_SIZE; j++) begin
                    if (btb_table[j].enable && btb_table[j].pc == branch_prediction_pc) begin
                        is_exist_in_btb = 1'b1;
                        if (branch_prediction_taken) begin
                            case(btb_table[j].prediction)
                                STRONG_TAKEN: btb_table[j].prediction <= STRONG_TAKEN;
                                WEAK_TAKEN: btb_table[j].prediction <= STRONG_TAKEN;
                                WEAK_UNTAKEN: btb_table[j].prediction <= WEAK_TAKEN;
                                STRONG_UNTAKEN: btb_table[j].prediction <= WEAK_UNTAKEN;
                                default: btb_table[j].prediction <= WEAK_TAKEN;
                            endcase
                        end else begin
                            case(btb_table[j].prediction)
                                STRONG_TAKEN: btb_table[j].prediction <= WEAK_TAKEN;
                                WEAK_TAKEN: btb_table[j].prediction <= WEAK_UNTAKEN;
                                WEAK_UNTAKEN: btb_table[j].prediction <= STRONG_UNTAKEN;
                                STRONG_UNTAKEN: btb_table[j].prediction <= STRONG_UNTAKEN;
                                default: btb_table[j].prediction <= WEAK_TAKEN;
                            endcase
                        end
                    end
                end

                // if not update, then insert a new entry
                if (~is_exist_in_btb) begin
                    for (int j=0; j<BTB_SIZE; j++) begin
                        if (new_btb_update_mask[j]) begin
                            btb_table[j].enable <= 1'b1;
                            btb_table[j].pc <= branch_prediction_pc;
                            btb_table[j].target <= branch_prediction_pc + branch_prediction_bias;
                            btb_table[j].prediction <= branch_prediction_taken ? WEAK_TAKEN : WEAK_UNTAKEN;
                        end
                    end

                    for (int j=0; j<BTB_SIZE; j++) begin
                        new_btb_update_mask[(j+1)%BTB_SIZE] <= new_btb_update_mask[j];
                    end
                end
                //now, no empty entries found

            end
        end
    end



    always_ff @(posedge clk) begin
        head_mask <= 'b1;
        head_mask[head] <= 1'b0;
        // tmphead <= head;
        new_if_mask_tmp <= new_if_mask;
        if (reset) begin     
            is_branch <= 'b0;   
            for (int i = 0; i < DEPTH; i++) begin
                pc[i] = 32'h8000_0000;
                next_pc[i] = 32'h80000004;
                inside_mask[i] = 1'b1;
            end
            inside_mask[0] = 1'b0;

            // output_mask = inside_mask;
            //is_branch <= 'b0;
            // next_pc[0] = 'b100;
            // inside_mask[head] = 1'b0;

        end else begin
            if (refill) begin
                inside_mask = inside_mask | mask;
                // output_mask = inside_mask;

                if (if_clear_signal) begin
                    next_pc[if_set_pt]=if_next_pc;
                end
                if (id_clear_signal) begin
                    next_pc[id_set_pt]=id_next_pc;
                end
                if (exe_clear_signal) begin
                    next_pc[exe_set_pt]=exe_next_pc;
                end
                if (mem_clear_signal) begin
                    next_pc[mem_set_pt]=mem_next_pc;
                end 

            end else begin
                // for (int j=0;j<IF_PORT;j++) begin
                    // inside_mask[tmphead] = 1'b0;
                //inside_mask = inside_mask & head_mask;

                for (int j=0; j< IF_PORT; j++) begin
                    if (BTB_query_output_enable[j]) begin
                        inside_mask[BTB_query_output_query_entry[j]] = 1'b0;
                        next_pc[BTB_query_output_query_entry[j]] = BTB_query_output_next_pc[j];
                        pc[(BTB_query_output_query_entry[j])] = BTB_query_output_pc[j];
                        is_branch[BTB_query_output_query_entry[j]] = BTB_query_output_is_branch[j];
                        if (BTB_query_output_is_branch[j]) begin
                            break;
                        end
                    end

                end

                // inside_mask = {inside_mask[DEPTH-2:0], inside_mask[DEPTH-1]};
                // inside_mask[tmphead] = 1'b0;
                //inside_mask = inside_mask & head_mask;
                // end          
            end
            //$display("first:%b", new_if_mask);
            inside_mask = inside_mask | new_if_mask;
            // output_mask = output_mask | new_if_mask;

            //$display("next:%b", inside_mask);

        


        end
    end


endmodule



