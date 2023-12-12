module pc_gen_pipeline #(
    parameter DEPTH = 64,
    parameter ADDR_WIDTH = 6,
    parameter PC_WIDTH = 32,
    parameter IF_PORT = 2,
    parameter int BTB_SIZE = 3
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

    typedef struct packed {
        logic [PC_WIDTH-1:0] PC;
        logic mask;
        logic is_branch;
    } next_pc_gen_t;

    function next_pc_gen_t [IF_PORT-1:0] generate_next_pcs_and_masks(
        input logic [PC_WIDTH-1:0] tmppc,
        input logic [IF_PORT-1:0] original_mask,
        input logic [IF_PORT-1:0] is_branch,
        input logic [IF_PORT-1:0][PC_WIDTH-1:0] pc,
        input btb_entry_t [BTB_SIZE-1:0] tmpbtb_table
    );
        next_pc_gen_t [IF_PORT-1:0] next_info;
        logic is_stop_mask [IF_PORT-1:0];
        logic is_branch_mask [IF_PORT-1:0];
        logic is_stopped;


        for (int i=0; i<IF_PORT; i++) begin
            is_stop_mask[i] = 1'b0;
            is_branch_mask[i] = 1'b0;
            if (original_mask[i]) begin
                next_info[i].PC = tmppc + i*4+4;
                next_info[i].is_branch = 1'b0;
                next_info[i].mask = 1'b0;
                for (int j=0; j<BTB_SIZE; j++) begin
                    if (tmpbtb_table[j].enable) begin
                        if (tmpbtb_table[j].pc == tmppc + i*4) begin
                            is_branch_mask[i] = 1'b1;
                            if ((tmpbtb_table[j].prediction == WEAK_TAKEN) || (tmpbtb_table[j].prediction == STRONG_TAKEN)) begin
                                next_info[i].PC = tmpbtb_table[j].target;
                                next_info[i].is_branch = 1'b1;
                            end else begin
                                // next_info[i].PC = tmppc + i*4 + 4;
                                // next_info[i].is_branch = 1'b0;
                            end
                            // next_info[i].mask = 1'b0;
                        end
                    end
                end
            end else begin
                is_stop_mask[i] = 1'b1;
            end
        end

        is_stopped = 1'b0;
        for (int i=0; i<IF_PORT; i++) begin
            if (is_stop_mask[i] == 1'b1) is_stopped = 1'b1;
            if (is_stopped) begin
                next_info[i].mask = original_mask[i];
                next_info[i].PC = pc[i];
                next_info[i].is_branch = is_branch[i];
            end
            if (is_branch_mask[i] == 1'b1) is_stopped = 1'b1;
        end
        return next_info;

    endfunction

    next_pc_gen_t [IF_PORT-1:0] next_info;


    logic [ADDR_WIDTH-1:0] tmphead;
    logic [DEPTH-1:0][PC_WIDTH-1:0] next_pc;
    logic [DEPTH-1:0] inside_mask;
    logic [DEPTH-1:0] head_mask;
    logic new_if_mask_tmp;
    logic is_exist_in_btb;

    assign pc_ready = ~inside_mask & ~new_if_mask_tmp;


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
        tmphead <= head;
        new_if_mask_tmp <= new_if_mask;
        if (reset) begin     
            is_branch <= 'b0;   
            for (int i = 0; i < DEPTH; i++) begin
                pc[i] = 32'h8000_0000;
                next_pc[i] = 32'h80000004;
                inside_mask[i] = 1'b1;
            end
            inside_mask[0] = 1'b0;
            //is_branch <= 'b0;
            // next_pc[0] = 'b100;
            // inside_mask[head] = 1'b0;

        end else begin
            if (refill) begin
                inside_mask = inside_mask | mask;
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
                //$display("inside_mask", inside_mask);
                // inside_mask = {inside_mask[DEPTH-2:0], inside_mask[DEPTH-1]};
                // inside_mask[head] = 1'b0;
                // for (int j=0;j<IF_PORT;j++) begin
                //     inside_mask[head] = 1'b0;
                //     for (int i=0; i<DEPTH; i++) begin
                //         if (inside_mask[i] & ~inside_mask[(i+DEPTH-1)%DEPTH]) begin
                //             pc[i] = next_pc[(i+DEPTH-1)%DEPTH];
                //             next_pc[i] = pc[i] + 4;
                //             // next_pc[i] = next_pc_gen(pc[i]);
                //         end                     
                //     end
                //     inside_mask = {inside_mask[DEPTH-2:0], inside_mask[DEPTH-1]};
                //     inside_mask[head] = 1'b0;
                // end    

            end else begin
                // for (int j=0;j<IF_PORT;j++) begin
                    // inside_mask[tmphead] = 1'b0;
                //inside_mask = inside_mask & head_mask;
                for (int i=0; i<DEPTH; i++) begin
                    if (inside_mask[i] & ~inside_mask[(i+DEPTH-1)%DEPTH]) begin
                        logic [IF_PORT-1:0] tmp_mask;
                        logic [IF_PORT-1:0] tmp_branch;
                        logic [IF_PORT-1:0][PC_WIDTH-1:0] tmppcs;

                        for (int j=0;j<IF_PORT;j++) begin
                            tmp_mask[j] = inside_mask[(i+j)%DEPTH];
                            tmppcs[j] = next_pc[(i+j)%DEPTH];
                            tmp_branch[j] = is_branch[(i+j)%DEPTH];
                        end

                        next_info = generate_next_pcs_and_masks(
                        next_pc[(i+DEPTH-1)%DEPTH], 
                        tmp_mask,
                        tmp_branch,
                        tmppcs,
                        btb_table
                        );
                        pc[(i+DEPTH)%DEPTH] = next_pc[(i+DEPTH-1)%DEPTH];

                        for (int j=0;j<IF_PORT;j++) begin
                            next_pc[(i+j)%DEPTH] = next_info[j].PC;
                            is_branch[(i+j)%DEPTH] = next_info[j].is_branch;
                        end

                        for (int j=1;j<IF_PORT;j++) begin
                            if (inside_mask[(i+j)%DEPTH]) begin
                                pc[(i+j)%DEPTH] = next_pc[(i+j-1)%DEPTH];
                            end
                        end

                        for (int j=0;j<IF_PORT;j++) begin
                            inside_mask[(i+j)%DEPTH] = next_info[j].mask;
                        end
                        break;


                    end                     
                end
                // inside_mask = {inside_mask[DEPTH-2:0], inside_mask[DEPTH-1]};
                // inside_mask[tmphead] = 1'b0;
                //inside_mask = inside_mask & head_mask;
                // end          
            end
            //$display("first:%b", new_if_mask);
            inside_mask = inside_mask | new_if_mask;

            //$display("next:%b", inside_mask);

        


        end
    end


endmodule



