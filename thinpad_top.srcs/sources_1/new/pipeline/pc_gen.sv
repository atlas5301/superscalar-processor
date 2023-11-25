module pc_gen_pipeline #(
    parameter DEPTH = 64,
    parameter ADDR_WIDTH = 6,
    parameter PC_WIDTH = 32,
    parameter IF_PORT = 2
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
    input wire [PC_WIDTH-1:0] mem_next_pc  
);

    // function logic [PC_WIDTH-1:0] next_pc_gen(
    //     input logic [PC_WIDTH-1:0] tmppc
    // );
    //     logic next_pc = tmppc + 4;
    //     return next_pc;
    // endfunction
    typedef struct packed {
        logic [31:0] PC;
        logic mask;
        logic is_branch;
    } next_pc_gen_t;

    function next_pc_gen_t [IF_PORT-1:0] generate_next_pcs_and_masks(
        input logic [PC_WIDTH-1:0] tmppc,
        input logic [IF_PORT-1:0] original_mask,
        input logic [IF_PORT-1:0] is_branch,
        input logic [IF_PORT-1:0][PC_WIDTH-1:0] pc
    );
        next_pc_gen_t [IF_PORT-1:0] next_info;
        for (int i=0; i<IF_PORT; i++) begin
            next_info[i].mask = original_mask[i];
            next_info[i].PC = pc[i];
            next_info[i].is_branch = is_branch[i];
        end
        for (int i=0; i<IF_PORT; i++) begin
            if (original_mask[i]) begin
                next_info[i].PC = tmppc + i*4+4;
                next_info[i].is_branch = 1'b0;
                next_info[i].mask = 1'b0;
                if (next_info[i].is_branch) begin
                    return next_info;
                end
            end else begin
                return next_info;
            end
        end
        return next_info;

    endfunction

    next_pc_gen_t [IF_PORT-1:0] next_info;


    logic [ADDR_WIDTH-1:0] tmphead;
    // logic [DEPTH-1:0][PC_WIDTH-1:0] next_pc;
    logic [DEPTH-1:0] inside_mask;
    logic [DEPTH-1:0] head_mask;
    logic new_if_mask_tmp;

    assign pc_ready = ~inside_mask & ~new_if_mask_tmp;

    always_ff @(posedge clk) begin
        head_mask <= 'b1;
        head_mask[head] <= 1'b0;
        tmphead <= head;
        new_if_mask_tmp <= new_if_mask;
        if (reset) begin     
            is_branch <= 'b0;   
            for (int i = 0; i < DEPTH; i++) begin
                pc[i] = 32'h8000_0000;
                inside_mask[i] = 1'b1;
            end
            inside_mask[0] = 1'b0;
            is_branch = 'b0;
            // next_pc[0] = 'b100;
            // inside_mask[head] = 1'b0;

        end else begin
            if (refill) begin
                inside_mask = inside_mask | mask;
                if (if_clear_signal) begin
                    pc[(if_set_pt+1)%DEPTH]=if_next_pc;
                end
                if (id_clear_signal) begin
                    pc[(id_set_pt+1)%DEPTH]=id_next_pc;
                end
                if (exe_clear_signal) begin
                    pc[(exe_set_pt+1)%DEPTH]=exe_next_pc;
                end
                if (mem_clear_signal) begin
                    pc[(mem_set_pt+1)%DEPTH]=mem_next_pc;
                end

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
                            tmppcs[j] = pc[(i+j)%DEPTH];
                            tmp_branch[j] = is_branch[(i+j)%DEPTH];
                        end

                        next_info = generate_next_pcs_and_masks(
                        pc[(i+DEPTH-1)%DEPTH], 
                        tmp_mask,
                        tmp_branch,
                        tmppcs
                        );

                        for (int j=0;j<IF_PORT;j++) begin
                            inside_mask[(i+j)%DEPTH] = next_info[j].mask;
                            pc[(i+j)%DEPTH] = next_info[j].PC;
                            is_branch[(i+j)%DEPTH] = next_info[j].is_branch;
                        end
                        break;

                    end                     
                end
                // inside_mask = {inside_mask[DEPTH-2:0], inside_mask[DEPTH-1]};
                // inside_mask[tmphead] = 1'b0;
                //inside_mask = inside_mask & head_mask;
                // end          
            end

            inside_mask <= inside_mask | new_if_mask;

        


        end
    end


endmodule



