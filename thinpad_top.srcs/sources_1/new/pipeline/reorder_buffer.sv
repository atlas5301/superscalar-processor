import signals::*;

module ReorderBuffer_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter NUM_REGS = 32,
    parameter IF_PORT = 2,
    parameter ID_PORT = 2,
    parameter OF_PORT = 2,
    parameter EXE_PORT = 2,
    parameter MEM_PORT = 2,
    parameter WB_PORT = 2
) (
    // Clock and reset
    input wire clk,
    input wire reset,

    // Interface with IF stage
    output logic [IF_PORT-1:0][ROB_ADDR_WIDTH-1:0] if_ports_available,    //delivered ports for IF stage
    output logic [IF_PORT-1:0] if_enable,   //status of the delivered ports for IF stage
    input wire if_clear_signal,     //whether instructions should be emptied from buffer.
    input wire [DEPTH-1:0] if_clear_mask,    //the address of instructions in ROB to be emptied.
    input wire [ROB_ADDR_WIDTH-1:0] if_set_pt,   //the entry to set PC
    input wire [PC_WIDTH-1:0] if_next_pc,    //value of PC
    input wire is_if_ready,     // is IF ready for restart after pipeline reset
    input wire if_signals_t if_entries_i [DEPTH-1:0],   //IF generated signals
    input wire stage_t [DEPTH-1:0] current_status_if,   //IF status updates
    input wire [DEPTH-1:0] current_status_if_enable,   //IF status updates enable
    output logic [DEPTH-1:0][PC_WIDTH-1:0] PC_IF,
    output logic [DEPTH-1:0] PC_ready,
    output logic [DEPTH-1:0] is_branch,
    


    // Interface with ID stage
    output logic [ID_PORT-1:0][ROB_ADDR_WIDTH-1:0] id_ports_available,    //delivered ports for ID stage
    output logic [ID_PORT-1:0] id_enable,   //status of the delivered ports for ID stage
    input wire id_clear_signal,     //whether instructions should be emptied from buffer.
    input wire [DEPTH-1:0] id_clear_mask,    //the address of instructions in ROB to be emptied.
    input wire [ROB_ADDR_WIDTH-1:0] id_set_pt,   //the entry to set next_PC
    input wire [PC_WIDTH-1:0] id_next_pc,    //value of next_PC
    input wire is_id_ready,     // is ID ready for restart after pipeline reset
    input wire id_signals_t id_entries_i [DEPTH-1:0],   //ID generated signals
    input wire stage_t [DEPTH-1:0] current_status_id,   //ID status updates
    input wire [DEPTH-1:0] current_status_id_enable,   //ID status updates enable

    // Interface with OF(Operands Fetch) stage, NEED ROB REG CONFLIC CONTROL.
    input wire of_stall,     //whether the pipeline should be stalled.
    input wire is_of_ready,     // is OF ready for restart after pipeline reset
    input wire of_signals_t of_entries_i [DEPTH-1:0],   //OF generated signals
    input wire stage_t [DEPTH-1:0] current_status_of,   //OF status updates
    input wire [DEPTH-1:0] current_status_of_enable,   //OF status updates enable

    // Interface with EXE stage, CONFLICT CONTROL IN EXE
    // output logic [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] exe_ports_available,    //delivered ports for EXE stage
    // output logic [EXE_PORT-1:0] exe_enable,   //status of the delivered ports for EXE stage
    input wire exe_clear_signal,     //whether instructions should be emptied from buffer.
    input wire [DEPTH-1:0] exe_clear_mask,    //the address of instructions in ROB to be emptied.
    input wire [ROB_ADDR_WIDTH-1:0] exe_set_pt,   //the entry to set next_PC
    input wire [PC_WIDTH-1:0] exe_next_pc,    //value of next_PC
    input wire is_exe_ready,     // is EXE ready for restart after pipeline reset
    input wire exe_signals_t exe_entries_i [DEPTH-1:0],   //EXE generated signals
    input wire stage_t [DEPTH-1:0] current_status_exe,   //EXE status updates
    input wire [DEPTH-1:0] current_status_exe_enable,   //EXE status updates enable

    // Interface with MEM stage, CONFLICT CONTROL IN MEM
    output logic [MEM_PORT-1:0][ROB_ADDR_WIDTH-1:0] mem_ports_available,    //delivered ports for MEM stage
    output logic [MEM_PORT-1:0] mem_enable,   //status of the delivered ports for MEM stage
    input wire mem_clear_signal,     //whether instructions should be emptied from buffer.
    input wire [DEPTH-1:0] mem_clear_mask,    //the address of instructions in ROB to be emptied.
    input wire [ROB_ADDR_WIDTH-1:0] mem_set_pt,   //the entry to set next_PC
    input wire [PC_WIDTH-1:0] mem_next_pc,    //value of next_PC
    input wire is_mem_ready,     // is MEM ready for restart after pipeline reset
    input wire mem_signals_t mem_entries_i [DEPTH-1:0],   //MEM generated signals
    input wire stage_t [DEPTH-1:0] current_status_mem,   //MEM status updates
    input wire [DEPTH-1:0] current_status_mem_enable,   //MEM status updates enable

    // Interface with WB stage, CONFLICT CONTROL IN WB
    output logic [WB_PORT-1:0][ROB_ADDR_WIDTH-1:0] wb_ports_available,
    output logic [WB_PORT-1:0] wb_enable,
    input wire [ROB_ADDR_WIDTH-1:0] next_head,
    input wire is_wb_ready,     // is WB ready for restart after pipeline reset
    input wire wb_signals_t wb_entries_i [DEPTH-1:0],   //WB generated signals
    input wire stage_t [DEPTH-1:0] current_status_wb,   //WB status updates
    input wire [DEPTH-1:0] current_status_wb_enable,   //WB status updates enable

    output riscv_pipeline_signals_t entries_o [DEPTH-1:0],
    output stage_t [DEPTH-1:0] current_status,
    output stage_t [DEPTH-1:0] status,
    output logic [DEPTH-1:0] is_at_exe_fast, 



    // Status signals
    output wire is_ready,
    output wire is_pipeline_stall,
    output logic [ROB_ADDR_WIDTH-1:0] head,

    input wire branch_prediction_en,
    input wire [PC_WIDTH-1:0] branch_prediction_pc,
    input wire [PC_WIDTH-1:0] branch_prediction_bias,
    input wire branch_prediction_taken,
    input wire clear_btb
);
    import signals::*;
    assign head = next_head;
    
    assign is_ready = is_if_ready & is_id_ready & is_of_ready & is_exe_ready & is_mem_ready & is_wb_ready;
    assign is_pipeline_stall = if_clear_signal | id_clear_signal | exe_clear_signal | mem_clear_signal | of_stall;
    
    generate
        for (genvar i1 = 0; i1 < DEPTH; i1++) begin : combine_signals
            assign entries_o[i1].if_signals = if_entries_i[i1];
            assign entries_o[i1].id_signals = id_entries_i[i1];
            assign entries_o[i1].of_signals = of_entries_i[i1];
            assign entries_o[i1].exe_signals = exe_entries_i[i1];
            assign entries_o[i1].mem_signals = mem_entries_i[i1];
            assign entries_o[i1].wb_signals = wb_entries_i[i1];
        end
    endgenerate


    localparam NUM_ARRAYS = 8;
    logic [NUM_ARRAYS-1:0][DEPTH-1:0] masks;
    stage_t [NUM_ARRAYS-1:0][DEPTH-1:0] arrays;

    assign masks[0] = is_pipeline_stall ? if_clear_mask | id_clear_mask | exe_clear_mask | mem_clear_mask : '{DEPTH{1'b0}};
    assign masks[6] = current_status_if_enable;
    assign masks[5] = current_status_id_enable;
    assign masks[4] = current_status_of_enable;
    assign masks[3] = current_status_exe_enable;
    assign masks[2] = current_status_mem_enable;
    assign masks[1] = current_status_wb_enable;
    assign masks[NUM_ARRAYS-1] = '{DEPTH{1'b1}};

    assign arrays[0] = '{DEPTH{IF}};
    assign arrays[6] = current_status_if;
    assign arrays[5] = current_status_id;
    assign arrays[4] = current_status_of;
    assign arrays[3] = current_status_exe;
    assign arrays[2] = current_status_mem;
    assign arrays[1] = current_status_wb;
    assign arrays[NUM_ARRAYS-1] = status;


    function stage_t get_output_status(
        input int index, 
        logic [NUM_ARRAYS-1:0][DEPTH-1:0] masks,
        stage_t [NUM_ARRAYS-1:0][DEPTH-1:0] arrays
    );
        import signals::*;
        stage_t output_status;
        output_status = IF;

        for (int j = 0; j < NUM_ARRAYS; j++) begin
            if (masks[j][index]) begin
                output_status = arrays[j][index];
                return output_status;
            end
        end
        return output_status;
    endfunction

    generate
        for (genvar i2 = 0; i2 < DEPTH; i2++) begin: output_loop
            assign current_status[i2] = get_output_status(i2, masks, arrays);
        end
    endgenerate

    // function stage_t get_output_status;
    //     input int index;
    //     begin
    //         get_output_status = IF; // Set all bits to zero initially
    //         for (int j = 0; j < NUM_ARRAYS; j++) begin
    //             if (masks[j][index]) begin
    //                 get_output_status = arrays[j][index];
    //                 return get_output_status;
    //             end
    //         end
    //     end
    // endfunction

    logic [DEPTH-1:0] is_at_if;
    logic [DEPTH-1:0] is_at_id;
    logic [DEPTH-1:0] is_at_of;
    logic [DEPTH-1:0] is_at_exe; 
    logic [DEPTH-1:0] is_at_mem;
    logic [DEPTH-1:0] is_at_wb;


    generate
        for (genvar i3 = 0; i3 < DEPTH; i3++) begin : combine_signals2
            assign is_at_if[i3]=(current_status[i3] == IF);
            assign is_at_id[i3]=(current_status[i3] == ID);
            assign is_at_of[i3]=(current_status[i3] == OF);
            assign is_at_exe[i3]=(current_status[i3] == EXE);
            assign is_at_mem[i3]=(current_status[i3] == MEM);
            assign is_at_wb[i3]=(current_status[i3] == WB);

            assign is_at_exe_fast[i3]=current_status_exe_enable[i3] ? (current_status_exe[i3] == EXE) : (status[i3] == EXE);
        end
    endgenerate


    port_select_pipeline #(
        .DEPTH(DEPTH),
        .ROB_ADDR_LEN(ROB_ADDR_WIDTH),
        .NUM_OUTPUT(IF_PORT)
    ) port_select_if (
        .head(head),
        .enable(is_at_if),
        .output_enable(if_enable),
        .output_ports(if_ports_available)
    );

    port_select_pipeline #(
        .DEPTH(DEPTH),
        .ROB_ADDR_LEN(ROB_ADDR_WIDTH),
        .NUM_OUTPUT(ID_PORT)
    ) port_select_id (
        .head(head),
        .enable(is_at_id),
        .output_enable(id_enable),
        .output_ports(id_ports_available)
    );

    // port_select_pipeline #(
    //     .DEPTH(DEPTH),
    //     .ROB_ADDR_LEN(ROB_ADDR_WIDTH),
    //     .NUM_OUTPUT(EXE_PORT)
    // ) port_select_exe (
    //     .head(head),
    //     .enable(is_at_exe),
    //     .output_enable(exe_enable),
    //     .output_ports(exe_ports_available)
    // );

    port_select_first_n_pipeline_with_ignore_mask #(
        .DEPTH(DEPTH),
        .ROB_ADDR_LEN(ROB_ADDR_WIDTH),
        .NUM_OUTPUT(MEM_PORT)
    ) port_select_mem (
        .head(head),
        .enable(is_at_mem),
        .ignore(is_at_wb),
        .output_enable(mem_enable),
        .output_ports(mem_ports_available)
    );

    port_select_first_n_pipeline #(
        .DEPTH(DEPTH),
        .ROB_ADDR_LEN(ROB_ADDR_WIDTH),
        .NUM_OUTPUT(WB_PORT)
    ) port_select_wb (
        .head(head),
        .enable(is_at_wb),
        .output_enable(wb_enable),
        .output_ports(wb_ports_available)
    );


    logic [DEPTH-1:0] new_if_mask;
    logic [ROB_ADDR_WIDTH-1:0] new_head_pc_gen;
    always_comb begin
        new_if_mask = 'b0;
        new_head_pc_gen = head;
        for (int j = 0; j < WB_PORT; j = j + 1) begin
            if (wb_enable[j]) begin
                new_if_mask[wb_ports_available[j]] = 1'b1;
                new_head_pc_gen = (wb_ports_available[j]+1)%DEPTH;
            end else begin
                break;
            end
        end
    end

    pc_gen_pipeline #(
        .DEPTH(DEPTH),
        .ADDR_WIDTH(ROB_ADDR_WIDTH),
        .PC_WIDTH(PC_WIDTH),
        .IF_PORT(IF_PORT)
    ) PC_gen_inst (
        .clk(clk),
        .reset(reset),
        .new_if_mask(new_if_mask),
        .mask(masks[0]),
        .head(new_head_pc_gen),
        .refill(is_pipeline_stall && ~is_ready),

        .pc(PC_IF),
        .pc_ready(PC_ready),
        .is_branch(is_branch),
        
        .if_clear_signal(if_clear_signal),
        .if_set_pt(if_set_pt),
        .if_next_pc(if_next_pc),

        .id_clear_signal(id_clear_signal),
        .id_set_pt(id_set_pt),
        .id_next_pc(id_next_pc),

        .exe_clear_signal(exe_clear_signal),
        .exe_set_pt(exe_set_pt),
        .exe_next_pc(exe_next_pc),

        .mem_clear_signal(mem_clear_signal),
        .mem_set_pt(mem_set_pt),
        .mem_next_pc(mem_next_pc),

        .branch_prediction_en(branch_prediction_en),
        .branch_prediction_pc(branch_prediction_pc),
        .branch_prediction_bias(branch_prediction_bias),
        .branch_prediction_taken(branch_prediction_taken),
        .clear_btb(clear_btb)
    );

    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i=0; i<DEPTH;i++) begin
                status[i] <= IF;
            end
        end else begin
            status <= current_status;

            // for (int i=0; i<DEPTH;i++) begin
            //     if (entries_o[i].if_signals.PC == 32'h80000074) begin
            //         $display("inst at %d", i, current_status[i]);
            //     end
            // end
        end

    end


endmodule