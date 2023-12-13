import signals::*;

module of_module_pipeline #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter OF_PORT = 2,
    parameter REG_DATA_WIDTH = 32,         
    parameter REG_ADDR_WIDTH = 5           
) (

    input wire clk,
    input wire reset,

    input wire [OF_PORT-1:0][ROB_ADDR_WIDTH-1:0] of_ports_available,    //delivered ports for OF stage
    input wire [OF_PORT-1:0] of_enable,   //status of the delivered ports for OF stage 
    output logic of_stall,     //whether the pipeline should be stalled.  
    output logic is_of_ready,     // is OF ready for restart after pipeline reset
    output of_signals_t of_entries_i [DEPTH-1:0],   //OF generated signals
    output stage_t [DEPTH-1:0] current_status_of,   //OF status updates
    output logic [DEPTH-1:0] current_status_of_enable,   //OF status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    // Status signals
    input wire is_ready,
    input wire is_pipeline_stall,

    output reg [2*OF_PORT-1:0][REG_ADDR_WIDTH-1:0] rd_addr,
    input wire [2*OF_PORT-1:0][REG_DATA_WIDTH-1:0] rd_data
);
    import signals::*;
    logic [DEPTH-1:0] mask_of;
    logic [DEPTH-1:0] mask_of2;
    logic [OF_PORT-1:0] enable_addr_of;
    logic [OF_PORT-1:0] enable_addr_of2;   
    logic [OF_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_of;
    logic [OF_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_of2;
    

    // assign current_status_of_enable = mask_of | mask_of2;

    // always_comb begin
    //     mask_of = 'b0;
    //     mask_of2 = 'b0;
    //     for (int j = 0; j < OF_PORT; j = j + 1) begin
    //         if (enable_addr_of[j]) begin
    //             mask_of[addr_of[j]] = 1'b1;
    //         end
    //         if (enable_addr_of2[j]) begin
    //             mask_of2[addr_of2[j]] = 1'b1;
    //         end
    //     end
    // end

    logic is_stall_cycle2;

    always_ff @(posedge clk) begin
        current_status_of_enable <= mask_of | mask_of2;
        if (reset) begin
            is_of_ready = 1'b0;
            enable_addr_of = 'b0;
            enable_addr_of2 = 'b0;
            is_stall_cycle2 <= 1'b0;
            of_stall <= 1'b0;
            current_status_of <= '{DEPTH{IF}};
        end else begin
            for (int i=0;i<OF_PORT;i++) begin
                if (enable_addr_of[i]) begin
                    of_entries_i[addr_of[i]].rf_rdata_a <= rd_data[i*2];
                    of_entries_i[addr_of[i]].rf_rdata_b <= rd_data[i*2+1];
                    current_status_of[addr_of[i]] <= EXE;
                end
            end
            addr_of2 = addr_of;
            enable_addr_of2 = enable_addr_of;
            enable_addr_of = 'b0;
            if (is_pipeline_stall) begin
                is_of_ready <= 1'b1;
                if (is_ready) begin
                    of_stall <= 1'b0;
                end
                // current_status_of_enable => 'b0;
            end else begin
                is_of_ready = 1'b0;

                addr_of = of_ports_available;
                enable_addr_of = of_enable;
                for (int i=0;i<OF_PORT;i++) begin
                    if (enable_addr_of[i]) begin
                        rd_addr[i*2] <= entries_o[addr_of[i]].id_signals.rr_a;
                        rd_addr[i*2+1] <= entries_o[addr_of[i]].id_signals.rr_b;
                        current_status_of[addr_of[i]] <= OF2;
                    end
                end

            end

        end
        mask_of = 'b0;
        mask_of2 = 'b0;
        for (int j = 0; j < OF_PORT; j = j + 1) begin
            if (enable_addr_of[j]) begin
                mask_of[addr_of[j]] = 1'b1;
            end
            if (enable_addr_of2[j]) begin
                mask_of2[addr_of2[j]] = 1'b1;
            end
        end
    end


endmodule




module wb_module_pipeline  #(
    parameter PC_WIDTH = 32,
    parameter DEPTH = 64,
    parameter ROB_ADDR_WIDTH = 6,
    parameter WB_PORT = 2,
    parameter REG_DATA_WIDTH = 32,         
    parameter REG_ADDR_WIDTH = 5 
) (

    input wire clk,
    input wire reset,

    input wire [WB_PORT-1:0][ROB_ADDR_WIDTH-1:0] wb_ports_available,
    input wire [WB_PORT-1:0] wb_enable,
    output logic [ROB_ADDR_WIDTH-1:0] next_head,
    output logic is_wb_ready,     // is WB ready for restart after pipeline reset
    output wb_signals_t wb_entries_i [DEPTH-1:0],   //WB generated signals
    output stage_t [DEPTH-1:0] current_status_wb,   //WB status updates
    output logic [DEPTH-1:0] current_status_wb_enable,   //WB status updates enable

    input riscv_pipeline_signals_t entries_o [DEPTH-1:0],

    // Status signals
    input wire is_ready,
    input wire is_pipeline_stall,

    output reg [WB_PORT-1:0][REG_ADDR_WIDTH-1:0] wr_addr,
    output reg [WB_PORT-1:0][REG_DATA_WIDTH-1:0] wr_data,
    output reg [WB_PORT-1:0] wr_en
);
    import signals::*;
    logic [DEPTH-1:0] mask_wb;
    logic [WB_PORT-1:0] enable_addr_wb;
    logic [WB_PORT-1:0][ROB_ADDR_WIDTH-1:0] addr_wb;

    logic [31:0] debug_PC;

    // assign current_status_wb_enable = mask_wb;

    always_comb begin
        mask_wb = 'b0;
        for (int j = 0; j < WB_PORT; j = j + 1) begin
            if (enable_addr_wb[j]) begin
                mask_wb[addr_wb[j]] = 1'b1;
            end
        end
    end


    always_ff @(posedge clk) begin
        current_status_wb_enable <= mask_wb;
        if (reset) begin
            is_wb_ready = 1'b0;
            enable_addr_wb <= 'b0;
            wr_en <= 'b0;
            current_status_wb <= '{DEPTH{IF}};
            next_head <= 0;

        end else begin
            if (is_pipeline_stall) begin
                is_wb_ready <= 1'b1;
                enable_addr_wb <= 'b0;
                wr_en <= 'b0;
            end else begin
                is_wb_ready <= 1'b0;
                addr_wb = wb_ports_available;
                enable_addr_wb = wb_enable;
                wr_en <= 'b0;
                for (int i=0;i<WB_PORT;i++) begin
                    // wr_addr[i] <= 'b0;
                    wr_en[i] <= 'b0;
                end
                for (int i=0;i<WB_PORT;i++) begin
                    if (enable_addr_wb[i]) begin
                        wr_addr[i] <= entries_o[addr_wb[i]].id_signals.rr_dst;
                        wr_data[i] <= entries_o[addr_wb[i]].mem_signals.rf_wdata_mem;
                        wr_en[i] <= 1'b1;
                        current_status_wb[addr_wb[i]] <= IF;
                        next_head <= (addr_wb[i]+1)%DEPTH;

                        debug_PC <= entries_o[addr_wb[i]].if_signals.PC;
                    end else begin
                        break;
                    end
                end
            end

        end
    end



endmodule
