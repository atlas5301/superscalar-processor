package signals;
    // Constants for alu_op
    typedef enum logic [3:0] {ADD=4'b0000, SUB=4'b0001, AND=4'b0010, OR=4'b0011, NOT=4'b0100, XOR=4'b0101} alu_ops;

    typedef enum logic [3:0] {IF, IF2, ID, OF, OF2, EXE, MEM, WB} stage_t;

    typedef enum logic [2:0] {BEQ, BNE, BGE, BGEU, BLT, BLTU} branch_t;

    // IF Stage
    typedef struct {
        logic [31:0] inst;
        logic [31:0] PC;
        logic branch_taken;
    } if_signals_t;

    // ID Stage
    typedef struct {
        logic [31:0] immediate;

        alu_ops alu_op;      // ALU operation type
        logic use_rs2;   // Flag to use rs2 or immediate
        logic mem_en;    // Memory access enable flag
        logic mem_write;    // Register file write enable flag
        logic mem_is_signed;
        logic [2:0] mem_len;   //the read/write len of mem

        logic is_pc_op;  // Flag for potential PC modification
        logic is_branch; // Flag to indicate if the instruction is a branch
        branch_t branch_op;

        // Additional signals for OOOE and superscalar support
        logic [4:0]  rr_a;          // Renamed register a
        logic [4:0]  rr_b;          // Renamed register b
        logic [4:0]  rr_dst;        // Renamed register destination
        logic [5:0]  src_rf_tag_a;  // Source register file tag for operand A
        logic [5:0]  src_rf_tag_b;  // Source register file tag for operand B
        logic [5:0]  dst_rf_tag;    // Destination register file tag

    } id_signals_t;

    // OF Stage
    typedef struct {
        logic [31:0] rf_rdata_a;    //in load/store instructions, will be used to store original addr
        logic [31:0] rf_rdata_b;    //in store instructions, will be used to store the data to write
        logic prepared_a;
        logic pending_a;
        logic prepared_b;
        logic pending_b;
    } of_signals_t;

    // EXE Stage
    typedef struct {
        logic [31:0] rf_wdata_exe;  //in load/store instructions, will be used to store final addr
    } exe_signals_t;

    // MEM Stage
    typedef struct {
        logic [31:0] rf_wdata_mem;
        // No new signals are generated; signals are propagated.
    } mem_signals_t;

    // WB Stage
    typedef struct {
        logic placeholder;
        // No signals are generated.
    } wb_signals_t;

    typedef struct{
        // IF Stage
        if_signals_t if_signals;

        // ID Stage
        id_signals_t id_signals;

        // OF Stage
        of_signals_t of_signals;

        // EXE Stage
        exe_signals_t exe_signals;

        // MEM Stage
        mem_signals_t mem_signals;

        // WB Stage
        wb_signals_t wb_signals;

    } riscv_pipeline_signals_t;

endpackage