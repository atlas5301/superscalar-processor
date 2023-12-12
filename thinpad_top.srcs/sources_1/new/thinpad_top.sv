import signals::*;
// `include "./pipeline/signals.sv"
// import signals::*;

module thinpad_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

  // `include "./pipeline/signals.sv"
  // import signals::*;
  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M, clk_40M, clk_35M, clk_30M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设置
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设置
      .clk_out3(clk_40M),
      .clk_out4(clk_35M),
      .clk_out5(clk_30M),
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出，"1"表示时钟稳定，
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end


//   logic global_clock = clk_10M;
//   logic global_reset = reset_of_clk10M;

  logic global_clock; 
//   assign global_clock = clk_20M;
//   assign global_clock = clk_10M;
  assign global_clock = clk_30M;
//   assign global_clock = clk_35M;
//   assign global_clock = clk_40M;
//   assign global_clock = clk_50M;
  logic global_reset; 
  assign global_reset = reset_of_clk10M;
//   assign global_reset = reset_btn;
  localparam int CLK_FREQ = 30_000_000;

  // 不使用内存、串口时，禁用其使能信号

  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;

  logic        wbm_cyc_o;
  logic        wbm_stb_o;
  logic        wbm_ack_i;
  logic [31:0] wbm_adr_o;
  logic [31:0] wbm_dat_o;
  logic [31:0] wbm_dat_i;
  logic [ 3:0] wbm_sel_o;
  logic        wbm_we_o;

  // lab5_master #(
  //     .ADDR_WIDTH(32),
  //     .DATA_WIDTH(32)
  // ) u_lab5_master (
  //     .clk_i(global_clock),
  //     .rst_i(global_reset),

  //     // TODO: 添加需要的控制信号，例如按键开关？
  //     .base_address_in(dip_sw),

  //     // wishbone master
  //     .wb_cyc_o(wbm_cyc_o),
  //     .wb_stb_o(wbm_stb_o),
  //     .wb_ack_i(wbm_ack_i),
  //     .wb_adr_o(wbm_adr_o),
  //     .wb_dat_o(wbm_dat_o),
  //     .wb_dat_i(wbm_dat_i),
  //     .wb_sel_o(wbm_sel_o),
  //     .wb_we_o (wbm_we_o)
  // );

    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 32;

    localparam int PC_WIDTH = 32;
    localparam int DEPTH = 16;
    localparam int ROB_ADDR_WIDTH = 4;
    localparam int NUM_REGS = 32;
    localparam int REG_ADDR_LEN = 5;
    localparam int IF_PORT = 2;
    localparam int ID_PORT = 2;
    localparam int OF_PORT = 6;
    localparam int EXE_PORT = 2;
    localparam int MEM_PORT = 2;
    localparam int WB_PORT = 2;

    localparam int CACHE_WAYS = 4;
    localparam int CACHE_WIDTH = 32;
    localparam int CACHE_ENABLE = 1;

    localparam int REG_DATA_WIDTH = DATA_WIDTH;          // Bitwidth of data
    localparam int REG_ADDR_WIDTH = REG_ADDR_LEN;           // Bitwidth of address, supports 2^N registers by default
    localparam int NUM_READ_PORTS = OF_PORT;       // Number of read ports
    localparam int NUM_WRITE_PORTS = WB_PORT;       // Number of write ports

    localparam int NUM_LOGICAL_REGISTERS = 32;
    localparam int REG_ASSIGN_SET_SIZE = 16;
    localparam int REG_ASSIGN_SET_NUM = 3; 
    localparam int NUM_PHYSICAL_REGISTERS = REG_ASSIGN_SET_NUM * REG_ASSIGN_SET_SIZE;
    localparam int LOGICAL_REGISTERS_ADDR_LEN = 5;
    localparam int PHYSICAL_REGISTERS_ADDR_LEN = 6;
    localparam int ASSIGN_PORTS = ID_PORT;
    localparam int SUBMIT_PORTS = WB_PORT;
    localparam int EXE_WRITE_PORTS = EXE_PORT;
    localparam int MEM_WRITE_PORTS = 1;


    logic enable_IF;
    logic write_IF;
    logic [ADDR_WIDTH-1:0] address_IF;
    logic [DATA_WIDTH-1:0] write_data_IF;
    logic [3:0] sel_IF;

    logic [DATA_WIDTH-1:0] read_data_IF;
    logic finished_IF;

    logic enable_MEM;
    logic write_MEM;
    logic [ADDR_WIDTH-1:0] address_MEM;
    logic [DATA_WIDTH-1:0] write_data_MEM;
    logic [3:0] sel_MEM;

    logic [DATA_WIDTH-1:0] read_data_MEM;
    logic finished_MEM;


  wishbone_controller #(
      .ADDR_WIDTH(32), // Address width
      .DATA_WIDTH(32)  // Data width
  ) wishbone_controller_inst (
      .clk(global_clock), // System clock
      .rst(global_reset), // System reset, active high

      .enable_IF(enable_IF),
      .write_IF(write_IF),
      .address_IF(address_IF),
      .write_data_IF(write_data_IF),
      .sel_IF(sel_IF),
      .read_data_IF(read_data_IF),
      .finished_IF(finished_IF),

      .enable_MEM(enable_MEM),
      .write_MEM(write_MEM),
      .address_MEM(address_MEM),
      .write_data_MEM(write_data_MEM),
      .sel_MEM(sel_MEM),
      .read_data_MEM(read_data_MEM),
      .finished_MEM(finished_MEM),

      // Wishbone interface
      .wb_cyc_o(wbm_cyc_o),
      .wb_stb_o(wbm_stb_o),
      .wb_ack_i(wbm_ack_i),
      .wb_adr_o(wbm_adr_o),
      .wb_dat_o(wbm_dat_o),
      .wb_dat_i(wbm_dat_i),
      .wb_sel_o(wbm_sel_o),
      .wb_we_o(wbm_we_o)
  );

logic [IF_PORT-1:0][ROB_ADDR_WIDTH-1:0] if_ports_available;
logic [IF_PORT-1:0] if_enable;
logic if_clear_signal;
logic [DEPTH-1:0] if_clear_mask;
logic [ROB_ADDR_WIDTH-1:0] if_set_pt;
logic [PC_WIDTH-1:0] if_next_pc;
wire is_if_ready;
if_signals_t if_entries_i [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status_if;
logic [DEPTH-1:0] current_status_if_enable;
logic [DEPTH-1:0][PC_WIDTH-1:0] PC_IF;
logic [DEPTH-1:0] PC_ready;
logic [DEPTH-1:0] is_branch;

logic i_cache_reset;

logic [ID_PORT-1:0][ROB_ADDR_WIDTH-1:0] id_ports_available;
logic [ID_PORT-1:0] id_enable;
logic id_clear_signal;
logic [DEPTH-1:0] id_clear_mask;
logic [ROB_ADDR_WIDTH-1:0] id_set_pt;
logic [PC_WIDTH-1:0] id_next_pc;
wire is_id_ready;
id_signals_t id_entries_i [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status_id;
logic [DEPTH-1:0] current_status_id_enable;

logic [OF_PORT-1:0][ROB_ADDR_WIDTH-1:0] of_ports_available;
logic [OF_PORT-1:0] of_port_is_b;
logic [OF_PORT-1:0] of_enable;
logic [DEPTH-1:0] is_not_at_of;
logic of_stall;
wire is_of_ready;
of_signals_t of_entries_i [DEPTH-1:0] ;
stage_t [DEPTH-1:0] current_status_of;
logic [DEPTH-1:0] current_status_of_enable;

logic [EXE_PORT-1:0][ROB_ADDR_WIDTH-1:0] exe_ports_available;
logic [EXE_PORT-1:0] exe_enable;
logic [EXE_PORT-1:0] exe_is_ready_a;
logic [EXE_PORT-1:0] exe_is_ready_b;
logic exe_clear_signal;
logic [DEPTH-1:0] exe_clear_mask;
logic [ROB_ADDR_WIDTH-1:0] exe_set_pt;
logic [PC_WIDTH-1:0] exe_next_pc;
wire is_exe_ready;
exe_signals_t exe_entries_i [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status_exe;
logic [DEPTH-1:0] current_status_exe_enable;

logic [MEM_PORT-1:0][ROB_ADDR_WIDTH-1:0] mem_ports_available;
logic [MEM_PORT-1:0] mem_enable;
logic mem_clear_signal;
logic [DEPTH-1:0] mem_clear_mask;
logic [ROB_ADDR_WIDTH-1:0] mem_set_pt;
logic [PC_WIDTH-1:0] mem_next_pc;
wire is_mem_ready;
mem_signals_t mem_entries_i [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status_mem;
logic [DEPTH-1:0] current_status_mem_enable;

logic [WB_PORT-1:0][ROB_ADDR_WIDTH-1:0] wb_ports_available;
logic [WB_PORT-1:0] wb_enable;
logic [ROB_ADDR_WIDTH-1:0] next_head;
wire is_wb_ready;
wb_signals_t wb_entries_i [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status_wb;
logic [DEPTH-1:0] current_status_wb_enable;

riscv_pipeline_signals_t entries_o [DEPTH-1:0];
stage_t [DEPTH-1:0] current_status;
stage_t [DEPTH-1:0] status;
logic [DEPTH-1:0] is_at_exe_fast;

wire is_ready;
wire is_pipeline_stall;
logic [ROB_ADDR_WIDTH-1:0] head;


logic [NUM_LOGICAL_REGISTERS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] latest_table_out;

logic [ASSIGN_PORTS-1:0] available_regs_enable;
logic [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] available_physical_regs;

logic [ASSIGN_PORTS-1:0] assign_regs_enable;
logic [ASSIGN_PORTS-1:0][LOGICAL_REGISTERS_ADDR_LEN-1:0] assign_logical_regs;
logic [ASSIGN_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] assign_physical_regs;

logic [SUBMIT_PORTS-1:0] submit_regs_enable;
logic [SUBMIT_PORTS-1:0][LOGICAL_REGISTERS_ADDR_LEN-1:0] submit_logical_regs;
logic [SUBMIT_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] submit_physical_regs;

logic [NUM_PHYSICAL_REGISTERS-1:0] reg_valid;

logic [EXE_WRITE_PORTS-1:0] exe_wr_enable;
logic [EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] exe_wr_physical_addr;

logic [NUM_PHYSICAL_REGISTERS-1:0] is_cached_exe;

logic [MEM_WRITE_PORTS-1:0] mem_wr_enable;
logic [MEM_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] mem_wr_physical_addr;

rename_register_mapping_table #(
    .NUM_LOGICAL_REGISTERS(NUM_LOGICAL_REGISTERS),
    .NUM_PHYSICAL_REGISTERS(NUM_PHYSICAL_REGISTERS),
    .REG_ASSIGN_SET_NUM(REG_ASSIGN_SET_NUM),
    .REG_ASSIGN_SET_SIZE(REG_ASSIGN_SET_SIZE),
    .LOGICAL_REGISTERS_ADDR_LEN(LOGICAL_REGISTERS_ADDR_LEN),
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN),
    .ASSIGN_PORTS(ASSIGN_PORTS),
    .SUBMIT_PORTS(SUBMIT_PORTS),
    .EXE_WRITE_PORTS(EXE_WRITE_PORTS),
    .MEM_WRITE_PORTS(MEM_WRITE_PORTS),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .OF_PORT(OF_PORT),
    .EXE_PORT(EXE_PORT)
) rename_register_mapping_table_inst (
    .clk(global_clock),
    .reset(global_reset),
    .re_map(is_pipeline_stall),
    .latest_table_out(latest_table_out),

    .available_regs_enable(available_regs_enable),
    .available_physical_regs(available_physical_regs),

    .assign_regs_enable(assign_regs_enable),
    // .assign_logical_regs(assign_logical_regs),
    .assign_physical_regs(assign_physical_regs),

    .submit_regs_enable(submit_regs_enable),
    .submit_logical_regs(submit_logical_regs),
    .submit_physical_regs(submit_physical_regs),

    .entries_o(entries_o),
    .current_status(current_status),
    .status(status),
    .is_at_exe_fast(is_at_exe_fast),
    .head(head),


    .reg_valid(reg_valid),

    .exe_wr_enable(exe_wr_enable),
    .exe_wr_physical_addr(exe_wr_physical_addr),

    .mem_wr_enable(mem_wr_enable),
    .mem_wr_physical_addr(mem_wr_physical_addr),  

    .of_ports_available(of_ports_available),    //delivered ports for OF stage
    .of_port_is_b(of_port_is_b),
    .of_enable(of_enable),   //status of the delivered ports for OF stage 
    .is_not_at_of(is_not_at_of),

    .exe_ports_available(exe_ports_available),    //delivered ports for EXE stage
    .exe_enable(exe_enable),   //status of the delivered ports for EXE stage  
    .exe_is_ready_a(exe_is_ready_a),
    .exe_is_ready_b(exe_is_ready_b),
    .is_cached_exe(is_cached_exe)
);




ReorderBuffer_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .NUM_REGS(NUM_REGS),
    .IF_PORT(IF_PORT),
    .ID_PORT(ID_PORT),
    .OF_PORT(OF_PORT),
    .EXE_PORT(EXE_PORT),
    .MEM_PORT(MEM_PORT),
    .WB_PORT(WB_PORT)
) ReorderBuffer_pipeline_inst (
.clk(global_clock),
.reset(global_reset),

.if_ports_available(if_ports_available),
.if_enable(if_enable),
.if_clear_signal(if_clear_signal),
.if_clear_mask(if_clear_mask),
.if_set_pt(if_set_pt),
.if_next_pc(if_next_pc),
.is_if_ready(is_if_ready),
.if_entries_i(if_entries_i),
.current_status_if(current_status_if),
.current_status_if_enable(current_status_if_enable),
.PC_IF(PC_IF),
.PC_ready(PC_ready),
.is_branch(is_branch),

.id_ports_available(id_ports_available),
.id_enable(id_enable),
.id_clear_signal(id_clear_signal),
.id_clear_mask(id_clear_mask),
.id_set_pt(id_set_pt),
.id_next_pc(id_next_pc),
.is_id_ready(is_id_ready),
.id_entries_i(id_entries_i),
.current_status_id(current_status_id),
.current_status_id_enable(current_status_id_enable),

.of_stall(of_stall),
.is_of_ready(is_of_ready),
.of_entries_i(of_entries_i),
.current_status_of(current_status_of),
.current_status_of_enable(current_status_of_enable),

// .exe_ports_available(exe_ports_available),
// .exe_enable(exe_enable),
.exe_clear_signal(exe_clear_signal),
.exe_clear_mask(exe_clear_mask),
.exe_set_pt(exe_set_pt),
.exe_next_pc(exe_next_pc),
.is_exe_ready(is_exe_ready),
.exe_entries_i(exe_entries_i),
.current_status_exe(current_status_exe),
.current_status_exe_enable(current_status_exe_enable),

.mem_ports_available(mem_ports_available),
.mem_enable(mem_enable),
.mem_clear_signal(mem_clear_signal),
.mem_clear_mask(mem_clear_mask),
.mem_set_pt(mem_set_pt),
.mem_next_pc(mem_next_pc),
.is_mem_ready(is_mem_ready),
.mem_entries_i(mem_entries_i),
.current_status_mem(current_status_mem),
.current_status_mem_enable(current_status_mem_enable),

.wb_ports_available(wb_ports_available),
.wb_enable(wb_enable),
.next_head(next_head),
.is_wb_ready(is_wb_ready),
.wb_entries_i(wb_entries_i),
.current_status_wb(current_status_wb),
.current_status_wb_enable(current_status_wb_enable),

.entries_o(entries_o),
.current_status(current_status),
.status(status),
.is_at_exe_fast(is_at_exe_fast),
.is_ready(is_ready),
.is_pipeline_stall(is_pipeline_stall),
.head(head)
);

if_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .IF_PORT(IF_PORT),
    .ADDR_WIDTH(32), // Address width
    .DATA_WIDTH(32),  // Data width
    .CACHE_WAYS(CACHE_WAYS),
    .CACHE_WIDTH(CACHE_WIDTH),
    .CACHE_ENABLE(CACHE_ENABLE)
) if_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .if_ports_available(if_ports_available),
    .if_enable(if_enable),
    .if_clear_signal(if_clear_signal),
    .if_clear_mask(if_clear_mask),
    .if_set_pt(if_set_pt),
    .if_next_pc(if_next_pc),
    .is_if_ready(is_if_ready),
    .if_entries_i(if_entries_i),
    .current_status_if(current_status_if),
    .current_status_if_enable(current_status_if_enable),
    .PC_IF(PC_IF),
    .PC_ready(PC_ready),
    .is_branch(is_branch),
    .entries_o(entries_o),
    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),
    .i_cache_reset(i_cache_reset),
    .enable_IF(enable_IF),
    .write_IF(write_IF),
    .address_IF(address_IF),
    .write_data_IF(write_data_IF),
    .sel_IF(sel_IF),
    .read_data_IF(read_data_IF),
    .finished_IF(finished_IF)
);


mem_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .MEM_PORT(MEM_PORT),
    .ADDR_WIDTH(32), // Address width
    .DATA_WIDTH(32), // Data width
    .REG_DATA_WIDTH(REG_DATA_WIDTH),
    .MEM_WRITE_PORTS(MEM_WRITE_PORTS), 
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN)
) mem_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .mem_ports_available(mem_ports_available),
    .mem_enable(mem_enable),
    .mem_clear_signal(mem_clear_signal),
    .mem_clear_mask(mem_clear_mask),
    .mem_set_pt(mem_set_pt),
    .mem_next_pc(mem_next_pc),
    .is_mem_ready(is_mem_ready),
    .mem_entries_i(mem_entries_i),
    .current_status_mem(current_status_mem),
    .current_status_mem_enable(current_status_mem_enable),
    .entries_o(entries_o),
    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),
    .enable_MEM(enable_MEM),
    .write_MEM(write_MEM),
    .address_MEM(address_MEM),
    .write_data_MEM(write_data_MEM),
    .sel_MEM(sel_MEM),
    .read_data_MEM(read_data_MEM),
    .finished_MEM(finished_MEM),

    .mem_wr_enable(mem_wr_enable),
    .mem_wr_physical_addr(mem_wr_physical_addr),

    .wr_en_mem(wr_en_mem),
    .wr_addr_mem(wr_addr_mem),
    .wr_data_mem(wr_data_mem)
);

id_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .ID_PORT(ID_PORT),
    .NUM_LOGICAL_REGISTERS(NUM_LOGICAL_REGISTERS),
    .NUM_PHYSICAL_REGISTERS(NUM_PHYSICAL_REGISTERS),
    .LOGICAL_REGISTERS_ADDR_LEN(LOGICAL_REGISTERS_ADDR_LEN),
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN),
    .ASSIGN_PORTS(ASSIGN_PORTS)
) id_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .id_ports_available(id_ports_available),
    .id_enable(id_enable),

    .id_clear_signal(id_clear_signal),
    .id_clear_mask(id_clear_mask),
    .id_set_pt(id_set_pt),
    .id_next_pc(id_next_pc),

    .is_id_ready(is_id_ready),

    .id_entries_i(id_entries_i),
    .current_status_id(current_status_id),
    .current_status_id_enable(current_status_id_enable),

    .entries_o(entries_o),

    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),

    .latest_table_out(latest_table_out),

    .available_regs_enable(available_regs_enable),
    .available_physical_regs(available_physical_regs),

    .assign_regs_enable(assign_regs_enable),
    // .assign_logical_regs(assign_logical_regs),
    .assign_physical_regs(assign_physical_regs)
);


logic [NUM_WRITE_PORTS-1:0] wr_en;
logic [NUM_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] wr_addr;
logic [NUM_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data;
logic [NUM_READ_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] rd_addr;
logic [NUM_READ_PORTS-1:0][REG_DATA_WIDTH-1:0] rd_data;

reg [EXE_WRITE_PORTS-1:0] wr_en_exe;
reg [EXE_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] wr_addr_exe;
reg [EXE_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data_exe;

reg [MEM_WRITE_PORTS-1:0] wr_en_mem;
reg [MEM_WRITE_PORTS-1:0][PHYSICAL_REGISTERS_ADDR_LEN-1:0] wr_addr_mem;
reg [MEM_WRITE_PORTS-1:0][REG_DATA_WIDTH-1:0] wr_data_mem;


register_file_pipeline #(
    .REG_DATA_WIDTH(DATA_WIDTH),          // Bitwidth of data
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN),           // Bitwidth of address, supports 2^N registers by default
    .NUM_READ_PORTS(OF_PORT),       // Number of read ports
    .NUM_WRITE_PORTS(WB_PORT),      // Number of write ports
    .EXE_WRITE_PORTS(EXE_WRITE_PORTS),
    .MEM_WRITE_PORTS(MEM_WRITE_PORTS)
) register_file_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),
    .wr_en(wr_en),
    .wr_addr(wr_addr),
    .wr_data(wr_data),
    .rd_addr(rd_addr),
    .rd_data(rd_data),

    .wr_en_exe(wr_en_exe),
    .wr_addr_exe(wr_addr_exe),
    .wr_data_exe(wr_data_exe),

    .wr_en_mem(wr_en_mem),
    .wr_addr_mem(wr_addr_mem),
    .wr_data_mem(wr_data_mem)
    // output reg [REG_DATA_WIDTH-1:0] debug
);

of_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .OF_PORT(OF_PORT),
    .REG_DATA_WIDTH(DATA_WIDTH),         
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN)    
) of_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .of_ports_available(of_ports_available),
    .of_port_is_b(of_port_is_b),
    .of_enable(of_enable),
    .is_not_at_of(is_not_at_of),
    .of_stall(of_stall),
    .is_of_ready(is_of_ready),
    .of_entries_i(of_entries_i),
    .current_status_of(current_status_of),
    .current_status_of_enable(current_status_of_enable),

    .entries_o(entries_o),

    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),

    .rd_addr(rd_addr),
    .rd_data(rd_data)
);

wb_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .WB_PORT(WB_PORT),
    .REG_DATA_WIDTH(DATA_WIDTH),         
    .LOGICAL_REGISTERS_ADDR_LEN(LOGICAL_REGISTERS_ADDR_LEN),
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN),
    .SUBMIT_PORTS(SUBMIT_PORTS)  
) wb_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .wb_ports_available(wb_ports_available),
    .wb_enable(wb_enable),
    .next_head(next_head),
    .is_wb_ready(is_wb_ready),
    .wb_entries_i(wb_entries_i),
    .current_status_wb(current_status_wb),
    .current_status_wb_enable(current_status_wb_enable),

    .entries_o(entries_o),

    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),

    .wr_addr(wr_addr),
    .wr_data(wr_data),
    .wr_en(wr_en),

    .submit_regs_enable(submit_regs_enable),
    .submit_logical_regs(submit_logical_regs),
    .submit_physical_regs(submit_physical_regs)
);

exe_module_pipeline #(
    .PC_WIDTH(PC_WIDTH),
    .DEPTH(DEPTH),
    .ROB_ADDR_WIDTH(ROB_ADDR_WIDTH),
    .EXE_PORT(EXE_PORT),
    .REG_DATA_WIDTH(DATA_WIDTH),
    .EXE_WRITE_PORTS(EXE_WRITE_PORTS), 
    .PHYSICAL_REGISTERS_ADDR_LEN(PHYSICAL_REGISTERS_ADDR_LEN),
    .NUM_PHYSICAL_REGISTERS(NUM_PHYSICAL_REGISTERS)      
) exe_module_pipeline_inst (
    .clk(global_clock),
    .reset(global_reset),

    .exe_ports_available(exe_ports_available),
    .exe_enable(exe_enable),
    .exe_is_ready_a(exe_is_ready_a),
    .exe_is_ready_b(exe_is_ready_b),
    .exe_clear_signal(exe_clear_signal),
    .exe_clear_mask(exe_clear_mask),
    .exe_set_pt(exe_set_pt),
    .exe_next_pc(exe_next_pc),
    .is_exe_ready(is_exe_ready),
    .exe_entries_i(exe_entries_i),
    .current_status_exe(current_status_exe),
    .current_status_exe_enable(current_status_exe_enable),
    .i_cache_reset(i_cache_reset),
    .entries_o(entries_o),
    .is_ready(is_ready),
    .is_pipeline_stall(is_pipeline_stall),
    .head(head),

    .exe_wr_enable(exe_wr_enable),
    .exe_wr_physical_addr(exe_wr_physical_addr),

    .wr_en_exe(wr_en_exe),
    .wr_addr_exe(wr_addr_exe),
    .wr_data_exe(wr_data_exe),

    .is_cached_exe(is_cached_exe)
);



  /* =========== Lab5 Master end =========== */

  /* =========== Lab5 MUX begin =========== */
  // Wishbone MUX (Masters) => bus slaves
  logic wbs0_cyc_o;
  logic wbs0_stb_o;
  logic wbs0_ack_i;
  logic [31:0] wbs0_adr_o;
  logic [31:0] wbs0_dat_o;
  logic [31:0] wbs0_dat_i;
  logic [3:0] wbs0_sel_o;
  logic wbs0_we_o;

  logic wbs1_cyc_o;
  logic wbs1_stb_o;
  logic wbs1_ack_i;
  logic [31:0] wbs1_adr_o;
  logic [31:0] wbs1_dat_o;
  logic [31:0] wbs1_dat_i;
  logic [3:0] wbs1_sel_o;
  logic wbs1_we_o;

  logic wbs2_cyc_o;
  logic wbs2_stb_o;
  logic wbs2_ack_i;
  logic [31:0] wbs2_adr_o;
  logic [31:0] wbs2_dat_o;
  logic [31:0] wbs2_dat_i;
  logic [3:0] wbs2_sel_o;
  logic wbs2_we_o;

  wb_mux_3 wb_mux (
      .clk(global_clock),
      .rst(global_reset),

      // Master interface (to Lab5 master)
      .wbm_adr_i(wbm_adr_o),
      .wbm_dat_i(wbm_dat_o),
      .wbm_dat_o(wbm_dat_i),
      .wbm_we_i (wbm_we_o),
      .wbm_sel_i(wbm_sel_o),
      .wbm_stb_i(wbm_stb_o),
      .wbm_ack_o(wbm_ack_i),
      .wbm_err_o(),
      .wbm_rty_o(),
      .wbm_cyc_i(wbm_cyc_o),

      // Slave interface 0 (to BaseRAM controller)
      // Address range: 0x8000_0000 ~ 0x803F_FFFF
      .wbs0_addr    (32'h8000_0000),
      .wbs0_addr_msk(32'hFFC0_0000),

      .wbs0_adr_o(wbs0_adr_o),
      .wbs0_dat_i(wbs0_dat_i),
      .wbs0_dat_o(wbs0_dat_o),
      .wbs0_we_o (wbs0_we_o),
      .wbs0_sel_o(wbs0_sel_o),
      .wbs0_stb_o(wbs0_stb_o),
      .wbs0_ack_i(wbs0_ack_i),
      .wbs0_err_i('0),
      .wbs0_rty_i('0),
      .wbs0_cyc_o(wbs0_cyc_o),

      // Slave interface 1 (to ExtRAM controller)
      // Address range: 0x8040_0000 ~ 0x807F_FFFF
      .wbs1_addr    (32'h8040_0000),
      .wbs1_addr_msk(32'hFFC0_0000),

      .wbs1_adr_o(wbs1_adr_o),
      .wbs1_dat_i(wbs1_dat_i),
      .wbs1_dat_o(wbs1_dat_o),
      .wbs1_we_o (wbs1_we_o),
      .wbs1_sel_o(wbs1_sel_o),
      .wbs1_stb_o(wbs1_stb_o),
      .wbs1_ack_i(wbs1_ack_i),
      .wbs1_err_i('0),
      .wbs1_rty_i('0),
      .wbs1_cyc_o(wbs1_cyc_o),

      // Slave interface 2 (to UART controller)
      // Address range: 0x1000_0000 ~ 0x1000_FFFF
      .wbs2_addr    (32'h1000_0000),
      .wbs2_addr_msk(32'hFFFF_0000),

      .wbs2_adr_o(wbs2_adr_o),
      .wbs2_dat_i(wbs2_dat_i),
      .wbs2_dat_o(wbs2_dat_o),
      .wbs2_we_o (wbs2_we_o),
      .wbs2_sel_o(wbs2_sel_o),
      .wbs2_stb_o(wbs2_stb_o),
      .wbs2_ack_i(wbs2_ack_i),
      .wbs2_err_i('0),
      .wbs2_rty_i('0),
      .wbs2_cyc_o(wbs2_cyc_o)
  );

  /* =========== Lab5 MUX end =========== */

  /* =========== Lab5 Slaves begin =========== */
  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_base (
      .clk_i(global_clock),
      .rst_i(global_reset),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs0_cyc_o),
      .wb_stb_i(wbs0_stb_o),
      .wb_ack_o(wbs0_ack_i),
      .wb_adr_i(wbs0_adr_o),
      .wb_dat_i(wbs0_dat_o),
      .wb_dat_o(wbs0_dat_i),
      .wb_sel_i(wbs0_sel_o),
      .wb_we_i (wbs0_we_o),

      // To SRAM chip
      .sram_addr(base_ram_addr),
      .sram_data(base_ram_data),
      .sram_ce_n(base_ram_ce_n),
      .sram_oe_n(base_ram_oe_n),
      .sram_we_n(base_ram_we_n),
      .sram_be_n(base_ram_be_n)
  );

  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_ext (
      .clk_i(global_clock),
      .rst_i(global_reset),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs1_cyc_o),
      .wb_stb_i(wbs1_stb_o),
      .wb_ack_o(wbs1_ack_i),
      .wb_adr_i(wbs1_adr_o),
      .wb_dat_i(wbs1_dat_o),
      .wb_dat_o(wbs1_dat_i),
      .wb_sel_i(wbs1_sel_o),
      .wb_we_i (wbs1_we_o),

      // To SRAM chip
      .sram_addr(ext_ram_addr),
      .sram_data(ext_ram_data),
      .sram_ce_n(ext_ram_ce_n),
      .sram_oe_n(ext_ram_oe_n),
      .sram_we_n(ext_ram_we_n),
      .sram_be_n(ext_ram_be_n)
  );

  // 串口控制器模块
  // NOTE: 如果修改系统时钟频率，也需要修改此处的时钟频率参数
  uart_controller #(
      .CLK_FREQ(CLK_FREQ),
      .BAUD    (115200)
  ) uart_controller (
      .clk_i(global_clock),
      .rst_i(global_reset),

      .wb_cyc_i(wbs2_cyc_o),
      .wb_stb_i(wbs2_stb_o),
      .wb_ack_o(wbs2_ack_i),
      .wb_adr_i(wbs2_adr_o),
      .wb_dat_i(wbs2_dat_o),
      .wb_dat_o(wbs2_dat_i),
      .wb_sel_i(wbs2_sel_o),
      .wb_we_i (wbs2_we_o),

      // to UART pins
      .uart_txd_o(txd),
      .uart_rxd_i(rxd)
  );

  /* =========== Lab5 Slaves end =========== */

endmodule
