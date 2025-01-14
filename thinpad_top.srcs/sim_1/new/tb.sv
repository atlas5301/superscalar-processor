`timescale 10ns / 1ns
module tb;

  wire clk_50M, clk_11M0592;

  reg push_btn;   // BTN5 按钮开关，带消抖电路，按下时为 1
  reg reset_btn;  // BTN6 复位按钮，带消抖电路，按下时为 1

  reg [3:0] touch_btn; // BTN1~BTN4，按钮开关，按下时为 1
  reg [31:0] dip_sw;   // 32 位拨码开关，拨到“ON”时为 1

  wire [15:0] leds;  // 16 位 LED，输出时 1 点亮
  wire [7:0] dpy0;   // 数码管低位信号，包括小数点，输出 1 点亮
  wire [7:0] dpy1;   // 数码管高位信号，包括小数点，输出 1 点亮

  wire txd;  // 直连串口发送端
  wire rxd;  // 直连串口接收端

  wire [31:0] base_ram_data;  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
  wire [19:0] base_ram_addr;  // BaseRAM 地址
  wire[3:0] base_ram_be_n;    // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
  wire base_ram_ce_n;  // BaseRAM 片选，低有效
  wire base_ram_oe_n;  // BaseRAM 读使能，低有效
  wire base_ram_we_n;  // BaseRAM 写使能，低有效

  wire [31:0] ext_ram_data;  // ExtRAM 数据
  wire [19:0] ext_ram_addr;  // ExtRAM 地址
  wire[3:0] ext_ram_be_n;    // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
  wire ext_ram_ce_n;  // ExtRAM 片选，低有效
  wire ext_ram_oe_n;  // ExtRAM 读使能，低有效
  wire ext_ram_we_n;  // ExtRAM 写使能，低有效

  wire [22:0] flash_a;  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
  wire [15:0] flash_d;  // Flash 数据
  wire flash_rp_n;   // Flash 复位信号，低有效
  wire flash_vpen;   // Flash 写保护信号，低电平时不能擦除、烧写
  wire flash_ce_n;   // Flash 片选信号，低有效
  wire flash_oe_n;   // Flash 读使能信号，低有效
  wire flash_we_n;   // Flash 写使能信号，低有效
  wire flash_byte_n; // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

  wire uart_rdn;  // 读串口信号，低有效
  wire uart_wrn;  // 写串口信号，低有效
  wire uart_dataready;  // 串口数据准备好
  wire uart_tbre;  // 发送数据标志
  wire uart_tsre;  // 数据发送完毕标志

  // Windows 需要注意路径分隔符的转义，例如 "D:\\foo\\bar.bin"
  parameter BASE_RAM_INIT_FILE = "C:\\Users\\Fuyuki\\Documents\\Studying\\2023fall\\cod\\rv-2023\\supervisor-rv\\kernel\\kernel.bin"; // BaseRAM 初始化文件，请修改为实际的绝对路径
  parameter EXT_RAM_INIT_FILE = "C:\\Users\\atlas5301\\Downloads\\test.bin";  // ExtRAM 初始化文件，请修改为实际的绝对路径
  parameter FLASH_INIT_FILE = "C:\\Users\\atlas5301\\Downloads\\sum.elf";  // Flash 初始化文件，请修改为实际的绝对路径

  initial begin
    // 在这里可以自定义测试输入序列，例如：
    dip_sw = 32'h2;
    touch_btn = 0;
    reset_btn = 0;
    push_btn = 0;

    #1000;
    reset_btn = 1;
    #100;
    reset_btn = 0;
    $display("reset");
    for (integer i = 0; i < 20; i = i + 1) begin
      #100;  // 等待 100ns
      push_btn = 1;  // 按下 push_btn 按钮
      #100;  // 等待 100ns
      push_btn = 0;  // 松开 push_btn 按钮
    end
  
    // 模拟 PC 通过直连串口，向 FPGA 发送字符
    uart.pc_send_byte(8'h32); // ASCII '2'
    #10000;
    uart.pc_send_byte(8'h33); // ASCII '3'
  end

  // 待测试用户设计
  thinpad_top dut (
      .clk_50M(clk_50M),
      .clk_11M0592(clk_11M0592),
      .push_btn(push_btn),
      .reset_btn(reset_btn),
      .touch_btn(touch_btn),
      .dip_sw(dip_sw),
      .leds(leds),
      .dpy1(dpy1),
      .dpy0(dpy0),
      .txd(txd),
      .rxd(rxd),
      .uart_rdn(uart_rdn),
      .uart_wrn(uart_wrn),
      .uart_dataready(uart_dataready),
      .uart_tbre(uart_tbre),
      .uart_tsre(uart_tsre),
      .base_ram_data(base_ram_data),
      .base_ram_addr(base_ram_addr),
      .base_ram_ce_n(base_ram_ce_n),
      .base_ram_oe_n(base_ram_oe_n),
      .base_ram_we_n(base_ram_we_n),
      .base_ram_be_n(base_ram_be_n),
      .ext_ram_data(ext_ram_data),
      .ext_ram_addr(ext_ram_addr),
      .ext_ram_ce_n(ext_ram_ce_n),
      .ext_ram_oe_n(ext_ram_oe_n),
      .ext_ram_we_n(ext_ram_we_n),
      .ext_ram_be_n(ext_ram_be_n),
      .flash_d(flash_d),
      .flash_a(flash_a),
      .flash_rp_n(flash_rp_n),
      .flash_vpen(flash_vpen),
      .flash_oe_n(flash_oe_n),
      .flash_ce_n(flash_ce_n),
      .flash_byte_n(flash_byte_n),
      .flash_we_n(flash_we_n)
  );

  logic [15:0] is_at_if;
  assign is_at_if = dut.ReorderBuffer_pipeline_inst.is_at_if;

  reg [31:0] result_DEBUG;
  reg [31:0] sel_MEM_DEBUG;
  assign result_DEBUG = dut.mem_module_pipeline_inst.result_DEBUG;
  assign sel_MEM_DEBUG = dut.mem_module_pipeline_inst.sel_MEM_DEBUG;

  reg [31:0] debug_PC_WB;
  assign debug_PC_WB = dut.wb_module_pipeline_inst.debug_PC;

  reg [31:0] debug_PC_EXE;
  assign debug_PC_EXE = dut.exe_module_pipeline_inst.debug_PC;

  reg [31:0] debug_PC_IF;
  assign debug_PC_IF = dut.if_module_pipeline_inst.debug_PC;
  reg [3:0] debug_entry;
  assign debug_entry = dut.if_module_pipeline_inst.debug_entry;

  reg [0:0][4:0] wr_addr;
  reg [0:0][31:0] wr_data;
  reg [0:0] wr_en;

  assign wr_addr = dut.wr_addr;
  assign wr_data = dut.wr_data;
  assign wr_en = dut.wr_en;

  logic [1:0][4:0] rd_addr;
  logic [1:0][31:0] rd_data;

  assign rd_addr = dut.rd_addr;
  assign rd_data = dut.rd_data;

  logic [31:0] registers [0:63];
  assign registers = dut.register_file_pipeline_inst.registers;

  logic [3:0] test_head;
  assign test_head = dut.next_head;

  logic [1:0] state_MEM;
  assign state_MEM = dut.mem_module_pipeline_inst.state_MEM;

  logic [15:0][31:0] test_PC;
  // logic [15:0][31:0] next_PC;
  assign test_PC = dut.ReorderBuffer_pipeline_inst.PC_IF;
  // assign next_PC = dut.ReorderBuffer_pipeline_inst.PC_gen_inst.next_pc; 

  logic [31:0] head_pc;
  assign head_pc = test_PC[test_head];

  logic branch_taken;
  logic [31:0] next_branch_pc;
  assign branch_taken = dut.exe_module_pipeline_inst.branch_taken;
  assign next_branch_pc = dut.exe_module_pipeline_inst.next_pc;

  logic [31:0] a,b,result;
  assign a = dut.exe_module_pipeline_inst.a;
  assign b = dut.exe_module_pipeline_inst.b;
  assign result = dut.exe_module_pipeline_inst.result; 

  logic [31:0] mstatus;  // 0X300, Machine status register.
  logic [31:0] mie;      // 0X304, Machine interrupt-enable register.
  logic [31:0] mtvec;    // 0X305, Machine trap-handler base address.
  logic [31:0] mscratch; // 0X340, Scratch register for machine trap handlers.
  logic [31:0] mepc;     // 0X341, Machine exception program counter.
  logic [31:0] mcause;   // 0X342, Machine trap cause.
  logic [31:0] mip;      // 0X344, Machine interrupt pending.
  logic is_csr;
  logic is_branch;

  assign is_csr = dut.exe_module_pipeline_inst.is_csr;
  assign is_branch = dut.exe_module_pipeline_inst.is_branch;
  assign mstatus = dut.exe_module_pipeline_inst.mstatus;
  assign mie = dut.exe_module_pipeline_inst.mie;
  assign mtvec = dut.exe_module_pipeline_inst.mtvec;
  assign mscratch = dut.exe_module_pipeline_inst.mscratch;
  assign mepc = dut.exe_module_pipeline_inst.mepc;
  assign mcause = dut.exe_module_pipeline_inst.mcause;
  assign mip = dut.exe_module_pipeline_inst.mip;

  localparam REG_DATA_WIDTH = 32;

  logic [REG_DATA_WIDTH-1:0] mtime_low;// 0X100, Machine Time.
  logic [REG_DATA_WIDTH-1:0] mtime_high;
                                        // 0X101, Machine Time.
  logic [REG_DATA_WIDTH-1:0] mtimecmp_low;
                                      // 0X120, Machine Time Cmp.
  logic [REG_DATA_WIDTH-1:0] mtimecmp_high;
                                      // 0X121, Machine Time Cmp.

  logic [REG_DATA_WIDTH-1:0] new_mtime_low;
  logic [REG_DATA_WIDTH-1:0] new_mtime_high;

  logic [1:0] privilege_mode;

  logic [16:0] cnt;

  assign mtime_low = dut.exe_module_pipeline_inst.mtime_low;
  assign mtime_high = dut.exe_module_pipeline_inst.mtime_high;
  assign mtimecmp_low = dut.exe_module_pipeline_inst.mtimecmp_low;
  assign mtimecmp_high = dut.exe_module_pipeline_inst.mtimecmp_high;
  assign new_mtime_low = dut.exe_module_pipeline_inst.new_mtime_low;
  assign new_mtime_high = dut.exe_module_pipeline_inst.new_mtime_high;
  assign cnt = dut.exe_module_pipeline_inst.cnt;

  assign privilege_mode = dut.exe_module_pipeline_inst.privilege_mode;

  logic [0:0] test_if_enable;
  logic [0:0][3:0] test_if_ports_available;
  import signals::*;
  stage_t [15:0] test_current_status;
  stage_t [15:0] test_status;

  logic [7:0][15:0] test_masks;
  logic [15:0] mask_layer1;
  logic [15:0] inside_mask;
  assign mask_layer1 = test_masks[0];
  assign inside_mask = dut.ReorderBuffer_pipeline_inst.PC_gen_inst.inside_mask;

  
  stage_t [7:0][15:0] test_arrays;

  logic test_is_pipeline_stall, test_is_ready;

  assign test_if_enable = dut.if_enable;
  assign test_if_ports_available = dut.if_ports_available;
  assign test_current_status = dut.ReorderBuffer_pipeline_inst.current_status;
  assign test_status = dut.ReorderBuffer_pipeline_inst.status;
  assign test_masks = dut.ReorderBuffer_pipeline_inst.masks;
  assign test_arrays = dut.ReorderBuffer_pipeline_inst.arrays;
  assign test_is_pipeline_stall = dut.is_pipeline_stall;
  assign test_is_ready = dut.is_ready;


  // 时钟源
  clock osc (
      .clk_11M0592(clk_11M0592),
      .clk_50M    (clk_50M)
  );
  // CPLD 串口仿真模型
  cpld_model cpld (
      .clk_uart(clk_11M0592),
      .uart_rdn(uart_rdn),
      .uart_wrn(uart_wrn),
      .uart_dataready(uart_dataready),
      .uart_tbre(uart_tbre),
      .uart_tsre(uart_tsre),
      .data(base_ram_data[7:0])
  );
  // 直连串口仿真模型
  uart_model uart (
    .rxd (txd),
    .txd (rxd)
  );
  // BaseRAM 仿真模型
  sram_model base1 (
      .DataIO(base_ram_data[15:0]),
      .Address(base_ram_addr[19:0]),
      .OE_n(base_ram_oe_n),
      .CE_n(base_ram_ce_n),
      .WE_n(base_ram_we_n),
      .LB_n(base_ram_be_n[0]),
      .UB_n(base_ram_be_n[1])
  );
  sram_model base2 (
      .DataIO(base_ram_data[31:16]),
      .Address(base_ram_addr[19:0]),
      .OE_n(base_ram_oe_n),
      .CE_n(base_ram_ce_n),
      .WE_n(base_ram_we_n),
      .LB_n(base_ram_be_n[2]),
      .UB_n(base_ram_be_n[3])
  );
  // ExtRAM 仿真模型
  sram_model ext1 (
      .DataIO(ext_ram_data[15:0]),
      .Address(ext_ram_addr[19:0]),
      .OE_n(ext_ram_oe_n),
      .CE_n(ext_ram_ce_n),
      .WE_n(ext_ram_we_n),
      .LB_n(ext_ram_be_n[0]),
      .UB_n(ext_ram_be_n[1])
  );
  sram_model ext2 (
      .DataIO(ext_ram_data[31:16]),
      .Address(ext_ram_addr[19:0]),
      .OE_n(ext_ram_oe_n),
      .CE_n(ext_ram_ce_n),
      .WE_n(ext_ram_we_n),
      .LB_n(ext_ram_be_n[2]),
      .UB_n(ext_ram_be_n[3])
  );
  // Flash 仿真模型
  x28fxxxp30 #(
      .FILENAME_MEM(FLASH_INIT_FILE)
  ) flash (
      .A   (flash_a[1+:22]),
      .DQ  (flash_d),
      .W_N (flash_we_n),      // Write Enable 
      .G_N (flash_oe_n),      // Output Enable
      .E_N (flash_ce_n),      // Chip Enable
      .L_N (1'b0),            // Latch Enable
      .K   (1'b0),            // Clock
      .WP_N(flash_vpen),      // Write Protect
      .RP_N(flash_rp_n),      // Reset/Power-Down
      .VDD ('d3300),
      .VDDQ('d3300),
      .VPP ('d1800),
      .Info(1'b1)
  );

  initial begin
    wait (flash_byte_n == 1'b0);
    $display("8-bit Flash interface is not supported in simulation!");
    $display("Please tie flash_byte_n to high");
    $stop;
  end

  // 从文件加载 BaseRAM
  initial begin
    reg [31:0] tmp_array[0:1048575];
    integer n_File_ID, n_Init_Size;
    n_File_ID = $fopen(BASE_RAM_INIT_FILE, "rb");
    if (!n_File_ID) begin
      n_Init_Size = 0;
      $display("Failed to open BaseRAM init file");
    end else begin
      n_Init_Size = $fread(tmp_array, n_File_ID);
      n_Init_Size /= 4;
      $fclose(n_File_ID);
    end
    $display("BaseRAM Init Size(words): %d", n_Init_Size);
    for (integer i = 0; i < n_Init_Size; i++) begin
      base1.mem_array0[i] = tmp_array[i][24+:8];
      base1.mem_array1[i] = tmp_array[i][16+:8];
      base2.mem_array0[i] = tmp_array[i][8+:8];
      base2.mem_array1[i] = tmp_array[i][0+:8];
    end
  end

  // 从文件加载 ExtRAM
  initial begin
    reg [31:0] tmp_array[0:1048575];
    integer n_File_ID, n_Init_Size;
    n_File_ID = $fopen(EXT_RAM_INIT_FILE, "rb");
    if (!n_File_ID) begin
      n_Init_Size = 0;
      $display("Failed to open ExtRAM init file");
    end else begin
      n_Init_Size = $fread(tmp_array, n_File_ID);
      n_Init_Size /= 4;
      $fclose(n_File_ID);
    end
    $display("ExtRAM Init Size(words): %d", n_Init_Size);
    for (integer i = 0; i < n_Init_Size; i++) begin
      ext1.mem_array0[i] = tmp_array[i][24+:8];
      ext1.mem_array1[i] = tmp_array[i][16+:8];
      ext2.mem_array0[i] = tmp_array[i][8+:8];
      ext2.mem_array1[i] = tmp_array[i][0+:8];
    end
  end
endmodule
