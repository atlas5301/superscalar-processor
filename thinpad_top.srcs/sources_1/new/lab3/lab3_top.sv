`default_nettype none

module lab3_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮�?关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时�? 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时�? 1
    output wire [15:0] leds,       // 16 �? LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信�?
    output wire uart_rdn,        // 读串口信号，低有�?
    output wire uart_wrn,        // 写串口信号，低有�?
    input  wire uart_dataready,  // 串口数据准备�?
    input  wire uart_tbre,       // 发�?�数据标�?
    input  wire uart_tsre,       // 数据发�?�完毕标�?

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共�?
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持�? 0
    output wire base_ram_ce_n,  // BaseRAM 片�?�，低有�?
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有�?
    output wire base_ram_we_n,  // BaseRAM 写使能，低有�?

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持�? 0
    output wire ext_ram_ce_n,  // ExtRAM 片�?�，低有�?
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有�?
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有�?

    // 直连串口信号
    output wire txd,  // 直连串口发�?�端
    input  wire rxd,  // 直连串口接收�?

    // Flash 存储器信号，参�?? JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效�?16bit 模式无意�?
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧�?
    output wire flash_ce_n,  // Flash 片�?�信号，低有�?
    output wire flash_oe_n,  // Flash 读使能信号，低有�?
    output wire flash_we_n,  // Flash 写使能信号，低有�?
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash �? 16 位模式时请设�? 1

    // USB 控制器信号，参�?? SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参�?? DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素�?3 �?
    output wire [2:0] video_green,  // 绿色像素�?3 �?
    output wire [1:0] video_blue,   // 蓝色像素�?2 �?
    output wire       video_hsync,  // 行同步（水平同步）信�?
    output wire       video_vsync,  // 场同步（垂直同步）信�?
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐�?
);

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设�?
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设�?
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出�?"1"表示时钟稳定�?
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，�? locked 信号转为后级电路的复�? reset_of_clk10M
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end

  wire global_clock;
  wire global_reset;
  assign global_clock = clk_50M;
  assign global_reset = reset_btn;
  
  /* =========== Demo code end =========== */

  // TODO: 内部信号声明
    localparam ALU_WIDTH = 16;
    localparam SHIFT_WIDTH = 4;

    wire [ALU_WIDTH-1:0] alu_a, alu_b;
    wire [ALU_WIDTH-1:0] alu_result;

    reg [ALU_WIDTH-1:0] alu_a_ff, alu_b_ff;
    assign alu_a = alu_a_ff;
    assign alu_b = alu_b_ff;

    logic [15:0] immediate;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [4:0] rd;
    logic [3:0] opcode;
    logic is_rtype;
    logic is_itype;
    logic is_peek;
    logic is_poke;

    // Define the parameters for the register_file module
    localparam DATA_WIDTH = 16;
    localparam ADDR_WIDTH = 5;
    localparam NUM_READ_PORTS = 2;
    localparam NUM_WRITE_PORTS = 1;

    // Declare wires and registers to connect to the register_file
    reg [NUM_WRITE_PORTS-1:0] wr_en;
    reg [NUM_WRITE_PORTS-1:0][ADDR_WIDTH-1:0] wr_addr;
    reg [NUM_WRITE_PORTS-1:0][DATA_WIDTH-1:0] wr_data;
    reg [NUM_READ_PORTS-1:0][ADDR_WIDTH-1:0] rd_addr;
    reg [NUM_READ_PORTS-1:0][DATA_WIDTH-1:0] rd_data;
    // logic [DATA_WIDTH-1:0] debug_reg;


    reg [15:0] led_value;
    assign leds=led_value;

    // Instantiate the register_file
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .NUM_READ_PORTS(NUM_READ_PORTS),
        .NUM_WRITE_PORTS(NUM_WRITE_PORTS)
    ) reg_file_instance (
        .clk(global_clock),
        .rst_n(global_reset),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .rd_addr(rd_addr),
        .rd_data(rd_data)
        // .debug(debug_reg)
    );


    // Instantiate the ALU
    alu #(ALU_WIDTH, SHIFT_WIDTH) alu_lab3 (
        .a(alu_a),
        .b(alu_b),
        .opcode(opcode),
        .result(alu_result)
    );

  // TODO: 实验模块例化
  inst_decoder decoder(
    .instruction(dip_sw),
    .immediate(immediate),
    .rs1(rs1),
    .rs2(rs2),
    .rd(rd),
    .opcode(opcode),
    .is_rtype(is_rtype), 
    .is_itype(is_itype),
    .is_peek(is_peek),
    .is_poke(is_poke)
  );

  function void initialize();
    for (int i = 0; i < NUM_WRITE_PORTS; i++) begin
      wr_en[i] <= 0;
    end
  endfunction

  function void reset_output();
    led_value <= 16'b0;
  endfunction

  reg [15:0] debug_a;
  reg [15:0] debug_b;
  reg [15:0] debug_opcode;
  reg [15:0] debug_alu_result;
  // assign debug_a = rd_addr[0][ADDR_WIDTH-1:0];
  // assign debug_a = rd_addr[1][ADDR_WIDTH-1:0];



typedef enum logic [3:0] {
    ST_INIT,
    ST_DECODE,
    ST_CALC,
    ST_READ_REG,
    ST_WRITE_REG
  } state_t;

  // 状�?�机当前状�?�寄存器
  state_t state;

  // 状�?�机逻辑
  always_ff @(posedge global_clock) begin
    if (global_reset) begin
      // TODO: 复位各个输出信号
      initialize();
      reset_output();
      state <= ST_INIT;
    end else begin
      case (state)
        ST_INIT: begin
          //initialize();
          wr_en[0] <= 0;
          if (push_btn) begin
            state <= ST_DECODE;
          end
        end

        ST_DECODE: begin
          if (is_rtype) begin
            // 把寄存器地址交给寄存器堆，读取操作数
            rd_addr[0][ADDR_WIDTH-1:0] = rs1;
            rd_addr[1][ADDR_WIDTH-1:0] = rs2;

            debug_opcode <= opcode;
            state <= ST_CALC;
          end else if (is_peek) begin
              rd_addr[0][ADDR_WIDTH-1:0] = rd;
              state <= ST_READ_REG;
          end else if (is_poke) begin
              wr_addr[0][ADDR_WIDTH-1:0] = rd;
              wr_data[0][DATA_WIDTH-1:0] = immediate;
              wr_en[0] = 1;
              state <= ST_WRITE_REG;
          end else begin
              state <= ST_INIT;
          end
        end

        ST_CALC: begin
          // TODO: 将数据交�? ALU，并�? ALU 获取结果
          alu_a_ff = rd_data[0][DATA_WIDTH-1:0];

          debug_a = rd_data[0][DATA_WIDTH-1:0];
          //debug_a <= rd_addr[0][ADDR_WIDTH-1:0];
          //debug_a <= alu_a;


          alu_b_ff = rd_data[1][DATA_WIDTH-1:0];

          debug_b = rd_data[1][DATA_WIDTH-1:0];
            //debug_b <= rd_addr[1][ADDR_WIDTH-1:0];
          //debug_b <= alu_b;

          
          //debug_alu_result <= alu_result;
          state <= ST_WRITE_REG;
        end

        ST_WRITE_REG: begin
          // TODO: 将结果存入寄存器
          if(is_rtype) begin
            // $display("pass");
            wr_addr[0][ADDR_WIDTH-1:0] = rd;
            wr_data[0][DATA_WIDTH-1:0] = alu_result;
            wr_en[0] = 1;
          end else begin
          wr_en[0] <= 0;
          state <= ST_INIT;
          end
          // led_value <= debug_reg;
        end

        ST_READ_REG: begin
          led_value <= rd_data[0][DATA_WIDTH-1:0];
          // TODO: 将数据从寄存器中读出，存�? leds
          state <= ST_INIT;
        end

        default: begin
          state <= ST_INIT;
        end
      endcase
    end
  end



endmodule
