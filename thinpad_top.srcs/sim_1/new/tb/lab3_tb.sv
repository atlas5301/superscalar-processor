`timescale 10ns / 1ns
module lab3_tb;

  wire clk_50M, clk_11M0592;

  reg push_btn;   // BTN5 按钮�??关，带消抖电路，按下时为 1
  reg reset_btn;  // BTN6 复位按钮，带消抖电路，按下时�?? 1

  reg [3:0] touch_btn; // BTN1~BTN4，按钮开关，按下时为 1
  reg [31:0] dip_sw;   // 32 位拨码开关，拨到“ON”时�?? 1

  wire [15:0] leds;  // 16 �?? LED，输出时 1 点亮
  wire [7:0] dpy0;   // 数码管低位信号，包括小数点，输出 1 点亮
  wire [7:0] dpy1;   // 数码管高位信号，包括小数点，输出 1 点亮

  // 实验 3 用到的指令格�??
  `define inst_rtype(rd, rs1, rs2, op) \
    {7'b0, rs2, rs1, 3'b0, rd, op, 3'b001}

  `define inst_itype(rd, imm, op) \
    {imm, 4'b0, rd, op, 3'b010}
  
  `define inst_poke(rd, imm) `inst_itype(rd, imm, 4'b0001)
  `define inst_peek(rd, imm) `inst_itype(rd, imm, 4'b0010)

  // opcode table
  typedef enum logic [3:0] {
    ADD = 4'b0001,
    SUB = 4'b0010,
    AND = 4'b0011,
    OR  = 4'b0100,
    XOR = 4'b0101,
    NOT = 4'b0110,
    SLL = 4'b0111,
    SRL = 4'b1000,
    SRA = 4'b1001,
    ROL = 4'b1010
  } opcode_t;

  logic [15:0] imm;
  logic [4:0] rd, rs1, rs2;
  logic [3:0] opcode;

  reg [15:0] expected_rd_value;
  reg [15:0] rs1_value;
  reg [15:0] rs2_value;
  reg [15:0] rd_value;




task pulse_push_btn_several_cycles();
    push_btn = 1;          // Set the push_btn high

    //repeat(2) @(posedge clk_50M);
    #4;

    push_btn = 0; // Set the push_btn low
endtask

task wait_for_stable();
    //repeat(50) @(posedge clk_50M);
    #100;
endtask



// This function is used to execute a PEEK instruction for a given register number.
// It waits for a while for the instruction to complete and then returns the peeked value.
task execute_peek(input [4:0] reg_num, output reg [15:0] result);
    dip_sw = `inst_peek(reg_num, 0); 
    pulse_push_btn_several_cycles();
    wait_for_stable();
    //#1000;
    result = leds;
endtask



task verify_alu_operation(input [15:0] rs1_value, 
input [15:0] rs2_value, 
input [15:0] rd_value, 
input [3:0] opcode_value, 
input [4:0] rs1, 
input [4:0] rs2, 
input [4:0] rd);
    // Calculate expected result based on ALU logic
    case (opcode_value)
        ADD: expected_rd_value = rs1_value + rs2_value;
        SUB: expected_rd_value = rs1_value - rs2_value;
        AND: expected_rd_value = rs1_value & rs2_value;
        OR:  expected_rd_value = rs1_value | rs2_value;
        XOR: expected_rd_value = rs1_value ^ rs2_value;
        NOT: expected_rd_value = ~rs1_value;
        SLL: expected_rd_value = rs1_value << rs2_value[3:0];
        SRL: expected_rd_value = rs1_value >> rs2_value[3:0];
        SRA: expected_rd_value = signed'(rs1_value) >>> rs2_value[3:0];
        ROL: expected_rd_value = (rs1_value << rs2_value[3:0]) | (rs1_value >> (16-rs2_value[3:0]));
        default: expected_rd_value = 0;
    endcase

    if (rd == 0)  begin
      expected_rd_value = 0;
    end


    // Assertion to check if expected value matches the actual value in 'leds' after executing PEEK for 'rd'
    assert (expected_rd_value == rd_value) begin
    end else begin
        $fatal(2, "Error: Mismatch detected for a= %0b, b= %0b, opcode: %0d. Expected: %0b, Actual: %0b \n rs1: %0d, rs2: %0d, rd: %0d", rs1_value, rs2_value, opcode_value, expected_rd_value, rd_value, rs1, rs2, rd);
    end
endtask

// ... (rest of the simulation loop remains unchanged)





  initial begin
    // 在这里可以自定义测试输入序列，例如：
    dip_sw = 32'h0;
    touch_btn = 0;
    reset_btn = 0;
    push_btn = 0;

    #10000;
    reset_btn = 1;
    wait_for_stable();
    #100;
    reset_btn = 0;
    #1000;  // 等待复位结束

    // 样例：使�?? POKE 指令为寄存器赋随机初�??
    for (int i = 1; i < 32; i = i + 1) begin
      #100;
      rd = i;   // only lower 5 bits
      dip_sw = `inst_poke(rd, $urandom_range(0, 65536));
      // push_btn = 1;

      // #100;
      // push_btn = 0;
      pulse_push_btn_several_cycles();

      wait_for_stable();

      //#1000;
    end


    // Randomly test various instructions
    for (int i = 0; i < 10000; i = i + 1) begin
        #10;
        // Randomly select instruction type and operands
        opcode = opcode_t'($urandom_range(0, 9));  // Random opcode
        rd = $urandom_range(0, 31);
        rs1 = $urandom_range(0, 31);
        rs2 = $urandom_range(0, 31);
        imm = $urandom_range(0, 65535);

        // Set up instruction based on type
        if (i % 3 == 0) begin  // Roughly 1/3 of instructions are I-type
            if (i % 6 == 0) begin  // Half of I-type instructions are POKE, half are PEEK
                dip_sw = `inst_poke(rd, imm);
            end else begin
                dip_sw = `inst_peek(rd, imm);
            end
        end else begin  // 2/3 of instructions are R-type
            execute_peek(rs1, rs1_value);
            execute_peek(rs2, rs2_value);
   
            dip_sw = `inst_rtype(rd, rs1, rs2, opcode);

            pulse_push_btn_several_cycles();

            wait_for_stable();
            
            execute_peek(rd, rd_value);

            
            verify_alu_operation(rs1_value, rs2_value, rd_value, opcode, rs1, rs2, rd);

            $display("pass %0d", i);
        end

        // push_btn = 1;  // Trigger instruction execution
        // #100;
        // push_btn = 0;
        wait_for_stable();

        //#1000;  // Wait for instruction to complete

        // TODO: Check the result (e.g., check rd register value or LEDs for expected outcome)

//        if (/* result does not match expected */) begin
//            $display("Error: Instruction %0d failed. Expected: ..., Observed: ...", i);
//        end
    end
    $finish;
  end

  // 待测试用户设�??
  lab3_top dut (
      .clk_50M(clk_50M),
      .clk_11M0592(clk_11M0592),
      .push_btn(push_btn),
      .reset_btn(reset_btn),
      .touch_btn(touch_btn),
      .dip_sw(dip_sw),
      .leds(leds),
      .dpy1(dpy1),
      .dpy0(dpy0),

      .txd(),
      .rxd(1'b1),
      .uart_rdn(),
      .uart_wrn(),
      .uart_dataready(1'b0),
      .uart_tbre(1'b0),
      .uart_tsre(1'b0),
      .base_ram_data(),
      .base_ram_addr(),
      .base_ram_ce_n(),
      .base_ram_oe_n(),
      .base_ram_we_n(),
      .base_ram_be_n(),
      .ext_ram_data(),
      .ext_ram_addr(),
      .ext_ram_ce_n(),
      .ext_ram_oe_n(),
      .ext_ram_we_n(),
      .ext_ram_be_n(),
      .flash_d(),
      .flash_a(),
      .flash_rp_n(),
      .flash_vpen(),
      .flash_oe_n(),
      .flash_ce_n(),
      .flash_byte_n(),
      .flash_we_n()
  );

  // 时钟�??
  clock osc (
      .clk_11M0592(clk_11M0592),
      .clk_50M    (clk_50M)
  );


  reg [15:0] debug_a;
  reg [15:0] debug_b;
  reg [15:0] debug_opcode;
  reg [15:0] debug_alu_result;
  reg [1:0][4:0] debug_rd_addr;
  reg [1:0][15:0] debug_rd_data;
  reg [0:0][15:0] debug_wr_data;
  assign debug_a = dut.debug_a;
  assign debug_b = dut.debug_b;
  assign debug_opcode = dut.debug_opcode;
  assign debug_rd_addr = dut.rd_addr;
  assign debug_rd_data = dut.rd_data;
  assign debug_alu_result = dut.alu_result;
  assign debug_wr_data = dut.wr_data;

endmodule
