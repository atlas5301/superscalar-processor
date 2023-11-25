// module inst_decoder(
//     input logic [31:0] instruction,
//     output opcode,
//     output data_path_op1,
//     output data_info_op1,
//     output data_path_op2,
//     output data_info_op1,
//     output data_path_dst,
//     output data_info_dst,
    
// );

//     assign opcode = instruction[6:0];
//     assign rd = instruction[11:7];
//     assign funct3 = instruction[14:12];
//     assign rs1 = instruction[19:15];
//     assign rs2 = instruction[24:20];
//     assign funct7 = instruction[31:25];

//     assign immediate_I_type = instruction[31:20];
//     assign immediate_S_type = {instruction[31:25], instruction[11:7]};
//     assign immediate_B_type = {instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
//     assign immediate_U_type = instruction[31:12];
//     assign immediate_J_type = {instruction[31], instruction[19:12], instruction[20], instruction[30:21]};



// endmodule

module inst_decoder(
    input wire [31:0] instruction,
    output reg [15:0] immediate,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [4:0] rd,
    output reg [3:0] opcode,
    output reg is_rtype, 
    output reg is_itype,
    output reg is_peek,
    output reg is_poke
);
    assign immediate = instruction[31:16];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign rd = instruction[11:7];
    assign opcode = instruction[6:3];

    assign is_rtype = (instruction[2:0] == 3'b001);
    assign is_itype = (instruction[2:0] == 3'b010);
    assign is_peek = is_itype && (instruction[6:3] == 4'b0010);
    assign is_poke = is_itype && (instruction[6:3] == 4'b0001);


endmodule