module alu #(
    parameter WIDTH = 32,
    parameter SHIFT_WIDTH = 5
    
) (
    input wire [WIDTH-1:0] a, b,
    input wire [3:0] opcode,  // 4-bit opcode to support up to 16 operations
    output reg [WIDTH-1:0] result
);

    // Define the ALU operations using an enum
    typedef enum logic [3:0]{
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
    } alu_operations_e;

    // ALU logic
    always_comb begin
        case (opcode)
            ADD: result = a + b;
            SUB: result = a - b;
            AND: result = a & b;
            OR:  result = a | b;
            XOR: result = a ^ b;
            NOT: result = ~a;
            SLL: result = a << b[SHIFT_WIDTH-1:0];
            SRL: result = a >> b[SHIFT_WIDTH-1:0];  // logical right shift
            SRA: result = signed'(a) >>> b[SHIFT_WIDTH-1:0];   // arithmetic right shift
            ROL: result = (a << b[SHIFT_WIDTH-1:0]) | (a >> (WIDTH-b[SHIFT_WIDTH-1:0])); // Rotate left
            default: result = 0;  // Default case for unrecognized opcodes
        endcase
    end

endmodule
