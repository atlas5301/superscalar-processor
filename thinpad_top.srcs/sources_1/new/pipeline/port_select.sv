module conflict_control_pipeline #(
    parameter DEPTH = 64,
    parameter WIDTH = 32,
    parameter ROB_ADDR_LEN = 6,    
    parameter REG_ADDR_LEN = 5,
    parameter NUM_OUTPUT_WB = 4,
    parameter NUM_OUTPUT_OF = 4
)(
    input wire [ROB_ADDR_LEN-1:0] head,
    input wire [DEPTH-1:0][REG_ADDR_LEN-1:0] read1,
    input wire [DEPTH-1:0][REG_ADDR_LEN-1:0] read2,
    input wire [DEPTH-1:0][REG_ADDR_LEN-1:0] write1,
    input wire [DEPTH-1:0] entrymask_write,
    input wire [DEPTH-1:0] entrymask_read,
    output logic [NUM_OUTPUT_WB-1:0] output_wbenable,
    output logic [NUM_OUTPUT_OF-1:0] output_ofenable,    
    output logic [NUM_OUTPUT_WB-1:0][ROB_ADDR_LEN-1:0] output_ports_wb,
    output logic [NUM_OUTPUT_OF-1:0][ROB_ADDR_LEN-1:0] output_ports_of
);

    logic [DEPTH-1:0][REG_ADDR_LEN-1:0] rotated_read1;
    logic [DEPTH-1:0][REG_ADDR_LEN-1:0] rotated_read2;
    logic [DEPTH-1:0][REG_ADDR_LEN-1:0] rotated_write1;
    logic [DEPTH-1:0] rotated_entrymask_write;
    logic [DEPTH-1:0] rotated_entrymask_read;

    logic [DEPTH-1:0] output_readenable, output_writeenable;
    int current_port;
    
    always_comb begin
        for (int i = 0; i < DEPTH; i++) begin
            output_readenable[i] = 1'b0;
            output_writeenable[i] = 1'b0;
        end

        for (int i = 0; i < DEPTH; i++) begin
            rotated_read1[i] = 0;
            rotated_read2[i] = 0;
            rotated_write1[i] = 0;
            rotated_entrymask_read[i] = 0;
            rotated_entrymask_write[i] = 0;
        end

        for (int i = 0; i < DEPTH; i++) begin
            int new_index = (i - head + DEPTH) % DEPTH; // Calculate new index
            rotated_read1[new_index] = read1[i];
            rotated_read2[new_index] = read2[i];
            rotated_write1[new_index] = write1[i];
            rotated_entrymask_read[new_index] = entrymask_read[i];
            rotated_entrymask_write[new_index] = entrymask_write[i];
        end

        // Check from head to DEPTH
        for (int i = 0; i < DEPTH; i++) begin
            output_readenable[i] = rotated_entrymask_read[i];
            output_writeenable[i] = rotated_entrymask_write[i];

            for (int j = 0; j < i; j++) begin
                output_readenable[i] &= !(rotated_read1[i] && (rotated_read1[i] == rotated_write1[j])) && !(rotated_read2[i] && (rotated_read2[i] == rotated_write1[j]));
                output_writeenable[i] &= !(rotated_write1[i] && (rotated_write1[i] == rotated_read1[j] || rotated_write1[i] == rotated_read2[j]));
            end
        end

        // Set output_enable and output_ports
        for (int i = 0; i < NUM_OUTPUT_OF; i++) begin
            output_ofenable[i] = 0;
            output_ports_of[i] = 0;
        end

        for (int i = 0; i < NUM_OUTPUT_WB; i++) begin
            output_wbenable[i] = 0;
            output_ports_wb[i] = 0;
        end
        current_port = 0;

        // Check from head to DEPTH
        for (int i = 0; i < DEPTH; i++) begin
            if (output_readenable[i]) begin
                if (current_port == NUM_OUTPUT_OF)
                    break;

                output_ofenable[current_port] = 1;
                output_ports_of[current_port] = (i + head) % DEPTH;
                current_port += 1;
            end
        end

        current_port = 0;

        for (int i = 0; i < DEPTH; i++) begin
            if (output_writeenable[i]) begin
                if (current_port == NUM_OUTPUT_WB)
                    break;

                output_wbenable[current_port] = 1;
                output_ports_wb[current_port] = (i + head) % DEPTH;
                current_port += 1;
            end else begin
                break;
            end
        end

    end
endmodule

module port_select_pipeline #(
    parameter DEPTH = 64,
    parameter ROB_ADDR_LEN = 6,
    parameter NUM_OUTPUT = 4
)(
    input wire [ROB_ADDR_LEN-1:0] head,
    input wire [DEPTH-1:0] enable,
    output logic [NUM_OUTPUT-1:0] output_enable,
    output logic [NUM_OUTPUT-1:0][ROB_ADDR_LEN-1:0] output_ports
);

    int current_port;

    logic [DEPTH-1:0] rotated_enable;

    always_comb begin
        for (int i = 0; i < DEPTH; i++) begin
            rotated_enable[i] = 0;
        end

        for (int i = 0; i < DEPTH; i++) begin
            int new_index = (i - head + DEPTH) % DEPTH; // Calculate new index
            rotated_enable[new_index] = enable[i];
        end
        // Set output_enable and output_ports
        for (int i = 0; i < NUM_OUTPUT; i++) begin
            output_enable[i] = 0;
            output_ports[i] = 0;
        end

        current_port = 0;

        // Check from head to DEPTH
        for (int i = 0; i < DEPTH; i++) begin
            if (rotated_enable[i]) begin
                if (current_port == NUM_OUTPUT)
                    break;

                output_enable[current_port] = 1;
                output_ports[current_port] = (i + head) % DEPTH;
                current_port += 1;
            end
        end

    end
endmodule

module port_select_first_n_pipeline #(
    parameter DEPTH = 64,
    parameter ROB_ADDR_LEN = 5,
    parameter NUM_OUTPUT = 4
)(
    input wire [ROB_ADDR_LEN-1:0] head,
    input wire [DEPTH-1:0] enable,
    output logic [NUM_OUTPUT-1:0] output_enable,
    output logic [NUM_OUTPUT-1:0][ROB_ADDR_LEN-1:0] output_ports
);

    int current_port;

    logic [DEPTH-1:0] rotated_enable;

    always_comb begin
        for (int i = 0; i < DEPTH; i++) begin
            rotated_enable[i] = 0;
        end
        // Set output_enable and output_ports
        for (int i = 0; i < NUM_OUTPUT; i++) begin
            output_enable[i] = 0;
            output_ports[i] = 0;
        end
        for (int i = 0; i < DEPTH; i++) begin
            int new_index = (i - head + DEPTH) % DEPTH; // Calculate new index
            rotated_enable[new_index] = enable[i];
        end

        current_port = 0;


        for (int i = 0; i < DEPTH; i++) begin
            if (rotated_enable[i]) begin
                if (current_port == NUM_OUTPUT)
                    break;

                output_enable[current_port] = 1;
                output_ports[current_port] = (i + head) % DEPTH;
                current_port += 1;
            end else begin
                break;
            end
        end

    end
endmodule

