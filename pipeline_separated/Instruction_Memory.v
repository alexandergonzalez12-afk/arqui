
`timescale 1ns / 1ns

module Instruction_Memory (
    input [7:0] address,            // Accepts an 8-bit address
    output reg [31:0] instruction   // Outputs a 32-bit instruction
);
    reg [7:0] memory [0:63];        // Memory to store 8-bit segments; adjusted size for four 8-bit values per 32-bit instruction

    integer i;
    reg [31:0] temp_instruction [0:16]; // Temporary storage for 32-bit instructions

    initial begin
        // Load 32-bit instructions into a temporary array
        $readmemh("instructions.hex", temp_instruction);

        // Split each 32-bit instruction into four 8-bit segments for storage in memory
        for (i = 0; i < 18; i = i + 1) begin
            memory[i * 4]     = temp_instruction[i][31:24];
            memory[i * 4 + 1] = temp_instruction[i][23:16];
            memory[i * 4 + 2] = temp_instruction[i][15:8];
            memory[i * 4 + 3] = temp_instruction[i][7:0];
        end
    end

    // Concatenate four 8-bit values to form a 32-bit instruction based on the input address
    always @(*) begin
        instruction = {memory[address], memory[address + 1], memory[address + 2], memory[address + 3]};
    end
endmodule
