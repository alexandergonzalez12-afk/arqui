
`timescale 1ns / 1ns

module Instruction_Memory (
    input [7:0] address,            // Accepts an 8-bit address
    output reg [31:0] instruction   // Outputs a 32-bit instruction
);
    reg [7:0] memory [0:63];        // Memory to store 8-bit segments; adjusted size for four 8-bit values per 32-bit instruction

    integer i;
    reg [31:0] temp_instruction [0:16]; // Temporary storage for 32-bit instructions

    end
endmodule
