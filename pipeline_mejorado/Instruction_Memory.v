
`timescale 1ns / 1ns

module Instruction_Memory (
    input [7:0] address,
    output reg [31:0] instruction
);

    reg [31:0] Mem [0:255];

    // initial begin
    //     $readmemh("instructions.hex", Mem);
    // end
        // initial begin
        // $readmemb("codigo_validacion.txt", Mem);
    //end
    always @(address) begin
        instruction = Mem[address >> 2]; // divide by 4 for word addressing
    end
endmodule
