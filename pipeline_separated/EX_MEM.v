// EX_MEM.v
`timescale 1ns / 1ns

module EX_MEM (
    input clk,
    input reset,
    input [31:0] instr_in,
    output reg [31:0] instr_out
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            instr_out <= 32'b0;
        else
            instr_out <= instr_in;
    end
endmodule
