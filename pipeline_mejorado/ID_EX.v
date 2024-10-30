
`timescale 1ns / 1ns

module ID_EX (
    input clk,
    input reset,
    input [31:0] instr_in,
    input [31:0] pc_in,
    output reg [31:0] instr_out,
    output reg [31:0] pc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_out <= 32'b0;
            pc_out <= 32'b0;
        end else begin
            instr_out <= instr_in;
            pc_out <= pc_in;
        end
    end
endmodule
