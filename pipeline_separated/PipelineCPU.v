
`timescale 1ns / 1ns

module PipelineCPU (
    input clk,
    input reset,
    output [31:0] PC,
    output [31:0] IF_ID_instr,
    output [31:0] ID_EX_instr,
    output [31:0] EX_MEM_instr,
    output [31:0] MEM_WB_instr
);

    // Program Counter
    reg [31:0] pc_reg;
    assign PC = pc_reg;

    // Instruction memory output
    wire [31:0] instr;

    // Program Counter update
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 32'b0;
        else
            pc_reg <= pc_reg + 4;
    end

    // Instruction Memory Module
    Instruction_Memory instr_mem (
        .address(pc_reg[7:0]),
        .instruction(instr)
    );

    // Pipeline registers for each stage
    wire [31:0] IF_ID_pc;
    wire [31:0] ID_EX_pc;

    // IF/ID stage
    IF_ID if_id (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),
        .instr_in(instr),
        .pc_in(pc_reg),
        .instr_out(IF_ID_instr),
        .pc_out(IF_ID_pc)
    );

    // ID/EX stage
    ID_EX id_ex (
        .clk(clk),
        .reset(reset),
        .instr_in(IF_ID_instr),
        .pc_in(IF_ID_pc),
        .instr_out(ID_EX_instr),
        .pc_out(ID_EX_pc)
    );

    // EX/MEM stage
    EX_MEM ex_mem (
        .clk(clk),
        .reset(reset),
        .instr_in(ID_EX_instr),
        .instr_out(EX_MEM_instr)
    );

    // MEM/WB stage
    MEM_WB mem_wb (
        .clk(clk),
        .reset(reset),
        .instr_in(EX_MEM_instr),
        .instr_out(MEM_WB_instr)
    );

endmodule
