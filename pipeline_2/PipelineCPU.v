`timescale 1ns / 1ns

// Program Counter Module
module ProgramCounter(
    input clk,
    input reset,
    input enable,
    output reg [31:0] pc_out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_out <= 0;
        else if (enable)
            pc_out <= pc_out + 4;
    end
endmodule

// Instruction Memory Module
module InstructionMemory (
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] memory [0:255];
    initial begin
        $readmemh("instructions.hex", Mem, 0, 9); // Adjust the range as needed
    end

    always @(address) begin
        instruction = memory[address >> 2]; // Shift address to word-aligned
    end
endmodule

// Control Unit Module
module ControlUnit (
    input [31:0] instruction,
    output reg [31:0] control_signals
);
    always @(*) begin
        // Decode control signals based on opcode
        case (instruction[31:26])
            6'b000000: control_signals = 32'b00000000000000000000000000000010; // R-type
            6'b100011: control_signals = 32'b00000000000000000000000000000100; // Load
            6'b101011: control_signals = 32'b00000000000000000000000000001000; // Store
            6'b000100: control_signals = 32'b00000000000000000000000000010000; // Branch
            default:   control_signals = 32'b0;
        endcase
    end
endmodule

// IF/ID Register
module IF_ID (
    input clk,
    input reset,
    input enable,
    input [31:0] instr_in,
    input [31:0] pc_in,
    output reg [31:0] instr_out,
    output reg [31:0] pc_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_out <= 0;
            pc_out <= 0;
        end else if (enable) begin
            instr_out <= instr_in;
            pc_out <= pc_in;
        end
    end
endmodule

// ID/EX Register
module ID_EX (
    input clk,
    input reset,
    input [31:0] control_signals_in,
    input [31:0] instr_in,
    input [31:0] pc_in,
    output reg [31:0] control_signals_out,
    output reg [31:0] instr_out,
    output reg [31:0] pc_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            control_signals_out <= 0;
            instr_out <= 0;
            pc_out <= 0;
        end else begin
            control_signals_out <= control_signals_in;
            instr_out <= instr_in;
            pc_out <= pc_in;
        end
    end
endmodule

// EX/MEM Register
module EX_MEM (
    input clk,
    input reset,
    input [31:0] control_signals_in,
    input [31:0] alu_result_in,
    input [31:0] data_in,
    output reg [31:0] control_signals_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] data_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            control_signals_out <= 0;
            alu_result_out <= 0;
            data_out <= 0;
        end else begin
            control_signals_out <= control_signals_in;
            alu_result_out <= alu_result_in;
            data_out <= data_in;
        end
    end
endmodule

// MEM/WB Register
module MEM_WB (
    input clk,
    input reset,
    input [31:0] control_signals_in,
    input [31:0] alu_result_in,
    input [31:0] mem_data_in,
    output reg [31:0] control_signals_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] mem_data_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            control_signals_out <= 0;
            alu_result_out <= 0;
            mem_data_out <= 0;
        end else begin
            control_signals_out <= control_signals_in;
            alu_result_out <= alu_result_in;
            mem_data_out <= mem_data_in;
        end
    end
endmodule

// Top-level Pipeline Module
module PipelineCPU (
    input clk,
    input reset
);
    // Wires
    wire [31:0] pc, instruction, control_signals;
    wire [31:0] IF_ID_instr, IF_ID_pc;
    wire [31:0] ID_EX_instr, ID_EX_pc, ID_EX_control_signals;
    wire [31:0] EX_MEM_control_signals, EX_MEM_alu_result, EX_MEM_data;
    wire [31:0] MEM_WB_control_signals, MEM_WB_alu_result, MEM_WB_data;

    // Program Counter
    ProgramCounter pc_reg (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),
        .pc_out(pc)
    );

    // Instruction Memory
    InstructionMemory imem (
        .address(pc),
        .instruction(instruction)
    );

    // IF/ID Register
    IF_ID if_id (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),
        .instr_in(instruction),
        .pc_in(pc),
        .instr_out(IF_ID_instr),
        .pc_out(IF_ID_pc)
    );

    // Control Unit
    ControlUnit control_unit (
        .instruction(IF_ID_instr),
        .control_signals(control_signals)
    );

    // ID/EX Register
    ID_EX id_ex (
        .clk(clk),
        .reset(reset),
        .control_signals_in(control_signals),
        .instr_in(IF_ID_instr),
        .pc_in(IF_ID_pc),
        .control_signals_out(ID_EX_control_signals),
        .instr_out(ID_EX_instr),
        .pc_out(ID_EX_pc)
    );

    // EX/MEM Register (ALU results and data forwarding)
    EX_MEM ex_mem (
        .clk(clk),
        .reset(reset),
        .control_signals_in(ID_EX_control_signals),
        .alu_result_in(32'b0),  // Placeholder ALU result
        .data_in(32'b0),        // Placeholder data
        .control_signals_out(EX_MEM_control_signals),
        .alu_result_out(EX_MEM_alu_result),
        .data_out(EX_MEM_data)
    );

    // MEM/WB Register (forward ALU result or memory data)
    MEM_WB mem_wb (
        .clk(clk),
        .reset(reset),
        .control_signals_in(EX_MEM_control_signals),
        .alu_result_in(EX_MEM_alu_result),
        .mem_data_in(32'b0),  // Placeholder memory data
        .control_signals_out(MEM_WB_control_signals),
        .alu_result_out(MEM_WB_alu_result),
        .mem_data_out(MEM_WB_data)
    );
endmodule
