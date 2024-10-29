`timescale 1ns / 1ns

module ARM_Pipeline (
    input wire Clk, Reset
);

    // Program Counter and Instruction Memory
    reg [31:0] PC;
    wire [31:0] instruction;
    wire [31:0] pc_plus_4;

    // Pipeline Registers
    reg [31:0] IF_ID_instruction, IF_ID_PC;
    reg [31:0] ID_EX_instruction, ID_EX_PC;
    reg [31:0] EX_MEM_instruction, EX_MEM_alu_out, EX_MEM_data;
    reg [31:0] MEM_WB_data, MEM_WB_alu_out;

    // Control Signals
    wire [31:0] control_signals;
    reg [31:0] EX_control_signals, MEM_control_signals, WB_control_signals;

    // PC increment by 4
    assign pc_plus_4 = PC + 4;

    // Program Counter with enable and reset
    always @(posedge Clk) begin
        if (Reset)
            PC <= 0;
        else
            PC <= pc_plus_4;
    end

    // Instruction Memory Module
    Instruction_Memory_ROM imem (
        .A(PC[7:0]), 
        .I(instruction)
    );

    // IF/ID Register
    always @(posedge Clk) begin
        if (Reset) begin
            IF_ID_instruction <= 32'b0;
            IF_ID_PC <= 32'b0;
        end else begin
            IF_ID_instruction <= instruction;
            IF_ID_PC <= pc_plus_4;
        end
    end

    // Control Unit
    Control_Unit control_unit (
        .instruction(IF_ID_instruction),
        .Clk(Clk),
        .Reset(Reset),
        .control_signals(control_signals)
    );

    // ID/EX Register
    always @(posedge Clk) begin
        if (Reset) begin
            ID_EX_instruction <= 32'b0;
            ID_EX_PC <= 32'b0;
            EX_control_signals <= 32'b0;
        end else begin
            ID_EX_instruction <= IF_ID_instruction;
            ID_EX_PC <= IF_ID_PC;
            EX_control_signals <= control_signals;
        end
    end

    // ALU and Shifter
    wire [31:0] alu_out;
    wire [31:0] shifter_out;
    wire Z, N, C, V;

    ARM_ALU alu (
        .A(ID_EX_instruction[19:0]),  // Use 32-bit A input for ALU
        .B(ID_EX_instruction[3:0]),    // Use 32-bit B input for ALU
        .CIN(1'b0),
        .OP(EX_control_signals[3:0]),  // ALU opcode
        .Out(alu_out),
        .Z(Z), .N(N), .C(C), .V(V)
    );

    ARM_Shifter shifter (
        .Rm(ID_EX_instruction[31:0]),  // Shift amount or immediate
        .I(ID_EX_instruction[11:0]),
        .AM(EX_control_signals[1:0]),  // Addressing mode
        .N(shifter_out)
    );

    // EX/MEM Register
    always @(posedge Clk) begin
        if (Reset) begin
            EX_MEM_instruction <= 32'b0;
            EX_MEM_alu_out <= 32'b0;
            EX_MEM_data <= 32'b0;
            MEM_control_signals <= 32'b0;
        end else begin
            EX_MEM_instruction <= ID_EX_instruction;
            EX_MEM_alu_out <= alu_out;
            EX_MEM_data <= shifter_out;
            MEM_control_signals <= EX_control_signals;
        end
    end

    // Data Memory Module
    wire [31:0] data_memory_out;

    Data_Memory_RAM dmem (
        .address(EX_MEM_alu_out[7:0]),
        .data_in(EX_MEM_data),
        .data_out(data_memory_out),
        .size(2'b01),  // Word size
        .rw(MEM_control_signals[0]),  // Read/Write control
        .enable(1'b1)
    );

    // MEM/WB Register
    always @(posedge Clk) begin
        if (Reset) begin
            MEM_WB_data <= 32'b0;
            MEM_WB_alu_out <= 32'b0;
            WB_control_signals <= 32'b0;
        end else begin
            MEM_WB_data <= data_memory_out;
            MEM_WB_alu_out <= EX_MEM_alu_out;
            WB_control_signals <= MEM_control_signals;
        end
    end

endmodule
module Instruction_Memory_ROM (
    input [7:0] A,
    output reg [31:0] I
);
    reg [7:0] Mem [0:255];
    
    initial begin
        // Load instructions into memory from a hex file
        $readmemh("instructions.hex", Mem);
    end

    always @(A) begin
        I = {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};
    end
endmodule
