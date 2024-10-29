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

    // Multiplexer select signal (no external control needed)
    reg S;

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
        .A({12'b0, ID_EX_instruction[19:16]}),  // Expanding to 32 bits
        .B({28'b0, ID_EX_instruction[3:0]}),    // Expanding to 32 bits
        .CIN(1'b0),
        .OP(EX_control_signals[3:0]),  // ALU opcode
        .Out(alu_out),
        .Z(Z), .N(N), .C(C), .V(V)
    );

    ARM_Shifter shifter (
        .Rm(ID_EX_instruction[11:0]),  // Shift amount or immediate
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
module ARM_ALU (
    input [31:0] A, B,
    input CIN,
    input [3:0] OP,
    output reg [31:0] Out,
    output reg Z, N, C, V
);
    always @(*) begin
        Out = 32'b0; Z = 0; N = 0; C = 0; V = 0;
        case (OP)
            4'b0000: {C, Out} = A & B;                  // AND
            4'b0001: {C, Out} = A ^ B;                  // EOR
            4'b0010: {C, Out} = A - B;                  // SUB
            4'b0011: {C, Out} = B - A;                  // RSB
            4'b0100: {C, Out} = A + B;                  // ADD
            4'b0101: {C, Out} = A + B + CIN;            // ADC
            4'b0110: {C, Out} = A - B - !CIN;           // SBC
            4'b0111: {C, Out} = B - A - !CIN;           // RSC
            4'b1000: Out = A & B;                       // TST (for flag update)
            4'b1001: Out = A ^ B;                       // TEQ
            4'b1010: {C, Out} = A - B;                  // CMP
            4'b1011: {C, Out} = A + B;                  // CMN
            4'b1100: Out = A | B;                       // ORR
            4'b1101: Out = B;                           // MOV
            4'b1110: Out = A & ~B;                      // BIC
            4'b1111: Out = ~B;                          // MVN
            default: Out = 32'b0;
        endcase

        Z = (Out == 32'b0);  // Zero flag
        N = Out[31];         // Negative flag
        // Overflow and carry detection only for ADD, SUB, etc.
    end
endmodule
module Control_Unit (
    input [31:0] instruction,
    input Clk, Reset,
    output reg [31:0] control_signals
);
    reg Z, N, C, V;  // Flags for condition checking
    wire [3:0] cond = instruction[31:28];

    always @(*) begin
        // Check conditions based on cond field
        case (cond)
            4'b0000: control_signals = (Z == 1) ? 32'b1 : 32'b0;  // EQ
            4'b0001: control_signals = (Z == 0) ? 32'b1 : 32'b0;  // NE
            4'b0010: control_signals = (C == 1) ? 32'b1 : 32'b0;  // CS/HS
            4'b0011: control_signals = (C == 0) ? 32'b1 : 32'b0;  // CC/LO
            // ... (more cases based on condition codes)
            4'b1110: control_signals = 32'b1;                      // AL (Always)
            default: control_signals = 32'b0;
        endcase
    end
endmodule
module ARM_Shifter (
    input [31:0] Rm,
    input [11:0] I,
    input [1:0] AM,
    output reg [31:0] N
);
    always @(*) begin
        case (AM)
            2'b00: N = ({24'b0, I[7:0]} >> (2 * I[11:8])) | ({24'b0, I[7:0]} << (32 - 2 * I[11:8]));  // Rotate right
            2'b01: N = Rm;                                    // Pass through
            2'b10: N = {20'b0, I};                            // Zero extend
            2'b11: case (I[6:5])
                2'b00: N = Rm << I[11:7];                    // LSL
                2'b01: N = Rm >> I[11:7];                    // LSR
                2'b10: N = $signed(Rm) >>> I[11:7];          // ASR
                2'b11: N = {Rm, Rm} >> I[11:7];              // ROR
            endcase
        endcase
    end
endmodule
module Data_Memory_RAM (
    input [7:0] address,
    input [31:0] data_in,
    output reg [31:0] data_out,
    input [1:0] size,
    input rw,
    input enable
);
    reg [7:0] Mem [0:255];

    always @(address or rw or enable) begin
        if (rw == 0 && enable) begin
            data_out <= {Mem[address], Mem[address+1], Mem[address+2], Mem[address+3]};
        end else if (rw == 1 && enable) begin
            {Mem[address], Mem[address+1], Mem[address+2], Mem[address+3]} <= data_in;
        end
    end
endmodule
