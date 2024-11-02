module adder(
    output reg [31:0] NextPC, 
    input [31:0] PC);

always @(*) begin
    NextPC = PC + 4; // Increment by 4
end
endmodule

module ProgramCounter (
    output reg [31:0] Qs, 
    input [31:0] Ds, 
    input enable, 
    clk, 
    reset
);
    always @(posedge clk) begin
        if (reset) begin
            Qs = 32'h00000000;
        end else if (enable) begin
            Qs = Ds;
        end
    end
endmodule

module Instruction_Memory_ROM (output reg [31:0] I, input [7:0] A);
    // Memory array for instructions
    reg [7:0] Mem [0:255];

    // Load 32-bit instruction from memory based on address
    always @(A) begin
        I <= {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};
    end
endmodule

module ControlUnit (
    output reg STORE_CC, ID_MEM_E, ID_RF_E, ID_B,
    ID_MEM_WRITE, ID_MEM_SIZE, ID_BL,
    output reg [1:0] ID_AM,
    output reg [3:0] ALU_OP,
    output reg [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
    input [31:0] instruction
);
    always @(instruction) begin
        if (instruction == 32'b00000000000000000000000000000000) begin
            STORE_CC = 0;
            ID_MEM_E = 0;
            ID_RF_E = 0;
            ID_B = 0;
            ID_MEM_WRITE = 0;
            ID_MEM_SIZE = 0;
            ID_BL = 0;
            ID_AM = 2'b00;
            ALU_OP = 4'b0000;
            ID_mnemonic0 = "N";
            ID_mnemonic1 = "O";
            ID_mnemonic2 = "P";
        end else begin
            case (instruction[27:25])
                3'b000: begin  // Data processing with immediate shift
                    if (instruction[4] == 0) begin
                        STORE_CC = instruction[20];
                        ID_MEM_E = 0;
                        ID_RF_E = 1; 
                        ID_B = 0;
                        ID_MEM_WRITE = 0;
                        ID_MEM_SIZE = 0; 
                        ID_BL = 0;
                        ID_AM = 2'b11;
                        case (instruction[24:21])
                            4'b0000: begin ALU_OP = 4'b0110; ID_mnemonic0 = "A"; ID_mnemonic1 = "N"; ID_mnemonic2 = "D"; end
                            4'b0001: begin ALU_OP = 4'b1000; ID_mnemonic0 = "E"; ID_mnemonic1 = "O"; ID_mnemonic2 = "R"; end
                            4'b0010: begin ALU_OP = 4'b0010; ID_mnemonic0 = "S"; ID_mnemonic1 = "U"; ID_mnemonic2 = "B"; end
                            4'b0100: begin ALU_OP = 4'b0000; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "D"; end
                            4'b0101: begin ALU_OP = 4'b0001; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "C"; end
                        endcase
                    end
                end
                // Other cases for different instruction types can be defined similarly.
            endcase
        end
    end        
endmodule

module MUX (
    output reg [1:0] AM,
    output reg [3:0] opcode,
    output reg S, load, RFenable, B, BL, size, ReadWrite,
    input [1:0] ID_AM,
    input [3:0] ALU_OP,
    input STORE_CC, ID_MEM_E, ID_RF_E, ID_B, ID_MEM_WRITE, ID_MEM_SIZE, ID_BL, select
);
    always @(*) begin
        if (select) begin
            AM = 2'b00;
            opcode = 4'b0000;
            S = 1'b0;
            load = 1'b0;
            RFenable = 1'b0;
            B = 1'b0;
            BL = 1'b0;
            size = 1'b0;
            ReadWrite = 1'b0;
        end else begin
            AM = ID_AM;
            opcode = ALU_OP;
            S = STORE_CC;
            load = ID_MEM_E;
            ReadWrite = ID_MEM_WRITE;
            RFenable = ID_RF_E;
            B = ID_B;
            BL = ID_BL;
            size = ID_MEM_SIZE;
        end
    end
endmodule

module IF_ID (
    input Clk, 
    input Reset,
    input IF_ID_enable,
    input [31:0] IF_instruction, // Instruction from IF stage
    output reg [31:0] ID_instruction // Instruction to pass to ID stage
);
    always @(posedge Clk) begin
        if (Reset) begin
            ID_instruction <= 32'b0; // Clear instruction on reset
        end else if (IF_ID_enable) begin
            ID_instruction <= IF_instruction; // Pass instruction to ID stage
        end
    end
endmodule

module ID_EX (
    input Clk,
    input Reset,
    input STORE_CC,
    input [3:0] ALU_OP,
    input ID_MEM_E,
    input ID_RF_E,
    input ID_MEM_WRITE,
    input ID_MEM_SIZE,
    input ID_BL,
    input ID_B,
    input [1:0] ID_AM,
    output reg EX_STORE_CC,
    output reg [3:0] EX_alu_op,
    output reg EX_MEM_E,
    output reg EX_RF_E,
    output reg EX_MEM_WRITE,
    output reg EX_MEM_SIZE,
    output reg EX_BL,
    output reg EX_B,
    output reg [1:0] EX_AM
);
    always @(posedge Clk) begin
        if (Reset) begin
            EX_STORE_CC <= 1'b0;
            EX_alu_op <= 4'b0000;
            EX_MEM_E <= 1'b0;
            EX_RF_E <= 1'b0;
            EX_MEM_WRITE <= 1'b0;
            EX_MEM_SIZE <= 1'b0;
            EX_BL <= 1'b0;
            EX_AM <= 2'b00;
            EX_B <= 0;
        end else begin
            EX_STORE_CC <= STORE_CC;
            EX_alu_op <= ALU_OP;
            EX_MEM_E <= ID_MEM_E;
            EX_RF_E <= ID_RF_E;
            EX_MEM_WRITE <= ID_MEM_WRITE;
            EX_MEM_SIZE <= ID_MEM_SIZE;
            EX_BL <= ID_BL;
            EX_AM <= ID_AM;
            EX_B <= ID_B;
        end
    end
endmodule

module EX_MEM (
    input Clk,
    input Reset,
    input EX_MEM_WRITE,
    input EX_MEM_SIZE,
    input EX_RF_E,
    input EX_MEM_E,
    output reg MEM_WRITE,
    output reg MEM_size,
    output reg MEM_RF_E,
    output reg MEM_E
);
    always @(posedge Clk) begin
        if (Reset) begin
            MEM_WRITE <= 1'b0;
            MEM_size <= 1'b0;
            MEM_RF_E <= 1'b0;
        end else begin
            MEM_WRITE <= EX_MEM_WRITE;
            MEM_size <= EX_MEM_SIZE;
            MEM_RF_E <= EX_RF_E;
        end
    end
endmodule

module MEM_WB (
    input Clk,
    input Reset,
    input MEM_RF_E,
    output reg WB_RF_E
);
    always @(posedge Clk) begin
        if (Reset) begin
            WB_RF_E <= 1'b0;
        end else begin
            WB_RF_E <= MEM_RF_E;
        end
    end
endmodule