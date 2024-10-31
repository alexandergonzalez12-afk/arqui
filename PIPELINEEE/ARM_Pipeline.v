module ARM_Pipeline;
    reg clk, reset;
    reg [7:0] pc;
    wire [31:0] instruction;
    
    // Control signals
    wire [3:0] ALU_op;
    wire [1:0] am;
    wire b, b1, S, load, rf_e, size, rw, e;

    // EX stage control signals
    wire [3:0] EX_opcode;
    wire [1:0] EX_am;
    wire EX_s, EX_load, EX_rf_e, EX_size, EX_rw, EX_e;

    // MEM stage control signals
    wire MEM_load, MEM_rf_e, MEM_size, MEM_rw, MEM_e;

    // WB stage control signals
    wire WB_rf_e;

    // Instantiate the Instruction Memory with preloaded instructions
    Instruction_Memory imem (
        .instruction(instruction),
        .address(pc)
    );

    // Control Unit
    Control_Unit ctrl (
        .instruction(instruction),
        .ALU_op(ALU_op),
        .am(am),
        .b(b),
        .b1(b1),
        .S(S),
        .load(load),
        .rf_e(rf_e),
        .size(size),
        .rw(rw),
        .e(e)
    );

    // Pipeline Registers
    reg [3:0] EX_opcode_reg;
    reg [1:0] EX_am_reg;
    reg EX_s_reg, EX_load_reg, EX_rf_e_reg, EX_size_reg, EX_rw_reg, EX_e_reg;
    reg MEM_load_reg, MEM_rf_e_reg, MEM_size_reg, MEM_rw_reg, MEM_e_reg;
    reg WB_rf_e_reg;

    // Clock signal generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Simulation control
    initial begin
        reset = 1;
        pc = 0;
        #3 reset = 0;
        
        // Run the simulation for 200 time units
        #200 $finish;
    end

    // Update PC and control signals
    always @(posedge clk) begin
        if (!reset) pc <= pc + 4;
        
        // EX stage signals
        EX_opcode_reg <= ALU_op;
        EX_am_reg <= am;
        EX_s_reg <= S;
        EX_load_reg <= load;
        EX_rf_e_reg <= rf_e;
        EX_size_reg <= size;
        EX_rw_reg <= rw;
        EX_e_reg <= e;

        // MEM stage signals
        MEM_load_reg <= EX_load_reg;
        MEM_rf_e_reg <= EX_rf_e_reg;
        MEM_size_reg <= EX_size_reg;
        MEM_rw_reg <= EX_rw_reg;
        MEM_e_reg <= EX_e_reg;

        // WB stage signals
        WB_rf_e_reg <= MEM_rf_e_reg;
    end

    // Display output
    always @(posedge clk) begin
        $display("Time: %3d | PC: %2d | Instruction: %b | ALU_op: %4b | am: %2b | b: %1b | b1: %1b | S: %1b | load: %1b | rf_e: %1b | size: %1b | rw: %1b | e: %1b",
                 $time, pc, instruction, ALU_op, am, b, b1, S, load, rf_e, size, rw, e);
        $display("EX_stage | EX_opcode: %4b | EX_am: %2b | EX_s: %1b | EX_load: %1b | EX_rf_e: %1b | EX_size: %1b | EX_rw: %1b | EX_e: %1b",
                 EX_opcode_reg, EX_am_reg, EX_s_reg, EX_load_reg, EX_rf_e_reg, EX_size_reg, EX_rw_reg, EX_e_reg);
        $display("MEM_stage | MEM_load: %1b | MEM_rf_e: %1b | MEM_size: %1b | MEM_rw: %1b | MEM_e: %1b",
                 MEM_load_reg, MEM_rf_e_reg, MEM_size_reg, MEM_rw_reg, MEM_e_reg);
        $display("WB_stage | WB_rf_e: %1b\n", WB_rf_e_reg);
    end
endmodule

// Instruction Memory with preloaded instructions
module Instruction_Memory (
    output reg [31:0] instruction,
    input [7:0] address
);
    reg [31:0] memory [0:15];

    // Preload instructions
    initial begin
        memory[0] = 32'b11100010000100010000000000000000;
        memory[1] = 32'b11100000100000000101000110000011;
        memory[2] = 32'b11100111110100010010000000000000;
        memory[3] = 32'b11100101100010100101000000000000;
        memory[4] = 32'b00011010111111111111111111111101;
        memory[5] = 32'b11011011000000000000000000001001;
        memory[6] = 32'b11100010000000010000000000000000;
        memory[7] = 32'b11100000100000000101000110000011;
        memory[8] = 32'b11100111110100010010000000000000;
        memory[9] = 32'b11100101100010100101000000000000;
        memory[10] = 32'b00011010111111111111111111111101;
        memory[11] = 32'b00000000000000000000000000000000;
        memory[12] = 32'b00000000000000000000000000000000;
        memory[13] = 32'b00000000000000000000000000000000;
        memory[14] = 32'b00000000000000000000000000000000;
        memory[15] = 32'b00000000000000000000000000000000;
    end

    always @(address) begin
        instruction <= memory[address[3:0]];
    end
endmodule

// Control Unit
module Control_Unit (
    input [31:0] instruction,
    output [3:0] ALU_op,
    output [1:0] am,
    output b, b1, S, load, rf_e, size, rw, e
);
    // Implement your control unit logic here based on instruction decoding
    assign ALU_op = instruction[24:21];
    assign am = instruction[27:26];
    assign b = instruction[25];
    assign b1 = instruction[4];
    assign S = instruction[20];
    assign load = (instruction[27:26] == 2'b01) ? 1 : 0;
    assign rf_e = (instruction[27:26] == 2'b00) ? 1 : 0;
    assign size = instruction[7];
    assign rw = instruction[11];
    assign e = 1'b1; // Set enable to 1 for this phase
endmodule
