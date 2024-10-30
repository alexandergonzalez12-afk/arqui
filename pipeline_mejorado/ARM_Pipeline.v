`timescale 1ns / 1ns

module ARM_Pipeline (
    input clk,
    input reset,
    input [7:0] instr_address,  // Address input for the instruction memory
    output [31:0] pc_out        // Output the program counter for observation
);

    // Wires for inter-stage communication
    wire [31:0] instr_IF_ID, pc_IF_ID;
    wire [31:0] instr_ID_EX, pc_ID_EX;
    wire [31:0] instr_EX_MEM;
    wire [31:0] instr_MEM_WB;

    // Control signals
    wire [3:0] ID_ALU_op;
    wire ID_Load, ID_MEM_WRITE, ID_AM, STORE_CC, ID_BL, ID_B, ID_MEM_ENABLE, RF_ENABLE;
    wire [1:0] ID_MEM_SIZE;

    // IF Stage: Instruction Fetch
    wire [31:0] instruction;
    Instruction_Memory instruction_memory (
        .address(instr_address),
        .instruction(instruction)
    );

    // IF/ID Pipeline Register
    IF_ID if_id (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),               // Assume always enabled; you could add logic here
        .instr_in(instruction),
        .pc_in(instr_address),       // Using instr_address as PC for simplicity
        .instr_out(instr_IF_ID),
        .pc_out(pc_IF_ID)
    );

    // ID Stage: Instruction Decode & Control Unit
    Control_Unit control_unit (
        .instruction(instr_IF_ID),
        .ID_ALU_op(ID_ALU_op),
        .ID_Load(ID_Load),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .ID_AM(ID_AM),
        .STORE_CC(STORE_CC),
        .ID_BL(ID_BL),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_B(ID_B),
        .ID_MEM_ENABLE(ID_MEM_ENABLE),
        .RF_ENABLE(RF_ENABLE)
    );

    // ID/EX Pipeline Register
    ID_EX id_ex (
        .clk(clk),
        .reset(reset),
        .instr_in(instr_IF_ID),
        .pc_in(pc_IF_ID),
        .instr_out(instr_ID_EX),
        .pc_out(pc_ID_EX)
    );

    // EX Stage: Execute
    // Note: This example doesn’t have a full ALU; add ALU logic as needed.
    assign instr_EX_MEM = instr_ID_EX;  // Passing instruction directly for simplicity

    // EX/MEM Pipeline Register
    EX_MEM ex_mem (
        .clk(clk),
        .reset(reset),
        .instr_in(instr_ID_EX),
        .instr_out(instr_EX_MEM)
    );

    // MEM Stage: Memory Access
    // Control signals could determine memory behavior; placeholder only
    assign instr_MEM_WB = instr_EX_MEM; // Passing through for now

    // MEM/WB Pipeline Register
    MEM_WB mem_wb (
        .clk(clk),
        .reset(reset),
        .instr_in(instr_EX_MEM),
        .instr_out(instr_MEM_WB)
    );

    // Output the final instruction in the pipeline for observation (simulating WB stage)
    assign pc_out = pc_IF_ID;  // Output PC from IF/ID register for monitoring

endmodule
