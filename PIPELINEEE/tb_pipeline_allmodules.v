`timescale 1ns / 1ps

module tb_pipeline;

    // Inputs
    reg clk;
    reg reset;
    reg enable_pc;
    reg enable_ifid;
    reg S; // Multiplexer select

    // Outputs
    wire [7:0] pc;
    wire [31:0] instruction;

    // Control signals from ControlUnit
    wire [3:0] ALU_OP;
    wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E;
    wire [1:0] ID_AM;

    // Outputs from Multiplexer
    wire [3:0] mux_alu_op;
    wire mux_id_load, mux_id_mem_write, mux_store_cc, mux_id_b, mux_id_bl, mux_id_mem_size, mux_id_mem_e, mux_rf_e;
    wire [1:0] mux_id_am;

    integer fi, code;
    reg [31:0] data;       // For loading instruction data
    reg [7:0] address;     // Temporary address variable

    // Instantiate the PC module with PC increment of 4
    PC uut_pc (
        .clk(clk),
        .reset(reset),
        .E(enable_pc),
        .next_pc(pc + 8'd4), // Increment PC by 4
        .pc(pc)
    );

    // Instantiate the ControlUnit module
    ControlUnit uut_control (
        .instruction(instruction),
        .ALU_OP(ALU_OP),
        .ID_LOAD(ID_LOAD),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .ID_AM(ID_AM),
        .STORE_CC(STORE_CC),
        .ID_B(ID_B),
        .ID_BL(ID_BL),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_MEM_E(ID_MEM_E),
        .RF_E(RF_E)
    );

    // Instantiate the Multiplexer
    Multiplexer uut_mux (
        .alu_op(mux_alu_op),
        .id_load(mux_id_load),
        .id_mem_write(mux_id_mem_write),
        .store_cc(mux_store_cc),
        .id_b(mux_id_b),
        .id_bl(mux_id_bl),
        .id_mem_size(mux_id_mem_size),
        .id_mem_e(mux_id_mem_e),
        .rf_e(mux_rf_e),
        .id_am(mux_id_am),
        .S(S),
        .ALU_OP(ALU_OP),
        .ID_LOAD(ID_LOAD),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .STORE_CC(STORE_CC),
        .ID_B(ID_B),
        .ID_BL(ID_BL),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_MEM_E(ID_MEM_E),
        .RF_E(RF_E),
        .ID_AM(ID_AM)
    );

    // Instantiate the instruction memory (ROM)
    Instruction_Memory_ROM rom_inst (
        .I(instruction),
        .A(pc) // Connect the program counter to the memory address
    );

    // Clock generation with 2 time units toggle
    initial begin
        clk = 0;
        forever #2 clk = ~clk;
    end

    // Preload instructions from the file into the instruction memory
    initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: File could not be opened.");
            $finish;
        end

        // Start loading instructions from the file
        address = 8'd0;
        while (!$feof(fi)) begin
           code = $fscanf(fi, "%b", data);
            rom_inst.Mem[address] = data; // Preload the ROM memory
            address = address + 1;
        end
        $fclose(fi);

    end

    // Test sequence with enforced stop time at 40
    initial begin
        // Initialize signals
        reset = 1;
        enable_pc = 1;
        enable_ifid = 1;
        S = 0;

        // Start simulation
        #3 reset = 0;
        #32 S = 1;
        #20 $finish; // Stop simulation at time 40
    end

    // Display outputs for each clock cycle
    always @(posedge clk) begin
        $display("PC: %0d | Instruction: %b", pc, instruction);
        $display("-----------------------------------------------------------");
        $display("Fetch Stage:    Instruction: %b", instruction);
        $display("Decode Stage:   ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", ALU_OP, ID_AM, ID_LOAD, RF_E);
        $display("Execute Stage:  ALU_OP: %b | AM: %b | S: %b | Load: %b", mux_alu_op, mux_id_am, S, mux_id_load);
        $display("Memory Stage:   Load: %b | RF_E: %b | Mem Size: %b | RW: %b", mux_id_load, mux_rf_e, mux_id_mem_size, mux_id_mem_write);
        $display("Write Back:     RF_E: %b", mux_rf_e);
        $display("-----------------------------------------------------------\n");
    end
endmodule
