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

    // IF/ID Stage outputs (control signals only)
    wire [3:0] if_id_alu_op;
    wire if_id_load, if_id_mem_write, if_id_store_cc, if_id_b, if_id_bl, if_id_mem_size, if_id_mem_e, if_id_rf_e;
    wire [1:0] if_id_am;

    // ID/EX Stage outputs (control signals only)
    wire [3:0] id_ex_alu_op;
    wire id_ex_load, id_ex_mem_write, id_ex_store_cc, id_ex_b, id_ex_bl, id_ex_mem_size, id_ex_mem_e, id_ex_rf_e;
    wire [1:0] id_ex_am;

    // EX/MEM Stage outputs (control signals only)
    wire ex_mem_load, ex_mem_mem_write, ex_mem_mem_size, ex_mem_rf_e;

    // MEM/WB Stage outputs (control signals only)
    wire mem_wb_rf_e;

    integer fi, code;
    reg [31:0] data;       // For loading instruction data
    reg [7:0] address;     // Temporary address variable

    // Helper function to get the keyword based on opcode
    function [7*8:1] get_keyword;
        input [3:0] opcode;
        begin
            if (instruction == 32'b0)
                get_keyword = "NOP";
            else
                case (opcode)
                    4'b0000: get_keyword = "AND";
                    4'b0001: get_keyword = "EOR";
                    4'b0010: get_keyword = "SUB";
                    4'b0011: get_keyword = "RSB";
                    4'b0100: get_keyword = "ADD";
                    4'b0101: get_keyword = "ADC";
                    4'b0110: get_keyword = "SBC";
                    4'b0111: get_keyword = "RSC";
                    4'b1000: get_keyword = "TST";
                    4'b1001: get_keyword = "TEQ";
                    4'b1010: get_keyword = "CMP";
                    4'b1011: get_keyword = "CMN";
                    4'b1100: get_keyword = "ORR";
                    4'b1101: get_keyword = "MOV";
                    4'b1110: get_keyword = "BIC";
                    4'b1111: get_keyword = "MVN";
                    default: get_keyword = "NOP"; // Default to NOP if opcode is unknown
                endcase
        end
    endfunction

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
        .alu_op(if_id_alu_op),
        .id_load(if_id_load),
        .id_mem_write(if_id_mem_write),
        .store_cc(if_id_store_cc),
        .id_b(if_id_b),
        .id_bl(if_id_bl),
        .id_mem_size(if_id_mem_size),
        .id_mem_e(if_id_mem_e),
        .rf_e(if_id_rf_e),
        .id_am(if_id_am),
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

    // Instantiate the IF/ID Pipeline Register for control signals only
    IF_ID if_id (
        .clk(clk),
        .reset(reset),
        .E(enable_ifid),
        .instr_in(instruction),
        .instr_out(if_id_instruction)
    );

    // Instantiate the ID/EX Pipeline Register for control signals only
    ID_EX id_ex (
        .clk(clk),
        .reset(reset),
        .ID_ALU_OP(if_id_alu_op),
        .ID_LOAD(if_id_load),
        .ID_MEM_WRITE(if_id_mem_write),
        .ID_MEM_SIZE(if_id_mem_size),
        .ID_MEM_ENABLE(if_id_mem_e),
        .ID_AM(if_id_am),
        .STORE_CC(if_id_store_cc),
        .ID_BL(if_id_bl),
        .ID_B(if_id_b),
        .RF_ENABLE(if_id_rf_e),
        .id_alu_op(id_ex_alu_op),
        .id_load(id_ex_load),
        .id_mem_write(id_ex_mem_write),
        .id_mem_size(id_ex_mem_size),
        .id_mem_enable(id_ex_mem_e),
        .id_am(id_ex_am),
        .store_cc(id_ex_store_cc),
        .id_bl(id_ex_bl),
        .id_b(id_ex_b),
        .rf_enable(id_ex_rf_e)
    );

    // Instantiate the EX/MEM Pipeline Register for control signals only
    EX_MEM ex_mem (
        .clk(clk),
        .reset(reset),
        .ID_LOAD(id_ex_load),
        .ID_MEM_WRITE(id_ex_mem_write),
        .ID_MEM_SIZE(id_ex_mem_size),
        .ID_MEM_ENABLE(id_ex_mem_e),
        .RF_ENABLE(id_ex_rf_e),
        .id_load(ex_mem_load),
        .id_mem_size(ex_mem_mem_size),
        .id_mem_write(ex_mem_mem_write),
        .id_mem_enable(ex_mem_mem_e),
        .rf_enable(ex_mem_rf_e)
    );

    // Instantiate the MEM/WB Pipeline Register for control signals only
    MEM_WB mem_wb (
        .clk(clk),
        .reset(reset),
        .RF_ENABLE(ex_mem_rf_e),
        .rf_enable(mem_wb_rf_e)
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

    // Test sequence timing
    initial begin
        reset = 1;
        enable_pc = 1;
        enable_ifid = 1;
        S = 0;

        @(posedge clk) reset = 0;  // Release reset synchronously
        #32 S = 1;                 // Change S to 1 at time 32
        #40 $finish;               // Stop simulation at time 40
    end

    // Display outputs for each clock cycle
    always @(posedge clk) begin
        $display("PC: %0d | Opcode: %s", pc, get_keyword(instruction[24:21]));
        $display("-----------------------------------------------------------");
        $display("Fetch Stage:    Instruction: %b", if_instruction);
        $display("Decode Stage:   ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", id_ALU_OP, id_AM, id_LOAD, id_RF_E);
        $display("Execute Stage:  ALU_OP: %b | AM: %b | S: %b | Load: %b", ex_ALU_OP, ex_AM, ex_S, ex_LOAD);
        $display("Memory Stage:   Load: %b | RF_E: %b | Mem Size: %b | RW: %b", mem_LOAD, mem_RF_E, mem_SIZE, mem_RW);
        $display("Write Back:     RF_E: %b", wb_RF_E);
        $display("-----------------------------------------------------------\n");
    end
endmodule
