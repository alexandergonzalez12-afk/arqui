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

    // Pipeline registers for each stage
    reg [31:0] if_instruction;
    reg [3:0] id_ALU_OP;
    reg [1:0] id_AM;
    reg id_LOAD, id_RF_E;
    reg [3:0] ex_ALU_OP;
    reg [1:0] ex_AM;
    reg ex_S, ex_LOAD;
    reg mem_LOAD, mem_RF_E, mem_SIZE, mem_RW;
    reg wb_RF_E;

    // Control signals from ControlUnit
    wire [3:0] ALU_OP;
    wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E;
    wire [1:0] ID_AM;

    // Outputs from Multiplexer
    wire [3:0] mux_alu_op;
    wire mux_id_load, mux_id_mem_write, mux_store_cc, mux_id_b, mux_id_bl, mux_id_mem_size, mux_id_mem_e, mux_rf_e;
    wire [1:0] mux_id_am;

    // Registers for monitoring
    reg [31:0] r1, r2, r3, r5;

    integer fi, code;
    reg [31:0] data;       // For loading instruction data
    reg [7:0] address;     // Temporary address variable

    // Helper function to get the keyword based on opcode
    function [7*8:1] get_keyword;
      input [3:0] opcode;
      begin
        if(instruction == 32'b0)
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

    // Instantiate the data memory (RAM)
    Data_Memory_RAM data_mem_inst (
        .data_out(data_out),
        .address(dm_address),
        .data_in(pd),
        .size(mem_SIZE),
        .rw(mem_RW),
        .enable(mem_RF_E)
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

    // Preload data memory from the file
    initial begin
        fi = $fopen("file_precarga_fase_I.txt", "r");
        if (fi == 0) begin
            $display("Error: File could not be opened.");
            $finish;
        end

        // Start loading data into the memory
        address = 8'd0;
        while (!$feof(fi)) begin
            code = $fscanf(fi, "%b", data);
            data_mem_inst.Mem[address] = data; // Preload the RAM memory
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

    // Pipeline stages update
    always @(posedge clk) begin
        // IF stage
        if_instruction <= instruction;

        // ID stage
        id_ALU_OP <= ALU_OP;
        id_AM <= ID_AM;
        id_LOAD <= ID_LOAD;
        id_RF_E <= RF_E;

        // EX stage
        ex_ALU_OP <= mux_alu_op;
        ex_AM <= mux_id_am;
        ex_S <= S;
        ex_LOAD <= mux_id_load;

        // MEM stage
        mem_LOAD <= mux_id_load;
        mem_RF_E <= mux_rf_e;
        mem_SIZE <= mux_id_mem_size;
        mem_RW <= mux_id_mem_write;

        // WB stage
        wb_RF_E <= mux_rf_e;
    end

    // Monitor outputs for each clock cycle
    initial begin
        $monitor("PC: %0d | Address: %0d | r1: %0d | r2: %0d | r3: %0d | r5: %0d", pc, address, r1, r2, r3, r5);
    end

    // Display control signals and instruction in ID stage when required
    always @(posedge clk) begin
        $display("Control Signals in ID Stage: ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", id_ALU_OP, id_AM, id_LOAD, id_RF_E);
        $display("Instruction in ID Stage: %b", if_instruction);
    end
endmodule