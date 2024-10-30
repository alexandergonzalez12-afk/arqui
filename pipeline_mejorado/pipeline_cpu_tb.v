`timescale 1ns / 1ns

module pipeline_cpu_tb;

    reg clk;
    reg reset;
    reg S;           // Control signal for multiplexer
    reg enable;      // Enable signal for PC and IF/ID registers
    integer file, r, i;
    reg [31:0] line_data; // Temporary variable for each instruction line

    // Instantiate the CPU (assuming top-level module is ARM_Pipeline)
    ARM_Pipeline cpu (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation: toggle every 1 time unit (produces a 2-time unit clock period)
    always #1 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        S = 0;
        enable = 1; // Enable the PC and IF/ID registers during this test
        i = 0;

        // Open the file and load instructions
        file = $fopen("codigo_validacion.txt", "r");
        if (file == 0) begin
            $display("Error: File could not be opened.");
            $finish;
        end

        // Load instructions from the file into the Instruction Memory
        while (!$feof(file)) begin
            r = $fscanf(file, "%b\n", line_data);
            if (r == 1) begin
                cpu.instr_mem.Mem[i] = line_data; // Store instruction in memory array
                i = i + 1;
            end else begin
                $display("Error: Malformed line in file.");
            end
        end
        $fclose(file);

        // Initialize remaining memory locations to zero
        for (i = i; i < 256; i = i + 1) begin
            cpu.instr_mem.Mem[i] = 32'b0;
        end

        // Simulation timeline as per document
        #3 reset = 0;   // Deactivate reset at time 3
        #32 S = 1;      // Set S to 1 at time 32
        #40 $finish;    // End simulation at time 40
    end

    // Header for the display output to match format
    initial begin
        $display("CLK | Keyword | PC    | opcode | am | b | bl | S | load | rf_e | size | rw | e | EX_opcode | EX_am | EX_S | EX_Tload | EX_rf_e | EX_size | EX_rw | EX_e | MEM_Load | MEM_rf_e | MEM_size | MEM_rw | MEM_e | WB_rf_e");
        $display("-----------------------------------------------------------------------------------------------------------");
    end

    // Display output for each clock cycle in the specified format
    always @(posedge clk) begin
        $display("%b    | %s  | %d | %b | %b | %b | %b | %b | %b | %b | %b | %b | %b | %b      | %b   | %b   | %b     | %b     | %b    | %b  | %b  | %b      | %b      | %b     | %b    | %b   | %b",
            clk,
            get_instruction_keyword(cpu.instr_if_id),
            cpu.PC,
            cpu.control_unit.ID_ALU_op, cpu.control_unit.ID_AM, cpu.control_unit.ID_B,
            cpu.control_unit.ID_BL, S,
            cpu.control_unit.ID_Load, cpu.control_unit.RF_ENABLE,
            cpu.control_unit.ID_MEM_SIZE, cpu.control_unit.ID_MEM_WRITE,
            enable,
            cpu.EX_MEM.EX_opcode, cpu.EX_MEM.EX_am, cpu.EX_MEM.EX_S,
            cpu.EX_MEM.EX_Tload, cpu.EX_MEM.EX_rf_e, cpu.EX_MEM.EX_size,
            cpu.EX_MEM.EX_rw, cpu.EX_MEM.EX_e,
            cpu.MEM_WB.MEM_Load, cpu.MEM_WB.MEM_rf_e, cpu.MEM_WB.MEM_size,
            cpu.MEM_WB.MEM_rw, cpu.MEM_WB.MEM_e, cpu.MEM_WB.WB_rf_e
        );
    end

// Function to get the instruction keyword (AND, EOR, SUB, etc.) based on opcode at bits 24:21
function [8*4:1] get_instruction_keyword;
    input [31:0] instruction;
    case (instruction[24:21]) // Opcode located in bits 24:21
        4'b0000: get_instruction_keyword = "AND";
        4'b0001: get_instruction_keyword = "EOR";
        4'b0010: get_instruction_keyword = "SUB";
        4'b0011: get_instruction_keyword = "RSB";
        4'b0100: get_instruction_keyword = "ADD";
        4'b0101: get_instruction_keyword = "ADC";
        4'b0110: get_instruction_keyword = "SBC";
        4'b0111: get_instruction_keyword = "RSC";
        4'b1000: get_instruction_keyword = "TST";
        4'b1001: get_instruction_keyword = "TEQ";
        4'b1010: get_instruction_keyword = "CMP";
        4'b1011: get_instruction_keyword = "CMN";
        4'b1100: get_instruction_keyword = "ORR";
        4'b1101: get_instruction_keyword = "MOV";
        4'b1110: get_instruction_keyword = "BIC";
        4'b1111: get_instruction_keyword = "MVN";
        default: get_instruction_keyword = "NOP";
    endcase
endfunction

