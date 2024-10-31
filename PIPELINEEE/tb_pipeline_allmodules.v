`timescale 1ns / 1ps

module tb_pipeline;

  // Inputs
  reg clk;
  reg reset;
  reg enable_pc;
  reg enable_ifid;
  reg S;  // Multiplexer select

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

  // Define the instruction memory array
  reg [31:0] instruction_memory [0:12];

  // Helper function to get the keyword based on opcode
  function [7*8:1] get_keyword;
    input [7:0] pc_value;
    input [3:0] opcode;
    begin
      if (pc_value == 8'd0) 
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
          default: get_keyword = "UNKNOWN";
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

  // Clock generation with 2 time units toggle
  initial begin
    clk = 0;
    forever #2 clk = ~clk;
  end

  // Initialize the instruction memory with given instructions
  initial begin
    instruction_memory[0] = 32'b11100010_00010001_00000000_00000000;
    instruction_memory[1] = 32'b11100000_10000000_01010001_10000011;
    instruction_memory[2] = 32'b11100111_11010001_00100000_00000000;
    instruction_memory[3] = 32'b11100101_10001010_01010000_00000000;
    instruction_memory[4] = 32'b00011010_11111111_11111111_11111101;
    instruction_memory[5] = 32'b11011011_00000000_00000000_00001001;
    instruction_memory[6] = 32'b11100010_00000001_00000000_00000000;
    instruction_memory[7] = 32'b11100000_10000000_01010001_10000011;
    instruction_memory[8] = 32'b11100111_11010001_00100000_00000000;
    instruction_memory[9] = 32'b11100101_10001010_01010000_00000000;
    instruction_memory[10] = 32'b00011010_11111111_11111111_11111101;
    instruction_memory[11] = 32'b00000000_00000000_00000000_00000000;
    instruction_memory[12] = 32'b00000000_00000000_00000000_00000000;
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
    #5 $finish; // Stop simulation at time 40
  end

  // Fetch instruction based on PC
  assign instruction = (pc == 8'd0) ? 32'b00000000_00000000_00000000_00000000 : instruction_memory[pc >> 2]; // NOP if PC is 0

  // Display outputs for each clock cycle with divided stages
  always @(posedge clk) begin
    $display("PC: %0d | Opcode: %s", pc, get_keyword(pc, instruction[24:21]));
    $display("-----------------------------------------------------------");
    $display("Fetch Stage:    Instruction: %b", instruction);
    $display("Decode Stage:   ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", ALU_OP, ID_AM, ID_LOAD, RF_E);
    $display("Execute Stage:  ALU_OP: %b | AM: %b | S: %b | Load: %b", mux_alu_op, mux_id_am, S, mux_id_load);
    $display("Memory Stage:   Load: %b | RF_E: %b | Mem Size: %b | RW: %b", mux_id_load, mux_rf_e, mux_id_mem_size, mux_id_mem_write);
    $display("Write Back:     RF_E: %b", mux_rf_e);
    $display("-----------------------------------------------------------\n");
  end
endmodule
