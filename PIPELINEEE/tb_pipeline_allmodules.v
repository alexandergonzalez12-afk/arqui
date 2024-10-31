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

  // Instantiate the PC module
  PC uut_pc (
    .clk(clk),
    .reset(reset),
    .E(enable_pc),
    .next_pc(pc + 8'd1), // Assume it increments for this example
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

  // Clock generation
  always #1 clk = ~clk;

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

  // Test sequence
  initial begin
    // Initialize signals
    clk = 0;
    reset = 1;
    enable_pc = 1;
    enable_ifid = 1;
    S = 0;

    // Start simulation
    #3 reset = 0;
    #32 S = 1;
    #40 $finish;
  end

  // Fetch instruction based on PC
  assign instruction = instruction_memory[pc];

  // Display outputs for each clock cycle
  always @(posedge clk) begin
    $display("CLK | Keyword | PC | opcode | am | b | b1 | S | load | rf_e | size | rw | e | EX_opcode | EX_am | EX_s | EX_load | EX_rf_e | EX_size | EX_rw | EX_e | MEM_load | MEM_rf_e | MEM_size | MEM_rw | MEM_e | WB_rf_e");
    $display("--------------------------------------------------------------------------------------------------------------------------");
    $display("%d   | %s     | %d  | %b    | %b  | %b | %b  | %b | %b    | %b    | %b    | %b  | %b | %b       | %b     | %b    | %b       | %b       | %b       | %b     | %b    | %b        | %b         | %b        | %b      | %b    | %b",
             $time,
             instruction[27:24] == 4'b1110 ? "ANDS" :
             instruction[27:24] == 4'b1111 ? "ADD" :
             instruction[27:24] == 4'b0001 ? "LDRB" :
             instruction[27:24] == 4'b0000 ? "STR" :
             instruction[27:24] == 4'b1011 ? "BNE" : "NOP",
             pc,
             instruction[27:24], // opcode
             instruction[23:22], // am
             instruction[21],    // b
             instruction[20],    // b1
             S,
             mux_id_load,
             mux_rf_e,
             mux_id_mem_size,
             mux_id_mem_write,
             enable_ifid,
             mux_alu_op,
             mux_id_am,
             mux_store_cc,
             mux_id_load,
             mux_rf_e,
             mux_id_mem_size,
             mux_id_mem_write,
             enable_pc,
             mux_id_load,
             mux_rf_e,
             mux_id_mem_size,
             mux_id_mem_write,
             mux_rf_e
             );
  end
endmodule
