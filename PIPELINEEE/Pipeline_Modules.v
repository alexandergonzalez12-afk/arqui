module ControlUnit(
  input [31:0] instruction,
  output reg [3:0] ALU_OP,
  output reg ID_LOAD,
  output reg ID_MEM_WRITE,
  output reg [1:0] ID_AM,
  output reg STORE_CC,
  output reg ID_B,
  output reg ID_BL,
  output reg ID_MEM_SIZE,
  output reg ID_MEM_E,
  output reg RF_E
);

always @(*) begin
    // Reset control signals by default or for NOP instructions
    ALU_OP = 4'b0000;
    ID_LOAD = 0;
    ID_MEM_WRITE = 0;
    ID_AM = 2'b00;
    STORE_CC = 0;
    ID_B = 0;
    ID_BL = 0;
    ID_MEM_SIZE = 0;
    ID_MEM_E = 0;
    RF_E = 0;

    if (instruction != 32'b00000000000000000000000000000000) begin
        // Assign control signals based on instruction
        ALU_OP = instruction[24:21];
        ID_LOAD = instruction[20];
        ID_MEM_WRITE = instruction[21];
        ID_AM = instruction[26:25];
        STORE_CC = instruction[20];
        ID_B = instruction[24];
        ID_BL = instruction[27];
        ID_MEM_SIZE = instruction[22];
        ID_MEM_E = instruction[23];
        RF_E = instruction[19];
    end
end    
endmodule

module Multiplexer (
  output reg id_load, id_mem_write, store_cc, id_b, id_bl, id_mem_size, id_mem_e, rf_e,
  output reg [3:0] alu_op,
  output reg [1:0] id_am,
  input S,
  input [3:0] ALU_OP,
  input ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E,
  input [1:0] ID_AM
);

  always @ (*) begin
    if (S == 1'b0) begin
      id_load = ID_LOAD;
      id_mem_write = ID_MEM_WRITE;
      store_cc = STORE_CC;
      id_b = ID_B;
      id_bl = ID_BL;
      id_mem_size = ID_MEM_SIZE;
      id_mem_e = ID_MEM_E;
      rf_e = RF_E;
      id_am = ID_AM;
      alu_op = ALU_OP;
    end else begin
      id_load = 0;
      id_mem_write = 0;
      store_cc = 0;
      id_b = 0;
      id_bl = 0;
      id_mem_size = 0;
      id_mem_e = 0;
      rf_e = 0;
      id_am = 2'b00;
      alu_op = 4'b0000;
    end
  end
endmodule

module PC (
  input clk,                // Clock signal
  input reset,              // Reset signal to initialize PC to 0
  input E,                  // Enable signal for updating PC
  input [7:0] next_pc,      // 8-bit external incremented PC input
  output reg [7:0] pc       // 8-bit Program Counter
);

  always @(posedge clk or posedge reset) begin
    if (reset)
      pc <= 8'b0;               // Reset PC to 0
    else if (E)
      pc <= next_pc;            // Update PC from external adder result
  end
endmodule

module adder (
  input [7:0] address,       // 8-bit input address
  output [7:0] result        // 8-bit output result (address + 4)
);

  assign result = address + 8'd4;  // Increment address by 4
endmodule

module IF_ID (
    input E,
    input reset,
    input clk,
    input [31:0] instr_in,
    output reg [31:0] instr_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_out <= 32'b0;
        end else if (E) begin
            instr_out <= instr_in;
        end
    end
endmodule

module ID_EX (
    input clk,
    input reset,
    input [3:0] ID_ALU_OP,
    input ID_LOAD,
    input ID_MEM_WRITE,
    input ID_MEM_SIZE,
    input ID_MEM_ENABLE,
    input [1:0] ID_AM,
    input STORE_CC,
    input ID_BL,
    input ID_B,
    input RF_ENABLE,

    output reg [3:0] id_alu_op,
    output reg id_load,
    output reg id_mem_write,
    output reg id_mem_size,
    output reg id_mem_enable,
    output reg [1:0] id_am,
    output reg store_cc,
    output reg id_bl,
    output reg id_b,
    output reg rf_enable
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            id_alu_op <= 4'b0;
            id_load <= 0;
            id_mem_write <= 0;
            id_mem_size <= 0;
            id_mem_enable <= 0;
            id_am <= 2'b0;
            store_cc <= 0;
            id_bl <= 0;
            id_b <= 0;
            rf_enable <= 0;
        end else begin
            id_alu_op <= ID_ALU_OP;
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            id_am <= ID_AM;
            store_cc <= STORE_CC;
            id_bl <= ID_BL;
            id_b <= ID_B;
            rf_enable <= RF_ENABLE;
        end
    end
endmodule

module EX_MEM(
    input clk,
    input reset,
    input ID_LOAD,
    input ID_MEM_WRITE,
    input ID_MEM_SIZE,
    input ID_MEM_ENABLE,
    input RF_ENABLE,

    output reg id_load,
    output reg id_mem_size,
    output reg id_mem_write,
    output reg id_mem_enable,
    output reg rf_enable
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            id_load <= 0;
            id_mem_write <= 0;
            id_mem_size <= 0;
            id_mem_enable <= 0;
            rf_enable <= 0;
        end else begin
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            rf_enable <= RF_ENABLE;
        end
    end
endmodule

module MEM_WB (
    input clk,
    input reset,
    input RF_ENABLE,
    output reg rf_enable
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rf_enable <= 0;
        end else begin
            rf_enable <= RF_ENABLE;
        end
    end
endmodule
