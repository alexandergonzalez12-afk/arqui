

//------------------------PHASE 4 MODULES -----------------------------
module Binary_Decoder ( 
  input [3:0] I,        //input of 4 bits
  input LE,             //Load Enable of 1 bit
  output reg [15:0] O   //output of 16 bits
  );

  always @(*) begin
    if (LE == 1'b1) begin // When Load Enables is 1, it enables the corresponding one
          case (I)
              4'b0000: O = 16'b0000000000000001; 
              4'b0001: O = 16'b0000000000000010; 
              4'b0010: O = 16'b0000000000000100; 
              4'b0011: O = 16'b0000000000001000; 
              4'b0100: O = 16'b0000000000010000; 
              4'b0101: O = 16'b0000000000100000; 
              4'b0110: O = 16'b0000000001000000; 
              4'b0111: O = 16'b0000000010000000; 
              4'b1000: O = 16'b0000000100000000; 
              4'b1001: O = 16'b0000001000000000; 
              4'b1010: O = 16'b0000010000000000; 
              4'b1011: O = 16'b0000100000000000; 
              4'b1100: O = 16'b0001000000000000; 
              4'b1101: O = 16'b0010000000000000; 
              4'b1110: O = 16'b0100000000000000; 
              4'b1111: O = 16'b1000000000000000; 
          
      endcase
      end else 
          O = 16'b0000000000000000;  // If Load Enable is 0, disable all registers
  end    
endmodule

module Register (
  output reg [31:0] O, 
  input [31:0] PW,       // data to be written
  input LE, Clk         //Load enable and Clock
);
    
  always @ (posedge Clk)
    if (LE) 
      O = PW;           // Load data if LE is 1
  
endmodule

// Selects one of 16 32-bit inputs based on the 4-bit select signal
module Mux_RF (
  output reg [31:0] Z,
  input [3:0] S,
  input [31:0] r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15
);
  
  always @ (*)
    begin
    case(S)         // Selects the registers form RO to R15(Program Counter)
    4'b0000: Z = r0;
    4'b0001: Z = r1;
    4'b0010: Z = r2;
    4'b0011: Z = r3;
    4'b0100: Z = r4;
    4'b0101: Z = r5;
    4'b0110: Z = r6;
    4'b0111: Z = r7;
    4'b1000: Z = r8;
    4'b1001: Z = r9;
    4'b1010: Z = r10;
    4'b1011: Z = r11;
    4'b1100: Z = r12;
    4'b1101: Z = r13;
    4'b1110: Z = r14;
    4'b1111: Z = r15; 
 	
    endcase
    end
endmodule

module Three_port_register_file (
  input [3:0] RA, RB, RD, RW,   // registers of 4 bits
  input [31:0] PW,              // Data to be written
  input [31:0] PC,              // value stored in R15
  input Clk, LE,                // Clock and Load Enable
  output [31:0] PA, PB, PD     // Register output values
);

  wire [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15;
  wire [15:0] O;

  // Instantiate the Binary Decoder
  Binary_Decoder BD (RW, LE, O);

  // Instantiate Registers
  Register Regis0 (R0, PW, O[0], Clk);
  Register Regis1 (R1, PW, O[1], Clk);
  Register Regis2 (R2, PW, O[2], Clk);
  Register Regis3 (R3, PW, O[3], Clk);
  Register Regis4 (R4, PW, O[4], Clk);
  Register Regis5 (R5, PW, O[5], Clk);
  Register Regis6 (R6, PW, O[6], Clk);
  Register Regis7 (R7, PW, O[7], Clk);
  Register Regis8 (R8, PW, O[8], Clk);
  Register Regis9 (R9, PW, O[9], Clk);
  Register Regis10 (R10, PW, O[10], Clk);
  Register Regis11 (R11, PW, O[11], Clk);
  Register Regis12 (R12, PW, O[12], Clk);
  Register Regis13 (R13, PW, O[13], Clk);
  Register Regis14 (R14, PW, O[14], Clk);
  Register Regis15 (R15, PC, 1'b1, Clk);

  // Instantiate Multiplexers for outputs
  Mux_RF MUX1 (PA, RA, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);
  Mux_RF MUX2 (PB, RB, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);
  Mux_RF MUX3 (PD, RD, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);

endmodule

module MUX_PA (
    input [31:0] pa, jump_EX_pa, jump_MEM_pa, jump_WB_pa,
    input [1:0] S_PA,
    output reg [31:0] rf_pa  // Changed to reg for assignment in procedural block
);

always @(*) begin
    case (S_PA)
        2'b00: rf_pa = pa;
        2'b01: rf_pa = jump_EX_pa;
        2'b10: rf_pa = jump_MEM_pa;
        2'b11: rf_pa = jump_WB_pa;
    endcase
end

endmodule

module MUX_PB (
    input [31:0] pb, jump_EX_pb, jump_MEM_pb, jump_WB_pb,
    input [1:0] S_PB,
    output reg [31:0] rf_pb  // Changed to reg
);
always @(*) begin
    case (S_PB)
        2'b00: rf_pb = pb;
        2'b01: rf_pb = jump_EX_pb;
        2'b10: rf_pb = jump_MEM_pb;
        2'b11: rf_pb = jump_WB_pb;
    endcase
end

endmodule

module MUX_I15_I12 (
    input [3:0] inst_I15_I12, 
    input BL_out,
    output reg [3:0] result  // Changed to reg
);
always @(*) begin
    case (BL_out)
        1'b0: result = 1'b1110;
        1'b1: result = inst_I15_I12;
    endcase
end

endmodule

module SUM_RF (
    input [7:0] instr_SE,
    input [7:0] nextpc,
    output reg [7:0] TA  // Changed to reg
);
always @(*) begin
    TA = instr_SE + nextpc;
end

endmodule

module MUX_RFenable (
    input id_rf_e,
    input s_rfenable,
    output reg out_rf_enable  // Changed to reg
);
always @(*) begin
    case (s_rfenable)
        1'b0: out_rf_enable = id_rf_e;
        1'b1: out_rf_enable = 1'b1;
    endcase
end

endmodule

module X4_SE(
    input [23:0] instr_I23_I0,  // Input 24-bit signal
    output reg [7:0] instr_SE   // Output 8-bit sign-extended signal
);

    wire [7:0] selected_bits;  // Extracted bits for multiplication
    wire [9:0] multiplied_bits; // Result after multiplication by 4

    // Extract specific bits (for example, the 8 LSBs)
    assign selected_bits = instr_I23_I0[7:0];

    // Multiply by 4
    assign multiplied_bits = selected_bits * 4;

    always @(*) begin
        // Perform sign extension
        if (multiplied_bits[9] == 1) begin
            // If the result is negative (assuming signed operation)
            instr_SE = {1'b1, multiplied_bits[7:0]}; // Extend with MSB = 1
        end else begin
            // If the result is positive
            instr_SE = multiplied_bits[7:0]; // Use lower 8 bits directly
        end
    end
endmodule

module Data_Memory_RAM (
    output reg [31:0] data_out, 
    input [7:0] address, 
    input [31:0] data_in, 
    input [1:0] size, 
    input rw, 
    input enable
    );

  // Memory array "Mem" to hold 256 bytes, each element is 8 bits
  reg [7:0] Mem [0:255];

  // Always block to handle both read and write operations
  always @(*) begin
    // Check the condition for read or write operation
    if (rw == 0) begin  // Read operation if rw  =0 then check for size
      if (size == 0) begin  // If size =0 returns a  the 32-bit value
        data_out <= {24'b0, Mem[address]};  // Return a 32-bit value with only the least significant byte little endian
      end else if (size == 1) begin  // Read a word (4 bytes)
        data_out <= {Mem[address], Mem[address+1], Mem[address+2], Mem[address+3]};  // Return 4 consecutive bytes
      end
    end else if (rw == 1 && enable == 1) begin  // Write operation
      if (size == 0) begin  // Write a single byte
        Mem[address] <= data_in[7:0];  // Write only the least significant byte of data_in
      end else if (size == 1) begin  // Write a word (4 bytes)
        Mem[address] <= data_in[31:24];      // Write the most significant byte
        Mem[address+1] <= data_in[23:16];    // Write the next byte
        Mem[address+2] <= data_in[15:8];     // Write the next byte
        Mem[address+3] <= data_in[7:0];      // Write the least significant byte
      end
    end
  end

endmodule

module MUX_DataMemory(
    input [7:0] Addr,        
    input [31:0] DataOut,    
    input Sel,               
    output reg [31:0] MuxOut 
);

    always @(*) begin
        if (Sel)
            MuxOut = DataOut;  // Select DataOut when Sel = 1
        else
            MuxOut = {24'b0, Addr};  // Zero-extend Addr to 32 bits when Sel = 0
    end

endmodule

module MUX_Fetch (
    input [7:0] SUMOUT,        
    input [7:0] TA,         
    input Sel,               
    output reg [7:0] MuxOut  
);

    always @(*) begin
        if (Sel)
            MuxOut = TA;    // Select In2 when Sel = 1
        else
            MuxOut = SUMOUT;    
    end

endmodule

module ALU (
    input [31:0] A, B,
    input [3:0] alu_op,
    input C_IN,
    output reg [31:0] result,
    output reg N, Z, C, V
);
    always @(*) begin
        case (alu_op)
            4'b0000: result = A & B; // AND
            4'b0001: result = A | B; // OR
            4'b0010: {C, result} = A - B; // SUB with carry out
            4'b0011: {C, result} = A + B; // ADD with carry out
            default: result = 32'b0;
        endcase
        N = result[31];
        Z = (result == 32'b0);
        V = ((A[31] == B[31]) && (result[31] != A[31]));
    end
endmodule

module MUX_ALU (
    input [31:0] alu_result,
    input [7:0] Next_PC,
    input BL_OUT,

    output reg [7:0] DM_address
);
    always @(*) begin
        if (BL_OUT)
            DM_address = Next_PC;    
        else
            DM_address = alu_result;    //check if can caught an issue due to having different bit sizes 
    end
endmodule

module FlagRegister (
    input clk,
    input reset,
    input update,
    input STORE_CC,             //added signal store cc
    input N_in, Z_in, C_in, V_in,
    output reg N, Z, C, V
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            N <= 1'b0;
            Z <= 1'b0;
            C <= 1'b0;
            V <= 1'b0;
        end else if (update) begin
            N <= N_in;
            Z <= Z_in;
            C <= C_in;
            V <= V_in;
        end
    end
endmodule

module ConditionHandler (
    input [3:0] ConditionCode,
    input N, Z, C, V,
    input [31:28] instruction,
    input SIG_B,
    input SIG_BL,
    output reg Branch,
    output reg BranchLink,

);
    always @(*) begin
        Branch = 1'b0;
        BranchLink = 1'b0;
        if (instruction == 3'b0101) begin
            Branch = 1'b1;
            BranchLink = instruction[24];           //double check this implementation
        end
    end
endmodule

module ARM_Shifter (
    input [31:0] Rm,            // Input register Rm
    input [11:0] I,             // Immediate value
    input [1:0] AM,             // Addressing Mode
    output reg [31:0] N         // Output
);

always @(*) begin
    case (AM)
        2'b00: N = ({24'b0, I[7:0]} >> (2 * I[11:8])) | ({24'b0, I[7:0]} << (32 - 2 * I[11:8]));  // Rotate right
        2'b01: N = Rm;  // Pass Rm value
        2'b10: N = {20'b0, I};  // Zero extend I[11:0]
        2'b11: begin  // Addressing Mode 11: Shift Rm based on I[11:7]
            case (I[6:5])
                2'b00: N = Rm << I[11:7];  // Logical Shift Left (LSL)
                2'b01: N = Rm >> I[11:7];  // Logical Shift Right (LSR)
                2'b10: N = $signed(Rm) >>> I[11:7];  // Arithmetic Shift Right (ASR)
                2'b11: N = {Rm, Rm} >> I[11:7];  // Rotate Right (ROR)
                default: N = 32'b0; // Default case for safety
            endcase
        end
        default: N = 32'b0; // Default output
    endcase
end

endmodule

module HazardUnit (         // double check this    must have 10 inputs and 4 outputs look at diagram!

    input [4:0] ID_Rn,             // Source register in ID stage
    input [4:0] ID_Rm,             // Source register in ID stage
    input [4:0] EX_Rd,             // Destination register in EX stage
    input [4:0] MEM_Rd,            // Destination register in MEM stage
    input EX_MemRead,              // EX stage memory read signal
    input MEM_MemRead,             // MEM stage memory read signal
    input Branch,                  // Branch signal from ConditionHandler
    output reg stall,              // Signal to stall the pipeline
    output reg flush               // Signal to flush the pipeline
);

always @(*) begin
    // Default values
    stall = 0;
    flush = 0;

    // checking for data hazards
    if (EX_MemRead && (EX_Rd != 0) && ((EX_Rd == ID_Rn) || (EX_Rd == ID_Rm))) begin
        stall = 1; // Stall pipeline if there's a data hazard
    end else if (MEM_MemRead && (MEM_Rd != 0) && ((MEM_Rd == ID_Rn) || (MEM_Rd == ID_Rm))) begin
        stall = 1; // if theres a hazard then a delay is created 
    end

    // control hazard
    if (Branch) begin
        flush = 1; // Flush pipeline on branch
    end
end

endmodule

//------------------------PHASE 3 MODULES -----------------------------
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
module Instruction_Memory_ROM (output reg [31:0] I, input [7:0] A);
  
  // Variable I - Instruction(output) of 32-bit
  // Variable A = Address(intput) of 8-bit
  
  // Variable Mem = memory holds 256 bytes as an array 
  reg [7:0] Mem [0:255];

    // Preload the memory with the content inside the file
   

    // Always reads the instruction from memory
  always @(A) begin
      I <= {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};
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
    input signal_Hazard,                //added input signals
    input [7:0] next_pc,

    output reg [31:0] instr_out,
    output reg [23:0] instr_i23_i0,    // added output signals
    output reg [7:0] Next_PC,
    output reg [3:0] instr_i3_i0,
    output reg [3:0] instr_i19_i16,
    output reg [3:0] instr_i31_i28,
    output reg [11:0] instr_i11_i0,
    output reg [3:0] instr_i15_i12

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
    input BL_OUT,                   //added conections
    input [7:0] NEXT_PC,
    input [31:0] MUX_PA,
    input [31:0] MUX_PB,
    input [31:0] PD,
    input [3:0] MUX_INSTR_I15_I12,
    input [11:0] INSTR_I11_I0       // finished added conection

    output reg [3:0] id_alu_op,
    output reg id_load,
    output reg id_mem_write,
    output reg id_mem_size,
    output reg id_mem_enable,
    output reg [1:0] id_am,
    output reg store_cc,
    output reg id_bl,
    output reg id_b,
    output reg rf_enable,
    output reg bl_out,              //added conections
    output reg [7:0] next_pc,
    output reg [31:0] mux_pa,
    output reg [31:0] mux_pb,
    output reg [31:0] mux_pc,
    output reg [31:0] pd,
    output reg [3:0] mux_instr_i15_i12,
    output reg [11:0] instr_i11_i0  // finished added conection
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
    input [31:0] PD,
    input [7:0] DM_ADDRESS,
    input [3:0] MUX_INSTR_I15_I12,

    output reg id_load,
    output reg id_mem_size,
    output reg id_mem_write,
    output reg id_mem_enable,
    output reg rf_enable,
    output reg [31:0] pd,
    output reg [7:0] dm_address,
    output reg [3:0] mux_instr_i15_i12
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
    input [31:0] MUX_DATAMEMORY,
    input [3:0] MUX_INSTR_I15_I12,
    output reg rf_enable,
    output reg [3:0] mux_instr_i15_i12,
    output reg [31:0] mux_datamemory
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rf_enable <= 0;
        end else begin
            rf_enable <= RF_ENABLE;
        end
    end
endmodule