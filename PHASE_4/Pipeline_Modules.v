//------------------------------------------ID---------------------------------------------
//------------------------------------------Register_File----------------------------------
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

module MUX_PD (
    input [31:0] pd, jump_EX_pd, jump_MEM_pd, jump_WB_pd,
    input [1:0] S_PD,
    output reg [31:0] rf_pd  // Changed to reg
);
always @(*) begin
    case (S_PD)
        2'b00: rf_pd = pd;
        2'b01: rf_pd = jump_EX_pd;
        2'b10: rf_pd = jump_MEM_pd;
        2'b11: rf_pd = jump_WB_pd;
    endcase
end

endmodule

module MUX_I15_I12 (
    input [3:0] inst_I15_I12, val14,
    input BL_out,
    output reg [3:0] result  // Changed to reg
);
always @(*) begin
    case (BL_out)
        1'b0: result = val14;
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
    input val1,
    input id_rf_e,
    input s_rfenable,
    output reg out_rf_enable  // Changed to reg
);
always @(*) begin
    case (s_rfenable)
        1'b0: out_rf_enable = id_rf_e;
        1'b1: out_rf_enable = val1;
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

//------------------------------------------Register_File----------------------------------
//------------------------------------------ID---------------------------------------------
module ControlUnit (
    output reg ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr,       // hacer el MUX para el RF_Enable
    ID_load_store_instr, ID_size, ID_BL_instr,
    output reg [1:0] ID_shift_AM,
    output reg [3:0] ID_alu_op,
    output reg [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
    input [31:0] instruction
);
always @(instruction) begin
    if (instruction == 32'b00000000000000000000000000000000) begin
        ID_S_bit = 0;
        ID_load_instr = 0;
        ID_RF_enable = 0;
        ID_B_instr = 0;
        ID_load_store_instr = 0;
        ID_size = 0;
        ID_BL_instr = 0;
        ID_shift_AM = 2'b00;
        ID_alu_op = 4'b0000;
        ID_mnemonic0 = "N";
        ID_mnemonic1 = "O";
        ID_mnemonic2 = "P";
    end else begin
        case (instruction[27:25])
            3'b000: begin  
                if (instruction[4] == 0) begin
                    ID_S_bit = instruction[20];
                    ID_load_instr = 0;
                    ID_RF_enable = 1; 
                    ID_B_instr = 0;
                    ID_load_store_instr = 0;
                    ID_size = 0; 
                    ID_BL_instr = 0;
                    ID_shift_AM = 2'b11;
                    case (instruction[24:21])
                        4'b0000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "A"; ID_mnemonic1 = "N"; ID_mnemonic2 = "D"; end
                        4'b0001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "E"; ID_mnemonic1 = "O"; ID_mnemonic2 = "R"; end
                        4'b0010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "S"; ID_mnemonic1 = "U"; ID_mnemonic2 = "B"; end
                        4'b0011: begin ID_alu_op = 4'b0100; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "B"; end
                        4'b0100: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "D"; end
                        4'b0101: begin ID_alu_op = 4'b0001; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "C"; end
                        4'b0110: begin ID_alu_op = 4'b0011; ID_mnemonic0 = "S"; ID_mnemonic1 = "B"; ID_mnemonic2 = "C"; end
                        4'b0111: begin ID_alu_op = 4'b0101; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "C"; end
                        4'b1000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "T"; ID_mnemonic1 = "S"; ID_mnemonic2 = "T"; end
                        4'b1001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "T"; ID_mnemonic1 = "E"; ID_mnemonic2 = "Q"; end
                        4'b1010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "P"; end
                        4'b1011: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "N"; end
                        4'b1100: begin ID_alu_op = 4'b0111; ID_mnemonic0 = "O"; ID_mnemonic1 = "R"; ID_mnemonic2 = "R"; end
                        4'b1101: begin ID_alu_op = 4'b1010; ID_mnemonic0 = "M"; ID_mnemonic1 = "O"; ID_mnemonic2 = "V"; end
                        4'b1110: begin ID_alu_op = 4'b1100; ID_mnemonic0 = "B"; ID_mnemonic1 = "I"; ID_mnemonic2 = "C"; end
                        4'b1111: begin ID_alu_op = 4'b1011; ID_mnemonic0 = "M"; ID_mnemonic1 = "V"; ID_mnemonic2 = "N"; end
                    endcase
                end
            end
            3'b001: begin 
                ID_S_bit = instruction[20];
                ID_load_instr = 0;
                ID_RF_enable = 1;
                ID_B_instr = 0;
                ID_load_store_instr = 0;
                ID_size = 0; 
                ID_BL_instr = 0;
                ID_shift_AM = 2'b00;
                case (instruction[24:21])
                    4'b0000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "A"; ID_mnemonic1 = "N"; ID_mnemonic2 = "D"; end
                    4'b0010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "S"; ID_mnemonic1 = "U"; ID_mnemonic2 = "B"; end
                    4'b1111: begin ID_alu_op = 4'b1011; ID_mnemonic0 = "M"; ID_mnemonic1 = "V"; ID_mnemonic2 = "N"; end
                endcase
            end
            3'b011: begin
                ID_load_store_instr = 1;
                ID_size = instruction[22]; // 1 if word; 0 if byte
                if (instruction[20] == 1) begin
                    ID_RF_enable = 1;
                    ID_load_instr = 1;
                end else begin
                    ID_RF_enable = 0;
                    ID_load_instr = 0;
                end
                ID_mnemonic0 = instruction[20] ? "L" : "S";
                ID_mnemonic1 = instruction[20] ? "D" : "T";
                ID_mnemonic2 = "R";
            end
            default: begin
                ID_S_bit = 0;
                ID_load_instr = 0;
                ID_RF_enable = 0;
                ID_B_instr = 0;
                ID_load_store_instr = 0;
                ID_size = 0;
                ID_BL_instr = 0;
                ID_shift_AM = 2'b00;
                ID_alu_op = 4'b0000;
                ID_mnemonic0 = "U";
                ID_mnemonic1 = "N";
                ID_mnemonic2 = "K"; // Unknown Instruction
            end
        endcase
    end
end        
endmodule


module Adder(output reg [31:0] NextPC, input [31:0] PC);
always @(*) begin
    NextPC = PC + 4; 
end
endmodule

module Instruction_Memory_ROM (
    input [7:0] Address,
    output reg [31:0] Instruction  // Changed to reg
);
    reg [7:0] mem [0:255];
    always @(*) begin
        Instruction = {mem[Address], mem[Address + 1], mem[Address + 2], mem[Address + 3]};
    end
endmodule

module Data_Memory_RAM (output reg [31:0] data_out,
 input [7:0] address,
  input [31:0] data_in,
   input [1:0] size,
    input rw,
     input enable);

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

module PC (
    output reg [31:0] Qs, input [31:0] Ds, input enable, clk, reset
);
    always @(posedge clk) begin
        if(reset) begin
            Qs = 32'h00000000;
        end else if(enable) begin
            Qs = Ds;
        end
    end
endmodule

module Multiplexer (
    output reg [1:0] AM,
    output reg [3:0] opcode,
    output reg S, load, RFenable, B, BL, size, ReadWrite,
    input [1:0] ID_shift_AM,
    input [3:0] ID_alu_op,
    input ID_S_Bit, ID_load_instr, ID_RF_enable, ID_B_intr, ID_load_store_instr, ID_size, ID_BL_instr, select
);
    always @(*) begin
        if (select) begin
            AM = 2'b00;
            opcode = 4'b0000;
            S = 1'b0;
            load = 1'b0;
            RFenable = 1'b0;
            B = 1'b0;
            BL = 1'b0;
            size = 1'b0;
            ReadWrite = 1'b0;
        end else begin
            AM = ID_shift_AM;
            opcode = ID_alu_op;
            S = ID_S_Bit;
            load = ID_load_instr;
            ReadWrite = ID_load_store_instr;
            RFenable = ID_RF_enable;
            B = ID_B_intr;
            BL = ID_BL_instr;
            size = ID_size;
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
    input [7:0] In1,        
    input [7:0] In2,         
    input Sel,               
    output reg [7:0] MuxOut  
);

    always @(*) begin
        if (Sel)
            MuxOut = In2;    // Select In2 when Sel = 1
        else
            MuxOut = In1;    
    end

endmodule


module IF_ID (
    input Clk, 
    input Reset,
    input IF_ID_enable,
    input [31:0] IF_instruction, 
    output reg [31:0] ID_instruction 
);
    always @(posedge Clk) begin
        if (Reset) begin
            ID_instruction <= 32'b0; 
        end else if (IF_ID_enable) begin
            ID_instruction <= IF_instruction; 
        end
    end
endmodule

module ID_EX (
    input Clk,
    input Reset,
    input ID_S_instr,
    input [3:0] ID_alu_op,
    input ID_load_instr,
    input ID_RF_enable,
    input ID_load_store_instr,
    input ID_size,
    input ID_BL_instr,
    input ID_B_instr,
    input [1:0] ID_shift_AM,
    output reg EX_S_instr,
    output reg [3:0] EX_alu_op,
    output reg EX_load_instr,
    output reg EX_RF_enable,
    output reg EX_load_store_instr,
    output reg EX_size,
    output reg EX_BL_instr,
    output reg EX_B_instr,
    output reg [1:0] EX_shift_AM
);

always @(posedge Clk or posedge Reset) begin
    if (Reset) begin
        EX_S_instr <= 1'b0;
        EX_alu_op <= 4'b0000;
        EX_load_instr <= 1'b0;
        EX_RF_enable <= 1'b0;
        EX_load_store_instr <= 1'b0;
        EX_size <= 1'b0;
        EX_BL_instr <= 1'b0;
        EX_shift_AM <= 2'b00;
        EX_B_instr <= 1'b0;
    end else begin
        EX_S_instr <= ID_S_instr;
        EX_alu_op <= ID_alu_op;
        EX_load_instr <= ID_load_instr;
        EX_RF_enable <= ID_RF_enable;
        EX_load_store_instr <= ID_load_store_instr;
        EX_size <= ID_size;
        EX_BL_instr <= ID_BL_instr;
        EX_shift_AM <= ID_shift_AM;
        EX_B_instr <= ID_B_instr;
    end
end

endmodule


module EX_MEM (
    input Clk,
    input Reset,
    input EX_load_store_instr,
    input EX_size,
    input EX_RF_enable,
    input EX_load_instr,
    output reg MEM_load_store_instr,
    output reg MEM_size,
    output reg MEM_RF_enable,
    output reg MEM_load_instr
);
    always @(posedge Clk) begin
        if (Reset) begin
            MEM_load_store_instr <= 1'b0;
            MEM_size <= 1'b0;
            MEM_RF_enable <= 1'b0;
        end else begin
            MEM_load_store_instr <= EX_load_store_instr;
            MEM_size <= EX_size;
            MEM_RF_enable <= EX_RF_enable;
        end
    end
endmodule

module MEM_WB (
    input Clk,
    input Reset,
    input MEM_RF_enable,
    output reg WB_RF_enable
);
    always @(posedge Clk) begin
        if (Reset) begin
            WB_RF_enable <= 1'b0;
        end else begin
            WB_RF_enable <= MEM_RF_enable;
        end
    end
endmodule

module ALU (
    input [31:0] A, B,
    input [3:0] alu_op,
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

module FlagRegister (
    input clk,
    input reset,
    input update,
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
    input [31:0] instruction,
    output reg Branch,
    output reg BranchLink
);
    always @(*) begin
        Branch = 1'b0;
        BranchLink = 1'b0;
        if (instruction[27:25] == 3'b101) begin
            Branch = 1'b1;
            BranchLink = instruction[24];
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



module EX_Stage (
    input clk,
    input reset,
    input [31:0] A, B,          // Inputs to the ALU
    input [3:0] alu_op,         // ALU operation
    input update_flags,         // Signal to update flags
    input [31:0] instruction,   // Current instruction
    input [31:0] Rm,            // Register Rm input for Shifter
    input [11:0] I,             // Immediate input for Shifter
    input [1:0] AM,             // Addressing Mode fxxor Shifter
    output [31:0] result,       // ALU result
    output [31:0] shifted_value, // Shifter output
    output N, Z, C, V,          // Flags
    output Branch, BranchLink   // ConditionHandler outputs
);

    wire N_in, Z_in, C_in, V_in;

    // Shifter instance
    ARM_Shifter shifter (
        .Rm(Rm),
        .I(I),
        .AM(AM),
        .N(shifted_value)
    );

    // ALU instance
    ALU alu (
        .A(A),
        .B(shifted_value),  // Shifter output as input to ALU
        .alu_op(alu_op),
        .result(result),
        .N(N_in),
        .Z(Z_in),
        .C(C_in),
        .V(V_in)
    );

    // FlagRegister instance
    FlagRegister flag_register (
        .clk(clk),
        .reset(reset),
        .update(update_flags),
        .N_in(N_in),
        .Z_in(Z_in),
        .C_in(C_in),
        .V_in(V_in),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V)
    );

    // ConditionHandler instance
    ConditionHandler condition_handler (
        .ConditionCode(instruction[31:28]),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V),
        .instruction(instruction),
        .Branch(Branch),
        .BranchLink(BranchLink)
    );
endmodule

module HazardUnit (
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
