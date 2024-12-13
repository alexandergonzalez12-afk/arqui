

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
  input LE, Clk, reset         //Load enable and Clock
);
    
  always @ (posedge Clk or posedge reset) begin
    if (reset)
      O <= 32'b0;
    if (LE) 
      O <= PW;           // Load data if LE is 1
  end
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
  input Clk, LE, reset,              // Clock and Load Enable
  output [31:0] PA, PB, PD     // Register output values
);

  wire [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15;
  wire [15:0] O;

  // Instantiate the Binary Decoder
  Binary_Decoder BD (RW, LE, O);

  // Instantiate Registers
  Register Regis0 (R0, PW, O[0], Clk, reset);
  Register Regis1 (R1, PW, O[1], Clk, reset);
  Register Regis2 (R2, PW, O[2], Clk, reset);
  Register Regis3 (R3, PW, O[3], Clk, reset);
  Register Regis4 (R4, PW, O[4], Clk, reset);
  Register Regis5 (R5, PW, O[5], Clk, reset);
  Register Regis6 (R6, PW, O[6], Clk, reset);
  Register Regis7 (R7, PW, O[7], Clk, reset);
  Register Regis8 (R8, PW, O[8], Clk, reset);
  Register Regis9 (R9, PW, O[9], Clk, reset);
  Register Regis10 (R10, PW, O[10], Clk, reset);
  Register Regis11 (R11, PW, O[11], Clk, reset);
  Register Regis12 (R12, PW, O[12], Clk, reset);
  Register Regis13 (R13, PW, O[13], Clk, reset);
  Register Regis14 (R14, PW, O[14], Clk, reset);
  Register Regis15 (R15, PC, 1'b1, Clk, reset);

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
    input [3:0] inst_I15_I12, 
    input BL_out,
    output reg [3:0] result  // Changed to reg
);
always @(*) begin
    case (BL_out)
        1'b1: result = 4'b1110;
        1'b0: result = inst_I15_I12;
    endcase
end

endmodule

module MUX_CC (
    input [3:0] Flag_out, ConditionCode,
    input SIG_s, //selector
    output reg [3:0] ConditionCodes  // Changed to reg
);
always @(*) begin
    case (SIG_s)
        1'b0: ConditionCodes = ConditionCode; // show this to professor, is this in order?
        1'b1: ConditionCodes = Flag_out;
    endcase
end

endmodule

module SUM_RF (
    input [31:0] instr_SE,
    input [31:0] nextpc,
    output reg [31:0] TA  // Changed to reg
);
always @(*) begin
    TA = instr_SE + nextpc;
end

endmodule

module MUX_RFenable (
    input       id_rf_e,
    input       s_rfenable,
    output reg  out_rf_enable  // Changed to reg
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
    output reg [31:0] instr_SE // Output 32-bit sign-extended signal
);
    always @(*) begin
    instr_SE = $signed({{6{instr_I23_I0[23]}},$signed(instr_I23_I0 << 2)});
end
endmodule


module Data_Memory_RAM (
    output reg [31:0] data_out, 
    input [7:0] address, 
    input [31:0] data_in, 
    input size, 
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
    input [31:0] Addr,        
    input [31:0] DataOut,    
    input Sel,               
    output reg [31:0] MuxOut 
);

    always @(*) begin
        if (Sel)
            MuxOut = DataOut;  // Select DataOut when Sel = 1
        else
            MuxOut = Addr;  // Zero-extend Addr to 32 bits when Sel = 0
    end

endmodule

module MUX_Fetch (
    input [31:0] SUMOUT,        
    input [31:0] TA,         
    input Sel,               
    output reg [31:0] MuxOut  
);

    always @(*) begin
        if (Sel)
            MuxOut = TA;    // Select In2 when Sel = 1
        else
            MuxOut = SUMOUT;    
    end

endmodule

module ALU (
    input [31:0] A, B,         // A & B 
    input C_IN,                 // Carry in 
    input [3:0] alu_op,            // Operation code 
    output reg [31:0] result,     // Output de 32
    output reg Z, N, C, V      // Flags
);

    always @(A, B, alu_op) begin
        case(alu_op)
            4'b0000: begin
                //A+B
                {C, result} = A + B;
                Z = (result == 0);
                N = result[31];
                V = ~(A[31] ^ B[31]) & (A[31] ^ result[31]);
            end
            
            4'b0001: begin
                //A+B+Cin
                {C, result} = A + B + C_IN;
                Z = (result == 0);
                N = result[31];
                V = ~(A[31] ^ B[31]) & (A[31] ^ result[31]);
            end
            
            4'b0010: begin
                //A-B
                result = A - B;
                Z = (result == 0);
                N = result[31];
                C = (A < B) ? 1 : 0;
                V = (A[31] ^ B[31]) & (A[31] ^ result[31]);
            end
            
            4'b0011: begin
                //A-B-Cin
                result = A - B - C_IN;
                Z = (result == 0);
                N = result[31];
                C = (A < (B+C_IN)) ? 1 : 0;
                V = (A[31] ^ B[31]) & (A[31] ^ result[31]);
            end
            
            4'b0100: begin
                //B-A
                result = B - A;
                Z = (result == 0);
                N = result[31];
                C = (B < A) ? 1 : 0;
                V = (B[31] ^ A[31]) & (B[31] ^ result[31]);
            end
            
            4'b0101: begin
                //B-A-Cin
                result = B - A - C_IN;
                Z = (result == 0);
                N = result[31];
                C = (B < (A+C_IN)) ? 1 : 0;
                V = (B[31] ^ A[31]) & (B[31] ^ result[31]);
            end
            
            4'b0110: begin
                //A and B
                result = A & B;
                Z = (result == 0);
                N = result[31];
            end
            
            4'b0111: begin
                //A or B
                result = A | B;
                Z = (result == 0);
                N = result[31];
            end
            
            4'b1000: begin
                //A xor B
                result = A ^ B;
                Z = (result == 0);
                N = result[31];
            end
            
            4'b1001: begin
                //A
                result = A;
                Z = (result == 0);
                N = result[31];
            end
            
            4'b1010: begin
                //B
                result = B;
                Z = (result == 0);
                N = result[31];
            end
            
            4'b1011: begin
                //not B
                result = (~B);
                Z = (result == 0);
                N = result[31];
            end
            
            4'b1100: begin
                //A and (not B)
                result = A & (~B);
                Z = (result == 0);
                N = result[31];
            end
            
        endcase
    end    
    
endmodule


module MUX_ALU (
    input [31:0] alu_result,
    input [31:0] Next_PC,
    input BL_OUT,

    output reg [31:0] DM_address
);
    always @(*) begin
        if (BL_OUT)
            DM_address = Next_PC;    
        else
            DM_address = alu_result;    //check if can caught an issue due to having different bit sizes 
    end
endmodule

module PSR (
 input      clk,
 input      [3:0] ConditionCode,
 output reg [3:0] PSR_ConditionCode, //Z, N, C, V
 output reg C_in
);
always @(posedge clk) begin
  C_in <= ConditionCode[1];
  PSR_ConditionCode <= ConditionCode;
end
endmodule




module ConditionHandler(    //needs fixing
  input ID_BL_instr,
  input ID_B_instr,
  input [3:0] ConditionCode,
  input [3:0] Condition,
  output reg EX_BL_instr, 
  output reg Branched
);

  wire N = ConditionCode[3];
  wire Z = ConditionCode[2];
  wire C = ConditionCode[1];
  wire V = ConditionCode[0];
always @(*) begin
        Branched = 0;
        EX_BL_instr = 0;
        if (ID_B_instr) begin
            case (Condition)
                4'b0000: Branched = (Z == 1);                 // : Branch if equal
                4'b0001: Branched = (Z == 0);                 // : Branch if not equal
                4'b0010: Branched = (C == 1);                 // : Branch if carry set
                4'b0011: Branched = (C == 0);                 // : Branch if carry clear
                4'b0100: Branched = (N == 1);                 // : Branch if minus
                4'b0101: Branched = (N == 0);                 // : Branch if pos or zero
                4'b0110: Branched = (V == 1);                 // : Branch if overflow
                4'b0111: Branched = (V == 0);                 // : Branch if no overflow
                4'b1000: Branched = ((C==1)&&(Z==0));          // Branch if unsigned higher
                4'b1001: Branched = ((C==0)||(Z==1));          // Branch if unsigned lower or same
                4'b1010: Branched = (N==V);                    // Branch if greater or equal
                4'b1011: Branched = (N!=V);                    // Branch if less than
                4'b1100: Branched = ((Z==0)&&(N==V));          // Branch if greater than
                4'b1101: Branched = ((Z==1)||(N!=V));          // Branch if less than or equal
                4'b1110: Branched = 1;
            endcase
            if (ID_BL_instr) begin
            EX_BL_instr = 1; // Always branch-and-link if ID_BL_Instr is active
        end
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
                2'b11: N = (Rm >> I[11:7]) | (Rm << (32 - I[11:7]));  // Rotate Right (ROR)
                default: N = 32'b0; // Default case for safety
            endcase
        end
        default: N = 32'b0; // Default output
    endcase
end

endmodule

module HazardUnit(
  input [3:0] ID_Rn, //RA
  input [3:0] ID_Rm, //RB
  input [3:0] ID_Rd, //RD
  input [3:0] EX_Rd,
  input [3:0] MEM_Rd,
  input [3:0] WB_Rd,
  input EX_Load,
  input EX_RF_enable,
  input MEM_RF_enable,
  input WB_RF_enable,
  input ID_Load,
  input ID_Enable,
  input [1:0]sop_count,
  output reg [1:0] forward_Rn,
  output reg [1:0] forward_Rm,
  output reg [1:0] forward_Rg,
  output reg IF_IF_Enable,
  output reg PC_Enable,
  output reg NOP_EX
);


  always @(*) begin 
    forward_Rn <= 2'b00;
    forward_Rm <= 2'b00;
    forward_Rg <= 2'b00;
    IF_IF_Enable <= 1;
    PC_Enable <= 1;
    NOP_EX <= 0;
    // $display("EX_RD %b", EX_Rd);
    // $display("MEM_RD %b", MEM_Rd);
    // $display("WB_RD %b", WB_Rd);
    // $display("ID_Rn %b", ID_Rn);
    if (sop_count > 2'b00) begin
        // Data Forwarding for Source Operand 1 (Rn / ID_Rn)
        if (sop_count >= 2'b01) begin
            if (EX_RF_enable && (ID_Rn == EX_Rd)) forward_Rn = 2'b01;
            else if (MEM_RF_enable && (ID_Rn == MEM_Rd)) forward_Rn = 2'b10;
            else if (WB_RF_enable && (ID_Rn == WB_Rd)) forward_Rn = 2'b11;
            // $display("forward_Rn %b", forward_Rn);
        end

         // Data Forwarding for Source Operand 2 (Rm / ID_Rm)
        if (sop_count >= 2'b10) begin
            if (EX_RF_enable && (ID_Rm == EX_Rd)) forward_Rm = 2'b01;
            else if (MEM_RF_enable && (ID_Rm == MEM_Rd)) forward_Rm = 2'b10;
            else if (WB_RF_enable && (ID_Rm == WB_Rd)) forward_Rm = 2'b11;
            // $display("forward_Rm %b", forward_Rm );
        end

         // Data Forwarding for Source Operand 3 (e.g., ID_RD for some instructions)
        if (sop_count == 2'b11) begin
            if (EX_RF_enable && (ID_Rd == EX_Rd)) forward_Rg = 2'b01;
            else if (MEM_RF_enable && (ID_Rd == MEM_Rd)) forward_Rg = 2'b10;
            else if (WB_RF_enable && (ID_Rd == WB_Rd)) forward_Rg = 2'b11;
            // $display("forward_Rg %b", forward_Rg);
        end
    end
        if(EX_Load && ((ID_Rn == EX_Rd) || (ID_Rm == EX_Rd))) begin //load hazard
                NOP_EX = 1;
                IF_IF_Enable = 0;
                PC_Enable = 0;
        end
        if(ID_Enable) begin
             if(!ID_Load) begin // Store hazard
                // sop_count == 3
                if (sop_count == 2'b11) begin
                    if (EX_RF_enable && ((ID_Rn == EX_Rd) || (ID_Rm == EX_Rd) || (ID_Rd == EX_Rd))) forward_Rg = 2'b01;
                    else if (MEM_RF_enable && ((ID_Rn == MEM_Rd) || (ID_Rm == MEM_Rd) || (ID_Rd == MEM_Rd))) forward_Rg = 2'b10;
                    else if (WB_RF_enable && ((ID_Rn == WB_Rd) || (ID_Rm == WB_Rd) || (ID_Rd == WB_Rd))) forward_Rg = 2'b11;
                end
                // sop_count == 2 (Rn and Rd only)
                else if (sop_count == 2'b10) begin
                    if (EX_RF_enable && ((ID_Rn == EX_Rd) || (ID_Rd == EX_Rd))) forward_Rg = 2'b01;
                    else if (MEM_RF_enable && ((ID_Rn == MEM_Rd) || (ID_Rd == MEM_Rd))) forward_Rg = 2'b10;
                    else if (WB_RF_enable && ((ID_Rn == WB_Rd) || (ID_Rd == WB_Rd))) forward_Rg = 2'b11;
                end
            end
        end
    end
endmodule



//------------------------PHASE 3 MODULES -----------------------------
module ControlUnit(
    output reg ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr,
    ID_enable_instr, ID_size, ID_BL_instr, ID_RW,
    output reg [1:0] ID_shift_AM, sop_count,
    output reg [3:0] ID_alu_op, 
    output reg [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
    input [31:0] instruction
);

always @(instruction) begin
/*
signals for the number of source operands
00 - 0 source operands
01 - 1 source operand
10 - 2 source operands
11 - 3 source operands
*/
    if (instruction == 32'b00000000000000000000000000000000) begin
        ID_S_bit = 0;
        ID_load_instr = 0;
        ID_RF_enable = 0;
        ID_B_instr = 0;
        ID_enable_instr = 0;
        ID_size = 0;
        ID_RW = 0;
        ID_BL_instr = 0;
        ID_shift_AM = 2'b00;
        ID_alu_op = 4'b0000;
        ID_mnemonic0 = "N";
        ID_mnemonic1 = "O";
        ID_mnemonic2 = "P";
    end else begin
        case (instruction[27:25])
            3'b000: begin  // data processing immediate shift
                if (instruction[4] == 0) begin
                    ID_S_bit = instruction[20];
                    ID_load_instr = 0;
                    ID_RW = 0;
                    ID_RF_enable = 1; 
                    ID_B_instr = 0;
                    ID_enable_instr = 0;
                    ID_size = 0; 
                    ID_BL_instr = 0;
                    ID_shift_AM = 2'b11;
                    
                    /* THE ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b10; // 2 source operands
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
            3'b001: begin // data processing immediate
                ID_S_bit = instruction[20];
                ID_load_instr = 0;
                ID_RW = 0;
                ID_RF_enable = 1;
                ID_B_instr = 0;
                ID_enable_instr = 0;
                ID_size = 0; 
                ID_BL_instr = 0;
                ID_shift_AM = 2'b00;
                
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                sop_count = 2'b01; // 1 source operand
                // if this doesnt work, add another condition for the mov case for 0 operands (should work because the only mov instructions occur with 1 source operand)
                
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
            3'b010: begin // load store immediate offset
                ID_S_bit = 0;
                ID_load_instr = instruction[20]; 
                ID_B_instr = 0;
                ID_enable_instr = 1;
                if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                ID_BL_instr = 0;
                ID_shift_AM = 2'b10;
                if (instruction[20] == 0) begin ID_RW = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0; 
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                
                sop_count = 2'b10; // 2 source operands
                
                end
                else begin ID_RW = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1;
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                
                sop_count = 2'b01; // 1 source operand
                
                end
                ID_alu_op = (instruction[23] == 1) ? 4'b0000 : 4'b0010;
            end
            3'b011: begin //Load/Store register offset
                    ID_S_bit = 0;
                    ID_B_instr = 0;
                    ID_enable_instr = 1;
                    ID_load_instr = instruction[20];
                    if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                    ID_BL_instr = 0;
                    if (instruction[20] == 0) begin ID_RW = 1; ID_RW = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0;
                    /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b11; // 3 source operands
                    
                    end
                else begin ID_RW = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1; end
                    
                    //scaled register offset
                    ID_shift_AM = 2'b11;
                    
                    /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b01;
                            
                    if(instruction[23] == 1) begin
                        ID_alu_op = 4'b0000;
                    end
                            
                    else
                        ID_alu_op = 4'b0010;
                    end
            3'b101: begin // branch or branch and link
                ID_S_bit = 0;
                ID_load_instr = 0;
                ID_RF_enable = 0; 
                ID_B_instr = 1;
                ID_enable_instr = 0;
                ID_size = 0;
                ID_BL_instr = instruction[24];
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                sop_count = 2'b00;
                if (instruction[24] == 0) begin ID_mnemonic0 = "B"; ID_mnemonic1 = " "; ID_mnemonic2 = " "; end
                else begin ID_mnemonic0 = "B"; ID_mnemonic1 = "L"; ID_mnemonic2 = " "; end
            end
        endcase
    end
end        
endmodule

module Multiplexer (
  output reg id_load, id_mem_write, id_s, id_b, id_bl, id_mem_size, id_mem_e, rf_e,
  output reg [3:0] alu_op,
  output reg [1:0] id_am,
  input S,
  input [3:0] ALU_OP,
  input ID_LOAD, ID_MEM_WRITE, ID_S, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E,
  input [1:0] ID_AM
);

  always @ (*) begin
    if (S) begin
        id_load = 0;
        id_mem_write = 0;
        id_s = 0;
        id_b = 0;
        id_bl = 0;
        id_mem_size = 0;
        id_mem_e = 0;
        rf_e = 0;
        id_am = 2'b00;
        alu_op = 4'b0000;
    end else begin
        id_load = ID_LOAD;
        id_mem_write = ID_MEM_WRITE;
        id_s = ID_S;
        id_b = ID_B;
        id_bl = ID_BL;
        id_mem_size = ID_MEM_SIZE;
        id_mem_e = ID_MEM_E;
        rf_e = RF_E;
        id_am = ID_AM;
        alu_op = ALU_OP;
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
  input [31:0] next_pc,      // 8-bit external incremented PC input
  output reg [31:0] pc       // 8-bit Program Counter
);

  always @(posedge clk or posedge reset) begin
    if (reset)
      pc <= 32'b0;               // Reset PC to 0
    else if (E)
      pc <= next_pc;            // Update PC from external adder result
  end
endmodule

module adder (
  input [31:0] pc,       // 8-bit input address
  output [31:0] PC        // 8-bit output result (address + 4)
);

  assign PC = pc + 32'd4;  // Increment address by 4
endmodule

module IF_ID (
    input E,
    input reset,
    input clk,
    input [31:0] instr_in,
    input [31:0] next_pc,

    output reg [31:0] instr_out,
    output reg signed [23:0] instr_i23_i0,    // imm val
    output reg [31:0] Next_PC,
    output reg [3:0] instr_i3_i0,   //RB
    output reg [3:0] instr_i19_i16, //RA
    output reg [3:0] instr_i31_i28, //ICC
    output reg [11:0] instr_i11_i0, //shift
    output reg [3:0] instr_i15_i12 // RD

);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_out <= 32'b0;
            instr_i23_i0 <=24'b0;
            Next_PC <=32'b0;
            instr_i3_i0 <= instr_in[3:0];
            instr_i19_i16 <= instr_in[19:16];
            instr_i31_i28 <= instr_in[31:28];
            instr_i11_i0 <= instr_in[11:0];
            instr_i15_i12 <= instr_in[15:12];


        end else if (E) begin
            instr_out <= instr_in;
            instr_i23_i0 <= instr_in[23:0];
            Next_PC <= next_pc;
            instr_i3_i0 <= instr_in[3:0];
            instr_i19_i16 <= instr_in[19:16];
            instr_i31_i28 <= instr_in[31:28];
            instr_i11_i0 <= instr_in[11:0];
            instr_i15_i12 <= instr_in[15:12];
        end
    end
endmodule

module ID_EX (
    input clk,
    input reset,
    input [3:0] ID_ALU_OP,
    input ID_LOAD,                  //ID load
    input ID_MEM_WRITE,             //RW
    input ID_MEM_SIZE,              //ID size
    input ID_MEM_ENABLE,            // ID enable
    input [1:0] ID_AM,              //shift AM
    input ID_S,                     // ID S
    input RF_ENABLE,                //ID RFEnable
    input BL_OUT,                   //ID_BL
    input [31:0] NEXT_PC,           //ID Next PC
    input [31:0] MUX_PA,            //PA
    input [31:0] MUX_PB,            //PB
    input [31:0] MUX_PD,            //PD
    input [3:0]  MUX_INSTR_I15_I12, //RD
    input [11:0] INSTR_I11_I0,      // shiftA

    output reg [3:0] id_alu_op,
    output reg id_load,
    output reg id_mem_write,
    output reg id_mem_size,
    output reg id_mem_enable,
    output reg [1:0] id_am,
    output reg id_s,
    output reg rf_enable,
    output reg bl_out,              
    output reg [31:0] next_pc,
    output reg [31:0] mux_pa,
    output reg [31:0] mux_pb,
    output reg [31:0] mux_pd,
    output reg [3:0]  mux_instr_i15_i12,      //RD
    output reg [11:0] instr_i11_i0  // shiftA
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            id_alu_op <= 4'b0;
            id_load <= 0;
            id_mem_write <= 0;
            id_mem_size <= 0;
            id_mem_enable <= 0;
            id_am <= 2'b0;
            id_s <= 0;
            rf_enable <= 0;
            bl_out <= 0;              //added connections
            next_pc <= 32'b0;
            mux_pa <= 32'b0;
            mux_pb <= 32'b0;
            mux_pd <= 32'b0;
            mux_instr_i15_i12 <= 4'b0;
            instr_i11_i0 <= 12'b0;    // finished added connection
        end else begin
            id_alu_op <= ID_ALU_OP;
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            id_am <= ID_AM;
            id_s <= ID_S;
            rf_enable <= RF_ENABLE;
            bl_out <= BL_OUT;              //added connections
            next_pc <= NEXT_PC;
            mux_pa <= MUX_PA;
            mux_pb <= MUX_PB;
            mux_pd <= MUX_PD;
            mux_instr_i15_i12 <= MUX_INSTR_I15_I12;
            instr_i11_i0 <= INSTR_I11_I0;    // finished added connection
        end
    end
endmodule

module EX_MEM(
    input               clk,
    input               reset,
    input               ID_LOAD,            //EX Load
    input               ID_MEM_WRITE,       //EX_RW
    input               ID_MEM_SIZE,        //EX_SIZE
    input               ID_MEM_ENABLE,      //EX_Enable
    input               RF_ENABLE,          //RF_Enable
    input [31:0]        MUX_PA,              //EX_PA
    input [31:0]        MUX_PD,             //EX_PD
    input [31:0]        DM_ADDRESS,         //ALU out
    input [3:0]         MUX_INSTR_I15_I12,  //EX_RD

    output reg          id_load,
    output reg          id_mem_size,
    output reg          id_mem_write,
    output reg          id_mem_enable,
    output reg          rf_enable,
    output reg [31:0]   mux_pa,              //MEM_PA
    output reg [31:0]   mux_pd,             //MEM_PD
    output reg [31:0]   dm_address,         //ALU out
    output reg [3:0]    mux_instr_i15_i12   //MEM_RD
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            id_load <= 0;
            id_mem_write <= 0;
            id_mem_size <= 0;
            id_mem_enable <= 0;
            rf_enable <= 0;
            mux_pa <= 32'b0;
            mux_pd <= 32'b0;
            dm_address <= 32'b0;
            mux_instr_i15_i12 <= 4'b0;
        end else begin
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            rf_enable <= RF_ENABLE;
            mux_pa <= MUX_PA;
            mux_pd <= MUX_PD;
            dm_address <= DM_ADDRESS;
            mux_instr_i15_i12 <= MUX_INSTR_I15_I12;
        end
    end
endmodule

module MEM_WB (
    input clk,
    input reset,
    input RF_ENABLE,                          //mem rf enable
    input ID_LOAD,                          
    input [31:0] MUX_DATAMEMORY,              //mem mux out
    input [3:0] MUX_INSTR_I15_I12,            //mem RD
    output reg rf_enable,
    output reg id_load, 
    output reg [3:0] mux_instr_i15_i12,       //wb RD
    output reg [31:0] mux_datamemory          //wb muxout
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rf_enable <= 0;
            id_load <= 0;
            mux_instr_i15_i12 <= 4'b0;
            mux_datamemory <= 32'b0;
        end else begin
            rf_enable <= RF_ENABLE;
            id_load <= ID_LOAD;
            mux_instr_i15_i12 <= MUX_INSTR_I15_I12;
            mux_datamemory <= MUX_DATAMEMORY;
        end
    end
endmodule
