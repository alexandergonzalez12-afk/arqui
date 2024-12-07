

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
        1'b0: result = 4'b1110;
        1'b1: result = inst_I15_I12;
    endcase
end

endmodule

module MUX_CC (
    input N,Z,C,V,
    input [3:0] Flag_out,
    input SIG_store_cc, //selector
    output reg [3:0] ConditionCodes  // Changed to reg
);
always @(*) begin
    case (SIG_store_cc)
        1'b0: ConditionCodes = {N,Z,C,V}; // show this to professor, is this in order?
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

    reg [31:0] sign_extended;  // Intermediate sign-extended signal
    reg [31:0] multiplied_value; // Result after multiplication by 4

    always @(*) begin
        // Perform sign extension from 24 bits to 32 bits
        sign_extended = {{8{instr_I23_I0[23]}}, instr_I23_I0};

        // Multiply the sign-extended value by 4
        multiplied_value = sign_extended << 2;

        // Assign the result to the output
        instr_SE = multiplied_value;
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

// Logic to handle ALU operations and set flags
always @(*) begin
    // Inicializamos
    result = 32'b0;
    Z = 0;
    N = 0;
    C = 0;
    V = 0;

    case (alu_op)
        4'b0000: result = A & B;             // A & B
        4'b0101: {C, result} = A + B + C_IN;       // A + B + CIN (with carry)
        4'b0010: {C, result} = A - B;             // A - B
        4'b0110: {C, result} = A - B - C_IN;       // A - B - CIN (with carry)
        4'b0011: {C, result} = B - A;             // B - A
        4'b0101: {C, result} = B - A - C_IN;       // B - A - CIN
        4'b0100: {C, result} = A + B;             // A + B
        4'b1100: result = A | B;                  // Bitwise OR
        4'b0001: result = A ^ B;                  // Bitwise XOR
        4'b1001: result = A;                      // Pass A
        4'b1010: result = B;                      // Pass B
        4'b1011: result = ~B;                     // Bitwise NOT B
        4'b1110: result = A & ~B;                 // Bit Clear (A AND NOT B)
        default: result = 32'b0;                  // Default case
    endcase

    // Setear los flags
    Z = (result == 32'b0);      // Z flag (Zero flag)
    N = result[31];             // N flag (Negative flag)

    // Handler para Carry & Overflow 
    if (alu_op == 4'b0100 || alu_op == 4'b0001) begin  // add
        V = (~A[31] & ~B[31] & result[31]) | (A[31] & B[31] & ~result[31]); // overflow detection for A + B
    end else if (alu_op == 4'b0010 || alu_op == 4'b0011) begin  // sub
        V = (A[31] & ~B[31] & ~result[31]) | (~A[31] & B[31] & result[31]); // overflow detection for A - B
    end
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
 input STORE_CC,
 input Z_in,C_in,N_in,V_in,
 output reg Z_out,N_out,C_out,V_out //Z, N, C, V
);
always @(*) begin
  if(STORE_CC) begin
    Z_out <= Z_in;
    N_out <= N_in;
    C_out <= C_in;
    V_out <= V_in;
  end
end
endmodule




module ConditionHandler(
  input ID_BL_instr,
  input ID_B_instr,
  input Z,
  input N,
  input C,
  input V,
  input [3:0] Condition,
  output reg EX_BL_instr, 
  output reg Branched
);
always @(*) begin 
  EX_BL_instr = 0;
  Branched = 0;

  if(ID_B_instr) begin
  case (Condition) 
      4'b0000: begin
        if(Z) begin
          Branched = 1;
        end
      end
      4'b0001: begin
        if(!Z) begin
          Branched = 1;
        end
      end
      4'b0010: begin
        if(C) begin
          Branched = 1;
        end
      end
      4'b0011: begin
        if(!C) begin
          Branched = 1;
        end
      end
      4'b0100: begin
        if(N) begin
          Branched = 1;
        end
      end
      4'b0101: begin
        if(!N) begin
          Branched = 1;
        end
      end
      4'b0110: begin
      if(V) begin
          Branched = 1;
      end
      end
      4'b0111: begin
      if(!V) begin
          Branched = 1;
      end
      end
      4'b1000: begin
      if(C & !Z) begin
          Branched = 1;
      end
      end
      4'b1001: begin
      if(!C || Z) begin
          Branched = 1;
      end
      end
      4'b1010: begin
        if(N == V) begin
          Branched = 1;
      end
      end
      4'b1011: begin
        if(!N == V) begin
          Branched = 1;
      end
      end
      4'b1100: begin
        if(Z == 0 & N == V) begin
          Branched = 1;
      end
      end
      4'b1101: begin
        if(Z == 1 || N == !V) begin
          Branched = 1;
      	end   
      end
    
    endcase
  end
         
   else if(ID_BL_instr) begin
  	case (Condition) 
      4'b0000: begin
        if(Z) begin
          Branched = 1;
          EX_BL_instr = 1;
        end
      end
      4'b0001: begin
        if(!Z) begin
          Branched = 1;
          EX_BL_instr = 1;          
        end
      end
      4'b0010: begin
        if(C) begin
          Branched = 1;
          EX_BL_instr = 1;
        end
      end
      4'b0011: begin
        if(!C) begin
          Branched = 1;
          EX_BL_instr = 1;

        end
      end
      4'b0100: begin
        if(N) begin
          Branched = 1;
          EX_BL_instr = 1;

        end
      end
      4'b0101: begin
        if(!N) begin
          Branched = 1;
          EX_BL_instr = 1;

        end
      end
      4'b0110: begin
      if(V) begin
          Branched = 1;
          EX_BL_instr = 1;

      end
      end
      4'b0111: begin
      if(!V) begin
          Branched = 1;
          EX_BL_instr = 1;

      end
      end
      4'b1000: begin
      if(C & !Z) begin
          Branched = 1;
          EX_BL_instr = 1;
	  	end
      end
      4'b1001: begin
      if(!C || Z) begin
          Branched = 1;
          EX_BL_instr = 1;
      	end
      end
      4'b1010: begin
        if(N == V) begin
          Branched = 1;
          EX_BL_instr = 1;
      	end
      end
      4'b1011: begin
        if(!N == V) begin
          Branched = 1;
          EX_BL_instr = 1;
    	end
      end
      4'b1100: begin
        if(Z == 0 & N == V) begin
          Branched = 1;
          EX_BL_instr = 1;
      	end
      end
      4'b1101: begin
        if(Z == 1 || N == !V) begin
          Branched = 1;
          EX_BL_instr = 1;
      	end
      end 
    endcase
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

module HazardUnit(
  input wire [3:0] ID_Rn,
  input wire [3:0] ID_Rm,
  input wire [3:0] ID_Rd,
  input wire [3:0] EX_Rd,
  input wire [3:0] MEM_Rd,
  input wire [3:0] WB_Rd,
  input wire EX_Load,
  input wire EX_RF_enable,
  input wire MEM_RF_enable,
  input wire WB_RF_enable,
  input wire ID_Store,
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
    // && ((ID_Rn == EX_Rd) || (ID_Rm == 			EX_Rd))
  	if(EX_Load && ((ID_Rn == EX_Rd) || (ID_Rm == EX_Rd) || (ID_Rd == EX_Rd))) begin
        IF_IF_Enable <= 0;
        PC_Enable <= 0;
        NOP_EX <= 1;
    end
    else 
    if (EX_RF_enable & (ID_Rn == EX_Rd)) begin
            forward_Rn <= 2'b11;
    end
    else if (MEM_RF_enable & (ID_Rn == MEM_Rd))begin
            forward_Rn <= 2'b01;
    end
    else if (WB_RF_enable & (ID_Rn == WB_Rd))begin
            forward_Rn <= 2'b10;
    end
    else if (EX_RF_enable & (ID_Rm == EX_Rd))begin
            forward_Rm <= 2'b11;
    end
    else if (MEM_RF_enable & (ID_Rm == MEM_Rd))begin
            forward_Rm <= 2'b01;
    end
    else if (WB_RF_enable & (ID_Rm == WB_Rd))begin
            forward_Rm <= 2'b10;
    end
    else if (EX_RF_enable & (ID_Rd == EX_Rd))begin         
            forward_Rg <= 2'b11;
    end
    else if (MEM_RF_enable & (ID_Rd == MEM_Rd))begin
            forward_Rg <= 2'b01;
    end
    else if (WB_RF_enable & (ID_Rd == WB_Rd))begin    
            forward_Rg <= 2'b10;
    end

  end
endmodule



//------------------------PHASE 3 MODULES -----------------------------
module ControlUnit(
  input [31:0] instruction,
  output reg [1:0] ID_SHIFT_am,
  output reg [3:0] ID_ALU_op,
  output reg ID_load_instr,
  output reg ID_RF_enable,
  output reg ID_DM_size,
  output reg ID_DM_rfw,
  output reg ID_DM_enable,
  output reg ID_DP_instr,
  output reg ID_B_instr,
  output reg ID_BL_instr
);
  always @(*) begin
    if (instruction == 32'b0) begin
          ID_SHIFT_am = 2'b00;
          ID_ALU_op = 4'b0000;
          ID_load_instr = 0;
          ID_RF_enable = 0;
          ID_DM_size = 0;
          ID_DM_rfw = 0;
          ID_DM_enable = 0;
          ID_DP_instr = 0;
          ID_B_instr = 0;
          ID_BL_instr = 0;
      end else if (instruction != 32'b0) begin
          // Decodificación de la instrucción cuando reset no está activo y la instrucción no es cero
          case (instruction[27:25])
              3'b000: begin // Data Processing Immediate or Register Shift
                ID_BL_instr = 0;
                ID_B_instr = 0;
                ID_SHIFT_am = 2'b11;
                case(instruction[4])
                    1'b0: begin
                      case (instruction[24:21]) 
                              4'b0000: begin
                                ID_ALU_op = 4'b0110;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0001: begin
                                ID_ALU_op = 4'b1000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0010: begin
                                ID_ALU_op = 4'b0010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0011: begin
                                ID_ALU_op = 4'b0100;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0100: begin
                                ID_ALU_op = 4'b0000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0101: begin
                                ID_ALU_op = 4'b0001;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0110: begin
                                ID_ALU_op = 4'b0011;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0111: begin
                                ID_ALU_op = 4'b0101;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1000: begin
                                ID_ALU_op = 4'b0110;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1001: begin
                                ID_ALU_op = 4'b1000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1010: begin
                                ID_ALU_op = 4'b0010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1011: begin
                                ID_ALU_op = 4'b0000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1100: begin
                                ID_ALU_op = 4'b0111;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1101: begin
                                ID_ALU_op = 4'b1010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1110: begin
                                ID_ALU_op = 4'b1100;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1111: begin
                                ID_ALU_op = 4'b1011;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                          endcase
                      end
                   1'b1: begin
                   end
                 endcase
              end
                     
                 
              
              3'b001: begin // Data Processing Immediate
                ID_BL_instr = 0;
                ID_B_instr = 0;
                ID_SHIFT_am = 2'b00;
                  case (instruction[24:21])
                    	4'b0000: begin
                                ID_ALU_op = 4'b0110;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0001: begin
                                ID_ALU_op = 4'b1000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0010: begin
                                ID_ALU_op = 4'b0010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0011: begin
                                ID_ALU_op = 4'b0100;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0100: begin
                                ID_ALU_op = 4'b0000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0101: begin
                                ID_ALU_op = 4'b0001;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0110: begin
                                ID_ALU_op = 4'b0011;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b0111: begin
                                ID_ALU_op = 4'b0101;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1000: begin
                                ID_ALU_op = 4'b0110;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1001: begin
                                ID_ALU_op = 4'b1000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1010: begin
                                ID_ALU_op = 4'b0010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1011: begin
                                ID_ALU_op = 4'b0000;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 0;
                              end
                              4'b1100: begin
                                ID_ALU_op = 4'b0111;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1101: begin//mov
                                ID_ALU_op = 4'b1010;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1110: begin
                                ID_ALU_op = 4'b1100;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                              4'b1111: begin
                                ID_ALU_op = 4'b1011;
                                ID_DP_instr = instruction[20];
                                ID_RF_enable = 1;
                              end
                  endcase
              end
              3'b010: begin // Load/Store Immediate Offset
                  ID_load_instr = 1;
                  ID_DM_enable = 1;
                  ID_DP_instr = 0;
                  ID_DM_rfw = (!instruction[20] == 1);
                  ID_DM_size = (!instruction[22] == 1);
                  ID_SHIFT_am = 2'b10;
              end
              3'b011: begin // Load/Store Conditional
                  if (instruction[4] == 0) begin
                      ID_load_instr = 1;
                      ID_DM_enable = 1;
                    	ID_DM_rfw = (!instruction[20] == 1);
                    	ID_DM_size = (!instruction[22] == 1);
                    	ID_DP_instr = 0;
                    	ID_SHIFT_am = 2'b11;
                  end
              end
              3'b101: begin // Branch
                  if (instruction[24] == 1'b1)
                    begin
                      ID_DP_instr = 0;
                      ID_BL_instr = 1;                  
                	    ID_B_instr = 0;
                      ID_ALU_op = 4'b1111;
                    end
                  else begin
                  	  ID_DP_instr = 0;
                      ID_B_instr = 1;
                	    ID_BL_instr = 0;
                    	ID_ALU_op = 4'b1111;
                  end
              end
          endcase
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
    output reg [23:0] instr_i23_i0,    // added output signals
    output reg [31:0] Next_PC,
    output reg [3:0] instr_i3_i0,
    output reg [3:0] instr_i19_i16,
    output reg [3:0] instr_i31_i28,
    output reg [11:0] instr_i11_i0,
    output reg [3:0] instr_i15_i12

);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_out <= 32'b0;
            instr_i23_i0 <=24'b0;
            Next_PC <=32'b0;
            instr_i3_i0 <= 4'b0;
            instr_i19_i16 <= 4'b0;
            instr_i31_i28 <= 4'b0;
            instr_i11_i0 <= 12'b0;
            instr_i15_i12 <=4'b0;


        end else if (E) begin
            instr_out <= instr_in;
            Next_PC <= next_pc;
            instr_i3_i0 <= instr_in[3:0];
            instr_i19_i16 <= instr_in[19:16];
            instr_i31_i28 <= instr_in[31:28];
            instr_i11_i0 <= instr_in[11:0];
            instr_i15_i12 <=instr_in[15:12];
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
    input RF_ENABLE,
    input BL_OUT,                   //added connections
    input [31:0] NEXT_PC,
    input [31:0] MUX_PA,
    input [31:0] MUX_PB,
    input [31:0] MUX_PD,
    input [3:0]  MUX_INSTR_I15_I12,
    input [11:0] INSTR_I11_I0,      // finished added connection

    output reg [3:0] id_alu_op,
    output reg id_load,
    output reg id_mem_write,
    output reg id_mem_size,
    output reg id_mem_enable,
    output reg [1:0] id_am,
    output reg store_cc,
    output reg rf_enable,
    output reg bl_out,              //added connections
    output reg [31:0] next_pc,
    output reg [31:0] mux_pa,
    output reg [31:0] mux_pb,
    output reg [31:0] mux_pd,
    output reg [3:0]  mux_instr_i15_i12,
    output reg [11:0] instr_i11_i0  // finished added connection
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
            store_cc <= STORE_CC;
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
    input               ID_LOAD,
    input               ID_MEM_WRITE,
    input               ID_MEM_SIZE,
    input               ID_MEM_ENABLE,
    input               RF_ENABLE,
    input [31:0]        MUX_PD,
    input [31:0]         DM_ADDRESS,
    input [3:0]         MUX_INSTR_I15_I12,

    output reg          id_load,
    output reg          id_mem_size,
    output reg          id_mem_write,
    output reg          id_mem_enable,
    output reg          rf_enable,
    output reg [31:0]   mux_pd,
    output reg [31:0]    dm_address,
    output reg [3:0]    mux_instr_i15_i12
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            id_load <= 0;
            id_mem_write <= 0;
            id_mem_size <= 0;
            id_mem_enable <= 0;
            rf_enable <= 0;
            mux_pd <= 32'b0;
            dm_address <= 32'b0;
            mux_instr_i15_i12 <= 4'b0;
        end else begin
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            rf_enable <= RF_ENABLE;
            mux_pd <= MUX_PD;
            dm_address <= DM_ADDRESS;
            mux_instr_i15_i12 <= MUX_INSTR_I15_I12;
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
            mux_instr_i15_i12 <= 4'b0;
            mux_datamemory <= 32'b0;
        end else begin
            rf_enable <= RF_ENABLE;
            mux_instr_i15_i12 <= MUX_INSTR_I15_I12;
            mux_datamemory <= MUX_DATAMEMORY;
        end
    end
endmodule
