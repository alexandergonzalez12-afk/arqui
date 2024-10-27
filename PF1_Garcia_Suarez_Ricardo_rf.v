`timescale 1ns / 1ns

// Takes a 4-bit input and outputs a one-hot 16-bit signal based on the input address
// The output is enabled only when LE (Load Enable) is 1
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
module Multiplexer (
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
  Multiplexer MUX1 (PA, RA, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);
  Multiplexer MUX2 (PB, RB, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);
  Multiplexer MUX3 (PD, RD, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);

endmodule

module Three_Port_Register_File_tb;
  reg Clk, LE;
  reg [3:0] RA, RB, RD, RW;
  reg [31:0] PW, PC;            
  wire [31:0] PA, PB, PD;

  Three_port_register_file dut (
    .RA(RA), .RB(RB), .RD(RD), .RW(RW),
    .PW(PW),
    .PA(PA), .PB(PB), .PD(PD),
    .Clk(Clk), .LE(LE), .PC(PC)
  );

  initial begin
      // Initialize inputs
      PC = 32;        
      PW = 20;        
      RW = 4'b0000;   
      RA = 4'b0000;   
      RB = 4'b1111;   
      RD = 4'b1110;

      #1;
      LE = 1; // Enable writing

      // Loop through registers and increment values
      repeat(16) begin
          #4; // Wait 4 time units
          if (RA < 15) begin
              RW = RW + 1; 
              RA = RA + 1; 
              RB = RB + 1; 
              PW = PW + 1; 
              RD = RD + 1; 
          end
          #4; // Wait 4 time units
      end

      LE = 0; // Disable writing
  end

  // Clock creation
  initial begin   
      Clk = 0;
      forever #2 Clk = ~Clk; // Changes clock every 2 time units
  end

  initial begin   // // Display Outputs using $monitor
    $monitor("RW=%2d, RA=%2d, RB=%2d, RD=%2d, PW=%2d, PA=%2d, PB=%2d, PD=%2d", RW, RA, RB, RD, PW, PA, PB, PD);
  end

  initial begin 
      #200 $finish; //finishes the simulation
  end

endmodule
