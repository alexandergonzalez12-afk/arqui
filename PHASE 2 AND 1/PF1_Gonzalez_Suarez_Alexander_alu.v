/**
 * @module ARM_ALU
 * Implementing the ALU with op and flags Z, N, C, V with inputs A and B
 */

module ARM_ALU (
    input [31:0] A, B,         // A & B 
    input CIN,                 // Carry in 
    input [3:0] OP,            // Operation code 
    output reg [31:0] Out,     // Output de 32
    output reg Z, N, C, V      // Flags
);

// Logic to handle ALU operations and set flags
always @(*) begin
    // Inicializamos
    Out = 32'b0;
    Z = 0;
    N = 0;
    C = 0;
    V = 0;

    case (OP)
        4'b0000: {C, Out} = A & B;             // A & B
        4'b0101: {C, Out} = A + B + CIN;       // A + B + CIN (with carry)
        4'b0010: {C, Out} = A - B;             // A - B
        4'b0110: {C, Out} = A - B - CIN;       // A - B - CIN (with carry)
        4'b0011: {C, Out} = B - A;             // B - A
        4'b0101: {C, Out} = B - A - CIN;       // B - A - CIN
        4'b0100: Out = A + B;                  // Bitwise AND
        4'b1100: Out = A | B;                  // Bitwise OR
        4'b0001: Out = A ^ B;                  // Bitwise XOR
        4'b1001: Out = A;                      // Pass A
        4'b1010: Out = B;                      // Pass B
        4'b1011: Out = ~B;                     // Bitwise NOT B
        4'b1110: Out = A & ~B;                 // Bit Clear (A AND NOT B)
        default: Out = 32'b0;                  // Default case
    endcase

    // Setear los flags
    Z = (Out == 32'b0);      // Z flag (Zero flag)
    N = Out[31];             // N flag (Negative flag)

    // Handler para Carry & Overflow 
    if (OP == 4'b0100 || OP == 4'b0001) begin  // add
        V = (~A[31] & ~B[31] & Out[31]) | (A[31] & B[31] & ~Out[31]); // overflow detection for A + B
    end else if (OP == 4'b0010 || OP == 4'b0011) begin  // sub
        V = (A[31] & ~B[31] & ~Out[31]) | (~A[31] & B[31] & Out[31]); // overflow detection for A - B
    end
end

endmodule

/**
 * @module ARM_ALU_testbench
 * @brief Testbench for the ARM ALU module with the required test values.
 */

`timescale 1ns / 1ns

module ARM_ALU_testbench;

reg [31:0] A, B;           // Input operands A and B
reg CIN;                   // Carry in
reg [3:0] OP;              // Operation code
wire [31:0] Out;           // ALU output
wire Z, N, C, V;           // ALU flags

// Instantiate el ALU
ARM_ALU dut (A, B, CIN, OP, Out, Z, N, C, V);

reg [256*8:1] operation_str;  // Intermediate net for operation string
reg [31:0] A_decimal, B_decimal, Out_decimal;  // Intermediate variables for decimal representation

// Monitor statement
initial begin
    $monitor("\nOperation: %s\n-------------------------\nA: %0d (decimal), %b (binary)\nB: %0d (decimal), %b (binary)\nOutput: %0d (decimal), %b (binary)\nFlags: Z=%b N=%b C=%b V=%b\n-------------------------",
              operation_str, A_decimal, A, B_decimal, B, Out_decimal, Out, Z, N, C, V);
end

// Update operation string and decimal values
always @(OP or A or B or Out) begin
    operation_str = get_operation_string(OP);
    A_decimal = A;
    B_decimal = B;
    Out_decimal = Out;
end

// Function to get operation string
function [256*8:1] get_operation_string;
    input [3:0] OP;
    begin
        case (OP)
            4'b0000: get_operation_string = "A + B";
            4'b0001: get_operation_string = "A + B + CIN";
            4'b0010: get_operation_string = "A - B";
            4'b0011: get_operation_string = "A - B - CIN";
            4'b0100: get_operation_string = "B - A";
            4'b0101: get_operation_string = "B - A - CIN";
            4'b0110: get_operation_string = "A & B (AND)";
            4'b0111: get_operation_string = "A | B (OR)";
            4'b1000: get_operation_string = "A ^ B (XOR)";
            4'b1001: get_operation_string = "Pass A";
            4'b1010: get_operation_string = "Pass B";
            4'b1011: get_operation_string = "NOT B";
            4'b1100: get_operation_string = "A & ~B (Bit Clear)";
            default: get_operation_string = "Unknown Operation";
        endcase
    end
endfunction

initial begin
    // dumpfile para el waveform
    $dumpfile("alu_dump.vcd");
    $dumpvars(0, ARM_ALU_testbench);

    // Los tests values asignados
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    CIN = 0;  // Carry in es 0

    // Los Ops val con delay de 2
    OP = 4'b0000; #2;
    OP = 4'b0010; #2;
    OP = 4'b0100; #2;
    OP = 4'b0110; #2;
    OP = 4'b1000; #2;
    OP = 4'b1010; #2;
    OP = 4'b1100; #2;

    // Cambiar carry in a 1 & repetimos el OP de 0000 a 0101
    CIN = 1;
    OP = 4'b0000; #2;
    OP = 4'b0001; #2;
    OP = 4'b0010; #2;
    OP = 4'b0011; #2;
    OP = 4'b0100; #2;
    OP = 4'b0101; #2;

    // Test for OR (A | B)
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    CIN = 0;
    OP = 4'b0111; #10;  // A | B (OR)

    // Test for Pass A
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    OP = 4'b1001; #10;  // Pass A

    // Test for Pass B
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    OP = 4'b1010; #10;  // Pass B

    // Test for NOT B
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    OP = 4'b1011; #10;  // NOT B

    // Test for A & ~B (Bit Clear)
    A = 32'b10011100000000000000000000111000;
    B = 32'b01110000000000000000000000000011;
    OP = 4'b1100; #10;  // A & ~B (Bit Clear)

    $finish;
end

endmodule
