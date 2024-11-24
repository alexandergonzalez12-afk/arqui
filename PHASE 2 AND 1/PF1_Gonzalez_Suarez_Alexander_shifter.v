/**
 * @module ARM_Shifter
 * Implements Shifter/Sign Extender
 */

module ARM_Shifter (
    input [31:0] Rm,            // registro RM (input)
    input [11:0] I,             // valor inmediato
    input [1:0] AM,             // Addressing Mode
    output reg [31:0] N         //  output
);

always @(*) begin
    case (AM)
        2'b00: N = ({24'b0, I[7:0]} >> (2 * I[11:8])) | ({24'b0, I[7:0]} << (32 - 2 * I[11:8]));  // Rotate right (arreglado)
        2'b01: N = Rm;  // Pasa el valor Rm
        2'b10: N = {20'b0, I};  // Zero extend I[11:0] (ecuacion arreglada)
        2'b11: begin  // Addressing Mode 11: Shift Rm basado en I[11:7]
            case (I[6:5])
                2'b00: N = Rm << I[11:7];  // LSL
                2'b01: N = Rm >> I[11:7];  // LSR
                2'b10: N = $signed(Rm) >>> I[11:7];  // ASR
                2'b11: N = {Rm, Rm} >> I[11:7];  // ROR
                // default: N = 32'b0;
            endcase
        end
        // default: N = 32'b0;  // Default case para inicializar
    endcase
end

endmodule

/**
 * @module ARM_Shifter_TestBench
 * Testbench for ARM_Shifter
 */

`timescale 1ns / 1ns

module ARM_Shifter_TestBench;

// Inputs
reg [31:0] Rm;  
reg [11:0] I;   
reg [1:0] AM;   

// Outputs
wire [31:0] N;  
// Instantiate the Shifter module
ARM_Shifter dut (Rm, I, AM, N);

// nota para el profe: removi lo de dividir los strings por shift en addressing mode 11 por que confundia a la hora de leer
initial begin
    $monitor("\nAM Mode: %b \nRm: %b (binary)\nI: %b (binary)\nOutput N: %b (binary)\n",
              AM, Rm, I, N);
end

// Test cases
initial begin
    // Dumpfile for waveform
    $dumpfile("shifter_dump.vcd");
    $dumpvars(0, ARM_Shifter_TestBench);

    // Inputs as per task
    Rm = 32'b10000100001100011111111111101010;  
    I = 12'b001001101100;  

    // Test Case para AM = 00
    AM = 2'b00; #10;  

    // Test Case para AM = 01 
    AM = 2'b01; #10;  

    // Test Case para AM = 10 
    AM = 2'b10; #10;

    // Test Case para AM = 11 (Shift operations)
    AM = 2'b11; 
    // I = 12'b010000001100; #10;  // LSL
    // I = 12'b100000001100; #10;  // ASR
    // I = 12'b110000001100; #10;  // ROR

    // Finish the simulation
    $finish;
end

endmodule
