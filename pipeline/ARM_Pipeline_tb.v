`timescale 1ns / 1ns

module ARM_Pipeline_tb;
    reg Clk, Reset;
    ARM_Pipeline uut (.Clk(Clk), .Reset(Reset));

    initial begin
        // Initialize clock and reset
        Clk = 0;
        Reset = 1;

        // Generate clock signal: toggle every 2 time units
        forever #2 Clk = ~Clk;
    end

    initial begin
        // Simulation control based on demonstration requirements
        #3 Reset = 0;        // Release Reset at time 3
        #32 uut.S = 1;       // Set S to 1 at time 32
        #40 $finish;         // End simulation at time 40
    end

    initial begin
        // Preload the instruction memory with the required instructions
        $readmemh("instructions.hex", uut.imem.Mem); // Loading instructions.hex into instruction memory
    end

    // Monitor pipeline outputs at each clock cycle
    always @(posedge Clk) begin
        $display("Time=%0d | PC=%0d | Instruction=%h | Control Signals=%b | EX Control=%b | MEM Control=%b | WB Control=%b",
                  $time, uut.PC, uut.IF_ID_instruction, uut.control_unit.control_signals, uut.EX_control_signals, uut.MEM_control_signals, uut.WB_control_signals);
    end
endmodule
