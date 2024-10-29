`timescale 1ns / 1ns

module ARM_Pipeline_tb;
    reg Clk, Reset;
    ARM_Pipeline uut (.Clk(Clk), .Reset(Reset));

    initial begin
        // Initialize clock and reset
        Clk = 0;
        Reset = 1;

        // Clock generation: toggle every 2 time units
        forever #2 Clk = ~Clk;
    end

    initial begin
        // Simulation control for demonstration requirements
        #3 Reset = 0; // Release Reset at time 3
        #40 $finish; // End simulation at time 40
    end

    // Monitor outputs
    always @(posedge Clk) begin
        $display("Time=%0d | PC=%0d | Instruction=%h | Control Signals=%b | EX Control=%b | MEM Control=%b | WB Control=%b",
                  $time, uut.PC, uut.IF_ID_instruction, uut.control_unit.control_signals, uut.EX_control_signals, uut.MEM_control_signals, uut.WB_control_signals);
    end
endmodule
