`timescale 1ns / 1ns

module pipeline_cpu_tb;
    reg clk;
    reg reset;

    // Instantiate the Pipeline CPU
    PipelineCPU cpu (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Reset sequence
    initial begin
        reset = 1;
        #15 reset = 0; // Release reset after 15ns
    end

    // Monitor pipeline stages
    initial begin
        $monitor("Time=%0dns | PC=%h | IF/ID Instr=%h | ID/EX Instr=%h | EX/MEM Instr=%h | MEM/WB Instr=%h",
                 $time, cpu.pc, cpu.IF_ID_instr, cpu.ID_EX_instr, cpu.EX_MEM_control_signals, cpu.MEM_WB_control_signals);
    end

    // Simulation end condition
    initial begin
        #100 $finish; // End simulation after 100ns
    end
endmodule
