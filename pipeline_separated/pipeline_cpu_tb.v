// pipeline_cpu_tb.v
`timescale 1ns / 1ns

module pipeline_cpu_tb;
    reg clk, reset;

    // Outputs from the DUT
    wire [31:0] PC;
    wire [31:0] IF_ID_instr;
    wire [31:0] ID_EX_instr;
    wire [31:0] EX_MEM_instr;
    wire [31:0] MEM_WB_instr;

    // Instantiate the Pipeline CPU
    PipelineCPU dut (
        .clk(clk),
        .reset(reset),
        .PC(PC),
        .IF_ID_instr(IF_ID_instr),
        .ID_EX_instr(ID_EX_instr),
        .EX_MEM_instr(EX_MEM_instr),
        .MEM_WB_instr(MEM_WB_instr)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Simulation control
    initial begin
        reset = 1;
        #10 reset = 0;
        #200 $finish;
    end

    // Monitor output
    initial begin
        $monitor("Time=%0dns | PC=%h | IF/ID Instr=%h | ID/EX Instr=%h | EX/MEM Instr=%h | MEM/WB Instr=%h",
                 $time, PC, IF_ID_instr, ID_EX_instr, EX_MEM_instr, MEM_WB_instr);
    end
endmodule
