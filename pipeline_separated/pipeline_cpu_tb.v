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

    reg [7*8:1] opcode_name; // Register to store the opcode name as a string

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
        forever #2 clk = ~clk;
    end

    // Simulation control
    initial begin
        reset = 1;
        #3 reset = 0;

        //begin
        //    #4;
        //    if ($time % 4 == 0) begin
        //        reset = 0;
        //    end else if ($time >= 37) begin
        //        $finish;
        //    end
        //end

        #37 $finish;
    end

    // Opcode decoding logic
    always @(*) begin
        case (IF_ID_instr[24:21])
            4'b0000: opcode_name = "AND";
            4'b0001: opcode_name = "EOR";
            4'b0010: opcode_name = "SUB";
            4'b0011: opcode_name = "RSB";
            4'b0100: opcode_name = "ADD";
            4'b0101: opcode_name = "ADC";
            4'b0110: opcode_name = "SBC";
            4'b0111: opcode_name = "RSC";
            4'b1000: opcode_name = "TST";
            4'b1001: opcode_name = "TEQ";
            4'b1010: opcode_name = "CMP";
            4'b1011: opcode_name = "CMN";
            4'b1100: opcode_name = "ORR";
            4'b1101: opcode_name = "MOV";
            4'b1110: opcode_name = "BIC";
            4'b1111: opcode_name = "MVN";
            default: opcode_name = "NOP";
        endcase
    end

    initial begin

    end

endmodule
