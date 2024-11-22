`timescale 1ns / 1ns

module tb_pipeline();
    reg clk;
    reg reset;

    // Outputs from the pipeline CPU
    wire [31:0] pc_out;
    wire [31:0] instruction;
    wire [31:0] alu_result;
    wire branch_taken;

    // Temporary variables for reading instructions into memory
    reg [7:0] Address;
    reg [7:0] data;
    integer fi;

    // Instantiate the Pipeline CPU
    PipelineCPU pipeline (
        .clk(clk),
        .reset(reset),
        .pc(pc_out),
        .instruction(instruction),
        .alu_result(alu_result),
        .branch_taken(branch_taken)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #2 clk = ~clk;  // Toggle clock every 2 time units
    end

    // Simulation Control
    initial begin
        reset = 1;
        #3 reset = 0;  // Deassert reset after 3 time units
    end

    // Preload Instructions into ROM
    initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi) begin
            for (Address = 0; Address < 256; Address = Address + 1) begin
                if ($fscanf(fi, "%b", data) != -1) begin
                    pipeline.instruction_memory.mem[Address] = data;  
                end else begin
                    disable preload_loop;  // Stop when end-of-file is reached
                end
            end
            $fclose(fi);  
        end else begin
            $display("Error: Could not open file 'codigo_validacion.txt'");
            $finish;
        end
    end

    // Monitor Outputs
    initial begin
        $monitor(
            "Time=%0t | PC=%0d | Instr=%b | ALU_Result=%b | Branch=%b | Flags: N=%b Z=%b C=%b V=%b",
            $time, pc_out, instruction, alu_result, branch_taken,
            pipeline.ex_stage.N, pipeline.ex_stage.Z, pipeline.ex_stage.C, pipeline.ex_stage.V
        );
        #1000 $finish;  // End simulation after 1000 time units
    end
endmodule
