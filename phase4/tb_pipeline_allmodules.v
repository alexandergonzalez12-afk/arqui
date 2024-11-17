`timescale 1ns / 1ns

module tb_pipeline();
    reg clk;
    reg reset;
    reg select;

    // PC, Instruction, and NextPC
    wire [31:0] pc_out;
    wire [31:0] instruction;
    wire [31:0] NextPC;

    // ID Signals
    wire [1:0] ID_shift_AM;
    wire [3:0] ID_alu_op;
    wire ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr, ID_load_store_instr, ID_size, ID_BL_instr;
    wire [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2;

    // EX Signals
    wire [31:0] ALU_result;
    wire [31:0] shifted_value; // Shifter output
    wire [3:0] EX_alu_op;
    wire [1:0] EX_shift_AM;
    wire EX_S_instr, EX_load_instr, EX_RF_enable, EX_load_store_instr, EX_size, EX_BL_instr, EX_B_instr;
    wire N, Z, C, V; // ALU Flags
    wire Branch, BranchLink;

    // MEM and WB Signals
    wire MEM_load_store_instr, MEM_size, MEM_RF_enable, MEM_load_instr;
    wire WB_RF_enable;

    // Multiplexer Outputs
    wire [1:0] AM;
    wire [3:0] opcode;
    wire S, load, RFenable, B, BL, size, ReadWrite;

    // Inputs for Shifter
    reg [31:0] Rm;  
    reg [11:0] I;   
    reg [1:0] AM_input;   

    // Temporary variables for reading memory
    reg [7:0] Address;
    reg [7:0] data;
    integer fi;

    initial begin
        // Read instructions from file into ROM
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi) begin
            for (Address = 0; Address < 48; Address = Address + 1) begin
                if ($fscanf(fi, "%b", data) != -1) begin
                    rom.mem[Address] = data;  
                end
            end
            $fclose(fi);  
        end
    end

    // Instantiate PC
    PC pc (
        .Qs(pc_out),
        .Ds(reset ? 32'b0 : NextPC),
        .enable(1'b1),
        .clk(clk),
        .reset(reset)
    );

    // Instantiate Adder
    Adder adder (
        .NextPC(NextPC),
        .PC(pc_out)
    );

    // Instantiate Instruction Memory
    Instruction_Memory_ROM rom (
        .Address(pc_out[7:0]),  
        .Instruction(instruction)
    );

    // Instantiate Control Unit
    ControlUnit controlunit (
        .ID_S_bit(ID_S_bit),
        .ID_load_instr(ID_load_instr),
        .ID_RF_enable(ID_RF_enable),
        .ID_B_instr(ID_B_instr),
        .ID_load_store_instr(ID_load_store_instr),
        .ID_size(ID_size),
        .ID_BL_instr(ID_BL_instr),
        .ID_shift_AM(ID_shift_AM),
        .ID_alu_op(ID_alu_op),
        .ID_mnemonic0(ID_mnemonic0),
        .ID_mnemonic1(ID_mnemonic1),
        .ID_mnemonic2(ID_mnemonic2),
        .instruction(instruction)
    );

    // Instantiate Multiplexer
    Multiplexer mux (
        .AM(AM),
        .opcode(opcode),
        .S(S),
        .load(load),
        .RFenable(RFenable),
        .B(B),
        .BL(BL),
        .size(size),
        .ReadWrite(ReadWrite),
        .ID_shift_AM(ID_shift_AM),
        .ID_alu_op(ID_alu_op),
        .ID_S_Bit(ID_S_bit),
        .ID_load_instr(ID_load_instr),
        .ID_RF_enable(ID_RF_enable),
        .ID_B_intr(ID_B_instr),
        .ID_load_store_instr(ID_load_store_instr),
        .ID_size(ID_size),
        .ID_BL_instr(ID_BL_instr),
        .select(select)
    );

    // Instantiate IF/ID Register
    IF_ID if_idreg (
        .Clk(clk),
        .Reset(reset),
        .IF_ID_enable(1'b1),
        .IF_instruction(instruction),
        .ID_instruction(ID_instruction)
    );

    // Instantiate ID/EX Register
    ID_EX id_exreg (
        .Clk(clk),
        .Reset(reset),
        .ID_S_instr(S),
        .ID_alu_op(ID_alu_op),
        .ID_load_instr(load),
        .ID_RF_enable(RFenable),
        .ID_load_store_instr(ReadWrite),
        .ID_size(size),
        .ID_BL_instr(BL),
        .ID_shift_AM(AM),
        .ID_B_instr(ID_B_instr),
        .EX_S_instr(EX_S_instr),
        .EX_alu_op(EX_alu_op),
        .EX_load_instr(EX_load_instr),
        .EX_RF_enable(EX_RF_enable),
        .EX_load_store_instr(EX_load_store_instr),
        .EX_size(EX_size),
        .EX_BL_instr(EX_BL_instr),
        .EX_B_instr(EX_B_instr),
        .EX_shift_AM(EX_shift_AM)
    );

    // Instantiate EX Stage
    EX_Stage ex_stage (
        .clk(clk),
        .reset(reset),
        .A(EX_S_instr),           // Replace with actual ALU input
        .B(EX_shift_AM),          // Replace with actual ALU input
        .alu_op(EX_alu_op),
        .update_flags(1'b1),
        .instruction(ID_instruction),
        .Rm(Rm),                  // Register Rm input for Shifter
        .I(I),                    // Immediate input for Shifter
        .AM(AM_input),            // Addressing Mode for Shifter
        .result(ALU_result),
        .shifted_value(shifted_value), // Shifter output
        .N(N),
        .Z(Z),
        .C(C),
        .V(V),
        .Branch(Branch),
        .BranchLink(BranchLink)
    );

    // Instantiate EX/MEM Register
    EX_MEM ex_memreg (
        .Clk(clk),
        .Reset(reset),
        .EX_load_store_instr(EX_load_store_instr),
        .EX_size(EX_size),
        .EX_RF_enable(EX_RF_enable),
        .EX_load_instr(EX_load_instr),
        .MEM_load_instr(MEM_load_instr),
        .MEM_load_store_instr(MEM_load_store_instr),
        .MEM_size(MEM_size),
        .MEM_RF_enable(MEM_RF_enable)
    );

    // Instantiate MEM/WB Register
    MEM_WB mem_wbreg (
        .Clk(clk),
        .Reset(reset),
        .MEM_RF_enable(MEM_RF_enable),
        .WB_RF_enable(WB_RF_enable)
    );

    // Clock Generation
    initial begin
        clk = 0;
        forever #2 clk = ~clk;  // Toggle clk every 2 time units
    end

    // Simulation Control
    initial begin
        reset = 1;
        select = 0;
        #3 reset = 0;
        #32 select = 1;
    end

    // Monitoring the Simulation
    initial begin
        $monitor("PC=%0d | Instr=%b | Shifted Value=%b | Branch=%b | BranchLink=%b | Flags: N=%b Z=%b C=%b V=%b\n    ID Signals: S=%b AM=%b Opcode=%b RF_E=%b B=%b BL=%b RW=%b size=%b",
                 pc_out, instruction, shifted_value, Branch, BranchLink, N, Z, C, V,
                 ID_S_bit, ID_shift_AM, ID_alu_op, ID_RF_enable, ID_B_instr, ID_BL_instr, ID_load_store_instr, ID_size);
        #52 $finish;
    end

endmodule
