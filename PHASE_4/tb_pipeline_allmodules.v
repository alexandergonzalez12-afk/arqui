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

    // Hazard Unit Signals
    wire stall;
    wire flush;

    // Register for Shifter and immediate value
    wire [31:0] Rm;
    wire [11:0] I;
    wire [1:0] AM;

    // Additional wires for hazard detection
    wire [4:0] ID_Rn, ID_Rm, EX_Rd, MEM_Rd;
    wire EX_MemRead, MEM_MemRead;

    // Forwarding paths
    wire [31:0] jump_EX_pa, jump_MEM_pa, jump_WB_pa;
    wire [31:0] jump_EX_pb, jump_MEM_pb, jump_WB_pb;
    wire [31:0] jump_EX_pd, jump_MEM_pd, jump_WB_pd;

    // MUX and Register File Signals
    wire [31:0] PA, PB, PD; // Outputs from Register File
    wire [31:0] rf_pa, rf_pb, rf_pd; // Outputs from MUX
    wire [1:0] S_PA, S_PB, S_PD;

    // Instruction decoding signals
    wire [4:0] RA, RB, RD, RW; // Updated to 5 bits
    wire [31:0] PW;
    wire LE;

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
      

    // initial begin
    //     // Preload the memory with the content inside the file
    //     fi = $fopen("file_precarga_fase_I.txt", "r");  // Open the file in read mode
    //     if (fi) begin
    //     Address = 8'b00000000;                       // Start at address 0
    //     while (!$feof(fi)) begin                     // Loop until end of file
    //         code = $fscanf(fi, "%b", DataIn);          // Read 8-bit data from the file
    //         ram_inst.Mem[Address] = DataIn;            // Store the data in the RAM memory
    //         Address = Address + 1;                     // Increment address for next data
    //     end
    //     $fclose(fi);                                 // Close the input file
    //     end 

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
        $monitor("Time=%0t | PC=%0d | Instr=%b | ALU Result=%b | Flags: N=%b Z=%b C=%b V=%b | Branch=%b | Stall=%b | Flush=%b",
                 $time, pc_out, instruction, ALU_result, N, Z, C, V, Branch, stall, flush);
        #60 $finish;
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

    // Instantiate Instruction Memory
    // Data_Memory_RAM ram_inst (
    //     .data_out(DataOut), 
    //     .address(Address), 
    //     .data_in(DataIn), 
    //     .size(Size), 
    //     .rw(RW), 
    //     .enable(E)
    //     );

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

    // Instantiate EX_Stage
    EX_Stage ex_stage (
        .clk(clk),
        .reset(reset),
        .A(rf_pa),
        .B(rf_pb),
        .alu_op(EX_alu_op),
        .update_flags(1'b1),
        .instruction(instruction),
        .Rm(Rm),
        .I(I),
        .AM(AM),
        .result(ALU_result),
        .shifted_value(shifted_value),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V),
        .Branch(Branch),
        .BranchLink(BranchLink)
    );

    // Instantiate HazardUnit
    HazardUnit hazard_unit (
        .ID_Rn(RA[4:0]),   // Use only the lower 5 bits of RA
        .ID_Rm(RB[4:0]),   // Use only the lower 5 bits of RB
        .EX_Rd(RD[4:0]),   // Use only the lower 5 bits of RD
        .MEM_Rd(RW[4:0]),  // Use only the lower 5 bits of RW
        .EX_MemRead(EX_MemRead),
        .MEM_MemRead(MEM_MemRead),
        .Branch(Branch),
        .stall(stall),
        .flush(flush)
    );

    // Instantiate MUX_PA
    MUX_PA mux_pa (
        .pa(PA),
        .jump_EX_pa(jump_EX_pa),
        .jump_MEM_pa(jump_MEM_pa),
        .jump_WB_pa(jump_WB_pa),
        .S_PA(S_PA),
        .rf_pa(rf_pa)
    );

    // Instantiate MUX_PB
    MUX_PB mux_pb (
        .pb(PB),
        .jump_EX_pb(jump_EX_pb),
        .jump_MEM_pb(jump_MEM_pb),
        .jump_WB_pb(jump_WB_pb),
        .S_PB(S_PB),
        .rf_pb(rf_pb)
    );

    // Instantiate MUX_PD
    MUX_PD mux_pd (
        .pd(PD),
        .jump_EX_pd(jump_EX_pd),
        .jump_MEM_pd(jump_MEM_pd),
        .jump_WB_pd(jump_WB_pd),
        .S_PD(S_PD),
        .rf_pd(rf_pd)
    );

    // Instantiate FlagRegister
    FlagRegister flag_register (
        .clk(clk),
        .reset(reset),
        .update(1'b1),
        .N_in(N),
        .Z_in(Z),
        .C_in(C),
        .V_in(V),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V)
    );

    // Instantiate ARM_Shifter
    ARM_Shifter shifter (
        .Rm(rf_pd),
        .I(I),
        .AM(AM),
        .N(shifted_value)
    );

    // Instantiate ConditionHandler
    ConditionHandler condition_handler (
        .ConditionCode(instruction[31:28]),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V),
        .instruction(instruction),
        .Branch(Branch),
        .BranchLink(BranchLink)
    );
endmodule
