`timescale 1ns / 1ps

module testbench();
    reg clk;
    reg reset;
    wire [31:0] pc_out, NextPC;
    wire IF_ID_enable;
    wire[31:0] ID_PA, ID_PB, ID_PD;
    wire [31:0] instruction, ID_instruction;
    integer fi;
    reg [7:0] Address, data;
    wire [3:0] ID_RA, ID_RB, ID_RD;
    wire [11:0] ID_shiftA;
    wire [3:0] ID_ICC;
    wire signed [23:0] imm_value;
    wire signed [31:0] imm_valuex4;
    wire ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr,ID_enable_instr, ID_size, ID_BL_instr, ID_RW;
    wire EX_RF_enable, MEM_RF_enable, WB_RF_enable;
    wire [3:0] EX_RD, MEM_RD, WB_RD;
    wire [31:0] TA,mux1;
    wire [1:0] ID_shift_AM;
    wire NOP;
    wire [3:0] ID_alu_op;
    wire [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2;
    wire [3:0] muxRDout;
    reg [31:0] prev_pc;
    integer loop_count;
    integer MAX_PROGRAM_SIZE,file_size;

    wire CH_B_ID;
    wire CH_BL_ID;
    wire [31:0] muxPAout, muxPBout, muxPDout;
    wire [1:0] AM;
    wire [3:0] opcode;
    wire S, load, RFenable, B, BL, size, ReadWrite, Enable;
    wire EX_S_instr;
    wire EX_load_instr;
    wire EX_enable_instr;
    wire EX_RW;
    wire EX_size;
    wire EX_BL_instr;
    wire EX_B_instr;
    wire [3:0] EX_alu_op;
    wire [1:0] EX_shift_AM;
    wire [31:0] EX_NextPC, ID_NextPC;
    wire CH_EX_BL;
    wire [31:0] EX_PA, EX_PB, EX_PD;
    wire [11:0] EX_shiftA;
    wire [3:0] PSR_CC;
    wire Cin;
    wire [3:0] ALU_CC;
    wire [31:0] ALU_out;
    wire [3:0] muxCCout;
    wire [31:0] muxALUout;
    wire [31:0] shifterOut;
    wire PC_LE;
    // Output wires for MEM stage
    wire MEM_enable_instr;
    wire MEM_size;
    wire MEM_load_instr;
    wire MEM_RW;
    wire [31:0] MEM_PA,MEM_PD;
    wire [31:0] MEM_ALU_out;
    wire [31:0] dataOut, muxDataout;
    wire WB_load_instr;
    wire [31:0] WB_MUX_out;
    wire [1:0] sop_count;

    wire [1:0] ID_MUX_PA, ID_MUX_PB, ID_MUX_PD;    
    reg prev_pc1, prev_pc2;
    
    ProgramCounter pc (
        .Qs(pc_out),
        .Ds(reset ? 32'b0 : mux1),
        .enable(PC_LE),
        .clk(clk),
        .reset(reset)
    );
    adder pc_adder (
        .NextPC(NextPC),
        .PC(pc_out)
    );

    rom256x8 rom (
        .Address(pc_out[7:0]),
        .Instruction(instruction)
    );
    IF_ID_PipelineReg if_id (
        .Clk(clk),
        .Reset(reset || CH_B_ID),
        .IF_ID_enable(IF_ID_enable),
        .IF_NextPC(NextPC),
        .IF_instruction(instruction),
        .ID_instruction(ID_instruction),
        .imm_value(imm_value),
        .ID_RA(ID_RA),
        .ID_RB(ID_RB),
        .ID_RD(ID_RD),
        .ID_shiftA(ID_shiftA),
        .ID_ICC(ID_ICC),
        .ID_NextPC(ID_NextPC)
    );
    control_unit control (
        .ID_S_bit(ID_S_bit),
        .ID_load_instr(ID_load_instr),
        .ID_RF_enable(ID_RF_enable),
        .ID_B_instr(ID_B_instr),
        .ID_enable_instr(ID_enable_instr),
        .ID_size(ID_size),
        .ID_BL_instr(ID_BL_instr),
        .ID_RW(ID_RW),
        .ID_shift_AM(ID_shift_AM),
        .ID_alu_op(ID_alu_op),
        .ID_mnemonic0(ID_mnemonic0),
        .ID_mnemonic1(ID_mnemonic1),
        .ID_mnemonic2(ID_mnemonic2),
        .sop_count(sop_count),
        .instruction(ID_instruction)
    );

    x4SE x4SE (
        .imm_valuex4(imm_valuex4),
        .imm_value(imm_value)
    );

    NextPC_Adder adder (
        .NextPC(ID_NextPC),
        .imm_valuex4(imm_valuex4),
        .out(TA)
    );

    mux2x32 muxNextPC (
        .out(mux1),
        .portA(NextPC),
        .portB(TA),
        .select(CH_B_ID)
    );

    RegisterFile RegisterFile(
        .clk(clk),
        .reset(reset),
        .LE(WB_RF_enable),
        .RW(WB_RD),
        .PW(WB_MUX_out),
        .RA(ID_RA),
        .RB(ID_RB),
        .RD(ID_RD),
        .PC(pc_out),
        .PA(ID_PA),
        .PB(ID_PB),
        .PD(ID_PD)
    );

    mux4x32 muxPA(
        .out(muxPAout),
        .portA(ID_PA),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PA)
    );

    mux4x32 muxPB(
        .out(muxPBout),
        .portA(ID_PB),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PB)
    );
    mux4x32 muxPD(
        .out(muxPDout),
        .portA(ID_PD),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PD)
    );

    mux2x4 muxRD(
        .out(muxRDout),
        .portA(ID_RD),
        .portB(4'b1110),
        .select(CH_BL_ID)
    );

    control_MUX cuMux(
        .AM(AM),
        .opcode(opcode),
        .S(S),
        .load(load),
        .RFenable(RFenable),
        .B(B),
        .BL(BL),
        .size(size),
        .ReadWrite(ReadWrite),
        .Enable(Enable),
        .ID_shift_AM(ID_shift_AM),
        .ID_alu_op(ID_alu_op),
        .ID_S_Bit(ID_S_bit),
        .ID_load_instr(ID_load_instr),
        .ID_RF_enable(ID_RF_enable),
        .ID_B_intr(ID_B_instr), 
        .ID_enable_instr(ID_enable_instr), 
        .ID_RW(ID_RW), 
        .ID_size(ID_size), 
        .ID_BL_instr(ID_BL_instr), 
        .select(NOP)
    );

    ID_EX_PipelineReg id_ex (
    .Clk(clk),                       // Connect `clk` signal to `Clk` input
    .Reset(reset),                   // Connect `reset` signal to `Reset` input
    .ID_S_instr(S),           // Connect `ID_S_Bit` to `ID_S_instr` input
    .ID_alu_op(opcode),           // Connect `ID_alu_op` to `ID_alu_op` input
    .ID_load_instr(load),   // Connect `ID_load_instr` to `ID_load_instr` input
    .ID_RF_enable(RFenable),     // Connect `ID_RF_enable` to `ID_RF_enable` input
    .ID_enable_instr(Enable), // Connect `ID_enable_instr` to `ID_enable_instr` input
    .ID_size(size),                  // Connect `size` signal to `ID_size` input
    .ID_RW(ID_RW),               // Connect `ReadWrite` to `ID_RW` input
    .ID_BL_instr(BL),                // Connect `BL` signal to `ID_BL_instr` input
    .ID_B_instr(B),                  // Connect `B` signal to `ID_B_instr` input
    .ID_shift_AM(AM),                // Connect `AM` signal to `ID_shift_AM` input
    .ID_PA(muxPAout),                // Connect `muxPAout` to `ID_PA` input
    .ID_PB(muxPBout),                // Connect `muxPBout` to `ID_PB` input
    .ID_PD(muxPDout),                // Connect `muxPDout` to `ID_PD` input
    .ID_shiftA(ID_shiftA),           // Connect `ID_shiftA` signal to `ID_shiftA` input
    .ID_RD(muxRDout),                // Connect `muxRDout` to `ID_RD` input
    .ID_NextPC(NextPC),              // Connect `NextPC` signal to `ID_NextPC` input
    .CH_ID_BL(CH_BL_ID), // Connect your signal to `CH_ID_BL` input
    .EX_S_instr(EX_S_instr),         // Connect `EX_S_instr` output
    .EX_load_instr(EX_load_instr),   // Connect `EX_load_instr` output
    .EX_RF_enable(EX_RF_enable),     // Connect `EX_RF_enable` output
    .EX_enable_instr(EX_enable_instr), // Connect `EX_enable_instr` output
    .EX_RW(EX_RW),                   // Connect `EX_RW` output
    .EX_size(EX_size),               // Connect `EX_size` output
    .EX_BL_instr(EX_BL_instr),       // Connect `EX_BL_instr` output
    .EX_B_instr(EX_B_instr),         // Connect `EX_B_instr` output
    .EX_alu_op(EX_alu_op),           // Connect `EX_alu_op` output
    .EX_shift_AM(EX_shift_AM),       // Connect `EX_shift_AM` output
    .EX_NextPC(EX_NextPC),           // Connect `EX_NextPC` output
    .CH_EX_BL(CH_EX_BL),             // Connect `CH_EX_BL` output
    .EX_RD(EX_RD),                   // Connect `EX_RD` output
    .EX_PA(EX_PA),                   // Connect `EX_PA` output
    .EX_PB(EX_PB),                   // Connect `EX_PB` output
    .EX_PD(EX_PD),                   // Connect `EX_PD` output
    .EX_shiftA(EX_shiftA)              // Connect `EX_shiftA` output
    );

    PSR PSR(
        .PSR_CC(PSR_CC),
        .Cin(Cin),
        .Alu_CC(ID_instruction[31:28]),
        .clk(clk)
    );

    condition_handler CH(
        .CC(ALU_CC),
        .ID_ICC_3128(ID_ICC),
        .ID_B_Instr(B),
        .ID_BL_Instr(BL),
        .Out_B(CH_B_ID),
        .Out_BL(CH_BL_ID)
    );

    ALU ALU(
        .Z(ALU_CC[2]),
        .N(ALU_CC[3]),
        .C(ALU_CC[1]),
        .V(ALU_CC[0]),
        .Out(ALU_out),
        .A(EX_PA),
        .B(shifterOut),
        .Cin(Cin),
        .opcode(EX_alu_op)

    );

    mux2x4 muxCC(
        .out(muxCCout),
        .portA(ALU_CC),
        .portB(PSR_CC),
        .select(EX_S_instr)
    );

    mux2x32 muxALU(
        .out(muxALUout),
        .portA(ALU_out),
        .portB(EX_NextPC),
        .select(CH_EX_BL)
    );

    shifter_sign_extender shifter(
        .N(shifterOut),
        .Rm(EX_PB),
        .I(EX_shiftA),
        .AM(EX_shift_AM)
    );

    HazardForwardingUnit HFU(
        .ID_MUX_PA(ID_MUX_PA),
        .ID_MUX_PB(ID_MUX_PB),
        .ID_MUX_PD(ID_MUX_PD),
        .NOP(NOP),
        .IF_ID_LE(IF_ID_enable),
        .PC_LE(PC_LE),
        .EX_Rd(EX_RD),
        .MEM_Rd(MEM_RD),
        .ID_load_instr(ID_load_instr),
        .ID_enable_instr(ID_enable_instr),
        .WB_Rd(WB_RD),
        .ID_RD(ID_RD),
        .ID_RA(ID_RA),
        .ID_RB(ID_RB),
        .EX_RF_enable(EX_RF_enable),
        .MEM_RF_enable(MEM_RF_enable),
        .WB_RF_enable(WB_RF_enable),
        .EX_load_instr(EX_load_instr),
        .sop_count(sop_count)
    );

    EX_MEM_PipelineReg ex_mem (
        .Clk(clk),                      // Clock signal
        .Reset(reset),                  // Reset signal
        .EX_enable_instr(EX_enable_instr),  // Input from EX stage
        .EX_size(EX_size),              // Size input from EX stage
        .EX_RF_enable(EX_RF_enable),    // Register File enable from EX stage
        .EX_load_instr(EX_load_instr),  // Load instruction flag from EX stage
        .EX_RW(EX_RW),                  // Read/Write control from EX stage
        .EX_PA(EX_PA),            
        .EX_PD(EX_PD),     // Address or data from EX stage
        .EX_ALU_out(muxALUout),        // ALU output from EX stage
        .EX_RD(EX_RD),
        .MEM_enable_instr(MEM_enable_instr),  // MEM stage enable
        .MEM_size(MEM_size),            // MEM stage size
        .MEM_RF_enable(MEM_RF_enable),  // MEM stage Register File enable
        .MEM_load_instr(MEM_load_instr), // MEM stage load instruction flag
        .MEM_RW(MEM_RW),                // MEM stage Read/Write control
        .MEM_PA(MEM_PA),                // MEM stage Address or data
        .MEM_ALU_out(MEM_ALU_out),      // MEM stage ALU output
        .MEM_RD(MEM_RD),   
        .MEM_PD(MEM_PD)
);

    ram256x8 ram(
        .DataOut(dataOut),
        .Enable(MEM_enable_instr),
        .ReadWrite(MEM_RW),
        .Address(MEM_ALU_out[7:0]),
        .DataIn(MEM_PD),
        .Size(MEM_size)
    );

    mux2x32 muxData(
      .out(muxDataout),
      .portA(MEM_ALU_out),
      .portB(dataOut),
      .select(MEM_load_instr)  
    );

    MEM_WB_PipelineReg mem_wb (
        .Clk(clk),
        .Reset(reset),
        .MEM_RF_enable(MEM_RF_enable),
        .MEM_load_instr(MEM_load_instr),
        .MEM_MUX_out(muxDataout),
        .MEM_RD(MEM_RD),
        .WB_RF_enable(WB_RF_enable),
        .WB_MUX_out(WB_MUX_out),
        .WB_RD(WB_RD)
    );
    // Replace ROM and RAM initialization logic with dynamic file size determination
     initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: Cannot open file.");
            $finish;
        end

        MAX_PROGRAM_SIZE = 0;

        // Read file line by line to determine file size and load ROM
        while (!$feof(fi)) begin
            if ($fscanf(fi, "%b", data) == 1) begin
                rom.mem[MAX_PROGRAM_SIZE] = data;
                MAX_PROGRAM_SIZE = MAX_PROGRAM_SIZE + 1;
            end
        end
        

        $display("Program loaded with %0d instructions.", MAX_PROGRAM_SIZE);
        $fclose(fi);
    end
     initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: Could not open input file.");
            $finish;
        end

        Address = 8'b00000000;
        while ($fscanf(fi, "%b", data) != -1) begin 
            ram.mem[Address] = data;  
            Address = Address + 1;
        end
        
        $fclose(fi);
    end


    initial begin
        clk = 0;       // Initialize clock
        reset = 1;     
        #3 reset = 0; 
        for (i = 0; i < 256; i = i + 4) begin
    // Check if at least one value in the current block is valid
    
    end
    end
    always #1 clk = ~clk; 
   



    // Monitor signal values throughout the simulation
initial begin
    $monitor("Time: %0d | PC: %d | R1: %d | R2: %d | R3: %d | R5: %d",
             $time, pc_out, 
             RegisterFile.reg_file[1], RegisterFile.reg_file[2], RegisterFile.reg_file[3], 
             RegisterFile.reg_file[5]) ;
end

// // Counter logic
reg [31:0] pc_history [0:10];
integer pc_count;
integer cnt;
integer i;
    initial begin
        pc_count=0;
        cnt=0;
    end

    integer file;

always @(posedge clk) begin
    pc_history[pc_count] = pc_out;
    pc_count = pc_count +1;
    if(pc_count==11) pc_count =0;
    for(i = 0;i <= 10; i= i+1) begin
        if(pc_out==pc_history[i]) cnt = cnt+1;
    end
    if(cnt>=11) begin
        $display("Infinite Loop Detected");
       for (i = 0; i < 256; i = i + 4) begin
    // Check if at least one value in the current block is valid
    if ((ram.mem[i] !== 8'bx) || (ram.mem[i+1] !== 8'bx) || 
        (ram.mem[i+2] !== 8'bx) || (ram.mem[i+3] !== 8'bx)) begin
        $display("RAM[%0d:%0d] = %b %b %b %b", 
                 i, i+3, 
                 ram.mem[i], ram.mem[i+1], ram.mem[i+2], ram.mem[i+3]);
                 
    end
end



        $finish;
    end else begin
        pc_count = 0;
    end
end




endmodule
