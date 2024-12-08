`timescale 1ns / 1ps
`include "Modules.v"

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
    //done
    PC pc (
        .pc(pc_out),
        .next_pc(reset ? 32'b0 : mux1),
        .E(PC_LE),
        .clk(clk),
        .reset(reset)
    );
    //done  
    adder pc_adder (
        .PC(NextPC),
        .pc(pc_out)
    );
    //done
    Instruction_Memory_ROM rom (
        .A(pc_out[7:0]),
        .I(instruction)
    );
    //done
    IF_ID if_id (
        .clk(clk),
        .reset(reset || CH_B_ID),
        .E(IF_ID_enable),
        .next_pc(NextPC),
        .instr_in(instruction),
        .instr_out(ID_instruction),
        .instr_i23_i0(imm_value),
        .instr_i19_i16(ID_RA),
        .instr_i3_i0(ID_RB),
        .instr_i15_i12(ID_RD),
        .instr_i11_i0(ID_shiftA),
        .instr_i31_i28(ID_ICC),
        .Next_PC(ID_NextPC)
    );
    //done
    ControlUnit control (
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
//done
    x4SE x4SE (
        .instr_SE(imm_valuex4),
        .instr_I23_I0(imm_value)
    );
//done
    SUM_RF adder (
        .nextpc(ID_NextPC),
        .instr_SE(imm_valuex4),
        .TA(TA)
    );
//done
    MUX_Fetch muxNextPC (
        .MuxOut(mux1),
        .SUMOUT(NextPC),
        .TA(TA),
        .select(CH_B_ID)
    );
//done
    RegisterFile RegisterFile(
        .Clk(clk),
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
//done
    MUX_PA muxPA(
        .rf_pa(muxPAout),
        .pa(ID_PA),
        .jump_EX_pa(muxALUout),
        .jump_MEM_pa(muxDataout),
        .jump_WB_pa(WB_MUX_out),
        .S_PA(ID_MUX_PA)
    );
//done
    MUX_PB muxPB(
        .rf_pb(muxPBout),
        .pb(ID_PB),
        .jump_EX_pb(muxALUout),
        .jump_MEM_pb(muxDataout),
        .jump_WB_pb(WB_MUX_out),
        .S_PB(ID_MUX_PB)
    );
    //done
    MUX_PD muxPD(
        .rf_pd(muxPDout),
        .pd(ID_PD),
        .jump_EX_pd(muxALUout),
        .jump_MEM_pd(muxDataout),
        .jump_WB_pd(WB_MUX_out),
        .S_PD(ID_MUX_PD)
    );
//done
    MUX_I15_I12 muxRD(
        .result(muxRDout),
        .inst_I15_I12(ID_RD),
        
        .select(CH_BL_ID)
    );
//done
    Multiplexer cuMux(
        .id_am(AM),
        .alu_op(opcode),
        .id_s(S), //TBD
        .id_load(load),
        .rf_e(RFenable),
        .id_b(B),
        .id_bl(BL),
        .id_mem_size(size),
        .id_mem_write(ReadWrite),
        .id_mem_e(Enable),
        //input
        .ID_AM(ID_shift_AM),
        .ALU_OP(ID_alu_op),
        .ID_S(ID_S_bit),
        .ID_LOAD(ID_load_instr),
        .RF_E(ID_RF_enable),
        .ID_B(ID_B_instr), 
        .ID_MEM_E(ID_enable_instr), 
        .ID_MEM_WRITE(ID_RW), 
        .ID_MEM_SIZE(ID_size), 
        .ID_BL(ID_BL_instr), 
        .S(NOP)
    );
//done
    ID_EX id_ex (
    .clk(clk),                       // Connect `clk` signal to `Clk` input
    .reset(reset),                   // Connect `reset` signal to `Reset` input
    .ID_S(S),           // Connect `ID_S_Bit` to `ID_S_instr` input
    .ID_ALU_OP(opcode),           // Connect `ID_alu_op` to `ID_alu_op` input
    .ID_LOAD(load),   // Connect `ID_load_instr` to `ID_load_instr` input
    .RF_ENABLE(RFenable),     // Connect `ID_RF_enable` to `ID_RF_enable` input
    .ID_MEM_ENABLE(Enable), // Connect `ID_enable_instr` to `ID_enable_instr` input
    .ID_MEM_SIZE(size),                  // Connect `size` signal to `ID_size` input
    .ID_MEM_WRITE(ID_RW),               // Connect `ReadWrite` to `ID_RW` input
    .BL_OUT(BL),                // Connect `BL` signal to `ID_BL_instr` input
   // .B_OUT(B),               //added   // Connect `B` signal to `ID_B_instr` input
    .ID_AM(AM),                // Connect `AM` signal to `ID_shift_AM` input
    .MUX_PA(muxPAout),                // Connect `muxPAout` to `ID_PA` input
    .MUX_PB(muxPBout),                // Connect `muxPBout` to `ID_PB` input
    .MUX_PD(muxPDout),                // Connect `muxPDout` to `ID_PD` input
    .INSTR_I11_I0(ID_shiftA),           // Connect `ID_shiftA` signal to `ID_shiftA` input
    .MUX_INSTR_I15_I12(muxRDout),                // Connect `muxRDout` to `ID_RD` input
    .NEXT_PC(NextPC),              // Connect `NextPC` signal to `ID_NextPC` input
//======================================================================================
    //Hay que implementar CH_ID_BL
    //.CH_ID_BL(CH_BL_ID), // Connect your signal to `CH_ID_BL` input
//======================================================================================
    .id_s(EX_S_instr),         // Connect `EX_S_instr` output
    .EX_load_instr(EX_load_instr),   // Connect `EX_load_instr` output
    .EX_RF_enable(EX_RF_enable),     // Connect `EX_RF_enable` output
    .EX_enable_instr(EX_enable_instr), // Connect `EX_enable_instr` output
    .EX_RW(EX_RW),                   // Connect `EX_RW` output
    .EX_size(EX_size),               // Connect `EX_size` output
    .bl_out(EX_BL_instr),       // Connect `EX_BL_instr` output
   // .b_out(EX_B_instr),    //added     // Connect `EX_B_instr` output
    .id_alu_op(EX_alu_op),           // Connect `EX_alu_op` output
    .id_am(EX_shift_AM),       // Connect `EX_shift_AM` output
    .NEXT_PC(EX_NextPC),           // Connect `EX_NextPC` output
   // .CH_EX_BL(CH_EX_BL),             // Connect `CH_EX_BL` output
    .MUX_PD(EX_RD),                   // Connect `EX_RD` output
    .MUX_PA(EX_PA),                   // Connect `EX_PA` output
    .MUX_PB(EX_PB),                   // Connect `EX_PB` output
    .MUX_PD(EX_PD),                   // Connect `EX_PD` output
    .instr_i11_i0(EX_shiftA)              // Connect `EX_shiftA` output
    );
//done
    PSR PSR(
        .PSR_ConditionCode(PSR_CC),
        .C_in(Cin),
        .ConditionCode(ID_instruction[31:28]),
        .clk(clk)
    );
//done
    ConditionHandler CH(
        .ConditionCode(ALU_CC),
        .Condition(ID_ICC),
        .ID_B_instr(B),
        .ID_BL_instr(BL),
        .Branched(CH_B_ID),
        .EX_BL_instr(CH_BL_ID)
    );
//done
    ALU ALU(
        .Z(ALU_CC[2]),
        .N(ALU_CC[3]),
        .C(ALU_CC[1]),
        .V(ALU_CC[0]),
        .result(ALU_out),
        .A(EX_PA),
        .B(shifterOut),
        .C_IN(Cin),
        .alu_op(EX_alu_op)

    );
//done
    MUX_CC muxCC(
        .ConditionCodes(muxCCout),
        .ConditionCode(ALU_CC),
        .Flag_out(PSR_CC),
        .SIG_s(EX_S_instr)
    );
//done
    MUX_ALU muxALU(
        .DM_address(muxALUout),
        .alu_result(ALU_out),
        .Next_PC(EX_NextPC),
        .BL_OUT(CH_EX_BL)
    );
//done
    ARM_Shifter shifter(
        .N(shifterOut),
        .Rm(EX_PB),
        .I(EX_shiftA),
        .AM(EX_shift_AM)
    );
//done
    HazardUnit HFU(
        .forward_Rn(ID_MUX_PA),
        .forward_Rm(ID_MUX_PB),
        .forward_Rg(ID_MUX_PD),
        .NOP_EX(NOP),
        .IF_IF_Enable(IF_ID_enable),
        .PC_Enable(PC_LE),
        .EX_Rd(EX_RD),
        .MEM_Rd(MEM_RD),
        .ID_Load(ID_load_instr),
        .ID_Enable(ID_enable_instr),
        .WB_Rd(WB_RD),
        .ID_Rd(ID_RD),
        .ID_Rn(ID_RA),
        .ID_Rm(ID_RB),
        .EX_RF_enable(EX_RF_enable),
        .MEM_RF_enable(MEM_RF_enable),
        .WB_RF_enable(WB_RF_enable),
        .EX_Load(EX_load_instr),
        .sop_count(sop_count)
    );
//done
    EX_MEM ex_mem (
        .clk(clk),                      // Clock signal
        .reset(reset),                  // Reset signal
        .ID_MEM_ENABLE(EX_enable_instr),  // Input from EX stage
        .ID_MEM_SIZE(EX_size),              // Size input from EX stage
        .RF_ENABLE(EX_RF_enable),    // Register File enable from EX stage
        .ID_LOAD(EX_load_instr),  // Load instruction flag from EX stage
        .EXID_MEM_WRITE_RW(EX_RW),                  // Read/Write control from EX stage
        .MUX_PA(EX_PA),            
        .MUX_PD(EX_PD),     // Address or data from EX stage
        .DM_ADDRESS(muxALUout),        // ALU output from EX stage
        .MUX_INSTR_I15_I12(EX_RD),
        .id_mem_enable(MEM_enable_instr),  // MEM stage enable
        .id_mem_size(MEM_size),            // MEM stage size
        .rf_enable(MEM_RF_enable),  // MEM stage Register File enable
        .id_load(MEM_load_instr), // MEM stage load instruction flag
        .id_mem_write(MEM_RW),                // MEM stage Read/Write control
        .mux_pa(MEM_PA),                // MEM stage Address or data
        .dm_address(MEM_ALU_out),      // MEM stage ALU output
        .mux_instr_i15_i12(MEM_RD),   
        .mux_pd(MEM_PD)
);

    Data_Memory_RAM ram(
        .data_out(dataOut),
        .enable(MEM_enable_instr),
        .rw(MEM_RW),
        .address(MEM_ALU_out[7:0]),
        .data_In(MEM_PD),
        .size(MEM_size)
    );

    mux2x32 muxData(
      .MuxOut(muxDataout),
      .Addr(MEM_ALU_out),
      .DataOut(dataOut),
      .Sel(MEM_load_instr)  
    );

    MEM_WB_PipelineReg mem_wb (
        .clk(clk),
        .reset(reset),
        .RF_ENABLE(MEM_RF_enable),
        .ID_LOAD(MEM_load_instr),
        .MUX_DATAMEMORY(muxDataout),
        .MUX_INSTR_I15_I12(MEM_RD),
        .rf_enable(WB_RF_enable),
        .mux_datamemory(WB_MUX_out),
        .mux_instr_i15_i12(WB_RD)
    );
    // Replace ROM and RAM initialization logic with dynamic file size determination
     initial begin
        fi = $fopen("testcode_arm_ppu_2.txt", "r");
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
        fi = $fopen("testcode_arm_ppu_2.txt", "r");
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
    $monitor("Time: %0d | PC: %d | R1: %d | R2: %d | R3: %d | R5: %b",
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
        cnt = 0;
    end
end




endmodule