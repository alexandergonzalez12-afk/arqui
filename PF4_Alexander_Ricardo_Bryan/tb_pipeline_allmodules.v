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
    integer PROGRAM_SIZE,file_size;
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
    
    P_C pc (
        .pc(pc_out),
        .npc(reset ? 32'b0 : mux1),
        .enable(PC_LE),
        .clk(clk),
        .reset(reset)
    );
    adder pc_adder (
        .NextPC(NextPC),
        .PC(pc_out)
    );

    rom Rom (
        .Address(pc_out[7:0]),
        .Instruction(instruction)
    );
    IF_ID if_id (
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

    x4_SE x4SE (
        .imm_valuex4(imm_valuex4),
        .imm_value(imm_value)
    );

    NextPCadder adder (
        .NextPC(ID_NextPC),
        .imm_valuex4(imm_valuex4),
        .out(TA)
    );

    mux2IN32OUT muxNextPC (
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

    mux4IN32OUT muxPA(
        .out(muxPAout),
        .portA(ID_PA),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PA)
    );

    mux4IN32OUT muxPB(
        .out(muxPBout),
        .portA(ID_PB),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PB)
    );
    mux4IN32OUT muxPD(
        .out(muxPDout),
        .portA(ID_PD),
        .portB(muxALUout),
        .portC(muxDataout),
        .portD(WB_MUX_out),
        .select(ID_MUX_PD)
    );

    mux2IN4OUT muxRD(
        .out(muxRDout),
        .portA(ID_RD),
        .portB(4'b1110),
        .select(CH_BL_ID)
    );

    Multiplexer cuMux(
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

    ID_EX id_ex (
    .Clk(clk),                               
    .Reset(reset),                          
    .ID_S_instr(S),                         
    .ID_alu_op(opcode),                     
    .ID_load_instr(load),                   
    .ID_RF_enable(RFenable),                
    .ID_enable_instr(Enable),               
    .ID_size(size),                          
    .ID_RW(ID_RW),                          
    .ID_BL_instr(BL),                        
    .ID_B_instr(B),                          
    .ID_shift_AM(AM),                        
    .ID_PA(muxPAout),                       
    .ID_PB(muxPBout),                       
    .ID_PD(muxPDout),                       
    .ID_shiftA(ID_shiftA),                   
    .ID_RD(muxRDout),                       
    .ID_NextPC(NextPC),                      
    .CH_ID_BL(CH_BL_ID),                    
    .EX_S_instr(EX_S_instr),                
    .EX_load_instr(EX_load_instr),          
    .EX_RF_enable(EX_RF_enable),            
    .EX_enable_instr(EX_enable_instr),      
    .EX_RW(EX_RW),                          
    .EX_size(EX_size),                      
    .EX_BL_instr(EX_BL_instr),              
    .EX_B_instr(EX_B_instr),                
    .EX_alu_op(EX_alu_op),                  
    .EX_shift_AM(EX_shift_AM),              
    .EX_NextPC(EX_NextPC),                  
    .CH_EX_BL(CH_EX_BL),                    
    .EX_RD(EX_RD),                          
    .EX_PA(EX_PA),                          
    .EX_PB(EX_PB),                          
    .EX_PD(EX_PD),                          
    .EX_shiftA(EX_shiftA)                    
    );

    PSR PSR(
        .PSR_CC(PSR_CC),
        .Cin(Cin),
        .Alu_CC(ID_instruction[31:28]),
        .clk(clk)
    );

    ConditionHandler CH(
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

    mux2IN4OUT muxCC(
        .out(muxCCout),
        .portA(ALU_CC),
        .portB(PSR_CC),
        .select(EX_S_instr)
    );

    mux2IN32OUT muxALU(
        .out(muxALUout),
        .portA(ALU_out),
        .portB(EX_NextPC),
        .select(CH_EX_BL)
    );

    shifter Shifter(
        .N(shifterOut),
        .Rm(EX_PB),
        .I(EX_shiftA),
        .AM(EX_shift_AM)
    );

    HazardUnit HFU(
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

    EX_MEM ex_mem (
        .Clk(clk),                      
        .Reset(reset),                  
        .EX_enable_instr(EX_enable_instr),  
        .EX_size(EX_size),              
        .EX_RF_enable(EX_RF_enable),    
        .EX_load_instr(EX_load_instr),  
        .EX_RW(EX_RW),                  
        .EX_PA(EX_PA),            
        .EX_PD(EX_PD),     
        .EX_ALU_out(muxALUout),        
        .EX_RD(EX_RD),
        .MEM_enable_instr(MEM_enable_instr),  
        .MEM_size(MEM_size),            
        .MEM_RF_enable(MEM_RF_enable),  
        .MEM_load_instr(MEM_load_instr), 
        .MEM_RW(MEM_RW),                
        .MEM_PA(MEM_PA),                
        .MEM_ALU_out(MEM_ALU_out),      
        .MEM_RD(MEM_RD),   
        .MEM_PD(MEM_PD)
);

    ram Ram(
        .DataOut(dataOut),
        .Enable(MEM_enable_instr),
        .ReadWrite(MEM_RW),
        .Address(MEM_ALU_out[7:0]),
        .DataIn(MEM_PD),
        .Size(MEM_size)
    );

    mux2IN32OUT muxData(
      .out(muxDataout),
      .portA(MEM_ALU_out),
      .portB(dataOut),
      .select(MEM_load_instr)  
    );

    MEM_WB mem_wb (
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

    
     initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: Cannot open file.");
            $finish;
        end
        PROGRAM_SIZE = 0;
        // Read file line by line to determine file size and load ROM
        while (!$feof(fi)) begin
            if ($fscanf(fi, "%b", data) == 1) begin
                Rom.mem[PROGRAM_SIZE] = data;
                PROGRAM_SIZE = PROGRAM_SIZE + 1;
            end
        end
        $display("Program loaded %0d instructions.", PROGRAM_SIZE);
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
            Ram.mem[Address] = data;  
            Address = Address + 1;
        end
        $fclose(fi);
    end
    initial begin
        clk = 0;       
        reset = 1;     
        #3 reset = 0; 
        for (i = 0; i < 256; i = i + 4) begin
    end
    end
    always #1 clk = ~clk; 


    // Monitor signal values 
initial begin
    $monitor("PC: %d | R1: %d | R2: %d | R3: %d | R5: %d | R6: %d", pc_out, RegisterFile.regi[1], RegisterFile.regi[2], RegisterFile.regi[3], RegisterFile.regi[5] , RegisterFile.regi[6]) ;
end


// Counter logic
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
        if(pc_count == 10) pc_count = 0;
        for(i = 0; i <= 10; i = i + 1) begin
            if(pc_out == pc_history[i]) cnt = cnt+1;
        end
        if(cnt>=12) begin
            $display("Infinite Loop Detected");
            for (i = 0; i < 256; i = i + 4) begin
                //Check if at least one value in the current block is valid
                if ((Ram.mem[i] !== 8'bx) || (Ram.mem[i+1] !== 8'bx) || (Ram.mem[i+2] !== 8'bx) || (Ram.mem[i+3] !== 8'bx)) begin
                    //$display("RAM[%0d:%0d] = %b %b %b %b", i, i+3, Ram.mem[i], Ram.mem[i+1], Ram.mem[i+2], Ram.mem[i+3]);                 
                end
            end
            $finish;
        end 
        else begin
            pc_count = 0;
        end
    end
endmodule
