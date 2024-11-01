`timescale 1ns / 1ns

module testbench();
    reg clk;
    reg reset;
    reg select;
    wire [31:0] pc_out;
    wire [31:0] instruction;
    wire ID_RF_enable, EX_RF_enable, MEM_RF_enable, WB_RF_enable, RFenable;
    wire [1:0] AM; 
    wire [31:0] NextPC;
    wire [1:0] ID_shift_AM,EX_shift_AM;
    wire [3:0] ID_alu_op,EX_alu_op,opcode;   

    wire [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2;
    wire ID_S_bit, ID_load_instr, ID_B_instr, ID_load_store_instr, ID_size, ID_BL_instr;
    wire [31:0] ID_instruction; 

    reg [7:0] Address;
    reg [7:0] data;
    integer fi;

    initial begin
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

    ProgramCounter pc (
        .Qs(pc_out),
        .Ds(reset ? 32'b0 : NextPC),
        .enable(1'b1),
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
        .Reset(reset),
        .IF_ID_enable(1'b1),
        .IF_instruction(instruction),
        .ID_instruction(ID_instruction)
    );

    control_unit control (
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
        .instruction(ID_instruction)
    );

    MUX mux (
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

    ID_EX_PipelineReg id_ex (
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

    EX_MEM_PipelineReg ex_mem (
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

    MEM_WB_PipelineReg mem_wb (
        .Clk(clk),
        .Reset(reset),
        .MEM_RF_enable(MEM_RF_enable),
        .WB_RF_enable(WB_RF_enable)
    );

    initial begin
        clk = 0;
        forever #2 clk = ~clk;  // Toggle clk every 2 time units
    end

    initial begin
        reset = 1;
        select = 0;
        #3 reset = 0;
        #32 select = 1;
    end

   initial begin
    $monitor("PC=%0d | Instruction=%b | Mnemonic=%c%c%c\n    ID Signals: S = %b AM = %b op = %b E = %b RF_Enable = %b B = %b BL = %b ReadWrite = %b size = %b\n    EX Signals: S_bit = %b AM = %b ALU_op = %b E = %b RF_Enable = %b B = %b BL = %b ReadWrite = %b size = %b\n    MEM Signals: E = %b ReadWrite = %b RF_Enable = %b size = %b\n    WB Signal: RF_Enable = %b",
             pc_out, instruction, ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
             ID_S_bit,ID_shift_AM,ID_alu_op, ID_load_store_instr, ID_RF_enable, ID_B_instr,ID_BL_instr, ID_load_instr, ID_size,
             EX_S_instr,EX_shift_AM, EX_alu_op, EX_load_store_instr, EX_RF_enable,EX_B_instr,EX_BL_instr, EX_load_instr, EX_size,
             MEM_load_store_instr, MEM_load_instr, MEM_RF_enable, MEM_size,
             WB_RF_enable);
             
    #52 $finish;
end

endmodule
