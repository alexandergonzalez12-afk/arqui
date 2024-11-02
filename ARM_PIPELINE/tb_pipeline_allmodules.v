`timescale 1ns / 1ns

module testbench();
    reg clk;
    reg reset;
    reg select;
    wire [31:0] pc_out;
    wire [31:0] instruction;
    wire ID_RF_E, EX_RF_E, MEM_RF_E, WB_RF_E, RFenable;
    wire [1:0] AM; 
    wire [31:0] NextPC;
    wire [1:0] ID_AM,EX_AM;
    wire [3:0] ALU_OP,EX_alu_op,opcode;   

    wire [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2;
    wire STORE_CC, ID_MEM_E, ID_B, ID_MEM_WRITE, ID_MEM_SIZE, ID_BL;
    wire [31:0] ID_instruction; 

    reg [7:0] Address;
    reg [7:0] data;
    integer fi;

    initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi) begin
            for (Address = 0; Address < 48; Address = Address + 1) begin
                if ($fscanf(fi, "%b", data) != -1) begin
                    rom.Mem[Address] = data;  
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

    
    Instruction_Memory_ROM rom (
        .A(pc_out[7:0]),  
        .I(instruction)
    );

    IF_ID if_id (
        .Clk(clk),
        .Reset(reset),
        .IF_ID_enable(1'b1),
        .IF_instruction(instruction),
        .ID_instruction(ID_instruction)
    );

    ControlUnit control (
        .STORE_CC(STORE_CC),
        .ID_MEM_E(ID_MEM_E),
        .ID_RF_E(ID_RF_E),
        .ID_B(ID_B),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_BL(ID_BL),
        .ID_AM(ID_AM),
        .ALU_OP(ALU_OP),
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
        .ID_AM(ID_AM),
        .ALU_OP(ALU_OP),
        .STORE_CC(STORE_CC),
        .ID_MEM_E(ID_MEM_E),
        .ID_RF_E(ID_RF_E),
        .ID_B(ID_B),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_BL(ID_BL),
        .select(select)
    );

    ID_EX id_ex (
        .Clk(clk),
        .Reset(reset),
        .STORE_CC(S),
        .ALU_OP(ALU_OP),
        .ID_MEM_E(load),
        .ID_RF_E(RFenable),
        .ID_MEM_WRITE(ReadWrite),
        .ID_MEM_SIZE(size),
        .ID_BL(BL),
        .ID_AM(AM),
        .ID_B(ID_B),
        .EX_STORE_CC(EX_STORE_CC),
        .EX_alu_op(EX_alu_op),
        .EX_MEM_E(EX_MEM_E),
        .EX_RF_E(EX_RF_E),
        .EX_MEM_WRITE(EX_MEM_WRITE),
        .EX_MEM_SIZE(EX_MEM_SIZE),
        .EX_BL(EX_BL),
        .EX_B(EX_B),
        .EX_AM(EX_AM)
    );

    EX_MEM ex_mem (
        .Clk(clk),
        .Reset(reset),
        .EX_MEM_WRITE(EX_MEM_WRITE),
        .EX_MEM_SIZE(EX_MEM_SIZE),
        .EX_RF_E(EX_RF_E),
        .EX_MEM_E(EX_MEM_E),
        .MEM_E(MEM_E),
        .MEM_WRITE(MEM_WRITE),
        .MEM_size(MEM_size),
        .MEM_RF_E(MEM_RF_E)
    );

    MEM_WB mem_wb (
        .Clk(clk),
        .Reset(reset),
        .MEM_RF_E(MEM_RF_E),
        .WB_RF_E(WB_RF_E)
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
             STORE_CC,ID_AM,ALU_OP, ID_MEM_WRITE, ID_RF_E, ID_B,ID_BL, ID_MEM_E, ID_MEM_SIZE,
             EX_STORE_CC,EX_AM, EX_alu_op, EX_MEM_WRITE, EX_RF_E,EX_B,EX_BL, EX_MEM_E, EX_MEM_SIZE,
             MEM_WRITE, MEM_E, MEM_RF_E, MEM_size,
             WB_RF_E);
             
    #52 $finish;
end

endmodule
