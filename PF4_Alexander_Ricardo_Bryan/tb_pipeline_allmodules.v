`timescale 1ns / 1ps

module tb_pipeline;

    // Inputs
    reg clk;
    reg reset;
    reg enable_pc;
    reg enable_ifid;
    reg S; // Multiplexer select

    // Outputs
    wire [31:0] pc;
    wire [31:0] instruction;

    // Pipeline registers for each stage
    // IF
    reg [31:0] if_instruction;
    reg ID_Load; // Signal LE for IF/ID
    reg [31:0] Next_PC; // PC + 4
    // ID
    reg [3:0] id_ALU_OP;
    reg [1:0] id_AM;
    reg id_LOAD, id_RF_E;
    reg [31:0] mux_pa;
    reg [31:0] mux_pb;
    reg [31:0] mux_pd;
    reg [3:0] mux_I15_I12;
    reg [11:0] instr_I11_I0;
    // EX
    reg [3:0] ex_ALU_OP;
    reg [1:0] ex_AM;
    reg ex_S, ex_LOAD;
    reg [7:0] mux_alu;
    // MEM
    reg mem_LOAD, mem_RF_E, mem_SIZE, mem_RW;
    reg [31:0] mux_datamemory;
    // WB
    reg wb_RF_E;

    // register file 
    reg [3:0] RA;
    reg [3:0] RB;
    reg [3:0] RD;
    reg [31:0] PC;
    reg [3:0] RW;    // double check this bits 
    reg [31:0] PW;   // double check this bits 
    reg [3:0] INSTR_I15_I12;

    //inputs mux pa, pb, pd
     // all mux pa,pb,pd jumps
    reg [31:0] jump_EX_pa;
    reg [31:0] jump_MEM_pa;
    reg [31:0] jump_WB_pa;
    reg [31:0] jump_EX_pb;
    reg [31:0] jump_MEM_pb;
    reg [31:0] jump_WB_pb;
    reg [31:0] jump_EX_pd;
    reg [31:0] jump_MEM_pd;
    reg [31:0] jump_WB_pd;

    //Condition Handler inputs 
    reg [3:0] ConditionCode;
    reg [1:0] N, Z, C, V;
    reg [31:0] instruction;
    reg [1:0] SIG_B;
    reg [1:0] SIG_BL;
    reg [1:0]  STORE_CC;
    reg [3:0] c_field;
    //mux for condition codes inputs
    reg [3:0] ConditionCode;
    reg [31:0] jump_MEM_instr;
    reg SIG_store_cc;

    //ALU inputs
    reg {31:0} A;
    reg {31:0} B;
    reg [3:0] alu_op;
    reg C_IN;

    //MUX_ALU inputs
    reg [31:0] alu_result;
    reg [31:0] Next_PC;
    reg BL_OUT;

    // Shifter inputs
    reg [31:0] Rm;            
    reg [11:0] I;           
    reg [1:0] AM;        

    //MUX DATA MEMORY
    reg [31:0] Addr;     
    reg [31:0] DataOut;    
    reg Sel;

    //Data Memory
    reg [31:0] dataMemoryIn;
    reg [7:0] dataMemoryAddress;
    reg dataMemory_mem_size, R/W, dataMemoryEnable; 

    //Flag register inputs (PSR)
    reg clk;
    reg reset;
    reg update;
    reg store_cc;
    reg N_in; 
    reg Z_in;
    reg C_in;
    reg V_in;
    
    // Pipeline outputs
    // IF/ID
    wire [23:0]instr_i23_i0;
    wire [31:0] NEXT_PC;
    wire [3:0] instr_i3_i0;
    wire [3:0] instr_i19_i16;
    wire [3:0] instr_i31_i28;
    wire [11:0] instr_i11_i0;
    wire [3:0] instr_i15_i12;
    // ID/EX
    // uses next pc from previous stage
    wire [31:0] MUX_PA;
    wire [31:0] MUX_PB;
    wire [31:0] MUX_PD;
    wire [3:0] MUX_I15_I12;
    // EX/MEM
    // uses the same i15_i12
    // uses the same pd
    wire [7:0] MUX_ALU;
    // MEM/WB
    // uses the same i15_i12
    wire [31:0] MUX_DATAMEMORY;

    // Control signals from ControlUnit
    wire [3:0] ALU_OP;
    wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E;
    wire [1:0] ID_AM;

    // Outputs from Multiplexer
    wire [3:0] mux_alu_op;
    wire mux_id_load, mux_id_mem_write, mux_store_cc, mux_id_b, mux_id_bl, mux_id_mem_size, mux_id_mem_e, mux_rf_e;
    wire [1:0] mux_id_am;

    // mux for condition codes
    wire [3:0] ConditionCodes;

    //ALU outputs
    wire [31:0] ALU_result;
    wire [31:0] Next_PC;
    wire BL_OUT;
    wire N, Z, C, V;

    //MUX_ALU
    wire [7:0] DM_address;

    // shifter output
    wire [31:0] N;

    //DATA MEMORY
    wire [31:0] Data_Memory_Out;    

    //MUX DATA MEMORY Output
    wire [31:0] MuxOut;

    // output TA
    wire [31:0] TA;
    wire [31:0] instr_SE;

    //Register File
    wire [31:0] PA;
    wire [31:0] PB;
    wire [31:32] PD;
    // all mux pa,pb,pd jumps
    wire [31:0] jump_EX_pa;
    wire [31:0] jump_MEM_pa;
    wire [31:0] jump_WB_pa;
    wire [31:0] jump_EX_pb;
    wire [31:0] jump_MEM_pb;
    wire [31:0] jump_WB_pb;
    wire [31:0] jump_EX_pd;
    wire [31:0] jump_MEM_pd;
    wire [31:0] jump_WB_pd;

    //Condition Handler Outputs
    wire [1:0] Branch;
    wire [1:0] BranchLink;
    wire [1:0] Stall;
    wire [1:0] NOP_EX;

    // Flag register outputs
    wire [1:0] N_out;
    wire [1:0] Z_out;
    wire [1:0] C_out;
    wire [1:0] V_out;

    // Registers for monitoring
    reg [31:0] r1, r2, r3, r5;

    integer fi, code;
    reg [31:0] data;       // For loading instruction data
    reg [7:0] address;     // Temporary address variable

    // Helper function to get the keyword based on opcode
    function [7*8:1] get_keyword;
      input [3:0] opcode;
      begin
        if(instruction == 32'b0)
          get_keyword = "NOP";
        else
          case (opcode)
            4'b0000: get_keyword = "AND";
            4'b0001: get_keyword = "EOR";
            4'b0010: get_keyword = "SUB";
            4'b0011: get_keyword = "RSB";
            4'b0100: get_keyword = "ADD";
            4'b0101: get_keyword = "ADC";
            4'b0110: get_keyword = "SBC";
            4'b0111: get_keyword = "RSC";
            4'b1000: get_keyword = "TST";
            4'b1001: get_keyword = "TEQ";
            4'b1010: get_keyword = "CMP";
            4'b1011: get_keyword = "CMN";
            4'b1100: get_keyword = "ORR";
            4'b1101: get_keyword = "MOV";
            4'b1110: get_keyword = "BIC";
            4'b1111: get_keyword = "MVN";
            default: get_keyword = "NOP"; // Default to NOP if opcode is unknown
          endcase
      end
    endfunction
    // Instantiate the IF/ID stage 
    IF_ID if_id (
        .E(enable_ifid),
        .reset(reset),
        .clk(clk),
        .instr_in(if_instruction),
        .next_pc(Next_PC),

        .instr_out(if_instruction),
        .instr_i23_i0(if_instruction),    
        .Next_PC(next_pc),
        .instr_i3_i0(if_instruction),
        .instr_i19_i16(if_instruction),
        .instr_i31_i28(if_instruction),
        .instr_i11_i0(if_instruction),
        .instr_i15_i12(if_instruction)
    );
    // // Instantiate ID/EX stage
    // EX_ID ex_id(
    //  .clk(clk),
    //  .reset(),
    //  .ID_ALU_OP(),
    //  .ID_LOAD(),
    //  .ID_MEM_WRITE(),
    //  .ID_MEM_SIZE(),
    //  .ID_MEM_ENABLE(),
    //  .ID_AM(),
    //  .STORE_CC(),
    //  .ID_BL(),
    //  .ID_B(),
    //  .RF_ENABLE(),
    //  .BL_OUT(),                   
    //  .NEXT_PC(),
    //  .MUX_PA(),
    //  .MUX_PB(),
    //  .PD(),
    //  .MUX_INSTR_I15_I12(),
    //  .INSTR_I11_I0(),      

    //  .id_alu_op(),
    //  .id_load(),
    //  .id_mem_write(),
    //  .id_mem_size(),
    //  .id_mem_enable(),
    //  .id_am(),
    //  .store_cc(),
    //  .id_bl(),
    //  .id_b(),
    //  .rf_enable(),
    //  .bl_out(),             
    //  .next_pc(),
    //  .mux_pa(),
    //  .mux_pb(),
    //  .pd(),
    //  .mux_instr_i15_i12(),
    //  .instr_i11_i0() 
    // );

    // Instantiate the PC module with PC increment of 4
    PC uut_pc (
        .clk(clk),
        .reset(reset),
        .E(enable_pc),
        .next_pc(pc + 32'd4), // Increment PC by 4
        .pc(pc)
    );

    // Instantiate the ControlUnit module
    ControlUnit uut_control (
        .instruction(instruction),
        .ALU_OP(ALU_OP),
        .ID_LOAD(ID_LOAD),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .ID_AM(ID_AM),
        .STORE_CC(STORE_CC),
        .ID_B(ID_B),
        .ID_BL(ID_BL),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_MEM_E(ID_MEM_E),
        .RF_E(RF_E)
    );

    // Instantiate the Multiplexer
    Multiplexer uut_mux (
        .alu_op(mux_alu_op),
        .id_load(mux_id_load),
        .id_mem_write(mux_id_mem_write),
        .store_cc(mux_store_cc),
        .id_b(mux_id_b),
        .id_bl(mux_id_bl),
        .id_mem_size(mux_id_mem_size),
        .id_mem_e(mux_id_mem_e),
        .rf_e(mux_rf_e),
        .id_am(mux_id_am),
        .S(S),
        .ALU_OP(ALU_OP),
        .ID_LOAD(ID_LOAD),
        .ID_MEM_WRITE(ID_MEM_WRITE),
        .STORE_CC(STORE_CC),
        .ID_B(ID_B),
        .ID_BL(ID_BL),
        .ID_MEM_SIZE(ID_MEM_SIZE),
        .ID_MEM_E(ID_MEM_E),
        .RF_E(RF_E),
        .ID_AM(ID_AM)
    );

    // Instantiate the instruction memory (ROM)
    Instruction_Memory_ROM rom_inst (
        .I(instruction),
        .A(pc) // Connect the program counter to the memory address
    );

    // Instantiate the data memory MUX
    MUX_DataMemory MUXdatamemory (
        .Addr(Addr),
        .DataOut(DataOut),
        .Sel(Sel),
        .MuxOut( Data_Memory_Out)
    );

    // Instantiate the data memory (RAM)
    Data_Memory_RAM data_mem_inst (
        .data_out(Data_Memory_Out),
        .address(dataMemoryAddress),
        .data_in(dataMemoryIn),
        .size(dataMemory_mem_size),
        .rw(R/W),
        .enable(dataMemoryEnable)
    );


    // Instantiate the ConditionHandler
    ConditionHandler uut_condition_handler (
        .ConditionCode(ConditionCode),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V),
        .instruction(instruction[31:28]),
        .SIG_B(SIG_B),
        .SIG_BL(SIG_BL),
        .STORE_CC(STORE_CC),
        .c_field(c_field), 
        .Branch(Branch),
        .BranchLink(BranchLink),
        .Stall(Stall),
        .NOP_EX(NOP_EX)
    );

    // Instantiate the ALU module
    ALU uut_alu (
        .A(A),
        .B(B),
        .ALU_OP(alu_op),
        .C_IN(C_IN),
        .ALU_result(ALU_result),
        .N(N),
        .Z(Z),
        .C(C),
        .V(V)
    );

    // instantiate the flag register module
    FlagRegister uut_flag_register (
        .clk(clk),
        .reset(reset),
        .update(update),
        .store_cc(store_cc),
        .N_in(N_in),
        .Z_in(Z_in),
        .C_in(C_in),
        .V_in(V_in),
        .N_out(N_out),
        .Z_out(Z_out),
        .C_out(C_out),
        .V_out(V_out)
    );
    // Clock generation with 2 time units toggle
    initial begin
        clk = 0;
        forever #2 clk = ~clk;
    end

    // Preload instructions from the file into the instruction memory
    initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: File could not be opened.");
            $finish;
        end

        // Start loading instructions from the file
        address = 8'd0;
        while (!$feof(fi)) begin
            code = $fscanf(fi, "%b", data);
            rom_inst.Mem[address] = data; // Preload the ROM memory
            address = address + 1;
        end
        $fclose(fi);
    end

    // Preload data memory from the file
    initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: File could not be opened.");
            $finish;
        end

        // Start loading data into the memory
        address = 8'd0;
        while (!$feof(fi)) begin
            code = $fscanf(fi, "%b", data);
            data_mem_inst.Mem[address] = data; // Preload the RAM memory
            address = address + 1;
        end
        $fclose(fi);
    end

    // Test sequence with enforced stop time at 40
    initial begin
        // Initialize signals
        reset = 1;
        enable_pc = 1;
        enable_ifid = 1;
        S = 0;

        // Start simulation
        #3 reset = 0;
        #32 S = 1;
        #20 $finish; // Stop simulation at time 40
    end

    always @(*) begin
        PC <= pc;

    end

    // Pipeline stages update
    always @(posedge clk) begin
        // IF stage
        if_instruction <= instruction;

        // //Recently added
        Next_PC <= pc;

        // ID stage
        id_ALU_OP <= ALU_OP;
        id_AM <= ID_AM;
        id_LOAD <= ID_LOAD;
        id_RF_E <= RF_E;

        //Recently added
        RA <= instr_i3_i0;
        RB <= instr_i19_i16;
        RD <= instr_i15_i12;

        // EX stage
        ex_ALU_OP <= mux_alu_op;
        ex_AM <= mux_id_am;
        ex_S <= S;
        ex_LOAD <= mux_id_load;

        // MEM stage
        mem_LOAD <= mux_id_load;
        mem_RF_E <= mux_rf_e;
        mem_SIZE <= mux_id_mem_size;
        mem_RW <= mux_id_mem_write;

        // WB stage
        wb_RF_E <= mux_rf_e;
    end

    // Monitor outputs for each clock cycle
    initial begin
        $monitor("PC: %0d | Address: %0d | r1: %0d | r2: %0d | r3: %0d | r5: %0d", pc, address, r1, r2, r3, r5);
    end

    // Display control signals and instruction in ID stage when required
    always @(posedge clk) begin
        $display("Control Signals in ID Stage: ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", id_ALU_OP, id_AM, id_LOAD, id_RF_E);
        $display("Instruction in ID Stage: %b", if_instruction);
    end
endmodule
