`timescale 1ns / 1ps
`include "Pipeline_Modules.v"

module tb_pipeline;

    // Inputs
    reg clk;
    reg reset;
    reg enable_pc;
    reg enable_ifid;
    reg S; // Multiplexer select


    wire [31:0] adderrf_ta_fetch;
    wire chandler_branch_mux;

    // Outputs
    wire [31:0] pc;
    wire [31:0] npc;
    wire [31:0] fetch_npc_pc;
    wire [31:0] if_npc_fetch;
    wire [31:0] pc_adder;
    wire [31:0] instruction;

    // Pipeline registers for each stage
    wire [31:0] if_instruction;

    // Pipeline outputs
    // IF/ID
    wire [23:0]instr_i23_i0;
    wire [31:0] NEXT_PC;
    wire [3:0] instr_i3_i0;
    wire [3:0] instr_i19_i16;
    wire [3:0] instr_i31_i28;
    wire [11:0] instr_i11_i0;
    wire [3:0] instr_i15_i12;

    wire [3:0] cu_idaluop_mux;
    wire cu_idload_mux;
    wire cu_idmemwrite_mux;
    wire [1:0] cu_idam_mux;
    wire cu_storecc_mux;
    wire cu_idb_mux;
    wire cu_idbl_mux;
    wire cu_idmemsize_mux;
    wire cu_idmeme_mux;
    wire cu_rfe_rfmux;

    // ====================================
    // RF Enable Mux
    // ====================================
    wire rfmux_rfe_cumux;
    wire chandler_blout_rfmux;


    // ====================================
    // Control Unit Mux wires
    // ====================================

    wire [3:0] mux_aluop_id;
    wire mux_idload_id;
    wire mux_memwrite_id;
    wire mux_storecc_id;
    wire mux_memsize_id;
    wire mux_meme_id;
    wire mux_rfe_id;
    wire [1:0] mux_idam_id;

    wire mux_bl_chandler;
    wire mux_b_chandler;
    wire hazard_cumuxenable_mux;

    // ====================================
    // Control Unit Mux wires
    // ====================================

    wire [3:0] wb_registerrw_rf;
    wire wb_registerle_rf;
    wire [31:0] wb_registerpw_rf;
    
    wire [31:0] rf_registerpa_mux;
    wire [31:0] rf_registerpb_mux;
    wire [31:0] rf_registerpd_mux;

    
    
    
    
    
    






    // ====================================
    // 
    // ====================================


    /////////////////////////////////////////////////


    wire [3:0] id_ALU_OP;
    wire [1:0] id_AM;
    wire id_LOAD, id_RF_E;
    wire [3:0] ex_ALU_OP;
    wire [1:0] ex_AM;
    wire ex_S, ex_LOAD;
    wire mem_LOAD, mem_RF_E, mem_SIZE, mem_RW;
    wire wb_RF_E;

    // Control signals from ControlUnit
    wire [3:0] ALU_OP;
    wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E;
    wire [1:0] ID_AM;

    // Outputs from Multiplexer
    wire [3:0] mux_alu_op;
    wire mux_id_load, mux_id_mem_write, mux_store_cc, mux_id_b, mux_id_bl, mux_id_mem_size, mux_id_mem_e, mux_rf_e;
    wire [1:0] mux_id_am;




    // ID/EX
    // uses next pc from previous stage
    wire [31:0] MUX_PA;
    wire [31:0] MUX_PB;
    wire [31:0] MUX_PD;
    wire [3:0] MUX_I15_I12;


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

    adder addy(
        .PC (npc), // next pc
        .pc (pc_adder)
    );

    // Instantiate the PC module with PC increment of 4
    PC uut_pc (
        .next_pc    (pc + 32'd4), // input
        .pc         (pc_adder), // output
        .E          (enable_pc),
        .clk        (clk),
        .reset      (reset)
    );


    IF_ID if_id (
        .E(enable_ifid),
        .reset(reset),
        .clk(clk),
        .instr_in(instruction),
        .next_pc(npc),

        .instr_out(if_instruction),
        .instr_i23_i0(instr_i23_i0),    
        .Next_PC(if_npc_fetch),
        .instr_i3_i0(instr_i3_i0),
        .instr_i19_i16(instr_i19_i16),
        .instr_i31_i28(instr_i31_i28),
        .instr_i11_i0(instr_i11_i0),
        .instr_i15_i12(instr_i15_i12)
    );

    MUX_Fetch fetch (
        .SUMOUT (npc),
        .TA     (adderrf_ta_fetch),
        .Sel    (chandler_branch_mux),
        .MuxOut (fetch_npc_pc)
    );

    // Instantiate the ControlUnit module
    ControlUnit uut_control (
        .instruction        (instruction),
        .ALU_OP             (cu_idaluop_mux),
        .ID_LOAD            (cu_idload_mux),
        .ID_MEM_WRITE       (cu_idmemwrite_mux),
        .STORE_CC           (cu_storecc_mux),
        .ID_MEM_SIZE        (cu_idmemsize_mux),
        .ID_MEM_E           (cu_idmeme_mux),
        .RF_E               (cu_rfe_rfmux),
        .ID_AM              (cu_idam_mux),

        .ID_B               (cu_idb_mux),
        .ID_BL              (cu_idbl_mux)
    );

    // RF Enable Mux
    MUX_RFenable mux_rf (
        .id_rf_e (cu_rfe_rfmux),
        .s_rfenable (chandler_blout_rfmux),
        .out_rf_enable (rfmux_rfe_cumux)
    );
    

    // Instantiate the Multiplexer
    Multiplexer uut_mux (
        .alu_op         (cu_idaluop_mux),
        .id_load        (cu_idload_mux),
        .id_mem_write   (cu_idmemwrite_mux),
        .store_cc       (cu_storecc_mux),
        .id_mem_size    (cu_idmemsize_mux),
        .id_mem_e       (cu_idmeme_mux),
        .rf_e           (cu_rfe_rfmux),
        .id_am          (cu_idam_mux),

        .id_bl          (mux_bl_chandler),
        .id_b           (mux_b_chandler),

        .S              (hazard_cumuxenable_mux),

        .ALU_OP         (mux_aluop_id),
        .ID_LOAD        (mux_idload_id),
        .ID_MEM_WRITE   (mux_memwrite_id),
        .STORE_CC       (mux_storecc_id),
        .ID_MEM_SIZE    (mux_memsize_id),
        .ID_MEM_E       (mux_meme_id),
        .RF_E           (mux_rfe_id),
        .ID_AM          (mux_idam_id),

        .ID_B           (mux_bl_chandler),
        .ID_BL          (mux_b_chandler)
    );


    Three_port_register_file tprf (
    .RA   (instr_i3_i0),
    .RB   (instr_i19_i16),
    .RD   (instr_i15_i12),
    .RW   (wb_registerrw_rf),
    .PW   (wb_registerpw_rf),
    .PC   (pc),
    .Clk  (clk), 
    .LE   (wb_registerle_rf),
    .PA   (rf_registerpa_mux), 
    .PB   (rf_registerpb_mux), 
    .PD   (rf_registerpd_mux)
    );


    // ID_EX id_ex(

    // )


    // Instantiate the instruction memory (ROM)
    Instruction_Memory_ROM rom_inst (
        .I(instruction),
        .A(pc[7:0]) // Connect the program counter to the memory address
    );


    // // Instantiate the data memory (RAM)
    // Data_Memory_RAM data_mem_inst (
    //     .data_out(Data_Memory_Out),
    //     .address(dataMemoryAddress),
    //     .data_in(dataMemoryIn),
    //     .size(dataMemory_mem_size),
    //     .rw(R/W),
    //     .enable(dataMemoryEnable)
    // );

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
            //  data_mem_inst.Mem[address] = data; // Preload the RAM memory
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
        // #20 $finish; // Stop simulation at time 40
    end

//     // Pipeline stages update
    always @(posedge clk) begin


//         // // IF stage
//         // if_instruction <= instruction;

//         // // ID stage
//         // id_ALU_OP <= ALU_OP;
//         // id_AM <= ID_AM;
//         // id_LOAD <= ID_LOAD;
//         // id_RF_E <= RF_E;

//         // // EX stage
//         // ex_ALU_OP <= mux_alu_op;
//         // ex_AM <= mux_id_am;
//         // ex_S <= S;
//         // ex_LOAD <= mux_id_load;

//         // // MEM stage
//         // mem_LOAD <= mux_id_load;
//         // mem_RF_E <= mux_rf_e;
//         // mem_SIZE <= mux_id_mem_size;
//         // mem_RW <= mux_id_mem_write;

//         // WB stage
//         // wb_RF_E <= mux_rf_e;
    end

    // Display outputs for each clock cycle
    always @(posedge clk) begin
        $display("PC: %0d | Opcode: %s", pc, get_keyword(instruction[24:21]));
        $display("FUNCIONA CABRON: %d", if_instruction);
        $display("-----------------------------------------------------------");
        // $display("Fetch Stage:    Instruction: %b", if_instruction);
        // $display("Decode Stage:   ALU_OP: %b | AM: %b | Load: %b | RF_E: %b", id_ALU_OP, id_AM, id_LOAD, id_RF_E);
        // $display("Execute Stage:  ALU_OP: %b | AM: %b | S: %b | Load: %b", ex_ALU_OP, ex_AM, ex_S, ex_LOAD);
        // $display("Memory Stage:   Load: %b | RF_E: %b | Mem Size: %b | RW: %b", mem_LOAD, mem_RF_E, mem_SIZE, mem_RW);
        // $display("Write Back:     RF_E: %b", wb_RF_E);
        $display("-----------------------------------------------------------\n");
    end
endmodule
