`timescale 1ns / 1ps
`include "Pipeline_Modules.v"


module tb_pipeline;

    // Inputs
    reg clk;
    reg reset;
    reg enable_pc;
    reg enable_ifid;
    reg S; // Multiplexer select

    // Outputs
    wire [31:0] pc;
    wire [31:0] npc;


    wire [31:0] if_npc_fetch;
    wire [31:0] instruction;

    // Pipeline registers for each stage
    wire [31:0] if_instruction;
    reg [3:0] id_ALU_OP;
    reg [1:0] id_AM;
    reg id_LOAD, id_RF_E;
    reg [3:0] ex_ALU_OP;
    reg [1:0] ex_AM;
    reg ex_S, ex_LOAD;
    reg mem_LOAD, mem_RF_E, mem_SIZE, mem_RW;
    reg wb_RF_E;



    // Control signals from ControlUnit
    wire [3:0] ALU_OP;
    wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_MEM_SIZE, ID_MEM_E, RF_E;
    wire [1:0] ID_AM;

    // Outputs from Multiplexer
    wire [3:0] mux_alu_op;
    wire mux_id_load, mux_id_mem_write, mux_store_cc, mux_id_b, mux_id_bl, mux_id_mem_size, mux_id_mem_e, mux_rf_e;
    wire [1:0] mux_id_am;

    integer fi, code;
    reg [31:0] data;       // For loading instruction data
    reg [7:0] address;     // Temporary address variable


    // ====================================
    // Mux of Target Address
    // ====================================
    wire [31:0] adderrf_ta_fetch;
    wire chandler_branch_mux;

    // ====================================
    // IF/ID
    // ====================================
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
    wire cu_aluop_mux;
    wire cu_meme_mux;
    wire cu_rfe_mux;


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
    // register file 
    // ====================================

    wire [3:0] wb_registerrw_rf;
    wire wb_registerle_rf;
    wire [31:0] wb_registerpw_rf;
    
    wire [31:0] rf_registerpa_mux;
    wire [31:0] rf_registerpb_mux;
    wire [31:0] rf_registerpd_mux;


    // ====================================
    // 4X SE 
    // ====================================
    wire [31:0] x4_shift_adderta;

// ===
    wire [31:0] fetch_npc_pc;
//= ==

    // ====================================
    // Muxes or something
    // ====================================
    wire chandler_blout_idmux;
    wire [3:0] idmux_out_ex;


    // ====================================
    // Condition handler
    // ====================================

    wire alu_z_chandler;
    wire alu_n_chandler;
    wire alu_c_chandler;
    wire alu_v_chandler;
    wire [3:0] chandlermux_cc_chandler;

    // ====================================
    // ID/EX
    // ====================================

    wire [3:0] ex_aluop_alu;
    wire ex_load_mem;
    wire ex_memwrite_mem;
    wire ex_memsize_mem;
    wire ex_memenable_mem;
    wire [1:0] ex_am_shifter;
    wire ex_storecc_psr;
    wire ex_rfenable_mem;
    wire ex_blout_muxalu;
    wire [31:0] ex_nextpc_muxalu;
    wire [31:0] ex_muxpa_alu;
    wire [31:0] ex_muxpb_shifter;
    wire [31:0] ex_muxpd_mem;
    wire [3:0] ex_muxinstri15i12_memandhazard;
    wire [11:0] ex_instri11i0_shifter;

    // ====================================
    // ALU MUX / ALU
    // ====================================

    wire [31:0] alu_out_muxaluandidmuxes;
    wire [31:0] alumux_dmaddress_mem;

    // ====================================
    // SHIFTER
    // ====================================

    wire [31:0] shifter_n_alu;
    

    // ====================================
    // hazard Unit
    // ====================================

    wire [31:0] nop;
    reg foward_rm;
    reg foward_rn;
    reg foward_rd;
    reg foward_rg;

    // ====================================
    // hazard Unit
    // ====================================

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

    // Instantiate the PC module with PC increment of 4
    PC uut_pc (
        .clk(clk),
        .reset(reset),
        .E(enable_pc),
        .next_pc(fetch_npc_pc), // Increment PC by 4 // in
        .pc(pc) // out
    );

    adder addy (
        .pc (pc), // in
        .PC (npc) // out
    );


    X4_SE x4_se (
        .instr_I23_I0 (instr_i23_i0),
        .instr_SE     (x4_shift_adderta)
    );


    MUX_Fetch fetch (
        .SUMOUT (npc),
        .TA     (adderrf_ta_fetch),
        .Sel    (chandler_branch_mux),
        .MuxOut (fetch_npc_pc)
    );


    SUM_RF sum_rf (
        .instr_SE   (x4_shift_adderta),
        .nextpc     (if_npc_fetch),
        .TA         (adderrf_ta_fetch)
    );

    MUX_I15_I12 mux_i15_i12 ( // this is the immediate destiny mux btw
        .inst_I15_I12 (instr_i15_i12),
        .BL_out       (chandler_blout_idmux),
        .result       (idmux_out_ex)
    );




    // MUX_PA mux_pa (
    //     .pa             (rf_registerpa_mux),
    //     .jump_EX_pa     (),
    //     .jump_MEM_pa    (),
    //     .jump_WB_pa     (),
    //     .S_PB           (),
    //     .rf_pb          ()
    // );

//     MUX_PB mux_pb (
//         .pb             (rf_registerpb_mux),
//         .jump_EX_pb     (),
//         .jump_MEM_pb    (),
//         .jump_WB_pb     (),
//         .S_PB           (),
//         .rf_pB          ()
//     );

//     MUX_PD mux_pd (
//         .pd             (rf_registerpd_mux),
//         .jump_EX_pd     (),
//         .jump_MEM_pd    (),
//         .jump_WB_pd     (),
//         .S_PD           (),
//         .rf_pd          ()
//     );    


ID_EX id_ex (
    .clk                    (clk),
    .reset                  (reset),
    .ID_ALU_OP              (mux_aluop_id),
    .ID_LOAD                (mux_idload_id),
    .ID_MEM_WRITE           (mux_memwrite_id),
    .ID_MEM_SIZE            (mux_memsize_id),
    .ID_MEM_ENABLE          (mux_meme_id),
    .ID_AM                  (mux_idam_id),
    .STORE_CC               (mux_storecc_id),
    .RF_ENABLE              (mux_rfe_id),
    
    .BL_OUT                 (chandler_blout_idmux),
    .NEXT_PC                (if_npc_fetch),
    .MUX_PA                 (), // salida de los muxes de la etapa id/ex (inputs)
    .MUX_PB                 (), // salida de los muxes de la etapa id/ex (inputs)
    .MUX_PD                 (), // salida de los muxes de la etapa id/ex (inputs)
    .MUX_INSTR_I15_I12      (idmux_out_ex),
    .INSTR_I11_I0           (instr_i11_i0),

    .id_alu_op              (ex_aluop_alu),
    .id_load                (ex_load_mem),
    .id_mem_write           (ex_memwrite_mem),
    .id_mem_size            (ex_memsize_mem),
    .id_mem_enable          (ex_memenable_mem),
    .id_am                  (ex_am_shifter),
    .store_cc               (ex_storecc_psr),
    .rf_enable              (ex_rfenable_mem),
    .bl_out                 (ex_blout_muxalu),
    .next_pc                (ex_nextpc_muxalu),
    .mux_pa                 (ex_muxpa_alu),
    .mux_pb                 (ex_muxpb_shifter),
    .mux_pd                 (ex_muxpd_mem),
    .mux_instr_i15_i12      (ex_muxinstri15i12_memandhazard),
    .instr_i11_i0           (ex_instri11i0_shifter)
);


 ConditionHandler conditionhandler(
    .ID_BL_instr                (mux_bl_chandler),
    .ID_B_instr                 (mux_b_chandler),
    .Z                          (alu_z_chandler),
    .N                          (alu_n_chandler),
    .C                          (alu_c_chandler),
    .V                          (alu_v_chandler),
    .Condition                  (chandlermux_cc_chandler),
    .EX_BL_instr                (chandler_blout_idmux),
    .Branched                   (chandler_branch_mux)
 );

    MUX_ALU mux_alu (
        .alu_result  (alu_out_muxaluandidmuxes),
        .Next_PC     (ex_nextpc_muxalu),
        .BL_OUT      (ex_blout_muxalu),
        .DM_address  (alumux_dmaddress_mem)
    );

    ALU alu (
        .A          (ex_muxpa_alu),
        .B          (shifter_n_alu),
        .alu_op     (ex_aluop_alu),
        .C_IN       (psr_cin_alu),
        .result     (alu_out_muxaluandidmuxes),
        .Z          (alu_z_chandler),
        .N          (alu_n_chandler),
        .C          (alu_c_chandler),
        .V          (alu_v_chandler)
    );

    
    ARM_Shifter arm_shifter (
        .Rm (ex_muxpb_shifter),
        .I  (ex_instri11i0_shifter),
        .AM (ex_am_shifter),
        .N  (shifter_n_alu)
    );

    // HazardUnit hazardunit (
    //     .EX_RF_enable   (ex_rfenable_mem),
    //     .MEM_RF_enable  (),          
    //     .WB_RF_enable   (),
    //     .EX_Rd          (ex_muxpd_mem),
    //     .MEM_Rd         (),
    //     .WB_Rd          (),
    //     .ID_Rm          (instr_i3_i0),
    //     .ID_Rn          (instr_i19_i16),
    //     .ID_Rd          (instr_i15_i12),
    //     .EX_Load        (ex_load_mem),
    //     .ID_Store       (ex_storecc_psr),

    //     .PC_Enable      (enable_pc),
    //     .IF_IF_Enable   (enable_ifid),
    //     .foward_Rm      (foward_rm),
    //     .foward_Rn      (foward_rn),
    //     .foward_Rd      (foward_rd),
    //     .foward_Rg      (foward_rg),
    //     .NOP_EX         (nop)
    // );

    // EX_MEM ex_mem (
    //     .clk                    (),
    //     .reset                  (),
    //     .ID_LOAD                (),
    //     .ID_MEM_WRITE           (),
    //     .ID_MEM_SIZE            (),
    //     .ID_MEM_ENABLE          (),
    //     .RF_ENABLE              (),
    //     .MUX_PD                 (),
    //     .DM_ADDRESS             (),
    //     .MUX_INSTR_I15_I12      (),

    //     .id_load                (),
    //     .id_mem_size            (),
    //     .id_mem_write           (),
    //     .id_mem_enable          (),
    //     .rf_enable              (),
    //     .mux_pd                 (),
    //     .dm_address             (),
    //     .mux_instr_i15_i12      ()
    // );

    PSR psr (
        .STORE_CC           (ex_storecc_psr),
        .Z_in               (alu_z_chandler),
        .N_in               (alu_n_chandler),
        .C_in               (alu_c_chandler),
        .V_in               (alu_v_chandler),
        .Z_out              (psr_z_muxcc),
        .N_out              (psr_n_muxcc),
        .C_out              (psr_c_muxcc),
        .V_out              (psr_v_muxcc)
    );

    MUX_CC mux_cc (
        .Z                  (alu_z_chandler),
        .N                  (alu_n_chandler),
        .C                  (alu_c_chandler),
        .V                  (alu_v_chandler),
        .Flag_out           ({ psr_z_muxcc, psr_n_muxcc, psr_c_muxcc, psr_v_muxcc}),
        .ConditionCodes     (chandlermux_cc_chandler)
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

    // Instantiate the ControlUnit module
    ControlUnit uut_control (
        .instruction        (instruction),  // instruction
        .ALU_OP             (cu_idaluop_mux), //out
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

    // Instantiate the Multiplexer
    Multiplexer uut_mux (
        .ALU_OP         (cu_idaluop_mux),
        .ID_LOAD        (cu_idload_mux),
        .ID_MEM_WRITE   (cu_idmemwrite_mux),
        .STORE_CC       (cu_storecc_mux),
        .ID_BL          (cu_idbl_mux),
        .ID_B           (cu_idb_mux),
        .ID_MEM_SIZE    (cu_idmemsize_mux),
        .ID_MEM_E       (cu_meme_mux),
        .RF_E           (cu_rfe_mux),
        .ID_AM          (cu_idam_mux),
        .S              (hazard_cumuxenable_mux),
        .alu_op         (mux_aluop_id),
        .id_load        (mux_idload_id),
        .id_mem_write   (mux_memwrite_id),
        .store_cc       (mux_storecc_id),
        .id_bl          (mux_bl_chandler),
        .id_b           (mux_b_chandler),
        .id_mem_size    (mux_memsize_id),
        .id_mem_e       (mux_meme_id),
        .rf_e           (mux_rfe_id),
        .id_am          (mux_idam_id)
        
    );


    IF_ID if_id (
        .E(enable_ifid),
        .reset(reset),
        .clk(clk),
        .instr_in(instruction),
        .next_pc(pc),

        .instr_out(if_instruction),
        .instr_i23_i0(instr_i23_i0),    
        .Next_PC(if_npc_fetch),
        .instr_i3_i0(instr_i3_i0),
        .instr_i19_i16(instr_i19_i16),
        .instr_i31_i28(instr_i31_i28),
        .instr_i11_i0(instr_i11_i0),
        .instr_i15_i12(instr_i15_i12)
    );





    // Instantiate the instruction memory (ROM)
    Instruction_Memory_ROM rom_inst (
        .I(instruction),
        .A(pc[7:0]) // Connect the program counter to the memory address
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

    // Display outputs for each clock cycle
    always @(posedge clk) begin
        $display("PC: %0d | Opcode: %s", pc, get_keyword(instruction[24:21]));
        $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        $display("IF/ID");
        $display("Instruction           %b", if_instruction);
        $display("Register File:    RA: %b | RB: %b | RD: %b | PA: %b | PB: %b | PD %d", instr_i3_i0, instr_i19_i16,  instr_i15_i12, rf_registerpa_mux, rf_registerpb_mux, rf_registerpd_mux);
        $display("PC states:        PC: %b |nPC: %bPC | Fetch: %b", pc, npc, fetch_npc_pc);
        $display("Control Unit:     ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | STORE_CC: %b | ID_BL: %b | ID_B: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | RF_E: %b | ID_AM: %b", cu_idaluop_mux, cu_idload_mux, cu_idmemwrite_mux, cu_storecc_mux, cu_idbl_mux, cu_idb_mux, cu_idmemsize_mux, cu_meme_mux, cu_rfe_mux, cu_idam_mux);
        $display("Control Unit Mux: ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | STORE_CC: %b | ID_BL: %b | ID_B: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | RF_E: %b | ID_AM: %b", mux_aluop_id, mux_idload_id, mux_memwrite_id, mux_storecc_id, mux_bl_chandler, mux_b_chandler, mux_memsize_id, mux_meme_id, mux_rfe_id, mux_idam_id);
        $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        $display("ID/EX");
        $display("ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | ID_AM: %b | STORE_CC: %b | RF_E: %b | BL_Out: %b | Next PC: %b | MUX_PA: %b | MUX_PB: %b | MUX_PD: %b | MUX_15-12: %b | INSTR_11-0: %b", ex_aluop_alu, ex_load_mem, ex_memwrite_mem, ex_memsize_mem, ex_memenable_mem, ex_am_shifter, ex_storecc_psr, ex_rfenable_mem, ex_blout_muxalu, ex_nextpc_muxalu, ex_muxpa_alu, ex_muxpb_shifter, ex_muxpd_mem, ex_muxinstri15i12_memandhazard, ex_instri11i0_shifter);
        $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        $display("");
    end
endmodule
