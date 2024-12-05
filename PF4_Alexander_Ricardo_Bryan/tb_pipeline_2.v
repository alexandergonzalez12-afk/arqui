`timescale 1ns / 1ps
`include "Pipeline_Modules.v"


module tb_pipeline;

    // Inputs
    reg clk;
    reg reset;
    wire enable_pc;
    wire enable_ifid;
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

    reg [3:0] Ra,Rb,Rd;


    wire [3:0] wb_registerrw_rf;
    wire wb_registerle_rf;
    wire [31:0] wb_registerpw_rf;
    
    wire [31:0] rf_registerpa_mux;
    wire [31:0] rf_registerpb_mux;
    wire [31:0] rf_registerpd_mux;

    // ====================================
    // Muxes or something
    // ====================================

    wire  [31:0]  mux_pa_id;
    wire  [31:0]  mux_pb_id;
    wire  [31:0]  mux_pd_id;

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

    wire nop;
    wire [1:0] forward_rm;
    wire [1:0] forward_rn;
    wire [1:0] forward_rd;
    wire [1:0] forward_rg;

    // ====================================
    // EX/MEM
    // ====================================

    wire [31:0]  mem_pd_inputdm;
    wire [31:0]  mem_address_dmandmux;
    wire [3:0]   mem_muxi15i12_wb;

    // ====================================
    // DATA MEMORY
    // ====================================

    wire [31:0] dm_output_muxdm;
    wire mem_size_dm;

    // ====================================
    // RF DATA MEMORY
    // ====================================

    wire [31:0] muxdatamemory_wb;

    // ====================================
    // Finished wiring
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




    MUX_PA mux_pa (
        .pa             (rf_registerpa_mux),
        .jump_EX_pa     (alu_out_muxaluandidmuxes),
        .jump_MEM_pa    (muxdatamemory_wb),
        .jump_WB_pa     (wb_registerpw_rf),
        .S_PA           (forward_rn),
        .rf_pa          (mux_pa_id)
    );

    MUX_PB mux_pb (
        .pb             (rf_registerpb_mux),
        .jump_EX_pb     (alu_out_muxaluandidmuxes),
        .jump_MEM_pb    (muxdatamemory_wb),
        .jump_WB_pb     (wb_registerpw_rf),
        .S_PB           (forward_rm),
        .rf_pb          (mux_pb_id)
    );

    MUX_PD mux_pd (
        .pd             (rf_registerpd_mux),
        .jump_EX_pd     (alu_out_muxaluandidmuxes),
        .jump_MEM_pd    (muxdatamemory_wb),
        .jump_WB_pd     (wb_registerpw_rf),
        .S_PD           (forward_rg),
        .rf_pd          (mux_pd_id)
    );    


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
    .MUX_PA                 (mux_pa_id), 
    .MUX_PB                 (mux_pb_id), 
    .MUX_PD                 (mux_pd_id), 
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

    HazardUnit hazardunit (
        .EX_RF_enable   (ex_rfenable_mem),
        .MEM_RF_enable  (mem_rfenable_wb),          
        .WB_RF_enable   (wb_registerle_rf),
        .EX_Rd          (alu_out_muxaluandidmuxes[15:12]),
        .MEM_Rd         (muxdatamemory_wb[15:12]),
        .WB_Rd          (wb_registerrw_rf[15:12]),
        .ID_Rm          (instr_i3_i0),
        .ID_Rn          (instr_i19_i16),
        .ID_Rd          (instr_i15_i12),
        .EX_Load        (ex_load_mem),
        .ID_Store       (ex_storecc_psr),

        .PC_Enable      (enable_pc),
        .IF_IF_Enable   (enable_ifid),
        .forward_Rm     (forward_rm),
        .forward_Rn     (forward_rn),
        //.forward_Rd     (forward_rd),
        .forward_Rg     (forward_rg),
        .NOP_EX         (nop)
    );

    EX_MEM ex_mem (
        .clk                    (clk),
        .reset                  (reset),
        .ID_LOAD                (ex_load_mem),
        .ID_MEM_WRITE           (ex_memwrite_mem),
        .ID_MEM_SIZE            (ex_memsize_mem),
        .ID_MEM_ENABLE          (ex_memenable_mem),
        .RF_ENABLE              (ex_rfenable_mem),
        .MUX_PD                 (ex_muxpd_mem),
        .DM_ADDRESS             (alumux_dmaddress_mem),
        .MUX_INSTR_I15_I12      (ex_muxinstri15i12_memandhazard),

        .id_load                (mem_load_wb),
        .id_mem_size            (mem_size_dm),
        .id_mem_write           (mem_write_dm),
        .id_mem_enable          (mem_enable_dm),
        .rf_enable              (mem_rfenable_wb),
        .mux_pd                 (mem_pd_inputdm),
        .dm_address             (mem_address_dmandmux),
        .mux_instr_i15_i12      (mem_muxi15i12_wb)
    );

    Data_Memory_RAM ram_inst(
        .data_out       (dm_output_muxdm),
        .address        (mem_address_dmandmux[7:0]),
        .data_in        (mem_pd_inputdm),
        .size           (mem_size_dm),
        .rw             (mem_write_dm),
        .enable         (mem_enable_dm)

    );

    MUX_DataMemory mux_datamemory(
        .Addr       (mem_address_dmandmux),
        .DataOut    (dm_output_muxdm),
        .Sel        (mem_load_wb),
        .MuxOut     (muxdatamemory_wb)     
    );

    MEM_WB mem_wb(
        .clk                (clk),
        .reset              (reset),
        .RF_ENABLE          (mem_rfenable_wb),
        .MUX_DATAMEMORY     (muxdatamemory_wb),
        .MUX_INSTR_I15_I12  (mem_muxi15i12_wb),
        .rf_enable          (wb_registerle_rf),
        .mux_instr_i15_i12  (wb_registerrw_rf),
        .mux_datamemory     (wb_registerpw_rf)
    );

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
    
    MUX_RFenable mux_rfenable(
        .id_rf_e            (cu_rfe_mux),
        .s_rfenable         (chandler_blout_idmux),
        .out_rf_enable      (mux_rfenable_cumux)  
    );

    Three_port_register_file tprf (
        .RA   (instr_i3_i0),
        .RB   (instr_i19_i16),
        .RD   (instr_i15_i12),
        .RW   (wb_registerrw_rf),
        .PC   (pc),
        .Clk  (clk),
        .PW   (wb_registerpw_rf),
        .LE   (wb_registerle_rf),
        .PA   (rf_registerpa_mux),
        .PB   (rf_registerpb_mux),
        .PD   (rf_registerpd_mux)
    );

    // Instantiate the ControlUnit module
    ControlUnit uut_control (
        .instruction        (if_instruction),  // instruction
        .ID_ALU_op          (cu_idaluop_mux), //out
        .ID_load_instr      (cu_idload_mux),
        .ID_DM_rfw          (cu_idmemwrite_mux),
        .ID_DP_instr        (cu_storecc_mux),
        .ID_DM_size         (cu_idmemsize_mux),
        .ID_DM_enable       (cu_idmeme_mux),
        .ID_RF_enable       (cu_rfe_rfmux),
        .ID_SHIFT_am        (cu_idam_mux),
        .ID_B_instr         (cu_idb_mux),
        .ID_BL_instr        (cu_idbl_mux)
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
        .ID_MEM_E       (cu_idmeme_mux),
        .RF_E           (cu_rfe_rfmux),
        .ID_AM          (cu_idam_mux),
        .S              (chandler_blout_idmux),
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
            ram_inst.Mem[address] = data; // Preload the RAM memory
            address = address + 1;
        end
        $fclose(fi);
    end

    // Test sequence with enforced stop time at 40
    initial begin
        // Initialize signals
        reset = 1;
        //enable_pc = 1;
        //enable_ifid = 1;
        //S = 0;
        // Start simulation
        #3 reset = 0;
        //#32 S = 1;
    end

    initial begin
        $monitor("CLK: %b | Reset: %b | PC: %d | r1: %d | r2: %d | r3: %d | r5: %d | r6: %d", clk, reset, pc, tprf.R1, tprf.R2, tprf.R3, tprf.R5, tprf.R6);
        #60 $finish; // Stop simulation at time 52
    end
    // Display outputs for each clock cycle
    //always @(posedge clk) begin
        // $display("PC: %d | Instruction Type: %s", pc, get_keyword(instruction[24:21]));
        // $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        // $display("IF/ID");
        // $display("Instruction           %b", if_instruction);
        // $display("Register File:    RA: %b | RB: %b | RD: %b | RW: %b | PA: %b | PB: %b | PD %b", instr_i3_i0, instr_i19_i16,  instr_i15_i12, wb_registerrw_rf, rf_registerpa_mux, rf_registerpb_mux, rf_registerpd_mux);
        // $display("=====================================================================================================================================================================");
        // $display("ALU\nA: %b | B: %b | alu_op: %b | C_IN: %b | result: %b | Z: %b | N: %b | C: %b | V: %b", ex_muxpa_alu, shifter_n_alu, ex_aluop_alu, psr_cin_alu, alu_out_muxaluandidmuxes, alu_z_chandler, alu_n_chandler, alu_c_chandler, alu_v_chandler);
        // $display("PC states:        PC: %b |nPC: %bPC | Fetch: %b", pc, npc, fetch_npc_pc);
        // $display("Control Unit:     ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | STORE_CC: %b | ID_BL: %b | ID_B: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | RF_E: %b | ID_AM: %b", cu_idaluop_mux, cu_idload_mux, cu_idmemwrite_mux, cu_storecc_mux, cu_idbl_mux, cu_idb_mux, cu_idmemsize_mux, cu_idmeme_mux, cu_rfe_rfmux, cu_idam_mux);
        // $display("Control Unit Mux: ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | STORE_CC: %b | ID_BL: %b | ID_B: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | RF_E: %b | ID_AM: %b", mux_aluop_id, mux_idload_id, mux_memwrite_id, mux_storecc_id, mux_bl_chandler, mux_b_chandler, mux_memsize_id, mux_meme_id, mux_rfe_id, mux_idam_id);
        // $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        // $display("ID/EX");
        // $display("ALU_OP: %b | ID_LOAD: %b | ID_MEM_WRITE: %b | ID_MEM_SIZE: %b | ID_MEM_E: %b | ID_AM: %b | STORE_CC: %b | RF_E: %b | BL_Out: %b | Next PC: %b | MUX_PA: %b | MUX_PB: %b | MUX_PD: %b | MUX_15-12: %b | INSTR_11-0: %b", ex_aluop_alu, ex_load_mem, ex_memwrite_mem, ex_memsize_mem, ex_memenable_mem, ex_am_shifter, ex_storecc_psr, ex_rfenable_mem, ex_blout_muxalu, ex_nextpc_muxalu, ex_muxpa_alu, ex_muxpb_shifter, ex_muxpd_mem, ex_muxinstri15i12_memandhazard, ex_instri11i0_shifter);
        // $display("LE: %b | PW: %b", wb_registerpw_rf, wb_registerle_rf);
        // $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        // $display("EX/MEM");
        // $display("id_load: %b | id_mem_size: %b | id_mem_write: %b | id_mem_enable: %b | rf_enable: %b | mux_pd: %b | dm_address: %b | mux_instr_i15_i12:  %b", mem_load_wb, mem_size_dm, mem_write_dm, mem_enable_dm, mem_rfenable_wb, mem_pd_inputdm, mem_address_dmandmux, mem_muxi15i12_wb);
        // $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        // $display("MEM/WB");
        // $display("RF_ENABLE: %b | MUX_DATAMEMORY: %b | MUX_INSTR_I15_I12: %b | rf_enable: %b | mux_instr_i15_i12: %b | mux_datamemory: %b",  mem_rfenable_wb, muxdatamemory_wb, mem_muxi15i12_wb, wb_registerle_rf, wb_registerrw_rf, wb_registerpw_rf);
        // $display("-=-=-=-=-=-=--=-=-=-=--=-=-=-=-=-=-=-=--=-=-=-=--=-=--=-==-=-=-=-=-=-=--==-==-=-==-=-==-=-==-=-");
        // $display("Register File memmory -=-=-=-=-=-=-==-=-=");
        // $display("Selectv O : %d", tprf.O);
        // $display("R0: %d | A0", tprf.R0);
        // $display("R1: %d | A1", tprf.R1);
        // $display("R2: %d | A2", tprf.R2);
        // $display("R3: %d | A3", tprf.R3);
        // $display("R4: %d | A4", tprf.R4);
        // $display("R5: %d | A5", tprf.R5);
        // $display("R6: %d | A6", tprf.R6);
        // $display("R7: %d | A7", tprf.R7);
        // $display("R8: %d | A8", tprf.R8);
        // $display("R9: %d | A9", tprf.R9);
        // $display("R10: %d | A10", tprf.R10);
        // $display("R11: %d | A11", tprf.R11);
        // $display("R12: %d | A12", tprf.R12);
        // $display("R13: %d | A13", tprf.R13);
        // $display("R14: %d | A14", tprf.R14);
        // $display("R15: %d | A15", tprf.R15);


        // $display("------------------------------------------------------------------------------------------------------------------------------------------------------");
        

    //end
endmodule
