`timescale 1ns / 1ps
`include "Modules.v"


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

    wire [3:0] ID_alu_op;
    wire  ID_load_instr;
    wire  ID_RW;
    wire  ID_S_bit;
    wire  ID_size;
    wire  ID_enable_instr;
    wire  ID_RF_enable;
    wire [1:0] ID_shift_AM;
    wire  ID_B_instr;
    wire  ID_BL_instr;
    wire [7:0] ID_mnemonic0;
    wire [7:0] ID_mnemonic1;
    wire [7:0] ID_mnemonic2;
    wire [1:0] sop_count;

    wire [7:0] id_mnemonic0;
    wire [7:0] id_mnemonic1;
    wire [7:0] id_mnemonic2;
    wire  Sop_count;

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
    wire sig_s;
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
    wire [31:0] muxxpa;


    // ====================================
    // Condition handler
    // ====================================
    wire [3:0] CC;
    wire [3:0] cc;
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
    wire wb_load;
    integer loop_c, PROG_SIZE, file_size ;
    reg [7:0] Address;

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
    .ID_S                   (sig_s),
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
    .id_s                   (ex_s),
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
    .ConditionCode              (CC),
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
        .Z          (CC[2]),
        .N          (CC[3]),
        .C          (CC[1]),
        .V          (CC[0])
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
        .ID_Load        (mux_idload_id),
        .ID_Enable      (mux_meme_id),
        .sop_count      (Sop_count),

        .PC_Enable      (enable_pc),
        .IF_IF_Enable   (enable_ifid),
        .forward_Rm     (forward_rm),
        .forward_Rn     (forward_rn),
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
        .MUX_PA                 (ex_muxpa_alu),
        .MUX_PD                 (ex_muxpd_mem),
        .DM_ADDRESS             (alumux_dmaddress_mem),
        .MUX_INSTR_I15_I12      (ex_muxinstri15i12_memandhazard),

        .id_load                (mem_load_wb),
        .id_mem_size            (mem_size_dm),
        .id_mem_write           (mem_write_dm),
        .id_mem_enable          (mem_enable_dm),
        .rf_enable              (mem_rfenable_wb),
        .mux_pa                 (muxxpa),
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
        .ID_LOAD            (mem_load_wb),
        .MUX_DATAMEMORY     (muxdatamemory_wb),
        .MUX_INSTR_I15_I12  (mem_muxi15i12_wb),
        .rf_enable          (wb_registerle_rf),
        .id_load            (wb_load),
        .mux_instr_i15_i12  (wb_registerrw_rf),
        .mux_datamemory     (wb_registerpw_rf)
    );

    PSR psr (
        .clk                (clk),
        .ConditionCode      (CC),
        .PSR_ConditionCode  (cc),
        .C_in               (psr_cin_alu)
    );

    MUX_CC mux_cc (
        .ConditionCode      (CC),
        .Flag_out           (cc),
        .SIG_s              (ex_s),
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
        .ID_alu_op          (cu_idaluop_mux), //out
        .ID_load_instr      (cu_idload_mux),
        .ID_RW              (cu_idmemwrite_mux),
        .ID_S_bit           (cu_ids_mux),
        .ID_size            (cu_idmemsize_mux),
        .ID_enable_instr    (cu_idmeme_mux),
        .ID_RF_enable       (cu_rfe_rfmux),
        .ID_shift_AM        (cu_idam_mux),
        .ID_B_instr         (cu_idb_mux),
        .ID_BL_instr        (cu_idbl_mux),
        .ID_mnemonic0       (id_mnemonic0),
        .ID_mnemonic1       (id_mnemonic1),
        .ID_mnemonic2       (id_mnemonic2),
        .sop_count          (sop_count)
    );

    // Instantiate the Multiplexer
    Multiplexer uut_mux (
        .ALU_OP         (cu_idaluop_mux),
        .ID_LOAD        (cu_idload_mux),
        .ID_MEM_WRITE   (cu_idmemwrite_mux),
        .ID_S           (cu_ids_mux),
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
        .id_s           (sig_s),
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

     initial begin
        fi = $fopen("codigo_validacion.txt", "r");
        if (fi == 0) begin
            $display("Error: Cannot open file.");
            $finish;
        end

        PROG_SIZE = 0;

        // Read file line by line to determine file size and load ROM
        while (!$feof(fi)) begin
            if ($fscanf(fi, "%b", data) == 1) begin
                rom_inst.mem[PROG_SIZE] = data;
                PROG_SIZE = PROG_SIZE + 1;
            end
        end
        

        $display("Program load %0d instructions.", PROG_SIZE);
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
            ram_inst.mem[Address] = data;  
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
        $monitor("Time: %0d | PC: %d | r1: %d | r2: %d | r3: %d | r5: %d | r6: %d", $time, pc, tprf.R1, tprf.R2, tprf.R3, tprf.R5, tprf.R6);
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
        pc_history[pc_count] = pc;
        pc_count = pc_count +1;
        if(pc_count==11) pc_count =0;
        for(i = 0;i <= 10; i= i+1) begin
            if(pc==pc_history[i]) cnt = cnt+1;
        end
        if(cnt>=11) begin
            $display("Infinite Loop Detected");
           for (i = 0; i < 256; i = i + 4) begin
        // Check if at least one value in the current block is valid
        if ((ram_inst.mem[i] !== 8'bx) || (ram_inst.mem[i+1] !== 8'bx) || 
            (ram_inst.mem[i+2] !== 8'bx) || (ram_inst.mem[i+3] !== 8'bx)) begin
            $display("RAM[%0d:%0d] = %b %b %b %b", 
                     i, i+3, 
                     ram_inst.mem[i], ram_inst.mem[i+1], ram_inst.mem[i+2], ram_inst.mem[i+3]);

        end
    end



            $finish;
        end else begin
            cnt = 0;
        end
    end
endmodule