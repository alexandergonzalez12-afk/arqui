`timescale 1ns / 1ps

module tb_pipeline;

     reg clk;
    reg reset;
    reg enable_pc;
    reg enable_if_id;
    reg enable_id_ex;
    reg enable_ex_mem;
    reg enable_mem_wb;
    reg sel_mux;
    wire [31:0] instr_from_mem;
    wire [31:0] pc_out;

    // Control signals for each stage (after mux application)
    wire [3:0] ID_ALU_op, mux_ALU_op;
    wire [1:0] ID_SHIFT_am, mux_SHIFT_am;
    wire ID_load_instr, ID_RF_enable, ID_DM_size, ID_DM_rfw, ID_DM_enable, ID_DP_instr;
    wire mux_load_instr, mux_RF_enable, mux_DM_size, mux_DM_rfw, mux_DM_enable, mux_DP_instr;
    wire [1:0] EX_SHIFT_am;
    wire [3:0] EX_ALU_op;
    wire EX_load_instr, EX_RF_enable;
    wire EX_DM_size, EX_DM_rfw, EX_DM_enable, EX_DP_instr;
    wire MEM_load_instr, MEM_RF_enable, MEM_DM_size, MEM_DM_rfw, MEM_DM_enable;
    wire WB_RF_enable;

    integer file;
    integer cycle;
    reg [7*8:0] instruction_name;
    reg [7:0] byte0, byte1, byte2, byte3;  
    wire [31:0] IF_ID_instruction, IF_ID_pc_out;
    wire [31:0] ID_EX_pc_out, EX_MEM_pc_out;
    integer i;

    wire [31:0] pc_incremented;
    PC pc (
        .clk(clk),
        .reset(reset),
        .enable(enable_pc),
        .pc_in(pc_incremented),
        .pc_out(pc_out)
    );


    adder adder (
        .pc_in(pc_out),
        .pc_out(pc_incremented)
    );

    Instruction_Memory_ROM mem_inst (
        .A(pc_out[7:0]),
        .I(instr_from_mem)
    );

    ControlUnit control_unit_inst (
        .instruction(IF_ID_instruction),
        .ID_SHIFT_am(ID_SHIFT_am),
        .ID_ALU_op(ID_ALU_op),
        .ID_load_instr(ID_load_instr),
        .ID_RF_enable(ID_RF_enable),
        .ID_DM_size(ID_DM_size),
        .ID_DM_rfw(ID_DM_rfw),
        .ID_DM_enable(ID_DM_enable),
        .ID_DP_instr(ID_DP_instr),
        .ID_B_instr(ID_B_instr),
        .ID_BL_instr(ID_BL_instr)
    );


    Multiplexer mux (
        .sel(sel_mux),
        .control_ALU_op_in(ID_ALU_op),
        .control_SHIFT_am_in(ID_SHIFT_am),
        .control_load_instr_in(ID_load_instr),
        .control_RF_enable_in(ID_RF_enable),
        .control_DM_size_in(ID_DM_size),
        .control_DM_rfw_in(ID_DM_rfw),
        .control_DM_enable_in(ID_DM_enable),
        .control_DP_instr_in(ID_DP_instr),
        .control_ALU_op_out(mux_ALU_op),
        .control_SHIFT_am_out(mux_SHIFT_am),
        .control_load_instr_out(mux_load_instr),
        .control_RF_enable_out(mux_RF_enable),
        .control_DM_size_out(mux_DM_size),
        .control_DM_rfw_out(mux_DM_rfw),
        .control_DM_enable_out(mux_DM_enable),
        .control_DP_instr_out(mux_DP_instr)
    );


    IF_ID if_id_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable_if_id),
        .instruction_in(instr_from_mem),
        .pc_in(pc_out),
        .instruction_out(IF_ID_instruction),
        .pc_out(IF_ID_pc_out)
    );

    ID_EX id_ex_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable_id_ex),
        .pc_in(IF_ID_pc_out),
        .ID_SHIFT_am(mux_SHIFT_am),
        .ID_ALU_op(mux_ALU_op),
        .ID_load_instr(mux_load_instr),
        .ID_RF_enable(mux_RF_enable),
        .ID_DM_size(mux_DM_size),
        .ID_DM_rfw(mux_DM_rfw),
        .ID_DM_enable(mux_DM_enable),
        .ID_DP_instr(mux_DP_instr),
        .pc_out(ID_EX_pc_out),
        .EX_SHIFT_am(EX_SHIFT_am),
        .EX_ALU_op(EX_ALU_op),
        .EX_load_instr(EX_load_instr),
        .EX_RF_enable(EX_RF_enable),
        .EX_DM_size(EX_DM_size),
        .EX_DM_rfw(EX_DM_rfw),
        .EX_DM_enable(EX_DM_enable),
        .EX_DP_instr(EX_DP_instr)
    );

    EX_MEM ex_mem_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable_ex_mem),
        .pc_in(ID_EX_pc_out),
        .EX_load_instr(EX_load_instr),
        .EX_RF_enable(EX_RF_enable),
        .EX_DM_size(EX_DM_size),
        .EX_DM_rfw(EX_DM_rfw),
        .EX_DM_enable(EX_DM_enable),
        .pc_out(EX_MEM_pc_out),
        .MEM_load_instr(MEM_load_instr),
        .MEM_RF_enable(MEM_RF_enable),
        .MEM_DM_size(MEM_DM_size),
        .MEM_DM_rfw(MEM_DM_rfw),
        .MEM_DM_enable(MEM_DM_enable)
    );

    MEM_WB mem_wb_reg (
        .clk(clk),
        .reset(reset),
        .enable(enable_mem_wb),
        .MEM_RF_enable(MEM_RF_enable),
        .WB_RF_enable(WB_RF_enable)
    );

    always #2 clk = ~clk;

    initial begin
        clk = 0; reset = 1; enable_pc = 1; enable_if_id = 1;
        enable_id_ex = 1; enable_ex_mem = 1; enable_mem_wb = 1; sel_mux = 0;

        #3 reset = 0; 
        #210 $finish;  
    end

    initial begin
        i = 0;
        file = $fopen("codigo_validacion.txt", "r"); 
        if (file == 0) begin
            $display("Error: I cant open the file, please try again.");
            $finish;
        end

        while (!$feof(file)) begin
            if ($fscanf(file, "%8b %8b %8b %8b\n", byte0, byte1, byte2, byte3) == 4) begin
                mem_inst.Mem[i] = byte0;
                mem_inst.Mem[i+1] = byte1;
                mem_inst.Mem[i+2] = byte2;
                mem_inst.Mem[i+3] = byte3;
                i = i + 4;
            end else begin
                $display("didnt read file correctly.");
                $finish;
            end
        end
        $fclose(file);
    end

 always @(posedge clk) begin
    if (!reset) begin
        cycle = pc_out >> 2;

        if (pc_out >= i + 4) begin
            $display("Done.");
            $finish;
        end

        case (IF_ID_instruction[31:28])
            4'b1110: begin
                case (IF_ID_instruction[27:24])
                    4'b0001: instruction_name = "ANDS";
                    4'b0000: instruction_name = "AND";
                    4'b0111: instruction_name = "LDRB";
                    4'b0101: instruction_name = "STR";
                    default: instruction_name = "UNKNOWN";
                endcase
            end
            4'b0001: instruction_name = "BNE";
            4'b1101: instruction_name = "BLLE";
            default: instruction_name = "NOP";
        endcase

        $display("--------------------------------------------------------------");
        $display("Cycle: %0d | Time: %0t", cycle, $time);
        $display("PC: %0d | Opcode: %-7s", pc_out, instruction_name);
        $display("--------------------------------------------------------------");
        
        // Display instruction and control signals in a structured format
        $display("Instruction: %b", IF_ID_instruction);
        
        // ID Stage Control Signals
        $display("Control Signals (ID):     ALU_OP: %b | SHIFT_am: %b | Load: %b | RF_E: %b | DM_size: %b | DM_rfw: %b | DM_enable: %b | DP_instr: %b",
                 ID_ALU_op, ID_SHIFT_am, ID_load_instr, ID_RF_enable, ID_DM_size, ID_DM_rfw, ID_DM_enable, ID_DP_instr);
        
        // EX Stage Control Signals
        $display("Control Signals (EX):     ALU_OP: %b | SHIFT_am: %b | Load: %b | RF_E: %b | DM_size: %b | DM_rfw: %b | DM_enable: %b | DP_instr: %b",
                 EX_ALU_op, EX_SHIFT_am, EX_load_instr, EX_RF_enable, EX_DM_size, EX_DM_rfw, EX_DM_enable, EX_DP_instr);
        
        // MEM Stage Control Signals
        $display("Control Signals (MEM):    Load: %b | RF_E: %b | Mem Size: %b | RW: %b",
                 MEM_load_instr, MEM_RF_enable, MEM_DM_size, MEM_DM_rfw);
        
        // WB Stage Control Signal
        $display("Control Signals (WB):     RF_E: %b", WB_RF_enable);
        
        $display("--------------------------------------------------------------\n");
    end
end



endmodule
