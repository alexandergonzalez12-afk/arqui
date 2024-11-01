module ControlUnit(
    input wire [31:0] instruction,
    output reg [1:0] ID_SHIFT_am,
    output reg [3:0] ID_ALU_op,
    output reg ID_load_instr,
    output reg ID_RF_enable,
    output reg ID_DM_size,
    output reg ID_DM_rfw,
    output reg ID_DM_enable,
    output reg ID_DP_instr,
    output reg ID_B_instr,
    output reg ID_BL_instr
);
    always @(*) begin
        ID_SHIFT_am = 2'b00;
        ID_ALU_op = 4'b0000;
        ID_load_instr = 0;
        ID_RF_enable = 0;
        ID_DM_size = 0;
        ID_DM_rfw = 0;
        ID_DM_enable = 0;
        ID_DP_instr = 0;
        ID_B_instr = 0;
        ID_BL_instr = 0;

        case (instruction[31:28])
            4'b1110: begin
                case (instruction[27:24])
                    4'b0001: begin
                        ID_DP_instr = 1;
                        ID_RF_enable = 1;
                    end
                    4'b0000: begin
                        ID_ALU_op = 4'b0001;
                        ID_RF_enable = 1;
                    end
                    4'b0111: begin
                        ID_load_instr = 1;
                        ID_DM_enable = 1;
                    end
                    4'b0101: begin
                        ID_DM_rfw = 1;
                        ID_DM_enable = 1;
                    end
                endcase
            end
            4'b0001: ID_B_instr = 1;
            4'b1101: ID_BL_instr = 1;
            default: ;
        endcase
    end
endmodule
module Multiplexer(
    input wire sel, 
    input wire [3:0] control_ALU_op_in,
    input wire [1:0] control_SHIFT_am_in,
    input wire control_load_instr_in,
    input wire control_RF_enable_in,
    input wire control_DM_size_in,
    input wire control_DM_rfw_in,
    input wire control_DM_enable_in,
    input wire control_DP_instr_in,
    output reg [3:0] control_ALU_op_out,
    output reg [1:0] control_SHIFT_am_out,
    output reg control_load_instr_out,
    output reg control_RF_enable_out,
    output reg control_DM_size_out,
    output reg control_DM_rfw_out,
    output reg control_DM_enable_out,
    output reg control_DP_instr_out
);
    always @(*) begin
        if (sel) begin
            control_ALU_op_out = 4'b0000;
            control_SHIFT_am_out = 2'b00;
            control_load_instr_out = 0;
            control_RF_enable_out = 0;
            control_DM_size_out = 0;
            control_DM_rfw_out = 0;
            control_DM_enable_out = 0;
            control_DP_instr_out = 0;
        end else begin
            control_ALU_op_out = control_ALU_op_in;
            control_SHIFT_am_out = control_SHIFT_am_in;
            control_load_instr_out = control_load_instr_in;
            control_RF_enable_out = control_RF_enable_in;
            control_DM_size_out = control_DM_size_in;
            control_DM_rfw_out = control_DM_rfw_in;
            control_DM_enable_out = control_DM_enable_in;
            control_DP_instr_out = control_DP_instr_in;
        end
    end
endmodule

module PC(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire [31:0] pc_in,
    output reg [31:0] pc_out
);
    always @(posedge clk) begin
        if (reset) 
            pc_out <= 32'b0;
        else if (enable)
            pc_out <= pc_in;
    end
endmodule

module adder(
    input wire [31:0] pc_in,
    output wire [31:0] pc_out
);
    assign pc_out = pc_in + 32'd4;
endmodule

module  Instruction_Memory_ROM (
    input [7:0] A,
    output reg [31:0] I
);
    reg [7:0] Mem [0:255];
    always @(A) begin
        I = {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};
    end
endmodule





module IF_ID(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire [31:0] instruction_in,
    input wire [31:0] pc_in,
    output reg [31:0] instruction_out,
    output reg [31:0] pc_out
);
    always @(posedge clk) begin
        if (reset) begin
            instruction_out <= 32'b0;
            pc_out <= 32'b0;
        end else if (enable) begin
            instruction_out <= instruction_in;
            pc_out <= pc_in;
        end
    end
endmodule

module ID_EX(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire [31:0] pc_in,
    input wire [1:0] ID_SHIFT_am,
    input wire [3:0] ID_ALU_op,
    input wire ID_load_instr,
    input wire ID_RF_enable,
    input wire ID_DM_size,
    input wire ID_DM_rfw,
    input wire ID_DM_enable,
    input wire ID_DP_instr,
    input wire ID_B_instr,
    output reg [31:0] pc_out,
    output reg [1:0] EX_SHIFT_am,
    output reg [3:0] EX_ALU_op,
    output reg EX_load_instr,
    output reg EX_RF_enable,
    output reg EX_DM_size,
    output reg EX_DM_rfw,
    output reg EX_DM_enable,
    output reg EX_DP_instr,
    output reg EX_B_instr
);
    always @(posedge clk) begin
        if (reset) begin
            pc_out <= 32'b0;
            EX_SHIFT_am <= 2'b00;
            EX_ALU_op <= 4'b0000;
            EX_load_instr <= 0;
            EX_RF_enable <= 0;
            EX_DM_size <= 0;
            EX_DM_rfw <= 0;
            EX_DM_enable <= 0;
            EX_DP_instr <= 0;
            EX_B_instr <= 0;
        end else if (enable) begin
            pc_out <= pc_in;
            EX_SHIFT_am <= ID_SHIFT_am;
            EX_ALU_op <= ID_ALU_op;
            EX_load_instr <= ID_load_instr;
            EX_RF_enable <= ID_RF_enable;
            EX_DM_size <= ID_DM_size;
            EX_DM_rfw <= ID_DM_rfw;
            EX_DM_enable <= ID_DM_enable;
            EX_DP_instr <= ID_DP_instr;
            EX_B_instr <= ID_B_instr;
        end
    end
endmodule

module EX_MEM(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire [31:0] pc_in,
    input wire EX_load_instr,
    input wire EX_RF_enable,
    input wire EX_DM_size,
    input wire EX_DM_rfw,
    input wire EX_DM_enable,
    output reg [31:0] pc_out,
    output reg MEM_load_instr,
    output reg MEM_RF_enable,
    output reg MEM_DM_size,
    output reg MEM_DM_rfw,
    output reg MEM_DM_enable
);
    always @(posedge clk) begin
        if (reset) begin
            pc_out <= 32'b0;
            MEM_load_instr <= 0;
            MEM_RF_enable <= 0;
            MEM_DM_size <= 0;
            MEM_DM_rfw <= 0;
            MEM_DM_enable <= 0;
        end else if (enable) begin
            pc_out <= pc_in;
            MEM_load_instr <= EX_load_instr;
            MEM_RF_enable <= EX_RF_enable;
            MEM_DM_size <= EX_DM_size;
            MEM_DM_rfw <= EX_DM_rfw;
            MEM_DM_enable <= EX_DM_enable;
        end
    end
endmodule

module MEM_WB(
    input wire clk,
    input wire reset,
    input wire enable,
    input wire MEM_RF_enable,
    output reg WB_RF_enable
);
    always @(posedge clk) begin
        if (reset) begin
            WB_RF_enable <= 0;
        end else if (enable) begin
            WB_RF_enable <= MEM_RF_enable;
        end
    end
endmodule

