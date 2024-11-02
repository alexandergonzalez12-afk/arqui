module adder(output reg [31:0] NextPC, input [31:0] PC);
always @(*) begin
    NextPC = PC + 4; 
end
endmodule


module NextProgramCounter (
    output reg [31:0] next_pc, 
    input [31:0] current_pc    
);
always @(*) begin
    next_pc = current_pc + 4; 
end
endmodule


module ROM256x8 (
    input [7:0] address,           
    output reg [31:0] instruction  
);

    reg [7:0] mem [0:255]; 

    always @(*) begin
        instruction = {mem[address], mem[address + 1], mem[address + 2], mem[address + 3]}; 
    end
endmodule


module control_unit (
    output reg ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr,
    ID_load_store_instr, ID_size, ID_BL_instr,
    output reg [1:0] ID_shift_AM,
    output reg [3:0] ID_alu_op,
    output reg [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
    input [31:0] instruction
);

always @(instruction) begin
    if (instruction == 32'b00000000000000000000000000000000) begin
        ID_S_bit = 0;
        ID_load_instr = 0;
        ID_RF_enable = 0;
        ID_B_instr = 0;
        ID_load_store_instr = 0;
        ID_size = 0;
        ID_BL_instr = 0;
        ID_shift_AM = 2'b00;
        ID_alu_op = 4'b0000;
        ID_mnemonic0 = "N";
        ID_mnemonic1 = "O";
        ID_mnemonic2 = "P";
    end else begin
        case (instruction[27:25])
            3'b000: begin  
                if (instruction[4] == 0) begin
                    ID_S_bit = instruction[20];
                    ID_load_instr = 0;
                    ID_RF_enable = 1; 
                    ID_B_instr = 0;
                    ID_load_store_instr = 0;
                    ID_size = 0; 
                    ID_BL_instr = 0;
                    ID_shift_AM = 2'b11;
                    case (instruction[24:21])
                        4'b0000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "A"; ID_mnemonic1 = "N"; ID_mnemonic2 = "D"; end
                        4'b0001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "E"; ID_mnemonic1 = "O"; ID_mnemonic2 = "R"; end
                        4'b0010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "S"; ID_mnemonic1 = "U"; ID_mnemonic2 = "B"; end
                        4'b0011: begin ID_alu_op = 4'b0100; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "B"; end
                        4'b0100: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "D"; end
                        4'b0101: begin ID_alu_op = 4'b0001; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "C"; end
                        4'b0110: begin ID_alu_op = 4'b0011; ID_mnemonic0 = "S"; ID_mnemonic1 = "B"; ID_mnemonic2 = "C"; end
                        4'b0111: begin ID_alu_op = 4'b0101; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "C"; end
                        4'b1000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "T"; ID_mnemonic1 = "S"; ID_mnemonic2 = "T"; end
                        4'b1001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "T"; ID_mnemonic1 = "E"; ID_mnemonic2 = "Q"; end
                        4'b1010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "P"; end
                        4'b1011: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "N"; end
                        4'b1100: begin ID_alu_op = 4'b0111; ID_mnemonic0 = "O"; ID_mnemonic1 = "R"; ID_mnemonic2 = "R"; end
                        4'b1101: begin ID_alu_op = 4'b1010; ID_mnemonic0 = "M"; ID_mnemonic1 = "O"; ID_mnemonic2 = "V"; end
                        4'b1110: begin ID_alu_op = 4'b1100; ID_mnemonic0 = "B"; ID_mnemonic1 = "I"; ID_mnemonic2 = "C"; end
                        4'b1111: begin ID_alu_op = 4'b1011; ID_mnemonic0 = "M"; ID_mnemonic1 = "V"; ID_mnemonic2 = "N"; end
                    endcase
                end
            end
            3'b001: begin 
                ID_S_bit = instruction[20];
                ID_load_instr = 0;
                ID_RF_enable = 1;
                ID_B_instr = 0;
                ID_load_store_instr = 0;
                ID_size = 0; 
                ID_BL_instr = 0;
                ID_shift_AM = 2'b00;
                case (instruction[24:21])
                    4'b0000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "A"; ID_mnemonic1 = "N"; ID_mnemonic2 = "D"; end
                    4'b0001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "E"; ID_mnemonic1 = "O"; ID_mnemonic2 = "R"; end
                    4'b0010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "S"; ID_mnemonic1 = "U"; ID_mnemonic2 = "B"; end
                    4'b0011: begin ID_alu_op = 4'b0100; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "B"; end
                    4'b0100: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "D"; end
                    4'b0101: begin ID_alu_op = 4'b0001; ID_mnemonic0 = "A"; ID_mnemonic1 = "D"; ID_mnemonic2 = "C"; end
                    4'b0110: begin ID_alu_op = 4'b0011; ID_mnemonic0 = "S"; ID_mnemonic1 = "B"; ID_mnemonic2 = "C"; end
                    4'b0111: begin ID_alu_op = 4'b0101; ID_mnemonic0 = "R"; ID_mnemonic1 = "S"; ID_mnemonic2 = "C"; end
                    4'b1000: begin ID_alu_op = 4'b0110; ID_mnemonic0 = "T"; ID_mnemonic1 = "S"; ID_mnemonic2 = "T"; end
                    4'b1001: begin ID_alu_op = 4'b1000; ID_mnemonic0 = "T"; ID_mnemonic1 = "E"; ID_mnemonic2 = "Q"; end
                    4'b1010: begin ID_alu_op = 4'b0010; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "P"; end
                    4'b1011: begin ID_alu_op = 4'b0000; ID_mnemonic0 = "C"; ID_mnemonic1 = "M"; ID_mnemonic2 = "N"; end
                    4'b1100: begin ID_alu_op = 4'b0111; ID_mnemonic0 = "O"; ID_mnemonic1 = "R"; ID_mnemonic2 = "R"; end
                    4'b1101: begin ID_alu_op = 4'b1010; ID_mnemonic0 = "M"; ID_mnemonic1 = "O"; ID_mnemonic2 = "V"; end
                    4'b1110: begin ID_alu_op = 4'b1100; ID_mnemonic0 = "B"; ID_mnemonic1 = "I"; ID_mnemonic2 = "C"; end
                    4'b1111: begin ID_alu_op = 4'b1011; ID_mnemonic0 = "M"; ID_mnemonic1 = "V"; ID_mnemonic2 = "N"; end
                endcase
            end
            3'b010: begin 
                ID_S_bit = 0;
                ID_load_instr = instruction[20]; 
                ID_B_instr = 0;
                ID_load_store_instr = 1;
                if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                ID_BL_instr = 0;
                ID_shift_AM = 2'b10;
                if (instruction[20] == 0) begin ID_load_instr = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0; end
                else begin ID_load_instr = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1; end
                ID_alu_op = (instruction[23] == 1) ? 4'b0000 : 4'b0010;
            end
            3'b011: begin 
                    ID_S_bit = 0;
                    ID_B_instr = 0;
                    ID_load_store_instr = 1;
                    if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                    ID_BL_instr = 0;
                    if (instruction[20] == 0) begin ID_load_instr = 1; ID_load_instr = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0; end
                else begin ID_load_instr = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1; end
                    
                    
                    ID_shift_AM = 2'b11;
                            
                    if(instruction[23] == 1) begin
                        ID_alu_op = 4'b0000;
                    end
                            
                    else
                        ID_alu_op = 4'b0010;
                    end
            3'b101: begin 
                ID_S_bit = 0;
                ID_load_instr = 0;
                ID_RF_enable = 0; 
                ID_B_instr = 1;
                ID_load_store_instr = 0;
                ID_size = 0;
                ID_BL_instr = instruction[24];
                if (instruction[24] == 0) begin ID_mnemonic0 = "B"; ID_mnemonic1 = " "; ID_mnemonic2 = " "; end
                else begin ID_mnemonic0 = "B"; ID_mnemonic1 = "L"; ID_mnemonic2 = " "; end
            end
        endcase
    end
end        
endmodule

module MUX (
    output reg [1:0] AM,
    output reg [3:0] opcode,
    output reg S, load, RFenable, B, BL, size, ReadWrite,
    input [1:0] ID_shift_AM,
    input [3:0] ID_alu_op,
    input ID_S_Bit, ID_load_instr, ID_RF_enable, ID_B_intr, ID_load_store_instr, ID_size, ID_BL_instr, select
);
    always @(*) begin
        if (select) begin
            AM = 2'b00;
            opcode = 4'b0000;
            S = 1'b0;
            load = 1'b0;
            RFenable = 1'b0;
            B = 1'b0;
            BL = 1'b0;
            size = 1'b0;
            ReadWrite = 1'b0;
        end else begin
            AM = ID_shift_AM;
            opcode = ID_alu_op;
            S = ID_S_Bit;
            load = ID_load_instr;
            ReadWrite = ID_load_store_instr;
            RFenable = ID_RF_enable;
            B = ID_B_intr;
            BL = ID_BL_instr;
            size = ID_size;
        end
    end
endmodule

module ProgramCounter (
    output reg [31:0] Qs, input [31:0] Ds, input enable, clk, reset
);
    always @(posedge clk) begin
        if(reset) begin
            Qs = 32'h00000000;
        end else if(enable) begin
            Qs = Ds;
        end
    end
endmodule

module IF_ID_PipelineReg (
    input Clk, 
    input Reset,
    input IF_ID_enable,
    input [31:0] IF_instruction, 
    output reg [31:0] ID_instruction 
);

always @(posedge Clk) begin
    if (Reset) begin
        ID_instruction <= 32'b0; 
    end else if (IF_ID_enable) begin
        ID_instruction <= IF_instruction; 
    end
end
endmodule

module ID_EX_PipelineReg (
    input Clk,
    input Reset,
    input ID_S_instr,
    input [3:0] ID_alu_op,
    input ID_load_instr,
    input ID_RF_enable,
    input ID_load_store_instr,
    input ID_size,
    input ID_BL_instr,
    input ID_B_instr,
    input [1:0] ID_shift_AM,
    output reg EX_S_instr,
    output reg [3:0] EX_alu_op,
    output reg EX_load_instr,
    output reg EX_RF_enable,
    output reg EX_load_store_instr,
    output reg EX_size,
    output reg EX_BL_instr,
    output reg EX_B_instr,
    output reg [1:0] EX_shift_AM
);

always @(posedge Clk) begin
    if (Reset) begin
        EX_S_instr <= 1'b0;
        EX_alu_op <= 4'b0000;
        EX_load_instr <= 1'b0;
        EX_RF_enable <= 1'b0;
        EX_load_store_instr <= 1'b0;
        EX_size <= 1'b0;
        EX_BL_instr <= 1'b0;
        EX_shift_AM <= 2'b00;
        EX_B_instr <= 0;
    end else begin
        EX_S_instr <= ID_S_instr;
        EX_alu_op <= ID_alu_op;
        EX_load_instr <= ID_load_instr;
        EX_RF_enable <= ID_RF_enable;
        EX_load_store_instr <= ID_load_store_instr;
        EX_size <= ID_size;
        EX_BL_instr <= ID_BL_instr;
        EX_shift_AM <= ID_shift_AM;
        EX_B_instr <= ID_B_instr;
    end
end
endmodule


module EX_MEM_PipelineReg (
    input Clk,
    input Reset,
    input EX_load_store_instr,
    input EX_size,
    input EX_RF_enable,
    input EX_load_instr,
    output reg MEM_load_store_instr,
    output reg MEM_size,
    output reg MEM_RF_enable,
    output reg MEM_load_instr
);

always @(posedge Clk) begin
    if (Reset) begin
        MEM_load_store_instr <= 1'b0;
        MEM_size <= 1'b0;
        MEM_RF_enable <= 1'b0;
    end else begin
        MEM_load_store_instr <= EX_load_store_instr;
        MEM_size <= EX_size;
        MEM_RF_enable <= EX_RF_enable;
    end
end
endmodule

module MEM_WB_PipelineReg (
    input Clk,
    input Reset,
    input MEM_RF_enable,
    output reg WB_RF_enable
);

always @(posedge Clk) begin
    if (Reset) begin
        WB_RF_enable <= 1'b0;
    end else begin
        WB_RF_enable <= MEM_RF_enable;
    end
end
endmodule

module rom256x8 (
    input [7:0] Address,           
    output reg [31:0] Instruction  
);

    reg [7:0] mem [0:255];

    always @(*) begin
        Instruction = {mem[Address], mem[Address + 1], mem[Address + 2], mem[Address + 3]};
    end

endmodule
