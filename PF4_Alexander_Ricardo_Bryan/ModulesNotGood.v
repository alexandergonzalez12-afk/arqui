module adder(output reg [31:0] NextPC, input [31:0] PC);
always @(*) begin
    NextPC = PC + 4; // Increment by 4
end
endmodule

module ALU(output reg Z, N, C, V, output reg [31:0] Out, input [31:0] A, B, input Cin, input [3:0] opcode);
    always @(A, B, opcode) begin
        case(opcode)
            4'b0000: begin
                //A+B
                {C, Out} = A + B;
                Z = (Out == 0);
                N = Out[31];
                V = ~(A[31] ^ B[31]) & (A[31] ^ Out[31]);
            end
            
            4'b0001: begin
                //A+B+Cin
                {C, Out} = A + B + Cin;
                Z = (Out == 0);
                N = Out[31];
                V = ~(A[31] ^ B[31]) & (A[31] ^ Out[31]);
            end
            
            4'b0010: begin
                //A-B
                Out = A - B;
                Z = (Out == 0);
                N = Out[31];
                C = (A < B) ? 1 : 0;
                V = (A[31] ^ B[31]) & (A[31] ^ Out[31]);
            end
            
            4'b0011: begin
                //A-B-Cin
                Out = A - B - Cin;
                Z = (Out == 0);
                N = Out[31];
                C = (A < (B+Cin)) ? 1 : 0;
                V = (A[31] ^ B[31]) & (A[31] ^ Out[31]);
            end
            
            4'b0100: begin
                //B-A
                Out = B - A;
                Z = (Out == 0);
                N = Out[31];
                C = (B < A) ? 1 : 0;
                V = (B[31] ^ A[31]) & (B[31] ^ Out[31]);
            end
            
            4'b0101: begin
                //B-A-Cin
                Out = B - A - Cin;
                Z = (Out == 0);
                N = Out[31];
                C = (B < (A+Cin)) ? 1 : 0;
                V = (B[31] ^ A[31]) & (B[31] ^ Out[31]);
            end
            
            4'b0110: begin
                //A and B
                Out = A & B;
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b0111: begin
                //A or B
                Out = A | B;
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b1000: begin
                //A xor B
                Out = A ^ B;
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b1001: begin
                //A
                Out = A;
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b1010: begin
                //B
                Out = B;
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b1011: begin
                //not B
                Out = (~B);
                Z = (Out == 0);
                N = Out[31];
            end
            
            4'b1100: begin
                //A and (not B)
                Out = A & (~B);
                Z = (Out == 0);
                N = Out[31];
            end
            
        endcase
    end    
    
endmodule

module condition_handler (
    input [3:0] CC,            // Condition codes: N, Z, C, V
    input [3:0] ID_ICC_3128,   // Instruction condition field
    input ID_B_Instr,          // Indicates branch instruction
    input ID_BL_Instr,         // Indicates branch-and-link instruction
    output reg Out_B,          // Branch signal
    output reg Out_BL          // Branch-and-link signal
);
    
    // Extract condition code flags
    wire N = CC[3];  // Negative flag
    wire Z = CC[2];  // Zero flag
    wire C = CC[1];  // Carry flag
    wire V = CC[0];  // Overflow flag

    always @(*) begin
        // Default outputs
        Out_B = 0;
        Out_BL = 0;

        // Branch condition handling
        if (ID_B_Instr) begin
            case (ID_ICC_3128)
                4'b0000: Out_B = (Z == 1);                 // : Branch if equal
                4'b0001: Out_B = (Z == 0);                 // : Branch if not equal
                4'b0010: Out_B = (C == 1);                 // : Branch if carry set
                4'b0011: Out_B = (C == 0);                 // : Branch if carry clear
                4'b0100: Out_B = (N == 1);                 // : Branch if minus
                4'b0101: Out_B = (N == 0);                 // : Branch if pos or zero
                4'b0110: Out_B = (V == 1);                 // : Branch if overflow
                4'b0111: Out_B = (V == 0);                 // : Branch if no overflow
                4'b1000: Out_B = ((C==1)&&(Z==0));          // Branch if unsigned higher
                4'b1001: Out_B = ((C==0)||(Z==1));          // Branch if unsigned lower or same
                4'b1010: Out_B = (N==V);                    // Branch if greater or equal
                4'b1011: Out_B = (N!=V);                    // Branch if less than
                4'b1100: Out_B = ((Z==0)&&(N==V));          // Branch if greater than
                4'b1101: Out_B = ((Z==1)||(N!=V));          // Branch if less than or equal
                4'b1110: Out_B = 1;
            endcase
            if (ID_BL_Instr) begin
            Out_BL = 1; // Always branch-and-link if ID_BL_Instr is active
        end
        end
    end

endmodule


module control_MUX (
    output reg [1:0] AM,
    output reg [3:0] opcode,
    output reg S, load, RFenable, B, BL, size, ReadWrite,Enable,
    input [1:0] ID_shift_AM,
    input [3:0] ID_alu_op,
    input ID_S_Bit, ID_load_instr, ID_RF_enable, ID_B_intr, ID_enable_instr, ID_RW, ID_size, ID_BL_instr, select
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
            Enable = 1'b0;
            size = 1'b0;
            ReadWrite = 1'b0;
        end else begin
            AM = ID_shift_AM;
            opcode = ID_alu_op;
            S = ID_S_Bit;
            load = ID_load_instr;
            ReadWrite = ID_RW;
            Enable = ID_enable_instr;
            RFenable = ID_RF_enable;
            B = ID_B_intr;
            BL = ID_BL_instr;
            size = ID_size;
        end
    end
endmodule

module control_unit(
    output reg ID_S_bit, ID_load_instr, ID_RF_enable, ID_B_instr,
    ID_enable_instr, ID_size, ID_BL_instr, ID_RW,
    output reg [1:0] ID_shift_AM, sop_count,
    output reg [3:0] ID_alu_op, 
    output reg [7:0] ID_mnemonic0, ID_mnemonic1, ID_mnemonic2,
    input [31:0] instruction
);

always @(instruction) begin
/*
signals for the number of source operands
00 - 0 source operands
01 - 1 source operand
10 - 2 source operands
11 - 3 source operands
*/
    if (instruction == 32'b00000000000000000000000000000000) begin
        ID_S_bit = 0;
        ID_load_instr = 0;
        ID_RF_enable = 0;
        ID_B_instr = 0;
        ID_enable_instr = 0;
        ID_size = 0;
        ID_RW = 0;
        ID_BL_instr = 0;
        ID_shift_AM = 2'b00;
        ID_alu_op = 4'b0000;
        ID_mnemonic0 = "N";
        ID_mnemonic1 = "O";
        ID_mnemonic2 = "P";
    end else begin
        case (instruction[27:25])
            3'b000: begin  // data processing immediate shift
                if (instruction[4] == 0) begin
                    ID_S_bit = instruction[20];
                    ID_load_instr = 0;
                    ID_RW = 0;
                    ID_RF_enable = 1; 
                    ID_B_instr = 0;
                    ID_enable_instr = 0;
                    ID_size = 0; 
                    ID_BL_instr = 0;
                    ID_shift_AM = 2'b11;
                    
                    /* THE ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b10; // 2 source operands
                    
                    
                    
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
            3'b001: begin // data processing immediate
                ID_S_bit = instruction[20];
                ID_load_instr = 0;
                ID_RW = 0;
                ID_RF_enable = 1;
                ID_B_instr = 0;
                ID_enable_instr = 0;
                ID_size = 0; 
                ID_BL_instr = 0;
                ID_shift_AM = 2'b00;
                
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                sop_count = 2'b01; // 1 source operand
                // if this doesnt work, add another condition for the mov case for 0 operands (should work because the only mov instructions occur with 1 source operand)
                
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
            3'b010: begin // load store immediate offset
                ID_S_bit = 0;
                ID_load_instr = instruction[20]; 
                ID_B_instr = 0;
                ID_enable_instr = 1;
                if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                ID_BL_instr = 0;
                ID_shift_AM = 2'b10;
                if (instruction[20] == 0) begin ID_RW = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0; 
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                
                sop_count = 2'b10; // 2 source operands
                
                end
                else begin ID_RW = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1;
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                
                sop_count = 2'b01; // 1 source operand
                
                end
                ID_alu_op = (instruction[23] == 1) ? 4'b0000 : 4'b0010;
            end
            3'b011: begin //Load/Store register offset
                    ID_S_bit = 0;
                    ID_B_instr = 0;
                    ID_enable_instr = 1;
                    ID_load_instr = instruction[20];
                    if(instruction[22]) begin
                        ID_size = 0;
                    end else begin
                        ID_size = 1;
                    end
                    ID_BL_instr = 0;
                    if (instruction[20] == 0) begin ID_RW = 1; ID_RW = 1; ID_mnemonic0 = "S"; ID_mnemonic1 = "T"; ID_mnemonic2 = "R"; ID_RF_enable = 0;
                    /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b11; // 3 source operands
                    
                    end
                else begin ID_RW = 0; ID_mnemonic0 = "L"; ID_mnemonic1 = "D"; ID_mnemonic2 = "R"; ID_RF_enable = 1; end
                    
                    //scaled register offset
                    ID_shift_AM = 2'b11;
                    
                    /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                    sop_count = 2'b01;
                            
                    if(instruction[23] == 1) begin
                        ID_alu_op = 4'b0000;
                    end
                            
                    else
                        ID_alu_op = 4'b0010;
                    end
                        
                        
                        
                   
            
            3'b101: begin // branch or branch and link
                ID_S_bit = 0;
                ID_load_instr = 0;
                ID_RF_enable = 0; 
                ID_B_instr = 1;
                ID_enable_instr = 0;
                ID_size = 0;
                ID_BL_instr = instruction[24];
                /* ADDITION OF THE CODE ACCOUNTING FOR THE DIFFERENT OPERANDS. REMEMBER TO ADD THE OUTPUT OF THE SIGNAL CORRECTLY AND LINK IT TO THE HAZARD FORWARDING UNIT. MAKE A NEW INPUT IN THE HAZARD FORWARDING UNIT AS WELL. */
                sop_count = 2'b00;
                if (instruction[24] == 0) begin ID_mnemonic0 = "B"; ID_mnemonic1 = " "; ID_mnemonic2 = " "; end
                else begin ID_mnemonic0 = "B"; ID_mnemonic1 = "L"; ID_mnemonic2 = " "; end
            end
        endcase
    end
end        
endmodule

module HazardForwardingUnit(
    output reg [1:0] ID_MUX_PA, ID_MUX_PB,ID_MUX_PD,
    output reg NOP,
    output reg IF_ID_LE, PC_LE,
    input [3:0] EX_Rd, MEM_Rd, WB_Rd,
    input [3:0] ID_RD, ID_RA, ID_RB,
    input EX_RF_enable, MEM_RF_enable, WB_RF_enable,
    input ID_enable_instr,ID_load_instr, EX_load_instr,
    input [1:0] sop_count
);
/*
selection signal to multiplexers in ID phase

Register from id: 00
Register from ex: 01
Register from mem: 10
Register from wb: 11

*/
    always @(*) begin
        ID_MUX_PA = 2'b00;
        ID_MUX_PB = 2'b00;
        ID_MUX_PD = 2'b00;
        NOP = 0;
        IF_ID_LE = 1;
        PC_LE = 1;

    
        if (sop_count > 2'b00) begin
            // Data Forwarding for Source Operand 1 (Rn / ID_RA)
            if (sop_count >= 2'b01) begin
                if (EX_RF_enable && (ID_RA == EX_Rd)) ID_MUX_PA = 2'b01;
                else if (MEM_RF_enable && (ID_RA == MEM_Rd)) ID_MUX_PA = 2'b10;
                else if (WB_RF_enable && (ID_RA == WB_Rd)) ID_MUX_PA = 2'b11;
            end

            // Data Forwarding for Source Operand 2 (Rm / ID_RB)
            if (sop_count >= 2'b10) begin
                if (EX_RF_enable && (ID_RB == EX_Rd)) ID_MUX_PB = 2'b01;
                else if (MEM_RF_enable && (ID_RB == MEM_Rd)) ID_MUX_PB = 2'b10;
                else if (WB_RF_enable && (ID_RB == WB_Rd)) ID_MUX_PB = 2'b11;
            end

            // Data Forwarding for Source Operand 3 (e.g., ID_RD for some instructions)
            if (sop_count == 2'b11) begin
                if (EX_RF_enable && (ID_RD == EX_Rd)) ID_MUX_PD = 2'b01;
                else if (MEM_RF_enable && (ID_RD == MEM_Rd)) ID_MUX_PD = 2'b10;
                else if (WB_RF_enable && (ID_RD == WB_Rd)) ID_MUX_PD = 2'b11;
            end
        end
            if(EX_load_instr && ((ID_RA == EX_Rd) || (ID_RB == EX_Rd))) begin //load hazard
                    NOP = 1;
                    IF_ID_LE = 0;
                    PC_LE = 0;
            end
            if(ID_enable_instr) begin
                 if(!ID_load_instr) begin // Store hazard
                    // sop_count == 3
                    if (sop_count == 2'b11) begin
                        if (EX_RF_enable && ((ID_RA == EX_Rd) || (ID_RB == EX_Rd) || (ID_RD == EX_Rd))) ID_MUX_PD = 2'b01;
                        else if (MEM_RF_enable && ((ID_RA == MEM_Rd) || (ID_RB == MEM_Rd) || (ID_RD == MEM_Rd))) ID_MUX_PD = 2'b10;
                        else if (WB_RF_enable && ((ID_RA == WB_Rd) || (ID_RB == WB_Rd) || (ID_RD == WB_Rd))) ID_MUX_PD = 2'b11;
                    end
                    // sop_count == 2 (Rn and Rd only)
                    else if (sop_count == 2'b10) begin
                        if (EX_RF_enable && ((ID_RA == EX_Rd) || (ID_RD == EX_Rd))) ID_MUX_PD = 2'b01;
                        else if (MEM_RF_enable && ((ID_RA == MEM_Rd) || (ID_RD == MEM_Rd))) ID_MUX_PD = 2'b10;
                        else if (WB_RF_enable && ((ID_RA == WB_Rd) || (ID_RD == WB_Rd))) ID_MUX_PD = 2'b11;
                    end
                end
            end


    end
    
endmodule



module mux2x4(
    output reg [3:0] out,
    input [3:0] portA, portB,
    input select
);
    always @(*)begin
        if(!select) out = portA;
        else out = portB;
    end


endmodule

module mux2x32(
    output reg [31:0] out,
    input [31:0] portA, portB,
    input select
);
    always @(*)begin
        if(!select) out = portA;
        else out = portB;
    end

endmodule

module mux4x32(
    output reg [31:0] out,
    input [31:0] portA, portB, portC,portD,
    input [1:0] select
);
always @(*) begin
    if (select == 2'b00) out = portA;
    else if (select == 2'b01) out = portB;
    else if (select == 2'b10) out = portC;
    else if (select == 2'b11) out = portD;
    
end

endmodule

module NextPC_Adder(
    input [31:0] NextPC,
    input signed [31:0] imm_valuex4,
    output reg [31:0] out
);
always @(*) begin
    out = NextPC + imm_valuex4;
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
    input [31:0] IF_NextPC,
    input [31:0] IF_instruction, // Instruction from IF stage
    output reg [31:0] ID_instruction,
    output reg signed [23:0] imm_value,
    output reg [31:0] ID_NextPC,
    output reg [3:0] ID_RA,
    output reg[3:0] ID_RB,
    output reg[3:0] ID_RD,
    output reg [11:0] ID_shiftA,
    output reg [3:0] ID_ICC
);

always @(posedge Clk) begin
    if (Reset) begin
        ID_instruction <= 32'b0;
        ID_NextPC <= 32'b0;
        ID_RA <= ID_instruction[19:16];
        ID_RB <= ID_instruction[3:0];
        ID_RD <= ID_instruction[15:12];
        imm_value <= ID_instruction[23:0];
        ID_shiftA <= ID_instruction[11:0];
        ID_ICC <= ID_instruction[31:28];
    end else if (IF_ID_enable) begin
        ID_instruction <= IF_instruction;
        ID_NextPC <= IF_NextPC;
        ID_RA <= IF_instruction[19:16];
        ID_RB <= IF_instruction[3:0];
        ID_RD <= IF_instruction[15:12];
        imm_value <= IF_instruction[23:0];
        ID_shiftA <= IF_instruction[11:0];
        ID_ICC <= IF_instruction[31:28];
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
    input ID_enable_instr,
    input ID_size,
    input ID_RW,
    input ID_BL_instr,
    input ID_B_instr,
    input [1:0] ID_shift_AM,
    input [31:0] ID_PA, ID_PB, ID_PD,
    input [11:0] ID_shiftA, 
    input [3:0] ID_RD,
    input [31:0] ID_NextPC,
    input CH_ID_BL,
    output reg EX_S_instr,
    output reg EX_load_instr,
    output reg EX_RF_enable,
    output reg EX_enable_instr,
    output reg EX_RW,
    output reg EX_size,
    output reg EX_BL_instr,
    output reg EX_B_instr,
    output reg [3:0] EX_alu_op,
    output reg [1:0] EX_shift_AM,
    output reg [31:0] EX_NextPC,
    output reg CH_EX_BL,
    output reg [3:0] EX_RD,
    output reg [31:0] EX_PA, EX_PB, EX_PD,
    output reg [11:0] EX_shiftA
);

always @(posedge Clk) begin
    if (Reset) begin
        EX_S_instr <= 1'b0;
        EX_alu_op <= 4'b0;
        EX_load_instr <= 1'b0;
        EX_RF_enable <= 1'b0;
        EX_RW <= 1'b0;
        EX_enable_instr <= 1'b0;
        EX_size <= 1'b0;
        EX_BL_instr <= 1'b0;
        EX_shift_AM <= 2'b00;
        EX_B_instr <= 0;
        EX_PA <= 32'b0;
        EX_PB <= 32'b0;
        EX_PD <= 32'b0;
        CH_EX_BL <= 0;
        EX_RD <= 4'b0;
        EX_shiftA <= 12'b0;
        EX_NextPC <= 32'b0;
    end else begin
        EX_S_instr <= ID_S_instr;
        EX_alu_op <= ID_alu_op;
        EX_load_instr <= ID_load_instr;
        EX_RF_enable <= ID_RF_enable;
        EX_enable_instr <= ID_enable_instr;
        EX_size <= ID_size;
        EX_RW <= ID_RW;
        EX_BL_instr <= ID_BL_instr;
        EX_shift_AM <= ID_shift_AM;
        EX_B_instr <= ID_B_instr;
        EX_PA <= ID_PA;
        EX_PB <= ID_PB;
        EX_PD <= ID_PD;
        EX_NextPC <= ID_NextPC;
        CH_EX_BL <= CH_ID_BL;
        EX_RD <= ID_RD;
        EX_shiftA <= ID_shiftA;
    end
end
endmodule


module EX_MEM_PipelineReg (
    input Clk,
    input Reset,
    input EX_enable_instr,
    input EX_size,
    input EX_RF_enable,
    input EX_load_instr,
    input EX_RW,
    input [31:0] EX_PA, EX_PD,
    input [31:0] EX_ALU_out,
    input [3:0] EX_RD,
    output reg MEM_enable_instr,
    output reg MEM_size,
    output reg MEM_RF_enable,
    output reg MEM_load_instr,
    output reg MEM_RW,
    output reg [31:0] MEM_PA, MEM_PD,
    output reg [31:0] MEM_ALU_out,
    output reg [3:0] MEM_RD
);

always @(posedge Clk) begin
    if (Reset) begin
        MEM_enable_instr <= 1'b0;
        MEM_size <= 1'b0;
        MEM_RF_enable <= 1'b0;
        MEM_load_instr <= 1'b0;
        MEM_RW <= 1'b0;
        MEM_PA <= 32'b0;
        MEM_ALU_out <= 32'b0;
        MEM_RD <= 4'b0;
        MEM_PD <= 32'b0;
    end else begin
        MEM_enable_instr <= EX_enable_instr;
        MEM_size <= EX_size;
        MEM_RF_enable <= EX_RF_enable;
        MEM_load_instr <= EX_load_instr;
        MEM_ALU_out <= EX_ALU_out;
        MEM_PA <= EX_PA;
        MEM_RW <= EX_RW;
        MEM_RD <= EX_RD;
        MEM_PD <= EX_PD;
    end
end
endmodule

module MEM_WB_PipelineReg (
    input Clk,
    input Reset,
    input MEM_RF_enable,
    input MEM_load_instr,
    input [31:0] MEM_MUX_out,
    input [3:0] MEM_RD,
    output reg WB_RF_enable,
    output reg WB_load_instr,
    output reg [31:0] WB_MUX_out,
    output reg [3:0] WB_RD
);

always @(posedge Clk) begin
    if (Reset) begin
        WB_RF_enable <= 1'b0;
        WB_load_instr <= 1'b0;
        WB_MUX_out <= 32'b0;
        WB_RD <= 4'b0;
    end else begin
        WB_RF_enable <= MEM_RF_enable;
        WB_load_instr <= MEM_load_instr;
        WB_MUX_out <= MEM_MUX_out;
        WB_RD <= MEM_RD;
    end
end
endmodule
module PSR(
    output reg [3:0] PSR_CC,
    output reg Cin,
    input [3:0] Alu_CC,
    input clk
);
always @(posedge clk) begin
    Cin = Alu_CC[1];
    PSR_CC = Alu_CC;
end

endmodule
module ram256x8 (
    output reg [31:0] DataOut,
    input Enable,
    input ReadWrite,
    input [7:0] Address,
    input [31:0] DataIn,
    input Size
);

    reg [7:0] mem [0:255];  

    always @(*) begin
        if (!ReadWrite) begin  
            if (Size == 1) begin 
                DataOut = {mem[Address], mem[Address+1], mem[Address+2], mem[Address+3]};
            end else begin  
                DataOut = {24'b0,mem[Address]};
            end
        end else begin
            if (Size == 1) begin
                mem[Address+3]   = DataIn[7:0];
                mem[Address+2] = DataIn[15:8];
                mem[Address+1] = DataIn[23:16];
                mem[Address] = DataIn[31:24];
            end else begin
                mem[Address] = DataIn[7:0];
            end
        end
    end
endmodule

module register32 (
    output reg [31:0] Q,
    input [31:0] D,
    input LE,
    input Clk,
    input reset // Add reset signal
);
    always @ (posedge Clk or posedge reset) begin
        if (reset)
            Q <= 32'b0; // Initialize to 0 during reset
        else if (LE)
            Q <= D;
    end
endmodule



module Dec4to16 (
    input [3:0] addr,
    input E,
    output reg [15:0] O
);
    always @(*) begin
        if (E) begin
            O = 16'b0;
            O[addr] = 1'b1;
        end
        else 
            O = 16'b0;
    end
endmodule

module Mux16to1 (
    input [31:0] reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7,
    input [31:0] reg8, reg9, reg10, reg11, reg12, reg13, reg14, reg15,
    input [3:0] sel,
    output reg [31:0] out
);
    always @(*) begin
        case (sel)
            4'd0: out = reg0;
            4'd1: out = reg1;
            4'd2: out = reg2;
            4'd3: out = reg3;
            4'd4: out = reg4;
            4'd5: out = reg5;
            4'd6: out = reg6;
            4'd7: out = reg7;
            4'd8: out = reg8;
            4'd9: out = reg9;
            4'd10: out = reg10;
            4'd11: out = reg11;
            4'd12: out = reg12;
            4'd13: out = reg13;
            4'd14: out = reg14;
            4'd15: out = reg15;
            //default: out = 32'b0;
        endcase
    end
endmodule

module RegisterFile (
    input clk,
    input reset, 
    input LE,
    input [3:0] RW,
    input [31:0] PW,
    input [3:0] RA, RB, RD,
    input [31:0] PC,
    output [31:0] PA, PB, PD
);

    wire [15:0] write_enable;
    wire [31:0] reg_file [14:0];

    Dec4to16 decoder(
        .addr(RW),
        .E(LE),
        .O(write_enable)
    );

    register32 reg0 (
        .Q(reg_file[0]), 
        .D(PW), 
        .LE(write_enable[0]),
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg1 (
        .Q(reg_file[1]), 
        .D(PW), 
        .LE(write_enable[1]),
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg2 (
        .Q(reg_file[2]), 
        .D(PW), 
        .LE(write_enable[2]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg3 (
        .Q(reg_file[3]), 
        .D(PW), 
        .LE(write_enable[3]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg4 (
        .Q(reg_file[4]), 
        .D(PW), 
        .LE(write_enable[4]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg5 (
        .Q(reg_file[5]), 
        .D(PW), 
        .LE(write_enable[5]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg6 (
        .Q(reg_file[6]), 
        .D(PW), 
        .LE(write_enable[6]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg7 (
        .Q(reg_file[7]), 
        .D(PW), 
        .LE(write_enable[7]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg8 (
        .Q(reg_file[8]), 
        .D(PW), 
        .LE(write_enable[8]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg9 (
        .Q(reg_file[9]), 
        .D(PW), 
        .LE(write_enable[9]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg10 (
        .Q(reg_file[10]), 
        .D(PW), 
        .LE(write_enable[10]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg11 (
        .Q(reg_file[11]), 
        .D(PW), 
        .LE(write_enable[11]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg12 (
        .Q(reg_file[12]), 
        .D(PW), 
        .LE(write_enable[12]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg13 (
        .Q(reg_file[13]), 
        .D(PW), 
        .LE(write_enable[13]), 
        .Clk(clk),
        .reset(reset) 
    );

    register32 reg14 (
        .Q(reg_file[14]), 
        .D(PW), 
        .LE(write_enable[14]), 
        .Clk(clk),
        .reset(reset) 
    );

    

    Mux16to1 muxA (
        .reg0(reg_file[0]), .reg1(reg_file[1]), .reg2(reg_file[2]), .reg3(reg_file[3]),
        .reg4(reg_file[4]), .reg5(reg_file[5]), .reg6(reg_file[6]), .reg7(reg_file[7]),
        .reg8(reg_file[8]), .reg9(reg_file[9]), .reg10(reg_file[10]), .reg11(reg_file[11]),
        .reg12(reg_file[12]), .reg13(reg_file[13]), .reg14(reg_file[14]), .reg15(PC),
        .sel(RA),
        .out(PA)
    );

    Mux16to1 muxB (
        .reg0(reg_file[0]), .reg1(reg_file[1]), .reg2(reg_file[2]), .reg3(reg_file[3]),
        .reg4(reg_file[4]), .reg5(reg_file[5]), .reg6(reg_file[6]), .reg7(reg_file[7]),
        .reg8(reg_file[8]), .reg9(reg_file[9]), .reg10(reg_file[10]), .reg11(reg_file[11]),
        .reg12(reg_file[12]), .reg13(reg_file[13]), .reg14(reg_file[14]), .reg15(PC),
        .sel(RB),
        .out(PB)
    );

    Mux16to1 muxD (
        .reg0(reg_file[0]), .reg1(reg_file[1]), .reg2(reg_file[2]), .reg3(reg_file[3]),
        .reg4(reg_file[4]), .reg5(reg_file[5]), .reg6(reg_file[6]), .reg7(reg_file[7]),
        .reg8(reg_file[8]), .reg9(reg_file[9]), .reg10(reg_file[10]), .reg11(reg_file[11]),
        .reg12(reg_file[12]), .reg13(reg_file[13]), .reg14(reg_file[14]), .reg15(PC),
        .sel(RD),
        .out(PD)
    );
endmodule




module rom256x8 (
    input [7:0] Address,           // 8-bit address input for 256-entry memory
    output reg [31:0] Instruction  // 32-bit instruction output
);

    reg [7:0] mem [0:255];

    always @(*) begin
        Instruction = {mem[Address], mem[Address + 1], mem[Address + 2], mem[Address + 3]};
    end

endmodule

module shifter_sign_extender(output reg [31:0] N, input [31:0] Rm, input [11:0] I, input [1:0] AM);
    always @(AM, Rm, I) begin
        case(AM)
            2'b00: begin
                N = {24'b0, I[7:0]} << (32-2 * I[11:8]) | {24'b0, I[7:0]} >> (2 * I[11:8]);
            end
            
            2'b01: begin
                N = Rm;
            end
            
            2'b10: begin
                N = {20'b0, I[11:0]};
            end
            
            2'b11: begin
                
                case (I[6:5])
                
                    2'b00: begin
                    //logical shift left
                        N = Rm << (I[11:7]);
                    end
                
                    2'b01: begin
                    //logical shift right
                        N = Rm >> (I[11:7]);
                    end
                    
                    2'b10: begin
                    //arithmetic shift right
                        N = $signed(Rm) >>> I[11:7];
                    end
                    
                    2'b11: begin
                    //rotate right
                        N = (Rm >> I[11:7]) | (Rm << (32 - I[11:7]));
                    end
                    
                endcase
            
            end
            
        endcase
        
        
    end
    
endmodule

module x4SE (
    output reg signed [31:0] imm_valuex4,
    input signed [23:0] imm_value
);
always @(*) begin
    imm_valuex4 = $signed({{6{imm_value[23]}},$signed(imm_value << 2)});
end
endmodule