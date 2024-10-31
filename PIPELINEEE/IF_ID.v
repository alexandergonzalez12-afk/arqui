module IF_ID (
    input E
    input reset
    input clk
    input [31:0] instr_in

    
    output [31:0] instr_out

);
    always(posedge clk or posedge reset) begin

        if(reset)begin
            instr_out <= 32'b0;
        end
        else 
        instr_out <= instr_in