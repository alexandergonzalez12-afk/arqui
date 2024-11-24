module MUX_Fetch (
    input [7:0] In1,         // First 8-bit input
    input [7:0] In2,         // Second 8-bit input
    input Sel,               // Select signal
    output reg [7:0] MuxOut  // Multiplexer output
);

    always @(*) begin
        if (Sel)
            MuxOut = In2;    // Select In2 when Sel = 1
        else
            MuxOut = In1;    // Select In1 when Sel = 0
    end

endmodule
