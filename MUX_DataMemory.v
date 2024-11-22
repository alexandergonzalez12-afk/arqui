module MUX_DataMemory(
    input [7:0] Addr,        // Address input (8-bit)
    input [31:0] DataOut,    // Data Memory output (32-bit)
    input Sel,               // Select signal
    output reg [31:0] MuxOut // Multiplexer output
);

    always @(*) begin
        if (Sel)
            MuxOut = DataOut;  // Select DataOut when Sel = 1
        else
            MuxOut = {24'b0, Addr};  // Zero-extend Addr to 32 bits when Sel = 0
    end

endmodule

