module PC (
  input clk,                // Clock signal
  input reset,              // Reset signal to initialize PC to 0
  input E,                  // Enable signal for updating PC
  input [7:0] next_pc,      // 8-bit external incremented PC input
  output reg [7:0] pc       // 8-bit Program Counter
);

  always @(posedge clk or posedge reset) begin
    if (reset)
      pc <= 8'b0;               // Reset PC to 0
    else if (E)
      pc <= next_pc;            // Update PC from external adder result
  end
endmodule



module adder (
  input [7:0] address,       // 8-bit input address
  output [7:0] result        // 8-bit output result (address + 4)
);

  assign result = address + 8'd4;  // Increment address by 4
endmodule


        
