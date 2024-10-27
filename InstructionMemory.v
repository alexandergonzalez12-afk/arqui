module Instruction_Memory_ROM (output reg [31:0] I, input [7:0] A);
  
  // Variable I - Instruction(output) of 32-bit
  // Variable A = Address(intput) of 8-bit
  
  // Variable Mem = memory holds 256 bytes as an array 
  reg [7:0] Mem [0:255];

    // Preload the memory with the content inside the file
   

    // Always reads the instruction from memory
  always @(A) begin
      I <= {Mem[A], Mem[A+1], Mem[A+2], Mem[A+3]};
    end

endmodule

module tb_ROM;

  reg [7:0] Address;
  wire [31:0] Instruction;
  integer fi, fo, code;
  reg [7:0] data; // Temporary variable to read data from file

  // Instantiate the instruction memory (ROM)
  Instruction_Memory_ROM rom_inst (.I(Instruction), .A(Address));

  initial begin
    // Preload the memory with the content inside the file
    fi = $fopen("file_precarga_fase_I.txt", "r");
    if (fi) begin
      Address = 8'b00000000;
      while (!$feof(fi)) begin
        code = $fscanf(fi, "%b", data);
        rom_inst.Mem[Address] = data; // Preload the ROM memory
        Address = Address + 1;
      end
      $fclose(fi);
    end 

    // Open the output file
    fo = $fopen("Output_ROM.txt", "w");

    if (fo) begin
      Address = 8'd0;
      #10 $display("%d %h", Address, Instruction);
      $fdisplay(fo, "%d %h", Address, Instruction);

      Address = 8'd4;
      #10 $display("%d %h", Address, Instruction);
      $fdisplay(fo, "%d %h", Address, Instruction);

      Address = 8'd8;
      #10 $display("%d %h", Address, Instruction);
      $fdisplay(fo, "%d %h", Address, Instruction);

      Address = 8'd12;
      #10 $display("%d %h", Address, Instruction);
      $fdisplay(fo, "%d %h", Address, Instruction);

      $fclose(fo);
    end 

    // Finish and stop the simulation
    $finish;
  end

endmodule

