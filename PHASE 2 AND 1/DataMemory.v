module Data_Memory_RAM (output reg [31:0] data_out, input [7:0] address, input [31:0] data_in, input [1:0] size, input rw, input enable);

  // Memory array "Mem" to hold 256 bytes, each element is 8 bits
  reg [7:0] Mem [0:255];

  // Always block to handle both read and write operations
  always @(*) begin
    // Check the condition for read or write operation
    if (rw == 0) begin  // Read operation if rw  =0 then check for size
      if (size == 0) begin  // If size =0 returns a  the 32-bit value
        data_out <= {24'b0, Mem[address]};  // Return a 32-bit value with only the least significant byte little endian
      end else if (size == 1) begin  // Read a word (4 bytes)
        data_out <= {Mem[address], Mem[address+1], Mem[address+2], Mem[address+3]};  // Return 4 consecutive bytes
      end
    end else if (rw == 1 && enable == 1) begin  // Write operation
      if (size == 0) begin  // Write a single byte
        Mem[address] <= data_in[7:0];  // Write only the least significant byte of data_in
      end else if (size == 1) begin  // Write a word (4 bytes)
        Mem[address] <= data_in[31:24];      // Write the most significant byte
        Mem[address+1] <= data_in[23:16];    // Write the next byte
        Mem[address+2] <= data_in[15:8];     // Write the next byte
        Mem[address+3] <= data_in[7:0];      // Write the least significant byte
      end
    end
  end

endmodule



module tb_RAM;

  reg [7:0] Address;             // Address for memory operations
  reg [31:0] DataIn;             // Data input for write operations
  wire [31:0] DataOut;           // Data output for read operations
  reg RW;                        // Read/Write control (0 for read, 1 for write)
  reg E;                         // Enable control
  reg [1:0] Size;                // Size of read/write (0 for byte, 1 for word)
  integer fi, fo, code;          // File pointers and return code for file operations
  reg [7:0] Mem [0:255];         // Memory array to preload data

  // Instantiate the data memory (RAM)
  Data_Memory_RAM ram_inst (.data_out(DataOut), .address(Address), .data_in(DataIn), .size(Size), .rw(RW), .enable(E));

  initial begin
    // Preload the memory with the content inside the file
    fi = $fopen("file_precarga_fase_I.txt", "r");  // Open the file in read mode
    if (fi) begin
      Address = 8'b00000000;                       // Start at address 0
      while (!$feof(fi)) begin                     // Loop until end of file
        code = $fscanf(fi, "%b", DataIn);          // Read 8-bit data from the file
        ram_inst.Mem[Address] = DataIn;            // Store the data in the RAM memory
        Address = Address + 1;                     // Increment address for next data
      end
      $fclose(fi);                                 // Close the input file
    end 

    // Open the output file in write mode
    fo = $fopen("Output_RAM.txt", "w");

    // Set control signals for read operation
    E = 1;         // Enable the memory
    RW = 0;        // Set to read mode
    Size = 1;      // Set size to word (4 bytes)

    // Perform multiple read operations and display/write results
    Address = 8'd0;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    Address = 8'd4;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    Address = 8'd8;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    Address = 8'd12;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    
    //Bullet2 Read Address 0 and 4
    Size = 0;
    Address = 8'd0;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    
    Size = 1;
    Address = 8'd4;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    
    //Write Address 0, 2, 8
    
    RW = 1;
    
    Size = 0;
    Address = 8'd0;
    DataIn = 32'hA6;
    #10;
   
    
    Address = 8'd2;
    DataIn = 32'hDD;
    #10;
    
    Size = 1;
    Address = 8'd8;
    DataIn = 32'hABCDEF01;
    #10;
    RW = 0;
    
    // Read word from a different address
    Size = 1;
    Address = 8'd0;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    
    
    Address = 8'd4;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    // Perform more read operations
    Address = 8'd8;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    
    //For debbuging address 2
    Size = 0;
    Address = 8'd2;
    Size = 0;
    #10;
    $display("A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);
    $fdisplay(fo, "A=%d, DO=%h, Size=%b, RW=%b, E=%b", Address, DataOut, Size, RW, E);

    // Close the output file and end simulation
    $fclose(fo);
    $finish;
  end

endmodule
