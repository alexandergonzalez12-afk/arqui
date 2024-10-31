module ControlUnit(
  input [31:0] instruction,
  output reg [3:0] ALU_OP,
  output reg ID_LOAD,
  output reg ID_MEM_WRITE,
  output reg [1:0] ID_AM,
  output reg STORE_CC,
  output reg ID_B,
  output reg ID_BL,
  output reg ID_MEM_SIZE,
  output reg ID_MEM_E,
  output reg RF_E
);

always @(*) begin
    // Reiniciar las señales de control en caso de instrucción NOP o por defecto
    ALU_OP = 4'b0000;
    ID_LOAD = 0;
    ID_MEM_WRITE = 0;
    ID_AM = 2'b00;
    STORE_CC = 0;
    ID_B = 0;
    ID_BL = 0;
    ID_MEM_SIZE = 0;
    ID_MEM_E = 0;
    RF_E = 0;

    if (instruction != 32'b00000000000000000000000000000000) begin
        // Asignar valores a señales específicas de control desde la instrucción
        ALU_OP = instruction[24:21];       // Opcode para ALU
        ID_LOAD = instruction[20];         // Carga (load)
        ID_MEM_WRITE = instruction[21];    // Escritura de memoria
        ID_AM = instruction[26:25];        // Modo de direccionamiento
        STORE_CC = instruction[20];        // Condicional de almacenamiento
        ID_B = instruction[24];            // Bandera de bifurcación
        ID_BL = instruction[27];           // Bandera de bifurcación con link
        ID_MEM_SIZE = instruction[22];     // Tamaño de memoria
        ID_MEM_E = instruction[23];        // Extensión de memoria
        RF_E = instruction[19];            // Habilitación del registro de escritura
    end
end    
endmodule


module Multiplexer (
  output reg id_load, id_mem_write, store_cc, id_b, id_bl, id_mem_size, id_mem_e, rf_e,
  output reg [3:0] alu_op,
  output reg [1:0] id_am,
  input S,
  input [3:0] ALU_OP,
  input ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E,
  input [1:0] ID_AM
);
  
  always @ (*) begin
    if(S == 1'b0) begin
      id_load = ID_LOAD;
      id_mem_write = ID_MEM_WRITE;
      store_cc = STORE_CC;
      id_b = ID_B;
      id_bl = ID_BL;
      id_mem_size = ID_MEM_SIZE;
      id_mem_e = ID_MEM_E;
      rf_e = RF_E;
      id_am = ID_AM;
      alu_op = ALU_OP;
    end else begin
      id_load = 0;
      id_mem_write = 0;
      store_cc = 0;
      id_b = 0;
      id_bl = 0;
      id_mem_size = 0;
      id_mem_e = 0;
      rf_e = 0;
      id_am = 2'b00;
      alu_op = 4'b0000;
    end
  end
endmodule
