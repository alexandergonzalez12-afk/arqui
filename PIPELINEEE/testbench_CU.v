module ControlUnit_Multiplexer_tb;

  // Señales para ControlUnit
  reg [31:0] instruction;
  wire [3:0] ALU_OP;
  wire ID_LOAD, ID_MEM_WRITE, STORE_CC, ID_B, ID_BL, ID_MEM_SIZE, ID_MEM_E, RF_E;
  wire [1:0] ID_AM;

  // Señales para Multiplexer
  reg S;
  wire id_load, id_mem_write, store_cc, id_b, id_bl, id_mem_size, id_mem_e, rf_e;
  wire [3:0] alu_op;
  wire [1:0] id_am;

  // Instancia de ControlUnit
  ControlUnit control_unit_inst (
    .instruction(instruction),
    .ALU_OP(ALU_OP),
    .ID_LOAD(ID_LOAD),
    .ID_MEM_WRITE(ID_MEM_WRITE),
    .ID_AM(ID_AM),
    .STORE_CC(STORE_CC),
    .ID_B(ID_B),
    .ID_BL(ID_BL),
    .ID_MEM_SIZE(ID_MEM_SIZE),
    .ID_MEM_E(ID_MEM_E),
    .RF_E(RF_E)
  );

  // Instancia de Multiplexer
  Multiplexer mux_inst (
    .id_load(id_load), 
    .id_mem_write(id_mem_write), 
    .store_cc(store_cc), 
    .id_b(id_b), 
    .id_bl(id_bl), 
    .id_mem_size(id_mem_size), 
    .id_mem_e(id_mem_e), 
    .rf_e(rf_e),
    .alu_op(alu_op),
    .id_am(id_am),
    .S(S),
    .ALU_OP(ALU_OP),
    .ID_LOAD(ID_LOAD),
    .ID_MEM_WRITE(ID_MEM_WRITE),
    .STORE_CC(STORE_CC),
    .ID_B(ID_B),
    .ID_BL(ID_BL),
    .ID_MEM_SIZE(ID_MEM_SIZE),
    .ID_MEM_E(ID_MEM_E),
    .RF_E(RF_E),
    .ID_AM(ID_AM)
  );

  // Set de instrucciones
  reg [31:0] instruction_set [0:12];
  integer i;  // Declaración de la variable de control del bucle

  initial begin
    // Cargar el set de instrucciones
    instruction_set[0] = 32'b11100010000100010000000000000000;
    instruction_set[1] = 32'b11100000100000000101000110000011;
    instruction_set[2] = 32'b11100111110100010010000000000000;
    instruction_set[3] = 32'b11100101100010100101000000000000;
    instruction_set[4] = 32'b00011010111111111111111111111101;
    instruction_set[5] = 32'b11011011000000000000000000001001;
    instruction_set[6] = 32'b11100010000000010000000000000000;
    instruction_set[7] = 32'b11100000100000000101000110000011;
    instruction_set[8] = 32'b11100111110100010010000000000000;
    instruction_set[9] = 32'b11100101100010100101000000000000;
    instruction_set[10] = 32'b00011010111111111111111111111101;
    instruction_set[11] = 32'b00000000000000000000000000000000;
    instruction_set[12] = 32'b00000000000000000000000000000000;

    // Ejecución de las pruebas para cada instrucción con S en 0 y 1
    $display("Inicio de pruebas para ControlUnit y Multiplexer");

    // Iterar sobre el set de instrucciones
    for (i = 0; i < 13; i = i + 1) begin
      // Probar con S = 0
      instruction = instruction_set[i];
      S = 1'b0;
      #10;
      displayMuxOutputs(i, S);

      // Probar con S = 1
      S = 1'b1;
      #10;
      displayMuxOutputs(i, S);
    end

    $display("Fin de pruebas para ControlUnit y Multiplexer");
    $stop;
  end

  // Tarea para mostrar las salidas del MUX con un mensaje descriptivo
  task displayMuxOutputs;
    input integer inst_num;
    input S;
    begin
      $display("Resultados para Instrucción %0d con S=%b:", inst_num, S);
      $display("ALU_OP: %b", alu_op);
      $display("id_load: %b", id_load);
      $display("id_mem_write: %b", id_mem_write);
      $display("store_cc: %b", store_cc);
      $display("id_b: %b", id_b);
      $display("id_bl: %b", id_bl);
      $display("id_mem_size: %b", id_mem_size);
      $display("id_mem_e: %b", id_mem_e);
      $display("rf_e: %b", rf_e);
      $display("id_am: %b", id_am);
      $display("-----------------------------");
    end
  endtask

endmodule
