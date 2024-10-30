`timescale 1ns / 1ns

module Control_Unit (
  input [31:0] instruction,
  output reg [3:0] ID_ALU_op,
  output reg ID_Load,
  output reg ID_MEM_WRITE,
  output reg ID_AM,
  output reg STORE_CC,
  output reg ID_BL,
  output reg [1:0] ID_MEM_SIZE,
  output reg ID_B,
  output reg ID_MEM_ENABLE,
  output reg RF_ENABLE
);

  always @(*) begin
    case (instruction[31:26])
      6'b000000: begin
        ID_ALU_op <= 4'b0010;
        ID_Load <= 0;
        ID_MEM_WRITE <= 0;
        ID_AM <= 0;
        STORE_CC <= 0;
        ID_BL <= 0;
        ID_MEM_SIZE <= 2'b00;
        ID_B <= 0;
        ID_MEM_ENABLE <= 0;
        RF_ENABLE <= 1;
      end
      6'b100011: begin
        ID_ALU_op <= 4'b0000;
        ID_Load <= 1;
        ID_MEM_WRITE <= 0;
        ID_AM <= 0;
        STORE_CC <= 0;
        ID_BL <= 0;
        ID_MEM_SIZE <= 2'b10;
        ID_B <= 0;
        ID_MEM_ENABLE <= 1;
        RF_ENABLE <= 1;
      end
      6'b101011: begin
        ID_ALU_op <= 4'b0000;
        ID_Load <= 0;
        ID_MEM_WRITE <= 1;
        ID_AM <= 0;
        STORE_CC <= 0;
        ID_BL <= 0;
        ID_MEM_SIZE <= 2'b10;
        ID_B <= 0;
        ID_MEM_ENABLE <= 1;
        RF_ENABLE <= 0;
      end
      6'b000100: begin
        ID_ALU_op <= 4'b0001;
        ID_Load <= 0;
        ID_MEM_WRITE <= 0;
        ID_AM <= 0;
        STORE_CC <= 0;
        ID_BL <= 0;
        ID_MEM_SIZE <= 2'b00;
        ID_B <= 1;
        ID_MEM_ENABLE <= 0;
        RF_ENABLE <= 0;
      end
      default: begin
        ID_ALU_op <= 4'b0000;
        ID_Load <= 0;
        ID_MEM_WRITE <= 0;
        ID_AM <= 0;
        STORE_CC <= 0;
        ID_BL <= 0;
        ID_MEM_SIZE <= 2'b00;
        ID_B <= 0;
        ID_MEM_ENABLE <= 0;
        RF_ENABLE <= 0;
      end
    endcase
  end
endmodule
