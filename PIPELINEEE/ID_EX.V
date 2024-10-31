module ID_EX (
    input clk
    input reset
    input ID_ALU_OP
    input ID_LOAD
    input ID_MEM_WRITE
    input ID_MEM_SIZE
    input ID_MEM_WRITE
    input ID_MEM_ENABLE
    input ID_AM
    input STORE_CC
    input ID_BL
    input ID_B
    input RF_ENABLE

    output id_alu_op
    output id_load
    output id_mem_write
    output id_mem_size
    output id_mem_write
    output id_mem_enable
    output id_am
    output store_cc
    output id_bl
    output id_b
    output rf_enable

);
    always(posedge clk or posedge reset) begin
        if(reset)begin
            ID_ALU_OP <= 0'b0;
            ID_LOAD <= 0'b0;
            ID_MEM_WRITE <= 0'b0;
            ID_MEM_SIZE <= 0'b0;
            ID_MEM_WRITE <= 0'b0;
            ID_MEM_ENABLE <= 0'b0;
            ID_AM <= 0'b0;
            STORE_CC <= 0'b0; 
            ID_BL <= 0'b0;
            ID_B <= 0'b0;
            RF_ENABLE <= 0'b0;
        end
        else 
            id_alu_op <= ID_ALU_OP;
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            id_am <= ID_AM;
            store_cc <= STORE_CC;
            id_bl <= ID_BL;
            id_b <= ID_B;
            rf_enable <= RF_ENABLE;
    end
endmodule