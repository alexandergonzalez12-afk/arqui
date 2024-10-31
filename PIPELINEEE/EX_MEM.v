module EX_MEM(
    input clk
    input reset
 
    input ID_LOAD
    input ID_MEM_WRITE
    input ID_MEM_SIZE
    input ID_MEM_ENABLE
    input RF_ENABLE


    output id_load
    output id_mem_size
    output id_mem_write
    output id_mem_enable
    output rf_enable

);
    always(posedge clk or posedge reset) begin
        if(reset)begin

            ID_LOAD <= 0'b0;
            ID_MEM_WRITE <= 0'b0;
            ID_MEM_SIZE <= 0'b0;
            ID_MEM_ENABLE <= 0'b0;
            RF_ENABLE <= 0'b0;
        end
        else 
            id_load <= ID_LOAD;
            id_mem_write <= ID_MEM_WRITE;
            id_mem_size <= ID_MEM_SIZE;
            id_mem_enable <= ID_MEM_ENABLE;
            rf_enable <= RF_ENABLE;
    end
endmodule