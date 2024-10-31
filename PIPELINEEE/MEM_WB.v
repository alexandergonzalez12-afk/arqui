module MEM_WB (
    input clk
    input reset
    input RF_ENABLE

    output rf_enable

);
    always(posedge clk or posedge reset) begin
        if(reset)begin
            RF_ENABLE <= 0'b0;
        end
        else 
            rf_enable <= RF_ENABLE;
    end
endmodule