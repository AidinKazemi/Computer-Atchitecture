`timescale 1ps/1ps

module tb ();

    reg clk = 0,rst = 0;

    assign #5 clk = ~clk;
    cpu cp(clk,rst);
    initial begin
        #5 rst = 1;
        #5 rst = 0;
        #2000 $stop;
    end
endmodule