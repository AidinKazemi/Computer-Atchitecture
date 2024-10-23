`timescale 1ns/1ns
module tb();
reg [9:0] data_A,data_B;
reg clk = 0,start = 0,sclr = 0;
wire dvz,ovf,busy,valid;
wire[9:0] q;
divider the_div(clk,sclr,start,data_A,data_B,valid,ovf,dvz,busy,q);
always #5 clk = ~clk;
initial
begin
    #4 start=1;
    #3 data_A=10'b0100000000;data_B=10'b0;start=0;
    #30 start=1;
    #10 data_A=10'b0010010000;data_B=10'b0000110000;start=0;
    #300 start = 0; sclr = 1;
    #10 sclr = 0;
    #30 start = 1;
    #10 data_A=10'b0010010000;data_B=10'b0000110000;start=0;
    #610 start = 1;
    #10 data_A=10'b0001001000;data_B=10'b0000011000;start=0;
    #610 start = 1;
    #10 data_A=10'b0000001000;data_B=10'b0001001000;start=0;
    #610 start = 1;
    #10 data_A=10'b1111111111;data_B=10'b0000000001;start=0;
    #610 start=1;
    #10 data_A=10'b0110010001;data_B=10'b0100000011;start=0;
    #610 start=1;
    #10 data_A=10'b0110000010;data_B=0110000010;start=0;
    #10 start=1;
    #610 data_A=10'b0101000010;data_B=0110000100;start=0;
    #10 start=1;
    #610 data_A=10'b111110101; data_B=10'b101101110;start=0;
    #10 start=1;
    #610 data_A=10'b111110101; data_B=10'b101101110;start=0;
    #600 start=1;
    #800 repeat(15) begin #800 data_A=$random;data_B=$random; 
    end
    #1000 $stop;
end

endmodule

