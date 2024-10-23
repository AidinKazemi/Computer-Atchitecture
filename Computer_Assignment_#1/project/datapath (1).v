`timescale 1ns/1ns
module data_path (clk,sclr,A_data_in,B_data_in,ld_A,ld_B,ld_ACC,ld_ACCnext,ld_Q,ld_Qnext,cnt_en,init_cnt
                ,mux_init,mux_data,Q_out,dvz,ovf,cnt_co);
    input wire clk,sclr;
    input wire [9:0] A_data_in,B_data_in;
    input wire ld_A,ld_B,ld_ACC,ld_ACCnext,ld_Q,ld_Qnext;
    input wire cnt_en,init_cnt;
    input wire mux_init,mux_data;
    output wire dvz,ovf,cnt_co;
    output wire [9:0] Q_out;


    wire ACC_lt_B;
    wire[9:0] input_Q_next,input_Q;
    wire[9:0] output_Q_next,output_Q,output_A,output_B;
    wire[10:0] output_ACC_next,output_ACC,input_ACC_next,input_ACC;
    wire[10:0] sub_result;
    wire [3:0] cnt_out;

    my_reg #(.SIZE(10)) reg_A(.clk(clk),.sclr(sclr),.ld(ld_A),
                        .data_in(A_data_in),.reg_out(output_A));
    my_reg #(.SIZE(10)) reg_B(.clk(clk),.sclr(sclr),.ld(ld_B),
                        .data_in(B_data_in),.reg_out(output_B));
    my_reg #(.SIZE(11)) reg_ACC(.clk(clk),.sclr(sclr),.ld(ld_ACC),
                        .data_in(input_ACC),.reg_out(output_ACC));
    my_reg #(.SIZE(10)) reg_Q(.clk(clk),.sclr(sclr),.ld(ld_Q),
                        .data_in(input_Q),.reg_out(output_Q));
    my_reg #(.SIZE(11)) reg_ACCnext(.clk(clk),.sclr(sclr),.ld(ld_ACCnext),
                                    .data_in(input_ACC_next),.reg_out(output_ACC_next));
    my_reg #(.SIZE(10)) reg_Qnext(.clk(clk),.sclr(sclr),.ld(ld_Qnext),
                        .data_in(input_Q_next),.reg_out(output_Q_next));
    
    my_subtractor sub(.inp1(output_ACC),.inp2({1'b0,output_B}),.subresult(sub_result));
    my_comparetor comp(.inp1(output_ACC),.inp2({1'b0,output_B}),.one_little_than_two(ACC_lt_B));
    my_counter #(.data_default(2)) cntr(.clk(clk),.sclr(sclr),
                .init_cnt(init_cnt),.cnt_en(cnt_en),.co(cnt_co),.cnt_out(cnt_out));

    assign {input_ACC_next,input_Q_next} = ACC_lt_B ? {output_ACC[9:0],output_Q,1'b0} : 
                                                {sub_result[9:0],output_Q,1'b1};
    assign {input_ACC,input_Q} = mux_init ? {10'b0,output_A,1'b0} : 
                                    mux_data ? {output_ACC_next,output_Q_next} : 
                                    {output_ACC,output_Q};
    assign dvz = ~|output_B;
    assign ovf = (cnt_out == 4'b1011) & (|output_Q_next[9:4]);
    assign Q_out = output_Q;
    always @(posedge clk) begin
        $display("q_out = %d,acc_out = %d ; input_Q_next = %d ; acc_lt_b = %d ; subresult = %d",
         output_Q, output_ACC,input_Q_next, ACC_lt_B ,sub_result);
    end
endmodule

module my_reg #(parameter SIZE = 10) (clk,sclr,ld,data_in,reg_out);
    input wire clk,sclr,ld;
    input wire [SIZE - 1 : 0] data_in;
    output reg [SIZE - 1 : 0] reg_out;

    always @(posedge clk) begin
        if (sclr) reg_out <= 0;
        else if (ld) reg_out <= data_in;
    end
endmodule

module my_comparetor (inp1,inp2,one_little_than_two);
    input wire[10:0] inp1,inp2;
    output wire one_little_than_two;

    assign one_little_than_two = inp1 < inp2 ? 1 : 0;
endmodule



module my_subtractor (inp1,inp2,subresult);
    input wire[10:0] inp1,inp2;
    output wire [10:0] subresult;

    assign subresult = inp1 + ~inp2 + 1;
endmodule

module my_counter #(parameter data_default = 2) (clk,sclr,init_cnt,cnt_en,co,cnt_out);
    input wire clk,sclr,init_cnt,cnt_en;
    output wire co;
    output reg [3:0] cnt_out;

    always @(posedge clk) begin
        if (sclr) cnt_out <= 0;
        else begin
            if (init_cnt) cnt_out <= data_default;
            else if(cnt_en) cnt_out <= cnt_out + 1;
        end
    end

    assign co = &cnt_out;
          

endmodule