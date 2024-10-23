`timescale 1ns/1ns
module divider (clk,sclr,start,data_A,data_B,valid,ovf_flag,dvz_flag,busy,answer);
    input wire clk,sclr,start;
    input wire [9:0] data_A,data_B;
    output wire ovf_flag,dvz_flag,busy,valid;
    output wire [9:0] answer;

    wire ld_cnt,mux_init,mux_data,ld_q_next,ld_acc_next,ld_q,ld_acc,cnt_en,ld_a,ld_b,co,dvz,ovf;
    controller the_controler(.clk(clk),.sclr(sclr),.start(start),.ld_A(ld_a),.ld_B(ld_b),
        .dvz(dvz),.ovf(ovf),.co(co),.cnt_en(cnt_en),.ld_acc(ld_acc),.ld_q(ld_q),
        .ld_acc_next(ld_acc_next),.ld_q_next(ld_q_next),.mux_ld(mux_data),
        .mux_init(mux_init),.busy(busy),.valid(valid),.ld_counter(ld_cnt)
        ,.dvz_flag(dvz_flag),.ovf_flag(ovf_flag));

    data_path the_data_path(.clk(clk),.sclr(sclr),.A_data_in(data_A),.B_data_in(data_B),
        .ld_A(ld_a),.ld_B(ld_b),.ld_ACC(ld_acc),.ld_ACCnext(ld_acc_next),
        .ld_Q(ld_q),.ld_Qnext(ld_q_next),.cnt_en(cnt_en),.init_cnt(ld_cnt),
        .mux_init(mux_init),.mux_data(mux_data),.Q_out(answer),.dvz(dvz),.ovf(ovf),.cnt_co(co));
endmodule