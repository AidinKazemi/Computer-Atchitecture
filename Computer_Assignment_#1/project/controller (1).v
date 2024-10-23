`timescale 1ns/1ns
module controller(clk,sclr,start,ld_A,ld_B,
                dvz,ovf,co,cnt_en,ld_acc,ld_q,ld_acc_next,ld_q_next,mux_ld,
                mux_init,busy,valid,ld_counter,dvz_flag,ovf_flag);
    input start,dvz,co,ovf,clk,sclr;
    output reg ld_A,busy,valid,ld_counter,ld_B,cnt_en,ld_q,ovf_flag,
                ld_acc,ld_acc_next,ld_q_next,mux_init,mux_ld,dvz_flag;
    reg [3:0] ps = 0,ns = 0;
    parameter show_dvz = 8,show_ovf = 9;
    always @(ps or start or dvz or co or ovf) begin
    case (ps)
        0:ns=start?1:0;
        1:ns=2;
        2:ns=dvz?show_dvz:3;
        3:ns=4;
        4:ns=ovf?show_ovf:5;
        5:ns=6;
        6:ns=co?7:3;
        7:ns=0;
        show_dvz:ns=0;
        show_ovf:ns=0;
        default:ns=3'd0; 
    endcase 
    end
    always @(ps) begin
        {ld_A,valid,ld_B,cnt_en,ld_acc,ld_q,ld_acc_next,dvz_flag,
        ld_q_next,mux_ld,mux_init,mux_ld,ld_counter,ovf_flag} = 0;
        busy = 1;
        case (ps)
            0: busy = 0;
            1:{ld_B,ld_A}=2'b11;
            2:{ld_counter,ld_acc,ld_q,mux_init}=4'b1111;
            3:{ld_acc_next,ld_q_next}=2'b11;
            4:;
            5:{ld_acc,ld_q,mux_ld}=3'b111;
            6:cnt_en=1;
            7:valid=1;
            show_dvz: dvz_flag = 1;
            show_ovf: ovf_flag = 1;
            default:;
        endcase 
    end
    always @(posedge clk) begin
        if(sclr)
            ps<=3'b0;
        else
            ps<=ns;
        
    end

endmodule

