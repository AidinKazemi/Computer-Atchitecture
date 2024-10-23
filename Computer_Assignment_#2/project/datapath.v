`timescale 1ps/1ps

module datapath (clk,rst,PCSrc,ResultSrc,MemWrite,ALUControl,ALUSrc,ImmSrc,RegWrite,OPCode,Sel,
                    Func3,Func7,Zero,ALU_msb);
    input wire clk,rst;
    input wire MemWrite,ALUSrc,RegWrite;
    input wire ResultSrc;
    output wire [1:0] Sel,PCSrc;
    input wire [2:0] ALUControl,ImmSrc;
    output wire [6:0] OPCode,Func7;
    output wire [2:0] Func3;
    output wire Zero,ALU_msb;

    wire[31:0] pc_input,pc_output,instruction,RD_1,RD_2,Imm_out,ALU_inp,ALU_out,RD,WD;
    wire[31:0] WD_final,pc_plus_4,pc_plus_offset,pc_input_final;

    Multiplexer_4_to_1 #(.inout_size(32)) pc_mux (.select_signal(PCSrc),.input_0(pc_plus_4),
                    .input_1(pc_plus_offset),.input_2(WD),.input_3(0),.output_value(pc_input));

    Register pc_reg (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(pc_input),.output_value(pc_output)
                    ,.last_instruction(instruction));

    assign pc_plus_4 = pc_output + 4;

    InstructionMemory inst_mem  (.A(pc_output),.RD(instruction));

    RegisterFile RF (.clk(clk),.A_1(instruction[19:15]),.A_2(instruction[24:20]),.A_3(instruction[11:7]),
                    .write_en(RegWrite),.write_data(WD_final),.RD_1(RD_1),.RD_2(RD_2));

    Extend_unit im_ex (.input_A(instruction[31:7]),.select_signal(ImmSrc),.output_value(Imm_out));

    assign pc_plus_offset = pc_output + Imm_out;

    Multiplexer_2_to_1 #(.inout_size(32)) alu_mux (.select_signal(ALUSrc),.input_0(RD_2),
                            .input_1(Imm_out),.output_value(ALU_inp));

    ALU alu (.input_A(RD_1),.input_B(ALU_inp),.select_signal(ALUControl),
            .output_value(ALU_out),.zero(Zero),.alu_msb(ALU_msb));

    DataMemory DM (.clk(clk),.A(ALU_out),.write_en(MemWrite),.write_data(RD_2),.RD(RD));

    Multiplexer_2_to_1 #(.inout_size(32)) res_mux (.select_signal(ResultSrc),.input_1(RD),
                            .input_0(ALU_out),.output_value(WD));


    Multiplexer_4_to_1 #(.inout_size(32)) final_res_mux (.select_signal(Sel),.input_0(WD),
                .input_1(pc_plus_4),.input_2(Imm_out),.input_3(0),.output_value(WD_final));

    assign OPCode = instruction[6:0];
    assign Func3 = instruction[14:12];
    assign Func7 = instruction[31:25];

endmodule

module Multiplexer_4_to_1 #(parameter inout_size) 
        (select_signal,input_0,input_1,input_2,input_3,output_value);
    input wire [1 : 0] select_signal;
    input wire [inout_size - 1 : 0] input_0,input_1,input_2,input_3; 
    output reg [inout_size - 1 : 0] output_value; 
    assign output_value = (select_signal == 2'd0) ? input_0:
                           (select_signal == 2'd1) ? input_1:
                           (select_signal == 2'd2) ? input_2:
                           input_3;
endmodule

module Multiplexer_2_to_1 #(parameter inout_size) 
        (select_signal,input_0,input_1,output_value);
    input wire select_signal;
    input wire [inout_size - 1 : 0] input_0,input_1; 
    output reg [inout_size - 1 : 0] output_value; 
    assign output_value = (select_signal == 1'b0) ? input_0:
                           input_1;
endmodule

module ALU (input_A,input_B,select_signal,output_value,zero,alu_msb);
    input wire [2:0] select_signal;
    input wire [31 : 0] input_A,input_B; 
    output reg [31 : 0] output_value; 
    output reg zero,alu_msb;

    assign output_value = (select_signal == 3'd0) ? input_A + input_B:
                   (select_signal == 3'd1) ? input_A + ~{input_B} + 1:
                   (select_signal == 3'd2) ? input_A & input_B:
                   (select_signal == 3'd3) ? input_A | input_B:
                   (select_signal == 3'd4) ? 
                   (
                        (input_A[31] == input_B[31]) ?
                        (
                            (input_A[31] == 1) ? ~{input_A} + 1 > ~{input_B} + 1:
                            input_A < input_B
                        ):
                        (input_A[31] == 1) ? 1: 0
                   ):
                   (select_signal == 3'd5) ? (input_A < input_B):
                   (select_signal == 3'd6) ? (input_A ^ input_B):
                   0;

    assign zero = (input_A == input_B) & (select_signal == 1);
    assign alu_msb = output_value[31];
endmodule

module Extend_unit (input_A,select_signal,output_value);
    input wire [2:0] select_signal;
    input wire [24 : 0] input_A; 
    output reg [31 : 0] output_value; 

    assign output_value = (select_signal == 3'd0) ? {({input_A[24]} ? 20'b11111111111111111111 : 20'd0),
                                                        input_A[24:13]}: // I
                   (select_signal == 3'd1) ? {({input_A[24]} ? 20'b11111111111111111111 : 20'd0),
                                            input_A[24:18],input_A[4:0]}: // S
                   (select_signal == 3'd2) ? {({input_A[24]} ? 19'b1111111111111111111 : 19'd0),
                                                input_A[24],input_A[0],
                                                input_A[23:18],input_A[4:1],1'b0}: // B
                   (select_signal == 3'd3) ? {input_A[24:5],12'd0}: // U
                   (select_signal == 3'd4) ? {({input_A[24]} ? 11'b11111111111 : 11'd0),input_A[24],
                                             input_A[12:5],input_A[13],input_A[23:14],1'b0}: // J
                    0; 
endmodule

module Register (clk,rst,write_en,load_val,output_value,last_instruction);
    input wire clk,write_en,rst;
    input wire [31:0] load_val,last_instruction;
    output reg [31:0] output_value;

    always @(posedge clk,posedge rst) begin
        if (rst)
            output_value <= 0;
        else begin if(write_en & ~(last_instruction == 0))
            output_value <= load_val;
        end
    end
    
endmodule

module RegisterFile (clk,A_1,A_2,A_3,write_en,write_data,RD_1,RD_2);
    input wire clk,write_en;
    input wire [4:0] A_1,A_2,A_3;
    input wire [31:0] write_data;
    output reg [31:0] RD_1,RD_2;

    reg [31:0] register_space [31:0];

    assign register_space[0] = 0;

    always @(posedge clk) begin
        if(write_en & ~(A_3 == 0))
            register_space[A_3] <= write_data;
    end

    assign {RD_1,RD_2} = {register_space[A_1],register_space[A_2]};

endmodule

module DataMemory (clk,A,write_en,write_data,RD);
    input wire clk,write_en;
    input wire [31:0] A;
    input wire [31:0] write_data;
    output reg [31:0] RD;

    reg [7:0] register_space [50:0];

    assign {register_space[3], register_space[2], register_space[1], register_space[0]} = 32'd45;
    assign {register_space[7], register_space[6], register_space[5], register_space[4]} = 32'd23;
    assign {register_space[11], register_space[10], register_space[9], register_space[8]} = 32'd17;
    assign {register_space[15], register_space[14], register_space[13], register_space[12]} = 32'd92;
    assign {register_space[19], register_space[18], register_space[17], register_space[16]} = 32'd78;
    assign {register_space[23], register_space[22], register_space[21], register_space[20]} = 32'd61;
    assign {register_space[27], register_space[26], register_space[25], register_space[24]} = 32'd117;
    assign {register_space[31], register_space[30], register_space[29], register_space[28]} = 32'd72;
    assign {register_space[35], register_space[34], register_space[33], register_space[32]} = 32'd56;
    assign {register_space[39], register_space[38], register_space[37], register_space[36]} = 32'd125;

    always @(posedge clk) begin
        if(write_en)
            {register_space[A+3],register_space[A+2],
            register_space[A+1],register_space[A]} <= write_data;
    end

    assign RD = {register_space[A+3],register_space[A+2],
                register_space[A+1],register_space[A]};
endmodule

module InstructionMemory (A,RD);
    input wire [31:0] A;
    output reg [31:0] RD;

    reg [31:0] memory_space [100:0];

    assign memory_space[0] = 32'b00000000000000000000010000110011;
    assign memory_space[4] = 32'b00000000000001000000010010000011;
    assign memory_space[8] = 32'b00000000010000000000001100010011;
    assign memory_space[12] = 32'b00000010100000110010111000010011;
    assign memory_space[16] = 32'b00000010000011100000000101100011;
    assign memory_space[20] = 32'b00000000011000000000010000110011;
    assign memory_space[24] = 32'b00000000000001000000111010000011;
    assign memory_space[28] = 32'b00000000100111101100100001100011;
    assign memory_space[32] = 32'b00000000010000110000001100010011;
    assign memory_space[36] = 32'b00000000000011101000010010110011;
    assign memory_space[40] = 32'b11111110010111111111111111101111;
    assign memory_space[44] = 32'b00000000010000110000001100010011;
    assign memory_space[48] = 32'b11111101110111111111111111101111;
     assign memory_space[52] = 32'd0;

    assign RD = memory_space[A];
endmodule





