`timescale 1ps/1ps

module datapath (clk,rst,PCSrc,ResultSrcD,MemWriteD,ALUControlD,ALUSrcD,ImmSrc,RegWriteD,OPCodeD,
                    func3_inD,Func7D,Zero,ALU_msb,flushE,jumpD,branchD,forwardBE,forwardAE,jalr_sel,
                    stallD,flushD,stallF,Func3E,jumpE,branchE,
                    res_src_0,rdM,rdw,RegwriteM,rs1D,rs2D,RdE,rs2E,rs1E,RegwriteW);
    input wire clk,rst;
    input wire jalr_sel;
    input wire MemWriteD,ALUSrcD,RegWriteD,jumpD,branchD;
    input wire [1:0] ResultSrcD,forwardBE,forwardAE;
    input wire PCSrc;
    input wire [2:0] ALUControlD,ImmSrc;
    output wire [6:0] OPCodeD,Func7D;
    output wire [2:0] func3_inD,Func3E;
    output wire Zero,ALU_msb,jumpE,branchE;
    input wire flushE,stallD,flushD,stallF;
    output wire res_src_0,RegwriteM,RegwriteW;
    output wire [4:0] rs1D,rs2D,RdE,rs2E,rs1E,rdM,rdw;

    wire MemWriteM ,MemWriteE, memwriteW,RegWriteE;

    //memwrite regs
    wire[31:0] WD_final,pc_plus_4D,pc_plus_offset,pc_input_final;
    wire ALUSrcE;

    Register  #(.inout_size(1)) memwriteDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(MemWriteD),.output_value(MemWriteE));
    Register #(.inout_size(1))  memwriteEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(MemWriteE),.output_value(MemWriteM));
    // Register  #(.inout_size(1)) memwriteMW (.clk(clk),.rst(rst),.write_en(1),.load_val(MemWriteM),.output_value(MemWriteW));

    //regwrite reg
    Register  #(.inout_size(1)) RegWriteDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(RegWriteD),.output_value(RegWriteE));
    Register #(.inout_size(1))  RegWriteEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(RegWriteE),.output_value(RegwriteM));
    Register  #(.inout_size(1)) RegWriteMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(RegwriteM),.output_value(RegwriteW));
    
    //regBranch
    Register  #(.inout_size(1)) branchDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(branchD),.output_value(branchE));
    
    //regJump
    Register  #(.inout_size(1)) jumpDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(jumpD),.output_value(jumpE));

   //regALUSRC
    Register  #(.inout_size(1)) ALUSrcDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(ALUSrcD),.output_value(ALUSrcE));
    
    wire[2:0] ALUControlE;
    //regALUcontrol
    Register #(.inout_size(3)) ALUControlDE  (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(ALUControlD),.output_value(ALUControlE));
    
    wire[1:0] ResultSrcE,ResultSrcW,ResultSrcM;
    //regResultsrc
    Register #(.inout_size(2)) ResultSrcDE  (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(ResultSrcD),.output_value(ResultSrcE));
    Register #(.inout_size(2)) ResultSrcEM  (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(ResultSrcE),.output_value(ResultSrcM));
    Register #(.inout_size(2)) ResultSrcMW  (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(ResultSrcM),.output_value(ResultSrcW));

    wire[31:0] pc_inputD,pc_inputE,pc_inputF,pc_inputM,pc_inputW;

    Register #(.inout_size(32) ) pcRegFD (.clk(clk),.rst(flushD | rst),.write_en(~stallD),.load_val(pc_inputF),.output_value(pc_inputD));
    Register #(.inout_size(32) ) pcRegDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(pc_inputD),.output_value(pc_inputE));
    // Register #(.inout_size(32) ) pcRegEM (.clk(clk),.rst(rst),.write_en(1),.load_val(pc_inputE),.output_value(pc_inputM));
    // Register #(.inout_size(32) ) pcRegMW (.clk(clk),.rst(rst),.write_en(1),.load_val(pc_inputM),.output_value(pc_inputW));

    wire [31:0] pc_plus_4F,pc_plus_4E,pc_plus_4M,pc_plus_4W;

    Register #(.inout_size(32) ) pcRegplusFD (.clk(clk),.rst(flushD | rst),.write_en(~stallD),.load_val(pc_plus_4F),.output_value(pc_plus_4D));
    Register #(.inout_size(32) ) pcRegplusDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(pc_plus_4D),.output_value(pc_plus_4E));
    Register #(.inout_size(32) ) pcRegplusEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(pc_plus_4E),.output_value(pc_plus_4M));
    Register #(.inout_size(32) ) pcRegplusMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(pc_plus_4M),.output_value(pc_plus_4W));


    wire[31:0] instruction,RD_1D,RD_2D,RD_1E,RD_2E,Imm_out,ALU_inp,ALU_out,RD,WD,pc_input;

    Multiplexer_2_to_1 #(.inout_size(32)) pc_mux (.select_signal(PCSrc),.input_0(pc_plus_4F),
                    .input_1(pc_plus_offset),.output_value(pc_input));

    // Register_ pc_reg (.clk(clk),.rst(rst),.write_en(~stallF),.load_val(pc_input),.output_value(pc_inputF)
    //                 ,.last_instruction(instruction));

    Register #(.inout_size(32) ) pc_reg (.clk(clk),.rst(rst),.write_en(~stallF),.load_val(pc_input),.output_value(pc_inputF));


    wire[31:0] instructionF;

    InstructionMemory inst_mem  (.A(pc_inputF),.RD(instructionF));
    Register #(.inout_size(32) ) inst_reg (.clk(clk),.rst(flushD | rst),.write_en(~stallD),.load_val(instructionF),.output_value(instruction));


    assign pc_plus_4F = pc_inputF + 4;
    wire [4:0] dest_regE,dest_regW,dest_regM;
    wire [4:0] dest_regD=instruction[11:7];
    
    Register #(.inout_size(5) ) destRegDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(dest_regD),.output_value(dest_regE));
    Register #(.inout_size(5) ) destRegEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(dest_regE),.output_value(dest_regM));
    Register #(.inout_size(5) ) destRegMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(dest_regM),.output_value(dest_regW));

    RegisterFile RF (.clk(clk),.A_1(instruction[19:15]),.A_2(instruction[24:20]),.A_3(dest_regW),
                    .write_en(RegwriteW),.write_data(WD),.RD_1(RD_1D),.RD_2(RD_2D));

    wire [4:0] rs1_E,rs2_E;

    Register #(.inout_size(5) ) rs1regDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(instruction[19:15]),.output_value(rs1_E));
    Register #(.inout_size(5) ) rs2regDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(instruction[24:20]),.output_value(rs2_E));

    Register #(.inout_size(32) ) rd1regDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(RD_1D),.output_value(RD_1E));
    Register #(.inout_size(32) ) rd2regDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(RD_2D),.output_value(RD_2E));


    wire[31:0] Imm_outD,Imm_outE,Imm_outM,Imm_outW;

    Extend_unit im_ex (.input_A(instruction[31:7]),.select_signal(ImmSrc),.output_value(Imm_outD));

    Register #(.inout_size(32) ) Imm_outDE (.clk(clk),.rst(flushE | rst),.write_en(1'b1),.load_val(Imm_outD),.output_value(Imm_outE));
    Register #(.inout_size(32) ) Imm_outEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(Imm_outE),.output_value(Imm_outM));
    Register #(.inout_size(32) ) Imm_outMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(Imm_outM),.output_value(Imm_outW));

    wire [31:0] mux_jal;
    assign pc_plus_offset = mux_jal + Imm_outE;
    wire [31:0] A_in,B_inE,B_inM;

    wire [31:0] ALU_outE,ALU_outM,ALU_outW;

    Multiplexer_2_to_1 #(.inout_size(32)) alu_mux (.select_signal(ALUSrcE),.input_0(B_inE),
                            .input_1(Imm_outE),.output_value(ALU_inp));
    Multiplexer_2_to_1 #(.inout_size(32)) pc_adder(.select_signal(jalr_sel),.input_0(pc_inputE),
                            .input_1(RD_1E),.output_value(mux_jal));                     


    Multiplexer_4_to_1 #(.inout_size(32)) forwardA (.select_signal(forwardAE),.input_0(RD_1E),
                .input_1(WD),.input_2(ALU_outM),.input_3(0),.output_value(A_in));

    Multiplexer_4_to_1 #(.inout_size(32)) forwardB (.select_signal(forwardBE),.input_0(RD_2E),
                .input_1(WD),.input_2(ALU_outM),.input_3(0),.output_value(B_inE));                         

    Register #(.inout_size(32) ) B_inEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(B_inE),.output_value(B_inM));     

    ALU alu (.input_A(A_in),.input_B(ALU_inp),.select_signal(ALUControlE),
            .output_value(ALU_outE),.zero(Zero),.alu_msb(ALU_msb));

    Register #(.inout_size(32) ) ALU_outEM (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(ALU_outE),.output_value(ALU_outM));
    Register #(.inout_size(32) ) ALU_outMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(ALU_outM),.output_value(ALU_outW));         


    wire[31:0] ReadDataM,ReadDataW;

    DataMemory DM (.clk(clk),.A(ALU_outM),.write_en(MemWriteM),.write_data(B_inM),.RD(ReadDataM));

    Register #(.inout_size(32) ) ReadDataMW (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(ReadDataM),.output_value(ReadDataW));


    Multiplexer_4_to_1 #(.inout_size(32)) res_mux (.select_signal(ResultSrcW),.input_0(ALU_outW),
                .input_1(ReadDataW),.input_2(pc_plus_4W),.input_3(Imm_outW),.output_value(WD));

    // Multiplexer_4_to_1 #(.inout_size(32)) final_res_mux (.select_signal(Sel),.input_0(RD_1E),
    //             .input_1(WD),.input_2(ALU_outE),.input_3(0),.output_value(WD_final));


    assign OPCodeD = instruction[6:0];
    assign func3_inD = instruction[14:12];
    assign Func7D = instruction[31:25];

    Register #(.inout_size(3) ) func3_DE (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(func3_inD),.output_value(Func3E));
    // Register #(.inout_size(7) ) func3_DE (.clk(clk),.rst(rst),.write_en(1'b1),.load_val(Func7D),.output_value(Func7E));

    assign {res_src_0,rdM,rdw,rs1D,rs2D,RdE,rs2E,rs1E} = 
            {ResultSrcE[0],dest_regM,dest_regW,instruction[19:15],instruction[24:20],dest_regE,rs2_E,rs1_E};

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
                   (select_signal == 3'd1) ? input_A + ~{input_B} + 32'd1:
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
                   (select_signal == 3'd5) ? {20'd0,input_A[24:13]}: //SLTiu
                    0;
endmodule

module Register_ (clk,rst,write_en,load_val,output_value,last_instruction);
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

    always @(negedge clk) begin
        if(write_en & ~(A_3 == 0))
            register_space[A_3] <= write_data;
    end

    // always @(posedge clk) begin
    //     {RD_1,RD_2} <= {register_space[A_1],register_space[A_2]};
    // end
    assign {RD_1,RD_2} = {register_space[A_1],register_space[A_2]};

endmodule

// module DataMemory (clk,A,write_en,write_data,RD);
//     input wire clk,write_en;
//     input wire [31:0] A;
//     input wire [31:0] write_data;
//     output reg [31:0] RD;

//     reg [7:0] register_space [50:0];

//     assign {register_space[3], register_space[2], register_space[1], register_space[0]} = 32'd45;
//     assign {register_space[7], register_space[6], register_space[5], register_space[4]} = 32'd23;
//     assign {register_space[11], register_space[10], register_space[9], register_space[8]} = 32'd17;
//     assign {register_space[15], register_space[14], register_space[13], register_space[12]} = 32'd92;
//     assign {register_space[19], register_space[18], register_space[17], register_space[16]} = 32'd78;
//     assign {register_space[23], register_space[22], register_space[21], register_space[20]} = 32'd61;
//     assign {register_space[27], register_space[26], register_space[25], register_space[24]} = 32'd117;
//     assign {register_space[31], register_space[30], register_space[29], register_space[28]} = 32'd72;
//     assign {register_space[35], register_space[34], register_space[33], register_space[32]} = 32'd56;
//     assign {register_space[39], register_space[38], register_space[37], register_space[36]} = 32'd125;

//     always @(posedge clk) begin
//         if(write_en)
//             {register_space[A+3],register_space[A+2],
//             register_space[A+1],register_space[A]} <= write_data;
//     end

//     assign RD = {register_space[A+3],register_space[A+2],
//                 register_space[A+1],register_space[A]};
// endmodule

module DataMemory (clk,A,write_en,write_data,RD);
    input wire clk,write_en;
    input wire [31:0] A;
    input wire [31:0] write_data;
    output reg [31:0] RD;

    reg [7:0] register_space [1000:0];


    initial begin
        $readmemb("the_memory.mem",register_space);
    end
    
    always @(posedge write_en) begin
        if (write_en == 1)
         {register_space[A+3],register_space[A+2],
            register_space[A+1],register_space[A]} = write_en ? write_data : 32'd0 ;
    end

    assign RD = {register_space[A+3],register_space[A+2],
                register_space[A+1],register_space[A]};
endmodule

// module InstructionMemory (A,RD);
//     input wire [31:0] A;
//     output reg [31:0] RD;

//     reg [31:0] memory_space [100:0];

//     assign memory_space[0] = 32'b00000000000000000000010000110011;
//     assign memory_space[4] = 32'b00000000000001000000010010000011;
//     assign memory_space[8] = 32'b00000000010000000000001100010011;
//     assign memory_space[12] = 32'b00000010100000110010111000010011;
//     assign memory_space[16] = 32'b00000010000011100000000101100011;
//     assign memory_space[20] = 32'b00000000011000000000010000110011;
//     assign memory_space[24] = 32'b00000000000001000000111010000011;
//     assign memory_space[28] = 32'b00000000100111101100100001100011;
//     assign memory_space[32] = 32'b00000000010000110000001100010011;
//     assign memory_space[36] = 32'b00000000000011101000010010110011;
//     assign memory_space[40] = 32'b11111110010111111111111111101111;
//     assign memory_space[44] = 32'b00000000010000110000001100010011;
//     assign memory_space[48] = 32'b11111101110111111111111111101111;
//      assign memory_space[52] = 32'd0;

//     assign RD = memory_space[A];
// endmodule

module InstructionMemory (A,RD);
    input wire [31:0] A;
    output reg [31:0] RD;

    reg [7:0] register_space [1000:0];

    initial begin
        $readmemb("the_inst_mem.mem",register_space);
    end

    assign RD = {register_space[A+3],register_space[A+2],
                register_space[A+1],register_space[A]};
endmodule

module Register #(parameter inout_size)  (clk,rst,write_en,load_val,output_value);
    input wire clk,write_en,rst;
    input wire [inout_size - 1 : 0] load_val;
    output reg  [inout_size - 1 : 0] output_value;

    always @(posedge clk,posedge rst) begin
        if (rst)
            output_value <= 0;
        else begin if(write_en)
            output_value <= load_val;
        end
    end
    
endmodule

// 00000000000000000000010000110011
// 00000000000001000000010010000011
// 00000000010000000100001100010011
// 00000010100000110010111000010011
// 00000000000001110000100101100011
// 00000000011001000000010000110011
// 00000000000001000000111010000011
// 00000000100001001100010001100011
// 00000000010000110100001100010011
// 00000000000011101000010010110011
// 00000000101000000000111111101111
// 00000000010000110100001100010011
// 00000000101000000000111111101111



