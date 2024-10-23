`timescale 1ps/1ps

module cpu (clk,rst);
        input wire clk,rst;

        wire [6:0] OPCodeD,Func7D;
        wire [2:0] Func3D,Func3E;

        wire PCSrc,MemWriteD,ALUSrcD,RegWriteD,Zero,ALU_msb,flushE,jumpD,branchD,jalr_sel,stallD,flushD,stallF,jumpE,branchE;
        wire [1:0] ResultSrcD,forwardBE,forwardAE;
        wire [2:0] ALUControlD,ImmSrc;
        wire res_src_0,RegWriteW,RegWriteM;
        wire [4:0] rs1D,rs2D,RdE,rs2E,rs1E,rdM,rdw;

        controller ct (.PCSrc(PCSrc),.ResultSrc(ResultSrcD),.MemWrite(MemWriteD),.ALUControl(ALUControlD),.ALUSrc(ALUSrcD),
                        .ImmSrc(ImmSrc),.RegWrite(RegWriteD),.OPCodeD(OPCodeD),
                        .Func3D(Func3D),.Func7D(Func7D),.Zero(Zero),.ALU_msb(ALU_msb),.jumpD(jumpD),.branchD(branchD),
                        .jumpE(jumpE),.branchE(branchE),.Func3E(Func3E),.jalr_sel(jalr_sel));

        datapath dp (.clk(clk),.rst(rst),.PCSrc(PCSrc),.ResultSrcD(ResultSrcD),.MemWriteD(MemWriteD),.ALUControlD(ALUControlD),
                        .ALUSrcD(ALUSrcD),.ImmSrc(ImmSrc),.RegWriteD(RegWriteD),.OPCodeD(OPCodeD),
                        .func3_inD(Func3D),.Func7D(Func7D),.Zero(Zero),.ALU_msb(ALU_msb),.flushE(flushE),.jumpD(jumpD),
                        .branchD(branchD),.forwardBE(forwardBE),.forwardAE(forwardAE),.jalr_sel(jalr_sel),
                        .stallD(stallD),.flushD(flushD),.stallF(stallF),.Func3E(Func3E),.jumpE(jumpE),.branchE(branchE),
                        .rs2D(rs2D),.RdE(RdE),.rs2E(rs2E),.rs1E(rs1E),
                        .res_src_0(res_src_0),.rdM(rdM),.rdw(rdw),.rs1D(rs1D),
                        .RegwriteW(RegwriteW),.RegwriteM(RegwriteM));

        hazard_controller hzct(.clk(clk),.stallF(stallF),.stallD(stallD),.flushE(flushE),.flushD(flushD),.forwardAe(forwardAE),.forwardBe(forwardBE),
                                .rs2D(rs2D),.RdE(RdE),.rs2E(rs2E),.rs1E(rs1E),.pcsrc(PCSrc),
                                .res_src_0(res_src_0),.rdM(rdM),.rdw(rdw),.rs1D(rs1D),
                                .regwriteW(RegwriteW),.regwriteM(RegwriteM));

endmodule