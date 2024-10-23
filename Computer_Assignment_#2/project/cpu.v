`timescale 1ps/1ps

module cpu (clk,rst);
    input wire clk,rst;

    wire MemWrite,ALUSrc,RegWrite;
    wire ResultSrc;
    wire [1:0] Sel,PCSrc;
    wire [2:0] ALUControl,ImmSrc;
    wire [6:0] OPCode,Func7;
    wire [2:0] Func3;
    wire Zero,ALU_msb;

    datapath dp (.clk(clk),.rst(rst),.PCSrc(PCSrc),.ResultSrc(ResultSrc),.MemWrite(MemWrite),.ALUControl(ALUControl),
            .ALUSrc(ALUSrc),.ImmSrc(ImmSrc),.RegWrite(RegWrite),.OPCode(OPCode),.Sel(Sel),
                    .Func3(Func3),.Func7(Func7),.Zero(Zero),.ALU_msb(ALU_msb));

    controller ct(.PCSrc(PCSrc),.ResultSrc(ResultSrc),.MemWrite(MemWrite),.ALUControl(ALUControl),
            .ALUSrc(ALUSrc),.ImmSrc(ImmSrc),.RegWrite(RegWrite),.OPCode(OPCode),.Sel(Sel),
                    .Func3(Func3),.Func7(Func7),.Zero(Zero),.ALU_msb(ALU_msb));
endmodule