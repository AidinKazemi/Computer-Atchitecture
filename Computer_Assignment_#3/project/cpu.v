`timescale 1ps/1ps

module cpu (clk,rst);
    input wire clk,rst;

    wire MemWrite,RegWrite,IrWrite,adrSrc,PCwrite;
    wire [1:0] ResultSrc,alusrcA,alusrcB;
    wire [2:0] ALUControl,ImmSrc;
    wire [6:0] OPCode,Func7;
    wire [2:0] Func3;
    wire Zero,ALU_msb;

    datapath dp (.clk(clk),.rst(rst),.PCwrite(PCwrite),.ResultSrc(ResultSrc),.MemWrite(MemWrite),.ALUControl(ALUControl),
            .ImmSrc(ImmSrc),.RegWrite(RegWrite),.OPCode(OPCode),.adrSrc(adrSrc),.IrWrite(IrWrite),
                    .Func3(Func3),.Func7(Func7),.Zero(Zero),.ALU_msb(ALU_msb),.alusrcA(alusrcA),.alusrcB(alusrcB));

    controller ct(.clk(clk),.rst(rst),.ResultSrc(ResultSrc),.MemWrite(MemWrite),.ALUControl(ALUControl),
            .ImmSrc(ImmSrc),.RegWrite(RegWrite),.OPCode(OPCode),.IrWrite(IrWrite),.adrSrc(adrSrc),.PCwrite(PCwrite),
                    .Func3(Func3),.Func7(Func7),.Zero(Zero),.ALU_msb(ALU_msb),.alusrcA(alusrcA),.alusrcB(alusrcB));
endmodule