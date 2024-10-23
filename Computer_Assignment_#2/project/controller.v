`timescale 1ps/1ps

module controller (PCSrc,ResultSrc,MemWrite,ALUControl,ALUSrc,ImmSrc,RegWrite,OPCode,Sel,
                    Func3,Func7,Zero,ALU_msb);

    output wire MemWrite,ALUSrc,RegWrite;
    output wire ResultSrc;
    output wire [1:0] Sel,PCSrc;
    output wire [2:0] ALUControl,ImmSrc;
    input wire [6:0] OPCode,Func7;
    input wire [2:0] Func3;
    input wire Zero,ALU_msb;

    wire[1:0] alu_opc;

    control_unit CU(.OPCode(OPCode),.ImmSrc(ImmSrc),.RegWrite(RegWrite),.ALUSrc(ALUSrc),
                .MemWrite(MemWrite),.ResultSrc(ResultSrc),.alu_opc(alu_opc),.Sel(Sel));
    ALU_Control_unit ACU (.alu_opc(alu_opc),.Func3(Func3),.Func7(Func7),.ALUControl(ALUControl));
    PCSrc_control_unit PCU (.Zero(Zero),.ALU_msb(ALU_msb),.OPCode(OPCode),
                        .Func7(Func7),.Func3(Func3),.PCSrc(PCSrc));
    
endmodule

module control_unit (OPCode,ImmSrc,RegWrite,ALUSrc,MemWrite,ResultSrc,alu_opc,Sel);
    output reg MemWrite,ALUSrc,RegWrite;
    output reg ResultSrc;
    output reg [1:0] Sel,alu_opc;
    output reg [2:0] ImmSrc;
    input wire [6:0] OPCode;

    parameter[6:0] LUI = 7'b0110111,JAL = 7'b1101111,BRANCH = 7'b1100011,
    SW = 7'b0100011,LW = 7'b0000011,IMM = 7'b0010011,JALR = 7'b1100111,
    R_TYPE = 7'b0110011;

    always @(OPCode) begin
        case (OPCode)
            LUI: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                 {1'b0, 1'b0, 1'b0, 3'b011, 1'b1, 2'b10, 2'b00};
            JAL: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                 {1'b0, 1'b0, 1'b0, 3'b100, 1'b1, 2'b00, 2'b00};
            JALR: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                  {1'b0, 1'b0, 1'b1, 3'b000, 1'b1, 2'b00, 2'b00};
            LW: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                {1'b1, 1'b0, 1'b1, 3'b000, 1'b1, 2'b00, 2'b00};
            SW: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                {1'b0, 1'b1, 1'b1, 3'b001, 1'b0, 2'b00, 2'b00};
            BRANCH: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                    {1'b0, 1'b0, 1'b0, 3'b010, 1'b0, 2'b01, 2'b00};
            IMM: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                 {1'b0, 1'b0, 1'b1, 3'b000, 1'b1, 2'b10, 2'b00};
            R_TYPE: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} =
                    {1'b0, 1'b0, 1'b0, 3'b000, 1'b1, 2'b11, 2'b00};
            default: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, Sel} = 0;
        endcase
    end

endmodule

module ALU_Control_unit (alu_opc,Func3,Func7,ALUControl);
    output reg [2:0] ALUControl;
    input wire [6:0] Func7;
    input wire [2:0] Func3;
    input wire [1:0] alu_opc;

    always @(alu_opc, Func3, Func7) begin
        ALUControl = 3'd0;
        case (alu_opc)
            2'd0: ALUControl = 3'd0;
            2'd1: ALUControl = 3'd1;
            2'd2:
                case (Func3)
                    3'd0: ALUControl = 3'd0;
                    3'd2: ALUControl = 3'd4;
                    3'd3: ALUControl = 3'd5;
                    3'd4: ALUControl = 3'd6;
                    3'd6: ALUControl = 3'd3;
                    3'd7: ALUControl = 3'd2;
                    default: ALUControl = 3'd7;
                endcase
            2'd3:
                case (Func3)
                    3'd0:
                        case(Func7)
                            7'd0: ALUControl = 3'd0;
                            7'b0100000: ALUControl = 3'd1;
                        endcase
                    3'd2: ALUControl = 3'd4;
                    3'd3: ALUControl = 3'd5;
                    3'd4: ALUControl = 3'd6;
                    3'd6: ALUControl = 3'd3;
                    3'd7: ALUControl = 3'd2;
                    default: ALUControl = 3'd7;
                endcase
            default: ALUControl = 3'd7;
        endcase
    end


    always @(ALUControl) begin
        if (ALUControl == 7) begin // it means that we are in an unkown state
            $display("Error: problem in alu controller");
        end 
    end

endmodule

module PCSrc_control_unit (Zero,ALU_msb,OPCode,Func7,Func3,PCSrc);
    input wire Zero,ALU_msb;
    input wire [6:0] Func7,OPCode;
    input wire [2:0] Func3;
    output reg [1:0] PCSrc;

    parameter[6:0] LUI = 7'b0110111,JAL = 7'b1101111,BRANCH = 7'b1100011,
    SW = 7'b0100011,LW = 7'b0000011,IMM = 7'b0010011,JALR = 7'b1100111,
    R_TYPE = 7'b0110011;

    always @(OPCode, Func7, Func3, Zero, ALU_msb) begin
        PCSrc = 2'd0;
        case (OPCode)
            JALR: PCSrc = 2'd2;
            JAL: PCSrc = 2'd1;
            BRANCH:
                case (Func3)
                    3'd0: PCSrc = {1'b0, Zero};
                    3'd1: PCSrc = {1'b0, ~Zero};
                    3'd4: PCSrc = {1'b0, ALU_msb};
                    3'd5: PCSrc = {1'b0, ~ALU_msb | Zero};
                    default: PCSrc = 2'd3;
                endcase
            default: PCSrc = 2'd0;
        endcase
    end


    always @(PCSrc) begin
        if (PCSrc == 2'd3) begin // it means that we are in an unkown state
            $display("Error: problem in PCSrc controller");
        end 
    end

endmodule
