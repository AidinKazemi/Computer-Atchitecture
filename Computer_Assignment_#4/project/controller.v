`timescale 1ps/1ps

module controller (PCSrc,ResultSrc,MemWrite,ALUControl,ALUSrc,ImmSrc,RegWrite,OPCodeD,
                    Func3D,Func7D,Zero,ALU_msb,jumpD,branchD,jumpE,branchE,Func3E,jalr_sel);

    output wire MemWrite,ALUSrc,RegWrite,jumpD,branchD;
    output wire [1:0] ResultSrc;
    output wire PCSrc,jalr_sel;
    output wire [2:0] ALUControl,ImmSrc;
    input wire [6:0] OPCodeD,Func7D;
    input wire [2:0] Func3D,Func3E;
    input wire Zero,ALU_msb;
    input wire jumpE,branchE;
    wire[1:0] alu_opc;

    control_unit CU(.OPCode(OPCodeD),.ImmSrc(ImmSrc),.RegWrite(RegWrite),.ALUSrc(ALUSrc),.Func3D(Func3D),.jalr_sel(jalr_sel),
                .MemWrite(MemWrite),.ResultSrc(ResultSrc),.alu_opc(alu_opc),.jump(jumpD),.branch(branchD));
    ALU_Control_unit ACU (.alu_opc(alu_opc),.Func3(Func3D),.Func7(Func7D),.ALUControl(ALUControl));
    PCSrc_control_unit PCU (.Zero(Zero),.ALU_msb(ALU_msb),
                        .Func3(Func3E),.PCSrc(PCSrc),.jumpE(jumpE),.BranchE(branchE));
                        
    
endmodule

module control_unit (OPCode,ImmSrc,RegWrite,ALUSrc,MemWrite,ResultSrc,alu_opc,jump,branch,jalr_sel,Func3D);
    output reg MemWrite,ALUSrc,RegWrite,jump,branch,jalr_sel;
    output reg [1:0] ResultSrc;
    output reg [1:0] alu_opc;
    output reg [2:0] ImmSrc;
    input wire [6:0] OPCode;
    input wire [2:0] Func3D;

    parameter[6:0] LUI = 7'b0110111,JAL = 7'b1101111,BRANCH = 7'b1100011,
    SW = 7'b0100011,LW = 7'b0000011,IMM = 7'b0010011,JALR = 7'b1100111,
    R_TYPE = 7'b0110011;

    always @(OPCode,Func3D) begin
        case (OPCode)
            LUI: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                 {2'b11, 1'b0, 1'b0, 3'b011, 1'b1, 2'b10,1'b0,1'b0,1'b0};
            JAL: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                 {2'b10, 1'b0, 1'b0, 3'b100, 1'b1, 2'b00,1'b1,1'b0,1'b0};
            JALR: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                  {2'b10, 1'b0, 1'b1, 3'b000, 1'b1, 2'b00,1'b1,1'b0,1'b1};
            LW: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                {2'b01, 1'b0, 1'b1, 3'b000, 1'b1, 2'b00,1'b0,1'b0,1'b0};
            SW: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                {2'b00, 1'b1, 1'b1, 3'b001, 1'b0, 2'b00,1'b0,1'b0,1'b0};
            BRANCH: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                    {2'b00, 1'b0, 1'b0, 3'b010, 1'b0, 2'b01,1'b0,1'b1,1'b0};
            IMM: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                 {2'b00, 1'b0, 1'b1,(Func3D == 3'd3)? 3'b101: 3'b000, 1'b1, 2'b10,1'b0,1'b0,1'b0};
            R_TYPE: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} =
                    {2'b00, 1'b0, 1'b0, 3'b000, 1'b1, 2'b11,1'b0,1'b0,1'b0};
            default: {ResultSrc, MemWrite, ALUSrc, ImmSrc, RegWrite, alu_opc, jump,branch,jalr_sel} = 0;
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

module PCSrc_control_unit (Zero,ALU_msb,Func3,PCSrc,BranchE,jumpE);
    input wire Zero,ALU_msb,BranchE,jumpE;
    input wire [2:0] Func3;
    output reg PCSrc;
    reg branch_approve;
    always @(Func3, Zero, ALU_msb,BranchE,jumpE) begin
        branch_approve = 1'b0;
        case (Func3)
            3'd0: branch_approve = &{BranchE, Zero};
            3'd1: branch_approve = &{BranchE, ~Zero};
            3'd4: branch_approve = &{BranchE, ALU_msb};
            3'd5: branch_approve = &{BranchE, ~ALU_msb | Zero};
        endcase
    end

    assign PCSrc = jumpE | branch_approve; 

endmodule

module hazard_controller(clk,stallF,stallD,flushE,flushD,forwardAe,forwardBe,
                        res_src_0,rdM,rdw,regwriteW,rs1D,rs2D,RdE,rs2E,rs1E,pcsrc,regwriteM);
    output reg stallF,stallD,flushD,flushE;
    output reg [1:0] forwardAe,forwardBe;
    input wire clk,res_src_0,regwriteW,regwriteM,pcsrc;
    input wire [4:0] rs1D,rs2D,RdE,rs2E,rs1E,rdM,rdw;

    always @(rs1D,rs2D,RdE,rs2E,rs1E,res_src_0,rdM,rdw,regwriteW,regwriteM,pcsrc)
        begin 
            forwardAe=2'b00;
            forwardBe=2'b00;
            if((rs1E==rdM) & (regwriteM==1) & (rs1E != 5'd0))
                forwardAe=2'b10;
            else if((rs1E==rdw) & (regwriteW==1) & (rs1E!=5'd0))
                forwardAe=2'b01;
            if((rs2E==rdM) & (regwriteM==1) & (rs2E!=5'd0))
                forwardBe=2'b10;
            else if((rs2E==rdw) & (regwriteW==1) & (rs2E!=5'd0))
                forwardBe=2'b01;
        end

    always @(posedge clk)
        begin 
            flushE=0;
            
            if(((rs1D==RdE) | (rs2D==RdE )) & (res_src_0==1))
            begin
                flushE=1;
            end
    end

    always @(negedge clk)
        begin 
            flushE=0;
    end

    always @(rs1D,rs2D,RdE,res_src_0)
        begin 
            stallD=0;stallF=0;
            
            if(((rs1D==RdE) | (rs2D==RdE )) & (res_src_0==1))
            begin
                stallD=1;stallF=1;
            end
    end

    always @(posedge clk)
        begin 
            flushD=0; flushE=0;  
            if(pcsrc)
               begin flushD=1;flushE=1; end
    end

    always @(negedge clk)
        begin 
            flushE=0;flushD=0;
    end

endmodule
