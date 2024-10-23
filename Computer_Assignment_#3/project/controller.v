`timescale 1ps/1ps

module controller (clk,rst,ResultSrc,MemWrite,ALUControl,ImmSrc,RegWrite,OPCode,
                    Func3,Func7,Zero,ALU_msb,alusrcA,alusrcB,IrWrite,adrSrc,PCwrite);

    output wire MemWrite,RegWrite,PCwrite;
    output wire clk,rst,IrWrite,adrSrc;
    output wire [1:0] ResultSrc,alusrcA,alusrcB;
    output wire [2:0] ALUControl,ImmSrc;
    input wire [6:0] OPCode,Func7;
    input wire [2:0] Func3;
    input wire Zero,ALU_msb;

    wire branch,pcUpdate;
    wire[1:0] alu_opc;

    control_unit CU(.Func3(Func3),.OPCode(OPCode),.ImmSrc(ImmSrc),.RegWrite(RegWrite),
                .MemWrite(MemWrite),.ResultSrc(ResultSrc),.alu_opc(alu_opc),.alusrcA(alusrcA)
                ,.clk(clk),.rst(rst),.alusrcB(alusrcB),.IrWrite(IrWrite),.branch(branch),
                .adrSrc(adrSrc),.pcUpdate(pcUpdate));
    ALU_Control_unit ACU (.alu_opc(alu_opc),.Func3(Func3),.Func7(Func7),.ALUControl(ALUControl));
    PCwrite_control_unit PCU (.Zero(Zero),.ALU_msb(ALU_msb),.OPCode(OPCode),.branch(branch),
                        .Func7(Func7),.Func3(Func3),.PCwrite(PCwrite),.pcUpdate(pcUpdate));

    
endmodule

module control_unit (Func3,OPCode,ImmSrc,RegWrite,alusrcA,alusrcB,MemWrite,
                        ResultSrc,alu_opc,clk,rst,IrWrite,branch,adrSrc,pcUpdate);
    output reg MemWrite,RegWrite;
    output reg [1:0] ResultSrc;
    output reg [1:0] alu_opc;
    output reg [2:0] ImmSrc;
    input wire [2:0] Func3;
    input wire [6:0] OPCode;
    input wire clk,rst;
    output reg [1:0] alusrcA,alusrcB;
    output reg branch,adrSrc;
    output reg pcUpdate,IrWrite;
    reg [4:0] ps,ns;

    parameter[6:0] LUI = 7'b0110111,JAL = 7'b1101111,BRANCH = 7'b1100011,
    SW = 7'b0100011,LW = 7'b0000011,IMM = 7'b0010011,JALR = 7'b1100111,
    R_TYPE = 7'b0110011,HALT=7'd0;

    parameter [4:0] IF=5'd0,ID=5'd1,EX_RT=5'd2,EX_SW=5'd3,EX_IMM=5'd4,
    EX_Branch=5'd5,EX_JAL=5'd6,EX_JALR=5'd7,EX_lui=5'd8,MEM_RT=5'd9,MEM_SW=5'd10,MEM_IMM=5'd11,MEM_LW=5'd12
    ,SHIFT_LUI=5'd13,MEM_LUI=5'd14,WB_LW=5'd15,First_MEM_JAL=5'd16,SECOND_EX_JAL=5'd17,second_MEM_JAL=5'd18
    ,First_MEM_JALR=5'd19,SECOND_EX_JALR=5'd20,second_MEM_JALR=5'd21,EX_LW=5'd22;

    always @(ps,OPCode)
     begin
        case(ps)
        IF:ns=ID;
        ID:ns=(OPCode==R_TYPE)?EX_RT:
                (OPCode==SW)?EX_SW:
                (OPCode==LW)?EX_LW:
                (OPCode==IMM)?EX_IMM:
                (OPCode==BRANCH)?EX_Branch:
                (OPCode==JAL)?EX_JAL:
                (OPCode==JALR)?EX_JALR:
                (OPCode==HALT)?ID:EX_lui;
        EX_RT:ns=MEM_RT;
        EX_SW:ns=MEM_SW;
        EX_IMM:ns=MEM_IMM;
        EX_Branch:ns=IF;
        EX_JAL:ns=First_MEM_JAL;
        EX_JALR:ns=First_MEM_JALR;
        EX_lui:ns=MEM_LUI;
        EX_LW:ns=MEM_LW; 
        MEM_RT:ns=IF; 
        MEM_SW:ns=IF;       
        MEM_IMM:ns=IF;  
        MEM_LW:ns=WB_LW;  
        First_MEM_JAL:ns=SECOND_EX_JAL;
        First_MEM_JALR:ns=SECOND_EX_JALR;    
        MEM_LUI:ns=IF;
        SECOND_EX_JAL:ns=second_MEM_JAL;  
        SECOND_EX_JALR:ns=second_MEM_JALR; 
        second_MEM_JAL:ns=IF;
        second_MEM_JALR:ns=IF; 
        WB_LW:ns=IF;
               

        endcase
        
    end

    always @(ps,OPCode) begin
            
            {ImmSrc,RegWrite,alusrcA,alusrcB,MemWrite,
            ResultSrc,alu_opc,IrWrite,branch,adrSrc,pcUpdate} = 0;

            case(ps)
                IF:{IrWrite,alusrcA,alusrcB,alu_opc,ResultSrc,pcUpdate}=
                {1'b1,2'b00,2'b10,2'b00,2'b10,1'b1};
                ID:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                {2'b01,2'b01,2'b00,3'b010};
                EX_RT:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                    {2'b10,2'b00,2'b10,3'b010};
                EX_SW:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                    {2'b10,2'b01,2'b00,3'b001};
                EX_IMM:begin {alusrcA,alusrcB,alu_opc}=
                            {2'b10,2'b01,2'b11}; ImmSrc=(Func3 == 3'd3)? 3'b101: 3'b000; end
                EX_Branch:{alusrcA,alusrcB,alu_opc,ImmSrc,branch}=
                                {2'b10,2'b00,2'b01,3'b010,1'b1};
                EX_JAL:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                        {2'b00,2'b10,2'b00,3'd4};
                EX_JALR:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                            {2'b00,2'b10,2'b00,3'd4};
                EX_lui:{alusrcA,alusrcB,alu_opc,ImmSrc}={2'b10,2'b01,2'b00,3'd3};
                EX_LW:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                        {2'b10,2'b01,2'b00,3'b00}; 
                MEM_RT:{ResultSrc,RegWrite}={2'b00,1'b1}; 
                MEM_SW:{ResultSrc,adrSrc,MemWrite}={2'b00,1'b1,1'b1};       
                MEM_IMM:{ResultSrc,RegWrite}={2'b00,1'b1};  
                MEM_LW:{ResultSrc,adrSrc}=
                {2'b00,1'b1};  
                First_MEM_JAL:{ResultSrc,RegWrite}={2'b00,1'b1};
                First_MEM_JALR:{ResultSrc,RegWrite}={2'b00,1'b1}; 
                MEM_LUI:{ResultSrc,RegWrite}={2'b00,1'b1};
                SECOND_EX_JAL:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                                    {2'b01,2'b01,2'b00,3'd4}; 
                SECOND_EX_JALR:{alusrcA,alusrcB,alu_opc,ImmSrc}=
                                    {2'b10,2'b01,2'b00,3'd4}; 
                second_MEM_JAL:{ResultSrc,adrSrc,pcUpdate}={2'b00,1'b1,1'b1};
                second_MEM_JALR:{ResultSrc,adrSrc,pcUpdate}={2'b00,1'b1,1'b1}; 
                WB_LW:{ResultSrc,RegWrite}={2'b01,1'b1};

            endcase

    end
    always @(posedge clk,posedge rst) begin
        if(rst)
            ps<=5'd0;
        else
            ps<=ns;
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

module PCwrite_control_unit (Zero,ALU_msb,OPCode,Func7,Func3,PCwrite,branch,pcUpdate);
    input wire Zero,ALU_msb;
    input wire [6:0] Func7,OPCode;
    input wire [2:0] Func3;
    output reg PCwrite;
    input wire branch;
    input wire pcUpdate;

    parameter[6:0] LUI = 7'b0110111,JAL = 7'b1101111,BRANCH = 7'b1100011,
    SW = 7'b0100011,LW = 7'b0000011,IMM = 7'b0010011,JALR = 7'b1100111,
    R_TYPE = 7'b0110011;

    always @(OPCode, Func7, Func3, Zero, ALU_msb,branch,pcUpdate) begin
        PCwrite = 1'b0;
        case (OPCode)
            BRANCH:
                case (Func3)
                    3'd0: PCwrite = |{&{branch, Zero},pcUpdate};
                    3'd1: PCwrite = |{&{branch, ~Zero},pcUpdate};
                    3'd4: PCwrite = |{&{branch, ALU_msb},pcUpdate};
                    3'd5: PCwrite = |{&{branch, ~ALU_msb},pcUpdate};
                endcase
            default: PCwrite = pcUpdate;
        endcase
    end


    // always @(PCwrite) begin
    //     if (PCwrite == 2'd3) begin // it means that we are in an unkown state
    //         $display("Error: problem in PCSrc controller");
    //     end 
    // end

endmodule`