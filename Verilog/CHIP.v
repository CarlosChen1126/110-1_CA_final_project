// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    assign 

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit
    reg PC_t;
    always @(*) begin
        PC_t = PC + 32'h00000004;
        mux PC_m(.Z(PC_nxt), .ctrl(), .in0(PC_t), .in1());
    end
    reg [1:0] ALUOp;
    always @(*) begin
        case(mem_rdata_I[6:0])
            7'b0110011: ALUOp=2'b10;//R type
            7'b1100011: ALUOp=2'b01;//branch
            7'b0000011: ALUOp=2'b00;//ld
            7'b0100011: ALUOp=2'b11;//sd
        endcase
    end
    reg [3:0] ALUctrl;
    always @(*) begin
        case(ALUOp)
            2'b10:case (mem_rdata_I[14:12])
                3'b111: ALUctrl=4'b0000;
                3'b110: ALUctrl=4'b0001;
                3'b000:case (mem_rdata_I[30])
                    1'b0:ALUctrl=4'b0010;
                    1'b1:ALUctrl=4'b0110; 
                    default: ALUctrl=4'b0000;
                    endcase
                default: ALUctrl=4'b0000;
                endcase
            2'b01:ALUctrl=4'b0110;
            2'b00:ALUctrl=4'b0010;
            default: ALUctrl=4'b0000;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2

endmodule
module mux (Z, ctrl, in0, in1);
    output Z;
    input ctrl, in0, in1;
    always @(*) begin
        assign Z = ctrl?in1:in0;
    end
    
endmodule
module ALU (Z, ALUctrl, inA, inB);
    parameter ADD = 4'b0010;
    parameter SUB = 4'b0110;
    parameter AND = 4'b0000;
    parameter OR = 4'b0001;

    input [3:0] ALUctrl;
    input [31:0] inA, inB;
    output reg[31:0] Z;
    always @(*) begin
        case(ALUctrl)
            ADD:begin
                Z=inA+inB;
            end
            SUB:begin
                Z=inA-inB;
            end
            AND:begin
                Z=inA^inB;
            end
            OR:begin
                Z=inA|inB;
            end
            default:Z=31'd0;
        endcase
    end

endmodule