`default_nettype none

module processor (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    input clk, reset,
    output [12:0] instr_mem_addr,
    input [15:0] instr,
    output [7:0] data_mem_addr,
    input [15:0] read_data,
    output [15:0] write_data,
    output Dataw_en,
    input start,
    output hlt,
    input Serial_input,
    output Serial_output
    );
    wire [4:0] wR_addr, rR1_addr, rR2_addr;
    wire [15:0] R1_bus, R2_bus, write_bus;
    
    //------------------------------------for pipelining----------------------------------------------------
    reg [15:0] instr_stage1, instr_stage2;      // stage 0 is only for getting instruction from instr_memory
    always @(posedge clk, negedge reset)        // for stage 1 operations, the instruction which is stored 
        if(!reset) instr_stage1 <= 16'h0000;    // in the register is decoded and sends control signals 
        else instr_stage1 <= instr;             // to perform operations.. resulting data moves through bus
                                                // and waites at the destination register.
    always @(posedge clk, negedge reset)        // for stage 2 operations, the instruction is transfered
        if(!reset) instr_stage2 <= 16'h0000;    // to this stage register and it is decoded to send signals to store
        else instr_stage2 <= instr_stage1;      // in the corresponding register and at the begining of next cycle,
                                                // the data is being stored in destination register.
    
    
    assign wR_addr  = instr_stage2[12:8];       // as mentioned above data is written at the end of 2nd cycle
    assign rR1_addr = instr_stage1[12:8];       // and read at end of 1st cycle
    assign rR2_addr = instr_stage1[7:3];
    
    
    //------------------------------------register file-----------------------------------------------------
    wire Regw_en, rR1_en, rR2_en;
    Register_file REGISTER_FILE (clk, reset, wR_addr, rR1_addr, rR2_addr, Regw_en,
                                 rR1_en, rR2_en, write_bus, R1_bus, R2_bus);
    
    
    //-----------------------------------------ALU----------------------------------------------------------
    wire [15:0] A, B, Result;
    wire ALU_en;
    wire ALU_write_en;
    
    register_16bit regA (clk, reset, ALU_en, R1_bus, A);       // so that the read data stays on bus only in ALU_en cycle
    register_16bit regB (clk, reset, ALU_en, R2_bus, B);
    tri_state_buffer_16bit w_Result (write_bus, Result, ALU_write_en);  // write of reg file should also be enabled in stage 2
    
    reg p_Z, p_CY;
    wire Z, CY;
    always @(posedge clk, negedge reset)
    if(!reset) begin p_Z <= 1'b0; p_CY <= 1'b0; end
    else if(ALU_write_en) begin p_Z <= Z; p_CY <= CY; end
        
        //----------------------for serial communication-------------------------
        wire SI_en, SO_en;
        wire p_CY_mux_SI = SI_en ? Serial_input : p_CY; // Serial input => if opcode: 001 Rd xxxx1 010 (uses RCL) then MSB first,
                                                        // else if opcode: 001 Rd xxxx1 011 (uses RCR) then LSB first.
        assign Serial_output = p_CY;                    // Serial output=>  if opcode: 001 Rd xxxx0 010 (uses RCL) then MSB first,
                                                        // else if opcode: 001 Rd xxxx0 011 (uses RCR) then LSB first.
        
    ALU Arithmetic_Logic_Unit (instr_stage2[13], instr_stage2[2:0], A, B, p_Z, p_CY_mux_SI, Result, Z, CY);
    
    
    //---------------------------Load & Store operations : Data_memory--------------------------------------
    wire iRW_regw_en, iRW_regr_en;
    wire Datar_en;
    tri_state_buffer_16bit Data_R1 (R1_bus, read_data, Datar_en);                    // Data form memory => R1_bus
    
    wire [15:0] Q_iRW_reg;
    register_16bit iRW_reg (clk, reset, iRW_regw_en, R1_bus, Q_iRW_reg); // intermediate register b/w Read1 and write buses for pipelining
    tri_state_buffer_16bit iRW_reg_data (write_bus, Q_iRW_reg, iRW_regr_en);
    assign write_data = write_bus;                                       // since we have Dataw_en;
    
    
    //---------------------------------------MOV operation--------------------------------------------------
    wire mov_en;
    tri_state_buffer_16bit mov (R1_bus, R2_bus, mov_en);         // since we can send the data to intermediate register from R1_bus
    
    
    //-------------------------------program counter, JUMP & Call-------------------------------------------
    wire PC_load_en;
    wire [12:0] PC_load_addr, PC_load;
    wire R_nJ;
    
    assign PC_load = R_nJ ? write_bus[12:0] : instr_stage2[12:0];   // only for Return operation PC_load is connected to write_bus for retriving instr_mem_address from stack
                                                                    // so that stack read in stage1 is sent to iRW_reg and can be stored from write_bus in stage 2
    buffer pc_load [12:0] (PC_load_addr, PC_load);                  // JZ and JNZ are executed in stage2 because the zero flag of previous
                                                                    // operation gets updated when this instruction is at the end of stage1
    program_counter Program_counter (clk, reset, instr_mem_addr, PC_load_addr, PC_load_en, start, hlt);
    
    
    
    //-----------------------------------stack pointer & Call-----------------------------------------------
    wire SP_load_en, inr_SP, dcr_SP;
    wire [7:0] SP, SP_load;
    buffer sp_load [7:0] (SP_load, instr_stage1[10:3]);         // so as to execute any instruction related to SP without any NOP instruction and as this instruction 
                                                                // doesn't depend on p_Z, and other SP using instructions like Call and return
                                                                // have two NOP instructions following them. => changing in stage1 doesn't affect anything
    
    stack_pointer Stack_pointer (clk, reset, SP_load, SP, SP_load_en, inr_SP, dcr_SP);
    
        
    //-------------------------------------data_memory_addr-------------------------------------------------
    wire SPr, SPw;
    wire [7:0] read_addr = SPr ? SP : instr_stage1[7:0];
    wire [7:0] write_addr = SPw ? SP : instr_stage2[7:0];
    assign data_mem_addr = ({8{Dataw_en}} & write_addr) | ({8{Datar_en}} & read_addr);
    
    
    //---------------------------------------control unit---------------------------------------------------
    CU1 Control_unit1 (instr_stage1, ALU_en, rR1_en, rR2_en, mov_en, SI_en,
                        Datar_en, iRW_regw_en, SP_load_en, dcr_SP, SPr);
    
    CU2 Control_unit2 (instr_stage2, p_Z, ALU_write_en, Regw_en, iRW_regr_en,
                        Dataw_en, PC_load_en, R_nJ, inr_SP, SPw, hlt);
    
    
endmodule



//------------Control unit-----------------------------------------------------------------------

module CU1(
    input [15:0] instr_stage1,
    output ALU_en,
    output rR1_en, rR2_en, mov_en,
    output SI_en, Datar_en, iRW_regw_en,
    output SP_load_en, dcr_SP, SPr
    );
    wire [0:7] p;
    wire p1f5 = p[1]&( instr_stage1[2])&(~instr_stage1[1])&( instr_stage1[0]);
    wire RETZ  = p1f5&(~instr_stage1[12])&( instr_stage1[11]);
    wire RETNZ = p1f5&( instr_stage1[12])&(~instr_stage1[11]);
    wire CZ = p[6];
    wire CNZ = p[7];
    
    decoder_3x8 instr_decoder1 (p, instr_stage1[15:13], 1'b1);
    
    assign ALU_en = p[0] | (p[1]&(~instr_stage1[2]));
    assign rR1_en = ALU_en | p[3];
    assign rR2_en = ALU_en | mov_en;
    assign mov_en = p[1]&(instr_stage1[2])&(~instr_stage1[1])&(~instr_stage1[0]); // MOV opcode : 001 Rd Rs 100
    assign SI_en =  p[1]&(~instr_stage1[2])&(instr_stage1[1])& (instr_stage1[3]); // Serial input => if opcode: 001 Rd xxxx1 010 (uses RCL) then MSB first,
                                                                                  // else if opcode: 001 Rd xxxx1 011 (uses RCR) then LSB first.
    assign Datar_en = p[2] | SPr;             
    assign iRW_regw_en = p[2] | p[3] | mov_en | RETZ | RETNZ;                     // data gets transferred to intermediate register at the end of stage 1;
    assign SP_load_en = p1f5&(~instr_stage1[12])&(~instr_stage1[11]);
    assign SPr = RETZ | RETNZ;
    assign dcr_SP = CZ | CNZ;
    
endmodule

module CU2(
    input [15:0] instr_stage2,
    input p_Z,
    output ALU_write_en, Regw_en,
    output iRW_regr_en, Dataw_en,
    output PC_load_en, R_nJ,
    output inr_SP, SPw,
    output HLT
    );
    wire [0:7] p;
    wire mov = p[1]&(instr_stage2[2])&(~instr_stage2[1])&(~instr_stage2[0]); // MOV opcode : 001 xxxxx xxxxx 100
    wire JZ = p[4];
    wire JNZ = p[5];
    wire CZ = p[6];
    wire CNZ = p[7];
    wire Call_en = (CZ & p_Z) | (CNZ & ~p_Z);
    wire p1f5 = p[1]&( instr_stage2[2])&(~instr_stage2[1])&( instr_stage2[0]);
    wire RETZ  = p1f5&(~instr_stage2[12])&( instr_stage2[11]);
    wire RETNZ = p1f5&( instr_stage2[12])&(~instr_stage2[11]);
    assign HLT = p1f5&( instr_stage2[12])&( instr_stage2[11]);
    
    decoder_3x8 instr_decoder2 (p, instr_stage2[15:13], 1'b1);
        
    assign ALU_write_en = p[0] | (p[1]&(~instr_stage2[2]));
    assign Regw_en = ALU_write_en | mov | p[2];
    assign iRW_regr_en = p[2] | p[3] | mov | R_nJ;
    assign Dataw_en = p[3] | Call_en;
    assign PC_load_en = (JZ & p_Z) | (JNZ & ~p_Z) | R_nJ | Call_en;
    assign R_nJ = (RETZ & p_Z) | (RETNZ & ~p_Z);
    assign inr_SP = R_nJ | ~(Call_en);                    // since we decremented the SP in stage 1 without checking the condition
    assign SPw = Call_en;
     
endmodule

//----------ALU------------------------------------------------------------------------------------

module ALU(
    input op,
    input [2:0] ALU_func,
    input [15:0] A, B,
    input p_Z, p_CY,
    output [15:0] Y_out,
    output Z, CY
    );
    wire [15:0] Y0, Y1;
    wire Z0, CY0, Z1, CY1;
    
    assign Z = op ? Z1 : Z0;
    assign CY = op ? CY1 : CY0;
    assign Y_out = op ? Y1 : Y0;
    
    ALU000 ALU_000 (ALU_func, A, B, p_CY, Y0, Z0, CY0);
    ALU001 ALU_001 (ALU_func[1:0], A, B, p_Z, p_CY, Y1, Z1, CY1); 
    
endmodule


module ALU000(
    input [2:0] ALU_func,
    input [15:0] A, B,
    input p_CY,
    output [15:0] Y,
    output Z, CY
    );
    wire [15:0] A_in, B_in;
    wire C_in;
    wire C_out;
    
    
    wire [2:0] f = ALU_func;
    assign A_in = A & {16{(f[2] | !f[1] | !f[0])}};
    assign B_in = ({16{!f[1]}}&({16{f[2]}}^B))  |  {16{f[1]}} & (({16{f[2]}}&{16{!f[0]}}) | ({16{f[0]}}&(~B)));
    assign C_in = f[2] ^ ((p_CY | f[1]) & !f[0]);
    assign CY = f[2] ^ C_out;
    assign Z = (f[2]&f[1]&f[0]) ? CY : (~|Y);
    
    /*
    always @(*)
    case (ALU_func)
    3'b000 : begin A_in = A;        B_in = B;           C_in = p_CY;  CY = C_out;  Z = ~|Y; end      // {CY,Y} = A + B + p_CY;
    3'b001 : begin A_in = A;        B_in = B;           C_in = 1'b0;  CY = C_out;  Z = ~|Y; end      // {CY,Y} = A + B;
    3'b010 : begin A_in = A;        B_in = 16'h0000;    C_in = 1'b1;  CY = C_out;  Z = ~|Y; end      // {CY,Y} = A + 1;
    3'b011 : begin A_in = 16'h0000; B_in = ~B;          C_in = 1'b0;  CY = C_out;  Z = ~|Y; end      // {CY,Y} = ~B;
    3'b100 : begin A_in = A;        B_in = ~B;          C_in = ~p_CY; CY = ~C_out; Z = ~|Y; end      // {CY,Y} = A - B - p_CY;
    3'b101 : begin A_in = A;        B_in = ~B;          C_in = 1'b1;  CY = ~C_out; Z = ~|Y; end      // {CY,Y} = A - B;
    3'b110 : begin A_in = A;        B_in = 16'hFFFF;    C_in = 1'b0;  CY = ~C_out; Z = ~|Y; end      // {CY,Y} = A - 1;
    3'b111 : begin A_in = A;        B_in = ~B;          C_in = 1'b1;  CY = ~C_out; Z = CY;  end      // if(A<B) Z = 1; else Z = 0;
    endcase
    */
    
    adder_16bit Adder_16bit (A_in, B_in, C_in, Y, C_out);
    
endmodule

module ALU001(
    input [1:0] ALU_func,
    input [15:0] A, B,
    input p_Z, p_CY,
    output [15:0] Y,
    output Z, CY
    );
    /*
    always @(*)
    case (ALU_func)
    2'b00 : begin Y = A&B;           CY = 1'b0;  Z = ~|Y; end    // AND  
    2'b01 : begin Y = A|B;           CY = 1'b0;  Z = ~|Y; end    // OR   
    2'b10 : begin {Y,CY} = {A[14:0],p_CY,A[15};  Z = p_Z; end    // RCL: Rotate with Carry Left  
    2'b11 : begin {Y,CY} = {p_CY,A};             Z = p_Z; end    // RCR: Rotate with Carry Right 
    endcase
    */
    wire [16:0] i0 = {A&B,1'b0};
    wire [16:0] i1 = {A|B,1'b0};
    wire [16:0] i2 = {A[14:0],p_CY,A[15]};
    wire [16:0] i3 = {p_CY,A};
    wire [16:0] y_cy;
    assign {Y,CY} = y_cy;
    mux4x1 Y_CY [16:0] (i0, i1, i2, i3, {17{ALU_func[1]}}, {17{ALU_func[0]}}, y_cy);
    
    wire reduction_nor_Y = ~|Y;
    mux4x1 z (reduction_nor_Y, reduction_nor_Y, p_Z, p_Z, ALU_func[1], ALU_func[0], Z);
    
endmodule

module mux4x1(
    input i0, i1, i2, i3,
    input s1, s0,
    output y
    );
    
    assign y = (!s1)&(!s0)&i0 | (!s1)&(s0)&i1 | (s1)&(!s0)&i2 | (s1)&(s0)&i3;
    
endmodule

//---------adder_16bit----------------------------------------------------------------------------


module adder_16bit(
    input [15:0] a,b,
    input cin,
    output [15:0] sum,
    output cout
    );
    wire c1,c2,c3;

    carry_look_ahead_4bit cla1 (.a(a[3:0]), .b(b[3:0]), .cin(cin), .sum(sum[3:0]), .cout(c1));
    carry_look_ahead_4bit cla2 (.a(a[7:4]), .b(b[7:4]), .cin(c1), .sum(sum[7:4]), .cout(c2));
    carry_look_ahead_4bit cla3(.a(a[11:8]), .b(b[11:8]), .cin(c2), .sum(sum[11:8]), .cout(c3));
    carry_look_ahead_4bit cla4(.a(a[15:12]), .b(b[15:12]), .cin(c3), .sum(sum[15:12]), .cout(cout));

endmodule

//------carry_look_ahead_4bit---------------------------------------------------------------------

module carry_look_ahead_4bit(
    input [3:0] a,b,
    input cin,
    output [3:0] sum,
    output cout
    );
    wire [3:0] p,g,c;

    assign p=a^b;//propagate
    assign g=a&b; //generate

    //carry=gi + Pi.ci

    assign c[0]=cin;
    assign c[1]= g[0] | (p[0]&c[0]);
    assign c[2]= g[1] | (p[1]&g[0]) | p[1]&p[0]&c[0];
    assign c[3]= g[2] | (p[2]&g[1]) | p[2]&p[1]&g[0] | p[2]&p[1]&p[0]&c[0];
    assign cout= g[3] | (p[3]&g[2]) | p[3]&p[2]&g[1] | p[3]&p[2]&p[1]&g[0] | p[3]&p[2]&p[1]&p[0]&c[0];
    assign sum=p^c;

endmodule


//--------decoder_5x32-----------------------------------------------------------------------------

module decoder_5x32(
    output [0:31] Y,
    input [4:0] A,
    input en
    );
    
    decoder_3x8 D0 (Y[0:7]   ,A[2:0], (~A[4] & ~A[3] & en)),
                D1 (Y[8:15]  ,A[2:0], (~A[4] &  A[3] & en)),
                D2 (Y[16:23] ,A[2:0], ( A[4] & ~A[3] & en)),
                D3 (Y[24:31] ,A[2:0], ( A[4] &  A[3] & en));
    
endmodule

//-------decoder_3x8-------------------------------------------------------------------------------

module decoder_3x8(
    output [0:7] Y,
    input [2:0] A,
    input en
    );
    
    and Y0 (Y[0], ~A[2], ~A[1], ~A[0], en),
        Y1 (Y[1], ~A[2], ~A[1],  A[0], en),
        Y2 (Y[2], ~A[2],  A[1], ~A[0], en),
        Y3 (Y[3], ~A[2],  A[1],  A[0], en),
        Y4 (Y[4],  A[2], ~A[1], ~A[0], en),
        Y5 (Y[5],  A[2], ~A[1],  A[0], en),
        Y6 (Y[6],  A[2],  A[1], ~A[0], en),
        Y7 (Y[7],  A[2],  A[1],  A[0], en);
    
endmodule

//--------register_file----------------------------------------------------------------------------

module Register_file(
    input clk, reset,
    input [4:0] wR_addr, rR1_addr, rR2_addr,        // wR_addr => write register address, rR1_addr => read register 1 address
    input write_en, rR1_en, rR2_en,
    input [15:0] write_data,
    output [15:0] R1_data, R2_data
    );
    wire [0:31] w_control, r1_control, r2_control;
    wire [0:(32*16 - 1)] r;
    
    register_16bit Register [0:31] (clk, reset, w_control, {32{write_data}}, r);
    
    decoder_5x32 write_control (w_control, wR_addr, write_en),
                   rR1_control (r1_control, rR1_addr, rR1_en),
                   rR2_control (r2_control, rR2_addr, rR2_en);
    
    tri_state_buffer_16bit     buff1 [0:31] (R1_data, r, r1_control),
                               buff2 [0:31] (R2_data, r, r2_control);
    
endmodule

//--------register_16bit--------------------------------------------------------------------------

module register_16bit(
    input clk, reset,
    input write_en,
    input [15:0] D,
    output reg [15:0] Q
    );
    
    always @(posedge clk, negedge reset)
    begin
        if(!reset) Q <= 16'h0000;
        else if(write_en) Q <= D;
    end
    
endmodule

//--------buffers---------------------------------------------------------------------------------

module tri_state_buffer_16bit(
    output [15:0] o,
    input [15:0] i,
    input control
    );
    
    tri_state_buffer ts_buf [0:15] (o, i, control);
    
endmodule

module tri_state_buffer(
    output Y,
    input A,
    input C
    );
    
    bufif1 tri_state_buf (Y, A, C);
    
endmodule

module buffer(
    output Y,
    input A
    );
    
    buf normal_buf (Y, A);
    
endmodule

//---------program_counter-------------------------------------------------------------------------

module program_counter(
    input clk, reset,
    output reg [12:0] instr_mem_addr,
    input [12:0] PC_load_addr,
    input load_enable,
    input start,
    input hlt
    );
    
    always @(posedge clk, negedge reset)
    if(!reset) instr_mem_addr <= 13'b00000_0000_0000;
    else if(!start) instr_mem_addr <= 13'b00000_0000_0000;
    else if(load_enable) instr_mem_addr <= PC_load_addr;
    else if(!hlt) instr_mem_addr <= instr_mem_addr + 1'b1;           // implementation of up counter
endmodule

//--------stack_pointer----------------------------------------------------------------------------

module stack_pointer(
    input clk, reset,
    input [7:0] SP_load,
    output reg [7:0] SP,
    input SP_load_en,
    input inr_SP, dcr_SP
    );
    wire [7:0] SP_next;
    
    always @(posedge clk, negedge reset)
    if(!reset) SP <= 8'hF0;                 // after reset: stack pointer to 240th memory location
    else SP <= SP_next;
    
    // increment & decrement circuit
    wire [7:0] A_in = ({8{inr_SP}} & SP) | ({8{dcr_SP}} & ~SP);
    wire [7:0] Y;
    wire [7:0] Y_out = ({8{inr_SP}} & Y) | ({8{dcr_SP}} & ~Y);
    incrementer_8bit Incrementer_8bit (A_in, Y);
    
    wire no_change = (~SP_load_en) & (~inr_SP) & (~dcr_SP);
    assign SP_next = ({8{SP_load_en}} & SP_load) | Y_out | ({8{no_change}} & SP);
    
endmodule

//---------incrementer----------------------------------------------------------------------------

module incrementer_8bit(
    input [7:0] A,
    output [7:0] Y
    );
    wire [7:0] C;
    wire C_out;
    
    assign C[0] = 1'b1;
    
    half_adder HA [7:0] (A, C, Y, {C_out,C[7:1]});
    
endmodule



module half_adder(
    input A, B,
    output Y, C
    );
    
    assign Y = A^B;
    assign C = A|B;
    
endmodule

`default_nettype wire
