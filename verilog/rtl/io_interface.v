`default_nettype none

module io_interface (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
/*
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,
*/
    // Logic Analyzer Signals
    input  [0:0] la_data_in,
    output [127:0] la_data_out,
    input  [0:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,
    
    // clk & reset
    output clk,
    output reset,
    
    // IRQ
    output [2:0] irq,
    
    //high & low
    output high,
    output low,
    
    // processor related
    input [12:0] uP_instr_mem_addr,
    output [15:0] uP_instr,
    input [7:0] uP_data_mem_addr,
    input [15:0] uP_write_data,
    output [15:0] uP_read_data,
    input uP_dataw_en,
    output start,
    input hlt,
    input Serial_input,
    output Serial_output,
    
    // data memory related
    output data_mem_csb,
    output [3:0] data_wmask,
    output dataw_enb,
    output [7:0] data_mem_addr,
    output [15:0] data_write_data,
    input [15:0] data_read_data,
    
    // instr memory related
    output [0:7] instr_mem_csb,
    output [3:0] instr_wmask,
    output instrw_enb,
    output [8:0] instr_mem_addr_9bit,
    output [15:0] instr_write_data,
    input [31:0] instr_read_data0,
    input [31:0] instr_read_data1,
    input [31:0] instr_read_data2,
    input [31:0] instr_read_data3,
    input [31:0] instr_read_data4,
    input [31:0] instr_read_data5,
    input [31:0] instr_read_data6,
    input [31:0] instr_read_data7
    );
    
    //IRQ
    assign irq = 3'b000; // unused
    
    assign high = 1'b1;
    assign low = 1'b0;
    
    // clk & reset
    wire clk_io = io_in[37];
    assign io_oeb[37] = 1'b1;
    assign clk = start ? wb_clk_i : clk_io;
    assign reset = wb_rst_i;
    
    //processor related
    assign start = io_in[36]; 		//start
    assign io_oeb[36] = 1'b1;
    assign io_out[35] = hlt;			//done
    assign io_oeb[35] = 1'b0;
    
    // 16-bit input
    wire [15:0] data_in;			//data_in
    assign data_in = io_in[31:16];
    assign io_oeb[31:16] = 16'hFFFF;
     
    // 16-bit output
    wire [15:0] data_out;			//data_out
    assign io_out[15:0] = data_out;
    assign io_oeb[15:0] = 16'h0000;
    
    //control signals for data loading & reading
    wire addr_memb;				//addr_memb
    assign addr_memb = io_in[34];
    assign io_oeb[34] = 1'b1;
    wire instr_datab;				//instr_datab
    assign instr_datab = io_in[33];
    assign io_oeb[33] = 1'b1;
    wire wr_rdb;				//wr_rdb
    assign wr_rdb = io_in[32];
    assign io_oeb[32] = 1'b1;
    
    //start	wr_rdb		addr_memb	instr_datab	function
    //0	0		0		0		read data in memory specified by data  memory address register
    //0	0		0		1		read data in memory specified by instr memory address register
    //0	0		1		0		read data  memory address register
    //0	0		1		1		read instr memory address register
    //0	1		0		0		load input in data  memory
    //0	1		0		1		load input in instr memory
    //0	1		1		0		load input in data  memory address register
    //0	1		1		1		load input in instr memory address register
    //1	x		x		x		processor starts executing instructions
    
        
    reg [7:0] data_load_addr;
    always @(posedge clk, negedge reset)
    if(!reset) data_load_addr <= 7'b0000_0000;
    else if(!start && wr_rdb && addr_memb && !instr_datab) data_load_addr <= data_in[7:0];
    else data_load_addr <= data_load_addr + 1;
    
    reg [12:0] instr_load_addr;
    always @(posedge clk, negedge reset)
    if(!reset) instr_load_addr <= 13'b00000_0000_0000;
    else if(!start && wr_rdb && addr_memb && instr_datab) instr_load_addr <= data_in[12:0];
    else instr_load_addr <= instr_load_addr + 1;
    
    reg [31:0] instr_int;
    wire [12:0] instr_mem_addr;
    always @(*)
    case(instr_mem_addr[12:10])
    3'b000	:	instr_int = instr_read_data0;
    3'b001	:	instr_int = instr_read_data1;
    3'b010	:	instr_int = instr_read_data2;
    3'b011	:	instr_int = instr_read_data3;
    3'b100	:	instr_int = instr_read_data4;
    3'b101	:	instr_int = instr_read_data5;
    3'b110	:	instr_int = instr_read_data6;
    3'b111	:	instr_int = instr_read_data7;
    endcase
    wire [15:0] instr = instr_mem_addr[0] ? instr_int[31:16] : instr_int[15:0];
    
    reg [15:0] data_out0;
    always @(*)
    case({addr_memb,instr_datab})
    2'b00	:	data_out0 = data_read_data;
    2'b01	:	data_out0 = instr;
    2'b10	:	data_out0 = {8'h00,data_load_addr};
    2'b11	:	data_out0 = {3'b000,instr_load_addr};
    endcase
    //mux_4x1 Data_out0 [15:0] (data_read_data, instr, {8'h00,data_load_addr}, {3'b000,instr_load_addr}, {16{addr_memb}}, {16{instr_datab}}, data_out0);
    assign data_out = wr_rdb ? data_in : data_out0;
    
     
    assign data_mem_addr = (start) ? uP_data_mem_addr : data_load_addr;
    
    assign instr_mem_addr = (start) ? uP_instr_mem_addr : instr_load_addr;
    assign instr_mem_addr_9bit = instr_mem_addr [9:1];
    
    assign uP_instr = (start) ? instr : 16'b001_00000_00000_100;				//NOP when start = 0
    
    assign uP_read_data = data_read_data;
    
    wire dataw_en = (start) ? uP_dataw_en : wr_rdb && !addr_memb && !instr_datab;         
    assign dataw_enb = ~dataw_en;								//active low
    assign data_mem_csb = 1'b0;
    
    wire instrw_en = !start && wr_rdb && !addr_memb && instr_datab;			
    assign instrw_enb = ~instrw_en;								//active low
    decoderlow_3x8 instr_memory_csb (instr_mem_csb, instr_mem_addr[12:10]);
    
    assign instr_write_data = data_in;
    
    assign data_write_data = (start) ? uP_write_data : data_in;
    
    assign data_wmask = 4'b0011;
    
    assign instr_wmask = {instr_mem_addr[0], instr_mem_addr[0], ~instr_mem_addr[0], ~instr_mem_addr[0]};
    
    //LA
    //using all the pins as outputs except pin 0 : la_oenb = 128'b1
    assign la_data_out[127] = clk;
    assign la_data_out[126] = wb_clk_i;
    assign la_data_out[125] = clk_io;
    assign la_data_out[124] = reset;
    assign la_data_out[123] = uP_dataw_en;
    assign la_data_out[122] = dataw_en;
    assign la_data_out[121] = instrw_en;
    assign la_data_out[120] = start;
    assign la_data_out[119:104] = data_in;
    assign la_data_out[103:101] = io_in[34:32];	//addr_memb, instr_datab, wr_rdb
    assign la_data_out[100:85] = data_out;
    assign la_data_out[84:77] = data_mem_addr;
    assign la_data_out[76:61] = data_read_data;
    assign la_data_out[60:45] = data_write_data;
    assign la_data_out[44:32] = instr_mem_addr;
    assign la_data_out[31:16] = uP_instr;
    assign la_data_out[15:2] = 14'b0;			//unused
    assign la_data_out[1] = Serial_output;
    assign Serial_input = la_oenb[0] & la_data_in[0];

endmodule

module decoderlow_3x8(
    output [0:7] Y,
    input [2:0] A
    );
    
    nand Y0 (Y[0], ~A[2], ~A[1], ~A[0]),
         Y1 (Y[1], ~A[2], ~A[1],  A[0]),
         Y2 (Y[2], ~A[2],  A[1], ~A[0]),
         Y3 (Y[3], ~A[2],  A[1],  A[0]),
         Y4 (Y[4],  A[2], ~A[1], ~A[0]),
         Y5 (Y[5],  A[2], ~A[1],  A[0]),
         Y6 (Y[6],  A[2],  A[1], ~A[0]),
         Y7 (Y[7],  A[2],  A[1],  A[0]);
    
endmodule

`default_nettype wire
