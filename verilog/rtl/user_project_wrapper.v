// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

/*--------------------------------------*/
/* User project is instantiated  here   */
/*--------------------------------------*/
    wire high;
    wire low;
    
    //Processor related
    wire clk;
    wire reset;
    wire [12:0] uP_instr_mem_addr;
    wire [15:0] uP_instr;
    wire [7:0] uP_data_mem_addr;
    wire [15:0] uP_write_data;
    wire [15:0] uP_read_data;
    wire uP_dataw_en; 
    wire start;
    wire hlt;
    wire Serial_input;
    wire Serial_output;
    
    //Data memory related
    wire data_mem_csb;
    wire [3:0] data_wmask;
    wire dataw_enb;
    wire [7:0] data_mem_addr;
    wire [31:0] data_write_data;
    wire [31:0] data_read_data;
    
    //Instruction memory related
    wire [0:7] instr_mem_csb;
    wire [3:0] instr_wmask;
    wire instrw_enb;
    wire [8:0] instr_mem_addr_9bit;
    wire [15:0] instr_write_data;
    wire [31:0] instr_read_data0;
    wire [31:0] instr_read_data1;
    wire [31:0] instr_read_data2;
    wire [31:0] instr_read_data3;
    wire [31:0] instr_read_data4;
    wire [31:0] instr_read_data5;
    wire [31:0] instr_read_data6;
    wire [31:0] instr_read_data7;


//----------io_interface------------------------------------------------------------------
io_interface IO_interface (
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif

    .wb_clk_i(wb_clk_i),
    .wb_rst_i(wb_rst_i),

    // Logic Analyzer
    .la_data_in(la_data_in[0]),
    .la_data_out(la_data_out),
    .la_oenb(la_oenb[0]),

    //IOs
    .io_in (io_in),
    .io_out(io_out),
    .io_oeb(io_oeb),
    
    // clk & reset
    .clk(clk),
    .reset(reset),
    
    // IRQ
    .irq(user_irq),
    
    //high & low
    .high(high),
    .low(low),
    
    // processor related
    .uP_instr_mem_addr(uP_instr_mem_addr),
    .uP_instr(uP_instr),
    .uP_data_mem_addr(uP_data_mem_addr),
    .uP_read_data(uP_read_data),
    .uP_write_data(uP_write_data),
    .uP_dataw_en(uP_dataw_en),
    .start(start),
    .hlt(hlt),
    .Serial_input(Serial_input),
    .Serial_output(Serial_output),
    
    // data memory related
    .data_mem_csb(data_mem_csb),
    .data_wmask(data_wmask),
    .dataw_enb(dataw_enb),
    .data_mem_addr(data_mem_addr),
    .data_write_data(data_write_data[15:0]),
    .data_read_data(data_read_data[15:0]),
    
    // instr memory related
    .instr_mem_csb(instr_mem_csb),
    .instr_wmask(instr_wmask),
    .instrw_enb(instrw_enb),
    .instr_mem_addr_9bit(instr_mem_addr_9bit),
    .instr_write_data(instr_write_data),
    .instr_read_data0(instr_read_data0),
    .instr_read_data1(instr_read_data1),
    .instr_read_data2(instr_read_data2),
    .instr_read_data3(instr_read_data3),
    .instr_read_data4(instr_read_data4),
    .instr_read_data5(instr_read_data5),
    .instr_read_data6(instr_read_data6),
    .instr_read_data7(instr_read_data7)
    );

//---------Processor----------------------------------------------------------------------
        
processor uP(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
    .clk(clk), .reset(reset),
    .instr_mem_addr(uP_instr_mem_addr),
    .instr(uP_instr),
    .data_mem_addr(uP_data_mem_addr),
    .read_data(uP_read_data),		//since, the data is read only from memory
    .write_data(uP_write_data),
    .Dataw_en(uP_dataw_en),
    .start(start),
    .hlt(hlt),
    .Serial_input(Serial_input),
    .Serial_output(Serial_output)
    );
    
    
//-----------Data_memory------------------------------------------------------------------
sky130_sram_2kbyte_1rw1r_32x512_8 data_memory(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (data_mem_csb), 	// active low chip select
        .web0   (dataw_enb), 		// active low write control
        .wmask0 (data_wmask), // write mask (4 bit) should be high to write
        .addr0  ({low, data_mem_addr}), 	// addr (9 bit)
        .din0   (data_write_data),// data in (32 bit)
        .dout0  (data_read_data),  // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );
    

//------------Instruction_memory----------------------------------------------------------
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory0(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[0]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data0), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory1(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[1]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data1), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory2(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[2]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data2), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory3(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[3]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data3), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory4(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[4]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data4), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory5(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[5]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data5), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory6(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[6]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data6), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );    
    
sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory7(
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif
        // RW
        .clk0   (clk), 		// clock
        .csb0   (instr_mem_csb[7]), 	// active low chip select
        .web0   (instrw_enb), 	// active low write control
        .wmask0 (instr_wmask), // write mask (4 bit) should be high to write
        .addr0  (instr_mem_addr_9bit), 	// addr (9 bit)
        .din0   ({instr_write_data,instr_write_data}),// data in (32 bit)
        .dout0  (instr_read_data7), // data out (32 bit)
        .clk1   (clk),
        .csb1   (low)
    );
endmodule	// user_project_wrapper

`default_nettype wire
