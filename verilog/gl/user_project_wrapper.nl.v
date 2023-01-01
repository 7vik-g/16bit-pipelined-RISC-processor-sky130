// This is the unpowered netlist.
module user_project_wrapper (user_clock2,
    wb_clk_i,
    wb_rst_i,
    wbs_ack_o,
    wbs_cyc_i,
    wbs_stb_i,
    wbs_we_i,
    analog_io,
    io_in,
    io_oeb,
    io_out,
    la_data_in,
    la_data_out,
    la_oenb,
    user_irq,
    wbs_adr_i,
    wbs_dat_i,
    wbs_dat_o,
    wbs_sel_i);
 input user_clock2;
 input wb_clk_i;
 input wb_rst_i;
 output wbs_ack_o;
 input wbs_cyc_i;
 input wbs_stb_i;
 input wbs_we_i;
 inout [28:0] analog_io;
 input [37:0] io_in;
 output [37:0] io_oeb;
 output [37:0] io_out;
 input [127:0] la_data_in;
 output [127:0] la_data_out;
 input [127:0] la_oenb;
 output [2:0] user_irq;
 input [31:0] wbs_adr_i;
 input [31:0] wbs_dat_i;
 output [31:0] wbs_dat_o;
 input [3:0] wbs_sel_i;

 wire Serial_input;
 wire Serial_output;
 wire clk;
 wire \data_mem_addr[0] ;
 wire \data_mem_addr[1] ;
 wire \data_mem_addr[2] ;
 wire \data_mem_addr[3] ;
 wire \data_mem_addr[4] ;
 wire \data_mem_addr[5] ;
 wire \data_mem_addr[6] ;
 wire \data_mem_addr[7] ;
 wire data_mem_csb;
 wire \data_read_data[0] ;
 wire \data_read_data[10] ;
 wire \data_read_data[11] ;
 wire \data_read_data[12] ;
 wire \data_read_data[13] ;
 wire \data_read_data[14] ;
 wire \data_read_data[15] ;
 wire \data_read_data[16] ;
 wire \data_read_data[17] ;
 wire \data_read_data[18] ;
 wire \data_read_data[19] ;
 wire \data_read_data[1] ;
 wire \data_read_data[20] ;
 wire \data_read_data[21] ;
 wire \data_read_data[22] ;
 wire \data_read_data[23] ;
 wire \data_read_data[24] ;
 wire \data_read_data[25] ;
 wire \data_read_data[26] ;
 wire \data_read_data[27] ;
 wire \data_read_data[28] ;
 wire \data_read_data[29] ;
 wire \data_read_data[2] ;
 wire \data_read_data[30] ;
 wire \data_read_data[31] ;
 wire \data_read_data[3] ;
 wire \data_read_data[4] ;
 wire \data_read_data[5] ;
 wire \data_read_data[6] ;
 wire \data_read_data[7] ;
 wire \data_read_data[8] ;
 wire \data_read_data[9] ;
 wire \data_wmask[0] ;
 wire \data_wmask[1] ;
 wire \data_wmask[2] ;
 wire \data_wmask[3] ;
 wire \data_write_data[0] ;
 wire \data_write_data[10] ;
 wire \data_write_data[11] ;
 wire \data_write_data[12] ;
 wire \data_write_data[13] ;
 wire \data_write_data[14] ;
 wire \data_write_data[15] ;
 wire \data_write_data[16] ;
 wire \data_write_data[17] ;
 wire \data_write_data[18] ;
 wire \data_write_data[19] ;
 wire \data_write_data[1] ;
 wire \data_write_data[20] ;
 wire \data_write_data[21] ;
 wire \data_write_data[22] ;
 wire \data_write_data[23] ;
 wire \data_write_data[24] ;
 wire \data_write_data[25] ;
 wire \data_write_data[26] ;
 wire \data_write_data[27] ;
 wire \data_write_data[28] ;
 wire \data_write_data[29] ;
 wire \data_write_data[2] ;
 wire \data_write_data[30] ;
 wire \data_write_data[31] ;
 wire \data_write_data[3] ;
 wire \data_write_data[4] ;
 wire \data_write_data[5] ;
 wire \data_write_data[6] ;
 wire \data_write_data[7] ;
 wire \data_write_data[8] ;
 wire \data_write_data[9] ;
 wire dataw_enb;
 wire high;
 wire hlt;
 wire \instr_mem_addr_9bit[0] ;
 wire \instr_mem_addr_9bit[1] ;
 wire \instr_mem_addr_9bit[2] ;
 wire \instr_mem_addr_9bit[3] ;
 wire \instr_mem_addr_9bit[4] ;
 wire \instr_mem_addr_9bit[5] ;
 wire \instr_mem_addr_9bit[6] ;
 wire \instr_mem_addr_9bit[7] ;
 wire \instr_mem_addr_9bit[8] ;
 wire \instr_mem_csb[0] ;
 wire \instr_mem_csb[1] ;
 wire \instr_mem_csb[2] ;
 wire \instr_mem_csb[3] ;
 wire \instr_mem_csb[4] ;
 wire \instr_mem_csb[5] ;
 wire \instr_mem_csb[6] ;
 wire \instr_mem_csb[7] ;
 wire \instr_read_data0[0] ;
 wire \instr_read_data0[10] ;
 wire \instr_read_data0[11] ;
 wire \instr_read_data0[12] ;
 wire \instr_read_data0[13] ;
 wire \instr_read_data0[14] ;
 wire \instr_read_data0[15] ;
 wire \instr_read_data0[16] ;
 wire \instr_read_data0[17] ;
 wire \instr_read_data0[18] ;
 wire \instr_read_data0[19] ;
 wire \instr_read_data0[1] ;
 wire \instr_read_data0[20] ;
 wire \instr_read_data0[21] ;
 wire \instr_read_data0[22] ;
 wire \instr_read_data0[23] ;
 wire \instr_read_data0[24] ;
 wire \instr_read_data0[25] ;
 wire \instr_read_data0[26] ;
 wire \instr_read_data0[27] ;
 wire \instr_read_data0[28] ;
 wire \instr_read_data0[29] ;
 wire \instr_read_data0[2] ;
 wire \instr_read_data0[30] ;
 wire \instr_read_data0[31] ;
 wire \instr_read_data0[3] ;
 wire \instr_read_data0[4] ;
 wire \instr_read_data0[5] ;
 wire \instr_read_data0[6] ;
 wire \instr_read_data0[7] ;
 wire \instr_read_data0[8] ;
 wire \instr_read_data0[9] ;
 wire \instr_read_data1[0] ;
 wire \instr_read_data1[10] ;
 wire \instr_read_data1[11] ;
 wire \instr_read_data1[12] ;
 wire \instr_read_data1[13] ;
 wire \instr_read_data1[14] ;
 wire \instr_read_data1[15] ;
 wire \instr_read_data1[16] ;
 wire \instr_read_data1[17] ;
 wire \instr_read_data1[18] ;
 wire \instr_read_data1[19] ;
 wire \instr_read_data1[1] ;
 wire \instr_read_data1[20] ;
 wire \instr_read_data1[21] ;
 wire \instr_read_data1[22] ;
 wire \instr_read_data1[23] ;
 wire \instr_read_data1[24] ;
 wire \instr_read_data1[25] ;
 wire \instr_read_data1[26] ;
 wire \instr_read_data1[27] ;
 wire \instr_read_data1[28] ;
 wire \instr_read_data1[29] ;
 wire \instr_read_data1[2] ;
 wire \instr_read_data1[30] ;
 wire \instr_read_data1[31] ;
 wire \instr_read_data1[3] ;
 wire \instr_read_data1[4] ;
 wire \instr_read_data1[5] ;
 wire \instr_read_data1[6] ;
 wire \instr_read_data1[7] ;
 wire \instr_read_data1[8] ;
 wire \instr_read_data1[9] ;
 wire \instr_read_data2[0] ;
 wire \instr_read_data2[10] ;
 wire \instr_read_data2[11] ;
 wire \instr_read_data2[12] ;
 wire \instr_read_data2[13] ;
 wire \instr_read_data2[14] ;
 wire \instr_read_data2[15] ;
 wire \instr_read_data2[16] ;
 wire \instr_read_data2[17] ;
 wire \instr_read_data2[18] ;
 wire \instr_read_data2[19] ;
 wire \instr_read_data2[1] ;
 wire \instr_read_data2[20] ;
 wire \instr_read_data2[21] ;
 wire \instr_read_data2[22] ;
 wire \instr_read_data2[23] ;
 wire \instr_read_data2[24] ;
 wire \instr_read_data2[25] ;
 wire \instr_read_data2[26] ;
 wire \instr_read_data2[27] ;
 wire \instr_read_data2[28] ;
 wire \instr_read_data2[29] ;
 wire \instr_read_data2[2] ;
 wire \instr_read_data2[30] ;
 wire \instr_read_data2[31] ;
 wire \instr_read_data2[3] ;
 wire \instr_read_data2[4] ;
 wire \instr_read_data2[5] ;
 wire \instr_read_data2[6] ;
 wire \instr_read_data2[7] ;
 wire \instr_read_data2[8] ;
 wire \instr_read_data2[9] ;
 wire \instr_read_data3[0] ;
 wire \instr_read_data3[10] ;
 wire \instr_read_data3[11] ;
 wire \instr_read_data3[12] ;
 wire \instr_read_data3[13] ;
 wire \instr_read_data3[14] ;
 wire \instr_read_data3[15] ;
 wire \instr_read_data3[16] ;
 wire \instr_read_data3[17] ;
 wire \instr_read_data3[18] ;
 wire \instr_read_data3[19] ;
 wire \instr_read_data3[1] ;
 wire \instr_read_data3[20] ;
 wire \instr_read_data3[21] ;
 wire \instr_read_data3[22] ;
 wire \instr_read_data3[23] ;
 wire \instr_read_data3[24] ;
 wire \instr_read_data3[25] ;
 wire \instr_read_data3[26] ;
 wire \instr_read_data3[27] ;
 wire \instr_read_data3[28] ;
 wire \instr_read_data3[29] ;
 wire \instr_read_data3[2] ;
 wire \instr_read_data3[30] ;
 wire \instr_read_data3[31] ;
 wire \instr_read_data3[3] ;
 wire \instr_read_data3[4] ;
 wire \instr_read_data3[5] ;
 wire \instr_read_data3[6] ;
 wire \instr_read_data3[7] ;
 wire \instr_read_data3[8] ;
 wire \instr_read_data3[9] ;
 wire \instr_read_data4[0] ;
 wire \instr_read_data4[10] ;
 wire \instr_read_data4[11] ;
 wire \instr_read_data4[12] ;
 wire \instr_read_data4[13] ;
 wire \instr_read_data4[14] ;
 wire \instr_read_data4[15] ;
 wire \instr_read_data4[16] ;
 wire \instr_read_data4[17] ;
 wire \instr_read_data4[18] ;
 wire \instr_read_data4[19] ;
 wire \instr_read_data4[1] ;
 wire \instr_read_data4[20] ;
 wire \instr_read_data4[21] ;
 wire \instr_read_data4[22] ;
 wire \instr_read_data4[23] ;
 wire \instr_read_data4[24] ;
 wire \instr_read_data4[25] ;
 wire \instr_read_data4[26] ;
 wire \instr_read_data4[27] ;
 wire \instr_read_data4[28] ;
 wire \instr_read_data4[29] ;
 wire \instr_read_data4[2] ;
 wire \instr_read_data4[30] ;
 wire \instr_read_data4[31] ;
 wire \instr_read_data4[3] ;
 wire \instr_read_data4[4] ;
 wire \instr_read_data4[5] ;
 wire \instr_read_data4[6] ;
 wire \instr_read_data4[7] ;
 wire \instr_read_data4[8] ;
 wire \instr_read_data4[9] ;
 wire \instr_read_data5[0] ;
 wire \instr_read_data5[10] ;
 wire \instr_read_data5[11] ;
 wire \instr_read_data5[12] ;
 wire \instr_read_data5[13] ;
 wire \instr_read_data5[14] ;
 wire \instr_read_data5[15] ;
 wire \instr_read_data5[16] ;
 wire \instr_read_data5[17] ;
 wire \instr_read_data5[18] ;
 wire \instr_read_data5[19] ;
 wire \instr_read_data5[1] ;
 wire \instr_read_data5[20] ;
 wire \instr_read_data5[21] ;
 wire \instr_read_data5[22] ;
 wire \instr_read_data5[23] ;
 wire \instr_read_data5[24] ;
 wire \instr_read_data5[25] ;
 wire \instr_read_data5[26] ;
 wire \instr_read_data5[27] ;
 wire \instr_read_data5[28] ;
 wire \instr_read_data5[29] ;
 wire \instr_read_data5[2] ;
 wire \instr_read_data5[30] ;
 wire \instr_read_data5[31] ;
 wire \instr_read_data5[3] ;
 wire \instr_read_data5[4] ;
 wire \instr_read_data5[5] ;
 wire \instr_read_data5[6] ;
 wire \instr_read_data5[7] ;
 wire \instr_read_data5[8] ;
 wire \instr_read_data5[9] ;
 wire \instr_read_data6[0] ;
 wire \instr_read_data6[10] ;
 wire \instr_read_data6[11] ;
 wire \instr_read_data6[12] ;
 wire \instr_read_data6[13] ;
 wire \instr_read_data6[14] ;
 wire \instr_read_data6[15] ;
 wire \instr_read_data6[16] ;
 wire \instr_read_data6[17] ;
 wire \instr_read_data6[18] ;
 wire \instr_read_data6[19] ;
 wire \instr_read_data6[1] ;
 wire \instr_read_data6[20] ;
 wire \instr_read_data6[21] ;
 wire \instr_read_data6[22] ;
 wire \instr_read_data6[23] ;
 wire \instr_read_data6[24] ;
 wire \instr_read_data6[25] ;
 wire \instr_read_data6[26] ;
 wire \instr_read_data6[27] ;
 wire \instr_read_data6[28] ;
 wire \instr_read_data6[29] ;
 wire \instr_read_data6[2] ;
 wire \instr_read_data6[30] ;
 wire \instr_read_data6[31] ;
 wire \instr_read_data6[3] ;
 wire \instr_read_data6[4] ;
 wire \instr_read_data6[5] ;
 wire \instr_read_data6[6] ;
 wire \instr_read_data6[7] ;
 wire \instr_read_data6[8] ;
 wire \instr_read_data6[9] ;
 wire \instr_read_data7[0] ;
 wire \instr_read_data7[10] ;
 wire \instr_read_data7[11] ;
 wire \instr_read_data7[12] ;
 wire \instr_read_data7[13] ;
 wire \instr_read_data7[14] ;
 wire \instr_read_data7[15] ;
 wire \instr_read_data7[16] ;
 wire \instr_read_data7[17] ;
 wire \instr_read_data7[18] ;
 wire \instr_read_data7[19] ;
 wire \instr_read_data7[1] ;
 wire \instr_read_data7[20] ;
 wire \instr_read_data7[21] ;
 wire \instr_read_data7[22] ;
 wire \instr_read_data7[23] ;
 wire \instr_read_data7[24] ;
 wire \instr_read_data7[25] ;
 wire \instr_read_data7[26] ;
 wire \instr_read_data7[27] ;
 wire \instr_read_data7[28] ;
 wire \instr_read_data7[29] ;
 wire \instr_read_data7[2] ;
 wire \instr_read_data7[30] ;
 wire \instr_read_data7[31] ;
 wire \instr_read_data7[3] ;
 wire \instr_read_data7[4] ;
 wire \instr_read_data7[5] ;
 wire \instr_read_data7[6] ;
 wire \instr_read_data7[7] ;
 wire \instr_read_data7[8] ;
 wire \instr_read_data7[9] ;
 wire \instr_wmask[0] ;
 wire \instr_wmask[1] ;
 wire \instr_wmask[2] ;
 wire \instr_wmask[3] ;
 wire \instr_write_data[0] ;
 wire \instr_write_data[10] ;
 wire \instr_write_data[11] ;
 wire \instr_write_data[12] ;
 wire \instr_write_data[13] ;
 wire \instr_write_data[14] ;
 wire \instr_write_data[15] ;
 wire \instr_write_data[1] ;
 wire \instr_write_data[2] ;
 wire \instr_write_data[3] ;
 wire \instr_write_data[4] ;
 wire \instr_write_data[5] ;
 wire \instr_write_data[6] ;
 wire \instr_write_data[7] ;
 wire \instr_write_data[8] ;
 wire \instr_write_data[9] ;
 wire instrw_enb;
 wire low;
 wire reset;
 wire start;
 wire \uP_data_mem_addr[0] ;
 wire \uP_data_mem_addr[1] ;
 wire \uP_data_mem_addr[2] ;
 wire \uP_data_mem_addr[3] ;
 wire \uP_data_mem_addr[4] ;
 wire \uP_data_mem_addr[5] ;
 wire \uP_data_mem_addr[6] ;
 wire \uP_data_mem_addr[7] ;
 wire uP_dataw_en;
 wire \uP_instr[0] ;
 wire \uP_instr[10] ;
 wire \uP_instr[11] ;
 wire \uP_instr[12] ;
 wire \uP_instr[13] ;
 wire \uP_instr[14] ;
 wire \uP_instr[15] ;
 wire \uP_instr[1] ;
 wire \uP_instr[2] ;
 wire \uP_instr[3] ;
 wire \uP_instr[4] ;
 wire \uP_instr[5] ;
 wire \uP_instr[6] ;
 wire \uP_instr[7] ;
 wire \uP_instr[8] ;
 wire \uP_instr[9] ;
 wire \uP_instr_mem_addr[0] ;
 wire \uP_instr_mem_addr[10] ;
 wire \uP_instr_mem_addr[11] ;
 wire \uP_instr_mem_addr[12] ;
 wire \uP_instr_mem_addr[1] ;
 wire \uP_instr_mem_addr[2] ;
 wire \uP_instr_mem_addr[3] ;
 wire \uP_instr_mem_addr[4] ;
 wire \uP_instr_mem_addr[5] ;
 wire \uP_instr_mem_addr[6] ;
 wire \uP_instr_mem_addr[7] ;
 wire \uP_instr_mem_addr[8] ;
 wire \uP_instr_mem_addr[9] ;
 wire \uP_read_data[0] ;
 wire \uP_read_data[10] ;
 wire \uP_read_data[11] ;
 wire \uP_read_data[12] ;
 wire \uP_read_data[13] ;
 wire \uP_read_data[14] ;
 wire \uP_read_data[15] ;
 wire \uP_read_data[1] ;
 wire \uP_read_data[2] ;
 wire \uP_read_data[3] ;
 wire \uP_read_data[4] ;
 wire \uP_read_data[5] ;
 wire \uP_read_data[6] ;
 wire \uP_read_data[7] ;
 wire \uP_read_data[8] ;
 wire \uP_read_data[9] ;
 wire \uP_write_data[0] ;
 wire \uP_write_data[10] ;
 wire \uP_write_data[11] ;
 wire \uP_write_data[12] ;
 wire \uP_write_data[13] ;
 wire \uP_write_data[14] ;
 wire \uP_write_data[15] ;
 wire \uP_write_data[1] ;
 wire \uP_write_data[2] ;
 wire \uP_write_data[3] ;
 wire \uP_write_data[4] ;
 wire \uP_write_data[5] ;
 wire \uP_write_data[6] ;
 wire \uP_write_data[7] ;
 wire \uP_write_data[8] ;
 wire \uP_write_data[9] ;

 io_interface IO_interface (.Serial_input(Serial_input),
    .Serial_output(Serial_output),
    .clk(clk),
    .data_mem_csb(data_mem_csb),
    .dataw_enb(dataw_enb),
    .high(high),
    .hlt(hlt),
    .instrw_enb(instrw_enb),
    .la_data_in(la_data_in[0]),
    .la_oenb(la_oenb[0]),
    .low(low),
    .reset(reset),
    .start(start),
    .uP_dataw_en(uP_dataw_en),
    .wb_clk_i(wb_clk_i),
    .wb_rst_i(wb_rst_i),
    .data_mem_addr({\data_mem_addr[7] ,
    \data_mem_addr[6] ,
    \data_mem_addr[5] ,
    \data_mem_addr[4] ,
    \data_mem_addr[3] ,
    \data_mem_addr[2] ,
    \data_mem_addr[1] ,
    \data_mem_addr[0] }),
    .data_read_data({\data_read_data[15] ,
    \data_read_data[14] ,
    \data_read_data[13] ,
    \data_read_data[12] ,
    \data_read_data[11] ,
    \data_read_data[10] ,
    \data_read_data[9] ,
    \data_read_data[8] ,
    \data_read_data[7] ,
    \data_read_data[6] ,
    \data_read_data[5] ,
    \data_read_data[4] ,
    \data_read_data[3] ,
    \data_read_data[2] ,
    \data_read_data[1] ,
    \data_read_data[0] }),
    .data_wmask({\data_wmask[3] ,
    \data_wmask[2] ,
    \data_wmask[1] ,
    \data_wmask[0] }),
    .data_write_data({\data_write_data[15] ,
    \data_write_data[14] ,
    \data_write_data[13] ,
    \data_write_data[12] ,
    \data_write_data[11] ,
    \data_write_data[10] ,
    \data_write_data[9] ,
    \data_write_data[8] ,
    \data_write_data[7] ,
    \data_write_data[6] ,
    \data_write_data[5] ,
    \data_write_data[4] ,
    \data_write_data[3] ,
    \data_write_data[2] ,
    \data_write_data[1] ,
    \data_write_data[0] }),
    .instr_mem_addr_9bit({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .instr_mem_csb({\instr_mem_csb[0] ,
    \instr_mem_csb[1] ,
    \instr_mem_csb[2] ,
    \instr_mem_csb[3] ,
    \instr_mem_csb[4] ,
    \instr_mem_csb[5] ,
    \instr_mem_csb[6] ,
    \instr_mem_csb[7] }),
    .instr_read_data0({\instr_read_data0[31] ,
    \instr_read_data0[30] ,
    \instr_read_data0[29] ,
    \instr_read_data0[28] ,
    \instr_read_data0[27] ,
    \instr_read_data0[26] ,
    \instr_read_data0[25] ,
    \instr_read_data0[24] ,
    \instr_read_data0[23] ,
    \instr_read_data0[22] ,
    \instr_read_data0[21] ,
    \instr_read_data0[20] ,
    \instr_read_data0[19] ,
    \instr_read_data0[18] ,
    \instr_read_data0[17] ,
    \instr_read_data0[16] ,
    \instr_read_data0[15] ,
    \instr_read_data0[14] ,
    \instr_read_data0[13] ,
    \instr_read_data0[12] ,
    \instr_read_data0[11] ,
    \instr_read_data0[10] ,
    \instr_read_data0[9] ,
    \instr_read_data0[8] ,
    \instr_read_data0[7] ,
    \instr_read_data0[6] ,
    \instr_read_data0[5] ,
    \instr_read_data0[4] ,
    \instr_read_data0[3] ,
    \instr_read_data0[2] ,
    \instr_read_data0[1] ,
    \instr_read_data0[0] }),
    .instr_read_data1({\instr_read_data1[31] ,
    \instr_read_data1[30] ,
    \instr_read_data1[29] ,
    \instr_read_data1[28] ,
    \instr_read_data1[27] ,
    \instr_read_data1[26] ,
    \instr_read_data1[25] ,
    \instr_read_data1[24] ,
    \instr_read_data1[23] ,
    \instr_read_data1[22] ,
    \instr_read_data1[21] ,
    \instr_read_data1[20] ,
    \instr_read_data1[19] ,
    \instr_read_data1[18] ,
    \instr_read_data1[17] ,
    \instr_read_data1[16] ,
    \instr_read_data1[15] ,
    \instr_read_data1[14] ,
    \instr_read_data1[13] ,
    \instr_read_data1[12] ,
    \instr_read_data1[11] ,
    \instr_read_data1[10] ,
    \instr_read_data1[9] ,
    \instr_read_data1[8] ,
    \instr_read_data1[7] ,
    \instr_read_data1[6] ,
    \instr_read_data1[5] ,
    \instr_read_data1[4] ,
    \instr_read_data1[3] ,
    \instr_read_data1[2] ,
    \instr_read_data1[1] ,
    \instr_read_data1[0] }),
    .instr_read_data2({\instr_read_data2[31] ,
    \instr_read_data2[30] ,
    \instr_read_data2[29] ,
    \instr_read_data2[28] ,
    \instr_read_data2[27] ,
    \instr_read_data2[26] ,
    \instr_read_data2[25] ,
    \instr_read_data2[24] ,
    \instr_read_data2[23] ,
    \instr_read_data2[22] ,
    \instr_read_data2[21] ,
    \instr_read_data2[20] ,
    \instr_read_data2[19] ,
    \instr_read_data2[18] ,
    \instr_read_data2[17] ,
    \instr_read_data2[16] ,
    \instr_read_data2[15] ,
    \instr_read_data2[14] ,
    \instr_read_data2[13] ,
    \instr_read_data2[12] ,
    \instr_read_data2[11] ,
    \instr_read_data2[10] ,
    \instr_read_data2[9] ,
    \instr_read_data2[8] ,
    \instr_read_data2[7] ,
    \instr_read_data2[6] ,
    \instr_read_data2[5] ,
    \instr_read_data2[4] ,
    \instr_read_data2[3] ,
    \instr_read_data2[2] ,
    \instr_read_data2[1] ,
    \instr_read_data2[0] }),
    .instr_read_data3({\instr_read_data3[31] ,
    \instr_read_data3[30] ,
    \instr_read_data3[29] ,
    \instr_read_data3[28] ,
    \instr_read_data3[27] ,
    \instr_read_data3[26] ,
    \instr_read_data3[25] ,
    \instr_read_data3[24] ,
    \instr_read_data3[23] ,
    \instr_read_data3[22] ,
    \instr_read_data3[21] ,
    \instr_read_data3[20] ,
    \instr_read_data3[19] ,
    \instr_read_data3[18] ,
    \instr_read_data3[17] ,
    \instr_read_data3[16] ,
    \instr_read_data3[15] ,
    \instr_read_data3[14] ,
    \instr_read_data3[13] ,
    \instr_read_data3[12] ,
    \instr_read_data3[11] ,
    \instr_read_data3[10] ,
    \instr_read_data3[9] ,
    \instr_read_data3[8] ,
    \instr_read_data3[7] ,
    \instr_read_data3[6] ,
    \instr_read_data3[5] ,
    \instr_read_data3[4] ,
    \instr_read_data3[3] ,
    \instr_read_data3[2] ,
    \instr_read_data3[1] ,
    \instr_read_data3[0] }),
    .instr_read_data4({\instr_read_data4[31] ,
    \instr_read_data4[30] ,
    \instr_read_data4[29] ,
    \instr_read_data4[28] ,
    \instr_read_data4[27] ,
    \instr_read_data4[26] ,
    \instr_read_data4[25] ,
    \instr_read_data4[24] ,
    \instr_read_data4[23] ,
    \instr_read_data4[22] ,
    \instr_read_data4[21] ,
    \instr_read_data4[20] ,
    \instr_read_data4[19] ,
    \instr_read_data4[18] ,
    \instr_read_data4[17] ,
    \instr_read_data4[16] ,
    \instr_read_data4[15] ,
    \instr_read_data4[14] ,
    \instr_read_data4[13] ,
    \instr_read_data4[12] ,
    \instr_read_data4[11] ,
    \instr_read_data4[10] ,
    \instr_read_data4[9] ,
    \instr_read_data4[8] ,
    \instr_read_data4[7] ,
    \instr_read_data4[6] ,
    \instr_read_data4[5] ,
    \instr_read_data4[4] ,
    \instr_read_data4[3] ,
    \instr_read_data4[2] ,
    \instr_read_data4[1] ,
    \instr_read_data4[0] }),
    .instr_read_data5({\instr_read_data5[31] ,
    \instr_read_data5[30] ,
    \instr_read_data5[29] ,
    \instr_read_data5[28] ,
    \instr_read_data5[27] ,
    \instr_read_data5[26] ,
    \instr_read_data5[25] ,
    \instr_read_data5[24] ,
    \instr_read_data5[23] ,
    \instr_read_data5[22] ,
    \instr_read_data5[21] ,
    \instr_read_data5[20] ,
    \instr_read_data5[19] ,
    \instr_read_data5[18] ,
    \instr_read_data5[17] ,
    \instr_read_data5[16] ,
    \instr_read_data5[15] ,
    \instr_read_data5[14] ,
    \instr_read_data5[13] ,
    \instr_read_data5[12] ,
    \instr_read_data5[11] ,
    \instr_read_data5[10] ,
    \instr_read_data5[9] ,
    \instr_read_data5[8] ,
    \instr_read_data5[7] ,
    \instr_read_data5[6] ,
    \instr_read_data5[5] ,
    \instr_read_data5[4] ,
    \instr_read_data5[3] ,
    \instr_read_data5[2] ,
    \instr_read_data5[1] ,
    \instr_read_data5[0] }),
    .instr_read_data6({\instr_read_data6[31] ,
    \instr_read_data6[30] ,
    \instr_read_data6[29] ,
    \instr_read_data6[28] ,
    \instr_read_data6[27] ,
    \instr_read_data6[26] ,
    \instr_read_data6[25] ,
    \instr_read_data6[24] ,
    \instr_read_data6[23] ,
    \instr_read_data6[22] ,
    \instr_read_data6[21] ,
    \instr_read_data6[20] ,
    \instr_read_data6[19] ,
    \instr_read_data6[18] ,
    \instr_read_data6[17] ,
    \instr_read_data6[16] ,
    \instr_read_data6[15] ,
    \instr_read_data6[14] ,
    \instr_read_data6[13] ,
    \instr_read_data6[12] ,
    \instr_read_data6[11] ,
    \instr_read_data6[10] ,
    \instr_read_data6[9] ,
    \instr_read_data6[8] ,
    \instr_read_data6[7] ,
    \instr_read_data6[6] ,
    \instr_read_data6[5] ,
    \instr_read_data6[4] ,
    \instr_read_data6[3] ,
    \instr_read_data6[2] ,
    \instr_read_data6[1] ,
    \instr_read_data6[0] }),
    .instr_read_data7({\instr_read_data7[31] ,
    \instr_read_data7[30] ,
    \instr_read_data7[29] ,
    \instr_read_data7[28] ,
    \instr_read_data7[27] ,
    \instr_read_data7[26] ,
    \instr_read_data7[25] ,
    \instr_read_data7[24] ,
    \instr_read_data7[23] ,
    \instr_read_data7[22] ,
    \instr_read_data7[21] ,
    \instr_read_data7[20] ,
    \instr_read_data7[19] ,
    \instr_read_data7[18] ,
    \instr_read_data7[17] ,
    \instr_read_data7[16] ,
    \instr_read_data7[15] ,
    \instr_read_data7[14] ,
    \instr_read_data7[13] ,
    \instr_read_data7[12] ,
    \instr_read_data7[11] ,
    \instr_read_data7[10] ,
    \instr_read_data7[9] ,
    \instr_read_data7[8] ,
    \instr_read_data7[7] ,
    \instr_read_data7[6] ,
    \instr_read_data7[5] ,
    \instr_read_data7[4] ,
    \instr_read_data7[3] ,
    \instr_read_data7[2] ,
    \instr_read_data7[1] ,
    \instr_read_data7[0] }),
    .instr_wmask({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }),
    .instr_write_data({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .io_in({io_in[37],
    io_in[36],
    io_in[35],
    io_in[34],
    io_in[33],
    io_in[32],
    io_in[31],
    io_in[30],
    io_in[29],
    io_in[28],
    io_in[27],
    io_in[26],
    io_in[25],
    io_in[24],
    io_in[23],
    io_in[22],
    io_in[21],
    io_in[20],
    io_in[19],
    io_in[18],
    io_in[17],
    io_in[16],
    io_in[15],
    io_in[14],
    io_in[13],
    io_in[12],
    io_in[11],
    io_in[10],
    io_in[9],
    io_in[8],
    io_in[7],
    io_in[6],
    io_in[5],
    io_in[4],
    io_in[3],
    io_in[2],
    io_in[1],
    io_in[0]}),
    .io_oeb({io_oeb[37],
    io_oeb[36],
    io_oeb[35],
    io_oeb[34],
    io_oeb[33],
    io_oeb[32],
    io_oeb[31],
    io_oeb[30],
    io_oeb[29],
    io_oeb[28],
    io_oeb[27],
    io_oeb[26],
    io_oeb[25],
    io_oeb[24],
    io_oeb[23],
    io_oeb[22],
    io_oeb[21],
    io_oeb[20],
    io_oeb[19],
    io_oeb[18],
    io_oeb[17],
    io_oeb[16],
    io_oeb[15],
    io_oeb[14],
    io_oeb[13],
    io_oeb[12],
    io_oeb[11],
    io_oeb[10],
    io_oeb[9],
    io_oeb[8],
    io_oeb[7],
    io_oeb[6],
    io_oeb[5],
    io_oeb[4],
    io_oeb[3],
    io_oeb[2],
    io_oeb[1],
    io_oeb[0]}),
    .io_out({io_out[37],
    io_out[36],
    io_out[35],
    io_out[34],
    io_out[33],
    io_out[32],
    io_out[31],
    io_out[30],
    io_out[29],
    io_out[28],
    io_out[27],
    io_out[26],
    io_out[25],
    io_out[24],
    io_out[23],
    io_out[22],
    io_out[21],
    io_out[20],
    io_out[19],
    io_out[18],
    io_out[17],
    io_out[16],
    io_out[15],
    io_out[14],
    io_out[13],
    io_out[12],
    io_out[11],
    io_out[10],
    io_out[9],
    io_out[8],
    io_out[7],
    io_out[6],
    io_out[5],
    io_out[4],
    io_out[3],
    io_out[2],
    io_out[1],
    io_out[0]}),
    .irq({user_irq[2],
    user_irq[1],
    user_irq[0]}),
    .la_data_out({la_data_out[127],
    la_data_out[126],
    la_data_out[125],
    la_data_out[124],
    la_data_out[123],
    la_data_out[122],
    la_data_out[121],
    la_data_out[120],
    la_data_out[119],
    la_data_out[118],
    la_data_out[117],
    la_data_out[116],
    la_data_out[115],
    la_data_out[114],
    la_data_out[113],
    la_data_out[112],
    la_data_out[111],
    la_data_out[110],
    la_data_out[109],
    la_data_out[108],
    la_data_out[107],
    la_data_out[106],
    la_data_out[105],
    la_data_out[104],
    la_data_out[103],
    la_data_out[102],
    la_data_out[101],
    la_data_out[100],
    la_data_out[99],
    la_data_out[98],
    la_data_out[97],
    la_data_out[96],
    la_data_out[95],
    la_data_out[94],
    la_data_out[93],
    la_data_out[92],
    la_data_out[91],
    la_data_out[90],
    la_data_out[89],
    la_data_out[88],
    la_data_out[87],
    la_data_out[86],
    la_data_out[85],
    la_data_out[84],
    la_data_out[83],
    la_data_out[82],
    la_data_out[81],
    la_data_out[80],
    la_data_out[79],
    la_data_out[78],
    la_data_out[77],
    la_data_out[76],
    la_data_out[75],
    la_data_out[74],
    la_data_out[73],
    la_data_out[72],
    la_data_out[71],
    la_data_out[70],
    la_data_out[69],
    la_data_out[68],
    la_data_out[67],
    la_data_out[66],
    la_data_out[65],
    la_data_out[64],
    la_data_out[63],
    la_data_out[62],
    la_data_out[61],
    la_data_out[60],
    la_data_out[59],
    la_data_out[58],
    la_data_out[57],
    la_data_out[56],
    la_data_out[55],
    la_data_out[54],
    la_data_out[53],
    la_data_out[52],
    la_data_out[51],
    la_data_out[50],
    la_data_out[49],
    la_data_out[48],
    la_data_out[47],
    la_data_out[46],
    la_data_out[45],
    la_data_out[44],
    la_data_out[43],
    la_data_out[42],
    la_data_out[41],
    la_data_out[40],
    la_data_out[39],
    la_data_out[38],
    la_data_out[37],
    la_data_out[36],
    la_data_out[35],
    la_data_out[34],
    la_data_out[33],
    la_data_out[32],
    la_data_out[31],
    la_data_out[30],
    la_data_out[29],
    la_data_out[28],
    la_data_out[27],
    la_data_out[26],
    la_data_out[25],
    la_data_out[24],
    la_data_out[23],
    la_data_out[22],
    la_data_out[21],
    la_data_out[20],
    la_data_out[19],
    la_data_out[18],
    la_data_out[17],
    la_data_out[16],
    la_data_out[15],
    la_data_out[14],
    la_data_out[13],
    la_data_out[12],
    la_data_out[11],
    la_data_out[10],
    la_data_out[9],
    la_data_out[8],
    la_data_out[7],
    la_data_out[6],
    la_data_out[5],
    la_data_out[4],
    la_data_out[3],
    la_data_out[2],
    la_data_out[1],
    la_data_out[0]}),
    .uP_data_mem_addr({\uP_data_mem_addr[7] ,
    \uP_data_mem_addr[6] ,
    \uP_data_mem_addr[5] ,
    \uP_data_mem_addr[4] ,
    \uP_data_mem_addr[3] ,
    \uP_data_mem_addr[2] ,
    \uP_data_mem_addr[1] ,
    \uP_data_mem_addr[0] }),
    .uP_instr({\uP_instr[15] ,
    \uP_instr[14] ,
    \uP_instr[13] ,
    \uP_instr[12] ,
    \uP_instr[11] ,
    \uP_instr[10] ,
    \uP_instr[9] ,
    \uP_instr[8] ,
    \uP_instr[7] ,
    \uP_instr[6] ,
    \uP_instr[5] ,
    \uP_instr[4] ,
    \uP_instr[3] ,
    \uP_instr[2] ,
    \uP_instr[1] ,
    \uP_instr[0] }),
    .uP_instr_mem_addr({\uP_instr_mem_addr[12] ,
    \uP_instr_mem_addr[11] ,
    \uP_instr_mem_addr[10] ,
    \uP_instr_mem_addr[9] ,
    \uP_instr_mem_addr[8] ,
    \uP_instr_mem_addr[7] ,
    \uP_instr_mem_addr[6] ,
    \uP_instr_mem_addr[5] ,
    \uP_instr_mem_addr[4] ,
    \uP_instr_mem_addr[3] ,
    \uP_instr_mem_addr[2] ,
    \uP_instr_mem_addr[1] ,
    \uP_instr_mem_addr[0] }),
    .uP_read_data({\uP_read_data[15] ,
    \uP_read_data[14] ,
    \uP_read_data[13] ,
    \uP_read_data[12] ,
    \uP_read_data[11] ,
    \uP_read_data[10] ,
    \uP_read_data[9] ,
    \uP_read_data[8] ,
    \uP_read_data[7] ,
    \uP_read_data[6] ,
    \uP_read_data[5] ,
    \uP_read_data[4] ,
    \uP_read_data[3] ,
    \uP_read_data[2] ,
    \uP_read_data[1] ,
    \uP_read_data[0] }),
    .uP_write_data({\uP_write_data[15] ,
    \uP_write_data[14] ,
    \uP_write_data[13] ,
    \uP_write_data[12] ,
    \uP_write_data[11] ,
    \uP_write_data[10] ,
    \uP_write_data[9] ,
    \uP_write_data[8] ,
    \uP_write_data[7] ,
    \uP_write_data[6] ,
    \uP_write_data[5] ,
    \uP_write_data[4] ,
    \uP_write_data[3] ,
    \uP_write_data[2] ,
    \uP_write_data[1] ,
    \uP_write_data[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 data_memory (.csb0(data_mem_csb),
    .csb1(low),
    .web0(dataw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({low,
    \data_mem_addr[7] ,
    \data_mem_addr[6] ,
    \data_mem_addr[5] ,
    \data_mem_addr[4] ,
    \data_mem_addr[3] ,
    \data_mem_addr[2] ,
    \data_mem_addr[1] ,
    \data_mem_addr[0] }),
    .addr1({_NC1,
    _NC2,
    _NC3,
    _NC4,
    _NC5,
    _NC6,
    _NC7,
    _NC8,
    _NC9}),
    .din0({\data_write_data[31] ,
    \data_write_data[30] ,
    \data_write_data[29] ,
    \data_write_data[28] ,
    \data_write_data[27] ,
    \data_write_data[26] ,
    \data_write_data[25] ,
    \data_write_data[24] ,
    \data_write_data[23] ,
    \data_write_data[22] ,
    \data_write_data[21] ,
    \data_write_data[20] ,
    \data_write_data[19] ,
    \data_write_data[18] ,
    \data_write_data[17] ,
    \data_write_data[16] ,
    \data_write_data[15] ,
    \data_write_data[14] ,
    \data_write_data[13] ,
    \data_write_data[12] ,
    \data_write_data[11] ,
    \data_write_data[10] ,
    \data_write_data[9] ,
    \data_write_data[8] ,
    \data_write_data[7] ,
    \data_write_data[6] ,
    \data_write_data[5] ,
    \data_write_data[4] ,
    \data_write_data[3] ,
    \data_write_data[2] ,
    \data_write_data[1] ,
    \data_write_data[0] }),
    .dout0({\data_read_data[31] ,
    \data_read_data[30] ,
    \data_read_data[29] ,
    \data_read_data[28] ,
    \data_read_data[27] ,
    \data_read_data[26] ,
    \data_read_data[25] ,
    \data_read_data[24] ,
    \data_read_data[23] ,
    \data_read_data[22] ,
    \data_read_data[21] ,
    \data_read_data[20] ,
    \data_read_data[19] ,
    \data_read_data[18] ,
    \data_read_data[17] ,
    \data_read_data[16] ,
    \data_read_data[15] ,
    \data_read_data[14] ,
    \data_read_data[13] ,
    \data_read_data[12] ,
    \data_read_data[11] ,
    \data_read_data[10] ,
    \data_read_data[9] ,
    \data_read_data[8] ,
    \data_read_data[7] ,
    \data_read_data[6] ,
    \data_read_data[5] ,
    \data_read_data[4] ,
    \data_read_data[3] ,
    \data_read_data[2] ,
    \data_read_data[1] ,
    \data_read_data[0] }),
    .dout1({_NC10,
    _NC11,
    _NC12,
    _NC13,
    _NC14,
    _NC15,
    _NC16,
    _NC17,
    _NC18,
    _NC19,
    _NC20,
    _NC21,
    _NC22,
    _NC23,
    _NC24,
    _NC25,
    _NC26,
    _NC27,
    _NC28,
    _NC29,
    _NC30,
    _NC31,
    _NC32,
    _NC33,
    _NC34,
    _NC35,
    _NC36,
    _NC37,
    _NC38,
    _NC39,
    _NC40,
    _NC41}),
    .wmask0({\data_wmask[3] ,
    \data_wmask[2] ,
    \data_wmask[1] ,
    \data_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory0 (.csb0(\instr_mem_csb[0] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC42,
    _NC43,
    _NC44,
    _NC45,
    _NC46,
    _NC47,
    _NC48,
    _NC49,
    _NC50}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data0[31] ,
    \instr_read_data0[30] ,
    \instr_read_data0[29] ,
    \instr_read_data0[28] ,
    \instr_read_data0[27] ,
    \instr_read_data0[26] ,
    \instr_read_data0[25] ,
    \instr_read_data0[24] ,
    \instr_read_data0[23] ,
    \instr_read_data0[22] ,
    \instr_read_data0[21] ,
    \instr_read_data0[20] ,
    \instr_read_data0[19] ,
    \instr_read_data0[18] ,
    \instr_read_data0[17] ,
    \instr_read_data0[16] ,
    \instr_read_data0[15] ,
    \instr_read_data0[14] ,
    \instr_read_data0[13] ,
    \instr_read_data0[12] ,
    \instr_read_data0[11] ,
    \instr_read_data0[10] ,
    \instr_read_data0[9] ,
    \instr_read_data0[8] ,
    \instr_read_data0[7] ,
    \instr_read_data0[6] ,
    \instr_read_data0[5] ,
    \instr_read_data0[4] ,
    \instr_read_data0[3] ,
    \instr_read_data0[2] ,
    \instr_read_data0[1] ,
    \instr_read_data0[0] }),
    .dout1({_NC51,
    _NC52,
    _NC53,
    _NC54,
    _NC55,
    _NC56,
    _NC57,
    _NC58,
    _NC59,
    _NC60,
    _NC61,
    _NC62,
    _NC63,
    _NC64,
    _NC65,
    _NC66,
    _NC67,
    _NC68,
    _NC69,
    _NC70,
    _NC71,
    _NC72,
    _NC73,
    _NC74,
    _NC75,
    _NC76,
    _NC77,
    _NC78,
    _NC79,
    _NC80,
    _NC81,
    _NC82}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory1 (.csb0(\instr_mem_csb[1] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC83,
    _NC84,
    _NC85,
    _NC86,
    _NC87,
    _NC88,
    _NC89,
    _NC90,
    _NC91}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data1[31] ,
    \instr_read_data1[30] ,
    \instr_read_data1[29] ,
    \instr_read_data1[28] ,
    \instr_read_data1[27] ,
    \instr_read_data1[26] ,
    \instr_read_data1[25] ,
    \instr_read_data1[24] ,
    \instr_read_data1[23] ,
    \instr_read_data1[22] ,
    \instr_read_data1[21] ,
    \instr_read_data1[20] ,
    \instr_read_data1[19] ,
    \instr_read_data1[18] ,
    \instr_read_data1[17] ,
    \instr_read_data1[16] ,
    \instr_read_data1[15] ,
    \instr_read_data1[14] ,
    \instr_read_data1[13] ,
    \instr_read_data1[12] ,
    \instr_read_data1[11] ,
    \instr_read_data1[10] ,
    \instr_read_data1[9] ,
    \instr_read_data1[8] ,
    \instr_read_data1[7] ,
    \instr_read_data1[6] ,
    \instr_read_data1[5] ,
    \instr_read_data1[4] ,
    \instr_read_data1[3] ,
    \instr_read_data1[2] ,
    \instr_read_data1[1] ,
    \instr_read_data1[0] }),
    .dout1({_NC92,
    _NC93,
    _NC94,
    _NC95,
    _NC96,
    _NC97,
    _NC98,
    _NC99,
    _NC100,
    _NC101,
    _NC102,
    _NC103,
    _NC104,
    _NC105,
    _NC106,
    _NC107,
    _NC108,
    _NC109,
    _NC110,
    _NC111,
    _NC112,
    _NC113,
    _NC114,
    _NC115,
    _NC116,
    _NC117,
    _NC118,
    _NC119,
    _NC120,
    _NC121,
    _NC122,
    _NC123}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory2 (.csb0(\instr_mem_csb[2] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC124,
    _NC125,
    _NC126,
    _NC127,
    _NC128,
    _NC129,
    _NC130,
    _NC131,
    _NC132}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data2[31] ,
    \instr_read_data2[30] ,
    \instr_read_data2[29] ,
    \instr_read_data2[28] ,
    \instr_read_data2[27] ,
    \instr_read_data2[26] ,
    \instr_read_data2[25] ,
    \instr_read_data2[24] ,
    \instr_read_data2[23] ,
    \instr_read_data2[22] ,
    \instr_read_data2[21] ,
    \instr_read_data2[20] ,
    \instr_read_data2[19] ,
    \instr_read_data2[18] ,
    \instr_read_data2[17] ,
    \instr_read_data2[16] ,
    \instr_read_data2[15] ,
    \instr_read_data2[14] ,
    \instr_read_data2[13] ,
    \instr_read_data2[12] ,
    \instr_read_data2[11] ,
    \instr_read_data2[10] ,
    \instr_read_data2[9] ,
    \instr_read_data2[8] ,
    \instr_read_data2[7] ,
    \instr_read_data2[6] ,
    \instr_read_data2[5] ,
    \instr_read_data2[4] ,
    \instr_read_data2[3] ,
    \instr_read_data2[2] ,
    \instr_read_data2[1] ,
    \instr_read_data2[0] }),
    .dout1({_NC133,
    _NC134,
    _NC135,
    _NC136,
    _NC137,
    _NC138,
    _NC139,
    _NC140,
    _NC141,
    _NC142,
    _NC143,
    _NC144,
    _NC145,
    _NC146,
    _NC147,
    _NC148,
    _NC149,
    _NC150,
    _NC151,
    _NC152,
    _NC153,
    _NC154,
    _NC155,
    _NC156,
    _NC157,
    _NC158,
    _NC159,
    _NC160,
    _NC161,
    _NC162,
    _NC163,
    _NC164}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory3 (.csb0(\instr_mem_csb[3] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC165,
    _NC166,
    _NC167,
    _NC168,
    _NC169,
    _NC170,
    _NC171,
    _NC172,
    _NC173}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data3[31] ,
    \instr_read_data3[30] ,
    \instr_read_data3[29] ,
    \instr_read_data3[28] ,
    \instr_read_data3[27] ,
    \instr_read_data3[26] ,
    \instr_read_data3[25] ,
    \instr_read_data3[24] ,
    \instr_read_data3[23] ,
    \instr_read_data3[22] ,
    \instr_read_data3[21] ,
    \instr_read_data3[20] ,
    \instr_read_data3[19] ,
    \instr_read_data3[18] ,
    \instr_read_data3[17] ,
    \instr_read_data3[16] ,
    \instr_read_data3[15] ,
    \instr_read_data3[14] ,
    \instr_read_data3[13] ,
    \instr_read_data3[12] ,
    \instr_read_data3[11] ,
    \instr_read_data3[10] ,
    \instr_read_data3[9] ,
    \instr_read_data3[8] ,
    \instr_read_data3[7] ,
    \instr_read_data3[6] ,
    \instr_read_data3[5] ,
    \instr_read_data3[4] ,
    \instr_read_data3[3] ,
    \instr_read_data3[2] ,
    \instr_read_data3[1] ,
    \instr_read_data3[0] }),
    .dout1({_NC174,
    _NC175,
    _NC176,
    _NC177,
    _NC178,
    _NC179,
    _NC180,
    _NC181,
    _NC182,
    _NC183,
    _NC184,
    _NC185,
    _NC186,
    _NC187,
    _NC188,
    _NC189,
    _NC190,
    _NC191,
    _NC192,
    _NC193,
    _NC194,
    _NC195,
    _NC196,
    _NC197,
    _NC198,
    _NC199,
    _NC200,
    _NC201,
    _NC202,
    _NC203,
    _NC204,
    _NC205}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory4 (.csb0(\instr_mem_csb[4] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC206,
    _NC207,
    _NC208,
    _NC209,
    _NC210,
    _NC211,
    _NC212,
    _NC213,
    _NC214}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data4[31] ,
    \instr_read_data4[30] ,
    \instr_read_data4[29] ,
    \instr_read_data4[28] ,
    \instr_read_data4[27] ,
    \instr_read_data4[26] ,
    \instr_read_data4[25] ,
    \instr_read_data4[24] ,
    \instr_read_data4[23] ,
    \instr_read_data4[22] ,
    \instr_read_data4[21] ,
    \instr_read_data4[20] ,
    \instr_read_data4[19] ,
    \instr_read_data4[18] ,
    \instr_read_data4[17] ,
    \instr_read_data4[16] ,
    \instr_read_data4[15] ,
    \instr_read_data4[14] ,
    \instr_read_data4[13] ,
    \instr_read_data4[12] ,
    \instr_read_data4[11] ,
    \instr_read_data4[10] ,
    \instr_read_data4[9] ,
    \instr_read_data4[8] ,
    \instr_read_data4[7] ,
    \instr_read_data4[6] ,
    \instr_read_data4[5] ,
    \instr_read_data4[4] ,
    \instr_read_data4[3] ,
    \instr_read_data4[2] ,
    \instr_read_data4[1] ,
    \instr_read_data4[0] }),
    .dout1({_NC215,
    _NC216,
    _NC217,
    _NC218,
    _NC219,
    _NC220,
    _NC221,
    _NC222,
    _NC223,
    _NC224,
    _NC225,
    _NC226,
    _NC227,
    _NC228,
    _NC229,
    _NC230,
    _NC231,
    _NC232,
    _NC233,
    _NC234,
    _NC235,
    _NC236,
    _NC237,
    _NC238,
    _NC239,
    _NC240,
    _NC241,
    _NC242,
    _NC243,
    _NC244,
    _NC245,
    _NC246}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory5 (.csb0(\instr_mem_csb[5] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC247,
    _NC248,
    _NC249,
    _NC250,
    _NC251,
    _NC252,
    _NC253,
    _NC254,
    _NC255}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data5[31] ,
    \instr_read_data5[30] ,
    \instr_read_data5[29] ,
    \instr_read_data5[28] ,
    \instr_read_data5[27] ,
    \instr_read_data5[26] ,
    \instr_read_data5[25] ,
    \instr_read_data5[24] ,
    \instr_read_data5[23] ,
    \instr_read_data5[22] ,
    \instr_read_data5[21] ,
    \instr_read_data5[20] ,
    \instr_read_data5[19] ,
    \instr_read_data5[18] ,
    \instr_read_data5[17] ,
    \instr_read_data5[16] ,
    \instr_read_data5[15] ,
    \instr_read_data5[14] ,
    \instr_read_data5[13] ,
    \instr_read_data5[12] ,
    \instr_read_data5[11] ,
    \instr_read_data5[10] ,
    \instr_read_data5[9] ,
    \instr_read_data5[8] ,
    \instr_read_data5[7] ,
    \instr_read_data5[6] ,
    \instr_read_data5[5] ,
    \instr_read_data5[4] ,
    \instr_read_data5[3] ,
    \instr_read_data5[2] ,
    \instr_read_data5[1] ,
    \instr_read_data5[0] }),
    .dout1({_NC256,
    _NC257,
    _NC258,
    _NC259,
    _NC260,
    _NC261,
    _NC262,
    _NC263,
    _NC264,
    _NC265,
    _NC266,
    _NC267,
    _NC268,
    _NC269,
    _NC270,
    _NC271,
    _NC272,
    _NC273,
    _NC274,
    _NC275,
    _NC276,
    _NC277,
    _NC278,
    _NC279,
    _NC280,
    _NC281,
    _NC282,
    _NC283,
    _NC284,
    _NC285,
    _NC286,
    _NC287}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory6 (.csb0(\instr_mem_csb[6] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC288,
    _NC289,
    _NC290,
    _NC291,
    _NC292,
    _NC293,
    _NC294,
    _NC295,
    _NC296}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data6[31] ,
    \instr_read_data6[30] ,
    \instr_read_data6[29] ,
    \instr_read_data6[28] ,
    \instr_read_data6[27] ,
    \instr_read_data6[26] ,
    \instr_read_data6[25] ,
    \instr_read_data6[24] ,
    \instr_read_data6[23] ,
    \instr_read_data6[22] ,
    \instr_read_data6[21] ,
    \instr_read_data6[20] ,
    \instr_read_data6[19] ,
    \instr_read_data6[18] ,
    \instr_read_data6[17] ,
    \instr_read_data6[16] ,
    \instr_read_data6[15] ,
    \instr_read_data6[14] ,
    \instr_read_data6[13] ,
    \instr_read_data6[12] ,
    \instr_read_data6[11] ,
    \instr_read_data6[10] ,
    \instr_read_data6[9] ,
    \instr_read_data6[8] ,
    \instr_read_data6[7] ,
    \instr_read_data6[6] ,
    \instr_read_data6[5] ,
    \instr_read_data6[4] ,
    \instr_read_data6[3] ,
    \instr_read_data6[2] ,
    \instr_read_data6[1] ,
    \instr_read_data6[0] }),
    .dout1({_NC297,
    _NC298,
    _NC299,
    _NC300,
    _NC301,
    _NC302,
    _NC303,
    _NC304,
    _NC305,
    _NC306,
    _NC307,
    _NC308,
    _NC309,
    _NC310,
    _NC311,
    _NC312,
    _NC313,
    _NC314,
    _NC315,
    _NC316,
    _NC317,
    _NC318,
    _NC319,
    _NC320,
    _NC321,
    _NC322,
    _NC323,
    _NC324,
    _NC325,
    _NC326,
    _NC327,
    _NC328}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 sky130_sram_2kbyte_1rw1r_32x512_8 instr_memory7 (.csb0(\instr_mem_csb[7] ),
    .csb1(low),
    .web0(instrw_enb),
    .clk0(clk),
    .clk1(clk),
    .addr0({\instr_mem_addr_9bit[8] ,
    \instr_mem_addr_9bit[7] ,
    \instr_mem_addr_9bit[6] ,
    \instr_mem_addr_9bit[5] ,
    \instr_mem_addr_9bit[4] ,
    \instr_mem_addr_9bit[3] ,
    \instr_mem_addr_9bit[2] ,
    \instr_mem_addr_9bit[1] ,
    \instr_mem_addr_9bit[0] }),
    .addr1({_NC329,
    _NC330,
    _NC331,
    _NC332,
    _NC333,
    _NC334,
    _NC335,
    _NC336,
    _NC337}),
    .din0({\instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] ,
    \instr_write_data[15] ,
    \instr_write_data[14] ,
    \instr_write_data[13] ,
    \instr_write_data[12] ,
    \instr_write_data[11] ,
    \instr_write_data[10] ,
    \instr_write_data[9] ,
    \instr_write_data[8] ,
    \instr_write_data[7] ,
    \instr_write_data[6] ,
    \instr_write_data[5] ,
    \instr_write_data[4] ,
    \instr_write_data[3] ,
    \instr_write_data[2] ,
    \instr_write_data[1] ,
    \instr_write_data[0] }),
    .dout0({\instr_read_data7[31] ,
    \instr_read_data7[30] ,
    \instr_read_data7[29] ,
    \instr_read_data7[28] ,
    \instr_read_data7[27] ,
    \instr_read_data7[26] ,
    \instr_read_data7[25] ,
    \instr_read_data7[24] ,
    \instr_read_data7[23] ,
    \instr_read_data7[22] ,
    \instr_read_data7[21] ,
    \instr_read_data7[20] ,
    \instr_read_data7[19] ,
    \instr_read_data7[18] ,
    \instr_read_data7[17] ,
    \instr_read_data7[16] ,
    \instr_read_data7[15] ,
    \instr_read_data7[14] ,
    \instr_read_data7[13] ,
    \instr_read_data7[12] ,
    \instr_read_data7[11] ,
    \instr_read_data7[10] ,
    \instr_read_data7[9] ,
    \instr_read_data7[8] ,
    \instr_read_data7[7] ,
    \instr_read_data7[6] ,
    \instr_read_data7[5] ,
    \instr_read_data7[4] ,
    \instr_read_data7[3] ,
    \instr_read_data7[2] ,
    \instr_read_data7[1] ,
    \instr_read_data7[0] }),
    .dout1({_NC338,
    _NC339,
    _NC340,
    _NC341,
    _NC342,
    _NC343,
    _NC344,
    _NC345,
    _NC346,
    _NC347,
    _NC348,
    _NC349,
    _NC350,
    _NC351,
    _NC352,
    _NC353,
    _NC354,
    _NC355,
    _NC356,
    _NC357,
    _NC358,
    _NC359,
    _NC360,
    _NC361,
    _NC362,
    _NC363,
    _NC364,
    _NC365,
    _NC366,
    _NC367,
    _NC368,
    _NC369}),
    .wmask0({\instr_wmask[3] ,
    \instr_wmask[2] ,
    \instr_wmask[1] ,
    \instr_wmask[0] }));
 processor uP (.Dataw_en(uP_dataw_en),
    .Serial_input(Serial_input),
    .Serial_output(Serial_output),
    .clk(clk),
    .hlt(hlt),
    .reset(reset),
    .start(start),
    .data_mem_addr({\uP_data_mem_addr[7] ,
    \uP_data_mem_addr[6] ,
    \uP_data_mem_addr[5] ,
    \uP_data_mem_addr[4] ,
    \uP_data_mem_addr[3] ,
    \uP_data_mem_addr[2] ,
    \uP_data_mem_addr[1] ,
    \uP_data_mem_addr[0] }),
    .instr({\uP_instr[15] ,
    \uP_instr[14] ,
    \uP_instr[13] ,
    \uP_instr[12] ,
    \uP_instr[11] ,
    \uP_instr[10] ,
    \uP_instr[9] ,
    \uP_instr[8] ,
    \uP_instr[7] ,
    \uP_instr[6] ,
    \uP_instr[5] ,
    \uP_instr[4] ,
    \uP_instr[3] ,
    \uP_instr[2] ,
    \uP_instr[1] ,
    \uP_instr[0] }),
    .instr_mem_addr({\uP_instr_mem_addr[12] ,
    \uP_instr_mem_addr[11] ,
    \uP_instr_mem_addr[10] ,
    \uP_instr_mem_addr[9] ,
    \uP_instr_mem_addr[8] ,
    \uP_instr_mem_addr[7] ,
    \uP_instr_mem_addr[6] ,
    \uP_instr_mem_addr[5] ,
    \uP_instr_mem_addr[4] ,
    \uP_instr_mem_addr[3] ,
    \uP_instr_mem_addr[2] ,
    \uP_instr_mem_addr[1] ,
    \uP_instr_mem_addr[0] }),
    .read_data({\uP_read_data[15] ,
    \uP_read_data[14] ,
    \uP_read_data[13] ,
    \uP_read_data[12] ,
    \uP_read_data[11] ,
    \uP_read_data[10] ,
    \uP_read_data[9] ,
    \uP_read_data[8] ,
    \uP_read_data[7] ,
    \uP_read_data[6] ,
    \uP_read_data[5] ,
    \uP_read_data[4] ,
    \uP_read_data[3] ,
    \uP_read_data[2] ,
    \uP_read_data[1] ,
    \uP_read_data[0] }),
    .write_data({\uP_write_data[15] ,
    \uP_write_data[14] ,
    \uP_write_data[13] ,
    \uP_write_data[12] ,
    \uP_write_data[11] ,
    \uP_write_data[10] ,
    \uP_write_data[9] ,
    \uP_write_data[8] ,
    \uP_write_data[7] ,
    \uP_write_data[6] ,
    \uP_write_data[5] ,
    \uP_write_data[4] ,
    \uP_write_data[3] ,
    \uP_write_data[2] ,
    \uP_write_data[1] ,
    \uP_write_data[0] }));
endmodule

