`timescale 1ns/1ps

module SPLINEGEN #(
    parameter S_AXIS_DATAWIDTH = 16,
    parameter HBITS = 4,
    parameter O_DATA_WIDTH = 10
)(
    input wire aclk,
    input wire i_en,
    input wire [S_AXIS_DATAWIDTH-1:0] data,
    output wire [O_DATA_WIDTH-1:0] o_data_1,
    output wire [O_DATA_WIDTH-1:0] o_data_2,
    output wire [O_DATA_WIDTH-1:0] o_data_3,
    output wire [O_DATA_WIDTH-1:0] o_data_4
);

wire [31:0] x_abs = data[15] ? ~data+1 : data;
wire [9:0] x_rem_p = x_abs[9:0] & (10'b1111111111 >> HBITS);
wire [9:0] x_addr_p = x_rem_p << (HBITS-1);
wire [9:0] x_addr_n = 10'b0111111111 - x_addr_p;

sp_rom_a store_a (
    aclk,
    i_en,
    x_addr_p,
    x_addr_n,
    o_data_3,
    o_data_2
);

sp_rom_b store_b (
    aclk,
    i_en,
    ({1'b1, x_addr_p[8:0]}),
    ({1'b1, x_addr_n[8:0]}),
    o_data_4,
    o_data_1
);

endmodule