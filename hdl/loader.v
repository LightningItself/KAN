`timescale 1ns/1ps

module LOADER #(
    parameter S_AXIS_DATAWIDTH = 16,
    parameter M_AXIS_DATAWIDTH = 16,
    parameter ADDRWIDTH        = 32,
    parameter LG_LAYERSIZE     = 12,
    parameter HBITS            = 4
    // parameter BASE_ADDR        = 32'h00080000,
    // parameter ZERO_ADDR        = 32'h00010000
)(
    input wire                                  s_axis_aclk,
    input wire                                  s_axis_aresetn,

    input wire signed [S_AXIS_DATAWIDTH-1:0]    s_axis_tdata,
    input wire                                  s_axis_tvalid,
    output reg                                  s_axis_tready,

    output wire [M_AXIS_DATAWIDTH-1:0]          m_axis_tdata,
    output wire                                 m_axis_tvalid,
    input wire                                  m_axis_tready,

    input wire [31:0]                           BASE_ADDR,
    input wire [31:0]                           ZERO_ADDR,
    input wire [LG_LAYERSIZE-1:0]               inp_layer_size,
    input wire [LG_LAYERSIZE-1:0]               out_layer_size, //multiple of 4

    output reg [ADDRWIDTH-1:0]                  m_axi_araddr,
    output reg                                  m_axi_arvalid,
    input wire                                  m_axi_arready,
    output reg [7:0]                            m_axi_arlen,
    output reg [2:0]                            m_axi_arsize,
    output reg [1:0]                            m_axi_arburst
);

localparam GRID_SIZE = (1<<(HBITS+1));

wire [9:0] o_data_1, o_data_2, o_data_3, o_data_4;

wire en = s_axis_tvalid & s_axis_tready & s_axis_aresetn;

SPLINEGEN #(S_AXIS_DATAWIDTH, HBITS, 10) spline_generator (
    s_axis_aclk,
    en,
    s_axis_tdata,
    o_data_1,
    o_data_2,
    o_data_3,
    o_data_4
);

wire        [9:0]   sig_out;
reg signed  [31:0]  silu_out;
reg                 sig_overflow;
reg                 sig_underflow;

SIGMOID sigmoid_generator (
    s_axis_aclk,
    en,
    s_axis_tdata[13:0],
    sig_out
);

// get non zero spline index
reg         [13:0]  grid_id;
wire                grid_id_valid = grid_id < GRID_SIZE;
reg                 buff_axis_tvalid;
reg         [79:0]  buff_axis_tdata;
wire                buff_axis_tready;
reg signed  [15:0]  inp_data;

wire signed [15:0] sig_out_16 = sig_overflow ? 16'h0400 : sig_underflow ? 16'h0000 : {6'b000000, sig_out};

reg [2:0]   burst_counter;
reg [11:0]  node_counter;
reg [19:0]  curr_address;
reg [19:0]  node_address;
reg [1:0]   state;

localparam STATE_INPUT  = 0;
localparam STATE_OUTPUT = 1;
localparam STATE_BUFFER = 2;

always @(posedge s_axis_aclk) begin
    if(~s_axis_aresetn) begin
        state <= STATE_INPUT;
        burst_counter <= 0;
        node_counter <= 0;
        curr_address <= BASE_ADDR[31:12];
        node_address <= BASE_ADDR[31:12];
        grid_id <= 0;
        inp_data <= 0;
        sig_underflow <= 1;
        sig_overflow <= 1;
    end
    else begin
        case(state)
            STATE_INPUT: begin
                if(s_axis_tvalid) begin
                    state <= STATE_OUTPUT;
                    inp_data <= s_axis_tdata;
                    sig_overflow  <= (|s_axis_tdata[14:13]) & (~s_axis_tdata[15]);
                    sig_underflow <= ~(&s_axis_tdata[14:13]) & s_axis_tdata[15];
                    curr_address <= node_address + GRID_SIZE;
                    grid_id <= (s_axis_tdata >>> (10-HBITS)) + (1<<HBITS);
                end
            end
            STATE_OUTPUT: begin
                if(m_axi_arready) begin
                    if(burst_counter == 4)
                        state <= STATE_BUFFER;
                    //NODE ADDRESS LOGIC
                    if(burst_counter == 4)
                        node_address <= node_address + (GRID_SIZE+1);
                    //CURR ADDRESS LOGIC
                    if(burst_counter == 0) 
                        curr_address <= node_address + grid_id;
                    else 
                        curr_address <= curr_address + 1;
                    if(burst_counter == 4) 
                        burst_counter <= 0;
                    else
                        burst_counter <= burst_counter+1;
                end
            end
            STATE_BUFFER: begin
                if(m_axis_tready) begin
                    state <= STATE_INPUT;
                    if(node_counter == inp_layer_size-1)begin
                        node_counter <= 0;
                        node_address <= BASE_ADDR;
                    end
                    else
                        node_counter <= node_counter+1;
                end
            end
        endcase
    end
end

always @(posedge s_axis_aclk) begin
    silu_out <= inp_data * sig_out_16;
    

end

always @(*) begin
    buff_axis_tdata = {silu_out[25:10], {6'd0, o_data_4}, {6'd0, o_data_3}, {6'd0, o_data_2}, {6'd0, o_data_1}};

    m_axi_araddr = grid_id_valid ? curr_address<<12 : ZERO_ADDR;
    m_axi_arlen = ((out_layer_size>>2)-1);
    m_axi_arsize = 3'b011;
    m_axi_arburst = 2'b01;

    if(~s_axis_aresetn) begin
        s_axis_tready = 0;
        m_axi_arvalid = 0;
        buff_axis_tvalid = 0;
    end 
    else begin
        case(state)
            STATE_INPUT: begin
                s_axis_tready = 1;
                m_axi_arvalid = 0;
                buff_axis_tvalid = 0;
            end
            STATE_OUTPUT: begin
                s_axis_tready = 0;
                m_axi_arvalid = 1;
                buff_axis_tvalid = 0;
            end
            STATE_BUFFER: begin
                s_axis_tready = 0;
                m_axi_arvalid = 0;
                buff_axis_tvalid = 1;
            end
            default: begin
                s_axis_tready = 0;
                m_axi_arvalid = 0;
                buff_axis_tvalid = 0;
            end
        endcase
    end
end

//BUFFER MODULE
axis_dwidth_converter_0 buffer (
  s_axis_aclk,
  s_axis_aresetn,
  buff_axis_tvalid,
  buff_axis_tready,
  buff_axis_tdata,
  m_axis_tvalid,
  m_axis_tready,
  m_axis_tdata
);

endmodule


/*** 

for each input layer,
    for each basis value -> output node times values are needed
        for each input node -> 5 basis values,

DDR Addressing -> Simplification

Boundary -> 2**12 === 4096/2 == 2048 weight values.
Therefore, for each input node in a KAN, we need 5 different ranges of weight values. 4 basis + 1 SiLU
Hence, make start of each such burst at boundary. each burst can take multiple such ranges.

MAX weights per burst -> 2048. Hence, max layer size for simplified design -> 2048

addressing ->

|       input node 1        |x11 x12 x13       input node 2         |       input node 3        |      input node 4      |

for each input node section in memory ->

g_id 1   |     g_id 2    |   g_id 3     |    .....   |   SiLU weights at end   |


L_XXXX_BBB

4  L bits -> supports 16 layers
12 B bits -> supports 2048 output layer size

16 X bits -> for input layer size * grid size


baseaddress -> 0_0000_000
for first input node -> 0_000D_000, 0_000E_000, 0_000F_000, 0_0010_000, 0_001D_000 <- base + grid_size value for SiLU weights
for second input node -> increment address by 1_000. 
This is because next input node weight starts after SiLU weights which are at the end of first input node weights.

we make four burst requests + 1 burst request
if out of bounds, we default to another addr with zeros loaded.

addressing -> 

layer baseaddress -> 32 bit address during config

for each node input -> 
    make four AXI burst request


for each layer -> 3 bits ()
for each input -> 10 bits ()
for each grid -> 7bits (max grid size 128)
for each output -> burst (12 bits)

layer base address -> L_addr

address = L_addr + curr_inp_node*(5*grid_size) -> start of each input node region

4 bursts at id, id+1, id+2, id+3, then at grid_size.


inp_node_addr = BASE_ADDR;
for every inp_node, inp_node_addr += (grid_size+1).

for each input node, we make 5 burst reads. AT->

inp_node_addr + id
inp_node_addr + id+1
inp_node_addr + id+2
inp_node_addr + id+3
inp_node_addr + grid_size

one address operation is allowed per cycle.
inp_node_addr. init as BASE_ADDRESS
for 1 ->
curr_address = inp_node_addr + id
for 2, 3, 4
curr_address ++
for 5
curr_address = inp_node_addr + grid_size.
inp_node_addr = inp_node_addr + (grid_size+1)



***/

