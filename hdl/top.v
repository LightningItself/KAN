module ACCELKAN #(
    parameter S_AXIS_DATAWIDTH = 64,
    parameter M_AXIS_DATAWIDTH = 64,
    parameter M_AXI_DDR_DATAWIDTH  = 64,
    parameter S_AXIS_FIFO_DATAWIDTH = 16,
    parameter M_AXIS_FIFO_DATAWIDTH = 64,
    parameter ADDRWIDTH        = 32,
    parameter LG_LAYERSIZE     = 12
)(
    input wire s_axis_aclk,
    input wire s_axis_aresetn,

    //Input interface
    input  wire [S_AXIS_DATAWIDTH-1:0]   s_axis_tdata,
    input  wire                          s_axis_tvalid,
    input  wire                          s_axis_tlast,
    output wire                          s_axis_tready,

    //Output interface
    output wire [M_AXIS_DATAWIDTH-1:0]   m_axis_tdata,
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    input  wire                          m_axis_tready,

    //DDR interface
    output wire [ADDRWIDTH-1:0]          m_axi_araddr,
    output wire                          m_axi_arvalid,
    input  wire                          m_axi_arready,
    output wire [7:0]                    m_axi_arlen,
    output wire [2:0]                    m_axi_arsize,
    output wire [1:0]                    m_axi_arburst,

    input wire                           m_axi_rvalid,
    output wire                          m_axi_rready,
    input wire [M_AXI_DDR_DATAWIDTH-1:0] m_axi_rdata,
    input wire                           m_axi_rlast,
    input wire [1:0]                     m_axi_rresp
);  

localparam input_layersize = 10;
localparam output_layersize = 16;
localparam base_address = 32'h00080000;
localparam zero_address = 32'h00070000;

// reg [ADDRWIDTH-1:0]          base_address;
// reg [LG_LAYERSIZE-1:0]       input_layersize; //multiple of 4
// reg [LG_LAYERSIZE-1:0]       output_layersize; //multiple of 4
// reg [3:0]                    hbits;

// always @(*) begin
//     base_address = 32'h0008ffd0;
//     input_layersize = 1;
//     output_layersize = 8;
//     hbits = 3;
// end

wire [15:0] axis_fifo_tdata;
wire axis_fifo_tvalid;
wire axis_fifo_tready;

LOADER #(
    .S_AXIS_DATAWIDTH(16),
    .M_AXIS_DATAWIDTH(16),
    .ADDRWIDTH(32),
    .LG_LAYERSIZE(LG_LAYERSIZE),
    .HBITS(4)
) loader (
    s_axis_aclk,
    s_axis_aresetn,

    s_axis_tdata[15:0],
    s_axis_tvalid,
    s_axis_tready,

    axis_fifo_tdata,
    axis_fifo_tvalid,
    axis_fifo_tready,

    base_address,
    zero_address,
    input_layersize, //multiple of 4
    output_layersize, //multiple of 4

    m_axi_araddr,
    m_axi_arvalid,
    m_axi_arready,
    m_axi_arlen,
    m_axi_arsize,
    m_axi_arburst
);

KERNEL #(
    .S_AXIS_DATAWIDTH(16),
    .M_AXI_RDATAWIDTH(64),
    .M_AXIS_DATAWIDTH(64),
    .LG_LAYERSIZE(LG_LAYERSIZE),
    .ACC_DATAWIDTH(128)
) kernel (
    s_axis_aclk,
    s_axis_aresetn,

    axis_fifo_tdata,
    axis_fifo_tvalid,
    axis_fifo_tready,

    m_axi_rdata,
    m_axi_rvalid,
    m_axi_rlast,
    m_axi_rready,

    m_axis_tdata,
    m_axis_tvalid,
    m_axis_tlast,
    m_axis_tready,

    input_layersize*5,
    output_layersize
);

endmodule


