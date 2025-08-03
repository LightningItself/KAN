module KERNEL #(
    parameter S_AXIS_DATAWIDTH = 16,
    parameter M_AXI_RDATAWIDTH = 64,
    parameter M_AXIS_DATAWIDTH = 64,
    parameter LG_LAYERSIZE     = 12,
    parameter ACC_DATAWIDTH    = 128
)(
    input wire s_axis_aclk,
    input wire s_axis_aresetn,

    input wire [S_AXIS_DATAWIDTH-1:0] s_axis_tdata,
    input wire s_axis_tvalid,
    output reg s_axis_tready,

    input wire [M_AXI_RDATAWIDTH-1:0] m_axi_rdata,
    input wire m_axi_rvalid,
    input wire m_axi_rlast,
    output reg m_axi_rready,

    output reg [M_AXIS_DATAWIDTH-1:0] m_axis_tdata,
    output reg m_axis_tvalid,
    output reg m_axis_tlast,
    input wire m_axis_tready,

    input wire [LG_LAYERSIZE-1:0] input_layersize_x5,
    input wire [LG_LAYERSIZE-1:0] output_layersize
);

//pointers and counters
reg [LG_LAYERSIZE+2:0] write_counter;
reg signed [LG_LAYERSIZE-1:0] write_ptr, read_ptr;

//ram values
reg write_en;
reg [ACC_DATAWIDTH-1:0] write_data;
wire [ACC_DATAWIDTH-1:0] read_data;
reg [M_AXIS_DATAWIDTH-1:0] output_data;


//datapath reg
reg [31:0] mul1, mul2, mul3, mul4;
reg [ACC_DATAWIDTH-1:0] prev_value;

//control reg
reg step;
reg inp_valid, mul_valid, write_valid, first, mul_first;

//misc 
wire last_iter = (write_counter+1 == input_layersize_x5);
wire last_beat = (write_ptr+1 == output_layersize);

RAM #(
    .PTR_WIDTH(LG_LAYERSIZE),
    .DATA_WIDTH(ACC_DATAWIDTH)
) ram (s_axis_aclk, write_en, step, write_ptr, read_ptr, write_data, read_data);

//pipelined datapath
always @(posedge s_axis_aclk) begin
    if(~s_axis_aresetn) begin
        mul_valid <= 0;
        write_valid <= 0; 
    end
    else if(step) begin
        //mul + read stage
        mul1 <= s_axis_tdata*m_axi_rdata[15:0];
        mul2 <= s_axis_tdata*m_axi_rdata[31:16];
        mul3 <= s_axis_tdata*m_axi_rdata[47:32];
        mul4 <= s_axis_tdata*m_axi_rdata[63:48];
        mul_valid <= inp_valid;
        mul_first <= first;
        //add stage
        write_data <= (mul_first ? 0 : read_data)+{mul1, mul2, mul3, mul4};
        write_valid <= mul_valid;
    end
end

//control logic
always @(posedge s_axis_aclk) begin
    if(~s_axis_aresetn) begin
        write_counter <= 0;
        write_ptr <= 0;
        read_ptr <= 0;
        first <= 1;
    end
    else begin
        //handle read_ptr
        if(step & inp_valid) begin
            if(read_ptr+1 == output_layersize) begin
                read_ptr <= 0;
                if(last_iter)
                    first <= 1;
                else    
                    first <= 0;
            end
            else 
                read_ptr <= read_ptr + 1; 
        end
        //hande write_ptr
        if(step & write_valid) begin
            if(last_beat) begin
                write_ptr <= 0;
                if(last_iter)
                    write_counter <= 0;
                else 
                    write_counter <= write_counter + 1;
            end
            else
                write_ptr <= write_ptr + 1; 
        end
    end
end

always @(*) begin
    inp_valid = s_axis_tvalid & m_axi_rvalid;
    step = last_iter ? m_axis_tready : 1;
    write_en = step & write_valid & ~last_iter;
end

always @(*) begin
    if(~s_axis_aresetn) begin
        s_axis_tready = 0;
        m_axi_rready = 0;
        m_axis_tvalid = 0;
        m_axis_tdata = 0;
        m_axis_tlast = 0;
    end
    else begin
        s_axis_tready = (read_ptr+1 == output_layersize) & step & inp_valid;
        m_axi_rready = step & inp_valid;
        if(last_iter) begin
            m_axis_tvalid = write_valid;
            m_axis_tdata = {write_data[121:106], write_data[89:74], write_data[57:42], write_data[25:10]};
            m_axis_tlast = last_beat;
        end
        else begin
            m_axis_tvalid = 0;
            m_axis_tdata = 0;
            m_axis_tlast = 0;
        end
    end
end

endmodule