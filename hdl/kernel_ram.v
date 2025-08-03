module RAM #(
    parameter PTR_WIDTH = 12,
    parameter DATA_WIDTH = 128
)(clk, wen, ren, addra, addrb, dia, dob);

input clk,ren, wen;
input [PTR_WIDTH-1:0] addra,addrb;
input [DATA_WIDTH-1:0] dia;
output [DATA_WIDTH-1:0] dob;
reg [DATA_WIDTH-1:0] ram [(1<<PTR_WIDTH)-1:0];
reg [DATA_WIDTH-1:0] dob;

always @(posedge clk) begin
if (wen) begin
ram[addra] <= dia;
end
end

always @(posedge clk) begin
if(ren) begin
dob <= ram[addrb];
end
end

endmodule
