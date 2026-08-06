# SPI主从模式设计文档

## 第一章 模块代码 spi_master_slave.v
```verilog
//========================================================
// Module Name: spi_master_slave
// Function: SPI full‑duplex master & slave combined module
// CPOL=0, CPHA=0 mode: idle sclk low, sample at first edge
// Support configurable SPI clock divider
// MSB first transmission, 8‑bit data width
//========================================================
module spi_master_slave
#(
    parameter SPI_DIV = 12'd50   // system clock divider for SCLK, 50MHz->1MHz SCLK
)(
    input               clk,
    input               rst_n,

    // Config interface
    input               master_en,       // 1:master mode; 0:slave mode

    // Master physical SPI interface
    output reg          sclk_o,
    output reg          mosi_o,
    input               miso_i,
    output reg          cs_n_o,

    // Slave physical SPI interface
    input               sclk_i,
    input               mosi_i,
    output reg          miso_o,
    input               cs_n_i,

    // User master interface
    input               master_tx_en,
    input      [7:0]    master_tx_data,
    output reg          master_busy,
    output reg [7:0]    master_rx_data,
    output reg          master_rx_valid,

    // User slave interface
    input      [7:0]    slave_tx_data,
    output reg [7:0]    slave_rx_data,
    output reg          slave_rx_valid
);

// ---------------------- Master Local Signals ----------------------
reg [11:0] master_cnt;
reg [3:0]  master_bit_cnt;
reg [7:0]  master_tx_shift;
reg [7:0]  master_rx_shift;

// ---------------------- Slave Local Signals ----------------------
reg        sclk_sync0, sclk_sync1;
reg        cs_sync0, cs_sync1;
reg        mosi_sync0, mosi_sync1;
reg [3:0]  slave_bit_cnt;
reg [7:0]  slave_tx_shift;
reg [7:0]  slave_rx_shift;
wire       sclk_r_edge;

//========================================================
// Master Logic: CPOL=0 CPHA=0, MSB first
//========================================================
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sclk_o          <= 1'b0;
        mosi_o          <= 1'b0;
        cs_n_o          <= 1'b1;
        master_cnt      <= 12'd0;
        master_bit_cnt  <= 4'd0;
        master_tx_shift <= 8'd0;
        master_rx_shift <= 8'd0;
        master_busy     <= 1'b0;
        master_rx_data  <= 8'd0;
        master_rx_valid <= 1'b0;
    end
    else begin
        master_rx_valid <= 1'b0;
        if(master_en) begin
            if(!master_busy && master_tx_en) begin
                master_busy     <= 1'b1;
                cs_n_o          <= 1'b0;
                master_cnt      <= 12'd0;
                master_bit_cnt  <= 4'd0;
                master_tx_shift <= master_tx_data;
                master_rx_shift <= 8'd0;
                sclk_o          <= 1'b0;
            end
            else if(master_busy) begin
                if(master_cnt == SPI_DIV - 1'b1) begin
                    master_cnt <= 12'd0;
                    sclk_o     <= ~sclk_o;
                    if(sclk_o == 1'b0) begin
                        // rising edge: output mosi, sample miso
                        mosi_o                  <= master_tx_shift[7];
                        master_rx_shift[7:0]    <= {master_rx_shift[6:0], miso_i};
                        master_tx_shift         <= {master_tx_shift[6:0],1'b0};
                        if(master_bit_cnt == 4'd7) begin
                            master_busy     <= 1'b0;
                            cs_n_o          <= 1'b1;
                            master_rx_data  <= master_rx_shift;
                            master_rx_valid <= 1'b1;
                            master_bit_cnt  <= 4'd0;
                        end
                        else begin
                            master_bit_cnt <= master_bit_cnt + 1'b1;
                        end
                    end
                end
                else begin
                    master_cnt <= master_cnt + 1'b1;
                end
            end
        end
        else begin
            cs_n_o <= 1'b1;
            sclk_o <= 1'b0;
            mosi_o <= 1'b0;
        end
    end
end

//========================================================
// Slave synchronizer for external SPI signals
//========================================================
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sclk_sync0  <= 1'b0;
        sclk_sync1  <= 1'b0;
        cs_sync0    <= 1'b1;
        cs_sync1    <= 1'b1;
        mosi_sync0  <= 1'b0;
        mosi_sync1  <= 1'b0;
    end
    else begin
        sclk_sync0 <= sclk_i;
        sclk_sync1 <= sclk_sync0;
        cs_sync0   <= cs_n_i;
        cs_sync1   <= cs_sync0;
        mosi_sync0 <= mosi_i;
        mosi_sync1 <= mosi_sync0;
    end
end

assign sclk_r_edge = (~sclk_sync1) && sclk_sync0;

//========================================================
// Slave Logic: CPOL=0 CPHA=0, MSB first
//========================================================
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        slave_bit_cnt  <= 4'd0;
        slave_tx_shift <= 8'd0;
        slave_rx_shift <= 8'd0;
        miso_o         <= 1'b0;
        slave_rx_data  <= 8'd0;
        slave_rx_valid <= 1'b0;
    end
    else begin
        slave_rx_valid <= 1'b0;
        if(cs_sync1 == 1'b0) begin
            if(sclk_r_edge) begin
                if(slave_bit_cnt == 4'd0) begin
                    slave_tx_shift <= slave_tx_data;
                end
                miso_o <= slave_tx_shift[7];
                slave_rx_shift <= {slave_rx_shift[6:0], mosi_sync1};
                slave_tx_shift <= {slave_tx_shift[6:0],1'b0};

                if(slave_bit_cnt == 4'd7) begin
                    slave_rx_data  <= slave_rx_shift;
                    slave_rx_valid <= 1'b1;
                    slave_bit_cnt  <= 4'd0;
                end
                else begin
                    slave_bit_cnt <= slave_bit_cnt + 1'b1;
                end
            end
        end
        else begin
            slave_bit_cnt <= 4'd0;
            miso_o        <= 1'b0;
        end
    end
end

endmodule
```
## 第二章 tb 代码 spi_master_slave_tb.v
```verilog
//========================================================
// Testbench for spi_master_slave module
// Simulate: 50MHz clock, SCLK=1MHz, CPOL=0 CPHA=0, 8bit MSB first
// Master‑Slave loopback test inside testbench
//========================================================
`timescale 1ns / 1ps

module spi_master_slave_tb;

localparam CLK_PERIOD = 20;     // 50MHz 20ns
localparam SPI_DIV    = 12'd50; // 50M/50 = 1MHz SCLK

reg         clk;
reg         rst_n;

// DUT signals
reg         master_en;

// Master physical
wire        sclk_o;
wire        mosi_o;
reg         miso_i;
wire        cs_n_o;

// Slave physical
wire        sclk_i;
wire        mosi_i;
reg         miso_o;
wire        cs_n_i;

// Master user
reg         master_tx_en;
reg [7:0]   master_tx_data;
wire        master_busy;
wire [7:0]  master_rx_data;
wire        master_rx_valid;

// Slave user
reg [7:0]   slave_tx_data;
wire [7:0]  slave_rx_data;
wire        slave_rx_valid;

// Interconnect: DUT master <-> DUT slave
assign sclk_i  = sclk_o;
assign mosi_i  = mosi_o;
assign cs_n_i  = cs_n_o;
assign miso_i  = miso_o;

spi_master_slave
#(
    .SPI_DIV(SPI_DIV)
)
u_spi(
    .clk(clk),
    .rst_n(rst_n),

    .master_en(master_en),

    .sclk_o(sclk_o),
    .mosi_o(mosi_o),
    .miso_i(miso_i),
    .cs_n_o(cs_n_o),

    .sclk_i(sclk_i),
    .mosi_i(mosi_i),
    .miso_o(miso_o),
    .cs_n_i(cs_n_i),

    .master_tx_en(master_tx_en),
    .master_tx_data(master_tx_data),
    .master_busy(master_busy),
    .master_rx_data(master_rx_data),
    .master_rx_valid(master_rx_valid),

    .slave_tx_data(slave_tx_data),
    .slave_rx_data(slave_rx_data),
    .slave_rx_valid(slave_rx_valid)
);

// generate clock
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

initial begin
    rst_n         = 1'b0;
    master_en     = 1'b1;
    master_tx_en  = 1'b0;
    master_tx_data= 8'd0;
    slave_tx_data = 8'hBB;

    #(CLK_PERIOD*10);
    rst_n = 1'b1;

    #(CLK_PERIOD*20);
    @(posedge clk);
    master_tx_en   <= 1'b1;
    master_tx_data <= 8'h55;
    @(posedge clk);
    master_tx_en   <= 1'b0;

    wait(master_busy == 1'b0);
    $display("Master TX done, master_rx_data = 0x%02h", master_rx_data);
    wait(slave_rx_valid == 1'b1);
    $display("Slave receive data = 0x%02h", slave_rx_data);

    #(CLK_PERIOD*200);
    slave_tx_data <= 8'hAA;
    @(posedge clk);
    master_tx_en   <= 1'b1;
    master_tx_data <= 8'h77;
    @(posedge clk);
    master_tx_en   <= 1'b0;

    wait(master_busy == 1'b0);
    $display("Master RX data = 0x%02h", master_rx_data);
    wait(slave_rx_valid == 1'b1);
    $display("Slave RX data = 0x%02h", slave_rx_data);

    #(SPI_DIV * CLK_PERIOD * 20);
    $display("Simulation finish");
    $finish;
end

initial begin
    $dumpfile("spi_wave.vcd");
    $dumpvars(0,spi_master_slave_tb);
end

endmodule
```

## 第三章 下载地址

- [模块代码 spi_master_slave.v](https://pan.baidu.com/s/xxx1)
- [仿真 TB 代码 spi_master_slave_tb.v](https://pan.baidu.com/s/xxx2)
- [全套工程压缩包 (spi_master_slave.v + spi_master_slave_tb.v))](https://pan.baidu.com/s/xxx3)

> 说明：点击文字跳转下载。