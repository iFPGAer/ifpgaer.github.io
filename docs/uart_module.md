# UART收发模块设计文档

## 第一章 模块代码 uart.v
```verilog
//========================================================
// Module Name: uart
// Function: UART send & receive module, 8N1
// Baud rate configurable by BAUD_DIV
// 8 data bit, 1 start bit, 1 stop bit, no parity
//========================================================
module uart
#(
    parameter BAUD_DIV = 1041  // 50MHz clk, 9600: 50M/9600 = 5208; 115200: 434
)(
    input               clk,
    input               rst_n,

    // User TX interface
    input               tx_wr_en,
    input      [7:0]    tx_data,
    output reg          tx_busy,
    output              uart_txd,

    // User RX interface
    output reg          rx_rd_en,
    output reg [7:0]    rx_data,
    input               rx_rd_ack,
    input               uart_rxd
);

// ---------------------- TX Logic ----------------------
reg [11:0] tx_cnt;
reg [3:0]  tx_bit_cnt;
reg [9:0]  tx_shift_reg;
reg        txd_reg;

assign uart_txd = txd_reg;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        tx_cnt      <= 12'd0;
        tx_bit_cnt  <= 4'd0;
        tx_shift_reg<= 10'b1111111111;
        tx_busy     <= 1'b0;
        txd_reg     <= 1'b1;
    end
    else begin
        if(!tx_busy && tx_wr_en) begin
            tx_busy      <= 1'b1;
            tx_cnt       <= 12'd0;
            tx_bit_cnt   <= 4'd0;
            // start(0) + data[7:0] + stop(1)
            tx_shift_reg <= {1'b1, tx_data[7:0], 1'b0};
        end
        else if(tx_busy) begin
            if(tx_cnt == BAUD_DIV - 1'b1) begin
                tx_cnt <= 12'd0;
                txd_reg <= tx_shift_reg[tx_bit_cnt];
                if(tx_bit_cnt == 4'd9) begin
                    tx_busy <= 1'b0;
                end
                else begin
                    tx_bit_cnt <= tx_bit_cnt + 1'b1;
                end
            end
            else begin
                tx_cnt <= tx_cnt + 1'b1;
            end
        end
    end
end

// ---------------------- RX Logic ----------------------
reg        rxd_sync0, rxd_sync1;
reg [11:0] rx_cnt;
reg [3:0]  rx_bit_cnt;
reg [9:0]  rx_shift_reg;

// 两级同步消除亚稳态
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rxd_sync0 <= 1'b1;
        rxd_sync1 <= 1'b1;
    end
    else begin
        rxd_sync0 <= uart_rxd;
        rxd_sync1 <= rxd_sync0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rx_cnt      <= 12'd0;
        rx_bit_cnt  <= 4'd0;
        rx_shift_reg<= 10'd0;
        rx_rd_en    <= 1'b0;
        rx_data     <= 8'd0;
    end
    else begin
        if(rx_rd_ack) begin
            rx_rd_en <= 1'b0;
        end

        if(rx_bit_cnt == 4'd0 && rxd_sync1 == 1'b0) begin
            // detect start bit, sample at middle
            if(rx_cnt == (BAUD_DIV >> 1)) begin
                rx_cnt     <= 12'd0;
                rx_bit_cnt <= rx_bit_cnt + 1'b1;
            end
            else begin
                rx_cnt <= rx_cnt + 1'b1;
            end
        end
        else if(rx_bit_cnt >= 4'd1 && rx_bit_cnt <=4'd9) begin
            if(rx_cnt == BAUD_DIV - 1'b1) begin
                rx_cnt <= 12'd0;
                rx_shift_reg[rx_bit_cnt] <= rxd_sync1;
                if(rx_bit_cnt == 4'd9) begin
                    // receive complete
                    rx_data  <= rx_shift_reg[8:1];
                    rx_rd_en <= 1'b1;
                    rx_bit_cnt <= 4'd0;
                    rx_cnt     <= 12'd0;
                end
                else begin
                    rx_bit_cnt <= rx_bit_cnt + 1'b1;
                end
            end
            else begin
                rx_cnt <= rx_cnt + 1'b1;
            end
        end
        else begin
            rx_cnt <= 12'd0;
        end
    end
end

endmodule
```

## 第二章 tb代码 uart_tb.v
```verilog
//========================================================
// Testbench for uart module
// Simulate: 50MHz clock, baud 9600, 8N1
//========================================================
`timescale 1ns / 1ps

module uart_tb;

localparam CLK_PERIOD = 20;    // 50MHz  20ns
localparam BAUD_DIV   = 12'd5208; // 50M /9600

reg         clk;
reg         rst_n;

reg         tx_wr_en;
reg [7:0]   tx_data;
wire        tx_busy;
wire        uart_txd;

wire        rx_rd_en;
wire [7:0]  rx_data;
reg         rx_rd_ack;
reg         uart_rxd;

// DUT instance
uart
#(
    .BAUD_DIV(BAUD_DIV)
)
u_uart(
    .clk(clk),
    .rst_n(rst_n),

    .tx_wr_en(tx_wr_en),
    .tx_data(tx_data),
    .tx_busy(tx_busy),
    .uart_txd(uart_txd),

    .rx_rd_en(rx_rd_en),
    .rx_data(rx_data),
    .rx_rd_ack(rx_rd_ack),
    .uart_rxd(uart_rxd)
);

// generate clock
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// loopback: connect txd to rxd for self‑loop test
initial begin
    uart_rxd = 1'b1;
    #100;
    forever begin
        uart_rxd <= uart_txd;
        #CLK_PERIOD;
    end
end

initial begin
    rst_n     = 1'b0;
    tx_wr_en  = 1'b0;
    tx_data   = 8'd0;
    rx_rd_ack = 1'b0;

    #(CLK_PERIOD*10);
    rst_n = 1'b1;

    // Send byte 0x55
    #(CLK_PERIOD*20);
    @(posedge clk);
    tx_wr_en <= 1'b1;
    tx_data  <= 8'h55;
    @(posedge clk);
    tx_wr_en <= 1'b0;

    wait(tx_busy == 1'b0);
    $display("TX complete byte = 0x55");

    // wait rx flag
    wait(rx_rd_en == 1'b1);
    $display("RX receive byte = 0x%02h", rx_data);
    @(posedge clk);
    rx_rd_ack <= 1'b1;
    @(posedge clk);
    rx_rd_ack <= 1'b0;

    // Send another byte 0xAA
    #(CLK_PERIOD*100);
    @(posedge clk);
    tx_wr_en <= 1'b1;
    tx_data  <= 8'hAA;
    @(posedge clk);
    tx_wr_en <= 1'b0;

    wait(tx_busy == 1'b0);
    wait(rx_rd_en == 1'b1);
    $display("RX receive byte = 0x%02h", rx_data);
    @(posedge clk);
    rx_rd_ack <= 1'b1;
    @(posedge clk);
    rx_rd_ack <= 1'b0;

    #(BAUD_DIV * CLK_PERIOD * 12);
    $display("Simulation finish");
    $finish;
end

initial begin
    $dumpfile("uart_wave.vcd");
    $dumpvars(0, uart_tb);
end

endmodule
```

## 第三章 下载地址

- [模块代码 uart.v](https://pan.baidu.com/s/xxx1)
- [仿真 TB 代码 uart_tb.v](https://pan.baidu.com/s/xxx2)
- [全套工程压缩包(uart.v + uart_tb.v)](https://pan.baidu.com/s/xxx3)

> 说明：将上面括号内的链接替换为真实百度网盘分享地址，VSCode / Typora 预览下文字可点击跳转下载。