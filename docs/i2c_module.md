# I2C主机模块设计文档

## 第一章 模块代码 i2c_master.v
```verilog
//========================================================
// Module Name: i2c_master
// Function: I2C master controller, standard mode 100KHz
// 7‑bit slave address support, write & read operation
// Generate START, STOP, ACK check, MSB first
//========================================================
module i2c_master
#(
    parameter SYS_CLK_DIV = 12'd500   // 50MHz input, 100KHz I2C: 50M/100K/2 = 250, 这里500分频得到scl节拍
)(
    input               clk,
    input               rst_n,

    // User control interface
    input               i2c_tx_en,
    input      [6:0]    slave_addr,
    input               rw_flag,      // 0:write  1:read
    input      [7:0]    tx_data,
    output reg          busy,
    output reg [7:0]    rx_data,
    output reg          rx_valid,
    output reg          ack_error,    // 1:no ack received

    // Physical I2C interface
    inout               sda,
    output reg          scl
);

localparam IDLE     = 4'd0;
localparam START    = 4'd1;
localparam SEND_ADDR= 4'd2;
localparam CHK_ACK1 = 4'd3;
localparam SEND_DATA= 4'd4;
localparam CHK_ACK2 = 4'd5;
localparam RECV_DATA= 4'd6;
localparam SEND_ACK = 4'd7;
localparam STOP     = 4'd8;

reg [3:0]  curr_state;
reg [3:0]  next_state;

reg [11:0] cnt;
reg [3:0]  bit_cnt;
reg [7:0]  shift_reg;

reg        sda_out;
reg        sda_en;   // 1:drive sda_out; 0:high‑z input

assign sda = sda_en ? sda_out : 1'bz;

// state register
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        curr_state <= IDLE;
    end
    else begin
        curr_state <= next_state;
    end
end

// next state logic
always @(*) begin
    next_state = curr_state;
    case(curr_state)
        IDLE: begin
            if(i2c_tx_en) next_state = START;
        end
        START: next_state = SEND_ADDR;
        SEND_ADDR: begin
            if(bit_cnt == 4'd7) next_state = CHK_ACK1;
        end
        CHK_ACK1: begin
            if(ack_error) next_state = STOP;
            else begin
                if(rw_flag) next_state = RECV_DATA;
                else next_state = SEND_DATA;
            end
        end
        SEND_DATA: begin
            if(bit_cnt == 4'd7) next_state = CHK_ACK2;
        end
        CHK_ACK2: next_state = STOP;
        RECV_DATA: begin
            if(bit_cnt == 4'd7) next_state = SEND_ACK;
        end
        SEND_ACK: next_state = STOP;
        STOP: next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

// timing & bit operation
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cnt         <= 12'd0;
        bit_cnt     <= 4'd0;
        shift_reg   <= 8'd0;
        scl         <= 1'b1;
        sda_en      <= 1'b0;
        sda_out     <= 1'b1;
        busy        <= 1'b0;
        rx_data     <= 8'd0;
        rx_valid    <= 1'b0;
        ack_error   <= 1'b0;
    end
    else begin
        rx_valid <= 1'b0;
        if(cnt >= SYS_CLK_DIV - 1'b1) begin
            cnt <= 12'd0;
            case(curr_state)
                IDLE: begin
                    busy    <= 1'b0;
                    scl     <= 1'b1;
                    sda_en  <= 1'b0;
                    sda_out <= 1'b1;
                end
                START: begin
                    busy <= 1'b1;
                    // I2C start: scl high, sda fall
                    scl     <= 1'b1;
                    sda_en  <= 1'b1;
                    sda_out <= 1'b0;
                    bit_cnt <= 4'd0;
                    if(rw_flag) shift_reg <= {slave_addr,1'b1};
                    else shift_reg <= {slave_addr,1'b0};
                end
                SEND_ADDR: begin
                    scl <= ~scl;
                    if(scl == 1'b0) begin
                        sda_en  <= 1'b1;
                        sda_out <= shift_reg[7];
                        shift_reg <= {shift_reg[6:0],1'b0};
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                CHK_ACK1: begin
                    scl <= ~scl;
                    if(scl == 1'b0) begin
                        sda_en <= 1'b0; // release sda for slave ack
                        if(~sda) ack_error <= 1'b0;
                        else ack_error <= 1'b1;
                        bit_cnt <= 4'd0;
                        shift_reg <= tx_data;
                    end
                end
                SEND_DATA: begin
                    scl <= ~scl;
                    if(scl == 1'b0) begin
                        sda_en  <= 1'b1;
                        sda_out <= shift_reg[7];
                        shift_reg <= {shift_reg[6:0],1'b0};
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                CHK_ACK2: begin
                    scl <= ~scl;
                    if(scl == 1'b0) begin
                        sda_en <= 1'b0;
                        bit_cnt <= 4'd0;
                    end
                end
                RECV_DATA: begin
                    scl <= ~scl;
                    if(scl == 1'b1) begin
                        sda_en <= 1'b0;
                        shift_reg <= {shift_reg[6:0], sda};
                    end
                    else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                SEND_ACK: begin
                    scl <= ~scl;
                    if(scl == 1'b0) begin
                        sda_en  <= 1'b1;
                        sda_out <= 1'b0; // master ack
                        rx_data <= shift_reg;
                        rx_valid <= 1'b1;
                    end
                end
                STOP: begin
                    // I2C stop: scl high, sda rise
                    scl     <= 1'b1;
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    busy    <= 1'b0;
                    ack_error <= 1'b0;
                end
            endcase
        end
        else begin
            cnt <= cnt + 1'b1;
        end
    end
end

endmodule
```

## 第二章 tb 代码 i2c_master_tb.v
```verilog
//========================================================
// Testbench for i2c_master module
// Simulate: 50MHz clock, I2C 100KHz standard mode
// Simple i2c slave behavioral model inside tb
//========================================================
`timescale 1ns / 1ps

module i2c_master_tb;

localparam CLK_PERIOD  = 20;        // 50MHz 20ns
localparam SYS_CLK_DIV = 12'd500;   // match parameter in DUT
localparam SLAVE_ADDR  = 7'h20;

reg         clk;
reg         rst_n;

reg         i2c_tx_en;
reg [6:0]   slave_addr;
reg         rw_flag;
reg [7:0]   tx_data;
wire        busy;
wire [7:0]  rx_data;
wire        rx_valid;
wire        ack_error;

wire        sda;
wire        scl;

// DUT instance
i2c_master
#(
    .SYS_CLK_DIV(SYS_CLK_DIV)
)
u_i2c_master(
    .clk(clk),
    .rst_n(rst_n),

    .i2c_tx_en(i2c_tx_en),
    .slave_addr(slave_addr),
    .rw_flag(rw_flag),
    .tx_data(tx_data),
    .busy(busy),
    .rx_data(rx_data),
    .rx_valid(rx_valid),
    .ack_error(ack_error),

    .sda(sda),
    .scl(scl)
);

// Simple behavioral I2C‑slave model
reg sda_slave_drive;
reg sda_slave_en;
assign sda = sda_slave_en ? sda_slave_drive : 1'bz;

initial begin
    sda_slave_en = 1'b0;
    forever begin
        @(posedge scl);
        if(u_i2c_master.curr_state == u_i2c_master.CHK_ACK1 || u_i2c_master.curr_state == u_i2c_master.CHK_ACK2) begin
            #1;
            sda_slave_drive = 1'b0;
            sda_slave_en = 1'b1;
            @(negedge scl);
            sda_slave_en = 1'b0;
        end
    end
end

// generate clock
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

initial begin
    rst_n      = 1'b0;
    i2c_tx_en  = 1'b0;
    slave_addr = SLAVE_ADDR;
    rw_flag    = 1'b0;
    tx_data    = 8'd0;

    #(CLK_PERIOD*10);
    rst_n = 1'b1;

    // I2C write test
    #(CLK_PERIOD*30);
    @(posedge clk);
    i2c_tx_en  <= 1'b1;
    rw_flag    <= 1'b0;
    slave_addr <= SLAVE_ADDR;
    tx_data    <= 8'h33;
    @(posedge clk);
    i2c_tx_en  <= 1'b0;

    wait(busy == 1'b0);
    $display("I2C Write complete, ack_error = %b", ack_error);

    // I2C read test
    #(CLK_PERIOD*300);
    @(posedge clk);
    i2c_tx_en  <= 1'b1;
    rw_flag    <= 1'b1;
    slave_addr <= SLAVE_ADDR;
    tx_data    <= 8'h00;
    @(posedge clk);
    i2c_tx_en  <= 1'b0;

    wait(busy == 1'b0);
    $display("I2C Read complete, rx_data = 0x%02h, ack_error = %b", rx_data, ack_error);

    #(SYS_CLK_DIV * CLK_PERIOD * 40);
    $display("Simulation finish");
    $finish;
end

initial begin
    $dumpfile("i2c_wave.vcd");
    $dumpvars(0,i2c_master_tb);
end

endmodule
```

## 第三章 下载地址

- [模块代码 i2c_master.v](https://pan.baidu.com/s/xxx1)
- [仿真 TB 代码 i2c_master_tb.v](https://pan.baidu.com/s/xxx2)
- [全套工程压缩包 (i2c_master.v + i2c_master_tb.v)](https://pan.baidu.com/s/xxx3)

> 说明：点击文字跳转下载。