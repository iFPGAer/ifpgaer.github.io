# 简单计数器IP设计文档

## 第一章 模块代码 cnt_simple.v
```verilog
//========================================================
// Module Name: cnt_simple
// Function: Simple configurable up counter IP
// Parameter MAX_CNT: counter maximum value
// cnt_en: counter enable signal, high active
// rst_n: active low reset
// cnt_out: counter output
// overflow: overflow pulse flag, high for one clock cycle
//========================================================
module cnt_simple
#(
    parameter MAX_CNT = 12'd999   // Counter max value, default count 0~999
)(
    input               clk,
    input               rst_n,
    input               cnt_en,

    output reg [11:0]   cnt_out,
    output reg          overflow
);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cnt_out  <= 12'd0;
        overflow <= 1'b0;
    end
    else begin
        overflow <= 1'b0;
        if(cnt_en) begin
            if(cnt_out == MAX_CNT) begin
                cnt_out  <= 12'd0;
                overflow <= 1'b1;
            end
            else begin
                cnt_out <= cnt_out + 1'b1;
            end
        end
    end
end

endmodule
```

## 第二章 tb 代码 cnt_simple_tb.v
```verilog
//========================================================
// Testbench for cnt_simple module
// Simulate: 50MHz clock, test count, overflow, enable function
//========================================================
`timescale 1ns / 1ps

module cnt_simple_tb;

localparam CLK_PERIOD = 20;     // 50MHz 20ns
localparam MAX_CNT    = 12'd10; // tb set max count = 10

reg         clk;
reg         rst_n;
reg         cnt_en;

wire [11:0] cnt_out;
wire        overflow;

// DUT instance
cnt_simple
#(
    .MAX_CNT(MAX_CNT)
)
u_cnt_simple(
    .clk(clk),
    .rst_n(rst_n),
    .cnt_en(cnt_en),

    .cnt_out(cnt_out),
    .overflow(overflow)
);

// generate clock
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

initial begin
    rst_n   = 1'b0;
    cnt_en  = 1'b0;

    #(CLK_PERIOD*10);
    rst_n = 1'b1;

    // counter disable test
    #(CLK_PERIOD*20);
    $display("Counter disable, cnt_out = %0d", cnt_out);

    // enable counter
    @(posedge clk);
    cnt_en <= 1'b1;

    // wait overflow pulse
    repeat(25) begin
        @(posedge clk);
        if(overflow) begin
            $display("Overflow trigger! cnt_out = %0d", cnt_out);
        end
    end

    // disable counter
    @(posedge clk);
    cnt_en <= 1'b0;
    #(CLK_PERIOD*10);
    $display("Counter disable, cnt_out = %0d", cnt_out);

    #(CLK_PERIOD*20);
    $display("Simulation finish");
    $finish;
end

initial begin
    $dumpfile("cnt_wave.vcd");
    $dumpvars(0,cnt_simple_tb);
end

endmodule
```

## 第三章 下载地址

- [模块代码 cnt_simple.v](https://pan.baidu.com/s/xxx1)
- [仿真 TB 代码 cnt_simple_tb.v](https://pan.baidu.com/s/xxx2)
- [全套工程压缩包 (cnt_simple.v + cnt_simple_tb.v)](https://pan.baidu.com/s/xxx3)

> 说明：点击文字跳转下载。