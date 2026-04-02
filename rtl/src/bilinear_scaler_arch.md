# bilinear_scaler 架构设计文档 V2

## 1. 设计目标

基于 **双时钟域 + 3-Buffer 轮转** 架构，实现流式双线性插值放大器：
- **输入侧（clk_in 域）**：持续从 DDR 接收源图像数据，写入 3-Buffer
- **输出侧（clk_out 域）**：按显示 Timing 连续输出放大后的目标像素
- **完全流式**：无帧级状态机，读写操作彻底分离
- **跨时钟域同步**：异步 FIFO 传递 Buffer 状态

---

## 2. 时钟域与接口

### 2.1 修改后的模块接口

```verilog
module bilinear_scaler #(
    parameter DATA_WIDTH  = 8,
    parameter INT_BITS    = 12,
    parameter FRAC_BITS   = 8,
    parameter WEIGHT_BITS = 8,
    parameter MAX_WIDTH   = 4096,
    parameter MAX_HEIGHT  = 4096,
    parameter ADDR_WIDTH  = $clog2(MAX_WIDTH)
)(
    // 输入时钟域 (DDR/源图像)
    input  wire                          clk_in        ,//I1, 输入时钟
    input  wire                          rst_n_in      ,//I1, 输入复位
    
    // 输出时钟域 (显示/目标图像)
    input  wire                          clk_out       ,//I1, 输出时钟
    input  wire                          rst_n_out     ,//I1, 输出复位

    // 配置接口 (建议在 i_frame_start 前配置有效，在 clk_in 域采样)
    input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_x ,
    input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_y ,
    input  wire [15:0]                   cfg_dst_width   ,
    input  wire [15:0]                   cfg_dst_height  ,
    input  wire [15:0]                   cfg_src_width   ,
    input  wire [15:0]                   cfg_src_height  ,

    // 输入视频流 (AXI-Stream, clk_in 域)
    input  wire                          i_valid       ,//I1,
    input  wire [DATA_WIDTH-1:0]         i_data        ,//Ix,
    input  wire                          i_last        ,//I1,行结束
    input  wire                          i_frame_start ,//I1,帧开始
    output wire                          i_ready       ,//O1,

    // 输出视频流 (AXI-Stream, clk_out 域)
    output reg                           o_valid       ,//O1,
    output reg  [DATA_WIDTH-1:0]         o_data        ,//Ox,
    output reg                           o_last        ,//O1,行结束
    output reg                           o_frame_start ,//O1,帧开始
    input  wire                          o_ready        //I1,
);
```

### 2.2 双时钟域架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         bilinear_scaler (Dual Clock)                        │
│                                                                             │
│  ┌─────────────────────────────┐        ┌─────────────────────────────┐     │
│  │     Input Domain (clk_in)   │        │    Output Domain (clk_out)  │     │
│  │                             │        │                             │     │
│  │  ┌─────────────────────┐    │        │    ┌─────────────────────┐  │     │
│  │  │   Input Controller  │    │        │    │  Output Controller  │  │     │
│  │  │                     │    │        │    │                     │  │     │
│  │  │ - 接收 i_valid/data │    │ Async  │    │ - 持续生成 dst_x/y  │  │     │
│  │  │ - 管理 i_ready      │    │ FIFO   │    │ - 选择 Buffer 对    │  │     │
│  │  │ - 写入 3-Buffer     │◄───┼────────┼───►│ - 读取像素插值      │  │     │
│  │  │ - 更新 Buffer 状态  │    │ Status │    │ - 释放 Buffer       │  │     │
│  │  └─────────────────────┘    │        │    └─────────────────────┘  │     │
│  │            │                │        │            │                │     │
│  │            ▼                │        │            ▼                │     │
│  │  ┌─────────────────────┐    │        │    ┌─────────────────────┐  │     │
│  │  │   3-Line Buffers    │    │        │    │   Interpolation     │  │     │
│  │  │  (Dual-Port RAM)    │    │        │    │   Engine            │  │     │
│  │  │                     │    │        │    │                     │  │     │
│  │  │  ┌─────────────┐    │    │        │    │  - p00/p10/p01/p11  │  │     │
│  │  │  │  Buffer 0   │────┼────┼────────┼────┼─►│  - dx/dy          │  │     │
│  │  │  └─────────────┘    │    │        │    │  - bilinear calc    │  │     │
│  │  │  ┌─────────────┐    │    │        │    └─────────────────────┘  │     │
│  │  │  │  Buffer 1   │────┼────┼────────┼───────────────────────────┐   │     │
│  │  │  └─────────────┘    │    │        │                           │   │     │
│  │  │  ┌─────────────┐    │    │        │                           ▼   │     │
│  │  │  │  Buffer 2   │────┼────┼────────┼──────────────────────────►o_data│     │
│  │  │  └─────────────┘    │    │        │                               │     │
│  │  └─────────────────────┘    │        │                               │     │
│  │            │                │        │                               │     │
│  │            │                │        │    ┌─────────────────────┐    │     │
│  │            └────────────────┼────────┼───►│  Coord Calculator   │    │     │
│  │                             │        │    │  (Y: clk_out domain)│    │     │
│  └─────────────────────────────┘        │    └─────────────────────┘    │     │
│                                          └───────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 3-Buffer 双口 RAM 设计

### 3.1 存储结构

```verilog
// 3个独立的行缓冲（双口RAM）
// 写端口：clk_in 域
// 读端口：clk_out 域

reg [DATA_WIDTH-1:0] line_buffer_0 [0:MAX_WIDTH-1];
reg [DATA_WIDTH-1:0] line_buffer_1 [0:MAX_WIDTH-1];
reg [DATA_WIDTH-1:0] line_buffer_2 [0:MAX_WIDTH-1];

// 写端口信号（clk_in 域）
reg [2:0]              buf_wen;      // 每个buffer的写使能
reg [ADDR_WIDTH-1:0]   buf_waddr;    // 写地址（共享）
reg [DATA_WIDTH-1:0]   buf_wdata;    // 写数据（共享）

// 读端口信号（clk_out 域）
reg [2:0]              buf_ren;      // 每个buffer的读使能
reg [ADDR_WIDTH-1:0]   buf_raddr_0, buf_raddr_1;  // y0/y1读地址
wire [DATA_WIDTH-1:0]  buf_rdata_0, buf_rdata_1, buf_rdata_2;
```

### 3.2 读写分离实现

```verilog
//------------------------------------------------------------------------
// 写端口逻辑（clk_in 域）
//------------------------------------------------------------------------
always @(posedge clk_in) begin
    if (buf_wen[0]) line_buffer_0[buf_waddr] <= i_data;
    if (buf_wen[1]) line_buffer_1[buf_waddr] <= i_data;
    if (buf_wen[2]) line_buffer_2[buf_waddr] <= i_data;
end

//------------------------------------------------------------------------
// 读端口逻辑（clk_out 域）
//------------------------------------------------------------------------
// 使用组合逻辑输出，或寄存器输出（根据时序要求）
assign buf_rdata_0 = line_buffer_0[buf_raddr_0];
assign buf_rdata_1 = line_buffer_1[buf_raddr_0];  // 注意：y0/y1读地址可能不同
assign buf_rdata_2 = line_buffer_2[buf_raddr_0];

// y1读地址可能不同
wire [DATA_WIDTH-1:0] buf_rdata_0_y1 = line_buffer_0[buf_raddr_1];
wire [DATA_WIDTH-1:0] buf_rdata_1_y1 = line_buffer_1[buf_raddr_1];
wire [DATA_WIDTH-1:0] buf_rdata_2_y1 = line_buffer_2[buf_raddr_1];
```

---

## 4. 输入侧控制（clk_in 域）

### 4.1 Buffer 状态定义

```verilog
localparam BUF_EMPTY    = 2'b00;  // 空闲，可接收新行
localparam BUF_FILLING  = 2'b01;  // 正在写入当前行
localparam BUF_READY    = 2'b10;  // 已满，等待输出侧使用
localparam BUF_PENDING  = 2'b11;  // 已用完，等待释放（避免覆盖）

reg [1:0] buf_in_state [0:2];     // 每个buffer的输入侧状态
reg [15:0] buf_line_id [0:2];     // 该buffer存储的源图像行号
```

### 4.2 i_ready 控制逻辑

```verilog
// 只要有 EMPTY buffer，就可以接收数据
assign i_ready = (buf_in_state[0] == BUF_EMPTY) || 
                 (buf_in_state[1] == BUF_EMPTY) || 
                 (buf_in_state[2] == BUF_EMPTY);

// 帧开始检测
wire frame_start_detected = i_frame_start && i_valid;
```

### 4.3 Buffer 写入控制

```verilog
reg [1:0]  write_buf_id;          // 当前正在写入的buffer (0/1/2)
reg [15:0] wr_col_cnt;            // 行内列计数器
reg [15:0] src_row_cnt;           // 源图像行计数器

always @(posedge clk_in or negedge rst_n_in) begin
    if (!rst_n_in) begin
        write_buf_id <= 0;
        wr_col_cnt   <= 0;
        src_row_cnt  <= 0;
        buf_wen      <= 3'b000;
    end else begin
        buf_wen <= 3'b000;  // 默认无写使能
        
        if (i_valid && i_ready) begin
            // 写入当前buffer
            buf_wen              <= (3'b001 << write_buf_id);
            buf_waddr            <= wr_col_cnt;
            buf_wdata            <= i_data;
            
            if (i_last) begin
                // 行结束，标记为 READY，切换到下一个 EMPTY buffer
                buf_in_state[write_buf_id] <= BUF_READY;
                buf_line_id[write_buf_id]  <= src_row_cnt;
                
                // 找下一个 EMPTY buffer
                write_buf_id <= next_empty_buf();
                wr_col_cnt   <= 0;
                src_row_cnt  <= src_row_cnt + 1;
            end else begin
                wr_col_cnt <= wr_col_cnt + 1;
            end
        end
    end
end
```

---

## 5. 跨时钟域同步

### 5.1 异步 FIFO 传递 Buffer 状态

```verilog
//------------------------------------------------------------------------
// Buffer 就绪信息：clk_in → clk_out
// 通知输出侧"某行已准备好，可以开始读取"
//------------------------------------------------------------------------
wire [2:0]  buf_ready_set;        // 某buffer已就绪（clk_in 域脉冲）
wire [2:0]  buf_ready_set_sync;   // 同步到 clk_out 域

// 当 buf_in_state 从 FILLING → READY 时产生脉冲
generate
    genvar i;
    for (i = 0; i < 3; i = i + 1) begin : gen_sync_ready
        // 使用握手同步或脉冲同步器
        pulse_sync u_ready_sync (
            .clk_in  (clk_in),
            .rst_n_in(rst_n_in),
            .pulse_in(buf_ready_set[i]),
            .clk_out (clk_out),
            .rst_n_out(rst_n_out),
            .pulse_out(buf_ready_set_sync[i])
        );
    end
endgenerate

//------------------------------------------------------------------------
// Buffer 释放信息：clk_out → clk_in
// 通知输入侧"某行已用完，可以覆盖"
//------------------------------------------------------------------------
wire [2:0]  buf_release;          // 某buffer已释放（clk_out 域脉冲）
wire [2:0]  buf_release_sync;     // 同步到 clk_in 域

generate
    for (i = 0; i < 3; i = i + 1) begin : gen_sync_release
        pulse_sync u_release_sync (
            .clk_in  (clk_out),
            .rst_n_in(rst_n_out),
            .pulse_in(buf_release[i]),
            .clk_out (clk_in),
            .rst_n_out(rst_n_in),
            .pulse_out(buf_release_sync[i])
        );
    end
endgenerate
```

### 5.2 输出侧 Buffer 状态

```verilog
// clk_out 域维护的 buffer 状态
reg [1:0] buf_out_state [0:2];

always @(posedge clk_out or negedge rst_n_out) begin
    if (!rst_n_out) begin
        buf_out_state[0] <= BUF_EMPTY;
        buf_out_state[1] <= BUF_EMPTY;
        buf_out_state[2] <= BUF_EMPTY;
    end else begin
        // 新 buffer 就绪
        if (buf_ready_set_sync[0]) buf_out_state[0] <= BUF_READY;
        if (buf_ready_set_sync[1]) buf_out_state[1] <= BUF_READY;
        if (buf_ready_set_sync[2]) buf_out_state[2] <= BUF_READY;
        
        // buffer 被选中参与运算
        if (select_buf_0 == 0 || select_buf_1 == 0) buf_out_state[0] <= BUF_ACTIVE;
        if (select_buf_0 == 1 || select_buf_1 == 1) buf_out_state[1] <= BUF_ACTIVE;
        if (select_buf_0 == 2 || select_buf_1 == 2) buf_out_state[2] <= BUF_ACTIVE;
    end
end
```

---

## 6. 输出侧控制（clk_out 域）

### 6.1 目标坐标持续生成

```verilog
reg [15:0] dst_x_cnt;
reg [15:0] dst_y_cnt;
reg        output_active;         // 已开始输出

always @(posedge clk_out or negedge rst_n_out) begin
    if (!rst_n_out) begin
        dst_x_cnt     <= 0;
        dst_y_cnt     <= 0;
        output_active <= 0;
    end else if (o_valid && o_ready) begin
        output_active <= 1;
        if (dst_x_cnt >= r_dst_width - 1) begin
            dst_x_cnt <= 0;
            if (dst_y_cnt >= r_dst_height - 1) begin
                dst_y_cnt <= 0;
                output_active <= 0;  // 帧结束
            end else begin
                dst_y_cnt <= dst_y_cnt + 1;
            end
        end else begin
            dst_x_cnt <= dst_x_cnt + 1;
        end
    end
end
```

### 6.2 源坐标计算（双线性插值坐标）

```verilog
// 实例化坐标计算模块（在 clk_out 域）
wire [15:0] coord_y_int;
wire [FRAC_BITS-1:0] coord_y_frac;

bilinear_coord_calc #(
    .INT_BITS(INT_BITS),
    .FRAC_BITS(FRAC_BITS),
    .WEIGHT_BITS(WEIGHT_BITS)
) u_coord_y (
    .clk      (clk_out),
    .rst_n    (rst_n_out),
    .dst_idx  (dst_y_cnt),
    .inv_scale(r_inv_scale_y),
    .src_size (r_src_height),
    .src_pos_int (coord_y_int),
    .src_pos_frac(coord_y_frac),
    .valid    ()
);

// X方向坐标（类似）
wire [15:0] coord_x_int;
wire [FRAC_BITS-1:0] coord_x_frac;
```

### 6.3 Buffer 选择逻辑

```verilog
// 根据 coord_y_int 选择参与运算的两个 buffer
// y0 = coord_y_int, y1 = coord_y_int + 1

reg [1:0] select_buf_y0;          // y0对应的buffer id
reg [1:0] select_buf_y1;          // y1对应的buffer id
reg       use_same_buf;           // y0==y1时使用同一行（边界处理）

always @(*) begin
    // 查找包含行号 coord_y_int 的 buffer
    select_buf_y0 = find_buffer_with_line(coord_y_int);
    
    // 查找包含行号 coord_y_int+1 的 buffer
    if (coord_y_int + 1 < r_src_height)
        select_buf_y1 = find_buffer_with_line(coord_y_int + 1);
    else
        select_buf_y1 = select_buf_y0;  // 最后一行，复制y0
    
    // 边界处理：当 coord_y_int=0 且需要 y<0 时，复制第0行
    use_same_buf = (coord_y_int == 0 && coord_y_frac < HALF_N) ||
                   (coord_y_int + 1 >= r_src_height);
end

// 查找函数（组合逻辑）
function [1:0] find_buffer_with_line(input [15:0] line_id);
    begin
        if (buf_line_id[0] == line_id && buf_out_state[0] != BUF_EMPTY)
            find_buffer_with_line = 0;
        else if (buf_line_id[1] == line_id && buf_out_state[1] != BUF_EMPTY)
            find_buffer_with_line = 1;
        else if (buf_line_id[2] == line_id && buf_out_state[2] != BUF_EMPTY)
            find_buffer_with_line = 2;
        else
            find_buffer_with_line = 2'b11;  // 未找到（错误状态）
    end
endfunction
```

### 6.4 像素读取与插值

```verilog
// 根据 select_buf_y0/y1 设置读地址
always @(posedge clk_out) begin
    case (select_buf_y0)
        0: begin buf_raddr_0 <= coord_x_int; buf_rdata_y0 <= buf_rdata_0; end
        1: begin buf_raddr_0 <= coord_x_int; buf_rdata_y0 <= buf_rdata_1; end
        2: begin buf_raddr_0 <= coord_x_int; buf_rdata_y0 <= buf_rdata_2; end
    endcase
    
    if (use_same_buf) begin
        buf_rdata_y1 <= buf_rdata_y0;  // 复制（边界处理B方案）
    end else begin
        case (select_buf_y1)
            0: buf_rdata_y1 <= buf_rdata_0_y1;
            1: buf_rdata_y1 <= buf_rdata_1_y1;
            2: buf_rdata_y1 <= buf_rdata_2_y1;
        endcase
    end
end

// 双线性插值实例化（略，复用现有模块）
```

### 6.5 Buffer 释放机制

```verilog
// 当某行不再需要参与插值时，释放该 buffer
always @(posedge clk_out) begin
    for (i = 0; i < 3; i = i + 1) begin
        if (buf_out_state[i] == BUF_ACTIVE) begin
            // 如果该buffer的行号 < coord_y_int，说明不再需要
            if (buf_line_id[i] < coord_y_int) begin
                buf_release[i] <= 1;  // 产生释放脉冲
                buf_out_state[i] <= BUF_EMPTY;
            end
        end
    end
end
```

---

## 7. 边界处理方案 B（复制）

### 7.1 处理逻辑

| 场景 | 处理方式 | 说明 |
|------|----------|------|
| dst_y_cnt = 0 | y0 = src_row_0, y1 = src_row_0 | 输出第0行，复制源第0行 |
| 正常情况 | y0 = src_row_n, y1 = src_row_n+1 | 正常双线性插值 |
| 最后一行 | y0 = src_row_last, y1 = src_row_last | 复制最后一行 |

### 7.2 实现

```verilog
// 在 Buffer 选择逻辑中实现
wire need_y0_copy = (coord_y_int == 0) && (coord_y_frac < (1 <<< FRAC_BITS) / 2);
wire need_y1_copy = (coord_y_int + 1 >= r_src_height);

assign y0_data = buf_rdata_y0;
assign y1_data = (need_y0_copy || need_y1_copy) ? buf_rdata_y0 : buf_rdata_y1;
```

---

## 8. 代码结构（按数据流组织）

```verilog
module bilinear_scaler (...);
    //====================================================================
    // 1. 参数和接口（已修改：双时钟域）
    //====================================================================
    
    //====================================================================
    // 2. 3-Buffer 双口 RAM 定义
    //====================================================================
    
    //====================================================================
    // 3. 输入侧控制（clk_in 域）
    //    - i_ready 生成
    //    - Buffer 写入
    //    - 输入侧状态管理
    //====================================================================
    
    //====================================================================
    // 4. 跨时钟域同步（Async FIFO）
    //    - Buffer 就绪同步（in→out）
    //    - Buffer 释放同步（out→in）
    //====================================================================
    
    //====================================================================
    // 5. 输出侧控制（clk_out 域）
    //    - 目标坐标生成（dst_x/y_cnt）
    //    - 源坐标计算（coord_x/y_int/frac）
    //    - Buffer 选择与读取
    //    - 边界处理（复制方案）
    //    - Buffer 释放
    //====================================================================
    
    //====================================================================
    // 6. 双线性插值计算
    //====================================================================
    
    //====================================================================
    // 7. 输出接口（o_valid/data/last/frame_start）
    //====================================================================
    
endmodule
```

---

## 9. 关键时序与延迟

| 路径 | 延迟 | 说明 |
|------|------|------|
| i_valid → Buffer 写入 | 1 clk_in | 直接写入 RAM |
| Buffer READY → 输出侧可用 | 2-3 clk_out | 跨时钟域同步延迟 |
| Buffer 读取 → 插值结果 | 2-3 clk_out | 双线性插值流水线 |
| 总延迟 | ~5-6 clk_out | 输入到输出端到端延迟 |

---

## 10. 风险与应对

| 风险 | 应对策略 |
|------|----------|
| 跨时钟域亚稳态 | 使用标准脉冲同步器，2-3级触发器同步 |
| Buffer 不足 | 3-Buffer设计保证总有1个空闲用于预取 |
| 读写冲突 | 双口RAM天然读写分离，地址独立 |
| 显示时序抖动 | 输出侧独立时钟，不受输入速率影响 |
| 帧边界处理 | i_frame_start 重置所有状态机 |

---

**待确认问题**：
1. 是否需要配置缓存（config_buffer）在 clk_in 还是 clk_out 域锁存？
2. 是否需要行缓冲区空/满的深度阈值中断？
3. 是否需要统计/调试信号（如 buffer 命中率、欠载计数）？

请评审以上架构设计，确认后进入编码阶段。
