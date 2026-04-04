# bilinear_scaler V+H 架构设计文档

## 1. 架构概述

基于 PG231 的 **V+H 分离架构**（先垂直插值，后水平插值），采用 **4-Buffer 轮转 + 1 行输出 FIFO** 设计，确保显示连续性。

### 1.1 核心特性

- **V+H 分离**：V-filter（异行同列）→ H-filter（同行异列）
- **流式处理**：3 行输入后即可开始输出（双线性只需 2 行，第 3 行预缓冲）
- **双时钟域**：`clk_in`（输入/DDR）+ `clk_out`（输出/显示）
- **3-Buffer L1**：3 行输入缓冲，V-filter 滑动窗口管理
- **输出缓冲**：1 行 FIFO 吸收速率波动，防止显示 underrun

### 1.2 架构框图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         bilinear_scaler (V+H Architecture)                      │
│                                                                                 │
│  clk_in domain                              clk_out domain                      │
│  ┌─────────────────────┐                   ┌─────────────────────┐              │
│  │   Input Buffer      │                   │   Output FIFO       │              │
│  │   (3-Buffer Ring)   │                   │   (1-Line Buffer)   │              │
│  │                     │                   │                     │              │
│  │  ┌─────┐ ┌─────┐   │   Async FIFO      │  ┌─────────────┐   │              │
│  │  │Buf0 │ │Buf1 │   │   (Status Sync)   │  │  1-Line     │   │              │
│  │  │(0/1)│ │(0/1)│   │◄─────────────────►│  │  Buffer     │   │              │
│  │  └─────┘ └─────┘   │                   │  │  (3840x8b)  │   │              │
│  │  ┌─────┐           │                   │  └──────┬──────┘   │              │
│  │  │Buf2 │           │                   │         │          │              │
│  │  │(0/1)│           │                   │    o_valid/o_ready  │              │
│  │  └─────┘           │                   │         │          │              │
│  └──────────┬──────────┘                   └─────────┼──────────┘              │
│             │                                        │                         │
│             ▼                                        ▼                         │
│  ┌─────────────────────┐                   ┌─────────────────────┐              │
│  │   V-Filter          │                   │   H-Filter          │              │
│  │   (Vertical Interp) │                   │   (Horizontal Interp)│             │
│  │                     │                   │                     │              │
│  │  Input: 4 pixels    │                   │  Input: 4 VPix      │              │
│  │  (same col, diff row)│                  │  (same row, diff col)│             │
│  │                     │                   │                     │              │
│  │  Output: 1 VPix     │──────────────────►│  Output: 1 pixel    │─────────────►│
│  │  (per column)       │   (VPix stream)   │  (final result)     │              │
│  └─────────────────────┘                   └─────────────────────┘              │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 缓冲架构设计

### 2.1 三级缓冲架构

本设计采用 **三级缓冲架构**，平衡 DDR 效率、运算需求和显示连续性：

| 层级 | 数量 | 作用 | 实现方式 |
|------|------|------|----------|
| **L1: 输入缓冲** | **3 行** | DDR 预缓冲，V-filter滑动窗口 | line_buffer_wrapper ×3 |
| **L2: V-filter 缓冲** | **2 行** | 垂直插值运算（双线性） | scaler 内部 2-line |
| **L3: 输出 FIFO** | **1 行** | 显示连续性 | output_line_fifo |

**总存储需求：6 行 × src_width × DATA_WIDTH**

**关键改进**：
- **3-Buffer L1**：支持 V-filter 滑动窗口（2行运算+1行预缓冲）
- **1r0w 状态管理**：每个 buffer 独立标记可读(1)/可写(0)
- **动态释放**：根据 V-filter 窗口位置 (`v_min_src_row`) 决定释放时机

### 2.2 L1 输入缓冲设计（3-Buffer 架构）

#### 设计原则

- **3-Buffer 轮转**：支持 V-filter 滑动窗口（2行运算 + 1行预缓冲）
- **1r0w 状态管理**：1=可读，0=可写，避免多 buffer 同时可写的歧义
- **器件无关**：通过 wrapper 封装，内部可用 Generic RAM、Xilinx BRAM、Lattice EBR 等
- **时序收敛**：wrapper 内部对输入/输出打拍，上层模块无需关心时序细节

#### 核心信号定义

```verilog
// L1 写控制（clk_in 域）
reg  [1:0]  l1_wr_buf_sel;       // 写buffer选择: 0/1/2 循环
reg  [1:0]  l1_buf_busy_num;     // 已占用buffer数: 0/1/2/3
reg  [15:0] in_row_idx;          // 输入行计数器（0开始）
reg  [15:0] l1_buf0_line_id;     // buffer0存储的源行号
reg  [15:0] l1_buf1_line_id;     // buffer1存储的源行号
reg  [15:0] l1_buf2_line_id;     // buffer2存储的源行号

// datacnt：每个buffer当前存储的数据计数（读时钟域可见）
wire [ADDR_WIDTH:0] l1_buf0_datacnt;
wire [ADDR_WIDTH:0] l1_buf1_datacnt;
wire [ADDR_WIDTH:0] l1_buf2_datacnt;
```

#### i_ready 反压逻辑

```verilog
// i_ready：busy_num < 3 时无条件可写；busy_num == 3 时需判断可释放
assign i_ready = (l1_buf_busy_num < 2'd3) || buf_can_release;

// 判断当前选中的buffer是否可以释放（busy_num==3时）
assign buf_can_release = 
    (l1_buf_busy_num == 2'd3) && v_min_src_row_update &&
    ((l1_wr_buf_sel == 2'd0 && l1_buf0_line_id < v_min_src_row_synced) ||
     (l1_wr_buf_sel == 2'd1 && l1_buf1_line_id < v_min_src_row_synced) ||
     (l1_wr_buf_sel == 2'd2 && l1_buf2_line_id < v_min_src_row_synced));
```

#### 3-Buffer 状态机（写侧）

```verilog
// sel 循环：0 -> 1 -> 2 -> 0
// busy_num：0 -> 1 -> 2 -> 3（饱和）

// 初始状态（复位/帧开始）
sel = 0, busy_num = 0

// 写行0：i_last 时 sel->1, busy_num->1
// 写行1：i_last 时 sel->2, busy_num->2  
// 写行2：i_last 时 sel->0, busy_num->3（满）

// busy_num == 3 后，需要等待 V-filter 释放
// 当 v_min_src_row > buf0_line_id 时，buf0 可释放
// 写行3：覆盖 buf0，sel->1, busy_num 保持 3
```

#### 完整时间线验证

| 阶段 | sel | busy_num | buf0 | buf1 | buf2 | 操作 |
|------|-----|----------|------|------|------|------|
| 初始 | 0 | 0 | 空 | 空 | 空 | 写buf0，busy_num=0<3，直接写 |
| 写完行0 | 1 | 1 | 满(行0) | 空 | 空 | 写buf1，busy_num=1<3，直接写 |
| 写完行1 | 2 | 2 | 满(行0) | 满(行1) | 空 | 写buf2，busy_num=2<3，直接写 |
| 写完行2 | 0 | 3 | 满(行0) | 满(行1) | 满(行2) | busy_num=3，判断buf0是否可释放（行0 < v_min_src_row?），若可释放则写buf0覆盖 |
| 释放行0写行3 | 1 | 3 | 满(行3) | 满(行1) | 满(行2) | busy_num=3，判断buf1是否可释放（行1 < v_min_src_row?），若可释放则写buf1覆盖 |
| 释放行1写行4 | 2 | 3 | 满(行3) | 满(行4) | 满(行2) | busy_num=3，判断buf2是否可释放（行2 < v_min_src_row?），若可释放则写buf2覆盖 |

**关键观察**：
- `busy_num` 从 0 递增到 3 后保持饱和
- `sel` 始终指向**最老的** buffer（待写入/覆盖）
- 写前必须判断：`buf_line_id < v_min_src_row`（该行已不在 V-filter 窗口内）

#### 模块接口（line_buffer_wrapper）

```verilog
module line_buffer_wrapper #(
    parameter DATA_WIDTH = 8,
    parameter MAX_WIDTH  = 4096,
    parameter ADDR_WIDTH = $clog2(MAX_WIDTH),
    parameter VENDOR     = "GENERIC"
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // 写端口（DDR 输入）
    input  wire                  wr_en,
    input  wire [ADDR_WIDTH-1:0] wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  wr_full,
    
    // 读端口（V-filter 消费）
    input  wire                  rd_en,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_data_valid,
    
    // 数据计数（读时钟域可见，用于判断buffer是否可读满）
    output wire [ADDR_WIDTH:0]   datacnt
);
```

#### 5 个独立的 Always 块

```verilog
// 1. 写地址计数器（复位或正常写入递增）
always @(posedge clk_in) begin
    if (i_valid && i_ready)
        if (i_last) l1_wr_addr_cnt <= 0;
        else        l1_wr_addr_cnt <= l1_wr_addr_cnt + 1;
end

// 2. 输入行计数器（i_last 时 +1）
always @(posedge clk_in) begin
    if (i_frame_start)      in_row_idx <= 0;
    else if (i_last)        in_row_idx <= in_row_idx + 1;
end

// 3. Buffer 选择（0->1->2->0 循环）
always @(posedge clk_in) begin
    if (i_frame_start)      l1_wr_buf_sel <= 0;
    else if (i_last && i_ready) l1_wr_buf_sel <= l1_wr_buf_sel + 1;
end

// 4. Busy 计数器（0->1->2->3 饱和）
always @(posedge clk_in) begin
    if (i_frame_start)      l1_buf_busy_num <= 0;
    else if (i_last && i_ready && busy_num < 3)
                            l1_buf_busy_num <= l1_buf_busy_num + 1;
end

// 5. 行号记录（i_last 时记录 in_row_idx）
always @(posedge clk_in) begin
    if (i_last && i_ready)
        case (l1_wr_buf_sel)
            0: l1_buf0_line_id <= in_row_idx;
            1: l1_buf1_line_id <= in_row_idx;
            2: l1_buf2_line_id <= in_row_idx;
        endcase
end
```

#### 单元测试策略

1. **3-Buffer 轮转**：验证 sel 循环 0->1->2->0
2. **busy_num 饱和**：验证从 0->3 后保持 3
3. **v_min_src_row 释放**：模拟 V-filter 释放信号，验证覆盖逻辑
4. **datacnt 同步**：验证写时钟域的 datacnt 能正确反映到读侧
5. **帧开始复位**：验证 frame_start 时所有状态清零

### 2.3 L2 V-Filter 输出缓冲（VPix 行缓冲）

**关键纠正**：L2 **不存储从 L1 搬来的原始行**，而是存储 **V-filter 计算后的中间结果（VPix 行）**。

#### 设计原理

在 V+H 架构中：
1. **V-filter** 实时从 L1 读取 **异行同列** 的 2 个像素，计算 VPix
2. **L2** 存储 V-filter 输出的一整行 VPix
3. **H-filter** 从 L2 读取 VPix，进行水平插值

#### 存储需求

| 参数 | 数值 | 说明 |
|------|------|------|
| 深度 | MAX_WIDTH (如 4096) | 一行最大像素数 |
| 宽度 | DATA_WIDTH (如 8/10/12) | VPix 位宽 |
| 数量 | **1 行** | 双线性 V-filter 只需缓冲 1 行 VPix |

```verilog
// L2: 1 行 VPix 缓冲（双线性）
reg [DATA_WIDTH-1:0] vpix_line [0:MAX_WIDTH-1];

// 写控制：V-filter 输出
wire                  vpix_wr_en;
wire [ADDR_WIDTH-1:0] vpix_wr_addr;
wire [DATA_WIDTH-1:0] vpix_wr_data;

// 读控制：H-filter 输入
wire                  vpix_rd_en;
wire [ADDR_WIDTH-1:0] vpix_rd_addr;
reg  [DATA_WIDTH-1:0] vpix_rd_data;
```

### 2.4 L3 输出 FIFO 设计

1 行缓冲，平滑输出到显示接口。详见第 7 节。

---

## 3. 数据流详解（V+H 架构正确实现）

### 3.1 阶段划分（修正版）

| 阶段 | 操作 | 延迟 | 说明 |
|------|------|------|------|
| **Stage 1** | DDR 写入 L1 缓冲 | 3 行时间 | 3 行 buffer 攒满（2行运算+1行预缓冲） |
| **Stage 2** | V-filter 实时读取 | 1 时钟/列 | 从 L1 读 2 行同列像素，输出 VPix |
| **Stage 3** | VPix 行缓冲 | 1 行时间 | V-filter 输出写入 L2（1行 VPix） |
| **Stage 4** | H-filter | 1 时钟/像素 | 从 L2 读 VPix，水平插值输出 |
| **Stage 5** | 输出 FIFO | 1~N 时钟 | 缓冲 1 行，确保显示连续性 |

**关键纠正**：
- **无 DMA 传输**：L1 → L2 不是整行搬运，而是 V-filter **实时按需读取**
- **L2 存 VPix**：不是原始像素，而是 V-filter 计算后的中间结果
- **地址实时生成**：根据 `dst_x_cnt` 实时计算 L1 读地址

### 3.2 详细时序（正确 V+H 数据流）

```
【输入阶段】
T0-T2: DDR 依次写行0,1,2 到 L1 buf0,1,2
       L1状态: buf0=行0, buf1=行1, buf2=行2

【V-filter 开始计算 dst_y=0】
假设 need_src_y = 0.25 → 需要 行0 + 行1

时钟0: V-filter 读 L1
       - buf0[0] (行0,列0)
       - buf1[0] (行1,列0)
       计算 VPix[0] = interp(行0[0], 行1[0], 0.25)
       写入 L2[0]

时钟1: V-filter 读 L1
       - buf0[1] (行0,列1)
       - buf1[1] (行1,列1)
       计算 VPix[1] = interp(行0[1], 行1[1], 0.25)
       写入 L2[1]

...持续列计算...

时钟N: V-filter 完成整行 VPix，写入 L2

【H-filter 处理 L2】
H-filter 从 L2 读取 VPix[0], VPix[1], VPix[2], VPix[3]
凑齐 4-tap 后输出第一个最终像素

【V-filter 计算 dst_y=1】
假设 need_src_y = 0.75 → 仍需要 行0 + 行1
继续读 buf0, buf1 的对应列...

【V-filter 计算 dst_y=2】
假设 need_src_y = 1.25 → 需要 行1 + 行2
切换到读 buf1, buf2...

【窗口滑动】
当 need_src_y > 1（超过行1），行0 可释放
buf0 被复用写入新行...
```

### 3.3 L1 Buffer 与 V-filter 协作机制

```
协作原理：

1. 坐标计算（根据 dst_y_cnt）
   src_y = (dst_y + 0.5) / vsf - 0.5
   need_y0 = floor(src_y)
   need_y1 = need_y0 + 1

2. L1 Buffer 匹配
   检查 l1_buf0_line_id, l1_buf1_line_id, l1_buf2_line_id
   找到等于 need_y0 和 need_y1 的 buffer

3. 实时读取
   根据 dst_x_cnt 生成列地址
   同时读两个 buffer 的对应列

4. V-filter 计算
   2-tap 插值（双线性）
   VPix = p0 * (1-dy_frac) + p1 * dy_frac

5. 窗口滑动检测
   当 dst_y_cnt 增加，need_y0 可能前进
   当 need_y0 > buf_line_id，该 buffer 可释放
```

**关键时序**：
- 每个时钟周期：V-filter 读 L1 → 计算 VPix → 写 L2
- H-filter 延迟 3-4 时钟后从 L2 读数 → 输出最终像素
- L1 释放与写入并行：窗口滑动时，最老 buffer 立即复用

---

## 4. L2 VPix 行缓冲设计（修正版）

### 4.1 功能定位

**纠正**：L2 **不是**从 L1 搬运数据的缓冲，而是存储 **V-filter 计算后的 VPix 行**。

在 V+H 架构中：
- V-filter 实时读取 L1 的 2 个像素（异行同列）
- 计算得到 1 个 VPix（垂直插值结果）
- VPix 按列顺序写入 L2
- H-filter 从 L2 读取 VPix 进行水平插值

### 4.2 存储结构

```verilog
// L2: 1 行 VPix 缓冲（双线性只需 1 行）
// 双三次需要 4 行 VPix 缓冲用于 H-filter 的 4-tap 运算

// 双线性配置（2-tap V-filter, 2-tap H-filter）
reg [DATA_WIDTH-1:0] vpix_buf [0:MAX_WIDTH-1];

// 双三次配置（4-tap V-filter, 4-tap H-filter）
reg [DATA_WIDTH-1:0] vpix_buf [0:3][0:MAX_WIDTH-1];
```

### 4.3 接口定义

```verilog
// V-filter → L2 写入接口
wire                  vpix_wr_en;      // VPix 写使能
wire [ADDR_WIDTH-1:0] vpix_wr_addr;    // 列地址（等于 dst_x_cnt）
wire [DATA_WIDTH-1:0] vpix_wr_data;    // V-filter 输出
wire                  vpix_wr_line_done; // 整行完成

// L2 → H-filter 读取接口
wire                  vpix_rd_en;      // H-filter 读使能
wire [ADDR_WIDTH-1:0] vpix_rd_addr;    // 列地址
reg  [DATA_WIDTH-1:0] vpix_rd_data;    // 输出到 H-filter
wire                  vpix_rd_valid;   // 数据有效
```

### 4.4 VPix 行缓冲控制

```verilog
// V-filter 写入控制
always @(posedge clk_out) begin
    if (v_filter_valid) begin
        vpix_buf[vpix_wr_addr] <= #U_DLY v_filter_result;
    end
end

// H-filter 读取控制（可能延迟 3-4 时钟启动）
always @(posedge clk_out) begin
    if (vpix_rd_en)
        vpix_rd_data <= #U_DLY vpix_buf[vpix_rd_addr];
end
```

### 4.5 乒乓缓冲（可选）

如果 H-filter 需要连续读取而 V-filter 正在写入下一行：

```verilog
// 双缓冲方案
reg [DATA_WIDTH-1:0] vpix_buf_even [0:MAX_WIDTH-1];
reg [DATA_WIDTH-1:0] vpix_buf_odd  [0:MAX_WIDTH-1];
reg                  vpix_buf_sel;  // 0=even, 1=odd

// V-filter 写入当前 buffer
// H-filter 读取另一 buffer
```

---

## 5. L1 读控制与 V-filter 实时计算

### 5.1 L1 读控制逻辑

**核心**：根据 `dst_x_cnt` 和 `dst_y_cnt` 实时生成 L1 读地址和 buffer 选择。

```verilog
// 坐标计算模块（复用 bilinear_coord_calc）
wire [15:0] need_src_y0;  // = floor((dst_y + 0.5) / vsf - 0.5)
wire [15:0] need_src_y1;  // = need_src_y0 + 1
wire [FRAC_BITS-1:0] dy_frac;  // 垂直小数部分

// L1 buffer 匹配（组合逻辑）
wire buf0_has_y0 = (l1_buf0_line_id == need_src_y0) && (l1_buf_busy_num > 0);
wire buf1_has_y0 = (l1_buf1_line_id == need_src_y0) && (l1_buf_busy_num > 1);
wire buf2_has_y0 = (l1_buf2_line_id == need_src_y0) && (l1_buf_busy_num > 2);
// 同理判断 y1...

// Buffer 选择（优先级编码）
reg [1:0] sel_y0, sel_y1;
always @(*) begin
    if      (buf0_has_y0) sel_y0 = 2'd0;
    else if (buf1_has_y0) sel_y0 = 2'd1;
    else if (buf2_has_y0) sel_y0 = 2'd2;
    else                  sel_y0 = 2'd0;  // 默认
end

// L1 读地址（与 dst_x_cnt 直接相关）
assign l1_rd_addr = dst_x_cnt;  // 可能需要根据 hsf 调整

// L1 读使能（V-filter 需要时）
assign l1_rd_en = v_filter_active;
```

### 5.2 V-filter 实时计算

**双线性 V-filter（2-tap）**：

```verilog
module v_filter_2tap #(
    parameter DATA_WIDTH = 8,
    parameter FRAC_BITS  = 8
)(
    input  wire                          clk,
    input  wire                          rst_n,
    
    // 从 L1 读取的两个像素（异行同列）
    input  wire [DATA_WIDTH-1:0]         p0,         // 行0像素
    input  wire [DATA_WIDTH-1:0]         p1,         // 行1像素
    input  wire [FRAC_BITS-1:0]          dy_frac,    // 垂直权重
    input  wire                          valid_in,
    
    output reg  [DATA_WIDTH-1:0]         vpix,       // VPix 输出
    output reg                           valid_out
);
    // 权重计算
    wire [FRAC_BITS-1:0] w0 = (1 << FRAC_BITS) - dy_frac;  // 1-dy
    wire [FRAC_BITS-1:0] w1 = dy_frac;                      // dy
    
    // 定点数乘法
    wire [DATA_WIDTH+FRAC_BITS-1:0] mult0 = p0 * w0;
    wire [DATA_WIDTH+FRAC_BITS-1:0] mult1 = p1 * w1;
    wire [DATA_WIDTH+FRAC_BITS:0]   sum   = mult0 + mult1;
    
    // 输出（右移 FRAC_BITS 位）
    always @(posedge clk) begin
        vpix      <= #U_DLY sum >> FRAC_BITS;
        valid_out <= #U_DLY valid_in;
    end
endmodule
```

### 5.3 时序流程

```
时钟0: 生成 dst_x_cnt, dst_y_cnt
       计算 need_src_y0, need_src_y1
       匹配 L1 buffer，生成 sel_y0, sel_y1
       输出 l1_rd_addr

时钟1: L1 输出 p0, p1（2 时钟延迟后）
       V-filter 计算 VPix

时钟2: VPix 写入 L2

时钟3-6: H-filter 读取 VPix，凑齐 4-tap 后输出
```

---

## 6. H-Filter（水平插值）

### 6.1 功能描述

- **输入**：L2 存储的 VPix 行
- **输出**：最终像素（水平插值结果）
- **延迟**：2-4 时钟（取决于 tap 数）

### 6.2 双线性 H-filter（2-tap）

```verilog
module h_filter_2tap #(
    parameter DATA_WIDTH = 8,
    parameter FRAC_BITS  = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // 从 L2 读取的 VPix 流
    input  wire [DATA_WIDTH-1:0] vpix,       // V-filter 输出（已存 L2）
    input  wire [FRAC_BITS-1:0]  dx_frac,    // 水平权重
    input  wire                  valid_in,
    
    output reg  [DATA_WIDTH-1:0] h_pix,      // 最终输出像素
    output reg                   valid_out
);
    // 2-tap 移位寄存器
    reg [DATA_WIDTH-1:0] shift_reg0;
    reg [DATA_WIDTH-1:0] shift_reg1;
    
    always @(posedge clk) begin
        if (valid_in) begin
            shift_reg1 <= shift_reg0;  // 右移
            shift_reg0 <= vpix;
        end
    end
    
    // 水平插值
    wire [FRAC_BITS-1:0] w0 = (1 << FRAC_BITS) - dx_frac;
    wire [FRAC_BITS-1:0] w1 = dx_frac;
    
    wire [DATA_WIDTH+FRAC_BITS-1:0] mult0 = shift_reg0 * w0;
    wire [DATA_WIDTH+FRAC_BITS-1:0] mult1 = shift_reg1 * w1;
    wire [DATA_WIDTH+FRAC_BITS:0]   sum   = mult0 + mult1;
    
    always @(posedge clk) begin
        h_pix     <= #U_DLY sum >> FRAC_BITS;
        valid_out <= #U_DLY valid_in;  // 可能需要延迟 1 时钟
    end
endmodule
```

### 6.3 双三次 H-filter（4-tap）

```verilog
module h_filter_4tap #(
    parameter DATA_WIDTH = 8,
    parameter FRAC_BITS  = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    input  wire [DATA_WIDTH-1:0] vpix,       // 从 L2 读取
    input  wire [FRAC_BITS-1:0]  dx_frac,    // 水平权重
    input  wire                  valid_in,
    
    output reg  [DATA_WIDTH-1:0] h_pix,
    output reg                   valid_out
);
    // 4-tap 移位寄存器
    reg [DATA_WIDTH-1:0] shift_reg [0:3];
    reg [2:0]            shift_cnt;
    
    always @(posedge clk) begin
        if (valid_in) begin
            shift_reg[3] <= shift_reg[2];
            shift_reg[2] <= shift_reg[1];
            shift_reg[1] <= shift_reg[0];
            shift_reg[0] <= vpix;
            shift_cnt    <= shift_cnt + 1;
        end
    end
    
    // 凑齐 4 个后开始输出
    wire ready = (shift_cnt >= 3) && valid_in;
    
    // 4-tap 水平插值（双三次）
    always @(posedge clk) begin
        if (ready) begin
            // h_pix = bicubic_interp(shift_reg[0], shift_reg[1], shift_reg[2], shift_reg[3], dx_frac)
            h_pix     <= #U_DLY calc_result;
            valid_out <= #U_DLY 1'b1;
        end else begin
            valid_out <= #U_DLY 1'b0;
        end
    end
endmodule
```

### 6.4 H-filter 与 L2 的连接

```verilog
// H-filter 从 L2 读取 VPix
assign vpix_rd_addr = h_filter_x_cnt;  // H-filter 内部列计数
assign vpix_rd_en   = h_filter_active;

// 2 时钟延迟后 VPix 到达 H-filter
// H-filter 处理...
// 最终像素输出到 FIFO
```

---

## 7. 输出 FIFO（1 行缓冲）

### 7.1 功能需求

- **输入**：H-filter 输出的最终像素流
- **输出**：显示接口（o_valid/o_data/o_ready）
- **作用**：吸收速率波动，确保显示连续性

### 7.2 架构位置

```
H-filter 输出 → Output FIFO → 显示接口
                     ↓
                1 行缓冲（目标图像宽度）
```

### 7.3 实现要点

```verilog
module output_line_fifo #(
    parameter DATA_WIDTH = 8,
    parameter MAX_WIDTH  = 4096
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // 写端口（H-filter输出）
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire                  wr_line_start,
    input  wire                  wr_line_done,
    
    // 读端口（显示接口）
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_line_start,
    output reg                   rd_line_done,
    output wire                  empty,
    output wire                  full
);
    // 双缓冲或单缓冲 + 预填充策略
    // ...
endmodule
```

### 7.4 使用策略

```verilog
// 预填充策略：确保 FIFO 有至少半行数据后再开始输出
localparam PRE_FILL_THRESHOLD = MAX_WIDTH / 2;

assign o_valid = (fifo_cnt > PRE_FILL_THRESHOLD) ? !fifo_empty : 1'b0;

// 或者用 ready 反压
assign o_valid = !fifo_empty;
assign h_filter_pause = fifo_full;  // FIFO 满则暂停 H-filter
```

---

## 8. 输出 FIFO（1 行缓冲）

### 6.1 功能需求

- **深度**：1 行目标图像宽度（如 1920 或 3840）
- **宽度**：DATA_WIDTH（如 8/10/12bit）
- **时钟域**：clk_out（与 H-filter 和显示接口同域）
- **作用**：吸收速率波动，防止输入瞬态中断导致显示异常

### 6.2 实现方案

```verilog
module output_line_fifo #(
    parameter DATA_WIDTH = 8,
    parameter MAX_WIDTH  = 3840
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // 写端口（H-filter输出）
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire                  wr_line_start,
    input  wire                  wr_line_end,
    
    // 读端口（显示接口）
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_line_start,
    output reg                   rd_line_end,
    output wire                  empty,
    output wire                  full,
    output wire [ADDR_WIDTH:0]   fifo_cnt
);
    localparam ADDR_WIDTH = $clog2(MAX_WIDTH);
    
    reg [DATA_WIDTH-1:0] fifo_mem [0:MAX_WIDTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [ADDR_WIDTH:0]   fifo_cnt_reg;
    
    // 标准FIFO逻辑...
    
    // 关键：行同步信号处理
    always @(posedge clk) begin
        if (wr_line_start && wr_en)
            rd_line_start <= #U_DLY 1'b1;  // 延迟输出
        else if (rd_en)
            rd_line_start <= #U_DLY 1'b0;
    end
endmodule
```

### 6.3 使用策略

```verilog
// 预填充策略：确保FIFO有至少半行数据后再开始输出
localparam PRE_FILL_THRESHOLD = MAX_WIDTH / 2;

assign o_valid = (fifo_cnt > PRE_FILL_THRESHOLD) ? !fifo_empty : 1'b0;

// 或者：始终允许读，但用ready反压上游
assign o_valid = !fifo_empty;
assign h_filter_en = !fifo_full;  // FIFO满则暂停H-filter
```

---

## 9. 双时钟域同步

### 7.1 需要同步的信号

| 信号 | 方向 | 说明 |
|------|------|------|
| buf_ready | clk_in → clk_out | 某行buffer已写满 |
| buf_release | clk_out → clk_in | 某行buffer可覆盖 |
| frame_start | clk_in → clk_out | 帧开始同步 |

### 7.2 脉冲同步器实现

```verilog
module pulse_sync (
    input  wire clk_in,
    input  wire rst_n_in,
    input  wire pulse_in,      // 单时钟脉冲
    input  wire clk_out,
    input  wire rst_n_out,
    output reg  pulse_out      // 同步后的脉冲
);
    // 2级触发器同步 + 边沿检测
    reg [2:0] sync_reg;
    
    always @(posedge clk_out or negedge rst_n_out) begin
        if (!rst_n_out)
            sync_reg <= 3'b000;
        else
            sync_reg <= {sync_reg[1:0], pulse_in};
    end
    
    assign pulse_out = sync_reg[1] && !sync_reg[2];  // 上升沿检测
endmodule
```

---

## 10. 接口定义（双时钟域版本）

```verilog
module bilinear_scaler_vh #(
    parameter DATA_WIDTH  = 8,
    parameter INT_BITS    = 12,
    parameter FRAC_BITS   = 8,
    parameter MAX_WIDTH   = 4096,
    parameter MAX_HEIGHT  = 4096
)(
    // 输入时钟域 (源图像/DDR)
    input  wire                          clk_in,
    input  wire                          rst_n_in,
    
    // 配置接口 (clk_in域)
    input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_x,
    input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_y,
    input  wire [15:0]                   cfg_dst_width,
    input  wire [15:0]                   cfg_dst_height,
    input  wire [15:0]                   cfg_src_width,
    input  wire [15:0]                   cfg_src_height,
    
    // 输入视频流 (AXI-S, clk_in域)
    input  wire                          i_valid,
    input  wire [DATA_WIDTH-1:0]         i_data,
    input  wire                          i_last,
    input  wire                          i_frame_start,
    output wire                          i_ready,
    
    // 输出时钟域 (显示接口)
    input  wire                          clk_out,
    input  wire                          rst_n_out,
    
    // 输出视频流 (AXI-S, clk_out域)
    output reg                           o_valid,
    output reg  [DATA_WIDTH-1:0]         o_data,
    output reg                           o_last,
    output reg                           o_frame_start,
    input  wire                          o_ready
);
```

---

## 11. 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| **输入延迟** | 3 行 | 3 行 buffer 攒满后开始输出 |
| **流水线延迟** | 4~6 时钟 | V-filter(1) + H-filter(2) + FIFO(1~3) |
| **吞吐率** | 1 像素/时钟 | 满吞吐率运行 |
| **Buffer需求** | 3 行(L1) + 2 行(L2) + 1 行(FIFO) | 共6行存储 |
| **支持缩放** | 任意比例 | 0.25x ~ 4x（取决于系数精度） |
| **支持算法** | 双线性/双三次 | 双线性(2-tap)，双三次(4-tap) |
| **L1 管理** | 3-Buffer 轮转 | 1r0w 状态，V-filter 窗口驱动释放 |

---

## 12. 验证要点

### L1 3-Buffer 验证
1. **Buffer 轮转**：验证 sel 循环 0->1->2->0，busy_num 0->3 后保持
2. **行号记录**：验证 i_last 时正确记录 in_row_idx 到对应 buffer
3. **datacnt 同步**：验证写时钟域 datacnt 正确反映到读侧
4. **释放条件**：验证 `buf_line_id < v_min_src_row` 时才可释放
5. **i_ready 反压**：busy_num<3 时无条件可写；busy_num==3 时需等待释放

### V-filter 窗口验证
6. **放大场景**：验证源行被多次复用（v_min_src_row 前进慢）
7. **缩小场景**：验证源行快速释放（v_min_src_row 前进快）
8. **跨时钟域同步**：验证 v_min_src_row 正确同步到 clk_in 域

### 整体验证
9. **帧开始复位**：验证 frame_start 时所有状态清零
10. **FIFO边界**：验证 FIFO 满/空时数据流正确反压
11. **行同步**：验证 o_last/o_frame_start 时序正确
12. **数据完整性**：验证缩放前后图像内容正确（与软件golden对比）

---

## 13. 与PG231对比

| 特性 | PG231 (V+H) | 本设计 |
|------|-------------|--------|
| 架构 | V+H分离 | V+H分离 ✓ |
| 输入Buffer | 4行 | 4行 ✓ |
| 输出FIFO | 有 | 有 ✓ |
| 双时钟域 | 支持 | 支持 ✓ |
| 可配置tap数 | 6/8/10/12 | 4（双三次）/2（双线性） |
| 系数表 | 内置64-phase | 可配置 |

**本设计**完全遵循PG231的V+H架构思想，针对双线性/双三次插值进行了优化。

---

**下一步**：开始RTL编码实现？
