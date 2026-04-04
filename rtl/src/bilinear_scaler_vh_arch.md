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

### 2.3 L2 V-filter 缓冲设计

4 行环形缓冲，直接嵌入 scaler 内部（非 wrapper，因为与算法紧耦合）。

### 2.4 L3 输出 FIFO 设计

1 行缓冲，平滑输出到显示接口。详见第 6 节。

---

## 3. 数据流详解

### 3.1 阶段划分

| 阶段 | 操作 | 延迟 | 说明 |
|------|------|------|------|
| **Stage 1** | DDR 写入 L1 缓冲 | 3 行时间 | 3 行 buffer 攒满 |
| **Stage 2** | L1 → L2 传输 | - | 按需读取 L1 buffer 到 L2 |
| **Stage 3** | V-filter | 1 时钟/列 | 异行同列，输出 VPix |
| **Stage 4** | H-filter | 1 时钟/像素 | 同行异列，输出最终像素 |
| **Stage 5** | 输出 FIFO | 1~N 时钟 | 缓冲 1 行，确保显示连续性 |

**关键改进**：
- **3-Buffer L1**：支持动态释放，V-filter 窗口滑动时自动复用最老的行
- **datacnt 驱动**：根据 datacnt 判断 L1 buffer 是否可读满

### 3.2 详细时序（3-Buffer L1 工作示例）

```
T0: DDR 写行0 → L1 buf0
    L1状态: sel=0, busy_num=1, buf0=行0(可读), buf1=空, buf2=空
    i_ready=1 (busy_num < 3)

T1: DDR 写行1 → L1 buf1
    L1状态: sel=1, busy_num=2, buf0=行0, buf1=行1(可读), buf2=空
    i_ready=1 (busy_num < 3)

T2: DDR 写行2 → L1 buf2
    L1状态: sel=0, busy_num=3, buf0=行0, buf1=行1, buf2=行2(可读)
    i_ready=0 (busy_num == 3, 等待V-filter释放)
    【3行齐全，V-filter可开始】

T3: V-filter 窗口: 需要行0+行1
    v_min_src_row = 0（窗口底边）
    buf0_line_id=0 >= v_min_src_row，不可释放
    i_ready 保持 0

T4: V-filter 窗口滑动到: 行1+行2
    v_min_src_row = 1
    buf0_line_id=0 < v_min_src_row，可释放！
    v_min_src_row_update 脉冲同步到 clk_in
    i_ready=1, buf_can_release=1

T5: DDR 写行3 → L1 buf0（覆盖行0）
    L1状态: sel=1, busy_num=3, buf0=行3(可读), buf1=行1, buf2=行2
    i_ready=0 (busy_num == 3)

T6: V-filter 窗口滑动到: 行2+行3
    v_min_src_row = 2
    buf1_line_id=1 < v_min_src_row，可释放
    i_ready=1

T7: DDR 写行4 → L1 buf1（覆盖行1）
    L1状态: sel=2, busy_num=3, buf0=行3, buf1=行4, buf2=行2
    ...持续轮转
```

**关键观察**：
- busy_num 达到 3 后保持饱和
- sel 循环 0->1->2->0，始终指向**最老的** buffer（待释放）
- 释放条件：`buf_line_id < v_min_src_row`

### 3.3 L1 Buffer 与 V-filter 协作时序

```
L1 Buffer 状态演变（3-buffer 轮转）：

初始: [buf0=空, buf1=空, buf2=空], sel=0, busy=0

写行0: [buf0=行0(1), buf1=空, buf2=空], sel=1, busy=1
       └─ sel 指向当前可写的 buffer
       
写行1: [buf0=行0(1), buf1=行1(1), buf2=空], sel=2, busy=2
       
写行2: [buf0=行0(1), buf1=行1(1), buf2=行2(1)], sel=0, busy=3
       【全满，等待 V-filter 释放】

V-filter 窗口 [行0, 行1]: v_min_src_row = 0
              行0 >= 0，不可释放，i_ready=0

V-filter 窗口 [行1, 行2]: v_min_src_row = 1
              行0 < 1，可释放 buf0！
              
写行3: [buf0=行3(1), buf1=行1(1), buf2=行2(1)], sel=1, busy=3
       【覆盖行0，buf0 被复用】

V-filter 窗口 [行2, 行3]: v_min_src_row = 2
              行1 < 2，可释放 buf1
              
写行4: [buf0=行3(1), buf1=行4(1), buf2=行2(1)], sel=2, busy=3

...持续轮转，始终维持 3 行有效数据
```

**关键逻辑**：
1. `sel` 始终指向**最老的** buffer（待写入/覆盖）
2. `busy_num == 3` 时，需要等待 V-filter 窗口滑动才能释放
3. 释放条件：`buf_line_id < v_min_src_row`（该行已不在 V-filter 窗口内）

---

## 4. L2 V-Filter Buffer 详细设计

L2 是 scaler 内部的 4 行 V-filter 运算缓冲，与 L1 wrapper 通过标准接口连接。

### 4.1 接口定义

```verilog
// L2 从 L1 读取数据的接口
wire                  l2_rd_valid;
wire [ADDR_WIDTH-1:0] l2_rd_addr;
wire [DATA_WIDTH-1:0] l2_rd_data;      // 来自 L1 wrapper
wire                  l2_rd_data_valid;

// L2 内部 4 行缓冲
reg [DATA_WIDTH-1:0] v_buf [0:3][0:MAX_WIDTH-1];
```

### 4.2 数据从 L1 → L2 的传输

```verilog
// 状态机控制：将 L1 的一行数据转存到 L2
localparam TRANS_IDLE  = 2'b00;
localparam TRANS_READ  = 2'b01;
localparam TRANS_WRITE = 2'b10;

reg [1:0] trans_state;
reg [ADDR_WIDTH-1:0] trans_cnt;
reg [1:0] l2_wr_buf_id;  // 当前写入 L2 的 buffer ID

always @(posedge clk) begin
    case (trans_state)
        TRANS_IDLE: begin
            if (l1_buf_available && l2_has_empty_slot) begin
                trans_state <= TRANS_READ;
                trans_cnt   <= 0;
            end
        end
        
        TRANS_READ: begin
            // 从 L1 读取地址 trans_cnt 的数据
            l2_rd_valid <= 1'b1;
            l2_rd_addr  <= trans_cnt;
            trans_state <= TRANS_WRITE;
        end
        
        TRANS_WRITE: begin
            if (l2_rd_data_valid) begin
                // 写入 L2 buffer
                v_buf[l2_wr_buf_id][trans_cnt] <= l2_rd_data;
                
                if (trans_cnt >= src_width - 1) begin
                    trans_state <= TRANS_IDLE;
                    l2_wr_buf_id <= l2_wr_buf_id + 1;
                    l1_buf_release <= 1'b1;  // 释放 L1 buffer
                end else begin
                    trans_cnt <= trans_cnt + 1;
                    trans_state <= TRANS_READ;
                end
            end
        end
    endcase
end
```

### 4.3 L2 Buffer 释放策略（放大场景）

```verilog
// 当某源行不再被任何目标行使用时，标记为可释放
// 双三次（4-tap）：当 coord_y_int > src_row_id + 2 时可释放

always @(posedge clk_out) begin
    for (i = 0; i < 4; i = i + 1) begin
        if (buf_state[i] == BUF_ACTIVE) begin
            // 如果该buffer存储的行号 < (当前coord_y_int - 2)
            if (buf_line_id[i] < coord_y_int - 2) begin
                buf_state[i] <= BUF_EMPTY;
                // 通知 L1 侧：可继续接收新行
                l2_buf_release[i] <= 1'b1;
            end
        end
    end
end
```

---

## 5. L1 Wrapper 实现参考

### 4.1 Generic 实现（默认）

```verilog
// 标准 Verilog 数组，工具自动推断为 BRAM 或分布式 RAM
reg [DATA_WIDTH-1:0] mem0 [0:MAX_WIDTH-1];
reg [DATA_WIDTH-1:0] mem1 [0:MAX_WIDTH-1];

// 输入打拍
always @(posedge clk) begin
    wr_valid_d1 <= wr_valid;
    wr_addr_d1  <= wr_addr;
    wr_data_d1  <= wr_data;
    wr_buf_sel_d1 <= wr_buf_sel;
end

// RAM 写入（带使能）
always @(posedge clk) begin
    if (wr_valid_d1) begin
        if (wr_buf_sel_d1 == 0)
            mem0[wr_addr_d1] <= wr_data_d1;
        else
            mem1[wr_addr_d1] <= wr_data_d1;
    end
end

// RAM 读出（组合逻辑 + 输出打拍）
reg [DATA_WIDTH-1:0] rd_data_raw;
always @(*) begin
    rd_data_raw = (rd_buf_sel == 0) ? mem0[rd_addr] : mem1[rd_addr];
end

always @(posedge clk) begin
    if (rd_valid)
        rd_data <= rd_data_raw;
end
```

### 4.2 Xilinx 实现

使用 XPM_MEMORY_SDPRAM 原语，配置：
- `MEMORY_PRIMITIVE`: "block" 或 "distributed"
- `READ_LATENCY_B`: 2（内部打拍）
- `CLOCKING_MODE`: "common_clock"

### 4.3 Lattice 实现

使用 PMI_RAM_DP 原语，或标准 Verilog（Diamond 推断 EBR）。

---

## 6. V-Filter（垂直插值）

### 4.1 功能描述

- **输入**：4 个像素（异行同列：P(x,y), P(x,y+1), P(x,y+2), P(x,y+3)）
- **输出**：1 个 VPix（垂直插值结果）
- **延迟**：1 时钟（组合逻辑或1级寄存）

### 4.2 双三次插值实现

```verilog
module v_filter_4tap #(
    parameter DATA_WIDTH = 8,
    parameter FRAC_BITS  = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    input  wire [DATA_WIDTH-1:0] p0, p1, p2, p3,  // 4行同列像素
    input  wire [FRAC_BITS-1:0]  dy_frac,        // 垂直权重
    input  wire                  valid_in,
    
    output reg  [DATA_WIDTH-1:0] v_pix,          // 垂直插值结果
    output reg                   valid_out
);
    // 双三次滤波系数计算（基于dy_frac）
    // 或使用预计算的系数表
    
    wire [DATA_WIDTH+FRAC_BITS:0] mult0, mult1, mult2, mult3;
    wire [DATA_WIDTH+FRAC_BITS+1:0] sum;
    
    assign mult0 = p0 * coef0;
    assign mult1 = p1 * coef1;
    assign mult2 = p2 * coef2;
    assign mult3 = p3 * coef3;
    
    assign sum = mult0 + mult1 + mult2 + mult3;
    
    always @(posedge clk) begin
        v_pix    <= #U_DLY sum >> FRAC_BITS;
        valid_out <= #U_DLY valid_in;
    end
endmodule
```

---

## 7. H-Filter（水平插值）

### 5.1 功能描述

- **输入**：V-filter 输出的 VPix 流
- **输出**：最终像素（水平插值结果）
- **延迟**：4 时钟（4-tap 流水线）

### 5.2 4-Tap 移位寄存器实现

```verilog
module h_filter_4tap #(
    parameter DATA_WIDTH = 8,
    parameter FRAC_BITS  = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    input  wire [DATA_WIDTH-1:0] v_pix,      // V-filter输出
    input  wire [FRAC_BITS-1:0]  dx_frac,    // 水平权重
    input  wire                  valid_in,
    
    output reg  [DATA_WIDTH-1:0] h_pix,      // 最终输出
    output reg                   valid_out
);
    // 4-tap移位寄存器
    reg [DATA_WIDTH-1:0] shift_reg [0:3];
    reg [1:0]            shift_cnt;
    
    always @(posedge clk) begin
        if (valid_in) begin
            // 移位
            shift_reg[3] <= shift_reg[2];
            shift_reg[2] <= shift_reg[1];
            shift_reg[1] <= shift_reg[0];
            shift_reg[0] <= v_pix;
            
            shift_cnt <= shift_cnt + 1;
            
            // 凑齐4个后开始输出
            if (shift_cnt == 3) begin
                // 双三次水平插值
                h_pix <= calc_bicubic(shift_reg[0], shift_reg[1], 
                                      shift_reg[2], shift_reg[3], dx_frac);
                valid_out <= 1'b1;
            end
        end else begin
            valid_out <= 1'b0;
        end
    end
endmodule
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
