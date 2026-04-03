# bilinear_scaler V+H 架构设计文档

## 1. 架构概述

基于 PG231 的 **V+H 分离架构**（先垂直插值，后水平插值），采用 **4-Buffer 轮转 + 1 行输出 FIFO** 设计，确保显示连续性。

### 1.1 核心特性

- **V+H 分离**：V-filter（异行同列）→ H-filter（同行异列）
- **流式处理**：第 4 行输入时即可开始输出
- **双时钟域**：`clk_in`（输入/DDR）+ `clk_out`（输出/显示）
- **输出缓冲**：1 行 FIFO 吸收速率波动，防止显示 underrun

### 1.2 架构框图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         bilinear_scaler (V+H Architecture)                      │
│                                                                                 │
│  clk_in domain                              clk_out domain                      │
│  ┌─────────────────────┐                   ┌─────────────────────┐              │
│  │   Input Buffer      │                   │   Output FIFO       │              │
│  │   (4-Line Ring)     │                   │   (1-Line Buffer)   │              │
│  │                     │                   │                     │              │
│  │  ┌─────┐ ┌─────┐   │   Async FIFO      │  ┌─────────────┐   │              │
│  │  │Line0│ │Line1│   │   (Status Sync)   │  │  1-Line     │   │              │
│  │  │(v0) │ │(v1) │   │◄─────────────────►│  │  Buffer     │   │              │
│  │  └─────┘ └─────┘   │                   │  │  (3840x8b)  │   │              │
│  │  ┌─────┐ ┌─────┐   │                   │  └──────┬──────┘   │              │
│  │  │Line2│ │Line3│   │                   │         │          │              │
│  │  │(v2) │ │(v3) │   │                   │    o_valid/o_ready  │              │
│  │  └─────┘ └─────┘   │                   │         │          │              │
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
| **L1: 输入缓冲** | **2 行** | DDR 预缓冲，提高突发效率 | line_buffer_2row_wrapper |
| **L2: V-filter 缓冲** | **4 行** | 垂直插值运算 | scaler 内部 4-line ring |
| **L3: 输出 FIFO** | **1 行** | 显示连续性 | output_line_fifo |

**总存储需求：7 行 × src_width × DATA_WIDTH**

### 2.2 L1 输入缓冲设计（Wrapper 封装）

#### 设计原则

- **器件无关**：通过 wrapper 封装，内部可用 Generic RAM、Xilinx BRAM、Lattice EBR 等
- **时序收敛**：wrapper 内部对输入/输出打拍，上层模块无需关心时序细节
- **单元测试**：wrapper 可单独仿真验证，确保跨器件迁移时只需测试 wrapper

#### 模块接口

```verilog
module line_buffer_2row_wrapper #(
    parameter DATA_WIDTH = 8,
    parameter MAX_WIDTH  = 4096,
    parameter ADDR_WIDTH = $clog2(MAX_WIDTH),
    parameter VENDOR     = "GENERIC"  // "GENERIC", "XILINX", "LATTICE"
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // 写端口（DDR 输入）
    input  wire                  wr_valid,
    input  wire [ADDR_WIDTH-1:0] wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire                  wr_last,
    output wire                  wr_ready,      // 反压信号
    
    // 读端口（V-filter 消费）
    input  wire                  rd_valid,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [DATA_WIDTH-1:0] rd_data,       // 内部打拍输出
    output wire                  rd_data_valid, // 输出有效指示
    input  wire                  rd_switch,     // 切换 buffer（读完一行）
    output wire [1:0]            buf_status     // 00=空, 01=满1, 10=满2, 11=全满
);
```

#### 内部时序打拍设计

```verilog
// wrapper 内部实现：输入/输出各打一拍，方便时序收敛

// 输入打拍（消除外部输入 setup time 压力）
reg                  wr_valid_d1;
reg [ADDR_WIDTH-1:0] wr_addr_d1;
reg [DATA_WIDTH-1:0] wr_data_d1;

always @(posedge clk) begin
    wr_valid_d1 <= wr_valid;
    wr_addr_d1  <= wr_addr;
    wr_data_d1  <= wr_data;
end

// RAM 核心（根据 VENDOR 选择实现）
// ... （generic/xilinx/lattice 实现）

// 输出打拍（改善输出时钟到 clock out 的延迟）
reg [DATA_WIDTH-1:0] rd_data_raw;

always @(posedge clk) begin
    if (rd_valid)
        rd_data_raw <= mem[rd_addr];  // 组合逻辑读或同步读
end

always @(posedge clk) begin
    rd_data <= rd_data_raw;  // 再打一拍输出
end

assign rd_data_valid = 1'b1;  // 或根据实际延迟调整
```

#### 2 行 Ping-Pong 控制

```verilog
// 内部状态机控制 2 行 buffer 的切换
localparam BUF_EMPTY = 2'b00;
localparam BUF_HALF  = 2'b01;  // 1行满
localparam BUF_FULL  = 2'b10;  // 2行满

reg [1:0] buf_state;
reg       wr_buf_sel;  // 当前写入的 buffer（0 或 1）
reg       rd_buf_sel;  // 当前读取的 buffer（0 或 1）

// 写满一行后切换
always @(posedge clk) begin
    if (wr_last && wr_valid_d1) begin
        wr_buf_sel <= ~wr_buf_sel;
        // 标记当前 buffer 已满
    end
end

// 读完后切换（由外部 rd_switch 控制）
always @(posedge clk) begin
    if (rd_switch)
        rd_buf_sel <= ~rd_buf_sel;
end

// 反压控制：只有当两个 buffer 都满时才反压 DDR
assign wr_ready = (buf_state != BUF_FULL);
```

#### 单元测试策略

wrapper 单独测试，确保：
1. **写满一行后自动切换**：验证 ping-pong 逻辑
2. **读写完切换**：验证 rd_switch 功能
3. **反压时序**：验证 wr_ready 在满时拉低
4. **跨时钟域**（如果是双口 RAM）：验证异步读写

### 2.3 L2 V-filter 缓冲设计

4 行环形缓冲，直接嵌入 scaler 内部（非 wrapper，因为与算法紧耦合）。

### 2.4 L3 输出 FIFO 设计

1 行缓冲，平滑输出到显示接口。详见第 6 节。

---

## 3. 数据流详解

### 3.1 阶段划分

| 阶段 | 操作 | 延迟 | 说明 |
|------|------|------|------|
| **Stage 1** | DDR 写入 L1 缓冲 | 2 行时间 | 2 行 ping-pong 预缓冲 |
| **Stage 2** | L1 → L2 传输 | 1 行时间 | 读入 V-filter 4 行缓冲 |
| **Stage 3** | V-filter | 1 时钟/列 | 异行同列，输出 VPix |
| **Stage 4** | H-filter | 1 时钟/像素 | 同行异列，输出最终像素 |
| **Stage 5** | 输出 FIFO | 1~N 时钟 | 缓冲 1 行，确保显示连续性 |

### 3.2 详细时序（前 10 行）

```
T0: DDR 突发写行1 → L1 buffer0（输入 wrapper）
    L1状态: [buf0=行1, buf1=空]
    L2(V-filter): [v0~v3=空]

T1: DDR 突发写行2 → L1 buffer1
    L1状态: [buf0=行1, buf1=行2]
    【2行齐备，开始 transfer】
    
    Transfer: L1 buf0 → L2 v0
    （逐列读取，L1 输出打拍保证时序）

T2: DDR 突发写行3 → L1 buffer0（覆盖，因为行1已 transfer 完）
    Transfer: L1 buf1 → L2 v1

T3: DDR 突发写行4 → L1 buffer1
    Transfer: L1 buf0 → L2 v2

T4: DDR 突发写行5 → L1 buffer0
    Transfer: L1 buf1 → L2 v3
    【L2 4行齐全，开始 V-filter】
    
    V-filter开始：读 v0[0],v1[0],v2[0],v3[0] → VPix[0]
    H-filter：VPix 进入移位寄存器
    ...
    
T5: DDR 写行6 → L1
    Transfer 行6 → v0（覆盖已用完的行1）
    V-filter: 读 v1,v2,v3,v0(新) → 下一行 VPix
    
...持续流水线
```

### 2.2 详细时序（前 10 行）

```
T0: 行1 → Buffer v0（第1行源数据）
    Buffer状态: [v0=行1, v1=空, v2=空, v3=空]
    输出: 无（攒行中）

T1: 行2 → Buffer v1
    Buffer状态: [v0=行1, v1=行2, v2=空, v3=空]
    输出: 无

T2: 行3 → Buffer v2
    Buffer状态: [v0=行1, v1=行2, v2=行3, v3=空]
    输出: 无

T3: 行4 → Buffer v3
    Buffer状态: [v0=行1, v1=行2, v2=行3, v3=行4]
    【4行齐全，开始流水线】
    
    V-filter开始（第0列）:
    - 读 v0[0], v1[0], v2[0], v3[0]（异行同列）
    - 计算 VPix[0]
    - H-filter: VPix进入4-tap移位寄存器
    
    （继续第1列...第3列）
    - VPix[1], VPix[2], VPix[3] 进入移位寄存器
    - H-filter: 凑齐4个VPix，输出第0个像素 → 写入Output FIFO

T3~T4: 持续运算
    - 每时钟：V-filter输出1个VPix
    - H-filter每时钟输出1个像素（延迟3时钟后）
    
T4: 行5 → Buffer v0（覆盖行1，因为行1已用完）
    Buffer状态: [v0=行5, v1=行2, v2=行3, v3=行4]
    
    V-filter（下一目标行）:
    - 读 v1[0], v2[0], v3[0], v0[0]（新行5的第0列）
    - 计算下一行VPix[0]
    
    H-filter继续输出上一行剩余像素...

T5~T9: 循环使用 v0~v3
    Buffer轮转: v1→v2→v3→v0→v1...
    每来1行新数据，覆盖最旧的1行
```

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
| **输入延迟** | 4 行 | 攒够4行才能开始输出 |
| **流水线延迟** | 6~8 时钟 | V-filter(1) + H-filter(4) + FIFO(1~3) |
| **吞吐率** | 1 像素/时钟 | 满吞吐率运行 |
| **Buffer需求** | 4 行 + 1 行 FIFO | 共5行存储 |
| **支持缩放** | 任意比例 | 0.25x ~ 4x（取决于系数精度） |
| **支持算法** | 双线性/双三次 | 通过tap数和系数配置 |

---

## 12. 验证要点

1. **Buffer轮转**：验证4行buffer正确循环使用，无覆盖错误
2. **放大场景**：验证源行被多次复用后才释放
3. **缩小场景**：验证源行快速释放，无资源泄漏
4. **跨时钟域**：验证双时钟域同步无亚稳态
5. **FIFO边界**：验证FIFO满/空时数据流正确反压
6. **行同步**：验证o_last/o_frame_start时序正确

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
