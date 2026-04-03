# 双线性插值图像缩放器 - RTL 实现

基于 `python/algorithms/bilinear_fixed.py` 定点算法实现的 Verilog RTL。

## 目录结构

```
rtl/
├── README.md                          # 本文件
├── src/                               # RTL 源码
│   ├── bilinear_scaler_vh.v           # V+H架构顶层模块（双线性）
│   ├── line_buffer_2row_wrapper.v     # L1输入缓冲：2行ping-pong
│   ├── bilinear_coord_calc.v          # 坐标计算模块（可复用）
│   ├── bilinear_interp.v              # 插值计算模块（可复用）
│   ├── bilinear_scaler_arch.md        # 架构设计文档
│   ├── filelist.f                     # 仿真文件列表
│   └── common/                        # 器件特定代码（Vendor Specific）
│       └── （预留，用于Xilinx/Lattice特定实现）
├── sim/                               # 仿真测试
│   └── ut_bilinear_scaler/            # 单元测试平台
│       ├── ut_bilinear_scaler.v       # Testbench
│       ├── ut_bilinear_scaler.do      # ModelSim仿真脚本
│       └── work/                      # 仿真编译目录
└── ref/                               # 参考文档
    └── pg231-v-proc-ss-20240221.pdf   # Xilinx Video Processing Subsystem
```

## 设计状态

⚠️ **当前状态**：`bilinear_scaler.v` 采用 **2-Buffer + 单时钟域 + 状态机** 架构，存在以下问题：
- 输入输出速率不匹配时可能覆盖数据
- 帧级状态机无法实现流式连续处理
- 不适合显示接口的连续输出要求

✅ **重构方向**：参考 `src/bilinear_scaler_arch.md`，采用：
- **3-Buffer 轮转**（与 PG231 一致）
- **双时钟域**（`clk_in` + `clk_out`）
- **完全流式**（删除帧级状态机）

## 模块说明

### 1. bilinear_scaler (顶层模块 - 待重构)

双线性插值图像缩放器主模块，支持 AXI-Stream 风格的视频流接口。

**当前架构问题**：见上文"设计状态"

#### 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `DATA_WIDTH` | 8 | 像素数据位宽 |
| `INT_BITS` | 12 | 坐标整数位宽 m (Qm.n) |
| `FRAC_BITS` | 8 | 坐标小数位宽 n (Qm.n) |
| `WEIGHT_BITS` | 8 | 权重小数位宽 w (Q0.w) |
| `MAX_WIDTH` | 4096 | 最大支持图像宽度 |
| `MAX_HEIGHT` | 4096 | 最大支持图像高度 |

#### 接口（当前版本 - 单时钟域）

**配置接口**（在 `i_frame_start` 前配置有效）：
- `cfg_inv_scale_x` [COORD_BITS]：X方向逆缩放比例定点值 (Qm.n)
- `cfg_inv_scale_y` [COORD_BITS]：Y方向逆缩放比例定点值 (Qm.n)
- `cfg_dst_width` [16]：目标图像宽度
- `cfg_dst_height` [16]：目标图像高度
- `cfg_src_width` [16]：源图像宽度
- `cfg_src_height` [16]：源图像高度

**输入视频流** (AXI-Stream 风格)：**clk 域**
- `i_valid`：输入数据有效
- `i_data` [DATA_WIDTH]：输入像素数据
- `i_last`：行结束标记
- `i_frame_start`：帧开始标记 (tuser)
- `i_ready`：模块就绪，可接收数据

**输出视频流**：**clk 域**
- `o_valid`：输出数据有效
- `o_data` [DATA_WIDTH]：输出像素数据
- `o_last`：行结束标记
- `o_frame_start`：帧开始标记
- `o_ready`：下游模块就绪

#### 接口（V2架构 - 双时钟域，计划中）

```verilog
// 输入时钟域 (DDR/源图像)
input  wire clk_in,           // 输入时钟
input  wire rst_n_in,         // 输入复位

// 输出时钟域 (显示/目标图像)  
input  wire clk_out,          // 输出时钟
input  wire rst_n_out,        // 输出复位

// 输入/输出 AXI-S 信号分别对应各自时钟域
```

#### 定点配置计算

逆缩放比例的定点值计算公式（Python 参考）：

```python
# 从浮点到定点
inv_scale_float = src_size / dst_size
inv_scale_fixed = int(inv_scale_float * (2 ** FRAC_BITS))
```

**示例**：源图 100x100，目标图 200x200（放大2倍）
```python
inv_scale_x = int(100/200 * 256) = 128  # FRAC_BITS=8
inv_scale_y = int(100/200 * 256) = 128
```

---

### 2. bilinear_coord_calc (坐标计算模块)

计算目标像素对应的原图坐标（定点实现）。

**输入到输出延迟：4 拍**

| 拍数 | 操作 |
|------|------|
| 1 | 计算 (2*dst_idx + 1) |
| 2 | 乘法 (2*dst_idx + 1) * inv_scale |
| 3 | 右移1位，减0.5，边界保护 |
| 4 | 分离整数和小数部分 |

---

### 3. bilinear_interp (插值计算模块)

双线性插值核心计算（定点实现）。

**输入到输出延迟：3 拍**

| 拍数 | 操作 |
|------|------|
| 1 | 计算 (1-dx), (1-dy) 和 4个权重乘法 |
| 2 | 权重右移，像素乘权重 |
| 3 | 累加并右移，饱和处理 |

**插值公式**（定点化）：
```
w00 = (1-dx) * (1-dy) >> w
w10 = dx * (1-dy) >> w
w01 = (1-dx) * dy >> w
w11 = dx * dy >> w
result = (P00*w00 + P10*w10 + P01*w01 + P11*w11) >> w
```

---

### 4. line_buffer (行缓冲模块)

行缓冲模块，存储一行像素数据。

**存储器实现**：标准 Verilog 数组（综合为 Block RAM 或分布式 RAM）

| 操作 | 延迟 |
|------|------|
| 写入到存储器更新 | 1 拍 |
| 读出请求到数据有效 | 1 拍 |

---

## 仿真测试

### 运行仿真

```bash
# 进入仿真目录
cd rtl/sim/ut_bilinear_scaler

# 使用 ModelSim/QuestaSim
vsim -c -do "run -all; quit" ut_bilinear_scaler

# 或使用 GUI 模式
vsim ut_bilinear_scaler
```

### Testbench 特性

`ut_bilinear_scaler.v` 包含：
- AXI-S 协议握手逻辑（`i_ready` 响应）
- 流式数据生成（水平渐变测试图）
- 帧开始标记（首像素为 0xF5）
- 输出数据保存到 `output_verilog.hex`

---

## 参考文档

### PG231 - Video Processing Subsystem (Xilinx/AMD)

**文件**: `ref/pg231-v-proc-ss-20240221.pdf` (v2.4)

**关键参考点**：
- **3-Buffer 架构**：Deinterlacer 使用 3 个 field buffers（验证我们 V2 架构）
- **Scaler 模式**：Stream Mode（无 DDR）vs Memory Mode（有 DDR）
- **V+H 分离**：垂直和水平滤波分离可减少资源
- **Polyphase**：多相缩放原理（phase bin 概念）

详见 `src/bilinear_scaler_arch.md` 第5节。

---

## 使用示例

### 放大2倍 (100x100 -> 200x200)

```verilog
// 配置参数
bilinear_scaler #(
    .DATA_WIDTH  (8),
    .INT_BITS    (12),
    .FRAC_BITS   (8),
    .WEIGHT_BITS (8)
) u_scaler (
    .clk             (clk),
    .rst_n           (rst_n),
    
    // 配置 (Qm.n = Q12.8)
    .cfg_inv_scale_x (128),  // 0.5 * 256
    .cfg_inv_scale_y (128),
    .cfg_dst_width   (200),
    .cfg_dst_height  (200),
    .cfg_src_width   (100),
    .cfg_src_height  (100),
    
    // 输入 (连接上游视频源)
    .i_valid         (src_valid),
    .i_data          (src_data),
    .i_last          (src_last),
    .i_frame_start   (src_frame_start),
    .i_ready         (src_ready),
    
    // 输出 (连接下游模块)
    .o_valid         (dst_valid),
    .o_data          (dst_data),
    .o_last          (dst_last),
    .o_frame_start   (dst_frame_start),
    .o_ready         (1'b1)  // 假设下游始终就绪
);
```

---

## 综合说明

### 资源预估（当前版本）

| 模块 | LUT | FF | RAM |
|------|-----|----|-----|
| bilinear_coord_calc | ~200 | ~150 | 0 |
| bilinear_interp | ~300 | ~200 | 0 |
| line_buffer (1920x8) | 0 | ~50 | ~2KB |
| **bilinear_scaler (当前)** | ~800 | ~600 | ~4KB |

### 资源预估（V2 3-Buffer架构）

| 模块 | LUT | FF | RAM |
|------|-----|----|-----|
| bilinear_scaler | ~1000 | ~800 | ~6KB |
| 跨时钟域同步 | ~100 | ~200 | 0 |

*注：实际资源消耗取决于参数配置和综合工具优化*

### 时钟频率

- 目标频率：150 MHz+ (Artix-7 或同级器件)
- 关键路径：乘法器 -> 加法器链

---

## 与 Python 参考模型对比

| 项目 | Python | Verilog |
|------|--------|---------|
| 定点格式 | Q12.8 / Q0.8 | Q12.8 / Q0.8 (可配置) |
| 边界处理 | clip | 饱和到边界 |
| 精度误差 | 浮点参考 | 与 Python 定点版本一致 |

**验证方法**：
1. Python 生成测试向量和 golden 结果
2. Verilog testbench 读取测试向量
3. 对比 Verilog 输出与 golden
4. 允许误差：±1 (定点数舍入误差)

---

## 待办事项

- [ ] 重构 `bilinear_scaler.v`：3-Buffer + 双时钟域架构
- [ ] 删除帧级状态机，改为流式控制
- [ ] 增加跨时钟域同步模块（脉冲同步器）
- [ ] 更新 Testbench 支持双时钟激励
- [ ] 与 PG231 架构对比验证
