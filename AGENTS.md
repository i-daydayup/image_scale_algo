# FPGA 工程师 - 编程习惯与规范

> 本文件记录 FPGA 工程师（视频图像处理方向）的编程习惯，供 AI 助手参考。

---

## 1. 技术背景

- **领域**：FPGA 数字逻辑设计，专注视频图像处理
- **开发语言**：
  - Python：算法验证、测试向量生成、结果比对
  - Verilog：RTL 设计、硬件实现
- **典型项目流程**：
  1. Python 实现算法原型并验证
  2. 将算法转换为 Verilog RTL
  3. Python 生成测试向量，验证 Verilog 实现
  4. 对比 Python 参考模型与 Verilog 仿真结果

---

## 2. Python 代码规范

### 2.1 基础风格
- **缩进**：使用 `Tab`（而非空格）
- **编码**：UTF-8
- **行宽**：可以最大 140 字符
- **换行**：Unix 风格 (LF)

### 2.2 命名规范
```python
# 模块/文件：snake_case
image_scaler.py
video_pipeline.py

# 类：PascalCase
class ImageScaler:
    pass

# 函数/变量：snake_case
def scale_image(input_data, scale_factor):
    output_buffer = []
    pixel_count = 0
    return output_buffer

# 常量：全大写 + 下划线
MAX_IMAGE_WIDTH = 1920
DEFAULT_SCALE_FACTOR = 2.0

# 私有成员：单下划线前缀
_def _internal_helper():
    pass
```

### 2.3 代码对齐规范

#### 等号对齐原则
**相邻的多行赋值语句，等号必须对齐**（适用于 Python 和 Verilog）：

```python
# 正确：等号对齐
self.scale_factor = scale_factor
self.target_size  = target_size

# 正确：等号对齐（带行内注释）
gap          = 20  # 中间间隔
label_height = 40  # 标签高度

# 错误：等号未对齐
self.scale_factor = scale_factor
self.target_size = target_size
```

**规则说明：**
1. 上下相邻的赋值语句（逻辑上相关的代码块），等号应对齐
2. 等号两侧各保留一个空格
3. 变量名长度不同时，较短的变量名后用空格补齐，使等号位置一致
4. 如果有行内注释，注释与代码之间也保持对齐或至少一个空格

### 2.4 视频图像处理专用规范

#### 数组维度约定
```python
# 图像数据维度顺序: (height, width, channels) 或 (height, width)
# 避免使用 (width, height) 以免造成混淆

# 正确
image = np.zeros((1080, 1920, 3), dtype=np.uint8)  # HWC 格式

# 显式命名维度变量
img_height, img_width = image.shape[:2]
```

#### 像素值处理
```python
# 像素值范围显式说明
# 8-bit: 0-255
# 10-bit: 0-1023
# 定点数: 根据 Q 格式指定 (如 Q8.8 表示 8位整数, 8位小数)

# 饱和/截断函数显式定义
def clip_to_uint8(value):
    """将值裁剪到 0-255 范围"""
    return max(0, min(255, int(value)))

def fixed_point_multiply(a, b, frac_bits=8):
    """定点数乘法，输入为整数表示的定点数"""
    result = (a * b) >> frac_bits
    return result
```

#### 测试向量生成
```python
def generate_test_pattern(width, height, pattern='ramp'):
    """
    生成测试图像

    Args:
        width: 图像宽度
        height: 图像高度
        pattern: 'ramp'(渐变), 'checker'(棋盘), 'solid'(纯色)

    Returns:
        numpy.ndarray: 测试图像数据
    """
    if pattern == 'ramp':
        # 水平渐变，用于测试插值算法
        return np.tile(np.arange(width, dtype=np.uint8), (height, 1))
    # ...
```

### 2.4 算法验证代码结构
```python
#!/usr/bin/env python3
"""
算法名称: 双线性插值缩放
对应RTL: rtl/image_scaler.v
验证状态: ✓ 通过 / ✗ 未通过
"""

import numpy as np
from PIL import Image

# ==================== 参考模型 ====================

class BilinearScaler:
    """
    双线性插值缩放算法的 Python 参考实现
    用于与 Verilog RTL 结果对比
    """

    def __init__(self, scale_factor):
        self.scale_factor = scale_factor

    def process(self, input_image):
        """处理单帧图像"""
        # 实现算法...
        return output_image

# ==================== 测试框架 ====================

def run_test_case(test_image, expected_output=None):
    """运行单个测试用例"""
    scaler = BilinearScaler(scale_factor=2.0)
    result = scaler.process(test_image)

    if expected_output is not None:
        mismatch = np.sum(result != expected_output)
        assert mismatch == 0, f"Mismatch count: {mismatch}"

    return result

def save_test_vectors(filename, data):
    """保存测试向量供 Verilog 仿真使用（如 hex/binary 格式）"""
    with open(filename, 'w') as f:
        for row in data:
            for pixel in row:
                f.write(f"{pixel:02x}\n")  # 16进制格式

if __name__ == '__main__':
    # 生成测试数据
    test_img = generate_test_pattern(100, 100, 'ramp')

    # 运行验证
    result = run_test_case(test_img)

    # 保存测试向量
    save_test_vectors('test_vectors/input.hex', test_img)
    save_test_vectors('test_vectors/golden.hex', result)
```

---

## 3. Verilog 代码规范

### 3.1 基础风格
- **缩进**：Tab（1 Tab = 4 空格宽度）
- **命名**：`snake_case` 全小写 + 下划线
- **文件扩展名**：`.v` (Verilog-2001) 或 `.sv` (SystemVerilog)
- **换行**：Unix 风格 (LF)

#### 等号对齐（与 Python 相同规则）
相邻的多行赋值/声明语句，等号必须对齐：

```verilog
// 正确：等号对齐
localparam DATA_WIDTH = 8;
localparam IMG_WIDTH  = 1920;

// 正确：端口声明对齐
wire        pixel_valid;
wire [7:0]  pixel_data;
reg  [15:0] line_buffer;

// 正确：赋值语句对齐
assign ready = (state == IDLE);
assign done  = (state == DONE);
```

#### 信号声明四级对齐（模块端口声明）

模块端口声明按**四级结构**对齐（信号类型 + 位宽 + 信号名 + 注释）：

```verilog
// 正确示例：四级对齐
input  wire                          wr_en,      // 写使能
input  wire [$clog2(LINE_WIDTH)-1:0] wr_addr,    // 写地址
input  wire [DATA_WIDTH-1:0]         wr_data,    // 写数据

// 正确示例：不同位宽对齐
wire [15:0]                          dst_idx;    // I16
wire [INT_BITS+FRAC_BITS-1:0]        inv_scale;  // I20
wire [15:0]                          src_size;   // I16
```

**四级对齐规则：**

| 级别 | 内容 | 对齐方式 |
|------|------|----------|
| 第1列 | 信号类型 | `input`/`output`/`wire`/`reg` 左对齐 |
| 第2列 | 位宽 | `[DATA_WIDTH-1:0]` 等左对齐 |
| 第3列 | 信号名 | 位宽后空一格，信号名左对齐 |
| 第4列 | 注释 | `//` 对齐（调节信号名后的空格） |

**计算方法：**
1. 找出该组信号中最长的位宽声明
2. 所有位宽在该位置左对齐
3. 位宽后空一格写信号名
4. 信号名后空格调节，使 `//` 对齐

### 3.2 命名规范
```verilog
// 模块名：snake_case
module image_scaler (
    // 端口...
);

// 信号命名
wire        pixel_valid;      // 控制信号
wire [7:0]  pixel_data;       // 数据信号
reg  [15:0] line_buffer;      // 存储信号

// 时钟/复位
wire        clk;
wire        rst_n;            // 低电平有效复位，_n 后缀

// 参数/宏定义
localparam DATA_WIDTH = 8;
localparam IMG_WIDTH  = 1920;

// 状态机命名
typedef enum logic [2:0] {
    STATE_IDLE,
    STATE_READ,
    STATE_PROCESS,
    STATE_WRITE,
    STATE_DONE
} state_t;
state_t current_state, next_state;
```

### 3.3 文件头模板
```verilog
//============================================================================
// 模块名称    : image_scaler
// 功能描述    : 图像双线性插值缩放模块
// 作者        :
// 创建日期    :
// 版本历史    :
//   v1.0 - 初始版本
//   v1.1 - 修复边界条件处理
//
// 对应Python  : python/image_scaler.py
// 验证状态    : ✓ 通过 / ✗ 未通过 / ⏳ 验证中
//============================================================================

`timescale 1ns / 1ps

module image_scaler #(
    parameter DATA_WIDTH = 8,
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // 输入接口
    input  wire                  i_valid,
    input  wire [DATA_WIDTH-1:0] i_data,
    output wire                  i_ready,

    // 输出接口
    output reg                   o_valid,
    output reg  [DATA_WIDTH-1:0] o_data,
    input  wire                  o_ready
);

// 模块实现...

endmodule
```

### 3.4 视频接口规范

#### 常用视频流接口 (AXI-Stream 风格)
```verilog
// 输入视频流
input  wire                  s_axis_tvalid,
input  wire [DATA_WIDTH-1:0] s_axis_tdata,
input  wire                  s_axis_tlast,    // 行结束标记
input  wire                  s_axis_tuser,    // 帧开始标记
output wire                  s_axis_tready,

// 输出视频流
output reg                   m_axis_tvalid,
output reg  [DATA_WIDTH-1:0] m_axis_tdata,
output reg                   m_axis_tlast,
output reg                   m_axis_tuser,
input  wire                  m_axis_tready
```

#### 行缓冲/帧缓冲接口
```verilog
// 双端口 RAM 接口
output reg  [ADDR_WIDTH-1:0]  raddr,
input  wire [DATA_WIDTH-1:0]  rdata,
output reg                    ren,

output reg  [ADDR_WIDTH-1:0]  waddr,
output reg  [DATA_WIDTH-1:0]  wdata,
output reg                    wen
```

### 3.5 代码结构规范

```verilog
module example (
    // 端口声明
);

    //------------------------------------------------------------------------
    // 参数定义
    //------------------------------------------------------------------------
    localparam IDLE  = 3'b000;
    localparam READ  = 3'b001;
    // ...

    //------------------------------------------------------------------------
    // 信号声明
    //------------------------------------------------------------------------
    // 按功能分组

    // 控制信号
    reg [2:0] state;
    reg       frame_start;

    // 数据路径
    reg [7:0] pixel_reg [0:3];  // 4-tap 滤波器寄存器
    reg [15:0] accumulator;

    // 计数器
    reg [11:0] col_cnt;
    reg [10:0] row_cnt;

    //------------------------------------------------------------------------
    // 组合逻辑
    //------------------------------------------------------------------------

    // 状态机下一状态逻辑
    always @(*) begin
        case (state)
            IDLE: next_state = start ? READ : IDLE;
            // ...
        endcase
    end

    // 输出逻辑
    assign ready = (state == IDLE);

    //------------------------------------------------------------------------
    // 时序逻辑
    //------------------------------------------------------------------------

    // 状态寄存器
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            state <= IDLE;
        end
        else begin
            state <= next_state;
        end
    end

    // 数据路径
    always @(posedge clk) begin
        if (pixel_valid == 1'b1) begin
            pixel_reg[0] <= pixel_reg[1];
            pixel_reg[1] <= pixel_reg[2];
            pixel_reg[2] <= pixel_reg[3];
            pixel_reg[3] <= i_data;
        end
    end

endmodule
```

#### always 块编码规范

**1. 条件判断必须使用 `==` 方式，禁止使用 `!` 取反：**

```verilog
// 正确：使用 == 进行判断
always @(posedge clk or negedge rst_n) begin
    if (rst_n == 1'b0) begin
        state <= IDLE;
    end
    else if (start == 1'b1) begin
        state <= READ;
    end
    else begin
        state <= next_state;
    end
end

// 错误：使用 ! 取反
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin           // 禁止使用 !
        state <= IDLE;
    end else if (start) begin   // 禁止使用隐式判断
        state <= READ;
    end
end
```

**2. else 必须换行，不得跟在 end 后面：**

```verilog
// 正确：else 换行
if (rst_n == 1'b0) begin
    state <= IDLE;
end
else if (start == 1'b1) begin
    state <= READ;
end
else begin
    state <= next_state;
end

// 错误：else 跟在 end 后面
if (rst_n == 1'b0) begin
    state <= IDLE;
end else begin               // 禁止：else 应换行
    state <= next_state;
end
```

### 3.6 定点数运算规范

```verilog
// Qm.n 格式定义：m位整数，n位小数
// 例如 Q8.8: 8位整数，8位小数，共16位

localparam INT_BITS  = 8;   // 整数位
localparam FRAC_BITS = 8;   // 小数位
localparam TOTAL_BITS = INT_BITS + FRAC_BITS;

// 定点数乘法 (结果需要右移 FRAC_BITS 位)
// (a * b) >> FRAC_BITS
reg [TOTAL_BITS*2-1:0] mult_temp;
reg [TOTAL_BITS-1:0]   mult_result;

always @(posedge clk) begin
    mult_temp    <= coeff * pixel_value;        // 乘法
    mult_result  <= mult_temp >> FRAC_BITS;     // 右移取整
end

// 定点数加/减法：直接运算，注意溢出处理
reg [TOTAL_BITS:0] add_temp;  // 多1位用于溢出检测

always @(*) begin
    add_temp = operand_a + operand_b;
    if (add_temp[TOTAL_BITS])  // 溢出检测
        add_result = {TOTAL_BITS{1'b1}};  // 饱和到最大值
    else
        add_result = add_temp[TOTAL_BITS-1:0];
end
```

---

## 4. 项目目录结构

```
project/
├── docs/                       # 文档
│   ├── algorithm_spec.md       # 算法规格说明
│   └── interface_spec.md       # 接口规格说明
│
├── python/                     # Python 算法验证
│   ├── reference_models/       # 参考模型
│   │   ├── image_scaler.py
│   │   └── color_space.py
│   ├── testbenches/            # 测试脚本
│   │   ├── test_scaler.py
│   │   └── test_color_convert.py
│   ├── utils/                  # 工具函数
│   │   ├── image_io.py
│   │   ├── test_vector_gen.py
│   │   └── compare_utils.py
│   └── results/                # 验证结果
│       ├── output_images/
│       └── test_vectors/       # 供Verilog使用的测试向量
│
├── rtl/                        # Verilog RTL 源码
│   ├── common/                 # 通用模块
│   │   ├── sync_fifo.v
│   │   ├── line_buffer.v
│   │   └── pixel_delay.v
│   ├── image_scaler/           # 图像缩放模块
│   │   ├── image_scaler.v
│   │   ├── bilinear_interp.v
│   │   └── scale_controller.v
│   └── top/                    # 顶层模块
│       └── video_processing_top.v
│
├── sim/                        # 仿真相关
│   ├── tb/                     # Testbench
│   │   └── tb_image_scaler.v
│   ├── modelsim/               # ModelSim 工程
│   └── vivado/                 # Vivado 仿真
│
├── constraints/                # 约束文件
│   └── timing.xdc
│
├── scripts/                    # 脚本工具
│   ├── run_simulation.py       # 一键仿真脚本
│   ├── check_results.py        # 结果比对脚本
│   └── build_bitstream.tcl     # 生成比特流
│
└── AGENTS.md                   # 本文件
```

---

## 5. 验证工作流程

### 5.1 算法验证流程
```
1. 编写 Python 参考模型
   └─> 使用浮点数实现黄金参考

2. 定点化分析
   └─> 确定所需的整数位/小数位
   └─> Python 实现定点化版本

3. 生成测试向量
   └─> Python 生成输入数据 (hex/bin)
   └─> Python 生成期望输出 (golden)

4. Verilog 实现
   └─> 编写 RTL
   └─> 编写 Testbench

5. 协同仿真
   └─> Verilog 仿真读取输入数据
   └─> 输出结果保存到文件

6. 结果比对
   └─> Python 读取 Verilog 输出
   └─> 与 golden 对比
   └─> 生成差异报告
```

### 5.2 Python 验证脚本模板
```python
#!/usr/bin/env python3
"""
RTL 验证脚本
对比 Verilog 仿真输出与 Python 参考模型
"""

import numpy as np
from pathlib import Path

def read_verilog_output(filepath, width, height):
    """读取 Verilog 仿真输出的 hex 文件"""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            data.append(int(line.strip(), 16))
    return np.array(data).reshape((height, width))

def compare_results(golden, actual, tolerance=1):
    """
    对比结果，允许一定误差(tolerance)
    定点数运算可能产生 ±1 的误差
    """
    diff = np.abs(golden.astype(np.int16) - actual.astype(np.int16))
    max_diff = np.max(diff)
    mismatch_count = np.sum(diff > tolerance)

    print(f"最大误差: {max_diff}")
    print(f"超差像素数: {mismatch_count} / {golden.size}")

    if mismatch_count > 0:
        print("错误位置:")
        y, x = np.where(diff > tolerance)
        for i in range(min(10, len(y))):  # 最多显示10个
            print(f"  ({y[i]}, {x[i]}): golden={golden[y[i],x[i]]}, "
                  f"actual={actual[y[i],x[i]]}, diff={diff[y[i],x[i]]}")

    return mismatch_count == 0

def main():
    # 读取 golden 参考
    golden = read_verilog_output('python/results/golden.hex', 1920, 1080)

    # 读取 Verilog 输出
    actual = read_verilog_output('sim/output.dat', 1920, 1080)

    # 对比
    if compare_results(golden, actual, tolerance=1):
        print("✓ 验证通过")
        return 0
    else:
        print("✗ 验证失败")
        return 1

if __name__ == '__main__':
    exit(main())
```

---

## 6. 常用代码片段

### 6.1 Python - 图像与测试向量转换
```python
def image_to_hex_file(image, filepath, format='rgb565'):
    """将图像保存为 Verilog 可读的 hex 文件"""
    with open(filepath, 'w') as f:
        for row in image:
            for pixel in row:
                if format == 'rgb565':
                    r, g, b = pixel[0] >> 3, pixel[1] >> 2, pixel[2] >> 3
                    value = (r << 11) | (g << 5) | b
                    f.write(f"{value:04x}\n")
                elif format == 'gray8':
                    f.write(f"{pixel:02x}\n")

def hex_file_to_image(filepath, width, height, format='gray8'):
    """从 hex 文件读取图像数据"""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            data.append(int(line.strip(), 16))

    if format == 'gray8':
        return np.array(data, dtype=np.uint8).reshape((height, width))
    # ...
```

### 6.2 Verilog - 行缓冲 (Line Buffer)
```verilog
module line_buffer #(
    parameter DATA_WIDTH = 8,
    parameter LINE_WIDTH = 1920
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,

    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_valid
);

    reg [DATA_WIDTH-1:0] mem [0:LINE_WIDTH-1];
    reg [$clog2(LINE_WIDTH)-1:0] wr_addr;
    reg [$clog2(LINE_WIDTH)-1:0] rd_addr;

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

    // 地址逻辑...

endmodule
```

---

## 7. AI 助手工作指南

### 编写 Python 代码时
- [ ] 使用 **Tab 缩进**
- [ ] **相邻赋值语句等号对齐**（见 2.3 代码对齐规范）
- [ ] 视频图像处理使用 `(H, W, C)` 维度顺序
- [ ] 提供清晰的定点数转换函数
- [ ] 包含算法对应的 Verilog 模块注释
- [ ] 生成测试向量保存到 `results/test_vectors/`

### 编写 Verilog 代码时
- [ ] 使用 **snake_case 命名**
- [ ] **相邻赋值/声明语句等号对齐**（见 3.1 基础风格）
- [ ] 包含标准文件头（模块描述、Python对应关系、验证状态）
- [ ] 信号名清晰表达功能（如 `pixel_valid` 而非 `vld`）
- [ ] 状态机使用 `localparam` 或 `typedef enum`
- [ ] 时钟使用 `clk`，低电平复位使用 `rst_n`
- [ ] 复杂算法先写注释说明定点化方案

### 算法转 RTL 时
- [ ] 询问/确认定点数格式 (Qm.n)
- [ ] 询问行缓冲深度、处理延迟等参数
- [ ] 生成对应的 Python 参考模型用于验证
- [ ] 提供测试向量生成脚本

---

## 8. 待补充/讨论
- [ ] 是否使用 SystemVerilog 特性？
- [ ] 验证工具：cocotb / 纯 Python / 其他？
- [ ] FPGA 厂商偏好：Xilinx / Altera / Lattice？
- [ ] 仿真工具：ModelSim / Vivado Sim / Verilator？
