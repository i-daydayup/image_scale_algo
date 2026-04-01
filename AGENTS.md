# FPGA 工程师 - 编程习惯与规范

> 本文件记录 FPGA 工程师（视频图像处理方向）的编程习惯，供 AI 助手参考。

---

## 1. 技术背景

- **领域**：FPGA 数字逻辑设计，专注视频图像处理
- **开发语言**：Python（算法验证）、Verilog（RTL 设计）
- **典型项目流程**：Python 原型 → Verilog RTL → 协同仿真 → 结果比对

---

## 2. Python 代码规范

### 2.1 基础风格
- **缩进**：Tab（1 Tab = 4 空格宽度）
- **编码**：UTF-8，换行：Unix 风格 (LF)
- **命名**：模块/函数/变量用 `snake_case`，类用 `PascalCase`，常量全大写

### 2.2 代码对齐规范
**相邻的多行赋值语句，等号必须对齐**（适用于 Python 和 Verilog）：

```python
# 正确：等号对齐
self.scale_factor = scale_factor
self.target_size  = target_size
gap          = 20  # 中间间隔
label_height = 40  # 标签高度
```

### 2.3 视频图像处理专用规范

```python
# 图像数据维度顺序: (height, width, channels) - HWC 格式
image = np.zeros((1080, 1920, 3), dtype=np.uint8)
img_height, img_width = image.shape[:2]

# 像素值范围显式说明：8-bit(0-255)、10-bit(0-1023)、定点数(Qm.n)
def clip_to_uint8(value):
    """将值裁剪到 0-255 范围"""
    return max(0, min(255, int(value)))

def fixed_point_multiply(a, b, frac_bits=8):
    """定点数乘法"""
    return (a * b) >> frac_bits
```

### 2.4 算法验证代码结构

```python
#!/usr/bin/env python3
"""算法名称: 双线性插值缩放 | 对应RTL: rtl/image_scaler.v | 验证状态: ✓ 通过"""

import numpy as np
from PIL import Image

class BilinearScaler:
    """双线性插值缩放算法的 Python 参考实现"""
    def __init__(self, scale_factor):
        self.scale_factor = scale_factor
    
    def process(self, input_image):
        """处理单帧图像"""
        # 实现算法...
        return output_image

def save_test_vectors(filename, data):
    """保存测试向量供 Verilog 仿真使用"""
    with open(filename, 'w') as f:
        for row in data:
            for pixel in row:
                f.write(f"{pixel:02x}\n")
```

---

## 3. Verilog 代码规范

### 3.1 基础风格
- **缩进**：Tab（1 Tab = 4 空格），**命名**：`snake_case`，**编码**：UTF-8
- **等号对齐**：相邻的多行赋值/声明语句，等号必须对齐（以**最长的左边**为基准）
- **信号/参数对齐**：相邻的多行赋值/声明/参数定义语句，左边（类型/关键字）必须对齐，等号/分号必须对齐（以**最长的左边**为基准，右边不足用空格补齐）

#### 信号声明四级对齐（模块端口声明）

```verilog
// 四级对齐：信号类型 + 位宽 + 信号名 + 注释
input  wire                          clk      ,//I1,
input  wire                          rst_n    ,//I1,
input  wire                          wr_en    ,//I1,写使能
input  wire [$clog2(LINE_WIDTH)-1:0] wr_addr  ,//Ix,写地址
input  wire [DATA_WIDTH-1:0]         wr_data  ,//Ix,写数据
output reg                           rd_valid ,//O1,读数据有效
```

**位宽标记**：`I{n}`/`O{n}` 表示固定位宽，`Ix`/`Ox` 表示参数化位宽

**对齐规则详解：**
1. 信号名后的逗号对齐（通过空格调节）
2. `//` 紧跟逗号，无空格
3. 最后一个信号没有逗号，用**空格占位**使 `//` 对齐

```verilog
output reg  [DATA_WIDTH-1:0]         rd_data_0 ,//Ox,当前行读数据
output reg  [DATA_WIDTH-1:0]         rd_data_1 ,//Ox,前一行读数据
output reg                           rd_valid   //O1,读数据有效
//                                    ^空格占位，使//对齐上面的,//
```

### 3.2 文件头模板

```verilog
//============================================================================
// 模块名称    : image_scaler
// 功能描述    : 图像双线性插值缩放模块
// 作者        : 
// 版本历史    : v1.0 - 初始版本
// 对应Python  : python/image_scaler.py
// 验证状态    : ⏳ 验证中
//============================================================================

`timescale 1ns / 1ps

module image_scaler #(
    parameter DATA_WIDTH = 8,
    parameter IMG_WIDTH  = 1920
)(
    input  wire                          clk     ,//I1,
    input  wire                          rst_n   ,//I1,
    input  wire                          i_valid ,//I1,
    input  wire [DATA_WIDTH-1:0]         i_data  ,//Ix,
    output reg                           o_valid ,//O1,
    output reg  [DATA_WIDTH-1:0]         o_data  ,//Ox,
    input  wire                          o_ready  //I1,
);
    // 仿真延时参数（放在 module 内最前面）
    localparam U_DLY = 1;
    
    // 模块实现...
endmodule
```

### 3.3 代码结构规范

```verilog
module example (
    // 端口声明
);
    //------------------------------------------------------------------------
    // 仿真延时参数
    //------------------------------------------------------------------------
    localparam U_DLY = 1;

    //------------------------------------------------------------------------
    // 参数定义
    //------------------------------------------------------------------------
    localparam IDLE = 3'b000;

    //------------------------------------------------------------------------
    // 信号声明（按功能分组）
    //------------------------------------------------------------------------
    // 控制信号
    reg [2:0] state       ;
    reg       frame_start ;

    // 数据路径
    reg [7:0]  pixel_reg[0:3]  ;
    reg [15:0] accumulator     ;

    //------------------------------------------------------------------------
    // 组合逻辑
    //------------------------------------------------------------------------
    always @(*) begin
        case (state)
            IDLE: next_state = start ? READ : IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // 时序逻辑
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin              // 条件判断必须用 ==，禁止用 !
            state <= IDLE;                     // 异步复位不加 U_DLY
        end
        else begin                            // else 必须换行，不跟在 end 后
            state <= #U_DLY next_state;        // 同步逻辑加 U_DLY
        end
    end

    always @(posedge clk) begin
        if (pixel_valid == 1'b1) begin
            pixel_reg[0] <= #U_DLY pixel_reg[1];
        end
    end
endmodule
```

#### always 块编码规范

| 规则 | 正确 | 错误 |
|------|------|------|
| **条件判断** | `if (valid == 1'b1)` | `if (valid)` 或 `if (!valid)` |
| 复位判断 | `if (rst_n == 1'b0)` | `if (!rst_n)` |
| else 换行 | `end`<br>`else begin` | `end else begin` |
| U_DLY 使用 | 同步逻辑 `<= #U_DLY`，异步复位 `<=` | 异步复位加 U_DLY |

**⚠️ 条件判断强制规范（重要）**：
- 所有条件必须显式比较 `== 1'b0` 或 `== 1'b1`
- **禁止**隐式布尔转换：`if (valid)` → `if (valid == 1'b1)`
- **禁止**逻辑非：`if (!valid)` → `if (valid == 1'b0)`
- 组合逻辑中的条件同样遵循此规则：`if (state == IDLE && valid == 1'b1)`

### 3.4 视频接口规范

#### AXI-Stream 风格接口

```verilog
// 输入视频流
input  wire                  s_axis_tvalid ,//I1,
input  wire [DATA_WIDTH-1:0] s_axis_tdata  ,//Ix,
input  wire                  s_axis_tlast  ,//I1,行结束
input  wire                  s_axis_tuser  ,//I1,帧开始
output wire                  s_axis_tready ,//O1,

// 输出视频流
output reg                   m_axis_tvalid ,//O1,
output reg  [DATA_WIDTH-1:0] m_axis_tdata  ,//Ox,
output reg                   m_axis_tlast  ,//O1,
output reg                   m_axis_tuser  ,//O1,
input  wire                  m_axis_tready ,//I1,
```

#### 行缓冲模块示例

```verilog
module line_buffer #(
    parameter DATA_WIDTH = 8,
    parameter LINE_WIDTH = 1920
)(
    input  wire                          clk     ,//I1,
    input  wire                          rst_n   ,//I1,
    input  wire                          wr_en   ,//I1,
    input  wire [DATA_WIDTH-1:0]         wr_data ,//Ix,
    input  wire                          rd_en   ,//I1,
    output reg  [DATA_WIDTH-1:0]         rd_data ,//Ox,
    output reg                           rd_valid //O1,
);
    localparam U_DLY = 1;
    
    reg [DATA_WIDTH-1:0]         mem [0:LINE_WIDTH-1]  ;
    reg [$clog2(LINE_WIDTH)-1:0] wr_addr               ;
    
    always @(posedge clk) begin
        if (wr_en == 1'b1)
            mem[wr_addr] <= #U_DLY wr_data;
    end
    
    always @(posedge clk) begin
        rd_data <= #U_DLY mem[rd_addr];
    end
endmodule
```

### 3.5 定点数运算规范

```verilog
// Qm.n 格式：m位整数，n位小数
localparam INT_BITS   = 8                    ;
localparam FRAC_BITS  = 8                    ;
localparam TOTAL_BITS = INT_BITS + FRAC_BITS ;

// 定点数乘法 (结果右移 FRAC_BITS 位)
reg [TOTAL_BITS*2-1:0] mult_temp  ;
reg [TOTAL_BITS-1:0]   mult_result;

always @(posedge clk) begin
    mult_temp   <= #U_DLY coeff * pixel_value;
    mult_result <= #U_DLY mult_temp >> FRAC_BITS;
end

// 溢出处理
reg [TOTAL_BITS:0] add_temp ;
always @(*) begin
    add_temp = operand_a + operand_b;
    if (add_temp[TOTAL_BITS])
        add_result = {TOTAL_BITS{1'b1}};  // 饱和
    else
        add_result = add_temp[TOTAL_BITS-1:0];
end
```

---

## 4. 项目目录结构

```
project/
├── docs/                       # 文档
├── python/                     # Python 算法验证
│   ├── reference_models/       # 参考模型
│   ├── testbenches/            # 测试脚本
│   ├── utils/                  # 工具函数
│   └── results/                # 验证结果
├── rtl/                        # Verilog RTL 源码
│   ├── common/                 # 通用模块
│   ├── image_scaler/           # 图像缩放模块
│   └── top/                    # 顶层模块
├── sim/                        # 仿真相关
├── constraints/                # 约束文件
└── scripts/                    # 脚本工具
```

---

## 5. 验证工作流程

```
1. Python 实现算法原型（浮点黄金参考）
2. 定点化分析 → Python 定点化版本
3. 生成测试向量（hex/bin）和期望输出（golden）
4. Verilog RTL 实现 + Testbench
5. 协同仿真 → Verilog 输出保存到文件
6. Python 对比结果 → 生成差异报告
```

---

## 6. AI 助手工作指南

### 编写 Python 代码时
- [ ] 使用 **Tab 缩进**，**等号对齐**
- [ ] 视频图像处理使用 `(H, W, C)` 维度顺序
- [ ] 生成测试向量保存到 `results/test_vectors/`

### 编写 Verilog 代码时
- [ ] 使用 **snake_case 命名**，**四级对齐**端口声明
- [ ] 端口信号添加 `//I1,` / `//Ox,` 位宽注释
- [ ] 包含标准文件头（模块描述、Python对应关系、验证状态）
- [ ] 条件判断用 `== 1'b0` / `== 1'b1`（**禁止** `!`，**禁止**隐式 `if (valid)`）
- [ ] `else` **换行**，不跟在 `end` 后面
- [ ] 同步逻辑加 `#U_DLY`，异步复位不加
- [ ] 时钟用 `clk`，低电平复位用 `rst_n`

### 算法转 RTL 时
- [ ] 询问定点数格式 (Qm.n)、行缓冲深度、处理延迟
- [ ] 生成 Python 参考模型和测试向量生成脚本
