# 20260401三
完成双线性verilog代码，待仿真

# 20260331二
完成最近邻、双线性、双立方、Lanczos的浮点代码验证

# 20260328六

视频图像缩放算法研究分析


D:\Projects\
└── image_scale_algo\              # 项目根目录（整个仓库）
    ├── python\                    # Python 算法验证
    │   ├── algorithms\            # 你的算法实现
    │   │   ├── __init__.py
    │   │   ├── bilinear.py        # 双线性插值
    │   │   ├── bicubic.py         # 双立方插值
    │   │   └── nearest.py         # 最近邻（对比用）
    │   ├── testbench\             # 测试框架
    │   │   ├── test_resize.py     # 主测试脚本
    │   │   └── utils.py           # PSNR 计算、图像对比
    │   ├── reference\             # OpenCV/PIL 参考实现
    │   │   └── opencv_impl.py
    │   └── requirements.txt       # pip install -r requirements.txt
    │
    ├── rtl\                       # Verilog 实现（以后放这里）
    │   ├── src\                   # RTL 源码
    │   ├── sim\                   # 仿真脚本
    │   └── tb\                    # Testbench
    │
    ├── test_images\               # 测试素材
    │   ├── 1080p\                 # 标准测试图
    │   ├── 720p\
    │   └── patterns\              # 纯色/条纹等测试图
    │
    ├── docs\                      # 文档
    │   ├── algorithm_notes.md     # 算法推导笔记
    │   └── rtl_plan.md            # RTL 实现计划
    │
    ├── results\                   # 输出结果（Git 忽略）
    │   └── .gitignore             # *.png *.jpg
    │
    └── .gitignore                 # 仓库根忽略文件