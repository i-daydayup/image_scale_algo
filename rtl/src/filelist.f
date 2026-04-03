// 双线性插值缩放器 RTL 文件列表（V+H架构）
// 使用方式：
//   1. 在仿真工具中添加此文件列表
//   2. 或使用命令：vlog -f filelist.f

// 通用模块
rtl/src/line_buffer_2row_wrapper.v     // L1输入缓冲：2行ping-pong，时序收敛

// V+H架构新模块
rtl/src/bilinear_scaler_vh.v           // V+H架构顶层模块（开发中）

// 以下模块为旧版架构，逐步迁移或替换
// rtl/src/bilinear_coord_calc.v        // 坐标计算模块（可复用）
// rtl/src/bilinear_interp.v            // 插值计算模块（可复用）
// rtl/src/line_buffer.v                // 旧版行缓冲（将被wrapper替代）
// rtl/src/bilinear_scaler.v            // 旧版顶层模块（将被vh替代）
