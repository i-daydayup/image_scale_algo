# #############################################################################
# ModelSim/QuestaSim 仿真脚本
# 使用方式：在QuestaSim中执行 do ut_bilinear_scaler.do
#############################################################################

# 设置UTF-8编码，解决中文乱码
encoding system utf-8

# 清理并创建工作库
if {[file exists work]} {
   file delete -force work
}
vlib work

# 项目路径（根据实际情况修改）
set P_DIR "D:/Projects/image_scale_algo/rtl"

# 编译RTL模块
# bilinear_scaler_vh.v 例化的子模块需要单独编译
vlog -sv	$P_DIR/src/bilinear_scaler_vh.v			\
			$P_DIR/src/line_buffer_wrapper.v		\
			$P_DIR/src/bilinear_coord_calc.v		\
			$P_DIR/sim/ut_bilinear_scaler/ut_bilinear_scaler.v

# 加载设计
# 选项说明：
#   -t ps          : 时间精度1ps
#   -novopt        : 不进行优化（便于调试，正式仿真可去掉）
#   +notimingchecks: 关闭时序检查
#   -pli novas.dll : Verdi调试支持（如无Verdi请注释掉）

# 加载设计（支持Debussy/Verdi波形调试）
vsim -t ps -novopt +notimingchecks -pli novas.dll work.ut_bilinear_scaler

# 添加波形（可选）
# add wave -position insertpoint sim:/ut_bilinear_scaler/*

# 运行仿真
run 2ms

# 如有Verdi，打开波形
# verdi -dbdir work -ssf ut_bilinear_scaler.fsdb &
