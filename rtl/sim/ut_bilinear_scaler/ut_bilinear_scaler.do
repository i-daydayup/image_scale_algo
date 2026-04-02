# #############################################################################
	# 设置 ModelSim/QuestaSim 使用UTF-8编码，解决打印信息是中文乱码的问题
	encoding system utf-8

	if {[file exists work]} {
	   file delete -force work
	}
	vlib work

	set P_DIR "D:/Projects/image_scale_algo/rtl"

	#Compile all rtl modules#
	vlog -sv 	$P_DIR/src/bilinear_scaler.v				\
				$P_DIR/src/bilinear_interp.v				\
				$P_DIR/src/bilinear_coord_calc.v			\
				$P_DIR/src/line_buffer.v					\
				$P_DIR/sim/ut_bilinear_scaler/ut_bilinear_scaler.v

	#Load the design. Use required libraries.#
	vsim -t ps -novopt +notimingchecks \
		work.ut_bilinear_scaler -pli novas.dll

	run 5ms
