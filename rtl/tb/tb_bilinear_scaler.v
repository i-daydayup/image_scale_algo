//============================================================================
// 测试平台    : tb_bilinear_scaler
// 功能描述    : bilinear_scaler 模块的仿真测试平台
//
// 测试内容：
//   1. 基本功能测试 (100x100 -> 200x200 放大2倍)
//   2. 边界条件测试
//   3. 与 Python 参考模型结果对比
//
// 使用方法：
//   1. 使用 Python 生成测试向量和 golden 结果
//   2. 运行仿真：vsim -c -do "run -all; quit" tb_bilinear_scaler
//   3. 对比输出结果
//============================================================================

`timescale 1ns / 1ps

module tb_bilinear_scaler;

	//------------------------------------------------------------------------
	// 参数定义
	//------------------------------------------------------------------------
	parameter DATA_WIDTH  = 8;
	parameter INT_BITS    = 12;
	parameter FRAC_BITS   = 8;
	parameter WEIGHT_BITS = 8;
	parameter COORD_BITS  = INT_BITS + FRAC_BITS;
	parameter CLK_PERIOD  = 10;  // 100MHz

	// 测试图像参数
	parameter SRC_WIDTH    = 100;
	parameter SRC_HEIGHT   = 100;
	parameter DST_WIDTH    = 200;
	parameter DST_HEIGHT   = 200;
	parameter SCALE_FACTOR = 2.0;

	// 计算逆缩放比例定点值
	parameter INV_SCALE_X = (SRC_WIDTH * (1 << FRAC_BITS)) / DST_WIDTH;
	parameter INV_SCALE_Y = (SRC_HEIGHT * (1 << FRAC_BITS)) / DST_HEIGHT;

	//------------------------------------------------------------------------
	// 信号声明
	//------------------------------------------------------------------------
	reg                  clk;
	reg                  rst_n;

	// 配置接口
	reg  [COORD_BITS-1:0] cfg_inv_scale_x;
	reg  [COORD_BITS-1:0] cfg_inv_scale_y;
	reg  [15:0]           cfg_dst_width;
	reg  [15:0]           cfg_dst_height;
	reg  [15:0]           cfg_src_width;
	reg  [15:0]           cfg_src_height;

	// 输入接口
	reg                   i_valid;
	reg  [DATA_WIDTH-1:0] i_data;
	reg                   i_last;
	reg                   i_frame_start;
	wire                  i_ready;

	// 输出接口
	wire                  o_valid;
	wire [DATA_WIDTH-1:0] o_data;
	wire                  o_last;
	wire                  o_frame_start;
	reg                   o_ready;

	//------------------------------------------------------------------------
	// DUT 实例化
	//------------------------------------------------------------------------
	bilinear_scaler #(
		.DATA_WIDTH  (DATA_WIDTH  ),//8,
		.INT_BITS    (INT_BITS    ),//12,
		.FRAC_BITS   (FRAC_BITS   ),//8,
		.WEIGHT_BITS (WEIGHT_BITS ),//8,
		.MAX_WIDTH   (1024        ),//10,
		.MAX_HEIGHT  (1024        ) //10
	) dut (
		.clk             (clk              ),//I1,
		.rst_n           (rst_n            ),//I1,
		.cfg_inv_scale_x (cfg_inv_scale_x  ),//I20,
		.cfg_inv_scale_y (cfg_inv_scale_y  ),//I20,
		.cfg_dst_width   (cfg_dst_width    ),//I16,
		.cfg_dst_height  (cfg_dst_height   ),//I16,
		.cfg_src_width   (cfg_src_width    ),//I16,
		.cfg_src_height  (cfg_src_height   ),//I16,
		.i_valid         (i_valid          ),//I1,
		.i_data          (i_data           ),//I8,
		.i_last          (i_last           ),//I1,
		.i_frame_start   (i_frame_start    ),//I1,
		.i_ready         (i_ready          ),//O1,
		.o_valid         (o_valid          ),//O1,
		.o_data          (o_data           ),//O8,
		.o_last          (o_last           ),//O1,
		.o_frame_start   (o_frame_start    ),//O1,
		.o_ready         (o_ready          ) //I1
	);

	//------------------------------------------------------------------------
	// 时钟生成
	//------------------------------------------------------------------------
	initial begin
		clk = 0;
		forever #(CLK_PERIOD/2) clk = ~clk;
	end

	//------------------------------------------------------------------------
	// 测试激励
	//------------------------------------------------------------------------
	integer row, col;
	integer out_file;
	integer pixel_count;

	initial begin
		// 初始化
		rst_n           = 0;
		i_valid         = 0;
		i_data          = 0;
		i_last          = 0;
		i_frame_start   = 0;
		o_ready         = 1;

		cfg_inv_scale_x = INV_SCALE_X;
		cfg_inv_scale_y = INV_SCALE_Y;
		cfg_dst_width   = DST_WIDTH;
		cfg_dst_height  = DST_HEIGHT;
		cfg_src_width   = SRC_WIDTH;
		cfg_src_height  = SRC_HEIGHT;

		// 复位
		#(CLK_PERIOD * 5);
		rst_n = 1;
		#(CLK_PERIOD * 2);

		$display("============================================");
		$display("双线性插值缩放器测试开始");
		$display("源图尺寸: %0dx%0d", SRC_WIDTH, SRC_HEIGHT);
		$display("目标尺寸: %0dx%0d", DST_WIDTH, DST_HEIGHT);
		$display("逆缩放比例定点值: X=%0d, Y=%0d", INV_SCALE_X, INV_SCALE_Y);
		$display("============================================");

		// 打开输出文件
		out_file = $fopen("output_verilog.hex", "w");

		// 发送帧开始
		@(posedge clk);
		i_frame_start = 1;
		i_valid       = 1;
		i_data        = 8'h00;

		// 发送测试图像数据 (水平渐变)
		for (row = 0; row < SRC_HEIGHT; row = row + 1) begin
			for (col = 0; col < SRC_WIDTH; col = col + 1) begin
				@(posedge clk);
				i_frame_start = 0;
				i_data        = col;  // 水平渐变：0 ~ 99
				i_last        = (col == SRC_WIDTH - 1);

				// 每10行显示一次进度
				if (col == 0 && row % 10 == 0)
					$display("发送行 %0d / %0d", row, SRC_HEIGHT);
			end
		end

		@(posedge clk);
		i_valid = 0;
		i_last  = 0;

		$display("输入数据发送完成，等待输出...");

		// 等待输出完成
		pixel_count = 0;
		repeat (10000) begin
			@(posedge clk);
			if (o_valid) begin
				$fwrite(out_file, "%02x\n", o_data);
				pixel_count = pixel_count + 1;

				if (pixel_count % 1000 == 0)
					$display("输出像素数: %0d / %0d", pixel_count, DST_WIDTH * DST_HEIGHT);

				if (pixel_count >= DST_WIDTH * DST_HEIGHT)
					disable wait_output;
			end
		end
		wait_output:;

		$fclose(out_file);

		$display("============================================");
		$display("测试完成");
		$display("输出像素数: %0d", pixel_count);
		$display("输出文件: output_verilog.hex");
		$display("============================================");

		#(CLK_PERIOD * 10);
		$finish;
	end

	//------------------------------------------------------------------------
	// 波形转储
	//------------------------------------------------------------------------
	initial begin
		$dumpfile("tb_bilinear_scaler.vcd");
		$dumpvars(0, tb_bilinear_scaler);
	end

	//------------------------------------------------------------------------
	// 超时检测
	//------------------------------------------------------------------------
	initial begin
		#(CLK_PERIOD * 50000);
		$display("错误：仿真超时！");
		$finish;
	end

endmodule
