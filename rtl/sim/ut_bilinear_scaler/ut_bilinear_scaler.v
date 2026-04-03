//============================================================================
// 测试平台    : ut_bilinear_scaler
// 功能描述    : bilinear_scaler 模块的仿真测试平台
//
// 测试内容：
//   1. 基本功能测试 (100x100 -> 200x200 放大2倍)
//   2. 边界条件测试
//   3. 与 Python 参考模型结果对比
//
// 使用方法：
//   1. 使用 Python 生成测试向量和 golden 结果
//   2. 运行仿真：vsim -c -do "run -all; quit" ut_bilinear_scaler
//   3. 对比输出结果
//============================================================================

`timescale 1ns / 1ps

module ut_bilinear_scaler;

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 参数定义
	//------------------------------------------------------------------------
	parameter DATA_WIDTH  = 8                   ;
	parameter INT_BITS    = 12                  ;
	parameter FRAC_BITS   = 8                   ;
	parameter WEIGHT_BITS = 8                   ;
	parameter COORD_BITS  = INT_BITS + FRAC_BITS;
	parameter CLK_PERIOD  = 10                  ;// 100MHz

	// 测试图像参数
	parameter SRC_WIDTH    = 100 ;
	parameter SRC_HEIGHT   = 10 ;
	parameter DST_WIDTH    = 200 ;
	parameter DST_HEIGHT   = 20 ;
	parameter SCALE_FACTOR = 2.0 ;

	// 计算逆缩放比例定点值
	parameter INV_SCALE_X = (SRC_WIDTH * (1 << FRAC_BITS)) / DST_WIDTH ;
	parameter INV_SCALE_Y = (SRC_HEIGHT * (1 << FRAC_BITS)) / DST_HEIGHT;

	//------------------------------------------------------------------------
	// 信号声明
	//------------------------------------------------------------------------
	reg                                clk             ;
	reg                                rst_n           ;

	// 配置接口
	reg  [COORD_BITS-1:0]              cfg_inv_scale_x ;
	reg  [COORD_BITS-1:0]              cfg_inv_scale_y ;
	reg  [15:0]                        cfg_dst_width   ;
	reg  [15:0]                        cfg_dst_height  ;
	reg  [15:0]                        cfg_src_width   ;
	reg  [15:0]                        cfg_src_height  ;

	// 输入接口
	reg                                i_valid         ;
	reg  [DATA_WIDTH-1:0]              i_data          ;
	reg                                i_last          ;
	reg                                i_frame_start   ;
	wire                               i_ready         ;

	// 输出接口
	wire                               o_valid         ;
	wire [DATA_WIDTH-1:0]              o_data          ;
	wire                               o_last          ;
	wire                               o_frame_start   ;
	reg                                o_ready         ;

	//------------------------------------------------------------------------
	// DUT 实例化
	//------------------------------------------------------------------------
//	bilinear_scaler #(
//		.DATA_WIDTH  (DATA_WIDTH  ),//8,
//		.INT_BITS    (INT_BITS    ),//12,
//		.FRAC_BITS   (FRAC_BITS   ),//8,
//		.WEIGHT_BITS (WEIGHT_BITS ),//8,
//		.MAX_WIDTH   (1024        ),//10,
//		.MAX_HEIGHT  (1024        ) //10
//	) dut (
//		.clk             (clk              ),//I1,
//		.rst_n           (rst_n            ),//I1,
//		.cfg_inv_scale_x (cfg_inv_scale_x  ),//I20,
//		.cfg_inv_scale_y (cfg_inv_scale_y  ),//I20,
//		.cfg_dst_width   (cfg_dst_width    ),//I16,
//		.cfg_dst_height  (cfg_dst_height   ),//I16,
//		.cfg_src_width   (cfg_src_width    ),//I16,
//		.cfg_src_height  (cfg_src_height   ),//I16,
//		.i_valid         (i_valid          ),//I1,
//		.i_data          (i_data           ),//I8,
//		.i_last          (i_last           ),//I1, 注意，仅1拍有效
//		.i_frame_start   (i_frame_start    ),//I1, 注意，仅1拍有效
//		.i_ready         (i_ready          ),//O1,
//		.o_valid         (o_valid          ),//O1,
//		.o_data          (o_data           ),//O8,
//		.o_last          (o_last           ),//O1,
//		.o_frame_start   (o_frame_start    ),//O1,
//		.o_ready         (o_ready          ) //I1
//	);


	bilinear_scaler_vh #(
		.DATA_WIDTH  (DATA_WIDTH  ),//8,
		.INT_BITS    (INT_BITS    ),//12,
		.FRAC_BITS   (FRAC_BITS   ),//8,
		// .WEIGHT_BITS (WEIGHT_BITS ),//8,
		.MAX_WIDTH   (1024        ),//10,
		.MAX_HEIGHT  (1024        ),//10
		.ADDR_WIDTH  ($clog2(1024))
	) dut (
		//------------------------------------------------------------------------
		// 输入时钟域 (源图像/DDR)
		//------------------------------------------------------------------------
		.clk_in			(clk              ),//I1,输入时钟
		.rst_n_in 		(rst_n            ),//I1,输入复位
		//------------------------------------------------------------------------
		// 配置接口 (clk_in域，在i_frame_start前配置有效)
		//------------------------------------------------------------------------
		.cfg_inv_scale_x (cfg_inv_scale_x  ),//I20,X方向逆缩放比例
		.cfg_inv_scale_y (cfg_inv_scale_y  ),//I20,Y方向逆缩放比例
		.cfg_dst_width   (cfg_dst_width    ),//I16,目标图像宽度
		.cfg_dst_height  (cfg_dst_height   ),//I16,目标图像高度
		.cfg_src_width   (cfg_src_width    ),//I16,源图像宽度
		.cfg_src_height  (cfg_src_height   ),//I16,源图像高度
		//------------------------------------------------------------------------
		// 输入视频流 (AXI-S风格, clk_in域)
		//------------------------------------------------------------------------
		.i_valid         (i_valid          	),//I1,输入数据有效
		.i_data          (i_data           	),//I8,输入像素数据
		.i_last          (i_last           	),//I1, 注意，仅1拍有效, 行结束标记
		.i_frame_start   (i_frame_start    	),//I1, 注意，仅1拍有效, 帧开始标记
		.i_ready         (i_ready          	),//O1, 模块就绪
		//------------------------------------------------------------------------
		// 输出时钟域 (显示接口)
		//------------------------------------------------------------------------
		.clk_out       	(clk				),//I1,输出时钟
		.rst_n_out     	(rst_n				),//I1,输出复位
		//------------------------------------------------------------------------
		// 输出视频流 (AXI-S风格, clk_out域)
		//------------------------------------------------------------------------
		.o_valid         (o_valid          ),//O1,输出数据有效
		.o_data          (o_data           ),//O8,输出像素数据
		.o_last          (o_last           ),//O1,行结束标记
		.o_frame_start   (o_frame_start    ),//O1,帧开始标记
		.o_ready         (o_ready          ) //I1 下游就绪
	);

	//------------------------------------------------------------------------
	// 时钟生成
	//------------------------------------------------------------------------
	initial begin
		clk = 0;
		// forever #(CLK_PERIOD/2) clk = ~clk;
		forever begin
			#(CLK_PERIOD/2)               clk = 1'b1;
			#(CLK_PERIOD - CLK_PERIOD/2)  clk = 1'b0;
		end
	end

	//------------------------------------------------------------------------
	// 测试激励
	//------------------------------------------------------------------------
	integer row, col   	;
	integer out_file   	;
	integer pixel_count	;
	integer frame_cnt	;

	localparam EXPECTED_PIXELS = DST_WIDTH * DST_HEIGHT;
	localparam OUTPUT_TIMEOUT  = EXPECTED_PIXELS * 1 + 1000;  // 1倍余量

	initial begin
		// 初始化信号
		frame_cnt		= 0;
		rst_n           = 1'b0;
		i_valid         = 1'b0;
		i_data          = 8'h00;
		i_last          = 1'b0;
		i_frame_start   = 1'b0;
		o_ready         = 1'b1;

		// 配置参数
		cfg_inv_scale_x = INV_SCALE_X ;
		cfg_inv_scale_y = INV_SCALE_Y ;
		cfg_dst_width   = DST_WIDTH   ;
		cfg_dst_height  = DST_HEIGHT  ;
		cfg_src_width   = SRC_WIDTH   ;
		cfg_src_height  = SRC_HEIGHT  ;

		// 复位
		#(CLK_PERIOD * 5);
		rst_n = 1'b1;
		#(CLK_PERIOD * 2);

		$display("============================================");
		$display("双线性插值缩放器测试开始");
		$display("源图尺寸: %0dx%0d", SRC_WIDTH, SRC_HEIGHT);
		$display("目标尺寸: %0dx%0d", DST_WIDTH, DST_HEIGHT);
		$display("逆缩放比例定点值: X=%0d, Y=%0d", INV_SCALE_X, INV_SCALE_Y);
		$display("============================================");

		// 打开输出文件
		out_file = $fopen("output_verilog.hex", "w");
	end

	always begin: send_frame_block
		// 等待复位完成
		@(posedge clk) #U_DLY;
		while (rst_n == 1'b0) @(posedge clk) #U_DLY;

		// 发送帧开始 (AXI-S握手: valid拉高后等待i_ready)
		i_frame_start = 1'b1;
		i_valid       = 1'b1;
		i_data        = 8'hf5;
		// 等待握手完成 (valid && ready)
		@(posedge clk) #U_DLY;
		while (i_ready == 1'b0) @(posedge clk) #U_DLY;

		// 发送测试图像数据 (水平渐变)
		// 第0行：帧开始已发8'hf5(第0像素)，这里从第1个像素开始
		// 第1~N-1行：从第0个像素开始发送
		for (row = 0; row < SRC_HEIGHT; row = row + 1) begin
			for (col = (row == 0) ? 1 : 0; col < SRC_WIDTH; col = col + 1) begin
				i_frame_start = 1'b0;
				i_valid       = 1'b1;
				i_data        = col;  // 水平渐变
				i_last        = (col == SRC_WIDTH - 1);
				// AXI-S握手：等待slave准备好 (valid && ready)
				@(posedge clk) #U_DLY;
				while (i_ready == 1'b0) @(posedge clk) #U_DLY;
			end
			// 行间隙：释放valid
			i_valid = 1'b0;
			i_last  = 1'b0;
			repeat(50) @(posedge clk) #U_DLY;
		end

		// 帧结束
		i_valid       = 1'b0;
		i_last        = 1'b0;
		i_frame_start = 1'b0;
		repeat(200) @(posedge clk) #U_DLY;

		// 一帧发送结束，循环发送下一帧
		frame_cnt = frame_cnt + 1'b1;
	end

		// $display("输入数据发送完成，等待输出...");

		// //------------------------------------------------------------------------
		// // 等待输出完成
		// //------------------------------------------------------------------------
		// pixel_count = 0;
		// repeat (10000) begin : wait_output_block
		// 	@(posedge clk);
		// 	if (o_valid == 1'b1) begin
		// 		$fwrite(out_file, "%02x\n", o_data);
		// 		pixel_count = pixel_count + 1;

		// 		if (pixel_count % 1000 == 0)
		// 			$display("输出像素数: %0d / %0d", pixel_count, DST_WIDTH * DST_HEIGHT);

		// 		if (pixel_count >= OUTPUT_TIMEOUT)
		// 			disable wait_output_block;
		// 	end
		// end

		// $fclose(out_file);

		// $display("============================================");
		// $display("测试完成");
		// $display("输出像素数: %0d", pixel_count);
		// $display("输出文件: output_verilog.hex");
		// $display("============================================");

		// #(CLK_PERIOD * 10);
		// // $finish;
	// end

	//------------------------------------------------------------------------
	// 波形转储
	//------------------------------------------------------------------------
	// initial begin
	// 	$dumpfile("tb_bilinear_scaler.vcd");
	// 	$dumpvars(0, tb_bilinear_scaler);
	// end
	//-----save the wave to debussy.
	initial begin
		$fsdbAutoSwitchDumpfile(1800,"ut_bilinear_scaler.fsdb",5); //file size max is 1.8Gb, file number max is 5.
		$fsdbDumpvars;
		# 50_000_000; //50ms, because 1ns/1ps
		$fsdbDumpflush;
	end

	// //------------------------------------------------------------------------
	// // 超时检测
	// //------------------------------------------------------------------------
	// initial begin
	// 	#(CLK_PERIOD * 50000);
	// 	$display("错误：仿真超时！");
	// 	// $finish;
	// end

endmodule
