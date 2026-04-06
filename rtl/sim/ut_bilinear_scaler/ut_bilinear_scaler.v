//============================================================================
// 测试平台    : ut_bilinear_scaler
// 功能描述    : bilinear_scaler_vh 模块的仿真测试平台
//
// 架构说明：
//   - 本Testbench模拟Scaler模块在完整视频系统中的工作场景
//   - 输出端：模拟显示器的刚性时序 (vsync + de)
//   - 输入端：模拟DDR控制器响应Scaler的读取请求
//   - 信号关联：
//     * display_vs (clk_out) ──同步──► i_frame_start (clk_in)
//     * display_de (clk_out) ──直接──► o_ready
//     * i_ready    (clk_in)  ◄──响应── 数据发送逻辑
//
// 测试内容：
//   1. 基本功能测试 (100x10 -> 200x20 放大2倍)
//   2. 显示时序与Scaler输出的对齐验证
//   3. 与 Python 参考模型结果对比
//
// 使用方法：
//   1. 运行仿真：vsim -c -do "run -all; quit" ut_bilinear_scaler
//   2. 对比输出结果与Python golden
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
	parameter DATA_WIDTH  		= 8                   ;
	parameter INT_BITS    		= 12                  ;
	parameter FRAC_BITS   		= 8                   ;
	parameter COORD_BITS  		= INT_BITS + FRAC_BITS;
	parameter CLK_PERIOD_IN 	= 16                  ;// 62.5MHz
	parameter CLK_PERIOD_OUT 	= 10                  ;// 100MHz

	// 测试图像参数
	parameter SRC_WIDTH    = 100 ;
	parameter SRC_HEIGHT   = 10  ;
	parameter DST_WIDTH    = 200 ;
	parameter DST_HEIGHT   = 20  ;

	// 计算逆缩放比例定点值
	parameter INV_SCALE_X = (SRC_WIDTH * (1 << FRAC_BITS)) / DST_WIDTH ;
	parameter INV_SCALE_Y = (SRC_HEIGHT * (1 << FRAC_BITS)) / DST_HEIGHT;

	//------------------------------------------------------------------------
	// 信号声明
	//------------------------------------------------------------------------
	reg                                clk_in          ;// Scaler输入时钟
	reg                                clk_out         ;// Scaler输出/显示时钟
	reg                                rst_n           ;

	// 配置接口
	reg  [COORD_BITS-1:0]              cfg_inv_scale_x ;
	reg  [COORD_BITS-1:0]              cfg_inv_scale_y ;
	reg  [15:0]                        cfg_dst_width   ;
	reg  [15:0]                        cfg_dst_height  ;
	reg  [15:0]                        cfg_src_width   ;
	reg  [15:0]                        cfg_src_height  ;

	// 输入接口 (AXI-S, clk_in域)
	reg                                i_valid         ;
	reg  [DATA_WIDTH-1:0]              i_data          ;
	reg                                i_last          ;
	reg                                i_frame_start   ;
	wire                               i_ready         ;

	// 输出接口 (AXI-S, clk_out域)
	wire                               o_valid         ;
	wire [DATA_WIDTH-1:0]              o_data          ;
	wire                               o_last          ;
	wire                               o_frame_start   ;
	reg                                o_ready         ;

	//------------------------------------------------------------------------
	// 显示时序信号 (clk_out域，模拟显示器)
	//------------------------------------------------------------------------
	reg                                display_vs      ;// 垂直同步(帧开始)
	reg                                display_de      ;// 数据使能
	reg  [15:0]                        display_x       ;// 显示X坐标
	reg  [15:0]                        display_y       ;// 显示Y坐标

	// display_vs 同步到 clk_in 域 (用于生成 i_frame_start)
	reg                                display_vs_d1   ;// 打拍
	reg                                display_vs_d2   ;
	wire                               display_vs_pulse ;// 边沿检测

	//------------------------------------------------------------------------
	// DUT 实例化: bilinear_scaler_vh (V+H架构，双线性缩放)
	//------------------------------------------------------------------------
	bilinear_scaler_vh #(
		.DATA_WIDTH  (DATA_WIDTH  ),//8,
		.INT_BITS    (INT_BITS    ),//12,
		.FRAC_BITS   (FRAC_BITS   ),//8,
		.MAX_WIDTH   (1024        ),//10,
		.MAX_HEIGHT  (1024        ),//10
		.ADDR_WIDTH  ($clog2(1024)) //10
	) dut (
		//------------------------------------------------------------------------
		// 输入时钟域 (源图像/DDR)
		//------------------------------------------------------------------------
		.clk_in          (clk_in         ),//I1,输入时钟
		.rst_n_in        (rst_n          ),//I1,输入复位
		//------------------------------------------------------------------------
		// 配置接口 (clk_in域)
		//------------------------------------------------------------------------
		.cfg_inv_scale_x (cfg_inv_scale_x),//Ix,X逆缩放比例
		.cfg_inv_scale_y (cfg_inv_scale_y),//Ix,Y逆缩放比例
		.cfg_dst_width   (cfg_dst_width  ),//I16,目标宽度
		.cfg_dst_height  (cfg_dst_height ),//I16,目标高度
		.cfg_src_width   (cfg_src_width  ),//I16,源图宽度
		.cfg_src_height  (cfg_src_height ),//I16,源图高度
		//------------------------------------------------------------------------
		// 输入视频流 (AXI-S风格, clk_in域)
		//------------------------------------------------------------------------
		.i_valid         (i_valid        ),//I1,输入有效
		.i_data          (i_data         ),//Ix,输入数据
		.i_last          (i_last         ),//I1,行结束
		.i_frame_start   (i_frame_start  ),//I1,帧开始
		.i_ready         (i_ready        ),//O1,输入就绪
		//------------------------------------------------------------------------
		// 输出时钟域 (显示接口)
		//------------------------------------------------------------------------
		.clk_out         (clk_out        ),//I1,输出时钟
		.rst_n_out       (rst_n          ),//I1,输出复位
		//------------------------------------------------------------------------
		// 输出视频流 (AXI-S风格, clk_out域)
		//------------------------------------------------------------------------
		.o_valid         (o_valid        ),//O1,输出有效
		.o_data          (o_data         ),//Ox,输出数据
		.o_last          (o_last         ),//O1,行结束
		.o_frame_start   (o_frame_start  ),//O1,帧开始
		.o_ready         (o_ready        ) //I1,下游就绪
	);

	//------------------------------------------------------------------------
	// 时钟生成 (避免CLK_PERIOD/2的奇数除法问题)
	//------------------------------------------------------------------------
	initial begin
		clk_in = 0;
		forever begin
			#(CLK_PERIOD_IN/2)                	clk_in = 1'b1;
			#(CLK_PERIOD_IN - CLK_PERIOD_IN/2)  	clk_in = 1'b0;
		end
	end

	initial begin
		clk_out = 0;
		forever begin
			#(CLK_PERIOD_OUT/2)                	clk_out = 1'b1;
			#(CLK_PERIOD_OUT - CLK_PERIOD_OUT/2) 	clk_out = 1'b0;
		end
	end

	//------------------------------------------------------------------------
	// 初始化与复位
	//------------------------------------------------------------------------
	initial begin
		// 初始化所有信号
		rst_n           = 1'b0;
		i_valid         = 1'b0;
		i_data          = 8'h00;
		i_last          = 1'b0;
		i_frame_start   = 1'b0;
		o_ready         = 1'b0;

		display_vs      = 1'b0;
		display_de      = 1'b0;
		display_x       = 0;
		display_y       = 0;

		// 配置参数
		cfg_inv_scale_x = INV_SCALE_X ;
		cfg_inv_scale_y = INV_SCALE_Y ;
		cfg_dst_width   = DST_WIDTH   ;
		cfg_dst_height  = DST_HEIGHT  ;
		cfg_src_width   = SRC_WIDTH   ;
		cfg_src_height  = SRC_HEIGHT  ;

		// 复位
		repeat(10) @(posedge clk_in);
		rst_n = 1'b1;
		repeat(5) @(posedge clk_in);

		$display("============================================");
		$display("双线性插值缩放器 V+H 架构测试开始");
		$display("源图尺寸: %0dx%0d", SRC_WIDTH, SRC_HEIGHT);
		$display("目标尺寸: %0dx%0d", DST_WIDTH, DST_HEIGHT);
		$display("输入时钟: %0.1fMHz, 输出时钟: %0.1fMHz",
		         1000.0/CLK_PERIOD_IN, 1000.0/CLK_PERIOD_OUT);
		$display("============================================");
	end

	//------------------------------------------------------------------------
	// display_vs 跨时钟域同步 (clk_out -> clk_in)
	// 在display_vs上升沿产生1拍的i_frame_start脉冲
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			display_vs_d1 <= 1'b0;
			display_vs_d2 <= 1'b0;
		end
		else begin
			display_vs_d1 <= display_vs;
			display_vs_d2 <= display_vs_d1;
		end
	end

	assign display_vs_pulse = display_vs_d1 && !display_vs_d2;  // 上升沿

	// 生成 i_frame_start (clk_in域，仅1拍脉冲)
	always @(posedge clk_in) begin
		if (display_vs_pulse)
			i_frame_start <= #U_DLY 1'b1;
		else
			i_frame_start <= #U_DLY 1'b0;
	end

	//------------------------------------------------------------------------
	// 显示时序生成 (clk_out域)
	// 标准视频时序: H_SYNC + H_BACK + H_ACTIVE + H_FRONT
	//               V_SYNC + V_BACK + V_ACTIVE + V_FRONT
	//------------------------------------------------------------------------
	localparam H_SYNC    = 8;    // 行同步
	localparam H_BACK    = 12;   // 行后肩
	localparam H_ACTIVE  = DST_WIDTH;
	localparam H_FRONT   = 8;    // 行前肩
	localparam H_TOTAL   = H_SYNC + H_BACK + H_ACTIVE + H_FRONT;

	localparam V_SYNC    = 1;    // 场同步
	localparam V_BACK    = 4;    // 场后肩 (用于DDR预加载)
	localparam V_ACTIVE  = DST_HEIGHT;
	localparam V_FRONT   = 3;    // 场前肩
	localparam V_TOTAL   = V_SYNC + V_BACK + V_ACTIVE + V_FRONT;

	initial begin
		@(posedge rst_n);  // 等待复位完成
		repeat(10) @(posedge clk_out);

		forever begin
			// V_SYNC: 帧开始脉冲
			display_vs = 1'b1;
			repeat(H_TOTAL * V_SYNC) @(posedge clk_out);
			display_vs = 1'b0;

			// V_BACK: 消隐期 (DDR在此期间预加载数据到L1 buffer)
			repeat(H_TOTAL * V_BACK) @(posedge clk_out);

			// V_ACTIVE: 有效显示期间
			for (display_y = 0; display_y < DST_HEIGHT; display_y = display_y + 1) begin
				// H_BACK: 行消隐
				repeat(H_BACK) @(posedge clk_out);

				// H_ACTIVE: 有效像素，de和o_ready有效
				for (display_x = 0; display_x < DST_WIDTH; display_x = display_x + 1) begin
					display_de <= #U_DLY 1'b1;
					o_ready    <= #U_DLY 1'b1;
					@(posedge clk_out);
				end

				// 行结束
				display_de <= #U_DLY 1'b0;
				o_ready    <= #U_DLY 1'b0;

				// H_FRONT + H_SYNC
				repeat(H_FRONT + H_SYNC) @(posedge clk_out);
			end

			// V_FRONT: 场消隐
			repeat(H_TOTAL * V_FRONT) @(posedge clk_out);
		end
	end

	//------------------------------------------------------------------------
	// DDR数据发送模块 (clk_in域)
	// 模拟DDR控制器：响应i_frame_start和i_ready，发送图像数据
	//------------------------------------------------------------------------
	integer row, col;
	integer frame_cnt;

	initial begin
		frame_cnt = 0;
		@(posedge rst_n);

		forever begin
			// 等待i_frame_start (由display_vs同步过来)
			@(posedge clk_in);
			while (i_frame_start == 1'b0) @(posedge clk_in);

			$display("[Frame %0d] DDR开始发送数据 @%0t", frame_cnt, $time);
			repeat(100) @(posedge clk_in); //帧头到来后，从DDR读取数据，之间有延时，设置100个周期。

			// 发送一帧图像数据 (水平渐变)
			for (row = 0; row < SRC_HEIGHT; row = row + 1) begin
				for (col = 0; col < SRC_WIDTH; col = col + 1) begin
					// 等待i_ready (Scaler准备好接收)
					while (i_ready == 1'b0) @(posedge clk_in);

					// 发送像素
					i_valid <= #U_DLY 1'b1;
					i_data  <= #U_DLY col[7:0];  // 水平渐变
					i_last  <= #U_DLY (col == SRC_WIDTH - 1) ? 1'b1 : 1'b0;
					@(posedge clk_in);
				end
			end

			// 帧结束
			i_valid <= #U_DLY 1'b0;
			i_last  <= #U_DLY 1'b0;

			$display("[Frame %0d] DDR发送完成 @%0t", frame_cnt, $time);
			frame_cnt = frame_cnt + 1;
		end
	end

	//------------------------------------------------------------------------
	// 输出数据捕获与验证
	//------------------------------------------------------------------------
	integer out_file;
	integer pixel_cnt;

	initial begin
		out_file = $fopen("output_verilog.hex", "w");
		pixel_cnt = 0;
	end

	always @(posedge clk_out) begin
		if (o_valid == 1'b1 && o_ready == 1'b1) begin
			$fwrite(out_file, "%02x\n", o_data);
			pixel_cnt = pixel_cnt + 1;

			if (pixel_cnt % 1000 == 0)
				$display("输出像素数: %0d / %0d", pixel_cnt, DST_WIDTH * DST_HEIGHT);
		end
	end

	//------------------------------------------------------------------------
	// 时序对齐验证
	//------------------------------------------------------------------------
	// 检查o_frame_start是否与display_vs对齐（允许一定延迟）
	reg [15:0] vs_to_frame_start_cnt;
	always @(posedge clk_out) begin
		if (display_vs == 1'b1)
			vs_to_frame_start_cnt <= 0;
		else if (o_frame_start == 1'b1)
			$display("VSYNC到o_frame_start延迟: %0d clk_out cycles", vs_to_frame_start_cnt);
		else
			vs_to_frame_start_cnt <= vs_to_frame_start_cnt + 1;
	end

	//------------------------------------------------------------------------
	// 波形转储
	//------------------------------------------------------------------------
	initial begin
		$fsdbAutoSwitchDumpfile(1800,"ut_bilinear_scaler.fsdb",5);
		$fsdbDumpvars;
		#50_000_000;  // 50ms超时
		$fsdbDumpflush;
		$display("仿真超时结束");
		$finish;
	end

endmodule
