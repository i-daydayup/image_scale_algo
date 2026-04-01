//============================================================================
// 模块名称    : bilinear_scaler
// 功能描述    : 双线性插值图像缩放器（定点实现）
// 作者        :
// 创建日期    :
// 版本历史    :
//   v1.0 - 初始版本
//
// 对应Python  : python/algorithms/bilinear_fixed.py
// 定点格式    : 坐标 Qm.n, 权重 Q0.w
// 验证状态    : 验证中
//============================================================================

`timescale 1ns / 1ps

module bilinear_scaler #(
	parameter DATA_WIDTH  = 8                    ,// 像素数据位宽
	parameter INT_BITS    = 12                   ,// 坐标整数位宽 m
	parameter FRAC_BITS   = 8                    ,// 坐标小数位宽 n
	parameter WEIGHT_BITS = 8                    ,// 权重小数位宽 w
	parameter MAX_WIDTH   = 4096                 ,// 最大图像宽度
	parameter MAX_HEIGHT  = 4096                  // 最大图像高度
)(
	input  wire                          clk          ,//I1,
	input  wire                          rst_n        ,//I1,

	// 配置接口 (在 i_frame_start 前配置有效)
	input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_x ,//Ix,X方向逆缩放比例定点值
	input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_y ,//Ix,Y方向逆缩放比例定点值
	input  wire [15:0]                   cfg_dst_width   ,//I16,目标图像宽度
	input  wire [15:0]                   cfg_dst_height  ,//I16,目标图像高度
	input  wire [15:0]                   cfg_src_width   ,//I16,源图像宽度
	input  wire [15:0]                   cfg_src_height  ,//I16,源图像高度

	// 输入视频流 (AXI-Stream 风格)
	input  wire                          i_valid       ,//I1,
	input  wire [DATA_WIDTH-1:0]         i_data        ,//Ix,
	input  wire                          i_last        ,//I1,行结束标记
	input  wire                          i_frame_start ,//I1,帧开始标记 (tuser)
	output wire                          i_ready       ,//O1,

	// 输出视频流
	output reg                           o_valid       ,//O1,
	output reg  [DATA_WIDTH-1:0]         o_data        ,//Ox,
	output reg                           o_last        ,//O1,行结束标记
	output reg                           o_frame_start ,//O1,帧开始标记
	input  wire                          o_ready        //I1,
);

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 参数定义
	//------------------------------------------------------------------------
	localparam COORD_BITS   = INT_BITS + FRAC_BITS                               ;// 坐标总位宽
	localparam SCALE_COORD  = 1 << FRAC_BITS                                     ;// 2^n
	localparam SCALE_WEIGHT = 1 << WEIGHT_BITS                                   ;// 2^w
	localparam HALF_N       = FRAC_BITS > 0 ? (1 << (FRAC_BITS - 1)) : 0         ;// 0.5 的定点表示
	localparam FRAC_MASK    = (1 << FRAC_BITS) - 1                               ;// 小数部分掩码

	// 状态机定义
	localparam IDLE      = 3'b000;
	localparam CONFIG    = 3'b001;
	localparam READ_LINE = 3'b010;
	localparam PROCESS   = 3'b011;
	localparam OUTPUT    = 3'b100;
	localparam DONE      = 3'b101;

	//------------------------------------------------------------------------
	// 信号声明
	//------------------------------------------------------------------------
	// 状态寄存器
	reg [2:0] state      ;
	reg [2:0] next_state ;

	// 配置寄存器
	reg [COORD_BITS-1:0] r_inv_scale_x ;
	reg [COORD_BITS-1:0] r_inv_scale_y ;
	reg [15:0]           r_dst_width   ;
	reg [15:0]           r_dst_height  ;
	reg [15:0]           r_src_width   ;
	reg [15:0]           r_src_height  ;

	// 行列计数器
	reg [15:0] dst_x_cnt ;
	reg [15:0] dst_y_cnt ;
	reg [15:0] src_x_cnt ;
	reg [15:0] src_y_cnt ;

	// 坐标计算结果
	wire [15:0]            coord_x_int  ;
	wire [WEIGHT_BITS-1:0] coord_x_frac ;
	wire [15:0]            coord_y_int  ;
	wire [WEIGHT_BITS-1:0] coord_y_frac ;
	wire                   coord_valid  ;

	// 行缓冲接口
	wire [DATA_WIDTH-1:0]        lb_rdata0 ;// 当前行读数据
	wire [DATA_WIDTH-1:0]        lb_rdata1 ;// 前一行读数据
	reg                          lb_wen    ;
	reg  [DATA_WIDTH-1:0]        lb_wdata  ;
	reg  [$clog2(MAX_WIDTH)-1:0] lb_waddr  ;
	reg  [$clog2(MAX_WIDTH)-1:0] lb_raddr  ;
	reg                          lb_swap   ;// 行缓冲交换信号

	// 像素寄存器 (4个相邻像素)
	reg [DATA_WIDTH-1:0] p00 ;// 左上 (y0, x0)
	reg [DATA_WIDTH-1:0] p10 ;// 右上 (y0, x1)
	reg [DATA_WIDTH-1:0] p01 ;// 左下 (y1, x0)
	reg [DATA_WIDTH-1:0] p11 ;// 右下 (y1, x1)

	// 插值结果
	wire [DATA_WIDTH-1:0] interp_result ;
	wire                  interp_valid  ;

	// 输入控制
	assign i_ready = (state == READ_LINE) && (src_x_cnt < r_src_width);

	//------------------------------------------------------------------------
	// 状态机 - 组合逻辑
	//------------------------------------------------------------------------
	always @(*) begin
		next_state = state;
		case (state)
			IDLE: begin
				if (i_frame_start == 1'b1 && i_valid == 1'b1)
					next_state = CONFIG;
			end
			CONFIG: begin
				next_state = READ_LINE;
			end
			READ_LINE: begin
				// 读完两行后开始处理
				if (src_y_cnt >= 2 && src_x_cnt >= r_src_width)
					next_state = PROCESS;
				else if (i_last == 1'b1 && i_valid == 1'b1)
					next_state = READ_LINE;
			end
			PROCESS: begin
				if (dst_y_cnt >= r_dst_height)
					next_state = DONE;
			end
			DONE: begin
				next_state = IDLE;
			end
			default: next_state = IDLE;
		endcase
	end

	//------------------------------------------------------------------------
	// 状态机 - 时序逻辑
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0)
			state <= IDLE;
		else
			state <= #U_DLY next_state;
	end

	//------------------------------------------------------------------------
	// 配置寄存器
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			r_inv_scale_x  <= 0;
			r_inv_scale_y  <= 0;
			r_dst_width    <= 0;
			r_dst_height   <= 0;
			r_src_width    <= 0;
			r_src_height   <= 0;
		end
		else if (state == CONFIG) begin
			r_inv_scale_x  <= #U_DLY cfg_inv_scale_x;
			r_inv_scale_y  <= #U_DLY cfg_inv_scale_y;
			r_dst_width    <= #U_DLY cfg_dst_width;
			r_dst_height   <= #U_DLY cfg_dst_height;
			r_src_width    <= #U_DLY cfg_src_width;
			r_src_height   <= #U_DLY cfg_src_height;
		end
	end

	//------------------------------------------------------------------------
	// 源图像行列计数器
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			src_x_cnt <= 0;
			src_y_cnt <= 0;
		end
		else if (state == IDLE) begin
			src_x_cnt <= #U_DLY 0;
			src_y_cnt <= #U_DLY 0;
		end
		else if (state == READ_LINE && i_valid == 1'b1) begin
			if (i_last == 1'b1) begin
				src_x_cnt <= #U_DLY 0;
				src_y_cnt <= #U_DLY src_y_cnt + 1;
			end
			else begin
				src_x_cnt <= #U_DLY src_x_cnt + 1;
			end
		end
	end

	//------------------------------------------------------------------------
	// 行缓冲控制
	// 使用双行缓冲存储最近的两行像素
	// 时序：写入延迟 1 拍，读出延迟 1 拍
	//------------------------------------------------------------------------
	reg [DATA_WIDTH-1:0] line_buffer_0 [0:MAX_WIDTH-1];  // 当前行
	reg [DATA_WIDTH-1:0] line_buffer_1 [0:MAX_WIDTH-1];  // 前一行

	// 行缓冲写入 (1拍延迟)
	always @(posedge clk) begin
		if (lb_wen == 1'b1) begin
			if (lb_swap == 1'b1) begin
				line_buffer_1[lb_waddr] <= #U_DLY lb_wdata;
			end
			else begin
				line_buffer_0[lb_waddr] <= #U_DLY lb_wdata;
			end
		end
	end

	// 行缓冲读出 (1拍延迟)
	reg [$clog2(MAX_WIDTH)-1:0] raddr_d1 ;
	reg                         swap_d1  ;
	always @(posedge clk) begin
		raddr_d1 <= #U_DLY lb_raddr;
		swap_d1  <= #U_DLY lb_swap;
	end

	assign lb_rdata0 = swap_d1 ? line_buffer_1[raddr_d1] : line_buffer_0[raddr_d1];
	assign lb_rdata1 = swap_d1 ? line_buffer_0[raddr_d1] : line_buffer_1[raddr_d1];

	// 行缓冲控制逻辑
	// 20260401三|魏：行结束时，应该多写一个地址，包括起点0和1也应该是相同的内容，代表边界多写。
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			lb_wen   <= 0;
			lb_wdata <= 0;
			lb_waddr <= 0;
			lb_raddr <= 0;
			lb_swap  <= 0;
		end
		else if (state == READ_LINE && i_valid == 1'b1) begin
			// 写入当前行
			lb_wen   <= #U_DLY 1;
			lb_wdata <= #U_DLY i_data;
			lb_waddr <= #U_DLY src_x_cnt;

			if (i_last == 1'b1) begin
				// 行结束，交换缓冲
				lb_swap <= #U_DLY ~lb_swap;
			end
		end
		else begin
			lb_wen <= #U_DLY 0;
		end
	end

	//------------------------------------------------------------------------
	// 坐标计算模块实例化
	//------------------------------------------------------------------------
	bilinear_coord_calc #(
		.INT_BITS    (INT_BITS    ),//12,
		.FRAC_BITS   (FRAC_BITS   ),//8,
		.WEIGHT_BITS (WEIGHT_BITS ) //8
	) u_coord_calc_x (
		.clk          (clk           ),//I1,
		.rst_n        (rst_n         ),//I1,
		.dst_idx      (dst_x_cnt     ),//I16,
		.inv_scale    (r_inv_scale_x ),//I20,
		.src_size     (r_src_width   ),//I16,
		.src_pos_int  (coord_x_int   ),//O16,
		.src_pos_frac (coord_x_frac  ),//O8,
		.valid        (coord_valid   ) //O1
	);

	bilinear_coord_calc #(
		.INT_BITS    (INT_BITS    ),//12,
		.FRAC_BITS   (FRAC_BITS   ),//8,
		.WEIGHT_BITS (WEIGHT_BITS ) //8
	) u_coord_calc_y (
		.clk          (clk            ),//I1,
		.rst_n        (rst_n          ),//I1,
		.dst_idx      (dst_y_cnt      ),//I16,
		.inv_scale    (r_inv_scale_y  ),//I20,
		.src_size     (r_src_height   ),//I16,
		.src_pos_int  (coord_y_int    ),//O16,
		.src_pos_frac (coord_y_frac   ),//O8,
		.valid        (               ) //O1
	);

	//------------------------------------------------------------------------
	// 像素获取状态机
	// 需要获取4个相邻像素，分多个周期完成
	//------------------------------------------------------------------------
	reg [2:0] pixel_fetch_state;
	localparam FETCH_IDLE  = 3'b000;
	localparam FETCH_P00   = 3'b001;
	localparam FETCH_P10   = 3'b010;
	localparam FETCH_P01   = 3'b011;
	localparam FETCH_P11   = 3'b100;
	localparam FETCH_DONE  = 3'b101;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			pixel_fetch_state <= FETCH_IDLE;
			p00 <= 0;
			p10 <= 0;
			p01 <= 0;
			p11 <= 0;
		end
		else if (state == PROCESS) begin
			case (pixel_fetch_state)
				FETCH_IDLE: begin
					pixel_fetch_state <= #U_DLY FETCH_P00;
					lb_raddr          <= #U_DLY coord_x_int;
				end
				FETCH_P00: begin
					// 第2拍：读取P00 (y0行, x0列)
					p00               <= #U_DLY lb_rdata0;
					pixel_fetch_state <= #U_DLY FETCH_P10;
					lb_raddr          <= #U_DLY coord_x_int + 1;
				end
				FETCH_P10: begin
					// 第3拍：读取P10 (y0行, x1列)
					p10               <= #U_DLY lb_rdata0;
					p01               <= #U_DLY lb_rdata1;  // 同时读取P01
					pixel_fetch_state <= #U_DLY FETCH_P11;
					lb_raddr          <= #U_DLY coord_x_int + 1;
				end
				FETCH_P11: begin
					// 第4拍：读取P11 (y1行, x1列)
					p11               <= #U_DLY lb_rdata1;
					pixel_fetch_state <= #U_DLY FETCH_DONE;
				end
				FETCH_DONE: begin
					pixel_fetch_state <= #U_DLY FETCH_IDLE;
				end
			endcase
		end
		else begin
			pixel_fetch_state <= #U_DLY FETCH_IDLE;
		end
	end

	//------------------------------------------------------------------------
	// 目标图像行列计数器
	//------------------------------------------------------------------------
	reg pixel_fetch_done_d1 ;
	always @(posedge clk) pixel_fetch_done_d1 <= #U_DLY (pixel_fetch_state == FETCH_DONE);

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			dst_x_cnt <= 0;
			dst_y_cnt <= 0;
		end
		else if (state == IDLE) begin
			dst_x_cnt <= #U_DLY 0;
			dst_y_cnt <= #U_DLY 0;
		end
		else if (state == PROCESS && pixel_fetch_done_d1 == 1'b1) begin
			if (dst_x_cnt >= r_dst_width - 1) begin
				dst_x_cnt <= #U_DLY 0;
				dst_y_cnt <= #U_DLY dst_y_cnt + 1;
			end
			else begin
				dst_x_cnt <= #U_DLY dst_x_cnt + 1;
			end
		end
	end

	//------------------------------------------------------------------------
	// 插值计算模块实例化
	//------------------------------------------------------------------------
	bilinear_interp #(
		.DATA_WIDTH  (DATA_WIDTH  ),//8,
		.WEIGHT_BITS (WEIGHT_BITS ) //8
	) u_interp (
		.clk       (clk                  ),//I1,
		.rst_n     (rst_n                ),//I1,
		.p00       (p00                  ),//I8,
		.p10       (p10                  ),//I8,
		.p01       (p01                  ),//I8,
		.p11       (p11                  ),//I8,
		.dx        (coord_x_frac         ),//I8,
		.dy        (coord_y_frac         ),//I8,
		.valid_in  (pixel_fetch_done_d1  ),//I1,
		.result    (interp_result        ),//O8,
		.valid_out (interp_valid         ) //O1
	);

	//------------------------------------------------------------------------
	// 输出接口
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			o_valid       <= 0;
			o_data        <= 0;
			o_last        <= 0;
			o_frame_start <= 0;
		end
		else begin
			o_valid       <= #U_DLY interp_valid && (state == PROCESS);
			o_data        <= #U_DLY interp_result;
			o_last        <= #U_DLY interp_valid && (dst_x_cnt >= r_dst_width - 1);
			o_frame_start <= #U_DLY (state == PROCESS) && (dst_y_cnt == 0) && (dst_x_cnt == 0) && pixel_fetch_done_d1;
		end
	end

endmodule
