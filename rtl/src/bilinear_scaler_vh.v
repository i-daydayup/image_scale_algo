//============================================================================
// 模块名称    : bilinear_scaler_vh
// 功能描述    : 双线性图像缩放器（V+H架构）
// 架构参考    : rtl/src/bilinear_scaler_vh_arch.md v1.1
//
// 关键特性：
//   - V+H分离架构：先垂直插值(V-filter)，后水平插值(H-filter)
//   - 流式处理：3行输入后即可开始输出
//   - 双时钟域：clk_in(DDR输入) + clk_out(显示输出)
//   - 3-Buffer L1：busy_num + line_id 管理
//   - VPix不进RAM：直接进H-filter移位寄存器
//   - 输出FIFO：1行缓冲确保显示连续性
//
// 版本历史    : v1.0 - 初始版本
//============================================================================

`timescale 1ns / 1ps

module bilinear_scaler_vh #(
	parameter DATA_WIDTH  = 8                    ,
	parameter INT_BITS    = 16                   ,// 支持最大16x放大
	parameter FRAC_BITS   = 8                    ,
	parameter MAX_WIDTH   = 4096                 ,
	parameter MAX_HEIGHT  = 4096                 ,
	parameter ADDR_WIDTH  = $clog2(MAX_WIDTH)
)(
	//------------------------------------------------------------------------
	// 输入时钟域 (源图像/DDR)
	//------------------------------------------------------------------------
	input  wire                          clk_in        ,//I1,输入时钟
	input  wire                          rst_n_in      ,//I1,输入复位

	//------------------------------------------------------------------------
	// 配置接口 (clk_in域，在i_frame_start前配置有效)
	//------------------------------------------------------------------------
	input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_x ,//Ix,X方向逆缩放比例
	input  wire [INT_BITS+FRAC_BITS-1:0] cfg_inv_scale_y ,//Ix,Y方向逆缩放比例
	input  wire [15:0]                   cfg_dst_width   ,//I16,目标图像宽度
	input  wire [15:0]                   cfg_dst_height  ,//I16,目标图像高度
	input  wire [15:0]                   cfg_src_width   ,//I16,源图像宽度
	input  wire [15:0]                   cfg_src_height  ,//I16,源图像高度

	//------------------------------------------------------------------------
	// 输入视频流 (AXI-S风格, clk_in域)
	//------------------------------------------------------------------------
	input  wire                          i_valid       ,//I1,输入数据有效
	input  wire [DATA_WIDTH-1:0]         i_data        ,//Ix,输入像素数据
	input  wire                          i_last        ,//I1,行结束标记
	input  wire                          i_frame_start ,//I1,帧开始标记
	output wire                          i_ready       ,//O1,模块就绪

	//------------------------------------------------------------------------
	// 输出时钟域 (显示接口)
	//------------------------------------------------------------------------
	input  wire                          clk_out       ,//I1,输出时钟
	input  wire                          rst_n_out     ,//I1,输出复位

	//------------------------------------------------------------------------
	// 输出视频流 (AXI-S风格, clk_out域)
	//------------------------------------------------------------------------
	output reg                           o_valid       ,//O1,输出数据有效
	output reg  [DATA_WIDTH-1:0]         o_data        ,//Ox,输出像素数据
	output reg                           o_last        ,//O1,行结束标记
	output reg                           o_frame_start ,//O1,帧开始标记
	input  wire                          o_ready        //I1,下游就绪
);

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 坐标计算位宽
	//------------------------------------------------------------------------
	localparam COORD_BITS = INT_BITS + FRAC_BITS;

	//------------------------------------------------------------------------
	// 全局信号声明（供多个部分使用）
	//------------------------------------------------------------------------
	// H-filter输出（在第5部分生成，第3部分和第7部分使用）
	reg [DATA_WIDTH-1:0] h_filter_out      ;
	reg                  h_filter_valid_out;

	//------------------------------------------------------------------------
	//========================================================================
	// 第1部分：配置寄存器（clk_out域，帧开始时锁存）
	//========================================================================
	reg [COORD_BITS-1:0] r_inv_scale_x ;
	reg [COORD_BITS-1:0] r_inv_scale_y ;
	reg [15:0]           r_dst_width   ;
	reg [15:0]           r_dst_height  ;
	reg [15:0]           r_src_width   ;
	reg [15:0]           r_src_height  ;

	// 帧开始同步（clk_in -> clk_out）
	reg       frame_start_toggle ;
	reg [2:0] frame_start_sync   ;
	wire      frame_start_pulse  ;

	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0) begin
			frame_start_toggle <= 1'b0;
		end
		else if (i_frame_start == 1'b1 && i_valid == 1'b1) begin
			frame_start_toggle <= #U_DLY ~frame_start_toggle;
		end
	end

	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			frame_start_sync <= 3'b000;
		end
		else begin
			frame_start_sync <= #U_DLY {frame_start_sync[1:0], frame_start_toggle};
		end
	end

	assign frame_start_pulse = frame_start_sync[2] ^ frame_start_sync[1];

	// 配置锁存（clk_out域）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			r_inv_scale_x  <= {COORD_BITS{1'b0}};
			r_inv_scale_y  <= {COORD_BITS{1'b0}};
			r_dst_width    <= 16'd0;
			r_dst_height   <= 16'd0;
			r_src_width    <= 16'd0;
			r_src_height   <= 16'd0;
		end
		else if (frame_start_pulse == 1'b1) begin
			r_inv_scale_x  <= #U_DLY cfg_inv_scale_x;
			r_inv_scale_y  <= #U_DLY cfg_inv_scale_y;
			r_dst_width    <= #U_DLY cfg_dst_width;
			r_dst_height   <= #U_DLY cfg_dst_height;
			r_src_width    <= #U_DLY cfg_src_width;
			r_src_height   <= #U_DLY cfg_src_height;
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第2部分：L1 输入缓冲（3-Buffer，busy_num+line_id管理）
	//========================================================================
	// 实例化3个单行buffer，busy_num+line_id管理

	// L1 写控制（clk_in域）
	reg  [ADDR_WIDTH-1:0] l1_wr_addr_cnt     ;// 写地址计数
	reg  [1:0]            l1_wr_buf_sel      ;// 写buffer选择: 0/1/2 循环
	reg  [1:0]            l1_buf_busy_num    ;// 已占用的buffer数量: 0/1/2/3
	reg  [15:0]           in_row_idx         ;// 输入行计数器（0开始）
	reg  [15:0]           l1_buf0_line_id    ;// buffer0存储的源行号
	reg  [15:0]           l1_buf1_line_id    ;// buffer1存储的源行号
	reg  [15:0]           l1_buf2_line_id    ;// buffer2存储的源行号

	// L1 读控制（clk_out域）
	wire [ADDR_WIDTH-1:0] l1_rd_addr         ;// 读地址
	reg  [1:0]            l1_rd_buf_sel_y0   ;// Y0行buffer选择
	reg  [1:0]            l1_rd_buf_sel_y1   ;// Y1行buffer选择
	wire                  l1_rd_en           ;// 读使能
	wire [DATA_WIDTH-1:0] l1_rd_data_0       ;// buffer0读数据
	wire [DATA_WIDTH-1:0] l1_rd_data_1       ;// buffer1读数据
	wire [DATA_WIDTH-1:0] l1_rd_data_2       ;// buffer2读数据
	wire                  l1_rd_data_0_valid ;// buffer0读有效
	wire                  l1_rd_data_1_valid ;// buffer1读有效
	wire                  l1_rd_data_2_valid ;// buffer2读有效
	wire [DATA_WIDTH-1:0] l1_rd_data_y0      ;// Y0行读数据（选择后）
	wire [DATA_WIDTH-1:0] l1_rd_data_y1      ;// Y1行读数据（选择后）
	reg                   l1_rd_data_valid   ;// 读数据有效

	// line_id 同步到 clk_out（简化：直接打2拍，适用于变化频率低且可容忍短暂错误）
	reg [15:0] l1_buf0_line_id_d0;// line_id 第1拍
	reg [15:0] l1_buf1_line_id_d0;
	reg [15:0] l1_buf2_line_id_d0;
	reg [15:0] l1_buf0_line_id_d1;// line_id 第2拍（clk_out域使用）
	reg [15:0] l1_buf1_line_id_d1;
	reg [15:0] l1_buf2_line_id_d1;

	// v_min_src_row 从clk_out同步到clk_in（跨时钟域，DMUX方案）
	reg  [15:0]           v_min_src_row        ;// clk_out域生成
	reg  [15:0]           v_min_src_row_even   ;// even缓冲（clk_out域）
	reg  [15:0]           v_min_src_row_odd    ;// odd缓冲（clk_out域）
	reg                   v_min_src_row_sel    ;// 选择信号（clk_out域，Toggle）
	// v_min_src_row_sel_dy 已定义为 reg [1:0]，见同步逻辑处
	reg  [15:0]           v_min_src_row_synced ;// 同步后的值（clk_in域）

	// 判断当前选中的buffer是否可以释放（busy_num==3时）
	wire                  buf_can_release;

	// L1 buffer写使能
	wire l1_buf0_wr_en = i_valid & i_ready & (l1_wr_buf_sel == 2'd0);
	wire l1_buf1_wr_en = i_valid & i_ready & (l1_wr_buf_sel == 2'd1);
	wire l1_buf2_wr_en = i_valid & i_ready & (l1_wr_buf_sel == 2'd2);

	// i_ready：busy_num < 3 时无条件可写；busy_num == 3 时需判断可释放
	assign i_ready = (l1_buf_busy_num < 2'd3) || buf_can_release;

	// 判断当前选中的buffer是否可以释放（busy_num==3时）
	// 使用同步后的 v_min_src_row_synced 进行比较
	assign buf_can_release = (l1_buf_busy_num == 2'd3) &&
	                         ((l1_wr_buf_sel == 2'd0 && l1_buf0_line_id < v_min_src_row_synced) ||
	                          (l1_wr_buf_sel == 2'd1 && l1_buf1_line_id < v_min_src_row_synced) ||
	                          (l1_wr_buf_sel == 2'd2 && l1_buf2_line_id < v_min_src_row_synced));

	// L1 wrapper实例0
	line_buffer_wrapper #(
		.DATA_WIDTH(DATA_WIDTH ),
		.MAX_WIDTH (MAX_WIDTH  ),
		.VENDOR    ("GENERIC"  )
	) u_l1_buf0 (
		.clk           (clk_in                ),//I1,
		.rst_n         (rst_n_in              ),//I1,
		.wr_en         (l1_buf0_wr_en         ),//I1,
		.wr_addr       (l1_wr_addr_cnt        ),//Ix,
		.wr_data       (i_data                ),//Ix,
		.wr_full       (                      ),//O1,
		.rd_en         (l1_rd_en              ),//I1,
		.rd_addr       (l1_rd_addr            ),//Ix,
		.rd_data       (l1_rd_data_0          ),//Ox,
		.rd_data_valid (l1_rd_data_0_valid    ),//O1,
		.datacnt       (                      ) //Ox+1, 未使用
	);

	// L1 wrapper实例1
	line_buffer_wrapper #(
		.DATA_WIDTH(DATA_WIDTH ),
		.MAX_WIDTH (MAX_WIDTH  ),
		.VENDOR    ("GENERIC"  )
	) u_l1_buf1 (
		.clk           (clk_in                ),//I1,
		.rst_n         (rst_n_in              ),//I1,
		.wr_en         (l1_buf1_wr_en         ),//I1,
		.wr_addr       (l1_wr_addr_cnt        ),//Ix,
		.wr_data       (i_data                ),//Ix,
		.wr_full       (                      ),//O1,
		.rd_en         (l1_rd_en              ),//I1,
		.rd_addr       (l1_rd_addr            ),//Ix,
		.rd_data       (l1_rd_data_1          ),//Ox,
		.rd_data_valid (l1_rd_data_1_valid    ),//O1,
		.datacnt       (                      ) //Ox+1, 未使用
	);

	// L1 wrapper实例2
	line_buffer_wrapper #(
		.DATA_WIDTH(DATA_WIDTH ),
		.MAX_WIDTH (MAX_WIDTH  ),
		.VENDOR    ("GENERIC"  )
	) u_l1_buf2 (
		.clk           (clk_in                ),//I1,
		.rst_n         (rst_n_in              ),//I1,
		.wr_en         (l1_buf2_wr_en         ),//I1,
		.wr_addr       (l1_wr_addr_cnt        ),//Ix,
		.wr_data       (i_data                ),//Ix,
		.wr_full       (                      ),//O1,
		.rd_en         (l1_rd_en              ),//I1,
		.rd_addr       (l1_rd_addr            ),//Ix,
		.rd_data       (l1_rd_data_2          ),//Ox,
		.rd_data_valid (l1_rd_data_2_valid    ),//O1,
		.datacnt       (                      ) //Ox+1, 未使用
	);

	//------------------------------------------------------------------------
	// L1 写控制 - 写地址计数器（单独always）
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0) begin
			l1_wr_addr_cnt <= {ADDR_WIDTH{1'b0}};
		end
		else if (i_valid == 1'b1 && i_ready == 1'b1) begin
			if (i_last == 1'b1)
				l1_wr_addr_cnt <= #U_DLY {ADDR_WIDTH{1'b0}};
			else
				l1_wr_addr_cnt <= #U_DLY l1_wr_addr_cnt + 1'b1;
			end
	end

	//------------------------------------------------------------------------
	// L1 写控制 - 输入行计数器（单独always）
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0)
			in_row_idx <= 16'd0;
		else if (i_frame_start == 1'b1 && i_valid == 1'b1)
			in_row_idx <= #U_DLY 16'd0;
		else if (i_valid == 1'b1 && i_ready == 1'b1 && i_last == 1'b1)
			in_row_idx <= #U_DLY in_row_idx + 1'b1;
	end

	//------------------------------------------------------------------------
	// L1 写控制 - buffer选择（0->1->2->0循环）
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0)
			l1_wr_buf_sel <= 2'd0;
		else if (i_frame_start == 1'b1 && i_valid == 1'b1)
			l1_wr_buf_sel <= #U_DLY 2'd0;
		else if (i_valid == 1'b1 && i_ready == 1'b1 && i_last == 1'b1) begin
			if (l1_wr_buf_sel >= 2'd2)
				l1_wr_buf_sel <= #U_DLY 2'd0;
			else
				l1_wr_buf_sel <= #U_DLY l1_wr_buf_sel + 1'b1;
			end
	end

	//------------------------------------------------------------------------
	// L1 写控制 - busy_num计数（0->1->2->3饱和）
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0)
			l1_buf_busy_num <= 2'd0;
		else if (i_frame_start == 1'b1 && i_valid == 1'b1)
			l1_buf_busy_num <= #U_DLY 2'd0;
		else if (i_valid == 1'b1 && i_ready == 1'b1 && i_last == 1'b1) begin
			if (l1_buf_busy_num < 2'd3)
				l1_buf_busy_num <= #U_DLY l1_buf_busy_num + 1'b1;
			end
	end

	//------------------------------------------------------------------------
	// L1 写控制 - 记录每个buffer存储的行号
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0) begin
			l1_buf0_line_id <= 16'd0;
			l1_buf1_line_id <= 16'd0;
			l1_buf2_line_id <= 16'd0;
		end
		else if (i_frame_start == 1'b1 && i_valid == 1'b1) begin
			l1_buf0_line_id <= #U_DLY 16'd0;
			l1_buf1_line_id <= #U_DLY 16'd0;
			l1_buf2_line_id <= #U_DLY 16'd0;
		end
		else if (i_valid == 1'b1 && i_ready == 1'b1 && i_last == 1'b1) begin
			case (l1_wr_buf_sel)
				2'd0: l1_buf0_line_id <= #U_DLY in_row_idx;
				2'd1: l1_buf1_line_id <= #U_DLY in_row_idx;
				2'd2: l1_buf2_line_id <= #U_DLY in_row_idx;
			endcase
		end
	end

	//------------------------------------------------------------------------
	// v_min_src_row 跨时钟域同步（clk_out -> clk_in，DMUX方案）
	//------------------------------------------------------------------------
	// clk_in域：同步选择信号（单bit用_dy风格）
	reg		[1:0]	v_min_src_row_sel_dy;
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0)
			v_min_src_row_sel_dy <= 2'd0;
		else begin
			// 选择信号2-stage同步（_dy风格）
			v_min_src_row_sel_dy <= #U_DLY {v_min_src_row_sel_dy[0], v_min_src_row_sel};
		end

		if (rst_n_in == 1'b0)
			v_min_src_row_synced <= 16'd0;
		else begin
			// 边沿检测时采样数据（even/odd直接组合选择，无需打拍）
			if (v_min_src_row_sel_dy[0] != v_min_src_row_sel_dy[1]) begin
				v_min_src_row_synced <= #U_DLY (v_min_src_row_sel_dy[0] == 1'b1) ? v_min_src_row_odd : v_min_src_row_even;
			end
		end
	end

	//------------------------------------------------------------------------
	// line_id 同步到 clk_out（多bit打2拍，简化方案）
	//------------------------------------------------------------------------
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			l1_buf0_line_id_d0 <= 16'd0;
			l1_buf1_line_id_d0 <= 16'd0;
			l1_buf2_line_id_d0 <= 16'd0;
			l1_buf0_line_id_d1 <= 16'd0;
			l1_buf1_line_id_d1 <= 16'd0;
			l1_buf2_line_id_d1 <= 16'd0;
		end
		else begin
			l1_buf0_line_id_d0 <= #U_DLY l1_buf0_line_id;
			l1_buf1_line_id_d0 <= #U_DLY l1_buf1_line_id;
			l1_buf2_line_id_d0 <= #U_DLY l1_buf2_line_id;
			l1_buf0_line_id_d1 <= #U_DLY l1_buf0_line_id_d0;
			l1_buf1_line_id_d1 <= #U_DLY l1_buf1_line_id_d0;
			l1_buf2_line_id_d1 <= #U_DLY l1_buf2_line_id_d0;
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第3部分：目标坐标生成（clk_out域）
	//========================================================================
	reg [15:0] dst_y_cnt    ;
	reg [15:0] dst_x_cnt    ;
	reg        output_active;

	// 源坐标（定点数）
	wire [COORD_BITS-1:0] coord_y     ;
	wire [INT_BITS-1:0]   coord_y_int ;
	wire [FRAC_BITS-1:0]  coord_y_frac;
	wire                  coord_y_valid;
	wire [COORD_BITS-1:0] coord_x     ;
	wire [INT_BITS-1:0]   coord_x_int ;
	wire [FRAC_BITS-1:0]  coord_x_frac;
	wire                  coord_x_valid;

	// 坐标计算实例化（复用现有模块）
	bilinear_coord_calc #(
		.INT_BITS    (INT_BITS    ),
		.FRAC_BITS   (FRAC_BITS   ),
		.WEIGHT_BITS (FRAC_BITS   )
	) u_coord_y (
		.clk          (clk_out        ),//I1,
		.rst_n        (rst_n_out      ),//I1,
		.dst_idx      (dst_y_cnt      ),//I16,
		.inv_scale    (r_inv_scale_y  ),//Ix,
		.src_size     (r_src_height   ),//I16,
		.src_pos_int  (coord_y_int    ),//O16,
		.src_pos_frac (coord_y_frac   ),//Ox,
		.valid        (coord_y_valid  ) //O1,
	);

	bilinear_coord_calc #(
		.INT_BITS    (INT_BITS    ),
		.FRAC_BITS   (FRAC_BITS   ),
		.WEIGHT_BITS (FRAC_BITS   )
	) u_coord_x (
		.clk          (clk_out        ),//I1,
		.rst_n        (rst_n_out      ),//I1,
		.dst_idx      (dst_x_cnt      ),//I16,
		.inv_scale    (r_inv_scale_x  ),//Ix,
		.src_size     (r_src_width    ),//I16,
		.src_pos_int  (coord_x_int    ),//O16,
		.src_pos_frac (coord_x_frac   ),//Ox,
		.valid        (coord_x_valid  ) //O1,
	);

	// 需要的源行号
	wire [15:0] need_src_y0 = coord_y_int;
	wire [15:0] need_src_y1 = coord_y_int + 1'b1;

	//------------------------------------------------------------------------
	//========================================================================
	// 第4部分：V-filter（双线性垂直插值，2-tap）
	//========================================================================
	// 读取L1的两行（y0和y1），根据coord_y_frac做插值

	// L1 buffer 匹配（组合逻辑）
	// 使用同步后的 line_id_d1（clk_out域）
	wire buf0_has_y0 = (l1_buf0_line_id_d1 == need_src_y0);
	wire buf1_has_y0 = (l1_buf1_line_id_d1 == need_src_y0);
	wire buf2_has_y0 = (l1_buf2_line_id_d1 == need_src_y0);
	wire buf0_has_y1 = (l1_buf0_line_id_d1 == need_src_y1);
	wire buf1_has_y1 = (l1_buf1_line_id_d1 == need_src_y1);
	wire buf2_has_y1 = (l1_buf2_line_id_d1 == need_src_y1);

	// V-filter启动条件：需要的源行是否就绪
	// 双线性需要2行：need_src_y0 和 need_src_y1
	// 边界情况：下边界时y1=y0（复制边界）
	wire need_y0_ready = buf0_has_y0 || buf1_has_y0 || buf2_has_y0;
	wire need_y1_ready = (need_src_y1 >= r_src_height) ? need_y0_ready :  // 下边界复制
	                     (buf0_has_y1 || buf1_has_y1 || buf2_has_y1);
	wire v_filter_ready = need_y0_ready && need_y1_ready;

	// 输出控制 - output_active（启动/停止控制）
	// 注意：依赖v_filter_ready信号（已在上方定义）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0)
			output_active <= 1'b0;
		else if (frame_start_pulse == 1'b1)
			output_active <= 1'b0;
		else if (output_active == 1'b0 && v_filter_ready)
			// V-filter所需行就绪即可开始输出
			output_active <= #U_DLY 1'b1;
		else if (output_active == 1'b1 && o_valid == 1'b1 && o_ready == 1'b1 &&
		         dst_x_cnt >= r_dst_width - 1 && dst_y_cnt >= r_dst_height - 1)
			// 最后一行最后一个像素输出完成，停止
			output_active <= #U_DLY 1'b0;
	end

	// 输出控制 - dst_x_cnt（行内像素计数）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0)
			dst_x_cnt <= 16'd0;
		else if (frame_start_pulse == 1'b1)
			dst_x_cnt <= 16'd0;
		else if (output_active == 1'b1 && h_filter_valid_out == 1'b1) begin
			if (dst_x_cnt >= r_dst_width - 1)
				dst_x_cnt <= 16'd0;
			else
				dst_x_cnt <= #U_DLY dst_x_cnt + 1'b1;
			end
	end

	// 输出控制 - dst_y_cnt（行计数）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0)
			dst_y_cnt <= 16'd0;
		else if (frame_start_pulse == 1'b1)
			dst_y_cnt <= 16'd0;
		else if (output_active == 1'b1 && h_filter_valid_out == 1'b1 &&
		         dst_x_cnt >= r_dst_width - 1) begin
			if (dst_y_cnt >= r_dst_height - 1)
				dst_y_cnt <= 16'd0;
			else
				dst_y_cnt <= #U_DLY dst_y_cnt + 1'b1;
			end
	end

	// Buffer选择（优先级编码）
	always @(*) begin
		if      (buf0_has_y0) l1_rd_buf_sel_y0 = 2'd0;
		else if (buf1_has_y0) l1_rd_buf_sel_y0 = 2'd1;
		else if (buf2_has_y0) l1_rd_buf_sel_y0 = 2'd2;
		else                  l1_rd_buf_sel_y0 = 2'd0;
	end

	always @(*) begin
		if      (buf0_has_y1) l1_rd_buf_sel_y1 = 2'd0;
		else if (buf1_has_y1) l1_rd_buf_sel_y1 = 2'd1;
		else if (buf2_has_y1) l1_rd_buf_sel_y1 = 2'd2;
		else                  l1_rd_buf_sel_y1 = 2'd0;
	end

	// L1读数据选择（带边界处理）
	// 当 need_src_y1 >= r_src_height 时，y1=y0（下边界复制）
	reg  [DATA_WIDTH-1:0] l1_rd_data_y0_raw;
	reg  [DATA_WIDTH-1:0] l1_rd_data_y1_raw;

	always @(*) begin
		case (l1_rd_buf_sel_y0)
			2'd0: l1_rd_data_y0_raw = l1_rd_data_0;
			2'd1: l1_rd_data_y0_raw = l1_rd_data_1;
			2'd2: l1_rd_data_y0_raw = l1_rd_data_2;
			default: l1_rd_data_y0_raw = l1_rd_data_0;
		endcase
	end

	always @(*) begin
		case (l1_rd_buf_sel_y1)
			2'd0: l1_rd_data_y1_raw = l1_rd_data_0;
			2'd1: l1_rd_data_y1_raw = l1_rd_data_1;
			2'd2: l1_rd_data_y1_raw = l1_rd_data_2;
			default: l1_rd_data_y1_raw = l1_rd_data_1;
		endcase
	end

	// 边界处理：上边界(need_src_y0<0)和下边界(need_src_y1>=r_src_height)
	// 由于 coord_y_int 已经 clamped，只需要处理下边界
	assign l1_rd_data_y0 = l1_rd_data_y0_raw;
	assign l1_rd_data_y1 = (need_src_y1 >= r_src_height) ? l1_rd_data_y0_raw : l1_rd_data_y1_raw;

	// L1读使能和地址
	assign l1_rd_addr = dst_x_cnt;
	assign l1_rd_en   = output_active;

	// V-filter 权重计算
	wire [FRAC_BITS-1:0] w_y0 = (1 << FRAC_BITS) - coord_y_frac;
	wire [FRAC_BITS-1:0] w_y1 = coord_y_frac;

	// V-filter 输出（VPix，不进RAM，直接进H-filter）
	reg [DATA_WIDTH-1:0] vpix_out;
	reg                  vpix_valid;
	reg [FRAC_BITS-1:0]  vpix_dx_frac;  // 水平权重，传递给H-filter

	always @(posedge clk_out) begin
		if (output_active == 1'b1) begin
			// 双线性垂直插值
			vpix_out    <= #U_DLY (l1_rd_data_y0 * w_y0 + l1_rd_data_y1 * w_y1) >> FRAC_BITS;
			vpix_valid  <= #U_DLY 1'b1;
			vpix_dx_frac<= #U_DLY coord_x_frac;
		end
		else begin
			vpix_valid <= #U_DLY 1'b0;
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第5部分：H-filter（双线性水平插值，2-tap）
	//========================================================================
	// VPix直接进移位寄存器，不进RAM

	// H-filter 移位寄存器
	reg [DATA_WIDTH-1:0] h_shift_reg0 ;
	reg [DATA_WIDTH-1:0] h_shift_reg1 ;
	reg [FRAC_BITS-1:0]  h_dx_frac    ;
	reg                  h_shift_valid;

	// 移位寄存器更新
	always @(posedge clk_out) begin
		if (vpix_valid == 1'b1) begin
			h_shift_reg1  <= #U_DLY h_shift_reg0;
			h_shift_reg0  <= #U_DLY vpix_out;
			h_dx_frac     <= #U_DLY vpix_dx_frac;
			h_shift_valid <= #U_DLY 1'b1;
		end
		else begin
			h_shift_valid <= #U_DLY 1'b0;
		end
	end

	// H-filter 权重
	wire [FRAC_BITS-1:0] w_x0 = (1 << FRAC_BITS) - h_dx_frac;
	wire [FRAC_BITS-1:0] w_x1 = h_dx_frac;

	// H-filter 输出（使用全局声明的 h_filter_out, h_filter_valid_out）
	always @(posedge clk_out) begin
		if (h_shift_valid == 1'b1) begin
			h_filter_out      <= #U_DLY (h_shift_reg0 * w_x0 + h_shift_reg1 * w_x1) >> FRAC_BITS;
			h_filter_valid_out<= #U_DLY 1'b1;
		end
		else begin
			h_filter_valid_out<= #U_DLY 1'b0;
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第6部分：v_min_src_row 生成与同步（驱动L1释放，DMUX方案）
	//========================================================================
	// v_min_src_row：当前V-filter窗口需要的最小源行号
	// 当某buffer的line_id < v_min_src_row时，该buffer可释放
	// 注：v_min_src_row 已在第2部分信号声明区域定义

	// v_min_src_row 生成逻辑
	// v_min_src_row = 当前V-filter窗口需要的最小源行号（即coord_y_int）
	// 当放大时，coord_y_int增加缓慢；缩小时，coord_y_int快速增加
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0)
			v_min_src_row <= 16'd0;
		else if (frame_start_pulse == 1'b1)
			v_min_src_row <= #U_DLY 16'd0;
		else if (output_active == 1'b1 && coord_y_valid == 1'b1)
			// 只要坐标有效，就实时跟踪最小源行号
			v_min_src_row <= #U_DLY coord_y_int;
	end

	// DMUX方案：双缓冲（even/odd）+ Toggle选择
	// 将v_min_src_row值交替写入even/odd缓冲，并Toggle选择信号
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			v_min_src_row_sel  <= 1'b0;
			v_min_src_row_even <= 16'd0;
			v_min_src_row_odd  <= 16'd0;
		end
		else begin
			// 检测v_min_src_row变化，交替写入even/odd
			if ((v_min_src_row_sel == 1'b0 && v_min_src_row != v_min_src_row_even) ||
			    (v_min_src_row_sel == 1'b1 && v_min_src_row != v_min_src_row_odd)) begin
				// 写入对应的缓冲
				if (v_min_src_row_sel == 1'b0)
					v_min_src_row_even <= #U_DLY v_min_src_row;
				else
					v_min_src_row_odd  <= #U_DLY v_min_src_row;
				// Toggle选择信号
				v_min_src_row_sel <= #U_DLY ~v_min_src_row_sel;
			end
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第7部分：L2 输出FIFO（1行缓冲）
	//========================================================================
	// 存储H-filter最终输出，确保显示连续性

	reg [DATA_WIDTH-1:0] l2_fifo_mem [0:MAX_WIDTH-1];
	reg [ADDR_WIDTH-1:0] l2_wr_ptr;
	reg [ADDR_WIDTH-1:0] l2_rd_ptr;
	reg [ADDR_WIDTH:0]   l2_fifo_cnt;
	reg                  l2_wr_line_done;
	reg                  l2_rd_line_done;

	// 预填充阈值
	localparam PRE_FILL_THRESHOLD = MAX_WIDTH / 2;

	// L2 写控制（H-filter输出）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			l2_wr_ptr <= {ADDR_WIDTH{1'b0}};
			l2_wr_line_done <= 1'b0;
		end
		else if (frame_start_pulse == 1'b1) begin
			l2_wr_ptr <= #U_DLY {ADDR_WIDTH{1'b0}};
			l2_wr_line_done <= #U_DLY 1'b0;
		end
		else if (h_filter_valid_out == 1'b1) begin
			l2_fifo_mem[l2_wr_ptr] <= #U_DLY h_filter_out;
			if (l2_wr_ptr >= r_dst_width - 1) begin
				l2_wr_ptr <= #U_DLY {ADDR_WIDTH{1'b0}};
				l2_wr_line_done <= #U_DLY 1'b1;
			end
			else begin
				l2_wr_ptr <= #U_DLY l2_wr_ptr + 1'b1;
				l2_wr_line_done <= #U_DLY 1'b0;
			end
		end
		else begin
			l2_wr_line_done <= #U_DLY 1'b0;
		end
	end

	// L2 读控制（显示接口）
	reg [DATA_WIDTH-1:0] l2_rd_data;

	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			l2_rd_ptr <= {ADDR_WIDTH{1'b0}};
			l2_rd_line_done <= 1'b0;
		end
		else if (frame_start_pulse == 1'b1) begin
			l2_rd_ptr <= #U_DLY {ADDR_WIDTH{1'b0}};
			l2_rd_line_done <= #U_DLY 1'b0;
		end
		else if (o_valid == 1'b1 && o_ready == 1'b1) begin
			l2_rd_data <= #U_DLY l2_fifo_mem[l2_rd_ptr];
			if (l2_rd_ptr >= r_dst_width - 1) begin
				l2_rd_ptr <= #U_DLY {ADDR_WIDTH{1'b0}};
				l2_rd_line_done <= #U_DLY 1'b1;
			end
			else begin
				l2_rd_ptr <= #U_DLY l2_rd_ptr + 1'b1;
				l2_rd_line_done <= #U_DLY 1'b0;
			end
		end
		else begin
			l2_rd_line_done <= #U_DLY 1'b0;
		end
	end

	// FIFO计数（读写指针差）
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0)
			l2_fifo_cnt <= {ADDR_WIDTH+1{1'b0}};
		else if (frame_start_pulse == 1'b1)
			l2_fifo_cnt <= #U_DLY {ADDR_WIDTH+1{1'b0}};
		else begin
			if (h_filter_valid_out == 1'b1 && !(o_valid == 1'b1 && o_ready == 1'b1))
				l2_fifo_cnt <= #U_DLY l2_fifo_cnt + 1'b1;
			else if (!(h_filter_valid_out == 1'b1) && (o_valid == 1'b1 && o_ready == 1'b1))
				l2_fifo_cnt <= #U_DLY l2_fifo_cnt - 1'b1;
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第8部分：输出接口
	//========================================================================
	// 预填充策略：确保FIFO有至少半行数据后再开始输出

	wire o_valid_pre = (l2_fifo_cnt > PRE_FILL_THRESHOLD[ADDR_WIDTH:0]) && !o_last;

	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			o_valid       <= 1'b0;
			o_data        <= {DATA_WIDTH{1'b0}};
			o_last        <= 1'b0;
			o_frame_start <= 1'b0;
		end
		else begin
			o_valid       <= #U_DLY o_valid_pre && (l2_fifo_cnt > 0);
			o_data        <= #U_DLY l2_rd_data;
			o_last        <= #U_DLY (l2_rd_ptr >= r_dst_width - 1) && o_valid_pre;
			o_frame_start <= #U_DLY (dst_y_cnt == 16'd0) && (l2_rd_ptr == 16'd0) && o_valid_pre;
		end
	end

endmodule
