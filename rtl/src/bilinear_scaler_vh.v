//============================================================================
// 模块名称    : bilinear_scaler_vh
// 功能描述    : 双线性图像缩放器（V+H架构）
// 架构参考    : rtl/src/bilinear_scaler_vh_arch.md
//
// 关键特性：
//   - V+H分离架构：先垂直插值(V-filter)，后水平插值(H-filter)
//   - 双线性插值：V-filter和H-filter各使用2点插值
//   - 三级缓冲：L1(2行) + L2(4行架构，实际用2行) + L3(1行FIFO)
//   - 双时钟域：clk_in(DDR输入) + clk_out(显示输出)
//
// 版本历史    : v1.0 - 双线性实现
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
	// 第2部分：L1 输入缓冲（2行Wrapper实例化，clk_in域）
	//========================================================================
	reg  [ADDR_WIDTH-1:0] l1_rd_addr       ;// 到L1的读地址
	wire [DATA_WIDTH-1:0] l1_rd_data       ;// 从L1读出的数据
	wire                  l1_rd_data_valid ;// L1读数据有效
	reg                   l1_rd_switch     ;// L1 buffer切换
	wire [1:0]            l1_buf_status    ;// L1 buffer状态

	// L1 写地址计数器
	reg [ADDR_WIDTH-1:0] l1_wr_addr_cnt ;

	// L2 读请求（到L1）
	reg                  l2_rd_req ;

	line_buffer_2row_wrapper #(
		.DATA_WIDTH(DATA_WIDTH	),
		.MAX_WIDTH (MAX_WIDTH 	),
		.VENDOR    ("GENERIC"	)
	) u_l1_buffer (
		.clk           (clk_in           ),//I1,
		.rst_n         (rst_n_in         ),//I1,
		.wr_valid      (i_valid          ),//I1,
		.wr_addr       (l1_wr_addr_cnt   ),//Ix,
		.wr_data       (i_data           ),//Ix,
		.wr_last       (i_last           ),//I1,
		.wr_ready      (i_ready          ),//O1,
		.rd_valid      (l2_rd_req        ),//I1,
		.rd_addr       (l1_rd_addr       ),//Ix,
		.rd_data       (l1_rd_data       ),//Ox,
		.rd_data_valid (l1_rd_data_valid ),//O1,
		.rd_switch     (l1_rd_switch     ),//I1,
		.buf_status    (l1_buf_status    ) //O2,
	);

	//------------------------------------------------------------------------
	// L1 写地址计数器
	//------------------------------------------------------------------------
	always @(posedge clk_in or negedge rst_n_in) begin
		if (rst_n_in == 1'b0) begin
			l1_wr_addr_cnt <= {ADDR_WIDTH{1'b0}};
		end
		else if (i_valid == 1'b1 && i_ready == 1'b1) begin
			if (i_last == 1'b1) begin
				l1_wr_addr_cnt <= {ADDR_WIDTH{1'b0}};
			end
			else begin
				l1_wr_addr_cnt <= #U_DLY l1_wr_addr_cnt + 1'b1;
			end
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第3部分：L2 V-filter缓冲（4行架构，双线性用2行，clk_out域）
	//========================================================================
	// 4行存储器，用于V-filter运算
	reg [DATA_WIDTH-1:0] l2_v_buf [0:3][0:MAX_WIDTH-1];

	// L2 buffer状态
	reg [1:0]  l2_wr_buf_id      ;// 当前写入的buffer
	reg [1:0]  l2_rd_buf_id [0:3];// 当前使用的4个buffer（动态变化）
	reg        l2_buf_valid [0:3];// buffer有效标志
	reg [15:0] l2_buf_line_id [0:3];// buffer存储的源行号

	// L2状态定义
	localparam L2_EMPTY   = 2'b00 ;
	localparam L2_FILLING = 2'b01 ;
	localparam L2_VALID   = 2'b10 ;
	reg [1:0]  l2_state           ;

	//------------------------------------------------------------------------
	// L1 到 L2 数据传输状态机（双线性简化版）
	//------------------------------------------------------------------------
	// 状态定义
	localparam TRANS_IDLE  = 3'b000;
	localparam TRANS_WAIT  = 3'b001;
	localparam TRANS_READ  = 3'b010;
	localparam TRANS_WRITE = 3'b011;
	localparam TRANS_NEXT  = 3'b100;

	reg [2:0]  trans_state        ;
	reg [ADDR_WIDTH-1:0] trans_addr_cnt;
	reg [1:0]  l2_fill_cnt        ;  // 已填充的L2 buffer数

	// 状态机：将数据从L1搬到L2
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			trans_state     <= TRANS_IDLE;
			trans_addr_cnt  <= {ADDR_WIDTH{1'b0}};
			l2_fill_cnt     <= 2'd0;
			l2_rd_req       <= 1'b0;
			l1_rd_addr      <= {ADDR_WIDTH{1'b0}};
			l1_rd_switch    <= 1'b0;
			l2_wr_buf_id    <= 2'd0;
			l2_buf_valid[0] <= 1'b0;
			l2_buf_valid[1] <= 1'b0;
			l2_buf_valid[2] <= 1'b0;
			l2_buf_valid[3] <= 1'b0;
		end
		else begin
			l1_rd_switch <= 1'b0;

			case (trans_state)
				TRANS_IDLE: begin
					if (frame_start_pulse == 1'b1) begin
						trans_state <= TRANS_WAIT;
						l2_fill_cnt <= 2'd0;
					end
				end

				TRANS_WAIT: begin
					// 等待L1有数据
					if (l1_buf_status != 2'b00) begin
						trans_state    <= TRANS_READ;
						trans_addr_cnt <= {ADDR_WIDTH{1'b0}};
						l2_rd_req      <= 1'b1;
					end
				end

				TRANS_READ: begin
					if (trans_addr_cnt >= r_src_width - 1) begin
						trans_state    <= TRANS_WRITE;
						trans_addr_cnt <= {ADDR_WIDTH{1'b0}};
					end
					else begin
						trans_addr_cnt <= #U_DLY trans_addr_cnt + 1'b1;
						l1_rd_addr     <= #U_DLY trans_addr_cnt + 1'b1;
					end
				end

				TRANS_WRITE: begin
					if (l1_rd_data_valid == 1'b1) begin
						l2_v_buf[l2_wr_buf_id][trans_addr_cnt] <= #U_DLY l1_rd_data;

						if (trans_addr_cnt >= r_src_width - 1) begin
							trans_state <= TRANS_NEXT;
						end
						else begin
							trans_addr_cnt <= #U_DLY trans_addr_cnt + 1'b1;
						end
					end
				end

				TRANS_NEXT: begin
					l2_buf_valid[l2_wr_buf_id]   <= #U_DLY 1'b1;
					l2_buf_line_id[l2_wr_buf_id] <= #U_DLY l2_fill_cnt;

					l2_wr_buf_id <= #U_DLY l2_wr_buf_id + 1'b1;
					l2_fill_cnt  <= #U_DLY l2_fill_cnt + 1'b1;
					l1_rd_switch <= 1'b1;

					// 双线性：2行就绪即可开始，继续填充更多行
					trans_state <= TRANS_WAIT;
				end

				default: trans_state <= TRANS_IDLE;
			endcase
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第4部分：目标坐标生成 + 源坐标计算（clk_out域）
	//========================================================================
	reg [15:0] dst_y_cnt     ;
	reg [15:0] dst_x_cnt     ;
	reg        output_active ;

	// 源坐标（定点数）
	wire [COORD_BITS-1:0] coord_y      ;
	wire [INT_BITS-1:0]   coord_y_int  ;
	wire [FRAC_BITS-1:0]  coord_y_frac ;
	wire [COORD_BITS-1:0] coord_x      ;
	wire [INT_BITS-1:0]   coord_x_int  ;
	wire [FRAC_BITS-1:0]  coord_x_frac ;

	// 坐标计算实例化（复用现有模块）
	bilinear_coord_calc #(
		.INT_BITS (INT_BITS ),
		.FRAC_BITS(FRAC_BITS),
		.WEIGHT_BITS(FRAC_BITS)
	) u_coord_y (
		.clk          (clk_out        ),
		.rst_n        (rst_n_out      ),
		.dst_idx      (dst_y_cnt      ),
		.inv_scale    (r_inv_scale_y  ),
		.src_size     (r_src_height   ),
		.src_pos_int  (coord_y_int    ),
		.src_pos_frac (coord_y_frac   ),
		.valid        (coord_y_valid  )
	);

	bilinear_coord_calc #(
		.INT_BITS (INT_BITS ),
		.FRAC_BITS(FRAC_BITS),
		.WEIGHT_BITS(FRAC_BITS)
	) u_coord_x (
		.clk          (clk_out        ),
		.rst_n        (rst_n_out      ),
		.dst_idx      (dst_x_cnt      ),
		.inv_scale    (r_inv_scale_x  ),
		.src_size     (r_src_width    ),
		.src_pos_int  (coord_x_int    ),
		.src_pos_frac (coord_x_frac   ),
		.valid        (coord_x_valid  )
	);

	// 输出控制
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			dst_y_cnt     <= 16'd0;
			dst_x_cnt     <= 16'd0;
			output_active <= 1'b0;
		end
		else if (frame_start_pulse == 1'b1) begin
			dst_y_cnt     <= 16'd0;
			dst_x_cnt     <= 16'd0;
			output_active <= 1'b0;
		end
		else if (output_active == 1'b1 || (l2_fill_cnt >= 2'd2)) begin
			// 双线性：L2有2行即可开始
			output_active <= 1'b1;
			if (o_valid == 1'b1 && o_ready == 1'b1) begin
				if (dst_x_cnt >= r_dst_width - 1) begin
					dst_x_cnt <= 16'd0;
					if (dst_y_cnt >= r_dst_height - 1) begin
						dst_y_cnt     <= 16'd0;
						output_active <= 1'b0;
					end
					else begin
						dst_y_cnt <= #U_DLY dst_y_cnt + 1'b1;
					end
				end
				else begin
					dst_x_cnt <= #U_DLY dst_x_cnt + 1'b1;
				end
			end
		end
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第5部分：V-filter（双线性垂直插值）
	//========================================================================
	// 读取L2的两行（y0和y1），根据coord_y_frac做插值
	reg [DATA_WIDTH-1:0] v_pix_0      ;
	reg [DATA_WIDTH-1:0] v_pix_1      ;
	reg [DATA_WIDTH-1:0] v_pix_result ;
	reg                  v_pix_valid  ;

	// V-filter权重计算（双线性：2点）
	wire [FRAC_BITS-1:0] w_y1 = coord_y_frac;
	wire [FRAC_BITS-1:0] w_y0 = (1 << FRAC_BITS) - coord_y_frac;

	// 读取L2的同一列不同行
	always @(posedge clk_out) begin
		if (coord_y_valid == 1'b1) begin
			// 选择哪两行做V-filter（双线性用当前行和下一行）
			v_pix_0 <= #U_DLY l2_v_buf[coord_y_int[1:0]][coord_x_int];
			v_pix_1 <= #U_DLY l2_v_buf[(coord_y_int[1:0] + 1'b1)][coord_x_int];
		end
	end

	// 双线性垂直插值：v_pix = v_pix_0 * (1-dy) + v_pix_1 * dy
	always @(posedge clk_out) begin
		v_pix_result <= #U_DLY (v_pix_0 * w_y0 + v_pix_1 * w_y1) >> FRAC_BITS;
		v_pix_valid  <= #U_DLY coord_y_valid;
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第6部分：H-filter（双线性水平插值）
	//========================================================================
	// H-filter像素寄存（双线性用h_pix_0/1，双三次可扩展h_pix_0/1/2/3）
	reg [DATA_WIDTH-1:0] h_pix_0     ;// 最新像素（x0）
	reg [DATA_WIDTH-1:0] h_pix_1     ;// 前一个像素（x1）
	reg [DATA_WIDTH-1:0] h_pix_result;
	reg                  h_pix_valid ;

	// H-filter权重（双线性：2点）
	wire [FRAC_BITS-1:0] w_x1 = coord_x_frac;
	wire [FRAC_BITS-1:0] w_x0 = (1 << FRAC_BITS) - coord_x_frac;

	// 移位寄存器：0->1，新值->0
	always @(posedge clk_out) begin
		if (v_pix_valid == 1'b1) begin
			h_pix_1 <= #U_DLY h_pix_0;
			h_pix_0 <= #U_DLY v_pix_result;
		end
	end

	// 双线性水平插值：h_pix_0*(1-dx) + h_pix_1*dx
	always @(posedge clk_out) begin
		h_pix_result <= #U_DLY (h_pix_0 * w_x0 + h_pix_1 * w_x1) >> FRAC_BITS;
		h_pix_valid  <= #U_DLY v_pix_valid;
	end

	//------------------------------------------------------------------------
	//========================================================================
	// 第7部分：输出FIFO（1行缓冲，简化版直接输出）
	//========================================================================
	// TODO: 完善FIFO实现，当前直接输出

	//------------------------------------------------------------------------
	//========================================================================
	// 第8部分：输出接口
	//========================================================================
	always @(posedge clk_out or negedge rst_n_out) begin
		if (rst_n_out == 1'b0) begin
			o_valid       <= 1'b0;
			o_data        <= {DATA_WIDTH{1'b0}};
			o_last        <= 1'b0;
			o_frame_start <= 1'b0;
		end
		else begin
			o_valid       <= #U_DLY h_pix_valid;
			o_data        <= #U_DLY h_pix_result;
			o_last        <= #U_DLY (dst_x_cnt >= r_dst_width - 1) && h_pix_valid;
			o_frame_start <= #U_DLY (dst_y_cnt == 16'd0 && dst_x_cnt == 16'd0) && h_pix_valid;
		end
	end

endmodule
