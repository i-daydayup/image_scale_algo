//============================================================================
// 模块名称    : bilinear_coord_calc
// 功能描述    : 双线性插值坐标计算器（定点实现）
//             计算目标像素对应的原图坐标
//
// 公式推导：
//     src_pos = (dst_idx + 0.5) / scale - 0.5
//     src_pos_fixed = (((2*dst_idx + 1) * inv_scale_fixed) >> 1) - (1 << (n-1))
//
// 时序说明：
//     - 输入到输出延迟：4 拍
//     - 第1拍：计算 (2*dst_idx + 1)
//     - 第2拍：乘法 (2*dst_idx + 1) * inv_scale
//     - 第3拍：右移1位，减0.5，边界保护
//     - 第4拍：分离整数和小数部分
//
// 对应Python  : python/algorithms/bilinear_fixed.py::_compute_src_coord_fixed
// 验证状态    : 验证中
//============================================================================

`timescale 1ns / 1ps

module bilinear_coord_calc #(
	parameter INT_BITS    = 12  ,// 坐标整数位宽 m
	parameter FRAC_BITS   = 8   ,// 坐标小数位宽 n
	parameter WEIGHT_BITS = 8    // 权重小数位宽 w
)(
	input  wire                      clk         ,//I1,
	input  wire                      rst_n       ,//I1,

	input  wire [15:0]                   dst_idx      ,//I16,目标像素索引
	input  wire [INT_BITS+FRAC_BITS-1:0] inv_scale    ,//Ix,逆缩放比例 (Qm.n)
	input  wire [15:0]                   src_size     ,//I16,源图像尺寸

	output reg  [15:0]                   src_pos_int  ,//O16,源图像素整数坐标
	output reg  [WEIGHT_BITS-1:0]        src_pos_frac ,//Ox,源图像素小数部分 (Q0.w)
	output reg                           valid         //O1,输出有效
);

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 参数定义
	//------------------------------------------------------------------------
	localparam COORD_BITS  = INT_BITS + FRAC_BITS                                       ;
	localparam HALF_N      = FRAC_BITS > 0 ? (1 << (FRAC_BITS - 1)) : 0                 ;// 0.5 定点值
	localparam FRAC_MASK   = (1 << FRAC_BITS) - 1                                       ;
	localparam WEIGHT_MASK = (1 << WEIGHT_BITS) - 1                                     ;

	//------------------------------------------------------------------------
	// 流水线第1拍：计算 (2*dst_idx + 1)
	//------------------------------------------------------------------------
	reg [16:0]               mult_a                       ;
	reg [COORD_BITS-1:0]     mult_b                       ;
	reg [15:0]               src_size_d1                  ;
	reg                      valid_d1                     ;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			mult_a      <= 0;
			mult_b      <= 0;
			src_size_d1 <= 0;
			valid_d1    <= 0;
		end
		else begin
			mult_a      <= #U_DLY {dst_idx, 1'b1} + 1;  // 2*dst_idx + 1
			mult_b      <= #U_DLY inv_scale;
			src_size_d1 <= #U_DLY src_size;
			valid_d1    <= #U_DLY 1;
		end
	end

	//------------------------------------------------------------------------
	// 流水线第2拍：乘法 (2*dst_idx + 1) * inv_scale
	//------------------------------------------------------------------------
	// 注意：乘法结果位宽为 COORD_BITS + 17
	reg [COORD_BITS+16:0]    mult_result                  ;
	reg [15:0]               src_size_d2                  ;
	reg                      valid_d2                     ;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			mult_result <= 0;
			src_size_d2 <= 0;
			valid_d2    <= 0;
		end
		else begin
			mult_result <= #U_DLY mult_a * mult_b;
			src_size_d2 <= #U_DLY src_size_d1;
			valid_d2    <= #U_DLY valid_d1;
		end
	end

	//------------------------------------------------------------------------
	// 流水线第3拍：右移1位，减0.5，边界保护
	//------------------------------------------------------------------------
	reg [COORD_BITS-1:0]     src_pos_fixed                ;
	reg [COORD_BITS-1:0]     src_pos_clamped              ;
	reg                      valid_d3                     ;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			src_pos_fixed   <= 0;
			src_pos_clamped <= 0;
			valid_d3        <= 0;
		end
		else begin
			// mult_result >> 1 - 0.5
			src_pos_fixed <= #U_DLY (mult_result >> 1) - HALF_N;

			// 边界保护：限制在 [0, (src_size-1) << FRAC_BITS]
			if (src_pos_fixed[COORD_BITS-1] == 1'b1 || src_pos_fixed > ((src_size_d2 - 1) << FRAC_BITS))
				src_pos_clamped <= (src_size_d2 - 1) << FRAC_BITS;
			else
				src_pos_clamped <= src_pos_fixed;

			valid_d3 <= #U_DLY valid_d2;
		end
	end

	//------------------------------------------------------------------------
	// 流水线第4拍：分离整数和小数部分
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			src_pos_int  <= 0;
			src_pos_frac <= 0;
			valid        <= 0;
		end
		else begin
			// 整数部分：右移 FRAC_BITS 位
			src_pos_int <= #U_DLY src_pos_clamped >> FRAC_BITS;

			// 小数部分：转换到 Q0.w 格式
			if (WEIGHT_BITS == FRAC_BITS)
				src_pos_frac <= src_pos_clamped[FRAC_BITS-1:0];
			else if (WEIGHT_BITS > FRAC_BITS)
				src_pos_frac <= src_pos_clamped[FRAC_BITS-1:0] << (WEIGHT_BITS - FRAC_BITS);
			else
				src_pos_frac <= src_pos_clamped[FRAC_BITS-1:FRAC_BITS-WEIGHT_BITS];

			valid <= #U_DLY valid_d3;
		end
	end

endmodule
