//============================================================================
// 模块名称    : bilinear_interp
// 功能描述    : 双线性插值计算核心（定点实现）
//
// 插值公式（定点化）：
//     w00 = (1-dx) * (1-dy) >> w
//     w10 = dx * (1-dy) >> w
//     w01 = (1-dx) * dy >> w
//     w11 = dx * dy >> w
//     result = (P00*w00 + P10*w10 + P01*w01 + P11*w11) >> w
//
// 时序说明：
//     - 输入到输出延迟：3 拍
//     - 第1拍：计算 (1-dx), (1-dy) 和 4个权重
//     - 第2拍：权重右移，像素乘权重
//     - 第3拍：累加并右移，饱和处理
//
// 对应Python  : python/algorithms/bilinear_fixed.py::_bilinear_interpolate_fixed
// 验证状态    : 验证中
//============================================================================

	imescale 1ns / 1ps

module bilinear_interp #(
	parameter DATA_WIDTH  = 8,   // 像素数据位宽
	parameter WEIGHT_BITS = 8    // 权重小数位宽 w (Q0.w)
)(
	input  wire                      clk,
	input  wire                      rst_n,

	input  wire [DATA_WIDTH-1:0]         p00,       // 左上像素 (y0, x0)
	input  wire [DATA_WIDTH-1:0]         p10,       // 右上像素 (y0, x1)
	input  wire [DATA_WIDTH-1:0]         p01,       // 左下像素 (y1, x0)
	input  wire [DATA_WIDTH-1:0]         p11,       // 右下像素 (y1, x1)
	input  wire [WEIGHT_BITS-1:0]        dx,        // X方向小数部分 (Q0.w)
	input  wire [WEIGHT_BITS-1:0]        dy,        // Y方向小数部分 (Q0.w)
	input  wire                          valid_in,  // 输入有效

	output reg  [DATA_WIDTH-1:0]         result,    // 插值结果
	output reg                           valid_out  // 输出有效
);

	//------------------------------------------------------------------------
	// 参数定义
	//------------------------------------------------------------------------
	localparam SCALE_WEIGHT = 1 << WEIGHT_BITS;       // 2^w
	localparam MULT_BITS    = DATA_WIDTH + WEIGHT_BITS;  // 乘法结果位宽
	localparam SUM_BITS     = MULT_BITS + 2;             // 累加结果位宽 (4项相加)

	//------------------------------------------------------------------------
	// 流水线第1拍：计算 (1-dx), (1-dy) 和权重
	//------------------------------------------------------------------------
	reg [WEIGHT_BITS-1:0]    one_minus_dx;
	reg [WEIGHT_BITS-1:0]    one_minus_dy;
	reg [WEIGHT_BITS-1:0]    dx_reg;
	reg [WEIGHT_BITS-1:0]    dy_reg;
	
	// 权重寄存器
	reg [2*WEIGHT_BITS-1:0]  w00_mult;
	reg [2*WEIGHT_BITS-1:0]  w10_mult;
	reg [2*WEIGHT_BITS-1:0]  w01_mult;
	reg [2*WEIGHT_BITS-1:0]  w11_mult;
	
	// 像素寄存器
	reg [DATA_WIDTH-1:0]     p00_reg;
	reg [DATA_WIDTH-1:0]     p10_reg;
	reg [DATA_WIDTH-1:0]     p01_reg;
	reg [DATA_WIDTH-1:0]     p11_reg;
	reg                      valid_d1;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			one_minus_dx <= 0;
			one_minus_dy <= 0;
			dx_reg       <= 0;
			dy_reg       <= 0;
			w00_mult     <= 0;
			w10_mult     <= 0;
			w01_mult     <= 0;
			w11_mult     <= 0;
			p00_reg      <= 0;
			p10_reg      <= 0;
			p01_reg      <= 0;
			p11_reg      <= 0;
			valid_d1     <= 0;
		end
		else begin
			// 计算 (1-dx) 和 (1-dy)
			one_minus_dx <= SCALE_WEIGHT - dx;
			one_minus_dy <= SCALE_WEIGHT - dy;
			dx_reg       <= dx;
			dy_reg       <= dy;
			
			// 计算权重乘法（完整精度）
			w00_mult <= (SCALE_WEIGHT - dx) * (SCALE_WEIGHT - dy);
			w10_mult <= dx * (SCALE_WEIGHT - dy);
			w01_mult <= (SCALE_WEIGHT - dx) * dy;
			w11_mult <= dx * dy;
			
			// 寄存像素值
			p00_reg <= p00;
			p10_reg <= p10;
			p01_reg <= p01;
			p11_reg <= p11;
			
			valid_d1 <= valid_in;
		end
	end

	//------------------------------------------------------------------------
	// 流水线第2拍：权重右移，像素乘权重，累加
	//------------------------------------------------------------------------
	reg [WEIGHT_BITS-1:0]    w00;
	reg [WEIGHT_BITS-1:0]    w10;
	reg [WEIGHT_BITS-1:0]    w01;
	reg [WEIGHT_BITS-1:0]    w11;
	reg [MULT_BITS-1:0]      p00_w00;
	reg [MULT_BITS-1:0]      p10_w10;
	reg [MULT_BITS-1:0]      p01_w01;
	reg [MULT_BITS-1:0]      p11_w11;
	reg                      valid_d2;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			w00      <= 0;
			w10      <= 0;
			w01      <= 0;
			w11      <= 0;
			p00_w00  <= 0;
			p10_w10  <= 0;
			p01_w01  <= 0;
			p11_w11  <= 0;
			valid_d2 <= 0;
		end
		else begin
			// 权重右移 WEIGHT_BITS 位 (除以 2^w)
			w00 <= w00_mult >> WEIGHT_BITS;
			w10 <= w10_mult >> WEIGHT_BITS;
			w01 <= w01_mult >> WEIGHT_BITS;
			w11 <= w11_mult >> WEIGHT_BITS;
			
			// 像素乘权重
			p00_w00 <= p00_reg * (w00_mult >> WEIGHT_BITS);
			p10_w10 <= p10_reg * (w10_mult >> WEIGHT_BITS);
			p01_w01 <= p01_reg * (w01_mult >> WEIGHT_BITS);
			p11_w11 <= p11_reg * (w11_mult >> WEIGHT_BITS);
			
			valid_d2 <= valid_d1;
		end
	end

	//------------------------------------------------------------------------
	// 流水线第3拍：累加并右移，饱和处理
	//------------------------------------------------------------------------
	reg [SUM_BITS-1:0]       sum;
	reg [DATA_WIDTH-1:0]     result_raw;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			sum       <= 0;
			result    <= 0;
			valid_out <= 0;
		end
		else begin
			// 累加4个乘积
			sum <= p00_w00 + p10_w10 + p01_w01 + p11_w11;
			
			// 右移 WEIGHT_BITS 位，并饱和到 0-255
			result_raw <= sum >> WEIGHT_BITS;
			
			// 饱和处理
			if ((sum >> WEIGHT_BITS) > 255)
				result <= 255;
			else
				result <= sum[WEIGHT_BITS+DATA_WIDTH-1:WEIGHT_BITS];
			
			valid_out <= valid_d2;
		end
	end

endmodule
