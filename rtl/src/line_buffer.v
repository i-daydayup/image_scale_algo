//============================================================================
// 模块名称    : line_buffer
// 功能描述    : 双行缓冲模块（用于双线性插值）
//             存储相邻两行像素，支持同时读取两行数据
//
// 接口说明：
//     - 写入：单端口，1拍延迟
//     - 读出：单端口，1拍延迟（与写入独立）
//     - 行交换：通过 swap 信号切换当前行和前一行
//
// 时序说明：
//     - 写入到可读延迟：2 拍（写入1拍 + 寄存1拍）
//     - 读出请求到数据有效：1 拍
//
// 存储器实现：
//     - 使用标准 Verilog 数组（器件无关）
//     - 综合工具会自动推断为 RAM 或寄存器
//
// 对应Python  : python/algorithms/bilinear_fixed.py
// 验证状态    : 验证中
//============================================================================

`timescale 1ns / 1ps

module line_buffer #(
	parameter DATA_WIDTH = 8,    // 像素数据位宽
	parameter LINE_WIDTH = 1920, // 最大行宽度
	parameter LINE_COUNT = 2     // 缓冲行数 (固定为2)
)(
	input  wire                          clk	,//I1,
	input  wire                          rst_n	,//I1,

	// 写入接口
	input  wire                          wr_en	,//I1,写使能
	input  wire [$clog2(LINE_WIDTH)-1:0] wr_addr	,//Ix,写地址
	input  wire [DATA_WIDTH-1:0]         wr_data	,//Ix,写数据

	// 读出接口
	input  wire                          rd_en	,//I1,读使能
	input  wire [$clog2(LINE_WIDTH)-1:0] rd_addr	,//Ix,读地址
	output reg  [DATA_WIDTH-1:0]         rd_data_0	,//Ox,当前行读数据
	output reg  [DATA_WIDTH-1:0]         rd_data_1	,//Ox,前一行读数据
	output reg                           rd_valid	,//O1,读数据有效

	// 控制接口
	input  wire                          swap	,//I1,行交换信号（高电平有效）
);

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 存储器定义
	//------------------------------------------------------------------------
	// 两行缓冲，使用标准 Verilog 数组
	reg [DATA_WIDTH-1:0] line_buf_0 [0:LINE_WIDTH-1];         // 当前行
	reg [DATA_WIDTH-1:0] line_buf_1 [0:LINE_WIDTH-1];         // 前一行

	// 当前写入的行选择
	reg                          wr_sel;

	//------------------------------------------------------------------------
	// 写入控制
	// 时序：1拍延迟
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			wr_sel <= 0;
		end
		else if (swap == 1'b1) begin
			wr_sel <= #U_DLY ~wr_sel;
		end
	end

	// 写入操作（异步复位，同步写入）
	always @(posedge clk) begin
		if (wr_en == 1'b1) begin
			if (wr_sel == 1'b1)
				line_buf_1[wr_addr] <= #U_DLY wr_data;
			else
				line_buf_0[wr_addr] <= #U_DLY wr_data;
		end
	end

	//------------------------------------------------------------------------
	// 读出控制
	// 时序：1拍延迟
	//------------------------------------------------------------------------
	reg                          rd_sel;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			rd_sel <= 0;
		end
		else if (swap == 1'b1) begin
			rd_sel <= #U_DLY ~rd_sel;
		end
	end

	// 读地址寄存（用于对齐读出数据）
	reg [$clog2(LINE_WIDTH)-1:0] rd_addr_reg;
	reg                          rd_en_reg;

	always @(posedge clk) begin
		rd_addr_reg <= #U_DLY rd_addr;
		rd_en_reg   <= #U_DLY rd_en;
	end

	// 读出数据（根据当前swap状态选择）
	// 当 wr_sel=0 时：buf_0是当前行，buf_1是前一行
	// 当 wr_sel=1 时：buf_1是当前行，buf_0是前一行
	always @(posedge clk) begin
		if (rd_sel == 1'b1) begin
			rd_data_0 <= #U_DLY line_buf_1[rd_addr_reg];  // 当前行
			rd_data_1 <= #U_DLY line_buf_0[rd_addr_reg];  // 前一行
		end
		else begin
			rd_data_0 <= #U_DLY line_buf_0[rd_addr_reg];  // 当前行
			rd_data_1 <= #U_DLY line_buf_1[rd_addr_reg];  // 前一行
		end
		rd_valid <= #U_DLY rd_en_reg;
	end

endmodule
