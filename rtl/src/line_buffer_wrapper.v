//============================================================================
// 模块名称    : line_buffer_wrapper
// 功能描述    : 单行双口RAM Wrapper（器件无关封装）
// 架构参考    : rtl/src/bilinear_scaler_vh_arch.md
// 设计原则    : 器件无关封装，内部打拍确保时序收敛
//
// 关键特性：
//   - 单行存储，双口访问（独立读写）
//   - 输入/输出各打一拍，方便时序收敛
//   - 支持Generic/Xilinx/Lattice实现（参数化选择）
//   - 上层可例化多个，实现多行缓冲
//
// 版本历史    : v1.0 - Generic实现
//============================================================================

`timescale 1ns / 1ps

module line_buffer_wrapper #(
	parameter DATA_WIDTH = 8                    ,
	parameter MAX_WIDTH  = 4096                 ,
	parameter ADDR_WIDTH = $clog2(MAX_WIDTH)    ,
	parameter VENDOR     = "GENERIC"             // "GENERIC", "XILINX", "LATTICE"
)(
	//------------------------------------------------------------------------
	// 系统接口
	//------------------------------------------------------------------------
	input  wire                  clk            ,//I1,时钟
	input  wire                  rst_n          ,//I1,复位

	//------------------------------------------------------------------------
	// 写端口
	//------------------------------------------------------------------------
	input  wire                  wr_en          ,//I1,写使能
	input  wire [ADDR_WIDTH-1:0] wr_addr        ,//Ix,写地址
	input  wire [DATA_WIDTH-1:0] wr_data        ,//Ix,写数据
	output reg                   wr_full         ,//O1,写满（可选，用于反压）

	//------------------------------------------------------------------------
	// 读端口
	//------------------------------------------------------------------------
	input  wire                  rd_en          ,//I1,读使能
	input  wire [ADDR_WIDTH-1:0] rd_addr        ,//Ix,读地址
	output reg  [DATA_WIDTH-1:0] rd_data        ,//Ox,读数据（打拍输出）
	output reg                   rd_data_valid  ,//O1,读数据有效
	output wire [ADDR_WIDTH:0]   datacnt         //Ox+1,当前写入的数据计数（读时钟域可见）
);

	//------------------------------------------------------------------------
	// 仿真延时参数
	//------------------------------------------------------------------------
	localparam U_DLY = 1;

	//------------------------------------------------------------------------
	// 存储器实例（Generic实现）
	//------------------------------------------------------------------------
	reg [DATA_WIDTH-1:0] mem [0:MAX_WIDTH-1];

	//------------------------------------------------------------------------
	// 写端口：输入打拍 + 写入
	//------------------------------------------------------------------------
	reg                  wr_en_d1   ;
	reg [ADDR_WIDTH-1:0] wr_addr_d1 ;
	reg [DATA_WIDTH-1:0] wr_data_d1 ;

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			wr_en_d1   <= 1'b0;
			wr_addr_d1 <= {ADDR_WIDTH{1'b0}};
			wr_data_d1 <= {DATA_WIDTH{1'b0}};
		end
		else begin
			wr_en_d1   <= #U_DLY wr_en;
			wr_addr_d1 <= #U_DLY wr_addr;
			wr_data_d1 <= #U_DLY wr_data;
		end
	end

	// 写入存储器
	always @(posedge clk) begin
		if (wr_en_d1 == 1'b1) begin
			mem[wr_addr_d1] <= #U_DLY wr_data_d1;
		end
	end

	//------------------------------------------------------------------------
	// 读端口：组合读 + 输出打拍
	//------------------------------------------------------------------------
	reg [DATA_WIDTH-1:0] rd_data_raw;

	always @(*) begin
		rd_data_raw = mem[rd_addr];
	end

	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			rd_data       <= {DATA_WIDTH{1'b0}};
			rd_data_valid <= 1'b0;
		end
		else begin
			if (rd_en == 1'b1) begin
				rd_data       <= #U_DLY rd_data_raw;
				rd_data_valid <= #U_DLY 1'b1;
			end
			else begin
				rd_data_valid <= #U_DLY 1'b0;
			end
		end
	end

	//------------------------------------------------------------------------
	// 写满标志（将满检测：地址达到 MAX_WIDTH-2 时置1）
	//------------------------------------------------------------------------
	always @(posedge clk or negedge rst_n) begin
		if (rst_n == 1'b0) begin
			wr_full <= 1'b0;
		end
		else begin
			// 将满检测：地址达到 MAX_WIDTH-2，给上层一个周期响应
			wr_full <= #U_DLY (wr_addr >= (MAX_WIDTH-2));
		end
	end

endmodule
