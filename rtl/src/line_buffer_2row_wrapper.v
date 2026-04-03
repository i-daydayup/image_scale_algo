//============================================================================
// 模块名称    : line_buffer_2row_wrapper
// 功能描述    : 2行输入缓冲Wrapper，支持DDR预取和时序收敛
// 架构参考    : rtl/src/bilinear_scaler_vh_arch.md
// 设计原则    : 器件无关封装，内部打拍确保时序收敛
//
// 关键特性：
//   - 2行ping-pong缓冲，提高DDR突发效率
//   - 输入/输出各打一拍，方便时序收敛
//   - 标准AXI-S风格接口，方便集成
//   - 支持Generic/Xilinx/Lattice实现（参数化选择）
//
// 版本历史    : v1.0 - Generic实现
//============================================================================

`timescale 1ns / 1ps

module line_buffer_2row_wrapper #(
    parameter DATA_WIDTH = 8,
    parameter MAX_WIDTH  = 4096,
    parameter ADDR_WIDTH = $clog2(MAX_WIDTH),
    parameter VENDOR     = "GENERIC"  // "GENERIC", "XILINX", "LATTICE"
)(
    //------------------------------------------------------------------------
    // 系统接口
    //------------------------------------------------------------------------
    input  wire                  clk,
    input  wire                  rst_n,
    
    //------------------------------------------------------------------------
    // 写端口（DDR/上游输入）
    //------------------------------------------------------------------------
    input  wire                  wr_valid,       //I1,写数据有效
    input  wire [ADDR_WIDTH-1:0] wr_addr,        //Ix,写地址
    input  wire [DATA_WIDTH-1:0] wr_data,        //Ix,写数据
    input  wire                  wr_last,        //I1,行结束
    output reg                   wr_ready,        //O1,可接收（反压）
    
    //------------------------------------------------------------------------
    // 读端口（下游V-filter消费）
    //------------------------------------------------------------------------
    input  wire                  rd_valid,       //I1,读使能
    input  wire [ADDR_WIDTH-1:0] rd_addr,        //Ix,读地址
    output reg  [DATA_WIDTH-1:0] rd_data,        //Ox,读数据（打拍输出）
    output reg                   rd_data_valid,  //O1,读数据有效
    input  wire                  rd_switch,      //I1,切换buffer（读完一行）
    output wire [1:0]            buf_status      //O2,00=空,01=满1,10=满2,11=全满
);
    
    //------------------------------------------------------------------------
    // 仿真延时参数
    //------------------------------------------------------------------------
    localparam U_DLY = 1;
    
    //------------------------------------------------------------------------
    // 状态定义
    //------------------------------------------------------------------------
    localparam BUF_EMPTY    = 2'b00;  // 两空
    localparam BUF_HALF     = 2'b01;  // 1行满
    localparam BUF_FULL     = 2'b10;  // 2行满
    
    //------------------------------------------------------------------------
    // 内部信号
    //------------------------------------------------------------------------
    // 写端口打拍（时序收敛）
    reg                  wr_valid_d1;
    reg [ADDR_WIDTH-1:0] wr_addr_d1;
    reg [DATA_WIDTH-1:0] wr_data_d1;
    reg                  wr_last_d1;
    
    // 2行存储器（Generic实现）
    reg [DATA_WIDTH-1:0] mem0 [0:MAX_WIDTH-1];
    reg [DATA_WIDTH-1:0] mem1 [0:MAX_WIDTH-1];
    
    // 写控制
    reg                  wr_buf_sel;        // 当前写入的buffer (0/1)
    reg                  wr_buf_sel_d1;
    reg [ADDR_WIDTH-1:0] wr_cnt;            // 写计数（检测行结束）
    reg                  wr_buf0_full;      // buffer0满标志
    reg                  wr_buf1_full;      // buffer1满标志
    
    // 读控制
    reg                  rd_buf_sel;        // 当前读取的buffer (0/1)
    reg [DATA_WIDTH-1:0] rd_data_raw;       // 读数据原始（未打拍）
    reg                  rd_buf0_empty;     // buffer0空标志（已读完）
    reg                  rd_buf1_empty;     // buffer1空标志
    
    //------------------------------------------------------------------------
    // 写端口：输入打拍
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            wr_valid_d1 <= 1'b0;
            wr_addr_d1  <= {ADDR_WIDTH{1'b0}};
            wr_data_d1  <= {DATA_WIDTH{1'b0}};
            wr_last_d1  <= 1'b0;
            wr_buf_sel_d1 <= 1'b0;
        end else begin
            wr_valid_d1 <= #U_DLY wr_valid;
            wr_addr_d1  <= #U_DLY wr_addr;
            wr_data_d1  <= #U_DLY wr_data;
            wr_last_d1  <= #U_DLY wr_last;
            wr_buf_sel_d1 <= #U_DLY wr_buf_sel;
        end
    end
    
    //------------------------------------------------------------------------
    // 写端口：状态机和存储器写入
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            wr_buf_sel     <= 1'b0;
            wr_cnt         <= {ADDR_WIDTH{1'b0}};
            wr_buf0_full   <= 1'b0;
            wr_buf1_full   <= 1'b0;
        end else begin
            if (wr_valid_d1 == 1'b1) begin
                // 写入存储器
                if (wr_buf_sel_d1 == 1'b0) begin
                    mem0[wr_addr_d1] <= #U_DLY wr_data_d1;
                end else begin
                    mem1[wr_addr_d1] <= #U_DLY wr_data_d1;
                end
                
                // 行结束检测（wr_last_d1）或地址溢出
                if (wr_last_d1 == 1'b1 || wr_addr_d1 >= MAX_WIDTH-1) begin
                    // 标记当前buffer已满
                    if (wr_buf_sel_d1 == 1'b0) begin
                        wr_buf0_full <= #U_DLY 1'b1;
                    end else begin
                        wr_buf1_full <= #U_DLY 1'b1;
                    end
                    
                    // 切换到另一个buffer
                    wr_buf_sel <= #U_DLY ~wr_buf_sel;
                end
            end
            
            // 当buffer被读完后，清除满标志
            if (rd_buf0_empty == 1'b1) begin
                wr_buf0_full <= #U_DLY 1'b0;
            end
            if (rd_buf1_empty == 1'b1) begin
                wr_buf1_full <= #U_DLY 1'b0;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // 写端口：反压控制
    // 只有当当前写入的buffer未满时才拉高wr_ready
    // 如果两个buffer都满了，反压上游（wr_ready=0）
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            wr_ready <= 1'b0;
        end else begin
            // 当前要写入的buffer是否已满？
            if (wr_buf_sel == 1'b0) begin
                wr_ready <= #U_DLY ~wr_buf0_full;
            end else begin
                wr_ready <= #U_DLY ~wr_buf1_full;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // 读端口：存储器读取（组合逻辑）
    //------------------------------------------------------------------------
    always @(*) begin
        if (rd_buf_sel == 1'b0) begin
            rd_data_raw = mem0[rd_addr];
        end else begin
            rd_data_raw = mem1[rd_addr];
        end
    end
    
    //------------------------------------------------------------------------
    // 读端口：输出打拍（时序收敛）
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            rd_data       <= {DATA_WIDTH{1'b0}};
            rd_data_valid <= 1'b0;
        end else begin
            if (rd_valid == 1'b1) begin
                rd_data       <= #U_DLY rd_data_raw;
                rd_data_valid <= #U_DLY 1'b1;
            end else begin
                rd_data_valid <= #U_DLY 1'b0;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // 读端口：buffer切换控制
    //------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) begin
            rd_buf_sel     <= 1'b0;
            rd_buf0_empty  <= 1'b0;
            rd_buf1_empty  <= 1'b0;
        end else begin
            if (rd_switch == 1'b1) begin
                // 切换读buffer
                rd_buf_sel <= #U_DLY ~rd_buf_sel;
                
                // 标记刚读完的buffer为空
                if (rd_buf_sel == 1'b0) begin
                    rd_buf0_empty <= #U_DLY 1'b1;
                end else begin
                    rd_buf1_empty <= #U_DLY 1'b1;
                end
            end else begin
                // 清除空标志（由写端口重新填满后）
                if (wr_buf0_full == 1'b1) begin
                    rd_buf0_empty <= #U_DLY 1'b0;
                end
                if (wr_buf1_full == 1'b1) begin
                    rd_buf1_empty <= #U_DLY 1'b0;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // 状态输出
    //------------------------------------------------------------------------
    assign buf_status = {wr_buf1_full, wr_buf0_full};
    
    //------------------------------------------------------------------------
    // 断言检查（仿真时使用）
    //------------------------------------------------------------------------
    `ifdef SIMULATION
    // 检查写溢出（写满后还写）
    always @(posedge clk) begin
        if (wr_valid == 1'b1 && wr_ready == 1'b0) begin
            $display("[WARNING] line_buffer_2row_wrapper: Write overflow at time %0t", $time);
        end
    end
    
    // 检查读空（读空buffer）
    always @(posedge clk) begin
        if (rd_valid == 1'b1) begin
            if (rd_buf_sel == 1'b0 && wr_buf0_full == 1'b0) begin
                $display("[WARNING] line_buffer_2row_wrapper: Read empty buffer 0 at time %0t", $time);
            end
            if (rd_buf_sel == 1'b1 && wr_buf1_full == 1'b0) begin
                $display("[WARNING] line_buffer_2row_wrapper: Read empty buffer 1 at time %0t", $time);
            end
        end
    end
    `endif

endmodule
