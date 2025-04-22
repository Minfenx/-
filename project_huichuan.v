`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/12/17 17:40:19
// Design Name: 
// Module Name: project_huichuan
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


    module project_huichuan(
	input  wire         i_sysclk,   
	input  wire         i_sysrst_n,
	input  wire         i_1k_trigger,				 // DSP 数据
	/*
		开启/关闭存储：
		0x13：开启循环存储；
		0x33：开启不循环存储；
		0x00：关闭存储；
	*/
	input  wire [7 : 0] i_huichuan_mode, 			 // 回传模式   0x00：实时传输模 存储模式
	input  wire [7 : 0] i_huichuan_save_enable,      // 0x00启存   0x11:关闭存储
	input  wire [7 : 0] i_huichuan_enable,  		 // 0x00:不回       
    input  wire [7 : 0] i_huichuan_rate, 
	input  wire [7 : 0] i_huichuan_save_rate,        // 1HZ 4HZ 100HZ 
	input  wire [7 : 0] i_huichuan_img_enable,       
	
	input  wire         i_time_trigger,
	input  wire         i_guidao_trigger,
	input  wire         i_taiyang_trigger,
	input  wire         i_zitai_trigger,
	
	output wire [2 : 0] o_time_addr,
	input  wire [7 : 0] i_time_data,
	output reg  [4 : 0] o_taiyang_addr,
	input  wire [7 : 0] i_taiyang_data,
	output reg  [6 : 0] o_zitai_addr,
	input  wire [7 : 0] i_zitai_data,
	output reg  [7 : 0] o_guidao_addr,
	input  wire [7 : 0] i_guidao_data,

	output wire [6 : 0] o_dsp_addr,
	input  wire [7 : 0] i_dsp_data,
	
	input  wire         i_xj_clk,
	input  wire         i_xj_data,
	
	input  wire         i_txj_clk,
	input  wire         i_txj_data,
	
	output wire         o_txj_clk,
	output reg          o_txj_data,
	
	// 
	input  wire [7 : 0] i_yc_request_cnt,
	input  wire [7 : 0] i_last_cmd,
	input  wire [7 : 0] i_err_cnt,
	input  wire [15: 0] i_project_guidao_sel,
	input  wire [7 : 0] i_project_xm_sel,
	input  wire [7 : 0] i_dsp_1k_cnt
	);
	
	localparam   TXJ_HUICHUAN_MODE_REALTIME = 8'h00, TXJ_HUICHUAN_MODE_SAVE = 8'h11, TXJ_HUICHUAN_MODE_SAVE1 = 8'h03; 
	
	reg [7 : 0]  mem_broadcast_time[5 : 0];
	reg [2 : 0]  reg_time_addr_savetime;
	assign o_time_addr = reg_time_addr_savetime;	

	reg [6 : 0]  reg_dsp_addr_save;
	reg [6 : 0]  reg_dsp_addr_real;
	assign o_dsp_addr = (i_huichuan_mode == TXJ_HUICHUAN_MODE_REALTIME) ? reg_dsp_addr_real : reg_dsp_addr_save;

	/*===================================================================================================================*/
	//    相机存储模块开始
	/* == 相机时钟 == */
	wire wire_xj_neg_edge;
	BUFG BUGXJ(
		.I (i_xj_clk),
		.O (wire_xj_neg_edge)
	);
	
	/* == 图像一帧的存储 == */
	reg          reg_img_recvframe_write_en;
	reg [9 : 0]  reg_img_recvframe_write_addr;
    reg [7 : 0]  reg_img_recvframe_write_data;
	reg [9 : 0]  reg_img_recvframe_read_addr;	
    wire[7 : 0]  reg_img_recvframe_read_data;
	reg          reg_img_recvframe_read_en;		
	blk_mem_img_frame blk_mem_img_frame_inst (
		.clka  (~wire_xj_neg_edge),
		.addra (reg_img_recvframe_write_addr),
		.dina  (reg_img_recvframe_write_data),
		.wea   (reg_img_recvframe_write_en),
		.ena   (reg_img_recvframe_write_en),
		
		.clkb  (i_sysclk),
		.addrb (reg_img_recvframe_read_addr),
		.doutb (reg_img_recvframe_read_data),
		.enb   (reg_img_recvframe_read_en)
	);
	
	/* == 相机数据采集模块, 用于接收相机数据、 解析，并慢缓冲到 BRAM 中去 == */
	reg [23: 0] reg_xj_header;
	reg [15: 0] reg_xj_byte_cnt;
	reg [3 : 0] reg_xj_bit_cnt;
	reg [7 : 0] reg_img_data_shift;
    reg [7 : 0] reg_img_frame_cnt;
	reg [7 : 0] reg_img_code1;
	reg [15: 0] reg_img_column_size;
	reg [15: 0] reg_img_row_size;
	reg [15: 0] reg_img_h_cnt;
	reg [7 : 0] reg_img_start_flag;
	reg [31: 0] reg_img_frame_end;
	reg 		reg_xj_recv_oneframe_trigger;
	always @(negedge wire_xj_neg_edge or negedge i_sysrst_n)  
	begin
		if (~i_sysrst_n) begin 
			reg_img_frame_cnt <= 0;
			reg_xj_byte_cnt <= 0;
			reg_xj_bit_cnt <= 0;
			reg_xj_header <= 24'h0;
			reg_img_recvframe_write_en <= 0;
			reg_img_recvframe_write_addr <= 0;
			reg_img_recvframe_write_data <= 0;			
		end 
		else begin
		    if (reg_xj_byte_cnt == 0) begin
				reg_xj_recv_oneframe_trigger <= 1'b0;
				reg_img_recvframe_write_en <= 1'b0;
				if (reg_xj_header == 24'hFF55AA) begin // 检测图像帧头
					reg_xj_byte_cnt <= 3;
					reg_xj_bit_cnt <= 1;
					reg_img_data_shift[0] <= i_xj_data;
					reg_img_data_shift[7 : 1] <= reg_img_data_shift[6 : 0];
					reg_img_recvframe_write_addr <= 10'd0;
				end 
				else begin 
					reg_xj_header[0] <= i_xj_data;
					reg_xj_header[23: 1] <= reg_xj_header[22: 0];
				end 
			end 
			else if (reg_xj_byte_cnt >= 3 && reg_xj_byte_cnt < 593) begin  // 若接收到帧头
				reg_img_data_shift[0] <= i_xj_data;
				reg_img_data_shift[7 : 1] <= reg_img_data_shift[6 : 0];			
				case (reg_xj_bit_cnt)
					3: begin reg_xj_bit_cnt <= reg_xj_bit_cnt + 1; reg_img_recvframe_write_en <= 1'b0; end 
					8: begin
						reg_xj_bit_cnt <= 1;				
						reg_xj_byte_cnt <= reg_xj_byte_cnt + 1;
						reg_img_recvframe_write_data <= reg_img_data_shift; 
						reg_img_recvframe_write_en <= 1'b1;					
						
						if (reg_xj_byte_cnt != 3) begin 
							reg_img_recvframe_write_addr <= reg_img_recvframe_write_addr + 1;
						end 
						
						case (reg_xj_byte_cnt) 
							3:     reg_img_code1 <= reg_img_data_shift;
							4:     reg_img_frame_cnt <= reg_img_data_shift;
							5:     reg_img_column_size[15: 8] <= reg_img_data_shift;
							6:     reg_img_column_size[7 : 0] <= reg_img_data_shift;
							7:     reg_img_row_size[15: 8] <= reg_img_data_shift;
							8:     reg_img_row_size[7 : 0] <= reg_img_data_shift;	
							9:     reg_img_h_cnt[15: 8] <= reg_img_data_shift;
							10:    reg_img_h_cnt[7 : 0] <= reg_img_data_shift;	
							11:    reg_img_start_flag <= reg_img_data_shift;  			
							593-4: reg_img_frame_end[31:24] <= reg_img_data_shift;
							593-3: reg_img_frame_end[23:16] <= reg_img_data_shift;
							593-2: reg_img_frame_end[15: 8] <= reg_img_data_shift;
							593-1: reg_img_frame_end[7 : 0] <= reg_img_data_shift;
						endcase 
					end 
					default: begin reg_xj_bit_cnt <= reg_xj_bit_cnt + 1; end
				endcase 
			end 
			else begin
				reg_xj_bit_cnt <= 0;
				reg_xj_byte_cnt <= 0;
				reg_xj_header <= 0;
				reg_xj_recv_oneframe_trigger <= 1'b1;
			end  
		end 
	end 
	
	/* == 相机2级缓冲 == */
	reg          reg_img_recvframe_write_cache_en;
	reg [9 : 0]  reg_img_recvframe_write_cache_addr;
    reg [7 : 0]  reg_img_recvframe_write_cache_data;
	wire         reg_img_recvframe_read_cache_en = ~reg_img_recvframe_write_cache_en;
	reg          reg_img_recvframe_read_cache_addr_sel;
	reg [9 : 0]  reg_img_recvframe_read_cache_addr_real;	
	reg [9 : 0]  reg_img_recvframe_read_cache_addr;	
	wire[9 : 0]  reg_img_recvframe_read_cache_addr_ture = (i_huichuan_mode == TXJ_HUICHUAN_MODE_REALTIME) ? reg_img_recvframe_read_cache_addr_real : reg_img_recvframe_read_cache_addr; 
    wire[7 : 0]  reg_img_recvframe_read_cache_data;	
	blk_mem_img_frame blk_mem_img_frame_cache_inst (
		.clka  (i_sysclk),
		.addra (reg_img_recvframe_write_cache_addr),
		.dina  (reg_img_recvframe_write_cache_data),
		.wea   (reg_img_recvframe_write_cache_en),
		.ena   (reg_img_recvframe_write_cache_en),
		
		.clkb  (i_sysclk),
		.addrb (reg_img_recvframe_read_cache_addr_ture),
		.doutb (reg_img_recvframe_read_cache_data),
		.enb   (reg_img_recvframe_read_cache_en)
	);	
	
	reg  reg_xj_recv_trigger_d1, reg_xj_recv_trigger_d0;
	wire wire_xj_recv_copy_flag = (~reg_xj_recv_trigger_d1) & reg_xj_recv_trigger_d0;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_xj_recv_trigger_d0 <= 0;
			reg_xj_recv_trigger_d1 <= 0;
		end 
		else begin 
			reg_xj_recv_trigger_d0 <= reg_xj_recv_oneframe_trigger;
			reg_xj_recv_trigger_d1 <= reg_xj_recv_trigger_d0;
		end 	
	end 
	
	/*== 将图像从一级缓冲拷贝到二级缓冲 ==*/
	reg         reg_img_copy_busy;
	reg         reg_img_update_busy;
	wire        wire_img_copy_update_busy = reg_img_copy_busy | reg_img_update_busy;
	reg [3 : 0] reg_img_copy_state;
	reg [7 : 0] reg_img_copy_sub_step;
	reg [9 : 0] reg_img_copy_cnt;
	reg [7 : 0] reg_img_code1_cache;
	reg [15: 0] reg_img_column_size_cache;
	reg [15: 0] reg_img_h_cnt_cache;
	reg 		reg_xj_recv_img_cached_trigger;
	localparam  COPY_IMG_STATE_IDLE = 0, COPY_IMG_STATE_JUDGE_BUSY = 1, COPY_IMG_STATE_COPY = 2, COPY_IMG_STATE_DONE = 3;
	localparam  COPY_IMG_BUSY = 1;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_img_copy_sub_step <= 0;
			reg_img_copy_cnt <= 0;
			reg_img_copy_state <= COPY_IMG_STATE_IDLE;
			reg_img_copy_busy <= 1'b0;
			reg_xj_recv_img_cached_trigger <= 0;
		end 
		else begin 
			if (reg_img_copy_state == COPY_IMG_STATE_IDLE) begin 
				if (wire_xj_recv_copy_flag) begin 
					reg_img_copy_sub_step <= 0;
					reg_img_copy_cnt <= 0;
					reg_img_copy_state <= 1;
					reg_img_code1_cache <= reg_img_code1;
					reg_img_column_size_cache <= reg_img_column_size;
					reg_img_h_cnt_cache <= reg_img_h_cnt;
					reg_xj_recv_img_cached_trigger <= 0;
					reg_img_copy_busy <= 1'b0;
				end 
			end 
			else if (reg_img_copy_state == COPY_IMG_STATE_JUDGE_BUSY) begin
				if (wire_img_copy_update_busy == COPY_IMG_BUSY) begin 
					reg_img_copy_state <= COPY_IMG_STATE_JUDGE_BUSY;
				end 
				else begin 
					reg_img_copy_busy <= 1;
					reg_img_copy_state <= COPY_IMG_STATE_COPY;
				end 
			end 
			else if (reg_img_copy_state == COPY_IMG_STATE_COPY) begin  // 100M 25M速率存图，40ns BYTE 4us, 20us存储一副图像
				if (reg_img_copy_cnt < 586) begin 
					case (reg_img_copy_sub_step)
					0: begin reg_img_recvframe_write_cache_addr <= reg_img_copy_cnt; 
						     reg_img_recvframe_read_addr <= reg_img_copy_cnt; 
							 reg_img_recvframe_read_en <= 1; 
							 reg_img_copy_sub_step <= reg_img_copy_sub_step + 1; 
				    end 
					1: begin reg_img_copy_sub_step <= reg_img_copy_sub_step + 1; end
					2: begin reg_img_recvframe_write_cache_data <= reg_img_recvframe_read_data; 
						     reg_img_recvframe_write_cache_en <= 1; 
							 reg_img_recvframe_read_en <= 1'b0; 
							 reg_img_copy_sub_step <= reg_img_copy_sub_step + 1; 
					end
					3: begin reg_img_copy_sub_step <= reg_img_copy_sub_step + 1; end
					4: begin reg_img_recvframe_write_cache_en <= 1'b0; reg_img_copy_cnt <= reg_img_copy_cnt + 1; reg_img_copy_sub_step <= 0; end
					default: begin reg_img_copy_sub_step <= 0; end
					endcase 
				end
				else begin 
					reg_img_copy_state <= COPY_IMG_STATE_DONE;
					reg_xj_recv_img_cached_trigger <= 1;
				end 
			end 
			else begin 
				reg_xj_recv_img_cached_trigger <= 0;
				reg_img_copy_busy <= 0;
				reg_img_copy_cnt <= 0;
				reg_img_copy_state <= COPY_IMG_STATE_IDLE;
			end 
		end 
	end 
	
	/* == 相机缓存一帧数据标志 == */
	reg  reg_img_trigger_d1, reg_img_trigger_d0;
	wire wire_img_save_flag = (~reg_img_trigger_d1) & reg_img_trigger_d0;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_img_trigger_d0 <= 0;
			reg_img_trigger_d1 <= 0;
		end 
		else begin 
			reg_img_trigger_d0 <= reg_xj_recv_img_cached_trigger;
			reg_img_trigger_d1 <= reg_img_trigger_d0;
		end 
	end 

	/* == 图像一副的存储 ==*/
	reg  [18: 0] wire_blk_img_addra;
	reg  [7 : 0] wire_blk_img_datain;
	reg  [18: 0] reg_blk_img_addrb;
	wire [7 : 0] wire_blk_img_dataout;
	reg          reg_blk_img_wen;
	blk_mem_img  blk_mem_img_inst (
		.clka  (i_sysclk),
		.addra (wire_blk_img_addra),
		.dina  (wire_blk_img_datain),
		.wea   (reg_blk_img_wen),
		
		.clkb  (i_sysclk),
		.addrb (reg_blk_img_addrb),
		.doutb (wire_blk_img_dataout)
	);
	
	/* == 相机存一副图 == */
	reg          reg_img_save_done_trigger;
	reg  [1 : 0] reg_img_large_img_interval;
	reg  [9 : 0] reg_img_large_img_cnt;
	reg  [9 : 0] reg_img_small_img_cnt;
	reg  [9 : 0] reg_img_small_img_interval;
	reg  [9 : 0] reg_save_byte_cnt;
	reg  [7 : 0] reg_save_img_state;
	reg  [3 : 0] reg_save_img_delay;
	reg  [15: 0] reg_save_img_lrc;
	localparam SAVE_IMG_STATE_IDLE = 0, SAVE_IMG_STATE_STARTSAVE_LARGE = 1, SAVE_IMG_STATE_WRITEONEFRAME_LARGE = 2, SAVE_IMG_STATE_STARTSAVE_SMALL = 3, SAVE_IMG_STATE_WRITEONEFRAME_SMALL = 4;
	localparam SAVE_IMG_STATE_WAIT_WRITE_LARGE = 5, SAVE_IMG_STATE_WAIT_WRITE_SMALL = 6, SAVE_IMG_STATE_WAIT_WRITE_DONE = 7;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_img_save_done_trigger <= 0;
			reg_img_large_img_interval <= 0; //间隔帧计数
			reg_img_small_img_interval <= 0;
			reg_img_large_img_cnt <= 0;
			reg_img_small_img_cnt <= 0;
			reg_save_img_delay <= 0;
			reg_save_byte_cnt <= 0;
			reg_img_recvframe_read_cache_addr <= 0;
			reg_save_img_state <= SAVE_IMG_STATE_IDLE;
			reg_blk_img_wen <= 1'b0;
		end 
		else begin 
			if (reg_save_img_state == SAVE_IMG_STATE_IDLE) begin  
				reg_img_save_done_trigger <= 0;
				reg_img_recvframe_read_cache_addr <= 0;
				if (i_huichuan_img_enable) begin // 若回传图像使能
					if (wire_img_save_flag && reg_img_code1_cache == 8'hC1 && reg_img_column_size_cache >= 500 && reg_img_h_cnt_cache == 0) begin  // 大窗 500行
						if (reg_img_large_img_interval == 0) begin 
							reg_img_large_img_cnt <= 0;
							reg_img_large_img_interval <= 1;
							reg_save_img_state <= SAVE_IMG_STATE_WRITEONEFRAME_LARGE;
						end 
						else begin 
							reg_img_large_img_interval <= 0;
						end
					end
					else if (wire_img_save_flag && reg_img_code1_cache == 8'hC1 && reg_img_column_size_cache < 500 && reg_img_h_cnt_cache == 0) begin  // 小窗 接收
						if (reg_img_small_img_interval == 0) begin 
							reg_img_small_img_cnt <= 0;
							reg_img_small_img_interval <= 1;
							reg_save_img_state <= SAVE_IMG_STATE_STARTSAVE_SMALL;
						end 
						else begin 
							if (reg_img_small_img_interval == 8) begin
								reg_img_small_img_interval <= 0;
							end 
							else begin
								reg_img_small_img_interval <= reg_img_small_img_interval + 1;
							end 
						end				
					end 
				end
				else begin 
					reg_img_large_img_interval <= 0;
					reg_img_small_img_interval <= 0;
					reg_img_large_img_cnt <= 0;
					reg_img_small_img_cnt <= 0;
					reg_save_img_delay <= 0;
					reg_save_byte_cnt <= 0;
					reg_save_img_state <= SAVE_IMG_STATE_IDLE;
				end
			end 
			else if (reg_save_img_state == SAVE_IMG_STATE_STARTSAVE_LARGE) begin 
				if (i_huichuan_img_enable) begin 
					if (reg_img_large_img_cnt < 500) begin
						if (wire_img_save_flag && reg_img_code1_cache == 8'hC1 && reg_img_column_size_cache >= 500) begin 
							reg_save_img_state <= SAVE_IMG_STATE_WRITEONEFRAME_LARGE;
							reg_save_img_lrc <= 0;
							reg_save_byte_cnt <= 0;
							reg_img_recvframe_read_cache_addr <= 0;
						end
						else begin 
							if  (reg_img_column_size_cache < 500) begin 
								reg_img_small_img_interval <= 0;
								reg_save_img_state <= SAVE_IMG_STATE_IDLE;
							end 
						end 
					end 
					else begin 
						reg_save_img_state <= SAVE_IMG_STATE_WAIT_WRITE_DONE;
					end 	
				end
				else begin 
					reg_img_large_img_interval <= 0;
					reg_img_small_img_interval <= 0;
					reg_img_large_img_cnt <= 0;
					reg_img_small_img_cnt <= 0;
					reg_save_img_delay <= 0;
					reg_save_byte_cnt <= 0;
					reg_save_img_state <= SAVE_IMG_STATE_IDLE;				
				end 
			end 			
			else if (reg_save_img_state == SAVE_IMG_STATE_WRITEONEFRAME_LARGE) begin 
				reg_save_img_delay <= 0;
				reg_save_img_state <= SAVE_IMG_STATE_WAIT_WRITE_LARGE;
				if (reg_save_byte_cnt < 598) begin 
					wire_blk_img_addra <= reg_img_large_img_cnt * 598 + reg_save_byte_cnt;
					reg_save_byte_cnt <= reg_save_byte_cnt + 1;
					case (reg_save_byte_cnt)
					0: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'h55;
						reg_save_img_lrc <= reg_save_img_lrc + 8'h55; 
					end 
					1: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'hAA;
						reg_save_img_lrc <= reg_save_img_lrc + 8'hAA; 
					end 
					2: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'h33;
						reg_save_img_lrc <= reg_save_img_lrc + 8'h33; 
					end 
					3: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[0];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[0]; 
					end 
					4: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[1];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[1]; 
					end 
					5: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[2];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[2];
					end 
					6: begin
						reg_blk_img_wen <= 1'b1;					
						wire_blk_img_datain<= mem_broadcast_time[3];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[3];
					end 				
					7: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= i_dsp_1k_cnt;
						reg_save_img_lrc <= reg_save_img_lrc + i_dsp_1k_cnt;
					end 
					
					8+586-1: begin 
						reg_blk_img_wen <= 1'b1;
						reg_img_recvframe_read_cache_addr <= reg_img_recvframe_read_cache_addr + 1;
						wire_blk_img_datain <= reg_img_recvframe_read_cache_data; 
					end 
					
					8+586+0: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain <= reg_save_img_lrc[15: 8];
					end 				
					8+586+1: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain <= reg_save_img_lrc[7 : 0];
					end 
					8+586+2: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain <= 8'h7E;
					end 
					8+586+3: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain <= 8'hFF;
					end 

					default: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain <= reg_img_recvframe_read_cache_data;	
						reg_img_recvframe_read_cache_addr <= reg_img_recvframe_read_cache_addr + 1;						
						reg_save_img_lrc <= reg_save_img_lrc + reg_img_recvframe_read_cache_data;						
					end 
					endcase
			    end 
				else begin  
					reg_img_large_img_cnt <= reg_img_large_img_cnt + 1;
					reg_save_img_state <= SAVE_IMG_STATE_STARTSAVE_LARGE;
				end 
			end 
			else if (reg_save_img_state == SAVE_IMG_STATE_WAIT_WRITE_LARGE) begin 
				if (reg_save_img_delay <= 3) begin 
					reg_save_img_delay <= reg_save_img_delay + 1;
				end 
				else begin 
					reg_blk_img_wen <= 1'b0;
					reg_save_img_state <= SAVE_IMG_STATE_WRITEONEFRAME_LARGE;
				end 
			end 			
			else if (reg_save_img_state == SAVE_IMG_STATE_STARTSAVE_SMALL) begin 
				if (i_huichuan_img_enable) begin 
					if (reg_img_small_img_cnt < 4) begin
						if (wire_img_save_flag && reg_img_code1_cache == 8'hC1 && reg_img_column_size_cache < 500) begin 
							reg_save_img_state <= SAVE_IMG_STATE_WRITEONEFRAME_SMALL;
							reg_save_img_lrc <= 0;
							reg_save_byte_cnt <= 0;
							reg_img_recvframe_read_cache_addr <= 0;
						end 
						else begin  
							if (reg_img_column_size_cache >= 500) begin 
								reg_img_small_img_interval <= 0;
								reg_save_img_state <= SAVE_IMG_STATE_IDLE;
							end 
						end 
					end 
					else begin 
						reg_save_img_state <= SAVE_IMG_STATE_WAIT_WRITE_DONE;
					end 	
				end
				else begin 
					reg_img_large_img_interval <= 0;
					reg_img_small_img_interval <= 0;
					reg_img_large_img_cnt <= 0;
					reg_img_small_img_cnt <= 0;
					reg_save_img_delay <= 0;
					reg_save_byte_cnt <= 0;
					reg_save_img_state <= SAVE_IMG_STATE_IDLE;				
				end 
			end 			
			else if (reg_save_img_state == SAVE_IMG_STATE_WRITEONEFRAME_SMALL) begin 
				reg_save_img_delay <= 0;
				reg_save_img_state <= SAVE_IMG_STATE_WAIT_WRITE_SMALL;
				if (reg_save_byte_cnt < 598) begin 
					wire_blk_img_addra <= reg_img_small_img_cnt * 598 + reg_save_byte_cnt;
					reg_save_byte_cnt <= reg_save_byte_cnt + 1;
					case (reg_save_byte_cnt)
					0: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'hA5;
						reg_save_img_lrc <= reg_save_img_lrc + 8'hA5; 
					end 
					
					1: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'hA5;
						reg_save_img_lrc <= reg_save_img_lrc + 8'hA5; 
					end 
					
					2: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'h5A;
						reg_save_img_lrc <= reg_save_img_lrc + 8'h5A; 
					end 
					
					3: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[0];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[0]; 
					end 
					
					4: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[1];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[1]; 
					end 
					
					5: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= mem_broadcast_time[2];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[2];
					end 
					
					6: begin
						reg_blk_img_wen <= 1'b1;					
						wire_blk_img_datain<= mem_broadcast_time[3];
						reg_save_img_lrc <= reg_save_img_lrc + mem_broadcast_time[3];
					end 				
					
					7: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= i_dsp_1k_cnt;
						reg_save_img_lrc <= reg_save_img_lrc + i_dsp_1k_cnt;
					end
					
					8+586-1: begin 
						reg_blk_img_wen <= 1'b1;
						reg_img_recvframe_read_cache_addr <= reg_img_recvframe_read_cache_addr + 1;	
						wire_blk_img_datain<= reg_img_recvframe_read_cache_data;
					end 
										
					8+586+0: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= reg_save_img_lrc[15: 8];
					end 				
					8+586+1: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= reg_save_img_lrc[7 : 0];
					end 
					8+586+2: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'h7E;
					end 
					8+586+3: begin 
						reg_blk_img_wen <= 1'b1;
						wire_blk_img_datain<= 8'hFF;
					end 

					default: begin 
						reg_blk_img_wen <= 1'b1;
						reg_img_recvframe_read_cache_addr <= reg_img_recvframe_read_cache_addr + 1;	
						wire_blk_img_datain <= reg_img_recvframe_read_cache_data;		
						reg_save_img_lrc <= reg_save_img_lrc + reg_img_recvframe_read_cache_data;						
					end 
					endcase
			    end 
				else begin  
					reg_img_small_img_cnt <= reg_img_small_img_cnt + 1;
					reg_save_img_state <= SAVE_IMG_STATE_STARTSAVE_SMALL;
				end 
			end 
			else if (reg_save_img_state == SAVE_IMG_STATE_WAIT_WRITE_SMALL) begin 
				if (reg_save_img_delay <= 3) begin 
					reg_save_img_delay <= reg_save_img_delay + 1;
				end 
				else begin 
					reg_blk_img_wen <= 1'b0;			
					reg_save_img_state <= SAVE_IMG_STATE_WRITEONEFRAME_SMALL;
				end 
			end 
			else if (reg_save_img_state == SAVE_IMG_STATE_WAIT_WRITE_DONE) begin 
				reg_img_save_done_trigger <= 1'b1;
				reg_save_img_state <= SAVE_IMG_STATE_IDLE;
			end 			
			else begin 
				reg_save_img_state <= SAVE_IMG_STATE_IDLE;
			end 
		end 
	end 
	
	/* == 相机存一副图标志 == */
	reg    reg_img_save_done_trigger_d1, reg_img_save_done_trigger_d0;
	wire   wire_img_update_done_flag = (~reg_img_save_done_trigger_d1) & reg_img_save_done_trigger_d0;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_img_save_done_trigger_d0 <= 0;
			reg_img_save_done_trigger_d1 <= 0;
		end 
		else begin 
			reg_img_save_done_trigger_d0 <= reg_img_save_done_trigger;
			reg_img_save_done_trigger_d1 <= reg_img_save_done_trigger_d0;
		end 
	end 
	
	/*== IMG 数据更新标志 == */
	reg 	reg_img_need_save_flag;
	reg 	reg_img_need_save_flag_clr;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_img_need_save_flag <= 0;
		end 
		else begin 
			if (wire_img_update_done_flag) begin 
				reg_img_need_save_flag <= 1'b1;
			end 
			else if (reg_img_need_save_flag_clr == 1) begin 
				reg_img_need_save_flag <= 1'b0;
			end 
		end 
	end 		
	
	/*== TXJ 数据更新标志 == */
	reg		reg_1k_d0, reg_1k_d1;
	wire    wire_to_txj_flag = reg_1k_d1 & ~reg_1k_d0;	
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin
		if (~i_sysrst_n) begin
			reg_1k_d0 <= 0;
			reg_1k_d1 <= 0;
		end
		else begin
			reg_1k_d0 <= i_1k_trigger;
			reg_1k_d1 <= reg_1k_d0;
		end 
	end 
	
	/*== DSP 数据更新标志 == */
	reg     reg_dsp_need_save;
	reg 	reg_dsp_need_save_flag;
	reg 	reg_dsp_need_save_flag_clr;
	wire    wire_dsp_update_done_flag = reg_dsp_need_save;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_dsp_need_save_flag <= 0;
		end 
		else begin 
			if (wire_dsp_update_done_flag) begin 
				reg_dsp_need_save_flag <= 1'b1;
			end 
			else if (reg_dsp_need_save_flag_clr == 1) begin 
				reg_dsp_need_save_flag <= 1'b0;
			end 
		end 
	end 

	/*== DSP 频率设置 ==*/
	reg [9 : 0]	reg_dsp_save_cnt;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_dsp_save_cnt <= 0;
			reg_dsp_need_save <= 0;
		end 
		else begin 
			reg_dsp_need_save <= 0;
			case (i_huichuan_save_rate)
			8'h00: begin //4 HZ
				if (wire_to_txj_flag) begin 
					if (reg_dsp_save_cnt == (1000 / 4) - 1) begin 
						reg_dsp_save_cnt <= 0;
						reg_dsp_need_save <= 1'b1;
					end 
					else begin 
						reg_dsp_save_cnt <= reg_dsp_save_cnt + 1;
					end 
				end
			end 
			
			8'h11: begin // 100HZ
				if (wire_to_txj_flag) begin 
					if (reg_dsp_save_cnt == (1000 / 100) - 1) begin 
						reg_dsp_save_cnt <= 0;
						reg_dsp_need_save <= 1'b1;
					end 
					else begin 
						reg_dsp_save_cnt <= reg_dsp_save_cnt + 1;
					end 
				end 
			end 
			
			8'h22: begin // 1KHZ
				if (wire_to_txj_flag) begin 
					reg_dsp_save_cnt <= 0;
					reg_dsp_need_save <= 1'b1;
				end 
			end 
			
			default: begin 
				if (wire_to_txj_flag) begin 
					if (reg_dsp_save_cnt == (1000 / 4) - 1) begin 
						reg_dsp_save_cnt <= 0;
						reg_dsp_need_save <= 1'b1;
					end 
					else begin 
						reg_dsp_save_cnt <= reg_dsp_save_cnt + 1;
					end 
				end
			end 
			endcase
		end 
	end 
	
	/*== 广播数据更新 ==*/	
	reg     reg_time_trigger_d0, reg_time_trigger_d1;
	wire    wire_time_trigger_flag = (reg_time_trigger_d1 != reg_time_trigger_d0) ? 1'b1 : 1'b0;
	
	reg     reg_guidao_trigger_d0, reg_guidao_trigger_d1;
	wire    wire_guidao_trigger_flag = (reg_guidao_trigger_d1 != reg_guidao_trigger_d0) ? 1'b1 : 1'b0;
	
	reg     reg_taiyang_trigger_d0, reg_taiyang_trigger_d1;
	wire    wire_taiyang_trigger_flag = (reg_taiyang_trigger_d1 != reg_taiyang_trigger_d0) ? 1'b1 : 1'b0;
	
	reg     reg_zitai_trigger_d0, reg_zitai_trigger_d1;
	wire    wire_zitai_trigger_flag = (reg_zitai_trigger_d1 != reg_zitai_trigger_d0) ? 1'b1 : 1'b0;
	
	/*== 广播数据更新 ==*/
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin 
		if (~i_sysrst_n) begin 
			reg_time_trigger_d0 <= 0;
			reg_time_trigger_d1 <= 0;
			
			reg_guidao_trigger_d0 <= 0;
			reg_guidao_trigger_d1 <= 0;
			
			reg_taiyang_trigger_d0 <= 0;
			reg_taiyang_trigger_d1 <= 0;
			
			reg_zitai_trigger_d0 <= 0;
			reg_zitai_trigger_d1 <= 0;			
		end 
		else begin 
			reg_time_trigger_d0 <= i_time_trigger;
			reg_time_trigger_d1 <= reg_time_trigger_d0;
			
			reg_guidao_trigger_d0 <= i_guidao_trigger;
			reg_guidao_trigger_d1 <= reg_guidao_trigger_d0;
			
			reg_taiyang_trigger_d0 <= i_taiyang_trigger;
			reg_taiyang_trigger_d1 <= reg_taiyang_trigger_d0;
			
			reg_zitai_trigger_d0 <= i_zitai_trigger;
			reg_zitai_trigger_d1 <= reg_zitai_trigger_d0;
		end 
	end 
	
	/*== 广播数据更新 ==*/
	reg     reg_broad_need_save;
	reg 	reg_broad_need_save_flag;
	reg 	reg_broad_need_save_clr;
	wire    wire_broad_update_done_flag = reg_broad_need_save;
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin 
		if (~i_sysrst_n) begin 
			reg_broad_need_save_flag <= 0;
		end 
		else begin 
			if (wire_broad_update_done_flag) begin 
				reg_broad_need_save_flag <= 1'b1;
			end 
			else if (reg_broad_need_save_clr == 1) begin 
				reg_broad_need_save_flag <= 1'b0;
			end 
		end 
	end 
	
	localparam   SAVE_BROAD_IDLE = 0, SAVE_BROAD_START = 1, SAVE_BROAD_WAIT = 2;
	reg [7 : 0]  reg_broadcast_save_write_addr;
	reg [7 : 0]  reg_broadcast_save_write_data;
	reg          reg_broadcast_save_write_en;
	reg [7 : 0]  reg_broadcast_save_read_addr;
	reg [7 : 0]  reg_broadcast_save_read_addr_real;
	wire[7 : 0]  wire_broadcast_save_read_addr_true = (i_huichuan_mode == TXJ_HUICHUAN_MODE_REALTIME) ? reg_broadcast_save_read_addr_real : reg_broadcast_save_read_addr;
	wire[7 : 0]  wire_broadcast_save_read_data;
	blk_mem_broad blk_mem_broad_inst (
		.clka  (i_sysclk),
		.addra (reg_broadcast_save_write_addr),
		.dina  (reg_broadcast_save_write_data),
		.wea   (reg_broadcast_save_write_en),
		
		.clkb  (i_sysclk),
		.addrb (wire_broadcast_save_read_addr_true),
		.doutb (wire_broadcast_save_read_data)
	);	
	
	reg [3: 0] reg_broad_need_save_delay;
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin 
		if (~i_sysrst_n) begin 
			reg_broad_need_save <= 0;
			reg_broad_need_save_delay <= 0;
		end 
		else begin 
			if (wire_time_trigger_flag | wire_guidao_trigger_flag | wire_taiyang_trigger_flag | wire_zitai_trigger_flag) begin 
				reg_broad_need_save_delay <= 0;
				reg_broad_need_save <= 1;
			end 
			else begin 
				if (reg_broad_need_save_delay < 3) begin 
					reg_broad_need_save_delay <= reg_broad_need_save_delay + 1;
				end 
				else begin 
					reg_broad_need_save <= 0;
				end 
			end 
		end 
	end 
	
	reg [1 : 0]  reg_save_broad_state;
	reg [7 : 0]  reg_broad_save_cnt;
	reg [7 : 0]  reg_broad_save_sub_step;
    reg [31: 0]  reg_broad_timecnt;
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin 
		if (~i_sysrst_n) begin 
			reg_save_broad_state <= SAVE_BROAD_IDLE;
			reg_broad_save_sub_step <= 0;
		end 
		else begin 
			if (reg_save_broad_state == SAVE_BROAD_IDLE) begin 
				if (reg_broad_timecnt < 32'd5_000_00) begin 
					reg_broad_timecnt <= reg_broad_timecnt + 1;
				end 
				else begin 
					reg_broad_timecnt <= 0;
					reg_broad_save_sub_step <= 0;
					reg_broad_save_cnt <= 0;					
					reg_save_broad_state <= SAVE_BROAD_START;
				end 
			end 
			else begin 
				if (reg_broad_save_cnt < 6) begin 
					case (reg_broad_save_sub_step) 
					8'h0: begin reg_time_addr_savetime <= reg_broad_save_cnt; reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8'h1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8'h2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8'h3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_time_data; 
							 mem_broadcast_time[reg_broad_save_cnt] <= i_time_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
						  end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else if (reg_broad_save_cnt >= 6 && reg_broad_save_cnt < (6 + 32)) begin 
					case (reg_broad_save_sub_step) 
					0: begin o_guidao_addr <= reg_broad_save_cnt-6; reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_guidao_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end  
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else if (reg_broad_save_cnt >= (6 + 32) && reg_broad_save_cnt < (6 + 65)) begin   // 轨道数据
					case (reg_broad_save_sub_step) 
					0: begin 
							 case (i_project_guidao_sel)
							 16'h0000: begin o_guidao_addr <= reg_broad_save_cnt-6; end 
							
 							 16'h0011, 16'h1111: 
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*0; end 
							 16'h0022, 16'h2222:  
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*1; end 
							 16'h3333, 16'h0033:
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*2; end 
							 16'h4444, 16'h0044: 
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*3; end 
							 16'h5555, 16'h0055: 
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*4; end
							 16'h6666, 16'h0066: 
									  begin o_guidao_addr <= reg_broad_save_cnt-6 + 33*5; end	  
							 default: begin o_guidao_addr <= reg_broad_save_cnt-6; end 
							 endcase 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_guidao_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 				
				else if (reg_broad_save_cnt >= (6 + 65) && reg_broad_save_cnt < (6 + 65 + 13)) begin   // 太阳矢量
					case (reg_broad_save_sub_step) 
					0: begin o_taiyang_addr <= reg_broad_save_cnt-(6 + 65); reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end  
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 					
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_taiyang_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 					
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else if (reg_broad_save_cnt >= (6 + 65 + 13) && reg_broad_save_cnt < (6 + 65 + 13 + 2)) begin  // 卫星姿态 2BYTE
					case (reg_broad_save_sub_step) 
					0: begin o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13); reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 						
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_zitai_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 	
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else if (reg_broad_save_cnt >= (6 + 65 + 13 + 2) && reg_broad_save_cnt < (6 + 65 + 13 + 2 + 18)) begin  // 卫星姿态 20BYTE
					case (reg_broad_save_sub_step) 
					0: begin 
						case (i_project_xm_sel)
						8'h11: begin 
							o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13) + 0;  end 
						8'h22: begin 
							o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13) + 18;  end 
						8'h33: begin 
							o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13) + 36;  end 
						default: begin o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13) + 0;  end 
						endcase 	
						reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
					end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_zitai_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else if (reg_broad_save_cnt >= (6 + 65 + 13 + 2 + 18) && reg_broad_save_cnt < (6 + 65 + 13 + 50)) begin  // 卫星姿态
					case (reg_broad_save_sub_step) 
					0: begin o_zitai_addr <= reg_broad_save_cnt - (6 + 65 + 13) + 36; reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					2: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 					
					3: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= i_zitai_data; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					4: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end    
					5: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					6: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					7: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 	
					8: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 				
				else if (reg_broad_save_cnt >= (6 + 65 + 13 + 50) && reg_broad_save_cnt < 151)	begin  
					case (reg_broad_save_sub_step) 
					0: begin reg_broadcast_save_write_addr <= reg_broad_save_cnt; 
					         reg_broadcast_save_write_data <= 0; 
							 reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; 
							 reg_broadcast_save_write_en <= 1'b1; 
					   end 
					1: begin reg_broad_save_sub_step <= reg_broad_save_sub_step + 1; end 
					2: begin reg_broadcast_save_write_en <= 1'b0; reg_broad_save_cnt <= reg_broad_save_cnt + 1;  reg_broad_save_sub_step <= 0; end 
					default: begin reg_broad_save_sub_step <= 0; end
					endcase 
				end 
				else begin 
					reg_save_broad_state <= SAVE_BROAD_IDLE;
				end 
			end 
		end 
	end 	
	
	/*== 通信机数据更新标 ==*/
	reg     reg_txj_need_save;
	reg 	reg_txj_need_save_flag;
	reg 	reg_txj_need_save_flag_clr;
	wire    wire_txj_update_done_flag = reg_txj_need_save;
	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin 
		if (~i_sysrst_n) begin 
			reg_txj_need_save_flag <= 0;
		end 
		else begin 
			if (wire_txj_update_done_flag) begin 
				reg_txj_need_save_flag <= 1'b1;
			end 
			else if (reg_txj_need_save_flag_clr == 1) begin 
				reg_txj_need_save_flag <= 0;
			end 
		end 
	end 

	/*== 通信机数据接收 == */
	wire wire_txj_neg_edge;
	BUFG BUGTXJ(
		.I (i_txj_clk),
		.O (wire_txj_neg_edge)
	);

	reg         reg_txj_frame_write_en;
	reg [8 : 0] reg_txj_frame_write_addr;
    reg [7 : 0] reg_txj_frame_write_data;
	reg         reg_txj_frame_read_en;
	reg         reg_txj_frame_read_addr_sel;
	reg [8 : 0] reg_txj_frame_read_addr_real;	
	reg [8 : 0] reg_txj_frame_read_addr;	
	wire[8 : 0] reg_txj_frame_read_addr_ture = (i_huichuan_mode == TXJ_HUICHUAN_MODE_REALTIME) ? reg_txj_frame_read_addr_real : reg_txj_frame_read_addr; 
    wire[7 : 0] reg_txj_frame_read_data;
	blk_mem_from_txj blk_mem_from_txj_frame_inst (
		.clka  (~wire_txj_neg_edge),
		.addra (reg_txj_frame_write_addr),
		.dina  (reg_txj_frame_write_data),
		.wea   (reg_txj_frame_write_en),
		.ena   (reg_txj_frame_write_en),
		
		.clkb  (i_sysclk),
		.addrb (reg_txj_frame_read_addr_ture),
		.doutb (reg_txj_frame_read_data),
		.enb   (1'b1)
	);	

	reg [7 : 0] reg_txj_bit_cnt;
	reg [7 : 0] reg_txj_byte_cnt;
	reg [15: 0] reg_txj_header;
	reg [7 : 0] reg_txj_frame_len;
	reg [7 : 0] reg_txj_frame_type;
	reg [7 : 0] reg_txj_data_shift;
    reg [7 : 0] reg_txj_frame_cnt;
    reg         reg_txj_recv_pingpang;
	always @(negedge wire_txj_neg_edge or negedge i_sysrst_n)  
	begin
		if (~i_sysrst_n) begin 
			reg_txj_frame_cnt <= 0;
			reg_txj_byte_cnt <= 0;
			reg_txj_bit_cnt <= 0;
			reg_txj_header <= 16'h0;
			reg_txj_recv_pingpang <= 1'b0;
			reg_txj_need_save <= 1'b0;
			reg_txj_frame_write_addr <= 0;
		end 
		else begin
			reg_txj_need_save <= 1'b0;
			reg_txj_header[0] <= i_txj_data;
			reg_txj_header[15: 1] <= reg_txj_header[14: 0];
		    if (reg_txj_byte_cnt == 0) begin
				reg_txj_frame_write_en <= 1'b0;
				if (reg_txj_header == 16'hFC1A) begin 
					//mem_txj_data[reg_txj_recv_pingpang * 160 + 0] <= 8'hFC;
					//mem_txj_data[reg_txj_recv_pingpang * 160 + 1] <= 8'h1A;
					reg_txj_frame_write_addr <= reg_txj_recv_pingpang * 160 + 2;
					reg_txj_byte_cnt <= 2;
					reg_txj_bit_cnt <= 1;
					reg_txj_data_shift[0] <= i_txj_data;
				end 
			end 
			else if (reg_txj_byte_cnt == 2) begin  // 若接收到帧头
				reg_txj_data_shift[0] <= i_txj_data;
				reg_txj_data_shift[7 : 1] <= reg_txj_data_shift[6 : 0];		
				case (reg_txj_bit_cnt)
				3: begin reg_txj_frame_write_en <= 1'b0; reg_txj_bit_cnt <= reg_txj_bit_cnt + 1; end 
				8: begin
					reg_txj_bit_cnt <= 1;				
					reg_txj_byte_cnt <= reg_txj_byte_cnt + 1;
					reg_txj_frame_write_data <= reg_txj_data_shift; 
					reg_txj_frame_write_addr <= reg_txj_recv_pingpang * 160 + reg_txj_byte_cnt;
					reg_txj_frame_write_en <= 1'b1;					
					if (reg_txj_data_shift > 158) begin 
					   reg_txj_frame_len <= 158;
					end 
					else begin  
					   reg_txj_frame_len <= reg_txj_data_shift; 
					end 
				end 
				default: begin reg_txj_bit_cnt <= reg_txj_bit_cnt + 1; end
				endcase 
			end			
			else if (reg_txj_byte_cnt >= 3 && reg_txj_byte_cnt < reg_txj_frame_len + 2) begin  // 若接收到帧头
				reg_txj_data_shift[0] <= i_txj_data;
				reg_txj_data_shift[7 : 1] <= reg_txj_data_shift[6 : 0];		
				case (reg_txj_bit_cnt)
				3: begin reg_txj_frame_write_en <= 1'b0; reg_txj_bit_cnt <= reg_txj_bit_cnt + 1; end 
				8: begin
					reg_txj_bit_cnt <= 1;				
					reg_txj_byte_cnt <= reg_txj_byte_cnt + 1;
					reg_txj_frame_write_addr <= reg_txj_recv_pingpang * 160 + reg_txj_byte_cnt;
					reg_txj_frame_write_data <= reg_txj_data_shift; 
					reg_txj_frame_write_en <= 1'b1;					
				end 
				default: begin reg_txj_bit_cnt <= reg_txj_bit_cnt + 1; end
				endcase 
			end 
			else begin
				reg_txj_need_save <= 1'b1;
			    reg_txj_recv_pingpang <= ~reg_txj_recv_pingpang;
				reg_txj_byte_cnt <= 0; 
			end  
		end 
	end 		
		
	localparam  TO_TXJ_DATA_LIANLUBIAOSHI = 16'd0;  						
	localparam  TO_TXJ_DATA_BAOQISHI    = TO_TXJ_DATA_LIANLUBIAOSHI + 1;
	localparam  TO_TXJ_DATA_BAOXINXI    = TO_TXJ_DATA_BAOQISHI + 4;
	localparam  TO_TXJ_DATA_GAOSU_YAOCE = TO_TXJ_DATA_BAOXINXI + 2;
	localparam  TO_TXJ_DATA_GAOSU_END1  = TO_TXJ_DATA_GAOSU_YAOCE + 423;
	localparam  TO_TXJ_DATA_IMG         = TO_TXJ_DATA_GAOSU_END1 + 4;
	localparam  TO_TXJ_DATA_GAOSU_END2  = TO_TXJ_DATA_IMG + 586;
	localparam  TO_TXJ_DATA_GAOSU_END   = TO_TXJ_DATA_GAOSU_END2 + 2;
	localparam  TXJ_SEND_STATE_WAITSTART = 0, TXJ_SEND_STATE_FILLDATA = 8'h1, TXJ_SEND_STATE_FILLDATA_DELAY = 8'h7, TXJ_SEND_STATE_READDATA = 8'h02, TXJ_SEND_STATE_SAVEDATA = 8'h3, TXJ_SEND_STATE_SENDDATA = 8'h4, TXJ_SEND_STATE_READDATA_WAITBUSY = 8'h5, TXJ_SEND_STATE_READDATA_WAITBUSYDONE = 8'h6;	
	localparam  TXJ_SEND_STATE_SAVETXJ = 8'h10, TXJ_SEND_STATE_SAVEIMG = 8'h20,  TXJ_SEND_STATE_SAVEDSP = 8'h30,  TXJ_SEND_STATE_SAVEBROAD = 8'h40, TXJ_SEND_STATE_SAVEDONE = 8'h80;
	
	reg  [39: 0] reg_ddr3_save_addr_cmp; 
	reg  [39: 0] reg_ddr3_save_addr;
	reg  [39: 0] reg_ddr3_read_addr;
	reg  [39: 0] reg_ddr3_cnt_addr;
	reg  [39: 0] reg_ddr3_save_end_addr;
	reg  [39: 0] reg_ddr3_save_end_addr1;
	reg  [7 : 0] reg_ddr3_save_data;
	wire [7 : 0] reg_ddr3_read_data;
	reg  [15: 0] reg_save_cnt;
	reg  [7 : 0] reg_save_substep;
	reg  [15: 0] reg_save_lrc;
	reg          reg_ddr3_we_en;
	
	// DDR 数据存储
	blk_mem_ddr blk_mem_ddr_inst (
		.clka  (i_sysclk),
		.addra (reg_ddr3_save_addr[16:0]),
		.dina  (reg_ddr3_save_data),
	    .wea   (reg_ddr3_we_en),
		
		.clkb  (i_sysclk),
		.addrb (reg_ddr3_read_addr[16:0]),
		.doutb (reg_ddr3_read_data)	
	);
	
	reg [7 : 0] reg_pre_save_mode;
	reg [31: 0] reg_txj_fillimgcnt;
	reg [23: 0] reg_check_framecnt;
	reg [15: 0] reg_check_frameend;
	reg [31: 0] reg_save_framecnt;
	reg [31: 0] reg_read_framecnt;
	reg [10: 0] reg_to_txj_fill_cnt;
	reg [10: 0] reg_to_txj_byte_cnt;
	reg [7 : 0] reg_to_txj_bit_cnt;
	reg [7 : 0] reg_to_txj_data_update_state;
	reg 		reg_to_txj_start_send_flag;
	reg [15: 0] reg_to_txj_framecnt;
	reg [7 : 0] reg_to_txj_debug_data;
	reg [7 : 0] reg_fill_sub_step;
    reg [7 : 0] reg_fill_delay_cnt;
	reg         reg_txj_send_pingpang;
	reg [7 : 0] reg_to_txj_img_framecnt;
	reg [7 : 0] reg_o_txj_send_state;
	reg [31: 0] reg_tbl_cnt;
	reg         reg_read_state_img;
	wire        wire_read_img_debug;
	wire        wire_txj_save_pingpang = ~ reg_txj_send_pingpang;
	
	reg [15: 0] reg_dsp_save_frame_cnt;
	
	reg         reg_to_txj_sendbusy;
	reg         reg_to_txj_fillbusy;
	reg         reg_to_txj_start_read_data_flag; 
	reg         reg_to_txj_start_fill_data_flag;
	
	//wire [31: 0] reg_interval_max;
	//reg  [31: 0] reg_interval_cnt;
	localparam  TXJ_HUICHUAN_ENABLE = 8'H11, TXJ_HUICHUAN_ENABLE1 = 8'H03;
	localparam  TXJ_HUICHUAN_SAVE_NOENABLE = 8'h00;
	localparam  TXJ_HUICHUAN_SAVE_EN_MODEXUNHUAN = 8'H13; // 循环存储；
	localparam  TXJ_HUICHUAN_SAVE_EN_MODENBUXUNHUAN = 8'H33; // 不循环存储
	
	wire        wire_huichuan_mode_realtime_flag = (i_huichuan_mode == TXJ_HUICHUAN_MODE_REALTIME) ? 1'b1 : 1'b0;
	wire        wire_huichuan_mode_save_flag = ((i_huichuan_mode == TXJ_HUICHUAN_MODE_SAVE) || (i_huichuan_mode == TXJ_HUICHUAN_MODE_SAVE1)) ? 1'b1 : 1'b0;
	wire        wire_huichuan_enable_realtime_flag = ((i_huichuan_enable == TXJ_HUICHUAN_ENABLE) || (i_huichuan_enable == TXJ_HUICHUAN_ENABLE1)) ? 1'b1 : 1'b0;		
	wire        wire_huichuan_enable_save_flag = ((i_huichuan_enable == TXJ_HUICHUAN_ENABLE) || (i_huichuan_enable == TXJ_HUICHUAN_ENABLE1)) ? 1'b1 : 1'b0;		
	/*== 实时数据或者存储数据数据更新控制 ==*/
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin
		if (~i_sysrst_n) begin
			reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
			reg_ddr3_save_end_addr <= 0;
			reg_ddr3_save_addr <= 0;
			reg_ddr3_save_data <= 0;

			reg_save_cnt <= 0;
			reg_save_substep <= 0;
			reg_save_lrc <= 0;
			reg_img_need_save_flag_clr <= 0;
			reg_broad_need_save_clr <= 0;
			reg_txj_need_save_flag_clr <= 0;
			reg_dsp_need_save_flag_clr <= 0;
			reg_dsp_save_frame_cnt <= 0;
			reg_ddr3_we_en <= 0;
			reg_tbl_cnt <= 0;
			reg_img_recvframe_read_cache_addr_sel <= 0;
			reg_dsp_addr_save <= 0;
			reg_ddr3_save_addr_cmp <= 0;
			
			reg_pre_save_mode <= 0;
		end 
		else begin
			if (reg_to_txj_data_update_state == TXJ_SEND_STATE_WAITSTART) begin
				
				/*== 逻辑控制 ==*/
				if (wire_huichuan_mode_realtime_flag) begin // 实时模式，并且回传使
					if (wire_to_txj_flag && wire_huichuan_enable_realtime_flag) begin
						reg_ddr3_save_addr <= 0;
						reg_dsp_save_frame_cnt <= 0; 
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_FILLDATA;
					end 
					else begin 
						reg_ddr3_save_addr <= 0;
						reg_dsp_save_frame_cnt <= 0; 
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
					end
				end 
				else if (wire_huichuan_mode_save_flag) begin  // 存储模式
					
					if (wire_huichuan_enable_save_flag) begin     // 使能回传
						/*==============================================================*/					
           				case (reg_pre_save_mode)
						TXJ_HUICHUAN_SAVE_EN_MODEXUNHUAN: begin 
							if (reg_ddr3_save_addr >= 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= 500 * 1024 * 1024;
								reg_ddr3_save_addr <= 0;
							end 
							else if (reg_ddr3_save_addr > 0 && reg_ddr3_save_addr < 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= reg_ddr3_save_addr;
								reg_ddr3_save_addr <= 0;
							end 
							reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA;						
						end 
						
						TXJ_HUICHUAN_SAVE_EN_MODENBUXUNHUAN: begin
							if (reg_ddr3_save_addr >= 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= 500 * 1024 * 1024;
								reg_ddr3_save_addr <= 0;
							end 
							else if (reg_ddr3_save_addr > 0 && reg_ddr3_save_addr < 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= reg_ddr3_save_addr;
								reg_ddr3_save_addr <= 0;
							end 			
							reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA;
						end 
						
						default: begin 	/*
							if (reg_ddr3_save_addr >= 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= 500 * 1024 * 1024;
								reg_ddr3_save_addr <= 0;
							end 
							else if (reg_ddr3_save_addr > 0 && reg_ddr3_save_addr < 500 * 1024 * 1024) begin 
								reg_ddr3_save_addr_cmp <= reg_ddr3_save_addr;
								reg_ddr3_save_addr <= 0;							
							end */			
							reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA;
						end 
						endcase
						/*==============================================================*/					
					end 
					else begin   
						// 在不回传模式下，如果使能存储则开始存储，如果关闭存储则关闭存储。
						/*==============================================================*/
						case (i_huichuan_save_enable)	
						TXJ_HUICHUAN_SAVE_EN_MODENBUXUNHUAN: begin
							reg_pre_save_mode <= i_huichuan_save_enable;						
							if (reg_ddr3_save_addr == 0) begin 
								reg_dsp_save_frame_cnt <= 0;
								reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDATA;
							end 
							else if (reg_ddr3_save_addr < 500 * 1024 * 1024) begin
								reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDATA;
							end 
							else begin 
								reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
							end 
						end 
						
						TXJ_HUICHUAN_SAVE_EN_MODEXUNHUAN: begin 
							reg_pre_save_mode <= i_huichuan_save_enable;
							if (reg_ddr3_save_addr == 0) begin 
								reg_dsp_save_frame_cnt <= 0;
								reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDATA;
							end 
							else begin 
								reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDATA;
							end 						
						end 
						
						default: begin reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART; end
						endcase 
						/*==============================================================*/						
					end
				end
				else begin
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
				end
				/*== 逻辑控制结束 ==*/
			end 
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_READDATA) begin   
				if (reg_ddr3_cnt_addr < reg_ddr3_save_addr_cmp) begin 
					reg_to_txj_start_read_data_flag <= 1;
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_FILLDATA_DELAY;
				end 
				else begin 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
				end 
			end
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_FILLDATA_DELAY) begin  // 实时包信息填充
				reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA_WAITBUSY;
			end			
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_READDATA_WAITBUSY) begin   
				reg_to_txj_start_fill_data_flag <= 0;
				reg_to_txj_start_read_data_flag <= 0;
				if (reg_to_txj_sendbusy == 0) begin 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA_WAITBUSY;
				end 
				else begin 
					//reg_read_framecnt <= reg_read_framecnt + 1; 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA_WAITBUSYDONE;
				end 
			end			
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_FILLDATA) begin        // 实时包信息填充
				reg_to_txj_start_fill_data_flag <= 1;
				reg_to_txj_data_update_state <= TXJ_SEND_STATE_FILLDATA_DELAY;
			end			
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_READDATA_WAITBUSYDONE) begin   
				if (reg_to_txj_sendbusy == 1) begin 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_READDATA_WAITBUSYDONE;
				end 
				else begin 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
				end 
			end			
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVEDATA) begin 
				reg_save_cnt <= 0;
				reg_save_substep <= 0;
				reg_save_lrc <= 0;
				if (reg_txj_need_save_flag) begin 
					reg_txj_need_save_flag_clr <= 1'b1;
					if (reg_dsp_save_frame_cnt < 4 * 256) begin 
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVETXJ;
					end 
					else begin 
						reg_ddr3_save_addr <= reg_ddr3_save_addr + 92;
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;
					end 
				end 
				else if (reg_dsp_need_save_flag) begin 
					reg_dsp_need_save_flag_clr <= 1'b1;
					if (reg_dsp_save_frame_cnt < 4 * 256) begin 
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDSP;
					end 
					else begin
						reg_ddr3_save_addr <= reg_ddr3_save_addr + 86;
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;					
					end
				end 
				else if (reg_img_need_save_flag) begin
					reg_img_need_save_flag_clr <= 1'b1;
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEIMG;
				end 
				else if (reg_broad_need_save_flag) begin
					reg_broad_need_save_clr <= 1'b1;
					if (reg_dsp_save_frame_cnt < 4 * 256) begin 
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEBROAD;
					end 
					else begin
						reg_ddr3_save_addr <= reg_ddr3_save_addr + 164;
						reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;					
					end					
				end 				
				else begin 
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;
				end 
			end
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVEIMG) begin     // 
				reg_img_need_save_flag_clr <= 1'b0;
				reg_save_cnt <= 0;
				reg_save_substep <= 0;
				reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;		
			end
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVETXJ) begin     // 0x95B06B
				if (reg_save_cnt == 0) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h95; reg_ddr3_we_en <= 1'b1; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 1) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'hB0; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 2) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h6B; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 3) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[0]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 4) begin 
					case (reg_save_substep)
					0: begin reg_dsp_addr_save <= 78; reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[1]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 5) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[2]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 6) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[3]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 7) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= i_dsp_data; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt >= 8 && reg_save_cnt < (3+5+80)) begin 
					case (reg_save_substep)
					0: begin reg_txj_frame_read_addr <= ~reg_txj_recv_pingpang * 160 + reg_save_cnt - 8 + 3; reg_save_substep <= reg_save_substep + 1; end
					1: begin reg_save_substep <= reg_save_substep + 1; end 
					2: begin reg_save_substep <= reg_save_substep + 1; end  
					3: begin reg_ddr3_save_data[7 : 0] <= reg_txj_frame_read_data; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					4: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					5: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					6: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					7: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+5+80)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7: 0] <= reg_save_lrc[15: 8]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+5+80+1)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7: 0] <= reg_save_lrc[7: 0]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+5+80+2)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h7E; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+5+80+2+1)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'hFF; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_ddr3_we_en <= 1'b0;  reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 				
				else begin 
					reg_ddr3_we_en <= 1'b0;
					reg_txj_need_save_flag_clr <= 1'b0;
					reg_save_cnt <= 0;
					reg_save_substep <= 0;
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;
				end 
			end 
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVEDSP) begin     // 0xEB901C  
				if (reg_save_cnt == 0) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'hEB; reg_ddr3_we_en <= 1'b1; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 1) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h90; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 2) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h1C; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 3) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= i_yc_request_cnt; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 4) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= i_last_cmd; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 5) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= i_err_cnt; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt >= 6 && reg_save_cnt < (3+79)) begin 
					case (reg_save_substep)
					0: begin reg_dsp_addr_save <= reg_save_cnt - 6 + 2; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_ddr3_save_data[7 : 0] <= i_dsp_data; 
							 if ((reg_save_cnt == (3 + 62)) && (i_dsp_data == 8'h9A)) begin 
								reg_tbl_cnt <= reg_tbl_cnt + 1;
							 end 	
						     reg_save_substep <= reg_save_substep + 1; 
					    end // 等待写入
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+79)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7: 0] <= reg_save_lrc[15: 8]; reg_save_substep <= reg_save_substep + 1; end  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+79+1)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7: 0] <= reg_save_lrc[7: 0]; reg_save_substep <= reg_save_substep + 1; end  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase
				end	
				else if (reg_save_cnt == (3+79+2)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h7E; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (3+79+3)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'hFF; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					4: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_ddr3_we_en <= 1'b0;  reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else begin 
					reg_ddr3_we_en <= 1'b0; 
					reg_dsp_need_save_flag_clr <= 1'b0;
					reg_save_cnt <= 0;
					reg_save_substep <= 0;
					reg_dsp_save_frame_cnt <= reg_dsp_save_frame_cnt + 1;  
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;
				end 			
			end 
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVEBROAD) begin   // 
				if (reg_save_cnt == 0) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h14; reg_ddr3_we_en <= 1'b1; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 1) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h6F;  reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 2) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h8A; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 3) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[0]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end
				else if (reg_save_cnt == 4) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[1]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 5) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[2]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 6) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= mem_broadcast_time[3]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == 7) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= i_dsp_1k_cnt; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt >= 8 && reg_save_cnt < (8+151)) begin 
					case (reg_save_substep)
					0: begin reg_broadcast_save_read_addr <= reg_save_cnt - 8; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					4: begin reg_ddr3_save_data[7 : 0] <= wire_broadcast_save_read_data; reg_save_lrc <= reg_save_lrc + wire_broadcast_save_read_data; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					5: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					6: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					7: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (8+151)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 0; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_lrc <= reg_save_lrc + reg_ddr3_save_data[7 : 0]; reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入						
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (8+151+1)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= reg_save_lrc[15: 8]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (8+151+2)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= reg_save_lrc[7: 0]; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (8+151+3)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'h7E; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 
				else if (reg_save_cnt == (8+151+4)) begin 
					case (reg_save_substep)
					0: begin reg_ddr3_save_data[7 : 0] <= 8'hFF; reg_save_substep <= reg_save_substep + 1; end  //写入数据,触发保存  
					1: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入
					2: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入延时
					3: begin reg_save_substep <= reg_save_substep + 1; end // 等待写入					
					4: begin reg_ddr3_save_addr <= reg_ddr3_save_addr + 1; reg_ddr3_we_en <= 1'b0; reg_save_cnt <= reg_save_cnt + 1; reg_save_substep <= 0; end // 等待写入延时
					default: reg_save_substep <= 0;
					endcase 
				end 				
				else begin 
					reg_ddr3_we_en <= 1'b0;
					reg_broad_need_save_clr <= 1'b0;
					reg_save_cnt <= 0;
					reg_save_substep <= 0;
					reg_to_txj_data_update_state <= TXJ_SEND_STATE_SAVEDONE;
				end 			
			end 
			else if (reg_to_txj_data_update_state == TXJ_SEND_STATE_SAVEDONE) begin  
				reg_dsp_need_save_flag_clr <= 1'b0;
				reg_txj_need_save_flag_clr <= 1'b0;
				reg_img_need_save_flag_clr <= 1'b0;
				reg_broad_need_save_clr <= 1'b0;
				reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;	
				if (reg_dsp_save_frame_cnt == 256 * 4) begin 
					reg_ddr3_save_end_addr <= reg_ddr3_save_addr;
					reg_dsp_save_frame_cnt <= reg_dsp_save_frame_cnt + 1;
				end 			
			end 			
			else begin 
				reg_to_txj_data_update_state <= TXJ_SEND_STATE_WAITSTART;	
			end 
		end 
	end 
	
	/*== 通信机数据更新 ==*/
	wire [10: 0] reg_mem_to_txj_w_addr = reg_to_txj_fill_cnt;
	reg  [7 : 0] reg_mem_to_txj_w_data;
	reg  [10: 0] reg_mem_to_txj_r_addr;
	wire [7 : 0] reg_mem_to_txj_r_data;
	reg          reg_mem_to_txj_data_wen;
	wire         wire_mem_to_txj_data_ren = ~reg_mem_to_txj_data_wen;
	reg  [7 : 0] reg_to_txj_fill_state;
	wire   		 wire_txj_read_clk;
	wire   		 wire_txj_fill_clk;
	BUFG BUGFILLTXJ(
		.I (i_sysclk),
		.O (wire_txj_fill_clk)
	);
	
	blk_mem_to_txj mem_to_txj_data (
		.clka  (wire_txj_fill_clk),
		.addra (reg_mem_to_txj_w_addr[9:0]),
		.dina  (reg_mem_to_txj_w_data),
		.wea   (reg_mem_to_txj_data_wen),
		.ena   (reg_mem_to_txj_data_wen),
		
		.clkb  (wire_txj_read_clk),
		.addrb (reg_mem_to_txj_r_addr[9:0]),
		.doutb (reg_mem_to_txj_r_data),	
		.enb   (wire_mem_to_txj_data_ren)
	);
	
	/*== 存储更新标志 ==*/
	reg		reg_to_txj_start_read_data_flag_d0, reg_to_txj_start_read_data_flag_d1;
	wire    wire_to_txj_start_read_data_flag = (~reg_to_txj_start_read_data_flag_d1) & reg_to_txj_start_read_data_flag_d0;	
	always @(posedge wire_txj_fill_clk or negedge i_sysrst_n) // wire_txj_fill_clk  i_sysclk
	begin
		if (~i_sysrst_n) begin
			reg_to_txj_start_read_data_flag_d0 <= 0;
			reg_to_txj_start_read_data_flag_d1 <= 0;
		end
		else begin
			reg_to_txj_start_read_data_flag_d0 <= reg_to_txj_start_read_data_flag;
			reg_to_txj_start_read_data_flag_d1 <= reg_to_txj_start_read_data_flag_d0;
		end 
	end 
	
	/*== 实时数据更新标志 ==*/
	reg		reg_to_txj_start_fill_data_flag_d0, reg_to_txj_start_fill_data_flag_d1;
	wire    wire_to_txj_start_fill_data_flag = (~reg_to_txj_start_fill_data_flag_d1) & reg_to_txj_start_fill_data_flag_d0;	
	always @(posedge wire_txj_fill_clk or negedge i_sysrst_n) // wire_txj_fill_clk  i_sysclk
	begin
		if (~i_sysrst_n) begin
			reg_to_txj_start_fill_data_flag_d0 <= 0;
			reg_to_txj_start_fill_data_flag_d1 <= 0;
		end
		else begin
			reg_to_txj_start_fill_data_flag_d0 <= reg_to_txj_start_fill_data_flag;
			reg_to_txj_start_fill_data_flag_d1 <= reg_to_txj_start_fill_data_flag_d0;
		end 
	end 
	
	reg s_axis_dividend_tvalid, s_axis_divisor_tvalid;
	reg [31 :0] s_axis_dividend_tdata, s_axis_divisor_tdata;
    wire m_axis_dout_tvalid;
	wire[31 :0] m_axis_dout_tdata;  // 假设输出位宽为32位（商16位+余数16位）
	  // 实例化Divider IP核
	div_gen_0 u_div (
		.aclk(i_sysclk),
		//.aresetn(i_sysrst_n),
		.s_axis_dividend_tvalid(s_axis_dividend_tvalid),
		.s_axis_dividend_tdata(s_axis_dividend_tdata),
		.s_axis_divisor_tvalid(s_axis_divisor_tvalid),
		.s_axis_divisor_tdata(s_axis_divisor_tdata),
		.m_axis_dout_tvalid(m_axis_dout_tvalid),
		.m_axis_dout_tdata(m_axis_dout_tdata)
	);
	
	
	reg [31: 0] reg_tbl_cnt1;
	reg [7 : 0] reg_o_txj_fill_state;
	reg [31: 0] reg_o_txj_send_allframe_cnt;
	/*== 数据更新模块 ==*/
	localparam TO_TXJ_FILL_STATE_IDLE = 0, TO_TXJ_FILL_STATE_FILLSAVE = 8'h11, TO_TXJ_FILL_STATE_FILLSAVE_IMG = 8'h22, TO_TXJ_FILL_STATE_FILLREAL = 8'h33;
	always @(posedge wire_txj_fill_clk or negedge i_sysrst_n) // wire_txj_fill_clk  i_sysclk
	begin
		if (~i_sysrst_n) begin 
			reg_img_update_busy <= 1'b0; 
			reg_tbl_cnt1 <= 0;
			reg_dsp_addr_real <= 0;
			reg_ddr3_cnt_addr <= 0;
			reg_to_txj_start_send_flag <= 0;
			reg_to_txj_framecnt <= 0;
			reg_ddr3_read_addr <= 0;
			reg_txj_fillimgcnt <= 0;
			reg_fill_delay_cnt <= 0;
			reg_fill_sub_step <= 0;
			reg_to_txj_fill_cnt <= 0;
			reg_check_frameend <= 0;
			reg_read_state_img <= 0;
			reg_check_framecnt <= 0;
			reg_save_framecnt <= 0;	
			reg_txj_send_pingpang <= 0;
			reg_blk_img_addrb <= 0;
			reg_mem_to_txj_data_wen <=  1'b0;
			reg_ddr3_save_end_addr1 <= 0;
			reg_o_txj_send_allframe_cnt <= 0;
			
			reg_read_framecnt <= 0;			
			s_axis_dividend_tdata <= 0;
			s_axis_divisor_tdata <= 0;
			s_axis_dividend_tvalid <= 1'b0;
			s_axis_divisor_tvalid <= 1'b0; 		
			
			reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
		end 
		else begin 
			if (reg_o_txj_fill_state == TO_TXJ_FILL_STATE_IDLE) begin 
				if (wire_to_txj_start_read_data_flag) begin       // 存储模式填充数据
					if (i_huichuan_img_enable == 0) begin 
						if (reg_read_framecnt <= 512 * 1024) begin 
							reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_FILLSAVE;
							reg_to_txj_fill_cnt <= 0;
							reg_fill_sub_step <= 0;
						end 
						else begin 
							reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
						end 
					end 
					else begin 
						if (reg_read_framecnt <= 512 * 1024) begin 
							reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_FILLSAVE_IMG;
							reg_to_txj_fill_cnt <= 0;
							reg_fill_sub_step <= 0;
						end 
						else begin 
							reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
						end 
					end 
					reg_ddr3_save_end_addr1 <= reg_ddr3_save_end_addr;
				end
				else if (wire_to_txj_start_fill_data_flag) begin  // 实时模式填充数据
					reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_FILLREAL;
					reg_to_txj_fill_cnt <= 0;
				end 
				else if (i_huichuan_save_enable) begin 
					reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
					reg_read_framecnt <= 0;	
					reg_ddr3_cnt_addr <= 0;					
				end 
				else begin 
					reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
				end 
				reg_to_txj_start_send_flag <= 0;
				reg_fill_sub_step <= 0;				
			end 
			else if (reg_o_txj_fill_state == TO_TXJ_FILL_STATE_FILLSAVE) begin
				if (reg_to_txj_fill_cnt == 0) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h01; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 1) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 2) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 3) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 4) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 5) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt == 6) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end  
				else if (reg_to_txj_fill_cnt > 6 && reg_to_txj_fill_cnt <= 1009) begin // 5050 0x5050 1003
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_ddr3_read_data[7 : 0]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end					
					3:  begin reg_ddr3_cnt_addr <= reg_ddr3_cnt_addr + 1; reg_fill_sub_step <= reg_fill_sub_step + 1; end
					4:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_fill_sub_step <= reg_fill_sub_step + 1; end
					5:  begin //reg_ddr3_read_addr <= (reg_ddr3_cnt_addr % reg_ddr3_save_end_addr1); 
							  if (reg_ddr3_save_end_addr1 > 0) begin
									//reg_ddr3_read_addr <= (reg_ddr3_cnt_addr % reg_ddr3_save_end_addr1);
									s_axis_dividend_tdata <= reg_ddr3_cnt_addr;
									s_axis_divisor_tdata <= reg_ddr3_save_end_addr1;
									s_axis_dividend_tvalid <= 1'b1;
									s_axis_divisor_tvalid <= 1'b1;
									reg_fill_sub_step <= 10; 
							  end 
						      else begin 
									reg_ddr3_read_addr <= reg_ddr3_cnt_addr;
									reg_fill_sub_step <= reg_fill_sub_step + 1; 
							  end 
							  
					    end// 
					6:  begin reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					
					10: begin if (m_axis_dout_tvalid == 1'b1) begin 
									reg_ddr3_read_addr <= m_axis_dout_tdata; s_axis_dividend_tvalid <= 1'b0; s_axis_divisor_tvalid <= 1'b0;	reg_fill_sub_step <= 6; 
						      end 
					end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 			
				else if (reg_to_txj_fill_cnt > 1009 && reg_to_txj_fill_cnt <= (1009+4)) begin // 脱靶量有效计数器
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h00; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == (1009+4+1)) begin // 回传帧计数
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_read_framecnt[15: 8]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase				
				end 
				else if (reg_to_txj_fill_cnt == (1009+4+2)) begin // 回传帧计数
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_read_framecnt[7: 0]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase				
				end 
				else if (reg_to_txj_fill_cnt > (1009+4+2) && reg_to_txj_fill_cnt <= (1009+4+2+4)) begin // 存储模式0x0F0F0F0F
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h0F; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase	
				end 
				else if (reg_to_txj_fill_cnt > (1009+4+2+4) && reg_to_txj_fill_cnt <= (1009+4+2+4+4)) begin // 5050 0x5050
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hFF; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase	
					reg_fill_delay_cnt <= 0;
				end 						
				else begin 
					if (reg_fill_delay_cnt == 0) begin 
						reg_to_txj_start_send_flag <= 1'b1;
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else if (reg_fill_delay_cnt > 0 && reg_fill_delay_cnt < 99) begin 
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else begin
						reg_read_framecnt <= reg_read_framecnt + 1;
						reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
					end 							
				end					
			end 
			else if (reg_o_txj_fill_state == TO_TXJ_FILL_STATE_FILLSAVE_IMG) begin
				if (reg_to_txj_fill_cnt == 0) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h01; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 1) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 2) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 3) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 4) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 5) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt == 6) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end   
				else if (reg_to_txj_fill_cnt > 6 && reg_to_txj_fill_cnt <= 1009) begin // 5050 0x5050 1003 //// IIIII
					if (reg_read_state_img == 0) begin 
						case (reg_fill_sub_step)
						0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						1:  begin   
								reg_mem_to_txj_data_wen <= 1'b1;
								reg_mem_to_txj_w_data <= reg_ddr3_read_data[7 : 0]; 										
								reg_check_frameend[7 : 0] <= reg_ddr3_read_data[7 : 0];
								reg_check_frameend[15: 8] <= reg_check_frameend[7 : 0];
								reg_fill_sub_step <= reg_fill_sub_step + 1;
							end 
						2:  begin reg_ddr3_cnt_addr <= reg_ddr3_cnt_addr + 1; reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						4:  begin //reg_ddr3_read_addr <= (reg_ddr3_cnt_addr % reg_ddr3_save_end_addr1); 
								  if (reg_ddr3_save_end_addr1 > 0) begin
										//reg_ddr3_read_addr <= (reg_ddr3_cnt_addr % reg_ddr3_save_end_addr1);
										s_axis_dividend_tdata <= reg_ddr3_cnt_addr;
										s_axis_divisor_tdata <= reg_ddr3_save_end_addr1;
										s_axis_dividend_tvalid <= 1'b1;
										s_axis_divisor_tvalid <= 1'b1;
										reg_fill_sub_step <= 10; 
								  end 
								  else begin 
										reg_ddr3_read_addr <= reg_ddr3_cnt_addr;
										reg_fill_sub_step <= reg_fill_sub_step + 1; 
								  end 
								  
							end// 
						
						10: begin if (m_axis_dout_tvalid == 1'b1) begin 
										reg_ddr3_read_addr <= m_axis_dout_tdata; s_axis_dividend_tvalid <= 1'b0; s_axis_divisor_tvalid <= 1'b0;	reg_fill_sub_step <= 5; 
								  end 
						end 
						
						5:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						6:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1;   reg_fill_sub_step <= 0; 
							if (reg_check_frameend == 16'h7EFF || wire_read_img_debug) begin //&& reg_img_need_save_flag) begin 
								reg_check_framecnt <= reg_check_framecnt + 1;
								case (i_huichuan_save_rate)
								8'h00: begin 
									if (reg_img_column_size_cache >= 500) begin 
									    if ((reg_check_framecnt % 8) == 0) begin 
											reg_read_state_img <= 1;
											reg_fill_sub_step <= 0;
											reg_txj_fillimgcnt <= 0;
									    end //4 HZ						
									end 
									else begin 
									    //if ((reg_check_framecnt % 2) == 0) begin 
											reg_read_state_img <= 1;
											reg_fill_sub_step <= 0;
											reg_txj_fillimgcnt <= 0;
									    //end //4 HZ											
									end 
								end 
								8'h11: begin 
									if (reg_img_column_size_cache >= 500) begin 
									    if ((reg_check_framecnt % 105) == 0) begin 
											reg_read_state_img <= 1;
											reg_fill_sub_step <= 0;	
											reg_txj_fillimgcnt <= 0;														
									    end //4 HZ
									end 
									else begin 
										if ((reg_check_framecnt % 8) == 0) begin 
											reg_read_state_img <= 1;
											reg_fill_sub_step <= 0;	
											reg_txj_fillimgcnt <= 0;														
									    end //4 HZ								
									end 
								end  
								8'h22: begin 
										if (reg_img_column_size >= 500) begin 
										    if ((reg_check_framecnt % 1004) == 0) begin 
												reg_read_state_img <= 1;
												reg_fill_sub_step <= 0;	
												reg_txj_fillimgcnt <= 0;														
										    end //4 HZ
										end 
										else begin 
										    if ((reg_check_framecnt % 125) == 0) begin 
												reg_read_state_img <= 1;
												reg_fill_sub_step <= 0;	
												reg_txj_fillimgcnt <= 0;														
										    end //4 HZ										
										end 
								end   
								default: begin 
									if ((reg_check_framecnt % 1004) == 0) begin 
										reg_read_state_img <= 1;
										reg_fill_sub_step <= 0;	
										reg_txj_fillimgcnt <= 0;														
									end //4 HZ							
								end
								endcase 
							end
						end 
						default: begin  reg_fill_sub_step <= 0; end
						endcase
					end 
					else begin 
						case (reg_fill_sub_step)
						0:  begin if (reg_img_column_size_cache >= 500) begin
									   reg_blk_img_addrb <= reg_txj_fillimgcnt; 
								  end 
								  else begin 
									   reg_blk_img_addrb <= reg_txj_fillimgcnt % 4 * 598;
								  end 
								  reg_fill_sub_step <= reg_fill_sub_step + 1;
							end 
						1:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end	
						2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_mem_to_txj_data_wen <= 1'b1;
								  reg_mem_to_txj_w_data <= wire_blk_img_dataout; 
								  reg_txj_fillimgcnt <= reg_txj_fillimgcnt + 1;   
								  reg_fill_sub_step <= reg_fill_sub_step + 1; 
							end
						4:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						5:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						6:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; 
							  if (reg_img_column_size_cache >= 500) begin 
									if (reg_txj_fillimgcnt >= 500 * 598) begin 
										reg_read_state_img <= 0;
									end 
									else begin
										reg_read_state_img <= 1;
									end 
							  end 
							  else begin 
									if (reg_txj_fillimgcnt >= 4 * 598) begin 
										reg_read_state_img <= 0;
									end 
									else begin 
										reg_read_state_img <= 1;
									end 
							  end 						
						end
						default: begin  reg_fill_sub_step <= 0; end
						endcase								
					end 				 		
				end 
				else if (reg_to_txj_fill_cnt > 1009 && reg_to_txj_fill_cnt <= (1009+4)) begin // 脱靶量有效计数器
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h00; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt == (1009+4+1)) begin // 回传帧计数
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_read_framecnt[15: 8]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase				
				end 
				else if (reg_to_txj_fill_cnt == (1009+4+2)) begin // 回传帧计数
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_read_framecnt[7: 0]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase				
				end 
				else if (reg_to_txj_fill_cnt > (1009+4+2) && reg_to_txj_fill_cnt <= (1009+4+2+4)) begin // 存储模式0x0F0F0F0F
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h0F; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase	
				end 
				else if (reg_to_txj_fill_cnt > (1009+4+2+4) && reg_to_txj_fill_cnt <= (1009+4+2+4+4)) begin // 5050 0x5050
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hFF; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase	
					reg_fill_delay_cnt <= 0;
				end 
				else begin 
					if (reg_fill_delay_cnt == 0) begin 
						reg_to_txj_start_send_flag <= 1'b1;
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else if (reg_fill_delay_cnt > 0 && reg_fill_delay_cnt < 99) begin 
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else begin
						reg_read_framecnt <= reg_read_framecnt + 1;
						reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
					end 							
				end	
			end 
			else if (reg_o_txj_fill_state == TO_TXJ_FILL_STATE_FILLREAL) begin
				if (reg_to_txj_fill_cnt == 0) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h01; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 1) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 2) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hE7; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 3) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 4) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hA5; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 5) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin 
						  	  if (reg_to_txj_img_framecnt == reg_img_frame_cnt) begin 
								  reg_mem_to_txj_w_data <= 8'hA0;
							  end 
							  else begin 
								  reg_mem_to_txj_w_data <= 8'h50;
								  reg_to_txj_img_framecnt <= reg_img_frame_cnt;
							  end 
							  reg_mem_to_txj_data_wen <= 1'b1; 
							  reg_fill_sub_step <= reg_fill_sub_step + 1; 
					end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt == 6) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin 
						  	  if (reg_to_txj_img_framecnt == reg_img_frame_cnt) begin 
								  reg_mem_to_txj_w_data <= 8'hA0;
							  end 
							  else begin 
								  reg_mem_to_txj_w_data <= 8'h50;
								  reg_to_txj_img_framecnt <= reg_img_frame_cnt;
							  end 
							  reg_mem_to_txj_data_wen <= 1'b1; 
							  reg_fill_sub_step <= reg_fill_sub_step + 1; 
					end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end   
				else if (reg_to_txj_fill_cnt == 7) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hDA; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt >= 8 && reg_to_txj_fill_cnt <= 86) begin 
					if ((reg_to_txj_fill_cnt-8) == 0) begin 
						case (reg_fill_sub_step)
						0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						1:  begin reg_mem_to_txj_w_data <= i_yc_request_cnt; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
						2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
						default: begin  reg_fill_sub_step <= 0; end
						endcase		
					end 
					else if ((reg_to_txj_fill_cnt-8) == 1) begin 
						case (reg_fill_sub_step)
						0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						1:  begin reg_mem_to_txj_w_data <= i_last_cmd; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
						2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
						default: begin  reg_fill_sub_step <= 0; end
						endcase					
					end 
					else if ((reg_to_txj_fill_cnt-8) == 2) begin 
						case (reg_fill_sub_step)
						0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						1:  begin reg_mem_to_txj_w_data <= i_err_cnt; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
						2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
						default: begin  reg_fill_sub_step <= 0; end
						endcase
					end 
					else begin // Hex(0x65：脱靶量无效 0x9A：脱靶量有效
						case (reg_fill_sub_step)
						0: begin reg_dsp_addr_real <= reg_to_txj_fill_cnt-8-1; reg_fill_sub_step <= reg_fill_sub_step + 1; end   
						1: begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						2: begin reg_mem_to_txj_w_data <= i_dsp_data; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3: begin 
								if (((reg_to_txj_fill_cnt - 8) == 8'd62) && (i_dsp_data == 8'h9A)) begin 
									reg_tbl_cnt1 <= reg_tbl_cnt1 + 1;
								end 	
								reg_fill_sub_step <= reg_fill_sub_step + 1;
						end
						4: begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end
						default: begin  reg_fill_sub_step <= 0; end
						endcase 					
					end 
				end 
				else if (reg_to_txj_fill_cnt == 87) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h11; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt >= 88 && reg_to_txj_fill_cnt < 239) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_broadcast_save_read_addr_real <= reg_to_txj_fill_cnt-88 ; reg_fill_sub_step <= reg_fill_sub_step + 1; end 
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					4:  begin reg_mem_to_txj_w_data <= wire_broadcast_save_read_data; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					5:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					6:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end		
				else if (reg_to_txj_fill_cnt >= 239 && reg_to_txj_fill_cnt <= 253) begin //本身就填写零的数
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 
				else if (reg_to_txj_fill_cnt == 254) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'h22; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end
				else if (reg_to_txj_fill_cnt >= 255 && reg_to_txj_fill_cnt <= 409) begin 
					case (reg_fill_sub_step)
					0:  begin reg_txj_frame_read_addr_real <= reg_txj_send_pingpang * 160 + reg_to_txj_fill_cnt - 255 + 3; reg_fill_sub_step <= reg_fill_sub_step + 1; end 
					1:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_w_data <= reg_txj_frame_read_data; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					4:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					5:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase				
				end 
				else if (reg_to_txj_fill_cnt >= 410 && reg_to_txj_fill_cnt <= 423) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase		
				end 
				else if (reg_to_txj_fill_cnt == 424) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_tbl_cnt1[31:24]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase		
				end 
				else if (reg_to_txj_fill_cnt == 425) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_tbl_cnt1[23:16]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase		
				end 
				else if (reg_to_txj_fill_cnt == 426) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_tbl_cnt1[15: 8]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase		
				end 
				else if (reg_to_txj_fill_cnt == 427) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_tbl_cnt1[7 : 0]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase		
				end 
				else if (reg_to_txj_fill_cnt == 428) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_to_txj_framecnt[15: 8]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase	
				end 
				else if (reg_to_txj_fill_cnt == 429) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= reg_to_txj_framecnt[7 : 0]; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt >= 430 && reg_to_txj_fill_cnt <= 1015) begin 
					if (wire_img_copy_update_busy) begin 
						reg_img_update_busy <= 1'b0; 
					end 
					else begin 
						reg_img_update_busy <= 1'b1;
						case (reg_fill_sub_step)
						0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						1:  begin reg_img_recvframe_read_cache_addr_real <= reg_to_txj_fill_cnt - 430; reg_fill_sub_step <= reg_fill_sub_step + 1; end
						2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						3:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						4:  begin reg_mem_to_txj_w_data <= reg_img_recvframe_read_cache_data; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
						5:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						6:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
						7:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
						default: begin  reg_fill_sub_step <= 0; end
						endcase
					end 
				end
				else if (reg_to_txj_fill_cnt >= 1016 && reg_to_txj_fill_cnt <= 1019) begin 
					reg_img_update_busy <= 0;
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hF0; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
				end 				
				else if (reg_to_txj_fill_cnt >= 1020 && reg_to_txj_fill_cnt <= 1023) begin 
					case (reg_fill_sub_step)
					0:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					1:  begin reg_mem_to_txj_w_data <= 8'hFF; reg_mem_to_txj_data_wen <= 1'b1; reg_fill_sub_step <= reg_fill_sub_step + 1; end		  
					2:  begin reg_fill_sub_step <= reg_fill_sub_step + 1; end
					3:  begin reg_mem_to_txj_data_wen <= 1'b0; reg_to_txj_fill_cnt <= reg_to_txj_fill_cnt + 1; reg_fill_sub_step <= 0; end 
					default: begin  reg_fill_sub_step <= 0; end
					endcase
					reg_fill_delay_cnt <= 0;
				end 	
				else begin 
					reg_mem_to_txj_data_wen <= 1'b0;
					if (reg_fill_delay_cnt == 0) begin 
						reg_txj_send_pingpang <= ~reg_txj_send_pingpang;
						reg_to_txj_framecnt <= reg_to_txj_framecnt + 1;
						reg_to_txj_start_send_flag <= 1'b1;
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else if (reg_fill_delay_cnt > 0 && reg_fill_delay_cnt < 99) begin 
						reg_fill_delay_cnt <= reg_fill_delay_cnt + 1;
					end 
					else begin
						reg_fill_delay_cnt <= 0;
						reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;
					end 
				end			
			end
			else begin 
				reg_fill_delay_cnt <= 0;
				reg_o_txj_fill_state <= TO_TXJ_FILL_STATE_IDLE;			
			end
		end
	end

	/*== 发送频率生成 ==*/
	reg  wire_o_txj_clk;
	wire wire_sysclk_50m;
	wire wire_sysclk_30m;
    wire wire_sysclk_25m;
	wire wire_sysclk_10m;
	wire wire_sysclk_100m;
    wire wire_pll_locked;
        clk_wiz_to_txj  clk_wiz_to_txj_inst (
        .clk_in1         (i_sysclk),
        .clk_out_25      (wire_sysclk_25m),
        .clk_out_30      (wire_sysclk_30m),
		.clk_out_50      (wire_sysclk_50m),
		.clk_out_10      (wire_sysclk_10m),
		.clk_out_100     (wire_sysclk_100m),
        .resetn          (1'b1),				
        .locked          (wire_pll_locked)
    );
	
 	always @(*)
	begin
		case (i_huichuan_mode)
			TXJ_HUICHUAN_MODE_REALTIME: begin  wire_o_txj_clk = wire_sysclk_25m; end 
			default: begin
				case (i_huichuan_rate) 
				8'h11:   begin wire_o_txj_clk = wire_sysclk_10m; end 
				8'h22:   begin wire_o_txj_clk = wire_sysclk_30m; end 
				8'h33:   begin wire_o_txj_clk = wire_sysclk_50m; end 
				8'h44:   begin wire_o_txj_clk = wire_sysclk_100m; end 
				default: begin wire_o_txj_clk = wire_sysclk_100m; end 
				endcase 
			end 
		endcase 
	end 
	
	
	wire wire_to_txj_neg_edge;
	BUFG BUGTOTXJ(
		.I (wire_o_txj_clk),
		.O (wire_to_txj_neg_edge)
	);
	
	assign o_txj_clk = wire_to_txj_neg_edge;
	assign wire_txj_read_clk = wire_to_txj_neg_edge;
	
	/*== 发送触发 ==*/
	reg    reg_o_txj_start_send_d1, reg_o_txj_start_send_d0;
	wire   wire_o_txj_start_send_flag = (~reg_o_txj_start_send_d1) & reg_o_txj_start_send_d0;
	always @(posedge wire_to_txj_neg_edge or negedge i_sysrst_n)
	begin
		if (~i_sysrst_n) begin 
			reg_o_txj_start_send_d1 <= 0;
			reg_o_txj_start_send_d0 <= 0;
		end
		else begin 
			reg_o_txj_start_send_d0 <= reg_to_txj_start_send_flag;
			reg_o_txj_start_send_d1 <= reg_o_txj_start_send_d0;
		end 
	end

	/*== 通信机发送下降 ==*/
	//reg  [31: 0]  reg_to_txj_send_cnt_debug;
	reg  [7 : 0]  reg_to_txj_out_reg; 
	reg  [7 : 0]  reg_to_txj_out_preread_reg;
	localparam  TO_TXJ_SEND_STATE_IDLE = 8'h00, TO_TXJ_SEND_STATE_SEND_PRE = 8'h1, TO_TXJ_SEND_STATE_SEND = 8'h3, TO_TXJ_SEND_STATE_SEND_DONE = 8'h0C;
	always @(posedge wire_to_txj_neg_edge or negedge i_sysrst_n)
	begin
		if (~i_sysrst_n) begin 
			reg_to_txj_bit_cnt <= 0;
			reg_to_txj_byte_cnt <= 0;
			reg_o_txj_send_state <= 0;
			reg_to_txj_sendbusy <= 0;
			reg_mem_to_txj_r_addr <= 0;
			reg_to_txj_out_reg <= 0;
			o_txj_data <= 1'b1;
			//reg_to_txj_send_cnt_debug <= 0;
		end
		else begin 
			if (reg_o_txj_send_state == TO_TXJ_SEND_STATE_IDLE) begin  
				if (wire_o_txj_start_send_flag) begin 
					reg_o_txj_send_state <= TO_TXJ_SEND_STATE_SEND_PRE;
					reg_to_txj_byte_cnt <= 0;
					reg_to_txj_bit_cnt <= 0;
					reg_to_txj_sendbusy <= 1;
					reg_mem_to_txj_r_addr <= 0;
				end 
			end 
			else if (reg_o_txj_send_state == TO_TXJ_SEND_STATE_SEND_PRE) begin			
				reg_to_txj_out_reg <= reg_mem_to_txj_r_data; 
				reg_o_txj_send_state <= TO_TXJ_SEND_STATE_SEND;
			end 
			else if (reg_o_txj_send_state == TO_TXJ_SEND_STATE_SEND) begin
				if (reg_to_txj_byte_cnt < 1024) begin 
					o_txj_data <= reg_to_txj_out_reg[7 - reg_to_txj_bit_cnt]; 
					case (reg_to_txj_bit_cnt) 
					0: begin reg_mem_to_txj_r_addr <= reg_mem_to_txj_r_addr + 1; reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end 
					1: begin reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end 
					2: begin reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end 
					3: begin reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end 
					4: begin reg_to_txj_out_preread_reg <= reg_mem_to_txj_r_data; reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end 
					5: begin reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end  
					6: begin reg_to_txj_bit_cnt <= reg_to_txj_bit_cnt + 1; end  
					7: begin reg_to_txj_out_reg <= reg_to_txj_out_preread_reg; reg_to_txj_bit_cnt <= 0; reg_to_txj_byte_cnt <= reg_to_txj_byte_cnt + 1; end 
					default: begin reg_to_txj_bit_cnt <= 0; end 
					endcase 
				end
				else begin
					o_txj_data <= 1'b1;
					reg_mem_to_txj_r_addr <= 0;
					reg_o_txj_send_state <= TO_TXJ_SEND_STATE_SEND_DONE;
				end
			end
			else begin 
				reg_o_txj_send_state <= TO_TXJ_SEND_STATE_IDLE;
				reg_to_txj_sendbusy <= 0;
				o_txj_data <= 1'b1;
				//reg_to_txj_send_cnt_debug <= reg_to_txj_send_cnt_debug + 1;
			end 
		end 
	end	
/*
	ila_1 ila_huichuan_inst (
		.clk     (i_sysclk),
		.probe0  (i_xj_clk),
		.probe1  (i_xj_data),
		.probe2  (reg_mem_to_txj_r_addr),
		.probe3  (reg_to_txj_out_reg),
		.probe4  (o_txj_clk),
		.probe5  (o_txj_data),
		.probe6  (reg_xj_header),
		.probe7  (reg_img_recvframe_write_addr),
		.probe8  (reg_img_data_shift),
		.probe9  (reg_img_recvframe_read_cache_addr_real),
		.probe10 (reg_img_recvframe_read_cache_data),
		.probe11 (wire_time_trigger_flag),
		.probe12 (wire_guidao_trigger_flag),
		.probe13 (wire_taiyang_trigger_flag),
		.probe14 (wire_zitai_trigger_flag),
		.probe15 (reg_broad_save_cnt)	
    );		
*/
	/*
	ila_huichuan ila_huichuan_inst (
		.clk     (i_sysclk),
		.probe0  (i_1k_trigger),
		.probe1  (i_xj_clk),
		.probe2  (i_xj_data),
		.probe3  (reg_xj_byte_cnt),
		.probe4  (reg_img_data_shift),
		.probe5  (i_txj_clk),
		.probe6  (i_txj_data),
		.probe7  (reg_txj_header),
		.probe8  (reg_txj_bit_cnt),
		.probe9  (reg_txj_byte_cnt),
		.probe10 (reg_txj_data_shift),
		.probe11 (o_txj_clk),
		.probe12 (o_txj_data),
		.probe13 (reg_to_txj_byte_cnt),
		.probe14 (reg_to_txj_bit_cnt),
		.probe15 (reg_to_txj_debug_data),
		.probe16 (reg_broad_need_save_flag),
		.probe17 (reg_dsp_need_save_flag),
		.probe18 (reg_img_need_save_flag),
		.probe19 (reg_txj_need_save_flag),
		.probe20 (reg_to_txj_data_update_state),
		.probe21 (reg_save_substep),
		.probe22 (reg_save_cnt),
		.probe23 (reg_save_substep),
		.probe24 (reg_img_code1),
		.probe25 (reg_img_h_cnt),
		.probe26 (reg_img_column_size),
		.probe27 (i_time_trigger),
		.probe28 (i_guidao_trigger),
		.probe29 (i_zitai_trigger),
		.probe30 (i_taiyang_trigger),
		.probe31 (reg_fill_sub_step),
		.probe32 (reg_check_framecnt),
		.probe33 (reg_read_state_img),
		.probe34 (reg_check_frameend)
	);
	*/
	
	vio_hc vio_hc_inst (
	    .clk         (i_sysclk),
		.probe_in0   (i_huichuan_mode),
		.probe_in1   (i_huichuan_save_enable),
		.probe_in2   (i_huichuan_enable),
		.probe_in3   (i_huichuan_rate),
		.probe_in4 	 (i_huichuan_save_rate),
		.probe_in5 	 (i_huichuan_img_enable),
		.probe_in6 	 (reg_ddr3_save_addr),
		.probe_in7 	 (reg_ddr3_read_addr),
		.probe_in8 	 (reg_txj_fillimgcnt),
		.probe_in9 	 (wire_read_img_debug),
		.probe_in10  (reg_read_framecnt),
		.probe_in11  (reg_ddr3_cnt_addr),
		.probe_in12  (reg_dsp_save_frame_cnt),
		.probe_in13  (reg_to_txj_data_update_state),
		.probe_in14  (reg_ddr3_save_end_addr),
		.probe_in15  (reg_ddr3_read_addr),
		.probe_in16  (reg_to_txj_fill_cnt),
		.probe_in17  (reg_fill_sub_step),
		.probe_in18  (reg_to_txj_send_cnt_debug),
		.probe_in19  (i_dsp_1k_cnt),
		.probe_in20  (i_project_guidao_sel),
		.probe_in21  (i_project_xm_sel),
		.probe_in22  (reg_img_code1),
		.probe_in23  (reg_img_column_size),
		.probe_in24  (reg_ddr3_save_addr_cmp)
		//.probe_out0  (reg_interval_max)
	);
	
	endmodule