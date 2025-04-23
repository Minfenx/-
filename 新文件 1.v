	/*== 星务重构 1Mbps 8 odd 1 XWCG==*/
	wire   		  wire_xw_cg_uart_data_done_flag;
	wire [7 : 0]  wire_xw_cg_uart_data;
	
	uart_recv_odd   #(
        .UART_BPS     (1_000_000),
        .CLK_FREQ     (100_000_000))
	uart_recv_odd_xw_cg_inst (
        .sys_clk      (i_sysclk),
        .sys_rst_n    (i_sysrst_n),
        .uart_rxd     (i_xw_cg_uart_rxd),
        .uart_done    (wire_xw_cg_uart_data_done_flag),
        .uart_data    (wire_xw_cg_uart_data),
        .rx_flag(),   
        .rx_cnt()
    );   	
	

    reg           reg_xw_cg_uart_data_d0;
    reg           reg_xw_cg_uart_data_d1;
    wire 		  wire_xw_cg_recv_flag = (~reg_xw_cg_uart_data_d1) & reg_xw_cg_uart_data_d0;  
	  
    always@ (posedge i_sysclk or negedge i_sysrst_n)
    begin
        if (~i_sysrst_n) begin
            reg_xw_cg_uart_data_d0 <= 0;
            reg_xw_cg_uart_data_d1 <= 0;
        end 
        else begin
            reg_xw_cg_uart_data_d0 <= wire_xw_cg_uart_data_done_flag;
            reg_xw_cg_uart_data_d1 <= reg_xw_cg_uart_data_d0;
        end
    end

	/*==星务重构接收ila*/
   	ila_txj ila_xw_rev_inst (
		.clk (i_sysclk),
		.probe0 (wire_xw_cg_uart_data_done_flag),
		.probe1 (reg_cg_recv_databyte[7:0]),
		.probe2 (wire_xw_cg_uart_data)
	);		

	/*== 重构状态 ==*/
	// 基址地址,用于重构帧数据处理
	localparam 	CG_FRAME_DATA_BASEADDR = 19;

	// 定义重构状态机的各个状态
	parameter  	CG_STATE_IDLE    = 0,   			// 空闲状态
				CG_STATE_WAIT_HEAD1 = 1, 			// 等待帧头1 7E
				CG_STATE_WAIT_HEAD2 = 2; 			// 等待帧头2 7E
				// CG_STATE_WAIT_HEAD3 = 3, 			// 等待帧头3 
				// CG_STATE_WAIT_HEAD4 = 4; 			// 等待帧头4

	parameter  	CG_STATE_LEN1   = 3,   				// 处理长度字节1
				CG_STATE_LEN2   = 4;     			// 处理长度字节2 
	
	parameter  	CG_STATE_SOURCE_ID1 = 5;   			// 源地址
	parameter  	CG_STATE_SOURCE_ID2 = 6;   			// 源地址

	parameter  	CG_STATE_DEVID_1 = 7,   			// 目的ID高位
				CG_STATE_DEVID_2  = 8,     			// 目的ID低位
				CG_STATE_REV_FRAME_CNT1 = 9,   		// 总帧数
				CG_STATE_REV_FRAME_CNT2 = 10,   	// 总帧数 
				CG_STATE_REV_FRAME_NUM1 = 11,		// 当前帧序号
				CG_STATE_REV_FRAME_NUM2 = 12;		 
	parameter	CG_STATE_YULIU1 = 40,    			// 预留字节1
				CG_STATE_YULIU2 = 41,    			// 预留字节2 
				CG_STATE_YULIU3 = 42,    			// 预留字节3
				CG_STATE_YULIU4 = 43,    			// 预留字节4 
				CG_STATE_YULIU5 = 44,    			// 预留字节5
				CG_STATE_YULIU6 = 45,    			// 预留字节6 
				CG_STATE_YULIU7 = 46,    			// 预留字节7
				CG_STATE_YULIU8 = 47;    			// 预留字节8


	parameter  	CG_STATE_DATA = 13,     			// 数据处理状态
				CG_STATE_END_CODE1 = 48,			// 结束码1
				CG_STATE_END_CODE2 = 49,			// 结束码2
				CG_STATE_CHECK1 = 14,    			// 校验状态1
				CG_STATE_CHECK2 = 15,    			// 校验状态2
				CG_STATE_CHECK3 = 16,    			// 校验状态3
				CG_STATE_CHECK_END = 17, 			// 校验结束状态
				CG_STATE_CHECK_END_ALLFILE = 18; 	// 整个文件校验完成状态      
	parameter   CG_STATE_HANDLER = 19;				//重构处理逻辑,DSP 写入MSRAM, 其他单机触发转发
	parameter   CG_STATE_HANDLER_WAITING = 20;		//等待处理完成
	parameter   CG_STATE_HANDLER_DONE = 21;			//重构处理完成
	parameter   CG_STATE_RESPOND = 22,  CG_STATE_RESPOND_CRC = 23, CG_STATE_RESPOND_WAITING = 24, CG_STATE_RESPOND_DONE = 25;
	parameter   CG_STATE_ERR_LEN = 128;				//帧长度错误
	parameter   CG_STATE_ERR_SUBDEVICE = 129;		//待重构的设备和发送数据不符
	parameter   CG_STATE_ERR_DEVICETYPE = 130;		//源和目的地址类型错误
	parameter   CG_STATE_ERR_DATATYPE = 131;		//净荷类型错误 0x02:重构数据 0xF2:重构响应帧
	parameter   CG_STATE_ERR_FRAMECNT = 132;		//帧序号错误,和已经接收到的帧计数对不上(此种方法不支持续传???)
	parameter   CG_STATE_ERR_TIMEOUT = 133;			//超时错误处理
	parameter   CG_STATE_ERR_CRC = 134;				//帧校验错误
	parameter   CG_STATE_ERR_FILE = 135;			//文件校验错误
	parameter   CG_STATE_ERR_SUM = 136;				//链路层和校验错误
	parameter   CG_STATE_ERR_OTHER = 254;			//其他错误
	parameter   CG_STATE_SUCCESS = 255;				//重构成功
	localparam  CG_WDT_TIMEOUT = 32'd100_000_000;
	reg [7 : 0]  reg_cg_lasterr_state;				//上次错误状态,调试查看使用
	reg [7 : 0]  reg_cg_state;						//重构状态机
	reg [31: 0]  reg_cg_recv_alllen;  				//总的接收数据长度(调试用)  			
	reg [15: 0]  reg_cg_framelen; 				    //当前数据帧长度(链路层数据长度):1048      			
	
	reg [15: 0]  reg_cg_recv_databyte;				// 净荷数据接受长度
	reg [15: 0]  reg_cg_source_id;					// 源ID  0x0057（管控主）0x0058（管控备）
	reg [15: 0]  reg_cg_destin_id;					// 目的ID 0x00A4~0X00A7
	reg [7 : 0]  reg_cg_recv_datatype;				// 净荷类型
	reg [15: 0]  reg_cg_recv_subdeviceid;    		// 子ID, 代表区段标识 高字节00 低字节:71 72 73 74 76 77
	reg [15: 0]  reg_cg_recv_framesum;     	 		// 帧校验 和校验
	reg [15: 0]  reg_cg_recv_datacrc;      	 		// 数据区校验 CRC 


	reg [31: 0]  reg_cg_recv_wdt;         	 		// 接收字节看门狗
	reg [7 : 0]  reg_cg_respond_code;  				// 主控接收XW重构应答码    
	reg [7 : 0]  mem_cg_respond[31: 0];				// 重构应答数据缓存
	reg          reg_cg_respond_start;				// 重构应答开始
	reg    		 reg_cg_respond_busy;				
	reg [10: 0]  reg_cg_sub_step;
	reg [18: 0]  reg_cg_msram_write_cnt;			//MRAM写入计数,1024字节共需要写入512次
	reg [18: 0]  reg_cg_crc_cnt;					//净荷CRC校验计数
	reg [31: 0]  reg_cg_retry_timecnt;				//TXJ重构重试计数
	reg [31: 0]  reg_cg_fpga_timeout_cnt;			//FPGA重构超时计数,接收到一帧完整的数据后,开始计时,按照刷新芯片的帧格式发送
	reg [63: 0]  reg_cg_file_check;					// 重构文件校验
	reg [31: 0]  reg_cg_dsp_reset_timecnt;   		// DSP 重构结束，DSP复位时间
	
	// 重构CRC校验 
	reg [7 : 0]  reg_xw_crc_data;
	wire[15: 0]  wire_xw_crc_result;				//净荷CRC校验结果				
	wire         wire_xw_crc_busy;
	reg          reg_xw_crc_restart;				//CRC模块复位
	reg          reg_xw_crc_onebyte_start_flag;		//CRC模块单字节开始标志

	reg [15:0]	 reg_cg_sum_check;					//链路层重构数据和校验和
	reg [15:0]   reg_cg_total_frame_cnt;			//重构数据帧总数

	reg [15:0]	 reg_cg_resp_check;					//XW响应和校验
	
	crc_16 #(
		.CRC_INIT     (16'h0000)					//初值为0					
	) 	
	crc_16_xw_inst (
		.i_sysclk     (i_sysclk),
		.i_sysrst_n   (reg_xw_crc_restart),
		.i_crcpoly    (16'h8005),
		.i_start_flag (reg_xw_crc_onebyte_start_flag),
		.i_data_in    (reg_xw_crc_data),
		.o_crc_value  (wire_xw_crc_result),
		.o_crc_busy   (wire_xw_crc_busy)
    );
	

	// VIO 设置重构状态
	wire [7 : 0] wire_vio_cg_sel;
	wire [7 : 0] wire_vio_cg_enable;
	wire [7 : 0] wire_vio_cg_device;
	
	// DSP 重构使能
	wire [7 : 0] wire_cg_enable = (wire_vio_cg_sel == 0) ? reg_cg_enable : wire_vio_cg_enable;  

	assign  wire_cg_device =  (wire_vio_cg_sel == 0) ? reg_cg_device : wire_vio_cg_device;
	
	// 重构处理主任务 
	
	always @(posedge i_sysclk or negedge i_sysrst_n)
	begin 
		if (~i_sysrst_n) begin
			reg_to_fresh_num <= 0;
			reg_cg_lasterr_state <= CG_STATE_IDLE;
			reg_cg_state <= CG_STATE_IDLE;
			reg_cg_framelen <= 0;
			reg_cg_recv_alllen <= 0;
			reg_cg_recv_framecnt <= 0;
			reg_cg_recv_wdt <= 0;
			reg_xw_crc_restart <= 0;
			reg_xw_crc_onebyte_start_flag <= 0;	
			reg_cg_txj_trigger <= 1'b0;
			reg_cg_fpga_timeout_cnt <= 0;
			reg_cg_dsp_reset_timecnt <= 0;
			o_mram_addr <= 20'd0;
			o_mram_data <= 16'hFFFF;
			o_mram_ub_n <= 1'b1;
			o_mram_lb_n <= 1'b1;
			o_mram_g_n <= 1'b1;
			o_mram_w_n <= 1'b1;
			o_mram_e_n <= 1'b1;
			reg_to_fresh_send_ycrequest_trigger <= 1'b0;
			reg_xj_fresh_send_ycrequest_trigger <= 1'b0;
			reg_to_fresh_sendmem_trigger <= 1'b0;
			reg_xj_fresh_sendmem_trigger <= 1'b0;
			reg_dsp_cg_reset <= 1'b1;
			reg_to_fresh_cg_fpgareset_trigger <= 1'b0;
			reg_cg_yaoce_state <= 8'h00;  // 0x01表示重构中，0x10表示重构成功，0x11表示重构失败；空闲0x00
			reg_cg_sum_check <= 16'd0;
			reg_cg_total_frame_cnt <= 16'd0;
			reg_cg_resp_check <= 16'd0;
		end
		else begin
			if (
				wire_cg_enable != 8'b11 ||
					(
						wire_cg_enable == 8'b11 &&
						wire_cg_device != CG_DEVICE_DSP &&
						wire_cg_device != CG_DEVICE_XJ &&
						wire_cg_device != CG_DEVICE_TXJ1 &&
						wire_cg_device != CG_DEVICE_TXJ2 &&
						wire_cg_device != CG_DEVICE_TXJ3 &&
						wire_cg_device != CG_DEVICE_FPGA
					)
				) 
			begin
				reg_cg_state <= CG_STATE_IDLE;
				reg_cg_framelen <= 0;
				reg_cg_recv_alllen <= 0;
				reg_cg_recv_framecnt <= 0;
				reg_cg_recv_wdt <= 0;
				reg_xw_crc_restart <= 0;
				reg_xw_crc_onebyte_start_flag <= 0;	
				reg_cg_txj_trigger <= 1'b0;
				
				o_mram_addr <= 20'd0;
				o_mram_data <= 16'hFFFF;
				o_mram_ub_n <= 1'b1;
				o_mram_lb_n <= 1'b1;
				o_mram_g_n <= 1'b1;
				o_mram_w_n <= 1'b1;
				o_mram_e_n <= 1'b1;
				
				reg_to_fresh_cg_fpgareset_trigger <= 1'b0;
				o_dsp_cg_en <= 1'b0;
				reg_cg_yaoce_state <= 8'h00;   						// 0x01表示重构中，0x10表示重构成功，0x11表示重构失败；空闲0x00
				reg_cg_yaoce_device <= 8'h00;  						// 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
				reg_cg_yaoce_error_code <= 8'h00;
				
			end 
			else begin 

				case (reg_cg_state)             
				CG_STATE_IDLE : begin   	// 空闲态
					reg_cg_recv_wdt <= 0;
					reg_cg_sum_check <= 16'd0;
					reg_cg_state <= CG_STATE_WAIT_HEAD1;
				end 
				
				CG_STATE_WAIT_HEAD1 : begin // 0x7E
					reg_xw_crc_restart <= 1'b0;
					reg_xw_crc_onebyte_start_flag <= 0;
					reg_cg_sum_check <= 16'd0;
					reg_cg_recv_endframeflag <= 16'h0000;							
					if (wire_xw_cg_recv_flag) begin
						if (wire_xw_cg_uart_data == 8'h7E) begin
							reg_cg_recv_databyte <= 0;
							reg_cg_recv_wdt <= 0;
							reg_cg_yaoce_state <= 8'h01;  				// 0x01表示重构中，0x10表示重构成功，0x11表示重构失败；空闲0x00
							reg_cg_sum_check <= wire_xw_cg_uart_data;
							reg_cg_state <= CG_STATE_WAIT_HEAD2;							
						end
						else begin
							reg_cg_state <= CG_STATE_WAIT_HEAD1;
						end
					end
				end
				
				CG_STATE_WAIT_HEAD2 : begin // // 0x7E
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						if (wire_xw_cg_uart_data == 8'h7E) begin
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_LEN1;							
						end
						else begin
							reg_cg_state <= CG_STATE_WAIT_HEAD1;
						end
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end 
									
				CG_STATE_LEN1 : begin 	   // 链路层帧长 = 1048(定长)
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_framelen[15: 8] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_LEN2;							
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end		
				
				CG_STATE_LEN2 : begin 	   // 帧长数据2 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_framelen[7: 0] <= wire_xw_cg_uart_data;	
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;					
						reg_cg_state <= CG_STATE_SOURCE_ID1;	
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end	

				CG_STATE_SOURCE_ID1 : begin    // 0X0057 0X0058
					if (reg_cg_framelen != 1048) begin // 固定帧长 =  1048
						reg_cg_state <= CG_STATE_ERR_LEN;		
					end
					else begin
						if (wire_xw_cg_recv_flag) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_source_id[15:8] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_state <= CG_STATE_SOURCE_ID2;		
						end
						else begin 
							reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
							if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
								reg_cg_recv_wdt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;
							end
						end
					end
				end

				CG_STATE_SOURCE_ID2 : begin    // 0X0057 0X0058
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_source_id[7:0] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_DEVID_1;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_DEVID_1 : begin    
					if ((reg_cg_source_id != 16'h0057) && (reg_cg_source_id != 16'h0058)) begin 	
						reg_cg_state <= CG_STATE_ERR_DEVICETYPE;									//源地址错误								
					end	
					else begin
						if (wire_xw_cg_recv_flag) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_destin_id[15:8] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_state <= CG_STATE_DEVID_2;		
						end
						else begin 
							reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
							if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
								reg_cg_recv_wdt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;
							end
						end
					end
				end

				CG_STATE_DEVID_2 : begin    
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_destin_id[7:0] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_REV_FRAME_CNT1;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_REV_FRAME_CNT1 : begin    
					if ((reg_cg_destin_id != 16'h00A4) && (reg_cg_destin_id != 16'h00A5) && (reg_cg_destin_id != 16'h00A6) && (reg_cg_destin_id != 16'h00A7)) begin 	
						reg_cg_state <= CG_STATE_ERR_DEVICETYPE;									//目的地址错误								
					end	
					else begin
						if (wire_xw_cg_recv_flag) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_total_frame_cnt[15:8] <= wire_xw_cg_uart_data;					//总帧数高位
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_state <= CG_STATE_REV_FRAME_CNT2;		
						end
						else begin 
							reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
							if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
								reg_cg_recv_wdt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;
							end
						end
					end
				end

				CG_STATE_REV_FRAME_CNT2 : begin    
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_total_frame_cnt[7:0] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_REV_FRAME_NUM1;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_REV_FRAME_NUM1 : begin    
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_recv_framenum[15:8] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_REV_FRAME_NUM2;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_REV_FRAME_NUM2 : begin    
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_recv_framenum[7:0] <= wire_xw_cg_uart_data;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU1;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_YULIU1 : begin  
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU2;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end		
				
				CG_STATE_YULIU2 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU3;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_YULIU3 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU4;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end	

				CG_STATE_YULIU4 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU5;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_YULIU5 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU6;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end	

				CG_STATE_YULIU6 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU7;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_YULIU7 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_YULIU8;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end				
				
				CG_STATE_YULIU8 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_recv_databyte <= 0;
						reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
						reg_cg_state <= CG_STATE_DATA;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end	

				CG_STATE_DATA : begin 
					if (wire_xw_cg_recv_flag) begin
						if (reg_cg_recv_databyte >= 0 && reg_cg_recv_databyte < CG_VALID_DATA_LEN) begin	  //0~1021		
							reg_cg_recv_wdt <= 0;
							mem_cg_recv_data[reg_cg_recv_databyte] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 						
						else if (reg_cg_recv_databyte == CG_VALID_DATA_LEN) begin						 	//数据内容字段CRC校验
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_datacrc[15: 8] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 
						else if (reg_cg_recv_databyte == (CG_VALID_DATA_LEN + 1)) begin						 //数据内容字段CRC校验
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_datacrc[7 : 0] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 
						else if (reg_cg_recv_databyte == (CG_VALID_DATA_LEN + 2)) begin						//链路层和校验
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_framesum[15: 8] <= wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 
						else if (reg_cg_recv_databyte == (CG_VALID_DATA_LEN + 3)) begin						
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_framesum[7 : 0] <= wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
							reg_cg_state <= CG_STATE_END_CODE1;		
						end 
						else begin 
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
							reg_cg_state <= CG_STATE_END_CODE1;							
						end 
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_END_CODE1 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_state <= CG_STATE_END_CODE2;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end

				CG_STATE_END_CODE2 : begin 
					if (wire_xw_cg_recv_flag) begin
						reg_cg_recv_wdt <= 0;
						reg_cg_state <= CG_STATE_CHECK1;		
					end
					else begin 
						reg_cg_recv_wdt <= reg_cg_recv_wdt + 1;
						if (reg_cg_recv_wdt > CG_WDT_TIMEOUT) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_state <= CG_STATE_ERR_TIMEOUT;
						end
					end
				end
				
				CG_STATE_CHECK1 : begin  // 链路层帧 和校验  	 
					if (reg_cg_sum_check != reg_cg_recv_framesum) begin     
						reg_cg_state <= CG_STATE_CHECK2;
					end
					else begin
						reg_cg_state <= CG_STATE_CHECK2;
					end
				end
				
				CG_STATE_CHECK2 : begin  // 遥控指令设置重构设备  区分是否为首帧
					if (reg_cg_recv_framenum == 0) begin 
						reg_cg_recv_framecnt <= 0;
						reg_cg_state <= CG_STATE_CHECK3;
						reg_cg_sub_step <= 0;
						reg_cg_crc_cnt <= 0;
						reg_xw_crc_restart <= 1'b1;						
					end 
					else begin
						if (reg_cg_recv_framenum != reg_cg_recv_framecnt) begin 
							reg_cg_state <= CG_STATE_ERR_FRAMECNT;
						end
						else begin
							reg_cg_state <= CG_STATE_CHECK3;
							reg_cg_sub_step <= 0;
							reg_cg_crc_cnt <= 0;
							reg_xw_crc_restart <= 1'b1;
						end
					end 
				end
				
				// 0x72：DSP
				// 0x71：相机
                // 0x73：通信机（预留）
                // 0X77：FPGA
				// 其余： 不响应
				CG_STATE_CHECK3 : begin //校验CRC 1022字节数据内容部分
					if (reg_cg_crc_cnt  < CG_VALID_DATA_LEN) begin
					
						case (reg_cg_sub_step)
						0: begin	
						  	reg_xw_crc_data <= mem_cg_recv_data[reg_cg_crc_cnt];
							reg_xw_crc_onebyte_start_flag <= 1'b1;
							reg_cg_sub_step <= reg_cg_sub_step + 1;
						end 
						
						1: begin
							reg_cg_sub_step <= reg_cg_sub_step + 1;
						end
						
						2: begin
							reg_cg_sub_step <= reg_cg_sub_step + 1;
						end

						3: begin
							reg_cg_sub_step <= reg_cg_sub_step + 1;
						end	
						
						4: begin 
							if (wire_xw_crc_busy == 1) begin
								reg_xw_crc_onebyte_start_flag <= 1'b0;
							end
							else begin 
								reg_cg_sub_step <= reg_cg_sub_step + 1;
							end 
						end
						
						5: begin 
							reg_cg_crc_cnt <= reg_cg_crc_cnt + 1;
							reg_cg_sub_step <= 0;
						end	
						
						default: begin 
							reg_cg_sub_step <= 0; 
						end 
						endcase 
					end					
					else begin 
						if (wire_xw_crc_result == reg_cg_recv_datacrc) begin   
							reg_cg_msram_write_cnt <= 0;
							reg_cg_state <= CG_STATE_CHECK_END;
						end 
						else begin 														//校验错误,没做处理
							reg_cg_msram_write_cnt <= 0;
							reg_cg_state <= CG_STATE_CHECK_END;						
						end
					end
				end	
				
				CG_STATE_CHECK_END : begin 												// 校验是否为最后一帧
					reg_xw_crc_restart <= 1'b0;
					if ((reg_cg_recv_framecnt + 16'd1) >= reg_cg_total_frame_cnt) begin 
        				reg_cg_recv_endframeflag <= 16'h00AA;    						// 尾帧标志
    				end
					else begin
						reg_cg_recv_endframeflag <= 16'h0000; 
					end
					if (wire_cg_device == CG_DEVICE_DSP) begin
						o_dsp_cg_en <= 1'b1;
					end
					reg_cg_txj_trigger <= 1'b0;
					reg_cg_retry_timecnt <= 0;
					reg_cg_dsp_reset_timecnt <= 0;
					reg_cg_state <= CG_STATE_HANDLER;
				end 
				
				CG_STATE_CHECK_END_ALLFILE : begin 										// 重构文件结束
//					reg_cg_file_check <= {mem_cg_recv_data[6], mem_cg_recv_data[7], mem_cg_recv_data[8], mem_cg_recv_data[9], mem_cg_recv_data[10], mem_cg_recv_data[11], mem_cg_recv_data[12], mem_cg_recv_data[13]};
					if (wire_cg_device == CG_DEVICE_DSP) begin
						if (reg_cg_dsp_reset_timecnt < 32'd500_000_00) begin 
							reg_dsp_cg_reset <= 1'b0; 
							reg_cg_dsp_reset_timecnt <= reg_cg_dsp_reset_timecnt + 1;
							o_dsp_cg_en <= 1'b0;
							reg_cg_state <= CG_STATE_CHECK_END_ALLFILE;
						end 
						else begin
							reg_dsp_cg_reset <= 1'b1;
							reg_cg_dsp_reset_timecnt <= 0;
							reg_cg_state <= CG_STATE_SUCCESS;
						end 
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // 通信机 1 区段
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // 通信机 2 区段
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // 通信机 3 区段  
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_FPGA) begin  // 重构FPGA, 此时应该发一个遥测请求
						reg_cg_state <= CG_STATE_SUCCESS;
					end					
					else if (wire_cg_device == CG_DEVICE_XJ) begin    // 相机 
						reg_cg_state <= CG_STATE_SUCCESS;
					end 
					else begin 
						reg_cg_state <= CG_STATE_ERR_OTHER;
					end
				end 
				
				// 重构一帧接收成功处理	, DSP 写入MSRAM, 其他单机触发转发			
				CG_STATE_HANDLER : begin	
					if (wire_cg_device == CG_DEVICE_DSP) begin
						reg_cg_yaoce_device <= 8'h01;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if (reg_cg_msram_write_cnt < 1024) begin
							case (reg_cg_sub_step)
								0: begin 
									if (reg_cg_recv_framenum < (128)) begin 
										o_mram_addr <= (reg_cg_recv_framenum * 1024 + reg_cg_msram_write_cnt) / 2;
									end
									else if (reg_cg_recv_framenum >= 128 && reg_cg_recv_framenum < 192) begin 
										o_mram_addr <= ((reg_cg_recv_framenum + 64) * 1024 + reg_cg_msram_write_cnt) / 2;
									end 
									else if (reg_cg_recv_framenum >= 192 && reg_cg_recv_framenum < (192+128)) begin 
										o_mram_addr <= 20'h80000 + ((reg_cg_recv_framenum - 192) * 1024 + reg_cg_msram_write_cnt) / 2;
									end 
									else begin 
										o_mram_addr <= 20'h80000 + ((reg_cg_recv_framenum - 192 + 64) * 1024 + reg_cg_msram_write_cnt) / 2;					
									end
									
									o_mram_data[7 : 0] <= mem_cg_recv_data[6 + reg_cg_msram_write_cnt];
									o_mram_data[15: 8] <= mem_cg_recv_data[6 + reg_cg_msram_write_cnt + 1];	
									o_mram_ub_n <= 1'b0;
									o_mram_lb_n <= 1'b0;
									o_mram_g_n <= 1'b1;
									o_mram_w_n <= 1'b1;
									o_mram_e_n <= 1'b1;
									reg_cg_sub_step <= reg_cg_sub_step + 1;
								end							
								1, 2: begin
									o_mram_e_n <= 1'b0;
									reg_cg_sub_step <= reg_cg_sub_step + 1;
								end
								3, 4, 5, 6: begin
									o_mram_w_n <= 1'b0;
									reg_cg_sub_step <= reg_cg_sub_step + 1;
								end							
								7: begin
									o_mram_w_n <= 1'b1;
									reg_cg_sub_step <= reg_cg_sub_step + 1;
								end								
								8: begin
									o_mram_e_n <= 1'b1;
									reg_cg_sub_step <= reg_cg_sub_step + 1;
								end								
								9: begin
									reg_cg_msram_write_cnt <= reg_cg_msram_write_cnt + 2;
									reg_cg_sub_step <= 0;
								end	
								default: begin	reg_cg_sub_step <= 0; end
							endcase 
						end
						else begin  
							reg_cg_state <= CG_STATE_HANDLER_WAITING;
						end 
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // 通信机 1 区段
						reg_cg_yaoce_device <= 8'h22;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if ((reg_txj_cg_tx_busy != 0) && (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT)) begin    
							reg_cg_txj_trigger <= 1'b0;
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end
						else begin 
							if (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT) begin
								reg_cg_txj_trigger <= 1'b1;
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_WAITING;									
							end 
							else begin 
								reg_cg_txj_trigger <= 1'b0;
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
							end 
						end 
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // 通信机 2 区段
						reg_cg_yaoce_device <= 8'h22;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if (reg_txj_cg_all_framecnt > 0) begin 
							if ((reg_txj_cg_tx_busy != 0) && (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT)) begin    
								reg_cg_txj_trigger <= 1'b0;
								reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
							end
							else begin 
								if (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT) begin
									reg_cg_txj_trigger <= 1'b1;
									reg_cg_retry_timecnt <= 0;
									reg_cg_state <= CG_STATE_HANDLER_WAITING;									
								end 
								else begin 
									reg_cg_txj_trigger <= 1'b0;
									reg_cg_retry_timecnt <= 0;
									reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
								end 
							end
						end 
						else begin 
							reg_cg_state <= CG_STATE_ERR_FRAMECNT;
						end 
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // 通信机 3 区段  
						reg_cg_yaoce_device <= 8'h22;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if (reg_txj_cg_all_framecnt > 0) begin 
							if ((reg_txj_cg_tx_busy != 0) && (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT)) begin    
								reg_cg_txj_trigger <= 1'b0;
								reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
							end
							else begin 
								if (reg_cg_retry_timecnt < TXJ_CG_TIMEOUT) begin
									reg_cg_txj_trigger <= 1'b1;
									reg_cg_retry_timecnt <= 0;
									reg_cg_state <= CG_STATE_HANDLER_WAITING;									
								end 
								else begin 
									reg_cg_txj_trigger <= 1'b0;
									reg_cg_retry_timecnt <= 0;
									reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
								end 
							end 
						end 
						else begin 
							reg_cg_state <= CG_STATE_ERR_FRAMECNT;
						end 
					end 	
					else if (wire_cg_device == CG_DEVICE_FPGA) begin  
						reg_cg_yaoce_device <= 8'h10;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if (reg_cg_msram_write_cnt < CG_VALID_DATA_LEN) begin
							mem_cg_to_fresh_mem[reg_cg_msram_write_cnt] <= mem_cg_recv_data[reg_cg_msram_write_cnt];
							reg_cg_msram_write_cnt <= reg_cg_msram_write_cnt + 1;
							reg_to_fresh_num <= CG_VALID_DATA_LEN;
						end 
						else begin
							reg_cg_fpga_timeout_cnt <= 0;
							reg_to_fresh_sendmem_trigger <= 1;
							reg_cg_state <= CG_STATE_HANDLER_WAITING;
						end 
					end					
					else if (wire_cg_device == CG_DEVICE_XJ) begin    
						reg_cg_yaoce_device <= 8'h11;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						if (reg_cg_msram_write_cnt < CG_VALID_DATA_LEN) begin
							mem_cg_xj_fresh_mem [reg_cg_msram_write_cnt] <= mem_cg_recv_data[reg_cg_msram_write_cnt];
							reg_cg_msram_write_cnt <= reg_cg_msram_write_cnt + 1;
							reg_xj_fresh_num <= CG_VALID_DATA_LEN;
						end 
						else begin
							reg_cg_fpga_timeout_cnt <= 0;
							reg_xj_fresh_sendmem_trigger <= 1;
							reg_cg_state <= CG_STATE_HANDLER_WAITING;
						end 
					end 
					else begin 
						reg_cg_yaoce_device <= 8'h00;  // 0x01表示DSP，0x10表示FPGA，0x11表示相机；默认0x00
						reg_cg_state <= CG_STATE_ERR_SUBDEVICE;					
					end 
				end	
				
				// 等待转发结束，判断转发状态
				CG_STATE_HANDLER_WAITING : begin   
					reg_xw_crc_restart <= 1'b0;	
					if (wire_cg_device == CG_DEVICE_DSP) begin        			// DSP 直接重构进MSRAM，无需等待
						o_dsp_cg_en <= 1'b0;
						reg_cg_state <= CG_STATE_HANDLER_DONE;
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  			// 通信机 1 区段
						reg_cg_txj_trigger <= 1'b0;
						if ((reg_txj_cg_recv_sucess_state != TXJ_CG_RECV_DONE) && (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT)) begin    
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end
						else begin 
							if (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT) begin
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;									
							end 
							else begin 
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
							end 
						end					
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // 通信机 2 区段
						reg_cg_txj_trigger <= 1'b0;
						if ((reg_txj_cg_recv_sucess_state != TXJ_CG_RECV_DONE) && (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT)) begin    
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end
						else begin 
							if (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT) begin
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;									
							end 
							else begin 
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
							end 
						end									
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // 通信机 3 区段  
						reg_cg_txj_trigger <= 1'b0;
						if ((reg_txj_cg_recv_sucess_state != TXJ_CG_RECV_DONE) && (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT)) begin    
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end
						else begin 
							if (reg_cg_retry_timecnt < 2 * TXJ_CG_TIMEOUT) begin
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;									
							end 
							else begin 
								reg_cg_retry_timecnt <= 0;
								reg_cg_state <= CG_STATE_ERR_TIMEOUT;								
							end 
						end					
					end 
					else if (wire_cg_device == CG_DEVICE_FPGA) begin  		// AAAAA
						if (reg_cg_fpga_timeout_cnt < 32'd1_200_00) begin   // 等待 1.2ms 触发
							reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
						end 
						else begin 
							if (reg_to_fresh_send_mem == 1) begin 
								reg_to_fresh_sendmem_trigger <= 0;
								reg_cg_fpga_timeout_cnt <= 0;								
							end 
							else begin 
								reg_to_fresh_sendmem_trigger <= 0;			//当reg_to_fresh_send_mem_clr置1,即发送完成后跳转
								reg_cg_fpga_timeout_cnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;								
							end 
						end
					end					
					else if (wire_cg_device == CG_DEVICE_XJ) begin    		// 相机 
						if (reg_cg_fpga_timeout_cnt < 32'd1_200_00) begin   // 等待 1ms 触发
							reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
						end 
						else begin 
							if (reg_xj_fresh_send_mem == 1) begin 
								reg_xj_fresh_sendmem_trigger <= 0;
								reg_cg_fpga_timeout_cnt <= 0;								
							end 
							else begin 
								reg_xj_fresh_sendmem_trigger <= 0;
								reg_cg_fpga_timeout_cnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;								
							end 
						end
					end 
					else begin 
						reg_cg_state <= CG_STATE_ERR_OTHER;
					end 				
				end	
				
				// 返回结果
				CG_STATE_HANDLER_DONE : begin   
					if (wire_cg_device == CG_DEVICE_DSP) begin        // DSP 直接重构进MSRAM，无需等待
						reg_cg_state <= CG_STATE_SUCCESS;
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // 通信机 1 区段
						//0xAA：接收正确
						//0x2： 单帧校验出错
						//0x3： 帧地址不连续
						//0x4： 帧序号不连续
						//0x9： 帧长度错误
						//0xF1：数据文件校验错误
						//0xFF：其他错误
						case (reg_txj_cg_respond_code) 
						8'hAA: begin reg_cg_state <= CG_STATE_SUCCESS; end 
						8'h02: begin reg_cg_state <= CG_STATE_ERR_CRC; end 
						8'h03: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end
						8'h04: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end 
						8'h09: begin reg_cg_state <= CG_STATE_ERR_LEN; end 
						8'hF1: begin reg_cg_state <= CG_STATE_ERR_FILE; end 
						default: begin reg_cg_state <= CG_STATE_ERR_OTHER; end
						endcase 
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // 通信机 2 区段
						case (reg_txj_cg_respond_code) 
						8'hAA: begin reg_cg_state <= CG_STATE_SUCCESS; end 
						8'h02: begin reg_cg_state <= CG_STATE_ERR_CRC; end 
						8'h03: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end
						8'h04: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end 
						8'h09: begin reg_cg_state <= CG_STATE_ERR_LEN; end 
						8'hF1: begin reg_cg_state <= CG_STATE_ERR_FILE; end 
						default: begin reg_cg_state <= CG_STATE_ERR_OTHER; end
						endcase 					
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // 通信机 3 区段  
 						case (reg_txj_cg_respond_code) 
						8'hAA: begin reg_cg_state <= CG_STATE_SUCCESS; end 
						8'h02: begin reg_cg_state <= CG_STATE_ERR_CRC; end 
						8'h03: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end
						8'h04: begin reg_cg_state <= CG_STATE_ERR_FRAMECNT; end 
						8'h09: begin reg_cg_state <= CG_STATE_ERR_LEN; end 
						8'hF1: begin reg_cg_state <= CG_STATE_ERR_FILE; end 
						default: begin reg_cg_state <= CG_STATE_ERR_OTHER; end
						endcase 					
					end 					
					else if (wire_cg_device == CG_DEVICE_FPGA) begin  // BBBBB
						if (reg_cg_recv_framenum < 100) begin 
							if (reg_cg_fpga_timeout_cnt < 32'd2500_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // 等待 2.5S 触发
								reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
							end 			
							else begin 
								//if (reg_to_fresh_flash_success == 1) begin 
									reg_cg_fpga_timeout_cnt <= 0;
									reg_cg_state <= CG_STATE_SUCCESS;
								//end 
								//else begin 
								//	reg_cg_fpga_timeout_cnt <= 0;
								//	reg_cg_state <= CG_STATE_SUCCESS;
								//end 
							end 
						end 
						else begin 
							if (reg_cg_fpga_timeout_cnt < 32'd150_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // 等待 150ms 触发
								reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
							end 			
							else begin 
								//if (reg_to_fresh_flash_success == 1) begin 
								//	reg_cg_fpga_timeout_cnt <= 0;
								//	reg_cg_state <= CG_STATE_SUCCESS;
								//end 
								//else begin 
									reg_cg_fpga_timeout_cnt <= 0;
									reg_cg_state <= CG_STATE_SUCCESS;
								//end 
							end					
						end 
   				    end
					else if (wire_cg_device == CG_DEVICE_XJ) begin    // 相机 
						if (reg_cg_recv_framenum < 100) begin 
							if (reg_cg_fpga_timeout_cnt < 32'd2500_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // 等待 2.5S 触发
								reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
							end 			
							else begin 
								//if (reg_to_fresh_flash_success == 1) begin 
									reg_cg_fpga_timeout_cnt <= 0;
									reg_cg_state <= CG_STATE_SUCCESS;
								//end 
								//else begin 
								//	reg_cg_fpga_timeout_cnt <= 0;
								//	reg_cg_state <= CG_STATE_SUCCESS;
								//end 
							end 
						end 
						else begin 
							if (reg_cg_fpga_timeout_cnt < 32'd150_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // 等待 150ms 触发
								reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
							end 			
							else begin 
								//if (reg_to_fresh_flash_success == 1) begin 
								//	reg_cg_fpga_timeout_cnt <= 0;
								//	reg_cg_state <= CG_STATE_SUCCESS;
								//end 
								//else begin 
									reg_cg_fpga_timeout_cnt <= 0;
									reg_cg_state <= CG_STATE_SUCCESS;
								//end 
							end					
						end 		
					end 
					else begin 
						reg_cg_state <= CG_STATE_ERR_SUBDEVICE;
					end 				
				end	
				
				CG_STATE_RESPOND : begin  	       // 填充回复帧
					if (reg_cg_respond_busy == 1) begin
						reg_cg_state <= CG_STATE_RESPOND;
					end
					else begin
						mem_cg_respond[0]  <= 8'h7E;						// 帧头	
						mem_cg_respond[1]  <= 8'h7E;
						mem_cg_respond[2]  <= 8'h00;						// 长度0x001A
						mem_cg_respond[3]  <= 8'h1A; 							 					
						mem_cg_respond[4]  <= 8'd00; 	 					 
						mem_cg_respond[5]  <= MAIN_CG_DEVICE_ID; 	 		// 源地址					
						mem_cg_respond[6]  <= 8'h00; 	 					// 目的地址 
						mem_cg_respond[7]  <= reg_cg_source_id[7:0]; 			
						mem_cg_respond[8]  <= reg_cg_total_frame_cnt[15:8]; // 总帧数
						mem_cg_respond[9]  <= reg_cg_total_frame_cnt[7:0]; 
						mem_cg_respond[10] <= reg_cg_recv_framenum[15:8];	// 本次应答的帧编号 
						mem_cg_respond[11] <= reg_cg_recv_framenum[7:0]; 						
						mem_cg_respond[12] <= 8'h00; 						// 预留6字节
						mem_cg_respond[13] <= 8'h00; 						// 预留6字节
						mem_cg_respond[14] <= 8'h00; 						// 预留6字节
						mem_cg_respond[15] <= 8'h00; 						// 预留6字节
						mem_cg_respond[16] <= 8'h00; 						// 预留6字节
						mem_cg_respond[17] <= wire_cg_device[7 : 0]; 		// 待重构的设备
						mem_cg_respond[18] <= 8'h00;						// 重构结果
						case (reg_cg_respond_code)
							8'hAA: 
								mem_cg_respond[19] <= 8'h01;	
							default:
								mem_cg_respond[19] <= reg_cg_respond_code; 
						endcase
						mem_cg_respond[20] <= 0;   							// 保留2BYTE
						mem_cg_respond[21] <= 0;   							// 保留2BYTE
						mem_cg_respond[24] <= 8'h0A;   						// 结束码
						mem_cg_respond[25] <= 8'h0D;   												     
						// reg_xw_crc_restart <= 1;
						reg_cg_crc_cnt <= 0;
						reg_cg_resp_check <= 16'd0;
						reg_cg_state <= CG_STATE_RESPOND_CRC;
						if (reg_cg_respond_code == 8'hAA) begin
							reg_cg_recv_alllen <= reg_cg_recv_framenum * 1024 + 1024;
							reg_cg_recv_framecnt <= reg_cg_recv_framecnt + 1;
						end 
						
						if (reg_cg_recv_endframeflag == 16'h00AA && (wire_cg_device == CG_DEVICE_TXJ1 || wire_cg_device == CG_DEVICE_TXJ2 || wire_cg_device == CG_DEVICE_TXJ3)) begin 
							if (reg_cg_respond_code == 8'hAA) begin 
								if (wire_cg_device == CG_DEVICE_TXJ1) begin 
									reg_txj_cg_all_framecnt <= reg_cg_recv_framenum;
								end 
								else begin
									reg_txj_cg_all_framecnt <= reg_txj_cg_all_framecnt + reg_cg_recv_framenum;
								end 
							end 
						end 
						
						if (reg_cg_recv_endframeflag == 16'h00AA) begin 
							if (reg_cg_respond_code == 8'hAA) begin 
								reg_cg_yaoce_state <= 8'h10;  // 0x01表示重构中，0x10表示重构成功，0x11表示重构失败；空闲0x00
							end 
							else begin 
								reg_cg_yaoce_state <= 8'h11; 
							end
						end
			
					end
				end	
				
				CG_STATE_RESPOND_CRC : begin       // CRC 性能
					if (reg_cg_crc_cnt  < 22) begin
						reg_cg_resp_check <= reg_cg_resp_check + mem_cg_respond[reg_cg_crc_cnt];
					end					
					else begin 
						mem_cg_respond[22] <= reg_cg_resp_check[15: 8];
						mem_cg_respond[23] <= reg_cg_resp_check[7 : 0];
						reg_cg_respond_start <= 1'b1;
						reg_cg_crc_cnt <= 0;
						reg_cg_state <= CG_STATE_RESPOND_WAITING;
					end
				end 
				
				CG_STATE_RESPOND_WAITING : begin   // 等待
					reg_cg_retry_timecnt <= 32'd0;
					reg_cg_state <= CG_STATE_RESPOND_DONE;
				end		
				
				CG_STATE_RESPOND_DONE : begin      // 回复帧是一个独立的处理流程
					reg_cg_respond_start <= 1'b0;
					if (reg_cg_recv_endframeflag == 16'h00AA && (wire_cg_device == CG_DEVICE_FPGA) && reg_cg_respond_code == 8'hAA) begin 
						if (reg_cg_retry_timecnt < TIME_1S_CNT) begin
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end 
						else if (reg_cg_retry_timecnt >= TIME_1S_CNT && reg_cg_retry_timecnt < 2 * TIME_1S_CNT) begin
							reg_to_fresh_cg_fpgareset_trigger <= 1'b1;
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
						end 
						begin 
							reg_cg_retry_timecnt <= 0;
							reg_to_fresh_cg_fpgareset_trigger <= 1'b0;
							reg_cg_state <= CG_STATE_WAIT_HEAD1;
						end 
					end
					else if (reg_cg_recv_endframeflag == 16'h00AA && (wire_cg_device == CG_DEVICE_DSP)) begin
						if (reg_cg_retry_timecnt < 32'd500_000_00) begin 
							reg_dsp_cg_reset <= 1'b0; 
							reg_cg_retry_timecnt <= reg_cg_retry_timecnt + 1;
							o_dsp_cg_en <= 1'b0;
						end 
						else begin
							reg_dsp_cg_reset <= 1'b1;
							reg_cg_retry_timecnt <= 0;
							reg_cg_state <= CG_STATE_WAIT_HEAD1;
						end 
					end 	 
					else begin 
						reg_cg_state <= CG_STATE_WAIT_HEAD1;
					end 
				end	
				
				// 0xF0：单帧数据校验错误
				// 0xF1：整个数据文件校验错误
				// 0xF3：加注单元标识错误
				// 0xF4：传输帧序号错误
				// 0xF5：传输帧长度错误；
				// 0xFF：其他失败
				CG_STATE_ERR_FILE : begin          // 单帧数据校验错误
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hF1;
					reg_cg_yaoce_error_code <=  8'hF1;
					reg_cg_state <= CG_STATE_RESPOND;	
				end 				
				
				CG_STATE_ERR_CRC : begin        
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hF0;
					reg_cg_yaoce_error_code <=  8'hF0;
					reg_cg_state <= CG_STATE_RESPOND;	
				end 

				CG_STATE_ERR_LEN : begin     	   
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hF5;
					reg_cg_yaoce_error_code <= 8'hF5;
					reg_cg_state <= CG_STATE_RESPOND;
				end		
				
				CG_STATE_ERR_TIMEOUT : begin       
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hFF;
					reg_cg_yaoce_error_code <= 8'hFF;
					reg_cg_state <= CG_STATE_RESPOND;
				end		
				
				CG_STATE_ERR_SUBDEVICE,        
				CG_STATE_ERR_DEVICETYPE,  	    
				CG_STATE_ERR_DATATYPE : begin      
					reg_cg_respond_code <= 8'hF3; 
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_yaoce_error_code <= 8'hF3;
					reg_cg_state <= CG_STATE_RESPOND;
				end
				
				CG_STATE_ERR_OTHER : begin   
					reg_cg_respond_code <= 8'hFF; 
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_yaoce_error_code <= 8'hFF;
					reg_cg_state <= CG_STATE_RESPOND;
				end 
				
				CG_STATE_ERR_FRAMECNT : begin    
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hF4;
					reg_cg_yaoce_error_code <= 8'hF4;
					reg_cg_state <= CG_STATE_RESPOND;
				end	
						
				
				CG_STATE_SUCCESS : begin 
					reg_cg_lasterr_state <= reg_cg_state;
					reg_cg_respond_code <= 8'hAA;
					reg_cg_yaoce_error_code <= 8'hAA;
					reg_cg_state <= CG_STATE_RESPOND;				
				end 
				
				default: begin
					reg_cg_state <= CG_STATE_IDLE;
				end 
				
				endcase 
			end
		end 
	end 
	

	/*== 重构响应数据 ==*/
	wire   			 wire_xw_cg_uart_tx_busy;
	reg    			 reg_xw_cg_uart_en;
	reg [7:0]   	 reg_xw_cg_uart_din;
	
	uart_send_odd   #(
        .UART_BPS 	 (1_000_000),
        .CLK_FREQ 	 (100_000_000))
	uart_send_odd_xw_cg_inst (
        .sys_clk     (i_sysclk),                  
        .sys_rst_n   (i_sysrst_n),              	
        .uart_en     (reg_xw_cg_uart_en),         
        .uart_din    (reg_xw_cg_uart_din),        
        .uart_tx_busy(wire_xw_cg_uart_tx_busy),       
        .uart_txd    (o_xw_cg_uart_txd)         		
    ); 
	

	/*== 重构响应 ==*/
	parameter   CG_RESPOND_STATE_IDLE = 0, CG_RESPOND_STATE_SEND = 1, CG_RESPOND_STATE_WAITBUSY = 2, CG_RESPOND_STATE_WAITDONE = 3;
	reg [7 : 0]	reg_cg_respond_cnt;					
	reg [7 : 0] reg_cg_respond_state;
	
/*==星务重构响应ila*/
   	ila_txj ila_xw_respond_inst (
		.clk (i_sysclk),
		.probe0 (reg_xw_cg_uart_en),
		.probe1 (reg_cg_respond_cnt),
		.probe2 (reg_xw_cg_uart_din)
	);	

	always @(posedge i_sysclk or negedge i_sysrst_n) 
	begin
		if (~i_sysrst_n) begin
			reg_cg_respond_cnt <= 0;
			reg_cg_respond_busy<= 0;
			reg_cg_respond_state <= CG_RESPOND_STATE_IDLE;
		end 
		else begin
			case (reg_cg_respond_state)  
				CG_RESPOND_STATE_IDLE:	begin
					if (reg_cg_respond_start) begin
						reg_cg_respond_cnt <= 0;
						reg_cg_respond_busy<= 1;
						reg_cg_respond_state <= CG_RESPOND_STATE_SEND;
					end
				end
				
				CG_RESPOND_STATE_SEND: begin
					if (wire_xw_cg_uart_tx_busy == 0) begin
						reg_xw_cg_uart_din <= mem_cg_respond[reg_cg_respond_cnt];
						reg_xw_cg_uart_en <= 1'b1;
						reg_cg_respond_state <= CG_RESPOND_STATE_WAITBUSY;
					end
				end 

				CG_RESPOND_STATE_WAITBUSY: begin 
					if (wire_xw_cg_uart_tx_busy == 0) begin
						reg_cg_respond_state <= CG_RESPOND_STATE_WAITBUSY;
					end
					else begin
						reg_cg_respond_cnt <= reg_cg_respond_cnt + 1;
						reg_xw_cg_uart_en <= 1'b0;
						reg_cg_respond_state <= CG_RESPOND_STATE_WAITDONE;
					end
				end

				CG_RESPOND_STATE_WAITDONE: begin
					if (wire_xw_cg_uart_tx_busy == 1) begin
						reg_cg_respond_state <= CG_RESPOND_STATE_WAITDONE;
					end
					else begin
						if (reg_cg_respond_cnt < 20) begin 
							reg_cg_respond_state <= CG_RESPOND_STATE_SEND;
						end 
						else begin
							reg_cg_respond_state <= CG_RESPOND_STATE_IDLE;
							reg_cg_respond_busy <= 0;
							reg_cg_respond_cnt <= 0;
						end
					end					
				end
				
				default: begin
					reg_cg_respond_cnt <= 0;
					reg_cg_respond_busy<= 0;
					reg_cg_respond_state <= CG_RESPOND_STATE_IDLE;				
				end 
				
			endcase 
		end 
	end