	/*== �����ع� 1Mbps 8 odd 1 XWCG==*/
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

	/*==�����ع�����ila*/
   	ila_txj ila_xw_rev_inst (
		.clk (i_sysclk),
		.probe0 (wire_xw_cg_uart_data_done_flag),
		.probe1 (reg_cg_recv_databyte[7:0]),
		.probe2 (wire_xw_cg_uart_data)
	);		

	/*== �ع�״̬ ==*/
	// ��ַ��ַ,�����ع�֡���ݴ���
	localparam 	CG_FRAME_DATA_BASEADDR = 19;

	// �����ع�״̬���ĸ���״̬
	parameter  	CG_STATE_IDLE    = 0,   			// ����״̬
				CG_STATE_WAIT_HEAD1 = 1, 			// �ȴ�֡ͷ1 7E
				CG_STATE_WAIT_HEAD2 = 2; 			// �ȴ�֡ͷ2 7E
				// CG_STATE_WAIT_HEAD3 = 3, 			// �ȴ�֡ͷ3 
				// CG_STATE_WAIT_HEAD4 = 4; 			// �ȴ�֡ͷ4

	parameter  	CG_STATE_LEN1   = 3,   				// �������ֽ�1
				CG_STATE_LEN2   = 4;     			// �������ֽ�2 
	
	parameter  	CG_STATE_SOURCE_ID1 = 5;   			// Դ��ַ
	parameter  	CG_STATE_SOURCE_ID2 = 6;   			// Դ��ַ

	parameter  	CG_STATE_DEVID_1 = 7,   			// Ŀ��ID��λ
				CG_STATE_DEVID_2  = 8,     			// Ŀ��ID��λ
				CG_STATE_REV_FRAME_CNT1 = 9,   		// ��֡��
				CG_STATE_REV_FRAME_CNT2 = 10,   	// ��֡�� 
				CG_STATE_REV_FRAME_NUM1 = 11,		// ��ǰ֡���
				CG_STATE_REV_FRAME_NUM2 = 12;		 
	parameter	CG_STATE_YULIU1 = 40,    			// Ԥ���ֽ�1
				CG_STATE_YULIU2 = 41,    			// Ԥ���ֽ�2 
				CG_STATE_YULIU3 = 42,    			// Ԥ���ֽ�3
				CG_STATE_YULIU4 = 43,    			// Ԥ���ֽ�4 
				CG_STATE_YULIU5 = 44,    			// Ԥ���ֽ�5
				CG_STATE_YULIU6 = 45,    			// Ԥ���ֽ�6 
				CG_STATE_YULIU7 = 46,    			// Ԥ���ֽ�7
				CG_STATE_YULIU8 = 47;    			// Ԥ���ֽ�8


	parameter  	CG_STATE_DATA = 13,     			// ���ݴ���״̬
				CG_STATE_END_CODE1 = 48,			// ������1
				CG_STATE_END_CODE2 = 49,			// ������2
				CG_STATE_CHECK1 = 14,    			// У��״̬1
				CG_STATE_CHECK2 = 15,    			// У��״̬2
				CG_STATE_CHECK3 = 16,    			// У��״̬3
				CG_STATE_CHECK_END = 17, 			// У�����״̬
				CG_STATE_CHECK_END_ALLFILE = 18; 	// �����ļ�У�����״̬      
	parameter   CG_STATE_HANDLER = 19;				//�ع������߼�,DSP д��MSRAM, ������������ת��
	parameter   CG_STATE_HANDLER_WAITING = 20;		//�ȴ��������
	parameter   CG_STATE_HANDLER_DONE = 21;			//�ع��������
	parameter   CG_STATE_RESPOND = 22,  CG_STATE_RESPOND_CRC = 23, CG_STATE_RESPOND_WAITING = 24, CG_STATE_RESPOND_DONE = 25;
	parameter   CG_STATE_ERR_LEN = 128;				//֡���ȴ���
	parameter   CG_STATE_ERR_SUBDEVICE = 129;		//���ع����豸�ͷ������ݲ���
	parameter   CG_STATE_ERR_DEVICETYPE = 130;		//Դ��Ŀ�ĵ�ַ���ʹ���
	parameter   CG_STATE_ERR_DATATYPE = 131;		//�������ʹ��� 0x02:�ع����� 0xF2:�ع���Ӧ֡
	parameter   CG_STATE_ERR_FRAMECNT = 132;		//֡��Ŵ���,���Ѿ����յ���֡�����Բ���(���ַ�����֧������???)
	parameter   CG_STATE_ERR_TIMEOUT = 133;			//��ʱ������
	parameter   CG_STATE_ERR_CRC = 134;				//֡У�����
	parameter   CG_STATE_ERR_FILE = 135;			//�ļ�У�����
	parameter   CG_STATE_ERR_SUM = 136;				//��·���У�����
	parameter   CG_STATE_ERR_OTHER = 254;			//��������
	parameter   CG_STATE_SUCCESS = 255;				//�ع��ɹ�
	localparam  CG_WDT_TIMEOUT = 32'd100_000_000;
	reg [7 : 0]  reg_cg_lasterr_state;				//�ϴδ���״̬,���Բ鿴ʹ��
	reg [7 : 0]  reg_cg_state;						//�ع�״̬��
	reg [31: 0]  reg_cg_recv_alllen;  				//�ܵĽ������ݳ���(������)  			
	reg [15: 0]  reg_cg_framelen; 				    //��ǰ����֡����(��·�����ݳ���):1048      			
	
	reg [15: 0]  reg_cg_recv_databyte;				// �������ݽ��ܳ���
	reg [15: 0]  reg_cg_source_id;					// ԴID  0x0057���ܿ�����0x0058���ܿر���
	reg [15: 0]  reg_cg_destin_id;					// Ŀ��ID 0x00A4~0X00A7
	reg [7 : 0]  reg_cg_recv_datatype;				// ��������
	reg [15: 0]  reg_cg_recv_subdeviceid;    		// ��ID, �������α�ʶ ���ֽ�00 ���ֽ�:71 72 73 74 76 77
	reg [15: 0]  reg_cg_recv_framesum;     	 		// ֡У�� ��У��
	reg [15: 0]  reg_cg_recv_datacrc;      	 		// ������У�� CRC 


	reg [31: 0]  reg_cg_recv_wdt;         	 		// �����ֽڿ��Ź�
	reg [7 : 0]  reg_cg_respond_code;  				// ���ؽ���XW�ع�Ӧ����    
	reg [7 : 0]  mem_cg_respond[31: 0];				// �ع�Ӧ�����ݻ���
	reg          reg_cg_respond_start;				// �ع�Ӧ��ʼ
	reg    		 reg_cg_respond_busy;				
	reg [10: 0]  reg_cg_sub_step;
	reg [18: 0]  reg_cg_msram_write_cnt;			//MRAMд�����,1024�ֽڹ���Ҫд��512��
	reg [18: 0]  reg_cg_crc_cnt;					//����CRCУ�����
	reg [31: 0]  reg_cg_retry_timecnt;				//TXJ�ع����Լ���
	reg [31: 0]  reg_cg_fpga_timeout_cnt;			//FPGA�ع���ʱ����,���յ�һ֡���������ݺ�,��ʼ��ʱ,����ˢ��оƬ��֡��ʽ����
	reg [63: 0]  reg_cg_file_check;					// �ع��ļ�У��
	reg [31: 0]  reg_cg_dsp_reset_timecnt;   		// DSP �ع�������DSP��λʱ��
	
	// �ع�CRCУ�� 
	reg [7 : 0]  reg_xw_crc_data;
	wire[15: 0]  wire_xw_crc_result;				//����CRCУ����				
	wire         wire_xw_crc_busy;
	reg          reg_xw_crc_restart;				//CRCģ�鸴λ
	reg          reg_xw_crc_onebyte_start_flag;		//CRCģ�鵥�ֽڿ�ʼ��־

	reg [15:0]	 reg_cg_sum_check;					//��·���ع����ݺ�У���
	reg [15:0]   reg_cg_total_frame_cnt;			//�ع�����֡����

	reg [15:0]	 reg_cg_resp_check;					//XW��Ӧ��У��
	
	crc_16 #(
		.CRC_INIT     (16'h0000)					//��ֵΪ0					
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
	

	// VIO �����ع�״̬
	wire [7 : 0] wire_vio_cg_sel;
	wire [7 : 0] wire_vio_cg_enable;
	wire [7 : 0] wire_vio_cg_device;
	
	// DSP �ع�ʹ��
	wire [7 : 0] wire_cg_enable = (wire_vio_cg_sel == 0) ? reg_cg_enable : wire_vio_cg_enable;  

	assign  wire_cg_device =  (wire_vio_cg_sel == 0) ? reg_cg_device : wire_vio_cg_device;
	
	// �ع����������� 
	
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
			reg_cg_yaoce_state <= 8'h00;  // 0x01��ʾ�ع��У�0x10��ʾ�ع��ɹ���0x11��ʾ�ع�ʧ�ܣ�����0x00
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
				reg_cg_yaoce_state <= 8'h00;   						// 0x01��ʾ�ع��У�0x10��ʾ�ع��ɹ���0x11��ʾ�ع�ʧ�ܣ�����0x00
				reg_cg_yaoce_device <= 8'h00;  						// 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
				reg_cg_yaoce_error_code <= 8'h00;
				
			end 
			else begin 

				case (reg_cg_state)             
				CG_STATE_IDLE : begin   	// ����̬
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
							reg_cg_yaoce_state <= 8'h01;  				// 0x01��ʾ�ع��У�0x10��ʾ�ع��ɹ���0x11��ʾ�ع�ʧ�ܣ�����0x00
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
									
				CG_STATE_LEN1 : begin 	   // ��·��֡�� = 1048(����)
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
				
				CG_STATE_LEN2 : begin 	   // ֡������2 
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
					if (reg_cg_framelen != 1048) begin // �̶�֡�� =  1048
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
						reg_cg_state <= CG_STATE_ERR_DEVICETYPE;									//Դ��ַ����								
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
						reg_cg_state <= CG_STATE_ERR_DEVICETYPE;									//Ŀ�ĵ�ַ����								
					end	
					else begin
						if (wire_xw_cg_recv_flag) begin
							reg_cg_recv_wdt <= 0;
							reg_cg_total_frame_cnt[15:8] <= wire_xw_cg_uart_data;					//��֡����λ
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
						else if (reg_cg_recv_databyte == CG_VALID_DATA_LEN) begin						 	//���������ֶ�CRCУ��
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_datacrc[15: 8] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 
						else if (reg_cg_recv_databyte == (CG_VALID_DATA_LEN + 1)) begin						 //���������ֶ�CRCУ��
							reg_cg_recv_wdt <= 0;
							reg_cg_recv_datacrc[7 : 0] <= wire_xw_cg_uart_data;
							reg_cg_sum_check <= reg_cg_sum_check + wire_xw_cg_uart_data;
							reg_cg_recv_databyte <= reg_cg_recv_databyte + 1;
						end 
						else if (reg_cg_recv_databyte == (CG_VALID_DATA_LEN + 2)) begin						//��·���У��
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
				
				CG_STATE_CHECK1 : begin  // ��·��֡ ��У��  	 
					if (reg_cg_sum_check != reg_cg_recv_framesum) begin     
						reg_cg_state <= CG_STATE_CHECK2;
					end
					else begin
						reg_cg_state <= CG_STATE_CHECK2;
					end
				end
				
				CG_STATE_CHECK2 : begin  // ң��ָ�������ع��豸  �����Ƿ�Ϊ��֡
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
				
				// 0x72��DSP
				// 0x71�����
                // 0x73��ͨ�Ż���Ԥ����
                // 0X77��FPGA
				// ���ࣺ ����Ӧ
				CG_STATE_CHECK3 : begin //У��CRC 1022�ֽ��������ݲ���
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
						else begin 														//У�����,û������
							reg_cg_msram_write_cnt <= 0;
							reg_cg_state <= CG_STATE_CHECK_END;						
						end
					end
				end	
				
				CG_STATE_CHECK_END : begin 												// У���Ƿ�Ϊ���һ֡
					reg_xw_crc_restart <= 1'b0;
					if ((reg_cg_recv_framecnt + 16'd1) >= reg_cg_total_frame_cnt) begin 
        				reg_cg_recv_endframeflag <= 16'h00AA;    						// β֡��־
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
				
				CG_STATE_CHECK_END_ALLFILE : begin 										// �ع��ļ�����
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
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // ͨ�Ż� 1 ����
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // ͨ�Ż� 2 ����
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // ͨ�Ż� 3 ����  
						reg_cg_state <= CG_STATE_HANDLER;
					end 
					else if (wire_cg_device == CG_DEVICE_FPGA) begin  // �ع�FPGA, ��ʱӦ�÷�һ��ң������
						reg_cg_state <= CG_STATE_SUCCESS;
					end					
					else if (wire_cg_device == CG_DEVICE_XJ) begin    // ��� 
						reg_cg_state <= CG_STATE_SUCCESS;
					end 
					else begin 
						reg_cg_state <= CG_STATE_ERR_OTHER;
					end
				end 
				
				// �ع�һ֡���ճɹ�����	, DSP д��MSRAM, ������������ת��			
				CG_STATE_HANDLER : begin	
					if (wire_cg_device == CG_DEVICE_DSP) begin
						reg_cg_yaoce_device <= 8'h01;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // ͨ�Ż� 1 ����
						reg_cg_yaoce_device <= 8'h22;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // ͨ�Ż� 2 ����
						reg_cg_yaoce_device <= 8'h22;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // ͨ�Ż� 3 ����  
						reg_cg_yaoce_device <= 8'h22;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
						reg_cg_yaoce_device <= 8'h10;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
						reg_cg_yaoce_device <= 8'h11;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
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
						reg_cg_yaoce_device <= 8'h00;  // 0x01��ʾDSP��0x10��ʾFPGA��0x11��ʾ�����Ĭ��0x00
						reg_cg_state <= CG_STATE_ERR_SUBDEVICE;					
					end 
				end	
				
				// �ȴ�ת���������ж�ת��״̬
				CG_STATE_HANDLER_WAITING : begin   
					reg_xw_crc_restart <= 1'b0;	
					if (wire_cg_device == CG_DEVICE_DSP) begin        			// DSP ֱ���ع���MSRAM������ȴ�
						o_dsp_cg_en <= 1'b0;
						reg_cg_state <= CG_STATE_HANDLER_DONE;
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  			// ͨ�Ż� 1 ����
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
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // ͨ�Ż� 2 ����
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
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // ͨ�Ż� 3 ����  
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
						if (reg_cg_fpga_timeout_cnt < 32'd1_200_00) begin   // �ȴ� 1.2ms ����
							reg_cg_fpga_timeout_cnt <= reg_cg_fpga_timeout_cnt + 1;
						end 
						else begin 
							if (reg_to_fresh_send_mem == 1) begin 
								reg_to_fresh_sendmem_trigger <= 0;
								reg_cg_fpga_timeout_cnt <= 0;								
							end 
							else begin 
								reg_to_fresh_sendmem_trigger <= 0;			//��reg_to_fresh_send_mem_clr��1,��������ɺ���ת
								reg_cg_fpga_timeout_cnt <= 0;
								reg_cg_state <= CG_STATE_HANDLER_DONE;								
							end 
						end
					end					
					else if (wire_cg_device == CG_DEVICE_XJ) begin    		// ��� 
						if (reg_cg_fpga_timeout_cnt < 32'd1_200_00) begin   // �ȴ� 1ms ����
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
				
				// ���ؽ��
				CG_STATE_HANDLER_DONE : begin   
					if (wire_cg_device == CG_DEVICE_DSP) begin        // DSP ֱ���ع���MSRAM������ȴ�
						reg_cg_state <= CG_STATE_SUCCESS;
					end
					else if (wire_cg_device == CG_DEVICE_TXJ1) begin  // ͨ�Ż� 1 ����
						//0xAA��������ȷ
						//0x2�� ��֡У�����
						//0x3�� ֡��ַ������
						//0x4�� ֡��Ų�����
						//0x9�� ֡���ȴ���
						//0xF1�������ļ�У�����
						//0xFF����������
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
					else if (wire_cg_device == CG_DEVICE_TXJ2) begin  // ͨ�Ż� 2 ����
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
					else if (wire_cg_device == CG_DEVICE_TXJ3) begin  // ͨ�Ż� 3 ����  
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
							if (reg_cg_fpga_timeout_cnt < 32'd2500_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // �ȴ� 2.5S ����
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
							if (reg_cg_fpga_timeout_cnt < 32'd150_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // �ȴ� 150ms ����
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
					else if (wire_cg_device == CG_DEVICE_XJ) begin    // ��� 
						if (reg_cg_recv_framenum < 100) begin 
							if (reg_cg_fpga_timeout_cnt < 32'd2500_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // �ȴ� 2.5S ����
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
							if (reg_cg_fpga_timeout_cnt < 32'd150_000_00) begin // && (reg_to_fresh_flash_busy == 1)) begin   // �ȴ� 150ms ����
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
				
				CG_STATE_RESPOND : begin  	       // ���ظ�֡
					if (reg_cg_respond_busy == 1) begin
						reg_cg_state <= CG_STATE_RESPOND;
					end
					else begin
						mem_cg_respond[0]  <= 8'h7E;						// ֡ͷ	
						mem_cg_respond[1]  <= 8'h7E;
						mem_cg_respond[2]  <= 8'h00;						// ����0x001A
						mem_cg_respond[3]  <= 8'h1A; 							 					
						mem_cg_respond[4]  <= 8'd00; 	 					 
						mem_cg_respond[5]  <= MAIN_CG_DEVICE_ID; 	 		// Դ��ַ					
						mem_cg_respond[6]  <= 8'h00; 	 					// Ŀ�ĵ�ַ 
						mem_cg_respond[7]  <= reg_cg_source_id[7:0]; 			
						mem_cg_respond[8]  <= reg_cg_total_frame_cnt[15:8]; // ��֡��
						mem_cg_respond[9]  <= reg_cg_total_frame_cnt[7:0]; 
						mem_cg_respond[10] <= reg_cg_recv_framenum[15:8];	// ����Ӧ���֡��� 
						mem_cg_respond[11] <= reg_cg_recv_framenum[7:0]; 						
						mem_cg_respond[12] <= 8'h00; 						// Ԥ��6�ֽ�
						mem_cg_respond[13] <= 8'h00; 						// Ԥ��6�ֽ�
						mem_cg_respond[14] <= 8'h00; 						// Ԥ��6�ֽ�
						mem_cg_respond[15] <= 8'h00; 						// Ԥ��6�ֽ�
						mem_cg_respond[16] <= 8'h00; 						// Ԥ��6�ֽ�
						mem_cg_respond[17] <= wire_cg_device[7 : 0]; 		// ���ع����豸
						mem_cg_respond[18] <= 8'h00;						// �ع����
						case (reg_cg_respond_code)
							8'hAA: 
								mem_cg_respond[19] <= 8'h01;	
							default:
								mem_cg_respond[19] <= reg_cg_respond_code; 
						endcase
						mem_cg_respond[20] <= 0;   							// ����2BYTE
						mem_cg_respond[21] <= 0;   							// ����2BYTE
						mem_cg_respond[24] <= 8'h0A;   						// ������
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
								reg_cg_yaoce_state <= 8'h10;  // 0x01��ʾ�ع��У�0x10��ʾ�ع��ɹ���0x11��ʾ�ع�ʧ�ܣ�����0x00
							end 
							else begin 
								reg_cg_yaoce_state <= 8'h11; 
							end
						end
			
					end
				end	
				
				CG_STATE_RESPOND_CRC : begin       // CRC ����
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
				
				CG_STATE_RESPOND_WAITING : begin   // �ȴ�
					reg_cg_retry_timecnt <= 32'd0;
					reg_cg_state <= CG_STATE_RESPOND_DONE;
				end		
				
				CG_STATE_RESPOND_DONE : begin      // �ظ�֡��һ�������Ĵ�������
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
				
				// 0xF0����֡����У�����
				// 0xF1�����������ļ�У�����
				// 0xF3����ע��Ԫ��ʶ����
				// 0xF4������֡��Ŵ���
				// 0xF5������֡���ȴ���
				// 0xFF������ʧ��
				CG_STATE_ERR_FILE : begin          // ��֡����У�����
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
	

	/*== �ع���Ӧ���� ==*/
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
	

	/*== �ع���Ӧ ==*/
	parameter   CG_RESPOND_STATE_IDLE = 0, CG_RESPOND_STATE_SEND = 1, CG_RESPOND_STATE_WAITBUSY = 2, CG_RESPOND_STATE_WAITDONE = 3;
	reg [7 : 0]	reg_cg_respond_cnt;					
	reg [7 : 0] reg_cg_respond_state;
	
/*==�����ع���Ӧila*/
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