/*
 * tlc2543.c
 *
 *  Created on: 2024��3��29��
 *      Author: Suzkfly
 *
 *      TLC2543�����ݸ�λ�ȴ���ʱ������������������λ���½������������ʱ�ӳ�̬Ϊ��
 *      һ�δ�������λ��12/8/16
 */
#include <string.h>
#include "tlc2543.h"
#include "cpu_timer0.h"
#include "cmd.h"
#include "cur_local_conf_cmd.h"
#include "tele_dat.h"
#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
float Y_TEMP1_fu,Y_TEMP2_fu,Y_TEMP3_fu,Y_POT3_fu,Y_POT2_fu,Y_POT1_fu,TEST_28V,TEST_12V,
      Y_POT1_zhen,Y_POT2_zhen,Y_POT3_zhen,Y_TEMP1_zhen,Y_TEMP2_zhen,Y_TEMP3_zhen,
      GND1, GND2;
/* A���ѹֵ��B���ѹֵ��5V����ֵ */
float VOL_PYA_A, VOL_PYA_B, VOL_NYA_A, VOL_NYA_B;

/**
 * \brief ��λ�Ƶ�ѹת��Ϊ�Ƕ�
 */
#include "boot_args.h"
#if 1
float32 Vol_To_Angle (char axis, char type, uint8_t isAdjust)
{
	float32 angle = 0;
	float32 adjust_angle = 0;
	float32 user_adjust_angle = 0;
    switch (axis) {
        case 'A':
            switch (type) {
                case '+':
#if BOARD_NAME == BOARD_DGM_4
                    if ((Y_POT1_zhen > 1) && (Y_POT1_zhen < 4)) {
                    	angle = (5 - Y_POT1_zhen) * 60.9 - 134.288971875;
                    } else {
                    	angle = (5 - Y_POT2_zhen) * 60.4 + 46.7752125;
                    }
#elif BOARD_NAME == BOARD_DGM_2
                    if ((Y_POT1_zhen > 1) && (Y_POT1_zhen < 4)) {
                    	//angle = (5000 - Y_POT2_zhen * 1000) * 0.0595 +125.274;
                    	angle = (5 - Y_POT1_zhen) * 60.4 + 184.1425;
                    } else {
                    	//angle = (5000 - Y_POT1_zhen * 1000) * 0.05858 + 303.052827;
                    	angle = (5 - Y_POT2_zhen) * 60.6 + 7.0114;
                    }
#endif
                    if (angle > 360) {
                    	angle -= 360;
                    } else if (angle < 0) {
                    	angle += 360;
                    }
                    adjust_angle = angle - *gp_boot_arg_angle_senser_adj_pya_1;
                    user_adjust_angle = adjust_angle - g_angle_senser_pya;
                	break;
                case '-':
#if BOARD_NAME == BOARD_DGM_4
                    if ((Y_POT1_fu > 1) && (Y_POT1_fu < 4)) {
                    	angle = (5 - Y_POT1_fu) * 60.4 + 72.2702484;
                    } else {
                    	angle = (5 - Y_POT2_fu) * 60.4 - 107.4801516;
                    }
#elif BOARD_NAME == BOARD_DGM_2
                    if ((Y_POT1_fu > 1) && (Y_POT1_fu < 4)) {
                    	//angle = (5000 - Y_POT2_fu * 1000) * 0.0595 + 125.274;
                    	angle = (5 - Y_POT1_fu) * 60.4 + 229.5834;
                    } else {
                    	//angle = (5000 - Y_POT1_fu * 1000) * 0.0598968784 + 309.86545455096;   /* Suzkfly�� */
                    	angle = (5 - Y_POT2_fu) * 60.4 + 50;
                    }
#endif
                    if (angle > 360) {
                    	angle -= 360;
                    } else if (angle < 0) {
                    	angle += 360;
                    }
                    adjust_angle = angle - *gp_boot_arg_angle_senser_adj_nya_1;
                    user_adjust_angle = adjust_angle - g_angle_senser_nya;
                	break;
            }
            adjust_angle < 0.0 ? adjust_angle += 360.0: adjust_angle;
            adjust_angle > 360.0 ? adjust_angle -= 360.0: adjust_angle;
            user_adjust_angle < 0.0 ? user_adjust_angle += 360.0: user_adjust_angle;
            user_adjust_angle > 360.0 ? user_adjust_angle -= 360.0: user_adjust_angle;
            break;
        case 'B':
            switch (type) {
                case '+':
                	angle = (5 - Y_POT3_zhen) * 60.4 -101.6979;
                	adjust_angle = angle - *gp_boot_arg_angle_senser_adj_pyb_3;
                    user_adjust_angle = adjust_angle - g_angle_senser_pyb;
                	break;
                case '-':
                	angle = (5 - Y_POT3_fu) * 60.67 - 95.86649;
                	adjust_angle = angle - *gp_boot_arg_angle_senser_adj_nyb_3;
                    user_adjust_angle = adjust_angle - g_angle_senser_nyb;
                	break;
            }
            //adjust_angle < 180 ? adjust_angle += 360 : adjust_angle;
            //adjust_angle > 180 ? adjust_angle -= 360 : adjust_angle;
            break;
    }
    if (0 == isAdjust){
    	return angle ;
    }else if (1 == isAdjust){
    	return adjust_angle ;
    }else if (2 == isAdjust){
    	return user_adjust_angle ;
    }
	return angle ;
	//return isAdjust != 0 ? adjust_angle : angle;
}
#else
float32 Vol_To_Angle (char axis, char type, uint8_t isAdjust)
{
	float32 vol1, vol2;
	float32 angle = 0;
	float32 adjust_angle = 0;
    switch (axis) {
        case 'A':
            switch (type) {
                case '+':
                	vol1 = 5.0 - Y_POT1_zhen;
                	vol2 = 5.0 - Y_POT2_zhen;
                    if ((vol1 >= 1.0) && (vol1 < 4.0)) {
                        angle = (vol1 - 1.0) * (180.0 / (5.0 - 2.0));
                    } else {
                        angle = (vol2 - 1.0) * (180.0 / (5.0 - 2.0)) + 180;
                        if (angle >= 360.0) {
                            angle -= 360.0;
                        }
                    }
                    adjust_angle = angle - *gp_boot_arg_angle_senser_adj_pya_1;
                	break;
                case '-':
                	vol1 = 5.0 - Y_POT1_fu;
                	vol2 = 5.0 - Y_POT2_fu;
                    if ((vol1 >= 1.0) && (vol1 < 4.0)) {
                        angle = (vol1 - 1.0) * (180.0 / (5.0 - 2.0));
                    } else {
                        angle = (vol2 - 1.0) * (180.0 / (5.0 - 2.0)) + 180;
                        if (angle >= 360.0) {
                            angle -= 360.0;
                        }
                    }
                    adjust_angle = angle - *gp_boot_arg_angle_senser_adj_nya_1;
                	break;
            }
            adjust_angle < 0.0 ? adjust_angle += 360.0: adjust_angle;
            adjust_angle > 360.0 ? adjust_angle -= 360.0: adjust_angle;
            break;
        case 'B':
            switch (type) {
                case '+':
                	vol1 = 5.0 - Y_POT3_zhen;
                	angle = vol1 * (300.0 / 5.0);
                	adjust_angle = angle - *gp_boot_arg_angle_senser_adj_pyb_3;
                	break;
                case '-':
                	vol1 = 5.0 - Y_POT3_fu;
                	angle = vol1 * (300.0 / 5.0);
                	adjust_angle = angle - *gp_boot_arg_angle_senser_adj_nyb_3;
                	break;
            }
            adjust_angle < 0.0 ? adjust_angle += 300 : adjust_angle;
            adjust_angle > 300.0 ? adjust_angle -= 300 : adjust_angle;
            break;
    }
	return isAdjust != 0 ? adjust_angle : angle;
}
#endif

int16 g_adc_cmd = -1;
int16 adc_ch8_index = JHSR1840_CH_NUM - 1;
/**
 * \brief ADC���β���״̬
 *
 * \note    #0 ���β�������
 *          #1 ���յ�������ɱ�־����δ��ȡ��������
 */
volatile static Uint16 __g_adc_once_stat = 0;
/**
 * \brief ����SPI�жϵı�־
 */
volatile static Uint16 __g_SPI_isr_flag = 0;

/**
 * \brief ����EINT1�жϵı�־
 */
volatile static Uint16 __g_EINT1_isr_flag = 0;
/**
 * \brief ����ADC�Ĳ�����������Σ�
 */
Uint16 g_adc_value = 0;
Uint16 adc_resulte0[8][5] = { 0 };						//�����
Uint16 adc_resulte0_cnt[8] = { 0 };						//��������ݼ���
Uint16 adc_resulte0_avrg[8] = { 0 };			    	//���������5��ƽ��ֵ

Uint16 adc_resulte1[JHSR1840_CH_NUM][5] = { 0 };		//��λ��
Uint16 adc_resulte1_cnt[JHSR1840_CH_NUM] = { 0 };		//��λ�����ݼ���
Uint16 adc_resulte1_avrg[JHSR1840_CH_NUM] = { 0 };		//��λ������5��ƽ��ֵ

/**
 * \brief ��·ѡ�񿪹� GPIO��ʼ��
 */
static void __Multi_SW_IO_Init (void)
{
    EALLOW;

    GpioMuxRegs.GPBMUX.bit.TCLKINB_GPIOB12 = 0;     //��ͨIO
    GpioMuxRegs.GPBDIR.bit.GPIOB12 = 1;             //���
    GpioDataRegs.GPBDAT.bit.GPIOB12 = 0;             //��ʼ���0

    GpioMuxRegs.GPBMUX.bit.C4TRIP_GPIOB13 = 0;     //��ͨIO
    GpioMuxRegs.GPBDIR.bit.GPIOB13 = 1;             //���
    GpioDataRegs.GPBDAT.bit.GPIOB13 = 0;             //��ʼ���0

    GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14 = 0;     //��ͨIO
    GpioMuxRegs.GPBDIR.bit.GPIOB14 = 1;             //���
    GpioDataRegs.GPBDAT.bit.GPIOB14 = 0;             //��ʼ���0

    GpioMuxRegs.GPBMUX.bit.C6TRIP_GPIOB15 = 0;     //��ͨIO
    GpioMuxRegs.GPBDIR.bit.GPIOB15 = 1;             //���
    GpioDataRegs.GPBDAT.bit.GPIOB15 = 0;             //��ʼ���0

    EDIS;
}

/**
 * \brief ��·����ѡ��
 */
void Multi_SW_Slect (Uint16 ch)
{
	Uint16 value;

//    GpioDataRegs.GPBDAT.bit.GPIOB12 = ch & 1;
//    GpioDataRegs.GPBDAT.bit.GPIOB13 = (ch >> 1) & 1;
//    GpioDataRegs.GPBDAT.bit.GPIOB14 = (ch >> 2) & 1;
//    GpioDataRegs.GPBDAT.bit.GPIOB15 = (ch >> 3) & 1;

//	if (ch == 12) {
//	    GpioDataRegs.GPFDAT.bit.GPIOF6 = 1;
//	} else if (ch == 13) {
//	    GpioDataRegs.GPFDAT.bit.GPIOF6 = 0;
//	}

    value = GpioDataRegs.GPBDAT.all;
    value &= 0x0FFF;
    value |= ch << 12;
    GpioDataRegs.GPBDAT.all = value;

    DELAY_US(50);
}

#pragma CODE_SECTION(spiRxFifoIsr, "ramfuncs");
interrupt void spiRxFifoIsr(void)
{
    __g_SPI_isr_flag = 2;
    g_adc_value = SpiaRegs.SPIRXBUF;   //��rx�Ĵ��� ,ֻ�ж�RXFIFO ,RXFFST�Ż�����

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;                                  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;                                  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= M_INT6;                                   // Issue PIE ack
}

/**
 * \brief SPIA��ʼ��
 */
static void __SPIA_Init(void)
{
    EALLOW;

    GpioMuxRegs.GPFMUX.all |= 0x000F;       // F0~F3ȫ������ΪSPI����

    PieVectTable.SPIRXINTA = &spiRxFifoIsr;

    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;     	// Enable PIE Group 6, INT 1
    //PieCtrlRegs.PIEIER6.bit.INTx2=1;     	// Enable PIE Group 6, INT 2
    IER |= M_INT6;                      	// Enable CPU INT6

    //Initialize SPI FIFO registers
    SpiaRegs.SPICCR.bit.SPISWRESET = 0; 	//SPI�����λλ���ı�����֮ǰ�����λ

    SpiaRegs.SPICCR.all = 11;           	//�����ط����ݣ��½��ض����ݣ�ʱ�ӳ�̬�ͣ�12λ ,�ر�SPI���Ͳ���
    SpiaRegs.SPICTL.all = 0x001F;       	//ʹ������жϱ�־λ ,CLOCK�ź��ӳٰ����ڣ�����ģʽ ,����ʹ�� ,ʹ��SPI�ж�
    SpiaRegs.SPISTS.all = 0x0000;       	//״̬�Ĵ�������

    SpiaRegs.SPIBRR = 89;             		// ������ = LSPCLK / ( SPIBRR + 1) ,�����ʹ��ߵ������� todo
    SpiaRegs.SPIFFTX.all = 0xC008;      	// ʹ��FIFO���� ,FIFO���Ϊ8 , ��ֹTXFIFO�ж�
    SpiaRegs.SPIFFRX.all = 0x0021;      	// ʹ��RXFIFO�ж� ,���Ϊ1 ,ʹ��RXFIFO�ж�
    SpiaRegs.SPIFFCT.all = 0x00;        	// ��ʱΪ0
    SpiaRegs.SPIPRI.all  = 0x0010;       	//�����жϹ��� ,SPI��������

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;     		//����ʹ�ܽ���FIFO����
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  //�ı�����֮�����ô�λ

    EDIS;
}

/**
 * \brief ��ʼ��XINT2��GPIO
 */
static void __InitXINT1Gpio (void)
{
    EALLOW;

    GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;      //����XINT1
//  GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;    //����XINT2
//  GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;     //����XINT3

    EDIS;
}

#pragma CODE_SECTION(XINT1_isr, "ramfuncs");
interrupt void XINT1_isr (void)
{
    __g_EINT1_isr_flag = 2;
    //DELAY_US(10);
    //TLC2548_Convert(0xE000);                          //��FIFO

    // ���PIEӦ��Ĵ����ĵ�1λ������Ӧ��1�ڵ������ж�����
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

static void __EXTI1_Init (void)
{
    __InitXINT1Gpio();                                  //��ʼ��IO
    /* �����ⲿ�жϼĴ��� */
    EALLOW;
    XIntruptRegs.XINT1CR.bit.POLARITY = 0;              //0���½����ж�,1���������ж�
    /* �����ж�ʹ�� */
    XIntruptRegs.XINT1CR.bit.ENABLE   = 1;              //ʹ��XINT1�ж�
    /* PIE�ж�ʹ�� */
	//PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    PieVectTable.XINT1 = &XINT1_isr ;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;  //PIE��1 ,���1 XINT1 ,ʹ��PIE�ж�

    /* cpu INT1�ж� */
    IER |= M_INT1;
    EDIS;
}

/**
 * \brief TLC2548 ��ʼADת��
 *
 * \param[in] ch��Ҫת����ͨ����0~10
 *
 * \note ��SPITXBUFд�������
 */
void TLC2548_Start_Convert (Uint16 ch)
{
    SpiaRegs.SPITXBUF = ((ch << 4) | OUTPUT_DATA_LEN_12_BITS | OUTPUT_DATA_MSBF | UNIPOLAR) << 8;
}

/**
 * \brief TLC2543��ʼ��
 */
average_t __g_average[CH_CNT];
void TLC2543_Init (void)
{
	memset(__g_average, 0, sizeof(__g_average));

	memset(adc_resulte0, 0, sizeof(adc_resulte0));
	memset(adc_resulte0_cnt, 0, sizeof(adc_resulte0_cnt));
	memset(adc_resulte0_avrg, 0, sizeof(adc_resulte0_avrg));

	memset(adc_resulte1, 0, sizeof(adc_resulte1));
	memset(adc_resulte1_cnt, 0, sizeof(adc_resulte1_cnt));
	memset(adc_resulte1_avrg, 0, sizeof(adc_resulte1_avrg));

    __Multi_SW_IO_Init();
    __SPIA_Init();
    __EXTI1_Init();
}
void ADC_task (void)
{
    const Uint16 JHSR1840_channel[JHSR1840_CH_NUM] = {JHSR1840_NY_TEMPIN3, JHSR1840_NY_TEMPIN2, JHSR1840_NY_TEMPIN1,
    									  	  	  	  JHSR1840_PY_TEMPIN3, JHSR1840_PY_TEMPIN2, JHSR1840_PY_TEMPIN1,
													  JHSR1840_PYB_POTIN3, JHSR1840_PYA_POTIN2, JHSR1840_PYA_POTIN1,
													  JHSR1840_NYB_POTIN3, JHSR1840_NYA_POTIN2, JHSR1840_NYA_POTIN1,
													  JHSR1840_TEST12V, JHSR1840_TEST28V,JHSR1840_TEST28V}; /* ��·ѡ������ͨ�� */

	static int16 s_adc_cmd = -1;
	static int16 s_ch8_index = JHSR1840_CH_NUM - 1;
    static Uint64 ADC_tick = 0;
    static Uint64 ADC_tick_back = 0;

    static Uint64 check_tick = 0;
    static Uint64 check_tick_flag = 0;
    static Uint16 SPI_isr_flag_old = 0;
    static Uint16 EINT1_isr_flag_old = 0;

    if (check_tick_flag == 0) {
    	check_tick_flag = 1;
    	check_tick = GetSysTick();
    }

    if (ElapsedTick(check_tick) >= TICK_200MS) {
    	check_tick = GetSysTick();
    	__g_SPI_isr_flag = 0;
    	__g_EINT1_isr_flag = 0;
    }

    if (__g_SPI_isr_flag != SPI_isr_flag_old) {
    	check_tick = GetSysTick();
    	SPI_isr_flag_old = __g_SPI_isr_flag;
    }

    if (__g_EINT1_isr_flag != EINT1_isr_flag_old) {
    	check_tick = GetSysTick();
    	EINT1_isr_flag_old = __g_EINT1_isr_flag;
    }



//    static Uint16 period_pya_old = 0;
//    static Uint16 period_nya_old = 0;
//    static Uint16 period_pyb_old = 0;
//    static Uint16 period_nyb_old = 0;
//
//    static Uint64 ADC_tick_pya_A = 0;
//    static Uint64 ADC_tick_pya_B = 0;
//    static Uint64 ADC_tick_nya_A = 0;
//    static Uint64 ADC_tick_nya_B = 0;
//    static Uint64 ADC_tick_pyb_A = 0;
//    static Uint64 ADC_tick_pyb_B = 0;
//    static Uint64 ADC_tick_nyb_A = 0;
//    static Uint64 ADC_tick_nyb_B = 0;

    if (0 == __g_SPI_isr_flag && 0 == __g_EINT1_isr_flag)
    {
#if 0
			if (WORK_MODE_KEEP != g_Axis_Conf.p_pya->work_mode_set && WORK_MODE_STANDBY != g_Axis_Conf.p_pya->work_mode_set)
			{
				if (period_pya_old != g_Axis_Conf.p_pya->period_cnt_1_4)
				{
					s_adc_cmd = g_adc_cmd;
					period_pya_old = g_Axis_Conf.p_pya->period_cnt_1_4;
					if ((period_pya_old == 0) || (period_pya_old == 2))
					{
						g_adc_cmd = TLC2543_CMD_SEL_PYA_B;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
					else if ((period_pya_old == 1) || (period_pya_old == 3))
					{
						g_adc_cmd = TLC2543_CMD_SEL_PYA_A;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
				}
			}
			else
			{
				if (ElapsedTick(ADC_tick_pya_A) >= TICK_50MS)
				{
					ADC_tick_pya_A = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_PYA_A;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
				else if (ElapsedTick(ADC_tick_pya_B) >= TICK_50MS)
				{
					ADC_tick_pya_B = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_PYA_B;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
			}


			if (WORK_MODE_KEEP != g_Axis_Conf.p_nya->work_mode_set && WORK_MODE_STANDBY != g_Axis_Conf.p_nya->work_mode_set)
			{
				if (period_nya_old != g_Axis_Conf.p_nya->period_cnt_1_4)
				{
					s_adc_cmd = g_adc_cmd;
					period_nya_old = g_Axis_Conf.p_nya->period_cnt_1_4;
					if ((period_nya_old == 0) || (period_nya_old == 2))
					{
						g_adc_cmd = TLC2543_CMD_SEL_NYA_B;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
					else if ((period_nya_old == 1) || (period_nya_old == 3))
					{
						g_adc_cmd = TLC2543_CMD_SEL_NYA_A;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
				}
			}
			else
			{
				if (ElapsedTick(ADC_tick_nya_A) >= TICK_50MS)
				{
					ADC_tick_nya_A = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_NYA_A;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
				else if (ElapsedTick(ADC_tick_nya_B) >= TICK_50MS)
				{
					ADC_tick_nya_B = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_NYA_B;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
			}
#if BOARD_NAME == BOARD_DGM_4
			if (WORK_MODE_KEEP != g_Axis_Conf.p_pyb->work_mode_set && WORK_MODE_STANDBY != g_Axis_Conf.p_pyb->work_mode_set)
			{
				if (period_pyb_old != g_Axis_Conf.p_pyb->period_cnt_1_4)
				{
					s_adc_cmd = g_adc_cmd;
					period_pyb_old = g_Axis_Conf.p_pyb->period_cnt_1_4;
		            if ((period_pyb_old == 0) || (period_pyb_old == 2))
		            {
		            	g_adc_cmd = TLC2543_CMD_SEL_PYB_B;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
		            }
		            else if ((period_pyb_old == 1) || (period_pyb_old == 3))
		            {
		            	g_adc_cmd = TLC2543_CMD_SEL_PYB_A;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
		            }
				}
			}
			else
			{
				if (ElapsedTick(ADC_tick_pyb_A) >= TICK_50MS)
				{
					ADC_tick_pyb_A = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_PYB_A;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
				else if (ElapsedTick(ADC_tick_pyb_B) >= TICK_50MS)
				{
					ADC_tick_pyb_B = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_PYB_B;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
			}

			if (WORK_MODE_KEEP != g_Axis_Conf.p_nyb->work_mode_set && WORK_MODE_STANDBY != g_Axis_Conf.p_nyb->work_mode_set)
			{
				if (period_nyb_old != g_Axis_Conf.p_nyb->period_cnt_1_4)  	/* ���� */
				{
					s_adc_cmd = g_adc_cmd;
					period_nyb_old = g_Axis_Conf.p_nyb->period_cnt_1_4;
					if ((period_nyb_old == 0) || (period_nyb_old == 2))			/* ��B�� */
					{
						g_adc_cmd = TLC2543_CMD_SEL_NYB_B;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
					else if ((period_nyb_old == 1) || (period_nyb_old == 3)) 	/* ��A�� */
					{
						g_adc_cmd = TLC2543_CMD_SEL_NYB_A;
			            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
			        	__g_SPI_isr_flag = 1;
			        	__g_EINT1_isr_flag = 1;
			        	return;
					}
				}
			}
			else
			{
				if (ElapsedTick(ADC_tick_nyb_A) >= TICK_50MS)
				{
					ADC_tick_nyb_A = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_NYB_A;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
				else if (ElapsedTick(ADC_tick_nyb_B) >= TICK_50MS)
				{
					ADC_tick_nyb_B = GetSysTick();
					s_adc_cmd = g_adc_cmd;
					g_adc_cmd = TLC2543_CMD_SEL_NYB_B;
		            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
		        	__g_SPI_isr_flag = 1;
		        	__g_EINT1_isr_flag = 1;
		        	return;
				}
			}
#endif
#endif
	    	if (ElapsedTick(ADC_tick) >= TICK_50MS)
	    	{
	            ADC_tick_back = GetSysTick();
				s_ch8_index = adc_ch8_index;
	            adc_ch8_index = (++adc_ch8_index) % JHSR1840_CH_NUM;
	            if (adc_ch8_index == JHSR1840_CH_NUM - 1)
	            {
	            	ADC_tick = ADC_tick_back;
	            }
	            Multi_SW_Slect(JHSR1840_channel[adc_ch8_index]);
				s_adc_cmd = g_adc_cmd;
	            g_adc_cmd = TLC2543_CMD_SEL_SELCT;
	        }
	    	else
	    	{
	    		return;
	    	}
            TLC2548_Start_Convert(g_adc_cmd);//��·ѡ�񿪹ظ���ͨ������λ�����ݲɼ����200ms��
        	__g_SPI_isr_flag = 1;
        	__g_EINT1_isr_flag = 1;
    } else if (2 == __g_SPI_isr_flag && 2 == __g_EINT1_isr_flag){
    	__g_SPI_isr_flag = 0;
    	__g_EINT1_isr_flag = 0;
        Uint16 i;
        if (s_adc_cmd == TLC2543_CMD_SEL_SELCT)
		{
        	adc_resulte1[s_ch8_index][adc_resulte1_cnt[s_ch8_index]] = g_adc_value;
        	adc_resulte1_cnt[s_ch8_index]++;

            if (adc_resulte1_cnt[s_ch8_index] == 1)
            {
            	adc_resulte1_cnt[s_ch8_index] = 0;
            	adc_resulte1_avrg[s_ch8_index] = 0;
                for (i = 0; i < 1; i++)
                {
                	adc_resulte1_avrg[s_ch8_index] += adc_resulte1[s_ch8_index][i];
                }
                adc_resulte1_avrg[s_ch8_index] /= 1.0;
                float value = adc_resulte1_avrg[s_ch8_index] * 5 / 4095.0;
                switch(s_ch8_index)
                {
					case 0:
						__Filter_Average(0, value, &Y_TEMP3_fu);
						break;
					case 1:
						__Filter_Average(1, value, &Y_TEMP2_fu);
						break;
					case 2:
						__Filter_Average(2, value, &Y_TEMP1_fu);
						break;
					case 3:
						__Filter_Average(3, value, &Y_TEMP3_zhen);
						break;
					case 4:
						__Filter_Average(4, value, &Y_TEMP2_zhen);
						break;
					case 5:
						__Filter_Average(5, value, &Y_TEMP1_zhen);
						break;
					case 6:
						__Filter_Average(6, value, &Y_POT3_zhen);
						break;
					case 7:
						__Filter_Average(7, value, &Y_POT2_zhen);
#if BOARD_NAME == BOARD_DGM_4
					    g_Tele_Dat.rev3[1] = g_adc_value & 0x00FF;
					    g_Tele_Dat.edac_single_err_cnt = (g_adc_value >> 8) + (g_Tele_Dat.edac_single_err_cnt & 0xF0);/* B104 EDAC������� */
#endif
						break;
					case 8:
						__Filter_Average(8, value, &Y_POT1_zhen);
#if BOARD_NAME == BOARD_DGM_4
					    g_Tele_Dat.rev3[0] = g_adc_value & 0x00FF;
					    g_Tele_Dat.edac_single_err_cnt = ((g_adc_value >> 8) << 4) + (g_Tele_Dat.edac_single_err_cnt & 0x0F);/* B104 EDAC������� */
#endif
						break;
					case 9:
						__Filter_Average(9, value, &Y_POT3_fu);
						break;
					case 10:
						__Filter_Average(10, value, &Y_POT2_fu);
#if BOARD_NAME == BOARD_DGM_4
					    g_Tele_Dat.rev3[3] = g_adc_value & 0x00FF;
					    g_Tele_Dat.edac_double_err_cnt = (g_adc_value >> 8) + (g_Tele_Dat.edac_double_err_cnt & 0xF0);/* B105 EDAC˫�����*/
#endif
						break;
					case 11:
						__Filter_Average(11, value, &Y_POT1_fu);
#if BOARD_NAME == BOARD_DGM_4
					    g_Tele_Dat.rev3[2] = g_adc_value & 0x00FF;
					    g_Tele_Dat.edac_double_err_cnt = ((g_adc_value >> 8) << 4) + (g_Tele_Dat.edac_double_err_cnt & 0x0F); /* B105 EDAC˫����� */
#endif
						break;
					case 12:
						__Filter_Average(12, value, &TEST_12V);
						break;
					case 13:
						__Filter_Average(13, value, &TEST_28V);
						break;
					default:break;
                }
            }
		}
        else if ((s_adc_cmd >= TLC2543_CMD_SEL_PYA_A) && (s_adc_cmd < TLC2543_CMD_SEL_SELCT))
        {
        	adc_resulte0[s_adc_cmd][adc_resulte0_cnt[s_adc_cmd]] = g_adc_value;
        	adc_resulte0_cnt[s_adc_cmd]++;

            if (adc_resulte0_cnt[s_adc_cmd] == 1)
            {
            	adc_resulte0_cnt[s_adc_cmd] = 0;
            	adc_resulte0_avrg[s_adc_cmd] = 0;
                for (i = 0; i < 1; i++)
                {
                	adc_resulte0_avrg[s_adc_cmd] += adc_resulte0[s_adc_cmd][i];
                }
                adc_resulte0_avrg[s_adc_cmd] /= 1.0;
                float32 value = (float32)adc_resulte0_avrg[s_adc_cmd] * REF_VOL / 4095;
                switch(s_adc_cmd){
					case TLC2543_CMD_SEL_PYA_A:
						VOL_PYA_A = value;
						break;
					case TLC2543_CMD_SEL_PYA_B:
						VOL_PYA_B = value;
						break;
					case TLC2543_CMD_SEL_PYB_A:;break;
					case TLC2543_CMD_SEL_PYB_B:;break;
					case TLC2543_CMD_SEL_NYA_A:
						VOL_NYA_A = value;
						break;
					case TLC2543_CMD_SEL_NYA_B:
						VOL_NYA_B = value;
						break;
					case TLC2543_CMD_SEL_NYB_A:;break;
					case TLC2543_CMD_SEL_NYB_B:;break;
                }
            }
        }
        else
        {
//        	GpioDataRegs.GPFTOGGLE.bit.GPIOF6 = 1;             //��ʼ���0
        }
    }


	return;
}

#if 1
/**
 * \brief ��������
 *
 * \param[in]  ch��Ҫ�����ͨ��
 * \param[in]  value��������Ҫ�жϵ���
 * \param[out] p_data���жϺ����Ҫ�󿽱����˿ռ�
 *
 * \retval #0���ɹ�
 * 		   #-1����δ�ﵽ������
 *
 */

int16 __Filter_Average (Uint16 ch, float value, float* p_data)
{
	if (ch >= CH_CNT) {
		return -1;
	}
	__g_average[ch].value = value;	//��¼һ�£�������Բ鿴
	if (0 == __g_average[ch].sample_ok && 0 == __g_average[ch].index){			/* ��һ����ֱ����� */
		__g_average[ch].sample[__g_average[ch].index] = value;
		__g_average[ch].index++;
		__g_average[ch].sum = value;
		__g_average[ch].average = value;
		*p_data = value;
	} else if (0 == __g_average[ch].sample_ok && __g_average[ch].index != 0){	/* ����������ʵ�������������ֵ */
		float sum_temp = __g_average[ch].sum;
		sum_temp += value;
		__g_average[ch].difference = sum_temp / (__g_average[ch].index + 1) - __g_average[ch].average;
		if (__g_average[ch].difference >= -THRESHOLD && __g_average[ch].difference <= THRESHOLD)
		{
			__g_average[ch].sum = sum_temp;
			__g_average[ch].average = sum_temp / (__g_average[ch].index + 1);
			__g_average[ch].sample[__g_average[ch].index] = value;
			__g_average[ch].index = ++__g_average[ch].index;
			if (SAMPLE_CNT == __g_average[ch].index)
			{
				__g_average[ch].index = 0;
				__g_average[ch].sample_ok = 1;
			}
			*p_data = __g_average[ch].average;
		}
		else
		{
			__Abnormal_Cnt(ch, value);
		}
	} else {																	/* ���������������޳��f�� ���ֵ */
		float sum_temp = __g_average[ch].sum;
		sum_temp -= __g_average[ch].sample[__g_average[ch].index];
		sum_temp += value;

		__g_average[ch].difference = sum_temp / SAMPLE_CNT - __g_average[ch].average;
		if (__g_average[ch].difference >= -THRESHOLD && __g_average[ch].difference <= THRESHOLD)
		{
			__g_average[ch].sum = sum_temp;
			__g_average[ch].average = sum_temp / SAMPLE_CNT;
			__g_average[ch].sample[__g_average[ch].index] = value;
			__g_average[ch].index = ++__g_average[ch].index % SAMPLE_CNT;
			*p_data = __g_average[ch].average;
		}
		else
		{
			__Abnormal_Cnt(ch, value);
		}
	}
	return 0;
}

void __Abnormal_Cnt (Uint16 ch, float value)
{
	//static float black_value = 0;
	//float temp = value - black_value;
	//if (temp <= 0.1 && temp >= -0.1)
	//{
		__g_average[ch].abnormal++;
	    if (__g_average[ch].abnormal >= ABNORMAL_CNT){
			__g_average[ch].index = 0;
			__g_average[ch].sum = 0;
			__g_average[ch].average = 0;
			__g_average[ch].sample_ok = 0;
			__g_average[ch].abnormal = 0;
			//black_value = 0;
	    }
	//}
	//else
	//{
	//	black_value = value;
	//	__g_average[ch].abnormal = 0;
	//}
}
#endif
#endif














