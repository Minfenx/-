/*
 * tlv2548.c
 *
 *  Created on: 2024年1月17日
 *      Author: Suzkfly
 *
 *      TLV2548高位先传，高4位用于配置16个命令中的其中1个，低12位为数据时钟上升沿锁定输入数据
 *      时钟频率最快20MHz
 */
#include "tlv2548.h"
#include <string.h>
#include "cpu_timer0.h"
#include "cmd.h"

#if BOARD_NAME == BOARD_GMS

/**
 * \brief ADC单次采样状态
 *
 * \note    #0 单次采样结束
 *          #1 接收到采样完成标志，尚未读取数据内容
 */
volatile static Uint16 __g_adc_once_stat = 0;
#pragma DATA_SECTION(__g_adc_once_stat, "pre_roll_data");

/**
 * \brief 进入SPI中断的标志
 */
volatile static Uint16 __g_SPI_isr_flag = 0;
#pragma DATA_SECTION(__g_SPI_isr_flag, "pre_roll_data");

/**
 * \brief 进入EINT1中断的标志
 */
volatile static Uint16 __g_EINT1_isr_flag = 0;
#pragma DATA_SECTION(__g_EINT1_isr_flag, "pre_roll_data");

/**
 * \brief 保存ADC的采样结果（单次）
 */
Uint16 g_adc_value = 0;
#pragma DATA_SECTION(g_adc_value, "pre_roll_data");

/* A相电压值，B相电压值，5V电流值 */
float VOL_PYB_A = 0, VOL_PYB_B = 0, VOL_CUR_5V = 0;
#pragma DATA_SECTION(VOL_PYB_A, "pre_roll_data");
#pragma DATA_SECTION(VOL_PYB_B, "pre_roll_data");
#pragma DATA_SECTION(VOL_CUR_5V, "pre_roll_data");

/**
 * \brief SPI发送半字
 */
void spi_send_halfword (Uint16 halfword)
{
    SpiaRegs.SPITXBUF = halfword;      // Send data
}

static void adc_delay_loop()
{
    Uint32      i;
    Uint32      j;
    for(i=0;i<32;i++)
    for (j = 0; j < 100; j++) {}
}

#pragma CODE_SECTION(spiRxFifoIsr, "ramfuncs");
interrupt void spiRxFifoIsr(void)
{
    Uint16 temp = 0;

    __g_SPI_isr_flag = 1;

    temp = SpiaRegs.SPIRXBUF;   //读rx寄存器 ,只有读RXFIFO ,RXFFST才会清零
    if (__g_adc_once_stat == 2) {
        g_adc_value = temp >> 4;
        __g_adc_once_stat = 0;
    }

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;                                  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;                                  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= M_INT6;                                   // Issue PIE ack
}


/**
 * \brief SPIA初始化
 */
static void __SPIA_Init(void)
{
    EALLOW;

    GpioMuxRegs.GPFMUX.all |= 0x000F;       // F0~F3全部设置为SPI复用

    PieVectTable.SPIRXINTA = &spiRxFifoIsr;

    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;     // Enable PIE Group 6, INT 1
    //     PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
    IER |= M_INT6;                      // Enable CPU INT6

    // Initialize SPI FIFO registers
    SpiaRegs.SPICCR.bit.SPISWRESET = 0; //SPI软件复位位，改变配置之前清除此位

    SpiaRegs.SPICCR.all = 0x000F;       //上升沿发数据，下降沿读数据，16位 ,关闭SPI回送测试
    SpiaRegs.SPICTL.all = 0x001F;       //使能溢出中断标志位 ,相位为0 ,下降沿锁存 , 主机模式 ,发送使能 ,使能SPI中断
    SpiaRegs.SPISTS.all = 0x0000;       //状态寄存器清零

    /* 89对应250K */
    /* 22对应约1M */
    /* 9对应约2.25M */
    SpiaRegs.SPIBRR = 22;             // 波特率 = LSPCLK / ( SPIBRR + 1) ,波特率过高导致误码
    SpiaRegs.SPIFFTX.all = 0xC008;      // 使能FIFO功能 ,FIFO深度为8 , 禁止TXFIFO中断
    SpiaRegs.SPIFFRX.all = 0x0021;      // 使能RXFIFO中断 ,深度为1 ,使能RXFIFO中断
    SpiaRegs.SPIFFCT.all = 0x00;        // 延时为0
    SpiaRegs.SPIPRI.all  = 0x0010;       //忽视中断挂起 ,SPI继续工作

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;     //重新使能接收FIFO操作
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  //改变配置之后，设置此位

    EDIS;
}


/**
 * \brief 初始化XINT2的GPIO
 */
static void __InitXINT1Gpio (void)
{
	EALLOW;

	GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//设置XINT1
//	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//设置XINT2
//	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//设置XINT3

    EDIS;
}

#pragma CODE_SECTION(XINT1_isr, "ramfuncs");
interrupt void XINT1_isr (void)
{
    __g_EINT1_isr_flag = 1;
    // 清除PIE应答寄存器的第1位，以响应组1内的其他中断请求；
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

static void __EXTI1_Init (void)
{
	__InitXINT1Gpio();									//初始化IO
	/* 配置外部中断寄存器 */
	EALLOW;
	XIntruptRegs.XINT1CR.bit.POLARITY = 0;				//0：下降沿中断,1：上升沿中断
	/* 外设中断使能 */
	XIntruptRegs.XINT1CR.bit.ENABLE   = 1;				//使能XINT1中断
	/* PIE中断使能 */

	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	PieVectTable.XINT1 = &XINT1_isr ;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;	//PIE组1 ,编号1 XINT1 ,使能PIE中断

	/* cpu INT1中断 */
	IER |= M_INT1;
    EDIS;
}

#if 0
/**
 * \brief 求滑动平均
 *
 * \param[in]  ch：要计算的通道
 * \param[in]  value：输入要进行求平均的数
 * \param[out] p_average：计算后的结果
 *
 * \retval #0：成功
 * 		   #-1：还未达到样本数
 *
 */
static average_t __g_average[CH_CNT];
#define SAMPLE_CNT 10
static int16 __Filter_Average (Uint16 ch, Uint16 value, Uint16* p_average)
{
	int i = 0;

	if (ch >= CH_CNT) {
		return -1;
	}

	if (__g_average[ch].sample_ok == 0) {	/* 样本数不够的情况下填充样本 */
		__g_average[ch].sample[__g_average[ch].index] = value;
		__g_average[ch].index++;
		if (__g_average[ch].index == SAMPLE_CNT) {
			__g_average[ch].index = 0;
			__g_average[ch].sample_ok = 1;
			__g_average[ch].sum = 0;
			for (i = 0; i < SAMPLE_CNT; i++) {
				__g_average[ch].sum += __g_average[ch].sample[i];
			}
			*p_average = __g_average[ch].sum / SAMPLE_CNT;
			return 0;
		}
	} else {
		__g_average[ch].sum -= __g_average[ch].sample[__g_average[ch].index];
		__g_average[ch].sample[__g_average[ch].index] = value;
		__g_average[ch].sum += value;

		__g_average[ch].index++;
		if (__g_average[ch].index == SAMPLE_CNT) {
			__g_average[ch].index = 0;
		}

		*p_average = __g_average[ch].sum / SAMPLE_CNT;

		return 0;
	}

	return -1;
}
#endif



/**
 * \brief TLV2548初始化
 */
Uint16 ad_val[5] = {0, 0, 0, 0, 0};
#pragma DATA_SECTION(ad_val, "pre_roll_data");

void TLV2548_Init (void)
{
	//memset(__g_average, 0, sizeof(__g_average));

	__SPIA_Init();
	__EXTI1_Init();

    spi_send_halfword(0xA000);                          //tlv2548初始化确定型号
    adc_delay_loop();
    spi_send_halfword(ADC_MODE);                        //确定采集模式
    adc_delay_loop();
//    spi_send_halfword((Uint16)TLV2548_CMD_READ_CFR);
//    adc_delay_loop();
}

/**
 * \brief ADC采集任务。SPI频率为250K时，采集两个通道需要410us。SPI频率为1M时，采集两个通道需要128us
 *
 * \note 电机不动时，每隔200ms采集一次，5次后求平均值
 *       电机运动时，在电机电流达到最大值时采集N次，采集策略如下：
 *       速度为0.6°/s时，单相每隔24ms有一次最大值，持续时间375us
 *       速度为0.01°/s时，单相每隔1440ms有一次最大值，持续时间22.5ms
 *       当速度为0.08°/s时，单相每隔180ms有一次最大值
 */

static Uint16 adc_restart_flag = 0;
static Uint16 adc_index = 0;
static Uint16 adc_sw = 1;
static Uint64 ADC_Tick = 0;
static Uint16 static_test_times = 0;
static Uint16 adc_resulte[5][3] = { 0 };
static Uint16 period_old = 0;
static Uint16 cur_5v_en = 0;    /* 使能5V电流采集 */
static Uint16 adc_times[3];
static Uint16 mode_lock = 1;
#pragma DATA_SECTION(adc_restart_flag, "pre_roll_data");
#pragma DATA_SECTION(adc_index, "pre_roll_data");
#pragma DATA_SECTION(adc_sw, "pre_roll_data");
#pragma DATA_SECTION(ADC_Tick, "pre_roll_data");
#pragma DATA_SECTION(static_test_times, "pre_roll_data");
#pragma DATA_SECTION(adc_resulte, "pre_roll_data");
#pragma DATA_SECTION(period_old, "pre_roll_data");
#pragma DATA_SECTION(cur_5v_en, "pre_roll_data");
#pragma DATA_SECTION(adc_times, "pre_roll_data");
#pragma DATA_SECTION(mode_lock, "pre_roll_data");

void ADC_task (void)
{
    Uint16 adc_resulte_avrg[3];
    const Uint16 channel_cmd[3] = {TLV2548_CMD_SEL_CH0, TLV2548_CMD_SEL_CH1, TLV2548_CMD_SEL_CH3}; /* 要采集的通道 */
    Uint16 i, j;

    if ((g_Axis_Conf.p_pyb->cur_angle_speed == 0) || (mode_lock == 1)) {	/* Suzkfly 2024-11-11 workmode_set为KEEP_MODE改为角速度为0，否则收到保持模式之后到停下来之前采集到的电机电流不对 */
        if (ElapsedTick(ADC_Tick) >= TICK_200MS) {
            ADC_Tick = GetSysTick();
            adc_sw = 1;
        }

        adc_times[0] = 0;
        adc_times[1] = 0;
        adc_times[2] = 0;

        if (adc_sw == 1) {
            mode_lock = 1;

            if (adc_restart_flag == 0) {
                adc_restart_flag = 1;
                spi_send_halfword((Uint16)channel_cmd[adc_index]);
            }

            if ((__g_adc_once_stat == 0) && (__g_SPI_isr_flag == 1) && (__g_EINT1_isr_flag == 1)) {
                __g_adc_once_stat = 1;
                __g_SPI_isr_flag = 0;
                __g_EINT1_isr_flag = 0;

                if (adc_index == 0) {
                    if (adc_restart_flag == 1) {
                        adc_restart_flag = 2;
                    } else {
                        adc_resulte[static_test_times][2] = g_adc_value;
                        adc_sw = 0;
                        adc_restart_flag = 0;
                        __g_adc_once_stat = 0;
                        adc_index = 0;
                        mode_lock = 0;
                        static_test_times++;

                        /*  求5次平均 */
                        if (static_test_times == 5) {
                            static_test_times = 0;
                            for (j = 0; j < 3; j++) {
                                adc_resulte_avrg[j] = 0;
                                for (i = 0; i < 5; i++) {
                                    adc_resulte_avrg[j] += adc_resulte[i][j];
                                }
                                adc_resulte_avrg[j] /= 5;
                            }
                            VOL_PYB_A  = (float32)adc_resulte_avrg[0] * REF_VOL / 4095;
                            VOL_PYB_B  = (float32)adc_resulte_avrg[1] * REF_VOL / 4095;
                            VOL_CUR_5V = (float32)adc_resulte_avrg[2] * REF_VOL / 4095;
                        }

                        return;
                    }
                } else {
                    adc_resulte[static_test_times][adc_index - 1] = g_adc_value;
                }
                adc_index++;
                if (adc_index == 3) {
                    adc_index = 0;
                }

                spi_send_halfword((Uint16)TLV2548_CMD_READ_FIFO);   //读FIFO
            } else if ((__g_adc_once_stat == 1) && (__g_SPI_isr_flag == 1)) {
                __g_adc_once_stat = 2;
                __g_SPI_isr_flag = 0;
                spi_send_halfword((Uint16)channel_cmd[adc_index]);
            }
        }
    } else {
        if (period_old != g_Axis_Conf.p_pyb->period_cnt_1_4) {  /* 换相 */
            period_old = g_Axis_Conf.p_pyb->period_cnt_1_4;

            if ((period_old == 0) || (period_old == 2)) {       /* 采B项 */
                adc_index = 1;
            } else if ((period_old == 1) || (period_old == 3)){ /* 采A项 */
                adc_index = 0;
            }

            if (ElapsedTick(ADC_Tick) >= TICK_200MS) {  /* 采5V电流 */
                ADC_Tick = GetSysTick();
                cur_5v_en = 1;
            }

            adc_sw = 1;
        }


        if (adc_sw == 1) {
            if (adc_restart_flag == 0) {
                adc_restart_flag = 1;
                spi_send_halfword((Uint16)channel_cmd[adc_index]);
            }

            if ((__g_adc_once_stat == 0) && (__g_SPI_isr_flag == 1) && (__g_EINT1_isr_flag == 1)) {
                __g_adc_once_stat = 1;
                __g_SPI_isr_flag = 0;
                __g_EINT1_isr_flag = 0;

                if (adc_restart_flag == 1) {
                    adc_restart_flag = 2;
                } else {

                    adc_resulte[adc_times[adc_index]][adc_index] = g_adc_value;
                    adc_times[adc_index]++;

                    if (adc_times[adc_index] == 5) {
                        adc_times[adc_index] = 0;
                        adc_resulte_avrg[adc_index] = 0;
                        for (i = 0; i < 5; i++) {
                            adc_resulte_avrg[adc_index] += adc_resulte[i][adc_index];
                        }
                        adc_resulte_avrg[adc_index] /= 5;

                        switch (adc_index) {
                            case 0: VOL_PYB_A   = (float32)adc_resulte_avrg[adc_index] * REF_VOL / 4095; break;
                            case 1: VOL_PYB_B   = (float32)adc_resulte_avrg[adc_index] * REF_VOL / 4095; break;
                            case 2: VOL_CUR_5V  = (float32)adc_resulte_avrg[adc_index] * REF_VOL / 4095; break;
                        }
                    }

                    if (cur_5v_en == 0) {
                        adc_sw = 0;
                        adc_restart_flag = 0;
                        __g_adc_once_stat = 0;

                        return;
                    } else {
                        cur_5v_en = 0;
                        adc_index = 2;
                    }
                }

                spi_send_halfword((Uint16)TLV2548_CMD_READ_FIFO);   //读FIFO
            } else if ((__g_adc_once_stat == 1) && (__g_SPI_isr_flag == 1)) {
                __g_adc_once_stat = 2;
                __g_SPI_isr_flag = 0;
                spi_send_halfword((Uint16)channel_cmd[2]);
            }
        }
    }
}

#endif










