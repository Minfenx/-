/*
 * tlv2548.c
 *
 *  Created on: 2024��1��17��
 *      Author: Suzkfly
 *
 *      TLV2548��λ�ȴ�����4λ��������16�������е�����1������12λΪ����ʱ��������������������
 *      ʱ��Ƶ�����20MHz
 */
#include "tlv2548.h"
#include <string.h>
#include "cpu_timer0.h"
#include "cmd.h"

#if BOARD_NAME == BOARD_GMS

/**
 * \brief ADC���β���״̬
 *
 * \note    #0 ���β�������
 *          #1 ���յ�������ɱ�־����δ��ȡ��������
 */
volatile static Uint16 __g_adc_once_stat = 0;
#pragma DATA_SECTION(__g_adc_once_stat, "pre_roll_data");

/**
 * \brief ����SPI�жϵı�־
 */
volatile static Uint16 __g_SPI_isr_flag = 0;
#pragma DATA_SECTION(__g_SPI_isr_flag, "pre_roll_data");

/**
 * \brief ����EINT1�жϵı�־
 */
volatile static Uint16 __g_EINT1_isr_flag = 0;
#pragma DATA_SECTION(__g_EINT1_isr_flag, "pre_roll_data");

/**
 * \brief ����ADC�Ĳ�����������Σ�
 */
Uint16 g_adc_value = 0;
#pragma DATA_SECTION(g_adc_value, "pre_roll_data");

/* A���ѹֵ��B���ѹֵ��5V����ֵ */
float VOL_PYB_A = 0, VOL_PYB_B = 0, VOL_CUR_5V = 0;
#pragma DATA_SECTION(VOL_PYB_A, "pre_roll_data");
#pragma DATA_SECTION(VOL_PYB_B, "pre_roll_data");
#pragma DATA_SECTION(VOL_CUR_5V, "pre_roll_data");

/**
 * \brief SPI���Ͱ���
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

    temp = SpiaRegs.SPIRXBUF;   //��rx�Ĵ��� ,ֻ�ж�RXFIFO ,RXFFST�Ż�����
    if (__g_adc_once_stat == 2) {
        g_adc_value = temp >> 4;
        __g_adc_once_stat = 0;
    }

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

    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;     // Enable PIE Group 6, INT 1
    //     PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
    IER |= M_INT6;                      // Enable CPU INT6

    // Initialize SPI FIFO registers
    SpiaRegs.SPICCR.bit.SPISWRESET = 0; //SPI�����λλ���ı�����֮ǰ�����λ

    SpiaRegs.SPICCR.all = 0x000F;       //�����ط����ݣ��½��ض����ݣ�16λ ,�ر�SPI���Ͳ���
    SpiaRegs.SPICTL.all = 0x001F;       //ʹ������жϱ�־λ ,��λΪ0 ,�½������� , ����ģʽ ,����ʹ�� ,ʹ��SPI�ж�
    SpiaRegs.SPISTS.all = 0x0000;       //״̬�Ĵ�������

    /* 89��Ӧ250K */
    /* 22��ӦԼ1M */
    /* 9��ӦԼ2.25M */
    SpiaRegs.SPIBRR = 22;             // ������ = LSPCLK / ( SPIBRR + 1) ,�����ʹ��ߵ�������
    SpiaRegs.SPIFFTX.all = 0xC008;      // ʹ��FIFO���� ,FIFO���Ϊ8 , ��ֹTXFIFO�ж�
    SpiaRegs.SPIFFRX.all = 0x0021;      // ʹ��RXFIFO�ж� ,���Ϊ1 ,ʹ��RXFIFO�ж�
    SpiaRegs.SPIFFCT.all = 0x00;        // ��ʱΪ0
    SpiaRegs.SPIPRI.all  = 0x0010;       //�����жϹ��� ,SPI��������

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;     //����ʹ�ܽ���FIFO����
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

	GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//����XINT1
//	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//����XINT2
//	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//����XINT3

    EDIS;
}

#pragma CODE_SECTION(XINT1_isr, "ramfuncs");
interrupt void XINT1_isr (void)
{
    __g_EINT1_isr_flag = 1;
    // ���PIEӦ��Ĵ����ĵ�1λ������Ӧ��1�ڵ������ж�����
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

static void __EXTI1_Init (void)
{
	__InitXINT1Gpio();									//��ʼ��IO
	/* �����ⲿ�жϼĴ��� */
	EALLOW;
	XIntruptRegs.XINT1CR.bit.POLARITY = 0;				//0���½����ж�,1���������ж�
	/* �����ж�ʹ�� */
	XIntruptRegs.XINT1CR.bit.ENABLE   = 1;				//ʹ��XINT1�ж�
	/* PIE�ж�ʹ�� */

	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	PieVectTable.XINT1 = &XINT1_isr ;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;	//PIE��1 ,���1 XINT1 ,ʹ��PIE�ж�

	/* cpu INT1�ж� */
	IER |= M_INT1;
    EDIS;
}

#if 0
/**
 * \brief �󻬶�ƽ��
 *
 * \param[in]  ch��Ҫ�����ͨ��
 * \param[in]  value������Ҫ������ƽ������
 * \param[out] p_average�������Ľ��
 *
 * \retval #0���ɹ�
 * 		   #-1����δ�ﵽ������
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

	if (__g_average[ch].sample_ok == 0) {	/* ������������������������ */
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
 * \brief TLV2548��ʼ��
 */
Uint16 ad_val[5] = {0, 0, 0, 0, 0};
#pragma DATA_SECTION(ad_val, "pre_roll_data");

void TLV2548_Init (void)
{
	//memset(__g_average, 0, sizeof(__g_average));

	__SPIA_Init();
	__EXTI1_Init();

    spi_send_halfword(0xA000);                          //tlv2548��ʼ��ȷ���ͺ�
    adc_delay_loop();
    spi_send_halfword(ADC_MODE);                        //ȷ���ɼ�ģʽ
    adc_delay_loop();
//    spi_send_halfword((Uint16)TLV2548_CMD_READ_CFR);
//    adc_delay_loop();
}

/**
 * \brief ADC�ɼ�����SPIƵ��Ϊ250Kʱ���ɼ�����ͨ����Ҫ410us��SPIƵ��Ϊ1Mʱ���ɼ�����ͨ����Ҫ128us
 *
 * \note �������ʱ��ÿ��200ms�ɼ�һ�Σ�5�κ���ƽ��ֵ
 *       ����˶�ʱ���ڵ�������ﵽ���ֵʱ�ɼ�N�Σ��ɼ��������£�
 *       �ٶ�Ϊ0.6��/sʱ������ÿ��24ms��һ�����ֵ������ʱ��375us
 *       �ٶ�Ϊ0.01��/sʱ������ÿ��1440ms��һ�����ֵ������ʱ��22.5ms
 *       ���ٶ�Ϊ0.08��/sʱ������ÿ��180ms��һ�����ֵ
 */

static Uint16 adc_restart_flag = 0;
static Uint16 adc_index = 0;
static Uint16 adc_sw = 1;
static Uint64 ADC_Tick = 0;
static Uint16 static_test_times = 0;
static Uint16 adc_resulte[5][3] = { 0 };
static Uint16 period_old = 0;
static Uint16 cur_5v_en = 0;    /* ʹ��5V�����ɼ� */
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
    const Uint16 channel_cmd[3] = {TLV2548_CMD_SEL_CH0, TLV2548_CMD_SEL_CH1, TLV2548_CMD_SEL_CH3}; /* Ҫ�ɼ���ͨ�� */
    Uint16 i, j;

    if ((g_Axis_Conf.p_pyb->cur_angle_speed == 0) || (mode_lock == 1)) {	/* Suzkfly 2024-11-11 workmode_setΪKEEP_MODE��Ϊ���ٶ�Ϊ0�������յ�����ģʽ֮��ͣ����֮ǰ�ɼ����ĵ���������� */
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

                        /*  ��5��ƽ�� */
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

                spi_send_halfword((Uint16)TLV2548_CMD_READ_FIFO);   //��FIFO
            } else if ((__g_adc_once_stat == 1) && (__g_SPI_isr_flag == 1)) {
                __g_adc_once_stat = 2;
                __g_SPI_isr_flag = 0;
                spi_send_halfword((Uint16)channel_cmd[adc_index]);
            }
        }
    } else {
        if (period_old != g_Axis_Conf.p_pyb->period_cnt_1_4) {  /* ���� */
            period_old = g_Axis_Conf.p_pyb->period_cnt_1_4;

            if ((period_old == 0) || (period_old == 2)) {       /* ��B�� */
                adc_index = 1;
            } else if ((period_old == 1) || (period_old == 3)){ /* ��A�� */
                adc_index = 0;
            }

            if (ElapsedTick(ADC_Tick) >= TICK_200MS) {  /* ��5V���� */
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

                spi_send_halfword((Uint16)TLV2548_CMD_READ_FIFO);   //��FIFO
            } else if ((__g_adc_once_stat == 1) && (__g_SPI_isr_flag == 1)) {
                __g_adc_once_stat = 2;
                __g_SPI_isr_flag = 0;
                spi_send_halfword((Uint16)channel_cmd[2]);
            }
        }
    }
}

#endif










