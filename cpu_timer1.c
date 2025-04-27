#include "project.h"
#if (BOARD_NAME == BOARD_GMS)
/*
 * cpu_timer1.c
 *
 *  Created on: 2024��2��28��
 *      Author: Suzkfly
 */
#include "cpu_timer1.h"
#include "GL_CJW_HE_5701.h"

/* ���Ƕ���ɵı�־ */
Uint16 g_CJW_HE_5701_read_finish = 0;

/* ������Ч��־ */
Uint16 g_CJW_HE_5701_read_acitve = 0;

/* ��ȡ���ĽǶ�ֵ��ԭʼֵ */
Uint16 g_CJW_HE_5701_angle = 0;

/* ׼����ȡ�Ƕ�ֵ */
Uint16 g_read_angle_ready = 0;

/* ���ڶ�ȡ�Ƕȵı�־�����ڶ�ȡ�Ƕȵ�ʱ���ܽ��봮�ڷ����жϣ������жϣ� */
Uint16 g_angle_is_reading = 0;

//#pragma DATA_SECTION(g_CJW_HE_5701_read_finish, "pre_roll_data");
//#pragma DATA_SECTION(g_CJW_HE_5701_read_acitve, "pre_roll_data");
//#pragma DATA_SECTION(g_CJW_HE_5701_angle, "pre_roll_data");
//#pragma DATA_SECTION(g_read_angle_ready, "pre_roll_data");

static Uint16 s_clk_level = 1;
static Uint16 s_clk_cnt = 0;        /* �½��ؼ��� */
static Uint16 s_data = 0;
static Uint16 s_check_bit_ok = 0;       /* У��λOK */
static Uint16 s_timeout_bit_ok = 0;     /* ��ʱλOK */
//#pragma DATA_SECTION(s_clk_level, "pre_roll_data");
//#pragma DATA_SECTION(s_clk_cnt, "pre_roll_data");
//#pragma DATA_SECTION(s_data, "pre_roll_data");
//#pragma DATA_SECTION(s_check_bit_ok, "pre_roll_data");
//#pragma DATA_SECTION(s_timeout_bit_ok, "pre_roll_data");

#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
interrupt void cpu_timer1_isr(void)
{
//	CpuTimer1.InterruptCount++;

	if (g_CJW_HE_5701_read_finish == 0) {   /* ������ж���Ϊ�˷�ֹ��ʱ���Ѿ�ֹͣ��ʱ���ˣ����ǻ��ǻ���жϣ����жϱ�־û�ã� */
        if (s_clk_level == 1) {
            if (s_clk_cnt < 18) {
                s_clk_level = 0;
                CJW_HE_5701_CLEAR_CLK();	/* ʱ������ */
            }

            s_clk_cnt++;

            if ((s_clk_cnt > 1) && (s_clk_cnt < 18)) {	/* ������ */
                s_data <<= 1;
                s_data |= CJW_HE_5701_Read_Bit();
            } else if (s_clk_cnt == 18) {               /* У��λ */
                if (CJW_HE_5701_Read_Bit()  == 1) { /* �ɹ� */
                    g_CJW_HE_5701_angle = s_data;
                    s_data = 0;
                    s_check_bit_ok = 1;     /* У��λOK */
                } else {
                    s_check_bit_ok = 0;
                }
            } else if (s_clk_cnt == 19) {
                if (CJW_HE_5701_Read_Bit() == 0) {  /* ��ʱλ�ɹ� */
                    s_timeout_bit_ok = 1;           /* ��ʱλOK */
                } else {
                    s_timeout_bit_ok = 0;
                }

                if ((s_check_bit_ok == 1) && (s_timeout_bit_ok == 1)) {
                    g_CJW_HE_5701_read_acitve = 1;  /* ������Ч */
                } else {
                    g_CJW_HE_5701_read_acitve = 0;  /* ������Ч */
                }

                s_clk_cnt = 0;
                g_CJW_HE_5701_read_finish = 1;
                g_angle_is_reading = 0;
                StopCpuTimer1();

                /* �رն�ʱ��֮��Ҫ��һ�¶�ʱ���жϱ�־�������ʱ�й���Ķ�ʱ���жϡ��⽫�����ٽ�һ���жϣ�ʱ���߱����� */
                IFR &= ~M_INT13;
                PieCtrlRegs.PIEIER2.bit.INTx6 = 1;      /* ����PWM�ж� */
            }
        } else {
            s_clk_level = 1;
            CJW_HE_5701_SET_CLK();	/* ʱ������ */
        }
	}
}

/**
 * \brief ��ʼ��CPU Timer1
 */
void CPU_Timer1_Init (void)
{
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.XINT13 = &cpu_timer1_isr;
	XIntruptRegs.XNMICR.bit.SELECT = 0;			//��ʱ��1���ӵ�INT13

   EDIS;    // This is needed to disable write to EALLOW protected registers

   /* ��CPUTimer0���Ѿ����ù��ú����������ٴε��ã�����CPUTimer0���жϽ���ȥ */
   //InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0 to interrupt every second:
// 100MHz CPU Freq, 1 second Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer1, 90, 1);
   //StartCpuTimer1();		/* ��ʼ��ʱ����CPUTimer1 */

   IFR &= ~M_INT13;
   IER |= M_INT13;
}
#endif
