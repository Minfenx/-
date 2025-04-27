#include "project.h"
#if (BOARD_NAME == BOARD_GMS)
/*
 * cpu_timer1.c
 *
 *  Created on: 2024年2月28日
 *      Author: Suzkfly
 */
#include "cpu_timer1.h"
#include "GL_CJW_HE_5701.h"

/* 读角度完成的标志 */
Uint16 g_CJW_HE_5701_read_finish = 0;

/* 数据有效标志 */
Uint16 g_CJW_HE_5701_read_acitve = 0;

/* 读取到的角度值，原始值 */
Uint16 g_CJW_HE_5701_angle = 0;

/* 准备读取角度值 */
Uint16 g_read_angle_ready = 0;

/* 正在读取角度的标志，正在读取角度的时候不能进入串口发送中断（或别的中断） */
Uint16 g_angle_is_reading = 0;

//#pragma DATA_SECTION(g_CJW_HE_5701_read_finish, "pre_roll_data");
//#pragma DATA_SECTION(g_CJW_HE_5701_read_acitve, "pre_roll_data");
//#pragma DATA_SECTION(g_CJW_HE_5701_angle, "pre_roll_data");
//#pragma DATA_SECTION(g_read_angle_ready, "pre_roll_data");

static Uint16 s_clk_level = 1;
static Uint16 s_clk_cnt = 0;        /* 下降沿计数 */
static Uint16 s_data = 0;
static Uint16 s_check_bit_ok = 0;       /* 校验位OK */
static Uint16 s_timeout_bit_ok = 0;     /* 超时位OK */
//#pragma DATA_SECTION(s_clk_level, "pre_roll_data");
//#pragma DATA_SECTION(s_clk_cnt, "pre_roll_data");
//#pragma DATA_SECTION(s_data, "pre_roll_data");
//#pragma DATA_SECTION(s_check_bit_ok, "pre_roll_data");
//#pragma DATA_SECTION(s_timeout_bit_ok, "pre_roll_data");

#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
interrupt void cpu_timer1_isr(void)
{
//	CpuTimer1.InterruptCount++;

	if (g_CJW_HE_5701_read_finish == 0) {   /* 加这个判断是为了防止有时候已经停止定时器了，但是还是会进中断（清中断标志没用） */
        if (s_clk_level == 1) {
            if (s_clk_cnt < 18) {
                s_clk_level = 0;
                CJW_HE_5701_CLEAR_CLK();	/* 时钟拉低 */
            }

            s_clk_cnt++;

            if ((s_clk_cnt > 1) && (s_clk_cnt < 18)) {	/* 读数据 */
                s_data <<= 1;
                s_data |= CJW_HE_5701_Read_Bit();
            } else if (s_clk_cnt == 18) {               /* 校验位 */
                if (CJW_HE_5701_Read_Bit()  == 1) { /* 成功 */
                    g_CJW_HE_5701_angle = s_data;
                    s_data = 0;
                    s_check_bit_ok = 1;     /* 校验位OK */
                } else {
                    s_check_bit_ok = 0;
                }
            } else if (s_clk_cnt == 19) {
                if (CJW_HE_5701_Read_Bit() == 0) {  /* 超时位成功 */
                    s_timeout_bit_ok = 1;           /* 超时位OK */
                } else {
                    s_timeout_bit_ok = 0;
                }

                if ((s_check_bit_ok == 1) && (s_timeout_bit_ok == 1)) {
                    g_CJW_HE_5701_read_acitve = 1;  /* 数据有效 */
                } else {
                    g_CJW_HE_5701_read_acitve = 0;  /* 数据无效 */
                }

                s_clk_cnt = 0;
                g_CJW_HE_5701_read_finish = 1;
                g_angle_is_reading = 0;
                StopCpuTimer1();

                /* 关闭定时器之后要清一下定时器中断标志，避免此时有挂起的定时器中断。这将导致再进一次中断，时钟线被拉低 */
                IFR &= ~M_INT13;
                PieCtrlRegs.PIEIER2.bit.INTx6 = 1;      /* 开启PWM中断 */
            }
        } else {
            s_clk_level = 1;
            CJW_HE_5701_SET_CLK();	/* 时钟拉高 */
        }
	}
}

/**
 * \brief 初始化CPU Timer1
 */
void CPU_Timer1_Init (void)
{
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.XINT13 = &cpu_timer1_isr;
	XIntruptRegs.XNMICR.bit.SELECT = 0;			//定时器1连接到INT13

   EDIS;    // This is needed to disable write to EALLOW protected registers

   /* 在CPUTimer0中已经调用过该函数，不能再次调用，否则CPUTimer0的中断进不去 */
   //InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0 to interrupt every second:
// 100MHz CPU Freq, 1 second Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer1, 90, 1);
   //StartCpuTimer1();		/* 初始化时不开CPUTimer1 */

   IFR &= ~M_INT13;
   IER |= M_INT13;
}
#endif
