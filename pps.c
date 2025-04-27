/*
 * pps.c
 *
 *  Created on: 2024年1月17日
 *      Author: Suzkfly
 */


/*
 * GL_PPS.c
 *
 *  Created on: 2023年12月13日
 *      Author: Suzkfly
 */
#include "pps.h"
#include "project.h"

/* 每接收到一次PPS，该值增加 */
Uint32 g_pps = 0;
#pragma DATA_SECTION(g_pps, "pre_roll_data");

#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
/**
 * \brief 初始化XINT13的GPIO
 */
static void __InitXINT2Gpio (void)
{
	EALLOW;
	//GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//设置XINT1
	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//设置XINT2
	//GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//设置XINT13
    EDIS;
}
#elif BOARD_NAME == BOARD_GMS
/**
 * \brief 初始化XINT13的GPIO
 */
static void __InitXINT13Gpio (void)
{
	EALLOW;
	//GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//设置XINT1
	//GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//设置XINT2
	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//设置XINT13
    EDIS;
}
#endif

#pragma CODE_SECTION(NMI_andI_NT2_isr, "ramfuncs");
interrupt void NMI_andI_NT2_isr (void)
{
	g_pps++;
}

#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
static void __EXTI2_Init (void)
{
	__InitXINT2Gpio();                                //初始化IO
    /* 配置外部中断寄存器 */
    EALLOW;
    XIntruptRegs.XINT2CR.bit.POLARITY = 0;              //0：下降沿中断,1：上升沿中断
    /* 外设中断使能 */
    XIntruptRegs.XINT2CR.bit.ENABLE   = 1;              //使能XINT1中断
    /* PIE中断使能 */
	//PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    PieVectTable.XINT2 = &NMI_andI_NT2_isr ;
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;  //PIE组1 ,编号1 XINT1 ,使能PIE中断

    /* cpu INT2中断 */
    IER |= M_INT2;
    EDIS;
}
#elif BOARD_NAME == BOARD_GMS
static void __NMI_Init (void)
{
	__InitXINT13Gpio();									//初始化IO
	/* 配置外部中断寄存器 */
	EALLOW;
	//XIntruptRegs.XNMICR.bit.SELECT = 0;				//用NMI中断不用配置这条

	XIntruptRegs.XNMICR.bit.POLARITY = 0;			//下降沿中断
	/* 外设中断使能 */
	XIntruptRegs.XNMICR.bit.ENABLE = 1;				//使能XINT13中断

	/* 中断地址设置 */
	//PieVectTable.XINT13 = &XINT13_isr;
	PieVectTable.XNMI = &NMI_andI_NT2_isr;

	/* 外部中断13不归PIE管 */

	/* cpu INT13中断 */
	//IER |= M_INT13;	/* 这里不能使能，如果使能的话除了会进NMI中断，也会进INT13中断 */
    EDIS;
}
#endif


/**
 * \brief PPS初始化
 */
void PPS_Init (void)
{
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	__EXTI2_Init();
#elif BOARD_NAME == BOARD_GMS
	__NMI_Init();
#endif
}


