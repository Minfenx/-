/*
 * pps.c
 *
 *  Created on: 2024��1��17��
 *      Author: Suzkfly
 */


/*
 * GL_PPS.c
 *
 *  Created on: 2023��12��13��
 *      Author: Suzkfly
 */
#include "pps.h"
#include "project.h"

/* ÿ���յ�һ��PPS����ֵ���� */
Uint32 g_pps = 0;
#pragma DATA_SECTION(g_pps, "pre_roll_data");

#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
/**
 * \brief ��ʼ��XINT13��GPIO
 */
static void __InitXINT2Gpio (void)
{
	EALLOW;
	//GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//����XINT1
	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//����XINT2
	//GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//����XINT13
    EDIS;
}
#elif BOARD_NAME == BOARD_GMS
/**
 * \brief ��ʼ��XINT13��GPIO
 */
static void __InitXINT13Gpio (void)
{
	EALLOW;
	//GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//����XINT1
	//GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//����XINT2
	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//����XINT13
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
	__InitXINT2Gpio();                                //��ʼ��IO
    /* �����ⲿ�жϼĴ��� */
    EALLOW;
    XIntruptRegs.XINT2CR.bit.POLARITY = 0;              //0���½����ж�,1���������ж�
    /* �����ж�ʹ�� */
    XIntruptRegs.XINT2CR.bit.ENABLE   = 1;              //ʹ��XINT1�ж�
    /* PIE�ж�ʹ�� */
	//PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    PieVectTable.XINT2 = &NMI_andI_NT2_isr ;
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;  //PIE��1 ,���1 XINT1 ,ʹ��PIE�ж�

    /* cpu INT2�ж� */
    IER |= M_INT2;
    EDIS;
}
#elif BOARD_NAME == BOARD_GMS
static void __NMI_Init (void)
{
	__InitXINT13Gpio();									//��ʼ��IO
	/* �����ⲿ�жϼĴ��� */
	EALLOW;
	//XIntruptRegs.XNMICR.bit.SELECT = 0;				//��NMI�жϲ�����������

	XIntruptRegs.XNMICR.bit.POLARITY = 0;			//�½����ж�
	/* �����ж�ʹ�� */
	XIntruptRegs.XNMICR.bit.ENABLE = 1;				//ʹ��XINT13�ж�

	/* �жϵ�ַ���� */
	//PieVectTable.XINT13 = &XINT13_isr;
	PieVectTable.XNMI = &NMI_andI_NT2_isr;

	/* �ⲿ�ж�13����PIE�� */

	/* cpu INT13�ж� */
	//IER |= M_INT13;	/* ���ﲻ��ʹ�ܣ����ʹ�ܵĻ����˻��NMI�жϣ�Ҳ���INT13�ж� */
    EDIS;
}
#endif


/**
 * \brief PPS��ʼ��
 */
void PPS_Init (void)
{
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	__EXTI2_Init();
#elif BOARD_NAME == BOARD_GMS
	__NMI_Init();
#endif
}


