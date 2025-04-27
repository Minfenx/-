/*
 * sci.c
 *
 *  Created on: 2024��1��16��
 *      Author: Suzkfly
 */
#include "sci.h"
#include <string.h>
#include "cpu_timer0.h"
#include "cpu_timer1.h"

SCI_REV_DATA  sRevData;
SCI_SEND_DATA sSendData;
static Uint16 SCI_ResetFlag = 0;  //SCI�����־λ
static Uint64 SCI_ResetTick = 0;
static Uint64 SCI_RevTick = 0;

//
//#ifndef  MIN
//#define  MIN(a,b) (((a)<(b))?(a):(b))
//#endif

static void __InitSciGpio(volatile struct SCI_REGS* p_Sci_x)
{
    EALLOW;
    if (p_Sci_x == &SciaRegs) {
        GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4 = 1;
        GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 1;	/* ����������Ϊ���� */
    } else if (p_Sci_x == &ScibRegs) {
        GpioMuxRegs.GPGMUX.bit.SCITXDB_GPIOG4 = 1;
        GpioMuxRegs.GPGMUX.bit.SCIRXDB_GPIOG5 = 1;	/* ����������Ϊ���� */
    }

    EDIS;
}

/**
 * \brief ��ʼ�������շ�����ʹ������
 *
 * \note 58����DEMO������Ҫ
 */

static void __DriverSciFifoInit(volatile struct SCI_REGS* p_Sci_x)
{
    EALLOW;
    p_Sci_x->SCIFFTX.all=0xE045;         //SCI��λ������FIFOʹ�ܣ�����FIFO��λ���������FIFO�жϣ����ܷ���FIFO�жϣ�FIFO�ж�ˮƽΪ0����˼�ǵ�����FIFO�е�������0ʱ���������жϣ����ʹ���˵Ļ���
    p_Sci_x->SCIFFRX.all=0x6021;         //�������FIFO�����־����λ����FIFO������FIFO�ж�ʹ�ܣ�����FIFO���Ϊ1�����յ�1���ֽھʹ����ж�
    p_Sci_x->SCIFFCT.all=0x00;
    p_Sci_x->SCIFFTX.bit.TXFIFOXRESET=1; //����������FIFOָ�븴λ��0
    p_Sci_x->SCIFFRX.bit.RXFIFORESET=1;  //����������FIFOָ�븴λ��0

    p_Sci_x->SCIFFRX.bit.RXFFINTCLR = 1;    //��������жϱ�־λ
    p_Sci_x->SCIFFRX.bit.RXFFOVRCLR = 1;    //������������־λ

    EDIS;
}

/**
 * \brief ��ʼ��SCI���ж�
 *
 * \param[in] p_Sci_x��Ҫ��ʼ���Ĵ���ָ��
 * \param[in] rx_isr�������жϺ���ָ��
 * \param[in] tx_isr�������жϺ���ָ��
 */
static void __InitSciInt (volatile struct SCI_REGS* p_Sci_x, PINT rx_isr, PINT tx_isr)
{
    EALLOW;
    PieCtrlRegs.PIECRTL.bit.ENPIE = 1;   // ʹ��PIEģ��
    if (p_Sci_x == &SciaRegs) {
        PieVectTable.RXAINT = rx_isr;
        PieVectTable.TXAINT = tx_isr;

        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // ʹ��SCIRXINTa��PIEIER9.1
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // ʹ��SCITXINTa��PIEIER9.2

        IER |= M_INT9;
    } else if (p_Sci_x == &ScibRegs) {
        PieVectTable.RXBINT = rx_isr;
        PieVectTable.TXBINT = tx_isr;

        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // ʹ��SCIRXINTb��PIEIER9.3
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      // ʹ��SCITXINTb��PIEIER9.4

        IER |= M_INT9;
    }
    EDIS;    // This is needed to disable write to EALLOW protected registers
}

/**
 * \brief ��ʼ��SCI
 *
 * \parma[in] p_Sci_x��ָ��SCI�Ĵ�����ָ��
 * \parma[in] Baud��������
 * \parma[in] Parity_En��У��λʹ�ܣ�#0����ʹ��У�飬#1��ʹ��У��
 * \parma[in] Parity�����Parity_EnΪ1����ô#0����У�飬#1��żУ��
 * \parma[in] Data_Bits������λ��������1��ʾ����λ���紫��7���ʾ8λ����λ
 * \parma[in] Stop_Bits��ֹͣλ��#0:1λֹͣλ��#1:2λֹͣλ
 * \parma[in] rx_isr�������жϺ���ָ��
 * \parma[in] tx_isr�������жϺ���ָ��
 */
void DriverSciInit(volatile struct SCI_REGS* p_Sci_x,
                                         Uint32 Baud,
                                         Uint16 Parity_En,
                                         Uint16 Parity,
                                         Uint16 Data_Bits,
                                         Uint16 Stop_Bits,
                                         PINT rx_isr,
                                         PINT tx_isr)
{
    Uint16 SciBaud = 0x0;

    __InitSciGpio(p_Sci_x);                      //����SCI��GPIO

    EALLOW;
    SciBaud = (unsigned int)(LSYSCLK/(Baud*8) - 1);//

    p_Sci_x->SCICTL1.all =0x0003;  //Enable  TX, RX, internal SCICLK,
                                   //Disable RX ERR, SLEEP, TXWAKE
    p_Sci_x->SCICCR.all = 0x0000;
    p_Sci_x->SCICCR.all = Data_Bits | (Parity_En<<5) | (Parity<<6) | (Stop_Bits<<7);

    p_Sci_x->SCICTL2.bit.RXBKINTENA = 0;  //����RXRDY/BRKDT�ж�����
    p_Sci_x->SCICTL2.bit.TXINTENA   = 0;  //����TXRDY�ж�����

    p_Sci_x->SCIHBAUD = (SciBaud>>8) & 0x00FF;
    p_Sci_x->SCILBAUD = SciBaud&0x00FF;  //  BRR=SYSCLKOUT/(2*LOSPCP*Baud*8)-1

    __DriverSciFifoInit(p_Sci_x);
    __InitSciInt(p_Sci_x, rx_isr, tx_isr);

    p_Sci_x->SCICTL1.all =0x0023;  // Relinquish SCI from Reset��
    EDIS;

    memset(&sRevData,  0, sizeof(sRevData));
    memset(&sSendData, 0, sizeof(sSendData));
}

#pragma CODE_SECTION(SCI_Rx_isr, "ramfuncs");
interrupt void SCI_Rx_isr(void)
{
	volatile struct SCI_REGS* p_SciRegs = &SciaRegs;
	volatile Uint16 temp;

    if (p_SciRegs->SCIRXST.bit.RXERROR != 0)
    {
    	p_SciRegs->SCICTL1.bit.SWRESET = 0;
        SCI_ResetFlag = TRUE;                //SCI Err
        SCI_ResetTick = GetSysTick();
    }
    else
    {
    	/** <\brief ��������½��յ������ݺ�RXRDY�ᱻ��λ����ȡSCIRXBUF��RXRDY�Զ����㡣ʵ�ʵ����з�������ʲôʱ��RXRDY��ԶΪ0��д1Ҳ��Ч  ,���ڿ�����FIFO��������RXRDY������λ*/
        //if ((sRevData.RevFinished == FALSE) && (p_SciRegs->SCIRXST.bit.RXRDY == 0) && (sRevData.RevLength < MAX_REV_SIZE))
        if ((sRevData.RevFinished == FALSE) && (p_SciRegs->SCIFFRX.bit.RXFFINT == 1) && (sRevData.RevLength < MAX_REV_SIZE)) {
#if 0		/**
 	 	 	 * \brief �п���ִ�е���һ���ʱ��FIFO�е�����ֹ1����
 	 	 	 * ������ôдҲû���⣬��ΪֻҪ�����˽���FIFO���Ϊ1�������ʱFIFO���Ѿ���2�����ݣ�����ֻȡ��1�����ݣ��´λ�����жϡ�
 	 	 	 * ����и������ȼ����жϷ�������ô������ܻ�������ȡ���ݣ��ĳɺ������ʽ��Ϊ�˷�ֹ���������
 	 	 	 */
            sRevData.RevBuffer[sRevData.RevLength++] = p_SciRegs->SCIRXBUF.all;
#else
            while (p_SciRegs->SCIFFRX.bit.RXFIFST > 0) {	/* ��FIFO�е�����ȫ��ȡ���� */
        		sRevData.RevBuffer[sRevData.RevLength++] = p_SciRegs->SCIRXBUF.all;
        	}
#endif
            SCI_RevTick = GetSysTick();		/* ÿ���յ������������SCI_RevTick������10msû��������Ϊһ�����ݽ�����  */
            sRevData.RevFlag = TRUE;
        } else {    /* ������������������һֱ���ж� */
            while (p_SciRegs->SCIFFRX.bit.RXFIFST > 0) {    /* ��FIFO�е�����ȫ��ȡ���� */
                temp = p_SciRegs->SCIRXBUF.all;
            }
        }
    }
    p_SciRegs->SCIFFRX.bit.RXFFOVRCLR = 1;        //������������־λ
    p_SciRegs->SCIFFRX.bit.RXFFINTCLR = 1;        //����FIFO INT�ж�
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;    //Ӧ���жϣ�ʹ��CPU����Ӧ���������ж�
}

#pragma CODE_SECTION(SciRevTimeout, "ramfuncs");
void SciRevTimeout(void)
{
	volatile struct SCI_REGS* p_SciRegs = &SciaRegs;
	/* �����ʼ�������� ,�ӽ��յ����ݿ�ʼ1ms���������ݸ��£���Ϊһ�����ݽ������*/
    if((TRUE == sRevData.RevFlag) && (ElapsedTick(SCI_RevTick) > TICK_1MS))
    {
        sRevData.RevFlag = FALSE;
        sRevData.RevFinished = TRUE;
    }

    //TODO ZKF��������߼���Ҫ�ٿ��ǿ���
    if ((TRUE == SCI_ResetFlag) && (ElapsedTick(SCI_ResetTick) > TICK_50MS))   //SCI�д�����ʱ�����50ms����λSCI
    {
        SCI_ResetFlag = FALSE;               //���SCI��λ��־λ
        p_SciRegs->SCICTL1.bit.SWRESET = 1;  //�ָ�SCI
    }
}

#pragma CODE_SECTION(SendByteArray_SCI, "ramfuncs");
#if 0
void SendByteArray_SCI()
{
	volatile struct SCI_REGS* p_SciRegs = &SciaRegs;

	//while (p_SciRegs->SCIFFTX.bit.TXFFST > 0);	/* ��ȷ��FIFO����û������ */
//    while (sSendData.Index < sSendData.Length) {
//    	if (p_SciRegs->SCIFFTX.bit.TXFFST < 16) {
//    		p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
//    	}
//    }

	/* �Ȱ�FIFO������ʣ��δ���͵����ݸ����������жϺ��� */
    while ((p_SciRegs->SCIFFTX.bit.TXFFST < 16) && (sSendData.Index != sSendData.Length)) {
    	p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
    }

    if (p_SciRegs->SCICTL2.bit.TXEMPTY == 0) {  //0h (R/W) = Transmitter buffer or shift register or both are loaded with data
    	EALLOW;
		p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;  //TXFIFO Interrupt Clear
		p_SciRegs->SCIFFTX.bit.TXFFIENA = 1;	  //TX FIFO Interrupt Enable����ʱʹ��FIFO�����жϲ�����жϺ�����ֻ�е�����FIFO�е����ݶ�����ȥ�Ժ�Ż�������жϺ���
		EDIS;
    } else {
    	sSendData.Index = 0;
    	sSendData.Length = 0;
    	sSendData.Buffer = 0;
    }
}

interrupt void SCI_TxFifo_isr(void)
{
    volatile struct SCI_REGS* p_SciRegs = &SciaRegs;

    if(p_SciRegs->SCIFFTX.bit.TXFFINT != 0) {  //TX FIFO Interrupt Flag
        EALLOW;
        p_SciRegs->SCIFFTX.bit.TXFFIENA = 0;
        p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;
        EDIS;

        SendByteArray_SCI();
    }

    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;    /* 3�����ӵ�PIE�ж϶��ǵ�9�� */
}
#else   /* ���� */
void SendByteArray_SCI()
{
    volatile struct SCI_REGS* p_SciRegs;
    p_SciRegs = &SciaRegs;

    /* �Ȱ�FIFO������ʣ��δ���͵����ݸ����������жϺ��� */
    p_SciRegs->SCIFFTX.bit.TXFIFOXRESET = 0;        //��λFIFOָ��
    p_SciRegs->SCIFFTX.bit.TXFIFOXRESET = 1;        //����ʹ�ܷ���FIFO����
    while ((p_SciRegs->SCIFFTX.bit.TXFFST < 16) && (sSendData.Index != sSendData.Length)) {
        p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
    }

    // ����������ݴ����ͣ����÷���FIFO�ж�
    if (sSendData.Index < sSendData.Length) {
        EALLOW;
        p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;        // ��շ���FIFO�жϱ�־
        p_SciRegs->SCIFFTX.bit.TXFFIENA = 1;        // ʹ�ܷ���FIFO�ж�
        EDIS;
    }
}

/** ************************************************************************
 * @brief   SCI�����жϷ�����
 * @return  None
 * @note
 **************************************************************************/
#pragma CODE_SECTION(SCI_TxFifo_isr, "ramfuncs");
interrupt void SCI_TxFifo_isr(void)
{
    volatile struct SCI_REGS* p_SciRegs;
    p_SciRegs = &SciaRegs;
    if(p_SciRegs->SCIFFTX.bit.TXFFINT != 0) {
        while ((p_SciRegs->SCIFFTX.bit.TXFFST < 16) && (sSendData.Index < sSendData.Length)) {
            p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
        }
        // ������������ѷ��ͣ����÷���FIFO�ж�
        if (sSendData.Index >= sSendData.Length) {  //���һ������FIFO�ر�TXFIFO�ж�
            EALLOW;
            p_SciRegs->SCIFFTX.bit.TXFFIENA = 0;   // ���ܷ���FIFO�ж�
            EDIS;
            sSendData.Index = 0;
            sSendData.Length = 0;
            sSendData.Buffer = NULL;

        }
        EALLOW;
        p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;      // ��շ���FIFO�жϱ�־
        EDIS;
    }
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}
#endif

/**
 * \brief ͨ�����ڷ�������
 *
 * \param[in] DataBuf��Ҫ���͵�����
 * \param[in] len��Ҫ���͵����ݳ���
 */
void SCI_SendData (Uint16 *DataBuf, Uint16 len)
{
	while (g_angle_is_reading == 1);	/* ������ڲ�ǣ���ô�ȴ������� */

    sSendData.Index  = 0;
	sSendData.Length = len;
	sSendData.Buffer = DataBuf;

    SendByteArray_SCI();
}
