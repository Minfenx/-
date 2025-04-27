/*
 * sci.c
 *
 *  Created on: 2024年1月16日
 *      Author: Suzkfly
 */
#include "sci.h"
#include <string.h>
#include "cpu_timer0.h"
#include "cpu_timer1.h"

SCI_REV_DATA  sRevData;
SCI_SEND_DATA sSendData;
static Uint16 SCI_ResetFlag = 0;  //SCI错误标志位
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
        GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 1;	/* 将引脚配置为串口 */
    } else if (p_Sci_x == &ScibRegs) {
        GpioMuxRegs.GPGMUX.bit.SCITXDB_GPIOG4 = 1;
        GpioMuxRegs.GPGMUX.bit.SCIRXDB_GPIOG5 = 1;	/* 将引脚配置为串口 */
    }

    EDIS;
}

/**
 * \brief 初始化串口收发器的使能引脚
 *
 * \note 58所的DEMO板上需要
 */

static void __DriverSciFifoInit(volatile struct SCI_REGS* p_Sci_x)
{
    EALLOW;
    p_Sci_x->SCIFFTX.all=0xE045;         //SCI复位；发送FIFO使能；发送FIFO复位；清除发送FIFO中断；禁能发送FIFO中断；FIFO中断水平为0，意思是当发送FIFO中的数减到0时触发发送中断（如果使能了的话）
    p_Sci_x->SCIFFRX.all=0x6021;         //清除接收FIFO溢出标志；复位接收FIFO；接收FIFO中断使能；接收FIFO深度为1，即收到1个字节就触发中断
    p_Sci_x->SCIFFCT.all=0x00;
    p_Sci_x->SCIFFTX.bit.TXFIFOXRESET=1; //将发送器的FIFO指针复位到0
    p_Sci_x->SCIFFRX.bit.RXFIFORESET=1;  //将接收器的FIFO指针复位到0

    p_Sci_x->SCIFFRX.bit.RXFFINTCLR = 1;    //清除接收中断标志位
    p_Sci_x->SCIFFRX.bit.RXFFOVRCLR = 1;    //清零接收溢出标志位

    EDIS;
}

/**
 * \brief 初始化SCI的中断
 *
 * \param[in] p_Sci_x：要初始化的串口指针
 * \param[in] rx_isr：接收中断函数指针
 * \param[in] tx_isr：发送中断函数指针
 */
static void __InitSciInt (volatile struct SCI_REGS* p_Sci_x, PINT rx_isr, PINT tx_isr)
{
    EALLOW;
    PieCtrlRegs.PIECRTL.bit.ENPIE = 1;   // 使能PIE模块
    if (p_Sci_x == &SciaRegs) {
        PieVectTable.RXAINT = rx_isr;
        PieVectTable.TXAINT = tx_isr;

        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // 使能SCIRXINTa：PIEIER9.1
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // 使能SCITXINTa：PIEIER9.2

        IER |= M_INT9;
    } else if (p_Sci_x == &ScibRegs) {
        PieVectTable.RXBINT = rx_isr;
        PieVectTable.TXBINT = tx_isr;

        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // 使能SCIRXINTb：PIEIER9.3
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      // 使能SCITXINTb：PIEIER9.4

        IER |= M_INT9;
    }
    EDIS;    // This is needed to disable write to EALLOW protected registers
}

/**
 * \brief 初始化SCI
 *
 * \parma[in] p_Sci_x：指向SCI寄存器的指针
 * \parma[in] Baud：波特率
 * \parma[in] Parity_En：校验位使能，#0：不使能校验，#1：使能校验
 * \parma[in] Parity：如果Parity_En为1，那么#0：奇校验，#1：偶校验
 * \parma[in] Data_Bits：数据位，该数加1表示数据位，如传入7则表示8位数据位
 * \parma[in] Stop_Bits：停止位，#0:1位停止位，#1:2位停止位
 * \parma[in] rx_isr：接收中断函数指针
 * \parma[in] tx_isr：发送中断函数指针
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

    __InitSciGpio(p_Sci_x);                      //配置SCI的GPIO

    EALLOW;
    SciBaud = (unsigned int)(LSYSCLK/(Baud*8) - 1);//

    p_Sci_x->SCICTL1.all =0x0003;  //Enable  TX, RX, internal SCICLK,
                                   //Disable RX ERR, SLEEP, TXWAKE
    p_Sci_x->SCICCR.all = 0x0000;
    p_Sci_x->SCICCR.all = Data_Bits | (Parity_En<<5) | (Parity<<6) | (Stop_Bits<<7);

    p_Sci_x->SCICTL2.bit.RXBKINTENA = 0;  //禁能RXRDY/BRKDT中断请求
    p_Sci_x->SCICTL2.bit.TXINTENA   = 0;  //禁能TXRDY中断请求

    p_Sci_x->SCIHBAUD = (SciBaud>>8) & 0x00FF;
    p_Sci_x->SCILBAUD = SciBaud&0x00FF;  //  BRR=SYSCLKOUT/(2*LOSPCP*Baud*8)-1

    __DriverSciFifoInit(p_Sci_x);
    __InitSciInt(p_Sci_x, rx_isr, tx_isr);

    p_Sci_x->SCICTL1.all =0x0023;  // Relinquish SCI from Reset，
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
    	/** <\brief 正常情况下接收到新数据后RXRDY会被置位，读取SCIRXBUF后RXRDY自动清零。实际调试中发现无论什么时候RXRDY永远为0，写1也无效  ,由于开启了FIFO功能所以RXRDY不再置位*/
        //if ((sRevData.RevFinished == FALSE) && (p_SciRegs->SCIRXST.bit.RXRDY == 0) && (sRevData.RevLength < MAX_REV_SIZE))
        if ((sRevData.RevFinished == FALSE) && (p_SciRegs->SCIFFRX.bit.RXFFINT == 1) && (sRevData.RevLength < MAX_REV_SIZE)) {
#if 0		/**
 	 	 	 * \brief 有可能执行到这一句的时候FIFO中的数不止1个。
 	 	 	 * 但是这么写也没问题，因为只要设置了接收FIFO深度为1，即便此时FIFO中已经有2个数据，并且只取了1个数据，下次还会进中断。
 	 	 	 * 如果有更高优先级的中断发生，那么程序可能会来不及取数据，改成后面的形式是为了防止这种情况。
 	 	 	 */
            sRevData.RevBuffer[sRevData.RevLength++] = p_SciRegs->SCIRXBUF.all;
#else
            while (p_SciRegs->SCIFFRX.bit.RXFIFST > 0) {	/* 将FIFO中的数据全部取出来 */
        		sRevData.RevBuffer[sRevData.RevLength++] = p_SciRegs->SCIRXBUF.all;
        	}
#endif
            SCI_RevTick = GetSysTick();		/* 每次收到新数据则更新SCI_RevTick，超过10ms没更新则认为一包数据接收完  */
            sRevData.RevFlag = TRUE;
        } else {    /* 必须把数读掉，否则会一直进中断 */
            while (p_SciRegs->SCIFFRX.bit.RXFIFST > 0) {    /* 将FIFO中的数据全部取出来 */
                temp = p_SciRegs->SCIRXBUF.all;
            }
        }
    }
    p_SciRegs->SCIFFRX.bit.RXFFOVRCLR = 1;        //清零接收溢出标志位
    p_SciRegs->SCIFFRX.bit.RXFFINTCLR = 1;        //消除FIFO INT中断
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;    //应答中断，使得CPU能响应该组其他中断
}

#pragma CODE_SECTION(SciRevTimeout, "ramfuncs");
void SciRevTimeout(void)
{
	volatile struct SCI_REGS* p_SciRegs = &SciaRegs;
	/* 如果开始接收数据 ,从接收到数据开始1ms不再有数据更新，认为一包数据接收完成*/
    if((TRUE == sRevData.RevFlag) && (ElapsedTick(SCI_RevTick) > TICK_1MS))
    {
        sRevData.RevFlag = FALSE;
        sRevData.RevFinished = TRUE;
    }

    //TODO ZKF出错处理的逻辑还要再考虑考虑
    if ((TRUE == SCI_ResetFlag) && (ElapsedTick(SCI_ResetTick) > TICK_50MS))   //SCI有错误，且时间大于50ms，则复位SCI
    {
        SCI_ResetFlag = FALSE;               //清除SCI复位标志位
        p_SciRegs->SCICTL1.bit.SWRESET = 1;  //恢复SCI
    }
}

#pragma CODE_SECTION(SendByteArray_SCI, "ramfuncs");
#if 0
void SendByteArray_SCI()
{
	volatile struct SCI_REGS* p_SciRegs = &SciaRegs;

	//while (p_SciRegs->SCIFFTX.bit.TXFFST > 0);	/* 先确保FIFO里面没有数据 */
//    while (sSendData.Index < sSendData.Length) {
//    	if (p_SciRegs->SCIFFTX.bit.TXFFST < 16) {
//    		p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
//    	}
//    }

	/* 先把FIFO填满，剩下未发送的数据给交给发送中断函数 */
    while ((p_SciRegs->SCIFFTX.bit.TXFFST < 16) && (sSendData.Index != sSendData.Length)) {
    	p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
    }

    if (p_SciRegs->SCICTL2.bit.TXEMPTY == 0) {  //0h (R/W) = Transmitter buffer or shift register or both are loaded with data
    	EALLOW;
		p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;  //TXFIFO Interrupt Clear
		p_SciRegs->SCIFFTX.bit.TXFFIENA = 1;	  //TX FIFO Interrupt Enable，此时使能FIFO发送中断不会进中断函数，只有当发送FIFO中的数据都发出去以后才会进发送中断函数
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

    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;    /* 3个板子的PIE中断都是第9组 */
}
#else   /* 明君 */
void SendByteArray_SCI()
{
    volatile struct SCI_REGS* p_SciRegs;
    p_SciRegs = &SciaRegs;

    /* 先把FIFO填满，剩下未发送的数据给交给发送中断函数 */
    p_SciRegs->SCIFFTX.bit.TXFIFOXRESET = 0;        //复位FIFO指针
    p_SciRegs->SCIFFTX.bit.TXFIFOXRESET = 1;        //重新使能发送FIFO操作
    while ((p_SciRegs->SCIFFTX.bit.TXFFST < 16) && (sSendData.Index != sSendData.Length)) {
        p_SciRegs->SCITXBUF = (*(sSendData.Buffer + sSendData.Index++)) & 0xFF;
    }

    // 如果还有数据待发送，启用发送FIFO中断
    if (sSendData.Index < sSendData.Length) {
        EALLOW;
        p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;        // 清空发送FIFO中断标志
        p_SciRegs->SCIFFTX.bit.TXFFIENA = 1;        // 使能发送FIFO中断
        EDIS;
    }
}

/** ************************************************************************
 * @brief   SCI发送中断服务函数
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
        // 如果所有数据已发送，禁用发送FIFO中断
        if (sSendData.Index >= sSendData.Length) {  //最后一次填满FIFO关闭TXFIFO中断
            EALLOW;
            p_SciRegs->SCIFFTX.bit.TXFFIENA = 0;   // 禁能发送FIFO中断
            EDIS;
            sSendData.Index = 0;
            sSendData.Length = 0;
            sSendData.Buffer = NULL;

        }
        EALLOW;
        p_SciRegs->SCIFFTX.bit.TXINTCLR = 1;      // 清空发送FIFO中断标志
        EDIS;
    }
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP9;
}
#endif

/**
 * \brief 通过串口发送数据
 *
 * \param[in] DataBuf：要发送的数据
 * \param[in] len：要发送的数据长度
 */
void SCI_SendData (Uint16 *DataBuf, Uint16 len)
{
	while (g_angle_is_reading == 1);	/* 如果正在测角，那么等待测角完成 */

    sSendData.Index  = 0;
	sSendData.Length = len;
	sSendData.Buffer = DataBuf;

    SendByteArray_SCI();
}
