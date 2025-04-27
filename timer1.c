/*
 * timer1.c
 *
 *  Created on: 2024年1月16日
 *      Author: Suzkfly
 */
#include "timer1.h"

#if 0

// 使用前，声明本文件中的相关函数
interrupt void eva_timer1_isr(void);
interrupt void eva_timer2_isr(void);
interrupt void evb_timer3_isr(void);
interrupt void evb_timer4_isr(void);
void init_eva_timer1(void);
void init_eva_timer2(void);
void init_evb_timer3(void);
void init_evb_timer4(void);

/**
 * \brief 定时器1初始化
 */
void Timer1_Init()
{
   EALLOW;
   PieVectTable.T1PINT = &eva_timer1_isr;
//   PieVectTable.T2PINT = &eva_timer2_isr;
//   PieVectTable.T3PINT = &evb_timer3_isr;
//   PieVectTable.T4PINT = &evb_timer4_isr;

   // 对于T1PINT使能PIE 组2中断4
   PieCtrlRegs.PIEIER2.all = M_INT4;
//   // 对于T2PINT使能PIE 组3中断1
//   PieCtrlRegs.PIEIER3.all = M_INT1;
//   //对于T3PINT使能PIE 组4中断4
//   PieCtrlRegs.PIEIER4.all = M_INT4;
//   // 对于T4PINT使能PIE 组5中断1
//   PieCtrlRegs.PIEIER5.all = M_INT1;

   // 对于T1PINT使能CPU IN2；
   // 对于T2PINT使能CPU IN3；对于T3PINT使能CPU IN4；对于T4PINT使能CPU IN5
//   IER |= (M_INT2 | M_INT3 | M_INT4 | M_INT5);
   IER |= M_INT2;
   EDIS;

   init_eva_timer1();
}

void init_eva_timer1(void)
{
    // 初始化 EVA定时器1:
    // 设置通用定时器控制寄存器（EV A）
    EvaRegs.GPTCONA.all = 0;

    EvaRegs.T1PR = 1405;       // 设置周期
    EvaRegs.T1CMPR = 0x0000;     // 比较寄存器

    // 使能定时器1相应的周期中断位
    // 使能T1中期中断，复位周期中断标志位
    EvaRegs.EVAIMRA.bit.T1PINT = 1;
    EvaRegs.EVAIFRA.bit.T1PINT = 1;

    // T1计数寄存器清零
    EvaRegs.T1CNT = 0x0000;
    EvaRegs.T1CON.all = 0x1542;//连续增计数模式，输入时钟为HSPCLK/32，使能定时器操作，使能定时器比较操作
}

#if 0
void init_eva_timer2(void)
{
    // 初始化 EVA定时器2:
    // 设置通用定时器控制寄存器（EV A）
    EvaRegs.GPTCONA.all = 0;

    // 设置通用定时器2的周期值为 0x0400
    EvaRegs.T2PR = 0x0400;       // 设置周期
    EvaRegs.T2CMPR = 0x0000;     // 比较寄存器

    // 使能定时器2相应的周期中断位
    // 使能T2中期中断，复位周期中断标志位
    EvaRegs.EVAIMRB.bit.T2PINT = 1;
    EvaRegs.EVAIFRB.bit.T2PINT = 1;

    // T2计数寄存器清零
    EvaRegs.T2CNT = 0x0000;
    EvaRegs.T2CON.all = 0x1742;

    // 设置定时器2周期中断时启动ADC转换
    EvaRegs.GPTCONA.bit.T2TOADC = 2;
}

void init_evb_timer3(void)
{
    // 初始化 EVB定时器3:
    // 设置通用定时器控制寄存器（EV A）
    EvbRegs.GPTCONB.all = 0;

    // 设置通用定时器3的周期值为 0x0800
    EvbRegs.T3PR = 0x0800;       // 设置周期
    EvbRegs.T3CMPR = 0x0000;     // 比较寄存器

    // 使能定时器3相应的周期中断位
    // 使能T3中期中断，复位周期中断标志位
    EvbRegs.EVBIMRA.bit.T3PINT = 1;
    EvbRegs.EVBIFRA.bit.T3PINT = 1;

    // T3计数寄存器清零
    EvbRegs.T3CNT = 0x0000;
    EvbRegs.T3CON.all = 0x1742;

    // 设置定时器3周期中断时启动ADC转换
    EvbRegs.GPTCONB.bit.T3TOADC = 2;
}

void init_evb_timer4(void)
{
    // 初始化 EVB定时器4:
    // 设置通用定时器控制寄存器（EV A）
    EvbRegs.GPTCONB.all = 0;

    // 设置通用定时器4的周期值为 0x1000
    EvbRegs.T4PR = 0x1000;       // 设置周期
    EvbRegs.T4CMPR = 0x0000;     // 比较寄存器

    // 使能定时器4相应的周期中断位
    // 使能T4中期中断，复位周期中断标志位
    EvbRegs.EVBIMRB.bit.T4PINT = 1;
    EvbRegs.EVBIFRB.bit.T4PINT = 1;

    // T4计数寄存器清零
    EvbRegs.T4CNT = 0x0000;
    EvbRegs.T4CON.all = 0x1742;

     // 设置定时器4周期中断时启动ADC转换
    EvbRegs.GPTCONB.bit.T4TOADC = 2;
}
#endif


interrupt void eva_timer1_isr(void)
{
   // 使能T1周期中断
   EvaRegs.EVAIMRA.bit.T1PINT = 1;

   // 清除T1PINT标志位
   EvaRegs.EVAIFRA.all = BIT7;

   // 应答位写1，允许PIE组2再次接受中断
   PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;
}

#if 0
interrupt void eva_timer2_isr(void)
{
  EvaTimer2InterruptCount++;
 // 使能T2周期中断
  EvaRegs.EVAIMRB.bit.T2PINT = 1;

  // 清除T2PINT标志位
  EvaRegs.EVAIFRB.all = BIT0;

  // 应答位写1，允许PIE组3再次接受中断
  PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

interrupt void evb_timer3_isr(void)
{
  EvbTimer3InterruptCount++;
  // 清除T3PINT标志位
  EvbRegs.EVBIFRA.all = BIT7;

  // 应答位写1，允许PIE组4再次接受中断
  PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;

}

interrupt void evb_timer4_isr(void)
{
   EvbTimer4InterruptCount++;
   // 清除T4PINT标志位
   EvbRegs.EVBIFRB.all = BIT0;

   // 应答位写1，允许PIE组5再次接受中断
   PieCtrlRegs.PIEACK.all |= PIEACK_GROUP5;

}
#endif

#endif
