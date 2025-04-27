/*
 * cpu_timer0.c
 *
 *  Created on: 2024年1月17日
 *      Author: Suzkfly
 */
#include "cpu_timer0.h"
#include "sci.h"

static volatile Uint64 g_sys_tick = 0;
#pragma DATA_SECTION(g_sys_tick, "pre_roll_data");


#pragma CODE_SECTION(GetSysTick, "ramfuncs");
//Uint64  GetSysTick(void)
//{
//    return g_sys_tick;
//}
//测试
//Uint16 test_cnt = 0;

Uint64 GetSysTick(void)
{
    Uint64 tick_1, tick_2, tick_3;

    while (1) {

        tick_1 = g_sys_tick;
        tick_2 = g_sys_tick;
        tick_3 = g_sys_tick;

        // 如果三次读取结果中有任意两次相同，返回相同的值
        if (tick_1 == tick_2 && tick_1 == tick_3) {
            return tick_1;
        }
        // 如果三次值都不同，说明数据在读取期间被中断修改，继续重读
    }
}

#pragma CODE_SECTION(ElapsedTick, "ramfuncs");
//Uint64 ElapsedTick(Uint64 start_tick)
//{
//    return (g_sys_tick - start_tick);
//}
Uint64 ElapsedTick(Uint64 start_tick)
{
	Uint64 tick = GetSysTick();
    return (tick - start_tick);
}

static Uint16 cpu_timer0_isr_times = 0;
#pragma DATA_SECTION(cpu_timer0_isr_times, "pre_roll_data");
interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	g_sys_tick++;

	cpu_timer0_isr_times++;
	if (cpu_timer0_isr_times >= 10) {
	    cpu_timer0_isr_times = 0;
	    SciRevTimeout();
    }

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

void CPU_Timer0_Init()
{
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

   InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0 to interrupt every second:
// 100MHz CPU Freq, 1 second Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 90, 1000);
   StartCpuTimer0();

   IER |= M_INT1;

   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}


