#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "xintf.h"
#include "sci.h"
#include "cpu_timer0.h"
#include "cpu_timer1.h"
#include "data_handler.h"
#include "tele_dat.h"
#include "cmd.h"
#include "pps.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "pwm.h"
#include "GL_CJW_HE_5701.h"
#include "mot_cur_conf_cmd.h"
#include "boot_args.h"

#if BOARD_NAME == BOARD_GMS
#include "tlv2548.h"
#include "cycle_hall.h"
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_CX20)
#include "tlc2543.h"
#endif

#if BOARD_NAME == BOARD_GMS
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pyb);
        }
    }
}
#elif BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_4
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pyb);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nyb);
        }
    }
}
#elif BOARD_NAME == BOARD_DGM_2
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nya);
        }
    }
}
#endif

/**
 * \brief 通信接口复位任务
 */
void Commu_Interf_Reset_Task (void)
{
    if (ElapsedTick(g_RS422_DATA_Tick) >= TICK_5S) {
        g_RS422_DATA_Tick = GetSysTick();

#if BOARD_NAME == BOARD_GMS
        if (g_Axis_Conf.p_pyb->work_mode_old == WORK_MODE_RESET) {  /* 回零模式 */
            g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 3;            /* 回零异常 */
        }
#endif

        /* 重新初始化串口 */
        DriverSciInit(&SciaRegs, 115200, 1, 0, 7, 0, SCI_Rx_isr, SCI_TxFifo_isr);  //波特率115200，奇校验，8个字符长度，1个停止位。

        g_Tele_Dat.comm_rest_cnt++;

        g_RS422_relink_times++;
        if (g_RS422_relink_times == 5) {
            g_RS422_relink_times = 0;
            *gp_boot_arg_serial_fault_reset = 1; /* 串口异常复位 */
            while (1);  /* 如果通信复位5次串口还未恢复，直接卡死，让看门狗复位 */
        }
    }
}

#if BOARD_NAME == BOARD_GMS
/**
 * \brief 软件限位任务
 */
extern uint8_t  g_last_work_mode_set;           /* 上次工作模式 */
extern int32    g_last_loc_given;               /* 上次位置给定 */
extern int16    g_last_speed_given;             /* 上次速度给定 */

//extern Uint16 g_force_move_cmd_cnts;

void Soft_Lim_Task (void)
{
    float32 mot_angle = 0;
//    static uint8_t max_acc_temp = 2;

    /* 判断是否到达限位位置 */
    mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;

    /* 如果已经执行了强制停止，并且已经处于停止状态，则将最大加速度改写回去 */
    if ((g_force_hold_flag == 1) && ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 0) ||
									 ((g_Axis_Conf.p_pyb->cur_local >= g_Axis_Conf.p_pyb->soft_lim_n + 444) &&
									  (g_Axis_Conf.p_pyb->cur_local <= g_Axis_Conf.p_pyb->soft_lim_p - 444)))) {	/* 如果检测到当前位置在限位范围内0.2°以内，则退出强制模式 */
//    	g_Axis_Conf.p_pyb->max_acc = max_acc_temp;
    	g_force_hold_flag = 0;

        g_last_work_mode_set = 0;           /* 上次工作模式 */
        g_last_loc_given = 0;               /* 上次位置给定 */
        g_last_speed_given = 0;             /* 上次速度给定 */
    }

	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
    	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    	if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
        	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        	if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {	/* 读3次，防止读错 */

				g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 1;

	//			if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) && (g_force_hold_flag == 0) && (g_force_move_cmd_cnts < 5)) {		/* 如果电机处于运动状态，超出了限位位置，则强制停止 */
				if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) && (g_force_hold_flag == 0)) {		/* 如果电机处于运动状态，超出了限位位置，则强制停止 */
					g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
					g_Axis_Conf.p_pyb->loc_given     = 0;
					g_Axis_Conf.p_pyb->speed_given   = 1333;
					g_Axis_Conf.run_delay     		 = 0;
					g_Axis_Conf.new_task_req 		 = true;

					g_Axis_Conf.p_pyb->new_task_flag = true;	/* 必须加这行，否则连续发送相同指令之后（导致该标志为false），则不会走回限位范围之内，并且也不响应其他指令 */

	//				max_acc_temp = g_Axis_Conf.p_pyb->max_acc;	/* 缓存加速度 */
	//				g_Axis_Conf.p_pyb->max_acc = 200;

					g_force_hold_flag = 1;
				}
        	} else {
        		g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
        	}
    	} else {
    		g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
    	}
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
    }

	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
    	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
        	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
            if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {	/* 读3次，防止读错 */

				g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 1;

				//if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) && (g_force_hold_flag == 0) && (g_force_move_cmd_cnts < 5)) {		/* 如果电机处于运动状态，超出了限位位置，则强制停止 */
				if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) && (g_force_hold_flag == 0)) {		/* 如果电机处于运动状态，超出了限位位置，则强制停止 */
					g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
					g_Axis_Conf.p_pyb->loc_given     = 0;
					g_Axis_Conf.p_pyb->speed_given   = 0 - 1333;
					g_Axis_Conf.run_delay     		 = 0;
					g_Axis_Conf.new_task_req 		 = true;

					g_Axis_Conf.p_pyb->new_task_flag = true;	/* 必须加这行，否则连续发送相同指令之后（导致该标志为false），则不会走回限位范围之内，并且也不响应其他指令 */

	//				max_acc_temp = g_Axis_Conf.p_pyb->max_acc;	/* 缓存加速度 */
	//				g_Axis_Conf.p_pyb->max_acc = 200;

					g_force_hold_flag = 1;
				}
            } else {
            	g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
            }
        } else {
        	g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
    }
}

/**
 * \brief 更新当前位置
 */
#pragma CODE_SECTION(Updat_Cur_Local, "ramfuncs");
void Updat_Cur_Local (void)
{
	static Uint16 updata_flag = 0;

    /* 在主函数中更新当前位置，必须等角度传感器稳定之后才更新 */
    if (g_angle_adj_finish == 1) {
    	if (updata_flag == 0) {	/* 先更新一次 */
    		updata_flag = 1;
    		*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
    	} else {
    		if (((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) < 0 - 11111)) ||
    			((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) > 11111))) {	/* 5°跳变 */
        		if (((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) < 0 - 11111)) ||
        			((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) > 11111))) {
        			g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;				/* 反向更新 */
        		} else {
        			*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
        		}
    		} else {
    			*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
    		}
    	}
    }
}
#endif

#pragma CODE_SECTION(App_task, "ramfuncs");
void App_task(void)
{
//	Uint64 Test_Tick = GetSysTick();
//	Uint64 SPI_SendTick = GetSysTick();
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	Uint64 Cur_Local_Tick = GetSysTick();
#endif
//	Uint16 cur_local_flag = 0;

#if 0//测试代码（测试在不接收指令情况下使电机运动）
#if BOARD_NAME == BOARD_DGM_2
	g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_TRACE;
	g_Axis_Conf.p_pya->loc_given     = 0;
	g_Axis_Conf.p_pya->speed_given   = 1333;
	g_Axis_Conf.p_pya->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#elif BOARD_NAME == BOARD_DGM_4
	g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_TRACE;
	g_Axis_Conf.p_pya->loc_given     = 0;
	g_Axis_Conf.p_pya->speed_given   = 1333;
	g_Axis_Conf.p_pya->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;

    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
    g_Axis_Conf.p_pyb->loc_given     = 0;
    g_Axis_Conf.p_pyb->speed_given   = 1333;
    g_Axis_Conf.p_pyb->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#elif BOARD_NAME == BOARD_GMS
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
    g_Axis_Conf.p_pyb->loc_given     = 0;
    g_Axis_Conf.p_pyb->speed_given   = 1333;
    g_Axis_Conf.run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#endif
#endif
    while (1) {

        KickDog();  /* 喂狗，RAM运行，无任务，27.25us；MRAM运行，无任务，140us，开任务最大700us */

#if 0   /* 看门狗测试 */
        if (ElapsedTick(Test_Tick) >= TICK_10S) {
            while(1) {
                GpioDataRegs.GPBTOGGLE.bit.GPIOB1 = 1;
            }
        }
#endif

        RevDataRead();

        /* 引脚翻转测试 */
//        if (ElapsedTick(Test_Tick) >= TICK_10MS) {
//            GpioDataRegs.GPGTOGGLE.bit.GPIOG4 = 1;
//            GpioDataRegs.GPGTOGGLE.bit.GPIOG5 = 1;
//            GpioDataRegs.GPFTOGGLE.bit.GPIOF6 = 1;
//            GpioDataRegs.GPFTOGGLE.bit.GPIOF7 = 1;
//
//        	Test_Tick = GetSysTick();
//        }

        AB_Axis_Mov_Task();
        ADC_task();
        //测试
        Commu_Interf_Reset_Task();

#if BOARD_NAME == BOARD_GMS
        Angle_Senser_Task();
        Cycle_Hall_Task();
        Soft_Lim_Task();
        Updat_Cur_Local();
        CJW_HE_5701_sw_req_task();
#elif BOARD_NAME == BOARD_DGM_2
        if ((cur_local_flag == 0) && (ElapsedTick(Cur_Local_Tick) >= TICK_2S)) {
        	cur_local_flag = 1;
			g_Axis_Conf.p_pya->cur_local = Vol_To_Angle('A', '+', 1) * 25600 / 9;
			g_Axis_Conf.p_nya->cur_local = Vol_To_Angle('A', '-', 1) * 25600 / 9;
        }
#elif BOARD_NAME == BOARD_DGM_4
        if ((cur_local_flag == 0) && (ElapsedTick(Cur_Local_Tick) >= TICK_2S)) {
            cur_local_flag = 1;
            g_Axis_Conf.p_pya->cur_local = Vol_To_Angle('A', '+', 1) * 25600 / 9;
            g_Axis_Conf.p_nya->cur_local = Vol_To_Angle('A', '-', 1) * 25600 / 9;
            g_Axis_Conf.p_pyb->cur_local = Vol_To_Angle('B', '+', 1) * 10000 / 9;
            g_Axis_Conf.p_nyb->cur_local = Vol_To_Angle('B', '-', 1) * 10000 / 9;
        }
#endif

        Over_Cur_Task();
    }
}

/**
 * \brief 初始化变量
 */
void Init_Variable(void)
{
    InitTeleDat();
    Init_Axis_Conf();

    if ((*gp_boot_arg_angle_senser_adj < -360) || (*gp_boot_arg_angle_senser_adj > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pya_1 < -360) || (*gp_boot_arg_angle_senser_adj_pya_1 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pya_1 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pya_1 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pya_2 < -360) || (*gp_boot_arg_angle_senser_adj_pya_2 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pya_2 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pya_2 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nya_1 < -360) || (*gp_boot_arg_angle_senser_adj_nya_1 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nya_1 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nya_1 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nya_2 < -360) || (*gp_boot_arg_angle_senser_adj_nya_2 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nya_2 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nya_2 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pyb_3 < -360) || (*gp_boot_arg_angle_senser_adj_pyb_3 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pyb_3 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pyb_3 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nyb_3 < -360) || (*gp_boot_arg_angle_senser_adj_nyb_3 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nyb_3 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nyb_3 = 0;
    }
}

/**
 * \brief 禁中断
 */
static void __disable_irq(void)
{
    DINT;
    DRTM;

	IER = 0x0000;
	IFR = 0x0000;
}

/**
 * \brief 开中断
 */
static void __enable_irq(void)
{
    /* 遇到一个问题，程序启动之后，如果不发送串口命令，那么该进的中断都没有进（包括CPU定时器、外部中断1、PWM），去掉这行则可以进 */
    //IFR = 0x0000;   /* 使能中断前先清除所有中断标志。 */

    EINT;
    ERTM;
}

extern Uint16 IQmathLoadStart;
extern Uint16 IQmathLoadEnd;
extern Uint16 IQmathRunStart;

/**
 * \brief 初始化看门狗
 *
 * \note 看门狗计数值达到256溢出
 */

uint8_t g_last_reset_reason = 0;
#pragma DATA_SECTION(g_last_reset_reason, "pre_roll_data");

void reset_reasion_update()
{
    Uint32* p_wdflag = (void*)0;
    Uint16 __g_last_reset_is_wdt = 0;   /* 上次复位原因是否是因为看门狗 */

#if BOARD_NAME == BOARD_GMS
    if (*p_wdflag == 0x12345678) {
        __g_last_reset_is_wdt = 1;
    } else {
        *p_wdflag = 0x12345678;
    }

    if ((*gp_boot_arg_serial_fault_reset) == 1) {     /* 因为串口异常主动让看门狗复位 */
        g_last_reset_reason = LAST_RESET_REASON_SERIAL_ERR;
    } else if (__g_last_reset_is_wdt == 1) {
        g_last_reset_reason = LAST_RESET_REASON_WDT;
    } else {
        g_last_reset_reason = LAST_RESET_REASON_POWER_ON;
    }

    *gp_boot_arg_serial_fault_reset = 0;
#endif
}

void InitWDT(void)
{
    /**< \note 官方勘误手册上说，281x系列通过WDFLAG判断是否是看门狗复位不可靠，但2833x系列和2823x系列已经解决了这个问题，因此2812要用别的方法判断是否是看门狗复位 */
    EALLOW;
//    if (SysCtrlRegs.WDCR & 0x80) {  /* 上次复位是由看门狗引起的 */
//        SysCtrlRegs.WDCR |= 0x80;   /* 清除看门狗复位标志 */
//    }

    /* 看门狗时钟为OSCCLK / 512 / 64，大约279.6ms看门狗复位。 */
    SysCtrlRegs.WDCR = 0x2F;
    EDIS;
}

void main(void)
{
	/*初始化系统*/
	InitSysCtrl();

    /*初始化外设*/
    InitGpio();
#ifdef RUN_MRAM
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    MemCopy(&IQmathLoadStart, &IQmathLoadEnd, &IQmathRunStart);
#endif
	xintf_zone6and7_timing();

	/*关中断*/
	__disable_irq();

	/*初始化PIE*/
	InitPieCtrl();

	/*初始化PIE中断矢量表*/
	InitPieVectTable();

	/**
	 * \brief 需要放到初始化变量前面 ,通过此语句将boot rom空间从新映射到片内空间。
	 * 通过Zone7启动必须加这条，并且赋值为0
	 *
	 * 如果出现程序下不进去的情况，报错信息：“File Loader: Verification failed: Values at address 0x3FFFC0@Program do not match Please verify target memory and memory map.”
	 * 那么将程序改为RAM运行，并将此条注释掉，运行一遍，然后再将程序恢复回来即可。
	 * ↑后来发现这样做并没有什么卵用，如果实在下不进去就把校验关掉。
	 */
	EALLOW;
	XintfRegs.XINTCNF2.bit.MPNMC = 0;
	EDIS;

    (*gp_boot_arg_system_reset_times)++;
    reset_reasion_update();

	Init_Variable();   /* 初始化变量，不能放在使能中断之后 */

	DriverSciInit(&SciaRegs, 115200, 1, 0, 7, 0, SCI_Rx_isr, SCI_TxFifo_isr);  //波特率115200，奇校验，8个字符长度，1个停止位。

	CPU_Timer0_Init();

#if BOARD_NAME == BOARD_GMS
    CPU_Timer1_Init();
    Hall_Init();
	CJW_HE_5701_Init();
	TLV2548_Init();
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
	TLC2543_Init();
#endif

	PPS_Init();
	InitEV_PWM();
	__enable_irq();

#if BOARD_NAME == BOARD_DGM_2
	GpioDataRegs.GPFDAT.bit.GPIOF8 = 0;    /* 使能164245 */
    g_Axis_Conf.p_pya->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
    g_Axis_Conf.p_nya->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
    GpioDataRegs.GPFDAT.bit.GPIOF8 = 0;    /* 使能164245 */
    g_Axis_Conf.p_pya->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
    g_Axis_Conf.p_nya->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
    g_Axis_Conf.p_pyb->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
    g_Axis_Conf.p_nyb->fn_write_brake(0);                                       /* 解除刹车 ,相当于使能信号*/
#elif BOARD_NAME == BOARD_GMS
//    g_Axis_Conf.p_pyb->fn_write_brake(0);										/* 解除刹车 ,相当于使能信号*/
#endif

    g_RS422_DATA_Tick = GetSysTick();

    InitWDT();
    App_task();
}

//===========================================================================
// No more.
//===========================================================================
