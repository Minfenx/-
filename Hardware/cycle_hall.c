/*
 * cycle_hall.c
 *
 *  Created on: 2024年2月27日
 *      Author: Suzkfly
 */
#include  "project.h"
#if BOARD_NAME == BOARD_GMS
#include "cycle_hall.h"
#include "cmd.h"
#include "GL_CJW_HE_5701.h"
#include "tele_dat.h"
#include "boot_args.h"

/**< \brief 定义霍尔的脉冲数 */
float g_hall_angle = 0;
#pragma DATA_SECTION(g_hall_angle, "pre_roll_data");

/* 查询方式 */
#if 1

static Uint16 __g_old_level = 0;
#pragma DATA_SECTION(__g_old_level, "pre_roll_data");

static void __hall_gpio_init (void)
{
    EALLOW;
    GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1 = 0;     //普通IO
    GpioMuxRegs.GPEDIR.bit.GPIOE1 = 0;                  //输入
    GpioMuxRegs.GPEQUAL.bit.QUALPRD = 0x2;             //QUALPRD = SYSCLKOUT / 4 这里不能给太大，
    EDIS;
}

/**
 * \brief PPS初始化
 */
void Hall_Init (void)
{
    g_hall_angle = 0;
    __hall_gpio_init();

    __g_old_level = GpioDataRegs.GPEDAT.bit.GPIOE1;
}

/**
 * \brief 更新霍尔角度
 */
void Hall_angle_updat (void)
{
    Uint16 new_level = 0;

    new_level = GpioDataRegs.GPEDAT.bit.GPIOE1;

    /* 电平发生了变化 */
    if (__g_old_level != new_level) {
        if ((__g_old_level == 0) && (new_level == 1)) {     /* 从低到高，todo停止之后戳霍尔信号 */
            if (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) {
                g_Axis_Conf.p_pyb->hall_angle += 2.88;
                (*gp_boot_arg_last_pd_local_hall) = g_Axis_Conf.p_pyb->hall_angle;
            }
        } else {
            if (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) {
                g_Axis_Conf.p_pyb->hall_angle -= 2.88;
                (*gp_boot_arg_last_pd_local_hall) = g_Axis_Conf.p_pyb->hall_angle;
            }
        }

        __g_old_level = new_level;
    }
}

#else   /* 中断方式 */
/**
 * \brief 初始化XINT2的GPIO
 */
static void __InitXINT2Gpio (void)
{
	EALLOW;

//	GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//设置XINT1
	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//设置XINT2
//	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//设置XINT3

    EDIS;
}

#pragma CODE_SECTION(XINT2_isr, "ramfuncs");
interrupt void XINT2_isr (void)
{
//	if (g_Axis_Conf.p_pyb->is_task_running) {
//		__g_hall_cnt += g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->stage_cnt].mov_dir;
//	} else {
//		//故障
//	}
    __g_hall_cnt++;
    // 清除PIE应答寄存器的第1位，以响应组1内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

static void __EXTI2_Init (void)
{
	__InitXINT2Gpio();									//初始化IO
	/* 配置外部中断寄存器 */
	EALLOW;
	XIntruptRegs.XINT2CR.bit.POLARITY = 1;				//上升沿中断
	/* 外设中断使能 */
	XIntruptRegs.XINT2CR.bit.ENABLE   = 1;				//使能XINT2中断
	/* PIE中断使能 */
	PieVectTable.XINT2 = &XINT2_isr ;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;	//PIE组1 ,编号5 XINT2 ,使能PIE中断
	/* cpu INT1中断 */
	IER |= M_INT1;
    EDIS;
}

/**
 * \brief PPS初始化
 */
void Hall_Init (void)
{
	__g_hall_cnt = 0;
	__EXTI2_Init();
}

/**
 * \brief 得到HALL的脉冲个数
 */
int16 get_hall_cnt (void)
{
	return __g_hall_cnt;
}
#endif

/**
 * \brief 更新霍尔角度，并判断霍尔角度是否正常
 */
void Cycle_Hall_Task (void)
{
    float32 mot_angle = 0;

    if (g_angle_adj_finish == 1) {
        Hall_angle_updat();

        /* 如果电机角度与角度传感器的角度相差不大，但霍尔角度与两者相差很大，则认为霍尔异常 */
        mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;

        if (g_angle_senser_hw_normal == 1) {    /* 角度传感器硬件正常的情况下，将角度传感器的值列入参考范围 */
            if (((mot_angle * 1000 >= g_Tele_Dat.p_yb_axis_angle_res) && (mot_angle * 1000 - g_Tele_Dat.p_yb_axis_angle_res < 2000)) ||
                ((mot_angle * 1000 <  g_Tele_Dat.p_yb_axis_angle_res) && (g_Tele_Dat.p_yb_axis_angle_res - mot_angle * 1000 < 2000))) {     /* 角度传感器的值与电机角度值相差不超过2° */
                if (((g_Axis_Conf.p_pyb->hall_angle >  mot_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6)) ||
                    ((g_Axis_Conf.p_pyb->hall_angle <= mot_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6))) {    /* 霍尔角度与角度传感器的值相差不超过5°（改为6度，可以增加一个沿的容错） */
                    g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 0;
                } else {
                    g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 1;
                }
            }
        } else {
            if (((g_Axis_Conf.p_pyb->hall_angle >  mot_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6)) ||
                ((g_Axis_Conf.p_pyb->hall_angle <= mot_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6))) {
                g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 0;
            } else {
                g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 1;
            }
        }
    }
}
#endif
