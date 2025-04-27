/*
 * GL_CJW_HE_5701.c
 *
 *  Created on: 2023年12月13日
 *      Author: Suzkfly
 */
#include "GL_CJW_HE_5701.h"
#include "project.h"
#include "cmd.h"
#include "boot_args.h"
#include "cpu_timer1.h"
#include "cpu_timer0.h"
#include "tele_dat.h"
#include "pwm.h"
#include "sci.h"

#if (BOARD_NAME == BOARD_GMS)

Uint16 g_angle_adj_finish = 0;     /* 角度标定完成的标志，如果角度标定未完成，则不接收运动控制指令 */
Uint16 g_angle_senser_hw_normal = 1;     /* 角度传感器正常的标志，正常为1，不正常为0 */
Uint16 g_angle_senser_cmd_en  	   		= 1;	/* 角度传感器自动使能标志 */
Uint16 g_angle_senser_hw_en             = 1;	/* 硬件使能标志 */
Uint16 g_angle_senser_cmd_eff_flag  	= 0;	/* 角度传感器指令有效标志 */

#pragma DATA_SECTION(g_angle_adj_finish, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_hw_normal, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_cmd_en, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_hw_en, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_cmd_eff_flag, "pre_roll_data");


/**
 * \brief 角位移传感器时钟线高
 */
#pragma CODE_SECTION(CJW_HE_5701_SET_CLK, "ramfuncs");
void CJW_HE_5701_SET_CLK (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    GpioDataRegs.GPASET.bit.GPIOA11 = 1;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    GpioDataRegs.GPFSET.bit.GPIOF8 = 1;
#endif
}

/**
 * \brief 角位移传感器时钟线低
 */
#pragma CODE_SECTION(CJW_HE_5701_CLEAR_CLK, "ramfuncs");
void CJW_HE_5701_CLEAR_CLK (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    GpioDataRegs.GPACLEAR.bit.GPIOA11 = 1;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    GpioDataRegs.GPFCLEAR.bit.GPIOF8 = 1;
#endif
}

/**
 * \brief 角位移传感器读取一个位
 */
#pragma CODE_SECTION(CJW_HE_5701_Read_Bit, "ramfuncs");
Uint16 CJW_HE_5701_Read_Bit (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    return GpioDataRegs.GPADAT.bit.GPIOA10;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    return GpioDataRegs.GPFDAT.bit.GPIOF13;
#endif
}

#if 0	/* 读取角度的函数放到CPUTimer1中 */

/**
 * \brief 角位移传感器读取数据
 *
 * \param[out] p_value：读取成功后填充值
 *
 * \retval 成功返回0，失败返回-1
 */
int16 CJW_HE_5701_Read_Data (Uint16* p_value)
{
	Uint16 i = 0;
	Uint16 temp = 0;

	//PieCtrlRegs.PIEIER2.bit.INTx6 = 0;

	/* 读完16个位后还要发一个下降沿和上升沿 */
	__CJW_HE_5701_CLEAR_CLK();	/* 第一个下降沿启动转换 */
	DELAY_US(1);
	__CJW_HE_5701_SET_CLK();	/* 第一个上升沿开始送数据，但是读数据要在第二个下降沿 */
	DELAY_US(1);

	for (i = 0; i < 16; i++) {
		temp <<= 1;
		__CJW_HE_5701_CLEAR_CLK();
		temp |= __CJW_HE_5701_Read_Bit();		/* 高位先传 */
		DELAY_US(1);
		__CJW_HE_5701_SET_CLK();
		DELAY_US(1);
	}
	/* 第17个上升沿之后数据线变为高电平，此位为错误位，如果不变高，意味着数据错误 */
	if (__CJW_HE_5701_Read_Bit() == 0) {	/* 如果发现数据线没有变为低电平，则认为传输有误 */
		//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
		return -1;
	}

	__CJW_HE_5701_CLEAR_CLK();
	DELAY_US(1);
	__CJW_HE_5701_SET_CLK();	/* 第18个上升沿之后数据变为低电平，如果不变低电平说明传输有误 */
	DELAY_US(1);

	/* 最后一个上升沿发出之后，数据线会保持16~24us的低电平 */
	if (__CJW_HE_5701_Read_Bit() == 1) {	/* 如果发现数据线没有变为低电平，则认为传输有误 */
		//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
		return -1;
	}

	//GpioDataRegs.GPADAT.bit.GPIOA9 = 0;
	/* i = 30，测试值为41us（主频90MHz，不开优化） */
	for (i = 30; i > 0; i--) {
		if (__CJW_HE_5701_Read_Bit() == 1) {
			break;
		}
	}
	//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
	//GpioDataRegs.GPADAT.bit.GPIOA9 = 1;
	if (i > 0) {	/* 传输成功 */
		*p_value = temp;
		return 0;
	} else {
		return -1;
	}
}
#endif

/**
 * \brief 角度传感器电源供电
 */
void CJW_HE_5701_Power_On (void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIOB14 = 1;
    g_Tele_Dat.comm_stat.bit.angle_senser_sw = ANGLE_SENSOR_SW_ON;
}

/**
 * \brief 角度传感器电源供电
 */
void CJW_HE_5701_Power_Off (void)
{
    GpioDataRegs.GPBSET.bit.GPIOB14 = 1;
    g_Tele_Dat.comm_stat.bit.angle_senser_sw = ANGLE_SENSOR_SW_OFF;
}

/**
 * \brief 角位移传感器初始化
 */
void CJW_HE_5701_Init (void)
{
    EALLOW;

    /* 角位移传感器IO初始化 */

#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    /* 数据线 */
    GpioMuxRegs.GPAMUX.bit.CAP3QI1_GPIOA10 = 0;		//普通IO
    GpioMuxRegs.GPADIR.bit.GPIOA10 = 0;				//输入

    /* 时钟线 */
    GpioMuxRegs.GPAMUX.bit.TDIRA_GPIOA11   = 0;		//普通IO
    GpioMuxRegs.GPADIR.bit.GPIOA11 = 1;				//输出
    GpioDataRegs.GPADAT.bit.GPIOA11 = 1;			//初始输出1
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    /* 数据线 */
    GpioMuxRegs.GPFMUX.bit.MDRA_GPIOF13 = 0;     //普通IO
    GpioMuxRegs.GPFDIR.bit.GPIOF13 = 0;             //输入

    /* 时钟线 */
    GpioMuxRegs.GPFMUX.bit.MCLKXA_GPIOF8   = 0;     //普通IO
    GpioMuxRegs.GPFDIR.bit.GPIOF8 = 1;             //输出
    GpioDataRegs.GPFDAT.bit.GPIOF8 = 1;            //初始输出1
#endif

    /* 电源使能引脚 */
    GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14   = 0;    //普通IO
    GpioMuxRegs.GPBDIR.bit.GPIOB14 = 1;             //输出

    /* 注意电源使能那一瞬间，数据线会短暂被拉低，拉低时间大概为4.4ms */
    CJW_HE_5701_Power_On();

    EDIS;
}




/**
 * \brief 角度传感器采集到的角度值换算为位置值
 */
int32 CJW_HE_5701_to_cur_local (Uint16 CJW_HE_5701_value)
{
    float angle_f = 0;

    angle_f = (float)CJW_HE_5701_value * 360 / 65535;
    angle_f = angle_f - 180 + *gp_boot_arg_angle_senser_adj;
    if (angle_f > 180) {
        angle_f -= 360;
    } else if (angle_f < -180) {
        angle_f += 360;
    }

    return angle_f / g_Axis_Conf.p_pyb->loc_to_angle_factor;
}

/**
 * \brief 判断这组数据是否超限
 *
 * \parma[in] p_data：要判断的数据
 * \parma[in] cnt： 判断的数据数量
 * \parma[in] lim：限制
 *
 * \retval 超限返回1，不超限返回0
 */
int16 is_over_lim (Uint16* p_data, Uint16 cnt, Uint16 lim)
{
    Uint16 i;
    Uint16 min, max;

    min = p_data[0];
    max = p_data[0];
    for (i = 1; i < cnt; i++) {
        if (p_data[i] < min) {
            min = p_data[i];
        } else if (p_data[i] > max) {
            max = p_data[i];
        }
    }

    if (max - min > lim) {
        return 1;
    }
    return 0;
}

/**
 * \brief 根据实际情况，需要扩大软件限位位置，或者报故障
 */
static void __adj_soft_lim (void)
{
    if (g_Axis_Conf.p_pyb->cur_local >= g_Axis_Conf.p_pyb->soft_lim_p) {
        //g_Axis_Conf.p_pyb->soft_lim_p = g_Axis_Conf.p_pyb->cur_local;
        //*gp_boot_arg_soft_lim_p = g_Axis_Conf.p_pyb->soft_lim_p;
        g_Tele_Dat.soft_lim_err_stat |= SOFT_LIM_ERR_P;     /* 超出正限位 */
    }

    if (g_Axis_Conf.p_pyb->cur_local <= g_Axis_Conf.p_pyb->soft_lim_n) {
        //g_Axis_Conf.p_pyb->soft_lim_n = g_Axis_Conf.p_pyb->cur_local;
        //*gp_boot_arg_soft_lim_n = g_Axis_Conf.p_pyb->soft_lim_n;
        g_Tele_Dat.soft_lim_err_stat |= SOFT_LIM_ERR_N;     /* 超出负限位 */
    }
}

static Uint64 GL_CJW_HE_5701_Tick = 0;
static Uint16 angle_init_read_times = 0;       /* 标定角度时读取的角度值次数 */
static Uint16 angle_temp[10] = { 0 };
static Uint16 invalid_data_cnt = 0;     /* 无效数据计数 */
static Uint16 read_enable = 1;          /* 读取使能 */
static Uint16 angle_senser_re_power_supply_times = 0;   /* 角度传感器重复加电的次数 */
#pragma DATA_SECTION(GL_CJW_HE_5701_Tick, "pre_roll_data");
#pragma DATA_SECTION(angle_init_read_times, "pre_roll_data");
#pragma DATA_SECTION(angle_temp, "pre_roll_data");
#pragma DATA_SECTION(invalid_data_cnt, "pre_roll_data");
#pragma DATA_SECTION(read_enable, "pre_roll_data");
#pragma DATA_SECTION(angle_senser_re_power_supply_times, "pre_roll_data");


/**
 * \brief 角度传感器任务
 */
void Angle_Senser_Task (void)
{
    int32  angle_senser_loc = 0;

    float32 mot_angle = 0;
    int32 angle_temp1 = 0;

    if ((g_angle_senser_hw_normal == 1) && (g_angle_senser_hw_en == 1)) {

        /* 准备获取角位移传感器的值 */
        if (read_enable == 1) {
            if (ElapsedTick(GL_CJW_HE_5701_Tick) >= TICK_200MS) {
				GL_CJW_HE_5701_Tick = GetSysTick();	//直接跳过

            	if ((sSendData.Index == 0) && (sSendData.Length == 0)) {	/* 只有串口没在发送的时候才能开始测角 */
					g_read_angle_ready = 1;
            	}
            }
        } else {
            if (ElapsedTick(GL_CJW_HE_5701_Tick) >= TICK_3S) {
                GL_CJW_HE_5701_Tick = GetSysTick();
                read_enable = 1;
                CJW_HE_5701_Power_On();
            }
        }

        /* 角度传感器的采集是PWM中开始的，如果进入了过温保护，则需要在这里开启角度采集 */
        if (g_pdpinta_pyb_flag == 1) {
            if (g_read_angle_ready) {
                g_read_angle_ready = 0;
                g_angle_is_reading = 1;
                StartCpuTimer1();           /* 开启角度采集 */
            }
        }

        if (g_CJW_HE_5701_read_finish == 1) {       	/* 一次读取结束 */
            g_CJW_HE_5701_read_finish = 0;

            if (g_angle_senser_cmd_en == 0) {	/* 关闭传感器 */
            	CJW_HE_5701_Power_Off();
            	g_angle_senser_hw_en = 0;
            }

            if (g_CJW_HE_5701_read_acitve == 1) {   /* 数据有效 */
                invalid_data_cnt = 0;
                angle_temp1 = g_CJW_HE_5701_angle;

                /* 用获取到的角度值 标定电机角度值和霍尔角度值 */
                if (g_angle_adj_finish == 0) {      /* 尚未完成角度标定 */
                    angle_temp[angle_init_read_times++] = angle_temp1;
#if 0       /* 模拟3个数相差太大的情况 */
                    angle_temp[0] = 10000;
                    angle_temp[1] = 10000;
                    angle_temp[2] = 10100;
#elif 0     /* 模拟角度传感器的值数相差不大，但是与保存的角度值相差较大 */
                    Uint16 i;
                    for (i = 0; i < 10; i++) {
                        angle_temp[i] = 10000;
                    }
                    angle_temp1 = 10000;
                    *gp_boot_arg_last_pd_local_motor = 0;
#elif 0     /* 模拟角度传感器的值数相差较大，但是数值勉强能用/不能用 */
                    Uint16 i;
                    for (i = 0; i < 10; i++) {
                        angle_temp[i] = 10000;
                    }
                    angle_temp1 = 10000;
                    angle_temp[0] = 10100;
                    *gp_boot_arg_last_pd_local_motor = 0;
#endif

                    /* 采集到的3个数偏差不能太大，否则重新采集 */
                    if (angle_init_read_times == 3) {
                        if (is_over_lim(angle_temp, 3, 5) == 0) {   /* 连续3个数相差不超过5 */
                        //if (is_over_lim(angle_temp, 3, 5000) == 0) {   /* 陪测软件，扩大阈值 */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            if (((angle_senser_loc >= (*gp_boot_arg_last_pd_local_motor)) && (angle_senser_loc - (*gp_boot_arg_last_pd_local_motor) < 4444)) ||     /* 与上次断电位置进行比较 ，不超过2°认为正常 */
                                ((angle_senser_loc <  (*gp_boot_arg_last_pd_local_motor)) && ((*gp_boot_arg_last_pd_local_motor) - angle_senser_loc < 4444))) {
                                g_angle_adj_finish = 1;
                                g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* 角度传感器正常 */
                                g_Axis_Conf.p_pyb->cur_local = angle_senser_loc;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                            }
                        } else if (is_over_lim(angle_temp, 3, 91) == 1) {
                            angle_init_read_times = 0;
                            read_enable = 0;
                            CJW_HE_5701_Power_Off();
                            angle_senser_re_power_supply_times++;
                            if (angle_senser_re_power_supply_times == 3) {
                                g_angle_senser_hw_normal = 0;
                                g_angle_adj_finish = 1;
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                            }
                        }
                    } else if (angle_init_read_times == 10) {
                        if (is_over_lim(angle_temp, 10, 5) == 0) {   /* 连续10个数相差不超过5 */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            g_angle_adj_finish = 1;
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* 角度传感器正常 */
                            g_Axis_Conf.p_pyb->cur_local = angle_senser_loc;
                            __adj_soft_lim();
                            g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                        } else if (is_over_lim(angle_temp, 10, 91) == 0) {          /* 连续10个数相差超过5，但是不超过91 */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            g_angle_adj_finish = 1;
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 1;    /* 角度传感器异常 */
                            g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                            __adj_soft_lim();
                            g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                        } else {
                            read_enable = 0;
                            CJW_HE_5701_Power_Off();
                            angle_senser_re_power_supply_times++;
                            angle_init_read_times = 0;
                            if (angle_senser_re_power_supply_times == 3) {
                                g_angle_senser_hw_normal = 0;
                                g_angle_adj_finish = 1;
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                            }
                        }
                    }
                } else {   /* 已经完成角度标定，判断角度传感器是否异常 */
                    angle_senser_re_power_supply_times = 0;

                    g_Tele_Dat.p_yb_axis_angle_res = CJW_HE_5701_to_cur_local(g_CJW_HE_5701_angle) * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                    mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                    if (((mot_angle >= g_Axis_Conf.p_pyb->hall_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6)) ||
                        ((mot_angle <  g_Axis_Conf.p_pyb->hall_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6))) {    /* 电机角度值与霍尔角度值相差不大于5°（改为6°可以增加一个沿的容错） */

                        if (((g_Tele_Dat.p_yb_axis_angle_res >   mot_angle * 1000) && (g_Tele_Dat.p_yb_axis_angle_res - mot_angle * 1000 < 2000)) ||
                            ((g_Tele_Dat.p_yb_axis_angle_res <=  mot_angle * 1000) && (mot_angle * 1000 - g_Tele_Dat.p_yb_axis_angle_res < 2000))) {    /* 电机角度值与角度传感器的值相差不大于2° */
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;
                        } else {
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 1;
                        }
                    }
                }
            } else {
                invalid_data_cnt++;

                if (g_angle_adj_finish == 0) {
                    angle_init_read_times = 0;
                }

                if (invalid_data_cnt >= 5) {   /* 连续5次读取失败 */
                    invalid_data_cnt = 0;
                    read_enable = 0;
                    CJW_HE_5701_Power_Off();

                    angle_senser_re_power_supply_times++;

                    if (angle_senser_re_power_supply_times == 3) {
                        angle_senser_re_power_supply_times = 0;
                        g_angle_senser_hw_normal = 0;
                        __adj_soft_lim();

                        if (g_angle_adj_finish == 0) {
                            /* 使用保存的电机角度值 */
                            if (((*gp_boot_arg_last_pd_local_motor) * g_Axis_Conf.p_pyb->loc_to_angle_factor - (*gp_boot_arg_last_pd_local_hall) <  6) &&
                                ((*gp_boot_arg_last_pd_local_motor) * g_Axis_Conf.p_pyb->loc_to_angle_factor - (*gp_boot_arg_last_pd_local_hall) > -6)) {
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            } else if ((*gp_boot_arg_last_pd_local_hall > 0 - 180) && ((*gp_boot_arg_last_pd_local_hall < 180))) {    /* 使用保存的霍尔角度值 */
                                g_Axis_Conf.p_pyb->hall_angle = *gp_boot_arg_last_pd_local_hall;
                                g_Axis_Conf.p_pyb->cur_local = g_Axis_Conf.p_pyb->hall_angle / g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            } else {	/* 还是使用电机角度值 */
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            }
                            //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                        }

                        g_angle_adj_finish = 1;
                    }
                }
            }
        }
    } else {	/* 不采角度传感器时角度传感器值跟随电机角度值 */
    	g_Tele_Dat.p_yb_axis_angle_res = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
    	g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* 角度传感器正常 */
    	GL_CJW_HE_5701_Tick = GetSysTick();
    }
}
#endif
