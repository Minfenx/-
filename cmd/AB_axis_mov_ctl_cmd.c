/*
 * AB_axis_mov_ctl_cmd.c
 *
 *  Created on: 2023年5月29日
 *      Author: mj
 */
#include "AB_axis_mov_ctl_cmd.h"
#include "cmd.h"
#include "project.h"
#include <math.h>
#include <string.h>
#include "cpu_timer0.h"
#include "tele_dat.h"
#include "pps.h"

#if BOARD_NAME == BOARD_GMS
uint8_t  g_last_work_mode_set = 0;          /* 上次工作模式 */
int32    g_last_loc_given     = 0;          /* 上次位置给定 */
int16    g_last_speed_given   = 0;          /* 上次速度给定 */
#elif (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_DGM_2)
uint8_t  g_last_work_mode_set[AXIS_CNT] = { 0 };          /* 上次工作模式 */
int32    g_last_loc_given[AXIS_CNT]     = { 0 };          /* 上次位置给定 */
int16    g_last_speed_given[AXIS_CNT]   = { 0 };          /* 上次速度给定 */
#endif

//Uint16 g_force_move_cmd_cnts = 0;

/**
 * \brief 计算占空比
 *
 * \param[in] k：第k个数，(0 ≤ k ≤ N)
 * \param[in] N_4：1/4个周期内包含的点数
 * \note sin的输入值为π/2 * (k/N)
 */
#if 1
static float32 __duty(Uint16 k, Uint16 N_4) {
    const _iq20 iq20_PI = _IQ20(M_PI);
    // 直接使用 IQ20 格式进行乘除运算
    _iq20 input = _IQ20div(_IQ20mpy(_IQ20(k), iq20_PI), _IQ20(2 * N_4));
    _iq20 sin_out = _IQ20sin(input);
    return _IQ20toF(sin_out);
}
#else
static float32 __duty (Uint16 k, Uint16 N_4)
{
    float32 dy;

    dy = sin(k * M_PI / (2 * N_4));

    return dy;
}
#endif

/**
 * \brief       得到整1/4周期内的所有微步的占空比值
 *
 * \param[in] N：1/4个周期内细分数
 * \param[out] p_duty：计算后的占空比数值
 *
 * \note sin(2*pi*t/T)的图像
 */
void get_all_duty (Uint16 N_4, float32* p_duty)
{
    Uint16 i = 0;
    //计算N_4+1个值
    for (i = 0; i <= N_4; i++) {
        *p_duty++ = __duty(i, N_4);
    }
}

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
	single_axis_conf_t  axis_temp[4];
	single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
	uint8_t i, j;

    /* 接收AB轴运动控制指令不需要判断任务是否正在进行 */

	/* 判断数据的合法性 */
	j = 6;
	for (i = 0; i < 4; i++) {
		axis_temp[i].work_mode_set  = pData[j++];
		axis_temp[i].loc_given      = ((Uint32)pData[j] << 24) | ((Uint32)pData[j+1] << 16) | (pData[j+2] << 8) | pData[j+3];
		j += 4;
		axis_temp[i].speed_given    = (pData[j] << 8) | pData[j+1];
		j += 2;
		
		if (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
			if ((axis_temp[i].loc_given < p_axis->min_loc) || (axis_temp[i].loc_given > p_axis->max_loc)) {
				return -1;
			}
		} 
		if (((axis_temp[i].work_mode_set == WORK_MODE_TRACE) && (i < 2)) || (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE)) {
			if ((axis_temp[i].speed_given < p_axis->min_speed) || (axis_temp[i].speed_given > p_axis->max_speed) || (axis_temp[i].speed_given == 0)) {	/* 速度给定不能为0 */
				return -1;
			}
		}

		if ((i >= 2) && (axis_temp[i].work_mode_old == WORK_MODE_LAUNCH)) {	/* B轴展开模式下不接收新任务指令 */
			return -1;
		}

		/* B轴捕获模式不能超过软件限位位置 */
		if ((i >= 2) && axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
			if (((axis_temp[i].loc_given > 0) && (axis_temp[i].loc_given >  p_axis->soft_lim)) ||
				((axis_temp[i].loc_given < 0) && (axis_temp[i].loc_given < -p_axis->soft_lim))) {
				return -1;
			}
		}
		p_axis = p_axis->p_next_axis;
	}
	
	p_axis = g_Axis_Conf.p_pya;
	for (i = 0; i < 4; i++) {
		p_axis->work_mode_set = axis_temp[i].work_mode_set;
		p_axis->loc_given     = axis_temp[i].loc_given;
		p_axis->speed_given   = axis_temp[i].speed_given;
		p_axis = p_axis->p_next_axis;
	}

    return 0;
}
#elif (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
    single_axis_conf_t  axis_temp[AXIS_CNT];
    single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
    uint8_t i, j;

    /* 接收AB轴运动控制指令不需要判断任务是否正在进行 */

    /* 判断数据的合法性 */
    j = 7;
    for (i = 0; i < AXIS_CNT; i++) {
        axis_temp[i].work_mode_set  = pData[j++];
        axis_temp[i].loc_given      = ((Uint32)pData[j] << 24) | ((Uint32)pData[j+1] << 16) | (pData[j+2] << 8) | pData[j+3];
        j += 4;
        axis_temp[i].speed_given    = (pData[j] << 8) | pData[j+1];
        j += 2;

        if (((axis_temp[i].work_mode_set == WORK_MODE_TRACE) && (i < 2)) || (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE)) {
            if ((axis_temp[i].speed_given < p_axis->min_speed) || (axis_temp[i].speed_given > p_axis->max_speed) || (axis_temp[i].speed_given == 0)) {  /* 速度给定不能为0 */
                return -1;
            }
        }

        if ((i >= 2) && (axis_temp[i].work_mode_old == WORK_MODE_LAUNCH)) { /* B轴展开模式下不接收新任务指令 */
            return -1;
        }

        /* 如果角度值不超过45°则不接收展开指令 */
        if (((i == 2) && (g_Axis_Conf.p_pyb->cur_local < 50000)) ||
            ((i == 3) && (g_Axis_Conf.p_nyb->cur_local < 50000))) {
        	if (axis_temp[i].work_mode_set == WORK_MODE_LAUNCH) {
        		return -1;
        	}
        }

        /* B轴捕获模式不能超过软件限位位置 */
        if ((i >= 2) && axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
            if (((axis_temp[i].loc_given > 0) && (axis_temp[i].loc_given >  p_axis->soft_lim)) ||
                ((axis_temp[i].loc_given < 0) && (axis_temp[i].loc_given < -p_axis->soft_lim))) {
                return -1;
            }
        }

        p_axis = p_axis->p_next_axis;
    }

    g_Axis_Conf.cmd_pps      = g_pps;
    g_Axis_Conf.run_delay	 = pData[j];

    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < AXIS_CNT; i++) {
        if ((g_last_work_mode_set[i] == axis_temp[i].work_mode_set) &&
            (g_last_loc_given[i]     == axis_temp[i].loc_given) &&
            (g_last_speed_given[i]   == axis_temp[i].speed_given)) {
            p_axis->new_task_flag = 0;
        } else {
            g_last_work_mode_set[i] = axis_temp[i].work_mode_set;
            g_last_loc_given[i]     = axis_temp[i].loc_given;
            g_last_speed_given[i]   = axis_temp[i].speed_given;
            p_axis->new_task_flag = 1;

            p_axis->work_mode_set = axis_temp[i].work_mode_set;
            p_axis->loc_given     = axis_temp[i].loc_given;
            p_axis->speed_given   = axis_temp[i].speed_given;
        }

        if ((p_axis->work_mode_set == WORK_MODE_KEEP) && (p_axis->work_mode_old == WORK_MODE_STANDBY)) {
        	p_axis->work_mode_old = WORK_MODE_KEEP;
        }

        if ((p_axis->work_mode_set == WORK_MODE_STANDBY) && (p_axis->work_mode_old == WORK_MODE_KEEP)) {
        	p_axis->work_mode_old = WORK_MODE_STANDBY;
        }

#if BOARD_NAME == BOARD_DGM_2
        if (!strncmp(p_axis->name, "PYA", 3)){
            g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }else if(!strncmp(p_axis->name, "NYA", 3)){
            g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }
#elif BOARD_NAME == BOARD_DGM_4
        if (!strncmp(p_axis->name, "PYA", 3)){
            g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }else if(!strncmp(p_axis->name, "NYA", 3)){
            g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }else if(!strncmp(p_axis->name, "PYB", 3)){
            g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }else if(!strncmp(p_axis->name, "NYB", 3)){
            g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
        }
#endif

        p_axis = p_axis->p_next_axis;
    }

    return 0;
}
#elif BOARD_NAME == BOARD_GMS
#include "GL_CJW_HE_5701.h"
#include "pwm.h"

uint8_t g_angle_sensor_task_req = 0;	/* 角度传感器任务请求，为0表示无任无，为1表示有任务 */
#pragma DATA_SECTION(g_angle_sensor_task_req, "pre_roll_data");

uint8_t g_angle_sensor_sw_req = 0;		/* 角度传感器开关，为0表示关，为1表示开 */
#pragma DATA_SECTION(g_angle_sensor_sw_req, "pre_roll_data");

static Uint64 __g_CJW_HE_5701_sw_Tick = 0;
#pragma DATA_SECTION(__g_CJW_HE_5701_sw_Tick, "pre_roll_data");

void CJW_HE_5701_sw_req_task (void)
{
	if ((g_angle_sensor_task_req == 1) && (ElapsedTick(__g_CJW_HE_5701_sw_Tick) >= TICK_30S)) {
		g_angle_sensor_task_req = 0;

		if ((g_angle_sensor_sw_req == 0) && (g_Tele_Dat.comm_stat.bit.angle_senser_sw == ANGLE_SENSOR_SW_ON)) {	/* 关 */
			g_angle_senser_cmd_en = 0;
			__g_CJW_HE_5701_sw_Tick = GetSysTick();
		} else if ((g_angle_sensor_sw_req == 1)  && (g_Tele_Dat.comm_stat.bit.angle_senser_sw == ANGLE_SENSOR_SW_OFF)) {	/* 开 */
        	g_angle_senser_cmd_en = 1;
            g_angle_senser_hw_en = 1;
            if (g_angle_senser_hw_normal == 1) { //硬件标志不赋值，但要判断
                CJW_HE_5701_Power_On();
            }
            __g_CJW_HE_5701_sw_Tick = GetSysTick();
		}
	}
}



/* 用于判断接收到相同的捕获模式需要有个时间间隔 */
uint8_t g_force_hold_flag = 0;		/* 强制停止标志 */
#pragma DATA_SECTION(g_force_hold_flag, "pre_roll_data");
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
	single_axis_conf_t  axis_temp;
	single_axis_conf_t* p_axis = g_Axis_Conf.p_pyb;

	if (g_force_hold_flag == 1) {
		return -1;
	}

//	if (g_Tele_Dat.soft_lim_err_stat != 0) {    /* 超出限位位置不处理运动控制指令 */
//	    return -1;
//	}

    /* 接收AB轴运动控制指令不需要判断任务是否正在进行，但是角度传感器的状态必须确定 */
	if ((g_angle_adj_finish == 1) && (g_pdpinta_pyb_flag == 0)) {
        /* 判断数据的合法性 */
        axis_temp.work_mode_set  = pData[6];
        axis_temp.loc_given      = ((Uint32)pData[7] << 24) | ((Uint32)pData[8] << 16) | (pData[9] << 8) | pData[10];
        axis_temp.speed_given    = (pData[11] << 8) | pData[12];
        g_Axis_Conf.run_delay	 = pData[13];

        /* 捕获模式不能超过限位位置 */
        if (axis_temp.work_mode_set == WORK_MODE_CAPTURE) {
            if ((axis_temp.loc_given < p_axis->soft_lim_n) || (axis_temp.loc_given > p_axis->soft_lim_p)) {
//                g_force_move_cmd_cnts++;
//                if (g_force_move_cmd_cnts < 5) {
//                    p_axis->new_task_flag = 0;
//                    return -1;
//                }
            	return -1;
            }
        }

        if ((axis_temp.work_mode_set == WORK_MODE_TRACE) || (axis_temp.work_mode_set == WORK_MODE_CAPTURE)) {
            if ((axis_temp.speed_given < p_axis->min_speed) || (axis_temp.speed_given > p_axis->max_speed) || (axis_temp.speed_given == 0)) {	/* 速度给定不能为0 */
				p_axis->new_task_flag = 0;
				return -1;
            }
        }

        /* 跟踪模式下，如果已经超过了限位位置，则不能接收往限位方向运动的指令 */
        if (axis_temp.work_mode_set == WORK_MODE_TRACE) {
            if (((p_axis->cur_local >= p_axis->soft_lim_p) && (axis_temp.speed_given > 0)) ||
                ((p_axis->cur_local <= p_axis->soft_lim_n) && (axis_temp.speed_given < 0))) {

				p_axis->new_task_flag = 0;
				return -1;
            }
        }

        /* 如果新设定的模式和数据与旧的模式与数据相同，那么就直接跳过 */
        if ((g_last_work_mode_set == axis_temp.work_mode_set) &&
            (g_last_loc_given     == axis_temp.loc_given) &&
            (g_last_speed_given   == axis_temp.speed_given)) {

            p_axis->new_task_flag = 0;
        } else {
            g_last_work_mode_set = axis_temp.work_mode_set;
            g_last_loc_given     = axis_temp.loc_given;
            g_last_speed_given   = axis_temp.speed_given;
            p_axis->new_task_flag = 1;

            p_axis->work_mode_set = axis_temp.work_mode_set;
            p_axis->loc_given     = axis_temp.loc_given;
            p_axis->speed_given   = axis_temp.speed_given;
            g_Axis_Conf.cmd_pps   = g_pps;

//            if ((!strncmp(p_axis->name, "PYB", 3)) && (axis_temp.work_mode_set != WORK_MODE_RESET)) {
//                  g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 0;    /* 未收到回零指令 */
//            }
        }

        /* 跟踪模式下自动关闭角度传感器 */
        if ((g_angle_senser_cmd_eff_flag == 0) && (p_axis->new_task_flag == 1)) {
            if (axis_temp.work_mode_set == WORK_MODE_TRACE) {
                g_angle_sensor_task_req = 1;
                g_angle_sensor_sw_req = 0;
            } else if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) || (axis_temp.work_mode_set == WORK_MODE_RESET)) {    /* 捕获模式或回零模式下自动开启角度传感器 */
                g_angle_sensor_task_req = 1;
                g_angle_sensor_sw_req = 1;
            }
        }

        return 0;
	}
    return -1;
}
#endif

// 快速左移函数，处理int32_t类型整数，保留最高位，避免溢出
// num: 要左移的整数
// shiftAmount: 左移的位数
// 返回左移后的结果
int32 fastLeftShift(int32 num, int shiftAmount) {
    int32 signBit = num & 0x80000000;                   // 保存最高位
//    if (shiftAmount >= 32 || shiftAmount <= 0) {
//        return 0;                                       // 可能溢出，返回0或者其他处理方式
//    }
    num =  (num << shiftAmount) & 0x7FFFFFFF;
    return num | signBit;
}

/**
 * \brief （旧）计算下个脉冲间隔和速度，V=V0+at t=k/v k=0.9/(160*16)，用这种方法计算的V是由V0得到的，下个V又是由前一个V得到的，经实际验证，误差非常大，而且误差会累计，不适用
 * 		      应该用x = v0t + (1/2)at^2 每次x的变化是固定的，只需要代入v0即可算出t
 * 		     一个x=0.0003515625°，浮点数真实值为0.000325，有截断误差，乘以1000倍以后为0.351562（少一个5）
 * 		     得到t = (sqrt(v0^2 + 2ax) - v0) / a
 *
 *
 * 		     （新）计算下一个微步中的50us脉冲数量。一整步为1/4个正弦周期，将其进行32细分，每个小段相当于1/32步，对应角度为x=0.05625°=1.8/32	（未减速前）
 * \note 为了节省计算量，将上次的计算结果t2用static类型保存，实际使用的时候elapsed_pulse_cnt必须只比上一次多1
 * \retval 返回下个微步内 , PWM波的个数 , 目前设置的pwm周期固定50us , 相当于最小时间颗粒度
 */
/* 函数耗时29.5us */

float real_v = 0;
#pragma DATA_SECTION(real_v, "pre_roll_data");

#pragma CODE_SECTION(get_next_segment_pulse_cnt, "ramfuncs");
Uint32 get_next_segment_pulse_cnt (single_axis_conf_t* p_axis)
{
    Uint32 elapsed_segment_cnt;
    int32 temp_cnt;
    int8_t dir;
    Uint32 ret;
    float64 t1;
    _iq1   elapsed_segments_iq , t1_us_iq ,t2_us_iq , ret_iq , v0_1000_iq ,a_1000_iq ;
    _iq24 a_iq24, part2_iq24, delt_iq24, v0_sq_iq24, v0_iq24, delt_x_iq24, sqrt_delt_iq24, v_diff_iq24, t_iq24;
//    float temp;
    const _iq30  g_1000_iq = _IQ30(0.001);
    const _iq1   g_1000000_iq = _IQ1(1000000);

//    GpioDataRegs.GPBDAT.bit.GPIOB1 = 1;

    /* 获取当前阶段输入轴的初速度V0 , 加速度a , 以及微步数*/
    /* -600~600 */
    v0_1000_iq = fastLeftShift(p_axis->stage[p_axis->cur_stage].v0_1000 , 1); //当前阶段初速度转化为IQ格式 , 输出轴实际速度 ,单位°/s
    //temp = _IQ1toF(v0_1000_iq);

    a_1000_iq  = fastLeftShift(p_axis->stage[p_axis->cur_stage].a_1000 , 1);  //当前阶段加速度转化为IQ格式  , 输出轴实际加速度 ,单位°/s2
    //temp = _IQ1toF(a_1000_iq);

    v0_iq24 = _IQ24mpyIQX(v0_1000_iq, 1, g_1000_iq, 30);
    //temp = _IQ24toF(v0_iq24);

    a_iq24  = _IQ24mpyIQX(a_1000_iq, 1, g_1000_iq, 30) ;
    //temp = _IQ24toF(a_iq24);

    /* 当前阶段已走过的微步数和方向 */
    elapsed_segment_cnt = p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt;
    dir = p_axis->stage[p_axis->cur_stage].mov_dir;

    if (elapsed_segment_cnt == 0) {
        t1 = 0;
    } else {
        t1 = p_axis->t2_us;         /* 上一个微步的脉冲时间*/
    }
    /* 获取输出轴下一个脉冲一共走过的角度  , 最大角度±2048*/

    temp_cnt = ((elapsed_segment_cnt + 1) * dir); //直接将这个表达式传入_IQ1会出现计算错误 ,因为elapsed_segment_cnt是个无符号数
    elapsed_segments_iq = fastLeftShift(temp_cnt , 1);
    delt_x_iq24 = _IQ24mpyIQX(elapsed_segments_iq , 1 , p_axis->step_angle_iq , 30);//两个不同的IQ数字相乘 返回一个IQ值 ,当前阶段走过的角度 ,主要计算加速和减速过程 ,角度变化范围有限
    //temp = _IQ24toF(delt_x_iq24);

    /* b^2 */
    v0_sq_iq24  = _IQ24mpy(v0_iq24 , v0_iq24);                                  //v0的平方 ,范围 0-4
    //temp = _IQ24toF(v0_sq_iq24);

    part2_iq24  = _IQ24mpy(a_iq24, delt_x_iq24);                               //a * x
    //temp = _IQ24toF(part2_iq24);

    /* -4ac */
    part2_iq24  = _IQ24mpy(part2_iq24, _IQ24(2));
    //temp = _IQ24toF(part2_iq24);

    delt_iq24 = v0_sq_iq24 + part2_iq24;                                      //V0方+2ax = v方
    //temp = _IQ24toF(delt_iq24);

    /* 由于浮点数计算精度的问题，在某个阶段的末尾可能会出现无实数根的情况，此时直接返回上个段数即可 */
    if (delt_iq24 < 0) {
        return p_axis->next_segment_pulse_cnt;
    }

    /*计算根号v ,分别考虑V为正和负的情况 ,最终计算出V与V0的差值*/
    sqrt_delt_iq24 = _IQ24sqrt(delt_iq24);
    real_v = _IQ24toF(sqrt_delt_iq24);

    /* -b + sqrt(b^2 - 4ac) */
    v_diff_iq24 = sqrt_delt_iq24 - v0_iq24;
    //temp = _IQ24toF(v_diff_iq24);

    t_iq24 = _IQ24div(v_diff_iq24 , a_iq24);
    //temp = _IQ24toF(t_iq24);

    t1_us_iq = _IQ1mpyIQX(t_iq24, 24, g_1000000_iq, 1);
    //temp = _IQ1toF(t1_us_iq);

    /* -b - sqrt(b^2 - 4ac) */
    v_diff_iq24 = (0 - sqrt_delt_iq24) - v0_iq24 ;
    //temp = _IQ24toF(v_diff_iq24);

    t_iq24 = _IQ24div(v_diff_iq24 , a_iq24);
    //temp = _IQ24toF(t_iq24);

    t2_us_iq = _IQ1mpyIQX(t_iq24, 24, g_1000000_iq, 1);
    //temp = _IQ1toF(t2_us_iq);

    /* 时间计算法 */
    if ((t1_us_iq < 0) && (t2_us_iq > 0)) {         /* 取正数根 */
        t1_us_iq = t2_us_iq;
    } else if ((t1_us_iq > 0) && (t2_us_iq < 0)) {
        t1_us_iq = t1_us_iq;
    } else if ((t1_us_iq > 0) && (t2_us_iq > 0)) {  /* 如果两个根都是正数，那么取小的那个 */
        t1_us_iq = (t1_us_iq > t2_us_iq) ? t2_us_iq : t1_us_iq;
    } else {
        return 1;                                               //都是负根 ,计算错误
    }

    p_axis->t2_us = _IQ1toF(t1_us_iq);
    ret_iq = t1_us_iq - _IQ1(t1);
    ret_iq = ret_iq > 0 ? ret_iq : -ret_iq;                     //取正值
    ret_iq = _IQ1div(ret_iq , 100);
    ret =(Uint32)_IQ1int(ret_iq);                              /*注意返回的是一个int型*/

//    if (cnt < 200) {
//        cnt_arr[cnt++] = ret;
//    } else {
//        cnt = 0;
//    }

//    GpioDataRegs.GPBDAT.bit.GPIOB1 = 0;

    return ret;
}


/**
 * \brief 给定初速度和加速度，得到该阶段的运动参数
 * \param[in]  p_axis：需要计算的轴
 * \param[in]  angle_speed_v0：初始角速度
 * \param[in]  speed_given：速度给定
 * \param[in]  a_given：给定的加速度值，一定是个正数
 * \param[out] p_stage：得到的该阶段的运动参数
 *
 * \note angle_speed_v0和speed_given符号一定不能相反，可以为0，但不能同时为0
 */
Uint16 test_cnt_1 = 0,test_cnt_2 = 0;
#pragma CODE_SECTION(__param_calc, "ramfuncs");
static void __param_calc (const single_axis_conf_t* p_axis, float64 angle_speed_v0, float64 speed_given, int16 a_given, stage_var_t* p_stage)
{
#if 0
	_iq v_iq, v0_iq, a_iq, t_iq, x_iq;
	_iq result_iq;
	_iq speed_ratio_iq = _IQ(p_axis->speed_ratio);
	_iq step_angle_iq = _IQ(p_axis->step_angle);

	//浮点转IQ , 目标速度最大 ±2°/s
	//float temp;
	v_iq = _IQ((float32)speed_given * p_axis->speedvalue_to_speed);
//	temp = _IQtoF(v_iq);

	/* angle_speed_v0为实际角速度 , 单位 °/s */
//	v0_iq = _IQ((float32)angle_speed_v0);
//	temp = _IQtoF(v0_iq);
	p_stage->v0_1000 = angle_speed_v0 * 1000;  //扩大1000倍,得到角速度
	v0_iq = _IQ(p_stage->v0_1000 * 0.001);	   //再转换为iq格式,防止精度损失



	//IQ与长整型数据相乘 ,返回IQ数据整数部分,损失了精度例如 v0 = 0.6004483185
//	p_stage->v0_1000 = _IQmpyI32int(v0_iq , 1000);

	/* 得到加速度方向 ,目标速度减去当前速度 , 大于0则a>0 ,小于0则a<0 */
	p_stage->a_1000 = a_given * 5;
	a_iq = _IQ((float32) a_given * 0.005);

	if ((v_iq - v0_iq) < 0) {
		a_iq = 0 - a_iq;		/* 加速度a = -a_given * 0.005 */
		p_stage->a_1000 = 0 - p_stage->a_1000;
	}

	//temp = _IQtoF(a_iq);

	/* 得到运动方向  */
	if (v0_iq > 0 || v_iq > 0) {
		p_stage->mov_dir = 1;
	} else if ((v0_iq < 0) || (v_iq < 0)) {
		p_stage->mov_dir = -1;
	}

	/* 得到当前阶段的时间  , δt = δv / a ,单位s */
	/* IQ15最大值65535*/
	t_iq = _IQdiv((v_iq - v0_iq) , a_iq);
	p_stage->t  = _IQtoF(t_iq);
	/* 得到输出轴角度变化  */
	x_iq = _IQmpy(v0_iq, t_iq) + _IQmpy(_IQdiv(a_iq, _IQ(2)), _IQmpy(t_iq, t_iq));		/* x = v0*t + 1/2 * at^2 */
	//temp = _IQtoF(x_iq);

	/* 得到整步差（每1/4个周期为1步）, 这种方法超过655°会溢出 */
	result_iq = _IQmpy(_IQdiv(speed_ratio_iq , step_angle_iq) , x_iq);
	// 获取绝对值并转换为浮点数 , 赋值给 p_stage->all_step_cnt
	p_stage->all_step_cnt = _IQtoF(_IQabs(result_iq));

	/* 微步数量 = 整步 * 细分数 , 此处为整数乘浮点数 ,不要求精度 ,所以选择IQ1 , 返回的为IQ整数部分*/
	p_stage->all_segment = p_stage->all_step_cnt * XIFEN_CNT;
	p_stage->loc_diff = p_stage->all_segment / 2;
	p_stage->elapsed_segment_cnt = 0;
#else
	float64 x_1000;
	int32 v_100;

	v_100 = (int16)nearbyintf((float32)speed_given * p_axis->speedvalue_to_speed * 100);

	/* 减速过程加速度方向与V0相反  */
	p_stage->v0_1000 = angle_speed_v0 * (float64)1000;	/* 初始速度的1000倍 , 即为实际角速度 , 单位 °/s */

	/* 得到加速度的1000倍，没有误差，单位： °/(s^2) */
//	if (v_100 * 10 - p_stage->v0_1000 < 0) {		/* 给定速度和初速度的差*/
//		p_stage->a_1000 = -a_given * 5;				/* 省略了 , 应该写为 -a_given * 0.005 * 1000*/
//	} else {
//		p_stage->a_1000 = a_given * 5;
//	}
	if (v_100 * 10 - p_stage->v0_1000 < 0) {		/* 给定速度和初速度的差*/
		p_stage->a_1000 = 0 - (a_given * 5);		/* 省略了 , 应该写为 -a_given * 0.005 * 1000*/
	} else {
		p_stage->a_1000 = a_given * 5;
	}
	/* 得到运动方向 */
	if ((p_stage->v0_1000 > 0) || (v_100 > 0)) {
		p_stage->mov_dir = 1;
	} else if ((p_stage->v0_1000 < 0) || (v_100 < 0)) {
		p_stage->mov_dir = -1;
	}
	else{
		test_cnt_1++;
	}

	/* 得到当前阶段的时间 */
	p_stage->t = (float64)(v_100 * 10 - p_stage->v0_1000) / p_stage->a_1000;

	/* 得到输出轴角度变化 , 扩大了1000倍 */
	x_1000 = (float64)p_stage->v0_1000 * p_stage->t + p_stage->a_1000 * p_stage->t * p_stage->t / 2 ;		/* x = v0*t + 1/2 * at^2 */

	/* 得到整步差（每1/4个周期为1步） */
	p_stage->all_step_cnt = fabs(x_1000 / 1000 * p_axis->speed_ratio / p_axis->step_angle);  				/* 这种处理相当于只走整步 */

	/* 微步数量 */
	p_stage->loc_diff = nearbyintf(fabs(x_1000 / p_axis->loc_to_angle_factor / 1000));

	p_stage->all_segment = p_stage->all_step_cnt * XIFEN_CNT; /* 细分之后的步数 */
	p_stage->elapsed_segment_cnt = 0;
#endif
}



/**
 * \brief 根据占空比和电流系数计算pwm的比较寄存器的值
 *
 * \param[in] p_axis：需要配置的轴
 * \param[in] p_pwm：无
 */
#pragma CODE_SECTION(duty_to_cycle, "ramfuncs");
void duty_to_cycle(single_axis_conf_t* p_axis)
{
    uint8_t i = 0;
    _iq	  pwm_iq;
    _iq1  xifen_cnt_iq = _IQ1(PWM_PERIOD);
    _iq30 duty_iq , factor_iq ;				//两变量范围都在0-1 ,所以选择iq30

    /* 计算运动时的周期值 */
    factor_iq = _IQ30(p_axis->duty_factor_mov);
    /* PWM设定的时基频率为45M，单个PWM周期固定为50us，需要2*PWM_PERIOD个时基，设定上下计数模式，则周期值为PWM_PERIOD */
    for (i = 0; i < XIFEN_CNT + 1; i++) {
    	duty_iq = _IQ30(g_Axis_Conf.duty[i]);
    	duty_iq = _IQ30mpy(duty_iq , factor_iq); 								  //两个小于0的系数先乘
    	pwm_iq  = _IQmpyIQX(xifen_cnt_iq , 1 , duty_iq , 30);
        p_axis->pulse_tbprd_mov[i] = PWM_PERIOD - _IQint(pwm_iq);   				  //返回整数部分
    }

    /* 计算保持时的周期值 */
    factor_iq = _IQ30(p_axis->duty_factor_hold);
    /* PWM设定的时基频率为45M，单个PWM周期固定为50us，需要2*PWM_PERIOD个时基，设定上下计数模式，则周期值为PWM_PERIOD */
    for (i = 0; i < XIFEN_CNT + 1; i++) {
        duty_iq = _IQ30(g_Axis_Conf.duty[i]);
        duty_iq = _IQ30mpy(duty_iq , factor_iq);                                  //两个小于0的系数先乘
        pwm_iq  = _IQmpyIQX(xifen_cnt_iq , 1 , duty_iq , 30);
        p_axis->pulse_tbprd_hold[i] = PWM_PERIOD - _IQint(pwm_iq);                     //返回整数部分
    }
}

/**
 * \brief 根据当前位置，目标位置，和速度方向得到位置差
 */
#pragma CODE_SECTION(__get_local_diff, "ramfuncs");
static int32 __get_local_diff (single_axis_conf_t* p_ab_axis, int32 loc_given, int32 speed_given)
{
    int32 loc_diff;

    loc_diff = loc_given - p_ab_axis->cur_local;
#if BOARD_NAME == BOARD_GMS
    //啥也不干
#elif BOARD_NAME == BOARD_DGM_2 /* 不规划最短路径 */

    if ((speed_given > 0) && (loc_diff < 0)) {
        loc_diff += 1024000;
    } else if ((speed_given < 0) && (loc_diff > 0)) {
        loc_diff -= 1024000;
    }
#elif BOARD_NAME == BOARD_DGM_4 /* 需要规划最短路径 */
    if ((!strncmp(p_ab_axis->name, "PYA", 3)) || (!strncmp(p_ab_axis->name, "NYA", 3))) {
        if (loc_diff > 512000) {
            loc_diff -= 1024000;
        } else if (loc_diff < -512000) {
            loc_diff += 1024000;
        }
    } else if ((!strncmp(p_ab_axis->name, "PYB", 3)) || (!strncmp(p_ab_axis->name, "NYB", 3))) {
        loc_diff = loc_given - p_ab_axis->cur_local;
    }
#endif
    return loc_diff;
}


/**
 * \brief 执行AB轴运动控制指令
 *
 * \retval 成功返回0，失败返回-1
 */
#pragma CODE_SECTION(ab_axis_mov_ctl_cmd_run, "ramfuncs");

int16 ab_axis_mov_ctl_cmd_run(single_axis_conf_t* p_ab_axis)
{
    float64  temp = 0;
    float64  v1_1000, x1_1000, x2_1000, t1, t2;

    single_axis_conf_t axis_temp;
    int16   speed_given = 0;   			/* 速度给定，复位模式给的是复位速度，捕获模式给的是捕获速度 */
    int32   loc_given = 0;     			/* 位置给定，复位模式给的是复位位置，捕获模式给的是捕获位置 */
    int32 	v0_1000;					/* 作为中间变量的数用定点数 */
    int32   loc_diff = 0;				/* 目标位置与当前位置之差 */
    int32   loc_acc1, loc_acc2 = 0;		/* 两个变速过程需要移动的位置 */
    int32   angle_diff_1000 = 0;

//    _iq30   time_factor_iq ;   			//当前pwm中断时间 ,单位s
    _iq23	angle_speed_iq;				//当前输出轴的速度,单位°/s
    _iq23   step_angle_iq ;             //输出轴步距角
    _iq23   step_t_iq;					//微步时间 ,单位 s
    _iq16	pulse_cnt_iq ;				//当前微步中,50uspwm波的个数 ,包括方向dir

//    GpioDataRegs.GPBDAT.bit.GPIOB2 = 1;

    /* 对轴里的数据进行缓存 */
    memcpy(&axis_temp, p_ab_axis, sizeof(single_axis_conf_t));

    /* 不需要开启新任务直接跳过 */
    if (axis_temp.new_task_flag == 0) {
        return 0;
    }
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
    if (axis_temp.work_mode_set == WORK_MODE_STANDBY) {
    	return 0;
    }
#endif


	/* 单个轴判定 */
		if ((axis_temp.work_mode_old != WORK_MODE_KEEP) || (axis_temp.work_mode_set != WORK_MODE_KEEP)) {	/* 没有保持模式转保持模式 */
//			axis_temp.cur_local = cur_local_modify_A(p_axis);
			/* 这段本来是放在中断里面，调试时可以随意看到当前速度，但是这段代码运行时间太长（大概20us），所以将其拿出 */
            //if (axis_temp.work_mode_old != WORK_MODE_KEEP) {    /* 旧模式不为保持模式，说明是从别的模式手动跳转过来的，要计算初速度 */ /* 不能这么写，如果手动发送保持模式，在电机还没停下来的时候又发送运动模式，则电机速度会突然降低到0 */
            if (g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) {    /* 判断电机是否正在运动，要计算初速度 */
				/* 计算的是细分+减速比之后的速度 , 即实际输出的速度 */
#if 0
				step_angle_iq =_IQ23(STEP_ANGLE);
				time_factor_iq = _IQ30(PWM_TICK * 0.000001);				   		  //必须强制转换 ,否则计算为0
				pulse_cnt_iq = _IQ1mpy(_IQ1(axis_temp.stage[axis_temp.cur_stage].mov_dir) , _IQ1(axis_temp.next_segment_pulse_cnt));
				angle_speed_iq = _IQ23mpyIQX(pulse_cnt_iq , 1 , time_factor_iq , 30);  //当前微步的时间 ,单位s,±256 ,精度0.000000119
				angle_speed_iq = _IQ23div(step_angle_iq , angle_speed_iq);			   //当前输出轴角速度 ,单位°/s,±256 ,精度0.000000119
                axis_temp.cur_angle_speed = _IQ23toF(angle_speed_iq);
#else
                if (axis_temp.next_segment_pulse_cnt != 0) {
                    if ( axis_temp.stage[ axis_temp.cur_stage].a_1000 != 0) {
 //                       axis_temp.cur_angle_speed = (float64)1000000 * axis_temp.step_angle / ((float64)XIFEN_CNT * axis_temp.next_segment_pulse_cnt * 50 * axis_temp.speed_ratio) * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                        axis_temp.cur_angle_speed = real_v * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                    } else {
                        axis_temp.cur_angle_speed = (float64)1000000 * axis_temp.step_angle / ((float64)XIFEN_CNT * ((float64)axis_temp.uniform_segment_pulse_cnt_zheng + (float64)axis_temp.uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * axis_temp.speed_ratio) * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                    }
                } else {
                    axis_temp.cur_angle_speed = 0;
                }
#endif
			} else {
				axis_temp.cur_angle_speed = 0;
			}

			axis_temp.reciprocat_mode_enable = 0;

			/* 确定目标位置、移动速度、运动方向 */
			switch (axis_temp.work_mode_set) {
				case WORK_MODE_TRACE:   /* 跟踪模式（展开模式，往复模式） */
				    speed_given = axis_temp.speed_given;
				    if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
#if BOARD_NAME == BOARD_GMS /* 往复模式 */
                        axis_temp.reciprocat_mode_enable = 1;

                        if (speed_given > 0) {
                            loc_given = axis_temp.soft_lim_p - 222;
                        } else if (speed_given < 0) {
                            loc_given = axis_temp.soft_lim_n + 222;
                        }

                        /* 刚好在给定位置 */
                        if (loc_given == axis_temp.cur_local) {
                            if (loc_given == axis_temp.soft_lim_p - 222) {    /* 刚好在正限位 */
                                loc_given = axis_temp.soft_lim_n;
                                if (speed_given > 0) {
                                    speed_given = 0 - speed_given;
                                }
                            } else if (loc_given == axis_temp.soft_lim_p + 222) {    /* 刚好在负限位 */
                                loc_given = axis_temp.soft_lim_p;
                                if (speed_given < 0) {
                                    speed_given = 0 - speed_given;
                                }
                            }
                        }

                        axis_temp.work_mode_set = WORK_MODE_CAPTURE;
#elif BOARD_NAME == BOARD_DGM_4 /* 展开模式 */
                        loc_given = axis_temp.reset_local;
                        speed_given = 1777;     /* 1.6°/s */
                        axis_temp.max_acc = 100; /* 加速度0.5°/s^2 */

                        if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                            ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                            speed_given = 0 - speed_given;
                        }

                        axis_temp.speed_given = speed_given;
                        axis_temp.loc_given   = 0;
#endif
                    }
					break;

				case WORK_MODE_CAPTURE:	/* 捕获模式，关心速度给定和位置给定 */
					speed_given = axis_temp.speed_given;
					loc_given = axis_temp.loc_given;
					if (loc_given == axis_temp.cur_local) { /* 位置给定等于当前位置直接返回0 */
					    return 0;
					}
#if (BOARD_NAME == BOARD_GMS)
					if (((loc_given < axis_temp.cur_local) && (speed_given > 0)) ||
					    ((loc_given > axis_temp.cur_local) && (speed_given < 0)))  {
						speed_given = -speed_given;
					}

#elif (BOARD_NAME == BOARD_DGM_4)    /* 寻求最短路径 */
                    if (loc_given > axis_temp.cur_local) {
                        if (((loc_given - axis_temp.cur_local >  512000) && (speed_given > 0)) ||
                            ((loc_given - axis_temp.cur_local <= 512000) && (speed_given < 0))) {
                            speed_given = 0 - speed_given;
                        }
                    } else if (loc_given < axis_temp.cur_local) {
                        if (((axis_temp.cur_local - loc_given >  512000) && (speed_given < 0)) ||
                            ((axis_temp.cur_local - loc_given <= 512000) && (speed_given > 0))) {
                            speed_given = 0 - speed_given;
                        }
                    } else if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
	                    if (((loc_given < axis_temp.cur_local) && (speed_given > 0)) ||
	                        ((loc_given > axis_temp.cur_local) && (speed_given < 0)))  {
	                        speed_given = 0 - speed_given;
	                    }
					}
#endif
				break;

				case WORK_MODE_RESET:	/* 复位模式，就是回0模式 */
                    speed_given = axis_temp.reset_speed_given;
                    loc_given = axis_temp.reset_local;
                    if (loc_given == axis_temp.cur_local) { /* 位置给定等于当前位置直接返回0 */
                        return 0;
                    }

#if BOARD_NAME == BOARD_GMS
                    if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                        ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                        speed_given = -speed_given;
                    }
                    g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 1;    /* 正在回零 */
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2)
                    if ((!strncmp(axis_temp.name, "PYA", 3)) || (!strncmp(axis_temp.name, "NYA", 3))) {
                        if (loc_given > axis_temp.cur_local) {
                            if (((loc_given - axis_temp.cur_local >  512000) && (speed_given > 0)) ||
                                ((loc_given - axis_temp.cur_local <= 512000) && (speed_given < 0))) {
                                speed_given = 0 - speed_given;
                            }
                        } else if (loc_given < axis_temp.cur_local) {
                            if (((axis_temp.cur_local - loc_given >  512000) && (speed_given < 0)) ||
                                ((axis_temp.cur_local - loc_given <= 512000) && (speed_given > 0))) {
                                speed_given = 0 - speed_given;
                            }
                        }
                    } else if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
                        if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                            ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                            speed_given = -speed_given;
                        }
                    }
                    if (!strncmp(axis_temp.name, "PYA", 3)){
                        g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 1;    /* 正在回零 */
                    }else if(!strncmp(axis_temp.name, "NYA", 3)){
                        g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 1;    /* 正在回零 */
                    }
#if BOARD_NAME == BOARD_DGM_4
                    else if(!strncmp(axis_temp.name, "PYB", 3)){
                        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 1;    /* 正在回零 */
                    }else if(!strncmp(axis_temp.name, "NYB", 3)){
                        g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 1;    /* 正在回零 */
                    }
#endif
#endif

				break;

				case WORK_MODE_KEEP:	/* 保持模式，不改变运动方向，缓慢减速直至停止 */
					/* 不用关心速度给定和位置给定 */
				break;
			}

			//axis_temp.cur_angle_speed = -0.05;  //测试用

			/* 如果转入保持模式 ,目标速度设置为0 , 只有一个阶段*/
			if (axis_temp.work_mode_set == WORK_MODE_KEEP) {
				__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0 , axis_temp.max_acc, &axis_temp.stage[0]);
				axis_temp.stage_cnt = 1;
			} else {
				/* 得到匀速时的脉冲周期（假设可以达到这个值） , 此处有颗粒度所以速度并不准确 */
				angle_speed_iq = _IQ23(speed_given * axis_temp.speedvalue_to_speed);   //#5035681 速度给定值乘以系数 = 目标速度(单位 °/s) ,±256 ,精度0.000000119
				step_angle_iq  = axis_temp.step_angle_out_iq23;							//#4718 输出轴步距角
				step_t_iq      = _IQ23div(step_angle_iq , angle_speed_iq);			 //#7859，如果是-1则不对。  匀速的微步时间 ,单位°/s
				pulse_cnt_iq   = _IQ16mpyIQX(step_t_iq , 23 , _IQ16(20000) , 16);       //#37，如果是-1则不对。 单位个 ,t*1000000/50 = t/20000	 ,简化运算 ,如果50us改了这里也要修改

				axis_temp.uniform_segment_pulse_cnt_zheng = _IQ16int(_IQ16abs(pulse_cnt_iq));  //由于速度有正负 ,所以先取绝对值 ,再取整 ,这种取整方式舍掉了小数 ,虽然能提高运算速度.
				axis_temp.uniform_segment_pulse_cnt_xiao_1000= _IQ16int(_IQ16mpyIQX(_IQ16abs(pulse_cnt_iq) - _IQ16(_IQ16int(_IQ16abs(pulse_cnt_iq))), 16, _IQ16(1000), 16));

				/* 规划阶段0 */
				axis_temp.stage_cnt = 0;

				loc_diff = __get_local_diff(&axis_temp, loc_given, speed_given);

				/* 给定速度和初速度方向不同，需要先减速到0，再返回 */
				if (((axis_temp.cur_angle_speed < 0) && (speed_given > 0)) || ((axis_temp.cur_angle_speed > 0) && (speed_given < 0))) {	/* 方向不同 */
					__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);
					__param_calc(&axis_temp, 0, speed_given, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);
					/* 当初速度方向为正时 ,由于给定位置在当前位置的后面 , 所以需要先减速 , 再反向加速 ,减速的过程往前又走了一段距离 */
					if (loc_diff < 0) {
						loc_diff -= axis_temp.stage[0].loc_diff;
					} else {
					/* 当初速度方向为负时 ,由于给定位置在当前位置的后面 , 所以需要先减速 , 再反向加速 ,减速的过程往前又走了一段距离*/
						loc_diff += axis_temp.stage[0].loc_diff;
					}
					/* 如果此时为复位或捕获模式，或者DGM_4的B轴展开模式*/
#if BOARD_NAME == BOARD_GMS || BOARD_NAME == BOARD_DGM_2
					if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
					    (axis_temp.work_mode_set == WORK_MODE_RESET)) {
#elif BOARD_NAME == BOARD_DGM_4
					if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)   ||
                        ((axis_temp.work_mode_set == WORK_MODE_LAUNCH) && ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))))) {
#endif
						if (((loc_diff > 0) && (loc_diff <= 2 * axis_temp.stage[1].loc_diff)) ||
							((loc_diff < 0) && (-loc_diff <= 2 * axis_temp.stage[1].loc_diff)))	{   /* 没有匀速阶段 */
							axis_temp.stage[1].a_1000 = axis_temp.stage[0].a_1000;
							axis_temp.stage[2].a_1000 = -axis_temp.stage[1].a_1000;
							if (loc_diff > 0) {
								axis_temp.stage[1].loc_diff = loc_diff / 2;
								axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
							} else {
								axis_temp.stage[1].loc_diff = -loc_diff / 2;
								axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
							}
							axis_temp.stage[1].mov_dir = -axis_temp.stage[0].mov_dir;
							axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
							axis_temp.stage[1].t = sqrt(((float64)axis_temp.stage[1].loc_diff * axis_temp.loc_to_angle_factor) * 2 / ((float64)axis_temp.max_acc * 0.005));
							axis_temp.stage[2].t = axis_temp.stage[1].t;
							axis_temp.stage[1].v0_1000 = 0;
							axis_temp.stage[2].v0_1000 = axis_temp.stage[1].a_1000 * axis_temp.stage[1].t;

                            axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;  //阶段2整步数
                            axis_temp.stage[1].all_segment  = axis_temp.stage[1].loc_diff * 2;		   				 //阶段2微步数
                            axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT * 2;  //阶段3整步数
                            axis_temp.stage[2].all_segment  = axis_temp.stage[2].loc_diff * 2;	   				 	 //阶段3微步数
                            axis_temp.stage[1].elapsed_segment_cnt = 0;						   				 //阶段1在807行调用函数已经清零了
                            axis_temp.stage[2].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 3;
						} else {	/* 有匀速阶段 */
							axis_temp.stage[3].a_1000 = (-1) * axis_temp.stage[1].a_1000;
							axis_temp.stage[3].loc_diff = axis_temp.stage[1].loc_diff;
							axis_temp.stage[3].mov_dir  = axis_temp.stage[1].mov_dir;
							axis_temp.stage[3].t = axis_temp.stage[1].t;
							axis_temp.stage[3].v0_1000  =  (int16)nearbyintf((float)speed_given * axis_temp.speedvalue_to_speed * 1000);  //目标速度扩大1000
                            axis_temp.stage[3].all_step_cnt = axis_temp.stage[3].loc_diff / (float32)XIFEN_CNT * 2;
                            axis_temp.stage[3].all_segment  = axis_temp.stage[3].loc_diff * 2;

							axis_temp.stage[2].a_1000 = 0;
							if (loc_diff < 0) {
								loc_diff = 0 - loc_diff;
							}
							axis_temp.stage[2].loc_diff = loc_diff - (axis_temp.stage[1].loc_diff + axis_temp.stage[3].loc_diff);
							axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
							axis_temp.stage[2].v0_1000 = axis_temp.stage[3].v0_1000;
							//时间计算有误????
							axis_temp.stage[2].t = fabs(((float64)axis_temp.stage[2].loc_diff) / speed_given);
                            axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT *2;
                            axis_temp.stage[2].all_segment = axis_temp.stage[2].loc_diff * 2;

                            axis_temp.stage[2].elapsed_segment_cnt = 0;
                            axis_temp.stage[3].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 4;
						}
					}
				} else {	/* 给定速度和初速度方向相同 */
					__param_calc(&axis_temp, axis_temp.cur_angle_speed, speed_given, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);

					/* 捕获模式，复位模式，以及B轴的展开模进 */
#if BOARD_NAME == BOARD_GMS || BOARD_NAME == BOARD_DGM_2
                    if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)) {
#elif BOARD_NAME == BOARD_DGM_4
                    if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)   ||
                        ((axis_temp.work_mode_set == WORK_MODE_LAUNCH) && ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))))) {
#endif
						__param_calc(&axis_temp, (float64)speed_given * axis_temp.speedvalue_to_speed, 0, axis_temp.max_acc, &axis_temp.stage[2]);

						loc_acc1 = axis_temp.stage[0].loc_diff * axis_temp.stage[0].mov_dir;
						loc_acc2 = axis_temp.stage[2].loc_diff * axis_temp.stage[2].mov_dir;

						if (((loc_diff <= 0) && (loc_acc1 + loc_acc2 <= 0) && (loc_diff >= loc_acc1 + loc_acc2)) ||
							((loc_diff > 0) && (loc_acc1 + loc_acc2 > 0) && (loc_diff < loc_acc1 + loc_acc2))) {		/* 速度不够加到给定速度 ,没有匀速阶段 */
								/* 不够的情况有2种，第一种是立刻以最大加速度减速到0依然要超出设定位置，那么直接以最大加速度减速到0再返回来，第二种是立刻以最大加速度减速到0的路程够，那么此时可以先加速到V，再减速到0。 */

								v0_1000 = axis_temp.cur_angle_speed * 1000;			/* 初始速度的1000倍 °/s */
								angle_diff_1000 = (float64)loc_diff * axis_temp.loc_to_angle_factor * 1000;

								t1 = fabs((float64)v0_1000 / (axis_temp.max_acc * 5));
								temp = 0.5 * (float64)axis_temp.max_acc * 0.005 * t1 * t1;
								loc_acc1 = temp / axis_temp.loc_to_angle_factor;

								/* 不能先加速，直接速度到0，然后返回 */
								if (((loc_diff <= 0) && (loc_diff > -loc_acc1)) ||
									((loc_diff > 0) && (loc_diff < loc_acc1))) {		/* 立即减速还要返回 */
									__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0, axis_temp.max_acc, &axis_temp.stage[0]);
									loc_diff = loc_acc1 - loc_diff;
									if (loc_diff < 0) {
										loc_diff = -loc_diff;
									}
									axis_temp.stage[1].a_1000 = axis_temp.stage[0].a_1000;
									axis_temp.stage[2].a_1000 = -axis_temp.stage[0].a_1000;
									axis_temp.stage[1].loc_diff = loc_diff / 2;
									axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
									axis_temp.stage[1].mov_dir = -axis_temp.stage[0].mov_dir;
									axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
									axis_temp.stage[1].t = sqrt(((float64)axis_temp.stage[1].loc_diff * axis_temp.loc_to_angle_factor) * 2 / ((float64)axis_temp.max_acc * 0.005));
									axis_temp.stage[2].t = axis_temp.stage[1].t;
									axis_temp.stage[1].v0_1000 = 0;
									axis_temp.stage[2].v0_1000 = axis_temp.stage[1].a_1000 * axis_temp.stage[1].t;

                                    axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;
                                    axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[2].all_segment = axis_temp.stage[2].loc_diff * 2;

                                    axis_temp.stage[1].elapsed_segment_cnt = 0;
                                    axis_temp.stage[2].elapsed_segment_cnt = 0;

									axis_temp.stage_cnt = 3;
								} else {	/* 可以先加速 */
									/* 假设初始速度为v0，中间最大速度为v1，加速路程为x1，减速路程为x2，加速时间为t1，减速时间为t2，加速过程加速度为a，减速过程加速度为-a，得到以下方程 */
									/* x1=v0t1+(1/2)at1^2	*/
									/* x2=v1t1-(1/2)at2^2	*/
									/* a=(v1-v0)/t1 */
									/* a=v1/t2 */
									/* x1+x2=angle_diff */
									/* 其中v0，a，angle_diff已知，求x1,x2,t1,t2,v1 */
									/* v1^2 = (angle_diff *2a+v0^2)/2 */

									v1_1000 = sqrt(fabs((angle_diff_1000 * 2 * axis_temp.stage[0].a_1000 + axis_temp.stage[0].v0_1000 * axis_temp.stage[0].v0_1000) / 2));
									if (speed_given < 0) {
										v1_1000 = -v1_1000;
									}

									t1 = (float64)(v1_1000 - v0_1000) / axis_temp.stage[0].a_1000;
									if(t1 < 0 ) {
										test_cnt_2++;
									}

									t2 = v1_1000 / axis_temp.stage[0].a_1000;
									x1_1000 = v0_1000 * t1 + axis_temp.stage[0].a_1000 * t1 * t1 / 2;
									x2_1000 = v1_1000 * t2 - axis_temp.stage[0].a_1000 * t2 * t2 / 2;

									axis_temp.stage[0].t = t1;
									axis_temp.stage[1].t = t2;
		  							axis_temp.stage[0].loc_diff = nearbyintf(fabs(x1_1000 / axis_temp.loc_to_angle_factor / 1000));
		  							axis_temp.stage[1].loc_diff = nearbyintf(fabs(x2_1000 / axis_temp.loc_to_angle_factor / 1000));

									/* 补上计算误差，确保位置0误差 */
		  							if (loc_diff < 0) {
		  								loc_diff = -loc_diff;
		  							}
									while (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff < loc_diff) {
										axis_temp.stage[0].loc_diff++;
										if (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff < loc_diff) {
											axis_temp.stage[1].loc_diff++;
										}
									}
									while (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff > loc_diff) {
										axis_temp.stage[0].loc_diff--;
										if (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff > loc_diff) {
											axis_temp.stage[1].loc_diff--;
										}
									}

                                    axis_temp.stage[0].all_step_cnt = axis_temp.stage[0].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[0].all_segment = axis_temp.stage[0].loc_diff * 2;
                                    axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;

		                            /* 第二段 */
									axis_temp.stage[1].v0_1000 = v1_1000;
									axis_temp.stage[1].a_1000 = (-1) * axis_temp.stage[0].a_1000;
									axis_temp.stage[1].mov_dir = axis_temp.stage[0].mov_dir;

		                            axis_temp.stage[0].elapsed_segment_cnt = 0;
		                            axis_temp.stage[1].elapsed_segment_cnt = 0;

									axis_temp.stage_cnt = 2;
								}
						} else {	/* 有匀速阶段 */
							if (loc_diff - (loc_acc1 + loc_acc2) > 0) {
								axis_temp.stage[1].loc_diff = loc_diff - (loc_acc1 + loc_acc2);
							} else {
								axis_temp.stage[1].loc_diff = (loc_acc1 + loc_acc2) - loc_diff;
							}
							axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
							axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;

							axis_temp.stage[1].v0_1000 = axis_temp.stage[2].v0_1000;
							axis_temp.stage[1].a_1000 = 0;

							axis_temp.stage[1].mov_dir = axis_temp.stage[0].mov_dir;
							axis_temp.stage[1].t = fabs(((float64)axis_temp.stage[1].loc_diff) / speed_given);
                            axis_temp.stage[1].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 3;
						}
					}
				}
#if BOARD_NAME == BOARD_GMS
				if ((axis_temp.work_mode_set == WORK_MODE_TRACE)) {    /* 跟踪模式 */
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
				if ((axis_temp.work_mode_set == WORK_MODE_TRACE) && ((!strncmp(axis_temp.name, "PYA", 3)) || (!strncmp(axis_temp.name, "NYA", 3)))) {    /* 跟踪模式 */
#endif
					axis_temp.stage[axis_temp.stage_cnt].v0_1000 = (int16)nearbyintf((float32)speed_given * axis_temp.speedvalue_to_speed * 1000);
					axis_temp.stage[axis_temp.stage_cnt].a_1000 = 0;
					axis_temp.stage[axis_temp.stage_cnt].mov_dir = axis_temp.stage[axis_temp.stage_cnt-1].mov_dir;
					axis_temp.stage[axis_temp.stage_cnt].loc_diff = (~0);
					axis_temp.stage[axis_temp.stage_cnt].all_segment = (~0);
                    axis_temp.stage[axis_temp.stage_cnt].elapsed_segment_cnt = 0;
					axis_temp.stage[axis_temp.stage_cnt++].t = 0;
				}
			}

			axis_temp.cur_stage = 0;
//			axis_temp.rest_puls_cnt = axis_temp.all_puls_cnt = axis_temp.stage[0].loc_diff;

			if (axis_temp.is_task_running == false) {
				g_Axis_Conf.task_cnt++;
			}


			axis_temp.work_mode_old = axis_temp.work_mode_set;

            axis_temp.tail_flag = 0;										/* 任务结束标志置零*/
            //axis_temp.cur_xifen_num = 0;							/* 细分数清零 */
            axis_temp.reverse_flag = 0;									/* 需要反向标志置零*/
            axis_temp.stage[0].elapsed_segment_cnt = 0;
            axis_temp.stage[1].elapsed_segment_cnt = 0;
            axis_temp.stage[2].elapsed_segment_cnt = 0;
            axis_temp.stage[3].elapsed_segment_cnt = 0;
            axis_temp.stage[4].elapsed_segment_cnt = 0;

			/* 第一段的时间间隔要先算出来 */
			axis_temp.next_segment_pulse_cnt = get_next_segment_pulse_cnt(&axis_temp);

    	    axis_temp.is_task_running = true;									/* 最后再开任务 */

    	    axis_temp.cur_xifen_num         = p_ab_axis->cur_xifen_num;
    	    axis_temp.cur_xifen_pulse_num   = p_ab_axis->cur_xifen_pulse_num;
    	    axis_temp.period_cnt_1_4        = p_ab_axis->period_cnt_1_4;
    	    axis_temp.cur_local             = p_ab_axis->cur_local;

    	    /* 为了防止运动过程中出现直线 */
    	    if (axis_temp.cur_xifen_pulse_num >= axis_temp.next_segment_pulse_cnt) {
    	        axis_temp.cur_xifen_pulse_num = 0;
    	    }

    	    memcpy(p_ab_axis, &axis_temp, sizeof(single_axis_conf_t));
		}

//	GpioDataRegs.GPBDAT.bit.GPIOB2 = 0;

	return 0;
}


/**
 * \brief  A轴cur_local值修正
 *
 * \note A轴需要修正，B轴不需要修正
 * \note 在每次任务开始或结束时修正
 */
#if (BOARD_NAME == BOARD_CX20) ||  (BOARD_NAME == BOARD_DGM_2) ||  (BOARD_NAME == BOARD_DGM_4)	/* 多媒体只有B轴，不需要修正 */
int32 cur_local_modify_A (single_axis_conf_t* p_axis)
{
	int32 cur_local;

	cur_local = p_axis->cur_local % p_axis->max_loc;

	if (cur_local < 0) {
		cur_local += p_axis->max_loc;
	}

	return cur_local;
}
#endif







