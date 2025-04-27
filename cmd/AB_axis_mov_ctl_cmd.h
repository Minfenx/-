/*
 * AB_axis_mov_ctl_cmd.h
 *
 *  Created on: 2023年5月29日
 *      Author: mj
 */

#ifndef SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_
#define SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_

#include "project.h"				//全局宏
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "cmd.h"

extern uint8_t g_force_hold_flag;		/* 强制停止标志 */

extern uint8_t g_angle_sensor_task_req;	/* 角度传感器任务请求，为0表示无任无，为1表示有任务 */
extern uint8_t g_angle_sensor_sw_req;		/* 角度传感器开关，为0表示关，为1表示开 */

extern void CJW_HE_5701_sw_req_task (void);

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
extern int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData);

/**
 * \brief 得到下一个微步中的脉冲数量
 *
 * \note 为了节省计算量，将上次的计算结果t2用static类型保存，实际使用的时候elapsed_pulse_cnt必须只比上一次多1
 */
extern Uint32 get_next_segment_pulse_cnt (single_axis_conf_t* p_axis);

/**
 * \brief 执行AB轴运动控制指令
 *
 * \retval 成功返回0，失败返回-1
 */
extern int16 ab_axis_mov_ctl_cmd_run( single_axis_conf_t* p_ab_axis );

/**
 * \brief 根据占空比和电流系数计算pwm的比较寄存器的值
 *
 * \param[in] p_axis：需要配置的轴
 * \param[in] p_pwm：无
 */
void duty_to_cycle(single_axis_conf_t* p_axis);
/**
 * \brief  A轴cur_local值修正
 *
 * \note A轴需要修正，B轴不需要修正
 * \note 在每次任务开始或结束时修正
 */
extern int32 cur_local_modify_A (single_axis_conf_t* p_axis);


#endif /* SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_ */
