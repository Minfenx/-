/*
 * AB_axis_mov_ctl_cmd.h
 *
 *  Created on: 2023��5��29��
 *      Author: mj
 */

#ifndef SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_
#define SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_

#include "project.h"				//ȫ�ֺ�
#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "cmd.h"

extern uint8_t g_force_hold_flag;		/* ǿ��ֹͣ��־ */

extern uint8_t g_angle_sensor_task_req;	/* �Ƕȴ�������������Ϊ0��ʾ�����ޣ�Ϊ1��ʾ������ */
extern uint8_t g_angle_sensor_sw_req;		/* �Ƕȴ��������أ�Ϊ0��ʾ�أ�Ϊ1��ʾ�� */

extern void CJW_HE_5701_sw_req_task (void);

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
extern int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData);

/**
 * \brief �õ���һ��΢���е���������
 *
 * \note Ϊ�˽�ʡ�����������ϴεļ�����t2��static���ͱ��棬ʵ��ʹ�õ�ʱ��elapsed_pulse_cnt����ֻ����һ�ζ�1
 */
extern Uint32 get_next_segment_pulse_cnt (single_axis_conf_t* p_axis);

/**
 * \brief ִ��AB���˶�����ָ��
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
 */
extern int16 ab_axis_mov_ctl_cmd_run( single_axis_conf_t* p_ab_axis );

/**
 * \brief ����ռ�ձȺ͵���ϵ������pwm�ıȽϼĴ�����ֵ
 *
 * \param[in] p_axis����Ҫ���õ���
 * \param[in] p_pwm����
 */
void duty_to_cycle(single_axis_conf_t* p_axis);
/**
 * \brief  A��cur_localֵ����
 *
 * \note A����Ҫ������B�᲻��Ҫ����
 * \note ��ÿ������ʼ�����ʱ����
 */
extern int32 cur_local_modify_A (single_axis_conf_t* p_axis);


#endif /* SOURCE_CMD_AB_AXIS_MOV_CTL_CMD_H_ */
