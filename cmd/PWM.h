/*
 * ePWM.h
 *
 *  Created on: 2023��8��10��
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_PWM_H_
#define SOURCE_CMD_PWM_H_

#include "project.h"     // DSP2833x Headerfile Include File
#include "cmd.h"

/* �����˹��±����ı�־ */
#if BOARD_NAME == BOARD_GMS
extern Uint16 g_pdpinta_pyb_flag;
#elif BOARD_NAME == BOARD_DGM_2
extern Uint16 g_pdpinta_pya_flag;
extern Uint16 g_pdpinta_nya_flag;
#elif BOARD_NAME == BOARD_DGM_4 ||BOARD_NAME == BOARD_CX20
extern Uint16 g_pdpinta_pya_flag;
extern Uint16 g_pdpinta_nya_flag;
extern Uint16 g_pdpinta_pyb_flag;
extern Uint16 g_pdpinta_nyb_flag;
#endif

/**
 * \brief ����PWM״̬������ÿ�������ϵ�֮�󣬽�PWM��ԭ���ϴζϵ�ʱ��״̬
 */
typedef struct {
	int16 pha_A;	/* A��״̬ */
	int16 pha_B;	/* B��״̬ */
} pwm_stat_t;

extern void InitEV_PWM(void);
extern void __pwm_handler(single_axis_conf_t* p_axis , uint8_t ch);
extern void PWM_task (void);

/**
 * \brief ������Ƴ�ʼ��
 */
extern void Motor_Control_Init (void);

/**
 * \brief ������������
 */
extern void Over_Cur_Task (void);

#endif /* SOURCE_CMD_PWM_H_ */



