/*
 * ePWM.h
 *
 *  Created on: 2023年8月10日
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_PWM_H_
#define SOURCE_CMD_PWM_H_

#include "project.h"     // DSP2833x Headerfile Include File
#include "cmd.h"

/* 进入了过温保护的标志 */
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
 * \brief 保存PWM状态，用于每次重新上电之后，将PWM还原到上次断电时的状态
 */
typedef struct {
	int16 pha_A;	/* A相状态 */
	int16 pha_B;	/* B相状态 */
} pwm_stat_t;

extern void InitEV_PWM(void);
extern void __pwm_handler(single_axis_conf_t* p_axis , uint8_t ch);
extern void PWM_task (void);

/**
 * \brief 电机控制初始化
 */
extern void Motor_Control_Init (void);

/**
 * \brief 过流保护任务
 */
extern void Over_Cur_Task (void);

#endif /* SOURCE_CMD_PWM_H_ */



