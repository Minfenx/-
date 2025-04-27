/*
 * GL_CJW_HE_5701.h
 *
 *  Created on: 2023年12月13日
 *      Author: Suzkfly
 */

#ifndef INCLUDE_GL_CJW_HE_5701_H_
#define INCLUDE_GL_CJW_HE_5701_H_

#include "project.h"

extern Uint16 g_angle_adj_finish;     /* 角度标定完成的标志，如果角度标定未完成，则不接收运动控制指令 */

extern Uint16 g_angle_senser_hw_normal;     /* 角度传感器正常的标志，正常为1，不正常为0 */

extern Uint16 g_angle_senser_cmd_en;		/* 角度传感器指令使能标志 */

extern Uint64 g_angle_senser_off_Tick;			/* 上电后2min角度传感器自动关闭 */

extern Uint16 g_angle_senser_hw_en;				/* 硬件使能标志 */

extern Uint16 g_angle_senser_cmd_eff_flag;	/* 角度传感器指令有效标志 */

/**
 * \brief 角位移传感器时钟线高
 */
extern void CJW_HE_5701_SET_CLK (void);

/**
 * \brief 角位移传感器时钟线低
 */
extern void CJW_HE_5701_CLEAR_CLK (void);

/**
 * \brief 角位移传感器读取一个位
 */
extern Uint16 CJW_HE_5701_Read_Bit (void);

/**
 * \brief 角度传感器电源供电
 */
extern void CJW_HE_5701_Power_On (void);

/**
 * \brief 角度传感器电源供电
 */
extern void CJW_HE_5701_Power_Off (void);


/**
 * \brief 角位移传感器初始化
 */
extern void CJW_HE_5701_Init (void);

/**
 * \brief 角位移传感器读取数据
 *
 * \param[out] p_value：读取成功后填充值
 *
 * \retval 成功返回0，失败返回-1
 */
extern int16 CJW_HE_5701_Read_Data (Uint16* p_value);

/**
 * \brief 角度传感器采集到的角度值换算为位置值
 */
extern int32 CJW_HE_5701_to_cur_local (Uint16 CJW_HE_5701_value);

/**
 * \brief 角度传感器任务
 */
extern void Angle_Senser_Task (void);

#endif /* INCLUDE_GL_CJW_HE_5701_H_ */
