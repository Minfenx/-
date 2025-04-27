/*
 * cycle_hall.h
 *
 *  Created on: 2024年2月27日
 *      Author: Suzkfly
 */

#ifndef INCLUDE_CYCLE_HALL_H_
#define INCLUDE_CYCLE_HALL_H_

#include "project.h"

/**< \brief 定义霍尔的脉冲数 */
extern float g_hall_angle;

/**
 * \brief 霍尔初始化
 */
extern void Hall_Init (void);

/**
 * \brief 更新霍尔角度
 */
extern void Hall_angle_updat (void);

extern void Cycle_Hall_Task(void);


#endif /* INCLUDE_CYCLE_HALL_H_ */
