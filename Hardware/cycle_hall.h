/*
 * cycle_hall.h
 *
 *  Created on: 2024��2��27��
 *      Author: Suzkfly
 */

#ifndef INCLUDE_CYCLE_HALL_H_
#define INCLUDE_CYCLE_HALL_H_

#include "project.h"

/**< \brief ��������������� */
extern float g_hall_angle;

/**
 * \brief ������ʼ��
 */
extern void Hall_Init (void);

/**
 * \brief ���»����Ƕ�
 */
extern void Hall_angle_updat (void);

extern void Cycle_Hall_Task(void);


#endif /* INCLUDE_CYCLE_HALL_H_ */
