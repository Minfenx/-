/*
 * project.h
 *
 *  Created on: 2024��1��16��
 *      Author: Suzkfly
 *
 *      DGM-4 û�д���ģʽ
 *      DGM-2 �д���ģʽ�����Ҵ���ģʽ�±��ֵ���Ϊ0����ɲ��
 */

#ifndef SOURCE_PROJECT_H_
#define SOURCE_PROJECT_H_

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "IQmathLib.h"

#ifndef   TRUE
#define   TRUE              1
#endif

#ifndef   FALSE
#define   FALSE             0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef int8_t
#define int8_t int16
#endif

#ifndef uint8_t
#define uint8_t Uint16
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

/**
 * \brief ����ϸ����
 */
#define XIFEN_CNT  32

/* ����SADE������ID */
#define SADE_PRIMARY    0x55    /* ���� */
#define SADE_BACKUP     0xAA    /* ���� */


/* ������ӵ����֣������ͬ�İ�����һ�ݳ��� */
#define BOARD_CX20      1   /* ����������20��4�ᣨ��壩 */
#define BOARD_DGM_2     2   /* �͸��ܣ�2�ᣨС�壩 */
#define BOARD_DGM_4     3   /* �͸��ܣ�4�ᣬA�� */
#define BOARD_GMS       4   /* ��ý�壬���ᣬC�� */

#define BOARD_NAME  BOARD_GMS

/* ��MRAM�����ж������ */
#define RUN_MRAM

/* ����汾�� */
#define PROGRAM_VERSION 203

/* ����������� */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4)
#define AXIS_CNT    4
#define SADE_PRI_BKUP_ID   SADE_BACKUP      /* �͸���һ���Ǳ��� */
#elif (BOARD_NAME == BOARD_DGM_2)
#define AXIS_CNT    2
#define SADE_PRI_BKUP_ID   SADE_BACKUP      /* �͸���һ���Ǳ��� */
#elif (BOARD_NAME == BOARD_GMS)
#define AXIS_CNT    1

/* ��Ҫ�޸����ݺͱ���ID�����޸���һ�䶨�� */
//#define SADE_PRI_BKUP_ID   SADE_PRIMARY
#define SADE_PRI_BKUP_ID   SADE_BACKUP

/* �����ý����Ӱ汾���Ƕȴ������ӵ�IO��һ�� */
#define GMS_BOARD_OLD    0
#define GMS_BOARD_NEW    1
#define GMS_BOARD_NEW2   2

#define GMS_BOARD_VERSION GMS_BOARD_NEW
#endif

#if GMS_BOARD_VERSION == GMS_BOARD_OLD || GMS_BOARD_VERSION == GMS_BOARD_NEW
//#define RUN_REGION PROGRAM_RUN_ADDR_LOW       /* �͵�ַ */
#define RUN_REGION PROGRAM_RUN_ADDR_HIGH   /* �ߵ�ַ */
#endif

#define PWM_PERIOD			(1125)		// PWM��ʱ������ֵ ,һ��PWM��50us
#define PWM_TICK			(50)		// һ���ж�50us
#define WHOLE_STEPS			(200)


#endif /* SOURCE_PROJECT_H_ */
