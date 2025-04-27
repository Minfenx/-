/*
 * project.h
 *
 *  Created on: 2024年1月16日
 *      Author: Suzkfly
 *
 *      DGM-4 没有待机模式
 *      DGM-2 有待机模式，并且待机模式下保持电流为0，不刹车
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
 * \brief 定义细分数
 */
#define XIFEN_CNT  32

/* 定义SADE主备份ID */
#define SADE_PRIMARY    0x55    /* 主份 */
#define SADE_BACKUP     0xAA    /* 备份 */


/* 定义板子的名字，多个不同的板子用一份程序 */
#define BOARD_CX20      1   /* 星网，创新20，4轴（大板） */
#define BOARD_DGM_2     2   /* 低高密，2轴（小板） */
#define BOARD_DGM_4     3   /* 低高密，4轴，A星 */
#define BOARD_GMS       4   /* 多媒体，单轴，C星 */

#define BOARD_NAME  BOARD_GMS

/* 在MRAM中运行定义此条 */
#define RUN_MRAM

/* 程序版本号 */
#define PROGRAM_VERSION 203

/* 定义轴的数量 */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4)
#define AXIS_CNT    4
#define SADE_PRI_BKUP_ID   SADE_BACKUP      /* 低高密一定是备份 */
#elif (BOARD_NAME == BOARD_DGM_2)
#define AXIS_CNT    2
#define SADE_PRI_BKUP_ID   SADE_BACKUP      /* 低高密一定是备份 */
#elif (BOARD_NAME == BOARD_GMS)
#define AXIS_CNT    1

/* 若要修改主份和备份ID，请修改下一句定义 */
//#define SADE_PRI_BKUP_ID   SADE_PRIMARY
#define SADE_PRI_BKUP_ID   SADE_BACKUP

/* 定义多媒体板子版本，角度传感器接的IO不一样 */
#define GMS_BOARD_OLD    0
#define GMS_BOARD_NEW    1
#define GMS_BOARD_NEW2   2

#define GMS_BOARD_VERSION GMS_BOARD_NEW
#endif

#if GMS_BOARD_VERSION == GMS_BOARD_OLD || GMS_BOARD_VERSION == GMS_BOARD_NEW
//#define RUN_REGION PROGRAM_RUN_ADDR_LOW       /* 低地址 */
#define RUN_REGION PROGRAM_RUN_ADDR_HIGH   /* 高地址 */
#endif

#define PWM_PERIOD			(1125)		// PWM定时器周期值 ,一个PWM波50us
#define PWM_TICK			(50)		// 一次中断50us
#define WHOLE_STEPS			(200)


#endif /* SOURCE_PROJECT_H_ */
