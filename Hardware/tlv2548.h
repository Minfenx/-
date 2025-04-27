/*
 * tlc2543.h
 *
 *  Created on: 2024年1月17日
 *      Author: Suzkfly
 */

#ifndef INCLUDE_TLV2548_H_
#define INCLUDE_TLV2548_H_

#include "project.h"

#if BOARD_NAME == BOARD_GMS

/* A相电压值，B相电压值，5V电流值 */
extern float VOL_PYB_A, VOL_PYB_B, VOL_CUR_5V;

/**
 * \brief TLV2548命令
 * @{
 */
#define TLV2548_CMD_SEL_CH0             (0 << 12)   /* 选择模拟输入通道0 */
#define TLV2548_CMD_SEL_CH1             (1 << 12)   /* 选择模拟输入通道1 */
#define TLV2548_CMD_SEL_CH2             (2 << 12)   /* 选择模拟输入通道2 */
#define TLV2548_CMD_SEL_CH3             (3 << 12)   /* 选择模拟输入通道3 */
#define TLV2548_CMD_SEL_CH4             (4 << 12)   /* 选择模拟输入通道4 */
#define TLV2548_CMD_SEL_CH5             (5 << 12)   /* 选择模拟输入通道5 */
#define TLV2548_CMD_SEL_CH6             (6 << 12)   /* 选择模拟输入通道6 */
#define TLV2548_CMD_SEL_CH7             (7 << 12)   /* 选择模拟输入通道7 */
#define TLV2548_CMD_SW_POWER_DOWN       (8 << 12)   /* 掉电 */
#define TLV2548_CMD_READ_CFR            (9 << 12)   /* 读CFR */
#define TLV2548_CMD_WRITE_CFR           (10 << 12)  /* 写CFR */
#define TLV2548_CMD_TEST_REFP_REFM_2    (11 << 12)  /* 测试 */
#define TLV2548_CMD_TEST_REFM           (12 << 12)  /* 测试 */
#define TLV2548_CMD_TEST_REFP           (13 << 12)  /* 测试 */
#define TLV2548_CMD_READ_FIFO           (14 << 12)  /* 读FIFO，SDO高12位为有效数据 */
/**
 * @}
 */


/** 0xA000：
 * [b15:b12]:1010：写CFR寄存器
 * b11:0:外部基准源，1：外部基准源
 * b10:0:内部4V基准，1：内部2V基准源
 * b9:采样周期。0：短采样，12个时钟；1：长采样，24个时钟
 * [b8:b7]采样时钟选择：00：内部时钟，采样时间2.33~3.86us    01：SCLK  10:SCLK/4  11：SCLK/2
 * [b6:b5]转换模式选择： 00：单次模式，不使用FIFO，[b1:b0]无效   01:重复模式  10：扫描模式  11：重复扫描模式
 * [b4:b3]扫描序列选择：
 * b2：EOC/INT引脚功能选择：0：INT  1:EOC
 * [b1:b0]FIFO触发深度
 *
 * 外部基准 , 采样周期12SCLK ,转换时钟为SCLK , 单次转换   ;  0xA800内部4V基准 , 采样周期12SCLK ,内部时钟源 , 单次转换*/
#define ADC_MODE    0xA000 //0xAC00    //0xA080

/* 定义参考电压，使用外部参考源有效 */
#define REF_VOL 3.450

/**
 * \brief TLV2548初始化
 */
extern void TLV2548_Init (void);

/**
 * \ADC采集任务
 */
extern void ADC_task (void);

#endif

#endif /* INCLUDE_TLV2548_H_ */
