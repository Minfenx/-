/*
 * tlc2543.h
 *
 *  Created on: 2024年3月29日
 *      Author: Suzkfly
 */

#ifndef SOURCE_HARDWARE_TLC2543_H_
#define SOURCE_HARDWARE_TLC2543_H_

#include "project.h"

#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
/**
 * \brief TLV2548命令
 * @{
 */
#define TLC2543_CMD_SEL_PYA_A             (0)   /* 选择模拟输入通道0 */
#define TLC2543_CMD_SEL_PYA_B             (1)   /* 选择模拟输入通道1 */
#define TLC2543_CMD_SEL_PYB_A             (2)   /* 选择模拟输入通道2 */
#define TLC2543_CMD_SEL_PYB_B             (3)   /* 选择模拟输入通道3 */
#define TLC2543_CMD_SEL_NYA_A             (4)   /* 选择模拟输入通道4 */
#define TLC2543_CMD_SEL_NYA_B             (5)   /* 选择模拟输入通道5 */
#define TLC2543_CMD_SEL_NYB_A             (6)   /* 选择模拟输入通道6 */
#define TLC2543_CMD_SEL_NYB_B             (7)   /* 选择模拟输入通道7 */
#define TLC2543_CMD_SEL_SELCT             (8)   /* 选择模拟输入通道8(多路选择) */

/**
 * \brief 多路选择
 * @{
 */
#define JHSR1840_NY_TEMPIN3             (1)
#define JHSR1840_NY_TEMPIN2             (2)
#define JHSR1840_NY_TEMPIN1             (3)
#define JHSR1840_PY_TEMPIN3             (5)
#define JHSR1840_PY_TEMPIN2             (6)
#define JHSR1840_PY_TEMPIN1             (7)

#define JHSR1840_PYB_POTIN3             (8)
#define JHSR1840_PYA_POTIN2             (9)
#define JHSR1840_PYA_POTIN1             (10)
#define JHSR1840_NYB_POTIN3             (11)
#define JHSR1840_NYA_POTIN2             (12)
#define JHSR1840_NYA_POTIN1             (13)
#define JHSR1840_TEST12V             	(14)
#define JHSR1840_TEST28V             	(15)

#define JHSR1840_CH_NUM 				 (15)

extern float Y_TEMP1_fu,Y_TEMP2_fu,Y_TEMP3_fu,Y_POT3_fu,Y_POT2_fu,Y_POT1_fu,TEST_28V,TEST_12V,
      Y_POT1_zhen,Y_POT2_zhen,Y_POT3_zhen,Y_TEMP1_zhen,Y_TEMP2_zhen,Y_TEMP3_zhen,
      GND1, GND2;
extern float VOL_PYA_A, VOL_PYA_B, VOL_NYA_A, VOL_NYA_B;

/* 定义参考电压，使用外部参考源有效 */
#define REF_VOL 5

#define CH_CNT (14)
#define SAMPLE_CNT 		(15)
#define THRESHOLD 		(0.01)    	// 阈值（正常数据此差值在10^-4到10^-5）
#define ABNORMAL_CNT 	(5)     	// 异常值数量统计
typedef struct {
    int sample_ok; 					// 样本是否已满的标志位
    int index; 						// 样本索引
    float sample[SAMPLE_CNT]; 		// 样本数组
    float sum; 						// 样本值的总和
    float average; 					//平均值
    float value;					//实际值，方便调试查看
    float difference;				//新值与平均值之差（用来与阈值比较）
    int abnormal;					//异常
} average_t;

/* 定义输出数据位数 */
#define OUTPUT_DATA_LEN_8_BITS       (1 << 2)
#define OUTPUT_DATA_LEN_12_BITS      (2 << 2)
#define OUTPUT_DATA_LEN_16_BITS      (3 << 2)

/* 定义数据格式 */
#define OUTPUT_DATA_MSBF    (0 << 1)    /* 高位先传 */
#define OUTPUT_DATA_LSBF    (1 << 1)    /* 低位先传 */

/* 定义极性 */
#define UNIPOLAR        0   /* 单极性 */
#define BIPOLAR         1   /* 双极性 */

/**
 * \brief 电位计电压转换为角度
 */
extern float32 Vol_To_Angle (char axis, char type, uint8_t isAdjust);

/**
 * \brief TLC2543初始化
 */
extern void TLC2543_Init (void);

/**
 * \ADC采集任务
 */
extern void ADC_task (void);

/**
 * \brief TLC2548 AD转换
 *
 * \param[in] ch：要转换的通道，0~10
 */
extern void TLC2548_Convert (Uint16 ch);

extern void __Abnormal_Cnt (Uint16 ch, float value);

extern int16 __Filter_Average (Uint16 ch, float value, float* p_data);
#endif

#endif /* SOURCE_HARDWARE_TLC2543_H_ */
