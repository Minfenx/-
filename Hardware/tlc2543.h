/*
 * tlc2543.h
 *
 *  Created on: 2024��3��29��
 *      Author: Suzkfly
 */

#ifndef SOURCE_HARDWARE_TLC2543_H_
#define SOURCE_HARDWARE_TLC2543_H_

#include "project.h"

#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
/**
 * \brief TLV2548����
 * @{
 */
#define TLC2543_CMD_SEL_PYA_A             (0)   /* ѡ��ģ������ͨ��0 */
#define TLC2543_CMD_SEL_PYA_B             (1)   /* ѡ��ģ������ͨ��1 */
#define TLC2543_CMD_SEL_PYB_A             (2)   /* ѡ��ģ������ͨ��2 */
#define TLC2543_CMD_SEL_PYB_B             (3)   /* ѡ��ģ������ͨ��3 */
#define TLC2543_CMD_SEL_NYA_A             (4)   /* ѡ��ģ������ͨ��4 */
#define TLC2543_CMD_SEL_NYA_B             (5)   /* ѡ��ģ������ͨ��5 */
#define TLC2543_CMD_SEL_NYB_A             (6)   /* ѡ��ģ������ͨ��6 */
#define TLC2543_CMD_SEL_NYB_B             (7)   /* ѡ��ģ������ͨ��7 */
#define TLC2543_CMD_SEL_SELCT             (8)   /* ѡ��ģ������ͨ��8(��·ѡ��) */

/**
 * \brief ��·ѡ��
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

/* ����ο���ѹ��ʹ���ⲿ�ο�Դ��Ч */
#define REF_VOL 5

#define CH_CNT (14)
#define SAMPLE_CNT 		(15)
#define THRESHOLD 		(0.01)    	// ��ֵ���������ݴ˲�ֵ��10^-4��10^-5��
#define ABNORMAL_CNT 	(5)     	// �쳣ֵ����ͳ��
typedef struct {
    int sample_ok; 					// �����Ƿ������ı�־λ
    int index; 						// ��������
    float sample[SAMPLE_CNT]; 		// ��������
    float sum; 						// ����ֵ���ܺ�
    float average; 					//ƽ��ֵ
    float value;					//ʵ��ֵ��������Բ鿴
    float difference;				//��ֵ��ƽ��ֵ֮���������ֵ�Ƚϣ�
    int abnormal;					//�쳣
} average_t;

/* �����������λ�� */
#define OUTPUT_DATA_LEN_8_BITS       (1 << 2)
#define OUTPUT_DATA_LEN_12_BITS      (2 << 2)
#define OUTPUT_DATA_LEN_16_BITS      (3 << 2)

/* �������ݸ�ʽ */
#define OUTPUT_DATA_MSBF    (0 << 1)    /* ��λ�ȴ� */
#define OUTPUT_DATA_LSBF    (1 << 1)    /* ��λ�ȴ� */

/* ���弫�� */
#define UNIPOLAR        0   /* ������ */
#define BIPOLAR         1   /* ˫���� */

/**
 * \brief ��λ�Ƶ�ѹת��Ϊ�Ƕ�
 */
extern float32 Vol_To_Angle (char axis, char type, uint8_t isAdjust);

/**
 * \brief TLC2543��ʼ��
 */
extern void TLC2543_Init (void);

/**
 * \ADC�ɼ�����
 */
extern void ADC_task (void);

/**
 * \brief TLC2548 ADת��
 *
 * \param[in] ch��Ҫת����ͨ����0~10
 */
extern void TLC2548_Convert (Uint16 ch);

extern void __Abnormal_Cnt (Uint16 ch, float value);

extern int16 __Filter_Average (Uint16 ch, float value, float* p_data);
#endif

#endif /* SOURCE_HARDWARE_TLC2543_H_ */
