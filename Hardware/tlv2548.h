/*
 * tlc2543.h
 *
 *  Created on: 2024��1��17��
 *      Author: Suzkfly
 */

#ifndef INCLUDE_TLV2548_H_
#define INCLUDE_TLV2548_H_

#include "project.h"

#if BOARD_NAME == BOARD_GMS

/* A���ѹֵ��B���ѹֵ��5V����ֵ */
extern float VOL_PYB_A, VOL_PYB_B, VOL_CUR_5V;

/**
 * \brief TLV2548����
 * @{
 */
#define TLV2548_CMD_SEL_CH0             (0 << 12)   /* ѡ��ģ������ͨ��0 */
#define TLV2548_CMD_SEL_CH1             (1 << 12)   /* ѡ��ģ������ͨ��1 */
#define TLV2548_CMD_SEL_CH2             (2 << 12)   /* ѡ��ģ������ͨ��2 */
#define TLV2548_CMD_SEL_CH3             (3 << 12)   /* ѡ��ģ������ͨ��3 */
#define TLV2548_CMD_SEL_CH4             (4 << 12)   /* ѡ��ģ������ͨ��4 */
#define TLV2548_CMD_SEL_CH5             (5 << 12)   /* ѡ��ģ������ͨ��5 */
#define TLV2548_CMD_SEL_CH6             (6 << 12)   /* ѡ��ģ������ͨ��6 */
#define TLV2548_CMD_SEL_CH7             (7 << 12)   /* ѡ��ģ������ͨ��7 */
#define TLV2548_CMD_SW_POWER_DOWN       (8 << 12)   /* ���� */
#define TLV2548_CMD_READ_CFR            (9 << 12)   /* ��CFR */
#define TLV2548_CMD_WRITE_CFR           (10 << 12)  /* дCFR */
#define TLV2548_CMD_TEST_REFP_REFM_2    (11 << 12)  /* ���� */
#define TLV2548_CMD_TEST_REFM           (12 << 12)  /* ���� */
#define TLV2548_CMD_TEST_REFP           (13 << 12)  /* ���� */
#define TLV2548_CMD_READ_FIFO           (14 << 12)  /* ��FIFO��SDO��12λΪ��Ч���� */
/**
 * @}
 */


/** 0xA000��
 * [b15:b12]:1010��дCFR�Ĵ���
 * b11:0:�ⲿ��׼Դ��1���ⲿ��׼Դ
 * b10:0:�ڲ�4V��׼��1���ڲ�2V��׼Դ
 * b9:�������ڡ�0���̲�����12��ʱ�ӣ�1����������24��ʱ��
 * [b8:b7]����ʱ��ѡ��00���ڲ�ʱ�ӣ�����ʱ��2.33~3.86us    01��SCLK  10:SCLK/4  11��SCLK/2
 * [b6:b5]ת��ģʽѡ�� 00������ģʽ����ʹ��FIFO��[b1:b0]��Ч   01:�ظ�ģʽ  10��ɨ��ģʽ  11���ظ�ɨ��ģʽ
 * [b4:b3]ɨ������ѡ��
 * b2��EOC/INT���Ź���ѡ��0��INT  1:EOC
 * [b1:b0]FIFO�������
 *
 * �ⲿ��׼ , ��������12SCLK ,ת��ʱ��ΪSCLK , ����ת��   ;  0xA800�ڲ�4V��׼ , ��������12SCLK ,�ڲ�ʱ��Դ , ����ת��*/
#define ADC_MODE    0xA000 //0xAC00    //0xA080

/* ����ο���ѹ��ʹ���ⲿ�ο�Դ��Ч */
#define REF_VOL 3.450

/**
 * \brief TLV2548��ʼ��
 */
extern void TLV2548_Init (void);

/**
 * \ADC�ɼ�����
 */
extern void ADC_task (void);

#endif

#endif /* INCLUDE_TLV2548_H_ */
