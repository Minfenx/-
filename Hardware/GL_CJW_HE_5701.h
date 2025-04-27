/*
 * GL_CJW_HE_5701.h
 *
 *  Created on: 2023��12��13��
 *      Author: Suzkfly
 */

#ifndef INCLUDE_GL_CJW_HE_5701_H_
#define INCLUDE_GL_CJW_HE_5701_H_

#include "project.h"

extern Uint16 g_angle_adj_finish;     /* �Ƕȱ궨��ɵı�־������Ƕȱ궨δ��ɣ��򲻽����˶�����ָ�� */

extern Uint16 g_angle_senser_hw_normal;     /* �Ƕȴ����������ı�־������Ϊ1��������Ϊ0 */

extern Uint16 g_angle_senser_cmd_en;		/* �Ƕȴ�����ָ��ʹ�ܱ�־ */

extern Uint64 g_angle_senser_off_Tick;			/* �ϵ��2min�Ƕȴ������Զ��ر� */

extern Uint16 g_angle_senser_hw_en;				/* Ӳ��ʹ�ܱ�־ */

extern Uint16 g_angle_senser_cmd_eff_flag;	/* �Ƕȴ�����ָ����Ч��־ */

/**
 * \brief ��λ�ƴ�����ʱ���߸�
 */
extern void CJW_HE_5701_SET_CLK (void);

/**
 * \brief ��λ�ƴ�����ʱ���ߵ�
 */
extern void CJW_HE_5701_CLEAR_CLK (void);

/**
 * \brief ��λ�ƴ�������ȡһ��λ
 */
extern Uint16 CJW_HE_5701_Read_Bit (void);

/**
 * \brief �Ƕȴ�������Դ����
 */
extern void CJW_HE_5701_Power_On (void);

/**
 * \brief �Ƕȴ�������Դ����
 */
extern void CJW_HE_5701_Power_Off (void);


/**
 * \brief ��λ�ƴ�������ʼ��
 */
extern void CJW_HE_5701_Init (void);

/**
 * \brief ��λ�ƴ�������ȡ����
 *
 * \param[out] p_value����ȡ�ɹ������ֵ
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
 */
extern int16 CJW_HE_5701_Read_Data (Uint16* p_value);

/**
 * \brief �Ƕȴ������ɼ����ĽǶ�ֵ����Ϊλ��ֵ
 */
extern int32 CJW_HE_5701_to_cur_local (Uint16 CJW_HE_5701_value);

/**
 * \brief �Ƕȴ���������
 */
extern void Angle_Senser_Task (void);

#endif /* INCLUDE_GL_CJW_HE_5701_H_ */
