/*
 * sade_drv_hot_bkup_conf_cmd.c
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */
#if 0
#include "sade_drv_hot_bkup_conf_cmd.h"
#include "cmd.h"

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t sade_drv_hot_bkup_handler (const Uint16 *pData)
{
	int8_t i = 0;
	single_axis_conf_t* p_axis = 0;

    if (g_Axis_Conf.task_cnt == 0) { 		/* ֻ������û��ִ��ʱ�ſɽ������� */
    	/* ������ݵ���Ч�� */
    	if (((pData[6] != 0) && (pData[6] != 0x55) && (pData[6] != 0xAA)) ||
    		((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
			((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA)) ||
			((pData[9] != 0) && (pData[9] != 0x55) && (pData[9] != 0xAA))) {
    		return -1;
    	}

    	/* ����Ϊ0��ʾ�������� */
    	p_axis = g_Axis_Conf.p_pya;
    	for (i = 0; i < 4; i++) {
			if ((pData[6 + i] != 0) && (p_axis->drv_conf != pData[6 + i])) {
				p_axis->drv_conf = pData[6 + i];
				if (p_axis->drv_conf == 0x55) {	/* �л�����·1 */
					p_axis->fn_s_disable();		/* ���ݽ��� */
					p_axis->fn_m_enable();		/* ����ʹ�� */
				} else if (p_axis->drv_conf == 0xAA) {	/* �л�����·2 */
					p_axis->fn_m_disable();		/* ���ݽ��� */
					p_axis->fn_s_enable();		/* ����ʹ�� */
				}
			}
			p_axis = p_axis->p_next_axis;
    	}

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int8_t sade_drv_hot_bkup_handler (const Uint16 *pData)
{
    int8_t i = 0;
    single_axis_conf_t* p_axis = 0;

    if (g_Axis_Conf.task_cnt == 0) {        /* ֻ������û��ִ��ʱ�ſɽ������� */
        /* ������ݵ���Ч�� */
        if (((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
            ((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA))) {
            return -1;
        }

        /* ����Ϊ0��ʾ�������� */
        p_axis = g_Axis_Conf.p_pya;
        for (i = 0; i < AXIS_CNT; i++) {
            if ((pData[7 + i] != 0) && (p_axis->drv_conf != pData[7 + i])) {
                p_axis->drv_conf = pData[7 + i];
                if (p_axis->drv_conf == 0x55) { /* �л�����·1 */
                    p_axis->fn_s_disable();     /* ���ݽ��� */
                    p_axis->fn_m_enable();      /* ����ʹ�� */
                } else if (p_axis->drv_conf == 0xAA) {  /* �л�����·2 */
                    p_axis->fn_m_disable();     /* ���ݽ��� */
                    p_axis->fn_s_enable();      /* ����ʹ�� */
                }
            }
            p_axis = p_axis->p_next_axis;
        }

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t sade_drv_hot_bkup_handler (const Uint16 *pData)
{
    int8_t i = 0;
    single_axis_conf_t* p_axis = 0;

    if (g_Axis_Conf.task_cnt == 0) {        /* ֻ������û��ִ��ʱ�ſɽ������� */
        /* ������ݵ���Ч�� */
        if (((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
            ((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA)) ||
            ((pData[9] != 0) && (pData[9] != 0x55) && (pData[9] != 0xAA)) ||
            ((pData[10] != 0) && (pData[10] != 0x55) && (pData[10] != 0xAA))) {
            return -1;
        }

        /* ����Ϊ0��ʾ�������� */
        p_axis = g_Axis_Conf.p_pya;
        for (i = 0; i < AXIS_CNT; i++) {
            if ((pData[7 + i] != 0) && (p_axis->drv_conf != pData[7 + i])) {
                p_axis->drv_conf = pData[7 + i];
                if (p_axis->drv_conf == 0x55) { /* �л�����·1 */
                    p_axis->fn_s_disable();     /* ���ݽ��� */
                    p_axis->fn_m_enable();      /* ����ʹ�� */
                } else if (p_axis->drv_conf == 0xAA) {  /* �л�����·2 */
                    p_axis->fn_m_disable();     /* ���ݽ��� */
                    p_axis->fn_s_enable();      /* ����ʹ�� */
                }
            }
            p_axis = p_axis->p_next_axis;
        }

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t sade_drv_hot_bkup_handler (const Uint16 *pData)
{
//    single_axis_conf_t* p_axis = g_Axis_Conf.p_pyb;
//
//    if (g_Axis_Conf.task_cnt == 0) {        /* ֻ������û��ִ��ʱ�ſɽ������� */
//        /* ������ݵ���Ч�� */
//        if ((pData[6] != 0) && (pData[6] != 0x55) && (pData[6] != 0xAA)) {
//            return -1;
//        }
//
//        /* ����Ϊ0��ʾ�������� */
//        if ((pData[6] != 0) && (p_axis->drv_conf != pData[6])) {
//            p_axis->drv_conf = pData[6];
//            if (p_axis->drv_conf == 0x55) { /* �л�����·1 */
//                p_axis->fn_s_disable();     /* ���ݽ��� */
//                p_axis->fn_m_enable();      /* ����ʹ�� */
//            } else if (p_axis->drv_conf == 0xAA) {  /* �л�����·2 */
//                p_axis->fn_m_disable();     /* ���ݽ��� */
//                p_axis->fn_s_enable();      /* ����ʹ�� */
//            }
//        }
//
//        return 0;
//    }
    return -1;
}
#endif
#endif

