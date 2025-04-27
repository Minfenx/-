/*
 * sadmb_soft_lim_conf_cmd.c
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */

#include "sadmb_soft_lim_conf_cmd.h"
#include "cmd.h"
#include "tele_dat.h"

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_sadmb_soft_lim_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* ֻ������û��ִ��ʱ�ſɽ������� */
        g_Axis_Conf.p_pyb->soft_lim = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        g_Axis_Conf.p_nyb->soft_lim = ((Uint32)pData[10] << 24) | ((Uint32)pData[11] << 16) | (pData[12] << 8) | pData[13];

        /* �����λֵΪ���� */
        if (g_Axis_Conf.p_pyb->soft_lim < 0) {
        	g_Axis_Conf.p_pyb->soft_lim = -g_Axis_Conf.p_pyb->soft_lim;
        }

        if (g_Axis_Conf.p_nyb->soft_lim < 0) {
        	g_Axis_Conf.p_nyb->soft_lim = -g_Axis_Conf.p_nyb->soft_lim;
        }

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t get_sadmb_soft_lim_conf_dat (const Uint16 *pData)
{
    int32 soft_lim_pyb_temp, soft_lim_nyb_temp;

    if (g_Axis_Conf.task_cnt == 0) { /* ֻ������û��ִ��ʱ�ſɽ������� */
//        g_Axis_Conf.p_pyb->soft_lim = ((Uint32)pData[7]  << 24) | ((Uint32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
//        g_Axis_Conf.p_nyb->soft_lim = ((Uint32)pData[11] << 24) | ((Uint32)pData[12] << 16) | (pData[13] << 8) | pData[14];

        soft_lim_pyb_temp = ((Uint32)pData[7]  << 24) | ((Uint32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
        soft_lim_nyb_temp = ((Uint32)pData[11] << 24) | ((Uint32)pData[12] << 16) | (pData[13] << 8) | pData[14];

        /* �����λֵΪ���� */
        if (soft_lim_pyb_temp < 0) {
            soft_lim_pyb_temp = 0 - soft_lim_pyb_temp;
        }

        if (soft_lim_nyb_temp < 0) {
            soft_lim_nyb_temp = 0 - soft_lim_nyb_temp;
        }

        /* �����λλ�ò��ܳ�����180�� */
        if ((soft_lim_pyb_temp > 200000) || (soft_lim_nyb_temp > 200000)) {
            return -1;
        }

        /* ��ǰλ�ñ����������λλ��֮�� */
        if ((g_Axis_Conf.p_pyb->cur_local > 0 - soft_lim_pyb_temp) && (g_Axis_Conf.p_pyb->cur_local < soft_lim_pyb_temp) &&
            (g_Axis_Conf.p_nyb->cur_local > 0 - soft_lim_nyb_temp) && (g_Axis_Conf.p_nyb->cur_local < soft_lim_nyb_temp)) {

            g_Axis_Conf.p_pyb->soft_lim = soft_lim_pyb_temp;
            g_Axis_Conf.p_nyb->soft_lim = soft_lim_nyb_temp;

            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t get_sadmb_soft_lim_conf_dat (const Uint16 *pData)
{
    int32 soft_lim_p_temp, soft_lim_n_temp;

    if (g_Axis_Conf.p_pyb->is_task_running == 0) { /* ֻ������û��ִ��ʱ�ſɽ������� */
        soft_lim_p_temp = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        soft_lim_n_temp = ((Uint32)pData[10] << 24) | ((Uint32)pData[11] << 16) | (pData[12] << 8) | pData[13];

        /* �����λλ�ò��ܳ�����180�� */
        if ((soft_lim_p_temp > 400000) || (soft_lim_n_temp < -400000)) {
            return -1;
        }

        if ((g_Axis_Conf.p_pyb->cur_local <= soft_lim_p_temp) && (g_Axis_Conf.p_pyb->cur_local >= soft_lim_n_temp)) {
            g_Axis_Conf.p_pyb->soft_lim_p = soft_lim_p_temp;
            g_Axis_Conf.p_pyb->soft_lim_n = soft_lim_n_temp;

            g_Tele_Dat.soft_lim_err_stat = 0;

            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#endif

