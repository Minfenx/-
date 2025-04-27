/*
 * sadmb_soft_lim_conf_cmd.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "sadmb_soft_lim_conf_cmd.h"
#include "cmd.h"
#include "tele_dat.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_sadmb_soft_lim_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        g_Axis_Conf.p_pyb->soft_lim = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        g_Axis_Conf.p_nyb->soft_lim = ((Uint32)pData[10] << 24) | ((Uint32)pData[11] << 16) | (pData[12] << 8) | pData[13];

        /* 软件限位值为正数 */
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

    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
//        g_Axis_Conf.p_pyb->soft_lim = ((Uint32)pData[7]  << 24) | ((Uint32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
//        g_Axis_Conf.p_nyb->soft_lim = ((Uint32)pData[11] << 24) | ((Uint32)pData[12] << 16) | (pData[13] << 8) | pData[14];

        soft_lim_pyb_temp = ((Uint32)pData[7]  << 24) | ((Uint32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
        soft_lim_nyb_temp = ((Uint32)pData[11] << 24) | ((Uint32)pData[12] << 16) | (pData[13] << 8) | pData[14];

        /* 软件限位值为正数 */
        if (soft_lim_pyb_temp < 0) {
            soft_lim_pyb_temp = 0 - soft_lim_pyb_temp;
        }

        if (soft_lim_nyb_temp < 0) {
            soft_lim_nyb_temp = 0 - soft_lim_nyb_temp;
        }

        /* 软件限位位置不能超过±180° */
        if ((soft_lim_pyb_temp > 200000) || (soft_lim_nyb_temp > 200000)) {
            return -1;
        }

        /* 当前位置必须在软件限位位置之内 */
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

    if (g_Axis_Conf.p_pyb->is_task_running == 0) { /* 只有任务没有执行时才可进行配置 */
        soft_lim_p_temp = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        soft_lim_n_temp = ((Uint32)pData[10] << 24) | ((Uint32)pData[11] << 16) | (pData[12] << 8) | pData[13];

        /* 软件限位位置不能超过±180° */
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

