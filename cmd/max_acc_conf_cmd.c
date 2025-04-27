/*
 * max_acc_conf_cmd.c
 *
 *  Created on: 2023年5月29日
 *      Author: Suzkfly
 */

#include "max_acc_conf_cmd.h"
#include "cmd.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 *
 * \retval 成功返回0，失败返回-1
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_max_acc_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        if ((pData[6] >= 1) && (pData[6] <= 200) &&
            (pData[7] >= 1) && (pData[7] <= 200) &&
            (pData[8] >= 1) && (pData[8] <= 200) &&
            (pData[9] >= 1) && (pData[9] <= 200)) {
            g_Axis_Conf.p_pya->max_acc = pData[6];
            g_Axis_Conf.p_nya->max_acc = pData[7];
            g_Axis_Conf.p_pyb->max_acc = pData[8];
            g_Axis_Conf.p_nyb->max_acc = pData[9];

            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int8_t get_max_acc_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        if ((pData[7] >= 1) && (pData[7] <= 20) &&
            (pData[8] >= 1) && (pData[8] <= 20)) {
            g_Axis_Conf.p_pya->max_acc = pData[7];
            g_Axis_Conf.p_nya->max_acc = pData[8];

            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t get_max_acc_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        if ((pData[7] >= 1) && (pData[7] <= 20) &&
            (pData[8] >= 1) && (pData[8] <= 20) &&
            (pData[9] >= 1) && (pData[9] <= 20) &&
            (pData[10] >= 1) && (pData[10] <= 20)) {
            g_Axis_Conf.p_pya->max_acc = pData[7];
            g_Axis_Conf.p_nya->max_acc = pData[8];
            g_Axis_Conf.p_pyb->max_acc = pData[9];
            g_Axis_Conf.p_nyb->max_acc = pData[10];

            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t get_max_acc_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.p_pyb->is_task_running == 0) { /* 只有任务没有执行时才可进行配置 */

        if ((pData[6] >= 1) && (pData[6] <= 200)) {
            g_Axis_Conf.p_pyb->max_acc = pData[6];
            return 0;
        } else {
            return -1;
        }
    }
    return -1;
}
#endif

