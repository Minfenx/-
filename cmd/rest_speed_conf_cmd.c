/*
 * rest_speed_conf.C
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "rest_speed_conf_cmd.h"
#include "cmd.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_rest_speed_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        g_Axis_Conf.p_pya->reset_speed_given = (pData[6]  << 8) | pData[7];
        g_Axis_Conf.p_nya->reset_speed_given = (pData[8]  << 8) | pData[9];
        g_Axis_Conf.p_pyb->reset_speed_given = (pData[10] << 8) | pData[11];
        g_Axis_Conf.p_nyb->reset_speed_given = (pData[12] << 8) | pData[13];
        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int8_t get_rest_speed_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        g_Axis_Conf.p_pya->reset_speed_given = (pData[7]  << 8) | pData[8];
        g_Axis_Conf.p_nya->reset_speed_given = (pData[9]  << 8) | pData[10];
        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t get_rest_speed_conf_dat (const Uint16 *pData)
{
    Uint16 reset_speed_given[4];
    Uint16 i;
    single_axis_conf_t* p_axis;

    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检查数据有效性 */
        for (i = 0; i < AXIS_CNT; i++) {
            reset_speed_given[i] = (pData[7 + i * 2] << 8) | pData[8 + i * 2];
            if (((i <  2) && ((reset_speed_given[i] < 85) || (reset_speed_given[i] > 1707))) ||
                ((i >= 2) && ((reset_speed_given[i] < 56) || (reset_speed_given[i] > 667)))) {
                return -1;
            }
        }

        p_axis = g_Axis_Conf.p_pya;
        for (i = 0; i < AXIS_CNT; i++) {
            p_axis->reset_speed_given = reset_speed_given[i];
            p_axis = p_axis->p_next_axis;
        }

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t get_rest_speed_conf_dat (const Uint16 *pData)
{
    if (g_Axis_Conf.p_pyb->is_task_running == 0) { /* 只有任务没有执行时才可进行配置 */
        g_Axis_Conf.p_pyb->reset_speed_given = (pData[6]  << 8) | pData[7];
        return 0;
    }
    return -1;
}
#endif


