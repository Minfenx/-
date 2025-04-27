/*
 * mot_overcur_prot_conf_cmd.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */
#if 0
#include "mot_overcur_prot_conf_cmd.h"
#include "cmd.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t mot_overcur_prot_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
		/* 检验数据合法性 */
    	if (((pData[6] != OVER_CUR_PROT_SW_OFF) && (pData[6] != OVER_CUR_PROT_SW_ON)) ||
			((pData[8] != OVER_CUR_PROT_SW_OFF) && (pData[8] != OVER_CUR_PROT_SW_ON)) ||
			((pData[10] != OVER_CUR_PROT_SW_OFF) && (pData[10] != OVER_CUR_PROT_SW_ON)) ||
			((pData[12] != OVER_CUR_PROT_SW_OFF) && (pData[12] != OVER_CUR_PROT_SW_ON))) {
			return -1;
		}
		if (((pData[6]  == OVER_CUR_PROT_SW_ON) && ((pData[7]  < 10) || (pData[7]  > 65))) ||
		    ((pData[8]  == OVER_CUR_PROT_SW_ON) && ((pData[9]  < 10) || (pData[9]  > 65))) ||
		    ((pData[10] == OVER_CUR_PROT_SW_ON) && ((pData[11] < 10) || (pData[11] > 95))) ||
		    ((pData[12] == OVER_CUR_PROT_SW_ON) && ((pData[13] < 10) || (pData[13] > 95)))) {
			return -1;
		}
		/* 如果开了过流保护配置，那么配置的电流不能比设置的运动电流和保持电流更小 */
		if (((pData[6]  == OVER_CUR_PROT_SW_ON) && ((pData[7]  < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[7]  < g_Axis_Conf.p_pya->keep_cur_given))) ||
			((pData[8]  == OVER_CUR_PROT_SW_ON) && ((pData[9]  < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[9]  < g_Axis_Conf.p_pya->keep_cur_given))) ||
			((pData[10] == OVER_CUR_PROT_SW_ON) && ((pData[11] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[11] < g_Axis_Conf.p_pya->keep_cur_given))) ||
			((pData[12] == OVER_CUR_PROT_SW_ON) && ((pData[13] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[13] < g_Axis_Conf.p_pya->keep_cur_given)))) {
			return -1;
		}

        g_Axis_Conf.p_pya->overcur_prot_sw = pData[6];
        g_Axis_Conf.p_pya->overcur_prot_th = pData[7];
        g_Axis_Conf.p_nya->overcur_prot_sw = pData[8];
        g_Axis_Conf.p_nya->overcur_prot_th = pData[9];
        g_Axis_Conf.p_pyb->overcur_prot_sw = pData[10];
        g_Axis_Conf.p_pyb->overcur_prot_th = pData[11];
        g_Axis_Conf.p_nyb->overcur_prot_sw = pData[12];
        g_Axis_Conf.p_nyb->overcur_prot_th = pData[13];

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int8_t mot_overcur_prot_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检验数据合法性 */
        if (((pData[7] != OVER_CUR_PROT_SW_OFF) && (pData[7] != OVER_CUR_PROT_SW_ON)) ||
            ((pData[9] != OVER_CUR_PROT_SW_OFF) && (pData[9] != OVER_CUR_PROT_SW_ON))) {
            return -1;
        }
        if (((pData[7]  == OVER_CUR_PROT_SW_ON) && ((pData[8]  < 10) || (pData[8]  > 65))) ||
            ((pData[9]  == OVER_CUR_PROT_SW_ON) && ((pData[10] < 10) || (pData[10] > 65)))) {
            return -1;
        }
        /* 如果开了过流保护配置，那么配置的电流不能比设置的运动电流和保持电流更小 */
        if (((pData[7]  == OVER_CUR_PROT_SW_ON) && ((pData[8]  < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[8]  < g_Axis_Conf.p_pya->keep_cur_given))) ||
            ((pData[9]  == OVER_CUR_PROT_SW_ON) && ((pData[10] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[10] < g_Axis_Conf.p_pya->keep_cur_given)))) {
            return -1;
        }

        g_Axis_Conf.p_pya->overcur_prot_sw = pData[7];
        g_Axis_Conf.p_pya->overcur_prot_th = pData[8];
        g_Axis_Conf.p_nya->overcur_prot_sw = pData[9];
        g_Axis_Conf.p_nya->overcur_prot_th = pData[10];

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t mot_overcur_prot_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检验数据合法性 */
        if (((pData[7] != OVER_CUR_PROT_SW_OFF) && (pData[7] != OVER_CUR_PROT_SW_ON)) ||
            ((pData[9] != OVER_CUR_PROT_SW_OFF) && (pData[9] != OVER_CUR_PROT_SW_ON)) ||
            ((pData[11] != OVER_CUR_PROT_SW_OFF) && (pData[11] != OVER_CUR_PROT_SW_ON)) ||
            ((pData[13] != OVER_CUR_PROT_SW_OFF) && (pData[13] != OVER_CUR_PROT_SW_ON))) {
            return -1;
        }
        if (((pData[7]  == OVER_CUR_PROT_SW_ON) && ((pData[8]  < 10) || (pData[8]  > 65))) ||
            ((pData[9]  == OVER_CUR_PROT_SW_ON) && ((pData[10] < 10) || (pData[10] > 65))) ||
            ((pData[11] == OVER_CUR_PROT_SW_ON) && ((pData[12] < 10) || (pData[12] > 95))) ||
            ((pData[13] == OVER_CUR_PROT_SW_ON) && ((pData[14] < 10) || (pData[14] > 95)))) {
            return -1;
        }
        /* 如果开了过流保护配置，那么配置的电流不能比设置的运动电流和保持电流更小 */
        if (((pData[7]  == OVER_CUR_PROT_SW_ON) && ((pData[8]  < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[8]  < g_Axis_Conf.p_pya->keep_cur_given))) ||
            ((pData[9]  == OVER_CUR_PROT_SW_ON) && ((pData[10] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[10] < g_Axis_Conf.p_pya->keep_cur_given))) ||
            ((pData[11] == OVER_CUR_PROT_SW_ON) && ((pData[12] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[12] < g_Axis_Conf.p_pya->keep_cur_given))) ||
            ((pData[13] == OVER_CUR_PROT_SW_ON) && ((pData[14] < g_Axis_Conf.p_pya->max_mov_cur_given) || (pData[14] < g_Axis_Conf.p_pya->keep_cur_given)))) {
            return -1;
        }

        g_Axis_Conf.p_pya->overcur_prot_sw = pData[7];
        g_Axis_Conf.p_pya->overcur_prot_th = pData[8];
        g_Axis_Conf.p_nya->overcur_prot_sw = pData[9];
        g_Axis_Conf.p_nya->overcur_prot_th = pData[10];
        g_Axis_Conf.p_pyb->overcur_prot_sw = pData[11];
        g_Axis_Conf.p_pyb->overcur_prot_th = pData[12];
        g_Axis_Conf.p_nyb->overcur_prot_sw = pData[13];
        g_Axis_Conf.p_nyb->overcur_prot_th = pData[14];

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t mot_overcur_prot_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检验数据合法性 */
        if ((pData[6] != OVER_CUR_PROT_SW_OFF) && (pData[6] != OVER_CUR_PROT_SW_ON)) {
            return -1;
        }
        if ((pData[6] == OVER_CUR_PROT_SW_ON) && ((pData[7]  < 10) || (pData[7]  > 65))) {
            return -1;
        }
        /* 如果开了过流保护配置，那么配置的电流不能比设置的运动电流和保持电流更小 */
        if ((pData[6]  == OVER_CUR_PROT_SW_ON) && ((pData[7]  < g_Axis_Conf.p_pyb->max_mov_cur_given) || (pData[7]  < g_Axis_Conf.p_pyb->keep_cur_given))) {
            return -1;
        }

        g_Axis_Conf.p_pyb->overcur_prot_sw = pData[6];
        g_Axis_Conf.p_pyb->overcur_prot_th = pData[7];

        return 0;
    }
    return -1;
}
#endif
#endif
