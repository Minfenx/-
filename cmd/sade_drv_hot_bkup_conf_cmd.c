/*
 * sade_drv_hot_bkup_conf_cmd.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */
#if 0
#include "sade_drv_hot_bkup_conf_cmd.h"
#include "cmd.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t sade_drv_hot_bkup_handler (const Uint16 *pData)
{
	int8_t i = 0;
	single_axis_conf_t* p_axis = 0;

    if (g_Axis_Conf.task_cnt == 0) { 		/* 只有任务没有执行时才可进行配置 */
    	/* 检查数据的有效性 */
    	if (((pData[6] != 0) && (pData[6] != 0x55) && (pData[6] != 0xAA)) ||
    		((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
			((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA)) ||
			((pData[9] != 0) && (pData[9] != 0x55) && (pData[9] != 0xAA))) {
    		return -1;
    	}

    	/* 数据为0表示跳过设置 */
    	p_axis = g_Axis_Conf.p_pya;
    	for (i = 0; i < 4; i++) {
			if ((pData[6 + i] != 0) && (p_axis->drv_conf != pData[6 + i])) {
				p_axis->drv_conf = pData[6 + i];
				if (p_axis->drv_conf == 0x55) {	/* 切换到线路1 */
					p_axis->fn_s_disable();		/* 备份禁能 */
					p_axis->fn_m_enable();		/* 主份使能 */
				} else if (p_axis->drv_conf == 0xAA) {	/* 切换到线路2 */
					p_axis->fn_m_disable();		/* 主份禁能 */
					p_axis->fn_s_enable();		/* 备份使能 */
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

    if (g_Axis_Conf.task_cnt == 0) {        /* 只有任务没有执行时才可进行配置 */
        /* 检查数据的有效性 */
        if (((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
            ((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA))) {
            return -1;
        }

        /* 数据为0表示跳过设置 */
        p_axis = g_Axis_Conf.p_pya;
        for (i = 0; i < AXIS_CNT; i++) {
            if ((pData[7 + i] != 0) && (p_axis->drv_conf != pData[7 + i])) {
                p_axis->drv_conf = pData[7 + i];
                if (p_axis->drv_conf == 0x55) { /* 切换到线路1 */
                    p_axis->fn_s_disable();     /* 备份禁能 */
                    p_axis->fn_m_enable();      /* 主份使能 */
                } else if (p_axis->drv_conf == 0xAA) {  /* 切换到线路2 */
                    p_axis->fn_m_disable();     /* 主份禁能 */
                    p_axis->fn_s_enable();      /* 备份使能 */
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

    if (g_Axis_Conf.task_cnt == 0) {        /* 只有任务没有执行时才可进行配置 */
        /* 检查数据的有效性 */
        if (((pData[7] != 0) && (pData[7] != 0x55) && (pData[7] != 0xAA)) ||
            ((pData[8] != 0) && (pData[8] != 0x55) && (pData[8] != 0xAA)) ||
            ((pData[9] != 0) && (pData[9] != 0x55) && (pData[9] != 0xAA)) ||
            ((pData[10] != 0) && (pData[10] != 0x55) && (pData[10] != 0xAA))) {
            return -1;
        }

        /* 数据为0表示跳过设置 */
        p_axis = g_Axis_Conf.p_pya;
        for (i = 0; i < AXIS_CNT; i++) {
            if ((pData[7 + i] != 0) && (p_axis->drv_conf != pData[7 + i])) {
                p_axis->drv_conf = pData[7 + i];
                if (p_axis->drv_conf == 0x55) { /* 切换到线路1 */
                    p_axis->fn_s_disable();     /* 备份禁能 */
                    p_axis->fn_m_enable();      /* 主份使能 */
                } else if (p_axis->drv_conf == 0xAA) {  /* 切换到线路2 */
                    p_axis->fn_m_disable();     /* 主份禁能 */
                    p_axis->fn_s_enable();      /* 备份使能 */
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
//    if (g_Axis_Conf.task_cnt == 0) {        /* 只有任务没有执行时才可进行配置 */
//        /* 检查数据的有效性 */
//        if ((pData[6] != 0) && (pData[6] != 0x55) && (pData[6] != 0xAA)) {
//            return -1;
//        }
//
//        /* 数据为0表示跳过设置 */
//        if ((pData[6] != 0) && (p_axis->drv_conf != pData[6])) {
//            p_axis->drv_conf = pData[6];
//            if (p_axis->drv_conf == 0x55) { /* 切换到线路1 */
//                p_axis->fn_s_disable();     /* 备份禁能 */
//                p_axis->fn_m_enable();      /* 主份使能 */
//            } else if (p_axis->drv_conf == 0xAA) {  /* 切换到线路2 */
//                p_axis->fn_m_disable();     /* 主份禁能 */
//                p_axis->fn_s_enable();      /* 备份使能 */
//            }
//        }
//
//        return 0;
//    }
    return -1;
}
#endif
#endif

