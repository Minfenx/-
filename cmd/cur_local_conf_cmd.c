/*
 * cur_loccal_conf_cmd.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "cur_local_conf_cmd.h"
#include "cmd.h"
#include "boot_args.h"
#include "tele_dat.h"
#include <tlc2543.h>

#if BOARD_NAME == BOARD_GMS
extern uint8_t  g_last_work_mode_set;          	/* 上次工作模式 */
extern int32    g_last_loc_given;          		/* 上次位置给定 */
extern int16    g_last_speed_given;          	/* 上次速度给定 */
#elif (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_DGM_2)
float32 g_angle_senser_pya = 0;
float32 g_angle_senser_nya = 0;
float32 g_angle_senser_pyb = 0;
float32 g_angle_senser_nyb = 0;

extern uint8_t  g_last_work_mode_set[AXIS_CNT];          	/* 上次工作模式 */
extern int32    g_last_loc_given[AXIS_CNT];          		/* 上次位置给定 */
extern int16    g_last_speed_given[AXIS_CNT];          		/* 上次速度给定 */
#endif

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_cur_local_conf_dat (const Uint16 *pData)
{
    Uint32 temp;
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        temp = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        g_Axis_Conf.p_pya->cur_local = temp;
        temp = ((Uint32)pData[10] << 24) | ((Uint32)pData[11] << 16) | (pData[12] << 8) | pData[13];
        g_Axis_Conf.p_nya->cur_local = temp;
        temp = ((Uint32)pData[14] << 24) | ((Uint32)pData[15] << 16) | (pData[16] << 8) | pData[17];
        g_Axis_Conf.p_pyb->cur_local = temp;
        temp = ((Uint32)pData[18] << 24) | ((Uint32)pData[19] << 16) | (pData[20] << 8) | pData[21];
        g_Axis_Conf.p_nyb->cur_local = temp;

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
//#pragma CODE_SECTION(get_cur_local_conf_dat, "ramfuncs");
int8_t get_cur_local_conf_dat (const Uint16 *pData)
{
    int32 temp;
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        temp = ((int32)pData[7]  << 24) | ((int32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
        if (temp != g_Axis_Conf.p_pya->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[0] = 0;           /* 上次工作模式 */
            g_last_loc_given[0] = 0;               /* 上次位置给定 */
            g_last_speed_given[0] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_pya->cur_local = temp;

        temp = ((int32)pData[11] << 24) | ((int32)pData[12] << 16) | (pData[13] << 8) | pData[14];
        if (temp != g_Axis_Conf.p_nya->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[1] = 0;           /* 上次工作模式 */
            g_last_loc_given[1] = 0;               /* 上次位置给定 */
            g_last_speed_given[1] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_nya->cur_local = temp;

#if 1
        g_angle_senser_pya = Vol_To_Angle('A', '+', 1) - g_Axis_Conf.p_pya->cur_local * 9 / 25600;
		g_angle_senser_nya = Vol_To_Angle('A', '-', 1) - g_Axis_Conf.p_nya->cur_local * 9 / 25600;
#else
        *gp_boot_arg_angle_senser_adj_pya_1 = Vol_To_Angle('A', '+', 0) - g_Axis_Conf.p_pya->cur_local * 9 / 25600;
        *gp_boot_arg_angle_senser_adj_nya_1 = Vol_To_Angle('A', '-', 0) - g_Axis_Conf.p_nya->cur_local * 9 / 25600;
#endif
        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
//#pragma CODE_SECTION(get_cur_local_conf_dat, "ramfuncs");
int8_t get_cur_local_conf_dat (const Uint16 *pData)
{
    int32 temp;
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */

        temp = ((int32)pData[7]  << 24) | ((int32)pData[8]  << 16) | (pData[9]  << 8) | pData[10];
        if (temp != g_Axis_Conf.p_pya->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[0] = 0;           /* 上次工作模式 */
            g_last_loc_given[0] = 0;               /* 上次位置给定 */
            g_last_speed_given[0] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_pya->cur_local = temp;

        temp = ((int32)pData[11] << 24) | ((int32)pData[12] << 16) | (pData[13] << 8) | pData[14];
        if (temp != g_Axis_Conf.p_nya->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[1] = 0;           /* 上次工作模式 */
            g_last_loc_given[1] = 0;               /* 上次位置给定 */
            g_last_speed_given[1] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_nya->cur_local = temp;

        temp = ((int32)pData[15] << 24) | ((int32)pData[16] << 16) | (pData[17] << 8) | pData[18];
        if ((temp > g_Axis_Conf.p_pyb->soft_lim) || (temp < 0 - g_Axis_Conf.p_pyb->soft_lim)) {
            return -1;
        }
        if (temp != g_Axis_Conf.p_pyb->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[2] = 0;           /* 上次工作模式 */
            g_last_loc_given[2] = 0;               /* 上次位置给定 */
            g_last_speed_given[2] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_pyb->cur_local = temp;

        temp = ((int32)pData[19] << 24) | ((int32)pData[20] << 16) | (pData[21] << 8) | pData[22];
        if ((temp > g_Axis_Conf.p_nyb->soft_lim) || (temp < 0 - g_Axis_Conf.p_nyb->soft_lim)){
            return -1;
        }
        if (temp != g_Axis_Conf.p_nyb->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set[3] = 0;           /* 上次工作模式 */
            g_last_loc_given[3] = 0;               /* 上次位置给定 */
            g_last_speed_given[3] = 0;             /* 上次速度给定 */
        }
        g_Axis_Conf.p_nyb->cur_local = temp;

#if 1
        g_angle_senser_pya = Vol_To_Angle('A', '+', 1) - g_Axis_Conf.p_pya->cur_local * 9 / 25600;
		g_angle_senser_nya = Vol_To_Angle('A', '-', 1) - g_Axis_Conf.p_nya->cur_local * 9 / 25600;
		g_angle_senser_pyb = Vol_To_Angle('B', '+', 1) - g_Axis_Conf.p_pyb->cur_local * 9 / 10000;
		g_angle_senser_nyb = Vol_To_Angle('B', '-', 1) - g_Axis_Conf.p_nyb->cur_local * 9 / 10000;
#else
        *gp_boot_arg_angle_senser_adj_pya_1 = Vol_To_Angle('A', '+', 0) - g_Axis_Conf.p_pya->cur_local * 9 / 25600;
        *gp_boot_arg_angle_senser_adj_nya_1 = Vol_To_Angle('A', '-', 0) - g_Axis_Conf.p_nya->cur_local * 9 / 25600;
        *gp_boot_arg_angle_senser_adj_pyb_3 = Vol_To_Angle('B', '+', 0) - g_Axis_Conf.p_pyb->cur_local * 9 / 10000;
        *gp_boot_arg_angle_senser_adj_nyb_3 = Vol_To_Angle('B', '-', 0) - g_Axis_Conf.p_nyb->cur_local * 9 / 10000;
#endif

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
#include "GL_CJW_HE_5701.h"
int8_t get_cur_local_conf_dat (const Uint16 *pData)
{
    int32 temp;
    int32 old_local = 0;

    if (g_Axis_Conf.p_pyb->is_task_running == 0 && g_angle_adj_finish == 1) { /* 只有任务没有执行时，并且角度传感器状态确定才可进行配置 */
        temp = ((Uint32)pData[6]  << 24) | ((Uint32)pData[7]  << 16) | (pData[8]  << 8) | pData[9];
        if ((temp > g_Axis_Conf.p_pyb->soft_lim_p) || (temp < g_Axis_Conf.p_pyb->soft_lim_n)){
            return -1;
        }

        old_local = (float32)g_Tele_Dat.p_yb_axis_angle_res / 1000 / g_Axis_Conf.p_pyb->loc_to_angle_factor;

        if (temp != g_Axis_Conf.p_pyb->cur_local) { /* 重新配置当前位置之后可以接收新指令 */
            g_last_work_mode_set = 0;           /* 上次工作模式 */
            g_last_loc_given = 0;               /* 上次位置给定 */
            g_last_speed_given = 0;             /* 上次速度给定 */
        }

        g_Tele_Dat.soft_lim_err_stat = 0;

        g_Axis_Conf.p_pyb->cur_local = temp;

        *gp_boot_arg_angle_senser_adj = (*gp_boot_arg_angle_senser_adj) + (g_Axis_Conf.p_pyb->cur_local - old_local) * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        if (*gp_boot_arg_angle_senser_adj > 360) {
            *gp_boot_arg_angle_senser_adj -= 360;
        } else if (*gp_boot_arg_angle_senser_adj < -360) {
            *gp_boot_arg_angle_senser_adj += 360;
        }
        g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
        *gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;	//不加这一行会导致主函数保存当前位置的时候,又将旧值恢复回去
        return 0;
    }
    return -1;
}
#endif

