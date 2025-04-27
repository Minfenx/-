/*
 * data_handler.c
 *
 *  Created on: 2024年1月17日
 *      Author: Suzkfly
 */
#include "data_handler.h"
#include "sci.h"
#include "tele_dat.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "cpu_timer0.h"
#include "cur_local_conf_cmd.h"
#include "max_acc_conf_cmd.h"
#include "rest_speed_conf_cmd.h"
#include "sadmb_soft_lim_conf_cmd.h"
#include "mot_cur_conf_cmd.h"
#include <string.h>
#include "pwm.h"
#include "angle_sensor_sw_cmd.h"
#include "reset_times_clear_cmd.h"

static Uint16 __g_RevBuffer[MAX_REV_SIZE];

/**
 * \brief 解析接收到的串口数据
 *
 * \retval 成功返回0，失败返回-1
 */
#pragma CODE_SECTION(RevDataRead, "ramfuncs");
int16 RevDataRead(void)
{
//  Uint16 rev_check;
    Uint16 check_sum = 0;
//  Uint16 i;
    Uint16 DataLen;
    Uint16 RevLen;
	int16 ret = -1;

    if (TRUE == sRevData.RevFinished) {

    	g_RS422_DATA_Tick = GetSysTick();
    	g_RS422_relink_times = 0;

        RevLen = sRevData.RevLength;  //读取接收到的数据长度
        sRevData.RevLength = 0;
        sRevData.RevFinished = FALSE;

        /* 应该先将数据拷贝到另一个数组中 */
        memcpy(__g_RevBuffer, sRevData.RevBuffer, RevLen);

        /* 调试用 */
//        if (__g_RevBuffer[4] != CMD_SERVE_TYPE_TELE_REQ) {
//            g_RS422_DATA_Tick = GetSysTick();
//        }

        //rev_check = CheckCalc(sRevData.RevBuffer,RevLen-1);

        /* zhukaifei 2023-05-29 */
        DataLen = (__g_RevBuffer[2] << 8) | __g_RevBuffer[3];	/* 得到数据里面的“数据长度” */
        if (RevLen < 10) {		/* 实际接收到的数据个数最少为10 */
        	g_Tele_Dat.wrong_cnt++;	/* 错误指令帧计数增加 */
        	return ret;
        }
        if (RevLen < DataLen + 8) {	/* 如果实际接收到的长度比数据包中的长度小，说明数据没接收完整，这种情况直接返回 */
        	g_Tele_Dat.wrong_cnt++;	/* 错误指令帧计数增加 */
        	return ret;
        }
        check_sum = CheckSumCalc(&__g_RevBuffer[2], DataLen + 2);	/* 这里应该用包中的“数据长度”计算 */
        if ((__g_RevBuffer[0] == 0xEB) &&
        	(__g_RevBuffer[1] == 0x90) &&
			(__g_RevBuffer[RevLen - 2] == 0x09) &&
			(__g_RevBuffer[RevLen - 1] == 0xD7)) {

            if (check_sum == ((__g_RevBuffer[RevLen - 4] << 8) | __g_RevBuffer[RevLen - 3])) {
            //if (1) {
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_GMS)
				uint8_t cmd_code = __g_RevBuffer[4];
#elif (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
				uint8_t cmd_code = __g_RevBuffer[6];
#endif
				switch (cmd_code) {
					case CMD_SERVE_TYPE_AB_AXIS_MOV_CTL:	/* AB轴运动控制指令 */
						ret = get_ab_axis_mov_ctl_dat(__g_RevBuffer);
						if (ret == 0) {
							g_Axis_Conf.new_task_req = true;
						}
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

					case CMD_SERVE_TYPE_MAX_ACC_CONF:			/* 最大加速度配置指令 */
						ret = get_max_acc_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

					case CMD_SERVE_TYPE_REST_SPEED_CONF:		/* 复位速度配置指令 */
						ret = get_rest_speed_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

					case CMD_SERVE_TYPE_MOT_CUR_CONF:	    	/* 电机电流配置指令 */
						ret = mot_cur_handler(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

//					case CMD_SERVE_TYPE_MOT_OVERCUR_PROT_CONF:	/* 电机过流保护配置指令 */
//						ret = mot_overcur_prot_handler(__g_RevBuffer);
//						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
//					break;

					case CMD_SERVE_TYPE_CUR_LOCAL_CONF:			/* 当前位置配置指令 */
						ret = get_cur_local_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

#if BOARD_NAME == BOARD_GMS
					case CMD_SERVE_TYPE_ANGLE_POW_SW_CONF:			/* 角度传感器开关指令  */
						ret = get_angle_sensor_sw_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;

                    case CMD_SERVE_TYPE_RESET_TIMES_CLEAR:          /* 复位次数清零指令  */
                        ret = reset_times_clear();
                        g_Tele_Dat.comm_stat.bit.cmd_stat = 0;      /* 指令码状态正确 */
                    break;
#endif

					/* DGM_2没有B轴 */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_GMS)
					case CMD_SERVE_TYPE_SADMB_SOFT_LIM_CONF:	/* SADM-B软件限位配置指令 */
						ret = get_sadmb_soft_lim_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
					break;
#endif

//					case CMD_SERVE_TYPE_SADM_DRV_HOT_BKUP_CONF:	/* SADM驱动热备配置指令 */
//						ret = sade_drv_hot_bkup_handler(__g_RevBuffer);
//						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* 指令码状态正确 */
//					    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
//					    g_Axis_Conf.p_pyb->loc_given     = 0;
//					    g_Axis_Conf.p_pyb->speed_given   = -4000;
//					    g_Axis_Conf.p_pyb->run_delay     = 0;
//					    g_Axis_Conf.new_task_req = true;
//					break;

					case CMD_SERVE_TYPE_TELE_REQ:				/* 遥测请求指令 */
						ret = tele_req_cmd_run();
					break;

					default :
						g_Tele_Dat.comm_stat.bit.cmd_stat = 1;	/* 指令码状态错误 */
				}

				if (cmd_code == CMD_SERVE_TYPE_TELE_REQ) {
				    return 0;
				}


//				if (cmd_code != CMD_SERVE_TYPE_RECONSTITUTION) {
//					g_Tele_Dat.comm_stat.bit.run_stat = 0;
//				}

				/* 无论指令是否成功执行，遥测请求指令都要增加 */
//				if (cmd_code != CMD_SERVE_TYPE_TELE_REQ) {
					g_Tele_Dat.comm_stat.bit.checksum_stat = 0;			/* 校验和状态正确 */
//				}

				g_Tele_Dat.last_cmd = cmd_code;        /* 最近执行指令码 */

				if (ret == 0) {
					if (cmd_code == CMD_SERVE_TYPE_AB_AXIS_MOV_CTL) {
						g_Tele_Dat.right_mov_cnt++;     /* 正确运动指令帧增加 */
					} else if ((cmd_code != CMD_SERVE_TYPE_RECONSTITUTION) && (cmd_code != CMD_SERVE_TYPE_TELE_REQ)) {
						g_Tele_Dat.right_conf_cnt++;    /* 正确配置指令帧增加 */
					}
					g_Tele_Dat.comm_stat.bit.dat_stat = 0;	/* 数据状态正确 */
				} else {
					g_Tele_Dat.wrong_cnt++;     /* 错误指令帧计数增加 */
					g_Tele_Dat.comm_stat.bit.dat_stat = 1;	/* 数据状态错误 */
				}
			} else {
				g_Tele_Dat.comm_stat.bit.checksum_stat = 1;	/* 校验和状态错误 */
			}
        }
    }

    return ret;
}
