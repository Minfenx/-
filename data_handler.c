/*
 * data_handler.c
 *
 *  Created on: 2024��1��17��
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
 * \brief �������յ��Ĵ�������
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
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

        RevLen = sRevData.RevLength;  //��ȡ���յ������ݳ���
        sRevData.RevLength = 0;
        sRevData.RevFinished = FALSE;

        /* Ӧ���Ƚ����ݿ�������һ�������� */
        memcpy(__g_RevBuffer, sRevData.RevBuffer, RevLen);

        /* ������ */
//        if (__g_RevBuffer[4] != CMD_SERVE_TYPE_TELE_REQ) {
//            g_RS422_DATA_Tick = GetSysTick();
//        }

        //rev_check = CheckCalc(sRevData.RevBuffer,RevLen-1);

        /* zhukaifei 2023-05-29 */
        DataLen = (__g_RevBuffer[2] << 8) | __g_RevBuffer[3];	/* �õ���������ġ����ݳ��ȡ� */
        if (RevLen < 10) {		/* ʵ�ʽ��յ������ݸ�������Ϊ10 */
        	g_Tele_Dat.wrong_cnt++;	/* ����ָ��֡�������� */
        	return ret;
        }
        if (RevLen < DataLen + 8) {	/* ���ʵ�ʽ��յ��ĳ��ȱ����ݰ��еĳ���С��˵������û�����������������ֱ�ӷ��� */
        	g_Tele_Dat.wrong_cnt++;	/* ����ָ��֡�������� */
        	return ret;
        }
        check_sum = CheckSumCalc(&__g_RevBuffer[2], DataLen + 2);	/* ����Ӧ���ð��еġ����ݳ��ȡ����� */
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
					case CMD_SERVE_TYPE_AB_AXIS_MOV_CTL:	/* AB���˶�����ָ�� */
						ret = get_ab_axis_mov_ctl_dat(__g_RevBuffer);
						if (ret == 0) {
							g_Axis_Conf.new_task_req = true;
						}
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

					case CMD_SERVE_TYPE_MAX_ACC_CONF:			/* �����ٶ�����ָ�� */
						ret = get_max_acc_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

					case CMD_SERVE_TYPE_REST_SPEED_CONF:		/* ��λ�ٶ�����ָ�� */
						ret = get_rest_speed_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

					case CMD_SERVE_TYPE_MOT_CUR_CONF:	    	/* �����������ָ�� */
						ret = mot_cur_handler(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

//					case CMD_SERVE_TYPE_MOT_OVERCUR_PROT_CONF:	/* ���������������ָ�� */
//						ret = mot_overcur_prot_handler(__g_RevBuffer);
//						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
//					break;

					case CMD_SERVE_TYPE_CUR_LOCAL_CONF:			/* ��ǰλ������ָ�� */
						ret = get_cur_local_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

#if BOARD_NAME == BOARD_GMS
					case CMD_SERVE_TYPE_ANGLE_POW_SW_CONF:			/* �Ƕȴ���������ָ��  */
						ret = get_angle_sensor_sw_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;

                    case CMD_SERVE_TYPE_RESET_TIMES_CLEAR:          /* ��λ��������ָ��  */
                        ret = reset_times_clear();
                        g_Tele_Dat.comm_stat.bit.cmd_stat = 0;      /* ָ����״̬��ȷ */
                    break;
#endif

					/* DGM_2û��B�� */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_GMS)
					case CMD_SERVE_TYPE_SADMB_SOFT_LIM_CONF:	/* SADM-B�����λ����ָ�� */
						ret = get_sadmb_soft_lim_conf_dat(__g_RevBuffer);
						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
					break;
#endif

//					case CMD_SERVE_TYPE_SADM_DRV_HOT_BKUP_CONF:	/* SADM�����ȱ�����ָ�� */
//						ret = sade_drv_hot_bkup_handler(__g_RevBuffer);
//						g_Tele_Dat.comm_stat.bit.cmd_stat = 0;	/* ָ����״̬��ȷ */
//					    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
//					    g_Axis_Conf.p_pyb->loc_given     = 0;
//					    g_Axis_Conf.p_pyb->speed_given   = -4000;
//					    g_Axis_Conf.p_pyb->run_delay     = 0;
//					    g_Axis_Conf.new_task_req = true;
//					break;

					case CMD_SERVE_TYPE_TELE_REQ:				/* ң������ָ�� */
						ret = tele_req_cmd_run();
					break;

					default :
						g_Tele_Dat.comm_stat.bit.cmd_stat = 1;	/* ָ����״̬���� */
				}

				if (cmd_code == CMD_SERVE_TYPE_TELE_REQ) {
				    return 0;
				}


//				if (cmd_code != CMD_SERVE_TYPE_RECONSTITUTION) {
//					g_Tele_Dat.comm_stat.bit.run_stat = 0;
//				}

				/* ����ָ���Ƿ�ɹ�ִ�У�ң������ָ�Ҫ���� */
//				if (cmd_code != CMD_SERVE_TYPE_TELE_REQ) {
					g_Tele_Dat.comm_stat.bit.checksum_stat = 0;			/* У���״̬��ȷ */
//				}

				g_Tele_Dat.last_cmd = cmd_code;        /* ���ִ��ָ���� */

				if (ret == 0) {
					if (cmd_code == CMD_SERVE_TYPE_AB_AXIS_MOV_CTL) {
						g_Tele_Dat.right_mov_cnt++;     /* ��ȷ�˶�ָ��֡���� */
					} else if ((cmd_code != CMD_SERVE_TYPE_RECONSTITUTION) && (cmd_code != CMD_SERVE_TYPE_TELE_REQ)) {
						g_Tele_Dat.right_conf_cnt++;    /* ��ȷ����ָ��֡���� */
					}
					g_Tele_Dat.comm_stat.bit.dat_stat = 0;	/* ����״̬��ȷ */
				} else {
					g_Tele_Dat.wrong_cnt++;     /* ����ָ��֡�������� */
					g_Tele_Dat.comm_stat.bit.dat_stat = 1;	/* ����״̬���� */
				}
			} else {
				g_Tele_Dat.comm_stat.bit.checksum_stat = 1;	/* У���״̬���� */
			}
        }
    }

    return ret;
}
