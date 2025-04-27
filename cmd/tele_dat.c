/*
 * tele_dat.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "tele_dat.h"
#include <string.h>
#include <tlv2548.h>
#include <tlc2543.h>
#include "AB_axis_mov_ctl_cmd.h"
#include "sci.h"
#include "boot_args.h"
#include <math.h>
#include "pps.h"
#include "mot_cur_conf_cmd.h"

tele_dat_t g_Tele_Dat;  /**< \brief 遥测数据 */
#pragma DATA_SECTION(g_Tele_Dat, "pre_roll_data");

extern uint8_t g_last_reset_reason;

/**
 * \brief 初始化遥测数据
 */
void InitTeleDat (void)
{
    memset(&g_Tele_Dat, 0, sizeof(g_Tele_Dat));
#if BOARD_NAME == BOARD_GMS
    g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 1;    /* 角度传感器上电默认为异常 */
    g_Tele_Dat.system_reset_times = *gp_boot_arg_system_reset_times;

    g_Tele_Dat.last_reset_reason = g_last_reset_reason;

    g_Tele_Dat.comm_stat.bit.angle_senser_sw = ANGLE_SENSOR_SW_ON;		/* 角度传感器上电默认是开的 */

#if (GMS_BOARD_VERSION == GMS_BOARD_OLD) || (GMS_BOARD_VERSION == GMS_BOARD_NEW)
    g_Tele_Dat.program_run_addr = RUN_REGION;
#elif (GMS_BOARD_VERSION == GMS_BOARD_NEW2)

#endif
#endif
}

/**
 * \brief 将结构体解析成串口数据
 *
 * \param[in]  p_src：需要解析的遥测数据
 * \param[out] pData：解析后的结果
 *
 * \retval 成功返回0，失败返回-1
 *
 * \note 要确保pData长度不小于115
 * \note 在此函数中会计算校验和
 */
#if (BOARD_NAME == BOARD_CX20)
static int16 __tele_dat_to__sci_dat (tele_dat_t* p_src, Uint16 *pData)
{
    pData[0] = (p_src->frame_head >> 8) & 0xFF;
    pData[1] = p_src->frame_head        & 0xFF;
    pData[2] = (p_src->dat_len >> 8)    & 0xFF;
    pData[3] = p_src->dat_len           & 0xFF;
    pData[4] = p_src->serve_type        & 0xFF;
    pData[5] = p_src->serve_state       & 0xFF;

    pData[6]  = p_src->pri_bkup_id          & 0xFF;
    pData[7]  = p_src->tele_req_cnt         & 0xFF;
    pData[8]  = p_src->right_mov_cnt        & 0xFF;
    pData[9]  = p_src->right_conf_cnt       & 0xFF;
    pData[10] = p_src->wrong_cnt            & 0xFF;
    pData[11] = p_src->comm_rest_cnt        & 0xFF;
    pData[12] = p_src->sec_pow_24v          & 0xFF;
    pData[13] = p_src->sec_pow_5v           & 0xFF;
    pData[14] = p_src->sec_pow_1v2          & 0xFF;
    pData[15] = p_src->py_res_base          & 0xFF;
    pData[16] = p_src->ny_res_base          & 0xFF;
    pData[17] = p_src->fst_gen_cur          & 0xFF;
    pData[18] = p_src->pow_5v_cur           & 0xFF;
    pData[19] = p_src->pow_3v3_cur          & 0xFF;
    pData[20] = p_src->pow_1v2_cur          & 0xFF;
    pData[21] = p_src->p_ya_axis_mot_cur    & 0xFF;
    pData[22] = p_src->n_ya_axis_mot_cur    & 0xFF;
    pData[23] = p_src->p_yb_axis_mot_cur    & 0xFF;
    pData[24] = p_src->n_yb_axis_mot_cur    & 0xFF;
    pData[25] = p_src->last_cmd             & 0xFF;
    pData[26] = p_src->comm_stat.all        & 0xFF;
    pData[27] = p_src->p_ya_axis_work_mode  & 0xFF;
    pData[28] = p_src->p_ya_axis_stat.all   & 0xFF;
    pData[29] = p_src->n_ya_axis_work_mode  & 0xFF;
    pData[30] = p_src->n_ya_axis_stat.all   & 0xFF;
    pData[31] = p_src->p_yb_axis_work_mode  & 0xFF;
    pData[32] = p_src->p_yb_axis_stat.all   & 0xFF;
    pData[33] = p_src->n_yb_axis_work_mode  & 0xFF;
    pData[34] = p_src->n_yb_axis_stat.all   & 0xFF;

    pData[35] = (p_src->p_ya_axis_angle_res >> 24)  & 0xFF;
    pData[36] = (p_src->p_ya_axis_angle_res >> 16)  & 0xFF;
    pData[37] = (p_src->p_ya_axis_angle_res >> 8)   & 0xFF;
    pData[38] = p_src->p_ya_axis_angle_res          & 0xFF;

    pData[39] = (p_src->n_ya_axis_angle_res >> 24)  & 0xFF;
    pData[40] = (p_src->n_ya_axis_angle_res >> 16)  & 0xFF;
    pData[41] = (p_src->n_ya_axis_angle_res >> 8)   & 0xFF;
    pData[42] = p_src->n_ya_axis_angle_res          & 0xFF;

    pData[43] = (p_src->p_yb_axis_angle_res >> 24)  & 0xFF;
    pData[44] = (p_src->p_yb_axis_angle_res >> 16)  & 0xFF;
    pData[45] = (p_src->p_yb_axis_angle_res >> 8)   & 0xFF;
    pData[46] = p_src->p_yb_axis_angle_res          & 0xFF;

    pData[47] = (p_src->n_yb_axis_angle_res >> 24)  & 0xFF;
    pData[48] = (p_src->n_yb_axis_angle_res >> 16)  & 0xFF;
    pData[49] = (p_src->n_yb_axis_angle_res >> 8)   & 0xFF;
    pData[50] = p_src->n_yb_axis_angle_res          & 0xFF;

    pData[51] = (p_src->p_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[52] = (p_src->p_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[53] = (p_src->p_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[54] = p_src->p_ya_axis_angle_mot          & 0xFF;

    pData[55] = (p_src->n_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[56] = (p_src->n_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[57] = (p_src->n_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[58] = p_src->n_ya_axis_angle_mot          & 0xFF;

    pData[59] = (p_src->p_yb_axis_angle_mot >> 24)  & 0xFF;
    pData[60] = (p_src->p_yb_axis_angle_mot >> 16)  & 0xFF;
    pData[61] = (p_src->p_yb_axis_angle_mot >> 8)   & 0xFF;
    pData[62] = p_src->p_yb_axis_angle_mot          & 0xFF;

    pData[63] = (p_src->n_yb_axis_angle_mot >> 24)  & 0xFF;
    pData[64] = (p_src->n_yb_axis_angle_mot >> 16)  & 0xFF;
    pData[65] = (p_src->n_yb_axis_angle_mot >> 8)   & 0xFF;
    pData[66] = p_src->n_yb_axis_angle_mot          & 0xFF;

    pData[67] = (p_src->p_ya_axis_angle_v >> 8)     & 0xFF;
    pData[68] = p_src->p_ya_axis_angle_v            & 0xFF;
    pData[69] = (p_src->n_ya_axis_angle_v >> 8)     & 0xFF;
    pData[70] = p_src->n_ya_axis_angle_v            & 0xFF;
    pData[71] = (p_src->p_yb_axis_angle_v >> 8)     & 0xFF;
    pData[72] = p_src->p_yb_axis_angle_v            & 0xFF;
    pData[73] = (p_src->n_yb_axis_angle_v >> 8)     & 0xFF;
    pData[74] = p_src->n_yb_axis_angle_v            & 0xFF;

    pData[75] = (p_src->p_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[76] = (p_src->p_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[77] = (p_src->p_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[78] = p_src->p_ya_axis_cmd_local          & 0xFF;
    pData[79] = (p_src->p_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[80] = p_src->p_ya_axis_cmd_speed          & 0xFF;

    pData[81] = (p_src->n_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[82] = (p_src->n_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[83] = (p_src->n_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[84] = p_src->n_ya_axis_cmd_local          & 0xFF;
    pData[85] = (p_src->n_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[86] = p_src->n_ya_axis_cmd_speed          & 0xFF;

    pData[87] = (p_src->p_yb_axis_cmd_local >> 24)  & 0xFF;
    pData[88] = (p_src->p_yb_axis_cmd_local >> 16)  & 0xFF;
    pData[89] = (p_src->p_yb_axis_cmd_local >> 8)   & 0xFF;
    pData[90] = p_src->p_yb_axis_cmd_local          & 0xFF;
    pData[91] = (p_src->p_yb_axis_cmd_speed >> 8)   & 0xFF;
    pData[92] = p_src->p_yb_axis_cmd_speed          & 0xFF;

    pData[93] = (p_src->n_yb_axis_cmd_local >> 24)  & 0xFF;
    pData[94] = (p_src->n_yb_axis_cmd_local >> 16)  & 0xFF;
    pData[95] = (p_src->n_yb_axis_cmd_local >> 8)   & 0xFF;
    pData[96] = p_src->n_yb_axis_cmd_local          & 0xFF;
    pData[97] = (p_src->n_yb_axis_cmd_speed >> 8)   & 0xFF;
    pData[98] = p_src->n_yb_axis_cmd_speed          & 0xFF;

    pData[99]  = p_src->p_ya_axis_line_center_loc   & 0xFF;
    pData[100] = p_src->p_ya_axis_input_wall        & 0xFF;
    pData[101] = p_src->n_ya_axis_line_center_loc   & 0xFF;
    pData[102] = p_src->n_ya_axis_input_wall        & 0xFF;
    pData[103] = p_src->rev1                        & 0xFF;
    pData[104] = p_src->edac_single_err_cnt         & 0xFF;
    pData[105] = p_src->edac_double_err_cnt         & 0xFF;

    pData[106] = p_src->rev3[0]             & 0xFF;
    pData[107] = p_src->rev3[1]             & 0xFF;
    pData[108] = p_src->rev3[2]             & 0xFF;
    pData[109] = p_src->rev3[3]             & 0xFF;
    pData[110] = p_src->rev3[4]             & 0xFF;

    p_src->check_sum = CheckSumCalc(&pData[2], p_src->dat_len + 2);

    pData[111] = (p_src->check_sum >> 8)    & 0xFF;
    pData[112] = p_src->check_sum           & 0xFF;

    pData[113] = (p_src->frame_tile >> 8)   & 0xFF;
    pData[114] = p_src->frame_tile          & 0xFF;

    return 0;
}
#elif (BOARD_NAME == BOARD_DGM_2)
static int16 __tele_dat_to__sci_dat (tele_dat_t* p_src, Uint16 *pData)
{
    pData[0] = (p_src->frame_head >> 8) & 0xFF;
    pData[1] = p_src->frame_head        & 0xFF;
    pData[2] = (p_src->dat_len >> 8)    & 0xFF;
    pData[3] = p_src->dat_len           & 0xFF;
    pData[4] = p_src->serve_type        & 0xFF;
    pData[5] = p_src->serve_state       & 0xFF;

    pData[6]  = p_src->pri_bkup_id          & 0xFF;
    pData[7]  = p_src->tele_req_cnt         & 0xFF;
    pData[8]  = p_src->right_mov_cnt        & 0xFF;
    pData[9]  = p_src->right_conf_cnt       & 0xFF;
    pData[10] = p_src->wrong_cnt            & 0xFF;
    pData[11] = p_src->comm_rest_cnt        & 0xFF;

    pData[12] = p_src->p_ya_max_mov_cur_set    & 0xFF;
    pData[13] = p_src->n_ya_max_mov_cur_set    & 0xFF;

    pData[14] = p_src->last_cmd             & 0xFF;
    pData[15] = p_src->comm_stat.all        & 0xFF;
    pData[16] = p_src->p_ya_axis_work_mode  & 0xFF;
    pData[17] = p_src->p_ya_axis_stat.all   & 0xFF;
    pData[18] = p_src->n_ya_axis_work_mode  & 0xFF;
    pData[19] = p_src->n_ya_axis_stat.all   & 0xFF;

    pData[20] = (p_src->p_ya_axis_angle_res >> 24)  & 0xFF;
    pData[21] = (p_src->p_ya_axis_angle_res >> 16)  & 0xFF;
    pData[22] = (p_src->p_ya_axis_angle_res >> 8)   & 0xFF;
    pData[23] = p_src->p_ya_axis_angle_res          & 0xFF;

    pData[24] = (p_src->n_ya_axis_angle_res >> 24)  & 0xFF;
    pData[25] = (p_src->n_ya_axis_angle_res >> 16)  & 0xFF;
    pData[26] = (p_src->n_ya_axis_angle_res >> 8)   & 0xFF;
    pData[27] = p_src->n_ya_axis_angle_res          & 0xFF;

    pData[28] = (p_src->p_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[29] = (p_src->p_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[30] = (p_src->p_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[31] = p_src->p_ya_axis_angle_mot          & 0xFF;

    pData[32] = (p_src->n_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[33] = (p_src->n_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[34] = (p_src->n_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[35] = p_src->n_ya_axis_angle_mot          & 0xFF;

    pData[36] = (p_src->p_ya_axis_angle_v >> 8)     & 0xFF;
    pData[37] = p_src->p_ya_axis_angle_v            & 0xFF;
    pData[38] = (p_src->n_ya_axis_angle_v >> 8)     & 0xFF;
    pData[39] = p_src->n_ya_axis_angle_v            & 0xFF;

    pData[40] = (p_src->p_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[41] = (p_src->p_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[42] = (p_src->p_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[43] = p_src->p_ya_axis_cmd_local          & 0xFF;
    pData[44] = (p_src->p_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[45] = p_src->p_ya_axis_cmd_speed          & 0xFF;

    pData[46] = (p_src->n_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[47] = (p_src->n_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[48] = (p_src->n_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[49] = p_src->n_ya_axis_cmd_local          & 0xFF;
    pData[50] = (p_src->n_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[51] = p_src->n_ya_axis_cmd_speed          & 0xFF;

    pData[52]  = p_src->p_ya_axis_line_center_loc   & 0xFF;
    pData[53] = p_src->p_ya_axis_input_wall        & 0xFF;
    pData[54] = p_src->n_ya_axis_line_center_loc   & 0xFF;
    pData[55] = p_src->n_ya_axis_input_wall        & 0xFF;
    pData[56] = p_src->rev1                  	& 0xFF;

    pData[57] = p_src->edac_single_err_cnt         & 0xFF;
    pData[58] = p_src->edac_double_err_cnt         & 0xFF;

    pData[59] = p_src->p_ya_hold_cur_set         & 0xFF;
    pData[60] = p_src->n_ya_hold_cur_set         & 0xFF;

    pData[61] = p_src->p_ya_max_acc_set         & 0xFF;
    pData[62] = p_src->n_ya_max_acc_set         & 0xFF;

    pData[63] = (p_src->p_ya_reset_speed_set >> 8)  & 0xFF;
    pData[64] = p_src->p_ya_reset_speed_set         & 0xFF;
    pData[65] = (p_src->n_ya_reset_speed_set >> 8)  & 0xFF;
    pData[66] = p_src->n_ya_reset_speed_set         & 0xFF;


    pData[67] = (p_src->p_ya_drv_cur_adc >> 8)   & 0xFF;
    pData[68] = p_src->p_ya_drv_cur_adc          & 0xFF;
    pData[69] = (p_src->n_ya_drv_cur_adc >> 8)   & 0xFF;
    pData[70] = p_src->n_ya_drv_cur_adc          & 0xFF;

    p_src->check_sum = CheckSumCalc(&pData[2], p_src->dat_len + 2);

    pData[71] = (p_src->check_sum >> 8)    & 0xFF;
    pData[72] = p_src->check_sum           & 0xFF;

    pData[73] = (p_src->frame_tile >> 8)   & 0xFF;
    pData[74] = p_src->frame_tile          & 0xFF;

    return 0;
}
#elif (BOARD_NAME == BOARD_DGM_4)
static int16 __tele_dat_to__sci_dat (tele_dat_t* p_src, Uint16 *pData)
{
    pData[0] = (p_src->frame_head >> 8) & 0xFF;
    pData[1] = p_src->frame_head        & 0xFF;
    pData[2] = (p_src->dat_len >> 8)    & 0xFF;
    pData[3] = p_src->dat_len           & 0xFF;
    pData[4] = p_src->serve_type        & 0xFF;
    pData[5] = p_src->serve_state       & 0xFF;

    pData[6]  = p_src->pri_bkup_id          & 0xFF;
    pData[7]  = p_src->tele_req_cnt         & 0xFF;
    pData[8]  = p_src->right_mov_cnt        & 0xFF;
    pData[9]  = p_src->right_conf_cnt       & 0xFF;
    pData[10] = p_src->wrong_cnt            & 0xFF;
    pData[11] = p_src->comm_rest_cnt        & 0xFF;

    pData[12] = p_src->p_ya_max_mov_cur_set    & 0xFF;
    pData[13] = p_src->n_ya_max_mov_cur_set    & 0xFF;
    pData[14] = p_src->p_yb_max_mov_cur_set    & 0xFF;
    pData[15] = p_src->n_yb_max_mov_cur_set    & 0xFF;
    
    pData[16] = p_src->last_cmd             & 0xFF;
    pData[17] = p_src->comm_stat.all        & 0xFF;
    pData[18] = p_src->p_ya_axis_work_mode  & 0xFF;
    pData[19] = p_src->p_ya_axis_stat.all   & 0xFF;
    pData[20] = p_src->n_ya_axis_work_mode  & 0xFF;
    pData[21] = p_src->n_ya_axis_stat.all   & 0xFF;
    pData[22] = p_src->p_yb_axis_work_mode  & 0xFF;
    pData[23] = p_src->p_yb_axis_stat.all   & 0xFF;
    pData[24] = p_src->n_yb_axis_work_mode  & 0xFF;
    pData[25] = p_src->n_yb_axis_stat.all   & 0xFF;

    pData[26] = (p_src->p_ya_axis_angle_res >> 24)  & 0xFF;
    pData[27] = (p_src->p_ya_axis_angle_res >> 16)  & 0xFF;
    pData[28] = (p_src->p_ya_axis_angle_res >> 8)   & 0xFF;
    pData[29] = p_src->p_ya_axis_angle_res          & 0xFF;

    pData[30] = (p_src->n_ya_axis_angle_res >> 24)  & 0xFF;
    pData[31] = (p_src->n_ya_axis_angle_res >> 16)  & 0xFF;
    pData[32] = (p_src->n_ya_axis_angle_res >> 8)   & 0xFF;
    pData[33] = p_src->n_ya_axis_angle_res          & 0xFF;

    pData[34] = (p_src->p_yb_axis_angle_res >> 24)  & 0xFF;
    pData[35] = (p_src->p_yb_axis_angle_res >> 16)  & 0xFF;
    pData[36] = (p_src->p_yb_axis_angle_res >> 8)   & 0xFF;
    pData[37] = p_src->p_yb_axis_angle_res          & 0xFF;

    pData[38] = (p_src->n_yb_axis_angle_res >> 24)  & 0xFF;
    pData[39] = (p_src->n_yb_axis_angle_res >> 16)  & 0xFF;
    pData[40] = (p_src->n_yb_axis_angle_res >> 8)   & 0xFF;
    pData[41] = p_src->n_yb_axis_angle_res          & 0xFF;

    pData[42] = (p_src->p_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[43] = (p_src->p_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[44] = (p_src->p_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[45] = p_src->p_ya_axis_angle_mot          & 0xFF;

    pData[46] = (p_src->n_ya_axis_angle_mot >> 24)  & 0xFF;
    pData[47] = (p_src->n_ya_axis_angle_mot >> 16)  & 0xFF;
    pData[48] = (p_src->n_ya_axis_angle_mot >> 8)   & 0xFF;
    pData[49] = p_src->n_ya_axis_angle_mot          & 0xFF;

    pData[50] = (p_src->p_yb_axis_angle_mot >> 24)  & 0xFF;
    pData[51] = (p_src->p_yb_axis_angle_mot >> 16)  & 0xFF;
    pData[52] = (p_src->p_yb_axis_angle_mot >> 8)   & 0xFF;
    pData[53] = p_src->p_yb_axis_angle_mot          & 0xFF;

    pData[54] = (p_src->n_yb_axis_angle_mot >> 24)  & 0xFF;
    pData[55] = (p_src->n_yb_axis_angle_mot >> 16)  & 0xFF;
    pData[56] = (p_src->n_yb_axis_angle_mot >> 8)   & 0xFF;
    pData[57] = p_src->n_yb_axis_angle_mot          & 0xFF;

    pData[58] = (p_src->p_ya_axis_angle_v >> 8)     & 0xFF;
    pData[59] = p_src->p_ya_axis_angle_v            & 0xFF;
    pData[60] = (p_src->n_ya_axis_angle_v >> 8)     & 0xFF;
    pData[61] = p_src->n_ya_axis_angle_v            & 0xFF;
    pData[62] = (p_src->p_yb_axis_angle_v >> 8)     & 0xFF;
    pData[63] = p_src->p_yb_axis_angle_v            & 0xFF;
    pData[64] = (p_src->n_yb_axis_angle_v >> 8)     & 0xFF;
    pData[65] = p_src->n_yb_axis_angle_v            & 0xFF;

    pData[66] = (p_src->p_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[67] = (p_src->p_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[68] = (p_src->p_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[69] = p_src->p_ya_axis_cmd_local          & 0xFF;
    pData[70] = (p_src->p_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[71] = p_src->p_ya_axis_cmd_speed          & 0xFF;

    pData[72] = (p_src->n_ya_axis_cmd_local >> 24)  & 0xFF;
    pData[73] = (p_src->n_ya_axis_cmd_local >> 16)  & 0xFF;
    pData[74] = (p_src->n_ya_axis_cmd_local >> 8)   & 0xFF;
    pData[75] = p_src->n_ya_axis_cmd_local          & 0xFF;
    pData[76] = (p_src->n_ya_axis_cmd_speed >> 8)   & 0xFF;
    pData[77] = p_src->n_ya_axis_cmd_speed          & 0xFF;

    pData[78] = (p_src->p_yb_axis_cmd_local >> 24)  & 0xFF;
    pData[79] = (p_src->p_yb_axis_cmd_local >> 16)  & 0xFF;
    pData[80] = (p_src->p_yb_axis_cmd_local >> 8)   & 0xFF;
    pData[81] = p_src->p_yb_axis_cmd_local          & 0xFF;
    pData[82] = (p_src->p_yb_axis_cmd_speed >> 8)   & 0xFF;
    pData[83] = p_src->p_yb_axis_cmd_speed          & 0xFF;

    pData[84] = (p_src->n_yb_axis_cmd_local >> 24)  & 0xFF;
    pData[85] = (p_src->n_yb_axis_cmd_local >> 16)  & 0xFF;
    pData[86] = (p_src->n_yb_axis_cmd_local >> 8)   & 0xFF;
    pData[87] = p_src->n_yb_axis_cmd_local          & 0xFF;
    pData[88] = (p_src->n_yb_axis_cmd_speed >> 8)   & 0xFF;
    pData[89] = p_src->n_yb_axis_cmd_speed          & 0xFF;

    pData[90]  = p_src->p_ya_axis_line_center_loc   & 0xFF;
    pData[91] = p_src->p_ya_axis_input_wall        & 0xFF;
    pData[92] = p_src->n_ya_axis_line_center_loc   & 0xFF;
    pData[93] = p_src->n_ya_axis_input_wall        & 0xFF;
    pData[94] = p_src->p_yb_temp                  & 0xFF;
    pData[95] = p_src->n_yb_temp                  & 0xFF;
    
    pData[96] = p_src->edac_single_err_cnt         & 0xFF;
    pData[97] = p_src->edac_double_err_cnt         & 0xFF;

    pData[98] = p_src->rev3[0]             & 0xFF;
    pData[99] = p_src->rev3[1]             & 0xFF;
    pData[100] = p_src->rev3[2]             & 0xFF;
    pData[101] = p_src->rev3[3]             & 0xFF;
 
    p_src->check_sum = CheckSumCalc(&pData[2], p_src->dat_len + 2);

    pData[102] = (p_src->check_sum >> 8)    & 0xFF;
    pData[103] = p_src->check_sum           & 0xFF;

    pData[104] = (p_src->frame_tile >> 8)   & 0xFF;
    pData[105] = p_src->frame_tile          & 0xFF;

    return 0;
}
#elif (BOARD_NAME == BOARD_GMS)
static int16 __tele_dat_to__sci_dat (tele_dat_t* p_src, Uint16 *pData)
{
	pData[0] = (p_src->frame_head >> 8) & 0xFF;
	pData[1] = p_src->frame_head 		& 0xFF;
	pData[2] = (p_src->dat_len >> 8) 	& 0xFF;
	pData[3] = p_src->dat_len 			& 0xFF;
	pData[4] = p_src->serve_type;
	pData[5] = p_src->serve_state;

	pData[6]  = p_src->pri_bkup_id;
	pData[7]  = p_src->tele_req_cnt;
	pData[8]  = p_src->right_mov_cnt;
	pData[9]  = p_src->right_conf_cnt;
	pData[10] = p_src->wrong_cnt;
	pData[11] = p_src->comm_rest_cnt;
	pData[12] = p_src->last_cmd;
	pData[13] = p_src->comm_stat.all;
	pData[14] = p_src->p_yb_axis_work_mode;
	pData[15] = p_src->p_yb_axis_stat.all;

	pData[16] = (p_src->p_yb_axis_angle_res >> 24) 	& 0xFF;
	pData[17] = (p_src->p_yb_axis_angle_res >> 16) 	& 0xFF;
	pData[18] = (p_src->p_yb_axis_angle_res >> 8) 	& 0xFF;
	pData[19] = p_src->p_yb_axis_angle_res 			& 0xFF;

	pData[20] = (p_src->p_yb_axis_angle_mot >> 24) 	& 0xFF;
	pData[21] = (p_src->p_yb_axis_angle_mot >> 16) 	& 0xFF;
	pData[22] = (p_src->p_yb_axis_angle_mot >> 8) 	& 0xFF;
	pData[23] = p_src->p_yb_axis_angle_mot 			& 0xFF;

	pData[24] = (p_src->p_yb_axis_angle_v >> 8) 	& 0xFF;
	pData[25] = p_src->p_yb_axis_angle_v 			& 0xFF;

	pData[26] = (p_src->p_yb_axis_cmd_local >> 24) 	& 0xFF;
	pData[27] = (p_src->p_yb_axis_cmd_local >> 16) 	& 0xFF;
	pData[28] = (p_src->p_yb_axis_cmd_local >> 8) 	& 0xFF;
	pData[29] = p_src->p_yb_axis_cmd_local 			& 0xFF;

	pData[30] = (p_src->p_yb_axis_cmd_speed >> 8) 	& 0xFF;
	pData[31] = p_src->p_yb_axis_cmd_speed 			& 0xFF;

	pData[32] = p_src->p_yb_cmd_work_mode;
	pData[33] = p_src->p_yb_max_acc_set;
	pData[34] = p_src->p_yb_drv_cur_set;
	pData[35] = p_src->p_yb_keep_cur_set;

    pData[36] = (p_src->p_yb_soft_lim_p >> 24)  & 0xFF;
    pData[37] = (p_src->p_yb_soft_lim_p >> 16)  & 0xFF;
    pData[38] = (p_src->p_yb_soft_lim_p >> 8)   & 0xFF;
    pData[39] = p_src->p_yb_soft_lim_p          & 0xFF;

    pData[40] = (p_src->p_yb_soft_lim_n >> 24)  & 0xFF;
    pData[41] = (p_src->p_yb_soft_lim_n >> 16)  & 0xFF;
    pData[42] = (p_src->p_yb_soft_lim_n >> 8)   & 0xFF;
    pData[43] = p_src->p_yb_soft_lim_n          & 0xFF;

    pData[44] = (p_src->p_yb_reset_speed_set >> 8)   & 0xFF;
    pData[45] = p_src->p_yb_reset_speed_set & 0xFF;

    pData[46] = p_src->cur_5v;

    pData[47] = p_src->program_run_addr & 0xFF;
    pData[48] = p_src->last_reset_reason & 0xFF;

    pData[49] = (p_src->system_reset_times >> 8) & 0xFF;
    pData[50] = p_src->system_reset_times & 0xFF;

    pData[51] = (p_src->drv_cur_adc >> 8)   & 0xFF;
    pData[52] = p_src->drv_cur_adc & 0xFF;

    pData[53] = (p_src->hall_angle >> 8)   & 0xFF;
    pData[54] = p_src->hall_angle & 0xFF;

    pData[55] = p_src->program_version;

    pData[56] = (p_src->program_err_bits >> 24)  & 0xFF;
    pData[57] = (p_src->program_err_bits >> 16)  & 0xFF;
    pData[58] = (p_src->program_err_bits >> 8)   & 0xFF;
    pData[59] = p_src->program_err_bits          & 0xFF;

    pData[60] = p_src->program_run_region;
    pData[61] = p_src->pps_cnt & 0xFF;

    pData[62] = p_src->soft_lim_err_stat;

	p_src->check_sum = CheckSumCalc(&pData[2], p_src->dat_len + 2);

	pData[63] = (p_src->check_sum >> 8) 	& 0xFF;
	pData[64] = p_src->check_sum 			& 0xFF;

	pData[65] = (p_src->frame_tile >> 8) 	& 0xFF;
	pData[66] = p_src->frame_tile 			& 0xFF;

	return 0;
}
#endif

/**
 * \brief 执行遥测请求指令
 *
 * \retval 成功返回0，失败返回-1
 */
#pragma CODE_SECTION(tele_req_cmd_run, "ramfuncs");
#if (BOARD_NAME == BOARD_CX20)
int16 tele_req_cmd_run(void)
{
    static Uint16 send_buf[115] = { 0 };
    single_axis_conf_t* p_axis = NULL;
    Uint16 i = 0;

    g_Tele_Dat.frame_head   = DAT_FRAME_HEAD;
    g_Tele_Dat.dat_len      = 0X006B;   //0x0047;
    g_Tele_Dat.serve_type   = 0x01;
    g_Tele_Dat.serve_state  = 0x55;

    g_Tele_Dat.pri_bkup_id = SADE_PRI_BKUP_ID;  /* 主备份ID */

    //g_Tele_Dat.comm_rest_cnt;     /* B11 通讯复位计数， todo  */

    g_Tele_Dat.sec_pow_24v = 88;        /* B12 二次电源24V，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.sec_pow_5v = 129;        /* B13 二次电源5V，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.sec_pow_1v2 = 46;        /* B14 二次电源1.2V，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.py_res_base = 232;       /* B15 +Y电位计激励基准，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.ny_res_base = 232;       /* B16 -Y电位计激励基准，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.fst_gen_cur = 49;        /* B17 一次母线电流，设备不支持，填一个合理范围内的固定值  */
    g_Tele_Dat.pow_5v_cur  = 39;        /* B18 5V供电电流，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.pow_3v3_cur = 39;        /* B19 3.3V供电电流，设备不支持，填一个合理范围内的固定值 */
    g_Tele_Dat.pow_1v2_cur = 116;       /* B20 1.2V供电电流，设备不支持，填一个合理范围内的固定值 */

    /* 如果是保持模式则返回保持模式的电流，否则返回运动模式的电流 */
    if (g_Axis_Conf.p_pya->work_mode_set == WORK_MODE_KEEP) {
        g_Tele_Dat.p_ya_axis_mot_cur = g_Axis_Conf.p_pya->keep_cur_given;  /* B21 +Y-A轴电机电流 */
    } else {
        g_Tele_Dat.p_ya_axis_mot_cur = g_Axis_Conf.p_pya->max_mov_cur_given;
    }

    if (g_Axis_Conf.p_nya->work_mode_set == WORK_MODE_KEEP) {
        g_Tele_Dat.n_ya_axis_mot_cur = g_Axis_Conf.p_nya->keep_cur_given;  /* B22 -Y-A轴电机电流 */
    } else {
        g_Tele_Dat.n_ya_axis_mot_cur = g_Axis_Conf.p_nya->max_mov_cur_given;
    }

    if (g_Axis_Conf.p_pyb->work_mode_set == WORK_MODE_KEEP) {
        g_Tele_Dat.p_yb_axis_mot_cur = g_Axis_Conf.p_pyb->keep_cur_given;  /* B23 +Y-B轴电机电流 */
    } else {
        g_Tele_Dat.p_yb_axis_mot_cur = g_Axis_Conf.p_pyb->max_mov_cur_given;
    }

    if (g_Axis_Conf.p_nyb->work_mode_set == WORK_MODE_KEEP) {
        g_Tele_Dat.n_yb_axis_mot_cur = g_Axis_Conf.p_nyb->keep_cur_given;  /* B23 +Y-B轴电机电流 */
    } else {
        g_Tele_Dat.n_yb_axis_mot_cur = g_Axis_Conf.p_nyb->max_mov_cur_given;
    }

    //g_Tele_Dat.last_cmd;              /* B25 最近执行指令码 */

    /* B26 通信状态 */
//  g_Tele_Dat.comm_stat.bit.cmd_stat = 0;
//  g_Tele_Dat.comm_stat.bit.dat_stat = 0;
//  g_Tele_Dat.comm_stat.bit.checksum_stat = 0;
//  g_Tele_Dat.comm_stat.bit.run_stat = 0;
//  g_Tele_Dat.comm_stat.bit.re_stat = 0;
    g_Tele_Dat.comm_stat.bit.rev = 0;

    g_Tele_Dat.p_ya_axis_work_mode = g_Axis_Conf.p_pya->work_mode_old; /* B27 +Y-A轴工作模式 */

    /* B28 +Y-A轴状态 */
    if (g_Axis_Conf.p_pya->drv_conf == 0x55) {     /* 驱动状态 */
        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 0;
    } else {
        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 1;
    }
    g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_pya->is_task_running;
    g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;   //无法判断是否过流
    g_Tele_Dat.p_ya_axis_stat.bit.temp_fault_stat = 0;  //todo 没有设定过温温度

    g_Tele_Dat.n_ya_axis_work_mode = (*(g_Axis_Conf.p_nya)).work_mode_old; /* B29 -Y-A轴工作模式 */

    /* B30 -Y-A轴状态 */
    if (g_Axis_Conf.p_nya->drv_conf == 0x55) {
        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 0;
    } else {
        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 1;
    }
    g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_nya->is_task_running;
    g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.n_ya_axis_stat.bit.temp_fault_stat = 0; //todo

    g_Tele_Dat.p_yb_axis_work_mode = g_Axis_Conf.p_pyb->work_mode_old; /* B31 +Y-B轴工作模式 */

    /* B32 +Y-B轴状态 */
    if (g_Axis_Conf.p_pyb->drv_conf == 0x55) {
        g_Tele_Dat.p_yb_axis_stat.bit.drv_stat = 0;
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.drv_stat = 1;
    }
    g_Tele_Dat.p_yb_axis_stat.bit.mov_stat = g_Axis_Conf.p_pyb->is_task_running;
    g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.p_yb_axis_stat.bit.temp_fault_stat = 0; //todo

    g_Tele_Dat.n_yb_axis_work_mode = g_Axis_Conf.p_nyb->work_mode_old; /* B33 -Y-B轴工作模式 */

    /* B34 -Y-B轴状态 */
    if (g_Axis_Conf.p_nyb->drv_conf == 0x55) {
        g_Tele_Dat.n_yb_axis_stat.bit.drv_stat = 0;
    } else {
        g_Tele_Dat.n_yb_axis_stat.bit.drv_stat = 1;
    }
    g_Tele_Dat.n_yb_axis_stat.bit.mov_stat = g_Axis_Conf.p_nyb->is_task_running;
    g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.n_yb_axis_stat.bit.temp_fault_stat = 0; //todo

    g_Tele_Dat.p_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_zhen, Y_POT2_zhen);     /* B35~B38 +Y-A轴角度值(电位计) */
    g_Tele_Dat.n_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_fu, Y_POT2_fu);     /* B39~B42 -Y-A轴角度值(电位计) */
    g_Tele_Dat.p_yb_axis_angle_res = Vol_To_Angle('B', Y_POT3_zhen, 0);      /* B43~B46 +Y-B轴角度值(电位计) */
    g_Tele_Dat.n_yb_axis_angle_res = Vol_To_Angle('B', Y_POT3_fu, 0);        /* B47~B50 -Y-A轴角度值(电位计) */

    g_Tele_Dat.p_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_pya);      /* B51~B54 +Y-A轴角度值(电机) */
    g_Tele_Dat.n_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_nya);      /* B55~B58 -Y-A轴角度值(电机) */
    g_Tele_Dat.p_yb_axis_angle_mot = g_Axis_Conf.p_pyb->cur_local;                 /* B59~B62 +Y-B轴角度值(电机) */
    g_Tele_Dat.n_yb_axis_angle_mot = g_Axis_Conf.p_nyb->cur_local;                 /* B63~B66 -Y-B轴角度值(电机) */


    /* B67~B68 +Y-A轴当前角速度 */
    /* B69~B70 -Y-A轴当前角速度 */
    /* B71~B72 +Y-B轴当前角速度 */
    /* B73~B74 -Y-B轴当前角速度 */
    //p_axis = g_Axis_Conf.p_pya;
    //for (i = 0; i < 4; i++) {
    //    if (p_axis->next_period_2_3_us != 0) {
    //        p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)p_axis->next_period_2_3_us * 2 / 3 * p_axis->speed_ratio * 16) * p_axis->stage[p_axis->cur_stage].mov_dir;
    //    } else {
    //        p_axis->cur_angle_speed = 0;
    //    }
    //    p_axis = p_axis->p_next_axis;
    //}
    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < 4; i++) {
        //if (p_axis->next_period_2_3_us != 0) {
        if (p_axis->next_segment_pulse_cnt != 0) {
            //p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)p_axis->next_period_2_3_us * 2 / 3 * p_axis->speed_ratio * 16) * p_axis->stage[p_axis->cur_stage].mov_dir;
            if ( p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * p_axis->next_segment_pulse_cnt * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            } else {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * ((float64)p_axis->uniform_segment_pulse_cnt_zheng + (float64)p_axis->uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            }

        } else {
            p_axis->cur_angle_speed = 0;
        }
        p_axis = p_axis->p_next_axis;
    }

    g_Tele_Dat.p_ya_axis_angle_v = g_Axis_Conf.p_pya->cur_angle_speed / g_Axis_Conf.p_pya->speedvalue_to_speed;
    g_Tele_Dat.n_ya_axis_angle_v = g_Axis_Conf.p_nya->cur_angle_speed / g_Axis_Conf.p_nya->speedvalue_to_speed;
    g_Tele_Dat.p_yb_axis_angle_v = g_Axis_Conf.p_pyb->cur_angle_speed / g_Axis_Conf.p_pyb->speedvalue_to_speed;
    g_Tele_Dat.n_yb_axis_angle_v = g_Axis_Conf.p_nyb->cur_angle_speed / g_Axis_Conf.p_nyb->speedvalue_to_speed;

    g_Tele_Dat.p_ya_axis_cmd_local = g_Axis_Conf.p_pya->loc_given;         /* B75~B78 +Y-A轴指令位置 */
    g_Tele_Dat.p_ya_axis_cmd_speed = g_Axis_Conf.p_pya->speed_given;       /* B79~B80 +Y-A轴指令速度 */
    g_Tele_Dat.n_ya_axis_cmd_local = g_Axis_Conf.p_nya->loc_given;         /* B81~B84 -Y-A轴指令位置 */
    g_Tele_Dat.n_ya_axis_cmd_speed = g_Axis_Conf.p_nya->speed_given;       /* B85~B86 -Y-A轴指令速度 */
    g_Tele_Dat.p_yb_axis_cmd_local = g_Axis_Conf.p_pyb->loc_given;         /* B87~B90 +Y-B轴指令位置 */
    g_Tele_Dat.p_yb_axis_cmd_speed = g_Axis_Conf.p_pyb->speed_given;       /* B91~B92 +Y-B轴指令速度 */
    g_Tele_Dat.n_yb_axis_cmd_local = g_Axis_Conf.p_nyb->loc_given;         /* B93~B96 -Y-B轴指令位置 */
    g_Tele_Dat.n_yb_axis_cmd_speed = g_Axis_Conf.p_nyb->speed_given;       /* B97~B98 -Y-B轴指令速度 */

    //todo 温度传感器的值
    g_Tele_Dat.p_ya_axis_line_center_loc = Y_TEMP1_zhen / 10;   /* B99  +Y-A轴线管中间位置 */
    g_Tele_Dat.p_ya_axis_input_wall = Y_TEMP3_zhen / 10;        /* B100 +Y-A轴输入轴系外壁 */
    g_Tele_Dat.n_ya_axis_line_center_loc = Y_TEMP1_fu / 10;     /* B101 -Y-A轴线管中间位置 */
    g_Tele_Dat.n_ya_axis_input_wall = Y_TEMP3_fu / 10;          /* B102 -Y-A轴输入轴系外壁 */

    g_Tele_Dat.rev1 = 0;                        /* B103 备用 */
    g_Tele_Dat.edac_single_err_cnt = 0;     /* B104 EDAC单错计数 */
    g_Tele_Dat.edac_double_err_cnt = 0;     /* B105 EDAC双错计数 */     //todo

    g_Tele_Dat.rev3[0] = 0;                 /* B106~B110 预留 */
    g_Tele_Dat.rev3[1] = 0;
    g_Tele_Dat.rev3[2] = 0;
    g_Tele_Dat.rev3[3] = 0;
    g_Tele_Dat.rev3[4] = 0;

    g_Tele_Dat.frame_tile = DAT_FRAME_TILE;

    g_Tele_Dat.tele_req_cnt++;  /* 遥测请求计数 */

    __tele_dat_to__sci_dat(&g_Tele_Dat, send_buf);

    SCI_SendData(send_buf, sizeof(send_buf));

    return 0;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int16 tele_req_cmd_run(void)
{
    static Uint16 send_buf[75] = { 0 };
    single_axis_conf_t* p_axis = NULL;
    Uint16 i = 0;

    g_Tele_Dat.frame_head   = DAT_FRAME_HEAD;
    g_Tele_Dat.dat_len      = 75 - 8;
    g_Tele_Dat.serve_type   = 0x01;
    g_Tele_Dat.serve_state  = 0x55;

    g_Tele_Dat.pri_bkup_id = SADE_PRI_BKUP_ID;  /* 主备份ID */

    //g_Tele_Dat.comm_rest_cnt;     /* B11 通讯复位计数， todo  */

    g_Tele_Dat.p_ya_max_mov_cur_set = g_Axis_Conf.p_pya->max_mov_cur_given;
    g_Tele_Dat.n_ya_max_mov_cur_set = g_Axis_Conf.p_nya->max_mov_cur_given;

    //g_Tele_Dat.last_cmd;              /* B25 最近执行指令码 */

    /* B26 通信状态 */
//  g_Tele_Dat.comm_stat.bit.cmd_stat = 0;
//  g_Tele_Dat.comm_stat.bit.dat_stat = 0;
//  g_Tele_Dat.comm_stat.bit.checksum_stat = 0;
//  g_Tele_Dat.comm_stat.bit.run_stat = 0;
//  g_Tele_Dat.comm_stat.bit.re_stat = 0;
    g_Tele_Dat.comm_stat.bit.rev = 0;

    g_Tele_Dat.p_ya_axis_work_mode = g_Axis_Conf.p_pya->work_mode_old; /* B27 +Y-A轴工作模式 */

    /* B28 +Y-A轴状态 */
//    if (g_Axis_Conf.p_pya->drv_conf == 0x55) {     /* 驱动状态 */
        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_pya->is_task_running;
    g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;   //无法判断是否过流
    g_Tele_Dat.p_ya_axis_stat.bit.temp_fault_stat = 0;  //todo 没有设定过温温度

    g_Tele_Dat.n_ya_axis_work_mode = (*(g_Axis_Conf.p_nya)).work_mode_old; /* B29 -Y-A轴工作模式 */

    /* B30 -Y-A轴状态 */
//    if (g_Axis_Conf.p_nya->drv_conf == 0x55) {
        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_nya->is_task_running;
    g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.n_ya_axis_stat.bit.temp_fault_stat = 0; //todo

#ifdef SHANGHAI_TEST
    g_Tele_Dat.p_ya_axis_angle_res = 0xFFFFFFFF;
    g_Tele_Dat.n_ya_axis_angle_res = 0xFFFFFFFF;
#else
	//g_Tele_Dat.p_ya_axis_angle_res = Y_POT1_fu *1000;     /* B35~B38 +Y-A轴角度值(电位计) */ //  2237
	//g_Tele_Dat.n_ya_axis_angle_res = Y_POT2_fu *1000;     /* B39~B42 -Y-A轴角度值(电位计) */

    g_Tele_Dat.p_ya_axis_angle_res = Vol_To_Angle('A', '+', 1) / 0.074;     /* B35~B38 +Y-A轴角度值(电位计) */
    g_Tele_Dat.n_ya_axis_angle_res = Vol_To_Angle('A', '-', 1) / 0.074;     /* B39~B42 -Y-A轴角度值(电位计) */

    //g_Tele_Dat.p_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_zhen, Y_POT2_zhen)  - *gp_boot_arg_angle_senser_adj_pya_1;     /* B35~B38 +Y-A轴角度值(电位计) */
    //g_Tele_Dat.n_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_fu, Y_POT2_fu) - *gp_boot_arg_angle_senser_adj_nya_1;     /* B39~B42 -Y-A轴角度值(电位计) */
#endif

    g_Tele_Dat.p_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_pya);      /* B51~B54 +Y-A轴角度值(电机) */
    g_Tele_Dat.n_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_nya);      /* B55~B58 -Y-A轴角度值(电机) */

#if 0
    float32 vol_temp;
    if (VOL_PYA_A > VOL_PYA_B) {
        vol_temp = VOL_PYA_A;
    } else {
        vol_temp = VOL_PYA_B;
    }

    /* 保持模式下最大值，平均值和有效值都是同一个数 */
    if (g_Axis_Conf.p_pya->work_mode_set == WORK_MODE_KEEP || g_Axis_Conf.p_pya->work_mode_set == WORK_MODE_STANDBY) {
		double val = nearbyint(vol_temp * 20.0948);          /* B51~B52 驱动电流(vol_temp * 1000000 / 13200 * 100 / 377) */\
		g_Tele_Dat.p_ya_drv_cur_adc = val;
    } else {    /* 运动状态下用有效值 */
    	g_Tele_Dat.p_ya_drv_cur_adc = nearbyint(vol_temp * 14.209205);         /* nearbyint(vol_temp * 20.0948) / 1.41421; */
    }
    //get_cur_set_factor();   /* 得到占空比系数，跑这步的时候把上两个数小数点往后移一位  */
#else
	g_Tele_Dat.p_ya_drv_cur_adc = 0x00;
	g_Tele_Dat.p_ya_drv_cur_adc = 0x00;
#endif

    /* B67~B68 +Y-A轴当前角速度 */
    /* B69~B70 -Y-A轴当前角速度 */
    /* B71~B72 +Y-B轴当前角速度 */
    /* B73~B74 -Y-B轴当前角速度 */
    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < AXIS_CNT; i++) {
        if (p_axis->next_segment_pulse_cnt != 0) {
            if ( p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * p_axis->next_segment_pulse_cnt * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            } else {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * ((float64)p_axis->uniform_segment_pulse_cnt_zheng + (float64)p_axis->uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            }

        } else {
            p_axis->cur_angle_speed = 0;
        }
        p_axis = p_axis->p_next_axis;
    }
    g_Tele_Dat.p_ya_axis_angle_v = g_Axis_Conf.p_pya->cur_angle_speed / g_Axis_Conf.p_pya->speedvalue_to_speed;
    g_Tele_Dat.n_ya_axis_angle_v = g_Axis_Conf.p_nya->cur_angle_speed / g_Axis_Conf.p_nya->speedvalue_to_speed;
#if 0
    if (g_Axis_Conf.p_pya->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.p_ya_axis_angle_v == g_Tele_Dat.p_ya_axis_cmd_speed)
    	{
            g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = 1;
    	}
    }

    if (g_Axis_Conf.p_nya->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.n_ya_axis_angle_v == g_Tele_Dat.n_ya_axis_cmd_speed)
    	{
            g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = 1;
    	}
    }
#endif
    g_Tele_Dat.p_ya_axis_cmd_local = g_Axis_Conf.p_pya->loc_given;         /* B75~B78 +Y-A轴指令位置 */
    g_Tele_Dat.p_ya_axis_cmd_speed = g_Axis_Conf.p_pya->speed_given;       /* B79~B80 +Y-A轴指令速度 */
    g_Tele_Dat.n_ya_axis_cmd_local = g_Axis_Conf.p_nya->loc_given;         /* B81~B84 -Y-A轴指令位置 */
    g_Tele_Dat.n_ya_axis_cmd_speed = g_Axis_Conf.p_nya->speed_given;       /* B85~B86 -Y-A轴指令速度 */

    //todo 温度传感器的值
    g_Tele_Dat.p_ya_axis_line_center_loc = Y_TEMP2_zhen * 4095.0 / 5.0 / 16.0;   /* B99  +Y-A轴线管中间位置 */
    g_Tele_Dat.p_ya_axis_input_wall = 0x00;        /* B100 +Y-A轴输入轴系外壁 */
    g_Tele_Dat.n_ya_axis_line_center_loc = Y_TEMP2_fu * 4095.0 / 5.0 / 16.0;     /* B101 -Y-A轴线管中间位置 */
    g_Tele_Dat.n_ya_axis_input_wall = 0x00;          /* B102 -Y-A轴输入轴系外壁 */

    g_Tele_Dat.rev1 = 0;

    g_Tele_Dat.edac_single_err_cnt = 0;     /* B104 EDAC单错计数 */
    g_Tele_Dat.edac_double_err_cnt = 0;     /* B105 EDAC双错计数 */

    g_Tele_Dat.p_ya_hold_cur_set = g_Axis_Conf.p_pya->keep_cur_given;   	/* B59 +Y-A轴保持电流设置 */
    g_Tele_Dat.n_ya_hold_cur_set = g_Axis_Conf.p_nya->keep_cur_given;   	/* B60 -Y-A轴保持电流设置 */

    g_Tele_Dat.p_ya_max_acc_set = g_Axis_Conf.p_pya->max_acc;		/* B61 +Y-A最大加速度设置 */
    g_Tele_Dat.n_ya_max_acc_set = g_Axis_Conf.p_nya->max_acc;		/* B62 -Y-A最大加速度设置 */

    g_Tele_Dat.p_ya_reset_speed_set = g_Axis_Conf.p_pya->reset_speed_given;	/* B63~B64 +Y-A复位速度设置 */
    g_Tele_Dat.n_ya_reset_speed_set = g_Axis_Conf.p_nya->reset_speed_given;	/* B65~B66 -Y-A复位速度设置 */

    g_Tele_Dat.frame_tile = DAT_FRAME_TILE;

    g_Tele_Dat.tele_req_cnt++;  /* 遥测请求计数 */

    __tele_dat_to__sci_dat(&g_Tele_Dat, send_buf);

    SCI_SendData(send_buf, sizeof(send_buf));
    return 0;
}
#elif  (BOARD_NAME == BOARD_DGM_4)
int16 tele_req_cmd_run(void)
{
    static Uint16 send_buf[106] = { 0 };
    single_axis_conf_t* p_axis = NULL;
    Uint16 i = 0;

    g_Tele_Dat.frame_head   = DAT_FRAME_HEAD;
    g_Tele_Dat.dat_len      = 0X0062;
    g_Tele_Dat.serve_type   = 0x01;
    g_Tele_Dat.serve_state  = 0x55;

    g_Tele_Dat.pri_bkup_id = SADE_PRI_BKUP_ID;  /* 主备份ID */
    //g_Tele_Dat.right_mov_cnt;          /* B8 正确运动指令帧计数(该字段在函数RevDataRead()内填充) */
    //g_Tele_Dat.right_conf_cnt;         /* B9 正确配置指令帧计数(该字段在函数RevDataRead()内填充) */
    //g_Tele_Dat.wrong_cnt;              /* B10 错误指令帧计数 (该字段在函数RevDataRead()内填充)*/
    //g_Tele_Dat.comm_rest_cnt;     	 /* B11 通讯复位计数 (该字段在函数App_task()内填充) */

    g_Tele_Dat.p_ya_max_mov_cur_set = g_Axis_Conf.p_pya->max_mov_cur_given;
    g_Tele_Dat.n_ya_max_mov_cur_set = g_Axis_Conf.p_nya->max_mov_cur_given;
    g_Tele_Dat.p_yb_max_mov_cur_set = g_Axis_Conf.p_pyb->max_mov_cur_given;
    g_Tele_Dat.n_yb_max_mov_cur_set = g_Axis_Conf.p_nyb->max_mov_cur_given;

    //g_Tele_Dat.last_cmd;              /* B16 最近执行指令码(该字段在函数RevDataRead()内填充) */

    /* B17 通信状态 */
	//g_Tele_Dat.comm_stat.bit.cmd_stat; 			/* B0：指令码状态，0正确，1错误；(该字段在函数RevDataRead()内填充) */
	//g_Tele_Dat.comm_stat.bit.dat_stat;			/* B1：数据状态，0正确，1错误；(该字段在函数RevDataRead()内填充) */
	//g_Tele_Dat.comm_stat.bit.checksum_stat;		/* B2：校验和状态，0正确，1错误；(该字段在函数RevDataRead()内填充) */
	//g_Tele_Dat.comm_stat.bit.run_stat;			/* B4B3：运行状态，11重构正序，其它原始程序；(该字段在函数RevDataRead()内填充) */
	//g_Tele_Dat.comm_stat.bit.re_stat;				/* B5：重构状态，0空闲，1重构中；(该字段在函数RevDataRead()内填充, BOARD_DGM_4 项目此比特也是预留)*/
	g_Tele_Dat.comm_stat.bit.rev = 0;				/* B7B6 B5：预留，暂时填0。*/

    g_Tele_Dat.p_ya_axis_work_mode = g_Axis_Conf.p_pya->work_mode_old; /* B27 +Y-A轴工作模式 */
    /* B28 +Y-A轴状态 */
//    if (g_Axis_Conf.p_pya->drv_conf == 0x55) {     /* 驱动状态 */
        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.p_ya_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_pya->is_task_running;
    g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;   //无法判断是否过流
    g_Tele_Dat.p_ya_axis_stat.bit.temp_fault_stat = 0;  //todo 没有设定过温温度

    g_Tele_Dat.n_ya_axis_work_mode = g_Axis_Conf.p_nya->work_mode_old; /* B29 -Y-A轴工作模式 */
    /* B30 -Y-A轴状态 */
//    if (g_Axis_Conf.p_nya->drv_conf == 0x55) {
        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.n_ya_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = g_Axis_Conf.p_nya->is_task_running;
    g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.n_ya_axis_stat.bit.temp_fault_stat = 0; //todo

    g_Tele_Dat.p_yb_axis_work_mode = g_Axis_Conf.p_pyb->work_mode_old; /* B31 +Y-B轴工作模式 */
    /* B32 +Y-B轴状态 */
//    if (g_Axis_Conf.p_pyb->drv_conf == 0x55) {
        g_Tele_Dat.p_yb_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.p_yb_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.p_yb_axis_stat.bit.mov_stat = g_Axis_Conf.p_pyb->is_task_running;
    g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.p_yb_axis_stat.bit.temp_fault_stat = 0; //todo

    g_Tele_Dat.n_yb_axis_work_mode = g_Axis_Conf.p_nyb->work_mode_old; /* B33 -Y-B轴工作模式 */
    /* B34 -Y-B轴状态 */
//    if (g_Axis_Conf.p_nyb->drv_conf == 0x55) {
        g_Tele_Dat.n_yb_axis_stat.bit.drv_stat = 0;
//    } else {
//        g_Tele_Dat.n_yb_axis_stat.bit.drv_stat = 1;
//    }
    g_Tele_Dat.n_yb_axis_stat.bit.mov_stat = g_Axis_Conf.p_nyb->is_task_running;
    g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
    g_Tele_Dat.n_yb_axis_stat.bit.temp_fault_stat = 0; //todo

#ifdef SHANGHAI_TEST
    g_Tele_Dat.p_ya_axis_angle_res = 0xFFFFFFFF;
    g_Tele_Dat.n_ya_axis_angle_res = 0xFFFFFFFF;
    g_Tele_Dat.p_yb_axis_angle_res = 0xFFFFFFFF;
    g_Tele_Dat.n_yb_axis_angle_res = 0xFFFFFFFF;
#else
    g_Tele_Dat.p_ya_axis_angle_res = (Vol_To_Angle('A', '+', 1)) / 0.074;     /* B35~B38 +Y-A轴角度值(电位计) */
    g_Tele_Dat.n_ya_axis_angle_res = (Vol_To_Angle('A', '-', 1)) / 0.074;     /* B39~B42 -Y-A轴角度值(电位计) */
    g_Tele_Dat.p_yb_axis_angle_res = (Vol_To_Angle('B', '+', 1)) / 0.074;     /* B35~B38 +Y-B轴角度值(电位计) */
    g_Tele_Dat.n_yb_axis_angle_res = (Vol_To_Angle('B', '-', 1)) / 0.074;     /* B39~B42 -Y-B轴角度值(电位计) */
    //g_Tele_Dat.p_yb_axis_angle_res = Y_POT3_zhen * 1000;
    //g_Tele_Dat.n_yb_axis_angle_res = Y_POT3_fu * 1000;

//    g_Tele_Dat.p_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_zhen, Y_POT2_zhen);     	/* B35~B38 +Y-A轴角度值(电位计) */
//    g_Tele_Dat.n_ya_axis_angle_res = Vol_To_Angle('A', Y_POT1_fu, Y_POT2_fu);     		/* B39~B42 -Y-A轴角度值(电位计) */
//    g_Tele_Dat.p_yb_axis_angle_res = Vol_To_Angle('B', Y_POT3_zhen, 0);      			/* B43~B46 +Y-B轴角度值(电位计) */
//    g_Tele_Dat.n_yb_axis_angle_res = Vol_To_Angle('B', Y_POT3_fu, 0);        			/* B47~B50 -Y-B轴角度值(电位计) */

#endif
    g_Tele_Dat.p_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_pya);      		/* B51~B54 +Y-A轴角度值(电机) */
    g_Tele_Dat.n_ya_axis_angle_mot = cur_local_modify_A(g_Axis_Conf.p_nya);      		/* B55~B58 -Y-A轴角度值(电机) */
    g_Tele_Dat.p_yb_axis_angle_mot = g_Axis_Conf.p_pyb->cur_local;                 		/* B59~B62 +Y-B轴角度值(电机) */
    g_Tele_Dat.n_yb_axis_angle_mot = g_Axis_Conf.p_nyb->cur_local;                 		/* B63~B66 -Y-B轴角度值(电机) */


    /* B67~B68 +Y-A轴当前角速度 */
    /* B69~B70 -Y-A轴当前角速度 */
    /* B71~B72 +Y-B轴当前角速度 */
    /* B73~B74 -Y-B轴当前角速度 */
    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < 4; i++) {
        //if (p_axis->next_period_2_3_us != 0) {
        if (p_axis->next_segment_pulse_cnt != 0) {
            //p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)p_axis->next_period_2_3_us * 2 / 3 * p_axis->speed_ratio * 16) * p_axis->stage[p_axis->cur_stage].mov_dir;
            if ( p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * p_axis->next_segment_pulse_cnt * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            } else {
            	p_axis->cur_angle_speed = (float64)1000000 * p_axis->step_angle / ((float64)XIFEN_CNT * ((float64)p_axis->uniform_segment_pulse_cnt_zheng + (float64)p_axis->uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * p_axis->speed_ratio) * p_axis->stage[p_axis->cur_stage].mov_dir;
            }

        } else {
            p_axis->cur_angle_speed = 0;
        }
        p_axis = p_axis->p_next_axis;
    }
    g_Tele_Dat.p_ya_axis_angle_v = g_Axis_Conf.p_pya->cur_angle_speed / g_Axis_Conf.p_pya->speedvalue_to_speed;
    g_Tele_Dat.n_ya_axis_angle_v = g_Axis_Conf.p_nya->cur_angle_speed / g_Axis_Conf.p_nya->speedvalue_to_speed;
    g_Tele_Dat.p_yb_axis_angle_v = g_Axis_Conf.p_pyb->cur_angle_speed / g_Axis_Conf.p_pyb->speedvalue_to_speed;
    g_Tele_Dat.n_yb_axis_angle_v = g_Axis_Conf.p_nyb->cur_angle_speed / g_Axis_Conf.p_nyb->speedvalue_to_speed;
#if 0
    if (g_Axis_Conf.p_pya->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.p_ya_axis_angle_v == g_Tele_Dat.p_ya_axis_cmd_speed)
    	{
            g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.p_ya_axis_stat.bit.mov_stat = 1;
    	}
    }

    if (g_Axis_Conf.p_nya->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.n_ya_axis_angle_v == g_Tele_Dat.n_ya_axis_cmd_speed)
    	{
            g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.n_ya_axis_stat.bit.mov_stat = 1;
    	}
    }
    if (g_Axis_Conf.p_pyb->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.p_yb_axis_angle_v == g_Tele_Dat.p_yb_axis_cmd_speed)
    	{
            g_Tele_Dat.p_yb_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.p_yb_axis_stat.bit.mov_stat = 1;
    	}
    }
    if (g_Axis_Conf.p_nyb->work_mode_old == WORK_MODE_TRACE)
    {
    	if (g_Tele_Dat.n_yb_axis_angle_v == g_Tele_Dat.n_yb_axis_cmd_speed)
    	{
            g_Tele_Dat.n_yb_axis_stat.bit.mov_stat = 0;
    	}else{
            g_Tele_Dat.n_yb_axis_stat.bit.mov_stat = 1;
    	}
    }
#endif
    g_Tele_Dat.p_ya_axis_cmd_local = g_Axis_Conf.p_pya->loc_given;         /* B75~B78 +Y-A轴指令位置 */
    g_Tele_Dat.p_ya_axis_cmd_speed = g_Axis_Conf.p_pya->speed_given;       /* B79~B80 +Y-A轴指令速度 */
    g_Tele_Dat.n_ya_axis_cmd_local = g_Axis_Conf.p_nya->loc_given;         /* B81~B84 -Y-A轴指令位置 */
    g_Tele_Dat.n_ya_axis_cmd_speed = g_Axis_Conf.p_nya->speed_given;       /* B85~B86 -Y-A轴指令速度 */
    g_Tele_Dat.p_yb_axis_cmd_local = g_Axis_Conf.p_pyb->loc_given;         /* B87~B90 +Y-B轴指令位置 */
    g_Tele_Dat.p_yb_axis_cmd_speed = g_Axis_Conf.p_pyb->speed_given;       /* B91~B92 +Y-B轴指令速度 */
    g_Tele_Dat.n_yb_axis_cmd_local = g_Axis_Conf.p_nyb->loc_given;         /* B93~B96 -Y-B轴指令位置 */
    g_Tele_Dat.n_yb_axis_cmd_speed = g_Axis_Conf.p_nyb->speed_given;       /* B97~B98 -Y-B轴指令速度 */

    //todo 温度传感器的值
    g_Tele_Dat.p_ya_axis_line_center_loc = Y_TEMP2_zhen * 4095.0 / 5.0 / 16.0;  /* B99  +Y-A轴线管中间位置 */
    g_Tele_Dat.p_ya_axis_input_wall = 0X00;        /* B100 +Y-A轴输入轴系外壁 */
    g_Tele_Dat.n_ya_axis_line_center_loc = Y_TEMP2_fu * 4095.0 / 5.0 / 16.0;     /* B101 -Y-A轴线管中间位置 */
    g_Tele_Dat.n_ya_axis_input_wall = 0X00;          /* B102 -Y-A轴输入轴系外壁 */

    g_Tele_Dat.p_yb_temp = g_Axis_Conf.p_pyb->keep_cur_given;   //TODO
    g_Tele_Dat.n_yb_temp = g_Axis_Conf.p_nyb->keep_cur_given;     //TODO
#if 0
    g_Tele_Dat.edac_single_err_cnt = 0;     /* B104 EDAC单错计数 */
    g_Tele_Dat.edac_double_err_cnt = 0;     /* B105 EDAC双错计数 */     //todo

    g_Tele_Dat.rev3[0] = 0;                 /* B106~B110 预留 */
    g_Tele_Dat.rev3[1] = 0;
    g_Tele_Dat.rev3[2] = 0;
    g_Tele_Dat.rev3[3] = 0;
#endif
    g_Tele_Dat.frame_tile = DAT_FRAME_TILE;

    g_Tele_Dat.tele_req_cnt++;  /* 遥测请求计数 */

    __tele_dat_to__sci_dat(&g_Tele_Dat, send_buf);

    SCI_SendData(send_buf, sizeof(send_buf));

    return 0;
}
#elif (BOARD_NAME == BOARD_GMS)
extern Uint16 g_last_reset_is_wdt;   /* 上次复位原因是否是因为看门狗 */
extern Uint16 g_CJW_HE_5701_angle;
extern void get_cur_set_factor (void);
int16 tele_req_cmd_run(void)
{
	static Uint16 send_buf[67] = { 0 };
//	Uint16 angle = 0;

	g_Tele_Dat.frame_head 	= DAT_FRAME_HEAD;
	g_Tele_Dat.dat_len 		= 0X003B;
	g_Tele_Dat.serve_type 	= 0x01;
	g_Tele_Dat.serve_state  = 0x00;

	g_Tele_Dat.pri_bkup_id = SADE_PRI_BKUP_ID;	/* 主备份ID */

	//g_Tele_Dat.last_cmd;				/* B12 最近执行指令码 */

	/* B13 通信状态 */
	g_Tele_Dat.comm_stat.bit.rev = 0;

	if ((g_Axis_Conf.p_pyb->reciprocat_mode_enable == 1) && (g_Axis_Conf.p_pyb->work_mode_set == WORK_MODE_CAPTURE)) {
	    g_Tele_Dat.p_yb_axis_work_mode = WORK_MODE_TRACE;
	} else {
	    g_Tele_Dat.p_yb_axis_work_mode = g_Axis_Conf.p_pyb->work_mode_old;  /* B14 +Y-B轴工作模式 */
	}

	/* B15 +Y-B轴状态 */
	g_Tele_Dat.p_yb_axis_stat.bit.mov_stat = g_Axis_Conf.p_pyb->is_task_running;

	/* 在main函数中填充 */
	//g_Tele_Dat.p_yb_axis_angle_res = 0;	/* B16~B19 +Y-B轴角度值(角位移传感器) */

	g_Tele_Dat.p_yb_axis_angle_mot = g_Axis_Conf.p_pyb->cur_local;		    /* B20~B23 +Y-B轴角度值(电机) */

	/* B24~B25 +Y-B轴当前角速度 */
    //if (g_Axis_Conf.p_pyb->next_period_2_3_us != 0) {
    if (g_Axis_Conf.p_pyb->next_segment_pulse_cnt != 0) {
        //g_Axis_Conf.p_pyb->cur_angle_speed = (float64)1000000 * g_Axis_Conf.p_pyb->step_angle / ((float64)g_Axis_Conf.p_pyb->next_period_2_3_us * 2 / 3 * g_Axis_Conf.p_pyb->speed_ratio * 16) * g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir;
        if ( g_Axis_Conf.p_pyb->stage[ g_Axis_Conf.p_pyb->cur_stage].a_1000 != 0) {
            g_Axis_Conf.p_pyb->cur_angle_speed = (float64)1000000 * g_Axis_Conf.p_pyb->step_angle / ((float64)XIFEN_CNT * g_Axis_Conf.p_pyb->next_segment_pulse_cnt * 50 * g_Axis_Conf.p_pyb->speed_ratio) * g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir;
        } else {
            g_Axis_Conf.p_pyb->cur_angle_speed = (float64)1000000 * g_Axis_Conf.p_pyb->step_angle / ((float64)XIFEN_CNT * ((float64)g_Axis_Conf.p_pyb->uniform_segment_pulse_cnt_zheng + (float64)g_Axis_Conf.p_pyb->uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * g_Axis_Conf.p_pyb->speed_ratio) * g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir;
        }
    } else {
        g_Axis_Conf.p_pyb->cur_angle_speed = 0;
    }

	g_Tele_Dat.p_yb_axis_angle_v = g_Axis_Conf.p_pyb->cur_angle_speed / g_Axis_Conf.p_pyb->speedvalue_to_speed;

	g_Tele_Dat.p_yb_axis_cmd_local = g_Axis_Conf.p_pyb->loc_given;			/* B26~B29 +Y-B轴指令位置 */
	g_Tele_Dat.p_yb_axis_cmd_speed = g_Axis_Conf.p_pyb->speed_given;		/* B30~B31 +Y-B轴指令速度 */

	if ((g_Axis_Conf.p_pyb->reciprocat_mode_enable == 1) && (g_Axis_Conf.p_pyb->work_mode_set == WORK_MODE_CAPTURE)) {
	    g_Tele_Dat.p_yb_cmd_work_mode = WORK_MODE_TRACE;
	} else {
	    g_Tele_Dat.p_yb_cmd_work_mode = g_Axis_Conf.p_pyb->work_mode_set;       /* B32 +Y-B轴指令工作模式 */
    }

    g_Tele_Dat.p_yb_max_acc_set = g_Axis_Conf.p_pyb->max_acc;               /* B33 +Y-B最大加速度设置 */
    g_Tele_Dat.p_yb_drv_cur_set = g_Axis_Conf.p_pyb->max_mov_cur_given;     /* B34 +Y-B轴驱动电流设置 */
    g_Tele_Dat.p_yb_keep_cur_set = g_Axis_Conf.p_pyb->keep_cur_given;       /* B35 +Y-B轴保持电流设置 */


    g_Tele_Dat.p_yb_soft_lim_p = g_Axis_Conf.p_pyb->soft_lim_p;             /* B36~B39 SADM软件限位配置(正方向) */
    g_Tele_Dat.p_yb_soft_lim_n = g_Axis_Conf.p_pyb->soft_lim_n;             /* B40~B43 SADM软件限位配置(负方向) */

    g_Tele_Dat.p_yb_reset_speed_set = g_Axis_Conf.p_pyb->reset_speed_given; /* B44~B45 回零速度设置值 */
    g_Tele_Dat.cur_5v = VOL_CUR_5V * 50;                                     /* B46 5V电流遥测(VOL_CUR_5V / 20 / 0.1 * 100) */


//    g_Tele_Dat.temp_senser_vol = 0;                                         /* B47~B48 温度传感器供电电压（多媒体预留） */
//    g_Tele_Dat.temperature = 0;                                             /* B49~B50 测温值（多媒体预留） */

    float32 vol_temp;
    if (VOL_PYB_A > VOL_PYB_B) {
        vol_temp = VOL_PYB_A;
    } else {
        vol_temp = VOL_PYB_B;
    }

    /* 保持模式下最大值，平均值和有效值都是同一个数 */
    if (g_Axis_Conf.p_pyb->work_mode_set == WORK_MODE_KEEP) {
        g_Tele_Dat.drv_cur_adc = nearbyint(vol_temp * 52.0102);          /* B51~B52 驱动电流(vol_temp * 1000000 / 5100 * 100 / 377) */
    } else {    /* 运动状态下用有效值 */
        g_Tele_Dat.drv_cur_adc = nearbyint(vol_temp * 36.77677);         /* nearbyint(vol_temp * 52.0102) / 1.41421; */
    }

    //get_cur_set_factor();   /* 得到占空比系数，跑这步的时候把上两个数小数点往后移一位  */

    g_Tele_Dat.hall_angle = g_Axis_Conf.p_pyb->hall_angle * 100;                  /* B53~B54 霍尔角度 */
    g_Tele_Dat.program_version = PROGRAM_VERSION;                           /* B55 程序版本号 */

    g_Tele_Dat.program_err_bits = *gp_boot_arg_err_bits;                   /* B56~B59 程序校验错误bit计数 */

    g_Tele_Dat.program_run_region = *gp_boot_arg_boot_region;              /* B60 当前程序运行区 */

    g_Tele_Dat.pps_cnt = g_pps;                                         /* B61 PPS计数值 */

	//g_Tele_Dat.soft_lim_err_stat = 0;                                      			/* B62 预留 */

	g_Tele_Dat.frame_tile = DAT_FRAME_TILE;

	g_Tele_Dat.tele_req_cnt++;	/* 遥测请求计数 */

	__tele_dat_to__sci_dat(&g_Tele_Dat, send_buf);

	SCI_SendData(send_buf, sizeof(send_buf));

	return 0;
}

#endif
