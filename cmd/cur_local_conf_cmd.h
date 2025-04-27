/*
 * cur_loccal_conf_cmd.h
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_CUR_LOCAL_CONF_CMD_H_
#define SOURCE_CMD_CUR_LOCAL_CONF_CMD_H_

#include "project.h"     // DSP2833x Headerfile Include File
#if (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_DGM_2)
extern float32 g_angle_senser_pya;
extern float32 g_angle_senser_nya;
extern float32 g_angle_senser_pyb;
extern float32 g_angle_senser_nyb;
#endif

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
extern int8_t get_cur_local_conf_dat (const Uint16 *pData);


#endif /* SOURCE_CMD_CUR_LOCAL_CONF_CMD_H_ */
