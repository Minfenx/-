/*
 * mot_cur_conf_cmd.h
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_MOT_CUR_CONF_CMD_H_
#define SOURCE_CMD_MOT_CUR_CONF_CMD_H_

#include "project.h"

/**
 * \brief 设置电机电流值
 */
extern void set_mot_cur (void);

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
extern int8_t mot_cur_handler (const Uint16 *pData);

/**
 * \brief 自动获得占空比系数
 */
extern void get_cur_set_factor (void);


#endif /* SOURCE_CMD_MOT_CUR_CONF_CMD_H_ */
