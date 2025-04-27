/*
 * sadmb_soft_lim_conf_cmd.h
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_SADMB_SOFT_LIM_CONF_CMD_H_
#define SOURCE_CMD_SADMB_SOFT_LIM_CONF_CMD_H_

#include "project.h"

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
extern int8_t get_sadmb_soft_lim_conf_dat (const Uint16 *pData);

#endif /* SOURCE_CMD_SADMB_SOFT_LIM_CONF_CMD_H_ */
