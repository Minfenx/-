/*
 * mot_overcur_prot_conf_cmd.h
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */
#if 0
#ifndef SOURCE_CMD_MOT_OVERCUR_PROT_CONF_CMD_H_
#define SOURCE_CMD_MOT_OVERCUR_PROT_CONF_CMD_H_

#include "project.h"     // DSP2833x Headerfile Include File

/**
 * \brief 串口数据处理
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
extern int8_t mot_overcur_prot_handler (const Uint16 *pData);


#endif /* SOURCE_CMD_MOT_OVERCUR_PROT_CONF_CMD_H_ */

#endif
