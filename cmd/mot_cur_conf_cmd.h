/*
 * mot_cur_conf_cmd.h
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_MOT_CUR_CONF_CMD_H_
#define SOURCE_CMD_MOT_CUR_CONF_CMD_H_

#include "project.h"

/**
 * \brief ���õ������ֵ
 */
extern void set_mot_cur (void);

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
extern int8_t mot_cur_handler (const Uint16 *pData);

/**
 * \brief �Զ����ռ�ձ�ϵ��
 */
extern void get_cur_set_factor (void);


#endif /* SOURCE_CMD_MOT_CUR_CONF_CMD_H_ */
