/*
 * angle_sensor_sw_cmd.c
 *
 *  Created on: 2024��8��21��
 *      Author: Suzkfly
 */
#include "cmd.h"
#if BOARD_NAME == BOARD_GMS
#include "angle_sensor_sw_cmd.h"
#include "GL_CJW_HE_5701.h"
int8_t get_angle_sensor_sw_dat (const Uint16 *pData)
{
	if (g_Axis_Conf.p_pyb->is_task_running == 0) {
		g_angle_senser_cmd_eff_flag = 1;	/* ָ����Ч */

		if (pData[6] == 0x55) {			/* �Ƕȴ������� */
			g_angle_senser_cmd_en = 0;
		} else if (pData[6] == 0x00) {	/* �Ƕȴ������� */
			g_angle_senser_cmd_en = 1;
			g_angle_senser_hw_en = 1;
			g_angle_senser_hw_normal = 1;
			CJW_HE_5701_Power_On();
		} else {
			return -1;
		}
		return 0;
	}

	return -1;
}

#endif

