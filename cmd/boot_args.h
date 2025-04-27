/*
 * boot_args.h
 *
 *  Created on: 2024��3��17��
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_BOOT_ARGS_H_
#define SOURCE_CMD_BOOT_ARGS_H_

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File


extern Uint32*  gp_boot_arg_err_bits;                    /* flash�����bit���� */
extern Uint16*  gp_boot_arg_boot_region;                 /* �������� */
extern int64*   gp_boot_arg_last_pd_local_motor;         /* �ϴζϵ�λ�� */
extern float32* gp_boot_arg_angle_senser_adj;            /* �Ƕȴ�����У׼ֵ */
extern Uint16*  gp_boot_arg_serial_fault_reset;          /* �ϴ�����Ϊ�����쳣���µĸ�λ */
extern Uint16*  gp_boot_arg_system_reset_times;          /* ϵͳ��λ���� */

extern float32*  gp_boot_arg_angle_senser_adj_pya_1;     /* +YA��Port1�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_angle_senser_adj_pya_2;     /* +YA��Port2�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_angle_senser_adj_nya_1;     /* -YA��Port1�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_angle_senser_adj_nya_2;     /* -YA��Port2�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_angle_senser_adj_pyb_3;     /* +YB��Port3�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_angle_senser_adj_nyb_3;     /* -YB��Port3�Ƕȴ�����У׼ֵ */
extern float32*  gp_boot_arg_last_pd_local_hall;         /* �ϴζϵ�λ�ã������� */

//extern Uint16*   gp_boot_arg_soft_lim_flag;              /* �����λʹ�ñ���ֵ�ı�־��Ϊ0xABCDʱ��ʾʹ�ñ���������λֵ */
//extern int32*    gp_boot_arg_soft_lim_p;                 /* �����λ����ֵ */
//extern int32*    gp_boot_arg_soft_lim_n;                 /* �����λ����ֵ */


#endif /* SOURCE_CMD_BOOT_ARGS_H_ */
