/*
 * boot_args.c
 *
 *  Created on: 2024��3��17��
 *      Author: Suzkfly
 */
#include "boot_args.h"

Uint32*   gp_boot_arg_err_bits            		= (void*)0x17A800;          /* flash�����bit���� */
Uint16*   gp_boot_arg_boot_region         		= (void*)0x17A802;          /* �������� */
int64*    gp_boot_arg_last_pd_local_motor       = (void*)0x17A804;          /* �ϴζϵ�λ�ã������ */
float32*  gp_boot_arg_angle_senser_adj    		= (void*)0x17A808;          /* �Ƕȴ�����У׼ֵ */
Uint16*   gp_boot_arg_serial_fault_reset  		= (void*)0x17A80A;          /* �ϴ�����Ϊ�����쳣���µĸ�λ */
Uint16*   gp_boot_arg_system_reset_times  		= (void*)0x17A80B;          /* ϵͳ��λ���� */
float32*  gp_boot_arg_angle_senser_adj_pya_1    = (void*)0x17A80C;          /* +YA��Port1�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_angle_senser_adj_pya_2    = (void*)0x17A80E;          /* +YA��Port2�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_angle_senser_adj_nya_1   	= (void*)0x17A810;          /* -YA��Port1�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_angle_senser_adj_nya_2    = (void*)0x17A812;          /* -YA��Port2�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_angle_senser_adj_pyb_3    = (void*)0x17A814;          /* +YB��Port3�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_angle_senser_adj_nyb_3    = (void*)0x17A816;          /* -YB��Port3�Ƕȴ�����У׼ֵ */
float32*  gp_boot_arg_last_pd_local_hall        = (void*)0x17A818;          /* �ϴζϵ�λ�ã������� */

//Uint16*   gp_boot_arg_soft_lim_flag             = (void*)0x17A818;          /* �����λʹ�ñ���ֵ�ı�־��Ϊ0xABCDʱ��ʾʹ�ñ���������λֵ */
//int32*    gp_boot_arg_soft_lim_p                = (void*)0x17A81A;          /* �����λ����ֵ */
//int32*    gp_boot_arg_soft_lim_n                = (void*)0x17A81C;          /* �����λ����ֵ */




