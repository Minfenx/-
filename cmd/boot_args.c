/*
 * boot_args.c
 *
 *  Created on: 2024年3月17日
 *      Author: Suzkfly
 */
#include "boot_args.h"

Uint32*   gp_boot_arg_err_bits            		= (void*)0x17A800;          /* flash出错的bit数量 */
Uint16*   gp_boot_arg_boot_region         		= (void*)0x17A802;          /* 启动区域 */
int64*    gp_boot_arg_last_pd_local_motor       = (void*)0x17A804;          /* 上次断电位置（电机） */
float32*  gp_boot_arg_angle_senser_adj    		= (void*)0x17A808;          /* 角度传感器校准值 */
Uint16*   gp_boot_arg_serial_fault_reset  		= (void*)0x17A80A;          /* 上次是因为串口异常导致的复位 */
Uint16*   gp_boot_arg_system_reset_times  		= (void*)0x17A80B;          /* 系统复位次数 */
float32*  gp_boot_arg_angle_senser_adj_pya_1    = (void*)0x17A80C;          /* +YA轴Port1角度传感器校准值 */
float32*  gp_boot_arg_angle_senser_adj_pya_2    = (void*)0x17A80E;          /* +YA轴Port2角度传感器校准值 */
float32*  gp_boot_arg_angle_senser_adj_nya_1   	= (void*)0x17A810;          /* -YA轴Port1角度传感器校准值 */
float32*  gp_boot_arg_angle_senser_adj_nya_2    = (void*)0x17A812;          /* -YA轴Port2角度传感器校准值 */
float32*  gp_boot_arg_angle_senser_adj_pyb_3    = (void*)0x17A814;          /* +YB轴Port3角度传感器校准值 */
float32*  gp_boot_arg_angle_senser_adj_nyb_3    = (void*)0x17A816;          /* -YB轴Port3角度传感器校准值 */
float32*  gp_boot_arg_last_pd_local_hall        = (void*)0x17A818;          /* 上次断电位置（霍尔） */

//Uint16*   gp_boot_arg_soft_lim_flag             = (void*)0x17A818;          /* 软件限位使用保存值的标志，为0xABCD时表示使用保存的软件限位值 */
//int32*    gp_boot_arg_soft_lim_p                = (void*)0x17A81A;          /* 软件限位配置值 */
//int32*    gp_boot_arg_soft_lim_n                = (void*)0x17A81C;          /* 软件限位配置值 */




