/*
 * boot_args.h
 *
 *  Created on: 2024年3月17日
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_BOOT_ARGS_H_
#define SOURCE_CMD_BOOT_ARGS_H_

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File


extern Uint32*  gp_boot_arg_err_bits;                    /* flash出错的bit数量 */
extern Uint16*  gp_boot_arg_boot_region;                 /* 启动区域 */
extern int64*   gp_boot_arg_last_pd_local_motor;         /* 上次断电位置 */
extern float32* gp_boot_arg_angle_senser_adj;            /* 角度传感器校准值 */
extern Uint16*  gp_boot_arg_serial_fault_reset;          /* 上次是因为串口异常导致的复位 */
extern Uint16*  gp_boot_arg_system_reset_times;          /* 系统复位次数 */

extern float32*  gp_boot_arg_angle_senser_adj_pya_1;     /* +YA轴Port1角度传感器校准值 */
extern float32*  gp_boot_arg_angle_senser_adj_pya_2;     /* +YA轴Port2角度传感器校准值 */
extern float32*  gp_boot_arg_angle_senser_adj_nya_1;     /* -YA轴Port1角度传感器校准值 */
extern float32*  gp_boot_arg_angle_senser_adj_nya_2;     /* -YA轴Port2角度传感器校准值 */
extern float32*  gp_boot_arg_angle_senser_adj_pyb_3;     /* +YB轴Port3角度传感器校准值 */
extern float32*  gp_boot_arg_angle_senser_adj_nyb_3;     /* -YB轴Port3角度传感器校准值 */
extern float32*  gp_boot_arg_last_pd_local_hall;         /* 上次断电位置（霍尔） */

//extern Uint16*   gp_boot_arg_soft_lim_flag;              /* 软件限位使用保存值的标志，为0xABCD时表示使用保存的软件限位值 */
//extern int32*    gp_boot_arg_soft_lim_p;                 /* 软件限位配置值 */
//extern int32*    gp_boot_arg_soft_lim_n;                 /* 软件限位配置值 */


#endif /* SOURCE_CMD_BOOT_ARGS_H_ */
