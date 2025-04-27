/*
 * cmd.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "cmd.h"
#include <string.h>
#include "project.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "mot_cur_conf_cmd.h"

/** <\brief 定义4个轴的配置，当接收到配置指令后将数据存放在这个变量中 */
axis_conf_t g_Axis_Conf;
#pragma DATA_SECTION(g_Axis_Conf, "pre_roll_data");
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4)
single_axis_conf_t g_pya;
single_axis_conf_t g_nya;
single_axis_conf_t g_pyb;
single_axis_conf_t g_nyb;
#elif (BOARD_NAME == BOARD_DGM_2)
single_axis_conf_t g_pya;
single_axis_conf_t g_nya;
#elif (BOARD_NAME == BOARD_GMS)
single_axis_conf_t g_pyb;
#pragma DATA_SECTION(g_pyb, "pre_roll_data");
#endif


/* 用于判断串口是否超时未接收到数据 */
Uint64 g_RS422_DATA_Tick = 0;
#pragma DATA_SECTION(g_RS422_DATA_Tick, "pre_roll_data");

/* 串口重新连接的次数 */
Uint16 g_RS422_relink_times = 0;
#pragma DATA_SECTION(g_RS422_relink_times, "pre_roll_data");

/**
 * \brief 初始化轴的配置数据
 */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4)
/**
 * \brief 设置PYA轴A相方向
 */
static void __write_PYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}
/**
 * \brief 设置PYA轴B相方向
 */
static void __write_PYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA7 = val;
}
/**
 * \brief 设置PYA轴刹车信号，写1刹车
 */
static void __write_PYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB1 = val;
}

/**
 * \brief 设置NYA轴A相方向
 */
static void __write_NYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA10 = val;
}
/**
 * \brief 设置NYA轴B相方向
 */
static void __write_NYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA11 = val;
}
/**
 * \brief 设置NYA轴刹车信号，写1刹车
 */
static void __write_NYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB5 = val;
}

/**
 * \brief 设置PYB轴A相方向
 */
static void __write_PYB_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA8 = val;
}
/**
 * \brief 设置PYB轴B相方向
 */
static void __write_PYB_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA9 = val;
}
/**
 * \brief 设置PYB轴刹车信号
 */
static void __write_PYB_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB3 = val;
}

/**
 * \brief 设置NYB轴A相方向
 */
static void __write_NYB_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA12 = val;
}
/**
 * \brief 设置PYB轴B相方向
 */
static void __write_NYB_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA13 = val;
}
/**
 * \brief 设置PYB轴刹车信号
 */
static void __write_NYB_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB7 = val;
}

void Init_Axis_Conf(void)
{
    memset(&g_Axis_Conf, 0, sizeof(g_Axis_Conf));
    memset(&g_pya, 0, sizeof(single_axis_conf_t));
    memset(&g_nya, 0, sizeof(single_axis_conf_t));
    memset(&g_pyb, 0, sizeof(single_axis_conf_t));
    memset(&g_nyb, 0, sizeof(single_axis_conf_t));

    g_pya.p_next_axis = &g_nya;
    g_nya.p_next_axis = &g_pyb;
    g_pyb.p_next_axis = &g_nyb;
    g_nyb.p_next_axis = 0;

    g_Axis_Conf.p_pya = &g_pya;
    g_Axis_Conf.p_nya = &g_nya;
    g_Axis_Conf.p_pyb = &g_pyb;
    g_Axis_Conf.p_nyb = &g_nyb;
    strcpy(g_Axis_Conf.p_pya->name, "PYA");
    strcpy(g_Axis_Conf.p_nya->name, "NYA");
    strcpy(g_Axis_Conf.p_pyb->name, "PYB");
    strcpy(g_Axis_Conf.p_nyb->name, "NYB");

    /* 每个轴都没有任务 */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pya->is_task_running = false;
    g_Axis_Conf.p_nya->is_task_running = false;
    g_Axis_Conf.p_pyb->is_task_running = false;
    g_Axis_Conf.p_nyb->is_task_running = false;

    /* DGM-4没有待机模式 */
    g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_nya->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_nyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pya->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_nya->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_nyb->work_mode_old = WORK_MODE_KEEP;

    /* 驱动线路都是1，55主份，AA备份 */
#if 0   /* 18200没有驱动热备 */
    g_Axis_Conf.p_pya->drv_conf = 0x55;
    g_Axis_Conf.p_nya->drv_conf = 0x55;
    g_Axis_Conf.p_pyb->drv_conf = 0x55;
    g_Axis_Conf.p_nyb->drv_conf = 0x55;
#endif

    /* 最大运动电流默认值 */
    g_Axis_Conf.p_pya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_nya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_pyb->max_mov_cur_given  = 40;
    g_Axis_Conf.p_nyb->max_mov_cur_given  = 40;

    /* 最大保持电流默认值 */
    g_Axis_Conf.p_pya->keep_cur_given     = 30;
    g_Axis_Conf.p_nya->keep_cur_given     = 30;
    g_Axis_Conf.p_pyb->keep_cur_given     = 20;
    g_Axis_Conf.p_nyb->keep_cur_given     = 20;

    /* 默认复位速度给定为 0.6°/s */
    g_Axis_Conf.p_pya->reset_speed_given = 1707;
    g_Axis_Conf.p_nya->reset_speed_given = 1707;
    g_Axis_Conf.p_pyb->reset_speed_given = 667;
    g_Axis_Conf.p_nyb->reset_speed_given = 667;

    /* 最大加速度配置，默认为1，等于0.005°/s2 */
    g_Axis_Conf.p_pya->max_acc = 1;
    g_Axis_Conf.p_nya->max_acc = 1;
    g_Axis_Conf.p_pyb->max_acc = 1;
    g_Axis_Conf.p_nyb->max_acc = 1;

#if 0
    /* 电机过流保护开关，默认为关 */
    g_Axis_Conf.p_pya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_pyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* 默认过流保护阈值，最大值 */
    g_Axis_Conf.p_pya->overcur_prot_th = 20;
    g_Axis_Conf.p_nya->overcur_prot_th = 20;
    g_Axis_Conf.p_pyb->overcur_prot_th = 75;
    g_Axis_Conf.p_nyb->overcur_prot_th = 75;
#endif

    /* 当前位置，上电后要通过指令进行配置，默认为0 */
    /* 当前位置与角度的对应关系：
     * #0：0°
     * #128000:45°
     * #256000:90°
     * #384000:135°
     * #512000:180°
     * #640000:225°
     * #768000:270°
     * #896000:315°
     * #1024000:360°
     */
    g_Axis_Conf.p_pya->cur_local = 0;
    g_Axis_Conf.p_nya->cur_local = 0;
//    g_Axis_Conf.p_pyb->cur_local = 0;
//    g_Axis_Conf.p_nyb->cur_local = 0;
    g_Axis_Conf.p_pyb->cur_local = 200000;
    g_Axis_Conf.p_nyb->cur_local = 200000;  /* B轴初始位置应该在180° */

    g_Axis_Conf.p_pyb->soft_lim = 50000;
    g_Axis_Conf.p_nyb->soft_lim = 50000;    /* 只有B轴有软件限位，默认是45° */

    /* 复位位置，默认是0，实际安装后会有偏差，到时候改值会是一个常数 */
    g_Axis_Conf.p_pya->reset_local = 0;
    g_Axis_Conf.p_nya->reset_local = 0;
    g_Axis_Conf.p_pyb->reset_local = 0;
    g_Axis_Conf.p_nyb->reset_local = 0;

    /* 转速比，A轴是160，B轴是125 */
    g_Axis_Conf.p_pya->speed_ratio = 160;
    g_Axis_Conf.p_nya->speed_ratio = 160;
    g_Axis_Conf.p_pyb->speed_ratio = 125;
    g_Axis_Conf.p_nyb->speed_ratio = 125;

    /* 步距角，A轴是0.9，B轴是1.8 */
    g_Axis_Conf.p_pya->step_angle = 0.9;
    g_Axis_Conf.p_nya->step_angle = 0.9;
    g_Axis_Conf.p_pyb->step_angle = 1.8;
    g_Axis_Conf.p_nyb->step_angle = 1.8;

    /* IQ格式，方便计算 */
    g_Axis_Conf.p_pya->step_angle_iq =_IQ30(g_Axis_Conf.p_pya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pya->speed_ratio));
    g_Axis_Conf.p_nya->step_angle_iq =_IQ30(g_Axis_Conf.p_nya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nya->speed_ratio));
    g_Axis_Conf.p_pyb->step_angle_iq =_IQ30(g_Axis_Conf.p_pyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pyb->speed_ratio));
    g_Axis_Conf.p_nyb->step_angle_iq =_IQ30(g_Axis_Conf.p_nyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nyb->speed_ratio));

    /* 输出轴步距角 */
    g_Axis_Conf.p_pya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A轴输出轴步距角 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_nya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A轴输出轴步距角 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_pyb->step_angle_out_iq23 = _IQ23(0.00045);          /* B轴输出轴步距角 1.8 / (32 * 125) = 0.00045 */
    g_Axis_Conf.p_nyb->step_angle_out_iq23 = _IQ23(0.00045);          /* B轴输出轴步距角 1.8 / (32 * 125) = 0.00045 */

    /* 从位置值转换到角度值需要乘上的系数 */
    g_Axis_Conf.p_pya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_nya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_pyb->loc_to_angle_factor = (float64)9 / 10000;
    g_Axis_Conf.p_nyb->loc_to_angle_factor = (float64)9 / 10000;

    /* 从速度值到速度需要乘上的系数 */
    g_Axis_Conf.p_pya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_nya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_pyb->speedvalue_to_speed = (float64)9 / 10000;
    g_Axis_Conf.p_nyb->speedvalue_to_speed = (float64)9 / 10000;

    /* 当前角速度默认都为0 */
    g_Axis_Conf.p_pya->cur_angle_speed = 0;
    g_Axis_Conf.p_nya->cur_angle_speed = 0;
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;
    g_Axis_Conf.p_nyb->cur_angle_speed = 0;


    /* A轴位置：0~12288000，对应角度：0~360° B轴位置：-800000~3200000，对应角度-45°~180° */
    g_Axis_Conf.p_pya->min_loc = 0;
    g_Axis_Conf.p_nya->min_loc = 0;
    g_Axis_Conf.p_pyb->min_loc = -50000;
    g_Axis_Conf.p_nyb->min_loc = -50000;

    g_Axis_Conf.p_pya->max_loc = 1024000;
    g_Axis_Conf.p_nya->max_loc = 1024000;
    g_Axis_Conf.p_pyb->max_loc = 200000;
    g_Axis_Conf.p_nyb->max_loc = 200000;

#if 0
    g_Axis_Conf.p_pya->min_angle = 0;
    g_Axis_Conf.p_nya->min_angle = 0;
    g_Axis_Conf.p_pyb->min_angle = -45;
    g_Axis_Conf.p_nyb->min_angle = -45;

    g_Axis_Conf.p_pya->max_angle = 360;
    g_Axis_Conf.p_nya->max_angle = 360;
    g_Axis_Conf.p_pyb->max_angle = 180;
    g_Axis_Conf.p_nyb->max_angle = 180;
#endif

    g_Axis_Conf.p_pya->min_speed = -1707;
    g_Axis_Conf.p_nya->min_speed = -1707;
    g_Axis_Conf.p_pyb->min_speed = 56;
    g_Axis_Conf.p_nyb->min_speed = 56;

    g_Axis_Conf.p_pya->max_speed = 1707;
    g_Axis_Conf.p_nya->max_speed = 1707;
    g_Axis_Conf.p_pyb->max_speed = 667;
    g_Axis_Conf.p_nyb->max_speed = 667;


    /* 初始化函数指针 */
    /* +Y-A轴 */
    g_Axis_Conf.p_pya->fn_write_a_dir    = __write_PYA_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_pya->fn_write_b_dir    = __write_PYA_dir_B;       /* 设置A相方向 */
    g_Axis_Conf.p_pya->fn_write_brake    = __write_PYA_brake;       /* 设置A相方向 */

    /* -Y-A轴 */
    g_Axis_Conf.p_nya->fn_write_a_dir    = __write_NYA_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_nya->fn_write_b_dir    = __write_NYA_dir_B;       /* 设置A相方向 */
    g_Axis_Conf.p_nya->fn_write_brake    = __write_NYA_brake;       /* 设置A相方向 */

    /* +Y-B轴 */
    g_Axis_Conf.p_pyb->fn_write_a_dir    = __write_PYB_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_pyb->fn_write_b_dir    = __write_PYB_dir_B;       /* 设置A相方向 */
    g_Axis_Conf.p_pyb->fn_write_brake    = __write_PYB_brake;       /* 设置A相方向 */

    /* -Y-B轴 */
    g_Axis_Conf.p_nyb->fn_write_a_dir    = __write_NYB_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_nyb->fn_write_b_dir    = __write_NYB_dir_B;       /* 设置A相方向 */
    g_Axis_Conf.p_nyb->fn_write_brake    = __write_NYB_brake;       /* 设置A相方向 */
    /**
     * \brief 占空比系数与电流的对应关系
     */
    set_mot_cur();
}
#elif (BOARD_NAME == BOARD_DGM_2)
/**
 * \brief 设置PYA轴A相方向
 */
static void __write_PYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}
/**
 * \brief 设置PYA轴B相方向
 */
static void __write_PYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA7 = val;
}
/**
 * \brief 设置PYA轴刹车信号，写1刹车
 */
static void __write_PYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB1 = val;
}

/**
 * \brief 设置NYA轴A相方向
 */
static void __write_NYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA10 = val;
}
/**
 * \brief 设置NYA轴B相方向
 */
static void __write_NYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA11 = val;
}
/**
 * \brief 设置NYA轴刹车信号，写1刹车
 */
static void __write_NYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB5 = val;
}

void Init_Axis_Conf(void)
{
//  single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
//  uint8_t i = 0;

    memset(&g_Axis_Conf, 0, sizeof(g_Axis_Conf));
    memset(&g_pya, 0, sizeof(single_axis_conf_t));
    memset(&g_nya, 0, sizeof(single_axis_conf_t));

    g_pya.p_next_axis = &g_nya;
    g_nya.p_next_axis = 0;

    g_Axis_Conf.p_pya = &g_pya;
    g_Axis_Conf.p_nya = &g_nya;

    strcpy(g_Axis_Conf.p_pya->name, "PYA");
    strcpy(g_Axis_Conf.p_nya->name, "NYA");

    /* 每个轴都没有任务 */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pya->is_task_running = false;
    g_Axis_Conf.p_nya->is_task_running = false;

    /* 默认工作模式都是待机模式 */
    g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_STANDBY;
    g_Axis_Conf.p_nya->work_mode_set = WORK_MODE_STANDBY;
    g_Axis_Conf.p_pya->work_mode_old = WORK_MODE_STANDBY;
    g_Axis_Conf.p_nya->work_mode_old = WORK_MODE_STANDBY;
#if 0
    /* 驱动线路都是1，55主份，AA备份 */
    g_Axis_Conf.p_pya->drv_conf = 0x55;
    g_Axis_Conf.p_nya->drv_conf = 0x55;
#endif

    /* 最大运动电流默认值 */
    g_Axis_Conf.p_pya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_nya->max_mov_cur_given  = 30;

    /* 最大保持电流默认值 */
    g_Axis_Conf.p_pya->keep_cur_given     = 30;
    g_Axis_Conf.p_nya->keep_cur_given     = 30;

    /* 默认复位速度给定为 0.6°/s */
    g_Axis_Conf.p_pya->reset_speed_given = 1707;
    g_Axis_Conf.p_nya->reset_speed_given = 1707;

    /* 最大加速度配置，默认为1，等于0.005°/s2 */
    g_Axis_Conf.p_pya->max_acc = 12;
    g_Axis_Conf.p_nya->max_acc = 12;

#if 0
    /* 电机过流保护开关，默认为关 */
    g_Axis_Conf.p_pya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* 默认过流保护阈值，最大值 */
    g_Axis_Conf.p_pya->overcur_prot_th = 20;
    g_Axis_Conf.p_nya->overcur_prot_th = 20;
#endif

    /* 当前位置，上电后要通过指令进行配置，默认为0 */
    /* 当前位置与角度的对应关系：
     * #0：0°
     * #128000:45°
     * #256000:90°
     * #384000:135°
     * #512000:180°
     * #640000:225°
     * #768000:270°
     * #896000:315°
     * #1024000:360°
     */
    g_Axis_Conf.p_pya->cur_local = 0;
    g_Axis_Conf.p_nya->cur_local = 0;

    /* 复位位置，默认是0，实际安装后会有偏差，到时候改值会是一个常数 */
    g_Axis_Conf.p_pya->reset_local = 0;
    g_Axis_Conf.p_nya->reset_local = 0;

    /* 转速比，A轴是160，B轴是125 */
    g_Axis_Conf.p_pya->speed_ratio = 160;
    g_Axis_Conf.p_nya->speed_ratio = 160;

    /* 步距角，A轴是0.9，B轴是1.8 */
    g_Axis_Conf.p_pya->step_angle = 0.9;
    g_Axis_Conf.p_nya->step_angle = 0.9;

    /* IQ格式，方便计算 */
    g_Axis_Conf.p_pya->step_angle_iq =_IQ30(g_Axis_Conf.p_pya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pya->speed_ratio));
    g_Axis_Conf.p_nya->step_angle_iq =_IQ30(g_Axis_Conf.p_nya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nya->speed_ratio));

    /* 输出轴步距角 */
    g_Axis_Conf.p_pya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A轴输出轴步距角 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_nya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A轴输出轴步距角 0.9 / (32 * 160) = 0.00017578125 */

    /* 从位置值转换到角度值需要乘上的系数 */
    g_Axis_Conf.p_pya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_nya->loc_to_angle_factor = (float64)9 / 25600;

    /* 从速度值到速度需要乘上的系数 */
    g_Axis_Conf.p_pya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_nya->speedvalue_to_speed = (float64)9 / 25600;

    /* 当前角速度默认都为0 */
    g_Axis_Conf.p_pya->cur_angle_speed = 0;
    g_Axis_Conf.p_nya->cur_angle_speed = 0;

    /* A轴位置：0~12288000，对应角度：0~360° B轴位置：-800000~3200000，对应角度-45°~180° */
    g_Axis_Conf.p_pya->min_loc = 0;
    g_Axis_Conf.p_nya->min_loc = 0;

    g_Axis_Conf.p_pya->max_loc = 1024000;
    g_Axis_Conf.p_nya->max_loc = 1024000;

#if 0
    g_Axis_Conf.p_pya->min_angle = 0;
    g_Axis_Conf.p_nya->min_angle = 0;

    g_Axis_Conf.p_pya->max_angle = 360;
    g_Axis_Conf.p_nya->max_angle = 360;
#endif

    g_Axis_Conf.p_pya->min_speed = -1707;
    g_Axis_Conf.p_nya->min_speed = -1707;

    g_Axis_Conf.p_pya->max_speed = 1707;
    g_Axis_Conf.p_nya->max_speed = 1707;

    /* 初始化函数指针 */
    /* +Y-A轴 */
    g_Axis_Conf.p_pya->fn_write_a_dir    = __write_PYA_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_pya->fn_write_b_dir    = __write_PYA_dir_B;       /* 设置B相方向 */
    g_Axis_Conf.p_pya->fn_write_brake    = __write_PYA_brake;       /* 设置刹车信号 */

    /* -Y-A轴 */
    g_Axis_Conf.p_nya->fn_write_a_dir    = __write_NYA_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_nya->fn_write_b_dir    = __write_NYA_dir_B;       /* 设置B相方向 */
    g_Axis_Conf.p_nya->fn_write_brake    = __write_NYA_brake;       /* 设置刹车信号 */
    /**
     * \brief 占空比系数与电流的对应关系
     */
    set_mot_cur();
}
#elif BOARD_NAME == BOARD_GMS
/**
 * \brief 设置A相方向
 */
static void __write_PYB_dir_A (uint8_t val)
{
	GpioDataRegs.GPADAT.bit.GPIOA3 = val;
}

/**
 * \brief 设置B相方向
 */
static void __write_PYB_dir_B (uint8_t val)
{
	GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}

/**
 * \brief 设置刹车信号
 */
static void __write_PYB_brake (uint8_t val)
{
	GpioDataRegs.GPADAT.bit.GPIOA4 = val;
	GpioDataRegs.GPADAT.bit.GPIOA6 = val;
}

void Init_Axis_Conf(void)
{
//  single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
//  uint8_t i = 0;

    memset(&g_Axis_Conf, 0, sizeof(g_Axis_Conf));
    memset(&g_pyb, 0, sizeof(single_axis_conf_t));
    g_Axis_Conf.p_pyb = &g_pyb;

    strcpy(g_Axis_Conf.p_pyb->name, "PYB");

    /* 每个轴都没有任务 */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pyb->is_task_running = false;

    /* 默认工作模式都是保持模式 */
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;

    /* 往复模式禁能 */
    g_Axis_Conf.p_pyb->reciprocat_mode_enable = 0;

    /* 最大运动电流默认值 */
    g_Axis_Conf.p_pyb->max_mov_cur_given  = 40;

    /* 最大保持电流默认值 */
    g_Axis_Conf.p_pyb->keep_cur_given     = 30;

    /* 默认复位速度给定为 0.6°/s */
    g_Axis_Conf.p_pyb->reset_speed_given = 1334;

    /* 最大加速度配置，默认为2，1等于0.005°/s2 */
    g_Axis_Conf.p_pyb->max_acc = 2;

#if 0
    /* 电机过流保护开关，默认为关 */
    g_Axis_Conf.p_pyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* 默认过流保护阈值，最大值 */
    g_Axis_Conf.p_pyb->overcur_prot_th = 65;
#endif

    /* 当前位置，上电后要通过指令进行配置，默认为0 */
    g_Axis_Conf.p_pyb->cur_local = 0;

    g_Axis_Conf.p_pyb->hall_angle = 0;

    //175°会撞限位，默认限位值改到174°
//    g_Axis_Conf.p_pyb->soft_lim_p =  388889;    /* 默认是175° */
//    g_Axis_Conf.p_pyb->soft_lim_n = -388889;    /* 默认是-175° */
    g_Axis_Conf.p_pyb->soft_lim_p =  386666;    /* 默认是174° */
    g_Axis_Conf.p_pyb->soft_lim_n = -386666;    /* 默认是-174° */


    /* 复位位置，默认是0，实际安装后会有偏差，到时候改值会是一个常数 */
    g_Axis_Conf.p_pyb->reset_local = 0;

    /* 转速比，A轴是160，B轴是125 */
    g_Axis_Conf.p_pyb->speed_ratio = 125;

    /* 步距角 */
    g_Axis_Conf.p_pyb->step_angle = 0.9;

    /* IQ格式，方便计算 */
    g_Axis_Conf.p_pyb->step_angle_iq =_IQ30(g_Axis_Conf.p_pyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pyb->speed_ratio));

    /* 输出轴步距角 */
    g_Axis_Conf.p_pyb->step_angle_out_iq23 = _IQ23(0.000225);          /* B轴输出轴步距角0.9 / (32 * 125) = 0.000225 */


    /* 从位置值转换到角度值需要乘上的系数 */
    g_Axis_Conf.p_pyb->loc_to_angle_factor = (float64)9 / 20000;

    /* 从速度值到速度需要乘上的系数 */
    g_Axis_Conf.p_pyb->speedvalue_to_speed = (float64)9 / 20000;

    /* 当前角速度默认都为0 */
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;

#if 0
    /* B轴位置：-388889~388889，对应角度-175.3°~175.3° */
    g_Axis_Conf.p_pyb->min_loc = -389555;
    g_Axis_Conf.p_pyb->max_loc = 389555;


    /* B轴角度：-45°~+180° */
    g_Axis_Conf.p_pyb->min_angle = -175;
    g_Axis_Conf.p_pyb->max_angle = 175;
#endif

    /* 最小速度 */
    g_Axis_Conf.p_pyb->min_speed = -1334;       /* -0.6°/s */

    /* 最大速度 */
    g_Axis_Conf.p_pyb->max_speed = 1334;    /* 0.6°/s */

    /* +Y-B轴函数指针 */
    g_Axis_Conf.p_pyb->fn_write_a_dir    = __write_PYB_dir_A;       /* 设置A相方向 */
    g_Axis_Conf.p_pyb->fn_write_b_dir    = __write_PYB_dir_B;       /* 设置A相方向 */
    g_Axis_Conf.p_pyb->fn_write_brake    = __write_PYB_brake;       /* 设置A相方向 */

    /**
     * \brief 占空比系数与电流的对应关系
     */
    set_mot_cur();
}
#endif
/**
 * \brief 计算校验和
 *
 * \param[in] pData：要计算的数据
 * \param[in] uiLen：要计算的数据长度
 *
 * \retval 返回校验和
 */
Uint16 CheckSumCalc (const Uint16 *pData, Uint16 uiLen)
{
    Uint16 i, CheckCode = 0;
    for (i = 0; i < uiLen; i++) {
    	CheckCode += ((*(pData+i)) & 0xFF);
    }
    return CheckCode;
}
