/*
 * cmd.c
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */

#include "cmd.h"
#include <string.h>
#include "project.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "mot_cur_conf_cmd.h"

/** <\brief ����4��������ã������յ�����ָ������ݴ������������� */
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


/* �����жϴ����Ƿ�ʱδ���յ����� */
Uint64 g_RS422_DATA_Tick = 0;
#pragma DATA_SECTION(g_RS422_DATA_Tick, "pre_roll_data");

/* �����������ӵĴ��� */
Uint16 g_RS422_relink_times = 0;
#pragma DATA_SECTION(g_RS422_relink_times, "pre_roll_data");

/**
 * \brief ��ʼ�������������
 */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_4)
/**
 * \brief ����PYA��A�෽��
 */
static void __write_PYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}
/**
 * \brief ����PYA��B�෽��
 */
static void __write_PYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA7 = val;
}
/**
 * \brief ����PYA��ɲ���źţ�д1ɲ��
 */
static void __write_PYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB1 = val;
}

/**
 * \brief ����NYA��A�෽��
 */
static void __write_NYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA10 = val;
}
/**
 * \brief ����NYA��B�෽��
 */
static void __write_NYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA11 = val;
}
/**
 * \brief ����NYA��ɲ���źţ�д1ɲ��
 */
static void __write_NYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB5 = val;
}

/**
 * \brief ����PYB��A�෽��
 */
static void __write_PYB_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA8 = val;
}
/**
 * \brief ����PYB��B�෽��
 */
static void __write_PYB_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA9 = val;
}
/**
 * \brief ����PYB��ɲ���ź�
 */
static void __write_PYB_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB3 = val;
}

/**
 * \brief ����NYB��A�෽��
 */
static void __write_NYB_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA12 = val;
}
/**
 * \brief ����PYB��B�෽��
 */
static void __write_NYB_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA13 = val;
}
/**
 * \brief ����PYB��ɲ���ź�
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

    /* ÿ���ᶼû������ */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pya->is_task_running = false;
    g_Axis_Conf.p_nya->is_task_running = false;
    g_Axis_Conf.p_pyb->is_task_running = false;
    g_Axis_Conf.p_nyb->is_task_running = false;

    /* DGM-4û�д���ģʽ */
    g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_nya->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_nyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pya->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_nya->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_nyb->work_mode_old = WORK_MODE_KEEP;

    /* ������·����1��55���ݣ�AA���� */
#if 0   /* 18200û�������ȱ� */
    g_Axis_Conf.p_pya->drv_conf = 0x55;
    g_Axis_Conf.p_nya->drv_conf = 0x55;
    g_Axis_Conf.p_pyb->drv_conf = 0x55;
    g_Axis_Conf.p_nyb->drv_conf = 0x55;
#endif

    /* ����˶�����Ĭ��ֵ */
    g_Axis_Conf.p_pya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_nya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_pyb->max_mov_cur_given  = 40;
    g_Axis_Conf.p_nyb->max_mov_cur_given  = 40;

    /* ��󱣳ֵ���Ĭ��ֵ */
    g_Axis_Conf.p_pya->keep_cur_given     = 30;
    g_Axis_Conf.p_nya->keep_cur_given     = 30;
    g_Axis_Conf.p_pyb->keep_cur_given     = 20;
    g_Axis_Conf.p_nyb->keep_cur_given     = 20;

    /* Ĭ�ϸ�λ�ٶȸ���Ϊ 0.6��/s */
    g_Axis_Conf.p_pya->reset_speed_given = 1707;
    g_Axis_Conf.p_nya->reset_speed_given = 1707;
    g_Axis_Conf.p_pyb->reset_speed_given = 667;
    g_Axis_Conf.p_nyb->reset_speed_given = 667;

    /* �����ٶ����ã�Ĭ��Ϊ1������0.005��/s2 */
    g_Axis_Conf.p_pya->max_acc = 1;
    g_Axis_Conf.p_nya->max_acc = 1;
    g_Axis_Conf.p_pyb->max_acc = 1;
    g_Axis_Conf.p_nyb->max_acc = 1;

#if 0
    /* ��������������أ�Ĭ��Ϊ�� */
    g_Axis_Conf.p_pya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_pyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* Ĭ�Ϲ���������ֵ�����ֵ */
    g_Axis_Conf.p_pya->overcur_prot_th = 20;
    g_Axis_Conf.p_nya->overcur_prot_th = 20;
    g_Axis_Conf.p_pyb->overcur_prot_th = 75;
    g_Axis_Conf.p_nyb->overcur_prot_th = 75;
#endif

    /* ��ǰλ�ã��ϵ��Ҫͨ��ָ��������ã�Ĭ��Ϊ0 */
    /* ��ǰλ����ǶȵĶ�Ӧ��ϵ��
     * #0��0��
     * #128000:45��
     * #256000:90��
     * #384000:135��
     * #512000:180��
     * #640000:225��
     * #768000:270��
     * #896000:315��
     * #1024000:360��
     */
    g_Axis_Conf.p_pya->cur_local = 0;
    g_Axis_Conf.p_nya->cur_local = 0;
//    g_Axis_Conf.p_pyb->cur_local = 0;
//    g_Axis_Conf.p_nyb->cur_local = 0;
    g_Axis_Conf.p_pyb->cur_local = 200000;
    g_Axis_Conf.p_nyb->cur_local = 200000;  /* B���ʼλ��Ӧ����180�� */

    g_Axis_Conf.p_pyb->soft_lim = 50000;
    g_Axis_Conf.p_nyb->soft_lim = 50000;    /* ֻ��B���������λ��Ĭ����45�� */

    /* ��λλ�ã�Ĭ����0��ʵ�ʰ�װ�����ƫ���ʱ���ֵ����һ������ */
    g_Axis_Conf.p_pya->reset_local = 0;
    g_Axis_Conf.p_nya->reset_local = 0;
    g_Axis_Conf.p_pyb->reset_local = 0;
    g_Axis_Conf.p_nyb->reset_local = 0;

    /* ת�ٱȣ�A����160��B����125 */
    g_Axis_Conf.p_pya->speed_ratio = 160;
    g_Axis_Conf.p_nya->speed_ratio = 160;
    g_Axis_Conf.p_pyb->speed_ratio = 125;
    g_Axis_Conf.p_nyb->speed_ratio = 125;

    /* ����ǣ�A����0.9��B����1.8 */
    g_Axis_Conf.p_pya->step_angle = 0.9;
    g_Axis_Conf.p_nya->step_angle = 0.9;
    g_Axis_Conf.p_pyb->step_angle = 1.8;
    g_Axis_Conf.p_nyb->step_angle = 1.8;

    /* IQ��ʽ��������� */
    g_Axis_Conf.p_pya->step_angle_iq =_IQ30(g_Axis_Conf.p_pya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pya->speed_ratio));
    g_Axis_Conf.p_nya->step_angle_iq =_IQ30(g_Axis_Conf.p_nya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nya->speed_ratio));
    g_Axis_Conf.p_pyb->step_angle_iq =_IQ30(g_Axis_Conf.p_pyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pyb->speed_ratio));
    g_Axis_Conf.p_nyb->step_angle_iq =_IQ30(g_Axis_Conf.p_nyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nyb->speed_ratio));

    /* ����Ჽ��� */
    g_Axis_Conf.p_pya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A������Ჽ��� 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_nya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A������Ჽ��� 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_pyb->step_angle_out_iq23 = _IQ23(0.00045);          /* B������Ჽ��� 1.8 / (32 * 125) = 0.00045 */
    g_Axis_Conf.p_nyb->step_angle_out_iq23 = _IQ23(0.00045);          /* B������Ჽ��� 1.8 / (32 * 125) = 0.00045 */

    /* ��λ��ֵת�����Ƕ�ֵ��Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_nya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_pyb->loc_to_angle_factor = (float64)9 / 10000;
    g_Axis_Conf.p_nyb->loc_to_angle_factor = (float64)9 / 10000;

    /* ���ٶ�ֵ���ٶ���Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_nya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_pyb->speedvalue_to_speed = (float64)9 / 10000;
    g_Axis_Conf.p_nyb->speedvalue_to_speed = (float64)9 / 10000;

    /* ��ǰ���ٶ�Ĭ�϶�Ϊ0 */
    g_Axis_Conf.p_pya->cur_angle_speed = 0;
    g_Axis_Conf.p_nya->cur_angle_speed = 0;
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;
    g_Axis_Conf.p_nyb->cur_angle_speed = 0;


    /* A��λ�ã�0~12288000����Ӧ�Ƕȣ�0~360�� B��λ�ã�-800000~3200000����Ӧ�Ƕ�-45��~180�� */
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


    /* ��ʼ������ָ�� */
    /* +Y-A�� */
    g_Axis_Conf.p_pya->fn_write_a_dir    = __write_PYA_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_pya->fn_write_b_dir    = __write_PYA_dir_B;       /* ����A�෽�� */
    g_Axis_Conf.p_pya->fn_write_brake    = __write_PYA_brake;       /* ����A�෽�� */

    /* -Y-A�� */
    g_Axis_Conf.p_nya->fn_write_a_dir    = __write_NYA_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_nya->fn_write_b_dir    = __write_NYA_dir_B;       /* ����A�෽�� */
    g_Axis_Conf.p_nya->fn_write_brake    = __write_NYA_brake;       /* ����A�෽�� */

    /* +Y-B�� */
    g_Axis_Conf.p_pyb->fn_write_a_dir    = __write_PYB_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_pyb->fn_write_b_dir    = __write_PYB_dir_B;       /* ����A�෽�� */
    g_Axis_Conf.p_pyb->fn_write_brake    = __write_PYB_brake;       /* ����A�෽�� */

    /* -Y-B�� */
    g_Axis_Conf.p_nyb->fn_write_a_dir    = __write_NYB_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_nyb->fn_write_b_dir    = __write_NYB_dir_B;       /* ����A�෽�� */
    g_Axis_Conf.p_nyb->fn_write_brake    = __write_NYB_brake;       /* ����A�෽�� */
    /**
     * \brief ռ�ձ�ϵ��������Ķ�Ӧ��ϵ
     */
    set_mot_cur();
}
#elif (BOARD_NAME == BOARD_DGM_2)
/**
 * \brief ����PYA��A�෽��
 */
static void __write_PYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}
/**
 * \brief ����PYA��B�෽��
 */
static void __write_PYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA7 = val;
}
/**
 * \brief ����PYA��ɲ���źţ�д1ɲ��
 */
static void __write_PYA_brake (uint8_t val)
{
    GpioDataRegs.GPBDAT.bit.GPIOB1 = val;
}

/**
 * \brief ����NYA��A�෽��
 */
static void __write_NYA_dir_A (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA10 = val;
}
/**
 * \brief ����NYA��B�෽��
 */
static void __write_NYA_dir_B (uint8_t val)
{
    GpioDataRegs.GPADAT.bit.GPIOA11 = val;
}
/**
 * \brief ����NYA��ɲ���źţ�д1ɲ��
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

    /* ÿ���ᶼû������ */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pya->is_task_running = false;
    g_Axis_Conf.p_nya->is_task_running = false;

    /* Ĭ�Ϲ���ģʽ���Ǵ���ģʽ */
    g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_STANDBY;
    g_Axis_Conf.p_nya->work_mode_set = WORK_MODE_STANDBY;
    g_Axis_Conf.p_pya->work_mode_old = WORK_MODE_STANDBY;
    g_Axis_Conf.p_nya->work_mode_old = WORK_MODE_STANDBY;
#if 0
    /* ������·����1��55���ݣ�AA���� */
    g_Axis_Conf.p_pya->drv_conf = 0x55;
    g_Axis_Conf.p_nya->drv_conf = 0x55;
#endif

    /* ����˶�����Ĭ��ֵ */
    g_Axis_Conf.p_pya->max_mov_cur_given  = 30;
    g_Axis_Conf.p_nya->max_mov_cur_given  = 30;

    /* ��󱣳ֵ���Ĭ��ֵ */
    g_Axis_Conf.p_pya->keep_cur_given     = 30;
    g_Axis_Conf.p_nya->keep_cur_given     = 30;

    /* Ĭ�ϸ�λ�ٶȸ���Ϊ 0.6��/s */
    g_Axis_Conf.p_pya->reset_speed_given = 1707;
    g_Axis_Conf.p_nya->reset_speed_given = 1707;

    /* �����ٶ����ã�Ĭ��Ϊ1������0.005��/s2 */
    g_Axis_Conf.p_pya->max_acc = 12;
    g_Axis_Conf.p_nya->max_acc = 12;

#if 0
    /* ��������������أ�Ĭ��Ϊ�� */
    g_Axis_Conf.p_pya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;
    g_Axis_Conf.p_nya->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* Ĭ�Ϲ���������ֵ�����ֵ */
    g_Axis_Conf.p_pya->overcur_prot_th = 20;
    g_Axis_Conf.p_nya->overcur_prot_th = 20;
#endif

    /* ��ǰλ�ã��ϵ��Ҫͨ��ָ��������ã�Ĭ��Ϊ0 */
    /* ��ǰλ����ǶȵĶ�Ӧ��ϵ��
     * #0��0��
     * #128000:45��
     * #256000:90��
     * #384000:135��
     * #512000:180��
     * #640000:225��
     * #768000:270��
     * #896000:315��
     * #1024000:360��
     */
    g_Axis_Conf.p_pya->cur_local = 0;
    g_Axis_Conf.p_nya->cur_local = 0;

    /* ��λλ�ã�Ĭ����0��ʵ�ʰ�װ�����ƫ���ʱ���ֵ����һ������ */
    g_Axis_Conf.p_pya->reset_local = 0;
    g_Axis_Conf.p_nya->reset_local = 0;

    /* ת�ٱȣ�A����160��B����125 */
    g_Axis_Conf.p_pya->speed_ratio = 160;
    g_Axis_Conf.p_nya->speed_ratio = 160;

    /* ����ǣ�A����0.9��B����1.8 */
    g_Axis_Conf.p_pya->step_angle = 0.9;
    g_Axis_Conf.p_nya->step_angle = 0.9;

    /* IQ��ʽ��������� */
    g_Axis_Conf.p_pya->step_angle_iq =_IQ30(g_Axis_Conf.p_pya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pya->speed_ratio));
    g_Axis_Conf.p_nya->step_angle_iq =_IQ30(g_Axis_Conf.p_nya->step_angle / (XIFEN_CNT * g_Axis_Conf.p_nya->speed_ratio));

    /* ����Ჽ��� */
    g_Axis_Conf.p_pya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A������Ჽ��� 0.9 / (32 * 160) = 0.00017578125 */
    g_Axis_Conf.p_nya->step_angle_out_iq23 = _IQ23(0.00017578125);    /* A������Ჽ��� 0.9 / (32 * 160) = 0.00017578125 */

    /* ��λ��ֵת�����Ƕ�ֵ��Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pya->loc_to_angle_factor = (float64)9 / 25600;
    g_Axis_Conf.p_nya->loc_to_angle_factor = (float64)9 / 25600;

    /* ���ٶ�ֵ���ٶ���Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pya->speedvalue_to_speed = (float64)9 / 25600;
    g_Axis_Conf.p_nya->speedvalue_to_speed = (float64)9 / 25600;

    /* ��ǰ���ٶ�Ĭ�϶�Ϊ0 */
    g_Axis_Conf.p_pya->cur_angle_speed = 0;
    g_Axis_Conf.p_nya->cur_angle_speed = 0;

    /* A��λ�ã�0~12288000����Ӧ�Ƕȣ�0~360�� B��λ�ã�-800000~3200000����Ӧ�Ƕ�-45��~180�� */
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

    /* ��ʼ������ָ�� */
    /* +Y-A�� */
    g_Axis_Conf.p_pya->fn_write_a_dir    = __write_PYA_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_pya->fn_write_b_dir    = __write_PYA_dir_B;       /* ����B�෽�� */
    g_Axis_Conf.p_pya->fn_write_brake    = __write_PYA_brake;       /* ����ɲ���ź� */

    /* -Y-A�� */
    g_Axis_Conf.p_nya->fn_write_a_dir    = __write_NYA_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_nya->fn_write_b_dir    = __write_NYA_dir_B;       /* ����B�෽�� */
    g_Axis_Conf.p_nya->fn_write_brake    = __write_NYA_brake;       /* ����ɲ���ź� */
    /**
     * \brief ռ�ձ�ϵ��������Ķ�Ӧ��ϵ
     */
    set_mot_cur();
}
#elif BOARD_NAME == BOARD_GMS
/**
 * \brief ����A�෽��
 */
static void __write_PYB_dir_A (uint8_t val)
{
	GpioDataRegs.GPADAT.bit.GPIOA3 = val;
}

/**
 * \brief ����B�෽��
 */
static void __write_PYB_dir_B (uint8_t val)
{
	GpioDataRegs.GPADAT.bit.GPIOA5 = val;
}

/**
 * \brief ����ɲ���ź�
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

    /* ÿ���ᶼû������ */
    g_Axis_Conf.task_cnt = 0;
    g_Axis_Conf.p_pyb->is_task_running = false;

    /* Ĭ�Ϲ���ģʽ���Ǳ���ģʽ */
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;

    /* ����ģʽ���� */
    g_Axis_Conf.p_pyb->reciprocat_mode_enable = 0;

    /* ����˶�����Ĭ��ֵ */
    g_Axis_Conf.p_pyb->max_mov_cur_given  = 40;

    /* ��󱣳ֵ���Ĭ��ֵ */
    g_Axis_Conf.p_pyb->keep_cur_given     = 30;

    /* Ĭ�ϸ�λ�ٶȸ���Ϊ 0.6��/s */
    g_Axis_Conf.p_pyb->reset_speed_given = 1334;

    /* �����ٶ����ã�Ĭ��Ϊ2��1����0.005��/s2 */
    g_Axis_Conf.p_pyb->max_acc = 2;

#if 0
    /* ��������������أ�Ĭ��Ϊ�� */
    g_Axis_Conf.p_pyb->overcur_prot_sw = OVER_CUR_PROT_SW_OFF;

    /* Ĭ�Ϲ���������ֵ�����ֵ */
    g_Axis_Conf.p_pyb->overcur_prot_th = 65;
#endif

    /* ��ǰλ�ã��ϵ��Ҫͨ��ָ��������ã�Ĭ��Ϊ0 */
    g_Axis_Conf.p_pyb->cur_local = 0;

    g_Axis_Conf.p_pyb->hall_angle = 0;

    //175���ײ��λ��Ĭ����λֵ�ĵ�174��
//    g_Axis_Conf.p_pyb->soft_lim_p =  388889;    /* Ĭ����175�� */
//    g_Axis_Conf.p_pyb->soft_lim_n = -388889;    /* Ĭ����-175�� */
    g_Axis_Conf.p_pyb->soft_lim_p =  386666;    /* Ĭ����174�� */
    g_Axis_Conf.p_pyb->soft_lim_n = -386666;    /* Ĭ����-174�� */


    /* ��λλ�ã�Ĭ����0��ʵ�ʰ�װ�����ƫ���ʱ���ֵ����һ������ */
    g_Axis_Conf.p_pyb->reset_local = 0;

    /* ת�ٱȣ�A����160��B����125 */
    g_Axis_Conf.p_pyb->speed_ratio = 125;

    /* ����� */
    g_Axis_Conf.p_pyb->step_angle = 0.9;

    /* IQ��ʽ��������� */
    g_Axis_Conf.p_pyb->step_angle_iq =_IQ30(g_Axis_Conf.p_pyb->step_angle / (XIFEN_CNT * g_Axis_Conf.p_pyb->speed_ratio));

    /* ����Ჽ��� */
    g_Axis_Conf.p_pyb->step_angle_out_iq23 = _IQ23(0.000225);          /* B������Ჽ���0.9 / (32 * 125) = 0.000225 */


    /* ��λ��ֵת�����Ƕ�ֵ��Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pyb->loc_to_angle_factor = (float64)9 / 20000;

    /* ���ٶ�ֵ���ٶ���Ҫ���ϵ�ϵ�� */
    g_Axis_Conf.p_pyb->speedvalue_to_speed = (float64)9 / 20000;

    /* ��ǰ���ٶ�Ĭ�϶�Ϊ0 */
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;

#if 0
    /* B��λ�ã�-388889~388889����Ӧ�Ƕ�-175.3��~175.3�� */
    g_Axis_Conf.p_pyb->min_loc = -389555;
    g_Axis_Conf.p_pyb->max_loc = 389555;


    /* B��Ƕȣ�-45��~+180�� */
    g_Axis_Conf.p_pyb->min_angle = -175;
    g_Axis_Conf.p_pyb->max_angle = 175;
#endif

    /* ��С�ٶ� */
    g_Axis_Conf.p_pyb->min_speed = -1334;       /* -0.6��/s */

    /* ����ٶ� */
    g_Axis_Conf.p_pyb->max_speed = 1334;    /* 0.6��/s */

    /* +Y-B�ắ��ָ�� */
    g_Axis_Conf.p_pyb->fn_write_a_dir    = __write_PYB_dir_A;       /* ����A�෽�� */
    g_Axis_Conf.p_pyb->fn_write_b_dir    = __write_PYB_dir_B;       /* ����A�෽�� */
    g_Axis_Conf.p_pyb->fn_write_brake    = __write_PYB_brake;       /* ����A�෽�� */

    /**
     * \brief ռ�ձ�ϵ��������Ķ�Ӧ��ϵ
     */
    set_mot_cur();
}
#endif
/**
 * \brief ����У���
 *
 * \param[in] pData��Ҫ���������
 * \param[in] uiLen��Ҫ��������ݳ���
 *
 * \retval ����У���
 */
Uint16 CheckSumCalc (const Uint16 *pData, Uint16 uiLen)
{
    Uint16 i, CheckCode = 0;
    for (i = 0; i < uiLen; i++) {
    	CheckCode += ((*(pData+i)) & 0xFF);
    }
    return CheckCode;
}
