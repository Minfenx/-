/*
 * cycle_hall.c
 *
 *  Created on: 2024��2��27��
 *      Author: Suzkfly
 */
#include  "project.h"
#if BOARD_NAME == BOARD_GMS
#include "cycle_hall.h"
#include "cmd.h"
#include "GL_CJW_HE_5701.h"
#include "tele_dat.h"
#include "boot_args.h"

/**< \brief ��������������� */
float g_hall_angle = 0;
#pragma DATA_SECTION(g_hall_angle, "pre_roll_data");

/* ��ѯ��ʽ */
#if 1

static Uint16 __g_old_level = 0;
#pragma DATA_SECTION(__g_old_level, "pre_roll_data");

static void __hall_gpio_init (void)
{
    EALLOW;
    GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1 = 0;     //��ͨIO
    GpioMuxRegs.GPEDIR.bit.GPIOE1 = 0;                  //����
    GpioMuxRegs.GPEQUAL.bit.QUALPRD = 0x2;             //QUALPRD = SYSCLKOUT / 4 ���ﲻ�ܸ�̫��
    EDIS;
}

/**
 * \brief PPS��ʼ��
 */
void Hall_Init (void)
{
    g_hall_angle = 0;
    __hall_gpio_init();

    __g_old_level = GpioDataRegs.GPEDAT.bit.GPIOE1;
}

/**
 * \brief ���»����Ƕ�
 */
void Hall_angle_updat (void)
{
    Uint16 new_level = 0;

    new_level = GpioDataRegs.GPEDAT.bit.GPIOE1;

    /* ��ƽ�����˱仯 */
    if (__g_old_level != new_level) {
        if ((__g_old_level == 0) && (new_level == 1)) {     /* �ӵ͵��ߣ�todoֹ֮ͣ��������ź� */
            if (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) {
                g_Axis_Conf.p_pyb->hall_angle += 2.88;
                (*gp_boot_arg_last_pd_local_hall) = g_Axis_Conf.p_pyb->hall_angle;
            }
        } else {
            if (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) {
                g_Axis_Conf.p_pyb->hall_angle -= 2.88;
                (*gp_boot_arg_last_pd_local_hall) = g_Axis_Conf.p_pyb->hall_angle;
            }
        }

        __g_old_level = new_level;
    }
}

#else   /* �жϷ�ʽ */
/**
 * \brief ��ʼ��XINT2��GPIO
 */
static void __InitXINT2Gpio (void)
{
	EALLOW;

//	GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0  = 1;		//����XINT1
	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1  = 1;	//����XINT2
//	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2  = 1;		//����XINT3

    EDIS;
}

#pragma CODE_SECTION(XINT2_isr, "ramfuncs");
interrupt void XINT2_isr (void)
{
//	if (g_Axis_Conf.p_pyb->is_task_running) {
//		__g_hall_cnt += g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->stage_cnt].mov_dir;
//	} else {
//		//����
//	}
    __g_hall_cnt++;
    // ���PIEӦ��Ĵ����ĵ�1λ������Ӧ��1�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

static void __EXTI2_Init (void)
{
	__InitXINT2Gpio();									//��ʼ��IO
	/* �����ⲿ�жϼĴ��� */
	EALLOW;
	XIntruptRegs.XINT2CR.bit.POLARITY = 1;				//�������ж�
	/* �����ж�ʹ�� */
	XIntruptRegs.XINT2CR.bit.ENABLE   = 1;				//ʹ��XINT2�ж�
	/* PIE�ж�ʹ�� */
	PieVectTable.XINT2 = &XINT2_isr ;
	PieCtrlRegs.PIEIER1.bit.INTx5 = 1;	//PIE��1 ,���5 XINT2 ,ʹ��PIE�ж�
	/* cpu INT1�ж� */
	IER |= M_INT1;
    EDIS;
}

/**
 * \brief PPS��ʼ��
 */
void Hall_Init (void)
{
	__g_hall_cnt = 0;
	__EXTI2_Init();
}

/**
 * \brief �õ�HALL���������
 */
int16 get_hall_cnt (void)
{
	return __g_hall_cnt;
}
#endif

/**
 * \brief ���»����Ƕȣ����жϻ����Ƕ��Ƿ�����
 */
void Cycle_Hall_Task (void)
{
    float32 mot_angle = 0;

    if (g_angle_adj_finish == 1) {
        Hall_angle_updat();

        /* �������Ƕ���Ƕȴ������ĽǶ����󣬵������Ƕ����������ܴ�����Ϊ�����쳣 */
        mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;

        if (g_angle_senser_hw_normal == 1) {    /* �Ƕȴ�����Ӳ������������£����Ƕȴ�������ֵ����ο���Χ */
            if (((mot_angle * 1000 >= g_Tele_Dat.p_yb_axis_angle_res) && (mot_angle * 1000 - g_Tele_Dat.p_yb_axis_angle_res < 2000)) ||
                ((mot_angle * 1000 <  g_Tele_Dat.p_yb_axis_angle_res) && (g_Tele_Dat.p_yb_axis_angle_res - mot_angle * 1000 < 2000))) {     /* �Ƕȴ�������ֵ�����Ƕ�ֵ������2�� */
                if (((g_Axis_Conf.p_pyb->hall_angle >  mot_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6)) ||
                    ((g_Axis_Conf.p_pyb->hall_angle <= mot_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6))) {    /* �����Ƕ���Ƕȴ�������ֵ������5�㣨��Ϊ6�ȣ���������һ���ص��ݴ� */
                    g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 0;
                } else {
                    g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 1;
                }
            }
        } else {
            if (((g_Axis_Conf.p_pyb->hall_angle >  mot_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6)) ||
                ((g_Axis_Conf.p_pyb->hall_angle <= mot_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6))) {
                g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 0;
            } else {
                g_Tele_Dat.p_yb_axis_stat.bit.hall_stat = 1;
            }
        }
    }
}
#endif
