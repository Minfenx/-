/*
 * GL_CJW_HE_5701.c
 *
 *  Created on: 2023��12��13��
 *      Author: Suzkfly
 */
#include "GL_CJW_HE_5701.h"
#include "project.h"
#include "cmd.h"
#include "boot_args.h"
#include "cpu_timer1.h"
#include "cpu_timer0.h"
#include "tele_dat.h"
#include "pwm.h"
#include "sci.h"

#if (BOARD_NAME == BOARD_GMS)

Uint16 g_angle_adj_finish = 0;     /* �Ƕȱ궨��ɵı�־������Ƕȱ궨δ��ɣ��򲻽����˶�����ָ�� */
Uint16 g_angle_senser_hw_normal = 1;     /* �Ƕȴ����������ı�־������Ϊ1��������Ϊ0 */
Uint16 g_angle_senser_cmd_en  	   		= 1;	/* �Ƕȴ������Զ�ʹ�ܱ�־ */
Uint16 g_angle_senser_hw_en             = 1;	/* Ӳ��ʹ�ܱ�־ */
Uint16 g_angle_senser_cmd_eff_flag  	= 0;	/* �Ƕȴ�����ָ����Ч��־ */

#pragma DATA_SECTION(g_angle_adj_finish, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_hw_normal, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_cmd_en, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_hw_en, "pre_roll_data");
#pragma DATA_SECTION(g_angle_senser_cmd_eff_flag, "pre_roll_data");


/**
 * \brief ��λ�ƴ�����ʱ���߸�
 */
#pragma CODE_SECTION(CJW_HE_5701_SET_CLK, "ramfuncs");
void CJW_HE_5701_SET_CLK (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    GpioDataRegs.GPASET.bit.GPIOA11 = 1;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    GpioDataRegs.GPFSET.bit.GPIOF8 = 1;
#endif
}

/**
 * \brief ��λ�ƴ�����ʱ���ߵ�
 */
#pragma CODE_SECTION(CJW_HE_5701_CLEAR_CLK, "ramfuncs");
void CJW_HE_5701_CLEAR_CLK (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    GpioDataRegs.GPACLEAR.bit.GPIOA11 = 1;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    GpioDataRegs.GPFCLEAR.bit.GPIOF8 = 1;
#endif
}

/**
 * \brief ��λ�ƴ�������ȡһ��λ
 */
#pragma CODE_SECTION(CJW_HE_5701_Read_Bit, "ramfuncs");
Uint16 CJW_HE_5701_Read_Bit (void)
{
#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    return GpioDataRegs.GPADAT.bit.GPIOA10;
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    return GpioDataRegs.GPFDAT.bit.GPIOF13;
#endif
}

#if 0	/* ��ȡ�Ƕȵĺ����ŵ�CPUTimer1�� */

/**
 * \brief ��λ�ƴ�������ȡ����
 *
 * \param[out] p_value����ȡ�ɹ������ֵ
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
 */
int16 CJW_HE_5701_Read_Data (Uint16* p_value)
{
	Uint16 i = 0;
	Uint16 temp = 0;

	//PieCtrlRegs.PIEIER2.bit.INTx6 = 0;

	/* ����16��λ��Ҫ��һ���½��غ������� */
	__CJW_HE_5701_CLEAR_CLK();	/* ��һ���½�������ת�� */
	DELAY_US(1);
	__CJW_HE_5701_SET_CLK();	/* ��һ�������ؿ�ʼ�����ݣ����Ƕ�����Ҫ�ڵڶ����½��� */
	DELAY_US(1);

	for (i = 0; i < 16; i++) {
		temp <<= 1;
		__CJW_HE_5701_CLEAR_CLK();
		temp |= __CJW_HE_5701_Read_Bit();		/* ��λ�ȴ� */
		DELAY_US(1);
		__CJW_HE_5701_SET_CLK();
		DELAY_US(1);
	}
	/* ��17��������֮�������߱�Ϊ�ߵ�ƽ����λΪ����λ���������ߣ���ζ�����ݴ��� */
	if (__CJW_HE_5701_Read_Bit() == 0) {	/* �������������û�б�Ϊ�͵�ƽ������Ϊ�������� */
		//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
		return -1;
	}

	__CJW_HE_5701_CLEAR_CLK();
	DELAY_US(1);
	__CJW_HE_5701_SET_CLK();	/* ��18��������֮�����ݱ�Ϊ�͵�ƽ���������͵�ƽ˵���������� */
	DELAY_US(1);

	/* ���һ�������ط���֮�������߻ᱣ��16~24us�ĵ͵�ƽ */
	if (__CJW_HE_5701_Read_Bit() == 1) {	/* �������������û�б�Ϊ�͵�ƽ������Ϊ�������� */
		//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
		return -1;
	}

	//GpioDataRegs.GPADAT.bit.GPIOA9 = 0;
	/* i = 30������ֵΪ41us����Ƶ90MHz�������Ż��� */
	for (i = 30; i > 0; i--) {
		if (__CJW_HE_5701_Read_Bit() == 1) {
			break;
		}
	}
	//PieCtrlRegs.PIEIER2.bit.INTx6 = 1;
	//GpioDataRegs.GPADAT.bit.GPIOA9 = 1;
	if (i > 0) {	/* ����ɹ� */
		*p_value = temp;
		return 0;
	} else {
		return -1;
	}
}
#endif

/**
 * \brief �Ƕȴ�������Դ����
 */
void CJW_HE_5701_Power_On (void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIOB14 = 1;
    g_Tele_Dat.comm_stat.bit.angle_senser_sw = ANGLE_SENSOR_SW_ON;
}

/**
 * \brief �Ƕȴ�������Դ����
 */
void CJW_HE_5701_Power_Off (void)
{
    GpioDataRegs.GPBSET.bit.GPIOB14 = 1;
    g_Tele_Dat.comm_stat.bit.angle_senser_sw = ANGLE_SENSOR_SW_OFF;
}

/**
 * \brief ��λ�ƴ�������ʼ��
 */
void CJW_HE_5701_Init (void)
{
    EALLOW;

    /* ��λ�ƴ�����IO��ʼ�� */

#if GMS_BOARD_VERSION == GMS_BOARD_OLD
    /* ������ */
    GpioMuxRegs.GPAMUX.bit.CAP3QI1_GPIOA10 = 0;		//��ͨIO
    GpioMuxRegs.GPADIR.bit.GPIOA10 = 0;				//����

    /* ʱ���� */
    GpioMuxRegs.GPAMUX.bit.TDIRA_GPIOA11   = 0;		//��ͨIO
    GpioMuxRegs.GPADIR.bit.GPIOA11 = 1;				//���
    GpioDataRegs.GPADAT.bit.GPIOA11 = 1;			//��ʼ���1
#elif GMS_BOARD_VERSION == GMS_BOARD_NEW
    /* ������ */
    GpioMuxRegs.GPFMUX.bit.MDRA_GPIOF13 = 0;     //��ͨIO
    GpioMuxRegs.GPFDIR.bit.GPIOF13 = 0;             //����

    /* ʱ���� */
    GpioMuxRegs.GPFMUX.bit.MCLKXA_GPIOF8   = 0;     //��ͨIO
    GpioMuxRegs.GPFDIR.bit.GPIOF8 = 1;             //���
    GpioDataRegs.GPFDAT.bit.GPIOF8 = 1;            //��ʼ���1
#endif

    /* ��Դʹ������ */
    GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14   = 0;    //��ͨIO
    GpioMuxRegs.GPBDIR.bit.GPIOB14 = 1;             //���

    /* ע���Դʹ����һ˲�䣬�����߻���ݱ����ͣ�����ʱ����Ϊ4.4ms */
    CJW_HE_5701_Power_On();

    EDIS;
}




/**
 * \brief �Ƕȴ������ɼ����ĽǶ�ֵ����Ϊλ��ֵ
 */
int32 CJW_HE_5701_to_cur_local (Uint16 CJW_HE_5701_value)
{
    float angle_f = 0;

    angle_f = (float)CJW_HE_5701_value * 360 / 65535;
    angle_f = angle_f - 180 + *gp_boot_arg_angle_senser_adj;
    if (angle_f > 180) {
        angle_f -= 360;
    } else if (angle_f < -180) {
        angle_f += 360;
    }

    return angle_f / g_Axis_Conf.p_pyb->loc_to_angle_factor;
}

/**
 * \brief �ж����������Ƿ���
 *
 * \parma[in] p_data��Ҫ�жϵ�����
 * \parma[in] cnt�� �жϵ���������
 * \parma[in] lim������
 *
 * \retval ���޷���1�������޷���0
 */
int16 is_over_lim (Uint16* p_data, Uint16 cnt, Uint16 lim)
{
    Uint16 i;
    Uint16 min, max;

    min = p_data[0];
    max = p_data[0];
    for (i = 1; i < cnt; i++) {
        if (p_data[i] < min) {
            min = p_data[i];
        } else if (p_data[i] > max) {
            max = p_data[i];
        }
    }

    if (max - min > lim) {
        return 1;
    }
    return 0;
}

/**
 * \brief ����ʵ���������Ҫ���������λλ�ã����߱�����
 */
static void __adj_soft_lim (void)
{
    if (g_Axis_Conf.p_pyb->cur_local >= g_Axis_Conf.p_pyb->soft_lim_p) {
        //g_Axis_Conf.p_pyb->soft_lim_p = g_Axis_Conf.p_pyb->cur_local;
        //*gp_boot_arg_soft_lim_p = g_Axis_Conf.p_pyb->soft_lim_p;
        g_Tele_Dat.soft_lim_err_stat |= SOFT_LIM_ERR_P;     /* ��������λ */
    }

    if (g_Axis_Conf.p_pyb->cur_local <= g_Axis_Conf.p_pyb->soft_lim_n) {
        //g_Axis_Conf.p_pyb->soft_lim_n = g_Axis_Conf.p_pyb->cur_local;
        //*gp_boot_arg_soft_lim_n = g_Axis_Conf.p_pyb->soft_lim_n;
        g_Tele_Dat.soft_lim_err_stat |= SOFT_LIM_ERR_N;     /* ��������λ */
    }
}

static Uint64 GL_CJW_HE_5701_Tick = 0;
static Uint16 angle_init_read_times = 0;       /* �궨�Ƕ�ʱ��ȡ�ĽǶ�ֵ���� */
static Uint16 angle_temp[10] = { 0 };
static Uint16 invalid_data_cnt = 0;     /* ��Ч���ݼ��� */
static Uint16 read_enable = 1;          /* ��ȡʹ�� */
static Uint16 angle_senser_re_power_supply_times = 0;   /* �Ƕȴ������ظ��ӵ�Ĵ��� */
#pragma DATA_SECTION(GL_CJW_HE_5701_Tick, "pre_roll_data");
#pragma DATA_SECTION(angle_init_read_times, "pre_roll_data");
#pragma DATA_SECTION(angle_temp, "pre_roll_data");
#pragma DATA_SECTION(invalid_data_cnt, "pre_roll_data");
#pragma DATA_SECTION(read_enable, "pre_roll_data");
#pragma DATA_SECTION(angle_senser_re_power_supply_times, "pre_roll_data");


/**
 * \brief �Ƕȴ���������
 */
void Angle_Senser_Task (void)
{
    int32  angle_senser_loc = 0;

    float32 mot_angle = 0;
    int32 angle_temp1 = 0;

    if ((g_angle_senser_hw_normal == 1) && (g_angle_senser_hw_en == 1)) {

        /* ׼����ȡ��λ�ƴ�������ֵ */
        if (read_enable == 1) {
            if (ElapsedTick(GL_CJW_HE_5701_Tick) >= TICK_200MS) {
				GL_CJW_HE_5701_Tick = GetSysTick();	//ֱ������

            	if ((sSendData.Index == 0) && (sSendData.Length == 0)) {	/* ֻ�д���û�ڷ��͵�ʱ����ܿ�ʼ��� */
					g_read_angle_ready = 1;
            	}
            }
        } else {
            if (ElapsedTick(GL_CJW_HE_5701_Tick) >= TICK_3S) {
                GL_CJW_HE_5701_Tick = GetSysTick();
                read_enable = 1;
                CJW_HE_5701_Power_On();
            }
        }

        /* �Ƕȴ������Ĳɼ���PWM�п�ʼ�ģ���������˹��±���������Ҫ�����￪���ǶȲɼ� */
        if (g_pdpinta_pyb_flag == 1) {
            if (g_read_angle_ready) {
                g_read_angle_ready = 0;
                g_angle_is_reading = 1;
                StartCpuTimer1();           /* �����ǶȲɼ� */
            }
        }

        if (g_CJW_HE_5701_read_finish == 1) {       	/* һ�ζ�ȡ���� */
            g_CJW_HE_5701_read_finish = 0;

            if (g_angle_senser_cmd_en == 0) {	/* �رմ����� */
            	CJW_HE_5701_Power_Off();
            	g_angle_senser_hw_en = 0;
            }

            if (g_CJW_HE_5701_read_acitve == 1) {   /* ������Ч */
                invalid_data_cnt = 0;
                angle_temp1 = g_CJW_HE_5701_angle;

                /* �û�ȡ���ĽǶ�ֵ �궨����Ƕ�ֵ�ͻ����Ƕ�ֵ */
                if (g_angle_adj_finish == 0) {      /* ��δ��ɽǶȱ궨 */
                    angle_temp[angle_init_read_times++] = angle_temp1;
#if 0       /* ģ��3�������̫������ */
                    angle_temp[0] = 10000;
                    angle_temp[1] = 10000;
                    angle_temp[2] = 10100;
#elif 0     /* ģ��Ƕȴ�������ֵ�����󣬵����뱣��ĽǶ�ֵ���ϴ� */
                    Uint16 i;
                    for (i = 0; i < 10; i++) {
                        angle_temp[i] = 10000;
                    }
                    angle_temp1 = 10000;
                    *gp_boot_arg_last_pd_local_motor = 0;
#elif 0     /* ģ��Ƕȴ�������ֵ�����ϴ󣬵�����ֵ��ǿ����/������ */
                    Uint16 i;
                    for (i = 0; i < 10; i++) {
                        angle_temp[i] = 10000;
                    }
                    angle_temp1 = 10000;
                    angle_temp[0] = 10100;
                    *gp_boot_arg_last_pd_local_motor = 0;
#endif

                    /* �ɼ�����3����ƫ���̫�󣬷������²ɼ� */
                    if (angle_init_read_times == 3) {
                        if (is_over_lim(angle_temp, 3, 5) == 0) {   /* ����3����������5 */
                        //if (is_over_lim(angle_temp, 3, 5000) == 0) {   /* ��������������ֵ */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            if (((angle_senser_loc >= (*gp_boot_arg_last_pd_local_motor)) && (angle_senser_loc - (*gp_boot_arg_last_pd_local_motor) < 4444)) ||     /* ���ϴζϵ�λ�ý��бȽ� ��������2����Ϊ���� */
                                ((angle_senser_loc <  (*gp_boot_arg_last_pd_local_motor)) && ((*gp_boot_arg_last_pd_local_motor) - angle_senser_loc < 4444))) {
                                g_angle_adj_finish = 1;
                                g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* �Ƕȴ��������� */
                                g_Axis_Conf.p_pyb->cur_local = angle_senser_loc;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                            }
                        } else if (is_over_lim(angle_temp, 3, 91) == 1) {
                            angle_init_read_times = 0;
                            read_enable = 0;
                            CJW_HE_5701_Power_Off();
                            angle_senser_re_power_supply_times++;
                            if (angle_senser_re_power_supply_times == 3) {
                                g_angle_senser_hw_normal = 0;
                                g_angle_adj_finish = 1;
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                            }
                        }
                    } else if (angle_init_read_times == 10) {
                        if (is_over_lim(angle_temp, 10, 5) == 0) {   /* ����10����������5 */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            g_angle_adj_finish = 1;
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* �Ƕȴ��������� */
                            g_Axis_Conf.p_pyb->cur_local = angle_senser_loc;
                            __adj_soft_lim();
                            g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                        } else if (is_over_lim(angle_temp, 10, 91) == 0) {          /* ����10��������5�����ǲ�����91 */
                            angle_senser_loc = CJW_HE_5701_to_cur_local(angle_temp1);
                            g_angle_adj_finish = 1;
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 1;    /* �Ƕȴ������쳣 */
                            g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                            __adj_soft_lim();
                            g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            g_Tele_Dat.p_yb_axis_angle_res = angle_senser_loc * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                        } else {
                            read_enable = 0;
                            CJW_HE_5701_Power_Off();
                            angle_senser_re_power_supply_times++;
                            angle_init_read_times = 0;
                            if (angle_senser_re_power_supply_times == 3) {
                                g_angle_senser_hw_normal = 0;
                                g_angle_adj_finish = 1;
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                __adj_soft_lim();
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                                //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                            }
                        }
                    }
                } else {   /* �Ѿ���ɽǶȱ궨���жϽǶȴ������Ƿ��쳣 */
                    angle_senser_re_power_supply_times = 0;

                    g_Tele_Dat.p_yb_axis_angle_res = CJW_HE_5701_to_cur_local(g_CJW_HE_5701_angle) * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
                    mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                    if (((mot_angle >= g_Axis_Conf.p_pyb->hall_angle) && (mot_angle - g_Axis_Conf.p_pyb->hall_angle < 6)) ||
                        ((mot_angle <  g_Axis_Conf.p_pyb->hall_angle) && (g_Axis_Conf.p_pyb->hall_angle - mot_angle < 6))) {    /* ����Ƕ�ֵ������Ƕ�ֵ������5�㣨��Ϊ6���������һ���ص��ݴ� */

                        if (((g_Tele_Dat.p_yb_axis_angle_res >   mot_angle * 1000) && (g_Tele_Dat.p_yb_axis_angle_res - mot_angle * 1000 < 2000)) ||
                            ((g_Tele_Dat.p_yb_axis_angle_res <=  mot_angle * 1000) && (mot_angle * 1000 - g_Tele_Dat.p_yb_axis_angle_res < 2000))) {    /* ����Ƕ�ֵ��Ƕȴ�������ֵ������2�� */
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;
                        } else {
                            g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 1;
                        }
                    }
                }
            } else {
                invalid_data_cnt++;

                if (g_angle_adj_finish == 0) {
                    angle_init_read_times = 0;
                }

                if (invalid_data_cnt >= 5) {   /* ����5�ζ�ȡʧ�� */
                    invalid_data_cnt = 0;
                    read_enable = 0;
                    CJW_HE_5701_Power_Off();

                    angle_senser_re_power_supply_times++;

                    if (angle_senser_re_power_supply_times == 3) {
                        angle_senser_re_power_supply_times = 0;
                        g_angle_senser_hw_normal = 0;
                        __adj_soft_lim();

                        if (g_angle_adj_finish == 0) {
                            /* ʹ�ñ���ĵ���Ƕ�ֵ */
                            if (((*gp_boot_arg_last_pd_local_motor) * g_Axis_Conf.p_pyb->loc_to_angle_factor - (*gp_boot_arg_last_pd_local_hall) <  6) &&
                                ((*gp_boot_arg_last_pd_local_motor) * g_Axis_Conf.p_pyb->loc_to_angle_factor - (*gp_boot_arg_last_pd_local_hall) > -6)) {
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            } else if ((*gp_boot_arg_last_pd_local_hall > 0 - 180) && ((*gp_boot_arg_last_pd_local_hall < 180))) {    /* ʹ�ñ���Ļ����Ƕ�ֵ */
                                g_Axis_Conf.p_pyb->hall_angle = *gp_boot_arg_last_pd_local_hall;
                                g_Axis_Conf.p_pyb->cur_local = g_Axis_Conf.p_pyb->hall_angle / g_Axis_Conf.p_pyb->loc_to_angle_factor;
                            } else {	/* ����ʹ�õ���Ƕ�ֵ */
                                g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;
                                g_Axis_Conf.p_pyb->hall_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
                                *gp_boot_arg_last_pd_local_hall = g_Axis_Conf.p_pyb->hall_angle;
                            }
                            //g_Tele_Dat.p_yb_axis_angle_res = 180000;
                        }

                        g_angle_adj_finish = 1;
                    }
                }
            }
        }
    } else {	/* ���ɽǶȴ�����ʱ�Ƕȴ�����ֵ�������Ƕ�ֵ */
    	g_Tele_Dat.p_yb_axis_angle_res = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor * 1000;
    	g_Tele_Dat.p_yb_axis_stat.bit.angle_senser_stat = 0;    /* �Ƕȴ��������� */
    	GL_CJW_HE_5701_Tick = GetSysTick();
    }
}
#endif
