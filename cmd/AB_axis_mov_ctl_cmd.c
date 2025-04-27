/*
 * AB_axis_mov_ctl_cmd.c
 *
 *  Created on: 2023��5��29��
 *      Author: mj
 */
#include "AB_axis_mov_ctl_cmd.h"
#include "cmd.h"
#include "project.h"
#include <math.h>
#include <string.h>
#include "cpu_timer0.h"
#include "tele_dat.h"
#include "pps.h"

#if BOARD_NAME == BOARD_GMS
uint8_t  g_last_work_mode_set = 0;          /* �ϴι���ģʽ */
int32    g_last_loc_given     = 0;          /* �ϴ�λ�ø��� */
int16    g_last_speed_given   = 0;          /* �ϴ��ٶȸ��� */
#elif (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_DGM_2)
uint8_t  g_last_work_mode_set[AXIS_CNT] = { 0 };          /* �ϴι���ģʽ */
int32    g_last_loc_given[AXIS_CNT]     = { 0 };          /* �ϴ�λ�ø��� */
int16    g_last_speed_given[AXIS_CNT]   = { 0 };          /* �ϴ��ٶȸ��� */
#endif

//Uint16 g_force_move_cmd_cnts = 0;

/**
 * \brief ����ռ�ձ�
 *
 * \param[in] k����k������(0 �� k �� N)
 * \param[in] N_4��1/4�������ڰ����ĵ���
 * \note sin������ֵΪ��/2 * (k/N)
 */
#if 1
static float32 __duty(Uint16 k, Uint16 N_4) {
    const _iq20 iq20_PI = _IQ20(M_PI);
    // ֱ��ʹ�� IQ20 ��ʽ���г˳�����
    _iq20 input = _IQ20div(_IQ20mpy(_IQ20(k), iq20_PI), _IQ20(2 * N_4));
    _iq20 sin_out = _IQ20sin(input);
    return _IQ20toF(sin_out);
}
#else
static float32 __duty (Uint16 k, Uint16 N_4)
{
    float32 dy;

    dy = sin(k * M_PI / (2 * N_4));

    return dy;
}
#endif

/**
 * \brief       �õ���1/4�����ڵ�����΢����ռ�ձ�ֵ
 *
 * \param[in] N��1/4��������ϸ����
 * \param[out] p_duty��������ռ�ձ���ֵ
 *
 * \note sin(2*pi*t/T)��ͼ��
 */
void get_all_duty (Uint16 N_4, float32* p_duty)
{
    Uint16 i = 0;
    //����N_4+1��ֵ
    for (i = 0; i <= N_4; i++) {
        *p_duty++ = __duty(i, N_4);
    }
}

/**
 * \brief �������ݽ���
 *
 * \param[in]  pData����Ҫ�����Ĵ�������ָ�룬��֡ͷ��ʼ
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
	single_axis_conf_t  axis_temp[4];
	single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
	uint8_t i, j;

    /* ����AB���˶�����ָ���Ҫ�ж������Ƿ����ڽ��� */

	/* �ж����ݵĺϷ��� */
	j = 6;
	for (i = 0; i < 4; i++) {
		axis_temp[i].work_mode_set  = pData[j++];
		axis_temp[i].loc_given      = ((Uint32)pData[j] << 24) | ((Uint32)pData[j+1] << 16) | (pData[j+2] << 8) | pData[j+3];
		j += 4;
		axis_temp[i].speed_given    = (pData[j] << 8) | pData[j+1];
		j += 2;
		
		if (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
			if ((axis_temp[i].loc_given < p_axis->min_loc) || (axis_temp[i].loc_given > p_axis->max_loc)) {
				return -1;
			}
		} 
		if (((axis_temp[i].work_mode_set == WORK_MODE_TRACE) && (i < 2)) || (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE)) {
			if ((axis_temp[i].speed_given < p_axis->min_speed) || (axis_temp[i].speed_given > p_axis->max_speed) || (axis_temp[i].speed_given == 0)) {	/* �ٶȸ�������Ϊ0 */
				return -1;
			}
		}

		if ((i >= 2) && (axis_temp[i].work_mode_old == WORK_MODE_LAUNCH)) {	/* B��չ��ģʽ�²�����������ָ�� */
			return -1;
		}

		/* B�Ჶ��ģʽ���ܳ��������λλ�� */
		if ((i >= 2) && axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
			if (((axis_temp[i].loc_given > 0) && (axis_temp[i].loc_given >  p_axis->soft_lim)) ||
				((axis_temp[i].loc_given < 0) && (axis_temp[i].loc_given < -p_axis->soft_lim))) {
				return -1;
			}
		}
		p_axis = p_axis->p_next_axis;
	}
	
	p_axis = g_Axis_Conf.p_pya;
	for (i = 0; i < 4; i++) {
		p_axis->work_mode_set = axis_temp[i].work_mode_set;
		p_axis->loc_given     = axis_temp[i].loc_given;
		p_axis->speed_given   = axis_temp[i].speed_given;
		p_axis = p_axis->p_next_axis;
	}

    return 0;
}
#elif (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
    single_axis_conf_t  axis_temp[AXIS_CNT];
    single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;
    uint8_t i, j;

    /* ����AB���˶�����ָ���Ҫ�ж������Ƿ����ڽ��� */

    /* �ж����ݵĺϷ��� */
    j = 7;
    for (i = 0; i < AXIS_CNT; i++) {
        axis_temp[i].work_mode_set  = pData[j++];
        axis_temp[i].loc_given      = ((Uint32)pData[j] << 24) | ((Uint32)pData[j+1] << 16) | (pData[j+2] << 8) | pData[j+3];
        j += 4;
        axis_temp[i].speed_given    = (pData[j] << 8) | pData[j+1];
        j += 2;

        if (((axis_temp[i].work_mode_set == WORK_MODE_TRACE) && (i < 2)) || (axis_temp[i].work_mode_set == WORK_MODE_CAPTURE)) {
            if ((axis_temp[i].speed_given < p_axis->min_speed) || (axis_temp[i].speed_given > p_axis->max_speed) || (axis_temp[i].speed_given == 0)) {  /* �ٶȸ�������Ϊ0 */
                return -1;
            }
        }

        if ((i >= 2) && (axis_temp[i].work_mode_old == WORK_MODE_LAUNCH)) { /* B��չ��ģʽ�²�����������ָ�� */
            return -1;
        }

        /* ����Ƕ�ֵ������45���򲻽���չ��ָ�� */
        if (((i == 2) && (g_Axis_Conf.p_pyb->cur_local < 50000)) ||
            ((i == 3) && (g_Axis_Conf.p_nyb->cur_local < 50000))) {
        	if (axis_temp[i].work_mode_set == WORK_MODE_LAUNCH) {
        		return -1;
        	}
        }

        /* B�Ჶ��ģʽ���ܳ��������λλ�� */
        if ((i >= 2) && axis_temp[i].work_mode_set == WORK_MODE_CAPTURE) {
            if (((axis_temp[i].loc_given > 0) && (axis_temp[i].loc_given >  p_axis->soft_lim)) ||
                ((axis_temp[i].loc_given < 0) && (axis_temp[i].loc_given < -p_axis->soft_lim))) {
                return -1;
            }
        }

        p_axis = p_axis->p_next_axis;
    }

    g_Axis_Conf.cmd_pps      = g_pps;
    g_Axis_Conf.run_delay	 = pData[j];

    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < AXIS_CNT; i++) {
        if ((g_last_work_mode_set[i] == axis_temp[i].work_mode_set) &&
            (g_last_loc_given[i]     == axis_temp[i].loc_given) &&
            (g_last_speed_given[i]   == axis_temp[i].speed_given)) {
            p_axis->new_task_flag = 0;
        } else {
            g_last_work_mode_set[i] = axis_temp[i].work_mode_set;
            g_last_loc_given[i]     = axis_temp[i].loc_given;
            g_last_speed_given[i]   = axis_temp[i].speed_given;
            p_axis->new_task_flag = 1;

            p_axis->work_mode_set = axis_temp[i].work_mode_set;
            p_axis->loc_given     = axis_temp[i].loc_given;
            p_axis->speed_given   = axis_temp[i].speed_given;
        }

        if ((p_axis->work_mode_set == WORK_MODE_KEEP) && (p_axis->work_mode_old == WORK_MODE_STANDBY)) {
        	p_axis->work_mode_old = WORK_MODE_KEEP;
        }

        if ((p_axis->work_mode_set == WORK_MODE_STANDBY) && (p_axis->work_mode_old == WORK_MODE_KEEP)) {
        	p_axis->work_mode_old = WORK_MODE_STANDBY;
        }

#if BOARD_NAME == BOARD_DGM_2
        if (!strncmp(p_axis->name, "PYA", 3)){
            g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }else if(!strncmp(p_axis->name, "NYA", 3)){
            g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }
#elif BOARD_NAME == BOARD_DGM_4
        if (!strncmp(p_axis->name, "PYA", 3)){
            g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }else if(!strncmp(p_axis->name, "NYA", 3)){
            g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }else if(!strncmp(p_axis->name, "PYB", 3)){
            g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }else if(!strncmp(p_axis->name, "NYB", 3)){
            g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
        }
#endif

        p_axis = p_axis->p_next_axis;
    }

    return 0;
}
#elif BOARD_NAME == BOARD_GMS
#include "GL_CJW_HE_5701.h"
#include "pwm.h"

uint8_t g_angle_sensor_task_req = 0;	/* �Ƕȴ�������������Ϊ0��ʾ�����ޣ�Ϊ1��ʾ������ */
#pragma DATA_SECTION(g_angle_sensor_task_req, "pre_roll_data");

uint8_t g_angle_sensor_sw_req = 0;		/* �Ƕȴ��������أ�Ϊ0��ʾ�أ�Ϊ1��ʾ�� */
#pragma DATA_SECTION(g_angle_sensor_sw_req, "pre_roll_data");

static Uint64 __g_CJW_HE_5701_sw_Tick = 0;
#pragma DATA_SECTION(__g_CJW_HE_5701_sw_Tick, "pre_roll_data");

void CJW_HE_5701_sw_req_task (void)
{
	if ((g_angle_sensor_task_req == 1) && (ElapsedTick(__g_CJW_HE_5701_sw_Tick) >= TICK_30S)) {
		g_angle_sensor_task_req = 0;

		if ((g_angle_sensor_sw_req == 0) && (g_Tele_Dat.comm_stat.bit.angle_senser_sw == ANGLE_SENSOR_SW_ON)) {	/* �� */
			g_angle_senser_cmd_en = 0;
			__g_CJW_HE_5701_sw_Tick = GetSysTick();
		} else if ((g_angle_sensor_sw_req == 1)  && (g_Tele_Dat.comm_stat.bit.angle_senser_sw == ANGLE_SENSOR_SW_OFF)) {	/* �� */
        	g_angle_senser_cmd_en = 1;
            g_angle_senser_hw_en = 1;
            if (g_angle_senser_hw_normal == 1) { //Ӳ����־����ֵ����Ҫ�ж�
                CJW_HE_5701_Power_On();
            }
            __g_CJW_HE_5701_sw_Tick = GetSysTick();
		}
	}
}



/* �����жϽ��յ���ͬ�Ĳ���ģʽ��Ҫ�и�ʱ���� */
uint8_t g_force_hold_flag = 0;		/* ǿ��ֹͣ��־ */
#pragma DATA_SECTION(g_force_hold_flag, "pre_roll_data");
int8_t get_ab_axis_mov_ctl_dat (const Uint16 *pData)
{
	single_axis_conf_t  axis_temp;
	single_axis_conf_t* p_axis = g_Axis_Conf.p_pyb;

	if (g_force_hold_flag == 1) {
		return -1;
	}

//	if (g_Tele_Dat.soft_lim_err_stat != 0) {    /* ������λλ�ò������˶�����ָ�� */
//	    return -1;
//	}

    /* ����AB���˶�����ָ���Ҫ�ж������Ƿ����ڽ��У����ǽǶȴ�������״̬����ȷ�� */
	if ((g_angle_adj_finish == 1) && (g_pdpinta_pyb_flag == 0)) {
        /* �ж����ݵĺϷ��� */
        axis_temp.work_mode_set  = pData[6];
        axis_temp.loc_given      = ((Uint32)pData[7] << 24) | ((Uint32)pData[8] << 16) | (pData[9] << 8) | pData[10];
        axis_temp.speed_given    = (pData[11] << 8) | pData[12];
        g_Axis_Conf.run_delay	 = pData[13];

        /* ����ģʽ���ܳ�����λλ�� */
        if (axis_temp.work_mode_set == WORK_MODE_CAPTURE) {
            if ((axis_temp.loc_given < p_axis->soft_lim_n) || (axis_temp.loc_given > p_axis->soft_lim_p)) {
//                g_force_move_cmd_cnts++;
//                if (g_force_move_cmd_cnts < 5) {
//                    p_axis->new_task_flag = 0;
//                    return -1;
//                }
            	return -1;
            }
        }

        if ((axis_temp.work_mode_set == WORK_MODE_TRACE) || (axis_temp.work_mode_set == WORK_MODE_CAPTURE)) {
            if ((axis_temp.speed_given < p_axis->min_speed) || (axis_temp.speed_given > p_axis->max_speed) || (axis_temp.speed_given == 0)) {	/* �ٶȸ�������Ϊ0 */
				p_axis->new_task_flag = 0;
				return -1;
            }
        }

        /* ����ģʽ�£�����Ѿ���������λλ�ã����ܽ�������λ�����˶���ָ�� */
        if (axis_temp.work_mode_set == WORK_MODE_TRACE) {
            if (((p_axis->cur_local >= p_axis->soft_lim_p) && (axis_temp.speed_given > 0)) ||
                ((p_axis->cur_local <= p_axis->soft_lim_n) && (axis_temp.speed_given < 0))) {

				p_axis->new_task_flag = 0;
				return -1;
            }
        }

        /* ������趨��ģʽ��������ɵ�ģʽ��������ͬ����ô��ֱ������ */
        if ((g_last_work_mode_set == axis_temp.work_mode_set) &&
            (g_last_loc_given     == axis_temp.loc_given) &&
            (g_last_speed_given   == axis_temp.speed_given)) {

            p_axis->new_task_flag = 0;
        } else {
            g_last_work_mode_set = axis_temp.work_mode_set;
            g_last_loc_given     = axis_temp.loc_given;
            g_last_speed_given   = axis_temp.speed_given;
            p_axis->new_task_flag = 1;

            p_axis->work_mode_set = axis_temp.work_mode_set;
            p_axis->loc_given     = axis_temp.loc_given;
            p_axis->speed_given   = axis_temp.speed_given;
            g_Axis_Conf.cmd_pps   = g_pps;

//            if ((!strncmp(p_axis->name, "PYB", 3)) && (axis_temp.work_mode_set != WORK_MODE_RESET)) {
//                  g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 0;    /* δ�յ�����ָ�� */
//            }
        }

        /* ����ģʽ���Զ��رսǶȴ����� */
        if ((g_angle_senser_cmd_eff_flag == 0) && (p_axis->new_task_flag == 1)) {
            if (axis_temp.work_mode_set == WORK_MODE_TRACE) {
                g_angle_sensor_task_req = 1;
                g_angle_sensor_sw_req = 0;
            } else if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) || (axis_temp.work_mode_set == WORK_MODE_RESET)) {    /* ����ģʽ�����ģʽ���Զ������Ƕȴ����� */
                g_angle_sensor_task_req = 1;
                g_angle_sensor_sw_req = 1;
            }
        }

        return 0;
	}
    return -1;
}
#endif

// �������ƺ���������int32_t�����������������λ���������
// num: Ҫ���Ƶ�����
// shiftAmount: ���Ƶ�λ��
// �������ƺ�Ľ��
int32 fastLeftShift(int32 num, int shiftAmount) {
    int32 signBit = num & 0x80000000;                   // �������λ
//    if (shiftAmount >= 32 || shiftAmount <= 0) {
//        return 0;                                       // �������������0������������ʽ
//    }
    num =  (num << shiftAmount) & 0x7FFFFFFF;
    return num | signBit;
}

/**
 * \brief ���ɣ������¸����������ٶȣ�V=V0+at t=k/v k=0.9/(160*16)�������ַ��������V����V0�õ��ģ��¸�V������ǰһ��V�õ��ģ���ʵ����֤�����ǳ��󣬶��������ۼƣ�������
 * 		      Ӧ����x = v0t + (1/2)at^2 ÿ��x�ı仯�ǹ̶��ģ�ֻ��Ҫ����v0�������t
 * 		     һ��x=0.0003515625�㣬��������ʵֵΪ0.000325���нض�������1000���Ժ�Ϊ0.351562����һ��5��
 * 		     �õ�t = (sqrt(v0^2 + 2ax) - v0) / a
 *
 *
 * 		     ���£�������һ��΢���е�50us����������һ����Ϊ1/4���������ڣ��������32ϸ�֣�ÿ��С���൱��1/32������Ӧ�Ƕ�Ϊx=0.05625��=1.8/32	��δ����ǰ��
 * \note Ϊ�˽�ʡ�����������ϴεļ�����t2��static���ͱ��棬ʵ��ʹ�õ�ʱ��elapsed_pulse_cnt����ֻ����һ�ζ�1
 * \retval �����¸�΢���� , PWM���ĸ��� , Ŀǰ���õ�pwm���ڹ̶�50us , �൱����Сʱ�������
 */
/* ������ʱ29.5us */

float real_v = 0;
#pragma DATA_SECTION(real_v, "pre_roll_data");

#pragma CODE_SECTION(get_next_segment_pulse_cnt, "ramfuncs");
Uint32 get_next_segment_pulse_cnt (single_axis_conf_t* p_axis)
{
    Uint32 elapsed_segment_cnt;
    int32 temp_cnt;
    int8_t dir;
    Uint32 ret;
    float64 t1;
    _iq1   elapsed_segments_iq , t1_us_iq ,t2_us_iq , ret_iq , v0_1000_iq ,a_1000_iq ;
    _iq24 a_iq24, part2_iq24, delt_iq24, v0_sq_iq24, v0_iq24, delt_x_iq24, sqrt_delt_iq24, v_diff_iq24, t_iq24;
//    float temp;
    const _iq30  g_1000_iq = _IQ30(0.001);
    const _iq1   g_1000000_iq = _IQ1(1000000);

//    GpioDataRegs.GPBDAT.bit.GPIOB1 = 1;

    /* ��ȡ��ǰ�׶�������ĳ��ٶ�V0 , ���ٶ�a , �Լ�΢����*/
    /* -600~600 */
    v0_1000_iq = fastLeftShift(p_axis->stage[p_axis->cur_stage].v0_1000 , 1); //��ǰ�׶γ��ٶ�ת��ΪIQ��ʽ , �����ʵ���ٶ� ,��λ��/s
    //temp = _IQ1toF(v0_1000_iq);

    a_1000_iq  = fastLeftShift(p_axis->stage[p_axis->cur_stage].a_1000 , 1);  //��ǰ�׶μ��ٶ�ת��ΪIQ��ʽ  , �����ʵ�ʼ��ٶ� ,��λ��/s2
    //temp = _IQ1toF(a_1000_iq);

    v0_iq24 = _IQ24mpyIQX(v0_1000_iq, 1, g_1000_iq, 30);
    //temp = _IQ24toF(v0_iq24);

    a_iq24  = _IQ24mpyIQX(a_1000_iq, 1, g_1000_iq, 30) ;
    //temp = _IQ24toF(a_iq24);

    /* ��ǰ�׶����߹���΢�����ͷ��� */
    elapsed_segment_cnt = p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt;
    dir = p_axis->stage[p_axis->cur_stage].mov_dir;

    if (elapsed_segment_cnt == 0) {
        t1 = 0;
    } else {
        t1 = p_axis->t2_us;         /* ��һ��΢��������ʱ��*/
    }
    /* ��ȡ�������һ������һ���߹��ĽǶ�  , ���Ƕȡ�2048*/

    temp_cnt = ((elapsed_segment_cnt + 1) * dir); //ֱ�ӽ�������ʽ����_IQ1����ּ������ ,��Ϊelapsed_segment_cnt�Ǹ��޷�����
    elapsed_segments_iq = fastLeftShift(temp_cnt , 1);
    delt_x_iq24 = _IQ24mpyIQX(elapsed_segments_iq , 1 , p_axis->step_angle_iq , 30);//������ͬ��IQ������� ����һ��IQֵ ,��ǰ�׶��߹��ĽǶ� ,��Ҫ������ٺͼ��ٹ��� ,�Ƕȱ仯��Χ����
    //temp = _IQ24toF(delt_x_iq24);

    /* b^2 */
    v0_sq_iq24  = _IQ24mpy(v0_iq24 , v0_iq24);                                  //v0��ƽ�� ,��Χ 0-4
    //temp = _IQ24toF(v0_sq_iq24);

    part2_iq24  = _IQ24mpy(a_iq24, delt_x_iq24);                               //a * x
    //temp = _IQ24toF(part2_iq24);

    /* -4ac */
    part2_iq24  = _IQ24mpy(part2_iq24, _IQ24(2));
    //temp = _IQ24toF(part2_iq24);

    delt_iq24 = v0_sq_iq24 + part2_iq24;                                      //V0��+2ax = v��
    //temp = _IQ24toF(delt_iq24);

    /* ���ڸ��������㾫�ȵ����⣬��ĳ���׶ε�ĩβ���ܻ������ʵ�������������ʱֱ�ӷ����ϸ��������� */
    if (delt_iq24 < 0) {
        return p_axis->next_segment_pulse_cnt;
    }

    /*�������v ,�ֱ���VΪ���͸������ ,���ռ����V��V0�Ĳ�ֵ*/
    sqrt_delt_iq24 = _IQ24sqrt(delt_iq24);
    real_v = _IQ24toF(sqrt_delt_iq24);

    /* -b + sqrt(b^2 - 4ac) */
    v_diff_iq24 = sqrt_delt_iq24 - v0_iq24;
    //temp = _IQ24toF(v_diff_iq24);

    t_iq24 = _IQ24div(v_diff_iq24 , a_iq24);
    //temp = _IQ24toF(t_iq24);

    t1_us_iq = _IQ1mpyIQX(t_iq24, 24, g_1000000_iq, 1);
    //temp = _IQ1toF(t1_us_iq);

    /* -b - sqrt(b^2 - 4ac) */
    v_diff_iq24 = (0 - sqrt_delt_iq24) - v0_iq24 ;
    //temp = _IQ24toF(v_diff_iq24);

    t_iq24 = _IQ24div(v_diff_iq24 , a_iq24);
    //temp = _IQ24toF(t_iq24);

    t2_us_iq = _IQ1mpyIQX(t_iq24, 24, g_1000000_iq, 1);
    //temp = _IQ1toF(t2_us_iq);

    /* ʱ����㷨 */
    if ((t1_us_iq < 0) && (t2_us_iq > 0)) {         /* ȡ������ */
        t1_us_iq = t2_us_iq;
    } else if ((t1_us_iq > 0) && (t2_us_iq < 0)) {
        t1_us_iq = t1_us_iq;
    } else if ((t1_us_iq > 0) && (t2_us_iq > 0)) {  /* ���������������������ôȡС���Ǹ� */
        t1_us_iq = (t1_us_iq > t2_us_iq) ? t2_us_iq : t1_us_iq;
    } else {
        return 1;                                               //���Ǹ��� ,�������
    }

    p_axis->t2_us = _IQ1toF(t1_us_iq);
    ret_iq = t1_us_iq - _IQ1(t1);
    ret_iq = ret_iq > 0 ? ret_iq : -ret_iq;                     //ȡ��ֵ
    ret_iq = _IQ1div(ret_iq , 100);
    ret =(Uint32)_IQ1int(ret_iq);                              /*ע�ⷵ�ص���һ��int��*/

//    if (cnt < 200) {
//        cnt_arr[cnt++] = ret;
//    } else {
//        cnt = 0;
//    }

//    GpioDataRegs.GPBDAT.bit.GPIOB1 = 0;

    return ret;
}


/**
 * \brief �������ٶȺͼ��ٶȣ��õ��ý׶ε��˶�����
 * \param[in]  p_axis����Ҫ�������
 * \param[in]  angle_speed_v0����ʼ���ٶ�
 * \param[in]  speed_given���ٶȸ���
 * \param[in]  a_given�������ļ��ٶ�ֵ��һ���Ǹ�����
 * \param[out] p_stage���õ��ĸý׶ε��˶�����
 *
 * \note angle_speed_v0��speed_given����һ�������෴������Ϊ0��������ͬʱΪ0
 */
Uint16 test_cnt_1 = 0,test_cnt_2 = 0;
#pragma CODE_SECTION(__param_calc, "ramfuncs");
static void __param_calc (const single_axis_conf_t* p_axis, float64 angle_speed_v0, float64 speed_given, int16 a_given, stage_var_t* p_stage)
{
#if 0
	_iq v_iq, v0_iq, a_iq, t_iq, x_iq;
	_iq result_iq;
	_iq speed_ratio_iq = _IQ(p_axis->speed_ratio);
	_iq step_angle_iq = _IQ(p_axis->step_angle);

	//����תIQ , Ŀ���ٶ���� ��2��/s
	//float temp;
	v_iq = _IQ((float32)speed_given * p_axis->speedvalue_to_speed);
//	temp = _IQtoF(v_iq);

	/* angle_speed_v0Ϊʵ�ʽ��ٶ� , ��λ ��/s */
//	v0_iq = _IQ((float32)angle_speed_v0);
//	temp = _IQtoF(v0_iq);
	p_stage->v0_1000 = angle_speed_v0 * 1000;  //����1000��,�õ����ٶ�
	v0_iq = _IQ(p_stage->v0_1000 * 0.001);	   //��ת��Ϊiq��ʽ,��ֹ������ʧ



	//IQ�볤����������� ,����IQ������������,��ʧ�˾������� v0 = 0.6004483185
//	p_stage->v0_1000 = _IQmpyI32int(v0_iq , 1000);

	/* �õ����ٶȷ��� ,Ŀ���ٶȼ�ȥ��ǰ�ٶ� , ����0��a>0 ,С��0��a<0 */
	p_stage->a_1000 = a_given * 5;
	a_iq = _IQ((float32) a_given * 0.005);

	if ((v_iq - v0_iq) < 0) {
		a_iq = 0 - a_iq;		/* ���ٶ�a = -a_given * 0.005 */
		p_stage->a_1000 = 0 - p_stage->a_1000;
	}

	//temp = _IQtoF(a_iq);

	/* �õ��˶�����  */
	if (v0_iq > 0 || v_iq > 0) {
		p_stage->mov_dir = 1;
	} else if ((v0_iq < 0) || (v_iq < 0)) {
		p_stage->mov_dir = -1;
	}

	/* �õ���ǰ�׶ε�ʱ��  , ��t = ��v / a ,��λs */
	/* IQ15���ֵ65535*/
	t_iq = _IQdiv((v_iq - v0_iq) , a_iq);
	p_stage->t  = _IQtoF(t_iq);
	/* �õ������Ƕȱ仯  */
	x_iq = _IQmpy(v0_iq, t_iq) + _IQmpy(_IQdiv(a_iq, _IQ(2)), _IQmpy(t_iq, t_iq));		/* x = v0*t + 1/2 * at^2 */
	//temp = _IQtoF(x_iq);

	/* �õ������ÿ1/4������Ϊ1����, ���ַ�������655������ */
	result_iq = _IQmpy(_IQdiv(speed_ratio_iq , step_angle_iq) , x_iq);
	// ��ȡ����ֵ��ת��Ϊ������ , ��ֵ�� p_stage->all_step_cnt
	p_stage->all_step_cnt = _IQtoF(_IQabs(result_iq));

	/* ΢������ = ���� * ϸ���� , �˴�Ϊ�����˸����� ,��Ҫ�󾫶� ,����ѡ��IQ1 , ���ص�ΪIQ��������*/
	p_stage->all_segment = p_stage->all_step_cnt * XIFEN_CNT;
	p_stage->loc_diff = p_stage->all_segment / 2;
	p_stage->elapsed_segment_cnt = 0;
#else
	float64 x_1000;
	int32 v_100;

	v_100 = (int16)nearbyintf((float32)speed_given * p_axis->speedvalue_to_speed * 100);

	/* ���ٹ��̼��ٶȷ�����V0�෴  */
	p_stage->v0_1000 = angle_speed_v0 * (float64)1000;	/* ��ʼ�ٶȵ�1000�� , ��Ϊʵ�ʽ��ٶ� , ��λ ��/s */

	/* �õ����ٶȵ�1000����û������λ�� ��/(s^2) */
//	if (v_100 * 10 - p_stage->v0_1000 < 0) {		/* �����ٶȺͳ��ٶȵĲ�*/
//		p_stage->a_1000 = -a_given * 5;				/* ʡ���� , Ӧ��дΪ -a_given * 0.005 * 1000*/
//	} else {
//		p_stage->a_1000 = a_given * 5;
//	}
	if (v_100 * 10 - p_stage->v0_1000 < 0) {		/* �����ٶȺͳ��ٶȵĲ�*/
		p_stage->a_1000 = 0 - (a_given * 5);		/* ʡ���� , Ӧ��дΪ -a_given * 0.005 * 1000*/
	} else {
		p_stage->a_1000 = a_given * 5;
	}
	/* �õ��˶����� */
	if ((p_stage->v0_1000 > 0) || (v_100 > 0)) {
		p_stage->mov_dir = 1;
	} else if ((p_stage->v0_1000 < 0) || (v_100 < 0)) {
		p_stage->mov_dir = -1;
	}
	else{
		test_cnt_1++;
	}

	/* �õ���ǰ�׶ε�ʱ�� */
	p_stage->t = (float64)(v_100 * 10 - p_stage->v0_1000) / p_stage->a_1000;

	/* �õ������Ƕȱ仯 , ������1000�� */
	x_1000 = (float64)p_stage->v0_1000 * p_stage->t + p_stage->a_1000 * p_stage->t * p_stage->t / 2 ;		/* x = v0*t + 1/2 * at^2 */

	/* �õ������ÿ1/4������Ϊ1���� */
	p_stage->all_step_cnt = fabs(x_1000 / 1000 * p_axis->speed_ratio / p_axis->step_angle);  				/* ���ִ����൱��ֻ������ */

	/* ΢������ */
	p_stage->loc_diff = nearbyintf(fabs(x_1000 / p_axis->loc_to_angle_factor / 1000));

	p_stage->all_segment = p_stage->all_step_cnt * XIFEN_CNT; /* ϸ��֮��Ĳ��� */
	p_stage->elapsed_segment_cnt = 0;
#endif
}



/**
 * \brief ����ռ�ձȺ͵���ϵ������pwm�ıȽϼĴ�����ֵ
 *
 * \param[in] p_axis����Ҫ���õ���
 * \param[in] p_pwm����
 */
#pragma CODE_SECTION(duty_to_cycle, "ramfuncs");
void duty_to_cycle(single_axis_conf_t* p_axis)
{
    uint8_t i = 0;
    _iq	  pwm_iq;
    _iq1  xifen_cnt_iq = _IQ1(PWM_PERIOD);
    _iq30 duty_iq , factor_iq ;				//��������Χ����0-1 ,����ѡ��iq30

    /* �����˶�ʱ������ֵ */
    factor_iq = _IQ30(p_axis->duty_factor_mov);
    /* PWM�趨��ʱ��Ƶ��Ϊ45M������PWM���ڹ̶�Ϊ50us����Ҫ2*PWM_PERIOD��ʱ�����趨���¼���ģʽ��������ֵΪPWM_PERIOD */
    for (i = 0; i < XIFEN_CNT + 1; i++) {
    	duty_iq = _IQ30(g_Axis_Conf.duty[i]);
    	duty_iq = _IQ30mpy(duty_iq , factor_iq); 								  //����С��0��ϵ���ȳ�
    	pwm_iq  = _IQmpyIQX(xifen_cnt_iq , 1 , duty_iq , 30);
        p_axis->pulse_tbprd_mov[i] = PWM_PERIOD - _IQint(pwm_iq);   				  //������������
    }

    /* ���㱣��ʱ������ֵ */
    factor_iq = _IQ30(p_axis->duty_factor_hold);
    /* PWM�趨��ʱ��Ƶ��Ϊ45M������PWM���ڹ̶�Ϊ50us����Ҫ2*PWM_PERIOD��ʱ�����趨���¼���ģʽ��������ֵΪPWM_PERIOD */
    for (i = 0; i < XIFEN_CNT + 1; i++) {
        duty_iq = _IQ30(g_Axis_Conf.duty[i]);
        duty_iq = _IQ30mpy(duty_iq , factor_iq);                                  //����С��0��ϵ���ȳ�
        pwm_iq  = _IQmpyIQX(xifen_cnt_iq , 1 , duty_iq , 30);
        p_axis->pulse_tbprd_hold[i] = PWM_PERIOD - _IQint(pwm_iq);                     //������������
    }
}

/**
 * \brief ���ݵ�ǰλ�ã�Ŀ��λ�ã����ٶȷ���õ�λ�ò�
 */
#pragma CODE_SECTION(__get_local_diff, "ramfuncs");
static int32 __get_local_diff (single_axis_conf_t* p_ab_axis, int32 loc_given, int32 speed_given)
{
    int32 loc_diff;

    loc_diff = loc_given - p_ab_axis->cur_local;
#if BOARD_NAME == BOARD_GMS
    //ɶҲ����
#elif BOARD_NAME == BOARD_DGM_2 /* ���滮���·�� */

    if ((speed_given > 0) && (loc_diff < 0)) {
        loc_diff += 1024000;
    } else if ((speed_given < 0) && (loc_diff > 0)) {
        loc_diff -= 1024000;
    }
#elif BOARD_NAME == BOARD_DGM_4 /* ��Ҫ�滮���·�� */
    if ((!strncmp(p_ab_axis->name, "PYA", 3)) || (!strncmp(p_ab_axis->name, "NYA", 3))) {
        if (loc_diff > 512000) {
            loc_diff -= 1024000;
        } else if (loc_diff < -512000) {
            loc_diff += 1024000;
        }
    } else if ((!strncmp(p_ab_axis->name, "PYB", 3)) || (!strncmp(p_ab_axis->name, "NYB", 3))) {
        loc_diff = loc_given - p_ab_axis->cur_local;
    }
#endif
    return loc_diff;
}


/**
 * \brief ִ��AB���˶�����ָ��
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
 */
#pragma CODE_SECTION(ab_axis_mov_ctl_cmd_run, "ramfuncs");

int16 ab_axis_mov_ctl_cmd_run(single_axis_conf_t* p_ab_axis)
{
    float64  temp = 0;
    float64  v1_1000, x1_1000, x2_1000, t1, t2;

    single_axis_conf_t axis_temp;
    int16   speed_given = 0;   			/* �ٶȸ�������λģʽ�����Ǹ�λ�ٶȣ�����ģʽ�����ǲ����ٶ� */
    int32   loc_given = 0;     			/* λ�ø�������λģʽ�����Ǹ�λλ�ã�����ģʽ�����ǲ���λ�� */
    int32 	v0_1000;					/* ��Ϊ�м���������ö����� */
    int32   loc_diff = 0;				/* Ŀ��λ���뵱ǰλ��֮�� */
    int32   loc_acc1, loc_acc2 = 0;		/* �������ٹ�����Ҫ�ƶ���λ�� */
    int32   angle_diff_1000 = 0;

//    _iq30   time_factor_iq ;   			//��ǰpwm�ж�ʱ�� ,��λs
    _iq23	angle_speed_iq;				//��ǰ�������ٶ�,��λ��/s
    _iq23   step_angle_iq ;             //����Ჽ���
    _iq23   step_t_iq;					//΢��ʱ�� ,��λ s
    _iq16	pulse_cnt_iq ;				//��ǰ΢����,50uspwm���ĸ��� ,��������dir

//    GpioDataRegs.GPBDAT.bit.GPIOB2 = 1;

    /* ����������ݽ��л��� */
    memcpy(&axis_temp, p_ab_axis, sizeof(single_axis_conf_t));

    /* ����Ҫ����������ֱ������ */
    if (axis_temp.new_task_flag == 0) {
        return 0;
    }
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
    if (axis_temp.work_mode_set == WORK_MODE_STANDBY) {
    	return 0;
    }
#endif


	/* �������ж� */
		if ((axis_temp.work_mode_old != WORK_MODE_KEEP) || (axis_temp.work_mode_set != WORK_MODE_KEEP)) {	/* û�б���ģʽת����ģʽ */
//			axis_temp.cur_local = cur_local_modify_A(p_axis);
			/* ��α����Ƿ����ж����棬����ʱ�������⿴����ǰ�ٶȣ�������δ�������ʱ��̫�������20us�������Խ����ó� */
            //if (axis_temp.work_mode_old != WORK_MODE_KEEP) {    /* ��ģʽ��Ϊ����ģʽ��˵���Ǵӱ��ģʽ�ֶ���ת�����ģ�Ҫ������ٶ� */ /* ������ôд������ֶ����ͱ���ģʽ���ڵ����ûͣ������ʱ���ַ����˶�ģʽ�������ٶȻ�ͻȻ���͵�0 */
            if (g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) {    /* �жϵ���Ƿ������˶���Ҫ������ٶ� */
				/* �������ϸ��+���ٱ�֮����ٶ� , ��ʵ��������ٶ� */
#if 0
				step_angle_iq =_IQ23(STEP_ANGLE);
				time_factor_iq = _IQ30(PWM_TICK * 0.000001);				   		  //����ǿ��ת�� ,�������Ϊ0
				pulse_cnt_iq = _IQ1mpy(_IQ1(axis_temp.stage[axis_temp.cur_stage].mov_dir) , _IQ1(axis_temp.next_segment_pulse_cnt));
				angle_speed_iq = _IQ23mpyIQX(pulse_cnt_iq , 1 , time_factor_iq , 30);  //��ǰ΢����ʱ�� ,��λs,��256 ,����0.000000119
				angle_speed_iq = _IQ23div(step_angle_iq , angle_speed_iq);			   //��ǰ�������ٶ� ,��λ��/s,��256 ,����0.000000119
                axis_temp.cur_angle_speed = _IQ23toF(angle_speed_iq);
#else
                if (axis_temp.next_segment_pulse_cnt != 0) {
                    if ( axis_temp.stage[ axis_temp.cur_stage].a_1000 != 0) {
 //                       axis_temp.cur_angle_speed = (float64)1000000 * axis_temp.step_angle / ((float64)XIFEN_CNT * axis_temp.next_segment_pulse_cnt * 50 * axis_temp.speed_ratio) * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                        axis_temp.cur_angle_speed = real_v * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                    } else {
                        axis_temp.cur_angle_speed = (float64)1000000 * axis_temp.step_angle / ((float64)XIFEN_CNT * ((float64)axis_temp.uniform_segment_pulse_cnt_zheng + (float64)axis_temp.uniform_segment_pulse_cnt_xiao_1000 / 1000) * 50 * axis_temp.speed_ratio) * axis_temp.stage[axis_temp.cur_stage].mov_dir;
                    }
                } else {
                    axis_temp.cur_angle_speed = 0;
                }
#endif
			} else {
				axis_temp.cur_angle_speed = 0;
			}

			axis_temp.reciprocat_mode_enable = 0;

			/* ȷ��Ŀ��λ�á��ƶ��ٶȡ��˶����� */
			switch (axis_temp.work_mode_set) {
				case WORK_MODE_TRACE:   /* ����ģʽ��չ��ģʽ������ģʽ�� */
				    speed_given = axis_temp.speed_given;
				    if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
#if BOARD_NAME == BOARD_GMS /* ����ģʽ */
                        axis_temp.reciprocat_mode_enable = 1;

                        if (speed_given > 0) {
                            loc_given = axis_temp.soft_lim_p - 222;
                        } else if (speed_given < 0) {
                            loc_given = axis_temp.soft_lim_n + 222;
                        }

                        /* �պ��ڸ���λ�� */
                        if (loc_given == axis_temp.cur_local) {
                            if (loc_given == axis_temp.soft_lim_p - 222) {    /* �պ�������λ */
                                loc_given = axis_temp.soft_lim_n;
                                if (speed_given > 0) {
                                    speed_given = 0 - speed_given;
                                }
                            } else if (loc_given == axis_temp.soft_lim_p + 222) {    /* �պ��ڸ���λ */
                                loc_given = axis_temp.soft_lim_p;
                                if (speed_given < 0) {
                                    speed_given = 0 - speed_given;
                                }
                            }
                        }

                        axis_temp.work_mode_set = WORK_MODE_CAPTURE;
#elif BOARD_NAME == BOARD_DGM_4 /* չ��ģʽ */
                        loc_given = axis_temp.reset_local;
                        speed_given = 1777;     /* 1.6��/s */
                        axis_temp.max_acc = 100; /* ���ٶ�0.5��/s^2 */

                        if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                            ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                            speed_given = 0 - speed_given;
                        }

                        axis_temp.speed_given = speed_given;
                        axis_temp.loc_given   = 0;
#endif
                    }
					break;

				case WORK_MODE_CAPTURE:	/* ����ģʽ�������ٶȸ�����λ�ø��� */
					speed_given = axis_temp.speed_given;
					loc_given = axis_temp.loc_given;
					if (loc_given == axis_temp.cur_local) { /* λ�ø������ڵ�ǰλ��ֱ�ӷ���0 */
					    return 0;
					}
#if (BOARD_NAME == BOARD_GMS)
					if (((loc_given < axis_temp.cur_local) && (speed_given > 0)) ||
					    ((loc_given > axis_temp.cur_local) && (speed_given < 0)))  {
						speed_given = -speed_given;
					}

#elif (BOARD_NAME == BOARD_DGM_4)    /* Ѱ�����·�� */
                    if (loc_given > axis_temp.cur_local) {
                        if (((loc_given - axis_temp.cur_local >  512000) && (speed_given > 0)) ||
                            ((loc_given - axis_temp.cur_local <= 512000) && (speed_given < 0))) {
                            speed_given = 0 - speed_given;
                        }
                    } else if (loc_given < axis_temp.cur_local) {
                        if (((axis_temp.cur_local - loc_given >  512000) && (speed_given < 0)) ||
                            ((axis_temp.cur_local - loc_given <= 512000) && (speed_given > 0))) {
                            speed_given = 0 - speed_given;
                        }
                    } else if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
	                    if (((loc_given < axis_temp.cur_local) && (speed_given > 0)) ||
	                        ((loc_given > axis_temp.cur_local) && (speed_given < 0)))  {
	                        speed_given = 0 - speed_given;
	                    }
					}
#endif
				break;

				case WORK_MODE_RESET:	/* ��λģʽ�����ǻ�0ģʽ */
                    speed_given = axis_temp.reset_speed_given;
                    loc_given = axis_temp.reset_local;
                    if (loc_given == axis_temp.cur_local) { /* λ�ø������ڵ�ǰλ��ֱ�ӷ���0 */
                        return 0;
                    }

#if BOARD_NAME == BOARD_GMS
                    if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                        ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                        speed_given = -speed_given;
                    }
                    g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 1;    /* ���ڻ��� */
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2)
                    if ((!strncmp(axis_temp.name, "PYA", 3)) || (!strncmp(axis_temp.name, "NYA", 3))) {
                        if (loc_given > axis_temp.cur_local) {
                            if (((loc_given - axis_temp.cur_local >  512000) && (speed_given > 0)) ||
                                ((loc_given - axis_temp.cur_local <= 512000) && (speed_given < 0))) {
                                speed_given = 0 - speed_given;
                            }
                        } else if (loc_given < axis_temp.cur_local) {
                            if (((axis_temp.cur_local - loc_given >  512000) && (speed_given < 0)) ||
                                ((axis_temp.cur_local - loc_given <= 512000) && (speed_given > 0))) {
                                speed_given = 0 - speed_given;
                            }
                        }
                    } else if ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))) {
                        if (((axis_temp.reset_local < axis_temp.cur_local) && (speed_given > 0)) ||
                            ((axis_temp.reset_local > axis_temp.cur_local) && (speed_given < 0))) {
                            speed_given = -speed_given;
                        }
                    }
                    if (!strncmp(axis_temp.name, "PYA", 3)){
                        g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 1;    /* ���ڻ��� */
                    }else if(!strncmp(axis_temp.name, "NYA", 3)){
                        g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 1;    /* ���ڻ��� */
                    }
#if BOARD_NAME == BOARD_DGM_4
                    else if(!strncmp(axis_temp.name, "PYB", 3)){
                        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 1;    /* ���ڻ��� */
                    }else if(!strncmp(axis_temp.name, "NYB", 3)){
                        g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 1;    /* ���ڻ��� */
                    }
#endif
#endif

				break;

				case WORK_MODE_KEEP:	/* ����ģʽ�����ı��˶����򣬻�������ֱ��ֹͣ */
					/* ���ù����ٶȸ�����λ�ø��� */
				break;
			}

			//axis_temp.cur_angle_speed = -0.05;  //������

			/* ���ת�뱣��ģʽ ,Ŀ���ٶ�����Ϊ0 , ֻ��һ���׶�*/
			if (axis_temp.work_mode_set == WORK_MODE_KEEP) {
				__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0 , axis_temp.max_acc, &axis_temp.stage[0]);
				axis_temp.stage_cnt = 1;
			} else {
				/* �õ�����ʱ���������ڣ�������Դﵽ���ֵ�� , �˴��п����������ٶȲ���׼ȷ */
				angle_speed_iq = _IQ23(speed_given * axis_temp.speedvalue_to_speed);   //#5035681 �ٶȸ���ֵ����ϵ�� = Ŀ���ٶ�(��λ ��/s) ,��256 ,����0.000000119
				step_angle_iq  = axis_temp.step_angle_out_iq23;							//#4718 ����Ჽ���
				step_t_iq      = _IQ23div(step_angle_iq , angle_speed_iq);			 //#7859�������-1�򲻶ԡ�  ���ٵ�΢��ʱ�� ,��λ��/s
				pulse_cnt_iq   = _IQ16mpyIQX(step_t_iq , 23 , _IQ16(20000) , 16);       //#37�������-1�򲻶ԡ� ��λ�� ,t*1000000/50 = t/20000	 ,������ ,���50us��������ҲҪ�޸�

				axis_temp.uniform_segment_pulse_cnt_zheng = _IQ16int(_IQ16abs(pulse_cnt_iq));  //�����ٶ������� ,������ȡ����ֵ ,��ȡ�� ,����ȡ����ʽ�����С�� ,��Ȼ����������ٶ�.
				axis_temp.uniform_segment_pulse_cnt_xiao_1000= _IQ16int(_IQ16mpyIQX(_IQ16abs(pulse_cnt_iq) - _IQ16(_IQ16int(_IQ16abs(pulse_cnt_iq))), 16, _IQ16(1000), 16));

				/* �滮�׶�0 */
				axis_temp.stage_cnt = 0;

				loc_diff = __get_local_diff(&axis_temp, loc_given, speed_given);

				/* �����ٶȺͳ��ٶȷ���ͬ����Ҫ�ȼ��ٵ�0���ٷ��� */
				if (((axis_temp.cur_angle_speed < 0) && (speed_given > 0)) || ((axis_temp.cur_angle_speed > 0) && (speed_given < 0))) {	/* ����ͬ */
					__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);
					__param_calc(&axis_temp, 0, speed_given, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);
					/* �����ٶȷ���Ϊ��ʱ ,���ڸ���λ���ڵ�ǰλ�õĺ��� , ������Ҫ�ȼ��� , �ٷ������ ,���ٵĹ�����ǰ������һ�ξ��� */
					if (loc_diff < 0) {
						loc_diff -= axis_temp.stage[0].loc_diff;
					} else {
					/* �����ٶȷ���Ϊ��ʱ ,���ڸ���λ���ڵ�ǰλ�õĺ��� , ������Ҫ�ȼ��� , �ٷ������ ,���ٵĹ�����ǰ������һ�ξ���*/
						loc_diff += axis_temp.stage[0].loc_diff;
					}
					/* �����ʱΪ��λ�򲶻�ģʽ������DGM_4��B��չ��ģʽ*/
#if BOARD_NAME == BOARD_GMS || BOARD_NAME == BOARD_DGM_2
					if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
					    (axis_temp.work_mode_set == WORK_MODE_RESET)) {
#elif BOARD_NAME == BOARD_DGM_4
					if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)   ||
                        ((axis_temp.work_mode_set == WORK_MODE_LAUNCH) && ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))))) {
#endif
						if (((loc_diff > 0) && (loc_diff <= 2 * axis_temp.stage[1].loc_diff)) ||
							((loc_diff < 0) && (-loc_diff <= 2 * axis_temp.stage[1].loc_diff)))	{   /* û�����ٽ׶� */
							axis_temp.stage[1].a_1000 = axis_temp.stage[0].a_1000;
							axis_temp.stage[2].a_1000 = -axis_temp.stage[1].a_1000;
							if (loc_diff > 0) {
								axis_temp.stage[1].loc_diff = loc_diff / 2;
								axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
							} else {
								axis_temp.stage[1].loc_diff = -loc_diff / 2;
								axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
							}
							axis_temp.stage[1].mov_dir = -axis_temp.stage[0].mov_dir;
							axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
							axis_temp.stage[1].t = sqrt(((float64)axis_temp.stage[1].loc_diff * axis_temp.loc_to_angle_factor) * 2 / ((float64)axis_temp.max_acc * 0.005));
							axis_temp.stage[2].t = axis_temp.stage[1].t;
							axis_temp.stage[1].v0_1000 = 0;
							axis_temp.stage[2].v0_1000 = axis_temp.stage[1].a_1000 * axis_temp.stage[1].t;

                            axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;  //�׶�2������
                            axis_temp.stage[1].all_segment  = axis_temp.stage[1].loc_diff * 2;		   				 //�׶�2΢����
                            axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT * 2;  //�׶�3������
                            axis_temp.stage[2].all_segment  = axis_temp.stage[2].loc_diff * 2;	   				 	 //�׶�3΢����
                            axis_temp.stage[1].elapsed_segment_cnt = 0;						   				 //�׶�1��807�е��ú����Ѿ�������
                            axis_temp.stage[2].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 3;
						} else {	/* �����ٽ׶� */
							axis_temp.stage[3].a_1000 = (-1) * axis_temp.stage[1].a_1000;
							axis_temp.stage[3].loc_diff = axis_temp.stage[1].loc_diff;
							axis_temp.stage[3].mov_dir  = axis_temp.stage[1].mov_dir;
							axis_temp.stage[3].t = axis_temp.stage[1].t;
							axis_temp.stage[3].v0_1000  =  (int16)nearbyintf((float)speed_given * axis_temp.speedvalue_to_speed * 1000);  //Ŀ���ٶ�����1000
                            axis_temp.stage[3].all_step_cnt = axis_temp.stage[3].loc_diff / (float32)XIFEN_CNT * 2;
                            axis_temp.stage[3].all_segment  = axis_temp.stage[3].loc_diff * 2;

							axis_temp.stage[2].a_1000 = 0;
							if (loc_diff < 0) {
								loc_diff = 0 - loc_diff;
							}
							axis_temp.stage[2].loc_diff = loc_diff - (axis_temp.stage[1].loc_diff + axis_temp.stage[3].loc_diff);
							axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
							axis_temp.stage[2].v0_1000 = axis_temp.stage[3].v0_1000;
							//ʱ���������????
							axis_temp.stage[2].t = fabs(((float64)axis_temp.stage[2].loc_diff) / speed_given);
                            axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT *2;
                            axis_temp.stage[2].all_segment = axis_temp.stage[2].loc_diff * 2;

                            axis_temp.stage[2].elapsed_segment_cnt = 0;
                            axis_temp.stage[3].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 4;
						}
					}
				} else {	/* �����ٶȺͳ��ٶȷ�����ͬ */
					__param_calc(&axis_temp, axis_temp.cur_angle_speed, speed_given, axis_temp.max_acc, &axis_temp.stage[axis_temp.stage_cnt++]);

					/* ����ģʽ����λģʽ���Լ�B���չ��ģ�� */
#if BOARD_NAME == BOARD_GMS || BOARD_NAME == BOARD_DGM_2
                    if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)) {
#elif BOARD_NAME == BOARD_DGM_4
                    if ((axis_temp.work_mode_set == WORK_MODE_CAPTURE) ||
                        (axis_temp.work_mode_set == WORK_MODE_RESET)   ||
                        ((axis_temp.work_mode_set == WORK_MODE_LAUNCH) && ((!strncmp(axis_temp.name, "PYB", 3)) || (!strncmp(axis_temp.name, "NYB", 3))))) {
#endif
						__param_calc(&axis_temp, (float64)speed_given * axis_temp.speedvalue_to_speed, 0, axis_temp.max_acc, &axis_temp.stage[2]);

						loc_acc1 = axis_temp.stage[0].loc_diff * axis_temp.stage[0].mov_dir;
						loc_acc2 = axis_temp.stage[2].loc_diff * axis_temp.stage[2].mov_dir;

						if (((loc_diff <= 0) && (loc_acc1 + loc_acc2 <= 0) && (loc_diff >= loc_acc1 + loc_acc2)) ||
							((loc_diff > 0) && (loc_acc1 + loc_acc2 > 0) && (loc_diff < loc_acc1 + loc_acc2))) {		/* �ٶȲ����ӵ������ٶ� ,û�����ٽ׶� */
								/* �����������2�֣���һ���������������ٶȼ��ٵ�0��ȻҪ�����趨λ�ã���ôֱ���������ٶȼ��ٵ�0�ٷ��������ڶ����������������ٶȼ��ٵ�0��·�̹�����ô��ʱ�����ȼ��ٵ�V���ټ��ٵ�0�� */

								v0_1000 = axis_temp.cur_angle_speed * 1000;			/* ��ʼ�ٶȵ�1000�� ��/s */
								angle_diff_1000 = (float64)loc_diff * axis_temp.loc_to_angle_factor * 1000;

								t1 = fabs((float64)v0_1000 / (axis_temp.max_acc * 5));
								temp = 0.5 * (float64)axis_temp.max_acc * 0.005 * t1 * t1;
								loc_acc1 = temp / axis_temp.loc_to_angle_factor;

								/* �����ȼ��٣�ֱ���ٶȵ�0��Ȼ�󷵻� */
								if (((loc_diff <= 0) && (loc_diff > -loc_acc1)) ||
									((loc_diff > 0) && (loc_diff < loc_acc1))) {		/* �������ٻ�Ҫ���� */
									__param_calc(&axis_temp, axis_temp.cur_angle_speed, 0, axis_temp.max_acc, &axis_temp.stage[0]);
									loc_diff = loc_acc1 - loc_diff;
									if (loc_diff < 0) {
										loc_diff = -loc_diff;
									}
									axis_temp.stage[1].a_1000 = axis_temp.stage[0].a_1000;
									axis_temp.stage[2].a_1000 = -axis_temp.stage[0].a_1000;
									axis_temp.stage[1].loc_diff = loc_diff / 2;
									axis_temp.stage[2].loc_diff = axis_temp.stage[1].loc_diff + loc_diff % 2;
									axis_temp.stage[1].mov_dir = -axis_temp.stage[0].mov_dir;
									axis_temp.stage[2].mov_dir = axis_temp.stage[1].mov_dir;
									axis_temp.stage[1].t = sqrt(((float64)axis_temp.stage[1].loc_diff * axis_temp.loc_to_angle_factor) * 2 / ((float64)axis_temp.max_acc * 0.005));
									axis_temp.stage[2].t = axis_temp.stage[1].t;
									axis_temp.stage[1].v0_1000 = 0;
									axis_temp.stage[2].v0_1000 = axis_temp.stage[1].a_1000 * axis_temp.stage[1].t;

                                    axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;
                                    axis_temp.stage[2].all_step_cnt = axis_temp.stage[2].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[2].all_segment = axis_temp.stage[2].loc_diff * 2;

                                    axis_temp.stage[1].elapsed_segment_cnt = 0;
                                    axis_temp.stage[2].elapsed_segment_cnt = 0;

									axis_temp.stage_cnt = 3;
								} else {	/* �����ȼ��� */
									/* �����ʼ�ٶ�Ϊv0���м�����ٶ�Ϊv1������·��Ϊx1������·��Ϊx2������ʱ��Ϊt1������ʱ��Ϊt2�����ٹ��̼��ٶ�Ϊa�����ٹ��̼��ٶ�Ϊ-a���õ����·��� */
									/* x1=v0t1+(1/2)at1^2	*/
									/* x2=v1t1-(1/2)at2^2	*/
									/* a=(v1-v0)/t1 */
									/* a=v1/t2 */
									/* x1+x2=angle_diff */
									/* ����v0��a��angle_diff��֪����x1,x2,t1,t2,v1 */
									/* v1^2 = (angle_diff *2a+v0^2)/2 */

									v1_1000 = sqrt(fabs((angle_diff_1000 * 2 * axis_temp.stage[0].a_1000 + axis_temp.stage[0].v0_1000 * axis_temp.stage[0].v0_1000) / 2));
									if (speed_given < 0) {
										v1_1000 = -v1_1000;
									}

									t1 = (float64)(v1_1000 - v0_1000) / axis_temp.stage[0].a_1000;
									if(t1 < 0 ) {
										test_cnt_2++;
									}

									t2 = v1_1000 / axis_temp.stage[0].a_1000;
									x1_1000 = v0_1000 * t1 + axis_temp.stage[0].a_1000 * t1 * t1 / 2;
									x2_1000 = v1_1000 * t2 - axis_temp.stage[0].a_1000 * t2 * t2 / 2;

									axis_temp.stage[0].t = t1;
									axis_temp.stage[1].t = t2;
		  							axis_temp.stage[0].loc_diff = nearbyintf(fabs(x1_1000 / axis_temp.loc_to_angle_factor / 1000));
		  							axis_temp.stage[1].loc_diff = nearbyintf(fabs(x2_1000 / axis_temp.loc_to_angle_factor / 1000));

									/* ���ϼ�����ȷ��λ��0��� */
		  							if (loc_diff < 0) {
		  								loc_diff = -loc_diff;
		  							}
									while (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff < loc_diff) {
										axis_temp.stage[0].loc_diff++;
										if (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff < loc_diff) {
											axis_temp.stage[1].loc_diff++;
										}
									}
									while (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff > loc_diff) {
										axis_temp.stage[0].loc_diff--;
										if (axis_temp.stage[0].loc_diff + axis_temp.stage[1].loc_diff > loc_diff) {
											axis_temp.stage[1].loc_diff--;
										}
									}

                                    axis_temp.stage[0].all_step_cnt = axis_temp.stage[0].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[0].all_segment = axis_temp.stage[0].loc_diff * 2;
                                    axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
                                    axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;

		                            /* �ڶ��� */
									axis_temp.stage[1].v0_1000 = v1_1000;
									axis_temp.stage[1].a_1000 = (-1) * axis_temp.stage[0].a_1000;
									axis_temp.stage[1].mov_dir = axis_temp.stage[0].mov_dir;

		                            axis_temp.stage[0].elapsed_segment_cnt = 0;
		                            axis_temp.stage[1].elapsed_segment_cnt = 0;

									axis_temp.stage_cnt = 2;
								}
						} else {	/* �����ٽ׶� */
							if (loc_diff - (loc_acc1 + loc_acc2) > 0) {
								axis_temp.stage[1].loc_diff = loc_diff - (loc_acc1 + loc_acc2);
							} else {
								axis_temp.stage[1].loc_diff = (loc_acc1 + loc_acc2) - loc_diff;
							}
							axis_temp.stage[1].all_step_cnt = axis_temp.stage[1].loc_diff / (float32)XIFEN_CNT * 2;
							axis_temp.stage[1].all_segment = axis_temp.stage[1].loc_diff * 2;

							axis_temp.stage[1].v0_1000 = axis_temp.stage[2].v0_1000;
							axis_temp.stage[1].a_1000 = 0;

							axis_temp.stage[1].mov_dir = axis_temp.stage[0].mov_dir;
							axis_temp.stage[1].t = fabs(((float64)axis_temp.stage[1].loc_diff) / speed_given);
                            axis_temp.stage[1].elapsed_segment_cnt = 0;

							axis_temp.stage_cnt = 3;
						}
					}
				}
#if BOARD_NAME == BOARD_GMS
				if ((axis_temp.work_mode_set == WORK_MODE_TRACE)) {    /* ����ģʽ */
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
				if ((axis_temp.work_mode_set == WORK_MODE_TRACE) && ((!strncmp(axis_temp.name, "PYA", 3)) || (!strncmp(axis_temp.name, "NYA", 3)))) {    /* ����ģʽ */
#endif
					axis_temp.stage[axis_temp.stage_cnt].v0_1000 = (int16)nearbyintf((float32)speed_given * axis_temp.speedvalue_to_speed * 1000);
					axis_temp.stage[axis_temp.stage_cnt].a_1000 = 0;
					axis_temp.stage[axis_temp.stage_cnt].mov_dir = axis_temp.stage[axis_temp.stage_cnt-1].mov_dir;
					axis_temp.stage[axis_temp.stage_cnt].loc_diff = (~0);
					axis_temp.stage[axis_temp.stage_cnt].all_segment = (~0);
                    axis_temp.stage[axis_temp.stage_cnt].elapsed_segment_cnt = 0;
					axis_temp.stage[axis_temp.stage_cnt++].t = 0;
				}
			}

			axis_temp.cur_stage = 0;
//			axis_temp.rest_puls_cnt = axis_temp.all_puls_cnt = axis_temp.stage[0].loc_diff;

			if (axis_temp.is_task_running == false) {
				g_Axis_Conf.task_cnt++;
			}


			axis_temp.work_mode_old = axis_temp.work_mode_set;

            axis_temp.tail_flag = 0;										/* ���������־����*/
            //axis_temp.cur_xifen_num = 0;							/* ϸ�������� */
            axis_temp.reverse_flag = 0;									/* ��Ҫ�����־����*/
            axis_temp.stage[0].elapsed_segment_cnt = 0;
            axis_temp.stage[1].elapsed_segment_cnt = 0;
            axis_temp.stage[2].elapsed_segment_cnt = 0;
            axis_temp.stage[3].elapsed_segment_cnt = 0;
            axis_temp.stage[4].elapsed_segment_cnt = 0;

			/* ��һ�ε�ʱ����Ҫ������� */
			axis_temp.next_segment_pulse_cnt = get_next_segment_pulse_cnt(&axis_temp);

    	    axis_temp.is_task_running = true;									/* ����ٿ����� */

    	    axis_temp.cur_xifen_num         = p_ab_axis->cur_xifen_num;
    	    axis_temp.cur_xifen_pulse_num   = p_ab_axis->cur_xifen_pulse_num;
    	    axis_temp.period_cnt_1_4        = p_ab_axis->period_cnt_1_4;
    	    axis_temp.cur_local             = p_ab_axis->cur_local;

    	    /* Ϊ�˷�ֹ�˶������г���ֱ�� */
    	    if (axis_temp.cur_xifen_pulse_num >= axis_temp.next_segment_pulse_cnt) {
    	        axis_temp.cur_xifen_pulse_num = 0;
    	    }

    	    memcpy(p_ab_axis, &axis_temp, sizeof(single_axis_conf_t));
		}

//	GpioDataRegs.GPBDAT.bit.GPIOB2 = 0;

	return 0;
}


/**
 * \brief  A��cur_localֵ����
 *
 * \note A����Ҫ������B�᲻��Ҫ����
 * \note ��ÿ������ʼ�����ʱ����
 */
#if (BOARD_NAME == BOARD_CX20) ||  (BOARD_NAME == BOARD_DGM_2) ||  (BOARD_NAME == BOARD_DGM_4)	/* ��ý��ֻ��B�ᣬ����Ҫ���� */
int32 cur_local_modify_A (single_axis_conf_t* p_axis)
{
	int32 cur_local;

	cur_local = p_axis->cur_local % p_axis->max_loc;

	if (cur_local < 0) {
		cur_local += p_axis->max_loc;
	}

	return cur_local;
}
#endif







