#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File
#include "xintf.h"
#include "sci.h"
#include "cpu_timer0.h"
#include "cpu_timer1.h"
#include "data_handler.h"
#include "tele_dat.h"
#include "cmd.h"
#include "pps.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "pwm.h"
#include "GL_CJW_HE_5701.h"
#include "mot_cur_conf_cmd.h"
#include "boot_args.h"

#if BOARD_NAME == BOARD_GMS
#include "tlv2548.h"
#include "cycle_hall.h"
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_CX20)
#include "tlc2543.h"
#endif

#if BOARD_NAME == BOARD_GMS
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pyb);
        }
    }
}
#elif BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_4
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pyb);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nyb);
        }
    }
}
#elif BOARD_NAME == BOARD_DGM_2
void AB_Axis_Mov_Task(void)
{
    if (g_Axis_Conf.new_task_req == true) {
        if (g_pps - g_Axis_Conf.cmd_pps >= g_Axis_Conf.run_delay) {
            g_Axis_Conf.new_task_req = false;
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_pya);
            ab_axis_mov_ctl_cmd_run(g_Axis_Conf.p_nya);
        }
    }
}
#endif

/**
 * \brief ͨ�Žӿڸ�λ����
 */
void Commu_Interf_Reset_Task (void)
{
    if (ElapsedTick(g_RS422_DATA_Tick) >= TICK_5S) {
        g_RS422_DATA_Tick = GetSysTick();

#if BOARD_NAME == BOARD_GMS
        if (g_Axis_Conf.p_pyb->work_mode_old == WORK_MODE_RESET) {  /* ����ģʽ */
            g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 3;            /* �����쳣 */
        }
#endif

        /* ���³�ʼ������ */
        DriverSciInit(&SciaRegs, 115200, 1, 0, 7, 0, SCI_Rx_isr, SCI_TxFifo_isr);  //������115200����У�飬8���ַ����ȣ�1��ֹͣλ��

        g_Tele_Dat.comm_rest_cnt++;

        g_RS422_relink_times++;
        if (g_RS422_relink_times == 5) {
            g_RS422_relink_times = 0;
            *gp_boot_arg_serial_fault_reset = 1; /* �����쳣��λ */
            while (1);  /* ���ͨ�Ÿ�λ5�δ��ڻ�δ�ָ���ֱ�ӿ������ÿ��Ź���λ */
        }
    }
}

#if BOARD_NAME == BOARD_GMS
/**
 * \brief �����λ����
 */
extern uint8_t  g_last_work_mode_set;           /* �ϴι���ģʽ */
extern int32    g_last_loc_given;               /* �ϴ�λ�ø��� */
extern int16    g_last_speed_given;             /* �ϴ��ٶȸ��� */

//extern Uint16 g_force_move_cmd_cnts;

void Soft_Lim_Task (void)
{
    float32 mot_angle = 0;
//    static uint8_t max_acc_temp = 2;

    /* �ж��Ƿ񵽴���λλ�� */
    mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;

    /* ����Ѿ�ִ����ǿ��ֹͣ�������Ѿ�����ֹͣ״̬���������ٶȸ�д��ȥ */
    if ((g_force_hold_flag == 1) && ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 0) ||
									 ((g_Axis_Conf.p_pyb->cur_local >= g_Axis_Conf.p_pyb->soft_lim_n + 444) &&
									  (g_Axis_Conf.p_pyb->cur_local <= g_Axis_Conf.p_pyb->soft_lim_p - 444)))) {	/* �����⵽��ǰλ������λ��Χ��0.2�����ڣ����˳�ǿ��ģʽ */
//    	g_Axis_Conf.p_pyb->max_acc = max_acc_temp;
    	g_force_hold_flag = 0;

        g_last_work_mode_set = 0;           /* �ϴι���ģʽ */
        g_last_loc_given = 0;               /* �ϴ�λ�ø��� */
        g_last_speed_given = 0;             /* �ϴ��ٶȸ��� */
    }

	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
    	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    	if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
        	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        	if (mot_angle <= g_Axis_Conf.p_pyb->soft_lim_n * g_Axis_Conf.p_pyb->loc_to_angle_factor) {	/* ��3�Σ���ֹ���� */

				g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 1;

	//			if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) && (g_force_hold_flag == 0) && (g_force_move_cmd_cnts < 5)) {		/* �����������˶�״̬����������λλ�ã���ǿ��ֹͣ */
				if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == -1) && (g_force_hold_flag == 0)) {		/* �����������˶�״̬����������λλ�ã���ǿ��ֹͣ */
					g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
					g_Axis_Conf.p_pyb->loc_given     = 0;
					g_Axis_Conf.p_pyb->speed_given   = 1333;
					g_Axis_Conf.run_delay     		 = 0;
					g_Axis_Conf.new_task_req 		 = true;

					g_Axis_Conf.p_pyb->new_task_flag = true;	/* ��������У���������������ָͬ��֮�󣨵��¸ñ�־Ϊfalse�����򲻻��߻���λ��Χ֮�ڣ�����Ҳ����Ӧ����ָ�� */

	//				max_acc_temp = g_Axis_Conf.p_pyb->max_acc;	/* ������ٶ� */
	//				g_Axis_Conf.p_pyb->max_acc = 200;

					g_force_hold_flag = 1;
				}
        	} else {
        		g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
        	}
    	} else {
    		g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
    	}
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.lim_n = 0;
    }

	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
    if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
    	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
        if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {
        	mot_angle = g_Axis_Conf.p_pyb->cur_local * g_Axis_Conf.p_pyb->loc_to_angle_factor;
            if (mot_angle >= g_Axis_Conf.p_pyb->soft_lim_p * g_Axis_Conf.p_pyb->loc_to_angle_factor) {	/* ��3�Σ���ֹ���� */

				g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 1;

				//if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) && (g_force_hold_flag == 0) && (g_force_move_cmd_cnts < 5)) {		/* �����������˶�״̬����������λλ�ã���ǿ��ֹͣ */
				if ((g_Tele_Dat.p_yb_axis_stat.bit.mov_stat == 1) && (g_Axis_Conf.p_pyb->stage[g_Axis_Conf.p_pyb->cur_stage].mov_dir == 1) && (g_force_hold_flag == 0)) {		/* �����������˶�״̬����������λλ�ã���ǿ��ֹͣ */
					g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;
					g_Axis_Conf.p_pyb->loc_given     = 0;
					g_Axis_Conf.p_pyb->speed_given   = 0 - 1333;
					g_Axis_Conf.run_delay     		 = 0;
					g_Axis_Conf.new_task_req 		 = true;

					g_Axis_Conf.p_pyb->new_task_flag = true;	/* ��������У���������������ָͬ��֮�󣨵��¸ñ�־Ϊfalse�����򲻻��߻���λ��Χ֮�ڣ�����Ҳ����Ӧ����ָ�� */

	//				max_acc_temp = g_Axis_Conf.p_pyb->max_acc;	/* ������ٶ� */
	//				g_Axis_Conf.p_pyb->max_acc = 200;

					g_force_hold_flag = 1;
				}
            } else {
            	g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
            }
        } else {
        	g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.lim_p = 0;
    }
}

/**
 * \brief ���µ�ǰλ��
 */
#pragma CODE_SECTION(Updat_Cur_Local, "ramfuncs");
void Updat_Cur_Local (void)
{
	static Uint16 updata_flag = 0;

    /* ���������и��µ�ǰλ�ã�����ȽǶȴ������ȶ�֮��Ÿ��� */
    if (g_angle_adj_finish == 1) {
    	if (updata_flag == 0) {	/* �ȸ���һ�� */
    		updata_flag = 1;
    		*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
    	} else {
    		if (((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) < 0 - 11111)) ||
    			((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) > 11111))) {	/* 5������ */
        		if (((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) < 0 - 11111)) ||
        			((g_Axis_Conf.p_pyb->cur_local - (*gp_boot_arg_last_pd_local_motor) > 11111))) {
        			g_Axis_Conf.p_pyb->cur_local = *gp_boot_arg_last_pd_local_motor;				/* ������� */
        		} else {
        			*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
        		}
    		} else {
    			*gp_boot_arg_last_pd_local_motor = g_Axis_Conf.p_pyb->cur_local;
    		}
    	}
    }
}
#endif

#pragma CODE_SECTION(App_task, "ramfuncs");
void App_task(void)
{
//	Uint64 Test_Tick = GetSysTick();
//	Uint64 SPI_SendTick = GetSysTick();
#if BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	Uint64 Cur_Local_Tick = GetSysTick();
#endif
//	Uint16 cur_local_flag = 0;

#if 0//���Դ��루�����ڲ�����ָ�������ʹ����˶���
#if BOARD_NAME == BOARD_DGM_2
	g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_TRACE;
	g_Axis_Conf.p_pya->loc_given     = 0;
	g_Axis_Conf.p_pya->speed_given   = 1333;
	g_Axis_Conf.p_pya->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#elif BOARD_NAME == BOARD_DGM_4
	g_Axis_Conf.p_pya->work_mode_set = WORK_MODE_TRACE;
	g_Axis_Conf.p_pya->loc_given     = 0;
	g_Axis_Conf.p_pya->speed_given   = 1333;
	g_Axis_Conf.p_pya->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;

    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
    g_Axis_Conf.p_pyb->loc_given     = 0;
    g_Axis_Conf.p_pyb->speed_given   = 1333;
    g_Axis_Conf.p_pyb->run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#elif BOARD_NAME == BOARD_GMS
    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_TRACE;
    g_Axis_Conf.p_pyb->loc_given     = 0;
    g_Axis_Conf.p_pyb->speed_given   = 1333;
    g_Axis_Conf.run_delay     = 0;
    g_Axis_Conf.new_task_req = true;
#endif
#endif
    while (1) {

        KickDog();  /* ι����RAM���У�������27.25us��MRAM���У�������140us�����������700us */

#if 0   /* ���Ź����� */
        if (ElapsedTick(Test_Tick) >= TICK_10S) {
            while(1) {
                GpioDataRegs.GPBTOGGLE.bit.GPIOB1 = 1;
            }
        }
#endif

        RevDataRead();

        /* ���ŷ�ת���� */
//        if (ElapsedTick(Test_Tick) >= TICK_10MS) {
//            GpioDataRegs.GPGTOGGLE.bit.GPIOG4 = 1;
//            GpioDataRegs.GPGTOGGLE.bit.GPIOG5 = 1;
//            GpioDataRegs.GPFTOGGLE.bit.GPIOF6 = 1;
//            GpioDataRegs.GPFTOGGLE.bit.GPIOF7 = 1;
//
//        	Test_Tick = GetSysTick();
//        }

        AB_Axis_Mov_Task();
        ADC_task();
        //����
        Commu_Interf_Reset_Task();

#if BOARD_NAME == BOARD_GMS
        Angle_Senser_Task();
        Cycle_Hall_Task();
        Soft_Lim_Task();
        Updat_Cur_Local();
        CJW_HE_5701_sw_req_task();
#elif BOARD_NAME == BOARD_DGM_2
        if ((cur_local_flag == 0) && (ElapsedTick(Cur_Local_Tick) >= TICK_2S)) {
        	cur_local_flag = 1;
			g_Axis_Conf.p_pya->cur_local = Vol_To_Angle('A', '+', 1) * 25600 / 9;
			g_Axis_Conf.p_nya->cur_local = Vol_To_Angle('A', '-', 1) * 25600 / 9;
        }
#elif BOARD_NAME == BOARD_DGM_4
        if ((cur_local_flag == 0) && (ElapsedTick(Cur_Local_Tick) >= TICK_2S)) {
            cur_local_flag = 1;
            g_Axis_Conf.p_pya->cur_local = Vol_To_Angle('A', '+', 1) * 25600 / 9;
            g_Axis_Conf.p_nya->cur_local = Vol_To_Angle('A', '-', 1) * 25600 / 9;
            g_Axis_Conf.p_pyb->cur_local = Vol_To_Angle('B', '+', 1) * 10000 / 9;
            g_Axis_Conf.p_nyb->cur_local = Vol_To_Angle('B', '-', 1) * 10000 / 9;
        }
#endif

        Over_Cur_Task();
    }
}

/**
 * \brief ��ʼ������
 */
void Init_Variable(void)
{
    InitTeleDat();
    Init_Axis_Conf();

    if ((*gp_boot_arg_angle_senser_adj < -360) || (*gp_boot_arg_angle_senser_adj > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pya_1 < -360) || (*gp_boot_arg_angle_senser_adj_pya_1 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pya_1 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pya_1 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pya_2 < -360) || (*gp_boot_arg_angle_senser_adj_pya_2 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pya_2 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pya_2 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nya_1 < -360) || (*gp_boot_arg_angle_senser_adj_nya_1 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nya_1 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nya_1 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nya_2 < -360) || (*gp_boot_arg_angle_senser_adj_nya_2 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nya_2 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nya_2 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_pyb_3 < -360) || (*gp_boot_arg_angle_senser_adj_pyb_3 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_pyb_3 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_pyb_3 = 0;
    }
    if ((*gp_boot_arg_angle_senser_adj_nyb_3 < -360) || (*gp_boot_arg_angle_senser_adj_nyb_3 > 360) || (*(Uint32*)gp_boot_arg_angle_senser_adj_nyb_3 == 0xFFFFFFFF)) {
        *gp_boot_arg_angle_senser_adj_nyb_3 = 0;
    }
}

/**
 * \brief ���ж�
 */
static void __disable_irq(void)
{
    DINT;
    DRTM;

	IER = 0x0000;
	IFR = 0x0000;
}

/**
 * \brief ���ж�
 */
static void __enable_irq(void)
{
    /* ����һ�����⣬��������֮����������ʹ��������ô�ý����ж϶�û�н�������CPU��ʱ�����ⲿ�ж�1��PWM����ȥ����������Խ� */
    //IFR = 0x0000;   /* ʹ���ж�ǰ����������жϱ�־�� */

    EINT;
    ERTM;
}

extern Uint16 IQmathLoadStart;
extern Uint16 IQmathLoadEnd;
extern Uint16 IQmathRunStart;

/**
 * \brief ��ʼ�����Ź�
 *
 * \note ���Ź�����ֵ�ﵽ256���
 */

uint8_t g_last_reset_reason = 0;
#pragma DATA_SECTION(g_last_reset_reason, "pre_roll_data");

void reset_reasion_update()
{
    Uint32* p_wdflag = (void*)0;
    Uint16 __g_last_reset_is_wdt = 0;   /* �ϴθ�λԭ���Ƿ�����Ϊ���Ź� */

#if BOARD_NAME == BOARD_GMS
    if (*p_wdflag == 0x12345678) {
        __g_last_reset_is_wdt = 1;
    } else {
        *p_wdflag = 0x12345678;
    }

    if ((*gp_boot_arg_serial_fault_reset) == 1) {     /* ��Ϊ�����쳣�����ÿ��Ź���λ */
        g_last_reset_reason = LAST_RESET_REASON_SERIAL_ERR;
    } else if (__g_last_reset_is_wdt == 1) {
        g_last_reset_reason = LAST_RESET_REASON_WDT;
    } else {
        g_last_reset_reason = LAST_RESET_REASON_POWER_ON;
    }

    *gp_boot_arg_serial_fault_reset = 0;
#endif
}

void InitWDT(void)
{
    /**< \note �ٷ������ֲ���˵��281xϵ��ͨ��WDFLAG�ж��Ƿ��ǿ��Ź���λ���ɿ�����2833xϵ�к�2823xϵ���Ѿ������������⣬���2812Ҫ�ñ�ķ����ж��Ƿ��ǿ��Ź���λ */
    EALLOW;
//    if (SysCtrlRegs.WDCR & 0x80) {  /* �ϴθ�λ���ɿ��Ź������ */
//        SysCtrlRegs.WDCR |= 0x80;   /* ������Ź���λ��־ */
//    }

    /* ���Ź�ʱ��ΪOSCCLK / 512 / 64����Լ279.6ms���Ź���λ�� */
    SysCtrlRegs.WDCR = 0x2F;
    EDIS;
}

void main(void)
{
	/*��ʼ��ϵͳ*/
	InitSysCtrl();

    /*��ʼ������*/
    InitGpio();
#ifdef RUN_MRAM
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    MemCopy(&IQmathLoadStart, &IQmathLoadEnd, &IQmathRunStart);
#endif
	xintf_zone6and7_timing();

	/*���ж�*/
	__disable_irq();

	/*��ʼ��PIE*/
	InitPieCtrl();

	/*��ʼ��PIE�ж�ʸ����*/
	InitPieVectTable();

	/**
	 * \brief ��Ҫ�ŵ���ʼ������ǰ�� ,ͨ������佫boot rom�ռ����ӳ�䵽Ƭ�ڿռ䡣
	 * ͨ��Zone7������������������Ҹ�ֵΪ0
	 *
	 * ������ֳ����²���ȥ�������������Ϣ����File Loader: Verification failed: Values at address 0x3FFFC0@Program do not match Please verify target memory and memory map.��
	 * ��ô�������ΪRAM���У���������ע�͵�������һ�飬Ȼ���ٽ�����ָ��������ɡ�
	 * ������������������û��ʲô���ã����ʵ���²���ȥ�Ͱ�У��ص���
	 */
	EALLOW;
	XintfRegs.XINTCNF2.bit.MPNMC = 0;
	EDIS;

    (*gp_boot_arg_system_reset_times)++;
    reset_reasion_update();

	Init_Variable();   /* ��ʼ�����������ܷ���ʹ���ж�֮�� */

	DriverSciInit(&SciaRegs, 115200, 1, 0, 7, 0, SCI_Rx_isr, SCI_TxFifo_isr);  //������115200����У�飬8���ַ����ȣ�1��ֹͣλ��

	CPU_Timer0_Init();

#if BOARD_NAME == BOARD_GMS
    CPU_Timer1_Init();
    Hall_Init();
	CJW_HE_5701_Init();
	TLV2548_Init();
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
	TLC2543_Init();
#endif

	PPS_Init();
	InitEV_PWM();
	__enable_irq();

#if BOARD_NAME == BOARD_DGM_2
	GpioDataRegs.GPFDAT.bit.GPIOF8 = 0;    /* ʹ��164245 */
    g_Axis_Conf.p_pya->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
    g_Axis_Conf.p_nya->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
    GpioDataRegs.GPFDAT.bit.GPIOF8 = 0;    /* ʹ��164245 */
    g_Axis_Conf.p_pya->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
    g_Axis_Conf.p_nya->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
    g_Axis_Conf.p_pyb->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
    g_Axis_Conf.p_nyb->fn_write_brake(0);                                       /* ���ɲ�� ,�൱��ʹ���ź�*/
#elif BOARD_NAME == BOARD_GMS
//    g_Axis_Conf.p_pyb->fn_write_brake(0);										/* ���ɲ�� ,�൱��ʹ���ź�*/
#endif

    g_RS422_DATA_Tick = GetSysTick();

    InitWDT();
    App_task();
}

//===========================================================================
// No more.
//===========================================================================
