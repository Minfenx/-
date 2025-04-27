/*
 * PWM.c
 *
 *  Created on: 2024��1��17��
 *      Author: 86132
 */
#include "PWM.h"
#include "cmd.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "tele_dat.h"
#include "GL_CJW_HE_5701.h"
#include "cpu_timer0.h"

/* �����˹��±����ı�־ */
#if BOARD_NAME == BOARD_GMS
Uint16 g_pdpinta_pyb_flag = 0;
#pragma DATA_SECTION(g_pdpinta_pyb_flag, "pre_roll_data");
#elif BOARD_NAME == BOARD_DGM_2
Uint16 g_pdpinta_pya_flag;
Uint16 g_pdpinta_nya_flag;
#elif BOARD_NAME == BOARD_DGM_4 ||BOARD_NAME == BOARD_CX20
Uint16 g_pdpinta_pya_flag = 0;
Uint16 g_pdpinta_nya_flag = 0;
Uint16 g_pdpinta_pyb_flag = 0;
Uint16 g_pdpinta_nyb_flag = 0;
#endif

volatile static Uint64 __g_Over_Cur_Tick = 0;
#pragma DATA_SECTION(__g_Over_Cur_Tick, "pre_roll_data");
/**
 * \note �������õ���pwm���費ͬ ,���Դ���һ��ͨ����
 */
#pragma CODE_SECTION(__pwm_handler, "ramfuncs");
#if (BOARD_NAME == BOARD_GMS)
void __pwm_handler(single_axis_conf_t* p_axis , uint8_t ch)
{
	int8_t dir_write;

    if (p_axis->is_task_running == true) {

		/* ����Ҫע�⣬��Ϊʹ����Ӱ�ӼĴ������������ŷ�ת��ʱ���Ӻ�һ������ */
    	/**
    	 * \brief ����任��ʱ�������������ڽ��紦�����ҵ�һ��������һ���ʱ�������պ����ڲ�����߲��ȵ�ʱ����з���任
    	 *
    	 * \note ���ŷ�ת�Ĳ���Ҫ������һ���ε�ǰ�棬��Ϊ���ŷ�ת��Ҫ�ķ�ʱ��
    	 */
    	if ((p_axis->cur_xifen_num == 0) && (p_axis->cur_xifen_pulse_num == (p_axis->next_segment_pulse_cnt / 2))) {
        	dir_write = p_axis->stage[p_axis->cur_stage].mov_dir == 1 ? 1 : 0;
            switch (p_axis->period_cnt_1_4) {							   //һ������������ʼ�ı�����
               case 0:
                   p_axis->fn_write_a_dir(dir_write);
                   p_axis->fn_write_b_dir(1);
               break;
               case 1:
                   p_axis->fn_write_a_dir(1);
                   p_axis->fn_write_b_dir(!dir_write);
               break;
               case 2:
                   p_axis->fn_write_a_dir(!dir_write);
                   p_axis->fn_write_b_dir(0);
               break;
               case 3:
                   p_axis->fn_write_a_dir(0);
                   p_axis->fn_write_b_dir(dir_write);
               break;
            }
    	}

		// ÿ����һ��PWM�жϣ����������һ��50us��PWM����������һ
		p_axis->cur_xifen_pulse_num++;

		/**
		 * \brief һ��΢���Ѿ�����
		 *
		 * \note һ��΢����������������Ǹ��ݼ��ٶȼ��������
		 */
		if (p_axis->cur_xifen_pulse_num == p_axis->next_segment_pulse_cnt) {
		    p_axis->cur_xifen_pulse_num = 0;

		    /* 0.9�Ĳ���ǣ���2�μ�һ����1�� */
		    p_axis->step_temp += p_axis->stage[p_axis->cur_stage].mov_dir;
		    if (p_axis->step_temp == 2) {
		        p_axis->cur_local += 1;      			/* ��ǰʵʱλ�ü�һ or ��һ*/
		        p_axis->step_temp = 0;
		    } else if (p_axis->step_temp == -2) {
                p_axis->cur_local -= 1;               /* ��ǰʵʱλ�ü�һ or ��һ*/
                p_axis->step_temp = 0;
		    }


			p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt++;		      				/* �����ĶΣ�΢��������һ */

		    p_axis->cur_xifen_num++;	/* ϸ�֣�΢��������һ */

		    // һ��1/4�����Ѿ����꣬period_cnt_1_4��1������һ������
		    if (p_axis->cur_xifen_num == XIFEN_CNT) {
		        p_axis->cur_xifen_num = 0;									   //ϸ�ּ�������

		        if (p_axis->stage[p_axis->cur_stage].mov_dir == 1) {
		        	p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 1) % 4; // ��ת ,��һ
		        } else if (p_axis->stage[p_axis->cur_stage].mov_dir == -1) {
		            p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 3) % 4; // ��ת ,��һ
		        }

			    if (p_axis->tail_flag == true) 							   /* ��������׶� */
		        {
//		        	p_axis->tail_flag = false;							   /* ����·���滮�������潫������־���� */
					p_axis->is_task_running = false;					   /* Ϊfalse��ʾ��ǰ��δ��������*/
					g_Axis_Conf.task_cnt--;

					/* ����ģʽ */
                    if (p_axis->work_mode_old == WORK_MODE_RESET) {
                        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 2;    /* �Ѿ��ص���λ */
                    }

					p_axis->work_mode_set = WORK_MODE_KEEP;				   /* Ĭ�ϱ���ģʽ*/
					p_axis->work_mode_old = WORK_MODE_KEEP;
					if (g_angle_senser_cmd_eff_flag == 0) {
		                g_angle_sensor_task_req = 1;
		                g_angle_sensor_sw_req = 0;
					}
					p_axis->cur_angle_speed = 0;						   /* ��ǰ�˶����ٶ�Ϊ0*/
					p_axis->next_segment_pulse_cnt = 0;					   /* �¸��׶ε�΢��������*/
					p_axis->stage[p_axis->cur_stage].mov_dir = 0;          /* �˶��������㣬������յ������ź��Ժ�����Ƕ���Ȼ�ᷢ���ı� */

					/* ���������ģʽ������ֹ֮ͣ�����¸������� */
					if (p_axis->reciprocat_mode_enable == 1) {
						p_axis->work_mode_set = WORK_MODE_TRACE;
						p_axis->speed_given   = 0 - p_axis->speed_given;
						g_Axis_Conf.run_delay     = 0;
						g_Axis_Conf.new_task_req = true;
						p_axis->new_task_flag = 1;
					}
		        }
			    if (p_axis->reverse_flag == true) {						   /* ��Ҫ���� ,ͣ��������֮�� ,�׶μ�һ */
			    	p_axis->reverse_flag = false;
			    	p_axis->cur_stage++;								   /* ������һ�׶� */
			    	p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt = 0;/* ��һ�׶δ�0��ʼ*/
			    }
		    }
		    /* ���������һ���жϺ��� */
			/* ÿ��΢�������� ,дӰ�ӼĴ��� ,���Ե��Բ鿴�ļĴ�����ֵ�Ӻ���, �ı�ȽϼĴ�����ֵ  ,�ͷ����޹� ,�˶�������DIR���ſ���  */
		    if(ch == 1) {
	            switch (p_axis->period_cnt_1_4) {
	                case 0:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A��ռ�ձ�������
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
	                break;
	                case 1:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B��ռ�ձ�������
	                break;
	                case 2:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A��ռ�ձ�������
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
	                break;
	                case 3:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B��ռ�ձ�������
	                break;
	            }
		    }
//		    else if(ch == 2) {
//	            switch (p_axis->period_cnt_1_4) {
//	                case 0:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A��ռ�ձ�������
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
//	                break;
//	                case 1:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B��ռ�ձ�������
//	                break;
//	                case 2:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A��ռ�ձ�������
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
//	                break;
//	                case 3:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B��ռ�ձ�������
//	                break;
//	            }
//		    }
			/* ��������β�׶� */
			/* �жϵ�ǰ�׶��Ƿ��Ѿ����� */
			/* ��ǰ�׶��߹��Ĳ��������ܲ��� ,���Ҳ�Ϊ����ģʽ */
			if(p_axis->tail_flag == false)
			{
	            if ((p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt >= p_axis->stage[p_axis->cur_stage].all_segment) && (p_axis->stage[p_axis->cur_stage].loc_diff != (~0))) {
	            	/* �����׶γ��ַ������� */
	            	if((p_axis->stage[p_axis->cur_stage].mov_dir != p_axis->stage[p_axis->cur_stage + 1].mov_dir) && (p_axis->cur_stage < (p_axis->stage_cnt - 1)))
	            	{
	            		p_axis->reverse_flag = true;						  /* ��Ҫ�����˶���,�Ȳ�������һ�׶� */
	            		p_axis->next_segment_pulse_cnt = WHOLE_STEPS;		  /* ���ٵ�0�� ,��һ����С���ٶ��ߵ�������*/
	            	}
	            	if(p_axis->reverse_flag == false) {						  /* ��������ڷ������� ,ֱ�ӽ�����һ�׶� */
		                p_axis->cur_stage++;
	            	}
	                /* ������н׶β����Ѿ�����  ,�����ͣ��������*/
	                if (p_axis->cur_stage >= p_axis->stage_cnt) {
	                	p_axis->tail_flag = true;							  /* ����Ҫ���� ,��һ������ֹͣ����*/
	                	p_axis->cur_stage--;								  /* ѡ�����һ���׶� */
	                	p_axis->next_segment_pulse_cnt = WHOLE_STEPS;		  /* ���ٵ�0�� ,��һ����С���ٶ��ߵ�������*/
	                } else {
	                    if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
	                    	p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);  //������һ��΢��ʱ��
	                    } else {
	                        p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
	                        p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
	                        if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
	                            p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
	                            p_axis->next_segment_pulse_cnt++;
	                        }
	                    }
	                }
	            } else {   //��ǰ�׶�û��������Ǹ���ģʽ
	                if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
	                    p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);
	                } else {
	                    p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
	                    p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
	                    if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
	                        p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
	                        p_axis->next_segment_pulse_cnt++;
	                    }
	                }
	            }
			}
		}
    }
    else {  //���û������ִ�� ,��Ҫ�������� ����������� ,���ݱ��ֵ�������
#if	0
        switch (p_axis->period_cnt_1_4) {
            case 0:
            	p_axis->fn_write_b_dir(1);							//B�����Ϊ��
            	EvaRegs.CMPR1 = PWM_PERIOD;							//A�����Ϊ0
            	EvaRegs.CMPR2 = p_axis->keep_cur_given; 			//B��������
            break;
            case 1:
            	p_axis->fn_write_a_dir(1);							//A�����Ϊ��
            	EvaRegs.CMPR1 = p_axis->keep_cur_given; 			//A��������
            	EvaRegs.CMPR2 = PWM_PERIOD;							//B�����Ϊ0
            break;
            case 2:
            	p_axis->fn_write_b_dir(0);							//B�����Ϊ��
            	EvaRegs.CMPR1 = PWM_PERIOD;							//A�����Ϊ0
            	EvaRegs.CMPR2 = p_axis->keep_cur_given; 			//B��������
            break;
            case 3:
            	p_axis->fn_write_a_dir(0);							//A�����Ϊ��
            	EvaRegs.CMPR1 = p_axis->keep_cur_given; 			//A��������
            	EvaRegs.CMPR2 = PWM_PERIOD;							//B�����Ϊ0
            break;
        }
#else
        if(ch == 1) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                	p_axis->fn_write_b_dir(1);							//B�����Ϊ��
                	EvaRegs.CMPR1 = PWM_PERIOD;							//A�����Ϊ0
                	//EvaRegs.CMPR2 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//B��������
                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 1:
                	p_axis->fn_write_a_dir(1);							//A�����Ϊ��
                    //EvaRegs.CMPR1 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT];     //A��������
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                	EvaRegs.CMPR2 = PWM_PERIOD;							//B�����Ϊ0
                break;

                case 2:
                	p_axis->fn_write_b_dir(0);							//B�����Ϊ��
                	EvaRegs.CMPR1 = PWM_PERIOD;							//A�����Ϊ0
                	//EvaRegs.CMPR2 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//B��������
                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 3:
                	p_axis->fn_write_a_dir(0);							//A�����Ϊ��
                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                	//EvaRegs.CMPR1 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//A��������
                	EvaRegs.CMPR2 = PWM_PERIOD;							//B�����Ϊ0
                break;
            }
        }
//        else if(ch == 2) {
//            switch (p_axis->period_cnt_1_4) {
//                case 0:
//                	p_axis->fn_write_b_dir(1);							//B�����Ϊ��
//                	EvbRegs.CMPR4 = PWM_PERIOD;							//A�����Ϊ0
//                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//B��������
//                break;
//                case 1:
//                	p_axis->fn_write_a_dir(1);							//A�����Ϊ��
//                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//A��������
//                	EvbRegs.CMPR5 = PWM_PERIOD;							//B�����Ϊ0
//                break;
//                case 2:
//                	p_axis->fn_write_b_dir(0);							//B�����Ϊ��
//                	EvbRegs.CMPR4 = PWM_PERIOD;							//A�����Ϊ0
//                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//B��������
//                break;
//                case 3:
//                	p_axis->fn_write_a_dir(0);							//A�����Ϊ��
//                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//A��������
//                	EvbRegs.CMPR5 = PWM_PERIOD;							//B�����Ϊ0
//                break;
//            }
//        }
#endif
    }
}
#elif BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20
void __pwm_handler(single_axis_conf_t* p_axis , uint8_t ch)
{
    int8_t dir_write;

    if (p_axis->is_task_running == true) {

        /* ����Ҫע�⣬��Ϊʹ����Ӱ�ӼĴ������������ŷ�ת��ʱ���Ӻ�һ������ */
        /**
         * \brief ����任��ʱ�������������ڽ��紦�����ҵ�һ��������һ���ʱ�������պ����ڲ�����߲��ȵ�ʱ����з���任
         *
         * \note ���ŷ�ת�Ĳ���Ҫ������һ���ε�ǰ�棬��Ϊ���ŷ�ת��Ҫ�ķ�ʱ��
         */
        if ((p_axis->cur_xifen_num == 0) && (p_axis->cur_xifen_pulse_num == (p_axis->next_segment_pulse_cnt / 2))) {
            dir_write = p_axis->stage[p_axis->cur_stage].mov_dir == 1 ? 1 : 0;

            switch (p_axis->period_cnt_1_4) {                              //һ������������ʼ�ı�����
               case 0:
                   p_axis->fn_write_a_dir(dir_write);
                   p_axis->fn_write_b_dir(1);
               break;
               case 1:
                   p_axis->fn_write_a_dir(1);
                   p_axis->fn_write_b_dir(!dir_write);
               break;
               case 2:
                   p_axis->fn_write_a_dir(!dir_write);
                   p_axis->fn_write_b_dir(0);
               break;
               case 3:
                   p_axis->fn_write_a_dir(0);
                   p_axis->fn_write_b_dir(dir_write);
               break;
            }
        }

        // ÿ����һ��PWM�жϣ����������һ��50us��PWM����������һ
        p_axis->cur_xifen_pulse_num++;

        /**
         * \brief һ��΢���Ѿ�����
         *
         * \note һ��΢����������������Ǹ��ݼ��ٶȼ��������
         */
        if (p_axis->cur_xifen_pulse_num == p_axis->next_segment_pulse_cnt) {
            p_axis->cur_xifen_pulse_num = 0;

            /* DGM-4B��1.8�Ĳ���ǣ���2�μ�һ����1�� */
            p_axis->step_temp += p_axis->stage[p_axis->cur_stage].mov_dir;
            if (p_axis->step_temp == 2) {
                p_axis->cur_local += 1;                 /* ��ǰʵʱλ�ü�һ or ��һ*/
                p_axis->step_temp = 0;
            } else if (p_axis->step_temp == -2) {
                p_axis->cur_local -= 1;               /* ��ǰʵʱλ�ü�һ or ��һ*/
                p_axis->step_temp = 0;

            }

#if BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
            if (ch < 3) {
            	if (p_axis->cur_local >= 1024000) {
            		p_axis->cur_local -= 1024000;
            	}
            	if (p_axis->cur_local < 0) {
            		p_axis->cur_local += 1024000;
            	}
            }
#endif

            p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt++;                         /* �����ĶΣ�΢��������һ */

            p_axis->cur_xifen_num++;    /* ϸ�֣�΢��������һ */

            // һ��1/4�����Ѿ����꣬period_cnt_1_4��1������һ������
            if (p_axis->cur_xifen_num == XIFEN_CNT) {
                p_axis->cur_xifen_num = 0;                                     //ϸ�ּ�������

                if (p_axis->stage[p_axis->cur_stage].mov_dir == 1) {
                    p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 1) % 4; // ��ת ,��һ
                } else if (p_axis->stage[p_axis->cur_stage].mov_dir == -1) {
                    p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 3) % 4; // ��ת ,��һ
                }

                if (p_axis->tail_flag == true)                             /* ��������׶� */
                {
//                  p_axis->tail_flag = false;                             /* ����·���滮�������潫������־���� */
                    p_axis->is_task_running = false;                       /* Ϊfalse��ʾ��ǰ��δ��������*/
                    g_Axis_Conf.task_cnt--;

					/* ����ģʽ */
                    if (p_axis->work_mode_old == WORK_MODE_RESET) {
                        if (!strncmp(p_axis->name, "PYA", 3)){
                             g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 2;    /* �Ѿ��ص���λ  */
                         }else if(!strncmp(p_axis->name, "NYA", 3)){
                             g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 2;    /* �Ѿ��ص���λ  */
                         }
#if BOARD_NAME == BOARD_DGM_4
                         else if(!strncmp(p_axis->name, "PYB", 3)){
                             g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 2;    /* �Ѿ��ص���λ  */
                         }else if(!strncmp(p_axis->name, "NYB", 3)){
                             g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 2;    /* �Ѿ��ص���λ  */
                         }
#endif
                    }
                    p_axis->work_mode_set = WORK_MODE_KEEP;                /* Ĭ�ϱ���ģʽ*/
                    p_axis->work_mode_old = WORK_MODE_KEEP;
                    p_axis->cur_angle_speed = 0;                           /* ��ǰ�˶����ٶ�Ϊ0*/
                    p_axis->next_segment_pulse_cnt = 0;                    /* �¸��׶ε�΢��������*/

                    /* ���������ģʽ������ֹ֮ͣ�����¸������� */
                    if (p_axis->reciprocat_mode_enable == 1) {
                        p_axis->work_mode_set = WORK_MODE_TRACE;
                        p_axis->speed_given   = -p_axis->speed_given;
                        g_Axis_Conf.run_delay = 0;
                        g_Axis_Conf.new_task_req = true;
                    }
                }
                if (p_axis->reverse_flag == true) {                        /* ��Ҫ���� ,ͣ��������֮�� ,�׶μ�һ */
                    p_axis->reverse_flag = false;
                    p_axis->cur_stage++;                                   /* ������һ�׶� */
                    p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt = 0;/* ��һ�׶δ�0��ʼ*/
                }
            }
            /* ���������һ���жϺ��� */
            /* ÿ��΢�������� ,дӰ�ӼĴ��� ,���Ե��Բ鿴�ļĴ�����ֵ�Ӻ���, �ı�ȽϼĴ�����ֵ  ,�ͷ����޹� ,�˶�������DIR���ſ���  */
            if(ch == 1) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 1:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                    case 2:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 3:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                }
            } else if(ch == 2) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 1:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                    case 2:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 3:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                }
            } else if (ch == 3) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 1:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                    case 2:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 3:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                }
            } else if (ch == 4) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 1:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                    case 2:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A��ռ�ձ�������
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B��ռ�ձ��𽥼�С
                    break;
                    case 3:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A��ռ�ձ��𽥼�С
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B��ռ�ձ�������
                    break;
                }
            }
            /* ��������β�׶� */
            /* �жϵ�ǰ�׶��Ƿ��Ѿ����� */
            /* ��ǰ�׶��߹��Ĳ��������ܲ��� ,���Ҳ�Ϊ����ģʽ */
            if (p_axis->tail_flag == false)
            {
                if ((p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt >= p_axis->stage[p_axis->cur_stage].all_segment) && (p_axis->stage[p_axis->cur_stage].loc_diff != (~0))) {
                    /* �����׶γ��ַ������� */
                    if((p_axis->stage[p_axis->cur_stage].mov_dir != p_axis->stage[p_axis->cur_stage + 1].mov_dir) && (p_axis->cur_stage < (p_axis->stage_cnt - 1)))
                    {
                        p_axis->reverse_flag = true;                          /* ��Ҫ�����˶���,�Ȳ�������һ�׶� */
                        p_axis->next_segment_pulse_cnt = WHOLE_STEPS;         /* ���ٵ�0�� ,��һ����С���ٶ��ߵ�������*/
                    }
                    if(p_axis->reverse_flag == false) {                       /* ��������ڷ������� ,ֱ�ӽ�����һ�׶� */
                        p_axis->cur_stage++;
                    }
                    /* ������н׶β����Ѿ�����  ,�����ͣ��������*/
                    if (p_axis->cur_stage >= p_axis->stage_cnt) {
                        p_axis->tail_flag = true;                             /* ����Ҫ���� ,��һ������ֹͣ����*/
                        p_axis->cur_stage--;                                  /* ѡ�����һ���׶� */
                        p_axis->next_segment_pulse_cnt = WHOLE_STEPS;         /* ���ٵ�0�� ,��һ����С���ٶ��ߵ�������*/
                    } else {
                        if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
                            p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);  //������һ��΢��ʱ��
                        } else {
                            p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
                            p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
                            if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
                                p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
                                p_axis->next_segment_pulse_cnt++;
                            }
                        }
                    }
                } else {   //��ǰ�׶�û��������Ǹ���ģʽ
                    if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
                        p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);
                    } else {
                        p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
                        p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
                        if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
                            p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
                            p_axis->next_segment_pulse_cnt++;
                        }
                    }
                }
            }
        }
    } else if (p_axis->work_mode_set == WORK_MODE_KEEP) {  //���û������ִ�� ,��Ҫ�������� ����������� ,���ݱ��ֵ�������
        if(ch == 1) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B�����Ϊ��
                    EvaRegs.CMPR1 = PWM_PERIOD;                         //A�����Ϊ0
                    EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 1:
                    p_axis->fn_write_a_dir(1);                          //A�����Ϊ��
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvaRegs.CMPR2 = PWM_PERIOD;                         //B�����Ϊ0
                break;

                case 2:
                    p_axis->fn_write_b_dir(0);                          //B�����Ϊ��
                    EvaRegs.CMPR1 = PWM_PERIOD;                         //A�����Ϊ0
                    EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 3:
                    p_axis->fn_write_a_dir(0);                          //A�����Ϊ��
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvaRegs.CMPR2 = PWM_PERIOD;                         //B�����Ϊ0
                break;
            }
        } else if (ch == 2) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B�����Ϊ��
                    EvbRegs.CMPR4 = PWM_PERIOD;                         //A�����Ϊ0
                    EvbRegs.CMPR5 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;
                case 1:
                    p_axis->fn_write_a_dir(1);                          //A�����Ϊ��
                    EvbRegs.CMPR4 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvbRegs.CMPR5 = PWM_PERIOD;                         //B�����Ϊ0
                break;
                case 2:
                    p_axis->fn_write_b_dir(0);                          //B�����Ϊ��
                    EvbRegs.CMPR4 = PWM_PERIOD;                         //A�����Ϊ0
                    EvbRegs.CMPR5 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;
                case 3:
                    p_axis->fn_write_a_dir(0);                          //A�����Ϊ��
                    EvbRegs.CMPR4 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvbRegs.CMPR5 = PWM_PERIOD;                         //B�����Ϊ0
                break;
            }
        } else if (ch == 3) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B�����Ϊ��
                    EvaRegs.CMPR3 = PWM_PERIOD;                         //A�����Ϊ0
                    EvaRegs.T1CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 1:
                    p_axis->fn_write_a_dir(1);                          //A�����Ϊ��
                    EvaRegs.CMPR3 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvaRegs.T1CMPR = PWM_PERIOD;                         //B�����Ϊ0
                break;

                case 2:
                    p_axis->fn_write_b_dir(0);                          //B�����Ϊ��
                    EvaRegs.CMPR3 = PWM_PERIOD;                         //A�����Ϊ0
                    EvaRegs.T1CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                break;

                case 3:
                    p_axis->fn_write_a_dir(0);                          //A�����Ϊ��
                    EvaRegs.CMPR3 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvaRegs.T1CMPR = PWM_PERIOD;                         //B�����Ϊ0
                break;
            }
        } else if (ch == 4) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B�����Ϊ��
                    EvbRegs.CMPR6 = PWM_PERIOD;                         //A�����Ϊ0
                    EvbRegs.T3CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                    break;
                case 1:
                    p_axis->fn_write_a_dir(1);                          //A�����Ϊ��
                    EvbRegs.CMPR6 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvbRegs.T3CMPR = PWM_PERIOD;                         //B�����Ϊ0
                    break;
                case 2:
                    p_axis->fn_write_b_dir(0);                          //B�����Ϊ��
                    EvbRegs.CMPR6 = PWM_PERIOD;                         //A�����Ϊ0
                    EvbRegs.T3CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B��������
                    break;
                case 3:
                    p_axis->fn_write_a_dir(0);                          //A�����Ϊ��
                    EvbRegs.CMPR6 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A��������
                    EvbRegs.T3CMPR = PWM_PERIOD;                         //B�����Ϊ0
                    break;
            }
        }
    } else if (p_axis->work_mode_set == WORK_MODE_STANDBY) {
    	switch (ch) {
    	case 1:
    		EvaRegs.CMPR1 = PWM_PERIOD;
    		EvaRegs.CMPR2 = PWM_PERIOD;
    		break;
    	case 2:
    		EvbRegs.CMPR4 = PWM_PERIOD;                         //A�����Ϊ0
    		EvbRegs.CMPR5 = PWM_PERIOD;
    		break;
    	case 3:
    		EvaRegs.CMPR3 = PWM_PERIOD;     //A��������
    		EvaRegs.T1CMPR = PWM_PERIOD;
    		break;
    	case 4:
    		EvbRegs.CMPR6 = PWM_PERIOD;     //A��������
    		EvbRegs.T3CMPR = PWM_PERIOD;
    		break;
    	}
    }
}
#endif
/**
 * \brief ��ʼ��pwm��GPIO
 */
#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_GMS)
static void __InitEPwmGpio (void)
{
	EALLOW;

//	GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6 = 1;
//	GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7 = 1;
	GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0  = 1;	//����PWM1����
//	GpioMuxRegs.GPAMUX.bit.PWM2_GPIOA1  = 1;
	GpioMuxRegs.GPAMUX.bit.PWM3_GPIOA2  = 1;	//����PWM3����
//	GpioMuxRegs.GPAMUX.bit.PWM4_GPIOA3  = 1;
//	GpioMuxRegs.GPAMUX.bit.PWM5_GPIOA4  = 1;
//	GpioMuxRegs.GPAMUX.bit.PWM6_GPIOA5  = 1;

    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 1;      /* ��GPIOA13����ΪC1TRIP���� */
    GpioMuxRegs.GPAMUX.bit.C2TRIP_GPIOA14 = 1;      /* ��GPIOA14����ΪC2TRIP���� */

//	GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6 = 1;
//	GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7 = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM7_GPIOB0  = 1;	//����PWM7����
//	GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM9_GPIOB2  = 1;	//����PWM9����
//	GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM12_GPIOB5  = 1;
    EDIS;
}
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
static void __InitEPwmGpio (void)
{
    EALLOW;

    //������̨���������·PWM, �ֱ��Ӧ������ϣ�PWM1_GPIOA0��PWM3_GPIOA2����PWM5_GPIOA4��T1PWM_GPIOA6����PWM7_GPIOB0��PWM9_GPIOB2����PWM11_GPIOB4��T3PWM_GPIOB6��
    //EVA
    GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0  = 1;    //����PWM1����
//  GpioMuxRegs.GPAMUX.bit.PWM2_GPIOA1  = 1;
    GpioMuxRegs.GPAMUX.bit.PWM3_GPIOA2  = 1;    //����PWM3����
//  GpioMuxRegs.GPAMUX.bit.PWM4_GPIOA3  = 1;

    GpioMuxRegs.GPAMUX.bit.PWM5_GPIOA4  = 1;
//  GpioMuxRegs.GPAMUX.bit.PWM6_GPIOA5  = 1;
    GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6 = 1;
//  GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7 = 1;

//    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 1;      /* ��GPIOA13����ΪC1TRIP���� */
//    GpioMuxRegs.GPAMUX.bit.C2TRIP_GPIOA14 = 1;      /* ��GPIOA14����ΪC2TRIP���� */
//    GpioMuxRegs.GPAMUX.bit.C3TRIP_GPIOA15 = 1;      /* ��GPIOA15����ΪC3TRIP���� */
//  GpioDataRegs.GPDTOGGLE.bit.GPIOD0 = 1;          /* ��GPIOD0����ΪT1CTRIP_PDPINTA���� */

    //EVB
    GpioMuxRegs.GPBMUX.bit.PWM7_GPIOB0  = 1;    //����PWM7����
//  GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1  = 1;
    GpioMuxRegs.GPBMUX.bit.PWM9_GPIOB2  = 1;    //����PWM9����
//  GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3  = 1;

    GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4  = 1;
//  GpioMuxRegs.GPBMUX.bit.PWM12_GPIOB5  = 1;
    GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6 = 1;
//  GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7 = 1;

    /* B12 ~ B15 ������·ͨ��ѡ�����ĵ�ַ�źţ������������б���ע�͵� */
//  GpioMuxRegs.GPBMUX.bit.C4TRIP_GPIOB13 = 1;      /* ��GPIOB13����ΪC1TRIP���� */
//  GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14 = 1;      /* ��GPIOB14����ΪC2TRIP���� */
//  GpioMuxRegs.GPBMUX.bit.C6TRIP_GPIOB15 = 1;      /* ��GPIOB15����ΪC3TRIP���� */

//  GpioDataRegs.GPDTOGGLE.bit.GPIOD5 = 1;          /* ��GPIOD5����ΪT3TRIP_PDPINTB���� */
    EDIS;
}
#endif

extern Uint16 g_read_angle_ready;
#pragma CODE_SECTION(Eva_T1_pwm_isr, "ramfuncs");
#if (BOARD_NAME == BOARD_GMS)
interrupt void Eva_T1_pwm_isr (void)
{
//    GpioDataRegs.GPBDAT.bit.GPIOB2 = 1;
#if BOARD_NAME == BOARD_GMS
	if ((g_angle_senser_hw_normal == 1) && (g_read_angle_ready == 1)) {
	    g_read_angle_ready = 0;
		PieCtrlRegs.PIEIER2.bit.INTx6 = 0;	/* �ر�PWM�ж� */
		//PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;
		StartCpuTimer1(); /* �����ǶȲɼ� */
	}
#endif
	// ����жϱ�־λ
	EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // ���PIEӦ��Ĵ����ĵ�2λ������Ӧ��2�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    __pwm_handler(g_Axis_Conf.p_pyb , 1);

//    GpioDataRegs.GPBDAT.bit.GPIOB2 = 0;
}
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
interrupt void Eva_T1_pwm_isr (void)
{
    // ����жϱ�־λ
    EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // ���PIEӦ��Ĵ����ĵ�2λ������Ӧ��2�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    GpioDataRegs.GPFDAT.bit.GPIOF6 = 1;
    __pwm_handler(g_Axis_Conf.p_pya , 1);
    __pwm_handler(g_Axis_Conf.p_pyb , 3);
    GpioDataRegs.GPFDAT.bit.GPIOF6 = 0;
}

interrupt void Eva_T3_pwm_isr (void)
{
    // ����жϱ�־λ
    EvbRegs.EVBIFRA.bit.T3UFINT = 1;

    // ���PIEӦ��Ĵ����ĵ�4λ������Ӧ��4�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;

    GpioDataRegs.GPFDAT.bit.GPIOF7 = 1;
    __pwm_handler(g_Axis_Conf.p_nya , 2);
    __pwm_handler(g_Axis_Conf.p_nyb , 4);
    GpioDataRegs.GPFDAT.bit.GPIOF7 = 0;
}
#elif BOARD_NAME == BOARD_DGM_2
interrupt void Eva_T1_pwm_isr (void)
{
    // ����жϱ�־λ
    EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // ���PIEӦ��Ĵ����ĵ�2λ������Ӧ��2�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    __pwm_handler(g_Axis_Conf.p_pya , 1);
}

interrupt void Eva_T3_pwm_isr (void)
{
    // ����жϱ�־λ
    EvbRegs.EVBIFRA.bit.T3UFINT = 1;

    // ���PIEӦ��Ĵ����ĵ�4λ������Ӧ��4�ڵ������ж�����
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;

    __pwm_handler(g_Axis_Conf.p_nya , 2);
}
#endif

#if BOARD_NAME == BOARD_GMS
interrupt void pdpinta_isr(void)
{
    g_pdpinta_pyb_flag = 1;

    /* ��������ܣ����������������һֱΪ�ͣ��ͻ�һֱ���ж� */
    EALLOW;
    EvaRegs.EVAIMRA.bit.PDPINTA = 0;      /* ���ܹ��������ж� */
    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 0;  /* ����Ϊ��ͨIO */
    GpioMuxRegs.GPADIR.bit.GPIOA13 = 0;         /* ���� */
    EDIS;

    /* �ر�PWM�ж� */
    DINT;
    DRTM;
    PieCtrlRegs.PIEIER2.bit.INTx6 = 0;
    asm(" nop ");
    asm(" nop ");
    asm(" nop ");
    asm(" nop ");
    asm(" nop ");
    IFR &= ~M_INT2;
    PieCtrlRegs.PIEIFR2.bit.INTx6 = 0;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;
    EvaRegs.EVAIMRA.bit.PDPINTA = 0;    /* ���ܹ��������ж� */
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;    /* ������������жϱ�־λ */

    EINT;
    ERTM;


    g_Axis_Conf.p_pyb->is_task_running = false;                       /* Ϊfalse��ʾ��ǰ��δ��������*/
    g_Axis_Conf.task_cnt = 0;



    /* ����ģʽ */
    if (g_Axis_Conf.p_pyb->work_mode_old == WORK_MODE_RESET) {
        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 3;    /* �����쳣 */
    }

    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;                /* Ĭ�ϱ���ģʽ*/
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;                           /* ��ǰ�˶����ٶ�Ϊ0*/
    g_Axis_Conf.p_pyb->next_segment_pulse_cnt = 0;                    /* �¸��׶ε�΢��������*/


    PieCtrlRegs.PIEACK.all |= 0x001;    // PIEӦ��λ��1�������´��ж�
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;      /* ���±����жϱ�־λ */

    __g_Over_Cur_Tick = GetSysTick();
}
#endif

extern interrupt void EMPTY_ISR(void);
#if (BOARD_NAME == BOARD_GMS)
void InitEV_PWM(void)
{
	__InitEPwmGpio();					//��ʼ��PWM����
	//Eva PWM1 && PWM3
	//Setup and load T1CON
    EALLOW;
	EvaRegs.T1CNT = 0x0000;				//����������
	EvaRegs.T1PR  = PWM_PERIOD;			//T = 50us

	EvaRegs.T1CON.bit.TMODE   = 0x1;	//�������� ,����50us
	EvaRegs.T1CON.bit.TPS     = 0x0;	//����ʱ��Ԥ��������:T1CLK = HSPCLK / 1 = 45MHz
	EvaRegs.T1CON.bit.TCLKS10 = 0x0;	//ʹ���ڲ�ʱ��T1CLK
	EvaRegs.T1CON.bit.TENABLE = 1;		//ʹ��T1����

	EvaRegs.ACTRA.bit.CMP1ACT = 0x2;	//PWM1�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�

	//Initialize CMPRx
	EvaRegs.CMPR1 = PWM_PERIOD;			//��ʼPWM1/2ռ�ձ�0%
	EvaRegs.CMPR2 = PWM_PERIOD;			//��ʼPWM3/4ռ�ձ�0%

	//���ñȽϿ��ƼĴ���
	EvaRegs.EXTCONA.bit.INDCOE  = 1;	//���ñȽ������������
	EvaRegs.COMCONA.bit.CENABLE = 1;	//ʹ��Eva�Ƚϵ�Ԫ�ıȽϲ���
//	EvaRegs.COMCONA.bit.FCOMPOE = 1;	//ʹ�����еıȽ���� ,PWM1/2/3/4/5/6
	EvaRegs.COMCONA.bit.C1TRIPE = 1;	//ʹ��C1TRIP. ��C1TRIP����Ϊ�͵�ƽʱ(���18200��������) ,FCMP1OE�����Ϊ����̬
	EvaRegs.COMCONA.bit.C2TRIPE = 1;	//ʹ��C2TRIP. ��C2TRIP����Ϊ�͵�ƽʱ(���18200��������) ,FCMP2OE�����Ϊ����̬
	EvaRegs.COMCONA.bit.C3TRIPE = 0;	//��ֹC3TRIP. ����C3TRIP��Ӱ�� FCMP3OE������Լ�PDPINTA��־λ
	EvaRegs.COMCONA.bit.FCMP1OE = 1;	//ʹ��PWM1/2�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����
	EvaRegs.COMCONA.bit.FCMP2OE = 1;	//ʹ��PWM3/4�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����
	//������ʱ�����ƼĴ���
	EvaRegs.DBTCONA.all = 0x0000; 		//��ֹ����

#if 0
    EvaRegs.EXTCONA.bit.INDCOE  = 0;    /* �����������Ƚ�ģ�� */
    EvaRegs.COMCONA.bit.FCOMPOE = 1;    /* ȫ�Ƚ������ʹ�� */
#else
    /* ǰ���Ѿ����ù��� */
//    EvaRegs.EXTCONA.bit.INDCOE  = 1;    /* �����������Ƚ�ģ�� */
//    EvaRegs.COMCONA.bit.FCMP1OE = 1;
//    EvaRegs.COMCONA.bit.FCMP2OE = 1;

    EvaRegs.GPTCONA.bit.T1CMPOE = 1;
    EvaRegs.GPTCONA.bit.T2CMPOE = 1;

    EvaRegs.GPTCONA.bit.T1CTRIPE = 1; /* ��T1CTRIPE���ű��ʱ����ֹPWM��� */
    EvaRegs.GPTCONA.bit.T2CTRIPE = 1; /* ��T2CTRIPE���ű��ʱ����ֹPWM��� */

    /* ʹ��C1TRIP����C1TRIP����Ϊ�͵�ƽʱ��PWM���ű�Ϊ���� */
//    EvaRegs.COMCONA.bit.C1TRIPE = 1;
//    EvaRegs.COMCONA.bit.C2TRIPE = 1;
#endif

	PieVectTable.CMP1INT = &EMPTY_ISR;       /* ������� */

    /* �����ж�ʹ�� */
	EvaRegs.EVAIMRA.bit.CMP1INT = 0;    //���Է��ֲ�д��һ����Զ�ʹ�� ,����Ƶ�����ж�

	/* PIE�ж�ʹ�� */
	PieVectTable.T1UFINT = &Eva_T1_pwm_isr;
    PieVectTable.PDPINTA = &pdpinta_isr;

	EvaRegs.EVAIMRA.bit.T1UFINT = 1;	//��ʱ��1�����ж�
	EvaRegs.EVAIFRA.bit.T1UFINT = 1;	//�����ʱ��1�����жϱ�־λ
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;    /* ������������жϱ�־λ */
    EvaRegs.EVAIMRA.bit.PDPINTA = 1;    /* ʹ�ܹ��������ж� */

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
	PieCtrlRegs.PIEIER2.bit.INTx6 = 1;	//PIE��2 ,���6 T1UFINT ,ʹ��PIE�ж�

	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1=1;     /* ʹ�ܹ��±����ж� */
	/* CPU INT2�ж�ʹ�� */
	IER |= M_INT2;
    IER |= M_INT1;

    EDIS;
}
#elif (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_2)
void InitEV_PWM(void)
{
    __InitEPwmGpio();                   //��ʼ��PWM����
    //Eva PWM1 && PWM3
    EALLOW;

    EvaRegs.T1CNT = 0x0000;             //����������
    EvaRegs.T1PR  = PWM_PERIOD;         //T = 50us

    EvaRegs.T1CON.bit.TMODE   = 0x1;    //��������
    EvaRegs.T1CON.bit.TPS     = 0x0;    //����ʱ��Ԥ��������:T1CLK = HSPCLK / 1 = 45MHz
    EvaRegs.T1CON.bit.TCLKS10 = 0x0;    //ʹ���ڲ�ʱ��
    EvaRegs.T1CON.bit.TCLD10  = 0x0;    //������Ϊ0ʱ����
    EvaRegs.T1CON.bit.TECMPR  = 1;      //ʹ�ܶ�ʱ��1�ıȽϹ���
    EvaRegs.T1CON.bit.TENABLE = 0;      //����T1�����������ʹ��


    EvaRegs.ACTRA.bit.CMP1ACT = 0x2;    //PWM1�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvaRegs.ACTRA.bit.CMP5ACT = 0x2;    //PWM5�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�

    EvaRegs.GPTCONA.bit.T1CMPOE = 1;    //��ʱ��1�Ƚ����ʹ��
    EvaRegs.GPTCONA.bit.T1PIN = 0x2;    // 2����Ч

    //Initialize CMPRx

    EvaRegs.CMPR1 = PWM_PERIOD;         //��ʼPWM1/2ռ�ձ�0%
    EvaRegs.CMPR2 = PWM_PERIOD;         //��ʼPWM3/4ռ�ձ�0%
    EvaRegs.CMPR3 = PWM_PERIOD;         //��ʼPWM5/6ռ�ձ�0%
    EvaRegs.T1CMPR = PWM_PERIOD;        //��ʼ��T1PWMռ�ձ�0%

    //���ñȽϿ��ƼĴ���
    EvaRegs.EXTCONA.bit.INDCOE  = 1;    //���ñȽ������������

    EvaRegs.COMCONA.bit.CENABLE = 1;    //ʹ��Eva�Ƚϵ�Ԫ�ıȽϲ���
//  EvaRegs.COMCONA.bit.FCOMPOE = 1;    //ʹ�����еıȽ���� ,PWM1/2/3/4/5/6
    EvaRegs.COMCONA.bit.C1TRIPE = 0;    //����TRIP����
    EvaRegs.COMCONA.bit.C2TRIPE = 0;
    EvaRegs.COMCONA.bit.C3TRIPE = 0;
    EvaRegs.COMCONA.bit.FCMP1OE = 1;    //ʹ��PWM1/2�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����
    EvaRegs.COMCONA.bit.FCMP2OE = 1;    //ʹ��PWM3/4�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����
    EvaRegs.COMCONA.bit.FCMP3OE = 1;    //ʹ��PWM5/6�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����


    //������ʱ�����ƼĴ���
    EvaRegs.DBTCONA.all = 0x0000;       //��ֹ����

    EvaRegs.GPTCONA.bit.T1CMPOE  = 1;
    EvaRegs.GPTCONA.bit.T1CTRIPE = 0; /* ��T1CTRIPE���ű��ʱ����ֹPWM��� */

    /* PIE�ж�ʹ�� */
    PieVectTable.T1UFINT = &Eva_T1_pwm_isr;

    EvaRegs.EVAIFRA.bit.T1UFINT = 1;    //�����ʱ��1�����жϱ�־λ
    EvaRegs.EVAIMRA.bit.T1UFINT = 1;    //��ʱ��1�����ж�

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx6 = 1;  //PIE��2 ,���6 T1UFINT ,ʹ��PIE�ж�

    /* CPU INT2�ж�ʹ�� */
    IER |= M_INT2;

    EvaRegs.T1CON.bit.TENABLE = 1;      //ʹ��T1����


    EvbRegs.T3CNT = 0x0000;             //����������
    EvbRegs.T3PR  = PWM_PERIOD;         //T = 50us

    EvbRegs.T3CON.bit.TMODE   = 0x1;    //��������
    EvbRegs.T3CON.bit.TPS     = 0x0;    //����ʱ��Ԥ��������:T3CLK = HSPCLK / 1 = 45MHz
    EvbRegs.T3CON.bit.TCLKS10 = 0x0;    //ʹ���ڲ�ʱ��
    EvbRegs.T3CON.bit.TCLD10  = 0x0;    //������Ϊ0ʱ����
    EvbRegs.T3CON.bit.TECMPR  = 1;      //ʹ�ܶ�ʱ��1�ıȽϹ���
    EvbRegs.T3CON.bit.TENABLE = 0;      //����T3�����������ʹ��

    EvbRegs.ACTRB.bit.CMP7ACT  = 0x2;    //PWM1�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvbRegs.ACTRB.bit.CMP9ACT  = 0x2;    //PWM3�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvbRegs.ACTRB.bit.CMP11ACT = 0x2;    //PWM5�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�

    EvbRegs.GPTCONB.bit.T3CMPOE = 1;    //��ʱ��1�Ƚ����ʹ��
    EvbRegs.GPTCONB.bit.T3PIN = 0x2;    // 2����Ч

    //Initialize CMPRx
    EvbRegs.CMPR4 = PWM_PERIOD;         //��ʼPWM1/2ռ�ձ�0%
    EvbRegs.CMPR5 = PWM_PERIOD;         //��ʼPWM3/4ռ�ձ�0%
    EvbRegs.CMPR6 = PWM_PERIOD;         //��ʼPWM5/6ռ�ձ�0%
    EvbRegs.T3CMPR = PWM_PERIOD;        //��ʼ��T3PWMռ�ձ�0%

    //���ñȽϿ��ƼĴ���
    EvbRegs.EXTCONB.bit.INDCOE  = 1;    //���ñȽ������������

    EvbRegs.COMCONB.bit.CENABLE = 1;    //ʹ��Eva�Ƚϵ�Ԫ�ıȽϲ���
//  EvbRegs.COMCONB.bit.FCOMPOE = 1;    //ʹ�����еıȽ���� ,PWM1/2/3/4/5/6
    EvbRegs.COMCONB.bit.C4TRIPE = 0;    //����TRIP����
    EvbRegs.COMCONB.bit.C5TRIPE = 0;
    EvbRegs.COMCONB.bit.C6TRIPE = 0;
    EvbRegs.COMCONB.bit.FCMP4OE = 1;    //ʹ��PWM1/2�ıȽ���� , EXTCONB[0]����Ϊ1����ܵ������ƱȽ����
    EvbRegs.COMCONB.bit.FCMP5OE = 1;    //ʹ��PWM3/4�ıȽ���� , EXTCONB[0]����Ϊ1����ܵ������ƱȽ����
    EvbRegs.COMCONB.bit.FCMP6OE = 1;    //ʹ��PWM5/6�ıȽ���� , EXTCONB[0]����Ϊ1����ܵ������ƱȽ����

    //������ʱ�����ƼĴ���
    EvbRegs.DBTCONB.all = 0x0000;       //��ֹ����

    EvbRegs.GPTCONB.bit.T3CMPOE  = 1;
    EvbRegs.GPTCONB.bit.T3CTRIPE = 0; /* ��T3CTRIPE���ű��ʱ����ֹPWM��� */

    /* PIE�ж�ʹ�� */
    PieVectTable.T3UFINT = &Eva_T3_pwm_isr;

    EvbRegs.EVBIFRA.bit.T3UFINT = 1;    //�����ʱ��1�����жϱ�־λ
    EvbRegs.EVBIMRA.bit.T3UFINT = 1;    //��ʱ��1�����ж�

    PieCtrlRegs.PIEACK.bit.ACK4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  //PIE��2 ,���6 T3UFINT ,ʹ��PIE�ж�

    /* CPU INT2�ж�ʹ�� */
    IER |= M_INT4;

    EvbRegs.T3CON.bit.TENABLE = 1;      //ʹ��T3����

    EDIS;
}
#endif
#if 0
//#elif BOARD_NAME == BOARD_DGM_2
void InitEV_PWM(void)
{
    __InitEPwmGpio();                   //��ʼ��PWM����
    //Eva PWM1 && PWM3
    EALLOW;

    EvaRegs.T1CNT = 0x0000;             //����������
    EvaRegs.T1PR  = PWM_PERIOD;         //T = 50us

    EvaRegs.T1CON.bit.TMODE   = 0x1;    //��������
    EvaRegs.T1CON.bit.TPS     = 0x0;    //����ʱ��Ԥ��������:T1CLK = HSPCLK / 1 = 45MHz
    EvaRegs.T1CON.bit.TCLKS10 = 0x0;    //ʹ���ڲ�ʱ��
    EvaRegs.T1CON.bit.TCLD10  = 0x0;    //������Ϊ0ʱ����
    EvaRegs.T1CON.bit.TECMPR  = 1;      //ʹ�ܶ�ʱ��1�ıȽϹ���
    EvaRegs.T1CON.bit.TENABLE = 0;      //����T1�����������ʹ��

    EvaRegs.ACTRA.bit.CMP1ACT = 0x2;    //PWM1�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�

    EvaRegs.CMPR1 = PWM_PERIOD;         //��ʼPWM1/2ռ�ձ�0%
    EvaRegs.CMPR2 = PWM_PERIOD;         //��ʼPWM3/4ռ�ձ�0%

    //���ñȽϿ��ƼĴ���
    EvaRegs.EXTCONA.bit.INDCOE  = 1;    //���ñȽ������������
    EvaRegs.COMCONA.bit.CENABLE = 1;    //ʹ��Eva�Ƚϵ�Ԫ�ıȽϲ���
    EvaRegs.COMCONA.bit.FCMP1OE = 1;    //ʹ��PWM1/2�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����
    EvaRegs.COMCONA.bit.FCMP2OE = 1;    //ʹ��PWM3/4�ıȽ���� , EXTCONA[0]����Ϊ1����ܵ������ƱȽ����

    //������ʱ�����ƼĴ���
    EvaRegs.DBTCONA.all = 0x0000;       //��ֹ����

    /* PIE�ж�ʹ�� */
    PieVectTable.T1UFINT = &Eva_T1_pwm_isr;

    EvaRegs.EVAIFRA.bit.T1UFINT = 1;    //�����ʱ��1�����жϱ�־λ
    EvaRegs.EVAIMRA.bit.T1UFINT = 1;    //��ʱ��1�����ж�

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx6 = 1;  //PIE��2 ,���6 T1UFINT ,ʹ��PIE�ж�

    /* CPU INT2�ж�ʹ�� */
    IER |= M_INT2;
    EvaRegs.T1CON.bit.TENABLE = 1;      //ʹ��T1����


    EvbRegs.T3CNT = 0x0000;             //����������
    EvbRegs.T3PR  = PWM_PERIOD;         //T = 50us

    EvbRegs.T3CON.bit.TMODE   = 0x1;    //��������
    EvbRegs.T3CON.bit.TPS     = 0x0;    //����ʱ��Ԥ��������:T3CLK = HSPCLK / 1 = 45MHz
    EvbRegs.T3CON.bit.TCLKS10 = 0x0;    //ʹ���ڲ�ʱ��
    EvbRegs.T3CON.bit.TCLD10  = 0x0;    //������Ϊ0ʱ����
    EvbRegs.T3CON.bit.TECMPR  = 1;      //ʹ�ܶ�ʱ��1�ıȽϹ���
    EvbRegs.T3CON.bit.TENABLE = 0;      //����T3�����������ʹ��

    EvbRegs.ACTRB.bit.CMP7ACT  = 0x2;    //PWM1�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�
    EvbRegs.ACTRB.bit.CMP9ACT  = 0x2;    //PWM3�Ƚ���������ϵļ��� ,1����Ч ,2����Ч ,0ǿ�Ƶ� ,3ǿ�Ƹ�

    //Initialize CMPRx
    EvbRegs.CMPR4 = PWM_PERIOD;         //��ʼPWM1/2ռ�ձ�0%
    EvbRegs.CMPR5 = PWM_PERIOD;         //��ʼPWM3/4ռ�ձ�0%

    //���ñȽϿ��ƼĴ���
    EvbRegs.EXTCONB.bit.INDCOE  = 1;    //���ñȽ������������

    EvbRegs.COMCONB.bit.CENABLE = 1;    //ʹ��Eva�Ƚϵ�Ԫ�ıȽϲ���
//  EvbRegs.COMCONB.bit.FCOMPOE = 1;    //ʹ�����еıȽ���� ,PWM1/2/3/4/5/6
    EvbRegs.COMCONB.bit.FCMP4OE = 1;    //ʹ��PWM1/2�ıȽ���� , EXTCONB[0]����Ϊ1����ܵ������ƱȽ����
    EvbRegs.COMCONB.bit.FCMP5OE = 1;    //ʹ��PWM3/4�ıȽ���� , EXTCONB[0]����Ϊ1����ܵ������ƱȽ����

    //������ʱ�����ƼĴ���
    EvbRegs.DBTCONB.all = 0x0000;       //��ֹ����

    /* PIE�ж�ʹ�� */
    PieVectTable.T3UFINT = &Eva_T3_pwm_isr;

    EvbRegs.EVBIFRA.bit.T3UFINT = 1;    //�����ʱ��1�����жϱ�־λ
    EvbRegs.EVBIMRA.bit.T3UFINT = 1;    //��ʱ��1�����ж�

    PieCtrlRegs.PIEACK.bit.ACK4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  //PIE��2 ,���6 T3UFINT ,ʹ��PIE�ж�

    /* CPU INT2�ж�ʹ�� */
    IER |= M_INT4;

    EvbRegs.T3CON.bit.TENABLE = 1;      //ʹ��T3����

    EDIS;
}
#endif

/**
 * \brief ������������
 */
void Over_Cur_Task (void)
{
    /* ������»���� */
#if BOARD_NAME == BOARD_DGM_2
    if (g_pdpinta_pya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB10) {     /* δ�ָ� */
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pya_flag = 0;
        }
    } else {
        g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB8) {     /* δ�ָ� */
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nya_flag = 0;
        }
    } else {
        g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    }
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
    if (g_pdpinta_pya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB10) {     /* δ�ָ� */
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pya_flag = 0;
        }
    } else {
        g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB8) {     /* δ�ָ� */
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nya_flag = 0;
        }
    } else {
        g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_pyb_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB11) {     /* δ�ָ� */
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pyb_flag = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nyb_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB9) {     /* δ�ָ� */
            g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* �ָ��� */
            InitEV_PWM();
            g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nyb_flag = 0;
        }
    } else {
        g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
    }
#elif BOARD_NAME == BOARD_GMS
    if (g_pdpinta_pyb_flag == 1) {
        if (GpioDataRegs.GPADAT.bit.GPIOA13 == 0) {         /* δ�ָ� */
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 1;
            __g_Over_Cur_Tick = GetSysTick();
        } else if (ElapsedTick(__g_Over_Cur_Tick) >= TICK_1S) {  /* �ָ��ˣ��ⲽ��Ϊ�˷�ֹ��������ν�����������ж� */
            InitEV_PWM();
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pyb_flag = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    }
#endif
}

