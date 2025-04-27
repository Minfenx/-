/*
 * PWM.c
 *
 *  Created on: 2024年1月17日
 *      Author: 86132
 */
#include "PWM.h"
#include "cmd.h"
#include "AB_axis_mov_ctl_cmd.h"
#include "tele_dat.h"
#include "GL_CJW_HE_5701.h"
#include "cpu_timer0.h"

/* 进入了过温保护的标志 */
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
 * \note 主副轴用到的pwm外设不同 ,所以传入一个通道数
 */
#pragma CODE_SECTION(__pwm_handler, "ramfuncs");
#if (BOARD_NAME == BOARD_GMS)
void __pwm_handler(single_axis_conf_t* p_axis , uint8_t ch)
{
	int8_t dir_write;

    if (p_axis->is_task_running == true) {

		/* 这里要注意，因为使用了影子寄存器，所以引脚翻转的时机延后一个周期 */
    	/**
    	 * \brief 方向变换的时机是在两个周期交界处，并且第一个段走了一半的时候，这样刚好是在波峰或者波谷的时候进行方向变换
    	 *
    	 * \note 引脚翻转的操作要放在这一整段的前面，因为引脚翻转需要耗费时间
    	 */
    	if ((p_axis->cur_xifen_num == 0) && (p_axis->cur_xifen_pulse_num == (p_axis->next_segment_pulse_cnt / 2))) {
        	dir_write = p_axis->stage[p_axis->cur_stage].mov_dir == 1 ? 1 : 0;
            switch (p_axis->period_cnt_1_4) {							   //一个整步结束开始改变相序
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

		// 每进入一次PWM中断，代表输出了一个50us的PWM波，计数加一
		p_axis->cur_xifen_pulse_num++;

		/**
		 * \brief 一个微步已经走完
		 *
		 * \note 一个微步里包含的脉冲数是根据加速度计算出来的
		 */
		if (p_axis->cur_xifen_pulse_num == p_axis->next_segment_pulse_cnt) {
		    p_axis->cur_xifen_pulse_num = 0;

		    /* 0.9的步距角，进2次加一（减1） */
		    p_axis->step_temp += p_axis->stage[p_axis->cur_stage].mov_dir;
		    if (p_axis->step_temp == 2) {
		        p_axis->cur_local += 1;      			/* 当前实时位置加一 or 减一*/
		        p_axis->step_temp = 0;
		    } else if (p_axis->step_temp == -2) {
                p_axis->cur_local -= 1;               /* 当前实时位置加一 or 减一*/
                p_axis->step_temp = 0;
		    }


			p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt++;		      				/* 经过的段（微步）数加一 */

		    p_axis->cur_xifen_num++;	/* 细分（微步）数加一 */

		    // 一个1/4周期已经走完，period_cnt_1_4加1，走了一个整步
		    if (p_axis->cur_xifen_num == XIFEN_CNT) {
		        p_axis->cur_xifen_num = 0;									   //细分计数清零

		        if (p_axis->stage[p_axis->cur_stage].mov_dir == 1) {
		        	p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 1) % 4; // 正转 ,加一
		        } else if (p_axis->stage[p_axis->cur_stage].mov_dir == -1) {
		            p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 3) % 4; // 反转 ,减一
		        }

			    if (p_axis->tail_flag == true) 							   /* 任务结束阶段 */
		        {
//		        	p_axis->tail_flag = false;							   /* 放在路径规划函数里面将结束标志置零 */
					p_axis->is_task_running = false;					   /* 为false表示当前轴未进行任务*/
					g_Axis_Conf.task_cnt--;

					/* 回零模式 */
                    if (p_axis->work_mode_old == WORK_MODE_RESET) {
                        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 2;    /* 已经回到零位 */
                    }

					p_axis->work_mode_set = WORK_MODE_KEEP;				   /* 默认保持模式*/
					p_axis->work_mode_old = WORK_MODE_KEEP;
					if (g_angle_senser_cmd_eff_flag == 0) {
		                g_angle_sensor_task_req = 1;
		                g_angle_sensor_sw_req = 0;
					}
					p_axis->cur_angle_speed = 0;						   /* 当前运动角速度为0*/
					p_axis->next_segment_pulse_cnt = 0;					   /* 下个阶段的微步数清零*/
					p_axis->stage[p_axis->cur_stage].mov_dir = 0;          /* 运动方向清零，否则接收到霍尔信号以后霍尔角度依然会发生改变 */

					/* 如果是往复模式，则在停止之后重新给个任务 */
					if (p_axis->reciprocat_mode_enable == 1) {
						p_axis->work_mode_set = WORK_MODE_TRACE;
						p_axis->speed_given   = 0 - p_axis->speed_given;
						g_Axis_Conf.run_delay     = 0;
						g_Axis_Conf.new_task_req = true;
						p_axis->new_task_flag = 1;
					}
		        }
			    if (p_axis->reverse_flag == true) {						   /* 需要反向 ,停在整步上之后 ,阶段加一 */
			    	p_axis->reverse_flag = false;
			    	p_axis->cur_stage++;								   /* 进入下一阶段 */
			    	p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt = 0;/* 下一阶段从0开始*/
			    }
		    }
		    /* 必须放在上一个判断后面 */
			/* 每个微步结束后 ,写影子寄存器 ,所以调试查看的寄存器的值延后了, 改变比较寄存器的值  ,和方向无关 ,运动方向由DIR引脚控制  */
		    if(ch == 1) {
	            switch (p_axis->period_cnt_1_4) {
	                case 0:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A相占空比逐渐增大
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
	                break;
	                case 1:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B相占空比逐渐增大
	                break;
	                case 2:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A相占空比逐渐增大
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
	                break;
	                case 3:
	                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
	                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B相占空比逐渐增大
	                break;
	            }
		    }
//		    else if(ch == 2) {
//	            switch (p_axis->period_cnt_1_4) {
//	                case 0:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A相占空比逐渐增大
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
//	                break;
//	                case 1:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B相占空比逐渐增大
//	                break;
//	                case 2:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//A相占空比逐渐增大
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
//	                break;
//	                case 3:
//	                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
//	                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];				//B相占空比逐渐增大
//	                break;
//	            }
//		    }
			/* 不处于收尾阶段 */
			/* 判断当前阶段是否已经走完 */
			/* 当前阶段走过的步数大于总步数 ,并且不为跟踪模式 */
			if(p_axis->tail_flag == false)
			{
	            if ((p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt >= p_axis->stage[p_axis->cur_stage].all_segment) && (p_axis->stage[p_axis->cur_stage].loc_diff != (~0))) {
	            	/* 非最后阶段出现反向的情况 */
	            	if((p_axis->stage[p_axis->cur_stage].mov_dir != p_axis->stage[p_axis->cur_stage + 1].mov_dir) && (p_axis->cur_stage < (p_axis->stage_cnt - 1)))
	            	{
	            		p_axis->reverse_flag = true;						  /* 需要反向运动了,先不进行下一阶段 */
	            		p_axis->next_segment_pulse_cnt = WHOLE_STEPS;		  /* 减速到0后 ,以一个较小的速度走到整步上*/
	            	}
	            	if(p_axis->reverse_flag == false) {						  /* 如果不存在反向的情况 ,直接进入下一阶段 */
		                p_axis->cur_stage++;
	            	}
	                /* 如果所有阶段步数已经走完  ,则必须停在整步上*/
	                if (p_axis->cur_stage >= p_axis->stage_cnt) {
	                	p_axis->tail_flag = true;							  /* 任务将要结束 ,下一个整步停止任务*/
	                	p_axis->cur_stage--;								  /* 选择最后一个阶段 */
	                	p_axis->next_segment_pulse_cnt = WHOLE_STEPS;		  /* 减速到0后 ,以一个较小的速度走到整步上*/
	                } else {
	                    if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
	                    	p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);  //计算下一次微步时间
	                    } else {
	                        p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
	                        p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
	                        if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
	                            p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
	                            p_axis->next_segment_pulse_cnt++;
	                        }
	                    }
	                }
	            } else {   //当前阶段没走完或者是跟踪模式
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
    else {  //如果没有任务执行 ,需要根据相序 输出保持力矩 ,根据保持电流设置
#if	0
        switch (p_axis->period_cnt_1_4) {
            case 0:
            	p_axis->fn_write_b_dir(1);							//B相电流为正
            	EvaRegs.CMPR1 = PWM_PERIOD;							//A相电流为0
            	EvaRegs.CMPR2 = p_axis->keep_cur_given; 			//B相电流最大
            break;
            case 1:
            	p_axis->fn_write_a_dir(1);							//A相电流为正
            	EvaRegs.CMPR1 = p_axis->keep_cur_given; 			//A相电流最大
            	EvaRegs.CMPR2 = PWM_PERIOD;							//B相电流为0
            break;
            case 2:
            	p_axis->fn_write_b_dir(0);							//B相电流为负
            	EvaRegs.CMPR1 = PWM_PERIOD;							//A相电流为0
            	EvaRegs.CMPR2 = p_axis->keep_cur_given; 			//B相电流最大
            break;
            case 3:
            	p_axis->fn_write_a_dir(0);							//A相电流为负
            	EvaRegs.CMPR1 = p_axis->keep_cur_given; 			//A相电流最大
            	EvaRegs.CMPR2 = PWM_PERIOD;							//B相电流为0
            break;
        }
#else
        if(ch == 1) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                	p_axis->fn_write_b_dir(1);							//B相电流为正
                	EvaRegs.CMPR1 = PWM_PERIOD;							//A相电流为0
                	//EvaRegs.CMPR2 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//B相电流最大
                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 1:
                	p_axis->fn_write_a_dir(1);							//A相电流为正
                    //EvaRegs.CMPR1 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT];     //A相电流最大
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                	EvaRegs.CMPR2 = PWM_PERIOD;							//B相电流为0
                break;

                case 2:
                	p_axis->fn_write_b_dir(0);							//B相电流为负
                	EvaRegs.CMPR1 = PWM_PERIOD;							//A相电流为0
                	//EvaRegs.CMPR2 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//B相电流最大
                	EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 3:
                	p_axis->fn_write_a_dir(0);							//A相电流为负
                	EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                	//EvaRegs.CMPR1 = (float32)PWM_PERIOD * (1 - p_axis->duty_factor_hold) * g_Axis_Conf.duty[XIFEN_CNT]; 	//A相电流最大
                	EvaRegs.CMPR2 = PWM_PERIOD;							//B相电流为0
                break;
            }
        }
//        else if(ch == 2) {
//            switch (p_axis->period_cnt_1_4) {
//                case 0:
//                	p_axis->fn_write_b_dir(1);							//B相电流为正
//                	EvbRegs.CMPR4 = PWM_PERIOD;							//A相电流为0
//                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//B相电流最大
//                break;
//                case 1:
//                	p_axis->fn_write_a_dir(1);							//A相电流为正
//                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//A相电流最大
//                	EvbRegs.CMPR5 = PWM_PERIOD;							//B相电流为0
//                break;
//                case 2:
//                	p_axis->fn_write_b_dir(0);							//B相电流为负
//                	EvbRegs.CMPR4 = PWM_PERIOD;							//A相电流为0
//                	EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//B相电流最大
//                break;
//                case 3:
//                	p_axis->fn_write_a_dir(0);							//A相电流为负
//                	EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT]; 	//A相电流最大
//                	EvbRegs.CMPR5 = PWM_PERIOD;							//B相电流为0
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

        /* 这里要注意，因为使用了影子寄存器，所以引脚翻转的时机延后一个周期 */
        /**
         * \brief 方向变换的时机是在两个周期交界处，并且第一个段走了一半的时候，这样刚好是在波峰或者波谷的时候进行方向变换
         *
         * \note 引脚翻转的操作要放在这一整段的前面，因为引脚翻转需要耗费时间
         */
        if ((p_axis->cur_xifen_num == 0) && (p_axis->cur_xifen_pulse_num == (p_axis->next_segment_pulse_cnt / 2))) {
            dir_write = p_axis->stage[p_axis->cur_stage].mov_dir == 1 ? 1 : 0;

            switch (p_axis->period_cnt_1_4) {                              //一个整步结束开始改变相序
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

        // 每进入一次PWM中断，代表输出了一个50us的PWM波，计数加一
        p_axis->cur_xifen_pulse_num++;

        /**
         * \brief 一个微步已经走完
         *
         * \note 一个微步里包含的脉冲数是根据加速度计算出来的
         */
        if (p_axis->cur_xifen_pulse_num == p_axis->next_segment_pulse_cnt) {
            p_axis->cur_xifen_pulse_num = 0;

            /* DGM-4B轴1.8的步距角，进2次加一（减1） */
            p_axis->step_temp += p_axis->stage[p_axis->cur_stage].mov_dir;
            if (p_axis->step_temp == 2) {
                p_axis->cur_local += 1;                 /* 当前实时位置加一 or 减一*/
                p_axis->step_temp = 0;
            } else if (p_axis->step_temp == -2) {
                p_axis->cur_local -= 1;               /* 当前实时位置加一 or 减一*/
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

            p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt++;                         /* 经过的段（微步）数加一 */

            p_axis->cur_xifen_num++;    /* 细分（微步）数加一 */

            // 一个1/4周期已经走完，period_cnt_1_4加1，走了一个整步
            if (p_axis->cur_xifen_num == XIFEN_CNT) {
                p_axis->cur_xifen_num = 0;                                     //细分计数清零

                if (p_axis->stage[p_axis->cur_stage].mov_dir == 1) {
                    p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 1) % 4; // 正转 ,加一
                } else if (p_axis->stage[p_axis->cur_stage].mov_dir == -1) {
                    p_axis->period_cnt_1_4 = (p_axis->period_cnt_1_4 + 3) % 4; // 反转 ,减一
                }

                if (p_axis->tail_flag == true)                             /* 任务结束阶段 */
                {
//                  p_axis->tail_flag = false;                             /* 放在路径规划函数里面将结束标志置零 */
                    p_axis->is_task_running = false;                       /* 为false表示当前轴未进行任务*/
                    g_Axis_Conf.task_cnt--;

					/* 回零模式 */
                    if (p_axis->work_mode_old == WORK_MODE_RESET) {
                        if (!strncmp(p_axis->name, "PYA", 3)){
                             g_Tele_Dat.p_ya_axis_stat.bit.rcv_reset = 2;    /* 已经回到零位  */
                         }else if(!strncmp(p_axis->name, "NYA", 3)){
                             g_Tele_Dat.n_ya_axis_stat.bit.rcv_reset = 2;    /* 已经回到零位  */
                         }
#if BOARD_NAME == BOARD_DGM_4
                         else if(!strncmp(p_axis->name, "PYB", 3)){
                             g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 2;    /* 已经回到零位  */
                         }else if(!strncmp(p_axis->name, "NYB", 3)){
                             g_Tele_Dat.n_yb_axis_stat.bit.rcv_reset = 2;    /* 已经回到零位  */
                         }
#endif
                    }
                    p_axis->work_mode_set = WORK_MODE_KEEP;                /* 默认保持模式*/
                    p_axis->work_mode_old = WORK_MODE_KEEP;
                    p_axis->cur_angle_speed = 0;                           /* 当前运动角速度为0*/
                    p_axis->next_segment_pulse_cnt = 0;                    /* 下个阶段的微步数清零*/

                    /* 如果是往复模式，则在停止之后重新给个任务 */
                    if (p_axis->reciprocat_mode_enable == 1) {
                        p_axis->work_mode_set = WORK_MODE_TRACE;
                        p_axis->speed_given   = -p_axis->speed_given;
                        g_Axis_Conf.run_delay = 0;
                        g_Axis_Conf.new_task_req = true;
                    }
                }
                if (p_axis->reverse_flag == true) {                        /* 需要反向 ,停在整步上之后 ,阶段加一 */
                    p_axis->reverse_flag = false;
                    p_axis->cur_stage++;                                   /* 进入下一阶段 */
                    p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt = 0;/* 下一阶段从0开始*/
                }
            }
            /* 必须放在上一个判断后面 */
            /* 每个微步结束后 ,写影子寄存器 ,所以调试查看的寄存器的值延后了, 改变比较寄存器的值  ,和方向无关 ,运动方向由DIR引脚控制  */
            if(ch == 1) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 1:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                    case 2:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 3:
                        EvaRegs.CMPR1 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvaRegs.CMPR2 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                }
            } else if(ch == 2) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 1:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                    case 2:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 3:
                        EvbRegs.CMPR4 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvbRegs.CMPR5 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                }
            } else if (ch == 3) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 1:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                    case 2:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 3:
                        EvaRegs.CMPR3 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvaRegs.T1CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                }
            } else if (ch == 4) {
                switch (p_axis->period_cnt_1_4) {
                    case 0:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 1:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                    case 2:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //A相占空比逐渐增大
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //B相占空比逐渐减小
                    break;
                    case 3:
                        EvbRegs.CMPR6 = p_axis->pulse_tbprd_mov[XIFEN_CNT - p_axis->cur_xifen_num]; //A相占空比逐渐减小
                        EvbRegs.T3CMPR = p_axis->pulse_tbprd_mov[p_axis->cur_xifen_num];             //B相占空比逐渐增大
                    break;
                }
            }
            /* 不处于收尾阶段 */
            /* 判断当前阶段是否已经走完 */
            /* 当前阶段走过的步数大于总步数 ,并且不为跟踪模式 */
            if (p_axis->tail_flag == false)
            {
                if ((p_axis->stage[p_axis->cur_stage].elapsed_segment_cnt >= p_axis->stage[p_axis->cur_stage].all_segment) && (p_axis->stage[p_axis->cur_stage].loc_diff != (~0))) {
                    /* 非最后阶段出现反向的情况 */
                    if((p_axis->stage[p_axis->cur_stage].mov_dir != p_axis->stage[p_axis->cur_stage + 1].mov_dir) && (p_axis->cur_stage < (p_axis->stage_cnt - 1)))
                    {
                        p_axis->reverse_flag = true;                          /* 需要反向运动了,先不进行下一阶段 */
                        p_axis->next_segment_pulse_cnt = WHOLE_STEPS;         /* 减速到0后 ,以一个较小的速度走到整步上*/
                    }
                    if(p_axis->reverse_flag == false) {                       /* 如果不存在反向的情况 ,直接进入下一阶段 */
                        p_axis->cur_stage++;
                    }
                    /* 如果所有阶段步数已经走完  ,则必须停在整步上*/
                    if (p_axis->cur_stage >= p_axis->stage_cnt) {
                        p_axis->tail_flag = true;                             /* 任务将要结束 ,下一个整步停止任务*/
                        p_axis->cur_stage--;                                  /* 选择最后一个阶段 */
                        p_axis->next_segment_pulse_cnt = WHOLE_STEPS;         /* 减速到0后 ,以一个较小的速度走到整步上*/
                    } else {
                        if (p_axis->stage[p_axis->cur_stage].a_1000 != 0) {
                            p_axis->next_segment_pulse_cnt = get_next_segment_pulse_cnt(p_axis);  //计算下一次微步时间
                        } else {
                            p_axis->next_segment_pulse_cnt = p_axis->uniform_segment_pulse_cnt_zheng;
                            p_axis->uniform_segment_pulse_cnt_xiao_sum += p_axis->uniform_segment_pulse_cnt_xiao_1000;
                            if (p_axis->uniform_segment_pulse_cnt_xiao_sum >= 1000) {
                                p_axis->uniform_segment_pulse_cnt_xiao_sum -= 1000;
                                p_axis->next_segment_pulse_cnt++;
                            }
                        }
                    }
                } else {   //当前阶段没走完或者是跟踪模式
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
    } else if (p_axis->work_mode_set == WORK_MODE_KEEP) {  //如果没有任务执行 ,需要根据相序 输出保持力矩 ,根据保持电流设置
        if(ch == 1) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B相电流为正
                    EvaRegs.CMPR1 = PWM_PERIOD;                         //A相电流为0
                    EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 1:
                    p_axis->fn_write_a_dir(1);                          //A相电流为正
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvaRegs.CMPR2 = PWM_PERIOD;                         //B相电流为0
                break;

                case 2:
                    p_axis->fn_write_b_dir(0);                          //B相电流为负
                    EvaRegs.CMPR1 = PWM_PERIOD;                         //A相电流为0
                    EvaRegs.CMPR2 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 3:
                    p_axis->fn_write_a_dir(0);                          //A相电流为负
                    EvaRegs.CMPR1 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvaRegs.CMPR2 = PWM_PERIOD;                         //B相电流为0
                break;
            }
        } else if (ch == 2) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B相电流为正
                    EvbRegs.CMPR4 = PWM_PERIOD;                         //A相电流为0
                    EvbRegs.CMPR5 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;
                case 1:
                    p_axis->fn_write_a_dir(1);                          //A相电流为正
                    EvbRegs.CMPR4 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvbRegs.CMPR5 = PWM_PERIOD;                         //B相电流为0
                break;
                case 2:
                    p_axis->fn_write_b_dir(0);                          //B相电流为负
                    EvbRegs.CMPR4 = PWM_PERIOD;                         //A相电流为0
                    EvbRegs.CMPR5 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;
                case 3:
                    p_axis->fn_write_a_dir(0);                          //A相电流为负
                    EvbRegs.CMPR4 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvbRegs.CMPR5 = PWM_PERIOD;                         //B相电流为0
                break;
            }
        } else if (ch == 3) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B相电流为正
                    EvaRegs.CMPR3 = PWM_PERIOD;                         //A相电流为0
                    EvaRegs.T1CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 1:
                    p_axis->fn_write_a_dir(1);                          //A相电流为正
                    EvaRegs.CMPR3 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvaRegs.T1CMPR = PWM_PERIOD;                         //B相电流为0
                break;

                case 2:
                    p_axis->fn_write_b_dir(0);                          //B相电流为负
                    EvaRegs.CMPR3 = PWM_PERIOD;                         //A相电流为0
                    EvaRegs.T1CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                break;

                case 3:
                    p_axis->fn_write_a_dir(0);                          //A相电流为负
                    EvaRegs.CMPR3 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvaRegs.T1CMPR = PWM_PERIOD;                         //B相电流为0
                break;
            }
        } else if (ch == 4) {
            switch (p_axis->period_cnt_1_4) {
                case 0:
                    p_axis->fn_write_b_dir(1);                          //B相电流为正
                    EvbRegs.CMPR6 = PWM_PERIOD;                         //A相电流为0
                    EvbRegs.T3CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                    break;
                case 1:
                    p_axis->fn_write_a_dir(1);                          //A相电流为正
                    EvbRegs.CMPR6 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvbRegs.T3CMPR = PWM_PERIOD;                         //B相电流为0
                    break;
                case 2:
                    p_axis->fn_write_b_dir(0);                          //B相电流为负
                    EvbRegs.CMPR6 = PWM_PERIOD;                         //A相电流为0
                    EvbRegs.T3CMPR = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //B相电流最大
                    break;
                case 3:
                    p_axis->fn_write_a_dir(0);                          //A相电流为负
                    EvbRegs.CMPR6 = p_axis->pulse_tbprd_hold[XIFEN_CNT];     //A相电流最大
                    EvbRegs.T3CMPR = PWM_PERIOD;                         //B相电流为0
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
    		EvbRegs.CMPR4 = PWM_PERIOD;                         //A相电流为0
    		EvbRegs.CMPR5 = PWM_PERIOD;
    		break;
    	case 3:
    		EvaRegs.CMPR3 = PWM_PERIOD;     //A相电流最大
    		EvaRegs.T1CMPR = PWM_PERIOD;
    		break;
    	case 4:
    		EvbRegs.CMPR6 = PWM_PERIOD;     //A相电流最大
    		EvbRegs.T3CMPR = PWM_PERIOD;
    		break;
    	}
    }
}
#endif
/**
 * \brief 初始化pwm的GPIO
 */
#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_GMS)
static void __InitEPwmGpio (void)
{
	EALLOW;

//	GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6 = 1;
//	GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7 = 1;
	GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0  = 1;	//设置PWM1引脚
//	GpioMuxRegs.GPAMUX.bit.PWM2_GPIOA1  = 1;
	GpioMuxRegs.GPAMUX.bit.PWM3_GPIOA2  = 1;	//设置PWM3引脚
//	GpioMuxRegs.GPAMUX.bit.PWM4_GPIOA3  = 1;
//	GpioMuxRegs.GPAMUX.bit.PWM5_GPIOA4  = 1;
//	GpioMuxRegs.GPAMUX.bit.PWM6_GPIOA5  = 1;

    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 1;      /* 将GPIOA13配置为C1TRIP功能 */
    GpioMuxRegs.GPAMUX.bit.C2TRIP_GPIOA14 = 1;      /* 将GPIOA14配置为C2TRIP功能 */

//	GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6 = 1;
//	GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7 = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM7_GPIOB0  = 1;	//设置PWM7引脚
//	GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM9_GPIOB2  = 1;	//设置PWM9引脚
//	GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4  = 1;
//	GpioMuxRegs.GPBMUX.bit.PWM12_GPIOB5  = 1;
    EDIS;
}
#elif BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_DGM_2
static void __InitEPwmGpio (void)
{
    EALLOW;

    //控制四台电机，共八路PWM, 分别对应引脚组合（PWM1_GPIOA0、PWM3_GPIOA2）（PWM5_GPIOA4、T1PWM_GPIOA6）（PWM7_GPIOB0、PWM9_GPIOB2）（PWM11_GPIOB4、T3PWM_GPIOB6）
    //EVA
    GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0  = 1;    //设置PWM1引脚
//  GpioMuxRegs.GPAMUX.bit.PWM2_GPIOA1  = 1;
    GpioMuxRegs.GPAMUX.bit.PWM3_GPIOA2  = 1;    //设置PWM3引脚
//  GpioMuxRegs.GPAMUX.bit.PWM4_GPIOA3  = 1;

    GpioMuxRegs.GPAMUX.bit.PWM5_GPIOA4  = 1;
//  GpioMuxRegs.GPAMUX.bit.PWM6_GPIOA5  = 1;
    GpioMuxRegs.GPAMUX.bit.T1PWM_GPIOA6 = 1;
//  GpioMuxRegs.GPAMUX.bit.T2PWM_GPIOA7 = 1;

//    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 1;      /* 将GPIOA13配置为C1TRIP功能 */
//    GpioMuxRegs.GPAMUX.bit.C2TRIP_GPIOA14 = 1;      /* 将GPIOA14配置为C2TRIP功能 */
//    GpioMuxRegs.GPAMUX.bit.C3TRIP_GPIOA15 = 1;      /* 将GPIOA15配置为C3TRIP功能 */
//  GpioDataRegs.GPDTOGGLE.bit.GPIOD0 = 1;          /* 将GPIOD0配置为T1CTRIP_PDPINTA功能 */

    //EVB
    GpioMuxRegs.GPBMUX.bit.PWM7_GPIOB0  = 1;    //设置PWM7引脚
//  GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1  = 1;
    GpioMuxRegs.GPBMUX.bit.PWM9_GPIOB2  = 1;    //设置PWM9引脚
//  GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3  = 1;

    GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4  = 1;
//  GpioMuxRegs.GPBMUX.bit.PWM12_GPIOB5  = 1;
    GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6 = 1;
//  GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7 = 1;

    /* B12 ~ B15 用作多路通道选择器的地址信号，所以下面三行必须注释掉 */
//  GpioMuxRegs.GPBMUX.bit.C4TRIP_GPIOB13 = 1;      /* 将GPIOB13配置为C1TRIP功能 */
//  GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14 = 1;      /* 将GPIOB14配置为C2TRIP功能 */
//  GpioMuxRegs.GPBMUX.bit.C6TRIP_GPIOB15 = 1;      /* 将GPIOB15配置为C3TRIP功能 */

//  GpioDataRegs.GPDTOGGLE.bit.GPIOD5 = 1;          /* 将GPIOD5配置为T3TRIP_PDPINTB功能 */
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
		PieCtrlRegs.PIEIER2.bit.INTx6 = 0;	/* 关闭PWM中断 */
		//PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;
		StartCpuTimer1(); /* 开启角度采集 */
	}
#endif
	// 清除中断标志位
	EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // 清除PIE应答寄存器的第2位，以响应组2内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    __pwm_handler(g_Axis_Conf.p_pyb , 1);

//    GpioDataRegs.GPBDAT.bit.GPIOB2 = 0;
}
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
interrupt void Eva_T1_pwm_isr (void)
{
    // 清除中断标志位
    EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // 清除PIE应答寄存器的第2位，以响应组2内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    GpioDataRegs.GPFDAT.bit.GPIOF6 = 1;
    __pwm_handler(g_Axis_Conf.p_pya , 1);
    __pwm_handler(g_Axis_Conf.p_pyb , 3);
    GpioDataRegs.GPFDAT.bit.GPIOF6 = 0;
}

interrupt void Eva_T3_pwm_isr (void)
{
    // 清除中断标志位
    EvbRegs.EVBIFRA.bit.T3UFINT = 1;

    // 清除PIE应答寄存器的第4位，以响应组4内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;

    GpioDataRegs.GPFDAT.bit.GPIOF7 = 1;
    __pwm_handler(g_Axis_Conf.p_nya , 2);
    __pwm_handler(g_Axis_Conf.p_nyb , 4);
    GpioDataRegs.GPFDAT.bit.GPIOF7 = 0;
}
#elif BOARD_NAME == BOARD_DGM_2
interrupt void Eva_T1_pwm_isr (void)
{
    // 清除中断标志位
    EvaRegs.EVAIFRA.bit.T1UFINT = 1;

    // 清除PIE应答寄存器的第2位，以响应组2内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;

    __pwm_handler(g_Axis_Conf.p_pya , 1);
}

interrupt void Eva_T3_pwm_isr (void)
{
    // 清除中断标志位
    EvbRegs.EVBIFRA.bit.T3UFINT = 1;

    // 清除PIE应答寄存器的第4位，以响应组4内的其他中断请求；
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP4;

    __pwm_handler(g_Axis_Conf.p_nya , 2);
}
#endif

#if BOARD_NAME == BOARD_GMS
interrupt void pdpinta_isr(void)
{
    g_pdpinta_pyb_flag = 1;

    /* 如果不禁能，如果过流保护引脚一直为低，就会一直进中断 */
    EALLOW;
    EvaRegs.EVAIMRA.bit.PDPINTA = 0;      /* 禁能过流保护中断 */
    GpioMuxRegs.GPAMUX.bit.C1TRIP_GPIOA13 = 0;  /* 设置为普通IO */
    GpioMuxRegs.GPADIR.bit.GPIOA13 = 0;         /* 输入 */
    EDIS;

    /* 关闭PWM中断 */
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
    EvaRegs.EVAIMRA.bit.PDPINTA = 0;    /* 禁能过流保护中断 */
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;    /* 清除过流保护中断标志位 */

    EINT;
    ERTM;


    g_Axis_Conf.p_pyb->is_task_running = false;                       /* 为false表示当前轴未进行任务*/
    g_Axis_Conf.task_cnt = 0;



    /* 回零模式 */
    if (g_Axis_Conf.p_pyb->work_mode_old == WORK_MODE_RESET) {
        g_Tele_Dat.p_yb_axis_stat.bit.rcv_reset = 3;    /* 回零异常 */
    }

    g_Axis_Conf.p_pyb->work_mode_set = WORK_MODE_KEEP;                /* 默认保持模式*/
    g_Axis_Conf.p_pyb->work_mode_old = WORK_MODE_KEEP;
    g_Axis_Conf.p_pyb->cur_angle_speed = 0;                           /* 当前运动角速度为0*/
    g_Axis_Conf.p_pyb->next_segment_pulse_cnt = 0;                    /* 下个阶段的微步数清零*/


    PieCtrlRegs.PIEACK.all |= 0x001;    // PIE应答位置1，允许下次中断
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;      /* 过温保护中断标志位 */

    __g_Over_Cur_Tick = GetSysTick();
}
#endif

extern interrupt void EMPTY_ISR(void);
#if (BOARD_NAME == BOARD_GMS)
void InitEV_PWM(void)
{
	__InitEPwmGpio();					//初始化PWM引脚
	//Eva PWM1 && PWM3
	//Setup and load T1CON
    EALLOW;
	EvaRegs.T1CNT = 0x0000;				//计数器清零
	EvaRegs.T1PR  = PWM_PERIOD;			//T = 50us

	EvaRegs.T1CON.bit.TMODE   = 0x1;	//增减计数 ,周期50us
	EvaRegs.T1CON.bit.TPS     = 0x0;	//输入时钟预定标因子:T1CLK = HSPCLK / 1 = 45MHz
	EvaRegs.T1CON.bit.TCLKS10 = 0x0;	//使用内部时钟T1CLK
	EvaRegs.T1CON.bit.TENABLE = 1;		//使能T1计数

	EvaRegs.ACTRA.bit.CMP1ACT = 0x2;	//PWM1比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高

	//Initialize CMPRx
	EvaRegs.CMPR1 = PWM_PERIOD;			//初始PWM1/2占空比0%
	EvaRegs.CMPR2 = PWM_PERIOD;			//初始PWM3/4占空比0%

	//设置比较控制寄存器
	EvaRegs.EXTCONA.bit.INDCOE  = 1;	//设置比较输出单独控制
	EvaRegs.COMCONA.bit.CENABLE = 1;	//使能Eva比较单元的比较操作
//	EvaRegs.COMCONA.bit.FCOMPOE = 1;	//使能所有的比较输出 ,PWM1/2/3/4/5/6
	EvaRegs.COMCONA.bit.C1TRIPE = 1;	//使能C1TRIP. 当C1TRIP引脚为低电平时(外接18200过温引脚) ,FCMP1OE输出变为高组态
	EvaRegs.COMCONA.bit.C2TRIPE = 1;	//使能C2TRIP. 当C2TRIP引脚为低电平时(外接18200过温引脚) ,FCMP2OE输出变为高组态
	EvaRegs.COMCONA.bit.C3TRIPE = 0;	//禁止C3TRIP. 引脚C3TRIP不影响 FCMP3OE的输出以及PDPINTA标志位
	EvaRegs.COMCONA.bit.FCMP1OE = 1;	//使能PWM1/2的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出
	EvaRegs.COMCONA.bit.FCMP2OE = 1;	//使能PWM3/4的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出
	//死区定时器控制寄存器
	EvaRegs.DBTCONA.all = 0x0000; 		//禁止死区

#if 0
    EvaRegs.EXTCONA.bit.INDCOE  = 0;    /* 独立开启各比较模块 */
    EvaRegs.COMCONA.bit.FCOMPOE = 1;    /* 全比较器输出使能 */
#else
    /* 前面已经设置过了 */
//    EvaRegs.EXTCONA.bit.INDCOE  = 1;    /* 独立开启各比较模块 */
//    EvaRegs.COMCONA.bit.FCMP1OE = 1;
//    EvaRegs.COMCONA.bit.FCMP2OE = 1;

    EvaRegs.GPTCONA.bit.T1CMPOE = 1;
    EvaRegs.GPTCONA.bit.T2CMPOE = 1;

    EvaRegs.GPTCONA.bit.T1CTRIPE = 1; /* 当T1CTRIPE引脚变低时，禁止PWM输出 */
    EvaRegs.GPTCONA.bit.T2CTRIPE = 1; /* 当T2CTRIPE引脚变低时，禁止PWM输出 */

    /* 使能C1TRIP，当C1TRIP引脚为低电平时，PWM引脚变为高阻 */
//    EvaRegs.COMCONA.bit.C1TRIPE = 1;
//    EvaRegs.COMCONA.bit.C2TRIPE = 1;
#endif

	PieVectTable.CMP1INT = &EMPTY_ISR;       /* 保险起见 */

    /* 外设中断使能 */
	EvaRegs.EVAIMRA.bit.CMP1INT = 0;    //测试发现不写这一句会自动使能 ,导致频繁进中断

	/* PIE中断使能 */
	PieVectTable.T1UFINT = &Eva_T1_pwm_isr;
    PieVectTable.PDPINTA = &pdpinta_isr;

	EvaRegs.EVAIMRA.bit.T1UFINT = 1;	//定时器1下溢中断
	EvaRegs.EVAIFRA.bit.T1UFINT = 1;	//清除定时器1下溢中断标志位
    EvaRegs.EVAIFRA.bit.PDPINTA = 1;    /* 清除过流保护中断标志位 */
    EvaRegs.EVAIMRA.bit.PDPINTA = 1;    /* 使能过流保护中断 */

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
	PieCtrlRegs.PIEIER2.bit.INTx6 = 1;	//PIE组2 ,编号6 T1UFINT ,使能PIE中断

	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1=1;     /* 使能过温保护中断 */
	/* CPU INT2中断使能 */
	IER |= M_INT2;
    IER |= M_INT1;

    EDIS;
}
#elif (BOARD_NAME == BOARD_DGM_4) || (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_2)
void InitEV_PWM(void)
{
    __InitEPwmGpio();                   //初始化PWM引脚
    //Eva PWM1 && PWM3
    EALLOW;

    EvaRegs.T1CNT = 0x0000;             //计数器清零
    EvaRegs.T1PR  = PWM_PERIOD;         //T = 50us

    EvaRegs.T1CON.bit.TMODE   = 0x1;    //增减计数
    EvaRegs.T1CON.bit.TPS     = 0x0;    //输入时钟预定标因子:T1CLK = HSPCLK / 1 = 45MHz
    EvaRegs.T1CON.bit.TCLKS10 = 0x0;    //使用内部时钟
    EvaRegs.T1CON.bit.TCLD10  = 0x0;    //计数器为0时重载
    EvaRegs.T1CON.bit.TECMPR  = 1;      //使能定时器1的比较功能
    EvaRegs.T1CON.bit.TENABLE = 0;      //禁能T1计数，最后再使能


    EvaRegs.ACTRA.bit.CMP1ACT = 0x2;    //PWM1比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvaRegs.ACTRA.bit.CMP5ACT = 0x2;    //PWM5比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高

    EvaRegs.GPTCONA.bit.T1CMPOE = 1;    //定时器1比较输出使能
    EvaRegs.GPTCONA.bit.T1PIN = 0x2;    // 2高有效

    //Initialize CMPRx

    EvaRegs.CMPR1 = PWM_PERIOD;         //初始PWM1/2占空比0%
    EvaRegs.CMPR2 = PWM_PERIOD;         //初始PWM3/4占空比0%
    EvaRegs.CMPR3 = PWM_PERIOD;         //初始PWM5/6占空比0%
    EvaRegs.T1CMPR = PWM_PERIOD;        //初始化T1PWM占空比0%

    //设置比较控制寄存器
    EvaRegs.EXTCONA.bit.INDCOE  = 1;    //设置比较输出单独控制

    EvaRegs.COMCONA.bit.CENABLE = 1;    //使能Eva比较单元的比较操作
//  EvaRegs.COMCONA.bit.FCOMPOE = 1;    //使能所有的比较输出 ,PWM1/2/3/4/5/6
    EvaRegs.COMCONA.bit.C1TRIPE = 0;    //禁用TRIP引脚
    EvaRegs.COMCONA.bit.C2TRIPE = 0;
    EvaRegs.COMCONA.bit.C3TRIPE = 0;
    EvaRegs.COMCONA.bit.FCMP1OE = 1;    //使能PWM1/2的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出
    EvaRegs.COMCONA.bit.FCMP2OE = 1;    //使能PWM3/4的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出
    EvaRegs.COMCONA.bit.FCMP3OE = 1;    //使能PWM5/6的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出


    //死区定时器控制寄存器
    EvaRegs.DBTCONA.all = 0x0000;       //禁止死区

    EvaRegs.GPTCONA.bit.T1CMPOE  = 1;
    EvaRegs.GPTCONA.bit.T1CTRIPE = 0; /* 当T1CTRIPE引脚变低时，禁止PWM输出 */

    /* PIE中断使能 */
    PieVectTable.T1UFINT = &Eva_T1_pwm_isr;

    EvaRegs.EVAIFRA.bit.T1UFINT = 1;    //清除定时器1下溢中断标志位
    EvaRegs.EVAIMRA.bit.T1UFINT = 1;    //定时器1下溢中断

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx6 = 1;  //PIE组2 ,编号6 T1UFINT ,使能PIE中断

    /* CPU INT2中断使能 */
    IER |= M_INT2;

    EvaRegs.T1CON.bit.TENABLE = 1;      //使能T1计数


    EvbRegs.T3CNT = 0x0000;             //计数器清零
    EvbRegs.T3PR  = PWM_PERIOD;         //T = 50us

    EvbRegs.T3CON.bit.TMODE   = 0x1;    //增减计数
    EvbRegs.T3CON.bit.TPS     = 0x0;    //输入时钟预定标因子:T3CLK = HSPCLK / 1 = 45MHz
    EvbRegs.T3CON.bit.TCLKS10 = 0x0;    //使用内部时钟
    EvbRegs.T3CON.bit.TCLD10  = 0x0;    //计数器为0时重载
    EvbRegs.T3CON.bit.TECMPR  = 1;      //使能定时器1的比较功能
    EvbRegs.T3CON.bit.TENABLE = 0;      //禁能T3计数，最后再使能

    EvbRegs.ACTRB.bit.CMP7ACT  = 0x2;    //PWM1比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvbRegs.ACTRB.bit.CMP9ACT  = 0x2;    //PWM3比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvbRegs.ACTRB.bit.CMP11ACT = 0x2;    //PWM5比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高

    EvbRegs.GPTCONB.bit.T3CMPOE = 1;    //定时器1比较输出使能
    EvbRegs.GPTCONB.bit.T3PIN = 0x2;    // 2高有效

    //Initialize CMPRx
    EvbRegs.CMPR4 = PWM_PERIOD;         //初始PWM1/2占空比0%
    EvbRegs.CMPR5 = PWM_PERIOD;         //初始PWM3/4占空比0%
    EvbRegs.CMPR6 = PWM_PERIOD;         //初始PWM5/6占空比0%
    EvbRegs.T3CMPR = PWM_PERIOD;        //初始化T3PWM占空比0%

    //设置比较控制寄存器
    EvbRegs.EXTCONB.bit.INDCOE  = 1;    //设置比较输出单独控制

    EvbRegs.COMCONB.bit.CENABLE = 1;    //使能Eva比较单元的比较操作
//  EvbRegs.COMCONB.bit.FCOMPOE = 1;    //使能所有的比较输出 ,PWM1/2/3/4/5/6
    EvbRegs.COMCONB.bit.C4TRIPE = 0;    //禁用TRIP引脚
    EvbRegs.COMCONB.bit.C5TRIPE = 0;
    EvbRegs.COMCONB.bit.C6TRIPE = 0;
    EvbRegs.COMCONB.bit.FCMP4OE = 1;    //使能PWM1/2的比较输出 , EXTCONB[0]设置为1后才能单独控制比较输出
    EvbRegs.COMCONB.bit.FCMP5OE = 1;    //使能PWM3/4的比较输出 , EXTCONB[0]设置为1后才能单独控制比较输出
    EvbRegs.COMCONB.bit.FCMP6OE = 1;    //使能PWM5/6的比较输出 , EXTCONB[0]设置为1后才能单独控制比较输出

    //死区定时器控制寄存器
    EvbRegs.DBTCONB.all = 0x0000;       //禁止死区

    EvbRegs.GPTCONB.bit.T3CMPOE  = 1;
    EvbRegs.GPTCONB.bit.T3CTRIPE = 0; /* 当T3CTRIPE引脚变低时，禁止PWM输出 */

    /* PIE中断使能 */
    PieVectTable.T3UFINT = &Eva_T3_pwm_isr;

    EvbRegs.EVBIFRA.bit.T3UFINT = 1;    //清除定时器1下溢中断标志位
    EvbRegs.EVBIMRA.bit.T3UFINT = 1;    //定时器1下溢中断

    PieCtrlRegs.PIEACK.bit.ACK4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  //PIE组2 ,编号6 T3UFINT ,使能PIE中断

    /* CPU INT2中断使能 */
    IER |= M_INT4;

    EvbRegs.T3CON.bit.TENABLE = 1;      //使能T3计数

    EDIS;
}
#endif
#if 0
//#elif BOARD_NAME == BOARD_DGM_2
void InitEV_PWM(void)
{
    __InitEPwmGpio();                   //初始化PWM引脚
    //Eva PWM1 && PWM3
    EALLOW;

    EvaRegs.T1CNT = 0x0000;             //计数器清零
    EvaRegs.T1PR  = PWM_PERIOD;         //T = 50us

    EvaRegs.T1CON.bit.TMODE   = 0x1;    //增减计数
    EvaRegs.T1CON.bit.TPS     = 0x0;    //输入时钟预定标因子:T1CLK = HSPCLK / 1 = 45MHz
    EvaRegs.T1CON.bit.TCLKS10 = 0x0;    //使用内部时钟
    EvaRegs.T1CON.bit.TCLD10  = 0x0;    //计数器为0时重载
    EvaRegs.T1CON.bit.TECMPR  = 1;      //使能定时器1的比较功能
    EvaRegs.T1CON.bit.TENABLE = 0;      //禁能T1计数，最后再使能

    EvaRegs.ACTRA.bit.CMP1ACT = 0x2;    //PWM1比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvaRegs.ACTRA.bit.CMP3ACT = 0x2;    //PWM3比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高

    EvaRegs.CMPR1 = PWM_PERIOD;         //初始PWM1/2占空比0%
    EvaRegs.CMPR2 = PWM_PERIOD;         //初始PWM3/4占空比0%

    //设置比较控制寄存器
    EvaRegs.EXTCONA.bit.INDCOE  = 1;    //设置比较输出单独控制
    EvaRegs.COMCONA.bit.CENABLE = 1;    //使能Eva比较单元的比较操作
    EvaRegs.COMCONA.bit.FCMP1OE = 1;    //使能PWM1/2的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出
    EvaRegs.COMCONA.bit.FCMP2OE = 1;    //使能PWM3/4的比较输出 , EXTCONA[0]设置为1后才能单独控制比较输出

    //死区定时器控制寄存器
    EvaRegs.DBTCONA.all = 0x0000;       //禁止死区

    /* PIE中断使能 */
    PieVectTable.T1UFINT = &Eva_T1_pwm_isr;

    EvaRegs.EVAIFRA.bit.T1UFINT = 1;    //清除定时器1下溢中断标志位
    EvaRegs.EVAIMRA.bit.T1UFINT = 1;    //定时器1下溢中断

    PieCtrlRegs.PIEACK.bit.ACK2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx6 = 1;  //PIE组2 ,编号6 T1UFINT ,使能PIE中断

    /* CPU INT2中断使能 */
    IER |= M_INT2;
    EvaRegs.T1CON.bit.TENABLE = 1;      //使能T1计数


    EvbRegs.T3CNT = 0x0000;             //计数器清零
    EvbRegs.T3PR  = PWM_PERIOD;         //T = 50us

    EvbRegs.T3CON.bit.TMODE   = 0x1;    //增减计数
    EvbRegs.T3CON.bit.TPS     = 0x0;    //输入时钟预定标因子:T3CLK = HSPCLK / 1 = 45MHz
    EvbRegs.T3CON.bit.TCLKS10 = 0x0;    //使用内部时钟
    EvbRegs.T3CON.bit.TCLD10  = 0x0;    //计数器为0时重载
    EvbRegs.T3CON.bit.TECMPR  = 1;      //使能定时器1的比较功能
    EvbRegs.T3CON.bit.TENABLE = 0;      //禁能T3计数，最后再使能

    EvbRegs.ACTRB.bit.CMP7ACT  = 0x2;    //PWM1比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高
    EvbRegs.ACTRB.bit.CMP9ACT  = 0x2;    //PWM3比较输出引脚上的极性 ,1低有效 ,2高有效 ,0强制低 ,3强制高

    //Initialize CMPRx
    EvbRegs.CMPR4 = PWM_PERIOD;         //初始PWM1/2占空比0%
    EvbRegs.CMPR5 = PWM_PERIOD;         //初始PWM3/4占空比0%

    //设置比较控制寄存器
    EvbRegs.EXTCONB.bit.INDCOE  = 1;    //设置比较输出单独控制

    EvbRegs.COMCONB.bit.CENABLE = 1;    //使能Eva比较单元的比较操作
//  EvbRegs.COMCONB.bit.FCOMPOE = 1;    //使能所有的比较输出 ,PWM1/2/3/4/5/6
    EvbRegs.COMCONB.bit.FCMP4OE = 1;    //使能PWM1/2的比较输出 , EXTCONB[0]设置为1后才能单独控制比较输出
    EvbRegs.COMCONB.bit.FCMP5OE = 1;    //使能PWM3/4的比较输出 , EXTCONB[0]设置为1后才能单独控制比较输出

    //死区定时器控制寄存器
    EvbRegs.DBTCONB.all = 0x0000;       //禁止死区

    /* PIE中断使能 */
    PieVectTable.T3UFINT = &Eva_T3_pwm_isr;

    EvbRegs.EVBIFRA.bit.T3UFINT = 1;    //清除定时器1下溢中断标志位
    EvbRegs.EVBIMRA.bit.T3UFINT = 1;    //定时器1下溢中断

    PieCtrlRegs.PIEACK.bit.ACK4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  //PIE组2 ,编号6 T3UFINT ,使能PIE中断

    /* CPU INT2中断使能 */
    IER |= M_INT4;

    EvbRegs.T3CON.bit.TENABLE = 1;      //使能T3计数

    EDIS;
}
#endif

/**
 * \brief 过流保护任务
 */
void Over_Cur_Task (void)
{
    /* 电机过温或过流 */
#if BOARD_NAME == BOARD_DGM_2
    if (g_pdpinta_pya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB10) {     /* 未恢复 */
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pya_flag = 0;
        }
    } else {
        g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB8) {     /* 未恢复 */
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nya_flag = 0;
        }
    } else {
        g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    }
#elif (BOARD_NAME == BOARD_DGM_4 || BOARD_NAME == BOARD_CX20)
    if (g_pdpinta_pya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB10) {     /* 未恢复 */
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pya_flag = 0;
        }
    } else {
        g_Tele_Dat.p_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nya_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB8) {     /* 未恢复 */
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nya_flag = 0;
        }
    } else {
        g_Tele_Dat.n_ya_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_pyb_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB11) {     /* 未恢复 */
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pyb_flag = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    }

    if (g_pdpinta_nyb_flag == 1) {
        if (0 == GpioDataRegs.GPBDAT.bit.GPIOB9) {     /* 未恢复 */
            g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 1;
        } else {                                        /* 恢复了 */
            InitEV_PWM();
            g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_nyb_flag = 0;
        }
    } else {
        g_Tele_Dat.n_yb_axis_stat.bit.cur_fault_stat = 0;
    }
#elif BOARD_NAME == BOARD_GMS
    if (g_pdpinta_pyb_flag == 1) {
        if (GpioDataRegs.GPADAT.bit.GPIOA13 == 0) {         /* 未恢复 */
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 1;
            __g_Over_Cur_Tick = GetSysTick();
        } else if (ElapsedTick(__g_Over_Cur_Tick) >= TICK_1S) {  /* 恢复了，这步是为了防止抖动，多次进入过流保护中断 */
            InitEV_PWM();
            g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
            g_pdpinta_pyb_flag = 0;
        }
    } else {
        g_Tele_Dat.p_yb_axis_stat.bit.cur_fault_stat = 0;
    }
#endif
}

