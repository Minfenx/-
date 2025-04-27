/*
 * Axis_cmd.h
 *
 *  Created on: 2024��1��18��
 *      Author: 86132
 */

#ifndef APP_INCLUDE_AXIS_CMD_H_
#define APP_INCLUDE_AXIS_CMD_H_

#include "project.h"				//ȫ�ֺ�



/**
 * \brief �����������
 *
 * @{
 */
#define CMD_SERVE_TYPE_AB_AXIS_MOV_CTL				0x11	/* AB���˶�����ָ�� */
#define CMD_SERVE_TYPE_MAX_ACC_CONF					0x22	/* �����ٶ�����ָ�� */
#define CMD_SERVE_TYPE_REST_SPEED_CONF				0x33	/* ��λ�ٶ�����ָ�� */
#define CMD_SERVE_TYPE_MOT_CUR_CONF					0x44	/* �����������ָ�� */
#define CMD_SERVE_TYPE_MOT_OVERCUR_PROT_CONF		0x55	/* ���������������ָ�� */
#define CMD_SERVE_TYPE_CUR_LOCAL_CONF				0x66	/* ��ǰλ������ָ�� */
#define CMD_SERVE_TYPE_SADMB_SOFT_LIM_CONF			0x88	/* SADM-B�����λ����ָ�� */
#define CMD_SERVE_TYPE_SADM_DRV_HOT_BKUP_CONF       0x77    /* SADM�����ȱ�����ָ�� */
#define CMD_SERVE_TYPE_ANGLE_POW_SW_CONF            0xAA    /* �Ƕȴ�������������ָ�� */
#define CMD_SERVE_TYPE_RESET_TIMES_CLEAR            0xBB    /* ��λ��������ָ�� */

#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_GMS)
#define CMD_SERVE_TYPE_TELE_REQ                     0x01    /* ң������ָ�� */
#elif (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
#define CMD_SERVE_TYPE_TELE_REQ	  	  				0xAA	/* ң������ָ�� */
#endif
#define CMD_SERVE_TYPE_RECONSTITUTION  	  			0x08	/* �ع�ָ�� */
/**
 * @}
 */

/**
 * \brief ���峣��֡������
 *
 * @{
 */
#define DAT_FRAME_HEAD	0xEB90		/* ֡ͷ */
#define DAT_FRAME_TILE	0x09D7		/* ֡β */
/**
 * @}
 */

/**
 * \brief ���幤��ģʽ
 *
 * @{
 */
#define WORK_MODE_TRACE     0x11     /* ����ģʽ */
#define WORK_MODE_LAUNCH    0x11     /* չ��ģʽ */
#define WORK_MODE_CAPTURE   0x22     /* ����ģʽ */
#define WORK_MODE_RESET     0x33     /* ��λģʽ */
#define WORK_MODE_KEEP      0x44     /* ����ģʽ */
#define WORK_MODE_STANDBY   0x99     /* ����ģʽ */
/**
 * @}
 */

/**
 * \brief ������������������ָ��
 *
 * @{
 */
#define OVER_CUR_PROT_SW_OFF     0x55    /* ���������� */
#define OVER_CUR_PROT_SW_ON      0xAA    /* ���������� */
/**
 * @}
 */

/**
 * \brief ����дIO�ĺ���ָ��
 */
typedef void (*pfn_write_io)(uint8_t val);
/**
 * \brief �����IO�ĺ���ָ��
 */
typedef Uint16 (*pfn_read_io)(void);

/**
 * \brief ����ÿ���׶�����Ҫ�������Ļ����������� x = v0t + (1/2)at^2 ����ÿ�������ʱ��
 *
 * \note ����Ǹ���ģʽ�����ٽ׶Σ���all_pulse_cnt��ֵΪ(!0)
 */
typedef struct {
	float32 all_step_cnt;			/* ������ */
	Uint32  loc_diff;			    /* ���߹���λ�� */
	Uint32  all_segment;			/* ����С�εĸ�������������ϸ��������С���� = ��ǰ�׶��ܵ�΢���� */
	Uint32  elapsed_segment_cnt;	/* �Ѿ���ȥ��С���� = �Ѿ��߹���΢���� */
	int32   v0_1000;				/* ��ǰ�׶εĳ�ʼ�ٶȣ�1000�� */
	int32   a_1000;					/* ��ǰ�׶εļ��ٶȣ�1000�� */
	float32 t;						/* ��¼��ǰ�׶ε�ʱ�� */
	int8_t 	mov_dir;				/* �˶�����Ϊ1��ʾ����Ϊ-1��ʾ���� */
} stage_var_t;


/**
 * \brief ����ÿ��������ýṹ
 *
 * @{
 */
typedef struct single_axis_conf {
	char name[4];					         /* ��ǰ������� */
	struct single_axis_conf* p_next_axis;    /* �¸���ĵ�ַ */

    uint8_t work_mode_set;          /* �趨���¹���ģʽ */
    uint8_t new_task_flag;          /* ��Ҫ����������ı�־ */
    int8_t reciprocat_mode_enable;	/* ����ģʽʹ�ܣ������жϵ�ǰ�ǲ���ģʽ��������ģʽ */
    int32   loc_given;              /* λ�ø���������ģʽ��Ч */
    int16   speed_given;            /* �ٶȸ���������ģʽ������ģʽ��Ч */
    float32 hall_angle;				/* �����Ƕ� */
    uint8_t max_acc;                /* �����ٶ� */
    Uint16  reset_speed_given;      /* ��λ�ٶȸ��� */
    Uint16  max_mov_cur_given;      /* ����˶��������� */
    Uint16  keep_cur_given;         /* ���ֵ������� */
#if 0   /* ����������2812���治��Ҫ */
    uint8_t overcur_prot_sw;        /* ������������ */
    uint8_t overcur_prot_th;        /* ����������ֵ */
#endif
    int32   cur_local;                /* ��ǰλ�ã�����������һ��λ�õ���0.000225�� */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
    int32   soft_lim;               /* �����λλ�� */
#elif BOARD_NAME == BOARD_GMS
    int32   soft_lim_p;               /* �����λλ�ã����� */
    int32   soft_lim_n;               /* �����λλ�ã����� */
#endif
//    uint8_t drv_conf;               /* �������ã�0x55��������·1��0xAA��������·2  */

    /**< \brief �������ݸ���ʵ��������� */
    float32 reset_local;            /* ��λλ�� */
    uint8_t work_mode_old;          /* �ɹ���ģʽ */
    Uint16  speed_ratio;            /* ת�ٱ� */
    Uint16  pulse_cnt_per_step;     /* ÿ1���������������� */
    float32 step_angle;             /* ����� */
    _iq23   step_angle_out_iq23;    /* ����Ჽ��� ���磺 1.8 / (32 * 125) */
    _iq30   step_angle_iq;

    float32 loc_to_angle_factor;    /* ��λ��ֵת�����Ƕ�ֵ��Ҫ���ϵ�ϵ�� */
    float32 speedvalue_to_speed;	/* ���ٶȸ���ֵת�����ٶ���Ҫ���ϵ�ϵ�� */

    int32   min_loc;                /* ��Сλ��ֵ */
    int32   max_loc;                /* ���λ��ֵ */

#if 0   /* ��2�����ò��� */
    float32 min_angle;              /* ��С�Ƕ�ֵ */
    float32 max_angle;              /* ���Ƕ�ֵ */
#endif
    int16   min_speed;				/* ��С�ٶ�ֵ */
    int16   max_speed;				/* ����ٶ�ֵ */

    float32 duty_factor_mov;		  	    /* ��ÿ��ռ�ձ��ϳ��ϵ�ϵ�����ı����ʱ�ã��˶����� */
    float32 duty_factor_hold;		  	    /* ��ÿ��ռ�ձ��ϳ��ϵ�ϵ�����ı����ʱ�ã����ֵ��� */
    Uint16  pulse_tbprd_mov[XIFEN_CNT+1];   /* 2812�д��ݸ��ȽϼĴ�������ֵ , CMP1 ,CMP2 */
    Uint16  pulse_tbprd_hold[XIFEN_CNT+1];   /* 2812�д��ݸ��ȽϼĴ�������ֵ , CMP1 ,CMP2 */

    /* �������ʱ��Ҫ�ı��� */
    uint8_t is_task_running;        	        /* Ϊtrue��ʾ��ǰ�����ڽ������� */
    Uint32  next_segment_pulse_cnt;		        /* ��һ��΢���е�pwm�������� */
    Uint32  uniform_segment_pulse_cnt_zheng;    /* �ﵽ�����˶�ʱÿ��΢���е�pwm�����������������֣� */
    Uint32  uniform_segment_pulse_cnt_xiao_1000;    /* �ﵽ�����˶�ʱÿ��΢���е�pwm����������С�����ֵ�1000���� */
    Uint32  uniform_segment_pulse_cnt_xiao_sum;     /* �ۼ�ֵ */
    int8_t  step_temp;

    float64 cur_angle_speed;        	/* ��ǰ�˶����ٶ� */
    uint8_t period_cnt_1_4;             /* ��ǰ������1/4������ 0~3 */
    Uint16  cur_xifen_pulse_num;        /* ��ǰ΢���ڵ������ź� */
    int16   cur_xifen_num;              /* ��ǰ������΢����� ,��һ��������ϸ�������  0~32*/
    uint8_t tail_flag;                  /* ���������־ */
    uint8_t reverse_flag;				/* ��Ҫ�����˶���־ */
    float64 t2_us;						/* ����ʱ��ʱ��Ҫ */
    stage_var_t stage[5];		        /* һ����������Ľ׶� */
    uint8_t 	stage_cnt;		        /* ��Ҫִ�еĽ׶����� */
    uint8_t   	cur_stage;              /* ��ǰ�˶��׶�  */

    /* ���ƺ���ָ�� */
    pfn_write_io fn_write_a_dir;     /* ����a��������� , д1���� Ϊ�� ,0���� Ϊ�� */
    pfn_write_io fn_write_b_dir;     /* ����b��������� , д1���� Ϊ�� ,0���� Ϊ��*/
    pfn_write_io fn_write_brake;     /* ����ɲ���źţ�A���B��һ�� */
} single_axis_conf_t;
/**
 * @}
 */


/**
 * \brief ����4���������
 *
 * \note ��������ʹ��4����ĵ�ַ��������ṹ��Ҫ�޸ģ���Ҫ��֤4����Ķ����м䲻�ܲ����ı�������
 * @{
 */
typedef struct {
    uint8_t new_task_req;        /* ���������󣨽��յ����˶�����ָ� */
    uint8_t task_cnt;              /* ��¼������������������ */
    Uint32  cmd_pps;               /* ���յ�����ʱ��PPSֵ */
    uint8_t run_delay;             /* ִ��״̬������ָ����ʱ����PPSΪ��׼����λ��s */
    float32 duty[XIFEN_CNT+1];     /* һ������1/4���ڣ�ÿһϸ������ռ�ձȣ���ϸ������һ���� ,��Ҫ��¼ռ�ձ�Ϊ0��Ϊ1��������Ա�ϸ������һ�� */
    single_axis_conf_t* p_pya;     /* Ԥ��*/
    single_axis_conf_t* p_nya;     /* Ԥ�� */
    single_axis_conf_t* p_pyb;     /* ������ */
    single_axis_conf_t* p_nyb;     /* ������ */
} axis_conf_t;
/**
 * @}
 */

/** <\brief ����4��������ã������յ�����ָ������ݴ������������� */
extern axis_conf_t g_Axis_Conf;

/* �����жϴ����Ƿ�ʱδ���յ����� */
extern Uint64 g_RS422_DATA_Tick;

/* �����������ӵĴ��� */
extern Uint16 g_RS422_relink_times;

/**
 * \brief ��ʼ�������������
 */
extern void Init_Axis_Conf(void);
/**
 * \brief �õ���1/4�����ڵ�����ռ�ձ�ֵ
 *
 * \param[in]  N��1/4�������ڰ�����ϸ����
 * \param[out] p_duty��������ռ�ձ���ֵ
 *
 * \note sin(2*pi*t/T)��ͼ��
 */
extern void get_all_duty (Uint16 N_4, float32* p_duty);

/**
 * \brief ����У���
 *
 * \param[in] pData��Ҫ���������
 * \param[in] uiLen��Ҫ��������ݳ���
 *
 * \retval ����У���
 */
extern Uint16 CheckSumCalc (const Uint16 *pData, Uint16 uiLen);


#endif /* APP_INCLUDE_AXIS_CMD_H_ */
