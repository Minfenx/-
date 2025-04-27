/*
 * tele_dat.h
 *
 *  Created on: 2023��5��30��
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_TELE_DAT_H_
#define SOURCE_CMD_TELE_DAT_H_

#include "project.h"
#include "cmd.h"

#define ANGLE_SENSOR_SW_ON		0
#define ANGLE_SENSOR_SW_OFF		1

/**
 * \brief ����ͨ��״̬
 * @{
 */
struct comm_stat {
	uint8_t cmd_stat:1;				/* B0��ָ����״̬��0��ȷ��1���� */
	uint8_t dat_stat:1;				/* B1������״̬��0��ȷ��1���� */
	uint8_t checksum_stat:1;		/* B2��У���״̬��0��ȷ��1���� */
	uint8_t run_stat:2;				/* B4B3������״̬��11�ع���������ԭʼ���� */
#if BOARD_NAME == BOARD_GMS
	uint8_t angle_senser_sw:1;	/* B5���Ƕȴ���������״̬��0����1�� */
#elif BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	uint8_t re_stat:1;			/* B5���ع�״̬��0���У�1�ع��У� */
#endif
	uint8_t rev:2;					/* B7B6��Ԥ������ʱ��0�� */
};

typedef union {
	struct comm_stat bit;
	uint8_t all;
}comm_stat_t;
/**
 * @}
 */


/**
 * \brief ������״̬
 * @{
 */
#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
struct axis_stat {
	uint8_t drv_stat:1;			/* B0������״̬��0������·1�����1������·2����� */
	uint8_t mov_stat:1;			/* B1���˶�״̬��0��λ��1�˶��� */
	uint8_t cur_fault_stat:1;	/* B2������״̬��0������1������ */
	uint8_t temp_fault_stat:1;	/* B3������״̬��0������1���£� */
	uint8_t rev:2;				/* B7~B4��Ԥ������ʱ��0�� */
	uint8_t rcv_reset:2;        /* 00:δ�յ�����ָ�01�����ڻ��㣻10���Ѿ��ص�0λ��11�������쳣�� �յ�����ָ�� */
	uint8_t rev2:8;
};
#elif (BOARD_NAME == BOARD_GMS)
struct axis_stat {
	uint8_t hall_stat:1;		/* B0������״̬��0������1�쳣  */
	uint8_t mov_stat:1;			/* B1���˶�״̬��0��λ��1�˶��� */
	uint8_t cur_fault_stat:1;	/* B2������״̬��0������1���»������ */
	uint8_t angle_senser_stat:1;	/* B3���Ƕȴ�����״̬��0������1�����쳣�� */
	uint8_t lim_n:1;				/* B4�����︺��λλ�� */
	uint8_t lim_p:1;                /* B5����������λλ�� */
	uint8_t rcv_reset:2;            /* 00:δ�յ�����ָ�01�����ڻ��㣻10���Ѿ��ص�0λ��11�������쳣�� �յ�����ָ�� */
};
#endif

typedef union {
	struct axis_stat bit;
	uint8_t all;
} axis_stat_t;

#define SOFT_LIM_ERR_P  0x05    /* ��������λ */
#define SOFT_LIM_ERR_N  0x50    /* ��������λ */

/**
 * @}
 */

/** <\brief ����ң������֡���ݽṹ */
#if (BOARD_NAME == BOARD_CX20)
typedef struct {
	Uint16  frame_head;				/* B0~B1 ֡ͷ */
	Uint16  dat_len;				/* B2~B3 ���ݳ��� */
	uint8_t serve_type;				/* B4 �������� */
	uint8_t serve_state;			/* B5 ����״̬  */

	uint8_t	pri_bkup_id;			/* B6 ������ID */
	uint8_t tele_req_cnt;			/* B7 ң��������� */
	uint8_t right_mov_cnt;			/* B8 ��ȷ�˶�ָ��֡���� */
	uint8_t right_conf_cnt;			/* B9 ��ȷ����ָ��֡���� */
	uint8_t wrong_cnt;				/* B10 ����ָ��֡���� */
	uint8_t comm_rest_cnt;			/* B11 ͨѶ��λ���� */
	uint8_t sec_pow_24v;			/* B12 ���ε�Դ24V */
	uint8_t sec_pow_5v;				/* B13 ���ε�Դ5V */
	uint8_t sec_pow_1v2;			/* B14 ���ε�Դ1.2V */
	uint8_t py_res_base;			/* B15 +Y��λ�Ƽ�����׼ */
	uint8_t ny_res_base;			/* B16 -Y��λ�Ƽ�����׼ */
	uint8_t fst_gen_cur;			/* B17 һ��ĸ�ߵ���  */
	uint8_t pow_5v_cur;				/* B18 5V������� */
	uint8_t pow_3v3_cur;			/* B19 3.3V������� */
	uint8_t pow_1v2_cur;			/* B20 1.2V������� */
	uint8_t p_ya_axis_mot_cur;		/* B21 +Y-A�������� */
	uint8_t n_ya_axis_mot_cur;		/* B22 -Y-A�������� */
	uint8_t p_yb_axis_mot_cur;		/* B23 +Y-B�������� */
	uint8_t n_yb_axis_mot_cur;		/* B24 -Y-B�������� */
	uint8_t last_cmd;				/* B25 ���ִ��ָ���� */
	comm_stat_t comm_stat;			/* B26 ͨ��״̬ */
	uint8_t p_ya_axis_work_mode;	/* B27 +Y-A�Ṥ��ģʽ */
	axis_stat_t p_ya_axis_stat;		/* B28 +Y-A��״̬ */
	uint8_t n_ya_axis_work_mode;	/* B29 -Y-A�Ṥ��ģʽ */
	axis_stat_t n_ya_axis_stat;		/* B30 -Y-A��״̬ */
	uint8_t p_yb_axis_work_mode;	/* B31 +Y-B�Ṥ��ģʽ */
	axis_stat_t p_yb_axis_stat;		/* B32 +Y-B��״̬ */
	uint8_t n_yb_axis_work_mode;	/* B33 -Y-B�Ṥ��ģʽ */
	axis_stat_t n_yb_axis_stat;		/* B34 -Y-B��״̬ */

	Uint32 p_ya_axis_angle_res;		/* B35~B38 +Y-A��Ƕ�ֵ(��λ��) */
	Uint32 n_ya_axis_angle_res;		/* B39~B42 -Y-A��Ƕ�ֵ(��λ��) */
	Uint32 p_yb_axis_angle_res;		/* B43~B46 +Y-B��Ƕ�ֵ(��λ��) */
	Uint32 n_yb_axis_angle_res;		/* B47~B50 -Y-B��Ƕ�ֵ(��λ��) */

	Uint32 p_ya_axis_angle_mot;		/* B51~B54 +Y-A��Ƕ�ֵ(���) */
	Uint32 n_ya_axis_angle_mot;		/* B55~B58 -Y-A��Ƕ�ֵ(���) */
	Uint32 p_yb_axis_angle_mot;		/* B59~B62 +Y-B��Ƕ�ֵ(���) */
	Uint32 n_yb_axis_angle_mot;		/* B63~B66 -Y-B��Ƕ�ֵ(���) */

	int16 p_ya_axis_angle_v;		/* B67~B68 +Y-A�ᵱǰ���ٶ� */
	int16 n_ya_axis_angle_v;		/* B69~B70 -Y-A�ᵱǰ���ٶ� */
	int16 p_yb_axis_angle_v;		/* B71~B72 +Y-B�ᵱǰ���ٶ� */
	int16 n_yb_axis_angle_v;		/* B73~B74 -Y-B�ᵱǰ���ٶ� */

	Uint32 p_ya_axis_cmd_local;		/* B75~B78 +Y-A��ָ��λ�� */
	int16  p_ya_axis_cmd_speed;		/* B79~B80 +Y-A��ָ���ٶ� */
	Uint32 n_ya_axis_cmd_local;		/* B81~B84 -Y-A��ָ��λ�� */
	int16  n_ya_axis_cmd_speed;		/* B85~B86 -Y-A��ָ���ٶ� */
	Uint32 p_yb_axis_cmd_local;		/* B87~B90 +Y-B��ָ��λ�� */
	int16  p_yb_axis_cmd_speed;		/* B91~B92 +Y-B��ָ���ٶ� */
	Uint32 n_yb_axis_cmd_local;		/* B93~B96 -Y-B��ָ��λ�� */
	int16  n_yb_axis_cmd_speed;		/* B97~B98 -Y-B��ָ���ٶ� */

	uint8_t p_ya_axis_line_center_loc;	/* B99  +Y-A���߹��м�λ�� */
	uint8_t p_ya_axis_input_wall;		/* B100 +Y-A��������ϵ��� */
	uint8_t n_ya_axis_line_center_loc;	/* B101 -Y-A���߹��м�λ�� */
	uint8_t n_ya_axis_input_wall;		/* B102 -Y-A��������ϵ��� */

	uint8_t rev1;						/* B103 ���� */
	uint8_t edac_single_err_cnt;		/* B104 EDAC������� */
	uint8_t edac_double_err_cnt;		/* B105 EDAC˫����� */

	uint8_t rev3[5];					/* B106~B110 Ԥ�� */

	Uint16	check_sum;				/* B111~B112	У��� */
	Uint16	frame_tile;				/* B113~B114	֡β */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_DGM_2)
typedef struct {
    Uint16  frame_head;             /* B0~B1 ֡ͷ */
    Uint16  dat_len;                /* B2~B3 ���ݳ��� */
    uint8_t serve_type;             /* B4 �������� */
    uint8_t serve_state;            /* B5 ����״̬  */

    uint8_t pri_bkup_id;            /* B6 ������ID */
    uint8_t tele_req_cnt;           /* B7 ң��������� */
    uint8_t right_mov_cnt;          /* B8 ��ȷ�˶�ָ��֡���� */
    uint8_t right_conf_cnt;         /* B9 ��ȷ����ָ��֡���� */
    uint8_t wrong_cnt;              /* B10 ����ָ��֡���� */
    uint8_t comm_rest_cnt;          /* B11 ͨѶ��λ���� */

    uint8_t p_ya_max_mov_cur_set;   /* B12 +Y-A������˶��������� */
    uint8_t n_ya_max_mov_cur_set;   /* B13 -Y-A������˶��������� */

    uint8_t last_cmd;               /* B14 ���ִ��ָ���� */
    comm_stat_t comm_stat;          /* B15 ͨ��״̬ */
    uint8_t p_ya_axis_work_mode;    /* B16 +Y-A�Ṥ��ģʽ */
    axis_stat_t p_ya_axis_stat;     /* B17 +Y-A��״̬ */
    uint8_t n_ya_axis_work_mode;    /* B18 -Y-A�Ṥ��ģʽ */
    axis_stat_t n_ya_axis_stat;     /* B19 -Y-A��״̬ */

    Uint32 p_ya_axis_angle_res;     /* B20~B23 +Y-A��Ƕ�ֵ(��λ��) */
    Uint32 n_ya_axis_angle_res;     /* B24~B27 -Y-A��Ƕ�ֵ(��λ��) */

    Uint32 p_ya_axis_angle_mot;     /* B28~B31 +Y-A��Ƕ�ֵ(���) */
    Uint32 n_ya_axis_angle_mot;     /* B32~B35 -Y-A��Ƕ�ֵ(���) */

    int16 p_ya_axis_angle_v;        /* B36~B37 +Y-A�ᵱǰ���ٶ� */
    int16 n_ya_axis_angle_v;        /* B38~B39 -Y-A�ᵱǰ���ٶ� */

    Uint32 p_ya_axis_cmd_local;     /* B40~B43 +Y-A��ָ��λ�� */
    int16  p_ya_axis_cmd_speed;     /* B44~B45 +Y-A��ָ���ٶ� */
    Uint32 n_ya_axis_cmd_local;     /* B46~B49 +Y-A��ָ��λ�� */
    int16  n_ya_axis_cmd_speed;     /* B50~B51 +Y-A��ָ���ٶ� */

    uint8_t p_ya_axis_line_center_loc;  /* B52  +Y-A���߹��м�λ�� */
    uint8_t p_ya_axis_input_wall;       /* B53 +Y-A��������ϵ��� */
    uint8_t n_ya_axis_line_center_loc;  /* B54 -Y-A���߹��м�λ�� */
    uint8_t n_ya_axis_input_wall;       /* B55 -Y-A��������ϵ��� */

    uint8_t rev1;						/* B56 ���� */

    uint8_t edac_single_err_cnt;        /* B57 EDAC������� */
    uint8_t edac_double_err_cnt;        /* B58 EDAC˫����� */

    uint8_t p_ya_hold_cur_set;   	/* B59 +Y-A�ᱣ�ֵ������� */
    uint8_t n_ya_hold_cur_set;   	/* B60 -Y-A�ᱣ�ֵ������� */

    uint8_t p_ya_max_acc_set;		/* B61 +Y-A�����ٶ����� */
    uint8_t n_ya_max_acc_set;		/* B62 -Y-A�����ٶ����� */

    Uint16 p_ya_reset_speed_set;	/* B63~B64 +Y-A��λ�ٶ����� */
    Uint16 n_ya_reset_speed_set;	/* B65~B66 -Y-A��λ�ٶ����� */

    Uint16 p_ya_drv_cur_adc;         /* B67~B70 Ԥ�� */
    Uint16 n_ya_drv_cur_adc;

    Uint16  check_sum;              /* B71~B72    У��� */
    Uint16  frame_tile;             /* B73~B74    ֡β */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_DGM_4)
typedef struct {
    Uint16  frame_head;             /* B0~B1 ֡ͷ */
    Uint16  dat_len;                /* B2~B3 ���ݳ��� */
    uint8_t serve_type;             /* B4 �������� */
    uint8_t serve_state;            /* B5 ����״̬  */

    uint8_t pri_bkup_id;            /* B6 ������ID */
    uint8_t tele_req_cnt;           /* B7 ң��������� */
    uint8_t right_mov_cnt;          /* B8 ��ȷ�˶�ָ��֡���� */
    uint8_t right_conf_cnt;         /* B9 ��ȷ����ָ��֡���� */
    uint8_t wrong_cnt;              /* B10 ����ָ��֡���� */
    uint8_t comm_rest_cnt;          /* B11 ͨѶ��λ���� */

    uint8_t p_ya_max_mov_cur_set;   /* B12 +Y-A������˶��������� */
    uint8_t n_ya_max_mov_cur_set;   /* B13 -Y-A������˶��������� */
    uint8_t p_yb_max_mov_cur_set;   /* B14 +Y-B������˶��������� */
    uint8_t n_yb_max_mov_cur_set;   /* B15 -Y-B������˶��������� */

    uint8_t last_cmd;               /* B16 ���ִ��ָ���� */
    comm_stat_t comm_stat;          /* B17 ͨ��״̬ */
    uint8_t p_ya_axis_work_mode;    /* B18 +Y-A�Ṥ��ģʽ */
    axis_stat_t p_ya_axis_stat;     /* B19 +Y-A��״̬ */
    uint8_t n_ya_axis_work_mode;    /* B20 -Y-A�Ṥ��ģʽ */
    axis_stat_t n_ya_axis_stat;     /* B21 -Y-A��״̬ */
    uint8_t p_yb_axis_work_mode;    /* B22 +Y-B�Ṥ��ģʽ */
    axis_stat_t p_yb_axis_stat;     /* B23 +Y-B��״̬ */
    uint8_t n_yb_axis_work_mode;    /* B24 -Y-B�Ṥ��ģʽ */
    axis_stat_t n_yb_axis_stat;     /* B25 -Y-B��״̬ */

    int32 p_ya_axis_angle_res;     /* B26~B29 +Y-A��Ƕ�ֵ(��λ��) */
    int32 n_ya_axis_angle_res;     /* B30~B33 -Y-A��Ƕ�ֵ(��λ��) */
    int32 p_yb_axis_angle_res;     /* B34~B37 +Y-B��Ƕ�ֵ(��λ��) */
    int32 n_yb_axis_angle_res;     /* B38~B41 -Y-B��Ƕ�ֵ(��λ��) */

    int32 p_ya_axis_angle_mot;     /* B42~B45 +Y-A��Ƕ�ֵ(���) */
    int32 n_ya_axis_angle_mot;     /* B46~B49 -Y-A��Ƕ�ֵ(���) */
    int32 p_yb_axis_angle_mot;     /* B50~B53 +Y-B��Ƕ�ֵ(���) */
    int32 n_yb_axis_angle_mot;     /* B54~B57 -Y-B��Ƕ�ֵ(���) */

    int16 p_ya_axis_angle_v;        /* B58~B59 +Y-A�ᵱǰ���ٶ� */
    int16 n_ya_axis_angle_v;        /* B60~B61 -Y-A�ᵱǰ���ٶ� */
    int16 p_yb_axis_angle_v;        /* B62~B63 +Y-B�ᵱǰ���ٶ� */
    int16 n_yb_axis_angle_v;        /* B64~B65 -Y-B�ᵱǰ���ٶ� */

    int32 p_ya_axis_cmd_local;      /* B66~B69 +Y-A��ָ��λ�� */
    int16  p_ya_axis_cmd_speed;     /* B70~B71 +Y-A��ָ���ٶ� */
    int32 n_ya_axis_cmd_local;      /* B72~B75 -Y-A��ָ��λ�� */
    int16  n_ya_axis_cmd_speed;     /* B76~B77 -Y-A��ָ���ٶ� */
    int32 p_yb_axis_cmd_local;      /* B78~B81 +Y-B��ָ��λ�� */
    int16  p_yb_axis_cmd_speed;     /* B82~B83 +Y-B��ָ���ٶ� */
    int32 n_yb_axis_cmd_local;      /* B84~B87 -Y-B��ָ��λ�� */
    int16  n_yb_axis_cmd_speed;     /* B88~B89 -Y-B��ָ���ٶ� */

    uint8_t p_ya_axis_line_center_loc;  /* B90  +Y-A���߹��м�λ�� */
    uint8_t p_ya_axis_input_wall;       /* B91 +Y-A��������ϵ��� */
    uint8_t n_ya_axis_line_center_loc;  /* B92 -Y-A���߹��м�λ�� */
    uint8_t n_ya_axis_input_wall;       /* B93 -Y-A��������ϵ��� */

    uint8_t p_yb_temp;                  /* B94 +Y-B�¶Ȳɼ� */
    uint8_t n_yb_temp;                  /* B95 -Y-B�¶Ȳɼ� */
    uint8_t edac_single_err_cnt;        /* B96 EDAC������� */
    uint8_t edac_double_err_cnt;        /* B97 EDAC˫����� */

    uint8_t rev3[4];                    /* B98~B101 Ԥ�� */

    Uint16  check_sum;              /* B102~B103    У��� */
    Uint16  frame_tile;             /* B104~B105    ֡β */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_GMS)

/**
 * \brief ����������е�ַ
 */
#define PROGRAM_RUN_ADDR_LOW        0x11    /* �͵�ַ */
#define PROGRAM_RUN_ADDR_HIGH       0x22    /* �ߵ�ַ */
#define PROGRAM_RUN_ADDR_UNKNOW     0x33    /* δ֪��ַ */

/**
 * \brief �����ϴθ�λԭ��
 */
#define LAST_RESET_REASON_POWER_ON       0x11    /* �ϵ縴λ */
#define LAST_RESET_REASON_WDT            0x22    /* ���Ź���λ */
#define LAST_RESET_REASON_SERIAL_ERR     0x33    /* �����쳣������λ */

typedef struct {
    Uint16  frame_head;             /* B0~B1 ֡ͷ */
    Uint16  dat_len;                /* B2~B3 ���ݳ��� */
    uint8_t serve_type;             /* B4 �������� */
    uint8_t serve_state;            /* B5 ����״̬  */
    uint8_t pri_bkup_id;            /* B6 ������ID */
    uint8_t tele_req_cnt;           /* B7 ң��������� */
    uint8_t right_mov_cnt;          /* B8 ��ȷ�˶�ָ��֡���� */
    uint8_t right_conf_cnt;         /* B9 ��ȷ����ָ��֡���� */
    uint8_t wrong_cnt;              /* B10 ����ָ��֡���� */
    uint8_t comm_rest_cnt;          /* B11 ͨѶ��λ���� */
    uint8_t last_cmd;               /* B12 ���ִ��ָ���� */
    comm_stat_t comm_stat;          /* B13 ͨ��״̬ */
    uint8_t p_yb_axis_work_mode;    /* B14 +Y-B�Ṥ��ģʽ */
    axis_stat_t p_yb_axis_stat;     /* B15 +Y-B��״̬ */
    int32 p_yb_axis_angle_res;      /* B16~B19 +Y-B��Ƕ�ֵ(��λ�ƴ�����) */
    int32 p_yb_axis_angle_mot;      /* B20~B23 +Y-B��Ƕ�ֵ(���) */
    int16 p_yb_axis_angle_v;        /* B24~B25 +Y-B�ᵱǰ���ٶ� */
    int32 p_yb_axis_cmd_local;      /* B26~B29 +Y-B��ָ��λ�� */
    int16  p_yb_axis_cmd_speed;     /* B30~B31 +Y-B��ָ���ٶ� */
    uint8_t p_yb_cmd_work_mode;     /* B32 +YB��ָ���ģʽ */
    uint8_t p_yb_max_acc_set;       /* B33 +YB�������ٶ����� */
    uint8_t p_yb_drv_cur_set;       /* B34 +YB�������������� */
    uint8_t p_yb_keep_cur_set;      /* B35 +YB�ᱣ�ֵ������� */
    int32   p_yb_soft_lim_p;        /* B36~B39 +YB�������λ���ã������� */
    int32   p_yb_soft_lim_n;        /* B40~B43 +YB�������λ���ã������� */
    Uint16  p_yb_reset_speed_set;   /* B44~B45 +YB������ٶ����� */
    uint8_t cur_5v;                 /* B46 5V����ң�� */

//    Uint16  temp_senser_vol;        /* B47~B48 �¶ȴ����������ѹ */
//    Uint16  temperature;            /* B49~B50 �¶�ֵ */
    uint8_t program_run_addr;       /* B47 ��ǰ�������е�ַ�� */
    uint8_t last_reset_reason;      /* B48 �ϴθ�λԭ�� */
    Uint16 system_reset_times;      /* B49~B50 ϵͳ��λ���� */

    Uint16  drv_cur_adc;            /* B51~B52 ����������ADC����ֵ */
    int16  hall_angle;              /* B53~B54 �����Ƕ� */
    uint8_t program_version;        /* B55 ����汾�� */
    Uint32  program_err_bits;       /* B56~B59 ����У�����bit���� */
    Uint16  program_run_region;     /* B60 ��ǰ���������� */

    uint8_t pps_cnt;                /* B61 PPS����ֵ */
    uint8_t soft_lim_err_stat;      /* B62 �����λ����״̬ */

    Uint16  check_sum;              /* B63~B64    У��� */
    Uint16  frame_tile;             /* B65~B66    ֡β */
} tele_dat_t;
#endif

#define RUN_REGION_MAIN             0xAA    /* �������� */
#define RUN_REGION_BACKUP           0xBB    /* �������� */
#define RUN_REGION_RAM              0xCC    /* RAM������ */


extern tele_dat_t g_Tele_Dat;  /**< \brief ң������ */

/**
 * \brief ִ��ң������ָ��
 *
 * \retval �ɹ�����0��ʧ�ܷ���-1
 */
extern int16 tele_req_cmd_run (void);


/**
 * \brief ��ʼ��ң������
 */
extern void InitTeleDat (void);


#endif /* SOURCE_CMD_TELE_DAT_H_ */
