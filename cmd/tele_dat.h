/*
 * tele_dat.h
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#ifndef SOURCE_CMD_TELE_DAT_H_
#define SOURCE_CMD_TELE_DAT_H_

#include "project.h"
#include "cmd.h"

#define ANGLE_SENSOR_SW_ON		0
#define ANGLE_SENSOR_SW_OFF		1

/**
 * \brief 定义通信状态
 * @{
 */
struct comm_stat {
	uint8_t cmd_stat:1;				/* B0：指令码状态，0正确，1错误； */
	uint8_t dat_stat:1;				/* B1：数据状态，0正确，1错误； */
	uint8_t checksum_stat:1;		/* B2：校验和状态，0正确，1错误； */
	uint8_t run_stat:2;				/* B4B3：运行状态，11重构正序，其它原始程序； */
#if BOARD_NAME == BOARD_GMS
	uint8_t angle_senser_sw:1;	/* B5：角度传感器开关状态，0开，1关 */
#elif BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
	uint8_t re_stat:1;			/* B5：重构状态，0空闲，1重构中； */
#endif
	uint8_t rev:2;					/* B7B6：预留，暂时填0。 */
};

typedef union {
	struct comm_stat bit;
	uint8_t all;
}comm_stat_t;
/**
 * @}
 */


/**
 * \brief 定义轴状态
 * @{
 */
#if (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
struct axis_stat {
	uint8_t drv_stat:1;			/* B0：驱动状态，0驱动线路1输出，1驱动线路2输出； */
	uint8_t mov_stat:1;			/* B1：运动状态，0到位，1运动； */
	uint8_t cur_fault_stat:1;	/* B2：故障状态，0正常，1过流； */
	uint8_t temp_fault_stat:1;	/* B3：故障状态，0正常，1过温； */
	uint8_t rev:2;				/* B7~B4：预留，暂时填0。 */
	uint8_t rcv_reset:2;        /* 00:未收到回零指令，01：正在回零；10：已经回到0位；11：回零异常。 收到回零指令 */
	uint8_t rev2:8;
};
#elif (BOARD_NAME == BOARD_GMS)
struct axis_stat {
	uint8_t hall_stat:1;		/* B0：霍尔状态，0正常，1异常  */
	uint8_t mov_stat:1;			/* B1：运动状态，0到位，1运动； */
	uint8_t cur_fault_stat:1;	/* B2：故障状态，0正常，1过温或过流； */
	uint8_t angle_senser_stat:1;	/* B3：角度传感器状态，0正常，1数据异常； */
	uint8_t lim_n:1;				/* B4：到达负限位位置 */
	uint8_t lim_p:1;                /* B5：到达正限位位置 */
	uint8_t rcv_reset:2;            /* 00:未收到回零指令，01：正在回零；10：已经回到0位；11：回零异常。 收到回零指令 */
};
#endif

typedef union {
	struct axis_stat bit;
	uint8_t all;
} axis_stat_t;

#define SOFT_LIM_ERR_P  0x05    /* 超出正限位 */
#define SOFT_LIM_ERR_N  0x50    /* 超出负限位 */

/**
 * @}
 */

/** <\brief 定义遥测数据帧数据结构 */
#if (BOARD_NAME == BOARD_CX20)
typedef struct {
	Uint16  frame_head;				/* B0~B1 帧头 */
	Uint16  dat_len;				/* B2~B3 数据长度 */
	uint8_t serve_type;				/* B4 服务类型 */
	uint8_t serve_state;			/* B5 服务状态  */

	uint8_t	pri_bkup_id;			/* B6 主备份ID */
	uint8_t tele_req_cnt;			/* B7 遥测请求计数 */
	uint8_t right_mov_cnt;			/* B8 正确运动指令帧计数 */
	uint8_t right_conf_cnt;			/* B9 正确配置指令帧计数 */
	uint8_t wrong_cnt;				/* B10 错误指令帧计数 */
	uint8_t comm_rest_cnt;			/* B11 通讯复位计数 */
	uint8_t sec_pow_24v;			/* B12 二次电源24V */
	uint8_t sec_pow_5v;				/* B13 二次电源5V */
	uint8_t sec_pow_1v2;			/* B14 二次电源1.2V */
	uint8_t py_res_base;			/* B15 +Y电位计激励基准 */
	uint8_t ny_res_base;			/* B16 -Y电位计激励基准 */
	uint8_t fst_gen_cur;			/* B17 一次母线电流  */
	uint8_t pow_5v_cur;				/* B18 5V供电电流 */
	uint8_t pow_3v3_cur;			/* B19 3.3V供电电流 */
	uint8_t pow_1v2_cur;			/* B20 1.2V供电电流 */
	uint8_t p_ya_axis_mot_cur;		/* B21 +Y-A轴电机电流 */
	uint8_t n_ya_axis_mot_cur;		/* B22 -Y-A轴电机电流 */
	uint8_t p_yb_axis_mot_cur;		/* B23 +Y-B轴电机电流 */
	uint8_t n_yb_axis_mot_cur;		/* B24 -Y-B轴电机电流 */
	uint8_t last_cmd;				/* B25 最近执行指令码 */
	comm_stat_t comm_stat;			/* B26 通信状态 */
	uint8_t p_ya_axis_work_mode;	/* B27 +Y-A轴工作模式 */
	axis_stat_t p_ya_axis_stat;		/* B28 +Y-A轴状态 */
	uint8_t n_ya_axis_work_mode;	/* B29 -Y-A轴工作模式 */
	axis_stat_t n_ya_axis_stat;		/* B30 -Y-A轴状态 */
	uint8_t p_yb_axis_work_mode;	/* B31 +Y-B轴工作模式 */
	axis_stat_t p_yb_axis_stat;		/* B32 +Y-B轴状态 */
	uint8_t n_yb_axis_work_mode;	/* B33 -Y-B轴工作模式 */
	axis_stat_t n_yb_axis_stat;		/* B34 -Y-B轴状态 */

	Uint32 p_ya_axis_angle_res;		/* B35~B38 +Y-A轴角度值(电位计) */
	Uint32 n_ya_axis_angle_res;		/* B39~B42 -Y-A轴角度值(电位计) */
	Uint32 p_yb_axis_angle_res;		/* B43~B46 +Y-B轴角度值(电位计) */
	Uint32 n_yb_axis_angle_res;		/* B47~B50 -Y-B轴角度值(电位计) */

	Uint32 p_ya_axis_angle_mot;		/* B51~B54 +Y-A轴角度值(电机) */
	Uint32 n_ya_axis_angle_mot;		/* B55~B58 -Y-A轴角度值(电机) */
	Uint32 p_yb_axis_angle_mot;		/* B59~B62 +Y-B轴角度值(电机) */
	Uint32 n_yb_axis_angle_mot;		/* B63~B66 -Y-B轴角度值(电机) */

	int16 p_ya_axis_angle_v;		/* B67~B68 +Y-A轴当前角速度 */
	int16 n_ya_axis_angle_v;		/* B69~B70 -Y-A轴当前角速度 */
	int16 p_yb_axis_angle_v;		/* B71~B72 +Y-B轴当前角速度 */
	int16 n_yb_axis_angle_v;		/* B73~B74 -Y-B轴当前角速度 */

	Uint32 p_ya_axis_cmd_local;		/* B75~B78 +Y-A轴指令位置 */
	int16  p_ya_axis_cmd_speed;		/* B79~B80 +Y-A轴指令速度 */
	Uint32 n_ya_axis_cmd_local;		/* B81~B84 -Y-A轴指令位置 */
	int16  n_ya_axis_cmd_speed;		/* B85~B86 -Y-A轴指令速度 */
	Uint32 p_yb_axis_cmd_local;		/* B87~B90 +Y-B轴指令位置 */
	int16  p_yb_axis_cmd_speed;		/* B91~B92 +Y-B轴指令速度 */
	Uint32 n_yb_axis_cmd_local;		/* B93~B96 -Y-B轴指令位置 */
	int16  n_yb_axis_cmd_speed;		/* B97~B98 -Y-B轴指令速度 */

	uint8_t p_ya_axis_line_center_loc;	/* B99  +Y-A轴线管中间位置 */
	uint8_t p_ya_axis_input_wall;		/* B100 +Y-A轴输入轴系外壁 */
	uint8_t n_ya_axis_line_center_loc;	/* B101 -Y-A轴线管中间位置 */
	uint8_t n_ya_axis_input_wall;		/* B102 -Y-A轴输入轴系外壁 */

	uint8_t rev1;						/* B103 备用 */
	uint8_t edac_single_err_cnt;		/* B104 EDAC单错计数 */
	uint8_t edac_double_err_cnt;		/* B105 EDAC双错计数 */

	uint8_t rev3[5];					/* B106~B110 预留 */

	Uint16	check_sum;				/* B111~B112	校验和 */
	Uint16	frame_tile;				/* B113~B114	帧尾 */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_DGM_2)
typedef struct {
    Uint16  frame_head;             /* B0~B1 帧头 */
    Uint16  dat_len;                /* B2~B3 数据长度 */
    uint8_t serve_type;             /* B4 服务类型 */
    uint8_t serve_state;            /* B5 服务状态  */

    uint8_t pri_bkup_id;            /* B6 主备份ID */
    uint8_t tele_req_cnt;           /* B7 遥测请求计数 */
    uint8_t right_mov_cnt;          /* B8 正确运动指令帧计数 */
    uint8_t right_conf_cnt;         /* B9 正确配置指令帧计数 */
    uint8_t wrong_cnt;              /* B10 错误指令帧计数 */
    uint8_t comm_rest_cnt;          /* B11 通讯复位计数 */

    uint8_t p_ya_max_mov_cur_set;   /* B12 +Y-A轴最大运动电流设置 */
    uint8_t n_ya_max_mov_cur_set;   /* B13 -Y-A轴最大运动电流设置 */

    uint8_t last_cmd;               /* B14 最近执行指令码 */
    comm_stat_t comm_stat;          /* B15 通信状态 */
    uint8_t p_ya_axis_work_mode;    /* B16 +Y-A轴工作模式 */
    axis_stat_t p_ya_axis_stat;     /* B17 +Y-A轴状态 */
    uint8_t n_ya_axis_work_mode;    /* B18 -Y-A轴工作模式 */
    axis_stat_t n_ya_axis_stat;     /* B19 -Y-A轴状态 */

    Uint32 p_ya_axis_angle_res;     /* B20~B23 +Y-A轴角度值(电位计) */
    Uint32 n_ya_axis_angle_res;     /* B24~B27 -Y-A轴角度值(电位计) */

    Uint32 p_ya_axis_angle_mot;     /* B28~B31 +Y-A轴角度值(电机) */
    Uint32 n_ya_axis_angle_mot;     /* B32~B35 -Y-A轴角度值(电机) */

    int16 p_ya_axis_angle_v;        /* B36~B37 +Y-A轴当前角速度 */
    int16 n_ya_axis_angle_v;        /* B38~B39 -Y-A轴当前角速度 */

    Uint32 p_ya_axis_cmd_local;     /* B40~B43 +Y-A轴指令位置 */
    int16  p_ya_axis_cmd_speed;     /* B44~B45 +Y-A轴指令速度 */
    Uint32 n_ya_axis_cmd_local;     /* B46~B49 +Y-A轴指令位置 */
    int16  n_ya_axis_cmd_speed;     /* B50~B51 +Y-A轴指令速度 */

    uint8_t p_ya_axis_line_center_loc;  /* B52  +Y-A轴线管中间位置 */
    uint8_t p_ya_axis_input_wall;       /* B53 +Y-A轴输入轴系外壁 */
    uint8_t n_ya_axis_line_center_loc;  /* B54 -Y-A轴线管中间位置 */
    uint8_t n_ya_axis_input_wall;       /* B55 -Y-A轴输入轴系外壁 */

    uint8_t rev1;						/* B56 备用 */

    uint8_t edac_single_err_cnt;        /* B57 EDAC单错计数 */
    uint8_t edac_double_err_cnt;        /* B58 EDAC双错计数 */

    uint8_t p_ya_hold_cur_set;   	/* B59 +Y-A轴保持电流设置 */
    uint8_t n_ya_hold_cur_set;   	/* B60 -Y-A轴保持电流设置 */

    uint8_t p_ya_max_acc_set;		/* B61 +Y-A最大加速度设置 */
    uint8_t n_ya_max_acc_set;		/* B62 -Y-A最大加速度设置 */

    Uint16 p_ya_reset_speed_set;	/* B63~B64 +Y-A复位速度设置 */
    Uint16 n_ya_reset_speed_set;	/* B65~B66 -Y-A复位速度设置 */

    Uint16 p_ya_drv_cur_adc;         /* B67~B70 预留 */
    Uint16 n_ya_drv_cur_adc;

    Uint16  check_sum;              /* B71~B72    校验和 */
    Uint16  frame_tile;             /* B73~B74    帧尾 */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_DGM_4)
typedef struct {
    Uint16  frame_head;             /* B0~B1 帧头 */
    Uint16  dat_len;                /* B2~B3 数据长度 */
    uint8_t serve_type;             /* B4 服务类型 */
    uint8_t serve_state;            /* B5 服务状态  */

    uint8_t pri_bkup_id;            /* B6 主备份ID */
    uint8_t tele_req_cnt;           /* B7 遥测请求计数 */
    uint8_t right_mov_cnt;          /* B8 正确运动指令帧计数 */
    uint8_t right_conf_cnt;         /* B9 正确配置指令帧计数 */
    uint8_t wrong_cnt;              /* B10 错误指令帧计数 */
    uint8_t comm_rest_cnt;          /* B11 通讯复位计数 */

    uint8_t p_ya_max_mov_cur_set;   /* B12 +Y-A轴最大运动电流设置 */
    uint8_t n_ya_max_mov_cur_set;   /* B13 -Y-A轴最大运动电流设置 */
    uint8_t p_yb_max_mov_cur_set;   /* B14 +Y-B轴最大运动电流设置 */
    uint8_t n_yb_max_mov_cur_set;   /* B15 -Y-B轴最大运动电流设置 */

    uint8_t last_cmd;               /* B16 最近执行指令码 */
    comm_stat_t comm_stat;          /* B17 通信状态 */
    uint8_t p_ya_axis_work_mode;    /* B18 +Y-A轴工作模式 */
    axis_stat_t p_ya_axis_stat;     /* B19 +Y-A轴状态 */
    uint8_t n_ya_axis_work_mode;    /* B20 -Y-A轴工作模式 */
    axis_stat_t n_ya_axis_stat;     /* B21 -Y-A轴状态 */
    uint8_t p_yb_axis_work_mode;    /* B22 +Y-B轴工作模式 */
    axis_stat_t p_yb_axis_stat;     /* B23 +Y-B轴状态 */
    uint8_t n_yb_axis_work_mode;    /* B24 -Y-B轴工作模式 */
    axis_stat_t n_yb_axis_stat;     /* B25 -Y-B轴状态 */

    int32 p_ya_axis_angle_res;     /* B26~B29 +Y-A轴角度值(电位计) */
    int32 n_ya_axis_angle_res;     /* B30~B33 -Y-A轴角度值(电位计) */
    int32 p_yb_axis_angle_res;     /* B34~B37 +Y-B轴角度值(电位计) */
    int32 n_yb_axis_angle_res;     /* B38~B41 -Y-B轴角度值(电位计) */

    int32 p_ya_axis_angle_mot;     /* B42~B45 +Y-A轴角度值(电机) */
    int32 n_ya_axis_angle_mot;     /* B46~B49 -Y-A轴角度值(电机) */
    int32 p_yb_axis_angle_mot;     /* B50~B53 +Y-B轴角度值(电机) */
    int32 n_yb_axis_angle_mot;     /* B54~B57 -Y-B轴角度值(电机) */

    int16 p_ya_axis_angle_v;        /* B58~B59 +Y-A轴当前角速度 */
    int16 n_ya_axis_angle_v;        /* B60~B61 -Y-A轴当前角速度 */
    int16 p_yb_axis_angle_v;        /* B62~B63 +Y-B轴当前角速度 */
    int16 n_yb_axis_angle_v;        /* B64~B65 -Y-B轴当前角速度 */

    int32 p_ya_axis_cmd_local;      /* B66~B69 +Y-A轴指令位置 */
    int16  p_ya_axis_cmd_speed;     /* B70~B71 +Y-A轴指令速度 */
    int32 n_ya_axis_cmd_local;      /* B72~B75 -Y-A轴指令位置 */
    int16  n_ya_axis_cmd_speed;     /* B76~B77 -Y-A轴指令速度 */
    int32 p_yb_axis_cmd_local;      /* B78~B81 +Y-B轴指令位置 */
    int16  p_yb_axis_cmd_speed;     /* B82~B83 +Y-B轴指令速度 */
    int32 n_yb_axis_cmd_local;      /* B84~B87 -Y-B轴指令位置 */
    int16  n_yb_axis_cmd_speed;     /* B88~B89 -Y-B轴指令速度 */

    uint8_t p_ya_axis_line_center_loc;  /* B90  +Y-A轴线管中间位置 */
    uint8_t p_ya_axis_input_wall;       /* B91 +Y-A轴输入轴系外壁 */
    uint8_t n_ya_axis_line_center_loc;  /* B92 -Y-A轴线管中间位置 */
    uint8_t n_ya_axis_input_wall;       /* B93 -Y-A轴输入轴系外壁 */

    uint8_t p_yb_temp;                  /* B94 +Y-B温度采集 */
    uint8_t n_yb_temp;                  /* B95 -Y-B温度采集 */
    uint8_t edac_single_err_cnt;        /* B96 EDAC单错计数 */
    uint8_t edac_double_err_cnt;        /* B97 EDAC双错计数 */

    uint8_t rev3[4];                    /* B98~B101 预留 */

    Uint16  check_sum;              /* B102~B103    校验和 */
    Uint16  frame_tile;             /* B104~B105    帧尾 */
} tele_dat_t;
#elif (BOARD_NAME == BOARD_GMS)

/**
 * \brief 定义程序运行地址
 */
#define PROGRAM_RUN_ADDR_LOW        0x11    /* 低地址 */
#define PROGRAM_RUN_ADDR_HIGH       0x22    /* 高地址 */
#define PROGRAM_RUN_ADDR_UNKNOW     0x33    /* 未知地址 */

/**
 * \brief 定义上次复位原因
 */
#define LAST_RESET_REASON_POWER_ON       0x11    /* 上电复位 */
#define LAST_RESET_REASON_WDT            0x22    /* 看门狗复位 */
#define LAST_RESET_REASON_SERIAL_ERR     0x33    /* 串口异常主动复位 */

typedef struct {
    Uint16  frame_head;             /* B0~B1 帧头 */
    Uint16  dat_len;                /* B2~B3 数据长度 */
    uint8_t serve_type;             /* B4 服务类型 */
    uint8_t serve_state;            /* B5 服务状态  */
    uint8_t pri_bkup_id;            /* B6 主备份ID */
    uint8_t tele_req_cnt;           /* B7 遥测请求计数 */
    uint8_t right_mov_cnt;          /* B8 正确运动指令帧计数 */
    uint8_t right_conf_cnt;         /* B9 正确配置指令帧计数 */
    uint8_t wrong_cnt;              /* B10 错误指令帧计数 */
    uint8_t comm_rest_cnt;          /* B11 通讯复位计数 */
    uint8_t last_cmd;               /* B12 最近执行指令码 */
    comm_stat_t comm_stat;          /* B13 通信状态 */
    uint8_t p_yb_axis_work_mode;    /* B14 +Y-B轴工作模式 */
    axis_stat_t p_yb_axis_stat;     /* B15 +Y-B轴状态 */
    int32 p_yb_axis_angle_res;      /* B16~B19 +Y-B轴角度值(角位移传感器) */
    int32 p_yb_axis_angle_mot;      /* B20~B23 +Y-B轴角度值(电机) */
    int16 p_yb_axis_angle_v;        /* B24~B25 +Y-B轴当前角速度 */
    int32 p_yb_axis_cmd_local;      /* B26~B29 +Y-B轴指令位置 */
    int16  p_yb_axis_cmd_speed;     /* B30~B31 +Y-B轴指令速度 */
    uint8_t p_yb_cmd_work_mode;     /* B32 +YB轴指令工作模式 */
    uint8_t p_yb_max_acc_set;       /* B33 +YB轴最大加速度设置 */
    uint8_t p_yb_drv_cur_set;       /* B34 +YB轴驱动电流设置 */
    uint8_t p_yb_keep_cur_set;      /* B35 +YB轴保持电流设置 */
    int32   p_yb_soft_lim_p;        /* B36~B39 +YB轴软件限位配置（正方向） */
    int32   p_yb_soft_lim_n;        /* B40~B43 +YB轴软件限位配置（负方向） */
    Uint16  p_yb_reset_speed_set;   /* B44~B45 +YB轴回零速度设置 */
    uint8_t cur_5v;                 /* B46 5V电流遥测 */

//    Uint16  temp_senser_vol;        /* B47~B48 温度传感器供电电压 */
//    Uint16  temperature;            /* B49~B50 温度值 */
    uint8_t program_run_addr;       /* B47 当前程序运行地址区 */
    uint8_t last_reset_reason;      /* B48 上次复位原因 */
    Uint16 system_reset_times;      /* B49~B50 系统复位次数 */

    Uint16  drv_cur_adc;            /* B51~B52 驱动电流，ADC采样值 */
    int16  hall_angle;              /* B53~B54 霍尔角度 */
    uint8_t program_version;        /* B55 程序版本号 */
    Uint32  program_err_bits;       /* B56~B59 程序校验错误bit计数 */
    Uint16  program_run_region;     /* B60 当前程序运行区 */

    uint8_t pps_cnt;                /* B61 PPS计数值 */
    uint8_t soft_lim_err_stat;      /* B62 软件限位故障状态 */

    Uint16  check_sum;              /* B63~B64    校验和 */
    Uint16  frame_tile;             /* B65~B66    帧尾 */
} tele_dat_t;
#endif

#define RUN_REGION_MAIN             0xAA    /* 主运行区 */
#define RUN_REGION_BACKUP           0xBB    /* 备运行区 */
#define RUN_REGION_RAM              0xCC    /* RAM运行区 */


extern tele_dat_t g_Tele_Dat;  /**< \brief 遥测数据 */

/**
 * \brief 执行遥测请求指令
 *
 * \retval 成功返回0，失败返回-1
 */
extern int16 tele_req_cmd_run (void);


/**
 * \brief 初始化遥测数据
 */
extern void InitTeleDat (void);


#endif /* SOURCE_CMD_TELE_DAT_H_ */
