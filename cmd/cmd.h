/*
 * Axis_cmd.h
 *
 *  Created on: 2024年1月18日
 *      Author: 86132
 */

#ifndef APP_INCLUDE_AXIS_CMD_H_
#define APP_INCLUDE_AXIS_CMD_H_

#include "project.h"				//全局宏



/**
 * \brief 定义服务类型
 *
 * @{
 */
#define CMD_SERVE_TYPE_AB_AXIS_MOV_CTL				0x11	/* AB轴运动控制指令 */
#define CMD_SERVE_TYPE_MAX_ACC_CONF					0x22	/* 最大加速度配置指令 */
#define CMD_SERVE_TYPE_REST_SPEED_CONF				0x33	/* 复位速度配置指令 */
#define CMD_SERVE_TYPE_MOT_CUR_CONF					0x44	/* 电机电流配置指令 */
#define CMD_SERVE_TYPE_MOT_OVERCUR_PROT_CONF		0x55	/* 电机过流保护配置指令 */
#define CMD_SERVE_TYPE_CUR_LOCAL_CONF				0x66	/* 当前位置配置指令 */
#define CMD_SERVE_TYPE_SADMB_SOFT_LIM_CONF			0x88	/* SADM-B软件限位配置指令 */
#define CMD_SERVE_TYPE_SADM_DRV_HOT_BKUP_CONF       0x77    /* SADM驱动热备配置指令 */
#define CMD_SERVE_TYPE_ANGLE_POW_SW_CONF            0xAA    /* 角度传感器开关配置指令 */
#define CMD_SERVE_TYPE_RESET_TIMES_CLEAR            0xBB    /* 复位次数清零指令 */

#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_GMS)
#define CMD_SERVE_TYPE_TELE_REQ                     0x01    /* 遥测请求指令 */
#elif (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
#define CMD_SERVE_TYPE_TELE_REQ	  	  				0xAA	/* 遥测请求指令 */
#endif
#define CMD_SERVE_TYPE_RECONSTITUTION  	  			0x08	/* 重构指令 */
/**
 * @}
 */

/**
 * \brief 定义常用帧中数据
 *
 * @{
 */
#define DAT_FRAME_HEAD	0xEB90		/* 帧头 */
#define DAT_FRAME_TILE	0x09D7		/* 帧尾 */
/**
 * @}
 */

/**
 * \brief 定义工作模式
 *
 * @{
 */
#define WORK_MODE_TRACE     0x11     /* 跟踪模式 */
#define WORK_MODE_LAUNCH    0x11     /* 展开模式 */
#define WORK_MODE_CAPTURE   0x22     /* 捕获模式 */
#define WORK_MODE_RESET     0x33     /* 复位模式 */
#define WORK_MODE_KEEP      0x44     /* 保持模式 */
#define WORK_MODE_STANDBY   0x99     /* 待机模式 */
/**
 * @}
 */

/**
 * \brief 定义电机过流保护配置指令
 *
 * @{
 */
#define OVER_CUR_PROT_SW_OFF     0x55    /* 过流保护关 */
#define OVER_CUR_PROT_SW_ON      0xAA    /* 过流保护开 */
/**
 * @}
 */

/**
 * \brief 定义写IO的函数指针
 */
typedef void (*pfn_write_io)(uint8_t val);
/**
 * \brief 定义读IO的函数指针
 */
typedef Uint16 (*pfn_read_io)(void);

/**
 * \brief 定义每个阶段中需要参与计算的基本量，根据 x = v0t + (1/2)at^2 计算每个脉冲的时间
 *
 * \note 如果是跟踪模式的匀速阶段，则all_pulse_cnt的值为(!0)
 */
typedef struct {
	float32 all_step_cnt;			/* 整步数 */
	Uint32  loc_diff;			    /* 需走过的位置 */
	Uint32  all_segment;			/* 所有小段的个数，步数乘以细分数等于小段数 = 当前阶段总的微步数 */
	Uint32  elapsed_segment_cnt;	/* 已经过去的小段数 = 已经走过的微步数 */
	int32   v0_1000;				/* 当前阶段的初始速度，1000倍 */
	int32   a_1000;					/* 当前阶段的加速度，1000倍 */
	float32 t;						/* 记录当前阶段的时间 */
	int8_t 	mov_dir;				/* 运动方向，为1表示正向，为-1表示反向 */
} stage_var_t;


/**
 * \brief 定义每个轴的配置结构
 *
 * @{
 */
typedef struct single_axis_conf {
	char name[4];					         /* 当前轴的名字 */
	struct single_axis_conf* p_next_axis;    /* 下个轴的地址 */

    uint8_t work_mode_set;          /* 设定的新工作模式 */
    uint8_t new_task_flag;          /* 需要开启新任务的标志 */
    int8_t reciprocat_mode_enable;	/* 往复模式使能，用于判断当前是捕获模式还是往复模式 */
    int32   loc_given;              /* 位置给定，捕获模式有效 */
    int16   speed_given;            /* 速度给定，捕获模式、跟踪模式有效 */
    float32 hall_angle;				/* 霍尔角度 */
    uint8_t max_acc;                /* 最大加速度 */
    Uint16  reset_speed_given;      /* 复位速度给定 */
    Uint16  max_mov_cur_given;      /* 最大运动电流给定 */
    Uint16  keep_cur_given;         /* 保持电流给定 */
#if 0   /* 这两个数在2812里面不需要 */
    uint8_t overcur_prot_sw;        /* 过流保护开关 */
    uint8_t overcur_prot_th;        /* 过流保护阈值 */
#endif
    int32   cur_local;                /* 当前位置（浮点数），一个位置等于0.000225° */
#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
    int32   soft_lim;               /* 软件限位位置 */
#elif BOARD_NAME == BOARD_GMS
    int32   soft_lim_p;               /* 软件限位位置（正） */
    int32   soft_lim_n;               /* 软件限位位置（负） */
#endif
//    uint8_t drv_conf;               /* 驱动配置：0x55：驱动线路1。0xAA：驱动线路2  */

    /**< \brief 以下内容根据实际情况增加 */
    float32 reset_local;            /* 复位位置 */
    uint8_t work_mode_old;          /* 旧工作模式 */
    Uint16  speed_ratio;            /* 转速比 */
    Uint16  pulse_cnt_per_step;     /* 每1步包含的脉冲数量 */
    float32 step_angle;             /* 步距角 */
    _iq23   step_angle_out_iq23;    /* 输出轴步距角 例如： 1.8 / (32 * 125) */
    _iq30   step_angle_iq;

    float32 loc_to_angle_factor;    /* 从位置值转换到角度值需要乘上的系数 */
    float32 speedvalue_to_speed;	/* 从速度给定值转换到速度需要乘上的系数 */

    int32   min_loc;                /* 最小位置值 */
    int32   max_loc;                /* 最大位置值 */

#if 0   /* 这2个数用不上 */
    float32 min_angle;              /* 最小角度值 */
    float32 max_angle;              /* 最大角度值 */
#endif
    int16   min_speed;				/* 最小速度值 */
    int16   max_speed;				/* 最大速度值 */

    float32 duty_factor_mov;		  	    /* 在每个占空比上乘上的系数，改变电流时用，运动电流 */
    float32 duty_factor_hold;		  	    /* 在每个占空比上乘上的系数，改变电流时用，保持电流 */
    Uint16  pulse_tbprd_mov[XIFEN_CNT+1];   /* 2812中传递给比较寄存器的数值 , CMP1 ,CMP2 */
    Uint16  pulse_tbprd_hold[XIFEN_CNT+1];   /* 2812中传递给比较寄存器的数值 , CMP1 ,CMP2 */

    /* 任务进行时需要的变量 */
    uint8_t is_task_running;        	        /* 为true表示当前轴正在进行任务 */
    Uint32  next_segment_pulse_cnt;		        /* 下一个微步中的pwm脉冲数量 */
    Uint32  uniform_segment_pulse_cnt_zheng;    /* 达到匀速运动时每个微步中的pwm脉冲数量（整数部分） */
    Uint32  uniform_segment_pulse_cnt_xiao_1000;    /* 达到匀速运动时每个微步中的pwm脉冲数量（小数部分的1000倍） */
    Uint32  uniform_segment_pulse_cnt_xiao_sum;     /* 累计值 */
    int8_t  step_temp;

    float64 cur_angle_speed;        	/* 当前运动角速度 */
    uint8_t period_cnt_1_4;             /* 当前所处的1/4周期数 0~3 */
    Uint16  cur_xifen_pulse_num;        /* 当前微步内的脉冲编号号 */
    int16   cur_xifen_num;              /* 当前所处的微步编号 ,即一个整步的细分数编号  0~32*/
    uint8_t tail_flag;                  /* 任务结束标志 */
    uint8_t reverse_flag;				/* 需要反向运动标志 */
    float64 t2_us;						/* 计算时间时需要 */
    stage_var_t stage[5];		        /* 一次任务包含的阶段 */
    uint8_t 	stage_cnt;		        /* 需要执行的阶段数量 */
    uint8_t   	cur_stage;              /* 当前运动阶段  */

    /* 控制函数指针 */
    pfn_write_io fn_write_a_dir;     /* 设置a相电流方向 , 写1电流 为正 ,0电流 为负 */
    pfn_write_io fn_write_b_dir;     /* 设置b相电流方向 , 写1电流 为正 ,0反向 为负*/
    pfn_write_io fn_write_brake;     /* 设置刹车信号（A相和B相一起） */
} single_axis_conf_t;
/**
 * @}
 */


/**
 * \brief 定义4个轴的配置
 *
 * \note 程序中有使用4个轴的地址，如果本结构体要修改，需要保证4个轴的定义中间不能插入别的变量定义
 * @{
 */
typedef struct {
    uint8_t new_task_req;        /* 新任务请求（接收到轴运动控制指令） */
    uint8_t task_cnt;              /* 记录正在做任务的轴的数量 */
    Uint32  cmd_pps;               /* 接收到命令时的PPS值 */
    uint8_t run_delay;             /* 执行状态，表征指令延时，以PPS为标准，单位：s */
    float32 duty[XIFEN_CNT+1];     /* 一整步（1/4周期）每一细分脉冲占空比，比细分数多一个数 ,需要记录占空比为0和为1的情况所以比细分数多一个 */
    single_axis_conf_t* p_pya;     /* 预留*/
    single_axis_conf_t* p_nya;     /* 预留 */
    single_axis_conf_t* p_pyb;     /* 主面轴 */
    single_axis_conf_t* p_nyb;     /* 副面轴 */
} axis_conf_t;
/**
 * @}
 */

/** <\brief 定义4个轴的配置，当接收到配置指令后将数据存放在这个变量中 */
extern axis_conf_t g_Axis_Conf;

/* 用于判断串口是否超时未接收到数据 */
extern Uint64 g_RS422_DATA_Tick;

/* 串口重新连接的次数 */
extern Uint16 g_RS422_relink_times;

/**
 * \brief 初始化轴的配置数据
 */
extern void Init_Axis_Conf(void);
/**
 * \brief 得到整1/4周期内的所有占空比值
 *
 * \param[in]  N：1/4个周期内包含的细分数
 * \param[out] p_duty：计算后的占空比数值
 *
 * \note sin(2*pi*t/T)的图像
 */
extern void get_all_duty (Uint16 N_4, float32* p_duty);

/**
 * \brief 计算校验和
 *
 * \param[in] pData：要计算的数据
 * \param[in] uiLen：要计算的数据长度
 *
 * \retval 返回校验和
 */
extern Uint16 CheckSumCalc (const Uint16 *pData, Uint16 uiLen);


#endif /* APP_INCLUDE_AXIS_CMD_H_ */
