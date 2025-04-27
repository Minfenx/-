/*
 * mot_cur_conf_cmd_.c
 *
 *  Created on: 2023年5月30日
 *      Author: Suzkfly
 */

#include "mot_cur_conf_cmd.h"
#include "cmd.h"
#include "AB_axis_mov_ctl_cmd.h"

#if (BOARD_NAME == BOARD_CX20) || (BOARD_NAME == BOARD_DGM_2) || (BOARD_NAME == BOARD_DGM_4)
/**< \brief 不同电流的占空比系数，实测值 */
//static float32 __g_cur_set_factor[61] = {0,     0.011, 0.012, 0.017, 0.023, 0.026, 0.032, 0.036, 0.041, 0.045, \
//                                         0.049, 0.054, 0.058, 0.063, 0.067, 0.071, 0.075, 0.080, 0.084, 0.088, \
//                                         0.093, 0.097, 0.101, 0.105, 0.110, 0.114, 0.119, 0.123, 0.127, 0.131, \
//                                         0.136, 0.140, 0.145, 0.149, 0.153, 0.158, 0.162, 0.167, 0.171, 0.175, \
//                                         0.180, 0.184, 0.188, 0.193, 0.198, 0.202, 0.206, 0.210, 0.215, 0.219, \
//                                         0.224, 0.228, 0.233, 0.237, 0.241, 0.246, 0.250, 0.256, 0.260, 0.265, \
//                                         0.270};
#endif

#if BOARD_NAME == BOARD_GMS

/* 自动生成的保持电流 */
static const float32 __g_cur_set_factor_hold[81]= {0.0000, 0.0110, 0.0120, 0.0130, 0.0140, 0.0170, 0.0240, 0.0320, 0.0360, 0.0409,
                                                   0.0459, 0.0508, 0.0558, 0.0604, 0.0649, 0.0694, 0.0744, 0.0784, 0.0834, 0.0875,
                                                   0.0924, 0.0969, 0.1019, 0.1059, 0.1109, 0.1155, 0.1204, 0.1244, 0.1282, 0.1345,
                                                   0.1384, 0.1430, 0.1479, 0.1530, 0.1566, 0.1618, 0.1667, 0.1732, 0.1759, 0.1814,
                                                   0.1883, 0.1919, 0.1952, 0.2003, 0.2053, 0.2099, 0.2158, 0.2197, 0.2262, 0.2292,
                                                   0.2344, 0.2398, 0.2446, 0.2506, 0.2545, 0.2588, 0.2647, 0.2687, 0.2738, 0.2792,
                                                   0.2852, 0.2897, 0.2942, 0.3005, 0.3061, 0.3120, 0.3173, 0.3225, 0.3285, 0.3335,
                                                   0.3395, 0.3450, 0.3511, 0.3571, 0.3618, 0.3699, 0.3752, 0.3819, 0.3879, 0.3939,
                                                   0.4003};
#if 0       /* 这个数组是最大值 */
/* 自动生成的运动电流 */
static const float32 __g_cur_set_factor_mov[81] = {0.0000, 0.0110, 0.0120, 0.0130, 0.0140, 0.0170, 0.0240, 0.0320, 0.0360, 0.0409,
                                                   0.0459, 0.0508, 0.0558, 0.0604, 0.0649, 0.0694, 0.0744, 0.0784, 0.0834, 0.0875,
                                                   0.1048, 0.1087, 0.1128, 0.1187, 0.1237, 0.1285, 0.1327, 0.1377, 0.1423, 0.1473,
                                                   0.1513, 0.1563, 0.1609, 0.1656, 0.1698, 0.1751, 0.1803, 0.1845, 0.1896, 0.1947,
                                                   0.1994, 0.2042, 0.2090, 0.2137, 0.2177, 0.2227, 0.2269, 0.2326, 0.2368, 0.2418,
                                                   0.2469, 0.2519, 0.2560, 0.2616, 0.2659, 0.2715, 0.2763, 0.2829, 0.2870, 0.2911,
                                                   0.2973, 0.3018, 0.3049, 0.3134, 0.3186, 0.3249, 0.3292, 0.3361, 0.3419, 0.3477,
                                                   0.3528, 0.3587, 0.3654, 0.3717, 0.3778, 0.3833, 0.3893, 0.3938, 0.4002, 0.4057,
                                                   0.4090};
#else       /* 这个数组是有效值，前20个不能用，电流太小电机容易失步 */
static const float32 __g_cur_set_factor_mov[71] = {//0.0000, 0.0110, 0.0120, 0.0130, 0.0140, 0.0170, 0.0240, 0.0320, 0.0360, 0.0409,
                                                   //0.0459, 0.0508, 0.0558, 0.0604, 0.0649, 0.0694, 0.0744, 0.0784, 0.0834, 0.0875,
                                                   0.1415, 0.1497, 0.1566, 0.1636, 0.1696, 0.1774, 0.1835, 0.1905, 0.1976, 0.2036,
                                                   0.2106, 0.2173, 0.2242, 0.2303, 0.2364, 0.2425, 0.2495, 0.2584, 0.2646, 0.2700,
                                                   0.2780, 0.2846, 0.2924, 0.3005, 0.3069, 0.3139, 0.3211, 0.3278, 0.3362, 0.3446,
                                                   0.3507, 0.3605, 0.3695, 0.3774, 0.3837, 0.3908, 0.3995, 0.4061, 0.4142, 0.4261,
                                                   0.4333, 0.4417, 0.4496, 0.4573, 0.4684, 0.4764, 0.4855, 0.4932, 0.5020, 0.5097,
                                                   0.5182, 0.5262, 0.5350, 0.5430, 0.5546, 0.5641, 0.5759, 0.5849, 0.5929, 0.6000,
                                                   0.6098, 0.6287, 0.6373, 0.6472, 0.6544, 0.6634, 0.6862, 0.6974, 0.7074, 0.7280,
                                                   0.7397};
#endif
#elif BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4
/* 自动生成的保持电流 */
static const float32 __g_cur_set_factor_hold[101]= {0.0000, 0.0110, 0.0120, 0.0130, 0.0140, 0.0170, 0.0240, 0.0320, 0.0360, 0.0409,
													0.0459, 0.0508, 0.0558, 0.0604, 0.0649, 0.0694, 0.0744, 0.0784, 0.0834, 0.0875,
													0.0675, 0.0720, 0.0764, 0.0808, 0.0853, 0.0897, 0.0943, 0.0986, 0.1031, 0.1075,
													0.1127, 0.1172, 0.1217, 0.1262, 0.1306, 0.1351, 0.1396, 0.1441, 0.1491, 0.1537,
													0.1583, 0.1627, 0.1679, 0.1723, 0.1768, 0.1814, 0.1864, 0.1911, 0.1956, 0.2008,
													0.2053, 0.2097, 0.2144, 0.2195, 0.2242, 0.2293, 0.2342, 0.2391, 0.2443, 0.2488,
													0.2542, 0.2595, 0.2640, 0.2693, 0.2745, 0.2791, 0.2843, 0.2895, 0.2942, 0.2995,
													0.3047, 0.3094, 0.3146, 0.3207, 0.3256, 0.3314, 0.3365, 0.3413, 0.3465, 0.3515,
													0.3567, 0.3617, 0.3671, 0.3724, 0.3784, 0.3838, 0.3893, 0.3948, 0.4009, 0.4073,
													0.4131, 0.4184, 0.4239, 0.4303, 0.4364, 0.4426, 0.4487, 0.4541, 0.4595, 0.4660,
													0.4716};

static const float32 __g_cur_set_factor_mov[105]= {0.1330, 0.1400, 0.1470, 0.1540, 0.1610, 0.1680, 0.1750, 0.1820, 0.1890, 0.1960,
													0.2030, 0.2100, 0.2170, 0.2240, 0.2310, 0.2380, 0.2450, 0.2520, 0.2590, 0.2660,
													0.2730, 0.2800, 0.2870, 0.2940, 0.3010, 0.3080, 0.3150, 0.3220, 0.3290, 0.3360,
													0.3430, 0.3500, 0.3570, 0.3640, 0.3710, 0.3780, 0.3850, 0.3920, 0.3990, 0.4060,
													0.4130, 0.4200, 0.4270, 0.4340, 0.4410, 0.4480, 0.4550, 0.4620, 0.4690, 0.4760,
													0.4830, 0.4900, 0.4970, 0.5040, 0.5110, 0.5180, 0.5250, 0.5320, 0.5390, 0.5460,
													0.5530, 0.5600, 0.5670, 0.5740, 0.5810, 0.5880, 0.5950, 0.6020, 0.6090, 0.6160,
													0.6230, 0.6300, 0.6370, 0.6440, 0.6510, 0.6580, 0.6650, 0.6720, 0.6790, 0.6860,
													0.6930, 0.7000, 0.7070, 0.7140, 0.7210, 0.7280, 0.7350, 0.7420, 0.7490, 0.7560,
													0.7630};
//float32 __g_cur_set_factor_mov[105] = { 0 };

#endif

#if 0

//float32 __g_cur_set_factor_hold[105] = { 0 };

/**
 * \brief 自动获得占空比系数
 */
#include "tele_dat.h"
#include <string.h>
Uint16 index = 10;
Uint16 times = 0;
Uint16 once_flag = 0;
Uint32 sum = 0;
Uint16 maoding_flag = 0;
Uint32 avrg = 0;
void get_cur_set_factor (void)
{
    if (once_flag == 0) {
        memset(__g_cur_set_factor_mov, 0, sizeof(__g_cur_set_factor_mov));
        once_flag = 1;
        index = 55;
        sum = 0;
        maoding_flag = 0;
        __g_cur_set_factor_mov[55] = 0.5;
        //g_Axis_Conf.p_pya->keep_cur_given = 20;
        g_Axis_Conf.p_pya->max_mov_cur_given = 55;

        set_mot_cur();
    }

    //if ((g_Axis_Conf.p_pyb->reciprocat_mode_enable == 1) && (g_Axis_Conf.p_pyb->work_mode_set == WORK_MODE_CAPTURE)) {
    if ((g_Axis_Conf.p_pya->work_mode_set != WORK_MODE_KEEP) && (g_Axis_Conf.p_pya->work_mode_set != WORK_MODE_STANDBY)) {
        if (maoding_flag == 0) {
            if (g_Tele_Dat.p_ya_drv_cur_adc >= index * 10) {
                maoding_flag = 1;
                sum = 0;
                times = 0;

            } else {
                __g_cur_set_factor_mov[index] += 0.001;
                set_mot_cur();
            }
        } else {
            sum += g_Tele_Dat.p_ya_drv_cur_adc;
            times++;
            if (times == 10) {
                times = 0;

                avrg = sum / 10;
                if (avrg == index * 10) {
                    maoding_flag = 0;
                    index++;
                    if (index == 102) {
                        while (1);
                    }
                    //g_Axis_Conf.p_pya->keep_cur_given++;
                    g_Axis_Conf.p_pya->max_mov_cur_given++;
                    __g_cur_set_factor_mov[index] = __g_cur_set_factor_mov[index - 1];
                } else if (avrg < index * 10) {
                    sum = 0;
                    __g_cur_set_factor_mov[index] += 0.0001;
                    set_mot_cur();
                } else {
                    sum = 0;
                    __g_cur_set_factor_mov[index] -= 0.0001;
                    set_mot_cur();
                }
            }
        }
    }
}
#endif

/**
 * \brief 设置电机电流值
 */
#if BOARD_NAME == BOARD_GMS
void set_mot_cur (void)
{
    g_Axis_Conf.p_pyb->duty_factor_mov  = __g_cur_set_factor_mov[g_Axis_Conf.p_pyb->max_mov_cur_given - 20];
	g_Axis_Conf.p_pyb->duty_factor_hold = __g_cur_set_factor_hold[g_Axis_Conf.p_pyb->keep_cur_given];		  	/* 在每个占空比上乘上的系数，改变电流时用，保持电流 */

    get_all_duty(XIFEN_CNT, g_Axis_Conf.duty);  /* 细分数确定以后占空比就确定了 */
    duty_to_cycle(g_Axis_Conf.p_pyb);           /* 计算不同占空比下比较寄存器的值  */
}
#elif (BOARD_NAME == BOARD_CX20 || BOARD_NAME == BOARD_DGM_2 || BOARD_NAME == BOARD_DGM_4)
void set_mot_cur (void)
{
    Uint16 i = 0;
    single_axis_conf_t* p_axis = g_Axis_Conf.p_pya;

    for (i = 0; i < AXIS_CNT; i++) {
        p_axis->duty_factor_mov  = __g_cur_set_factor_mov[p_axis->max_mov_cur_given - 20];
        p_axis->duty_factor_hold = __g_cur_set_factor_hold[p_axis->keep_cur_given];            /* 在每个占空比上乘上的系数，改变电流时用，保持电流 */
        p_axis = p_axis->p_next_axis;
    }

    get_all_duty(XIFEN_CNT, g_Axis_Conf.duty);  /* 细分数确定以后占空比就确定了 */

    p_axis = g_Axis_Conf.p_pya;
    for (i = 0; i < AXIS_CNT; i++) {
        duty_to_cycle(p_axis);           /* 计算不同占空比下比较寄存器的值  */
        p_axis = p_axis->p_next_axis;
    }
}
#endif

/**
 * \brief 串口数据解析
 *
 * \param[in]  pData：需要解析的串口数据指针，从帧头开始
 */
#if (BOARD_NAME == BOARD_CX20)
int8_t mot_cur_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
    	/* 检查数据的合法性 */
//    	if ((pData[6]  < 10) || (pData[6]  > 60) || (pData[7]  > 40) ||
//    		(pData[8]  < 10) || (pData[8]  > 60) || (pData[9]  > 40) ||
//			(pData[10] < 10) || (pData[10] > 90) || (pData[11] > 70) ||
//			(pData[12] < 10) || (pData[12] > 90) || (pData[13] > 70)) {
//    		return -1;
//    	}
//
//    	/* 运动电流和保持电流不能大于过流保护阈值 */
//    	if (((g_Axis_Conf.p_pya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[6]  > g_Axis_Conf.p_pya->overcur_prot_th)) ||
//    		((g_Axis_Conf.p_nya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[8]  > g_Axis_Conf.p_nya->overcur_prot_th)) ||
//			((g_Axis_Conf.p_pyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[10] > g_Axis_Conf.p_pyb->overcur_prot_th)) ||
//			((g_Axis_Conf.p_nyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[12] > g_Axis_Conf.p_nyb->overcur_prot_th))) {
//    		return -1;
//    	}

        g_Axis_Conf.p_pya->max_mov_cur_given  = pData[6];
        g_Axis_Conf.p_pya->keep_cur_given     = pData[7];
        g_Axis_Conf.p_nya->max_mov_cur_given  = pData[8];
        g_Axis_Conf.p_nya->keep_cur_given     = pData[9];
        g_Axis_Conf.p_pyb->max_mov_cur_given  = pData[10];
        g_Axis_Conf.p_pyb->keep_cur_given     = pData[11];
        g_Axis_Conf.p_nyb->max_mov_cur_given  = pData[12];
        g_Axis_Conf.p_nyb->keep_cur_given     = pData[13];

        set_mot_cur();

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_2)
int8_t mot_cur_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检查数据的合法性 */
		if ((pData[7] < 20) || (pData[7] > 90) || (pData[8]  > 90) ||
			(pData[9] < 20) || (pData[9] > 90) || (pData[10] > 90)) {
			return -1;
		}

//      /* 运动电流和保持电流不能大于过流保护阈值 */
//      if (((g_Axis_Conf.p_pya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[6]  > g_Axis_Conf.p_pya->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_nya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[8]  > g_Axis_Conf.p_nya->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_pyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[10] > g_Axis_Conf.p_pyb->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_nyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[12] > g_Axis_Conf.p_nyb->overcur_prot_th))) {
//          return -1;
//      }

        g_Axis_Conf.p_pya->max_mov_cur_given  = pData[7];
        g_Axis_Conf.p_pya->keep_cur_given     = pData[8];
        g_Axis_Conf.p_nya->max_mov_cur_given  = pData[9];
        g_Axis_Conf.p_nya->keep_cur_given     = pData[10];

        set_mot_cur();

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_DGM_4)
int8_t mot_cur_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.task_cnt == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检查数据的合法性 */
      if ((pData[7]  < 10) || (pData[7]  > 60) || (pData[8]  > 40) ||
          (pData[9]  < 10) || (pData[9]  > 60) || (pData[10] > 40) ||
          (pData[11] < 10) || (pData[11] > 90) || (pData[12] > 70) ||
          (pData[13] < 10) || (pData[13] > 90) || (pData[14] > 70)) {
          return -1;
      }
//
//      /* 运动电流和保持电流不能大于过流保护阈值 */
//      if (((g_Axis_Conf.p_pya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[6]  > g_Axis_Conf.p_pya->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_nya->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[8]  > g_Axis_Conf.p_nya->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_pyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[10] > g_Axis_Conf.p_pyb->overcur_prot_th)) ||
//          ((g_Axis_Conf.p_nyb->overcur_prot_sw == OVER_CUR_PROT_SW_ON) && (pData[12] > g_Axis_Conf.p_nyb->overcur_prot_th))) {
//          return -1;
//      }

        g_Axis_Conf.p_pya->max_mov_cur_given  = pData[7];
        g_Axis_Conf.p_pya->keep_cur_given     = pData[8];
        g_Axis_Conf.p_nya->max_mov_cur_given  = pData[9];
        g_Axis_Conf.p_nya->keep_cur_given     = pData[10];
        g_Axis_Conf.p_pyb->max_mov_cur_given  = pData[11];
        g_Axis_Conf.p_pyb->keep_cur_given     = pData[12];
        g_Axis_Conf.p_nyb->max_mov_cur_given  = pData[13];
        g_Axis_Conf.p_nyb->keep_cur_given     = pData[14];

        set_mot_cur();

        return 0;
    }
    return -1;
}
#elif (BOARD_NAME == BOARD_GMS)
int8_t mot_cur_handler (const Uint16 *pData)
{
    if (g_Axis_Conf.p_pyb->is_task_running == 0) { /* 只有任务没有执行时才可进行配置 */
        /* 检查数据的合法性 */
        //if ((pData[6]  < 10) || (pData[6]  > 60) || (pData[7]  > 40)) {
        if ((pData[6]  < 20) || (pData[6]  > 90) || (pData[7]  > 80)) {     /* 电流放开，最大可设置到0.8A */
            return -1;
        }

        g_Axis_Conf.p_pyb->max_mov_cur_given  = pData[6];
        g_Axis_Conf.p_pyb->keep_cur_given     = pData[7];

        set_mot_cur();

        return 0;
    }
    return -1;
}
#endif
