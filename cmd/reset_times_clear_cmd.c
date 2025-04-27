/*
 * reset_times_clear_cmd.c
 *
 *  Created on: 2024Äê8ÔÂ27ÈÕ
 *      Author: Suzkfly
 */
#include "cmd.h"
#if BOARD_NAME == BOARD_GMS
#include "reset_times_clear_cmd.h"
#include "boot_args.h"
#include "tele_dat.h"
int8_t reset_times_clear (void)
{
    (*gp_boot_arg_system_reset_times) = 0;
    g_Tele_Dat.system_reset_times = 0;

    return 0;
}

#endif


