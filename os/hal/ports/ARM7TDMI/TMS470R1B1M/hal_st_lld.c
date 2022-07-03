/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    LPC214x/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

static void st_lld_serve_interrupt(void)
{
    osalSysLockFromISR();
    INTREGS->RTICNTL = 0x00000040;
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void)
{
	INTREGS->RTICNTEN = 0x00000000;
	INTREGS->RTICNTL = 0x00000040;
	INTREGS->RTIPCTL = 0x00002800 + ((1000U / (64000000/getMclk())) - 1);

	registerIrqHandler(st_lld_serve_interrupt, IEMNUM_TAP);
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
