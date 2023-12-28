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
 * @file    simulator/posix/hal_lld.c
 * @brief   Posix simulator HAL subsystem low level driver code.
 *
 * @addtogroup POSIX_HAL
 * @{
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>

#include "hal.h"
#include "timespec_funcs.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static struct timespec deadline;
static struct timespec dt = {0UL, 1000000000UL / OSAL_ST_FREQUENCY};
static uint32_t ctr = 0;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void reschedule_if_needed (void)
{
    __dbg_check_lock();
    if (chSchIsPreemptionRequired())
    {
        chSchDoPreemption();
    }
    __dbg_check_unlock();
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief Low level HAL driver initialization.
 */
void hal_lld_init(void) {

#if defined(__APPLE__)
  puts("ChibiOS/RT simulator (OS X)\n");
#else
  puts("ChibiOS/RT simulator (Linux)\n");
#endif
  clock_gettime(CLOCK_MONOTONIC, &deadline);
  timespecadd(&deadline, &dt, &deadline);
}

/**
 * @brief   Interrupt simulation.
 */
void _sim_check_for_interrupts(void) {
  struct timespec now, tdiff;

#if HAL_USE_SERIAL
#if USE_SIM_SERIAL_TTY
#if USE_SIM_SERIAL1
  if (sd_lld_interrupt_pending(&SD1)) {
    sd_lld_irq_handler_serial1();
    reschedule_if_needed();
  }
#endif
#if USE_SIM_SERIAL2
  if (sd_lld_interrupt_pending(&SD2)) {
    sd_lld_irq_handler_serial2();
    reschedule_if_needed();
  }
#endif
#else
  if (sd_lld_interrupt_pending()) {
    reschedule_if_needed();
  }
#endif
#endif

#if HAL_USE_GPT
#if SIMULATOR_GPT_USE_TIM1
  if (gpt_lld_interrupt_pending(&GPTD1))
  {
    gpt_lld_irq_handler_tim1();
    reschedule_if_needed();
  }
#endif
#if SIMULATOR_GPT_USE_TIM2
  if (gpt_lld_interrupt_pending(&GPTD2))
  {
    gpt_lld_irq_handler_tim2();
    reschedule_if_needed();
  }
#endif
#endif

#if HAL_USE_ADC
#if SIMULATOR_ADC_USE_ADC1
  if (adc_lld_interrupt_pending0())
  {
    adc_lld_int_handler0();
    reschedule_if_needed();
  }
#endif
#if SIMULATOR_ADC_USE_ADC2
  if (adc_lld_interrupt_pending1())
  {
    adc_lld_int_handler1();
    reschedule_if_needed();
  }
#endif
#endif

  clock_gettime(CLOCK_MONOTONIC, &now);

  if (++ctr % 50 == 0)
  {
    timespecsub(&now, &deadline, &tdiff);
    int64_t nsec_divergence = tdiff.tv_sec * 1000000000ULL + tdiff.tv_nsec;
    if (((unsigned long long)nsec_divergence > 1000000000ULL) && (tdiff.tv_sec >= 0))
    {
        fprintf(stderr, "Current divergence is %llu msec. Consider reducing CH_CFG_ST_FREQUENCY.\n", nsec_divergence/1000000ULL);
    }
    ctr = 0;
  }

  if (timespeccmp(&now, &deadline, >=))
  {
    timespecadd(&deadline, &dt, &deadline);

    CH_IRQ_PROLOGUE();

    chSysLockFromISR();
    chSysTimerHandlerI();
    chSysUnlockFromISR();

    CH_IRQ_EPILOGUE();
    reschedule_if_needed();
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
}

/** @} */
