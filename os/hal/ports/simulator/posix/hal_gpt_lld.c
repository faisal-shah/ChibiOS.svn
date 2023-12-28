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
 * @file    hal_gpt_lld.c
 * @brief   SIMULATOR GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)
#include <time.h>
#include "timespec_funcs.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 */
#if (SIMULATOR_GPT_USE_TIM1 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif

#if (SIMULATOR_GPT_USE_TIM2 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SIMULATOR_GPT_USE_TIM1 || defined(__DOXYGEN__)
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
void gpt_lld_irq_handler_tim1(void) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_start_timer(&GPTD1, GPTD1.interval);

  _gpt_isr_invoke_cb(&GPTD1);

  if (GPTD1.config->secondary_callback != NULL)
  {
    GPTD1.config->secondary_callback(&GPTD1);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* SIMULATOR_GPT_USE_TIM1 */

#if SIMULATOR_GPT_USE_TIM2 || defined(__DOXYGEN__)
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
void gpt_lld_irq_handler_tim2(void) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_start_timer(&GPTD2, GPTD2.interval);

  _gpt_isr_invoke_cb(&GPTD2);

  if (GPTD2.config->secondary_callback != NULL)
  {
    GPTD2.config->secondary_callback(&GPTD2);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* SIMULATOR_GPT_USE_TIM2 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if SIMULATOR_GPT_USE_TIM1 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD1);
#endif
#if SIMULATOR_GPT_USE_TIM2 == TRUE
  /* Driver initialization.*/
  gptObjectInit(&GPTD2);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {

  if (gptp->state == GPT_STOP) {
    /* Enables the peripheral.*/
#if SIMULATOR_GPT_USE_TIM1 == TRUE
    if (&GPTD1 == gptp) {
      gpt_lld_stop_timer(gptp);
    }
#endif
#if SIMULATOR_GPT_USE_TIM2 == TRUE
    if (&GPTD2 == gptp) {
      gpt_lld_stop_timer(gptp);
    }
#endif
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  if (gptp->state == GPT_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if SIMULATOR_GPT_USE_TIM1 == TRUE
    if (&GPTD1 == gptp) {
      gpt_lld_stop_timer(gptp);
    }
#endif
#if SIMULATOR_GPT_USE_TIM2 == TRUE
    if (&GPTD2 == gptp) {
      gpt_lld_stop_timer(gptp);
    }
#endif
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {

  int ret;
  float ftime;
  struct timespec interval_time;

  ret = clock_gettime(CLOCK_MONOTONIC, &gptp->start_time);
  if (ret != 0)
  {
      perror("gpt_lld_start_timer");
      gpt_lld_stop_timer(gptp);
      return;
  }

  ftime = ((float)interval)/((float)gptp->config->frequency);
  interval_time.tv_sec = (time_t)ftime;
  interval_time.tv_nsec = 1000000000 * (ftime - (float)interval_time.tv_sec);

  timespecadd(&gptp->start_time, &interval_time, &gptp->match_time);

  gptp->interval = interval;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  timespecclear(&gptp->start_time);
  timespecclear(&gptp->match_time);
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  int ret;
  float ftime;
  struct timespec now, interval_time, match_time;

  ret = clock_gettime(CLOCK_MONOTONIC, &now);
  if (ret != 0)
  {
      perror("gpt_lld_polled_delay");
      osalDbgCheck(false);
  }

  ftime = ((float)interval)/((float)gptp->config->frequency);
  interval_time.tv_sec = (time_t)ftime;
  interval_time.tv_nsec = 1000000000 * (ftime - (float)interval_time.tv_sec);

  timespecadd(&now, &interval_time, &match_time);

  while (true)
  {
      ret = clock_gettime(CLOCK_MONOTONIC, &now);
      if (ret != 0)
      {
          perror("gpt_lld_polled_delay");
          osalDbgCheck(false);
      }
      if (timespeccmp(&now, &match_time, >))
      {
          break;
      }
  }
}

bool gpt_lld_interrupt_pending(GPTDriver *gptp) {

    int ret;
    struct timespec now;

    if (!timespecisset(&gptp->start_time) && !timespecisset(&gptp->match_time))
    {
        return false;
    }

    ret = clock_gettime(CLOCK_MONOTONIC, &now);
    if (ret != 0)
    {
        perror("gpt_lld_polled_delay");
        return false;
    }
    if (timespeccmp(&now, &gptp->match_time, >))
    {
        return true;
    }

    return false;
}

#endif /* HAL_USE_GPT == TRUE */

/** @} */
