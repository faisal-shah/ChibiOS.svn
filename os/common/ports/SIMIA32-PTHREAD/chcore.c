/*
    ChibiOS - Copyright (C) 2006,2007,2008,2009,2010,2011,2012,2013,2014,
              2015,2016,2017,2018,2019,2020,2021 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    SIMIA32/chcore.c
 * @brief   Simulator on IA32 port code.
 *
 * @addtogroup SIMIA32_GCC_CORE
 * @{
 */

#if defined(WIN32)
#include <windows.h>
#else
#include <errno.h>
#include <time.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <sys/sysinfo.h>
#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define LOG_DBG(format, ...) \
    printf("%s::%s:%d " format "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

bool port_isr_context_flag;
syssts_t port_irq_sts;
static cpu_set_t sCpuAffinity;

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void _wakeup_pthread(thread_t * tp) {

  struct port_context * pctx = &tp->ctx;
  if(pthread_self() != pctx->pthread)
  {
    pthread_mutex_lock(&pctx->sync);
    pctx->cond_trig = true;
    pthread_cond_signal(&pctx->cond);
    pthread_mutex_unlock(&pctx->sync);
  }
}

static void _suspend_pthread(thread_t * tp) {

  struct port_context * pctx = &tp->ctx;
  pthread_mutex_lock(&pctx->sync);

  while(pctx->cond_trig == false)
  {
    pthread_cond_wait(&pctx->cond, &pctx->sync);
  }

  pctx->cond_trig = false;
  pthread_mutex_unlock(&pctx->sync);
}

static int init_pthread_sync(thread_t * tp) {

  int ret;
  ret = pthread_mutex_init(&tp->ctx.sync, NULL);
  if (ret != 0)
  {
    LOG_DBG("%s%d", "Failed to initialize mutex with return value: ", ret);
  }
  ret = pthread_cond_init(&tp->ctx.cond, NULL);
  if (ret != 0)
  {
    LOG_DBG("%s%d", "Failed to initialize cond var with return value: ", ret);
  }
  tp->ctx.cond_trig = false;

  return ret;
}

/**
 * @brief   Start a thread by invoking its work function.
 * @details If the work function returns @p chThdExit() is automatically
 *          invoked.
 */
__attribute__((noreturn))
static void _port_thread_start(void *p) {
  thread_t * pthd = (thread_t *)p;
  _suspend_pthread(pthd);
  chSysUnlock();
  pthd->ctx.funcp(pthd->ctx.arg);
  chThdExit(0);
  while(1);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Port-related initialization code.
 */

void port_init(os_instance_t *oip) {

  int ret;

  port_irq_sts = (syssts_t)0;
  port_isr_context_flag = false;

  init_pthread_sync(&oip->mainthread);
  oip->mainthread.ctx.pthread = pthread_self();


  srand((unsigned int)pthread_self());
  CPU_ZERO(&sCpuAffinity);
  CPU_SET(rand() % get_nprocs(), &sCpuAffinity);
  ret = pthread_setaffinity_np(pthread_self(), sizeof(sCpuAffinity), &sCpuAffinity);
  if(ret != 0)
  {
    LOG_DBG("%s%d", "Failed to set cpu affinity return value: ", ret);
  }
}

void _port_enter_critical(void) {

  port_irq_sts++;
}

void _port_exit_critical(void) {

  port_irq_sts--;
}

void _port_switch(thread_t *ntp, thread_t *otp) {

  _wakeup_pthread(ntp);
  _suspend_pthread(otp);
}

void _port_setup_context(thread_t *tp, stkalign_t *wbase, stkalign_t *wtop,
                         tfunc_t pf, void *arg) {

  int ret;
  size_t stack_size;
  pthread_attr_t pthd_attr;

  stack_size = (size_t)((wtop - wbase + 1U) * sizeof(stkalign_t));

  tp->ctx.funcp = pf;
  tp->ctx.arg = arg;
  init_pthread_sync(tp);
  pthread_attr_init(&pthd_attr);
  ret = pthread_attr_setstack(&pthd_attr, (size_t)wtop - stack_size + 1U, stack_size);
  if(ret != 0)
  {
    LOG_DBG("%s%d", "[WARN] pthread_attr_setstack failed with return value: ", ret);
  }

  ret = pthread_attr_setaffinity_np(&pthd_attr, sizeof(sCpuAffinity), &sCpuAffinity);
  if(ret != 0)
  {
    LOG_DBG("%s%d", "Failed to set cpu affinity return value: ", ret);
  }

  ret = pthread_create(&tp->ctx.pthread, &pthd_attr, _port_thread_start, tp);
  if(ret != 0)
  {
    LOG_DBG("%s%d", "Failed to create pthread with return value: ", ret);
  }

  ret = pthread_attr_destroy(&pthd_attr);
  if(ret != 0)
  {
    LOG_DBG("%s%d", "Failed to destroy pthread attr with return value: ", ret);
  }
}

/**
 * @brief   Returns the current value of the realtime counter.
 *
 * @return              The realtime counter value.
 */
rtcnt_t port_rt_get_counter_value(void) {
#if defined(WIN32)
  LARGE_INTEGER n;

  QueryPerformanceCounter(&n);

  return (rtcnt_t)(n.QuadPart / 1000LL);
#else
  int ret;
  struct timespec ts;

  ret = clock_gettime(CLOCK_MONOTONIC, &ts);
  chDbgAssert(ret == 0, strerror(errno));

  return ((rtcnt_t)ts.tv_sec * (rtcnt_t)1000000) + (rtcnt_t)ts.tv_nsec/1000U;
#endif
}

/** @} */
