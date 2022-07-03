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
 * @file    TMS470R1B1M/hal_lld.h
 * @brief   TMS470R1B1M HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "tms470r1b1m-map.h"
#include "tms470r1b1m-ints.h"
#include "tms470r1b1m-bitdefs.h"
#include "nonvic.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS FALSE

/**
 * @brief   Platform name.
 */
#define PLATFORM_NAME   "TMS470R1B1M"

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Default action for the non vectored IRQ handler, nothing.
 */
#if !defined(TMS470_NON_VECTORED_IRQ_HOOK) || defined(__DOXYGEN__)
#define TMS470_NON_VECTORED_IRQ_HOOK()
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void __early_init(void);
  void __cpu_init(void);
  void tms470r1b1m_late_init(void);
  uint32_t getMclk(void);
  uint32_t getIclk(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
