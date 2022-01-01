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
 * @file    portab.h
 * @brief   Application portability macros and structures.
 *
 * @addtogroup application_portability
 * @{
 */

#ifndef PORTAB_H
#define PORTAB_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define PORTAB_LINE_LED1            LINE_LED1_RED
#define PORTAB_LINE_LED2            LINE_LED2_GREEN
#define PORTAB_LED_OFF              PAL_LOW
#define PORTAB_LED_ON               PAL_HIGH

#define PORTAB_LINE_BUTTON          LINE_BUTTON_USER
#define PORTAB_BUTTON_PRESSED       PAL_HIGH

#define PORTAB_SD1                  SD1

#if !defined(MFS_DUAL_BANK_CONFIG)
#define PORTAB_MFS_CFG \
  .flashp        = (BaseFlash *)&EFLD1, \
  .erased        = 0xFFFFFFFFU,         \
  .bank_size     = 768U*1024U,          \
  .bank0_start   = 6U,                 \
  .bank0_sectors = 3U,                  \
  .bank1_start   = 9U,                 \
  .bank1_sectors = 3U
#else
#define PORTAB_MFS_CFG \
  .flashp        = (BaseFlash *)&EFLD1, \
  .erased        = 0xFFFFFFFFU,         \
  .bank_size     = 384U*1024U,          \
  .bank0_start   = 18U,                 \
  .bank0_sectors = 3U,                  \
  .bank1_start   = 21U,                 \
  .bank1_sectors = 3U
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void portab_setup(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* PORTAB_H */

/** @} */
