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
 * @file    TMS470R1B1M/hal_serial_lld.h
 * @brief   TMS470R1B1M low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p TRUE .
 */
#if !defined(USE_TMS470R1B1M_UART0) || defined(__DOXYGEN__)
#define USE_TMS470R1B1M_UART0           TRUE
#endif

/**
 * @brief   UART1 driver enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_TMS470R1B1M_UART1) || defined(__DOXYGEN__)
#define USE_TMS470R1B1M_UART1           TRUE
#endif

/**
 * @brief   UART2 driver enable switch.
 * @details If set to @p TRUE the support for UART2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_TMS470R1B1M_UART2) || defined(__DOXYGEN__)
#define USE_TMS470R1B1M_UART2           TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   TMS470R1B1M Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /**
   * @brief Initialization value for the CCR register.
   */
  uint32_t                  ccr;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the USART registers block.*/                                \
  SCI_TypeDef               *uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_TMS470R1B1M_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if USE_TMS470R1B1M_UART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if USE_TMS470R1B1M_UART2 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
