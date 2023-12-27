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
 * @file    simulator/posix/hal_serial_lld.h
 * @brief   Posix simulator low level serial driver header.
 *
 * @addtogroup POSIX_SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Serial buffers size.
 * @details Configuration parameter, you can change the depth of the queue
 *          buffers depending on the requirements of your application.
 */
#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_BUFFERS_SIZE                 1024
#endif

/**
 * @brief   SD1 driver enable switch.
 * @details If set to @p TRUE the support for SD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_SIM_SERIAL1) || defined(__DOXYGEN__)
#define USE_SIM_SERIAL1                     TRUE
#endif

/**
 * @brief   SD2 driver enable switch.
 * @details If set to @p TRUE the support for SD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_SIM_SERIAL2) || defined(__DOXYGEN__)
#define USE_SIM_SERIAL2                     TRUE
#endif

/**
 * @brief   Listen port for SD1.
 */
#if !defined(SD1_PORT) || defined(__DOXYGEN__)
#define SIM_SD1_PORT                        29001
#endif

/**
 * @brief   Listen port for SD2.
 */
#if !defined(SD2_PORT) || defined(__DOXYGEN__)
#define SIM_SD2_PORT                        29002
#endif

/**
 * @brief   Enable TTY based serial drivers as alternative to socket,
 *          should be defined as "TRUE"/"FALSE" in mcuconf.h. Default is FALSE
 */
#if !defined(USE_SIM_SERIAL_TTY) || defined(__DOXYGEN__)
#define USE_SIM_SERIAL_TTY   FALSE
#endif

/*===========================================================================*/
/* Unsupported event flags and custom events.                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Generic Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
#if USE_SIM_SERIAL_TTY == TRUE || defined(__DOXYGEN__)
  /**
   * @brief   Device name with path: e.g, /dev/ttyS0, /dev/pts/0
   */
  const char * dev_node;
  /**
   * @brief   If True, setup termios. Otherwise do not, just open the port.
   */
  bool setup_termios;
  /**
   * @brief   Should be of type speed_t: B9600, B115200, etc..see termios.h
   */
  uint32_t speed;
  /**
   * @brief   Parity control: 0 = off, 1 = odd, 2 = even.
   */
  int parity;
  /**
   * @brief   Data bit length: 7,8 valid values.
   */
  int data_size;
  /**
   * @brief   Num of stop bits: 1,2 stop bits
   */
  int stop_bit;
  /**
   * @brief   Flow Ctl: 0 = off, 1 = on (XON/XOFF software flow control only)
   */
  int flow_control;
#endif
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#if USE_SIM_SERIAL_TTY
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
  /* pseudo terminal device name with path, e.g, /dev/pts/1 */              \
  const char * dev_node;                                                    \
  /* file descriptor associated with an open tty device */                  \
  int fd;
#else
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
  /* Listen socket for simulated serial port.*/                             \
  int                       com_listen;                                     \
  /* Data socket for simulated serial port.*/                               \
  int                       com_data;                                       \
  /* Port readable name.*/                                                  \
  const char                *com_name;
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_SIM_SERIAL1 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if USE_SIM_SERIAL2 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#if USE_SIM_SERIAL_TTY
  bool sd_lld_interrupt_pending(SerialDriver * sdp);
  void sd_lld_irq_handler_serial1(void);
  void sd_lld_irq_handler_serial2(void);
#else
  bool sd_lld_interrupt_pending(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
