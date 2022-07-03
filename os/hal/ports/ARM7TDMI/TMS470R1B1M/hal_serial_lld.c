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
 * @file    TMS470R1B1M/hal_serial_lld.c
 * @brief   TMS470R1B1M low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_TMS470R1B1M_UART0 || defined(__DOXYGEN__)
/** @brief UART0 serial driver identifier.*/
SerialDriver SD1;
#endif

#if USE_TMS470R1B1M_UART1 || defined(__DOXYGEN__)
/** @brief UART1 serial driver identifier.*/
SerialDriver SD2;
#endif

#if USE_TMS470R1B1M_UART2 || defined(__DOXYGEN__)
/** @brief UART2 serial driver identifier.*/
SerialDriver SD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief  Driver default configuration.*/
static const SerialConfig default_config = {
  .speed = SERIAL_DEFAULT_BITRATE,
  .ccr = SCI_TIMING_MODE | SCI_DATA_BITS(8),
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {

  uint32_t baudreg;
  SCI_TypeDef * u = sdp->uart;

  u->CTL3 &= ~SCI_SW_NRESET;           // Reset SCI state machine
  u->CCR = config->ccr;
  u->CTL1 |= SCI_RXENA;                // RX enabled
  u->CTL2 |= SCI_TXENA;                // TX enabled
  baudreg = (getIclk()/(8*config->speed)) - 1;
  u->HBAUD = (baudreg >> 16) & 0xFF;
  u->MBAUD = (baudreg >> 8) & 0xFF;
  u->LBAUD = baudreg & 0xFF;
  u->CTL3 |= SCI_SW_NRESET;            // Configure SCI1 state machine
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(SCI_TypeDef *u) {

  u->CTL3 &= ~SCI_SW_NRESET;           // Reset SCI state machine
  u->CTL1 &= ~SCI_RXENA;                // RX enabled
  u->CTL2 &= ~SCI_TXENA;                // TX enabled
  u->CTL3 |= SCI_SW_NRESET;            // Configure SCI1 state machine
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART LSR register value
 */
static void set_error(SerialDriver *sdp, uint8_t rxst) {

  eventflags_t sts = 0;

  if (rxst & SCI_OE)
    sts |= SD_OVERRUN_ERROR;
  if (rxst & SCI_PE)
    sts |= SD_PARITY_ERROR;
  if (rxst & SCI_FE)
    sts |= SD_FRAMING_ERROR;
  if (rxst & SCI_BRKDT)
    sts |= SD_BREAK_DETECTED;
  chnAddFlagsI(sdp, sts);
}

static void serve_rx_interrupt(SerialDriver *sdp)
{
  SCI_TypeDef * u = sdp->uart;
  uint8_t b;
  uint8_t ctl1 = u->CTL1;
  uint8_t rxst = u->RXST;

  osalSysLockFromISR();
  if (rxst & SCI_RXERR)
  {
      set_error(sdp, rxst);
  }
  if (ctl1 & SCI_RXRDY)
  {
      b = u->RXBUF;
      if (iqIsEmptyI(&sdp->iqueue))
          chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      if (iqPutI(&sdp->iqueue, b) < MSG_OK)
          chnAddFlagsI(sdp, SD_QUEUE_FULL_ERROR);
  }
  osalSysUnlockFromISR();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we dont want
 *          to go through the whole ISR and have another interrupt soon after.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_tx_interrupt(SerialDriver *sdp) {

  SCI_TypeDef * u = sdp->uart;
  uint8_t ctl2 = u->CTL2;

  osalSysLockFromISR();
  /* Transmission buffer empty.*/
  if ((ctl2 & SCI_TXRDY) && (ctl2 & SCI_TXENA)) {
    msg_t b;
    b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      u->CTL3 &= ~SCI_TXACTIONENA;
    }
    else
      u->TXBUF = b;
  }

  /* Physical transmission end.*/
  if ((ctl2 & SCI_TXEMPTY) && (ctl2 & SCI_TXENA)) {
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      u->CTL3 &= ~SCI_TXACTIONENA;
    }
  }
  osalSysUnlockFromISR();
}

/**
 * @brief   Driver SD1 output notification.
 */
#if USE_TMS470R1B1M_UART0 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

    SCI_TypeDef * u = SD1.uart;

    /*
     * If TX interrupt is active, then it's still clocking bits out, and there
     * is no need to reactivate it or write to TXBUF
     */
    if (u->CTL3 & SCI_TXACTIONENA)
    {
        return;
    }

    /* Transmission buffer empty.*/
    osalDbgAssert(u->CTL2 & SCI_TXRDY, "Must be ready if tx interrupt is disabled");
    msg_t b = oqGetI(qp);
    if (b < MSG_OK) {
        chnAddFlagsI(&SD1, CHN_OUTPUT_EMPTY);
        return;
    }
    u->CTL3 |= SCI_TXACTIONENA;
    u->TXBUF = b;
}
#endif

/**
 * @brief   Driver SD2 output notification.
 */
#if USE_TMS470R1B1M_UART1 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

    SCI_TypeDef * u = SD2.uart;

    /*
     * If TX interrupt is active, then it's still clocking bits out, and there
     * is no need to reactivate it or write to TXBUF
     */
    if (u->CTL3 & SCI_TXACTIONENA)
    {
        return;
    }

    /* Transmission buffer empty.*/
    osalDbgAssert(u->CTL2 & SCI_TXRDY, "Must be ready if tx interrupt is disabled");
    msg_t b = oqGetI(qp);
    if (b < MSG_OK) {
        chnAddFlagsI(&SD2, CHN_OUTPUT_EMPTY);
        return;
    }
    u->CTL3 |= SCI_TXACTIONENA;
    u->TXBUF = b;
}
#endif

/**
 * @brief   Driver SD3 output notification.
 */
#if USE_TMS470R1B1M_UART2 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

    SCI_TypeDef * u = SD3.uart;

    /*
     * If TX interrupt is active, then it's still clocking bits out, and there
     * is no need to reactivate it or write to TXBUF
     */
    if (u->CTL3 & SCI_TXACTIONENA)
    {
        return;
    }

    /* Transmission buffer empty.*/
    osalDbgAssert(u->CTL2 & SCI_TXRDY, "Must be ready if tx interrupt is disabled");
    msg_t b = oqGetI(qp);
    if (b < MSG_OK) {
        chnAddFlagsI(&SD3, CHN_OUTPUT_EMPTY);
        return;
    }
    u->CTL3 |= SCI_TXACTIONENA;
    u->TXBUF = b;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 *
 * @isr
 */
#if USE_TMS470R1B1M_UART0 || defined(__DOXYGEN__)
static void uartTxIrqHandler0 (void) {

  serve_tx_interrupt(&SD1);
}

static void uartRxIrqHandler0 (void) {

  serve_rx_interrupt(&SD1);
}
#endif

/**
 * @brief   UART1 IRQ handler.
 *
 * @isr
 */
#if USE_TMS470R1B1M_UART1 || defined(__DOXYGEN__)
static void uartTxIrqHandler1 (void) {

  serve_tx_interrupt(&SD2);
}

static void uartRxIrqHandler1 (void) {

  serve_rx_interrupt(&SD2);
}
#endif

/**
 * @brief   UART3 IRQ handler.
 *
 * @isr
 */
#if USE_TMS470R1B1M_UART2 || defined(__DOXYGEN__)
static void uartTxIrqHandler2 (void) {

  serve_tx_interrupt(&SD3);
}

static void uartRxIrqHandler2 (void) {

  serve_rx_interrupt(&SD3);
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if USE_TMS470R1B1M_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = SCI1;
  SD1.uart->CTL3 &= ~SCI_SW_NRESET;           // Reset SCI state machine
  SD1.uart->CTL3 |= SCI_CLOCK;                // Internal clock
  SD1.uart->PC2 |= SCI_RXFUNC;                // SCIRX is the SCI receive pin
  SD1.uart->PC3 |= SCI_TXFUNC;                // SCITX is the SCI transmit pin
  registerIrqHandler(&uartTxIrqHandler0, IEMNUM_SCI1TX);
  registerIrqHandler(&uartRxIrqHandler0, IEMNUM_SCI1RX);
#endif
#if USE_TMS470R1B1M_UART1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = SCI2;
  SD2.uart->CTL3 &= ~SCI_SW_NRESET;           // Reset SCI state machine
  SD2.uart->CTL3 |= SCI_CLOCK;                // Internal clock
  SD2.uart->PC2 |= SCI_RXFUNC;                // SCIRX is the SCI receive pin
  SD2.uart->PC3 |= SCI_TXFUNC;                // SCITX is the SCI transmit pin
  registerIrqHandler(&uartTxIrqHandler1, IEMNUM_SCI2TX);
  registerIrqHandler(&uartRxIrqHandler1, IEMNUM_SCI2RX);
#endif
#if USE_TMS470R1B1M_UART2
  sdObjectInit(&SD3, NULL, notify3);
  SD3.uart = SCI3;
  SD3.uart->CTL3 &= ~SCI_SW_NRESET;           // Reset SCI state machine
  SD3.uart->CTL3 |= SCI_CLOCK;                // Internal clock
  SD3.uart->PC2 |= SCI_RXFUNC;                // SCIRX is the SCI receive pin
  SD3.uart->PC3 |= SCI_TXFUNC;                // SCITX is the SCI transmit pin
  registerIrqHandler(&uartTxIrqHandler2, IEMNUM_SCI3TX);
  registerIrqHandler(&uartRxIrqHandler2, IEMNUM_SCI3RX);
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_TMS470R1B1M_UART0
    if (&SD1 == sdp) {
      SD1.uart->CTL3 |= SCI_RXACTIONENA;
    }
#endif
#if USE_TMS470R1B1M_UART1
    if (&SD2 == sdp) {
      SD2.uart->CTL3 |= SCI_RXACTIONENA;
    }
#endif
#if USE_TMS470R1B1M_UART2
    if (&SD3 == sdp) {
      SD3.uart->CTL3 |= SCI_RXACTIONENA;
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_TMS470R1B1M_UART0
    if (&SD1 == sdp) {
      SD1.uart->CTL3 &= ~SCI_RXACTIONENA;
      return;
    }
#endif
#if USE_TMS470R1B1M_UART1
    if (&SD2 == sdp) {
      SD2.uart->CTL3 &= ~SCI_RXACTIONENA;
      return;
    }
#endif
#if USE_TMS470R1B1M_UART2
    if (&SD3 == sdp) {
      SD3.uart->CTL3 &= ~SCI_RXACTIONENA;
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
