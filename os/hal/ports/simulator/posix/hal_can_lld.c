/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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
 * @file    CANv1/hal_can_lld.c
 * @brief   STM32 CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if HAL_USE_CAN || defined(__DOXYGEN__)

#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CAN1 driver identifier.*/
#if SIMULATOR_USE_CAN1 || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/** @brief CAN2 driver identifier.*/
#if SIMULATOR_USE_CAN2 || defined(__DOXYGEN__)
CANDriver CAND2;
#endif

/** @brief CAN3 driver identifier.*/
#if SIMULATOR_USE_CAN3 || defined(__DOXYGEN__)
CANDriver CAND3;
#endif
/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void halcantx_to_socket_can(const CANTxFrame * ctfp, struct can_frame *
frame)
{
  if (ctfp->IDE == 0)
  {
    /* Standard frame format */
    frame->can_id = 0;
    frame->can_id |= ctfp->SID;
  }
  else
  {
    /* Extended frame format */
    frame->can_id = 0;
    frame->can_id |= CAN_EFF_FLAG;
    frame->can_id |= ctfp->EID;
  }

  if (ctfp->RTR == 1)
    frame->can_id |= CAN_RTR_FLAG;

  frame->can_dlc = ctfp->DLC;

  *((uint64_t *)&frame->data[0]) = ctfp->data64[0];
}

static void socket_can_to_halcanrx(CANRxFrame * crfp, const struct can_frame *
frame)
{
  if (frame->can_id & CAN_EFF_FLAG)
  {
    crfp->IDE = 1;
    crfp->EID = frame->can_id & CAN_EFF_MASK;
  }
  else
  {
    crfp->IDE = 0;
    crfp->SID = frame->can_id & CAN_SFF_MASK;
  }

  if (frame->can_id & CAN_RTR_FLAG)
    crfp->RTR = 1;
  else
    crfp->RTR = 0;

  crfp->DLC = frame->can_dlc;

  crfp->data64[0] = *(uint64_t *)&frame->data[0];
}

/**
 * @brief   Determines the FMI from a received message (filter mailbox)
 *
 * @param[in] ctfp      pointer to the CAN frame buffer
 * @param[in] FMI       the resultant frame index
 * @return              TRUE if filter found, FALSE if not found.
 *
 * @notapi
 */
static bool can_lld_get_fmi(CANRxFrame * crfp, uint8_t * FMI) {
  (void)crfp;
  (void)FMI;
  return true;
}

/**
 * @brief   Common RX ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_rx_handler(CANDriver *canp) {
  struct can_frame frame;
  CANRxFrame * crfp;
  bool post_event = false;
  while (1)
  {
    int r = recv(canp->socketfd, &frame, sizeof(struct can_frame),
                 MSG_DONTWAIT);
    if (r < 0)
    {
      if (errno != EAGAIN)
        perror("can_lld_rx_handler: Error in recv from socket");
      break;
    }

    osalDbgAssert(r == sizeof(struct can_frame), "Unexpected length of received CAN frame");

    chSysLockFromISR();
    crfp = (CANRxFrame *)chFifoTakeObjectI(&canp->rxmbx_fifo);
    chSysUnlockFromISR();
    if (crfp == NULL)
    {
      continue;
    }
    socket_can_to_halcanrx(crfp, &frame);
    if (can_lld_get_fmi(crfp, &(crfp->FMI)))
    {
      chSysLockFromISR();
      chFifoSendObjectI(&canp->rxmbx_fifo, crfp);
      chSysUnlockFromISR();
      post_event = true;
    }
    else
    {
      chDbgAssert(true, "hey, everything should go thru!");
    }
  }

  if (post_event)
  {
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(1));
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SIMULATOR_USE_CAN1 || defined(__DOXYGEN__)
/**
 * @brief   CAN1 RX interrupt handler.
 *
 * @isr
 */
void can_lld_rx_int_handler0(void) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SIMULATOR_USE_CAN1 */

#if SIMULATOR_USE_CAN2 || defined(__DOXYGEN__)
/**
 * @brief   CAN2 RX interrupt handler.
 *
 * @isr
 */
void can_lld_rx_int_handler1(void) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx_handler(&CAND2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SIMULATOR_USE_CAN2 */

#if SIMULATOR_USE_CAN3 || defined(__DOXYGEN__)
/**
 * @brief   CAN1 RX interrupt handler.
 *
 * @isr
 */
void can_lld_rx_int_handler2(void) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx_handler(&CAND3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SIMULATOR_USE_CAN3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

    osalDbgAssert(sizeof(CANFilter) == sizeof(struct can_filter), "CANFilter does not match struct can_filter");
#if SIMULATOR_USE_CAN1
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.socketfd = -1;
#endif

#if SIMULATOR_USE_CAN2
  /* Driver initialization.*/
  canObjectInit(&CAND2);
  CAND2.socketfd = -1;
#endif

#if SIMULATOR_USE_CAN3
  /* Driver initialization.*/
  canObjectInit(&CAND3);
  CAND3.socketfd = -1;
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_start(CANDriver *canp) {

  const char * ifname = canp->config->ifname;
  osalDbgAssert(ifname != NULL, "socketCAN interface name cannot be empty");
  osalSysUnlock();
  chFifoObjectInitAligned(&canp->rxmbx_fifo, sizeof(CANRxFrame),
    SIM_CANLLD_RX_QLEN, PORT_NATURAL_ALIGN,
    (void *)canp->rxmbx_objbuf, canp->rxmbx_msgbuf);
  if ((canp->socketfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("can_lld_start:Error while opening socket");
    osalSysHalt("Error while opening socket");
  }

  struct ifreq ifr;
  strcpy(ifr.ifr_name, ifname);
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    osalSysHalt("Error in getting interface index");
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(canp->socketfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("can_lld_start:Error in socket bind");
    osalSysHalt("Error in socket bind");
  }
  osalSysLock();
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  /* If in ready state then disables the CAN peripheral.*/
  if (canp->state == CAN_READY) {
    if (close(canp->socketfd) < 0)
    {
      perror("can_lld_stop:Error in socket close");
      osalSysHalt("Error in socket close");
    }
  }
}
/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox) {
  (void)mailbox;

  struct pollfd request =
  {
    .fd = canp->socketfd,
    .events = POLLOUT,
    .revents = 0
  };
  int ret = poll(&request, 1, 0);
  if (ret == 0)
    return false;
  if (ret > 0)
  {
    if (request.revents & (POLLERR | POLLNVAL))
      return false;
    if (request.revents & (POLLOUT))
      return true;
  }
  if (ret < 0)
  {
    perror("can_lld_is_tx_empty:Error in polling on socket");
    return false;
  }
  return false;
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp,
                      canmbx_t mailbox,
                      const CANTxFrame *ctfp) {
(void)mailbox;

  struct can_frame frame = {0};
  halcantx_to_socket_can(ctfp, &frame);
  int ret = write(canp->socketfd, &frame, sizeof(struct can_frame));
  if (ret < 0)
  {
    perror("can_lld_transmit:Error in transmitting CAN frame");
  }
#if (CAN_ENFORCE_USE_CALLBACKS == FALSE)
  osalThreadDequeueAllI(&(canp)->txqueue, MSG_OK);
  /*
   * There is only one mailbox
   */
  osalEventBroadcastFlagsI(&(canp)->txempty_event, CAN_MAILBOX_TO_MASK(1));
#else
  if ((canp)->txempty_cb != NULL) {
    /*
     * There is only one mailbox
     */
    (canp)->txempty_cb(canp, CAN_MAILBOX_TO_MASK(1));
  }
  osalThreadDequeueAllI(&(canp)->txqueue, MSG_OK);
#endif
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {
  (void)mailbox;
  return (chMBGetUsedCountI(&canp->rxmbx_fifo.mbx) > 0);
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp,
                     canmbx_t mailbox,
                     CANRxFrame *crfp) {
  (void)mailbox;
  CANRxFrame * crfp_temp;
  if (MSG_OK == chFifoReceiveObjectI(&canp->rxmbx_fifo, (void **)&crfp_temp))
  {
    memcpy(crfp, crfp_temp, sizeof(CANRxFrame));
    chFifoReturnObjectI(&canp->rxmbx_fifo, crfp_temp);
  }
}

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {
  (void)canp;
  return;
}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {
  (void)canp;
  return;
}
#endif /* CAN_USE_SLEEP_MODE */

/**
 * @brief   Programs the filters.
 * @note    This is an STM32-specific API.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] can2sb    number of the first filter assigned to CAN2
 * @param[in] num       number of entries in the filters array, if zero then
 *                      a default filter is programmed
 * @param[in] cfp       pointer to the filters array, can be @p NULL if
 *                      (num == 0)
 *
 * @api
 */
void canSimulationSetFilters(CANDriver *canp, size_t num, const CANFilter *cfp)
{
    osalDbgCheck(canp != NULL);
    osalDbgCheck(canp->socketfd > 0);
    struct can_filter filter;
    if ((cfp == NULL) || (num == 0))
    {
        filter.can_mask = 0;
        if (setsockopt(canp->socketfd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(struct can_filter)) < 0)
        {
            osalDbgAssert(false, "Can't set filter");
        }
    }
    else
    {
        if (setsockopt(canp->socketfd, SOL_CAN_RAW, CAN_RAW_FILTER, cfp, num*sizeof(struct can_filter)) < 0)
        {
            osalDbgAssert(false, "Can't set filter");
        }
    }
}

bool can_lld_interrupt_pending0(void)
{
#if SIMULATOR_USE_CAN1
  if (CAND1.socketfd >= 0)
  {
    struct pollfd request =
    {
      .fd = CAND1.socketfd,
      .events = POLLIN,
      .revents = 0
    };
    int ret = poll(&request, 1, 0);
    if (ret == 0)
      return false;
    if (ret > 0)
    {
      if (request.revents & (POLLERR | POLLNVAL))
        return false;
      if (request.revents & (POLLIN))
        return true;
    }
    if (ret < 0)
    {
      perror("can_lld_interrupt_pending0:Error in POLLIN on socket");
      return false;
    }
  }
#endif
  return false;
}

bool can_lld_interrupt_pending1(void)
{
#if SIMULATOR_USE_CAN2
  if (CAND2.socketfd >= 0)
  {
    struct pollfd request =
    {
      .fd = CAND2.socketfd,
      .events = POLLIN,
      .revents = 0,
    };
    int ret = poll(&request, 1, 0);
    if (ret == 0)
      return false;
    if (ret > 0)
    {
      if (request.revents & (POLLERR | POLLNVAL))
        return false;
      if (request.revents & (POLLIN))
        return true;
    }
    if (ret < 0)
    {
      perror("can_lld_interrupt_pending0:Error in POLLIN on socket");
      return false;
    }
  }
#endif
  return false;
}

bool can_lld_interrupt_pending2(void)
{
#if SIMULATOR_USE_CAN3
  if (CAND3.socketfd >= 0)
  {
    struct pollfd request =
    {
      .fd = CAND3.socketfd,
      .events = POLLIN,
      .revents = 0,
    };
    int ret = poll(&request, 1, 0);
    if (ret == 0)
      return false;
    if (ret > 0)
    {
      if (request.revents & (POLLERR | POLLNVAL))
        return false;
      if (request.revents & (POLLIN))
        return true;
    }
    if (ret < 0)
    {
      perror("can_lld_interrupt_pending0:Error in POLLIN on socket");
      return false;
    }
  }
#endif
  return false;
}

#endif /* HAL_USE_CAN */

/** @} */
