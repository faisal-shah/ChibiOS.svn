#pragma once

#if HAL_USE_CAN || defined(__DOXYGEN__)

#include <linux/can.h>
#include "hal_can_lld_defs.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This switch defines whether the driver implementation supports
 *          a low power switch mode with automatic an wakeup feature.
 */
#define CAN_SUPPORTS_SLEEP          FALSE

/**
 * @brief   This implementation supports one transmit mailboxes.
 */
#define CAN_TX_MAILBOXES            1

/**
 * @brief   This implementation supports one receive mailboxes.
 */
#define CAN_RX_MAILBOXES            1
#define SIM_CANLLD_RX_QLEN          (300)

#define CAN_IDE_STD                 0           /**< @brief Standard id.    */
#define CAN_IDE_EXT                 1           /**< @brief Extended id.    */

#define CAN_RTR_DATA                0           /**< @brief Data frame.     */
#define CAN_RTR_REMOTE              1           /**< @brief Remote frame.   */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SIMULATOR_USE_CAN1) || defined(__DOXYGEN__)
#define SIMULATOR_USE_CAN1          TRUE
#endif

/**
 * @brief   CAN2 driver enable switch.
 * @details If set to @p TRUE the support for CAN2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SIMULATOR_USE_CAN2) || defined(__DOXYGEN__)
#define SIMULATOR_USE_CAN2          TRUE
#endif

/**
 * @brief   CAN3 driver enable switch.
 * @details If set to @p TRUE the support for CAN3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SIMULATOR_USE_CAN3) || defined(__DOXYGEN__)
#define SIMULATOR_USE_CAN3          TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/


#if CAN_USE_SLEEP_MODE && !CAN_SUPPORTS_SLEEP
#error "CAN sleep mode not supported in this architecture"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an CAN driver.
 */
typedef struct CANDriver CANDriver;

/**
 * @brief   Type of a transmission mailbox index.
 */
typedef uint32_t canmbx_t;

#if defined(CAN_ENFORCE_USE_CALLBACKS) || defined(__DOXYGEN__)
/**
 * @brief   Type of a CAN notification callback.
 *
 * @param[in] canp      pointer to the @p CANDriver object triggering the
 *                      callback
 * @param[in] flags     flags associated to the mailbox callback
 */
typedef void (*can_callback_t)(CANDriver *canp, uint32_t flags);
#endif

/**
 * @brief   CAN filter.
 * @note    Refer to the Linux socket CAN for info about filters.
 */
 typedef struct can_filter CANFilter;

/**
 * @brief   Structure representing an CAN driver.
 */
struct CANDriver {
  /**
   * @brief   Driver state.
   */
  canstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CANConfig           *config;
  /**
   * @brief   Transmission threads queue.
   */
  threads_queue_t           txqueue;
  /**
   * @brief   Receive threads queue.
   */
  threads_queue_t           rxqueue;
#if (CAN_ENFORCE_USE_CALLBACKS == FALSE) || defined(__DOXYGEN__)
  /**
   * @brief   One or more frames become available.
   * @note    After broadcasting this event it will not be broadcasted again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> broadcasted for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p canReceive() when listening to this event.
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   * @note    The flags associated to the listeners will indicate which
   *          receive mailboxes become non-empty.
   */
  event_source_t            rxfull_event;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the listeners will indicate which
   *          transmit mailboxes become empty.
   * @note    The upper 16 bits are transmission error flags associated
   *          to the transmit mailboxes.
   */
  event_source_t            txempty_event;
  /**
   * @brief   A CAN bus error happened.
   * @note    The flags associated to the listeners will indicate that
   *          receive error(s) have occurred.
   * @note    In this implementation the upper 16 bits are filled with the
   *          unprocessed content of the ESR register.
   */
  event_source_t            error_event;
#if CAN_USE_SLEEP_MODE || defined (__DOXYGEN__)
  /**
   * @brief   Entering sleep state event.
   */
  event_source_t            sleep_event;
  /**
   * @brief   Exiting sleep state event.
   */
  event_source_t            wakeup_event;
#endif /* CAN_USE_SLEEP_MODE */
#else /* defined(CAN_ENFORCE_USE_CALLBACKS) */
  /**
   * @brief   One or more frames become available.
   * @note    After calling this function it will not be called again
   *          until the received frames queue has been completely emptied. It
   *          is <b>not</b> called for each received frame. It is
   *          responsibility of the application to empty the queue by
   *          repeatedly invoking @p chTryReceiveI().
   *          This behavior minimizes the interrupt served by the system
   *          because CAN traffic.
   */
  can_callback_t            rxfull_cb;
  /**
   * @brief   One or more transmission mailbox become available.
   * @note    The flags associated to the callback will indicate which
   *          transmit mailboxes become empty.
   */
  can_callback_t            txempty_cb;
  /**
   * @brief   A CAN bus error happened.
   */
  can_callback_t            error_cb;
#if (CAN_USE_SLEEP_MODE == TRUE) || defined (__DOXYGEN__)
  /**
   * @brief   Exiting sleep state.
   */
  can_callback_t            wakeup_cb;
#endif
#endif
  /* End of the mandatory fields.*/
  objects_fifo_t            rxmbx_fifo;
  uint8_t                   rxmbx_objbuf[(SIM_CANLLD_RX_QLEN * sizeof(CANRxFrame))];
  msg_t                     rxmbx_msgbuf[SIM_CANLLD_RX_QLEN];
  int                       socketfd;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#define can_lld_abort(canp,mailbox)

#if SIMULATOR_USE_CAN1 && !defined(__DOXYGEN__)
extern CANDriver CAND1;
#endif

#if SIMULATOR_USE_CAN2 && !defined(__DOXYGEN__)
extern CANDriver CAND2;
#endif

#if SIMULATOR_USE_CAN3 && !defined(__DOXYGEN__)
extern CANDriver CAND3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void can_lld_init(void);
  void can_lld_start(CANDriver *canp);
  void can_lld_stop(CANDriver *canp);
  bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_transmit(CANDriver *canp,
                        canmbx_t mailbox,
                        const CANTxFrame *crfp);
  bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox);
  void can_lld_receive(CANDriver *canp,
                       canmbx_t mailbox,
                       CANRxFrame *ctfp);
#if CAN_USE_SLEEP_MODE
  void can_lld_sleep(CANDriver *canp);
  void can_lld_wakeup(CANDriver *canp);
#endif /* CAN_USE_SLEEP_MODE */
void canSimulationSetFilters(CANDriver *canp, size_t num, const CANFilter *cfp);
void can_lld_rx_int_handler0(void);
void can_lld_rx_int_handler1(void);
void can_lld_rx_int_handler2(void);
bool can_lld_interrupt_pending0(void);
bool can_lld_interrupt_pending1(void);
bool can_lld_interrupt_pending2(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CAN */
