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
 * @file    SIMIA32/chcore.h
 * @brief   Simulator on IA32 port macros and structures.
 *
 * @addtogroup SIMIA32_GCC_CORE
 * @{
 */

#ifndef CHCORE_H
#define CHCORE_H

#include <pthread.h>
#include <stdbool.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Port Capabilities and Constants
 * @{
 */
/**
 * @brief   This port supports a realtime counter.
 */
#define PORT_SUPPORTS_RT                TRUE

/**
 * @brief   Natural alignment constant.
 * @note    It is the minimum alignment for pointer-size variables.
 */
#define PORT_NATURAL_ALIGN              sizeof (void *)

/**
 * @brief   Stack alignment constant.
 * @note    It is the alignment required for the stack pointer.
 */
#define PORT_STACK_ALIGN                sizeof (stkalign_t)

/**
 * @brief   Working Areas alignment constant.
 * @note    It is the alignment to be enforced for thread working areas.
 */
#define PORT_WORKING_AREA_ALIGN         sizeof (stkalign_t)
/** @} */

/**
 * @name    Architecture and Compiler
 * @{
 */
/**
 * Macro defining the a simulated architecture into x86.
 */
#define PORT_ARCHITECTURE_SIMIA32

/**
 * Name of the implemented architecture.
 */
#define PORT_ARCHITECTURE_NAME          "Simulator pthread"

/**
 * @brief   Name of the architecture variant (optional).
 */
#define PORT_CORE_VARIANT_NAME          "x86"

/**
 * @brief   Name of the compiler supported by this port.
 */
#define PORT_COMPILER_NAME              "GCC " __VERSION__

/**
 * @brief   Port-specific information string.
 */
#define PORT_INFO                       "No preemption"
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Stack size for the system idle thread.
 * @details This size depends on the idle thread implementation, usually
 *          the idle thread should take no more space than those reserved
 *          by @p PORT_INT_REQUIRED_STACK.
 */
#if !defined(PORT_IDLE_THREAD_STACK_SIZE) || defined(__DOXYGEN__)
#define PORT_IDLE_THREAD_STACK_SIZE     256
#endif

/**
 * @brief   Per-thread stack overhead for interrupts servicing.
 * @details This constant is used in the calculation of the correct working
 *          area size.
 */
#if !defined(PORT_INT_REQUIRED_STACK) || defined(__DOXYGEN__)
#define PORT_INT_REQUIRED_STACK         16384
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_DBG_ENABLE_STACK_CHECK
#error "option CH_DBG_ENABLE_STACK_CHECK not supported by this port"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef void (*tfunc_t)(void *p);

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   16 bytes stack and memory alignment enforcement.
 */
typedef struct {
  uint8_t a[16];
} stkalign_t __attribute__((aligned(16)));

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during a
 *          preemption-capable interrupt handler.
 */
struct port_extctx {
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switch.
 */
struct port_intctx {
};

/**
 * @brief   Platform dependent part of the @p thread_t structure.
 * @details This structure usually contains just the saved stack pointer
 *          defined as a pointer to a @p port_intctx structure.
 */
struct port_context {
    /**
     * @brief   Thread function pointer.
     */
    tfunc_t funcp;
    /**
     * @brief   Thread argument.
     */
    void * arg;
    pthread_t pthread;
    pthread_mutex_t sync;
    pthread_cond_t cond;
    bool cond_trig;
    /** Dummy stack pointer var to make shell_cmd.c happy. This is also used as
     * a flag to cancel a pthread (pointed to by pThdToCancel) upon resume. 1 =
     * cancel, 0 = do nothing
     */
    uint32_t sp;
    pthread_t * pThdToCancel;
};

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Port level context switch
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself or from within the port layer.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 *
 * @special
 */
#define port_switch(ntp, otp) {                                           \
    _port_switch(ntp, otp);                                               \
}

/**
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p port_intctx structure.
 */
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) {                      \
    _port_setup_context(tp, (size_t)wbase, (size_t)wtop, pf, (void *)arg);  \
}

/**
 * @brief   Computes the thread working area global size.
 * @note    There is no need to perform alignments in this macro.
 */
#define PORT_WA_SIZE(n) ((sizeof (void *) * 4U) +                           \
                         sizeof (struct port_intctx) +                      \
                         ((size_t)(n)) +                                    \
                         ((size_t)(PORT_INT_REQUIRED_STACK)))

/**
 * @brief   Static working area allocation.
 * @details This macro is used to allocate a static thread working area
 *          aligned as both position and size.
 *
 * @param[in] s         the name to be assigned to the stack array
 * @param[in] n         the stack size to be assigned to the thread
 */
#define PORT_WORKING_AREA(s, n)                                             \
  stkalign_t s[THD_WORKING_AREA_SIZE(n) / sizeof (stkalign_t)]

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n) false

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n) false

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_PROLOGUE() {                                               \
  port_isr_context_flag = true;                                             \
}

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE() {                                               \
  port_isr_context_flag = false;                                            \
}

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#ifdef __cplusplus
#define PORT_IRQ_HANDLER(id) extern "C" void id(void)
#else
#define PORT_IRQ_HANDLER(id) void id(void)
#endif

/**
 * @brief   Fast IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#ifdef __cplusplus
#define PORT_FAST_IRQ_HANDLER(id) extern "C" void id(void)
#else
#define PORT_FAST_IRQ_HANDLER(id) void id(void)
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

extern bool port_isr_context_flag;
extern syssts_t port_irq_sts;

#ifdef __cplusplus
extern "C" {
#endif

  void port_init(os_instance_t *oip);
  void _port_enter_critical(void);
  void _port_exit_critical(void);
  void _port_setup_context(thread_t *tp, size_t wbase, size_t wtop, tfunc_t pf, void *arg);
  void _port_switch(thread_t *ntp, thread_t *otp);
  /*lint -restore*/
  rtcnt_t port_rt_get_counter_value(void);
  void _sim_check_for_interrupts(void);
#ifdef __cplusplus
}
#endif

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Returns a word encoding the current interrupts status.
 *
 * @return              The interrupts status.
 */
static inline syssts_t port_get_irq_status(void) {

  return port_irq_sts;
}

/**
 * @brief   Checks the interrupt status.
 *
 * @param[in] sts       the interrupt status word
 *
 * @return              The interrupt status.
 * @retval false        the word specified a disabled interrupts status.
 * @retval true         the word specified an enabled interrupts status.
 */
static inline bool port_irq_enabled(syssts_t sts) {

  return sts == (syssts_t)0;
}

/**
 * @brief   Determines the current execution context.
 *
 * @return              The execution context.
 * @retval false        not running in ISR mode.
 * @retval true         running in ISR mode.
 */
static inline bool port_is_isr_context(void) {

  return port_isr_context_flag;
}

/**
 * @brief   Kernel-lock action.
 * @details In this port this function disables interrupts globally.
 */
static inline void port_lock(void) {

  _port_enter_critical();
}

/**
 * @brief   Kernel-unlock action.
 * @details In this port this function enables interrupts globally.
 */
static inline void port_unlock(void) {

  _port_exit_critical();
}

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @details In this port this function disables interrupts globally.
 * @note    Same as @p port_lock() in this port.
 */
static inline void port_lock_from_isr(void) {

  port_lock();
}

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @details In this port this function enables interrupts globally.
 * @note    Same as @p port_lock() in this port.
 */
static inline void port_unlock_from_isr(void) {

  port_unlock();
}

/**
 * @brief   Disables all the interrupt sources.
 */
static inline void port_disable(void) {

  port_lock();
}

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 */
static inline void port_suspend(void) {

  port_lock();
}

/**
 * @brief   Enables all the interrupt sources.
 */
static inline void port_enable(void) {

  port_unlock();
}

/**
 * @brief   Enters an architecture-dependent IRQ-waiting mode.
 * @details The function is meant to return when an interrupt becomes pending.
 *          The simplest implementation is an empty function or macro but this
 *          would not take advantage of architecture-specific power saving
 *          modes.
 * @note    Implemented as an inlined @p WFI instruction.
 */
static inline void port_wait_for_interrupt(void) {

  _sim_check_for_interrupts();
}

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module late inclusions.                                                   */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

#if CH_CFG_ST_TIMEDELTA > 0
#include "chcore_timer.h"
#endif /* CH_CFG_ST_TIMEDELTA > 0 */

#endif /* !defined(_FROM_ASM_) */

#endif /* CHCORE_H */

/** @} */
