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
 * @file    TMS470R1B1M/vic.c
 * @brief   TMS470R1B1M VIC peripheral support code.
 *
 * @addtogroup TMS470R1B1M_VIC
 * @{
 */

#include "hal.h"

/** Dummy function for unhandled interrupts */
static void UnhandledInterrupt(void) {}

/** Global containing the handlers for the various interrupts */
static void (*gIrqHandlers[TOTAL_INT_LINES])(void) = {
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt, &UnhandledInterrupt,
    &UnhandledInterrupt, &UnhandledInterrupt};

static unsigned gNextAvailablePriority = 0;

/*
 * Non-vectored IRQs handler, the default action can be overridden by
 * redefining the @p TMS470_NON_VECTORED_IRQ_HOOK() hook macro.
 */
CH_IRQ_HANDLER(irq_handler) {

  CH_IRQ_PROLOGUE();

  TMS470_NON_VECTORED_IRQ_HOOK();

  int intVec;

  intVec = (0xFF & INTREGS->IRQIVEC) - 1;
  if(intVec >= TOTAL_INT_LINES) return true;
  gIrqHandlers[intVec]();

  CH_IRQ_EPILOGUE();
}

static bool isValidIrq(uint8_t intNum)
{
	if(intNum >= TOTAL_SYS_INTS)
	{
		return FALSE;
	}

	if(IEMREGS->INTCTRL[intNum] != PRIORITY_NOT_ASSIGNED)
	{
		return FALSE;
	}

	if( gNextAvailablePriority >= TOTAL_INT_LINES )
	{
		return FALSE;
	}

	return TRUE;
}

static void registerIrq(uint8_t intNum)
{
	if(isValidIrq(intNum))
	{
		IEMREGS->INTCTRL[intNum] = gNextAvailablePriority;    /* Set the CIM register */
		INTREGS->REQMASK |= (1 << gNextAvailablePriority);    /* Enable the interrupt */

		gNextAvailablePriority++;
	}
}

int registerIrqHandler(void *newHandler, uint8_t intNum)
{
	if(!isValidIrq(intNum)) return -1;

    gIrqHandlers[gNextAvailablePriority] = newHandler;    /* Set the handler */

    registerIrq(intNum);

    return 0;
}

int registerFiq(uint8_t intType)
{
	if(!isValidIrq(intType)) return -1;

	IEMREGS->INTCTRL[intType] = gNextAvailablePriority;    /* Set the CIM register */
    INTREGS->FIRQPR |= (1 << gNextAvailablePriority);
    INTREGS->REQMASK |= (1 << gNextAvailablePriority);    /* Enable the interrupt */

    gNextAvailablePriority++;

    return 0;
}

/**
 * @brief   VIC Initialization.
 * @note    Better reset everything in the VIC, it is a HUGE source of trouble.
 *
 * @notapi
 */
void nonvic_init(void) {
    asm("msr CPSR_c, 0x13 | 0x80 | 0x40\n\t");
    for (size_t i = 0; i < TOTAL_SYS_INTS; i++)
    {
        IEMREGS->INTCTRL[i] = PRIORITY_NOT_ASSIGNED;
    }
    asm("msr CPSR_c, 0x1F | 0x80 | 0x40\n\t");
}

/** @} */
