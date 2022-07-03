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
 * @file    TMS470R1B1M/hal_lld.c
 * @brief   TMS470R1B1M HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
/** Maximum address within the RAM */
const uint32_t MAX_RAM_ADDR = 0x00810000;
/** Minimum address within the RAM */
const uint32_t MIN_RAM_ADDR = 0x00800000;
/** Starting address of code */
const uint32_t CODE_START_ADDR = 0x00000000;
/** Starting address of external bus */
const uint32_t EXTBUS_BASE = 0x00900000;
/** External Address Bus Wait States */
const uint8_t EBM_WAITSTATES=0x0; /* 15-Wait States Max*/
/** External Data Bus Width */
const bool EBM_SIXTEENBIT=1; /* 0-8-bit, 1-16-bit */
#define PLL_MULTIPLIER_VALUES_LEN	(1)
#define PLL_PRESCALE_VALUES_LEN		(8)
static const uint32_t sPllMultiplierValues[PLL_MULTIPLIER_VALUES_LEN] = {8};
static const uint32_t sPllPrescaleValues[PLL_PRESCALE_VALUES_LEN]= {1,2,3,4,5,6,7,8};
#define DATA_START_ADDR     				(0x00080000)	/** Starting address of Data Flash */

/** Main CPU clock frequency in Hz */
static volatile uint32_t sMclkHz;
/** Peripheral clock */
static volatile uint32_t sIclkHz;

uint32_t getMclk(void) { return sMclkHz; }
uint32_t getIclk(void) { return sIclkHz; }

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

    nonvic_init();
}

__attribute__((naked))
void __cpu_init(void) {}

__attribute__((naked))
void __early_init (void)
{
    asm("msr CPSR_c, 0x13 | 0x80 | 0x40\n\t");
	SYSCNTL->GLBCTRL = 0x00000009;
    SYSCNTL->CLKCNTL = 0x00000090;      // Disable unused peripheral clocks
    SYSREGS->PCR = 0x01;                // ICLK = SYSCLK and ICLK is enabled
    SYSREGS->SMCR1 = 0x00000072;    	// Configure memory access times for HET

    SYSCNTL->GLBCTRL |= 0x00000010;		// Switch to flash control mode

    // If BAC2 is changed here, it should also be changed in prog.c
    //
    FLASH16->BAC2 = 0x00007F11;			// Set to minimum wait states

	FLASH32->REGOPT |= 0x00000001;		// Turn on pipeline mode
    SYSCNTL->GLBCTRL &= ~0x00000010;	// Out of flash control mode

    /* Setup Expansion Bus settings for Waitstates and Databus Width */
    /* If you are not going to use external bus, do not configure the external
     * Address and Data bus pins and it should work fine. Basically do not call
     * configEBM() from main unless you need to use external expansion bus.
     */
	SYSREGS->SMCR5 = 0x00000000 | ((EBM_WAITSTATES & 0xF) << 4)|(1<<2)|EBM_SIXTEENBIT;
	EBM->EBMCR1 = 0x00 | (EBM_SIXTEENBIT <<1);/*8/16-bit, Local Power Down disabled */

    MEMREGS->MFBAHR1 = DATA_START_ADDR>>16; // Flash block 2 start
    MEMREGS->MFBALR1 = 0x000000A0;          //  size  = 0x00080000 (512K)
    MEMREGS->MFBAHR2 = MIN_RAM_ADDR>>16;    // RAM block 1   start
    MEMREGS->MFBALR2 = 0x00000070;          //  size  = 0x00010000 (64K)
    MEMREGS->MFBAHR3 = 0x00000000;          // RAM block 2   start
    MEMREGS->MFBALR3 = 0x00000000;          //  size  = 0x00000000 (0K)
    MEMREGS->MFBAHR4 = HETRAM_BASE>>16;    	// HET RAM block start
    MEMREGS->MFBALR4 = 0x00000010;          //  size  = 0x00000400 (1K)
    MEMREGS->MCBAHR2 = EXTBUS_BASE>>16;    	// External bus start
    MEMREGS->MCBALR2 = 0x00000060;          //  size  = 0x00100000 (1M)
    MEMREGS->MFBAHR0 = CODE_START_ADDR>>16; // Flash block 1 start
    MEMREGS->MFBALR0 = 0x000001A0;          //  size  = 0x00080000 (512K)
    MPUREGS->MPUCTRL = 0x00000000;          // Disable MPU
    asm("msr CPSR_c, 0x1F | 0x80 | 0x40\n\t");
}


/**
 * @brief   TMS470R1B1M clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void tms470r1b1m_late_init(void)
{

	volatile int i, i_found, j, j_found, ret = 0;
	volatile uint32_t tmpFreqError, freqError;

	freqError = UINT32_MAX;
	for (i = 0; i < PLL_MULTIPLIER_VALUES_LEN; i++)
	{
		for (j = 0; j < PLL_PRESCALE_VALUES_LEN; j++)
		{
			tmpFreqError = TMS470R1B1M_CPU_HZ - (TMS470R1B1M_OSC_HZ*sPllMultiplierValues[i])/sPllPrescaleValues[j];
			if (tmpFreqError < 0)
			{
				tmpFreqError*=-1;
			}
			if (tmpFreqError < freqError)
			{
				i_found = i;
				j_found = j;
				freqError = tmpFreqError;
			}
		}
	}

	SYSCNTL->GLBCTRL = SYS_CLK_DIV_PRE(sPllPrescaleValues[j_found]-1);
	SYSCNTL->GLBCTRL &= ~SYS_MULT_4;
	sMclkHz = (float)TMS470R1B1M_OSC_HZ*(float)sPllMultiplierValues[i_found]/sPllPrescaleValues[j_found];
	sIclkHz = sMclkHz / (((SYSREGS->PCR & 0x1E) >> 1) + 1);

    for(i=0; i<TOTAL_SYS_INTS; i++)
    {
    	IEMREGS->INTCTRL[i] = PRIORITY_NOT_ASSIGNED;
    }
}

/** @} */
