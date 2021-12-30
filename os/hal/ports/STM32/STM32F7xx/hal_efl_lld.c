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
 * @file    hal_efl_lld.c
 * @brief   STM32F7xx Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define FLASH_KEY1                          0x45670123U
#define FLASH_KEY2                          0xCDEF89ABU

#define STM32_FLASH_SIZE_REGISTER_SCALE     (1024U)

#if defined(FLASH_CR_MER2)
#define STM32_IS_DUAL_BANK                  (1U)
#else
#define STM32_IS_DUAL_BANK                  (0U)
#endif

/**
 * @brief Maximum program/erase parallelism
 *
 * FLASH_CR_PSIZE_MASK is the mask to configure the parallelism value.
 * FLASH_CR_PSIZE_VALUE is the parallelism value suitable for the voltage range.
 *
 * PSIZE(1:0) is defined as:
 * 00 to program 8 bits per step
 * 01 to program 16 bits per step
 * 10 to program 32 bits per step
 * 11 to program 64 bits per step (unused in this driver, as this requires
 *                                external Vpp - programming voltage - to
 *                                be applied)
 */

#define FLASH_CR_PSIZE_MASK         (FLASH_CR_PSIZE_0 | FLASH_CR_PSIZE_1)
#if ((STM32_VDD >= 270) && (STM32_VDD <= 360))
#define FLASH_CR_PSIZE_VALUE        (FLASH_CR_PSIZE_1)
#define STM32_FLASH_LINE_SIZE       (4U)
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define FLASH_CR_PSIZE_VALUE        (FLASH_CR_PSIZE_0)
#define STM32_FLASH_LINE_SIZE       (2U)
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define FLASH_CR_PSIZE_VALUE        (FLASH_CR_PSIZE_0)
#define STM32_FLASH_LINE_SIZE       (2U)
#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define FLASH_CR_PSIZE_VALUE        (0U)
#define STM32_FLASH_LINE_SIZE       (1U)
#else
#error "invalid VDD voltage specified"
#endif

#define STM32_FLASH_LINE_MASK               (STM32_FLASH_LINE_SIZE - 1U)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier.
 */
EFlashDriver EFLD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F732xx) || \
    defined(STM32F733xx) || defined(STM32F730xx)
const flash_sector_descriptor_t efl_lld_sectors[FLASH_SECTOR_TOTAL] = {
    {
        .offset = 0U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 2U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 3U*16U*1024U,
        .size   = 16U*1024U,
    },
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F732xx) || \
    defined(STM32F733xx)
    {
        .offset = 4U*16U*1024U,
        .size   = 64U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 1U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 2U*128U*1024U,
        .size   = 128U*1024U,
    },
#endif
};

static const flash_descriptor_t efl_lld_descriptor = {
 .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                      FLASH_ATTR_MEMORY_MAPPED |
                      FLASH_ATTR_ECC_CAPABLE   |
                      FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
 .page_size         = STM32_FLASH_LINE_SIZE,
 .sectors_count     = FLASH_SECTOR_TOTAL,
 .sectors           = efl_lld_sectors,
 .sectors_size      = 0U,
 .address           = (uint8_t *)FLASHAXI_BASE,
 .size              = FLASH_END - FLASHAXI_BASE + 1U,
};
#endif

#if defined(STM32F756xx) || defined(STM32F745xx) || defined(STM32F746xx) || \
    defined(STM32F750xx)
const flash_sector_descriptor_t efl_lld_sectors[FLASH_SECTOR_TOTAL] = {
    {
        .offset = 0U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 1U*32U*1024U,
        .size   = 32U*1024U,
    },
#if defined(STM32F756xx) || defined(STM32F745xx) || defined(STM32F746xx)
    {
        .offset = 2U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 3U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 4U*32U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 1U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 2U*256U*1024U,
        .size   = 256U*1024U,
    },
#endif
};

static const flash_descriptor_t efl_lld_descriptor = {
 .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                      FLASH_ATTR_MEMORY_MAPPED |
                      FLASH_ATTR_ECC_CAPABLE   |
                      FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
 .page_size         = STM32_FLASH_LINE_SIZE,
 .sectors_count     = FLASH_SECTOR_TOTAL,
 .sectors           = efl_lld_sectors,
 .sectors_size      = 0U,
 .address           = (uint8_t *)FLASHAXI_BASE,
 .size              = FLASH_END - FLASHAXI_BASE + 1U,
};
#endif

#if defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F777xx) || defined(STM32F779xx)
#define FLASH_SECTOR_TOTAL_1M   (16U)
#define FLASH_SECTOR_TOTAL_2M   (24U)
const flash_sector_descriptor_t efl_lld_sectors_single_1M[FLASH_SECTOR_TOTAL_1M/2] = {
    {
        .offset = 0U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 1U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 2U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 3U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 4U*32U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 1U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 2U*256U*1024U,
        .size   = 256U*1024U,
    },
};

const flash_sector_descriptor_t efl_lld_sectors_dual_1M[FLASH_SECTOR_TOTAL_1M] = {
    /* First bank */
    {
        .offset = 0U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 2U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 3U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 4U*16U*1024U,
        .size   = 64U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 1U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 2U*128U*1024U,
        .size   = 128U*1024U,
    },
    /* Second bank */
    {
        .offset = 256U*1024U + 0U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 256U*1024U + 1U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 256U*1024U + 2U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 256U*1024U + 3U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 256U*1024U + 4U*16U*1024U,
        .size   = 64U*1024U,
    },
    {
        .offset = 256U*1024U + 4U*16U*1024U + 64U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 256U*1024U + 4U*16U*1024U + 64U*1024U + 1U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 256U*1024U + 4U*16U*1024U + 64U*1024U + 2U*128U*1024U,
        .size   = 128U*1024U,
    },
};


const flash_sector_descriptor_t efl_lld_sectors_single_2M[FLASH_SECTOR_TOTAL_2M/2] = {
    {
        .offset = 0U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 1U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 2U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 3U*32U*1024U,
        .size   = 32U*1024U,
    },
    {
        .offset = 4U*32U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 1U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 2U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 3U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 4U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 5U*256U*1024U,
        .size   = 256U*1024U,
    },
    {
        .offset = 4U*32U*1024U + 128U*1024U + 6U*256U*1024U,
        .size   = 256U*1024U,
    },
};

const flash_sector_descriptor_t efl_lld_sectors_dual_2M[FLASH_SECTOR_TOTAL_2M] = {
    /* First bank */
    {
        .offset = 0U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 2U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 3U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 4U*16U*1024U,
        .size   = 64U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 1U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 2U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 3U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 4U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 5U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 4U*16U*1024U + 64U*1024U + 6U*128U*1024U,
        .size   = 128U*1024U,
    },
    /* Second bank */
    {
        .offset = 1024U*1024U + 0U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1024U*1024U + 1U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1024U*1024U + 2U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1024U*1024U + 3U*16U*1024U,
        .size   = 16U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U,
        .size   = 64U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 1U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 2U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 3U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 4U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 5U*128U*1024U,
        .size   = 128U*1024U,
    },
    {
        .offset = 1024U*1024U + 4U*16U*1024U + 64U*1024U + 6U*128U*1024U,
        .size   = 128U*1024U,
    },

};

/* The descriptors for 1M device. */
static const flash_descriptor_t efl_lld_size1[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Bank 1 (SBM) organisation. */
        .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                             FLASH_ATTR_MEMORY_MAPPED |
                             FLASH_ATTR_ECC_CAPABLE   |
                             FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
        .page_size         = STM32_FLASH_LINE_SIZE,
        .sectors_count     = FLASH_SECTOR_TOTAL_1M/2,
        .sectors           = efl_lld_sectors_single_1M,
        .sectors_size      = 0U,
        .address           = (uint8_t *)FLASHAXI_BASE,
        .size              = 1U*1024U*1024U,
      },
      { /* Bank 1 & 2 (DBM) organisation. */
        .attributes        = FLASH_ATTR_ERASED_IS_ONE |
            FLASH_ATTR_MEMORY_MAPPED |
            FLASH_ATTR_ECC_CAPABLE   |
            FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
        .page_size         = STM32_FLASH_LINE_SIZE,
        .sectors_count     = FLASH_SECTOR_TOTAL_1M,
        .sectors           = efl_lld_sectors_dual_1M,
        .sectors_size      = 0U,
        .address           = (uint8_t *)FLASHAXI_BASE,
        .size              = 1U*1024U*1024U,
      }
};

/* The descriptors for 2M device. */
static const flash_descriptor_t efl_lld_size2[STM32_FLASH_NUMBER_OF_BANKS] = {
      { /* Bank 1 (SBM) organisation. */
        .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                             FLASH_ATTR_MEMORY_MAPPED |
                             FLASH_ATTR_ECC_CAPABLE   |
                             FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
        .page_size         = STM32_FLASH_LINE_SIZE,
        .sectors_count     = FLASH_SECTOR_TOTAL_2M/2,
        .sectors           = efl_lld_sectors_single_2M,
        .sectors_size      = 0U,
        .address           = (uint8_t *)FLASHAXI_BASE,
        .size              = 2U*1024U*1024U,
      },
      { /* Bank 1 & 2 (DBM) organisation. */
        .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                             FLASH_ATTR_MEMORY_MAPPED |
                             FLASH_ATTR_ECC_CAPABLE   |
                             FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
        .page_size         = STM32_FLASH_LINE_SIZE,
        .sectors_count     = FLASH_SECTOR_TOTAL_2M,
        .sectors           = efl_lld_sectors_dual_2M,
        .sectors_size      = 0U,
        .address           = (uint8_t *)FLASHAXI_BASE,
        .size              = 2U*1024U*1024U,
      }
};

/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
      {
       .desc = efl_lld_size1
      },
      {
       .desc = efl_lld_size2
      }
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void stm32_flash_lock(EFlashDriver *eflp) {

  eflp->flash->CR |= FLASH_CR_LOCK;
}

static inline void stm32_flash_unlock(EFlashDriver *eflp) {

  eflp->flash->KEYR |= FLASH_KEY1;
  eflp->flash->KEYR |= FLASH_KEY2;
}

static inline void stm32_flash_enable_pgm(EFlashDriver *eflp) {

  eflp->flash->CR |= FLASH_CR_PG;
}

static inline void stm32_flash_disable_pgm(EFlashDriver *eflp) {

  eflp->flash->CR &= ~FLASH_CR_PG;
}

static inline void stm32_flash_clear_status(EFlashDriver *eflp) {

  eflp->flash->SR = 0x0000FFFFU;
}

static inline void stm32_flash_wait_busy(EFlashDriver *eflp) {

  /* Wait for busy bit clear.*/
  while ((eflp->flash->SR & FLASH_SR_BSY) != 0U) {
  }
}

static inline size_t stm32_flash_get_size(void) {
  return *(uint16_t*)((uint32_t) FLASHSIZE_BASE) * STM32_FLASH_SIZE_REGISTER_SCALE;
}

static inline bool stm32_flash_dual_bank(EFlashDriver *eflp) {

#if STM32_IS_DUAL_BANK
  return ((eflp->flash->OPTCR & FLASH_OPTCR_nDBANK) == 0U);
#else
  (void)eflp;
  return false;
#endif
}

static inline flash_error_t stm32_flash_check_errors(EFlashDriver *eflp) {

  uint32_t sr = eflp->flash->SR;

  /* Clearing error conditions.*/
  eflp->flash->SR = sr & 0x0000FFFFU;

  /* Decoding relevant errors.*/
#if defined(FLASH_SR_RDERR)
  if ((sr & FLASH_SR_RDERR) != 0U) {
    return FLASH_ERROR_READ;
  }
#endif

  if ((sr & FLASH_SR_ERSERR) != 0U) {
    return FLASH_ERROR_ERASE;
  }

  else if ((sr & (FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR)) != 0U) {
    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void) {

  /* Driver initialization.*/
  eflObjectInit(&EFLD1);
  EFLD1.flash = FLASH;
#if STM32_IS_DUAL_BANK
  /* Find the size of the flash and set descriptor reference. */
  uint8_t i;
  for (i = 0; i < (sizeof(efl_lld_flash_sizes) / sizeof(efl_lld_size_t)); i++) {
    if (efl_lld_flash_sizes[i].desc->size == stm32_flash_get_size()) {
      EFLD1.descriptor = efl_lld_flash_sizes[i].desc;
      if (stm32_flash_dual_bank(&EFLD1)) {
        /* Point to the dual bank descriptor. */
        EFLD1.descriptor++;
      }
      return;
    }
  }
  osalDbgAssert(false, "invalid flash configuration");
#else
  EFLD1.descriptor = &efl_lld_descriptor;
#endif
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {
  stm32_flash_unlock(eflp);
  FLASH->CR = 0x00000000U;
  FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
  FLASH->CR |= FLASH_CR_PSIZE_VALUE;
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {

  stm32_flash_lock(eflp);
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 * @retval                          Pointer to single bank if DBM not enabled.
 * @retval                          Pointer to bank1 if DBM enabled.
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {

  EFlashDriver *devp = (EFlashDriver *)instance;
  return devp->descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));

  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Actual read implementation.*/
  memcpy((void *)rp, (const void *)(bank->address + offset), n);

  /* Checking for errors after reading.*/
  err = stm32_flash_check_errors(devp);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}

/**
 * @brief   Program operation.
 * @note    The device supports ECC. It is only possible to write erased
 *          pages once except when writing all zeroes to a location.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;
  size_t total_size = n;

  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Ensure flash is not busy */
  stm32_flash_wait_busy(devp);

  syssts_t sts = osalSysGetStatusAndLockX();

  /* Note: If an erase operation in Flash memory also concerns data in the ART
   * accelerator, the user has to make sure that these data are rewritten
   * before they are accessed during code execution. If this cannot be done
   * safely, it is recommended to flush and/or desactivate the ART accelerator
   * by setting respectively the bits ARTRST or ARTEN of the FLASH_CR register.
   * */
  devp->flash->ACR &= ~FLASH_ACR_ARTEN;
  devp->flash->ACR |= ~FLASH_ACR_ARTRST;
  devp->flash->ACR |= FLASH_ACR_ARTEN;

  /* Enabling PGM mode in the controller.*/
  stm32_flash_enable_pgm(devp);

  /* Actual program implementation.*/
  while (n > 0U) {
    volatile uint32_t *address;

    uint8_t line[STM32_FLASH_LINE_SIZE];
    /* Unwritten bytes are initialized to all ones.*/
    memset(line, 0xFF, sizeof(line));

    /* Programming address aligned to flash lines.*/
    address = (volatile uint32_t *)(bank->address +
                                    (offset & ~STM32_FLASH_LINE_MASK));

    /* Copying data inside the prepared line.*/
    do {
        *(line + (offset & STM32_FLASH_LINE_MASK)) = *pp;
        offset++;
        n--;
        pp++;
    } while ((n > 0U) & ((offset & STM32_FLASH_LINE_MASK) != 0U));

    /* Programming line.*/
#if STM32_FLASH_LINE_SIZE == 4
    *((uint32_t *)address) = *((uint32_t *)line);
#elif STM32_FLASH_LINE_SIZE == 2
    *((uint16_t *)address) = *((uint16_t *)line);
#elif STM32_FLASH_LINE_SIZE == 1
    *((uint8_t *)address) = *((uint8_t *)line);
#else
#error "STM32_FLASH_LINE_SIZE invalid"
#endif

    /* ensure the completion of write operation */
    __DSB();

    stm32_flash_wait_busy(devp);
    err = stm32_flash_check_errors(devp);
    if (err != FLASH_NO_ERROR) {
      break;
    }
  }

  /* Disabling PGM mode in the controller.*/
  stm32_flash_disable_pgm(devp);

  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(bank->address + offset),
          total_size);
  SCB_InvalidateICache();

  osalSysRestoreStatusX(sts);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function only erases bank 2 if it is present. Bank 1 is not
 *          allowed since it is normally where the primary program is located.
 *          Pages on bank 1 can be individually erased.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {
  osalDbgCheck(instance != NULL);
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

#if STM32_IS_DUAL_BANK
  /* If dual bank is active then mass erase bank2. */
  if (stm32_flash_dual_bank(devp)) {

    /* FLASH_ERASE state while the operation is performed.*/
    devp->state = FLASH_ERASE;

    /* Clearing error status bits.*/
    stm32_flash_clear_status(devp);

    devp->flash->CR |= FLASH_CR_MER2;

    syssts_t sts = osalSysGetStatusAndLockX();

    /* Note: If an erase operation in Flash memory also concerns data in the ART
     * accelerator, the user has to make sure that these data are rewritten
     * before they are accessed during code execution. If this cannot be done
     * safely, it is recommended to flush and/or desactivate the ART accelerator
     * by setting respectively the bits ARTRST or ARTEN of the FLASH_CR register.
     * */
    devp->flash->ACR &= ~FLASH_ACR_ARTEN;
    devp->flash->ACR |= ~FLASH_ACR_ARTRST;
    devp->flash->ACR |= FLASH_ACR_ARTEN;

    devp->flash->CR |= FLASH_CR_STRT;

    /* ensure the completion of write operation */
    __DSB();

    stm32_flash_wait_busy(devp);

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(bank->address +
                bank->sectors[bank->sectors_count/2U].offset), bank->size/2U);
    SCB_InvalidateICache();

    osalSysRestoreStatusX(sts);

    err = stm32_flash_check_errors(devp);
    return err;
  }
#else
  (void)bank;
  err = FLASH_ERROR_UNIMPLEMENTED;
#endif

  /* Mass erase not allowed. */
  return err;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 *                                  this is an index within the total sectors
 *                                  in a flash bank
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  osalDbgCheck(instance != NULL);
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;
  flash_sector_t sector_save = sector;
  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Ensure flash is not busy */
  stm32_flash_wait_busy(devp);

  /* Enable sector erase.*/
  devp->flash->CR |= FLASH_CR_SER;

#if STM32_IS_DUAL_BANK
/* If dual bank is active then adjust sector number as necessary. */
  if (stm32_flash_dual_bank(devp)) {
    if (sector >= (bank->sectors_count / 2)) {
      /* Second bank. Adjust sector index. */
      sector -= (bank->sectors_count / 2);
      sector |= (1<<4U);
    }
  }
#endif

  /* Mask off the sector selection bits.*/
  devp->flash->CR &= ~FLASH_CR_SNB;

  /* Set the sector selection bits.*/
  devp->flash->CR |= sector << FLASH_CR_SNB_Pos;

  syssts_t sts = osalSysGetStatusAndLockX();

  /* Note: If an erase operation in Flash memory also concerns data in the ART
   * accelerator, the user has to make sure that these data are rewritten
   * before they are accessed during code execution. If this cannot be done
   * safely, it is recommended to flush and/or desactivate the ART accelerator
   * by setting respectively the bits ARTRST or ARTEN of the FLASH_CR register.
   * */
  devp->flash->ACR &= ~FLASH_ACR_ARTEN;
  devp->flash->ACR |= ~FLASH_ACR_ARTRST;
  devp->flash->ACR |= FLASH_ACR_ARTEN;

  /* Start the erase.*/
  devp->flash->CR |= FLASH_CR_STRT;

  /* ensure the completion of write operation */
  __DSB();

  stm32_flash_wait_busy(devp);
  devp->flash->CR &= ~FLASH_CR_SER;

  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(bank->address +
          bank->sectors[sector_save].offset), bank->sectors[sector_save].size);
  SCB_InvalidateICache();

  osalSysRestoreStatusX(sts);


  err = stm32_flash_check_errors(devp);
  return err;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec) {
  osalDbgCheck(instance != NULL);
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {

    /* Checking for operation in progress.*/
    if ((devp->flash->SR & FLASH_SR_BSY) == 0U) {

      /* Disabling the various erase control bits.*/
      devp->flash->CR &= ~(FLASH_CR_MER |
#if STM32_IS_DUAL_BANK
                           FLASH_CR_MER2 |
#endif
                           FLASH_CR_SER);

      /* No operation in progress, checking for errors.*/
      err = stm32_flash_check_errors(devp);

      /* Back to ready state.*/
      devp->state = FLASH_READY;
    }
    else {
      /* Recommended time before polling again. This is a simplified
         implementation.*/
      if (msec != NULL) {
        *msec = (uint32_t)STM32_FLASH_WAIT_TIME_MS;
      }

      err = FLASH_BUSY_ERASING;
    }
  }
  else {
    err = FLASH_NO_ERROR;
  }

  return err;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  osalDbgCheck(instance != NULL);
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t *address;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  flash_error_t err = FLASH_NO_ERROR;
  unsigned i;

  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Address of the sector in the bank.*/
  address = (uint32_t *)(bank->address +
                         flashGetSectorOffset(getBaseFlash(devp), sector));

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Scanning the sector space.*/
  uint32_t sector_size = flashGetSectorSize(getBaseFlash(devp), sector);
  for (i = 0U; i < sector_size / sizeof(uint32_t); i++) {
    if (*address != 0xFFFFFFFFU) {
      err = FLASH_ERROR_VERIFY;
      break;
    }
    address++;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
