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
 * @brief   STM32F722/23/32/33 Embedded Flash subsystem low level
 *          driver source.
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

const flash_sector_descriptor_t efl_lld_sectors[] = {
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
};

static const flash_descriptor_t efl_lld_descriptor = {
 .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                      FLASH_ATTR_MEMORY_MAPPED |
                      FLASH_ATTR_ECC_CAPABLE   |
                      FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
 .page_size         = STM32_FLASH_LINE_SIZE,
 .sectors_count     = 8U,
 .sectors           = efl_lld_sectors,
 .sectors_size      = 0U,
 .address           = (uint8_t *)FLASHAXI_BASE,
 .size              = STM32_FLASH_TOTAL_SIZE,
};

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

  eflp->flash->SR |= (FLASH_SR_ERSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |
          FLASH_SR_WRPERR | FLASH_SR_EOP);
}

static inline void stm32_flash_wait_busy(EFlashDriver *eflp) {

  /* Wait for busy bit clear.*/
  while ((eflp->flash->SR & FLASH_SR_BSY) != 0U);
}

static inline flash_error_t stm32_flash_get_errors(EFlashDriver *eflp) {
  uint32_t sr = eflp->flash->SR;

  /* Clearing error conditions.*/
  eflp->flash->SR = sr & 0x0000FFFFU;

  /* Decoding relevant errors.*/
  if ((sr & FLASH_SR_RDERR) != 0U) {
    return FLASH_ERROR_READ;
  }
  else if ((sr & FLASH_SR_ERSERR) != 0U) {
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
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {

  (void)instance;

  return &efl_lld_descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
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
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)efl_lld_descriptor.size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READY state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Actual read implementation.*/
  memcpy((void *)rp, (const void *)efl_lld_descriptor.address + offset, n);

  /* Checking for errors after reading.*/
  if ((devp->flash->SR & FLASH_SR_RDERR) != 0U) {
    err = FLASH_ERROR_READ;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}

/**
 * @brief   Program operation.
 * @note    The device supports ECC, it is only possible to write erased
 *          pages once except when writing all zeroes.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
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
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;
  size_t total_size = n;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)efl_lld_descriptor.size);

  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

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
    memset(line, 0xFF, sizeof(line));

    /* Programming address aligned to flash lines.*/
    address = (volatile uint32_t *)(efl_lld_descriptor.address +
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
    err = stm32_flash_get_errors(devp);
    if (err != FLASH_NO_ERROR) {
      break;
    }
  }

  /* Disabling PGM mode in the controller.*/
  stm32_flash_disable_pgm(devp);

  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(efl_lld_descriptor.address +
          offset), total_size);
  SCB_InvalidateICache();

  osalSysRestoreStatusX(sts);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function doesn't do anything because the device only has one
 *          bank - from which the current program is running.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {

  (void)instance;
  return FLASH_NO_ERROR;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {

  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  devp->state = FLASH_ERASE;

  /* Clearing error status bits.*/
  stm32_flash_clear_status(devp);

  /* Ensure flash is not busy */
  stm32_flash_wait_busy(devp);

  /* Enable sector erase.*/
  devp->flash->CR |= FLASH_CR_SER;

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


  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(efl_lld_descriptor.address +
          efl_lld_descriptor.sectors[sector].offset),
          efl_lld_descriptor.sectors[sector].size);
  SCB_InvalidateICache();

  osalSysRestoreStatusX(sts);

  stm32_flash_wait_busy(devp);
  devp->flash->CR &= ~FLASH_CR_SER;

  err = stm32_flash_get_errors(devp);
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
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {

    /* Checking for operation in progress.*/
    if ((devp->flash->SR & FLASH_SR_BSY) == 0U) {

      /* Disabling the various erase control bits.*/
      devp->flash->CR &= ~(FLASH_CR_MER | FLASH_CR_SER);

      /* No operation in progress, checking for errors.*/
      err = stm32_flash_get_errors(devp);

      /* Back to ready state.*/
      devp->state = FLASH_READY;
    }
    else {
      /* Recommended time before polling again, this is a simplified
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
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t *address;
  uint32_t sector_size;
  flash_error_t err = FLASH_NO_ERROR;
  unsigned i;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Address of the sector.*/
  address = (uint32_t *)(efl_lld_descriptor.address +
                         flashGetSectorOffset(getBaseFlash(devp), sector));
  sector_size = flashGetDescriptor(devp)->sectors[sector].size;

  /* FLASH_READY state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Scanning the sector space.*/
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
