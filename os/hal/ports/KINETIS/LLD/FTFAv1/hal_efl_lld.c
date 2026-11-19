/*
    ChibiOS - Copyright (C) 2006..2023 Giovanni Di Sirio
    Copyright (C) 2026 Mecha Systems

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
 * @file    FTFAv1/hal_efl_lld.c
 * @brief   Kinetis FTFA Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include "hal.h"

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* FTFA commands.*/
#define KINETIS_FTFA_CMD_PROGRAM_LONGWORD   0x06U
#define KINETIS_FTFA_CMD_ERASE_SECTOR       0x09U

/* FSTAT error flags, cleared by writing one.*/
#define KINETIS_FTFA_FSTAT_ERRORS           (FTFA_FSTAT_RDCOLERR |          \
                                             FTFA_FSTAT_ACCERR |            \
                                             FTFA_FSTAT_FPVIOL)

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

/* The program flash starts at address zero. Its size is read from the device
   in efl_lld_init() because one device selection (e.g. KL27Zxxx) covers parts
   with different flash sizes.*/
static flash_descriptor_t efl_lld_descriptor = {
  .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                       FLASH_ATTR_MEMORY_MAPPED,
  .page_size         = KINETIS_FLASH_LINE_SIZE,
  .sectors_count     = 0U,
  .sectors           = NULL,
  .sectors_size      = KINETIS_FLASH_SECTOR_SIZE,
  .address           = (uint8_t *)0x00000000U,
  .size              = 0U
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Returns the program flash size in bytes.
 * @details Decodes SIM_FCFG1[PFSIZE], zero for an undefined code.
 */
static uint32_t kinetis_flash_size(void) {
  static const uint16_t pfsize_kb[16] = {
    [0x0] = 8U,
    [0x1] = 16U,
    [0x3] = 32U,
    [0x5] = 64U,
    [0x7] = 128U,
    [0x9] = 256U,
    [0xF] = 256U
  };
  uint32_t pfsize = (SIM->FCFG1 & SIM_FCFG1_PFSIZE_MASK) >>
                    SIM_FCFG1_PFSIZE_SHIFT;

  return (uint32_t)pfsize_kb[pfsize] * 1024U;
}

/**
 * @brief   Launches the command loaded in FCCOB and waits for completion.
 * @note    The flash cannot be read while a command is modifying it, so this
 *          function runs from RAM and must be called with interrupts
 *          disabled.
 */
CC_NO_INLINE CC_SECTION(".ramtext")
static void kinetis_ftfa_launch_and_wait(FTFA_TypeDef *flash) {

  flash->FSTAT = FTFA_FSTAT_CCIF;
  while ((flash->FSTAT & FTFA_FSTAT_CCIF) == 0U) {
  }
}

/**
 * @brief   Waits for the previous command, clears the error flags and loads
 *          the command code and the flash address.
 */
static void kinetis_ftfa_prepare(EFlashDriver *devp, uint8_t command,
                                 uint32_t address) {
  FTFA_TypeDef *flash = devp->flash;

  while ((flash->FSTAT & FTFA_FSTAT_CCIF) == 0U) {
  }
  flash->FSTAT  = KINETIS_FTFA_FSTAT_ERRORS;
  flash->FCCOB0 = command;
  flash->FCCOB1 = (uint8_t)(address >> 16);
  flash->FCCOB2 = (uint8_t)(address >> 8);
  flash->FCCOB3 = (uint8_t)address;
}

/**
 * @brief   Runs the loaded command.
 * @return              @p true if the command completed without errors.
 */
static bool kinetis_ftfa_execute(EFlashDriver *devp) {
  FTFA_TypeDef *flash = devp->flash;

  osalSysLock();
  kinetis_ftfa_launch_and_wait(flash);
  osalSysUnlock();

  /* The flash memory controller caches flash contents, the cache is cleared
     so that reads return the new contents.*/
  MCM->PLACR |= MCM_PLACR_CFCC;

  return (flash->FSTAT & (KINETIS_FTFA_FSTAT_ERRORS |
                          FTFA_FSTAT_MGSTAT0)) == 0U;
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

  efl_lld_descriptor.size          = kinetis_flash_size();
  efl_lld_descriptor.sectors_count = efl_lld_descriptor.size /
                                     KINETIS_FLASH_SECTOR_SIZE;

  /* Driver initialization.*/
  eflObjectInit(&EFLD1);
  EFLD1.flash = FTFA;
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {

  (void)eflp;
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {

  (void)eflp;
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @return              A flash device descriptor.
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
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes to be read
 * @param[out] rp       pointer to the data buffer
 * @return              An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const volatile uint8_t *p;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)efl_lld_descriptor.size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* The program flash starts at address zero, so the copy uses a volatile
     pointer instead of memcpy() with a pointer that can be NULL.*/
  p = (const volatile uint8_t *)(efl_lld_descriptor.address + offset);
  while (n > 0U) {
    *rp++ = *p++;
    n--;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Program operation.
 * @note    The FTFA programs aligned 4-byte longwords, @p offset and @p n
 *          must be multiples of @p KINETIS_FLASH_LINE_SIZE.
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes to be programmed
 * @param[in] pp        pointer to the data buffer
 * @return              An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)efl_lld_descriptor.size);
  osalDbgCheck(((offset % KINETIS_FLASH_LINE_SIZE) == 0U) &&
               ((n % KINETIS_FLASH_LINE_SIZE) == 0U));
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  while (n > 0U) {
    kinetis_ftfa_prepare(devp, KINETIS_FTFA_CMD_PROGRAM_LONGWORD,
                         (uint32_t)efl_lld_descriptor.address + offset);

    /* The FCCOB7 byte is programmed at the given address and the FCCOB4
       byte at the address plus three.*/
    devp->flash->FCCOB4 = pp[3];
    devp->flash->FCCOB5 = pp[2];
    devp->flash->FCCOB6 = pp[1];
    devp->flash->FCCOB7 = pp[0];

    if (!kinetis_ftfa_execute(devp)) {
      err = FLASH_ERROR_PROGRAM;
      break;
    }

    pp     += KINETIS_FLASH_LINE_SIZE;
    offset += KINETIS_FLASH_LINE_SIZE;
    n      -= KINETIS_FLASH_LINE_SIZE;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    Not supported, it would erase the running firmware.
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @return              An error code.
 * @retval FLASH_ERROR_UNIMPLEMENTED
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {

  (void)instance;

  return FLASH_ERROR_UNIMPLEMENTED;
}

/**
 * @brief   Starts an sector erase operation.
 * @note    The erase completes before this function returns.
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @param[in] sector    sector to be erased
 * @return              An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  kinetis_ftfa_prepare(devp, KINETIS_FTFA_CMD_ERASE_SECTOR,
                       (uint32_t)efl_lld_descriptor.address +
                       flashGetSectorOffset(getBaseFlash(devp), sector));
  if (!kinetis_ftfa_execute(devp)) {
    devp->state = FLASH_READY;
    return FLASH_ERROR_ERASE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 * @note    The erase has already completed in efl_lld_start_erase_sector().
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @param[out] wait_time recommended time, in milliseconds, that
 *                      should be spent before calling this
 *                      function again, can be @p NULL
 * @return              An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *wait_time) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  (void)wait_time;

  if (devp->state == FLASH_ERASE) {
    devp->state = FLASH_READY;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] instance  pointer to a @p EFlashDriver instance
 * @param[in] sector    sector to be verified
 * @return              An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const volatile uint32_t *address;
  flash_error_t err = FLASH_NO_ERROR;
  uint32_t i;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  address = (const volatile uint32_t *)(efl_lld_descriptor.address +
                       flashGetSectorOffset(getBaseFlash(devp), sector));
  for (i = 0U; i < KINETIS_FLASH_SECTOR_SIZE / sizeof(uint32_t); i++) {
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
