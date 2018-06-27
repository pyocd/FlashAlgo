/**
 ******************************************************************************
 * @file    flash.h
 * @author  Stef Servaes, Pierre Lebas, Pradeep Kumar G R
 * $Rev: 152 $
 * $Date: 2012-02-01 12:26:50 +0100 (Wed, 01 Feb 2012) $
 * @brief	(API) Public header of Flash driver
 ******************************************************************************
 * @copyright (c) 2012 ON Semiconductor. All rights reserved.
 * @internal
 * ON Semiconductor is supplying this software for use with ON Semiconductor
 * processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 * @endinternal
 *
 * @ingroup FLASH
 *
 * @details
 *
 * <h1> General description </h1>
 * <p>
 * </p>
 */
#ifndef _FLASH_H_
#define _FLASH_H_

#include "flash_map.h"
#include <stdint.h>

//MPL
#define SIREV 3

#define FLASH_MASS_ERASE_REQUEST           (uint8_t)0x01
#define FLASH_PAGE_ERASE_REQUEST           (uint8_t)0x02
#define FLASH_POWER_UP                     (uint8_t)0x03
#define FLASH_POWER_DOWN                    (uint8_t)0x04
#define FLASH_WAIT_UNTIL_DONE               (uint8_t)0x05

#if (SIREV == 2)
#define FLASH_PAGE_SIZE                     (uint16_t)0x1000
#define FLASH_PAGE_SIZE_INFO_BLOCK          (uint16_t)0x1000
#define FLASH_NR_INFO_BLOCK_PAGES           (uint8_t)0x02
#define FLASH_PAGE_MASK                     (uint32_t)0xFFFFF000
#define FLASH_FIRST_PAGE_MASK               (uint32_t)0xFFFFF000

#endif

#if (SIREV == 3)
#define FLASH_PAGE_SIZE                     (uint16_t)0x800
#define FLASH_PAGE_SIZE_INFO_BLOCK          (uint16_t)0x800
#define FLASH_NR_INFO_BLOCK_PAGES           (uint8_t)0x04
#define FLASH_PAGE_MASK                     (uint32_t)0xFFFFF800
#define FLASH_FIRST_PAGE_MASK               (uint32_t)0xFFFFF800
#endif

#define FLASH_A_OFFSET_MASK                 (uint32_t)0x00000000
#define FLASH_A_USER_AREA_OFFSET            (uint32_t)0x00002000
#define FLASH_B_OFFSET_MASK                 (uint32_t)0x00100000
#define FLASH_B_USER_AREA_OFFSET            (uint32_t)0x00102000
#define FLASH_B_OFFSET_SHIFT                (uint8_t)20

#define FLASHREG_BASE                       ((uint32_t)0x40017000)
#define FLASHREG                            ((FlashReg_t *)FLASHREG_BASE)

/* Only one controll register for both flash banks,
 * Only one interrupt line for both, hence, no irqn here.
 */
typedef struct flash_options {
     uint32_t       array_base_address;     /**< Base address of the array, word aligned */
     FlashReg_pt    membase;
     IRQn_Type      irq;                  /**< The IRQ number of the IRQ associated to the device. */
} flash_options_t, *flash_options_pt;

/**
 * Flash Interrupt handler
 */
void fFlashHandler(FlashReg_pt membase);

boolean fFlashOpen(flash_options_pt device);
void fFlashClose(flash_options_pt device);
boolean fFlashRead(flash_options_pt device, uint8_t **address, uint8_t * const buf, uint32_t len);
boolean fFlashWrite(flash_options_pt device, uint8_t **address, const uint8_t *buf, uint32_t len);
boolean fFlashIoctl(flash_options_pt device, uint32_t request, void *argument);

void fFlashPageErase(flash_options_pt device, uint32_t address);
void fFlashPowerUp(flash_options_pt device);
void fFlashStallUntilNotBusy(flash_options_pt device);
void fFlashMassErase(flash_options_pt device);

#endif // FLASH_H_
