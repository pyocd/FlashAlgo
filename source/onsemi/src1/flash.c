/**
 ******************************************************************************
 * @file    flash.c
 * @author  ON Semiconductor
 * $Rev:  $
 * $Date: 2015-11-15 $
 * @brief Implementation of a Flash memory driver
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
 */

#include "types.h"                     
#include <stdio.h>

#include "flash_map.h"
#include "flash.h"
#include <string.h>

extern int debug_flag;   
extern uint32_t firmwareAddress;
/** This integer indicates the number of devices connected to the device driver.
 *  In this case the number of devices is 2, which denotes FLASH A and FLASH B
 */
extern uint8_t numDev;

#define ERROR_FLASH_ILLEGAL_SRC_ADDR      (uint8_t) 0x01
#define ERROR_FLASH_ILLEGAL_DST_ADDR      (uint8_t) 0x02
#define ERROR_FLASH_UNSUPPORTED_LEN       (uint8_t) 0x03

#define IRQ_ACCESS_ERROR                  (uint8_t) 0x04
#define IRQ_ERASE_COMPLETE                (uint8_t) 0x02
#define IRQ_PROGRAM_COMPLETE              (uint8_t) 0x01
#define IRQ_NO_INTERRUPT                  (uint8_t) 0x00
#define FLAG_FLASH_RMW                    (uint8_t) 0x01

#define CTRL_POWER_UP                     (uint8_t) 0x00
#define CTRL_POWER_DOWN                   (uint8_t) 0x01
#define CTRL_STALL_ON_WRITE               (uint8_t) 0x01

#define CMD_NOP                           (uint8_t) 0x00
#define CMD_PAGE_ERASE                    (uint8_t) 0x01
#define CMD_MASS_ERASE                    (uint8_t) 0x02

#define CTRL_UNLOCK1                      (uint32_t)0xBB781AE9
#define CTRL_UNLOCK2                      (uint32_t)0xB56D9099

#define MAX_FLASH_DEV                     2  /* 2 flash devices supported */


#ifdef GLOBALCACHE
/**< Global storage to store flash page for read-modify-write operation */
/* Needs 32bit alignement to make sure DMA can handle the data */
/* Note - this feature is not used for Daplink.  Daplink uses its own mem buffer.
   If this Global Cache is enabled, it requires manual adjustment of the mem
   buffer location in flash_blob.c.  To avoid potential issues, it is turned off.
*/

#ifdef IAR
#pragma data_alignment = 4
#endif      /* IAR */
uint8_t GlobFlashPageCache[FLASH_PAGE_SIZE] 
#ifdef GNU
__attribute__((aligned(4)))
#endif     /* GNU */
;   
#endif /* GLOBALCACHE */

/* Two flash memory devices are present */
flash_options_pt flashDev[MAX_FLASH_DEV];

/** Unlock first page for write or erases
 * Note that TEST pin needs to be 1 to unlock.
 * Write of unlock for flash A or B needs to be done within 20clock
 * after writing the overall unlock1 register.
 *
 *  @param device pointer to the flash device
 */
void fFlashUnlock(flash_options_pt device)
{
     device->membase->UNLOCK1 = CTRL_UNLOCK1;

     if (device->array_base_address & FLASH_B_OFFSET_MASK) 
     {/* Flash B */
          device->membase->UNLOCKB = CTRL_UNLOCK2;
     } 
     else
     {/* Flash A */
          device->membase->UNLOCKA = CTRL_UNLOCK2;
     }
}

/** Stalls execution until busy flag is cleared
 *
 * @param device pointer to flash device
 */
void fFlashStallUntilNotBusy(flash_options_pt device)
{
     if (device->array_base_address & FLASH_B_OFFSET_MASK) 
     {/* Check flash B busy */
          while (device->membase->STATUS.BITS.FLASH_B_BUSY);
     }
     else
     {/* Check flash A busy */
          while (device->membase->STATUS.BITS.FLASH_A_BUSY);
     }
}

/** Power down the flash
 *
 * @param device pointer to flash device
 */
void fFlashPowerDown(flash_options_pt device)
{
     if (device->array_base_address & FLASH_B_OFFSET_MASK) 
     {/* Flash B */
          device->membase->CONTROL.BITS.FLASHB_PD = CTRL_POWER_DOWN;
     } 
     else
     {/* Flash A */
          device->membase->CONTROL.BITS.FLASHA_PD = CTRL_POWER_DOWN;
     }
}

/** Power up the flash
 *
 * @param device pointer to flash device
 */
void fFlashPowerUp(flash_options_pt device)
{
     if (device->array_base_address & FLASH_B_OFFSET_MASK) 
     {/* Flash B */
          device->membase->CONTROL.BITS.FLASHB_PD = CTRL_POWER_UP;
     } 
     else
     {/* Flash A */
          device->membase->CONTROL.BITS.FLASHA_PD = CTRL_POWER_UP;
     }
}

/** Erases a flash page
 * It is up to the caller to make sure busy flag is checked before continuing.
 *
 * @param device pointer to the flash device
 * @param address address, relative to flash, does not need to be page start
 */
void fFlashPageErase(flash_options_pt device, uint32_t address)
{
     /* Only one page erase address is available for both flash banks.
      * Hardware is masking least significant bits to identify what page
      * in what flash needs to be erased. */

     fFlashUnlock(device);

     /* Kick off erase cycle. */
     device->membase->ADDR = address;
     device->membase->COMMAND.WORD = CMD_PAGE_ERASE;
}

/** Erases a flash page
 * It is up to the caller to make sure busy flag is checked before continuing.
 *
 * @param device pointer to the flash device
 * @param address address, relative to flash, does not need to be page start
 */
void fFlashMassErase(flash_options_pt device)
{
     /* Only one mass erase control address is available for both flash banks.
      * Hardware is masking least significant bits to identify what page
      * in what flash needs to be erased. */
     if (device->array_base_address & FLASH_B_OFFSET_MASK) 
     {/* Flash device is on flash bank B. Add bank B offset to address. */
          device->membase->ADDR = FLASH_B_USER_AREA_OFFSET;
     } else 
     {
          device->membase->ADDR = FLASH_A_USER_AREA_OFFSET;
     }

     fFlashUnlock(device);

     /* Kick off erase cycle. */
     device->membase->COMMAND.WORD = CMD_MASS_ERASE;
}

/**
 * see driver.h for details
 */
void fFlashClose(flash_options_pt device) 
{
     /* TODO Optional ? */
}

/**
 * Read buffer from flash and increment address pointer
 *
 * Note that if flash is implemented on silicon, there's no need to call read function since read
 * is supported by regular memory read access.  Read function is to be used when large sections
 * of flash need to be copied in RAM, eg for write.
 *
 */
boolean fFlashRead(flash_options_pt device, uint8_t **address, uint8_t * const buf, uint32_t len)
{
     uint8_t *destination = buf;
     uint8_t *source;

     source = (uint8_t*) (*address);

     while (len-- > 0)
     {
          *destination++ = *source++;
     }
     *address = source;

     return True;
}

/**
 * Note: bootloader section (first 8K) can only be flashed when the test pin is 1 and the flash is
 * unlocked.
 *
 * @todo add support for at least 1 write across page boundary
 */
boolean fFlashWrite(flash_options_pt device, uint8_t **address, const uint8_t *buf, uint32_t len)
{
     uint8_t *destination;
     uint8_t *in_page_offset;
     const uint8_t *buffer;

     destination = (uint8_t*) (*address);
     buffer  = buf;

     //fprintf("Destination = %0x", destination);
       
     if ((uint32_t) (*address) < (FLASH_NR_INFO_BLOCK_PAGES * FLASH_PAGE_SIZE_INFO_BLOCK )) 
     {
          // First page write requested, info block has FLASH_NR_INFO_BLOCK_PAGES pages of size FLASH_PAGE_SIZE_INFO_BLOCK.

          /* No support for crossing page boundary */
          if (((uint32_t) (*address) % FLASH_PAGE_SIZE_INFO_BLOCK ) == 0) 
          {
               // Address is page aligned
               if (len > FLASH_PAGE_SIZE_INFO_BLOCK)
               {
                    return False;
               }
          }
          else 
          {
               in_page_offset = (uint8_t*) ((uint32_t) (*address) & ~FLASH_FIRST_PAGE_MASK );
               if (len > ((uint32_t)FLASH_PAGE_SIZE_INFO_BLOCK - (uint32_t)in_page_offset ))     
               {
                    return False;
               }
          }
     }

     /* No support for crossing page boundary */
     if (((uint32_t) (*address) % FLASH_PAGE_SIZE ) == 0)
     {
          // Address is page aligned
          if (len > FLASH_PAGE_SIZE)
          {
            //length is larger than a page - not supported
               return False;
          }
     } 
     else 
     {
          in_page_offset = (uint8_t*) ((uint32_t) (*address) & ~FLASH_PAGE_MASK );
          if (len > ((uint32_t)FLASH_PAGE_SIZE - (uint32_t)in_page_offset ))     
          {
               //length is larger than amount of space remaining before the start of the next page - not supported
               return False;
          }
     }

    /* Make sure the CM3 hangs while write is ongoing */
    device->membase->CONTROL.BITS.WRITE_BLOCK = CTRL_STALL_ON_WRITE;
    
    fFlashUnlock(device);

    /* Commit the page to flash */
    memcpy((void *)destination, (void *)buffer, len);
    fFlashStallUntilNotBusy(device);

     return True;
}

/**
 * Flash IO control function, supports call to erase, power on and off of flash
 * bank
 *
 * @param device pointer to the flash device descriptor
 * @param request the IOCTL
 * @param argument the optional arguments to the IOCTL
 */
boolean fFlashIoctl(flash_options_pt device, uint32_t request, void *argument)
{
     switch (request)
     {
          case FLASH_MASS_ERASE_REQUEST :
               /* Device instance dictates what bank will be mass erased */
               fFlashMassErase(device);
               fFlashStallUntilNotBusy(device);
               break;
          case FLASH_PAGE_ERASE_REQUEST :
               /* get the page from argument and erase */
               fFlashPageErase(device, *(uint32_t*) argument);
               fFlashStallUntilNotBusy(device);
               break;
          case FLASH_POWER_DOWN :
               fFlashPowerDown(device);
               break;
          case FLASH_POWER_UP :
               fFlashPowerUp(device);
               fFlashStallUntilNotBusy(device);
               break;
          case FLASH_WAIT_UNTIL_DONE :
               fFlashStallUntilNotBusy(device);
               break;
          default:
               break;
     }
     return True;
}

/**
 *
 */
void fFlashHandler(FlashReg_pt membase)
{
     if (membase->INT_STATUS.BITS.INT_PEND)
     {
          switch (membase->INT_STATUS.BITS.INT_TYPE)
          {
               case IRQ_ACCESS_ERROR :
                    break;
               case IRQ_ERASE_COMPLETE :
                    break;
               case IRQ_PROGRAM_COMPLETE :
                    break;
               default:
                    break;
          }
     }
}
