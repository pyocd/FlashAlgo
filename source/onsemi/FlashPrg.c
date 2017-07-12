/* Flash OS Routines
 * Copyright (c) 2009-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file FlashPrg.c */


#include "system_ARMCM3.h"
#include "core_cm3.h"
#include "types.h"
#include "flash.h"
#include "clock.h"
#include "FlashOS.h"
#include "FlashPrg.h"

#define RESULT_OK                  0
#define RESULT_ERROR               1
#define DEVICE_OPT_REG_ADRS        (uint32_t)0x4001E000
#define DEVICE_OPT_ALL_FEATURE_EN  (uint32_t)0x2082353F

void fInitGobjects(void);
void fInitRam(void);

/** Global flash A device options */
const flash_options_t GlobFlashOptionsA =
{
    0x00000000,
    FLASHREG,
    Flash_IRQn
};
/** Global flash B device options */
const flash_options_t GlobFlashOptionsB =
{
    0x00100000,
    FLASHREG,
    Flash_IRQn
};

uint8_t numDev;

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    // Called to configure the SoC. Should enable clocks
    //  watchdogs, peripherals and anything else needed to
    //  access or program memory. Fnc parameter has meaning
    //  but currently isnt used in MSC programming routines

    CLOCK_ENABLE(CLOCK_FLASH);
    CLOCK_ENABLE(CLOCK_DMA);

    volatile uint32_t *devreg = (uint32_t *)DEVICE_OPT_REG_ADRS;
    *devreg = DEVICE_OPT_ALL_FEATURE_EN;

    /* Disable all interrupts */
    NVIC->ICER[0] = 0x1F;
    /* Clear all Pending interrupts */
    NVIC->ICPR[0] = 0x1F;
    /* Clear all pending SV and systick */
    SCB->ICSR = (uint32_t)0x0A000000;

#if 0
    /* Relocate the exception vector table from default 0x0 to the location in RAM
     * from this download */
    __DSB();
    /* MSbit = 0 vector table in code region, 1 vector table in ram region */
    SCB->VTOR = (uint32_t)&__Vectors;
    __DSB();
#endif

    fFlashIoctl((flash_options_pt)&GlobFlashOptionsB, FLASH_POWER_UP, 0);
    return RESULT_OK;
}

uint32_t UnInit(uint32_t fnc)
{
    // When a session is complete this is called to powerdown
    //  communication channels and clocks that were enabled
    //  Fnc parameter has meaning but isnt used in MSC program
    //  routines

    /* Optional API */

    return RESULT_OK;
}

uint32_t BlankCheck(uint32_t adr, uint32_t sz, uint8_t pat)
{
    /* Optional API */
    return RESULT_OK;
}

uint32_t EraseChip(void)
{
    /* Erases the entire of flash memory region both flash A & B */

    fFlashMassErase((flash_options_pt)&GlobFlashOptionsA);
    /*Shall we leave flash b alone and let in-application-programming handle its contents?*/
    fFlashMassErase((flash_options_pt)&GlobFlashOptionsB);

    return RESULT_OK;
}

uint32_t EraseSector(uint32_t adr)
{
    if(adr >= FLASH_A_USER_AREA_OFFSET)
    {
        if((adr >= 0x2000) && (adr < 0x52000))
        {
            fFlashIoctl((flash_options_pt)&GlobFlashOptionsA, FLASH_PAGE_ERASE_REQUEST, &adr);
        }
        else if ((adr >= 0x00102000) && (adr < 0x00152000))
        {
            fFlashIoctl((flash_options_pt)&GlobFlashOptionsB, FLASH_PAGE_ERASE_REQUEST, &adr);
        }
        return RESULT_OK;
    }
    return RESULT_ERROR;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    boolean retVal = True;

    if(adr >= FLASH_A_USER_AREA_OFFSET)
    {
        /* Write to flash A or Flash B depending on the flash bank in use */
        if((adr >= 0x2000) && (adr < 0x52000))
        {
            retVal = fFlashWrite((flash_options_pt)&GlobFlashOptionsA,(uint8_t **)&adr,
                                               (uint8_t const *)buf,sz);
        }
        else if ((adr >= 0x00102000) && (adr < 0x00152000))
        {
            retVal = fFlashWrite((flash_options_pt)&GlobFlashOptionsB,(uint8_t **)&adr,
                                               (uint8_t const *)buf,sz);
        }

        if(retVal == True)
        {
          return RESULT_OK;
        }
    }
    return RESULT_ERROR;
}

uint32_t Verify(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    /* Optional API */
    return RESULT_OK;
}
