/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
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

#include "FlashOS.H"        // FlashOS Structures
#include "fsl_flashiap.h"
#include "fsl_power.h"
#include "string.h"

#define CORE_CLK   12000000

#define MEMMAP   (*((volatile unsigned long *) 0x40000000))

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */

    SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK(1);

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    SYSCON->MAINCLKSELA = 0U;
    SYSCON->MAINCLKSELB = 0U;

    /*!< Set FLASH wait states for core */
    SYSCON->FLASHCFG &= ~(SYSCON_FLASHCFG_FLASHTIM_MASK);

    /*!< Set up dividers */
    SYSCON->AHBCLKDIV = 0;

    /* User Flash mode */
    MEMMAP = 0x02;

    return (0);
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t UnInit(uint32_t fnc)
{
    return (0);
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseChip(void)
{
    int status = FLASHIAP_PrepareSectorForWrite(0, 15);
    if (status == kStatus_Success)
    {
        status = FLASHIAP_EraseSector(0, 15, CORE_CLK);
    }
    return status;
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSector(uint32_t adr)
{
    uint32_t n;
    uint32_t status;

    n = adr / FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES;   // Get Sector Number

    status = FLASHIAP_PrepareSectorForWrite(n, n);
    if (status == kStatus_Success)
    {
        status = FLASHIAP_EraseSector(n, n, CORE_CLK);
    }
    return status;
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t n;
    uint32_t status;

    if (adr == 0) {                              // Check for Vector Table
        n = *((unsigned long *)(buf + 0)) +
            *((unsigned long *)(buf + 1)) +
            *((unsigned long *)(buf + 2)) +
            *((unsigned long *)(buf + 3)) +
            *((unsigned long *)(buf + 4)) +
            *((unsigned long *)(buf + 5)) +
            *((unsigned long *)(buf + 6));
        *((unsigned long *)(buf + 7)) = 0 - n;  // Signature at Reserved Vector
    }

    n = adr / FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES;  // Get Sector Number

    status = FLASHIAP_PrepareSectorForWrite(n, n);
    if (status == kStatus_Success)
    {
        status = FLASHIAP_CopyRamToFlash(adr, buf, FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES, CORE_CLK);
    }
    return status;
}
