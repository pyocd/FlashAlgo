/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2019 Arm Limited
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

#include "FlashOS.h"        // FlashOS Structures
#include "gfc100_eflash_drv.h"

#define  SYSTEM_CLOCK    (40960000UL)

uint32_t initialized = 0;

struct gfc100_eflash_dev_cfg_t GFC100_DEV_CFG = {
    .base = MUSCA_B_EFLASH_REG_BASE,
};

struct gfc100_eflash_dev_data_t GFC100_DEV_DATA = {
    .is_initialized = false,
    .flash_size = 0x400000,
};

struct gfc100_eflash_dev_t GFC100_DEV;

/**
 * \brief Arm Flash device structure.
 */
struct arm_flash_dev_t {
    struct gfc100_eflash_dev_t* dev;   /*!< FLASH memory device structure */
};

static struct arm_flash_dev_t ARM_FLASH0_DEV;

/*
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
    if(initialized == 0) {
        /* For PIC the following assignments have to be in a function (run time) */
        GFC100_DEV.data = &GFC100_DEV_DATA;
        GFC100_DEV.cfg = &GFC100_DEV_CFG;
        ARM_FLASH0_DEV.dev = &GFC100_DEV;
        gfc100_eflash_init(ARM_FLASH0_DEV.dev, SYSTEM_CLOCK);
        initialized = 1;
    }
    return 0;
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {
    if(fnc == 0 && initialized == 1) {
        initialized = 0;
    }
    return 0;
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {
    if (GFC100_ERROR_NONE != gfc100_eflash_erase(ARM_FLASH0_DEV.dev, 0, GFC100_MASS_ERASE_ALL)) {
        return 1;
    }
    return 0;
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {
    /*
     * GFC100 suports up to 4MB of flash size.
     */
    adr = (adr - MUSCA_B_EFLASH_BASE) & (GFC100_DEV_DATA.flash_size - 1);
    if (GFC100_ERROR_NONE != gfc100_eflash_erase(ARM_FLASH0_DEV.dev, adr, GFC100_ERASE_PAGE)) {
        return 1;
    }
    return 0;
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
    unsigned int i;
    /*
     * GFC100 suports up to 4MB of flash size.
     */
    adr = (adr - MUSCA_B_EFLASH_BASE) & (GFC100_DEV_DATA.flash_size - 1);
    uint32_t len = GFC100_WRITE_BYTE_SIZE;

    /* Write is done in 4byte words, calculate how many writes we need */
    uint32_t max_write = sz / GFC100_WRITE_BYTE_SIZE;

    for (i=0; i<max_write; i++) {
        if (GFC100_ERROR_NONE != gfc100_eflash_write(ARM_FLASH0_DEV.dev, adr, buf, &len)) {
            return 1;
        }
        buf += GFC100_WRITE_BYTE_SIZE;
        adr += GFC100_WRITE_BYTE_SIZE;
    }
    return 0;
}

 /*
  *  Verify Flash Contents
  *    Parameter:      adr:  Start Address
  *                    sz:   Size (in bytes)
  *                    buf:  Data
  *    Return Value:   0 - OK, Failed Address
  */
unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf) {
    /*
     * GFC100 suports up to 4MB of flash size.
     */
    adr = (adr - MUSCA_B_EFLASH_BASE) & (GFC100_DEV_DATA.flash_size - 1);
    uint32_t len = GFC100_WRITE_BYTE_SIZE;
    unsigned int i, j;
    unsigned char data[GFC100_WRITE_BYTE_SIZE];
    for (i = 0;  i < sz; i = i + GFC100_WRITE_BYTE_SIZE)
    {
        gfc100_eflash_read(ARM_FLASH0_DEV.dev, adr, (uint8_t*) data, &len);
        for (j = 0; j < GFC100_WRITE_BYTE_SIZE; j++) {
            if( data[j] != buf[i + j] )
            {
                return (unsigned long)(MUSCA_B_EFLASH_BASE + adr + i);
            }
		}
        adr = adr + GFC100_WRITE_BYTE_SIZE;
    }
    return 0;
}

/*  Blank Check Block in Flash Memory
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat) {
    /*
     * GFC100 suports up to 4MB of flash size.
     */
    adr = (adr - MUSCA_B_EFLASH_BASE) & (GFC100_DEV_DATA.flash_size - 1);
    uint32_t len = GFC100_WRITE_BYTE_SIZE;
    unsigned int i, j;
    unsigned char data[GFC100_WRITE_BYTE_SIZE];
    for (i = 0;  i < sz; i = i + GFC100_WRITE_BYTE_SIZE)
    {
        gfc100_eflash_read(ARM_FLASH0_DEV.dev, adr, (uint8_t*) data, &len);
        for (j = 0; j < GFC100_WRITE_BYTE_SIZE; j++) {
            if( data[j] != pat )
            {
                return 1;
            }
		}
        adr = adr + GFC100_WRITE_BYTE_SIZE;
    }
    return 0;
}
