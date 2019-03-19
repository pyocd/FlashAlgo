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
#include "mt25ql_flash_lib.h"

static const struct qspi_ip6514e_dev_cfg_t QSPI_DEV_CFG = {
    .base = MUSCA_QSPI_REG_BASE,
    /*
     * On Musca-A1, only the 18 first address bits are used for any AHB
     * address in a request coming to the QSPI Flash controller.
     * It means that direct accesses are limited to the first 256 KiB of the
     * Flash memory (if the Remap register is not used) and that the Indirect
     * Trigger zone needs to be inside the first 256 KiB as well.
     */
    .addr_mask = (1U << 18) - 1, /* 256 KiB minus 1 byte */
};

uint32_t initialized = 0;

//struct qspi_ip6514e_dev_t QSPI_DEV = {
//    &QSPI_DEV_CFG
//};
struct qspi_ip6514e_dev_t QSPI_DEV;

struct mt25ql_dev_t MT25QL_DEV = {
//    .controller = &QSPI_DEV,
    .direct_access_start_addr = MUSCA_QSPI_FLASH_BASE,
    .baud_rate_div = 4U,
    /*
     * 8 MiB flash memory are advertised in the Arm Musca-A Test Chip and Board
     * Technical Reference Manual. The MT25QL Flash device may however contain
     * more.
     */
    .size = 0x00800000U, /* 8 MiB */
};

/**
 * \brief Arm Flash device structure.
 */
struct arm_flash_dev_t {
    struct mt25ql_dev_t* dev;   /*!< FLASH memory device structure */
};

//static struct arm_flash_dev_t ARM_FLASH0_DEV = {
//    .dev    = &(MT25QL_DEV)
//};
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
        QSPI_DEV.cfg = &QSPI_DEV_CFG;
        MT25QL_DEV.controller = &QSPI_DEV;
        ARM_FLASH0_DEV.dev = &MT25QL_DEV;

        qspi_ip6514e_enable(ARM_FLASH0_DEV.dev->controller);

        /* Configure QSPI Flash controller to operate in single SPI mode and
         * to use fast Flash commands */
        if (MT25QL_ERR_NONE != mt25ql_config_mode(ARM_FLASH0_DEV.dev, MT25QL_FUNC_STATE_FAST)) {
              return 1;
        }
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
        /* Restores the QSPI Flash controller and MT25QL to default state */
        if (MT25QL_ERR_NONE != mt25ql_restore_default_state(ARM_FLASH0_DEV.dev)) {
            return 1;
        }
        initialized = 0;
    }
    return 0;
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {
    if (MT25QL_ERR_NONE != mt25ql_erase(ARM_FLASH0_DEV.dev, 0, MT25QL_ERASE_ALL_FLASH)) {
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
    adr = (adr & 0x00FFFFFF) - MUSCA_QSPI_FLASH_BASE;
    if (MT25QL_ERR_NONE != mt25ql_erase(ARM_FLASH0_DEV.dev, adr, MT25QL_ERASE_SECTOR_64K)) {
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
    adr = (adr & 0x00FFFFFF) - MUSCA_QSPI_FLASH_BASE;
    enum mt25ql_error_t err = mt25ql_command_write(ARM_FLASH0_DEV.dev, adr, buf, sz);
    if (MT25QL_ERR_NONE != err) {
        return err;
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
    unsigned char* ptr = (unsigned char*)(adr & 0x00FFFFFF);
    unsigned int i;
    unsigned char data[4];
    ptr = ptr - MUSCA_QSPI_FLASH_BASE;

    for (i = 0;  i < sz; i = i + 4)
    {
        mt25ql_command_read(ARM_FLASH0_DEV.dev, (uint32_t)ptr, (uint8_t*) data, 4);
        if( data[0] != buf[i + 0] ||
            data[1] != buf[i + 1] ||
            data[2] != buf[i + 2] ||
            data[3] != buf[i + 3] )
        {
            return (unsigned long)(MUSCA_QSPI_FLASH_BASE + ptr + i);
        }

        ptr = ptr + 4;
    }

    return 0;
}

/*  Blank Check Block in Flash Memory
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat)
{
    unsigned char* ptr = (unsigned char*)(adr & 0x00FFFFFF);
    unsigned int i;
    unsigned char data[4];
    ptr = ptr - MUSCA_QSPI_FLASH_BASE;

    for (i = 0;  i < sz; i = i + 4)
    {
        mt25ql_command_read(ARM_FLASH0_DEV.dev, (uint32_t)ptr, data, 4);
        if( data[0] != pat ||
            data[1] != pat ||
            data[2] != pat ||
            data[3] != pat )
        {
            return (1);
        }

        ptr = ptr + 4;
    }
    return (0);
}
