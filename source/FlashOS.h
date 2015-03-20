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

#ifndef FLASHOS_H
#define FLASHOS_H

#include "stdint.h"

#ifdef __cplusplus
  extern "C" {
#endif

/**
    @ingroup FlashOS
    @{
*/

#define VERS       1      // Interface Version 1.01
#define NAME_MAX   128    // Max size of the routine name
#define PAGE_MAX   65536  // Max Page Size for Programming
#define SECTOR_NUM 512    // Max Number of Sector Items
#define SECTOR_END 0xFFFFFFFF, 0xFFFFFFFF

/**
    @enum FlashLoc
    @brief  An enumerated type of possible program memory locations and interfaces
 */
typedef enum {
    UNKNOWN = 0,   /*!< Unknown location */
    ONCHIP,        /*!< On-chip Flash Memory */
    EXT8BIT,       /*!< External Flash Device on 8-bit  Bus */
    EXT16BIT,      /*!< External Flash Device on 16-bit Bus */
    EXT32BIT,      /*!< External Flash Device on 32-bit Bus */
    EXTSPI         /*!< External Flash Device on SPI */
} FlashLoc;

/**
    @struct FlashSector
    @brief  A structure to describe the size and start address of a flash sector
 */
struct FlashSector {
    uint32_t szSector;      /*!< Sector Size in Bytes */
    uint32_t adrSector;     /*!< Address of Sector */
};

/**
    @struct FlashDevice
    @brief  A structure to describe particulars of a flash memory sub-system
        and requirements of the driver
 */
struct FlashDevice {
    uint16_t vers;          /*!< Version Number and Architecture */
    char devName[NAME_MAX]; /*!< Device Name and Description */
    enum FlashLoc devType;  /*!< Device Type: ONCHIP, EXT8BIT, EXT16BIT, ... */
    uint32_t devAdr;        /*!< Default Device Start Address */
    uint32_t szDev;         /*!< Total Size of Device */
    uint32_t szPage;        /*!< Programming Page Size */
    uint32_t res;           /*!< Reserved for future Extension */
    uint8_t  valEmpty;      /*!< Content of Erased Memory */
    uint32_t toProg;        /*!< Time Out of Program Page Function */
    uint32_t toErase;       /*!< Time Out of Erase Sector Function */
    struct FlashSector sectors[SECTOR_NUM]; /*!< Entries to describe flash memory layout */
};

#ifdef __cplusplus
  }
#endif

#endif
