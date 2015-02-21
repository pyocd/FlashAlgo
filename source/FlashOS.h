/* CMSIS-DAP Interface Firmware
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

#define VERS       1      // Interface Version 1.01
#define UNKNOWN    0      // Unknown
#define ONCHIP     1      // On-chip Flash Memory
#define EXT8BIT    2      // External Flash Device on 8-bit  Bus
#define EXT16BIT   3      // External Flash Device on 16-bit Bus
#define EXT32BIT   4      // External Flash Device on 32-bit Bus
#define EXTSPI     5      // External Flash Device on SPI
#define SECTOR_NUM 512    // Max Number of Sector Items
#define PAGE_MAX   65536  // Max Page Size for Programming
#define SECTOR_END 0xFFFFFFFF, 0xFFFFFFFF
#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashSectors {
  uint32_t szSector;      // Sector Size in Bytes
  uint32_t adrSector;     // Address of Sector
};

struct FlashDevice {
   uint16_t vers;         // Version Number and Architecture
   char     devName[128]; // Device Name and Description
   uint16_t devType;      // Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
   uint32_t devAdr;       // Default Device Start Address
   uint32_t szDev;        // Total Size of Device
   uint32_t szPage;       // Programming Page Size
   uint32_t res;          // Reserved for future Extension
   uint8_t  valEmpty;     // Content of Erased Memory
   uint32_t toProg;       // Time Out of Program Page Function
   uint32_t toErase;      // Time Out of Erase Sector Function
   struct FlashSectors sectors[SECTOR_NUM];
};

// Flash Programming Functions (Called by FlashOS)
extern uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc);
extern uint32_t UnInit(uint32_t fnc);
extern uint32_t BlankCheck(uint32_t adr, uint32_t sz, uint8_t pat);
extern uint32_t EraseChip(void);
extern uint32_t EraseSector(uint32_t adr);
extern uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf);
extern uint32_t Verify(uint32_t adr, uint32_t sz, uint32_t *buf);

#ifdef __cplusplus
  }
#endif

#endif
