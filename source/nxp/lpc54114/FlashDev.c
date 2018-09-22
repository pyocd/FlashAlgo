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
#include "fsl_device_registers.h"

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,               // Driver Version, do not modify!
    "LPC54114 IAP 256kB Flash", // Device Name
    ONCHIP,                           // Device Type
    0x00000000,                 // Device Start Address
    FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES,             // Device Size
    FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES,    // Programming Page Size
    0,                          // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    300,                        // Program Page Timeout 300 mSec
    3000,                       // Erase Sector Timeout 3000 mSec
    {{0x008000, 0x000000},      // Sector Size  32kB
    {SECTOR_END}}
};
