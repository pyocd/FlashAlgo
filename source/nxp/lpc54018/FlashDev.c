/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2017 ARM Limited
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

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,               // Driver Version, do not modify!
    "LPC54018 16MB Serial Flash", // Device Name
    EXTSPI,                       // Device Type
    0x10000000,                   // Device Start Address
    0x01000000,                   // Device Size
    256,                          // Programming Page Size
    0,                            // Reserved, must be 0
    0xFF,                         // Initial Content of Erased Memory
    2,                            // Program Page Timeout 1.5ms + 30 %
    140,                          // Erase Sector Timeout 120ms + 15%
    4 * 1024, 0x000000,           // Sector Size  4KB
    SECTOR_END
};
