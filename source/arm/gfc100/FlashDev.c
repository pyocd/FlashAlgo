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

#include "FlashOS.H"        // FlashOS Structures

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MuscaB_eflash",            // Device Name
   EXTSPI,                     // Device Type
   MUSCA_B_EFLASH_BASE,        // Device Start Address
   0x00400000,                 // Device Size (4MB)
   0x04000,                    // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x04000, 0x000000,         // Sector Size  16kB
   SECTOR_END
};


