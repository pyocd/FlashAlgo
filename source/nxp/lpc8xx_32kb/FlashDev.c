/* -----------------------------------------------------------------------------
 * Copyright (c) 2004 - 2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        20. August 2014
 * $Revision:    V1.00
 *
 * Project:      Flash Device Description for NXP LPC8xx Flash using IAP
 * --------------------------------------------------------------------------- */

#include "FlashOS.h"        // FlashOS Structures

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "LPC8xx IAP 32kB Flash",    // Device Name
   ONCHIP,                     // Device Type
   0x00000000,                 // Device Start Address
   0x00008000,                 // Device Size (32kB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   300,                        // Program Page Timeout 300 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x000400, 0x000000,         // Sector Size  1kB (32 Sectors)
   SECTOR_END
};
