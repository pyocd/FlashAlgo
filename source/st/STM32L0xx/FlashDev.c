/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2015 ARM Ltd.
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
 * $Date:        14. September 2015
 * $Revision:    V1.01
 *  
 * Project:      Flash Device Description for ST STM32L0xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.01
 *    Added EEPROM algorithm
 *  Version 1.00
 *    Initial release
 */ 

#include "FlashOS.h"        // FlashOS Structures

#ifdef FLASH_MEMORY

#ifdef STM32L0xx_8
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 8KB Flash",        // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00002000,                 // Device Size in Bytes (8kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (64 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_16

#ifdef STM32L0xx_16
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 16KB Flash",       // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00004000,                 // Device Size in Bytes (16kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (128 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_16

#ifdef STM32L0xx_32
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 32KB Flash",       // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00008000,                 // Device Size in Bytes (32kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (256 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_32

#ifdef STM32L0xx_64
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 64KB Flash",       // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00010000,                 // Device Size in Bytes (64kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (512 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_64

#ifdef STM32L0xx_128
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 128KB Flash",      // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00020000,                 // Device Size in Bytes (128kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (1024 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_128

#ifdef STM32L0xx_192
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 192KB Flash ",     // Device Name
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00030000,                 // Device Size in Bytes (192kB)
   1024,                       // Programming Page Size (program several half pages at once)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x080, 0x000000,           // Sector Size 128B (1536 Sectors)
   SECTOR_END
};
#endif // STM32L0xx_192

#endif // FLASH_MEMORY


#ifdef FLASH_EEPROM

#ifdef category1  // STM32L011 and STM32L021 devices
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 512 Byte Data EEPROM",  // Device Name (512B)
   ONCHIP,                     // Device Type
   0x08080000,                 // Device Start Address
   0x00000200,                 // Device Size in Bytes (512kB)
   256,                        // Programming Page Size (256B)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x200, 0x000000,            // Sector Size 256B (16 Sectors)
   SECTOR_END
};
#endif // category1

#ifdef category2  // STM32L031 and STM32L041 devices
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 1KB Data EEPROM",  // Device Name (1KB)
   ONCHIP,                     // Device Type
   0x08080000,                 // Device Start Address
   0x00000400,                 // Device Size in Bytes (1kB)
   256,                        // Programming Page Size (256B)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x400, 0x000000,            // Sector Size 1KB
   SECTOR_END
};
#endif // category2

#ifdef category3  // STM32L051, STM32L052, STM32L062, STM32L053 and STM32L063 devices devices
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 2 KByte Data EEPROM",  // Device Name (2KB)
   ONCHIP,                     // Device Type
   0x08080000,                 // Device Start Address
   0x00000800,                 // Device Size in Bytes (2kB)
   256,                        // Programming Page Size (256B)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size 2KB
   SECTOR_END
};
#endif // category3
#ifdef category5  // STM32L071, STM32L081, STM32L072, STM32L082, STM32L073 and STM32L083 devices
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 2 KByte Data EEPROM",  // Device Name (2KB)
   ONCHIP,                     // Device Type
   0x08080000,                 // Device Start Address
   0x00001800,                 // Device Size in Bytes (2kB)
   256,                        // Programming Page Size (256B)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
     0xC00, 0x000000,            // Sector Size 3KB
	0xC00, 0x000C00,            // Sector Size 3KB
   SECTOR_END
};
#endif // category5
#ifdef category5_64  // STM32L071, STM32L081, STM32L072, STM32L082, STM32L073 and STM32L083 devices
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0 2 KByte Data EEPROM",  // Device Name (2KB)
   ONCHIP,                     // Device Type
   0x08080C00,                 // Device Start Address
   0x00000C00,                 // Device Size in Bytes (2kB)
   256,                        // Programming Page Size (256B)
   0,                          // Reserved, must be 0
   0x00,                       // Initial Content of Erased Memory
   500,                        // Program Page Timeout 500 mSec
   500,                        // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   0xC00, 0x000000,            // Sector Size 3KB
   SECTOR_END
};
#endif // category5_64
#endif // FLASH_EEPROM


#ifdef FLASH_OPTION

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32L0xx Flash Options",  // Device Name
   ONCHIP,                     // Device Type
   0x1FF80000,                 // Device Start Address
   0x000000014,                 // Device Size in Bytes (20)
   12,                         // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 3 Sec
   3000,                       // Erase Sector Timeout 3 Sec

// Specify Size and Address of Sectors
   0x0014, 0x000000,           // Sector Size 20B
   SECTOR_END
};

#endif // FLASH_OPTION 
