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

/** @file FlashDev.c */

#include "FlashOS.H"

#define FLASH_DRV_VERS (0x0100+VERS)   // Driver Version, do not modify!
#define DEVICE_NAME    "EFM32 Giant Gecko"

#if defined(EFM32GG_1024)
struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,             // Driver Version, do not modify!
    DEVICE_NAME,                // Device Name (128 chars max)
    ONCHIP,                     // Device Type
    0x00000000,                 // Device Start Address
    0x00100000,                 // Device Size
    (16*1024),                  // Programming Page Size
    0x00000000,                 // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    150,                        // Program Page Timeout 150 mSec
    300,                        // Erase Sector Timeout 300 mSec
    {{4096, 0x00000000},        // Sector Size {4kb, starting at address 0}
    {SECTOR_END}}
};
#elif defined(EFM32GG_512)
struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,             // Driver Version, do not modify!
    DEVICE_NAME,                // Device Name (128 chars max)
    ONCHIP,                     // Device Type
    0x00000000,                 // Device Start Address
    0x00080000,                 // Device Size
    (16*1024),                  // Programming Page Size
    0x00000000,                 // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    150,                        // Program Page Timeout 150 mSec
    300,                        // Erase Sector Timeout 300 mSec
    {{4096, 0x00000000},        // Sector Size {4kb, starting at address 0}
    {SECTOR_END}}
};
#elif defined(EFM32GG_256)
struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,             // Driver Version, do not modify!
    DEVICE_NAME,                // Device Name (128 chars max)
    ONCHIP,                     // Device Type
    0x00000000,                 // Device Start Address
    0x00040000,                 // Device Size
    (16*512),                   // Programming Page Size
    0x00000000,                 // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    150,                        // Program Page Timeout 150 mSec
    300,                        // Erase Sector Timeout 300 mSec
    {{2048, 0x00000000},        // Sector Size {2kb, starting at address 0}
    {SECTOR_END}}
};
#elif defined(EFM32GG_128)
struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,             // Driver Version, do not modify!
    DEVICE_NAME,                // Device Name (128 chars max)
    ONCHIP,                     // Device Type
    0x00000000,                 // Device Start Address
    0x00020000,                 // Device Size
    (16*512),                   // Programming Page Size
    0x00000000,                 // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    150,                        // Program Page Timeout 150 mSec
    300,                        // Erase Sector Timeout 300 mSec
    {{2048, 0x00000000},        // Sector Size {2kb, starting at address 0}
    {SECTOR_END}}
};
#elif defined(EFM32GG_64)
struct FlashDevice const FlashDevice = {
    FLASH_DRV_VERS,             // Driver Version, do not modify!
    DEVICE_NAME,                // Device Name (128 chars max)
    ONCHIP,                     // Device Type
    0x00000000,                 // Device Start Address
    0x00010000,                 // Device Size
    (16*512),                   // Programming Page Size
    0x00000000,                 // Reserved, must be 0
    0xFF,                       // Initial Content of Erased Memory
    150,                        // Program Page Timeout 150 mSec
    300,                        // Erase Sector Timeout 300 mSec
    {{2048, 0x00000000},        // Sector Size {2kb, starting at address 0}
    {SECTOR_END}}
};
#else
#error "Unknown EFM32 Giant Gecko flash size"
#endif
