/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 ARM Ltd.
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
 * $Date:        15. April 2014
 * $Revision:    V1.00
 *  
 * Project:      Flash Programming Functions for ST Microelectronics STM32L15x Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "..\FlashOS.h"        // FlashOS Structures

typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define FLASH_BASE      0x40023C00

#define FLASH           ((FLASH_TypeDef*) FLASH_BASE)

// Flash Registers
typedef struct {
  vu32 ACR;                                     // offset  0x000 Flash access control register (FLASH_ACR)
  vu32 PECR;                                    // offset  0x004 Flash program erase control register (FLASH_PECR)
  vu32 PDKEYR;                                  // offset  0x008 Flash power down key register (FLASH_PDKEYR)
  vu32 PEKEYR;                                  // offset  0x00C Flash program erase key register (FLASH_PEKEYR)
  vu32 PRGKEYR;                                 // offset  0x010 Flash program memory key register (FLASH_PRGKEYR)
  vu32 OPTKEYR;                                 // offset  0x014 Flash option key register (FLASH_OPTKEYR)
  vu32 SR;                                      // offset  0x018 Flash status register (FLASH_SR)
  vu32 OBR;                                     // offset  0x01C Option byte register (FLASH_OBR)
  vu32 WRPR1;                                   // offset  0x020 Flash write protection register 1 (FLASH_WRPR1)
  vu32 RFU[23];                                 // 0x24 - 0x80
  vu32 WRPR2;                                   // offset  0x080 Flash write protection register 2 (FLASH_WRPR2)
} FLASH_TypeDef;


// Flash Keys
#define FLASH_PEKEY1        0x89ABCDEF
#define FLASH_PEKEY2        0x02030405
#define FLASH_PRGKEY1       0x8C9DAEBF
#define FLASH_PRGKEY2       0x13141516
#define FLASH_OPTKEY1       0xFBEAD9C8
#define FLASH_OPTKEY2       0x24252627

// Flash program erase control register (FLASH_PECR) definitions
#define FLASH_PELOCK        0x00000001			// FLASH_PECR and data memory lock 
#define FLASH_PRGLOCK       0x00000002			// Program memory lock
#define FLASH_OPTLOCK       0x00000004			// Option bytes block lock
#define FLASH_PROG          0x00000008			// Program memory selection
#define FLASH_DATA          0x00000010			// Data memory selection
#define FLASH_OPT           0x00000020			// Option Bytes memory selection
#define FLASH_FIX           0x00000100			// Fixed time data write for Byte, Half Word and Word programming
#define FLASH_ERASE         0x00000200			// Page or Double Word erase mode
#define FLASH_FPRG          0x00000400			// Half Page/Double Word programming mode
#define FLASH_GBHF_ER       0x00000800			// Global Half Erase mode

// Flash status register (FLASH_SR) definitions
#define FLASH_BSY           0x00000001			// Write/erase operations in progress  
#define FLASH_EOP           0x00000002			// End of operation
#define FLASH_ENDHV         0x00000004			// End of high voltage
#define FLASH_WRPERR        0x00000100			// Write protected error
#define FLASH_PGAERR        0x00000200			// Programming alignment error
#define FLASH_SIZERR        0x00000400			// Size error
#define FLASH_OPTVERR       0x00000800			// Option validity error

#define FLASH_ERRs         (FLASH_PGAERR | FLASH_WRPERR | FLASH_SIZERR | FLASH_OPTVERR)

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t Init (uint32_t adr, uint32_t clk, uint32_t fnc) {

  switch (fnc) {
    case 1:
    case 2:
      FLASH->SR |= FLASH_ERRs;                  // clear error flags
    break;
  }

  return (0);
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t UnInit (uint32_t fnc) {

  switch (fnc) {
    case 1:
    case 2:
      // Lock PECR register and program matrix
      FLASH->PECR |= FLASH_PRGLOCK;             // Program memory lock
      FLASH->PECR |= FLASH_PELOCK;              // FLASH_PECR and data memory lock
    break;
  }
  
  return (0);
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t EraseSector (uint32_t adr) {
  // Unlock PECR Register    
  if (FLASH->PECR & FLASH_PELOCK) {
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }

  // Unlock Program Matrix    
  if (FLASH->PECR & FLASH_PRGLOCK) {
    FLASH->PRGKEYR = FLASH_PRGKEY1;
    FLASH->PRGKEYR = FLASH_PRGKEY2;  
  }

  FLASH->PECR |= FLASH_ERASE;                   // Page or Double Word Erase enabled
  FLASH->PECR |= FLASH_PROG;                    // Program memory selected
     
  M32(adr) = 0x00000000;			            // write '0' to the first address to erase page

  while (FLASH->SR & FLASH_BSY);

  // Check for Errors
  if (FLASH->SR & (FLASH_ERRs)) {
    uint32_t ret = FLASH->SR;
    FLASH->SR |= FLASH_ERRs;                    // clear error flags
    return (ret);                               // Failed
  }

  FLASH->PECR &= ~FLASH_ERASE;                  // Page or Double Word Erase disabled
  FLASH->PECR &= ~FLASH_PROG;                   // Program memory deselected   

  return (0);                                   // Done
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t EraseChip (void) {
  uint32_t ret;
  for (uint32_t i = 0x08000000; i < 0x08040000; i += 0x100) {
    ret = EraseSector(i);
    if (ret)
      return (ret);
  }

  return (0);
}

/*  
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t BlankCheck (uint32_t adr, uint32_t sz, uint32_t pat) {
  return (1);                                   // Always Force Erase
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

uint32_t ProgramPage (uint32_t adr, uint32_t sz, uint32_t *buf) {
  uint32_t addr = adr;
  int i, j;

  // First half page programming cycle

  // Unlock PECR Register    
  if (FLASH->PECR & FLASH_PELOCK) {
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }

  // Unlock Program Matrix    
  if (FLASH->PECR & FLASH_PRGLOCK) {
    FLASH->PRGKEYR = FLASH_PRGKEY1;
    FLASH->PRGKEYR = FLASH_PRGKEY2;  
  }

  if ((FLASH->PECR & FLASH_PELOCK) || (FLASH->PECR & FLASH_PRGLOCK)) {
    return (1);
  }

  FLASH->PECR |= FLASH_FPRG;			// Half Page programming mode enabled
  FLASH->PECR |= FLASH_PROG;                    // Program memory selected

  while (FLASH->SR & FLASH_BSY);

  // write first half page
  for (i = 0, j = 0; i < 128; i += 4, j++) {
    *(uint32_t *)(addr + i) = *(buf + j);
  }

  // Check for Errors
  if (FLASH->SR & (FLASH_ERRs)) {
    uint32_t ret = FLASH->SR;
    FLASH->SR |= FLASH_ERRs;                    // clear error flags
    return (ret);                               // Failed
  }

  // Second half page programming cycle

  // Unlock PECR Register    
  if (FLASH->PECR & FLASH_PELOCK) {
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }

  // Unlock Program Matrix    
  if (FLASH->PECR & FLASH_PRGLOCK) {
    FLASH->PRGKEYR = FLASH_PRGKEY1;
    FLASH->PRGKEYR = FLASH_PRGKEY2;  
  }

  if ((FLASH->PECR & FLASH_PELOCK) || (FLASH->PECR & FLASH_PRGLOCK)) {
    return (1);
  }

  FLASH->PECR |= FLASH_FPRG;			// Half Page programming mode enabled
  FLASH->PECR |= FLASH_PROG;                    // Program memory selected

  while (FLASH->SR & FLASH_BSY);

  // write second half page
  for (i = 128, j = 32; i < 256; i += 4, j++) {
    *(uint32_t *)(addr + i) = *(buf + j);
  }

  // Check for Errors
  if (FLASH->SR & (FLASH_ERRs)) {
    uint32_t ret = FLASH->SR;
    FLASH->SR |= FLASH_ERRs;                    // clear error flags
    return (ret);                               // Failed
  }

  return (0);
}

