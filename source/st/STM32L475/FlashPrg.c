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
 * Project:      Flash Programming Functions for STMicroelectronics STM32F4xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "FlashOS.h"        // FlashOS Structures

typedef volatile unsigned char    vu8;
typedef          unsigned char     u8;
typedef volatile unsigned short   vu16;
typedef          unsigned short    u16;
typedef volatile unsigned long    vu32;
typedef          unsigned long     u32;

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define IWDG_BASE         0x40003000
#define FLASH_BASE        0x40022000

#define IWDG            ((IWDG_TypeDef *) IWDG_BASE)
#define FLASH           ((FLASH_TypeDef*) FLASH_BASE)

// Independent WATCHDOG
typedef struct {
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

// Flash Registers
typedef struct {
  vu32 ACR;
  vu32 PDKEY;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 ECCR;
  vu32 OPTCR;
  vu32 OPTCR1;
} FLASH_TypeDef;


// Flash Keys
//#define RDPRT_KEY       0x00A5
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB
#define FLASH_OPTKEY1   0x08192A3B
#define FLASH_OPTKEY2   0x4C5D6E7F

// Flash Control Register definitions
#define FLASH_PG                ((unsigned int)0x00000001)
#define FLASH_SER               ((unsigned int)0x00000002)
#define FLASH_MER1               ((unsigned int)0x00000004)
#define FLASH_MER2              ((unsigned int)0x00008000)
#define FLASH_PNB_POS           ((unsigned int)0x00000003)
#define FLASH_PNB_MSK           ((unsigned int)0x000007F8)
#define FLASH_STRT              ((unsigned int)0x00010000)
#define FLASH_LOCK              ((unsigned int)0x80000000)

// Flash Option Control Register definitions
// #define FLASH_OPTLOCK           ((unsigned int)0x00000001)
// #define FLASH_OPTSTRT           ((unsigned int)0x00000002)


#define FLASH_PSIZE_Byte        ((unsigned int)0x00000000)
#define FLASH_PSIZE_HalfWord    ((unsigned int)0x00000100)
#define FLASH_PSIZE_Word        ((unsigned int)0x00000200)
#define FLASH_PSIZE_DoubleWord  ((unsigned int)0x00000300)


// Flash Status Register definitions
#define FLASH_EOP               ((unsigned int)0x00000001)
#define FLASH_OPERR             ((unsigned int)0x00000002)
#define FLASH_WRPERR            ((unsigned int)0x00000010)
#define FLASH_PGAERR            ((unsigned int)0x00000020)
#define FLASH_PROGERR            ((unsigned int)0x00000008)
#define FLASH_PGSERR            ((unsigned int)0x00000080)
#define FLASH_BSY               ((unsigned int)0x00010000)

#define FLASH_PGERR             (FLASH_PGSERR | FLASH_PROGERR  | FLASH_PGAERR | FLASH_WRPERR)


// Flash Access Control Register definitions
#define FLASH_DCRST             ((unsigned int)0x00001000)
#define FLASH_ICRST             ((unsigned int)0x00000800)
#define FLASH_DCEN              ((unsigned int)0x00000400)
#define FLASH_ICEN              ((unsigned int)0x00000200)
#define FLASH_PRFTEN            ((unsigned int)0x00000100)

#define FLASH_CACHE_MASK        (FLASH_DCRST | FLASH_ICRST | FLASH_DCEN | FLASH_ICEN | FLASH_PRFTEN)

/*
 * Get Sector Number
 *    Parameter:      adr:  Sector Address
 *    Return Value:   Sector Number
 */

unsigned long GetSecNum (unsigned long adr) {
  unsigned long n;
  
  n = adr - 0x08000000 ;    // Get offset from base address
  return n >> 11;           // The devide the offset by the sector size (2048 = 0x800 = shift 11 bits) 
}


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  FLASH->SR  |= FLASH_PGERR;                            // Reset Error Flags

  if ((FLASH->OPTCR & 0x20) == 0x00) {                  // Test if IWDG is running (IWDG in HW mode)
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR     
    IWDG->PR  = 0x06;                                   // Set prescaler to 256  
    IWDG->RLR = 4095;                                   // Set reload value to 4095 (max value)
  }

  return (0);
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int UnInit (unsigned long fnc) {
  u32 acr;

  acr = FLASH->ACR;                                     // Clear flash caches
  FLASH->ACR = acr & ~FLASH_CACHE_MASK;
  FLASH->ACR = acr;

  FLASH->CR |=  FLASH_LOCK;                             // Lock Flash

  return (0);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseChip (void) {

  FLASH->CR |=  FLASH_MER1;                              // Mass Erase Enabled (sectors  0..11)
  FLASH->CR |=  FLASH_STRT;                             // Start Erase

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_MER1;                              // Mass Erase Disabled

  return (0);                                           // Done
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseSector (unsigned long adr) {
  unsigned long n;

  n = GetSecNum(adr);                                   // Get Sector Number

  FLASH->SR |= FLASH_PGERR;                             // Reset Error Flags

  FLASH->CR  =  FLASH_SER;                              // Sector Erase Enabled 
  FLASH->CR |=  ((n << FLASH_PNB_POS) & FLASH_PNB_MSK); // Sector Number
  FLASH->CR |=  FLASH_STRT;                             // Start Erase

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_SER;                              // Page Erase Disabled 

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR |= FLASH_PGERR;                           // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}



/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = (sz + 3) & ~3;                                   // Adjust size for Words
  
  FLASH->SR |= FLASH_PGERR;                             // Reset Error Flags
  FLASH->CR  =  0;                                      // reset CR 

  while (sz) {
    FLASH->CR |= (FLASH_PG              |               // Programming Enabled
                  FLASH_PSIZE_Word);                    // Programming Enabled (Word)

    M32(adr) = *((u32 *)buf);                           // Program Double Word
    while (FLASH->SR & FLASH_BSY);

    FLASH->CR &= ~FLASH_PG;                             // Programming Disabled

    if (FLASH->SR & FLASH_PGERR) {                      // Check for Error
      FLASH->SR |= FLASH_PGERR;                         // Reset Error Flags
      return (1);                                       // Failed
    }

    adr += 4;                                           // Go to next Word
    buf += 4;
    sz  -= 4;
  }

  return (0);                                           // Done
}
