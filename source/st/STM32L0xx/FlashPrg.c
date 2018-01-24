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
 * Project:      Flash Programming Functions for ST STM32L0xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.01
 *    Added EEPROM algorithm
 *  Version 1.00
 *    Initial release
 */ 

#include "..\FlashOS.H"        // FlashOS Structures

typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define IWDG_BASE       0x40003000
#define FLASH_BASE      0x40022000

#define IWDG            ((IWDG_TypeDef *) IWDG_BASE)
#define FLASH           ((FLASH_TypeDef*) FLASH_BASE)

// Independent WATCHDOG
typedef struct {
  vu32 KR;                                      // offset  0x000 Key register (IWDG_KR)
  vu32 PR;                                      // offset  0x004 Prescaler register (IWDG_PR)
  vu32 RLR;                                     // offset  0x008 Reload register (IWDG_RLR)
  vu32 SR;                                      // offset  0x00C Status register (IWDG_SR)
} IWDG_TypeDef;

// Flash Registers
typedef struct {
  vu32 ACR;                                     // offset  0x000 Access control register (FLASH_ACR)
  vu32 PECR;                                    // offset  0x004 Program and erase control register (FLASH_PECR)
  vu32 PDKEYR;                                  // offset  0x008 Power-down key register (FLASH_PDKEYR)
  vu32 PEKEYR;                                  // offset  0x00C PECR unlock key register (FLASH_PEKEYR)
  vu32 PRGKEYR;                                 // offset  0x010 Program and erase key register (FLASH_PRGKEYR)
  vu32 OPTKEYR;                                 // offset  0x014 Option bytes unlock key register (FLASH_OPTKEYR)
  vu32 SR;                                      // offset  0x018 Status register (FLASH_SR)
  vu32 OPTR;                                    // offset  0x01C Option bytes register (FLASH_OPTR)
  vu32 WRPROT;                                  // offset  0x020 Write protection register (FLASH_WRPROT)
} FLASH_TypeDef;


// Flash Keys
#define FLASH_PEKEY1           (0x89ABCDEFu)
#define FLASH_PEKEY2           (0x02030405u)
#define FLASH_PRGKEY1          (0x8C9DAEBFu)
#define FLASH_PRGKEY2          (0x13141516u)
#define FLASH_OPTKEY1          (0xFBEAD9C8u)
#define FLASH_OPTKEY2          (0x24252627u)

// Flash program erase control register (FLASH_PECR) definitions
#define FLASH_PELOCK           (0x00000001u)			// FLASH_PECR and data memory lock 
#define FLASH_PRGLOCK          (0x00000002u)			// Program memory lock
#define FLASH_OPTLOCK          (0x00000004u)			// Option bytes block lock
#define FLASH_PROG             (0x00000008u)			// Program memory selection
#define FLASH_DATA             (0x00000010u)			// Data memory selection
#define FLASH_OPT              (0x00000020u)			// Option Bytes memory selection
#define FLASH_FIX              (0x00000100u)			// Fixed time data write for Byte, Half Word and Word programming
#define FLASH_ERASE            (0x00000200u)			// Page or Double Word erase mode
#define FLASH_FPRG             (0x00000400u)			// Half Page/Double Word programming mode
#define FLASH_GBHF_ER          (0x00000800u)			// Global Half Erase mode

// Flash status register (FLASH_SR) definitions
#define FLASH_BSY              (0x00000001u)     // Write/erase operations in progress  
#define FLASH_EOP              (0x00000002u)			// End of operation
#define FLASH_ENDHV            (0x00000004u)			// End of high voltage
#define FLASH_WRPERR           (0x00000100u)			// Write protected error
#define FLASH_PGAERR           (0x00000200u)			// Programming alignment error
#define FLASH_SIZERR           (0x00000400u)			// Size error
#define FLASH_OPTVERR          (0x00000800u)			// Option validity error

#define FLASH_ERRs         (FLASH_PGAERR | FLASH_WRPERR | FLASH_SIZERR | FLASH_OPTVERR)

// Option byte register (FLASH_OBR) definitions
#define FLASH_IWDG_SW          (0x00100000u)            // Software IWDG or Hardware IWDG selected

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEMORY
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  switch (fnc) {
    case 1:
    case 2:
      FLASH->SR |= FLASH_ERRs;                  // clear error flags

      // Unlock PECR Register    
      FLASH->PEKEYR = FLASH_PEKEY1;
      FLASH->PEKEYR = FLASH_PEKEY2;

      // Unlock Program Matrix    
      FLASH->PRGKEYR = FLASH_PRGKEY1;
      FLASH->PRGKEYR = FLASH_PRGKEY2;  

      // Test if IWDG is running (IWDG in HW mode)
      if ((FLASH->OPTR & FLASH_IWDG_SW) == 0x00) {
        // Set IWDG time out to ~32.768 second
        IWDG->KR  = 0x5555;                     // Enable write access to IWDG_PR and IWDG_RLR     
        IWDG->PR  = 0x06;                       // Set prescaler to 256  
        IWDG->RLR = 0xFFF;                      // Set reload value to 4095
      }
    break;
  }

  return (0);
}
#endif  // FLASH_MEMORY

#ifdef FLASH_OPTION
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  switch (fnc) {
    case 1:
    case 2:
      FLASH->SR |= FLASH_ERRs;                  // clear error flags

      // Unlock PECR Register    
      FLASH->PEKEYR = FLASH_PEKEY1;
      FLASH->PEKEYR = FLASH_PEKEY2;

      // Unlock Option Byte Block    
      FLASH->OPTKEYR = FLASH_OPTKEY1;
      FLASH->OPTKEYR = FLASH_OPTKEY2;  

      // Test if IWDG is running (IWDG in HW mode)
      if ((FLASH->OPTR & FLASH_IWDG_SW) == 0x00) {
        // Set IWDG time out to ~32.768 second
        IWDG->KR  = 0x5555;                     // Enable write access to IWDG_PR and IWDG_RLR     
        IWDG->PR  = 0x06;                       // Set prescaler to 256  
        IWDG->RLR = 0xFFF;                      // Set reload value to 4095
      }
    break;
  }

  return (0);
}
#endif  // FLASH_OPTION

#ifdef FLASH_EEPROM
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  switch (fnc) {
    case 1:
    case 2:
      FLASH->SR |= FLASH_ERRs;                  // clear error flags

      // Unlock PECR Register    
      FLASH->PEKEYR = FLASH_PEKEY1;
      FLASH->PEKEYR = FLASH_PEKEY2;

      // Test if IWDG is running (IWDG in HW mode)
      if ((FLASH->OPTR & FLASH_IWDG_SW) == 0x00) {
        // Set IWDG time out to ~32.768 second
        IWDG->KR  = 0x5555;                     // Enable write access to IWDG_PR and IWDG_RLR     
        IWDG->PR  = 0x06;                       // Set prescaler to 256  
        IWDG->RLR = 0xFFF;                      // Set reload value to 4095
      }
    break;
  }

  return (0);
}
#endif //FLASH_EEPROM


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEMORY
int UnInit (unsigned long fnc) {

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
#endif  // FLASH_MEMORY

#ifdef FLASH_OPTION
int UnInit (unsigned long fnc) {

  switch (fnc) {
    case 1:
    case 2:
      // Lock PECR register and Option bytes
      FLASH->PECR |= FLASH_OPTLOCK;             // Option bytes block lock
      FLASH->PECR |= FLASH_PELOCK;              // FLASH_PECR and data memory lock
    break;
  }

  return (0);
}
#endif  // FLASH_OPTION

#ifdef FLASH_EEPROM
int UnInit (unsigned long fnc) {

  switch (fnc) {
    case 1:
    case 2:
      // Lock PECR register
      FLASH->PECR |= FLASH_PELOCK;              // FLASH_PECR and data memory lock
    break;
  }

  return (0);
}
#endif //FLASH_EEPROM


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_OPTION
int EraseChip (void) {

  // Unprotect Flash, set default values
  M32(0x1FF80000) = 0xFF5500AA;                 // set RDP Level 0
  M32(0x1FF80004) = 0x7F8F8070;                 // set USER default values
  M32(0x1FF80008) = 0xFFFF0000;                 // unprotect sectors
  M32(0x1FF80008) = 0xFFFF0000;                 // unprotect sectors

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                          // Reload IWDG
  }

  // Check for Errors
  if (FLASH->SR & (FLASH_ERRs)) {
    FLASH->SR |= FLASH_ERRs;                    // clear error flags
    return (1);                                 // Failed
  }

  return (0);                                   // Done
}
#endif  // FLASH_OPTION


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEMORY
int EraseSector (unsigned long adr) {

  FLASH->PECR |= FLASH_ERASE;                   // Page or Double Word Erase enabled
  FLASH->PECR |= FLASH_PROG;                    // Program memory selected
     
  M32(adr) = 0x00000000;                        // write '0' to the first address to erase page

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                          // Reload IWDG
  }

  FLASH->PECR &= ~FLASH_ERASE;                  // Page or Double Word Erase disabled
  FLASH->PECR &= ~FLASH_PROG;                   // Program memory deselected   

  return (0);                                   // Done
}
#endif  // FLASH_MEMORY

#ifdef FLASH_OPTION
int EraseSector (unsigned long adr) {

  /*
     No need to erase the Option Bytes.
     If the Option Bytes are erased and reprogrammed than the entire device is erased.
     Because protection level is changed form Level before -> Level 1 -> Level after
  
     see also errate sheet DM00034952. 
   */
  return (0);                                   // Done
}
#endif  // FLASH_OPTION

#ifdef FLASH_EEPROM
int EraseSector (unsigned long adr) {
  unsigned long  cnt = 256;

  adr = (adr + 255) & ~255;                     // adjust Address

  FLASH->PECR |= FLASH_ERASE;                   // Page or Word Erase enabled
  FLASH->PECR |= FLASH_DATA;                    // Program EEPROM selected

  while (cnt) {
    M32(adr) = 0x00000000;                      // write '0' to the first address to erase page

    while (FLASH->SR & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }

    if (FLASH->SR & (FLASH_ERRs)) {            // Check for Errors
      FLASH->SR |= FLASH_ERRs;                 // clear error flags
      return (1);                              // Failed
    }                                          

    adr += 4;
    cnt -= 4;
  }
	
  FLASH->PECR &= ~FLASH_ERASE;                  // Page or Word Erase disabled
  FLASH->PECR &= ~FLASH_DATA;                   // Program EEPROM deselected   
	
  return(0);
}
#endif  // FLASH_EEPROM


/*  
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_OPT
int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat) {
  return (1);                                   // Always Force Erase
}
#endif


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEMORY
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  unsigned long  cnt;
  unsigned long i;

  sz = (sz + 63) & ~63;                        // adjust programming size

  for (i = 0; i < (sz / 64); i++) {
    FLASH->PECR |= FLASH_FPRG;                 // Half Page programming mode enabled
    FLASH->PECR |= FLASH_PROG;                 // Program memory selected
                                               
    cnt = 64;                                  
    while (cnt ) {                             
       M32(adr) = *((unsigned long *)buf);     // Program Word
       adr += 4;                               
       buf += 4;                               
       cnt -= 4;                               
    }                                          
                                               
    while (FLASH->SR & FLASH_BSY) {            
      IWDG->KR = 0xAAAA;                       // Reload IWDG
    }                                          
                                               
    if (FLASH->SR & (FLASH_ERRs)) {            // Check for Errors
      FLASH->SR |= FLASH_ERRs;                 // clear error flags
      return (1);                              // Failed
    }                                          
                                               
    FLASH->PECR &= ~FLASH_FPRG;                // Half Page programming mode disabled
    FLASH->PECR &= ~FLASH_PROG;                // Program memory deselected   
  }                                            
                                               
  return (0);                                   // Done
}
#endif  // FLASH_MEMORY

#ifdef FLASH_OPTION
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = 20;                                      // Adjust programming size to 20 Bytes

  while (sz) {
    /* see errate sheet DM00034952.
       Mass erase is performed if RDP level 0 is written even if RDP level 0 is already set.
       Do not write if the value is already set
     */
    if (M32(adr) != *((unsigned long *)buf)) {
      M32(adr) = *((unsigned long *)buf);// Program Word
    }

    while (FLASH->SR & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }

    if (FLASH->SR & (FLASH_ERRs)) {
      FLASH->SR |= FLASH_ERRs;                  // clear error flags
      return (1);                               // Failed
    }

    adr += 4;
    buf += 4;
    sz  -= 4;
  }

  return (0);                                   // Done
}
#endif  // FLASH_OPTION

#ifdef FLASH_EEPROM
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  unsigned long  cnt = 256;
  
  sz = (sz + 255) & ~255;                       // adjust programming size

  FLASH->PECR &= ~FLASH_FIX;                    // Clear the FTDW bit

  while (sz && cnt) {

    M32(adr) = *((unsigned long *)buf);         // Program Word

    while (FLASH->SR & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }
    adr += 4;
    buf += 4;
    sz  -= 4;
    cnt -= 4;
  }

  return (0);                                   // Done
}
#endif  // FLASH_EEPROM

#ifdef FLASH_OPTION
unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = 20;                                      // Adjust programming size to 20 Bytes

  while (sz) {
    if (M32(adr) != *((unsigned long *)buf)) {
      return(adr);                              // failed
    }

    while (FLASH->SR & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }

    adr += 4;
    buf += 4;
    sz  -= 4;
  }

  return (adr + sz);                            // Done
}
#endif
