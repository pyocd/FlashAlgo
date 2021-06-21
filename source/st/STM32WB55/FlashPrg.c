/* -----------------------------------------------------------------------------
 * Copyright (c) 2021 ZeUGMA.
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
 * $Date:        16/06/2021
 * $Revision:    V1.00
 *  
 * Project:      Flash Programming Functions for STMicroelectronics STM32WB55xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "FlashOS.h"        // FlashOS Structures
#include "stm32wbxx.h"
#include "stm32wb55xx.h"

// Flash Keys
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB
#define FLASH_OPTKEY1   0x08192A3B
#define FLASH_OPTKEY2   0x4C5D6E7F

#define FLASH_PGERR             (FLASH_SR_PGSERR | FLASH_SR_PROGERR  | FLASH_SR_PGAERR | FLASH_SR_WRPERR)
#define FLASH_CACHE_MASK        (FLASH_ACR_DCRST | FLASH_ACR_ICRST | FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN)



/*
 * Get Sector Number
 *    Parameter:      adr:  Sector Address
 *    Return Value:   Sector Number
 */
uint32_t GetSecNum (uint32_t adr) {
  uint32_t n;

  n = adr - 0x08000000 ;    // Get offset from base address
  return n >> 12;           // Then devide the offset by the sector size (4096 = 0x1000 => shift 12 bits) 
}

/*
 *  Waiting for the flash being available
 */ 
void WaitFlashBusy(){
  while (FLASH->SR & FLASH_SR_BSY);
}

/*
 *  Waiting for the programming / erase are available
 */ 
void WaitProgrammingEraseSuspend(){
  while (FLASH->SR & FLASH_SR_PESD);
}

/*
 * Get the starting address of the first flash secured page
 */
uint32_t GetStartSecureFlash(){

  uint32_t base = ( FLASH->SFR & FLASH_SFR_SFSA );
  base <<= 12;

  return (base + 0x08000000);
}

/*
 * Write a double word on Flash memory
 */
void WriteDoubleWord(uint32_t address, uint64_t data){

  *(uint32_t *)address     = (uint32_t)(data);
  __ISB();
  *(uint32_t *)(address+4U) = (uint32_t)(data >> 32U);

}

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t Init (uint32_t adr, uint32_t clk, uint32_t fnc) {


  if( READ_BIT( FLASH->CR, FLASH_CR_LOCK ) != 0){           // Unlock Flash
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);
  }

  FLASH->SR |= FLASH_PGERR;                            // Reset Error Flags

  return 0;
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t UnInit (uint32_t fnc) {
  uint32_t acr;

  acr = FLASH->ACR;                                     // Clear flash caches
  FLASH->ACR = acr & ~FLASH_CACHE_MASK;
  FLASH->ACR = acr;

  FLASH->CR |=  FLASH_CR_LOCK;                             // Lock Flash


  return 0;
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSector (uint32_t adr) {
  uint32_t n;

  if( adr >= GetStartSecureFlash() ){
    return 1;
  }

  n = GetSecNum(adr);                                   // Get Sector Number

  WaitFlashBusy();
  WaitProgrammingEraseSuspend();

  FLASH->SR |= FLASH_PGERR;                                   // Reset Error Flags

  FLASH->CR  =  FLASH_CR_PER;                                 // Sector Erase Enabled 
  FLASH->CR |=  ((n << FLASH_CR_PNB_Pos) & FLASH_CR_PNB_Msk); // Sector Number
  FLASH->CR |=  FLASH_CR_STRT;                                // Start Erase

  WaitFlashBusy();

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR |= FLASH_PGERR;                           // Reset Error Flags
    return 1;                                         // Failed
  }

  return 0;                                           // Done
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseChip (void) {

  for( uint32_t adr = 0x08000000; adr < GetStartSecureFlash(); adr += 0x1000 ){
    if( EraseSector(adr) != 0 ){
      return 1;
    }
  }

  return 0;                   // Done
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t ProgramPage (uint32_t adr, uint32_t sz, uint32_t *buf) {
  sz  = (sz + 3) & ~3;  // Adjust size for Words
  adr &= ~7;            // Adjust to 64 bit address

  if( adr >= GetStartSecureFlash() ){
    return 0;
  }

  WaitFlashBusy();
  WaitProgrammingEraseSuspend();

  FLASH->SR |= FLASH_PGERR;                             // Reset Error Flags
  FLASH->CR = FLASH_CR_EOPIE;                         // Reset & Enable Programming Interrupt

  uint64_t data = 0;

  while (sz > 0) {

    FLASH->CR |= FLASH_CR_PG;                            // Programming Enabled

    if( adr >= GetStartSecureFlash() ){
      FLASH->CR &= ~FLASH_CR_PG;              // Programming Disabled
      FLASH->SR |= FLASH_PGERR;               // Reset Error Flags
      return 1;
    }

    data =  ((uint64_t)(*buf));
    data += ((uint64_t)(*(buf+1))) << 32;

    WriteDoubleWord(adr, data);

    adr += 8;
    buf += 2;
    sz  -= 8;

    WaitFlashBusy();

    if( (FLASH->SR & FLASH_SR_EOP) == 0 ){    // Writing failed
      FLASH->CR &= ~FLASH_CR_PG;              // Programming Disabled
      FLASH->SR |= FLASH_PGERR;               // Reset Error Flags
      return 1;                               // Failed
    }

    FLASH->SR |= FLASH_SR_EOP;            // Clear EOP flag
    FLASH->CR &= ~FLASH_CR_PG;            // Programming Disabled
  }

  return 0;                                           // Done
}
