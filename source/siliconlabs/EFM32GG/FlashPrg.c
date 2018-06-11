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

/** @file FlashPrg.c */

#include "FlashOS.h"        /* FlashOS Structures */
#include "FlashPrg.h"

/* Defines required by em_msc */
#include "core_cm3.h"

#define MSC_PRESENT
#define MSC_COUNT            1
#define MSC_BASE             (0x400C0000UL) /**< MSC base address  */
#define BITBAND_PER_BASE     ((uint32_t) 0x42000000UL) /**< Peripheral Address Space bit-band area */
#define BITBAND_RAM_BASE     ((uint32_t) 0x22000000UL) /**< SRAM Address Space bit-band area */
#define PER_MEM_BASE         ((uint32_t) 0x40000000UL) /**< PER base address  */

#include "efm32gg_msc.h"
#define MSC_UNLOCK_CODE       0x1B71                                  /**< MSC unlock code */
#define MSC                   ((MSC_TypeDef *) MSC_BASE)              /**< MSC base pointer */
#include "em_msc.h"

#define FLASH_PAGE_MASK (~(FLASH_PAGE_SIZE-1))

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Define page size */
#if defined(EFM32GG_1024) || defined(EFM32GG_512)
#define FLASH_PAGE_SIZE       4096
#else
#define FLASH_PAGE_SIZE       2048
#endif

static msc_Return_TypeDef MscStatusWait( uint32_t mask, uint32_t value )
{
  uint32_t status;
  int timeOut = MSC_PROGRAM_TIMEOUT;

  while (1)
  {
    status = MSC->STATUS;

    if ( status &
         ( MSC_STATUS_LOCKED | MSC_STATUS_INVADDR | MSC_STATUS_WORDTIMEOUT ) )
    {
      MSC->WRITECTRL &= ~(MSC_WRITECTRL_WREN | MSC_WRITECTRL_WDOUBLE);

      if ( status & MSC_STATUS_LOCKED )
        return mscReturnLocked;

      if ( status & MSC_STATUS_INVADDR )
        return mscReturnInvalidAddr;

      return mscReturnTimeOut;
    }

    if ( ( status & mask ) == value )
      return mscReturnOk;

    timeOut--;
    if ( timeOut == 0 )
      break;
  }
  return mscReturnTimeOut;
}

static msc_Return_TypeDef DoFlashCmd( uint32_t cmd )
{
  MSC->WRITECMD = cmd;
  return MscStatusWait( MSC_STATUS_BUSY, 0 );
}

static msc_Return_TypeDef PgmWord( uint32_t addr, uint32_t data )
{
  MSC->ADDRB    = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  MSC->WDATA    = data;
  return DoFlashCmd( MSC_WRITECMD_WRITEONCE );
}

static msc_Return_TypeDef PgmBurstDouble( uint32_t addr, uint32_t *p, uint32_t cnt )
{
  msc_Return_TypeDef retVal;

  if ( (retVal = MscStatusWait( MSC_STATUS_BUSY, 0 )) != mscReturnOk )
    return retVal;

  MSC->ADDRB    = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  MSC->WDATA    = *p++;
  MSC->WDATA    = *p++;
  MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;
  cnt          -= 8;

  while ( cnt > 7 )
  {
    if ( (retVal = MscStatusWait( MSC_STATUS_WDATAREADY, MSC_STATUS_WDATAREADY )) != mscReturnOk )
      return retVal;

    MSC->WDATA = *p++;
    MSC->WDATA = *p++;
    cnt       -= 8;
  }
  return mscReturnOk;
}

/*****************************************************************************
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 ****************************************************************************/
uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
  /* Unlock the MSC */
  MSC->LOCK = MSC_UNLOCK_CODE;

  return 0;
}


/*****************************************************************************
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 ****************************************************************************/
uint32_t UnInit(uint32_t fnc)
{
  /* Disable write in MSC */
  MSC->WRITECTRL &= ~(MSC_WRITECTRL_WREN | MSC_WRITECTRL_WDOUBLE);
  return 0;
}


/*****************************************************************************
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 ****************************************************************************/
uint32_t EraseChip(void)
{
  msc_Return_TypeDef result;

  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
  MSC->MASSLOCK   = MSC_MASSLOCK_LOCKKEY_UNLOCK;

  result = DoFlashCmd( MSC_WRITECMD_ERASEMAIN0 | MSC_WRITECMD_ERASEMAIN1 );

  MSC->MASSLOCK   = 0;
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

  return result == mscReturnOk ? 0 : 1;
}

/*****************************************************************************
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 ****************************************************************************/
uint32_t EraseSector(uint32_t adr)
{
  uint32_t            blank;
  msc_Return_TypeDef  result    = mscReturnOk;
  uint32_t            *p        = (uint32_t*)adr;
  uint32_t            pageSize  = FLASH_PAGE_SIZE;

  do
  {
    blank     = *p++;
    pageSize -= 4;
  } while ( pageSize && ( blank == 0xFFFFFFFF ) );

  if ( blank != 0xFFFFFFFF )
  {
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
    MSC->ADDRB      = adr;
    MSC->WRITECMD   = MSC_WRITECMD_LADDRIM;
    result          = DoFlashCmd( MSC_WRITECMD_ERASEPAGE );
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
  }

  return result == mscReturnOk ? 0 : 1;
}

/*****************************************************************************
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 ****************************************************************************/
uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
  uint32_t burst;

  sz = (sz + 3) & ~3;                     /* Make sure we are modulo 4. */

  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

  if ( sz > 7 )
  {
    if ( adr & 7 )    /* Start address not on 8 byte boundary ? */
    {
      if ( PgmWord( adr, *(uint32_t*)buf ) != mscReturnOk )
        return 1;

      buf += 4;
      adr += 4;
      sz  -= 4;
    }

    MSC->WRITECTRL |= MSC_WRITECTRL_WDOUBLE;
    while ( sz > 7 )
    {
      /* Max burst len is up to next flash page boundary. */
      burst = MIN( sz, ( ( adr + FLASH_PAGE_SIZE ) & FLASH_PAGE_MASK ) - adr );

      if ( burst & 4 )    /* Make sure we are modulo 8. */
        burst -= 4;

      if ( PgmBurstDouble( adr, (uint32_t*)buf, burst ) != mscReturnOk )
        return 1;

      buf += burst;
      adr += burst;
      sz  -= burst;
    }
    if ( MscStatusWait( MSC_STATUS_BUSY, 0 ) != mscReturnOk )
      return 1;

    MSC->WRITECTRL &= ~MSC_WRITECTRL_WDOUBLE;
  }

  if ( sz )
  {
    if ( PgmWord( adr, *(uint32_t*)buf ) != mscReturnOk )
      return 1;
  }

  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

  return 0;
}

/*
uint32_t BlankCheck(uint32_t adr, uint32_t sz, uint8_t pat)
{
    // Check that the memory at address adr for length sz is 
    //  empty or the same as pat
    return 1;
}

uint32_t Verify(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    // Given an adr and sz compare this against the content of buf
    return 1;
}
*/
