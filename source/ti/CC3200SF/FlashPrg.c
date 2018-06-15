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

#include "FlashOS.h"
#include "FlashPrg.h"
#include "inc/hw_types.h"
#include "inc/hw_flash_ctrl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gprcm.h"
#include "inc/hw_hib1p2.h"
#include "inc/hw_hib3p3.h"
#include "inc/hw_common_reg.h"
#include "inc/hw_stack_die_ctrl.h"

#define HAVE_WRITE_BUFFER       1

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    // Called to configure the SoC. Should enable clocks
    //  watchdogs, peripherals and anything else needed to
    //  access or program memory. Fnc parameter has meaning
    //  but currently isnt used in MSC programming routines
     unsigned long ulRegVal;

      //
      // DIG DCDC NFET SEL and COT mode disable
      //
      HWREG(0x4402F010) = 0x30031820;
      HWREG(0x4402F00C) = 0x04000000;

      int i =0;
			while(i<32000){
				i++;
			}

      //
      // ANA DCDC clock config
      //
      HWREG(0x4402F11C) = 0x099;
      HWREG(0x4402F11C) = 0x0AA;
      HWREG(0x4402F11C) = 0x1AA;

      //
      // PA DCDC clock config
      //
      HWREG(0x4402F124) = 0x099;
      HWREG(0x4402F124) = 0x0AA;
      HWREG(0x4402F124) = 0x1AA;

      //
      // TD Flash timing configurations in case of MCU WDT reset
      //
      if((HWREG(0x4402D00C) & 0xFF) == 0x00000005)
      {
          HWREG(0x400F707C) |= 0x01840082;
          HWREG(0x400F70C4)= 0x1;
          HWREG(0x400F70C4)= 0x0;
      }

      //
      // Take I2C semaphore
      //
      ulRegVal = HWREG(0x400F7000);
      ulRegVal = (ulRegVal & ~0x3) | 0x1;
      HWREG(0x400F7000) = ulRegVal;

      //
      // Take GPIO semaphore
      //
      ulRegVal = HWREG(0x400F703C);
      ulRegVal = (ulRegVal & ~0x3FF) | 0x155;
      HWREG(0x400F703C) = ulRegVal;

      //
      // Enable 32KHz internal RC oscillator
      //
      //PRCMHIBRegWrite(HIB3P3_BASE+HIB3P3_O_MEM_INT_OSC_CONF, 0x00000101);

			HWREG(HIB3P3_BASE+HIB3P3_O_MEM_INT_OSC_CONF) =  0x00000101;
			i =0;
			while(i<8000){
				i++;
			}

      //
      // Delay for a little bit.
      //
			i =0;
			while(i<8000){
				i++;
			}

      //
      // Enable 16MHz clock
      //
      HWREG(HIB1P2_BASE+HIB1P2_O_CM_OSC_16M_CONFIG) = 0x00010008;

      //
      // Delay for a little bit.
      //
      i =0;
			while(i<8000){
			i++;
			};
    	return 0;
}

uint32_t UnInit(uint32_t fnc)
{
    // When a session is complete this is called to powerdown
    //  communication channels and clocks that were enabled
    //  Fnc parameter has meaning but isnt used in MSC program
    //  routines
    HWREG(GPRCM_BASE+ GPRCM_O_APPS_SOFT_RESET) = 0x2;
    return 0;
}



uint32_t EraseChip(void)
{
    // Execute a sequence that erases the entire of flash memory region 

    //
    // Clear the flash access and error interrupts.
    //
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCMISC) =
      (FLASH_CTRL_FCMISC_AMISC | FLASH_CTRL_FCMISC_VOLTMISC |
                           FLASH_CTRL_FCMISC_ERMISC);

    //
    // Command the flash controller for mass erase.
    //
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC) =
      FLASH_CTRL_FMC_WRKEY | FLASH_CTRL_FMC_MERASE1;

    //
    // Wait until mass erase completes.
    //
    while(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC) & FLASH_CTRL_FMC_MERASE1)
    {

    }

    //
    // Return an error if an access violation or erase error occurred.
    //
    if(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCRIS)
       & (FLASH_CTRL_FCRIS_ARIS | FLASH_CTRL_FCRIS_VOLTRIS |
                             FLASH_CTRL_FCRIS_ERRIS))
    {
        return -1;
    }

    //
    // Success.
    //
    return 0;
}

uint32_t EraseSector(uint32_t adr)
{
    // Execute a sequence that erases the sector that adr resides in
    //
    // Check the arguments.
    //
    //ASSERT(!(adr & (FLASH_CTRL_ERASE_SIZE - 1)));
			

    //
    // Clear the flash access and error interrupts.
    //
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCMISC)
      = (FLASH_CTRL_FCMISC_AMISC | FLASH_CTRL_FCMISC_VOLTMISC |
                           FLASH_CTRL_FCMISC_ERMISC);

    // Erase the block.
    //
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMA) = adr;
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC)
                                = FLASH_CTRL_FMC_WRKEY | FLASH_CTRL_FMC_ERASE;

    //
    // Wait until the block has been erased.
    //
    while(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC) & FLASH_CTRL_FMC_ERASE)
    {
    }

    //
    // Return an error if an access violation or erase error occurred.
    //
    if(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCRIS)
       & (FLASH_CTRL_FCRIS_ARIS | FLASH_CTRL_FCRIS_VOLTRIS |
                             FLASH_CTRL_FCRIS_ERRIS))
    {
        return(-1);
    }

    //
    // Success.
    //
    return(0);
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    // Program the contents of buf starting at adr for length of sz
    //
    // Check the arguments.
    //
    //ASSERT(!(adr & 3));
    //ASSERT(!(sz & 3));
    
    //
    // Clear the flash access and error interrupts.
    //
    HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCMISC)
      = (FLASH_CTRL_FCMISC_AMISC | FLASH_CTRL_FCMISC_VOLTMISC |
                           FLASH_CTRL_FCMISC_INVDMISC | FLASH_CTRL_FCMISC_PROGMISC);


    //
    // See if this device has a write buffer.
    //

#if HAVE_WRITE_BUFFER
    {
        //
        // Loop over the words to be programmed.
        //
        while(sz)
        {
            //
            // Set the address of this block of words. for 1 MB
            //
            HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMA) = adr & ~(0x7F);

            //
            // Loop over the words in this 32-word block.
            //
            while(((adr & 0x7C) ||
                   (HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FWBVAL) == 0)) &&
                  (sz != 0))
            {
                //
                // Write this word into the write buffer.
                //
                HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FWBN
                      + (adr & 0x7C)) = *buf++;
                adr += 4;
                sz -= 4;
            }

            //
            // Program the contents of the write buffer into flash.
            //
            HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC2)
              = FLASH_CTRL_FMC2_WRKEY | FLASH_CTRL_FMC2_WRBUF;

            //
            // Wait until the write buffer has been programmed.
            //
            while(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FMC2) & FLASH_CTRL_FMC2_WRBUF)
            {
            }
        }
    }
#else
    {
    }
#endif
    //
    // Return an error if an access violation occurred.
    //

    if(HWREG(FLASH_CONTROL_BASE + FLASH_CTRL_O_FCRIS) & (FLASH_CTRL_FCRIS_ARIS | FLASH_CTRL_FCRIS_VOLTRIS |
                             FLASH_CTRL_FCRIS_INVDRIS | FLASH_CTRL_FCRIS_PROGRIS))

    {
        return(-1);
    }

    //
    // Success.
    //
    return(0);
}
/*uint32_t BlankCheck(uint32_t adr, uint32_t sz, uint8_t pat)
{
    // Check that the memory at address adr for length sz is 
    //  empty or the same as pat
    return 1;
}*/


/*
uint32_t Verify(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    // Given an adr and sz compare this against the content of buf
    return 1;
}*/
