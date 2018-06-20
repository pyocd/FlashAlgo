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
#include "inc/prcm.h"
#include "inc/hw_apps_rcm.h"
#include "inc/hw_mcspi.h"

#define HAVE_WRITE_BUFFER       1



//*****************************************************************************
// Global Peripheral clock and rest Registers
//*****************************************************************************
static const PRCM_PeriphRegs_t PRCM_PeriphRegsList[] =
{

	{APPS_RCM_O_CAMERA_CLK_GATING,   APPS_RCM_O_CAMERA_SOFT_RESET   },
	{APPS_RCM_O_MCASP_CLK_GATING,    APPS_RCM_O_MCASP_SOFT_RESET    },
	{APPS_RCM_O_MMCHS_CLK_GATING,    APPS_RCM_O_MMCHS_SOFT_RESET    },
	{APPS_RCM_O_MCSPI_A1_CLK_GATING, APPS_RCM_O_MCSPI_A1_SOFT_RESET },
	{APPS_RCM_O_MCSPI_A2_CLK_GATING, APPS_RCM_O_MCSPI_A2_SOFT_RESET },
	{APPS_RCM_O_UDMA_A_CLK_GATING,   APPS_RCM_O_UDMA_A_SOFT_RESET   },
	{APPS_RCM_O_GPIO_A_CLK_GATING,   APPS_RCM_O_GPIO_A_SOFT_RESET   },
	{APPS_RCM_O_GPIO_B_CLK_GATING,   APPS_RCM_O_GPIO_B_SOFT_RESET   },
	{APPS_RCM_O_GPIO_C_CLK_GATING,   APPS_RCM_O_GPIO_C_SOFT_RESET   },
	{APPS_RCM_O_GPIO_D_CLK_GATING,   APPS_RCM_O_GPIO_D_SOFT_RESET   },
	{APPS_RCM_O_GPIO_E_CLK_GATING,   APPS_RCM_O_GPIO_E_SOFT_RESET   },
	{APPS_RCM_O_WDOG_A_CLK_GATING,   APPS_RCM_O_WDOG_A_SOFT_RESET   },
	{APPS_RCM_O_UART_A0_CLK_GATING,  APPS_RCM_O_UART_A0_SOFT_RESET  },
	{APPS_RCM_O_UART_A1_CLK_GATING,  APPS_RCM_O_UART_A1_SOFT_RESET  },
	{APPS_RCM_O_GPT_A0_CLK_GATING ,  APPS_RCM_O_GPT_A0_SOFT_RESET   },
	{APPS_RCM_O_GPT_A1_CLK_GATING,   APPS_RCM_O_GPT_A1_SOFT_RESET   },
	{APPS_RCM_O_GPT_A2_CLK_GATING,   APPS_RCM_O_GPT_A2_SOFT_RESET   },
	{APPS_RCM_O_GPT_A3_CLK_GATING,   APPS_RCM_O_GPT_A3_SOFT_RESET   },
	{APPS_RCM_O_CRYPTO_CLK_GATING,   APPS_RCM_O_CRYPTO_SOFT_RESET   },
	{APPS_RCM_O_MCSPI_S0_CLK_GATING, APPS_RCM_O_MCSPI_S0_SOFT_RESET },
	{APPS_RCM_O_I2C_CLK_GATING,      APPS_RCM_O_I2C_SOFT_RESET      }

};

void
UtilsDelay(unsigned long ulCount)
{
	__asm("Count:   SUBS    ulCount, #1\n"
          "    BNE    Count");
}

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    // Called to configure the SoC. Should enable clocks
    //  watchdogs, peripherals and anything else needed to
    //  access or program memory. Fnc parameter has meaning
    //  but currently isnt used in MSC programming routines
    unsigned long ulRegValue;

    //
    // DIG DCDC LPDS ECO Enable
    //
    HWREG(0x4402F064) |= 0x800000;

    //
    // Enable hibernate ECO for PG 1.32 devices only. With this ECO enabled,
    // any hibernate wakeup source will be kept maked until the device enters
    // hibernate completely (analog + digital)
    //
    ulRegValue = HWREG(HIB3P3_BASE  + HIB3P3_O_MEM_HIB_REG0);
		UtilsDelay((80*200)/3);
	
    //PRCMHIBRegWrite(HIB3P3_BASE + HIB3P3_O_MEM_HIB_REG0, ulRegValue | (1<<4));
    HWREG(HIB3P3_BASE + HIB3P3_O_MEM_HIB_REG0) = ulRegValue | (1<<4);
    UtilsDelay((80*200)/3);
    
    //
    // Handling the clock switching (for 1.32 only)
    //
    HWREG(0x4402E16C) |= 0x3C;
    
    //
    //Enable uDMA
    //
    //
    // Enable the specified peripheral clocks, Nothing to be done for PRCM_ADC
    // as it is a dummy define for pinmux utility code generation
    //
    if(PRCM_UDMA != PRCM_ADC)
    {
        HWREG(ARCM_BASE + PRCM_PeriphRegsList[PRCM_UDMA].ulClkReg) |= PRCM_RUN_MODE_CLK;
    }
    
    //
    // Checking ROM Version less than 2.x.x. 
    // Only for driverlib backward compatibility
    //
    if( (HWREG(0x00000400) & 0xFFFF) < 2 )
    {
        //
        // Set the default clock for camera
        //
        if(PRCM_UDMA == PRCM_CAMERA)
        {
            HWREG(ARCM_BASE + APPS_RCM_O_CAMERA_CLK_GEN) = 0x0404;
        }
    }


    //
    // Reset uDMA
    volatile unsigned long ulDelay;

    if( PRCM_UDMA != PRCM_DTHE)
    {
        //
        // Assert the reset
        //
        HWREG(ARCM_BASE + PRCM_PeriphRegsList[PRCM_UDMA].ulRstReg)
                                                            |= 0x00000001;
        //
        // Delay a little bit.
        //
        for(ulDelay = 0; ulDelay < 16; ulDelay++)
        {
        }

        //
        // Deassert the reset
        //
        HWREG(ARCM_BASE+PRCM_PeriphRegsList[PRCM_UDMA].ulRstReg)
                                                            &= ~0x00000001;
    }


    //
    //
    // Disable uDMA
    HWREG(ARCM_BASE + PRCM_PeriphRegsList[PRCM_UDMA].ulClkReg) &= ~PRCM_RUN_MODE_CLK;

    //
    // Enable RTC
    //
    unsigned long lWakeupStatus = (HWREG(GPRCM_BASE+ GPRCM_O_APPS_RESET_CAUSE) & 0xFF);
    if(lWakeupStatus == PRCM_POWER_ON)
    {
        HWREG(0x4402F804) = PRCM_POWER_ON;
        UtilsDelay((80*200)/3);
    }

    //
    // SWD mode
    //
    if(((HWREG(0x4402F0C8) & 0xFF) == 0x2))
    {
        HWREG(0x4402E110) = ((HWREG(0x4402E110) & ~0xC0F) | 0x2);
        HWREG(0x4402E114) = ((HWREG(0x4402E114) & ~0xC0F) | 0x2);
    }
    
    //
    // Override JTAG mux
    //
    HWREG(0x4402E184) |= 0x2;
    

    //
    // DIG DCDC VOUT trim settings based on PROCESS INDICATOR
    //
    if(((HWREG(0x4402DC78) >> 22) & 0xF) == 0xE)
    {
        HWREG(0x4402F0B0) = ((HWREG(0x4402F0B0) & ~(0x00FC0000))|(0x32 << 18));
    }
    else
    {
        HWREG(0x4402F0B0) = ((HWREG(0x4402F0B0) & ~(0x00FC0000))|(0x29 << 18));
    }

    //
    // Enable SOFT RESTART in case of DIG DCDC collapse
    //
    HWREG(0x4402FC74) &= ~(0x10000000);

    //
    // Required only if ROM version is lower than 2.x.x
    //
    if( (HWREG(0x00000400) & 0xFFFF) < 2 )
    {
      //
      // Disable the sleep for ANA DCDC
      //
      HWREG(0x4402F0A8) |= 0x00000004 ;
    }
    else if( (HWREG(0x00000400) >> 16)  >= 1 )
    {
      //
      // Enable NWP force reset and HIB on WDT reset
      // Enable direct boot path for flash
      //
      HWREG(OCP_SHARED_BASE + 0x00000188) |= ((7<<5) | 0x1);
      if((HWREG(HIB3P3_BASE + HIB3P3_O_MEM_HIB_REG2) & 0x1) )
      {
          HWREG(HIB3P3_BASE + HIB3P3_O_MEM_HIB_REG2) &= ~0x1;
          HWREG(OCP_SHARED_BASE + 0x00000188) |= (1<<9);

          //
          // Clear the RTC hib wake up source
          //
          HWREG(HIB3P3_BASE+HIB3P3_O_MEM_HIB_RTC_WAKE_EN) &= ~0x1;

          //
          // Reset RTC match value
          //
          HWREG(HIB3P3_BASE + HIB3P3_O_MEM_HIB_RTC_WAKE_LSW_CONF) = 0;
          HWREG(HIB3P3_BASE + HIB3P3_O_MEM_HIB_RTC_WAKE_MSW_CONF) = 0;

      }
    }

    unsigned long efuse_reg2;
    unsigned long ulDevMajorVer, ulDevMinorVer;
    //
    // Read the device identification register
    //
    efuse_reg2= HWREG(GPRCM_BASE + GPRCM_O_GPRCM_EFUSE_READ_REG2);
    

    //
    // Read the ROM mojor and minor version
    //
    ulDevMajorVer = ((efuse_reg2 >> 28) & 0xF);
    ulDevMinorVer = ((efuse_reg2 >> 24) & 0xF);
  
    if(((ulDevMajorVer == 0x3) && (ulDevMinorVer == 0)) || (ulDevMajorVer < 0x3))
    {
      unsigned int Scratch, PreRegulatedMode;

      // 0x4402F840 => 6th bit “1” indicates device is in pre-regulated mode.
      PreRegulatedMode = (HWREG(0x4402F840) >> 6) & 1;
    
      if( PreRegulatedMode)
      {
        Scratch = HWREG(0x4402F028);
        Scratch &= 0xFFFFFF7F; // <7:7> = 0
        HWREG(0x4402F028) = Scratch;
        
        Scratch = HWREG(0x4402F010);
        Scratch &= 0x0FFFFFFF; // <31:28> = 0
        Scratch |= 0x10000000; // <31:28> = 1
        HWREG(0x4402F010) = Scratch;
      }
      else
      {
        Scratch = HWREG(0x4402F024);

        Scratch &= 0xFFFFFFF0; // <3:0> = 0
        Scratch |= 0x00000001; // <3:0> = 1
        Scratch &= 0xFFFFF0FF; // <11:8> = 0000
        Scratch |= 0x00000500; // <11:8> = 0101
        Scratch &= 0xFFFE7FFF; // <16:15> = 0000
        Scratch |= 0x00010000; // <16:15> = 10

        HWREG(0x4402F024) = Scratch;

        Scratch = HWREG(0x4402F028);

        Scratch &= 0xFFFFFF7F; // <7:7> = 0
        Scratch &= 0x0FFFFFFF; // <31:28> = 0
        Scratch &= 0xFF0FFFFF; // <23:20> = 0
        Scratch |= 0x00300000; // <23:20> = 0011
        Scratch &= 0xFFF0FFFF; // <19:16> = 0
        Scratch |= 0x00030000; // <19:16> = 0011

        HWREG(0x4402F028) = Scratch;
        HWREG(0x4402F010) &= 0x0FFFFFFF; // <31:28> = 0
      }
    }



    //
    // Success.
    //
    return 0;
}


uint32_t UnInit(uint32_t fnc)
{
    // When a session is complete this is called to powerdown
    //  communication channels and clocks that were enabled
    //  Fnc parameter has meaning but isnt used in MSC program
    //  routines
	
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
        return 1;
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
        return(1);
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
