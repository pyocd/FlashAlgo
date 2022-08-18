/* Flash Algorithm for NXP IAP 32kB devices
 * Copyright (c) 2009-2016 ARM Limited
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

#include "FlashOS.h"        // FlashOS Structures

// Memory Mapping Control
#define MEMMAP     (*((volatile unsigned long *) 0x40048000))

// Clock Control
#define MAINCLKSEL (*((volatile unsigned long *) 0x40048070))
#define MAINCLKUEN (*((volatile unsigned long *) 0x40048074))
#define MAINCLKDIV (*((volatile unsigned long *) 0x40048078))

// Phase Locked Loop (Main PLL)
#define PLL0CON    (*((volatile unsigned char *) 0x400FC080))
#define PLL0CFG    (*((volatile unsigned long *) 0x400FC084))
#define PLL0STAT   (*((volatile unsigned long *) 0x400FC088))
#define PLL0FEED   (*((volatile unsigned char *) 0x400FC08C))

#define END_SECTOR (7)
#define _CCLK      (12000)

struct sIAP {                  // IAP Structure
  unsigned long cmd;           // Command
  unsigned long par[4];        // Parameters
  unsigned long stat;          // Status
  unsigned long res[2];        // Result
} IAP;


/* IAP Call */
typedef void (*IAP_Entry) (unsigned long *cmd, unsigned long *stat);
#define IAP_Call ((IAP_Entry) 0x1FFF1FF1)


/*
 * Get Sector Number

 *    Parameter:      adr:  Sector Address
 *    Return Value:   Sector Number
 */

unsigned long GetSecNum (unsigned long adr)
{
    return (adr >> 12);
}

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc)
{
    MAINCLKSEL = 0;                              // Select Internal RC Oscillator
    MAINCLKUEN = 1;                              // Update Main Clock Source
    MAINCLKUEN = 0;                              // Toggle Update Register
    MAINCLKUEN = 1;

    while (!(MAINCLKUEN & 1))                    // Wait until updated
        ;

    MAINCLKDIV = 1;                              // Set Main Clock divider to 1
    MEMMAP     = 0x02;                           // User Flash Mode

    return (0);
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int UnInit (unsigned long fnc) {
    return (0);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseChip (void)
{
    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = 0;                              // Start Sector
    IAP.par[1] = END_SECTOR;                     // End Sector
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = 0;                              // Start Sector
    IAP.par[1] = END_SECTOR;                     // End Sector
    IAP.par[2] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }
    
    return (0);                                  // Finished without Errors
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Memory Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr)
{
    unsigned long n;

    n = GetSecNum(adr);                          // Get Sector Number

    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP.par[2] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    return (0);                                  // Finished without Errors
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf)
{
#warning why does not it use sz?
    unsigned long n;

    if (adr == 0) {                              // Check for Vector Table
        n = *((unsigned long *)(buf + 0x00)) +
            *((unsigned long *)(buf + 0x04)) +
            *((unsigned long *)(buf + 0x08)) +
            *((unsigned long *)(buf + 0x0C)) +
            *((unsigned long *)(buf + 0x10)) +
            *((unsigned long *)(buf + 0x14)) +
            *((unsigned long *)(buf + 0x18));
        *((unsigned long *)(buf + 0x1C)) = 0 - n;  // Signature at Reserved Vector
    }

    n = GetSecNum(adr);                          // Get Sector Number

    IAP.cmd    = 50;                             // Prepare Sector for Write
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    IAP.cmd    = 51;                             // Copy RAM to Flash
    IAP.par[0] = adr;                            // Destination Flash Address
    IAP.par[1] = (unsigned long)buf;             // Source RAM Address
    IAP.par[2] = 256;                            // Fixed Page Size
    IAP.par[3] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    return (0);                                  // Finished without Errors
}
