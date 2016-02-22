/* -----------------------------------------------------------------------------
 * Copyright (c) 2004 - 2016 ARM Ltd.
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
 * $Date:        22. Deb 2016
 * $Revision:    V1.10
 *
 * Project:      Flash Device Algorithm for NXP LPC8xx Flash using IAP
 * --------------------------------------------------------------------------- */

#include "FlashOS.H"        // FlashOS Structures

// Memory Mapping Control
#define MEMMAP     (*((volatile unsigned char *) 0x40048000))

// Main Clock
#define MAINCLKSEL (*((volatile unsigned long *) 0x40048070))
#define MAINCLKUEN (*((volatile unsigned long *) 0x40048074))
#define MAINCLKDIV (*((volatile unsigned long *) 0x40048078))

#define END_SECTOR    31
#define _CCLK         12000

struct sIAP {                  // IAP Structure
    unsigned long cmd;         // Command
    unsigned long par[4];      // Parameters
    unsigned long stat;        // Status
    unsigned long res[2];      // Result
} IAP;


/* IAP Call */
typedef void (*IAP_Entry) (unsigned long *cmd, unsigned long *stat);
#define IAP_Call ((IAP_Entry) 0x1FFF1FF1)

/**
 * Get Sector Number
 *    Parameter:      adr:  Sector Address
 *    Return Value:   Sector Number
 */
unsigned long GetSecNum (unsigned long adr)
{
    unsigned long n;
    n = adr >> 10;     //  1kB Sector
    return (n);
}


/**
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


/**
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int UnInit (unsigned long fnc)
{
    return (0);
}


/**
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseChip (void)
{
    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = 0;                              // Start Sector
    IAP.par[1] = END_SECTOR;                     // End Sector
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = 0;                              // Start Sector
    IAP.par[1] = END_SECTOR;                     // End Sector
    IAP.par[2] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed

    return (0);                                  // Finished without Errors
}


/**
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
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
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP.par[2] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed

    return (0);                                  // Finished without Errors
}


/**
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf)
{
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
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 51;                             // Copy RAM to Flash
    IAP.par[0] = adr;                            // Destination Flash Address
    IAP.par[1] = (unsigned long)buf;             // Source RAM Address
    IAP.par[2] = 512;                            // Fixed Page Size
    IAP.par[3] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) return (1);                    // Command Failed

    return (0);                                  // Finished without Errors
}
