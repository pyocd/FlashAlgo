/* Flash Algorithm for Embedded Artists LPC4088 boards with 512kB + QSPI
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

#include "FlashOS.H"        // FlashOS Structures

// Memory Mapping Control
#define MEMMAP   (*((volatile unsigned char *) 0x400FC040))

// Phase Locked Loop (Main PLL)
#define PLL0CON           (*((volatile unsigned char *) 0x400FC080))
#define PLL0CFG           (*((volatile unsigned long *) 0x400FC084))
#define PLL0STAT          (*((volatile unsigned long *) 0x400FC088))
#define PLL0FEED          (*((volatile unsigned char *) 0x400FC08C))

#define END_SECTOR     29

unsigned long _CCLK;           // CCLK in kHz

struct sIAP {                  // IAP Structure
    unsigned long cmd;         // Command
    unsigned long par[4];      // Parameters
    unsigned long stat;        // Status
    unsigned long res[2];      // Result
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
    unsigned long n;

    n = adr >> 12;                               //  4kB Sector
    if (n >= 0x10) {
        n = 0x0E + (n >> 3);                     // 32kB Sector
    }

    return (n);                                  // Sector Number
}

#define USE_SPIFI
#ifdef USE_SPIFI

typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

/* Include SPIFI ROM headers */
#include "spifi_rom_api.h"
#define SPIFIROMD_PRESENT
#include "rom_drivers.h"

#define ROM_DRIVERS_PTR ((ROM *)(*((unsigned int *)0x1FFF1FF8)))

/* Peripheral Clock Selection Register */
#define PCLKSEL      (*((volatile unsigned long *) 0x400FC1A8))

/* SPIFI Clock Selection Register */
#define SPIFICLKSEL  (*((volatile unsigned long *) 0x400FC1B4))

/* Power Control for Peripherals */
#define PCONP        (*((volatile unsigned long *) 0x400FC0C4))

static const SPIFI_RTNS *spifi;
static SPIFIobj obj;
static SPIFIopers opers;

/*
 *  Configure the pin
 *    Parameter:      portnum:   Port Number value, should be in range from 0..3.
 *                    pinnum:    Pin number value, should be in range from 0..31
 *                    funcnum:   Function number, should be range: 0..7
 *    Return Value:   0 - OK,  1 - Failed
 */
static void PINSEL_ConfigPin( uint8_t portnum, uint8_t pinnum, uint8_t funcnum)
{
	  volatile uint32_t *pPIN = (volatile uint32_t*)(0x4002C000 + ((portnum * 32 + pinnum)*sizeof(uint32_t)));
	  *pPIN &= ~0x00000007;//Clear function bits
	  *pPIN |= funcnum;
}

#define LPC_GPIO0_DIR      (*((volatile unsigned long *) 0x20098000))
#define LPC_GPIO0_SET      (*((volatile unsigned long *) 0x20098018))
#define LPC_GPIO0_CLR      (*((volatile unsigned long *) 0x2009801C))
#define LPC_GPIO1_DIR      (*((volatile unsigned long *) 0x20098020))
#define LPC_GPIO1_SET      (*((volatile unsigned long *) 0x20098038))
#define LPC_GPIO1_CLR      (*((volatile unsigned long *) 0x2009803C))
#define LPC_GPIO2_DIR      (*((volatile unsigned long *) 0x20098040))
#define LPC_GPIO2_SET      (*((volatile unsigned long *) 0x20098058))
#define LPC_GPIO2_CLR      (*((volatile unsigned long *) 0x2009805C))

#define LED1_OFF  LPC_GPIO1_SET = (1<<18)
#define LED1_ON   LPC_GPIO1_CLR = (1<<18)
#define LED2_OFF  LPC_GPIO0_SET = (1<<13)
#define LED2_ON   LPC_GPIO0_CLR = (1<<13)
#define LED3_OFF  LPC_GPIO1_CLR = (1<<13)
#define LED3_ON   LPC_GPIO1_SET = (1<<13)
#define LED4_OFF  LPC_GPIO2_CLR = (1<<19)
#define LED4_ON   LPC_GPIO2_SET = (1<<19)

#define LED_INIT  do { \
   PINSEL_ConfigPin(1, 18, 0); \
   PINSEL_ConfigPin(0, 13, 0); \
   PINSEL_ConfigPin(1, 13, 0); \
   PINSEL_ConfigPin(2, 19, 0); \
   LPC_GPIO0_DIR |= (1<<13); \
   LPC_GPIO1_DIR |= (1<<13) | (1<<18); \
   LPC_GPIO2_DIR |= (1<<19); \
} while(0)

#define QSPI_FLASH_ERASE_BLOCK_SIZE  4096

/*
 *  Program Page in SPIFI Memory. The adr parameter should be offset from
 *  the start of the SPIFI memory (0x28000000), i.e. the first write should
 *  have adr=0.
 *  SPIFI is programmed in blocks of 256bytes. SPIFI does not have to be
 *  erased in advace as all programming calls are made with a scratch buffer
 *  allowing the SPIFI ROM driver to erase as needed. The scratch buffer must
 *  be large enough to hold an entire erase sector.
 *
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
static int saveInSpifi(uint32_t adr, uint32_t sz, uint8_t* buf)
{
    uint32_t left = sz;
    uint32_t off = 0;
    int rc;

    static int led1toggle = 1;
    led1toggle++;
    if (led1toggle & 1) {
        LED1_ON;
    } else {
        LED1_OFF;
    }

    if ((adr % QSPI_FLASH_ERASE_BLOCK_SIZE) == 0) {
        /* Want to write at the start of an erase block.
           To speed up things, erase the entire block now. */

        opers.options = S_NO_VERIFY;
        opers.length = QSPI_FLASH_ERASE_BLOCK_SIZE;
        opers.dest = (char*)adr;
        opers.scratch = 0;
        opers.protect = 0;
        rc = spifi->spifi_erase(&obj, &opers);
        if (rc) {
            return 1;
        }
    }

    /* Now that we know that the SPIFI is erased for all offsets
       inside this erase block, it is save to set S_CALLER_ERASE
       i.e. tell spifi_program() that erasing is not needed.
       Without erasing there is no need for a scratch buffer. */
    opers.options = S_VERIFY_PROG | S_CALLER_ERASE;
    opers.scratch = 0;
    opers.protect = 0;

    while (left > 0) {
        uint32_t chunk = (left > 256) ? 256 : left;

        opers.length = chunk;
        opers.dest = (char *)(adr + off);
        rc = spifi->spifi_program(&obj, (char*)(buf+off), &opers);
        if (rc) {
            return 1;
        }
        off += chunk;
        left -= chunk;
    }
    return (0);
}
#endif


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc)
{
    _CCLK     = 12000;                // 12MHz Internal RC Oscillator

    PLL0CON  = 0x00;                  // Disable PLL (use Oscillator)
    PLL0FEED = 0xAA;                  // Feed Sequence Part #1
    PLL0FEED = 0x55;                  // Feed Sequence Part #2

    MEMMAP   = 0x01;                  // User Flash Mode

#ifdef USE_SPIFI
    LED_INIT;
    LED1_ON;
    LED2_OFF;

    PCONP |= (1<<16);

    SPIFICLKSEL = 0x00000002;    /* SPIFI Clock Selection */

    PINSEL_ConfigPin(2,  7, 5); /* SPIFI_CSN @ P2.7 */
    PINSEL_ConfigPin(0, 22, 5); /* SPIFI_CLK @ P0.22 */
    PINSEL_ConfigPin(0, 15, 5); /* SPIFI_IO2 @ P0.15 */
    PINSEL_ConfigPin(0, 16, 5); /* SPIFI_IO3 @ P0.16 */
    PINSEL_ConfigPin(0, 17, 5); /* SPIFI_IO1 @ P0.17 */
    PINSEL_ConfigPin(0, 18, 5); /* SPIFI_IO0 @ P0.18 */

    /* Get SPIFI API table pointer */
    spifi = ROM_DRIVERS_PTR->pSPIFID;

    {
        uint32_t spifi_clk_mhz = (_CCLK/1000)/(SPIFICLKSEL & 0x1f);
        int rc;
        /* Typical time tCS is 20 ns min, we give 200 ns to be on safer side */
        rc = spifi->spifi_init (&obj, spifi_clk_mhz/5, S_FULLCLK+S_RCVCLK, spifi_clk_mhz);
        if (rc) {
            /* Error while initializing SPIFI. How to handle? */
            LED1_OFF;
            LED2_ON;
            return (1);
        }
    }

    LED2_ON;
#endif

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
int EraseChip (void) {

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

#ifdef USE_SPIFI
    if (adr >= 0x28000000) {
        /* Address is in the SPIFI address space. SPIFI is only erased when needed
           so this call is ignored. */
        return (0);
    } else if (adr >= 0x80000) {
        /* This happens when a combined binary is flashed. The combined binary starts
           with 512kB that goes into the internal flash. After the 512kB comes the
           data to be written at the start of the SPIFI. SPIFI is erased on a
           need-to basis and not here. */
        return (0);
    }
#endif

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

#ifdef USE_SPIFI
    static int led2toggle = 0;
    led2toggle++;
    if (led2toggle & 1) {
        LED2_ON;
    } else {
        LED2_OFF;
    }
    if (adr >= 0x28000000) {
        /* Address is in the SPIFI address space. SPIFI is only erased when needed
           so this call is ignored. */
        return saveInSpifi(adr - 0x28000000, sz, buf);
    } else if (adr >= 0x80000) {
        /* This happens when a combined binary is flashed. The combined binary starts
           with 512kB that goes into the internal flash. After the 512kB comes the
           data to be written at the start of the SPIFI. SPIFI is erased on a
           need-to basis and not here. */
        return saveInSpifi(adr - 0x80000, sz, buf);
    }
#endif

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
    IAP.par[2] = 512;                            // Fixed Page Size
    IAP.par[3] = _CCLK;                          // CCLK in kHz
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    if (IAP.stat) {                              // Command Failed
        return (1);
    }

    return (0);                                  // Finished without Errors
}
