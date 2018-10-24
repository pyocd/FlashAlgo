/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
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
#include "fsl_spifi.h"
#include "string.h"

#define PAGE_SIZE       (256)

#define COMMAND_NUM     (7)
#define READ            (0)
#define PROGRAM_PAGE    (1)
#define GET_STATUS      (2)
#define ERASE_SECTOR    (3)
#define WRITE_ENABLE    (4)
#define WRITE_REGISTER  (5)
#define ERASE_CHIP      (6)

#define QUAD_MODE_VAL   0x02

#define SPI_BAUDRATE    24000000

#define SPIFI_PORT      0    /* SPIFI pins are connected to Port 0 */
#define SPIFI_IO0       24   /* SPIFI IO0 pin */
#define SPIFI_IO1       25   /* SPIFI IO1 pin */
#define SPIFI_IO2       28   /* SPIFI IO2 pin */
#define SPIFI_IO3       27   /* SPIFI IO3 pin */
#define SPIFI_CS        23   /* SPIFI CS pin */
#define SPIFI_CLK       26   /* SPIFI Clock pin */


#define IOCON_PIO_FUNC6             0x06u   /* Selects pin function 6 */
#define IOCON_PIO_MODE_PULLUP       0x20u   /* Selects pull-up function */
#define IOCON_PIO_INV_DI            0x00u   /* Input function is not inverted */
#define IOCON_PIO_DIGITAL_EN        0x0100u /* Enables digital function */
#define IOCON_PIO_INPFILT_OFF       0x0200u /* Input filter disabled */
#define IOCON_PIO_SLEW_STANDARD     0x00u
#define IOCON_PIO_OPENDRAIN_DI      0x00u   /* Open drain is disabled */


spifi_command_t command[COMMAND_NUM] = {
    {PAGE_SIZE, false, kSPIFI_DataInput, 1, kSPIFI_CommandDataQuad, kSPIFI_CommandOpcodeAddrThreeBytes, 0x6B},
    {PAGE_SIZE, false, kSPIFI_DataOutput, 0, kSPIFI_CommandDataQuad, kSPIFI_CommandOpcodeAddrThreeBytes, 0x32},
    {1, false, kSPIFI_DataInput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x05},
    {0, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeAddrThreeBytes, 0x20},
    {0, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x06},
    {1, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0x31},
    {0, false, kSPIFI_DataOutput, 0, kSPIFI_CommandAllSerial, kSPIFI_CommandOpcodeOnly, 0xC7}
};

void check_if_finish()
{
    uint8_t val = 0;
    /* Check WIP bit */
    do
    {
        SPIFI_SetCommand(SPIFI0, &command[GET_STATUS]);
        while ((SPIFI0->STAT & SPIFI_STAT_INTRQ_MASK) == 0U)
        {
        }
        val = SPIFI_ReadDataByte(SPIFI0);
    } while (val & 0x1);
}

void enable_quad_mode()
{
    /* Write enable */
    SPIFI_SetCommand(SPIFI0, &command[WRITE_ENABLE]);

    /* Set write register command */
    SPIFI_SetCommand(SPIFI0, &command[WRITE_REGISTER]);

    SPIFI_WriteDataByte(SPIFI0, QUAD_MODE_VAL);

    check_if_finish();
}

/* Get HF FRO Clk */
/*! brief	Return Frequency of High-Freq output of FRO
 *  return	Frequency of High-Freq output of FRO
 */
uint32_t CLOCK_GetFroHfFreq(void)
{
    if ((SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_FRO_MASK) || (!(SYSCON->FROCTRL & SYSCON_FROCTRL_HSPDCLK_MASK)))
    {
        return 0U;
    }

    if(SYSCON->FROCTRL & SYSCON_FROCTRL_SEL_MASK)
    {
        return 96000000U;
    }
    else
    {
        return 48000000U;
    }
}

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    uint32_t sourceClockFreq;
    spifi_config_t config = {0};
    uint32_t pinconfig = (IOCON_PIO_FUNC6 | IOCON_PIO_MODE_PULLUP | IOCON_PIO_INV_DI |
                          IOCON_PIO_DIGITAL_EN | IOCON_PIO_INPFILT_OFF | IOCON_PIO_OPENDRAIN_DI);

    /* Enables the clock for the IOCON block */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Setup the Pin function for SPIFI */
    IOCON->PIO[SPIFI_PORT][SPIFI_IO0] = pinconfig;
    IOCON->PIO[SPIFI_PORT][SPIFI_IO1] = pinconfig;
    IOCON->PIO[SPIFI_PORT][SPIFI_IO2] = pinconfig;
    IOCON->PIO[SPIFI_PORT][SPIFI_IO3] = pinconfig;
    IOCON->PIO[SPIFI_PORT][SPIFI_CS] = pinconfig;
    IOCON->PIO[SPIFI_PORT][SPIFI_CLK] = pinconfig;

    /* After reset the CPU clock is 48 MHz based on the 96 MHz FRO. */
    /* Set SPIFI clock source */
    SYSCON->SPIFICLKSEL = 3,

    sourceClockFreq = CLOCK_GetFroHfFreq();

    /* Set the clock divider */
    SYSCON->SPIFICLKDIV = ((sourceClockFreq / SPI_BAUDRATE) - 1);

    /* Initialize SPIFI */
    SPIFI_GetDefaultConfig(&config);
    SPIFI_Init(SPIFI0, &config);

    enable_quad_mode();

    return (0);
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t UnInit(uint32_t fnc)
{
    return (0);
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseChip(void)
{
    /* Reset the SPIFI to switch to command mode */
    SPIFI_ResetCommand(SPIFI0);

    /* Write enable */
    SPIFI_SetCommand(SPIFI0, &command[WRITE_ENABLE]);

    SPIFI_SetCommandAddress(SPIFI0, 0);

    /* Erase chip, takes about 50s for erase chip operation */
    SPIFI_SetCommand(SPIFI0, &command[ERASE_CHIP]);

    /* Check if finished */
    check_if_finish();

    return 0;
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSector(uint32_t adr)
{
    /* Reset the SPIFI to switch to command mode */
    SPIFI_ResetCommand(SPIFI0);

    /* Write enable */
    SPIFI_SetCommand(SPIFI0, &command[WRITE_ENABLE]);
    /* Set address */
    SPIFI_SetCommandAddress(SPIFI0, (adr - FSL_FEATURE_SPIFI_START_ADDR));
    /* Erase sector */
    SPIFI_SetCommand(SPIFI0, &command[ERASE_SECTOR]);
    /* Check if finished */
    check_if_finish();

    return 0;
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t i = 0;

    /* Reset the SPIFI to switch to command mode */
    SPIFI_ResetCommand(SPIFI0);

    SPIFI_SetCommand(SPIFI0, &command[WRITE_ENABLE]);
    SPIFI_SetCommandAddress(SPIFI0, (adr - FSL_FEATURE_SPIFI_START_ADDR));
    SPIFI_SetCommand(SPIFI0, &command[PROGRAM_PAGE]);

    for (i = 0; i < PAGE_SIZE; i += 4, buf++)
    {
        SPIFI_WriteData(SPIFI0, *buf);
    }

    check_if_finish();

    return 0;
}
