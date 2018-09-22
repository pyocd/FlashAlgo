/*
** ###################################################################
**     Processors:          MK80FN256CAx15
**                          MK80FN256VDC15
**                          MK80FN256VLL15
**                          MK80FN256VLQ15
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K80P121M150SF5RM, Rev. 2, May 2015
**     Version:             rev. 2.2, 2015-07-29
**     Build:               b151218
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MK80F25615
**
**     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2014-07-30)
**         Initial version
**     - rev. 1.1 (2014-08-28)
**         Update of startup files - possibility to override DefaultISR added.
**     - rev. 1.2 (2014-11-07)
**         Update according to the new version of reference manual Rev. 1 Draft A.
**     - rev. 2.0 (2015-04-01)
**         Update according to the new version of reference manual Rev. 1.
**     - rev. 2.1 (2015-05-28)
**         Update according to the reference manual Rev. 2.
**     - rev. 2.2 (2015-07-29)
**         Correction of backward compatibility.
**
** ###################################################################
*/

/*!
 * @file MK80F25615.h
 * @version 2.2
 * @date 2015-07-29
 * @brief CMSIS Peripheral Access Layer for MK80F25615
 *
 * CMSIS Peripheral Access Layer for MK80F25615
 */

#ifndef _MK80F25615_H_
#define _MK80F25615_H_                           /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0200U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0002U

/**
 * @brief Macro to calculate address of an aliased word in the peripheral
 *        bitband area for a peripheral register and bit (bit band region 0x40000000 to
 *        0x400FFFFF).
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return  Address of the aliased word in the peripheral bitband area.
 */
#define BITBAND_REGADDR(Reg,Bit) (0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 32bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG32(Reg,Bit) (*((uint32_t volatile*)(BITBAND_REGADDR((Reg),(Bit)))))
#define BITBAND_REG(Reg,Bit) (BITBAND_REG32((Reg),(Bit)))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 16bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG16(Reg,Bit) (*((uint16_t volatile*)(BITBAND_REGADDR((Reg),(Bit)))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 8bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG8(Reg,Bit) (*((uint8_t volatile*)(BITBAND_REGADDR((Reg),(Bit)))))

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 123                /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_DMA16_IRQn              = 0,                /**< DMA channel 0,16 transfer complete */
  DMA1_DMA17_IRQn              = 1,                /**< DMA channel 1,17 transfer complete */
  DMA2_DMA18_IRQn              = 2,                /**< DMA channel 2,18 transfer complete */
  DMA3_DMA19_IRQn              = 3,                /**< DMA channel 3,19 transfer complete */
  DMA4_DMA20_IRQn              = 4,                /**< DMA channel 4,20 transfer complete */
  DMA5_DMA21_IRQn              = 5,                /**< DMA channel 5,21 transfer complete */
  DMA6_DMA22_IRQn              = 6,                /**< DMA channel 6,22 transfer complete */
  DMA7_DMA23_IRQn              = 7,                /**< DMA channel 7,23 transfer complete */
  DMA8_DMA24_IRQn              = 8,                /**< DMA channel 8,24 transfer complete */
  DMA9_DMA25_IRQn              = 9,                /**< DMA channel 9,25 transfer complete */
  DMA10_DMA26_IRQn             = 10,               /**< DMA channel 10,26 transfer complete */
  DMA11_DMA27_IRQn             = 11,               /**< DMA channel 11,27 transfer complete */
  DMA12_DMA28_IRQn             = 12,               /**< DMA channel 12,28 transfer complete */
  DMA13_DMA29_IRQn             = 13,               /**< DMA channel 13,29 transfer complete */
  DMA14_DMA30_IRQn             = 14,               /**< DMA channel 14,30 transfer complete */
  DMA15_DMA31_IRQn             = 15,               /**< DMA channel 15,31 transfer complete */
  DMA_Error_IRQn               = 16,               /**< DMA channel 0 - 31 error */
  MCM_IRQn                     = 17,               /**< MCM normal interrupt */
  FTFA_IRQn                    = 18,               /**< FTFA command complete */
  Read_Collision_IRQn          = 19,               /**< FTFA read collision */
  LVD_LVW_IRQn                 = 20,               /**< PMC controller low-voltage detect, low-voltage warning */
  LLWU_IRQn                    = 21,               /**< Low leakage wakeup unit */
  WDOG_EWM_IRQn                = 22,               /**< Single interrupt vector for  WDOG and EWM */
  TRNG0_IRQn                   = 23,               /**< True randon number generator */
  I2C0_IRQn                    = 24,               /**< Inter-integrated circuit 0 */
  I2C1_IRQn                    = 25,               /**< Inter-integrated circuit 1 */
  SPI0_IRQn                    = 26,               /**< Serial peripheral Interface 0 */
  SPI1_IRQn                    = 27,               /**< Serial peripheral Interface 1 */
  I2S0_Tx_IRQn                 = 28,               /**< Integrated interchip sound 0 transmit interrupt */
  I2S0_Rx_IRQn                 = 29,               /**< Integrated interchip sound 0 receive interrupt */
  LPUART0_IRQn                 = 30,               /**< LPUART0 receive/transmit/error interrupt */
  LPUART1_IRQn                 = 31,               /**< LPUART1 receive/transmit/error interrupt */
  LPUART2_IRQn                 = 32,               /**< LPUART2 receive/transmit/error interrupt */
  LPUART3_IRQn                 = 33,               /**< LPUART3 receive/transmit/error interrupt */
  LPUART4_IRQn                 = 34,               /**< LPUART4 receive/transmit/error interrupt */
  Reserved51_IRQn              = 35,               /**< Reserved interrupt */
  Reserved52_IRQn              = 36,               /**< Reserved interrupt */
  EMVSIM0_IRQn                 = 37,               /**< EMVSIM0 common interrupt */
  EMVSIM1_IRQn                 = 38,               /**< EMVSIM1 common interrupt */
  ADC0_IRQn                    = 39,               /**< Analog-to-digital converter 0 */
  CMP0_IRQn                    = 40,               /**< Comparator 0 */
  CMP1_IRQn                    = 41,               /**< Comparator 1 */
  FTM0_IRQn                    = 42,               /**< FlexTimer module 0 fault, overflow and channels interrupt */
  FTM1_IRQn                    = 43,               /**< FlexTimer module 1 fault, overflow and channels interrupt */
  FTM2_IRQn                    = 44,               /**< FlexTimer module 2 fault, overflow and channels interrupt */
  CMT_IRQn                     = 45,               /**< Carrier modulator transmitter */
  RTC_IRQn                     = 46,               /**< Real time clock */
  RTC_Seconds_IRQn             = 47,               /**< Real time clock seconds */
  PIT0CH0_IRQn                 = 48,               /**< Periodic interrupt timer 0 channel 0 */
  PIT0CH1_IRQn                 = 49,               /**< Periodic interrupt timer 0 channel 1 */
  PIT0CH2_IRQn                 = 50,               /**< Periodic interrupt timer 0 channel 2 */
  PIT0CH3_IRQn                 = 51,               /**< Periodic interrupt timer 0 channel 3 */
  PDB0_IRQn                    = 52,               /**< Programmable delay block */
  USB0_IRQn                    = 53,               /**< USB OTG interrupt */
  USBDCD_IRQn                  = 54,               /**< USB charger detect */
  Reserved71_IRQn              = 55,               /**< Reserved interrupt */
  DAC0_IRQn                    = 56,               /**< Digital-to-analog converter 0 */
  MCG_IRQn                     = 57,               /**< Multipurpose clock generator */
  LPTMR0_LPTMR1_IRQn           = 58,               /**< Single interrupt vector for  Low Power Timer 0 and 1 */
  PORTA_IRQn                   = 59,               /**< Port A pin detect interrupt */
  PORTB_IRQn                   = 60,               /**< Port B pin detect interrupt */
  PORTC_IRQn                   = 61,               /**< Port C pin detect interrupt */
  PORTD_IRQn                   = 62,               /**< Port D pin detect interrupt */
  PORTE_IRQn                   = 63,               /**< Port E pin detect interrupt */
  SWI_IRQn                     = 64,               /**< Software interrupt */
  SPI2_IRQn                    = 65,               /**< Serial peripheral Interface 2 */
  Reserved82_IRQn              = 66,               /**< Reserved interrupt */
  Reserved83_IRQn              = 67,               /**< Reserved interrupt */
  Reserved84_IRQn              = 68,               /**< Reserved interrupt */
  Reserved85_IRQn              = 69,               /**< Reserved interrupt */
  FLEXIO0_IRQn                 = 70,               /**< FLEXIO0 */
  FTM3_IRQn                    = 71,               /**< FlexTimer module 3 fault, overflow and channels interrupt */
  Reserved88_IRQn              = 72,               /**< Reserved interrupt */
  Reserved89_IRQn              = 73,               /**< Reserved interrupt */
  I2C2_IRQn                    = 74,               /**< Inter-integrated circuit 2 */
  Reserved91_IRQn              = 75,               /**< Reserved interrupt */
  Reserved92_IRQn              = 76,               /**< Reserved interrupt */
  Reserved93_IRQn              = 77,               /**< Reserved interrupt */
  Reserved94_IRQn              = 78,               /**< Reserved interrupt */
  Reserved95_IRQn              = 79,               /**< Reserved interrupt */
  Reserved96_IRQn              = 80,               /**< Reserved interrupt */
  SDHC_IRQn                    = 81,               /**< Secured digital host controller */
  Reserved98_IRQn              = 82,               /**< Reserved interrupt */
  Reserved99_IRQn              = 83,               /**< Reserved interrupt */
  Reserved100_IRQn             = 84,               /**< Reserved interrupt */
  Reserved101_IRQn             = 85,               /**< Reserved interrupt */
  Reserved102_IRQn             = 86,               /**< Reserved interrupt */
  TSI0_IRQn                    = 87,               /**< Touch Sensing Input */
  TPM1_IRQn                    = 88,               /**< TPM1 single interrupt vector for all sources */
  TPM2_IRQn                    = 89,               /**< TPM2 single interrupt vector for all sources */
  Reserved106_IRQn             = 90,               /**< Reserved interrupt */
  I2C3_IRQn                    = 91,               /**< Inter-integrated circuit 3 */
  Reserved108_IRQn             = 92,               /**< Reserved interrupt */
  Reserved109_IRQn             = 93,               /**< Reserved interrupt */
  Reserved110_IRQn             = 94,               /**< Reserved interrupt */
  Reserved111_IRQn             = 95,               /**< Reserved interrupt */
  Reserved112_IRQn             = 96,               /**< Reserved interrupt */
  Reserved113_IRQn             = 97,               /**< Reserved interrupt */
  Reserved114_IRQn             = 98,               /**< Reserved interrupt */
  Reserved115_IRQn             = 99,               /**< Reserved interrupt */
  QuadSPI0_IRQn                = 100,              /**< qspi */
  Reserved117_IRQn             = 101,              /**< Reserved interrupt */
  Reserved118_IRQn             = 102,              /**< Reserved interrupt */
  Reserved119_IRQn             = 103,              /**< Reserved interrupt */
  Reserved120_IRQn             = 104,              /**< Reserved interrupt */
  Reserved121_IRQn             = 105,              /**< Reserved interrupt */
  Reserved122_IRQn             = 106               /**< Reserved interrupt */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M4 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M4 Core Configuration
 * @{
 */

#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __NVIC_PRIO_BITS               4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT                  1         /**< Defines if an FPU is present or not */

#include "core_cm4.h"                  /* Core Peripheral Access Layer */
#include "system_MK80F25615.h"         /* Device specific configuration file */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */


/* ----------------------------------------------------------------------------
   -- Mapping Information
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Mapping_Information Mapping Information
 * @{
 */

/** Mapping Information */
/*!
 * @addtogroup edma_request
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Structure for the DMA hardware request
 *
 * Defines the structure for the DMA hardware request collections. The user can configure the
 * hardware request into DMAMUX to trigger the DMA transfer accordingly. The index
 * of the hardware request varies according  to the to SoC.
 */
typedef enum _dma_request_source
{
    kDmaRequestMux0Disable          = 0|0x100U,    /**< DMAMUX TriggerDisabled. */
    kDmaRequestMux0TSI0             = 1|0x100U,    /**< TSI0. */
    kDmaRequestMux0LPUART0Rx        = 2|0x100U,    /**< LPUART0 Receive. */
    kDmaRequestMux0LPUART0Tx        = 3|0x100U,    /**< LPUART0 Transmit. */
    kDmaRequestMux0LPUART1Rx        = 4|0x100U,    /**< LPUART1 Receive. */
    kDmaRequestMux0LPUART1Tx        = 5|0x100U,    /**< LPUART1 Transmit. */
    kDmaRequestMux0LPUART2Rx        = 6|0x100U,    /**< LPUART2 Receive. */
    kDmaRequestMux0LPUART2Tx        = 7|0x100U,    /**< LPUART2 Transmit. */
    kDmaRequestMux0LPUART3Rx        = 8|0x100U,    /**< LPUART3 Receive. */
    kDmaRequestMux0LPUART3Tx        = 9|0x100U,    /**< LPUART3 Transmit. */
    kDmaRequestMux0LPUART4Rx        = 10|0x100U,   /**< LPUART4 Receive. */
    kDmaRequestMux0LPUART4Tx        = 11|0x100U,   /**< LPUART4 Transmit. */
    kDmaRequestMux0I2S0Rx           = 12|0x100U,   /**< I2S0 Receive. */
    kDmaRequestMux0I2S0Tx           = 13|0x100U,   /**< I2S0 Transmit. */
    kDmaRequestMux0SPI0Rx           = 14|0x100U,   /**< SPI0 Receive. */
    kDmaRequestMux0SPI0Tx           = 15|0x100U,   /**< SPI0 Transmit. */
    kDmaRequestMux0SPI1Rx           = 16|0x100U,   /**< SPI1 Receive. */
    kDmaRequestMux0SPI1Tx           = 17|0x100U,   /**< SPI1 Transmit. */
    kDmaRequestMux0I2C0I2C3         = 18|0x100U,   /**< I2C0 and I2C3. */
    kDmaRequestMux0I2C0             = 18|0x100U,   /**< I2C0 and I2C3. */
    kDmaRequestMux0I2C3             = 18|0x100U,   /**< I2C0 and I2C3. */
    kDmaRequestMux0I2C1I2C2         = 19|0x100U,   /**< I2C1 and I2C2. */
    kDmaRequestMux0I2C1             = 19|0x100U,   /**< I2C1 and I2C2. */
    kDmaRequestMux0I2C2             = 19|0x100U,   /**< I2C1 and I2C2. */
    kDmaRequestMux0FTM0Channel0     = 20|0x100U,   /**< FTM0 C0V. */
    kDmaRequestMux0FTM0Channel1     = 21|0x100U,   /**< FTM0 C1V. */
    kDmaRequestMux0FTM0Channel2     = 22|0x100U,   /**< FTM0 C2V. */
    kDmaRequestMux0FTM0Channel3     = 23|0x100U,   /**< FTM0 C3V. */
    kDmaRequestMux0FTM0Channel4     = 24|0x100U,   /**< FTM0 C4V. */
    kDmaRequestMux0FTM0Channel5     = 25|0x100U,   /**< FTM0 C5V. */
    kDmaRequestMux0FTM0Channel6     = 26|0x100U,   /**< FTM0 C6V. */
    kDmaRequestMux0FTM0Channel7     = 27|0x100U,   /**< FTM0 C7V. */
    kDmaRequestMux0FTM1Channel0     = 28|0x100U,   /**< FTM1 C0V. */
    kDmaRequestMux0FTM1Channel1     = 29|0x100U,   /**< FTM1 C1V. */
    kDmaRequestMux0FTM2Channel0     = 30|0x100U,   /**< FTM2 C0V. */
    kDmaRequestMux0FTM2Channel1     = 31|0x100U,   /**< FTM2 C1V. */
    kDmaRequestMux0FTM3Channel0     = 32|0x100U,   /**< FTM3 C0V. */
    kDmaRequestMux0FTM3Channel1     = 33|0x100U,   /**< FTM3 C1V. */
    kDmaRequestMux0FTM3Channel2     = 34|0x100U,   /**< FTM3 C2V. */
    kDmaRequestMux0FTM3Channel3     = 35|0x100U,   /**< FTM3 C3V. */
    kDmaRequestMux0FTM3Channel4     = 36|0x100U,   /**< FTM3 C4V. */
    kDmaRequestMux0FTM3Channel5     = 37|0x100U,   /**< FTM3 C5V. */
    kDmaRequestMux0FTM3Channel6     = 38|0x100U,   /**< FTM3 C6V. */
    kDmaRequestMux0FTM3Channel7     = 39|0x100U,   /**< FTM3 C7V. */
    kDmaRequestMux0ADC0             = 40|0x100U,   /**< ADC0. */
    kDmaRequestMux0Reserved41       = 41|0x100U,   /**< Reserved41 */
    kDmaRequestMux0CMP0             = 42|0x100U,   /**< CMP0. */
    kDmaRequestMux0CMP1             = 43|0x100U,   /**< CMP1. */
    kDmaRequestMux0Reserved44       = 44|0x100U,   /**< Reserved44 */
    kDmaRequestMux0DAC0             = 45|0x100U,   /**< DAC0. */
    kDmaRequestMux0Reserved46       = 46|0x100U,   /**< Reserved46 */
    kDmaRequestMux0CMT              = 47|0x100U,   /**< CMT. */
    kDmaRequestMux0PDB              = 48|0x100U,   /**< PDB0. */
    kDmaRequestMux0PortA            = 49|0x100U,   /**< PTA. */
    kDmaRequestMux0PortB            = 50|0x100U,   /**< PTB. */
    kDmaRequestMux0PortC            = 51|0x100U,   /**< PTC. */
    kDmaRequestMux0PortD            = 52|0x100U,   /**< PTD. */
    kDmaRequestMux0PortE            = 53|0x100U,   /**< PTE. */
    kDmaRequestMux0Reserved54       = 54|0x100U,   /**< Reserved54 */
    kDmaRequestMux0Reserved55       = 55|0x100U,   /**< Reserved55 */
    kDmaRequestMux0Reserved56       = 56|0x100U,   /**< Reserved56 */
    kDmaRequestMux0Reserved57       = 57|0x100U,   /**< Reserved57 */
    kDmaRequestMux0SPI2Rx           = 58|0x100U,   /**< SPI2 Receive. */
    kDmaRequestMux0SPI2Tx           = 59|0x100U,   /**< SPI2 Transmit. */
    kDmaRequestMux0AlwaysOn60       = 60|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn61       = 61|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn62       = 62|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn63       = 63|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0Group1Disable    = 0|0x200U,    /**< DMAMUX TriggerDisabled. */
    kDmaRequestMux0Group1FlexIO0Channel0 = 1|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel1 = 2|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel2 = 3|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel3 = 4|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel4 = 5|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel5 = 6|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel6 = 7|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1FlexIO0Channel7 = 8|0x200U, /**< FLEXIO0. */
    kDmaRequestMux0Group1Reserved9  = 9|0x200U,    /**< Reserved9 */
    kDmaRequestMux0Group1Reserved10 = 10|0x200U,   /**< Reserved10 */
    kDmaRequestMux0Group1Reserved11 = 11|0x200U,   /**< Reserved11 */
    kDmaRequestMux0Group1Reserved12 = 12|0x200U,   /**< Reserved12 */
    kDmaRequestMux0Group1Reserved13 = 13|0x200U,   /**< Reserved13 */
    kDmaRequestMux0Group1Reserved14 = 14|0x200U,   /**< Reserved14 */
    kDmaRequestMux0Group1Reserved15 = 15|0x200U,   /**< Reserved15 */
    kDmaRequestMux0Group1Reserved16 = 16|0x200U,   /**< Reserved16 */
    kDmaRequestMux0Group1Reserved17 = 17|0x200U,   /**< Reserved17 */
    kDmaRequestMux0Group1Reserved18 = 18|0x200U,   /**< Reserved18 */
    kDmaRequestMux0Group1Reserved19 = 19|0x200U,   /**< Reserved19 */
    kDmaRequestMux0Group1EMVSIM0Rx  = 20|0x200U,   /**< EMVSIM0 Receive. */
    kDmaRequestMux0Group1EMVSIM0Tx  = 21|0x200U,   /**< EMVSIM0 Transmit. */
    kDmaRequestMux0Group1EMVSIM1Rx  = 22|0x200U,   /**< EMVSIM1 Receive. */
    kDmaRequestMux0Group1EMVSIM1Tx  = 23|0x200U,   /**< EMVSIM1 Transmit. */
    kDmaRequestMux0Group1QSPI0Rx    = 24|0x200U,   /**< QuadSPI0 Receive. */
    kDmaRequestMux0Group1QSPI0Tx    = 25|0x200U,   /**< QuadSPI0 Transmit. */
    kDmaRequestMux0Group1Reserved26 = 26|0x200U,   /**< Reserved26 */
    kDmaRequestMux0Group1Reserved27 = 27|0x200U,   /**< Reserved27 */
    kDmaRequestMux0Group1SPI0Rx     = 28|0x200U,   /**< SPI0 Receive. */
    kDmaRequestMux0Group1SPI0Tx     = 29|0x200U,   /**< SPI0 Transmit. */
    kDmaRequestMux0Group1SPI1Rx     = 30|0x200U,   /**< SPI1 Receive. */
    kDmaRequestMux0Group1SPI1Tx     = 31|0x200U,   /**< SPI1 Transmit. */
    kDmaRequestMux0Group1Reserved32 = 32|0x200U,   /**< Reserved32 */
    kDmaRequestMux0Group1Reserved33 = 33|0x200U,   /**< Reserved33 */
    kDmaRequestMux0Group1Reserved34 = 34|0x200U,   /**< Reserved34 */
    kDmaRequestMux0Group1Reserved35 = 35|0x200U,   /**< Reserved35 */
    kDmaRequestMux0Group1Reserved36 = 36|0x200U,   /**< Reserved36 */
    kDmaRequestMux0Group1Reserved37 = 37|0x200U,   /**< Reserved37 */
    kDmaRequestMux0Group1Reserved38 = 38|0x200U,   /**< Reserved38 */
    kDmaRequestMux0Group1Reserved39 = 39|0x200U,   /**< Reserved39 */
    kDmaRequestMux0Group1Reserved40 = 40|0x200U,   /**< Reserved40 */
    kDmaRequestMux0Group1Reserved41 = 41|0x200U,   /**< Reserved41 */
    kDmaRequestMux0Group1TPM1Channel0 = 42|0x200U, /**< TPM1 C0V. */
    kDmaRequestMux0Group1TPM1Channel1 = 43|0x200U, /**< TPM1 C1V. */
    kDmaRequestMux0Group1TPM2Channel0 = 44|0x200U, /**< TPM2 C0V. */
    kDmaRequestMux0Group1TPM2Channel1 = 45|0x200U, /**< TPM2 C1V. */
    kDmaRequestMux0Group1Reserved46 = 46|0x200U,   /**< Reserved46 */
    kDmaRequestMux0Group1Reserved47 = 47|0x200U,   /**< Reserved47 */
    kDmaRequestMux0Group1Reserved48 = 48|0x200U,   /**< Reserved48 */
    kDmaRequestMux0Group1Reserved49 = 49|0x200U,   /**< Reserved49 */
    kDmaRequestMux0Group1Reserved50 = 50|0x200U,   /**< Reserved50 */
    kDmaRequestMux0Group1Reserved51 = 51|0x200U,   /**< Reserved51 */
    kDmaRequestMux0Group1Reserved52 = 52|0x200U,   /**< Reserved52 */
    kDmaRequestMux0Group1Reserved53 = 53|0x200U,   /**< Reserved53 */
    kDmaRequestMux0Group1Reserved54 = 54|0x200U,   /**< Reserved54 */
    kDmaRequestMux0Group1TPM1Overflow = 55|0x200U, /**< TPM1. */
    kDmaRequestMux0Group1TPM2Overflow = 56|0x200U, /**< TPM2. */
    kDmaRequestMux0Group1Reserved57 = 57|0x200U,   /**< Reserved57 */
    kDmaRequestMux0Group1Reserved58 = 58|0x200U,   /**< Reserved58 */
    kDmaRequestMux0Group1Reserved59 = 59|0x200U,   /**< Reserved59 */
    kDmaRequestMux0Group1AlwaysOn60 = 60|0x200U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0Group1AlwaysOn61 = 61|0x200U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0Group1AlwaysOn62 = 62|0x200U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0Group1AlwaysOn63 = 63|0x200U,   /**< DMAMUX Always Enabled slot. */
} dma_request_source_t;

/* @} */


/*!
 * @}
 */ /* end of group Mapping_Information */


/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC1[2];                            /**< ADC Status and Control Registers 1, array offset: 0x0, array step: 0x4 */
  __IO uint32_t CFG1;                              /**< ADC Configuration Register 1, offset: 0x8 */
  __IO uint32_t CFG2;                              /**< ADC Configuration Register 2, offset: 0xC */
  __I  uint32_t R[2];                              /**< ADC Data Result Register, array offset: 0x10, array step: 0x4 */
  __IO uint32_t CV1;                               /**< Compare Value Registers, offset: 0x18 */
  __IO uint32_t CV2;                               /**< Compare Value Registers, offset: 0x1C */
  __IO uint32_t SC2;                               /**< Status and Control Register 2, offset: 0x20 */
  __IO uint32_t SC3;                               /**< Status and Control Register 3, offset: 0x24 */
  __IO uint32_t OFS;                               /**< ADC Offset Correction Register, offset: 0x28 */
  __IO uint32_t PG;                                /**< ADC Plus-Side Gain Register, offset: 0x2C */
  __IO uint32_t MG;                                /**< ADC Minus-Side Gain Register, offset: 0x30 */
  __IO uint32_t CLPD;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x34 */
  __IO uint32_t CLPS;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x38 */
  __IO uint32_t CLP4;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x3C */
  __IO uint32_t CLP3;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x40 */
  __IO uint32_t CLP2;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x44 */
  __IO uint32_t CLP1;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x48 */
  __IO uint32_t CLP0;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x4C */
       uint8_t RESERVED_0[4];
  __IO uint32_t CLMD;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x54 */
  __IO uint32_t CLMS;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x58 */
  __IO uint32_t CLM4;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x5C */
  __IO uint32_t CLM3;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x60 */
  __IO uint32_t CLM2;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x64 */
  __IO uint32_t CLM1;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x68 */
  __IO uint32_t CLM0;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x6C */
} ADC_Type;

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name SC1 - ADC Status and Control Registers 1 */
#define ADC_SC1_ADCH_MASK                        (0x1FU)
#define ADC_SC1_ADCH_SHIFT                       (0U)
#define ADC_SC1_ADCH(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_ADCH_SHIFT)) & ADC_SC1_ADCH_MASK)
#define ADC_SC1_DIFF_MASK                        (0x20U)
#define ADC_SC1_DIFF_SHIFT                       (5U)
#define ADC_SC1_DIFF(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_DIFF_SHIFT)) & ADC_SC1_DIFF_MASK)
#define ADC_SC1_AIEN_MASK                        (0x40U)
#define ADC_SC1_AIEN_SHIFT                       (6U)
#define ADC_SC1_AIEN(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_AIEN_SHIFT)) & ADC_SC1_AIEN_MASK)
#define ADC_SC1_COCO_MASK                        (0x80U)
#define ADC_SC1_COCO_SHIFT                       (7U)
#define ADC_SC1_COCO(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_COCO_SHIFT)) & ADC_SC1_COCO_MASK)

/* The count of ADC_SC1 */
#define ADC_SC1_COUNT                            (2U)

/*! @name CFG1 - ADC Configuration Register 1 */
#define ADC_CFG1_ADICLK_MASK                     (0x3U)
#define ADC_CFG1_ADICLK_SHIFT                    (0U)
#define ADC_CFG1_ADICLK(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADICLK_SHIFT)) & ADC_CFG1_ADICLK_MASK)
#define ADC_CFG1_MODE_MASK                       (0xCU)
#define ADC_CFG1_MODE_SHIFT                      (2U)
#define ADC_CFG1_MODE(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_MODE_SHIFT)) & ADC_CFG1_MODE_MASK)
#define ADC_CFG1_ADLSMP_MASK                     (0x10U)
#define ADC_CFG1_ADLSMP_SHIFT                    (4U)
#define ADC_CFG1_ADLSMP(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADLSMP_SHIFT)) & ADC_CFG1_ADLSMP_MASK)
#define ADC_CFG1_ADIV_MASK                       (0x60U)
#define ADC_CFG1_ADIV_SHIFT                      (5U)
#define ADC_CFG1_ADIV(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADIV_SHIFT)) & ADC_CFG1_ADIV_MASK)
#define ADC_CFG1_ADLPC_MASK                      (0x80U)
#define ADC_CFG1_ADLPC_SHIFT                     (7U)
#define ADC_CFG1_ADLPC(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADLPC_SHIFT)) & ADC_CFG1_ADLPC_MASK)

/*! @name CFG2 - ADC Configuration Register 2 */
#define ADC_CFG2_ADLSTS_MASK                     (0x3U)
#define ADC_CFG2_ADLSTS_SHIFT                    (0U)
#define ADC_CFG2_ADLSTS(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_ADLSTS_SHIFT)) & ADC_CFG2_ADLSTS_MASK)
#define ADC_CFG2_ADHSC_MASK                      (0x4U)
#define ADC_CFG2_ADHSC_SHIFT                     (2U)
#define ADC_CFG2_ADHSC(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_ADHSC_SHIFT)) & ADC_CFG2_ADHSC_MASK)
#define ADC_CFG2_ADACKEN_MASK                    (0x8U)
#define ADC_CFG2_ADACKEN_SHIFT                   (3U)
#define ADC_CFG2_ADACKEN(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_ADACKEN_SHIFT)) & ADC_CFG2_ADACKEN_MASK)
#define ADC_CFG2_MUXSEL_MASK                     (0x10U)
#define ADC_CFG2_MUXSEL_SHIFT                    (4U)
#define ADC_CFG2_MUXSEL(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_MUXSEL_SHIFT)) & ADC_CFG2_MUXSEL_MASK)

/*! @name R - ADC Data Result Register */
#define ADC_R_D_MASK                             (0xFFFFU)
#define ADC_R_D_SHIFT                            (0U)
#define ADC_R_D(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_R_D_SHIFT)) & ADC_R_D_MASK)

/* The count of ADC_R */
#define ADC_R_COUNT                              (2U)

/*! @name CV1 - Compare Value Registers */
#define ADC_CV1_CV_MASK                          (0xFFFFU)
#define ADC_CV1_CV_SHIFT                         (0U)
#define ADC_CV1_CV(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_CV1_CV_SHIFT)) & ADC_CV1_CV_MASK)

/*! @name CV2 - Compare Value Registers */
#define ADC_CV2_CV_MASK                          (0xFFFFU)
#define ADC_CV2_CV_SHIFT                         (0U)
#define ADC_CV2_CV(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_CV2_CV_SHIFT)) & ADC_CV2_CV_MASK)

/*! @name SC2 - Status and Control Register 2 */
#define ADC_SC2_REFSEL_MASK                      (0x3U)
#define ADC_SC2_REFSEL_SHIFT                     (0U)
#define ADC_SC2_REFSEL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_SC2_REFSEL_SHIFT)) & ADC_SC2_REFSEL_MASK)
#define ADC_SC2_DMAEN_MASK                       (0x4U)
#define ADC_SC2_DMAEN_SHIFT                      (2U)
#define ADC_SC2_DMAEN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_DMAEN_SHIFT)) & ADC_SC2_DMAEN_MASK)
#define ADC_SC2_ACREN_MASK                       (0x8U)
#define ADC_SC2_ACREN_SHIFT                      (3U)
#define ADC_SC2_ACREN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACREN_SHIFT)) & ADC_SC2_ACREN_MASK)
#define ADC_SC2_ACFGT_MASK                       (0x10U)
#define ADC_SC2_ACFGT_SHIFT                      (4U)
#define ADC_SC2_ACFGT(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACFGT_SHIFT)) & ADC_SC2_ACFGT_MASK)
#define ADC_SC2_ACFE_MASK                        (0x20U)
#define ADC_SC2_ACFE_SHIFT                       (5U)
#define ADC_SC2_ACFE(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACFE_SHIFT)) & ADC_SC2_ACFE_MASK)
#define ADC_SC2_ADTRG_MASK                       (0x40U)
#define ADC_SC2_ADTRG_SHIFT                      (6U)
#define ADC_SC2_ADTRG(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ADTRG_SHIFT)) & ADC_SC2_ADTRG_MASK)
#define ADC_SC2_ADACT_MASK                       (0x80U)
#define ADC_SC2_ADACT_SHIFT                      (7U)
#define ADC_SC2_ADACT(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ADACT_SHIFT)) & ADC_SC2_ADACT_MASK)

/*! @name SC3 - Status and Control Register 3 */
#define ADC_SC3_AVGS_MASK                        (0x3U)
#define ADC_SC3_AVGS_SHIFT                       (0U)
#define ADC_SC3_AVGS(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_AVGS_SHIFT)) & ADC_SC3_AVGS_MASK)
#define ADC_SC3_AVGE_MASK                        (0x4U)
#define ADC_SC3_AVGE_SHIFT                       (2U)
#define ADC_SC3_AVGE(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_AVGE_SHIFT)) & ADC_SC3_AVGE_MASK)
#define ADC_SC3_ADCO_MASK                        (0x8U)
#define ADC_SC3_ADCO_SHIFT                       (3U)
#define ADC_SC3_ADCO(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_ADCO_SHIFT)) & ADC_SC3_ADCO_MASK)
#define ADC_SC3_CALF_MASK                        (0x40U)
#define ADC_SC3_CALF_SHIFT                       (6U)
#define ADC_SC3_CALF(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_CALF_SHIFT)) & ADC_SC3_CALF_MASK)
#define ADC_SC3_CAL_MASK                         (0x80U)
#define ADC_SC3_CAL_SHIFT                        (7U)
#define ADC_SC3_CAL(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_SC3_CAL_SHIFT)) & ADC_SC3_CAL_MASK)

/*! @name OFS - ADC Offset Correction Register */
#define ADC_OFS_OFS_MASK                         (0xFFFFU)
#define ADC_OFS_OFS_SHIFT                        (0U)
#define ADC_OFS_OFS(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_OFS_OFS_SHIFT)) & ADC_OFS_OFS_MASK)

/*! @name PG - ADC Plus-Side Gain Register */
#define ADC_PG_PG_MASK                           (0xFFFFU)
#define ADC_PG_PG_SHIFT                          (0U)
#define ADC_PG_PG(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_PG_PG_SHIFT)) & ADC_PG_PG_MASK)

/*! @name MG - ADC Minus-Side Gain Register */
#define ADC_MG_MG_MASK                           (0xFFFFU)
#define ADC_MG_MG_SHIFT                          (0U)
#define ADC_MG_MG(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_MG_MG_SHIFT)) & ADC_MG_MG_MASK)

/*! @name CLPD - ADC Plus-Side General Calibration Value Register */
#define ADC_CLPD_CLPD_MASK                       (0x3FU)
#define ADC_CLPD_CLPD_SHIFT                      (0U)
#define ADC_CLPD_CLPD(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPD_CLPD_SHIFT)) & ADC_CLPD_CLPD_MASK)

/*! @name CLPS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLPS_CLPS_MASK                       (0x3FU)
#define ADC_CLPS_CLPS_SHIFT                      (0U)
#define ADC_CLPS_CLPS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPS_CLPS_SHIFT)) & ADC_CLPS_CLPS_MASK)

/*! @name CLP4 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP4_CLP4_MASK                       (0x3FFU)
#define ADC_CLP4_CLP4_SHIFT                      (0U)
#define ADC_CLP4_CLP4(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP4_CLP4_SHIFT)) & ADC_CLP4_CLP4_MASK)

/*! @name CLP3 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP3_CLP3_MASK                       (0x1FFU)
#define ADC_CLP3_CLP3_SHIFT                      (0U)
#define ADC_CLP3_CLP3(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP3_CLP3_SHIFT)) & ADC_CLP3_CLP3_MASK)

/*! @name CLP2 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP2_CLP2_MASK                       (0xFFU)
#define ADC_CLP2_CLP2_SHIFT                      (0U)
#define ADC_CLP2_CLP2(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP2_CLP2_SHIFT)) & ADC_CLP2_CLP2_MASK)

/*! @name CLP1 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP1_CLP1_MASK                       (0x7FU)
#define ADC_CLP1_CLP1_SHIFT                      (0U)
#define ADC_CLP1_CLP1(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP1_CLP1_SHIFT)) & ADC_CLP1_CLP1_MASK)

/*! @name CLP0 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP0_CLP0_MASK                       (0x3FU)
#define ADC_CLP0_CLP0_SHIFT                      (0U)
#define ADC_CLP0_CLP0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP0_CLP0_SHIFT)) & ADC_CLP0_CLP0_MASK)

/*! @name CLMD - ADC Minus-Side General Calibration Value Register */
#define ADC_CLMD_CLMD_MASK                       (0x3FU)
#define ADC_CLMD_CLMD_SHIFT                      (0U)
#define ADC_CLMD_CLMD(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLMD_CLMD_SHIFT)) & ADC_CLMD_CLMD_MASK)

/*! @name CLMS - ADC Minus-Side General Calibration Value Register */
#define ADC_CLMS_CLMS_MASK                       (0x3FU)
#define ADC_CLMS_CLMS_SHIFT                      (0U)
#define ADC_CLMS_CLMS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLMS_CLMS_SHIFT)) & ADC_CLMS_CLMS_MASK)

/*! @name CLM4 - ADC Minus-Side General Calibration Value Register */
#define ADC_CLM4_CLM4_MASK                       (0x3FFU)
#define ADC_CLM4_CLM4_SHIFT                      (0U)
#define ADC_CLM4_CLM4(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLM4_CLM4_SHIFT)) & ADC_CLM4_CLM4_MASK)

/*! @name CLM3 - ADC Minus-Side General Calibration Value Register */
#define ADC_CLM3_CLM3_MASK                       (0x1FFU)
#define ADC_CLM3_CLM3_SHIFT                      (0U)
#define ADC_CLM3_CLM3(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLM3_CLM3_SHIFT)) & ADC_CLM3_CLM3_MASK)

/*! @name CLM2 - ADC Minus-Side General Calibration Value Register */
#define ADC_CLM2_CLM2_MASK                       (0xFFU)
#define ADC_CLM2_CLM2_SHIFT                      (0U)
#define ADC_CLM2_CLM2(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLM2_CLM2_SHIFT)) & ADC_CLM2_CLM2_MASK)

/*! @name CLM1 - ADC Minus-Side General Calibration Value Register */
#define ADC_CLM1_CLM1_MASK                       (0x7FU)
#define ADC_CLM1_CLM1_SHIFT                      (0U)
#define ADC_CLM1_CLM1(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLM1_CLM1_SHIFT)) & ADC_CLM1_CLM1_MASK)

/*! @name CLM0 - ADC Minus-Side General Calibration Value Register */
#define ADC_CLM0_CLM0_MASK                       (0x3FU)
#define ADC_CLM0_CLM0_SHIFT                      (0U)
#define ADC_CLM0_CLM0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLM0_CLM0_SHIFT)) & ADC_CLM0_CLM0_MASK)


/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base address */
#define ADC0_BASE                                (0x4003B000u)
/** Peripheral ADC0 base pointer */
#define ADC0                                     ((ADC_Type *)ADC0_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC0_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0 }
/** Interrupt vectors for the ADC peripheral type */
#define ADC_IRQS                                 { ADC0_IRQn }

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- AIPS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Peripheral_Access_Layer AIPS Peripheral Access Layer
 * @{
 */

/** AIPS - Register Layout Typedef */
typedef struct {
  __IO uint32_t MPRA;                              /**< Master Privilege Register A, offset: 0x0 */
       uint8_t RESERVED_0[28];
  __IO uint32_t PACRA;                             /**< Peripheral Access Control Register, offset: 0x20 */
  __IO uint32_t PACRB;                             /**< Peripheral Access Control Register, offset: 0x24 */
  __IO uint32_t PACRC;                             /**< Peripheral Access Control Register, offset: 0x28 */
  __IO uint32_t PACRD;                             /**< Peripheral Access Control Register, offset: 0x2C */
       uint8_t RESERVED_1[16];
  __IO uint32_t PACRE;                             /**< Peripheral Access Control Register, offset: 0x40 */
  __IO uint32_t PACRF;                             /**< Peripheral Access Control Register, offset: 0x44 */
  __IO uint32_t PACRG;                             /**< Peripheral Access Control Register, offset: 0x48 */
  __IO uint32_t PACRH;                             /**< Peripheral Access Control Register, offset: 0x4C */
  __IO uint32_t PACRI;                             /**< Peripheral Access Control Register, offset: 0x50 */
  __IO uint32_t PACRJ;                             /**< Peripheral Access Control Register, offset: 0x54 */
  __IO uint32_t PACRK;                             /**< Peripheral Access Control Register, offset: 0x58 */
  __IO uint32_t PACRL;                             /**< Peripheral Access Control Register, offset: 0x5C */
  __IO uint32_t PACRM;                             /**< Peripheral Access Control Register, offset: 0x60 */
  __IO uint32_t PACRN;                             /**< Peripheral Access Control Register, offset: 0x64 */
  __IO uint32_t PACRO;                             /**< Peripheral Access Control Register, offset: 0x68 */
  __IO uint32_t PACRP;                             /**< Peripheral Access Control Register, offset: 0x6C */
} AIPS_Type;

/* ----------------------------------------------------------------------------
   -- AIPS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Masks AIPS Register Masks
 * @{
 */

/*! @name MPRA - Master Privilege Register A */
#define AIPS_MPRA_MPL4_MASK                      (0x1000U)
#define AIPS_MPRA_MPL4_SHIFT                     (12U)
#define AIPS_MPRA_MPL4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL4_SHIFT)) & AIPS_MPRA_MPL4_MASK)
#define AIPS_MPRA_MTW4_MASK                      (0x2000U)
#define AIPS_MPRA_MTW4_SHIFT                     (13U)
#define AIPS_MPRA_MTW4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW4_SHIFT)) & AIPS_MPRA_MTW4_MASK)
#define AIPS_MPRA_MTR4_MASK                      (0x4000U)
#define AIPS_MPRA_MTR4_SHIFT                     (14U)
#define AIPS_MPRA_MTR4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR4_SHIFT)) & AIPS_MPRA_MTR4_MASK)
#define AIPS_MPRA_MPL3_MASK                      (0x10000U)
#define AIPS_MPRA_MPL3_SHIFT                     (16U)
#define AIPS_MPRA_MPL3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL3_SHIFT)) & AIPS_MPRA_MPL3_MASK)
#define AIPS_MPRA_MTW3_MASK                      (0x20000U)
#define AIPS_MPRA_MTW3_SHIFT                     (17U)
#define AIPS_MPRA_MTW3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW3_SHIFT)) & AIPS_MPRA_MTW3_MASK)
#define AIPS_MPRA_MTR3_MASK                      (0x40000U)
#define AIPS_MPRA_MTR3_SHIFT                     (18U)
#define AIPS_MPRA_MTR3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR3_SHIFT)) & AIPS_MPRA_MTR3_MASK)
#define AIPS_MPRA_MPL2_MASK                      (0x100000U)
#define AIPS_MPRA_MPL2_SHIFT                     (20U)
#define AIPS_MPRA_MPL2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL2_SHIFT)) & AIPS_MPRA_MPL2_MASK)
#define AIPS_MPRA_MTW2_MASK                      (0x200000U)
#define AIPS_MPRA_MTW2_SHIFT                     (21U)
#define AIPS_MPRA_MTW2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW2_SHIFT)) & AIPS_MPRA_MTW2_MASK)
#define AIPS_MPRA_MTR2_MASK                      (0x400000U)
#define AIPS_MPRA_MTR2_SHIFT                     (22U)
#define AIPS_MPRA_MTR2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR2_SHIFT)) & AIPS_MPRA_MTR2_MASK)
#define AIPS_MPRA_MPL1_MASK                      (0x1000000U)
#define AIPS_MPRA_MPL1_SHIFT                     (24U)
#define AIPS_MPRA_MPL1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL1_SHIFT)) & AIPS_MPRA_MPL1_MASK)
#define AIPS_MPRA_MTW1_MASK                      (0x2000000U)
#define AIPS_MPRA_MTW1_SHIFT                     (25U)
#define AIPS_MPRA_MTW1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW1_SHIFT)) & AIPS_MPRA_MTW1_MASK)
#define AIPS_MPRA_MTR1_MASK                      (0x4000000U)
#define AIPS_MPRA_MTR1_SHIFT                     (26U)
#define AIPS_MPRA_MTR1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR1_SHIFT)) & AIPS_MPRA_MTR1_MASK)
#define AIPS_MPRA_MPL0_MASK                      (0x10000000U)
#define AIPS_MPRA_MPL0_SHIFT                     (28U)
#define AIPS_MPRA_MPL0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL0_SHIFT)) & AIPS_MPRA_MPL0_MASK)
#define AIPS_MPRA_MTW0_MASK                      (0x20000000U)
#define AIPS_MPRA_MTW0_SHIFT                     (29U)
#define AIPS_MPRA_MTW0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW0_SHIFT)) & AIPS_MPRA_MTW0_MASK)
#define AIPS_MPRA_MTR0_MASK                      (0x40000000U)
#define AIPS_MPRA_MTR0_SHIFT                     (30U)
#define AIPS_MPRA_MTR0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR0_SHIFT)) & AIPS_MPRA_MTR0_MASK)

/*! @name PACRA - Peripheral Access Control Register */
#define AIPS_PACRA_TP7_MASK                      (0x1U)
#define AIPS_PACRA_TP7_SHIFT                     (0U)
#define AIPS_PACRA_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP7_SHIFT)) & AIPS_PACRA_TP7_MASK)
#define AIPS_PACRA_WP7_MASK                      (0x2U)
#define AIPS_PACRA_WP7_SHIFT                     (1U)
#define AIPS_PACRA_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP7_SHIFT)) & AIPS_PACRA_WP7_MASK)
#define AIPS_PACRA_SP7_MASK                      (0x4U)
#define AIPS_PACRA_SP7_SHIFT                     (2U)
#define AIPS_PACRA_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP7_SHIFT)) & AIPS_PACRA_SP7_MASK)
#define AIPS_PACRA_TP6_MASK                      (0x10U)
#define AIPS_PACRA_TP6_SHIFT                     (4U)
#define AIPS_PACRA_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP6_SHIFT)) & AIPS_PACRA_TP6_MASK)
#define AIPS_PACRA_WP6_MASK                      (0x20U)
#define AIPS_PACRA_WP6_SHIFT                     (5U)
#define AIPS_PACRA_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP6_SHIFT)) & AIPS_PACRA_WP6_MASK)
#define AIPS_PACRA_SP6_MASK                      (0x40U)
#define AIPS_PACRA_SP6_SHIFT                     (6U)
#define AIPS_PACRA_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP6_SHIFT)) & AIPS_PACRA_SP6_MASK)
#define AIPS_PACRA_TP5_MASK                      (0x100U)
#define AIPS_PACRA_TP5_SHIFT                     (8U)
#define AIPS_PACRA_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP5_SHIFT)) & AIPS_PACRA_TP5_MASK)
#define AIPS_PACRA_WP5_MASK                      (0x200U)
#define AIPS_PACRA_WP5_SHIFT                     (9U)
#define AIPS_PACRA_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP5_SHIFT)) & AIPS_PACRA_WP5_MASK)
#define AIPS_PACRA_SP5_MASK                      (0x400U)
#define AIPS_PACRA_SP5_SHIFT                     (10U)
#define AIPS_PACRA_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP5_SHIFT)) & AIPS_PACRA_SP5_MASK)
#define AIPS_PACRA_TP4_MASK                      (0x1000U)
#define AIPS_PACRA_TP4_SHIFT                     (12U)
#define AIPS_PACRA_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP4_SHIFT)) & AIPS_PACRA_TP4_MASK)
#define AIPS_PACRA_WP4_MASK                      (0x2000U)
#define AIPS_PACRA_WP4_SHIFT                     (13U)
#define AIPS_PACRA_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP4_SHIFT)) & AIPS_PACRA_WP4_MASK)
#define AIPS_PACRA_SP4_MASK                      (0x4000U)
#define AIPS_PACRA_SP4_SHIFT                     (14U)
#define AIPS_PACRA_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP4_SHIFT)) & AIPS_PACRA_SP4_MASK)
#define AIPS_PACRA_TP3_MASK                      (0x10000U)
#define AIPS_PACRA_TP3_SHIFT                     (16U)
#define AIPS_PACRA_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP3_SHIFT)) & AIPS_PACRA_TP3_MASK)
#define AIPS_PACRA_WP3_MASK                      (0x20000U)
#define AIPS_PACRA_WP3_SHIFT                     (17U)
#define AIPS_PACRA_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP3_SHIFT)) & AIPS_PACRA_WP3_MASK)
#define AIPS_PACRA_SP3_MASK                      (0x40000U)
#define AIPS_PACRA_SP3_SHIFT                     (18U)
#define AIPS_PACRA_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP3_SHIFT)) & AIPS_PACRA_SP3_MASK)
#define AIPS_PACRA_TP2_MASK                      (0x100000U)
#define AIPS_PACRA_TP2_SHIFT                     (20U)
#define AIPS_PACRA_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP2_SHIFT)) & AIPS_PACRA_TP2_MASK)
#define AIPS_PACRA_WP2_MASK                      (0x200000U)
#define AIPS_PACRA_WP2_SHIFT                     (21U)
#define AIPS_PACRA_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP2_SHIFT)) & AIPS_PACRA_WP2_MASK)
#define AIPS_PACRA_SP2_MASK                      (0x400000U)
#define AIPS_PACRA_SP2_SHIFT                     (22U)
#define AIPS_PACRA_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP2_SHIFT)) & AIPS_PACRA_SP2_MASK)
#define AIPS_PACRA_TP1_MASK                      (0x1000000U)
#define AIPS_PACRA_TP1_SHIFT                     (24U)
#define AIPS_PACRA_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP1_SHIFT)) & AIPS_PACRA_TP1_MASK)
#define AIPS_PACRA_WP1_MASK                      (0x2000000U)
#define AIPS_PACRA_WP1_SHIFT                     (25U)
#define AIPS_PACRA_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP1_SHIFT)) & AIPS_PACRA_WP1_MASK)
#define AIPS_PACRA_SP1_MASK                      (0x4000000U)
#define AIPS_PACRA_SP1_SHIFT                     (26U)
#define AIPS_PACRA_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP1_SHIFT)) & AIPS_PACRA_SP1_MASK)
#define AIPS_PACRA_TP0_MASK                      (0x10000000U)
#define AIPS_PACRA_TP0_SHIFT                     (28U)
#define AIPS_PACRA_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP0_SHIFT)) & AIPS_PACRA_TP0_MASK)
#define AIPS_PACRA_WP0_MASK                      (0x20000000U)
#define AIPS_PACRA_WP0_SHIFT                     (29U)
#define AIPS_PACRA_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP0_SHIFT)) & AIPS_PACRA_WP0_MASK)
#define AIPS_PACRA_SP0_MASK                      (0x40000000U)
#define AIPS_PACRA_SP0_SHIFT                     (30U)
#define AIPS_PACRA_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP0_SHIFT)) & AIPS_PACRA_SP0_MASK)

/*! @name PACRB - Peripheral Access Control Register */
#define AIPS_PACRB_TP7_MASK                      (0x1U)
#define AIPS_PACRB_TP7_SHIFT                     (0U)
#define AIPS_PACRB_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP7_SHIFT)) & AIPS_PACRB_TP7_MASK)
#define AIPS_PACRB_WP7_MASK                      (0x2U)
#define AIPS_PACRB_WP7_SHIFT                     (1U)
#define AIPS_PACRB_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP7_SHIFT)) & AIPS_PACRB_WP7_MASK)
#define AIPS_PACRB_SP7_MASK                      (0x4U)
#define AIPS_PACRB_SP7_SHIFT                     (2U)
#define AIPS_PACRB_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP7_SHIFT)) & AIPS_PACRB_SP7_MASK)
#define AIPS_PACRB_TP6_MASK                      (0x10U)
#define AIPS_PACRB_TP6_SHIFT                     (4U)
#define AIPS_PACRB_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP6_SHIFT)) & AIPS_PACRB_TP6_MASK)
#define AIPS_PACRB_WP6_MASK                      (0x20U)
#define AIPS_PACRB_WP6_SHIFT                     (5U)
#define AIPS_PACRB_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP6_SHIFT)) & AIPS_PACRB_WP6_MASK)
#define AIPS_PACRB_SP6_MASK                      (0x40U)
#define AIPS_PACRB_SP6_SHIFT                     (6U)
#define AIPS_PACRB_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP6_SHIFT)) & AIPS_PACRB_SP6_MASK)
#define AIPS_PACRB_TP5_MASK                      (0x100U)
#define AIPS_PACRB_TP5_SHIFT                     (8U)
#define AIPS_PACRB_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP5_SHIFT)) & AIPS_PACRB_TP5_MASK)
#define AIPS_PACRB_WP5_MASK                      (0x200U)
#define AIPS_PACRB_WP5_SHIFT                     (9U)
#define AIPS_PACRB_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP5_SHIFT)) & AIPS_PACRB_WP5_MASK)
#define AIPS_PACRB_SP5_MASK                      (0x400U)
#define AIPS_PACRB_SP5_SHIFT                     (10U)
#define AIPS_PACRB_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP5_SHIFT)) & AIPS_PACRB_SP5_MASK)
#define AIPS_PACRB_TP4_MASK                      (0x1000U)
#define AIPS_PACRB_TP4_SHIFT                     (12U)
#define AIPS_PACRB_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP4_SHIFT)) & AIPS_PACRB_TP4_MASK)
#define AIPS_PACRB_WP4_MASK                      (0x2000U)
#define AIPS_PACRB_WP4_SHIFT                     (13U)
#define AIPS_PACRB_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP4_SHIFT)) & AIPS_PACRB_WP4_MASK)
#define AIPS_PACRB_SP4_MASK                      (0x4000U)
#define AIPS_PACRB_SP4_SHIFT                     (14U)
#define AIPS_PACRB_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP4_SHIFT)) & AIPS_PACRB_SP4_MASK)
#define AIPS_PACRB_TP3_MASK                      (0x10000U)
#define AIPS_PACRB_TP3_SHIFT                     (16U)
#define AIPS_PACRB_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP3_SHIFT)) & AIPS_PACRB_TP3_MASK)
#define AIPS_PACRB_WP3_MASK                      (0x20000U)
#define AIPS_PACRB_WP3_SHIFT                     (17U)
#define AIPS_PACRB_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP3_SHIFT)) & AIPS_PACRB_WP3_MASK)
#define AIPS_PACRB_SP3_MASK                      (0x40000U)
#define AIPS_PACRB_SP3_SHIFT                     (18U)
#define AIPS_PACRB_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP3_SHIFT)) & AIPS_PACRB_SP3_MASK)
#define AIPS_PACRB_TP2_MASK                      (0x100000U)
#define AIPS_PACRB_TP2_SHIFT                     (20U)
#define AIPS_PACRB_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP2_SHIFT)) & AIPS_PACRB_TP2_MASK)
#define AIPS_PACRB_WP2_MASK                      (0x200000U)
#define AIPS_PACRB_WP2_SHIFT                     (21U)
#define AIPS_PACRB_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP2_SHIFT)) & AIPS_PACRB_WP2_MASK)
#define AIPS_PACRB_SP2_MASK                      (0x400000U)
#define AIPS_PACRB_SP2_SHIFT                     (22U)
#define AIPS_PACRB_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP2_SHIFT)) & AIPS_PACRB_SP2_MASK)
#define AIPS_PACRB_TP1_MASK                      (0x1000000U)
#define AIPS_PACRB_TP1_SHIFT                     (24U)
#define AIPS_PACRB_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP1_SHIFT)) & AIPS_PACRB_TP1_MASK)
#define AIPS_PACRB_WP1_MASK                      (0x2000000U)
#define AIPS_PACRB_WP1_SHIFT                     (25U)
#define AIPS_PACRB_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP1_SHIFT)) & AIPS_PACRB_WP1_MASK)
#define AIPS_PACRB_SP1_MASK                      (0x4000000U)
#define AIPS_PACRB_SP1_SHIFT                     (26U)
#define AIPS_PACRB_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP1_SHIFT)) & AIPS_PACRB_SP1_MASK)
#define AIPS_PACRB_TP0_MASK                      (0x10000000U)
#define AIPS_PACRB_TP0_SHIFT                     (28U)
#define AIPS_PACRB_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP0_SHIFT)) & AIPS_PACRB_TP0_MASK)
#define AIPS_PACRB_WP0_MASK                      (0x20000000U)
#define AIPS_PACRB_WP0_SHIFT                     (29U)
#define AIPS_PACRB_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP0_SHIFT)) & AIPS_PACRB_WP0_MASK)
#define AIPS_PACRB_SP0_MASK                      (0x40000000U)
#define AIPS_PACRB_SP0_SHIFT                     (30U)
#define AIPS_PACRB_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP0_SHIFT)) & AIPS_PACRB_SP0_MASK)

/*! @name PACRC - Peripheral Access Control Register */
#define AIPS_PACRC_TP7_MASK                      (0x1U)
#define AIPS_PACRC_TP7_SHIFT                     (0U)
#define AIPS_PACRC_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP7_SHIFT)) & AIPS_PACRC_TP7_MASK)
#define AIPS_PACRC_WP7_MASK                      (0x2U)
#define AIPS_PACRC_WP7_SHIFT                     (1U)
#define AIPS_PACRC_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP7_SHIFT)) & AIPS_PACRC_WP7_MASK)
#define AIPS_PACRC_SP7_MASK                      (0x4U)
#define AIPS_PACRC_SP7_SHIFT                     (2U)
#define AIPS_PACRC_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP7_SHIFT)) & AIPS_PACRC_SP7_MASK)
#define AIPS_PACRC_TP6_MASK                      (0x10U)
#define AIPS_PACRC_TP6_SHIFT                     (4U)
#define AIPS_PACRC_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP6_SHIFT)) & AIPS_PACRC_TP6_MASK)
#define AIPS_PACRC_WP6_MASK                      (0x20U)
#define AIPS_PACRC_WP6_SHIFT                     (5U)
#define AIPS_PACRC_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP6_SHIFT)) & AIPS_PACRC_WP6_MASK)
#define AIPS_PACRC_SP6_MASK                      (0x40U)
#define AIPS_PACRC_SP6_SHIFT                     (6U)
#define AIPS_PACRC_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP6_SHIFT)) & AIPS_PACRC_SP6_MASK)
#define AIPS_PACRC_TP5_MASK                      (0x100U)
#define AIPS_PACRC_TP5_SHIFT                     (8U)
#define AIPS_PACRC_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP5_SHIFT)) & AIPS_PACRC_TP5_MASK)
#define AIPS_PACRC_WP5_MASK                      (0x200U)
#define AIPS_PACRC_WP5_SHIFT                     (9U)
#define AIPS_PACRC_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP5_SHIFT)) & AIPS_PACRC_WP5_MASK)
#define AIPS_PACRC_SP5_MASK                      (0x400U)
#define AIPS_PACRC_SP5_SHIFT                     (10U)
#define AIPS_PACRC_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP5_SHIFT)) & AIPS_PACRC_SP5_MASK)
#define AIPS_PACRC_TP4_MASK                      (0x1000U)
#define AIPS_PACRC_TP4_SHIFT                     (12U)
#define AIPS_PACRC_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP4_SHIFT)) & AIPS_PACRC_TP4_MASK)
#define AIPS_PACRC_WP4_MASK                      (0x2000U)
#define AIPS_PACRC_WP4_SHIFT                     (13U)
#define AIPS_PACRC_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP4_SHIFT)) & AIPS_PACRC_WP4_MASK)
#define AIPS_PACRC_SP4_MASK                      (0x4000U)
#define AIPS_PACRC_SP4_SHIFT                     (14U)
#define AIPS_PACRC_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP4_SHIFT)) & AIPS_PACRC_SP4_MASK)
#define AIPS_PACRC_TP3_MASK                      (0x10000U)
#define AIPS_PACRC_TP3_SHIFT                     (16U)
#define AIPS_PACRC_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP3_SHIFT)) & AIPS_PACRC_TP3_MASK)
#define AIPS_PACRC_WP3_MASK                      (0x20000U)
#define AIPS_PACRC_WP3_SHIFT                     (17U)
#define AIPS_PACRC_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP3_SHIFT)) & AIPS_PACRC_WP3_MASK)
#define AIPS_PACRC_SP3_MASK                      (0x40000U)
#define AIPS_PACRC_SP3_SHIFT                     (18U)
#define AIPS_PACRC_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP3_SHIFT)) & AIPS_PACRC_SP3_MASK)
#define AIPS_PACRC_TP2_MASK                      (0x100000U)
#define AIPS_PACRC_TP2_SHIFT                     (20U)
#define AIPS_PACRC_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP2_SHIFT)) & AIPS_PACRC_TP2_MASK)
#define AIPS_PACRC_WP2_MASK                      (0x200000U)
#define AIPS_PACRC_WP2_SHIFT                     (21U)
#define AIPS_PACRC_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP2_SHIFT)) & AIPS_PACRC_WP2_MASK)
#define AIPS_PACRC_SP2_MASK                      (0x400000U)
#define AIPS_PACRC_SP2_SHIFT                     (22U)
#define AIPS_PACRC_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP2_SHIFT)) & AIPS_PACRC_SP2_MASK)
#define AIPS_PACRC_TP1_MASK                      (0x1000000U)
#define AIPS_PACRC_TP1_SHIFT                     (24U)
#define AIPS_PACRC_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP1_SHIFT)) & AIPS_PACRC_TP1_MASK)
#define AIPS_PACRC_WP1_MASK                      (0x2000000U)
#define AIPS_PACRC_WP1_SHIFT                     (25U)
#define AIPS_PACRC_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP1_SHIFT)) & AIPS_PACRC_WP1_MASK)
#define AIPS_PACRC_SP1_MASK                      (0x4000000U)
#define AIPS_PACRC_SP1_SHIFT                     (26U)
#define AIPS_PACRC_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP1_SHIFT)) & AIPS_PACRC_SP1_MASK)
#define AIPS_PACRC_TP0_MASK                      (0x10000000U)
#define AIPS_PACRC_TP0_SHIFT                     (28U)
#define AIPS_PACRC_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_TP0_SHIFT)) & AIPS_PACRC_TP0_MASK)
#define AIPS_PACRC_WP0_MASK                      (0x20000000U)
#define AIPS_PACRC_WP0_SHIFT                     (29U)
#define AIPS_PACRC_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_WP0_SHIFT)) & AIPS_PACRC_WP0_MASK)
#define AIPS_PACRC_SP0_MASK                      (0x40000000U)
#define AIPS_PACRC_SP0_SHIFT                     (30U)
#define AIPS_PACRC_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRC_SP0_SHIFT)) & AIPS_PACRC_SP0_MASK)

/*! @name PACRD - Peripheral Access Control Register */
#define AIPS_PACRD_TP7_MASK                      (0x1U)
#define AIPS_PACRD_TP7_SHIFT                     (0U)
#define AIPS_PACRD_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP7_SHIFT)) & AIPS_PACRD_TP7_MASK)
#define AIPS_PACRD_WP7_MASK                      (0x2U)
#define AIPS_PACRD_WP7_SHIFT                     (1U)
#define AIPS_PACRD_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP7_SHIFT)) & AIPS_PACRD_WP7_MASK)
#define AIPS_PACRD_SP7_MASK                      (0x4U)
#define AIPS_PACRD_SP7_SHIFT                     (2U)
#define AIPS_PACRD_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP7_SHIFT)) & AIPS_PACRD_SP7_MASK)
#define AIPS_PACRD_TP6_MASK                      (0x10U)
#define AIPS_PACRD_TP6_SHIFT                     (4U)
#define AIPS_PACRD_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP6_SHIFT)) & AIPS_PACRD_TP6_MASK)
#define AIPS_PACRD_WP6_MASK                      (0x20U)
#define AIPS_PACRD_WP6_SHIFT                     (5U)
#define AIPS_PACRD_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP6_SHIFT)) & AIPS_PACRD_WP6_MASK)
#define AIPS_PACRD_SP6_MASK                      (0x40U)
#define AIPS_PACRD_SP6_SHIFT                     (6U)
#define AIPS_PACRD_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP6_SHIFT)) & AIPS_PACRD_SP6_MASK)
#define AIPS_PACRD_TP5_MASK                      (0x100U)
#define AIPS_PACRD_TP5_SHIFT                     (8U)
#define AIPS_PACRD_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP5_SHIFT)) & AIPS_PACRD_TP5_MASK)
#define AIPS_PACRD_WP5_MASK                      (0x200U)
#define AIPS_PACRD_WP5_SHIFT                     (9U)
#define AIPS_PACRD_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP5_SHIFT)) & AIPS_PACRD_WP5_MASK)
#define AIPS_PACRD_SP5_MASK                      (0x400U)
#define AIPS_PACRD_SP5_SHIFT                     (10U)
#define AIPS_PACRD_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP5_SHIFT)) & AIPS_PACRD_SP5_MASK)
#define AIPS_PACRD_TP4_MASK                      (0x1000U)
#define AIPS_PACRD_TP4_SHIFT                     (12U)
#define AIPS_PACRD_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP4_SHIFT)) & AIPS_PACRD_TP4_MASK)
#define AIPS_PACRD_WP4_MASK                      (0x2000U)
#define AIPS_PACRD_WP4_SHIFT                     (13U)
#define AIPS_PACRD_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP4_SHIFT)) & AIPS_PACRD_WP4_MASK)
#define AIPS_PACRD_SP4_MASK                      (0x4000U)
#define AIPS_PACRD_SP4_SHIFT                     (14U)
#define AIPS_PACRD_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP4_SHIFT)) & AIPS_PACRD_SP4_MASK)
#define AIPS_PACRD_TP3_MASK                      (0x10000U)
#define AIPS_PACRD_TP3_SHIFT                     (16U)
#define AIPS_PACRD_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP3_SHIFT)) & AIPS_PACRD_TP3_MASK)
#define AIPS_PACRD_WP3_MASK                      (0x20000U)
#define AIPS_PACRD_WP3_SHIFT                     (17U)
#define AIPS_PACRD_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP3_SHIFT)) & AIPS_PACRD_WP3_MASK)
#define AIPS_PACRD_SP3_MASK                      (0x40000U)
#define AIPS_PACRD_SP3_SHIFT                     (18U)
#define AIPS_PACRD_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP3_SHIFT)) & AIPS_PACRD_SP3_MASK)
#define AIPS_PACRD_TP2_MASK                      (0x100000U)
#define AIPS_PACRD_TP2_SHIFT                     (20U)
#define AIPS_PACRD_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP2_SHIFT)) & AIPS_PACRD_TP2_MASK)
#define AIPS_PACRD_WP2_MASK                      (0x200000U)
#define AIPS_PACRD_WP2_SHIFT                     (21U)
#define AIPS_PACRD_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP2_SHIFT)) & AIPS_PACRD_WP2_MASK)
#define AIPS_PACRD_SP2_MASK                      (0x400000U)
#define AIPS_PACRD_SP2_SHIFT                     (22U)
#define AIPS_PACRD_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP2_SHIFT)) & AIPS_PACRD_SP2_MASK)
#define AIPS_PACRD_TP1_MASK                      (0x1000000U)
#define AIPS_PACRD_TP1_SHIFT                     (24U)
#define AIPS_PACRD_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP1_SHIFT)) & AIPS_PACRD_TP1_MASK)
#define AIPS_PACRD_WP1_MASK                      (0x2000000U)
#define AIPS_PACRD_WP1_SHIFT                     (25U)
#define AIPS_PACRD_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP1_SHIFT)) & AIPS_PACRD_WP1_MASK)
#define AIPS_PACRD_SP1_MASK                      (0x4000000U)
#define AIPS_PACRD_SP1_SHIFT                     (26U)
#define AIPS_PACRD_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP1_SHIFT)) & AIPS_PACRD_SP1_MASK)
#define AIPS_PACRD_TP0_MASK                      (0x10000000U)
#define AIPS_PACRD_TP0_SHIFT                     (28U)
#define AIPS_PACRD_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP0_SHIFT)) & AIPS_PACRD_TP0_MASK)
#define AIPS_PACRD_WP0_MASK                      (0x20000000U)
#define AIPS_PACRD_WP0_SHIFT                     (29U)
#define AIPS_PACRD_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP0_SHIFT)) & AIPS_PACRD_WP0_MASK)
#define AIPS_PACRD_SP0_MASK                      (0x40000000U)
#define AIPS_PACRD_SP0_SHIFT                     (30U)
#define AIPS_PACRD_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP0_SHIFT)) & AIPS_PACRD_SP0_MASK)

/*! @name PACRE - Peripheral Access Control Register */
#define AIPS_PACRE_TP7_MASK                      (0x1U)
#define AIPS_PACRE_TP7_SHIFT                     (0U)
#define AIPS_PACRE_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP7_SHIFT)) & AIPS_PACRE_TP7_MASK)
#define AIPS_PACRE_WP7_MASK                      (0x2U)
#define AIPS_PACRE_WP7_SHIFT                     (1U)
#define AIPS_PACRE_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP7_SHIFT)) & AIPS_PACRE_WP7_MASK)
#define AIPS_PACRE_SP7_MASK                      (0x4U)
#define AIPS_PACRE_SP7_SHIFT                     (2U)
#define AIPS_PACRE_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP7_SHIFT)) & AIPS_PACRE_SP7_MASK)
#define AIPS_PACRE_TP6_MASK                      (0x10U)
#define AIPS_PACRE_TP6_SHIFT                     (4U)
#define AIPS_PACRE_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP6_SHIFT)) & AIPS_PACRE_TP6_MASK)
#define AIPS_PACRE_WP6_MASK                      (0x20U)
#define AIPS_PACRE_WP6_SHIFT                     (5U)
#define AIPS_PACRE_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP6_SHIFT)) & AIPS_PACRE_WP6_MASK)
#define AIPS_PACRE_SP6_MASK                      (0x40U)
#define AIPS_PACRE_SP6_SHIFT                     (6U)
#define AIPS_PACRE_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP6_SHIFT)) & AIPS_PACRE_SP6_MASK)
#define AIPS_PACRE_TP5_MASK                      (0x100U)
#define AIPS_PACRE_TP5_SHIFT                     (8U)
#define AIPS_PACRE_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP5_SHIFT)) & AIPS_PACRE_TP5_MASK)
#define AIPS_PACRE_WP5_MASK                      (0x200U)
#define AIPS_PACRE_WP5_SHIFT                     (9U)
#define AIPS_PACRE_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP5_SHIFT)) & AIPS_PACRE_WP5_MASK)
#define AIPS_PACRE_SP5_MASK                      (0x400U)
#define AIPS_PACRE_SP5_SHIFT                     (10U)
#define AIPS_PACRE_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP5_SHIFT)) & AIPS_PACRE_SP5_MASK)
#define AIPS_PACRE_TP4_MASK                      (0x1000U)
#define AIPS_PACRE_TP4_SHIFT                     (12U)
#define AIPS_PACRE_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP4_SHIFT)) & AIPS_PACRE_TP4_MASK)
#define AIPS_PACRE_WP4_MASK                      (0x2000U)
#define AIPS_PACRE_WP4_SHIFT                     (13U)
#define AIPS_PACRE_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP4_SHIFT)) & AIPS_PACRE_WP4_MASK)
#define AIPS_PACRE_SP4_MASK                      (0x4000U)
#define AIPS_PACRE_SP4_SHIFT                     (14U)
#define AIPS_PACRE_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP4_SHIFT)) & AIPS_PACRE_SP4_MASK)
#define AIPS_PACRE_TP3_MASK                      (0x10000U)
#define AIPS_PACRE_TP3_SHIFT                     (16U)
#define AIPS_PACRE_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP3_SHIFT)) & AIPS_PACRE_TP3_MASK)
#define AIPS_PACRE_WP3_MASK                      (0x20000U)
#define AIPS_PACRE_WP3_SHIFT                     (17U)
#define AIPS_PACRE_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP3_SHIFT)) & AIPS_PACRE_WP3_MASK)
#define AIPS_PACRE_SP3_MASK                      (0x40000U)
#define AIPS_PACRE_SP3_SHIFT                     (18U)
#define AIPS_PACRE_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP3_SHIFT)) & AIPS_PACRE_SP3_MASK)
#define AIPS_PACRE_TP2_MASK                      (0x100000U)
#define AIPS_PACRE_TP2_SHIFT                     (20U)
#define AIPS_PACRE_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP2_SHIFT)) & AIPS_PACRE_TP2_MASK)
#define AIPS_PACRE_WP2_MASK                      (0x200000U)
#define AIPS_PACRE_WP2_SHIFT                     (21U)
#define AIPS_PACRE_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP2_SHIFT)) & AIPS_PACRE_WP2_MASK)
#define AIPS_PACRE_SP2_MASK                      (0x400000U)
#define AIPS_PACRE_SP2_SHIFT                     (22U)
#define AIPS_PACRE_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP2_SHIFT)) & AIPS_PACRE_SP2_MASK)
#define AIPS_PACRE_TP1_MASK                      (0x1000000U)
#define AIPS_PACRE_TP1_SHIFT                     (24U)
#define AIPS_PACRE_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP1_SHIFT)) & AIPS_PACRE_TP1_MASK)
#define AIPS_PACRE_WP1_MASK                      (0x2000000U)
#define AIPS_PACRE_WP1_SHIFT                     (25U)
#define AIPS_PACRE_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP1_SHIFT)) & AIPS_PACRE_WP1_MASK)
#define AIPS_PACRE_SP1_MASK                      (0x4000000U)
#define AIPS_PACRE_SP1_SHIFT                     (26U)
#define AIPS_PACRE_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP1_SHIFT)) & AIPS_PACRE_SP1_MASK)
#define AIPS_PACRE_TP0_MASK                      (0x10000000U)
#define AIPS_PACRE_TP0_SHIFT                     (28U)
#define AIPS_PACRE_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_TP0_SHIFT)) & AIPS_PACRE_TP0_MASK)
#define AIPS_PACRE_WP0_MASK                      (0x20000000U)
#define AIPS_PACRE_WP0_SHIFT                     (29U)
#define AIPS_PACRE_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_WP0_SHIFT)) & AIPS_PACRE_WP0_MASK)
#define AIPS_PACRE_SP0_MASK                      (0x40000000U)
#define AIPS_PACRE_SP0_SHIFT                     (30U)
#define AIPS_PACRE_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRE_SP0_SHIFT)) & AIPS_PACRE_SP0_MASK)

/*! @name PACRF - Peripheral Access Control Register */
#define AIPS_PACRF_TP7_MASK                      (0x1U)
#define AIPS_PACRF_TP7_SHIFT                     (0U)
#define AIPS_PACRF_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP7_SHIFT)) & AIPS_PACRF_TP7_MASK)
#define AIPS_PACRF_WP7_MASK                      (0x2U)
#define AIPS_PACRF_WP7_SHIFT                     (1U)
#define AIPS_PACRF_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP7_SHIFT)) & AIPS_PACRF_WP7_MASK)
#define AIPS_PACRF_SP7_MASK                      (0x4U)
#define AIPS_PACRF_SP7_SHIFT                     (2U)
#define AIPS_PACRF_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP7_SHIFT)) & AIPS_PACRF_SP7_MASK)
#define AIPS_PACRF_TP6_MASK                      (0x10U)
#define AIPS_PACRF_TP6_SHIFT                     (4U)
#define AIPS_PACRF_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP6_SHIFT)) & AIPS_PACRF_TP6_MASK)
#define AIPS_PACRF_WP6_MASK                      (0x20U)
#define AIPS_PACRF_WP6_SHIFT                     (5U)
#define AIPS_PACRF_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP6_SHIFT)) & AIPS_PACRF_WP6_MASK)
#define AIPS_PACRF_SP6_MASK                      (0x40U)
#define AIPS_PACRF_SP6_SHIFT                     (6U)
#define AIPS_PACRF_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP6_SHIFT)) & AIPS_PACRF_SP6_MASK)
#define AIPS_PACRF_TP5_MASK                      (0x100U)
#define AIPS_PACRF_TP5_SHIFT                     (8U)
#define AIPS_PACRF_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP5_SHIFT)) & AIPS_PACRF_TP5_MASK)
#define AIPS_PACRF_WP5_MASK                      (0x200U)
#define AIPS_PACRF_WP5_SHIFT                     (9U)
#define AIPS_PACRF_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP5_SHIFT)) & AIPS_PACRF_WP5_MASK)
#define AIPS_PACRF_SP5_MASK                      (0x400U)
#define AIPS_PACRF_SP5_SHIFT                     (10U)
#define AIPS_PACRF_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP5_SHIFT)) & AIPS_PACRF_SP5_MASK)
#define AIPS_PACRF_TP4_MASK                      (0x1000U)
#define AIPS_PACRF_TP4_SHIFT                     (12U)
#define AIPS_PACRF_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP4_SHIFT)) & AIPS_PACRF_TP4_MASK)
#define AIPS_PACRF_WP4_MASK                      (0x2000U)
#define AIPS_PACRF_WP4_SHIFT                     (13U)
#define AIPS_PACRF_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP4_SHIFT)) & AIPS_PACRF_WP4_MASK)
#define AIPS_PACRF_SP4_MASK                      (0x4000U)
#define AIPS_PACRF_SP4_SHIFT                     (14U)
#define AIPS_PACRF_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP4_SHIFT)) & AIPS_PACRF_SP4_MASK)
#define AIPS_PACRF_TP3_MASK                      (0x10000U)
#define AIPS_PACRF_TP3_SHIFT                     (16U)
#define AIPS_PACRF_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP3_SHIFT)) & AIPS_PACRF_TP3_MASK)
#define AIPS_PACRF_WP3_MASK                      (0x20000U)
#define AIPS_PACRF_WP3_SHIFT                     (17U)
#define AIPS_PACRF_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP3_SHIFT)) & AIPS_PACRF_WP3_MASK)
#define AIPS_PACRF_SP3_MASK                      (0x40000U)
#define AIPS_PACRF_SP3_SHIFT                     (18U)
#define AIPS_PACRF_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP3_SHIFT)) & AIPS_PACRF_SP3_MASK)
#define AIPS_PACRF_TP2_MASK                      (0x100000U)
#define AIPS_PACRF_TP2_SHIFT                     (20U)
#define AIPS_PACRF_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP2_SHIFT)) & AIPS_PACRF_TP2_MASK)
#define AIPS_PACRF_WP2_MASK                      (0x200000U)
#define AIPS_PACRF_WP2_SHIFT                     (21U)
#define AIPS_PACRF_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP2_SHIFT)) & AIPS_PACRF_WP2_MASK)
#define AIPS_PACRF_SP2_MASK                      (0x400000U)
#define AIPS_PACRF_SP2_SHIFT                     (22U)
#define AIPS_PACRF_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP2_SHIFT)) & AIPS_PACRF_SP2_MASK)
#define AIPS_PACRF_TP1_MASK                      (0x1000000U)
#define AIPS_PACRF_TP1_SHIFT                     (24U)
#define AIPS_PACRF_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP1_SHIFT)) & AIPS_PACRF_TP1_MASK)
#define AIPS_PACRF_WP1_MASK                      (0x2000000U)
#define AIPS_PACRF_WP1_SHIFT                     (25U)
#define AIPS_PACRF_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP1_SHIFT)) & AIPS_PACRF_WP1_MASK)
#define AIPS_PACRF_SP1_MASK                      (0x4000000U)
#define AIPS_PACRF_SP1_SHIFT                     (26U)
#define AIPS_PACRF_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP1_SHIFT)) & AIPS_PACRF_SP1_MASK)
#define AIPS_PACRF_TP0_MASK                      (0x10000000U)
#define AIPS_PACRF_TP0_SHIFT                     (28U)
#define AIPS_PACRF_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_TP0_SHIFT)) & AIPS_PACRF_TP0_MASK)
#define AIPS_PACRF_WP0_MASK                      (0x20000000U)
#define AIPS_PACRF_WP0_SHIFT                     (29U)
#define AIPS_PACRF_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_WP0_SHIFT)) & AIPS_PACRF_WP0_MASK)
#define AIPS_PACRF_SP0_MASK                      (0x40000000U)
#define AIPS_PACRF_SP0_SHIFT                     (30U)
#define AIPS_PACRF_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRF_SP0_SHIFT)) & AIPS_PACRF_SP0_MASK)

/*! @name PACRG - Peripheral Access Control Register */
#define AIPS_PACRG_TP7_MASK                      (0x1U)
#define AIPS_PACRG_TP7_SHIFT                     (0U)
#define AIPS_PACRG_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP7_SHIFT)) & AIPS_PACRG_TP7_MASK)
#define AIPS_PACRG_WP7_MASK                      (0x2U)
#define AIPS_PACRG_WP7_SHIFT                     (1U)
#define AIPS_PACRG_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP7_SHIFT)) & AIPS_PACRG_WP7_MASK)
#define AIPS_PACRG_SP7_MASK                      (0x4U)
#define AIPS_PACRG_SP7_SHIFT                     (2U)
#define AIPS_PACRG_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP7_SHIFT)) & AIPS_PACRG_SP7_MASK)
#define AIPS_PACRG_TP6_MASK                      (0x10U)
#define AIPS_PACRG_TP6_SHIFT                     (4U)
#define AIPS_PACRG_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP6_SHIFT)) & AIPS_PACRG_TP6_MASK)
#define AIPS_PACRG_WP6_MASK                      (0x20U)
#define AIPS_PACRG_WP6_SHIFT                     (5U)
#define AIPS_PACRG_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP6_SHIFT)) & AIPS_PACRG_WP6_MASK)
#define AIPS_PACRG_SP6_MASK                      (0x40U)
#define AIPS_PACRG_SP6_SHIFT                     (6U)
#define AIPS_PACRG_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP6_SHIFT)) & AIPS_PACRG_SP6_MASK)
#define AIPS_PACRG_TP5_MASK                      (0x100U)
#define AIPS_PACRG_TP5_SHIFT                     (8U)
#define AIPS_PACRG_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP5_SHIFT)) & AIPS_PACRG_TP5_MASK)
#define AIPS_PACRG_WP5_MASK                      (0x200U)
#define AIPS_PACRG_WP5_SHIFT                     (9U)
#define AIPS_PACRG_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP5_SHIFT)) & AIPS_PACRG_WP5_MASK)
#define AIPS_PACRG_SP5_MASK                      (0x400U)
#define AIPS_PACRG_SP5_SHIFT                     (10U)
#define AIPS_PACRG_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP5_SHIFT)) & AIPS_PACRG_SP5_MASK)
#define AIPS_PACRG_TP4_MASK                      (0x1000U)
#define AIPS_PACRG_TP4_SHIFT                     (12U)
#define AIPS_PACRG_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP4_SHIFT)) & AIPS_PACRG_TP4_MASK)
#define AIPS_PACRG_WP4_MASK                      (0x2000U)
#define AIPS_PACRG_WP4_SHIFT                     (13U)
#define AIPS_PACRG_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP4_SHIFT)) & AIPS_PACRG_WP4_MASK)
#define AIPS_PACRG_SP4_MASK                      (0x4000U)
#define AIPS_PACRG_SP4_SHIFT                     (14U)
#define AIPS_PACRG_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP4_SHIFT)) & AIPS_PACRG_SP4_MASK)
#define AIPS_PACRG_TP3_MASK                      (0x10000U)
#define AIPS_PACRG_TP3_SHIFT                     (16U)
#define AIPS_PACRG_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP3_SHIFT)) & AIPS_PACRG_TP3_MASK)
#define AIPS_PACRG_WP3_MASK                      (0x20000U)
#define AIPS_PACRG_WP3_SHIFT                     (17U)
#define AIPS_PACRG_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP3_SHIFT)) & AIPS_PACRG_WP3_MASK)
#define AIPS_PACRG_SP3_MASK                      (0x40000U)
#define AIPS_PACRG_SP3_SHIFT                     (18U)
#define AIPS_PACRG_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP3_SHIFT)) & AIPS_PACRG_SP3_MASK)
#define AIPS_PACRG_TP2_MASK                      (0x100000U)
#define AIPS_PACRG_TP2_SHIFT                     (20U)
#define AIPS_PACRG_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP2_SHIFT)) & AIPS_PACRG_TP2_MASK)
#define AIPS_PACRG_WP2_MASK                      (0x200000U)
#define AIPS_PACRG_WP2_SHIFT                     (21U)
#define AIPS_PACRG_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP2_SHIFT)) & AIPS_PACRG_WP2_MASK)
#define AIPS_PACRG_SP2_MASK                      (0x400000U)
#define AIPS_PACRG_SP2_SHIFT                     (22U)
#define AIPS_PACRG_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP2_SHIFT)) & AIPS_PACRG_SP2_MASK)
#define AIPS_PACRG_TP1_MASK                      (0x1000000U)
#define AIPS_PACRG_TP1_SHIFT                     (24U)
#define AIPS_PACRG_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP1_SHIFT)) & AIPS_PACRG_TP1_MASK)
#define AIPS_PACRG_WP1_MASK                      (0x2000000U)
#define AIPS_PACRG_WP1_SHIFT                     (25U)
#define AIPS_PACRG_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP1_SHIFT)) & AIPS_PACRG_WP1_MASK)
#define AIPS_PACRG_SP1_MASK                      (0x4000000U)
#define AIPS_PACRG_SP1_SHIFT                     (26U)
#define AIPS_PACRG_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP1_SHIFT)) & AIPS_PACRG_SP1_MASK)
#define AIPS_PACRG_TP0_MASK                      (0x10000000U)
#define AIPS_PACRG_TP0_SHIFT                     (28U)
#define AIPS_PACRG_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_TP0_SHIFT)) & AIPS_PACRG_TP0_MASK)
#define AIPS_PACRG_WP0_MASK                      (0x20000000U)
#define AIPS_PACRG_WP0_SHIFT                     (29U)
#define AIPS_PACRG_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_WP0_SHIFT)) & AIPS_PACRG_WP0_MASK)
#define AIPS_PACRG_SP0_MASK                      (0x40000000U)
#define AIPS_PACRG_SP0_SHIFT                     (30U)
#define AIPS_PACRG_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRG_SP0_SHIFT)) & AIPS_PACRG_SP0_MASK)

/*! @name PACRH - Peripheral Access Control Register */
#define AIPS_PACRH_TP7_MASK                      (0x1U)
#define AIPS_PACRH_TP7_SHIFT                     (0U)
#define AIPS_PACRH_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP7_SHIFT)) & AIPS_PACRH_TP7_MASK)
#define AIPS_PACRH_WP7_MASK                      (0x2U)
#define AIPS_PACRH_WP7_SHIFT                     (1U)
#define AIPS_PACRH_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP7_SHIFT)) & AIPS_PACRH_WP7_MASK)
#define AIPS_PACRH_SP7_MASK                      (0x4U)
#define AIPS_PACRH_SP7_SHIFT                     (2U)
#define AIPS_PACRH_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP7_SHIFT)) & AIPS_PACRH_SP7_MASK)
#define AIPS_PACRH_TP6_MASK                      (0x10U)
#define AIPS_PACRH_TP6_SHIFT                     (4U)
#define AIPS_PACRH_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP6_SHIFT)) & AIPS_PACRH_TP6_MASK)
#define AIPS_PACRH_WP6_MASK                      (0x20U)
#define AIPS_PACRH_WP6_SHIFT                     (5U)
#define AIPS_PACRH_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP6_SHIFT)) & AIPS_PACRH_WP6_MASK)
#define AIPS_PACRH_SP6_MASK                      (0x40U)
#define AIPS_PACRH_SP6_SHIFT                     (6U)
#define AIPS_PACRH_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP6_SHIFT)) & AIPS_PACRH_SP6_MASK)
#define AIPS_PACRH_TP5_MASK                      (0x100U)
#define AIPS_PACRH_TP5_SHIFT                     (8U)
#define AIPS_PACRH_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP5_SHIFT)) & AIPS_PACRH_TP5_MASK)
#define AIPS_PACRH_WP5_MASK                      (0x200U)
#define AIPS_PACRH_WP5_SHIFT                     (9U)
#define AIPS_PACRH_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP5_SHIFT)) & AIPS_PACRH_WP5_MASK)
#define AIPS_PACRH_SP5_MASK                      (0x400U)
#define AIPS_PACRH_SP5_SHIFT                     (10U)
#define AIPS_PACRH_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP5_SHIFT)) & AIPS_PACRH_SP5_MASK)
#define AIPS_PACRH_TP4_MASK                      (0x1000U)
#define AIPS_PACRH_TP4_SHIFT                     (12U)
#define AIPS_PACRH_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP4_SHIFT)) & AIPS_PACRH_TP4_MASK)
#define AIPS_PACRH_WP4_MASK                      (0x2000U)
#define AIPS_PACRH_WP4_SHIFT                     (13U)
#define AIPS_PACRH_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP4_SHIFT)) & AIPS_PACRH_WP4_MASK)
#define AIPS_PACRH_SP4_MASK                      (0x4000U)
#define AIPS_PACRH_SP4_SHIFT                     (14U)
#define AIPS_PACRH_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP4_SHIFT)) & AIPS_PACRH_SP4_MASK)
#define AIPS_PACRH_TP3_MASK                      (0x10000U)
#define AIPS_PACRH_TP3_SHIFT                     (16U)
#define AIPS_PACRH_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP3_SHIFT)) & AIPS_PACRH_TP3_MASK)
#define AIPS_PACRH_WP3_MASK                      (0x20000U)
#define AIPS_PACRH_WP3_SHIFT                     (17U)
#define AIPS_PACRH_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP3_SHIFT)) & AIPS_PACRH_WP3_MASK)
#define AIPS_PACRH_SP3_MASK                      (0x40000U)
#define AIPS_PACRH_SP3_SHIFT                     (18U)
#define AIPS_PACRH_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP3_SHIFT)) & AIPS_PACRH_SP3_MASK)
#define AIPS_PACRH_TP2_MASK                      (0x100000U)
#define AIPS_PACRH_TP2_SHIFT                     (20U)
#define AIPS_PACRH_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP2_SHIFT)) & AIPS_PACRH_TP2_MASK)
#define AIPS_PACRH_WP2_MASK                      (0x200000U)
#define AIPS_PACRH_WP2_SHIFT                     (21U)
#define AIPS_PACRH_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP2_SHIFT)) & AIPS_PACRH_WP2_MASK)
#define AIPS_PACRH_SP2_MASK                      (0x400000U)
#define AIPS_PACRH_SP2_SHIFT                     (22U)
#define AIPS_PACRH_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP2_SHIFT)) & AIPS_PACRH_SP2_MASK)
#define AIPS_PACRH_TP1_MASK                      (0x1000000U)
#define AIPS_PACRH_TP1_SHIFT                     (24U)
#define AIPS_PACRH_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP1_SHIFT)) & AIPS_PACRH_TP1_MASK)
#define AIPS_PACRH_WP1_MASK                      (0x2000000U)
#define AIPS_PACRH_WP1_SHIFT                     (25U)
#define AIPS_PACRH_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP1_SHIFT)) & AIPS_PACRH_WP1_MASK)
#define AIPS_PACRH_SP1_MASK                      (0x4000000U)
#define AIPS_PACRH_SP1_SHIFT                     (26U)
#define AIPS_PACRH_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP1_SHIFT)) & AIPS_PACRH_SP1_MASK)
#define AIPS_PACRH_TP0_MASK                      (0x10000000U)
#define AIPS_PACRH_TP0_SHIFT                     (28U)
#define AIPS_PACRH_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_TP0_SHIFT)) & AIPS_PACRH_TP0_MASK)
#define AIPS_PACRH_WP0_MASK                      (0x20000000U)
#define AIPS_PACRH_WP0_SHIFT                     (29U)
#define AIPS_PACRH_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_WP0_SHIFT)) & AIPS_PACRH_WP0_MASK)
#define AIPS_PACRH_SP0_MASK                      (0x40000000U)
#define AIPS_PACRH_SP0_SHIFT                     (30U)
#define AIPS_PACRH_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRH_SP0_SHIFT)) & AIPS_PACRH_SP0_MASK)

/*! @name PACRI - Peripheral Access Control Register */
#define AIPS_PACRI_TP7_MASK                      (0x1U)
#define AIPS_PACRI_TP7_SHIFT                     (0U)
#define AIPS_PACRI_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP7_SHIFT)) & AIPS_PACRI_TP7_MASK)
#define AIPS_PACRI_WP7_MASK                      (0x2U)
#define AIPS_PACRI_WP7_SHIFT                     (1U)
#define AIPS_PACRI_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP7_SHIFT)) & AIPS_PACRI_WP7_MASK)
#define AIPS_PACRI_SP7_MASK                      (0x4U)
#define AIPS_PACRI_SP7_SHIFT                     (2U)
#define AIPS_PACRI_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP7_SHIFT)) & AIPS_PACRI_SP7_MASK)
#define AIPS_PACRI_TP6_MASK                      (0x10U)
#define AIPS_PACRI_TP6_SHIFT                     (4U)
#define AIPS_PACRI_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP6_SHIFT)) & AIPS_PACRI_TP6_MASK)
#define AIPS_PACRI_WP6_MASK                      (0x20U)
#define AIPS_PACRI_WP6_SHIFT                     (5U)
#define AIPS_PACRI_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP6_SHIFT)) & AIPS_PACRI_WP6_MASK)
#define AIPS_PACRI_SP6_MASK                      (0x40U)
#define AIPS_PACRI_SP6_SHIFT                     (6U)
#define AIPS_PACRI_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP6_SHIFT)) & AIPS_PACRI_SP6_MASK)
#define AIPS_PACRI_TP5_MASK                      (0x100U)
#define AIPS_PACRI_TP5_SHIFT                     (8U)
#define AIPS_PACRI_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP5_SHIFT)) & AIPS_PACRI_TP5_MASK)
#define AIPS_PACRI_WP5_MASK                      (0x200U)
#define AIPS_PACRI_WP5_SHIFT                     (9U)
#define AIPS_PACRI_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP5_SHIFT)) & AIPS_PACRI_WP5_MASK)
#define AIPS_PACRI_SP5_MASK                      (0x400U)
#define AIPS_PACRI_SP5_SHIFT                     (10U)
#define AIPS_PACRI_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP5_SHIFT)) & AIPS_PACRI_SP5_MASK)
#define AIPS_PACRI_TP4_MASK                      (0x1000U)
#define AIPS_PACRI_TP4_SHIFT                     (12U)
#define AIPS_PACRI_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP4_SHIFT)) & AIPS_PACRI_TP4_MASK)
#define AIPS_PACRI_WP4_MASK                      (0x2000U)
#define AIPS_PACRI_WP4_SHIFT                     (13U)
#define AIPS_PACRI_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP4_SHIFT)) & AIPS_PACRI_WP4_MASK)
#define AIPS_PACRI_SP4_MASK                      (0x4000U)
#define AIPS_PACRI_SP4_SHIFT                     (14U)
#define AIPS_PACRI_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP4_SHIFT)) & AIPS_PACRI_SP4_MASK)
#define AIPS_PACRI_TP3_MASK                      (0x10000U)
#define AIPS_PACRI_TP3_SHIFT                     (16U)
#define AIPS_PACRI_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP3_SHIFT)) & AIPS_PACRI_TP3_MASK)
#define AIPS_PACRI_WP3_MASK                      (0x20000U)
#define AIPS_PACRI_WP3_SHIFT                     (17U)
#define AIPS_PACRI_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP3_SHIFT)) & AIPS_PACRI_WP3_MASK)
#define AIPS_PACRI_SP3_MASK                      (0x40000U)
#define AIPS_PACRI_SP3_SHIFT                     (18U)
#define AIPS_PACRI_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP3_SHIFT)) & AIPS_PACRI_SP3_MASK)
#define AIPS_PACRI_TP2_MASK                      (0x100000U)
#define AIPS_PACRI_TP2_SHIFT                     (20U)
#define AIPS_PACRI_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP2_SHIFT)) & AIPS_PACRI_TP2_MASK)
#define AIPS_PACRI_WP2_MASK                      (0x200000U)
#define AIPS_PACRI_WP2_SHIFT                     (21U)
#define AIPS_PACRI_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP2_SHIFT)) & AIPS_PACRI_WP2_MASK)
#define AIPS_PACRI_SP2_MASK                      (0x400000U)
#define AIPS_PACRI_SP2_SHIFT                     (22U)
#define AIPS_PACRI_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP2_SHIFT)) & AIPS_PACRI_SP2_MASK)
#define AIPS_PACRI_TP1_MASK                      (0x1000000U)
#define AIPS_PACRI_TP1_SHIFT                     (24U)
#define AIPS_PACRI_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP1_SHIFT)) & AIPS_PACRI_TP1_MASK)
#define AIPS_PACRI_WP1_MASK                      (0x2000000U)
#define AIPS_PACRI_WP1_SHIFT                     (25U)
#define AIPS_PACRI_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP1_SHIFT)) & AIPS_PACRI_WP1_MASK)
#define AIPS_PACRI_SP1_MASK                      (0x4000000U)
#define AIPS_PACRI_SP1_SHIFT                     (26U)
#define AIPS_PACRI_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP1_SHIFT)) & AIPS_PACRI_SP1_MASK)
#define AIPS_PACRI_TP0_MASK                      (0x10000000U)
#define AIPS_PACRI_TP0_SHIFT                     (28U)
#define AIPS_PACRI_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_TP0_SHIFT)) & AIPS_PACRI_TP0_MASK)
#define AIPS_PACRI_WP0_MASK                      (0x20000000U)
#define AIPS_PACRI_WP0_SHIFT                     (29U)
#define AIPS_PACRI_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_WP0_SHIFT)) & AIPS_PACRI_WP0_MASK)
#define AIPS_PACRI_SP0_MASK                      (0x40000000U)
#define AIPS_PACRI_SP0_SHIFT                     (30U)
#define AIPS_PACRI_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRI_SP0_SHIFT)) & AIPS_PACRI_SP0_MASK)

/*! @name PACRJ - Peripheral Access Control Register */
#define AIPS_PACRJ_TP7_MASK                      (0x1U)
#define AIPS_PACRJ_TP7_SHIFT                     (0U)
#define AIPS_PACRJ_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP7_SHIFT)) & AIPS_PACRJ_TP7_MASK)
#define AIPS_PACRJ_WP7_MASK                      (0x2U)
#define AIPS_PACRJ_WP7_SHIFT                     (1U)
#define AIPS_PACRJ_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP7_SHIFT)) & AIPS_PACRJ_WP7_MASK)
#define AIPS_PACRJ_SP7_MASK                      (0x4U)
#define AIPS_PACRJ_SP7_SHIFT                     (2U)
#define AIPS_PACRJ_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP7_SHIFT)) & AIPS_PACRJ_SP7_MASK)
#define AIPS_PACRJ_TP6_MASK                      (0x10U)
#define AIPS_PACRJ_TP6_SHIFT                     (4U)
#define AIPS_PACRJ_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP6_SHIFT)) & AIPS_PACRJ_TP6_MASK)
#define AIPS_PACRJ_WP6_MASK                      (0x20U)
#define AIPS_PACRJ_WP6_SHIFT                     (5U)
#define AIPS_PACRJ_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP6_SHIFT)) & AIPS_PACRJ_WP6_MASK)
#define AIPS_PACRJ_SP6_MASK                      (0x40U)
#define AIPS_PACRJ_SP6_SHIFT                     (6U)
#define AIPS_PACRJ_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP6_SHIFT)) & AIPS_PACRJ_SP6_MASK)
#define AIPS_PACRJ_TP5_MASK                      (0x100U)
#define AIPS_PACRJ_TP5_SHIFT                     (8U)
#define AIPS_PACRJ_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP5_SHIFT)) & AIPS_PACRJ_TP5_MASK)
#define AIPS_PACRJ_WP5_MASK                      (0x200U)
#define AIPS_PACRJ_WP5_SHIFT                     (9U)
#define AIPS_PACRJ_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP5_SHIFT)) & AIPS_PACRJ_WP5_MASK)
#define AIPS_PACRJ_SP5_MASK                      (0x400U)
#define AIPS_PACRJ_SP5_SHIFT                     (10U)
#define AIPS_PACRJ_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP5_SHIFT)) & AIPS_PACRJ_SP5_MASK)
#define AIPS_PACRJ_TP4_MASK                      (0x1000U)
#define AIPS_PACRJ_TP4_SHIFT                     (12U)
#define AIPS_PACRJ_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP4_SHIFT)) & AIPS_PACRJ_TP4_MASK)
#define AIPS_PACRJ_WP4_MASK                      (0x2000U)
#define AIPS_PACRJ_WP4_SHIFT                     (13U)
#define AIPS_PACRJ_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP4_SHIFT)) & AIPS_PACRJ_WP4_MASK)
#define AIPS_PACRJ_SP4_MASK                      (0x4000U)
#define AIPS_PACRJ_SP4_SHIFT                     (14U)
#define AIPS_PACRJ_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP4_SHIFT)) & AIPS_PACRJ_SP4_MASK)
#define AIPS_PACRJ_TP3_MASK                      (0x10000U)
#define AIPS_PACRJ_TP3_SHIFT                     (16U)
#define AIPS_PACRJ_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP3_SHIFT)) & AIPS_PACRJ_TP3_MASK)
#define AIPS_PACRJ_WP3_MASK                      (0x20000U)
#define AIPS_PACRJ_WP3_SHIFT                     (17U)
#define AIPS_PACRJ_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP3_SHIFT)) & AIPS_PACRJ_WP3_MASK)
#define AIPS_PACRJ_SP3_MASK                      (0x40000U)
#define AIPS_PACRJ_SP3_SHIFT                     (18U)
#define AIPS_PACRJ_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP3_SHIFT)) & AIPS_PACRJ_SP3_MASK)
#define AIPS_PACRJ_TP2_MASK                      (0x100000U)
#define AIPS_PACRJ_TP2_SHIFT                     (20U)
#define AIPS_PACRJ_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP2_SHIFT)) & AIPS_PACRJ_TP2_MASK)
#define AIPS_PACRJ_WP2_MASK                      (0x200000U)
#define AIPS_PACRJ_WP2_SHIFT                     (21U)
#define AIPS_PACRJ_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP2_SHIFT)) & AIPS_PACRJ_WP2_MASK)
#define AIPS_PACRJ_SP2_MASK                      (0x400000U)
#define AIPS_PACRJ_SP2_SHIFT                     (22U)
#define AIPS_PACRJ_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP2_SHIFT)) & AIPS_PACRJ_SP2_MASK)
#define AIPS_PACRJ_TP1_MASK                      (0x1000000U)
#define AIPS_PACRJ_TP1_SHIFT                     (24U)
#define AIPS_PACRJ_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP1_SHIFT)) & AIPS_PACRJ_TP1_MASK)
#define AIPS_PACRJ_WP1_MASK                      (0x2000000U)
#define AIPS_PACRJ_WP1_SHIFT                     (25U)
#define AIPS_PACRJ_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP1_SHIFT)) & AIPS_PACRJ_WP1_MASK)
#define AIPS_PACRJ_SP1_MASK                      (0x4000000U)
#define AIPS_PACRJ_SP1_SHIFT                     (26U)
#define AIPS_PACRJ_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP1_SHIFT)) & AIPS_PACRJ_SP1_MASK)
#define AIPS_PACRJ_TP0_MASK                      (0x10000000U)
#define AIPS_PACRJ_TP0_SHIFT                     (28U)
#define AIPS_PACRJ_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_TP0_SHIFT)) & AIPS_PACRJ_TP0_MASK)
#define AIPS_PACRJ_WP0_MASK                      (0x20000000U)
#define AIPS_PACRJ_WP0_SHIFT                     (29U)
#define AIPS_PACRJ_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_WP0_SHIFT)) & AIPS_PACRJ_WP0_MASK)
#define AIPS_PACRJ_SP0_MASK                      (0x40000000U)
#define AIPS_PACRJ_SP0_SHIFT                     (30U)
#define AIPS_PACRJ_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRJ_SP0_SHIFT)) & AIPS_PACRJ_SP0_MASK)

/*! @name PACRK - Peripheral Access Control Register */
#define AIPS_PACRK_TP7_MASK                      (0x1U)
#define AIPS_PACRK_TP7_SHIFT                     (0U)
#define AIPS_PACRK_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP7_SHIFT)) & AIPS_PACRK_TP7_MASK)
#define AIPS_PACRK_WP7_MASK                      (0x2U)
#define AIPS_PACRK_WP7_SHIFT                     (1U)
#define AIPS_PACRK_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP7_SHIFT)) & AIPS_PACRK_WP7_MASK)
#define AIPS_PACRK_SP7_MASK                      (0x4U)
#define AIPS_PACRK_SP7_SHIFT                     (2U)
#define AIPS_PACRK_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP7_SHIFT)) & AIPS_PACRK_SP7_MASK)
#define AIPS_PACRK_TP6_MASK                      (0x10U)
#define AIPS_PACRK_TP6_SHIFT                     (4U)
#define AIPS_PACRK_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP6_SHIFT)) & AIPS_PACRK_TP6_MASK)
#define AIPS_PACRK_WP6_MASK                      (0x20U)
#define AIPS_PACRK_WP6_SHIFT                     (5U)
#define AIPS_PACRK_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP6_SHIFT)) & AIPS_PACRK_WP6_MASK)
#define AIPS_PACRK_SP6_MASK                      (0x40U)
#define AIPS_PACRK_SP6_SHIFT                     (6U)
#define AIPS_PACRK_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP6_SHIFT)) & AIPS_PACRK_SP6_MASK)
#define AIPS_PACRK_TP5_MASK                      (0x100U)
#define AIPS_PACRK_TP5_SHIFT                     (8U)
#define AIPS_PACRK_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP5_SHIFT)) & AIPS_PACRK_TP5_MASK)
#define AIPS_PACRK_WP5_MASK                      (0x200U)
#define AIPS_PACRK_WP5_SHIFT                     (9U)
#define AIPS_PACRK_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP5_SHIFT)) & AIPS_PACRK_WP5_MASK)
#define AIPS_PACRK_SP5_MASK                      (0x400U)
#define AIPS_PACRK_SP5_SHIFT                     (10U)
#define AIPS_PACRK_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP5_SHIFT)) & AIPS_PACRK_SP5_MASK)
#define AIPS_PACRK_TP4_MASK                      (0x1000U)
#define AIPS_PACRK_TP4_SHIFT                     (12U)
#define AIPS_PACRK_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP4_SHIFT)) & AIPS_PACRK_TP4_MASK)
#define AIPS_PACRK_WP4_MASK                      (0x2000U)
#define AIPS_PACRK_WP4_SHIFT                     (13U)
#define AIPS_PACRK_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP4_SHIFT)) & AIPS_PACRK_WP4_MASK)
#define AIPS_PACRK_SP4_MASK                      (0x4000U)
#define AIPS_PACRK_SP4_SHIFT                     (14U)
#define AIPS_PACRK_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP4_SHIFT)) & AIPS_PACRK_SP4_MASK)
#define AIPS_PACRK_TP3_MASK                      (0x10000U)
#define AIPS_PACRK_TP3_SHIFT                     (16U)
#define AIPS_PACRK_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP3_SHIFT)) & AIPS_PACRK_TP3_MASK)
#define AIPS_PACRK_WP3_MASK                      (0x20000U)
#define AIPS_PACRK_WP3_SHIFT                     (17U)
#define AIPS_PACRK_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP3_SHIFT)) & AIPS_PACRK_WP3_MASK)
#define AIPS_PACRK_SP3_MASK                      (0x40000U)
#define AIPS_PACRK_SP3_SHIFT                     (18U)
#define AIPS_PACRK_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP3_SHIFT)) & AIPS_PACRK_SP3_MASK)
#define AIPS_PACRK_TP2_MASK                      (0x100000U)
#define AIPS_PACRK_TP2_SHIFT                     (20U)
#define AIPS_PACRK_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP2_SHIFT)) & AIPS_PACRK_TP2_MASK)
#define AIPS_PACRK_WP2_MASK                      (0x200000U)
#define AIPS_PACRK_WP2_SHIFT                     (21U)
#define AIPS_PACRK_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP2_SHIFT)) & AIPS_PACRK_WP2_MASK)
#define AIPS_PACRK_SP2_MASK                      (0x400000U)
#define AIPS_PACRK_SP2_SHIFT                     (22U)
#define AIPS_PACRK_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP2_SHIFT)) & AIPS_PACRK_SP2_MASK)
#define AIPS_PACRK_TP1_MASK                      (0x1000000U)
#define AIPS_PACRK_TP1_SHIFT                     (24U)
#define AIPS_PACRK_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP1_SHIFT)) & AIPS_PACRK_TP1_MASK)
#define AIPS_PACRK_WP1_MASK                      (0x2000000U)
#define AIPS_PACRK_WP1_SHIFT                     (25U)
#define AIPS_PACRK_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP1_SHIFT)) & AIPS_PACRK_WP1_MASK)
#define AIPS_PACRK_SP1_MASK                      (0x4000000U)
#define AIPS_PACRK_SP1_SHIFT                     (26U)
#define AIPS_PACRK_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP1_SHIFT)) & AIPS_PACRK_SP1_MASK)
#define AIPS_PACRK_TP0_MASK                      (0x10000000U)
#define AIPS_PACRK_TP0_SHIFT                     (28U)
#define AIPS_PACRK_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_TP0_SHIFT)) & AIPS_PACRK_TP0_MASK)
#define AIPS_PACRK_WP0_MASK                      (0x20000000U)
#define AIPS_PACRK_WP0_SHIFT                     (29U)
#define AIPS_PACRK_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_WP0_SHIFT)) & AIPS_PACRK_WP0_MASK)
#define AIPS_PACRK_SP0_MASK                      (0x40000000U)
#define AIPS_PACRK_SP0_SHIFT                     (30U)
#define AIPS_PACRK_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRK_SP0_SHIFT)) & AIPS_PACRK_SP0_MASK)

/*! @name PACRL - Peripheral Access Control Register */
#define AIPS_PACRL_TP7_MASK                      (0x1U)
#define AIPS_PACRL_TP7_SHIFT                     (0U)
#define AIPS_PACRL_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP7_SHIFT)) & AIPS_PACRL_TP7_MASK)
#define AIPS_PACRL_WP7_MASK                      (0x2U)
#define AIPS_PACRL_WP7_SHIFT                     (1U)
#define AIPS_PACRL_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP7_SHIFT)) & AIPS_PACRL_WP7_MASK)
#define AIPS_PACRL_SP7_MASK                      (0x4U)
#define AIPS_PACRL_SP7_SHIFT                     (2U)
#define AIPS_PACRL_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP7_SHIFT)) & AIPS_PACRL_SP7_MASK)
#define AIPS_PACRL_TP6_MASK                      (0x10U)
#define AIPS_PACRL_TP6_SHIFT                     (4U)
#define AIPS_PACRL_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP6_SHIFT)) & AIPS_PACRL_TP6_MASK)
#define AIPS_PACRL_WP6_MASK                      (0x20U)
#define AIPS_PACRL_WP6_SHIFT                     (5U)
#define AIPS_PACRL_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP6_SHIFT)) & AIPS_PACRL_WP6_MASK)
#define AIPS_PACRL_SP6_MASK                      (0x40U)
#define AIPS_PACRL_SP6_SHIFT                     (6U)
#define AIPS_PACRL_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP6_SHIFT)) & AIPS_PACRL_SP6_MASK)
#define AIPS_PACRL_TP5_MASK                      (0x100U)
#define AIPS_PACRL_TP5_SHIFT                     (8U)
#define AIPS_PACRL_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP5_SHIFT)) & AIPS_PACRL_TP5_MASK)
#define AIPS_PACRL_WP5_MASK                      (0x200U)
#define AIPS_PACRL_WP5_SHIFT                     (9U)
#define AIPS_PACRL_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP5_SHIFT)) & AIPS_PACRL_WP5_MASK)
#define AIPS_PACRL_SP5_MASK                      (0x400U)
#define AIPS_PACRL_SP5_SHIFT                     (10U)
#define AIPS_PACRL_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP5_SHIFT)) & AIPS_PACRL_SP5_MASK)
#define AIPS_PACRL_TP4_MASK                      (0x1000U)
#define AIPS_PACRL_TP4_SHIFT                     (12U)
#define AIPS_PACRL_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP4_SHIFT)) & AIPS_PACRL_TP4_MASK)
#define AIPS_PACRL_WP4_MASK                      (0x2000U)
#define AIPS_PACRL_WP4_SHIFT                     (13U)
#define AIPS_PACRL_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP4_SHIFT)) & AIPS_PACRL_WP4_MASK)
#define AIPS_PACRL_SP4_MASK                      (0x4000U)
#define AIPS_PACRL_SP4_SHIFT                     (14U)
#define AIPS_PACRL_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP4_SHIFT)) & AIPS_PACRL_SP4_MASK)
#define AIPS_PACRL_TP3_MASK                      (0x10000U)
#define AIPS_PACRL_TP3_SHIFT                     (16U)
#define AIPS_PACRL_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP3_SHIFT)) & AIPS_PACRL_TP3_MASK)
#define AIPS_PACRL_WP3_MASK                      (0x20000U)
#define AIPS_PACRL_WP3_SHIFT                     (17U)
#define AIPS_PACRL_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP3_SHIFT)) & AIPS_PACRL_WP3_MASK)
#define AIPS_PACRL_SP3_MASK                      (0x40000U)
#define AIPS_PACRL_SP3_SHIFT                     (18U)
#define AIPS_PACRL_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP3_SHIFT)) & AIPS_PACRL_SP3_MASK)
#define AIPS_PACRL_TP2_MASK                      (0x100000U)
#define AIPS_PACRL_TP2_SHIFT                     (20U)
#define AIPS_PACRL_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP2_SHIFT)) & AIPS_PACRL_TP2_MASK)
#define AIPS_PACRL_WP2_MASK                      (0x200000U)
#define AIPS_PACRL_WP2_SHIFT                     (21U)
#define AIPS_PACRL_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP2_SHIFT)) & AIPS_PACRL_WP2_MASK)
#define AIPS_PACRL_SP2_MASK                      (0x400000U)
#define AIPS_PACRL_SP2_SHIFT                     (22U)
#define AIPS_PACRL_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP2_SHIFT)) & AIPS_PACRL_SP2_MASK)
#define AIPS_PACRL_TP1_MASK                      (0x1000000U)
#define AIPS_PACRL_TP1_SHIFT                     (24U)
#define AIPS_PACRL_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP1_SHIFT)) & AIPS_PACRL_TP1_MASK)
#define AIPS_PACRL_WP1_MASK                      (0x2000000U)
#define AIPS_PACRL_WP1_SHIFT                     (25U)
#define AIPS_PACRL_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP1_SHIFT)) & AIPS_PACRL_WP1_MASK)
#define AIPS_PACRL_SP1_MASK                      (0x4000000U)
#define AIPS_PACRL_SP1_SHIFT                     (26U)
#define AIPS_PACRL_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP1_SHIFT)) & AIPS_PACRL_SP1_MASK)
#define AIPS_PACRL_TP0_MASK                      (0x10000000U)
#define AIPS_PACRL_TP0_SHIFT                     (28U)
#define AIPS_PACRL_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_TP0_SHIFT)) & AIPS_PACRL_TP0_MASK)
#define AIPS_PACRL_WP0_MASK                      (0x20000000U)
#define AIPS_PACRL_WP0_SHIFT                     (29U)
#define AIPS_PACRL_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_WP0_SHIFT)) & AIPS_PACRL_WP0_MASK)
#define AIPS_PACRL_SP0_MASK                      (0x40000000U)
#define AIPS_PACRL_SP0_SHIFT                     (30U)
#define AIPS_PACRL_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRL_SP0_SHIFT)) & AIPS_PACRL_SP0_MASK)

/*! @name PACRM - Peripheral Access Control Register */
#define AIPS_PACRM_TP7_MASK                      (0x1U)
#define AIPS_PACRM_TP7_SHIFT                     (0U)
#define AIPS_PACRM_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP7_SHIFT)) & AIPS_PACRM_TP7_MASK)
#define AIPS_PACRM_WP7_MASK                      (0x2U)
#define AIPS_PACRM_WP7_SHIFT                     (1U)
#define AIPS_PACRM_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP7_SHIFT)) & AIPS_PACRM_WP7_MASK)
#define AIPS_PACRM_SP7_MASK                      (0x4U)
#define AIPS_PACRM_SP7_SHIFT                     (2U)
#define AIPS_PACRM_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP7_SHIFT)) & AIPS_PACRM_SP7_MASK)
#define AIPS_PACRM_TP6_MASK                      (0x10U)
#define AIPS_PACRM_TP6_SHIFT                     (4U)
#define AIPS_PACRM_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP6_SHIFT)) & AIPS_PACRM_TP6_MASK)
#define AIPS_PACRM_WP6_MASK                      (0x20U)
#define AIPS_PACRM_WP6_SHIFT                     (5U)
#define AIPS_PACRM_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP6_SHIFT)) & AIPS_PACRM_WP6_MASK)
#define AIPS_PACRM_SP6_MASK                      (0x40U)
#define AIPS_PACRM_SP6_SHIFT                     (6U)
#define AIPS_PACRM_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP6_SHIFT)) & AIPS_PACRM_SP6_MASK)
#define AIPS_PACRM_TP5_MASK                      (0x100U)
#define AIPS_PACRM_TP5_SHIFT                     (8U)
#define AIPS_PACRM_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP5_SHIFT)) & AIPS_PACRM_TP5_MASK)
#define AIPS_PACRM_WP5_MASK                      (0x200U)
#define AIPS_PACRM_WP5_SHIFT                     (9U)
#define AIPS_PACRM_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP5_SHIFT)) & AIPS_PACRM_WP5_MASK)
#define AIPS_PACRM_SP5_MASK                      (0x400U)
#define AIPS_PACRM_SP5_SHIFT                     (10U)
#define AIPS_PACRM_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP5_SHIFT)) & AIPS_PACRM_SP5_MASK)
#define AIPS_PACRM_TP4_MASK                      (0x1000U)
#define AIPS_PACRM_TP4_SHIFT                     (12U)
#define AIPS_PACRM_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP4_SHIFT)) & AIPS_PACRM_TP4_MASK)
#define AIPS_PACRM_WP4_MASK                      (0x2000U)
#define AIPS_PACRM_WP4_SHIFT                     (13U)
#define AIPS_PACRM_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP4_SHIFT)) & AIPS_PACRM_WP4_MASK)
#define AIPS_PACRM_SP4_MASK                      (0x4000U)
#define AIPS_PACRM_SP4_SHIFT                     (14U)
#define AIPS_PACRM_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP4_SHIFT)) & AIPS_PACRM_SP4_MASK)
#define AIPS_PACRM_TP3_MASK                      (0x10000U)
#define AIPS_PACRM_TP3_SHIFT                     (16U)
#define AIPS_PACRM_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP3_SHIFT)) & AIPS_PACRM_TP3_MASK)
#define AIPS_PACRM_WP3_MASK                      (0x20000U)
#define AIPS_PACRM_WP3_SHIFT                     (17U)
#define AIPS_PACRM_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP3_SHIFT)) & AIPS_PACRM_WP3_MASK)
#define AIPS_PACRM_SP3_MASK                      (0x40000U)
#define AIPS_PACRM_SP3_SHIFT                     (18U)
#define AIPS_PACRM_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP3_SHIFT)) & AIPS_PACRM_SP3_MASK)
#define AIPS_PACRM_TP2_MASK                      (0x100000U)
#define AIPS_PACRM_TP2_SHIFT                     (20U)
#define AIPS_PACRM_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP2_SHIFT)) & AIPS_PACRM_TP2_MASK)
#define AIPS_PACRM_WP2_MASK                      (0x200000U)
#define AIPS_PACRM_WP2_SHIFT                     (21U)
#define AIPS_PACRM_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP2_SHIFT)) & AIPS_PACRM_WP2_MASK)
#define AIPS_PACRM_SP2_MASK                      (0x400000U)
#define AIPS_PACRM_SP2_SHIFT                     (22U)
#define AIPS_PACRM_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP2_SHIFT)) & AIPS_PACRM_SP2_MASK)
#define AIPS_PACRM_TP1_MASK                      (0x1000000U)
#define AIPS_PACRM_TP1_SHIFT                     (24U)
#define AIPS_PACRM_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP1_SHIFT)) & AIPS_PACRM_TP1_MASK)
#define AIPS_PACRM_WP1_MASK                      (0x2000000U)
#define AIPS_PACRM_WP1_SHIFT                     (25U)
#define AIPS_PACRM_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP1_SHIFT)) & AIPS_PACRM_WP1_MASK)
#define AIPS_PACRM_SP1_MASK                      (0x4000000U)
#define AIPS_PACRM_SP1_SHIFT                     (26U)
#define AIPS_PACRM_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP1_SHIFT)) & AIPS_PACRM_SP1_MASK)
#define AIPS_PACRM_TP0_MASK                      (0x10000000U)
#define AIPS_PACRM_TP0_SHIFT                     (28U)
#define AIPS_PACRM_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_TP0_SHIFT)) & AIPS_PACRM_TP0_MASK)
#define AIPS_PACRM_WP0_MASK                      (0x20000000U)
#define AIPS_PACRM_WP0_SHIFT                     (29U)
#define AIPS_PACRM_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_WP0_SHIFT)) & AIPS_PACRM_WP0_MASK)
#define AIPS_PACRM_SP0_MASK                      (0x40000000U)
#define AIPS_PACRM_SP0_SHIFT                     (30U)
#define AIPS_PACRM_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRM_SP0_SHIFT)) & AIPS_PACRM_SP0_MASK)

/*! @name PACRN - Peripheral Access Control Register */
#define AIPS_PACRN_TP7_MASK                      (0x1U)
#define AIPS_PACRN_TP7_SHIFT                     (0U)
#define AIPS_PACRN_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP7_SHIFT)) & AIPS_PACRN_TP7_MASK)
#define AIPS_PACRN_WP7_MASK                      (0x2U)
#define AIPS_PACRN_WP7_SHIFT                     (1U)
#define AIPS_PACRN_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP7_SHIFT)) & AIPS_PACRN_WP7_MASK)
#define AIPS_PACRN_SP7_MASK                      (0x4U)
#define AIPS_PACRN_SP7_SHIFT                     (2U)
#define AIPS_PACRN_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP7_SHIFT)) & AIPS_PACRN_SP7_MASK)
#define AIPS_PACRN_TP6_MASK                      (0x10U)
#define AIPS_PACRN_TP6_SHIFT                     (4U)
#define AIPS_PACRN_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP6_SHIFT)) & AIPS_PACRN_TP6_MASK)
#define AIPS_PACRN_WP6_MASK                      (0x20U)
#define AIPS_PACRN_WP6_SHIFT                     (5U)
#define AIPS_PACRN_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP6_SHIFT)) & AIPS_PACRN_WP6_MASK)
#define AIPS_PACRN_SP6_MASK                      (0x40U)
#define AIPS_PACRN_SP6_SHIFT                     (6U)
#define AIPS_PACRN_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP6_SHIFT)) & AIPS_PACRN_SP6_MASK)
#define AIPS_PACRN_TP5_MASK                      (0x100U)
#define AIPS_PACRN_TP5_SHIFT                     (8U)
#define AIPS_PACRN_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP5_SHIFT)) & AIPS_PACRN_TP5_MASK)
#define AIPS_PACRN_WP5_MASK                      (0x200U)
#define AIPS_PACRN_WP5_SHIFT                     (9U)
#define AIPS_PACRN_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP5_SHIFT)) & AIPS_PACRN_WP5_MASK)
#define AIPS_PACRN_SP5_MASK                      (0x400U)
#define AIPS_PACRN_SP5_SHIFT                     (10U)
#define AIPS_PACRN_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP5_SHIFT)) & AIPS_PACRN_SP5_MASK)
#define AIPS_PACRN_TP4_MASK                      (0x1000U)
#define AIPS_PACRN_TP4_SHIFT                     (12U)
#define AIPS_PACRN_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP4_SHIFT)) & AIPS_PACRN_TP4_MASK)
#define AIPS_PACRN_WP4_MASK                      (0x2000U)
#define AIPS_PACRN_WP4_SHIFT                     (13U)
#define AIPS_PACRN_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP4_SHIFT)) & AIPS_PACRN_WP4_MASK)
#define AIPS_PACRN_SP4_MASK                      (0x4000U)
#define AIPS_PACRN_SP4_SHIFT                     (14U)
#define AIPS_PACRN_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP4_SHIFT)) & AIPS_PACRN_SP4_MASK)
#define AIPS_PACRN_TP3_MASK                      (0x10000U)
#define AIPS_PACRN_TP3_SHIFT                     (16U)
#define AIPS_PACRN_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP3_SHIFT)) & AIPS_PACRN_TP3_MASK)
#define AIPS_PACRN_WP3_MASK                      (0x20000U)
#define AIPS_PACRN_WP3_SHIFT                     (17U)
#define AIPS_PACRN_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP3_SHIFT)) & AIPS_PACRN_WP3_MASK)
#define AIPS_PACRN_SP3_MASK                      (0x40000U)
#define AIPS_PACRN_SP3_SHIFT                     (18U)
#define AIPS_PACRN_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP3_SHIFT)) & AIPS_PACRN_SP3_MASK)
#define AIPS_PACRN_TP2_MASK                      (0x100000U)
#define AIPS_PACRN_TP2_SHIFT                     (20U)
#define AIPS_PACRN_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP2_SHIFT)) & AIPS_PACRN_TP2_MASK)
#define AIPS_PACRN_WP2_MASK                      (0x200000U)
#define AIPS_PACRN_WP2_SHIFT                     (21U)
#define AIPS_PACRN_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP2_SHIFT)) & AIPS_PACRN_WP2_MASK)
#define AIPS_PACRN_SP2_MASK                      (0x400000U)
#define AIPS_PACRN_SP2_SHIFT                     (22U)
#define AIPS_PACRN_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP2_SHIFT)) & AIPS_PACRN_SP2_MASK)
#define AIPS_PACRN_TP1_MASK                      (0x1000000U)
#define AIPS_PACRN_TP1_SHIFT                     (24U)
#define AIPS_PACRN_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP1_SHIFT)) & AIPS_PACRN_TP1_MASK)
#define AIPS_PACRN_WP1_MASK                      (0x2000000U)
#define AIPS_PACRN_WP1_SHIFT                     (25U)
#define AIPS_PACRN_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP1_SHIFT)) & AIPS_PACRN_WP1_MASK)
#define AIPS_PACRN_SP1_MASK                      (0x4000000U)
#define AIPS_PACRN_SP1_SHIFT                     (26U)
#define AIPS_PACRN_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP1_SHIFT)) & AIPS_PACRN_SP1_MASK)
#define AIPS_PACRN_TP0_MASK                      (0x10000000U)
#define AIPS_PACRN_TP0_SHIFT                     (28U)
#define AIPS_PACRN_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_TP0_SHIFT)) & AIPS_PACRN_TP0_MASK)
#define AIPS_PACRN_WP0_MASK                      (0x20000000U)
#define AIPS_PACRN_WP0_SHIFT                     (29U)
#define AIPS_PACRN_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_WP0_SHIFT)) & AIPS_PACRN_WP0_MASK)
#define AIPS_PACRN_SP0_MASK                      (0x40000000U)
#define AIPS_PACRN_SP0_SHIFT                     (30U)
#define AIPS_PACRN_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRN_SP0_SHIFT)) & AIPS_PACRN_SP0_MASK)

/*! @name PACRO - Peripheral Access Control Register */
#define AIPS_PACRO_TP7_MASK                      (0x1U)
#define AIPS_PACRO_TP7_SHIFT                     (0U)
#define AIPS_PACRO_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP7_SHIFT)) & AIPS_PACRO_TP7_MASK)
#define AIPS_PACRO_WP7_MASK                      (0x2U)
#define AIPS_PACRO_WP7_SHIFT                     (1U)
#define AIPS_PACRO_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP7_SHIFT)) & AIPS_PACRO_WP7_MASK)
#define AIPS_PACRO_SP7_MASK                      (0x4U)
#define AIPS_PACRO_SP7_SHIFT                     (2U)
#define AIPS_PACRO_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP7_SHIFT)) & AIPS_PACRO_SP7_MASK)
#define AIPS_PACRO_TP6_MASK                      (0x10U)
#define AIPS_PACRO_TP6_SHIFT                     (4U)
#define AIPS_PACRO_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP6_SHIFT)) & AIPS_PACRO_TP6_MASK)
#define AIPS_PACRO_WP6_MASK                      (0x20U)
#define AIPS_PACRO_WP6_SHIFT                     (5U)
#define AIPS_PACRO_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP6_SHIFT)) & AIPS_PACRO_WP6_MASK)
#define AIPS_PACRO_SP6_MASK                      (0x40U)
#define AIPS_PACRO_SP6_SHIFT                     (6U)
#define AIPS_PACRO_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP6_SHIFT)) & AIPS_PACRO_SP6_MASK)
#define AIPS_PACRO_TP5_MASK                      (0x100U)
#define AIPS_PACRO_TP5_SHIFT                     (8U)
#define AIPS_PACRO_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP5_SHIFT)) & AIPS_PACRO_TP5_MASK)
#define AIPS_PACRO_WP5_MASK                      (0x200U)
#define AIPS_PACRO_WP5_SHIFT                     (9U)
#define AIPS_PACRO_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP5_SHIFT)) & AIPS_PACRO_WP5_MASK)
#define AIPS_PACRO_SP5_MASK                      (0x400U)
#define AIPS_PACRO_SP5_SHIFT                     (10U)
#define AIPS_PACRO_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP5_SHIFT)) & AIPS_PACRO_SP5_MASK)
#define AIPS_PACRO_TP4_MASK                      (0x1000U)
#define AIPS_PACRO_TP4_SHIFT                     (12U)
#define AIPS_PACRO_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP4_SHIFT)) & AIPS_PACRO_TP4_MASK)
#define AIPS_PACRO_WP4_MASK                      (0x2000U)
#define AIPS_PACRO_WP4_SHIFT                     (13U)
#define AIPS_PACRO_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP4_SHIFT)) & AIPS_PACRO_WP4_MASK)
#define AIPS_PACRO_SP4_MASK                      (0x4000U)
#define AIPS_PACRO_SP4_SHIFT                     (14U)
#define AIPS_PACRO_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP4_SHIFT)) & AIPS_PACRO_SP4_MASK)
#define AIPS_PACRO_TP3_MASK                      (0x10000U)
#define AIPS_PACRO_TP3_SHIFT                     (16U)
#define AIPS_PACRO_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP3_SHIFT)) & AIPS_PACRO_TP3_MASK)
#define AIPS_PACRO_WP3_MASK                      (0x20000U)
#define AIPS_PACRO_WP3_SHIFT                     (17U)
#define AIPS_PACRO_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP3_SHIFT)) & AIPS_PACRO_WP3_MASK)
#define AIPS_PACRO_SP3_MASK                      (0x40000U)
#define AIPS_PACRO_SP3_SHIFT                     (18U)
#define AIPS_PACRO_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP3_SHIFT)) & AIPS_PACRO_SP3_MASK)
#define AIPS_PACRO_TP2_MASK                      (0x100000U)
#define AIPS_PACRO_TP2_SHIFT                     (20U)
#define AIPS_PACRO_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP2_SHIFT)) & AIPS_PACRO_TP2_MASK)
#define AIPS_PACRO_WP2_MASK                      (0x200000U)
#define AIPS_PACRO_WP2_SHIFT                     (21U)
#define AIPS_PACRO_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP2_SHIFT)) & AIPS_PACRO_WP2_MASK)
#define AIPS_PACRO_SP2_MASK                      (0x400000U)
#define AIPS_PACRO_SP2_SHIFT                     (22U)
#define AIPS_PACRO_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP2_SHIFT)) & AIPS_PACRO_SP2_MASK)
#define AIPS_PACRO_TP1_MASK                      (0x1000000U)
#define AIPS_PACRO_TP1_SHIFT                     (24U)
#define AIPS_PACRO_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP1_SHIFT)) & AIPS_PACRO_TP1_MASK)
#define AIPS_PACRO_WP1_MASK                      (0x2000000U)
#define AIPS_PACRO_WP1_SHIFT                     (25U)
#define AIPS_PACRO_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP1_SHIFT)) & AIPS_PACRO_WP1_MASK)
#define AIPS_PACRO_SP1_MASK                      (0x4000000U)
#define AIPS_PACRO_SP1_SHIFT                     (26U)
#define AIPS_PACRO_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP1_SHIFT)) & AIPS_PACRO_SP1_MASK)
#define AIPS_PACRO_TP0_MASK                      (0x10000000U)
#define AIPS_PACRO_TP0_SHIFT                     (28U)
#define AIPS_PACRO_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_TP0_SHIFT)) & AIPS_PACRO_TP0_MASK)
#define AIPS_PACRO_WP0_MASK                      (0x20000000U)
#define AIPS_PACRO_WP0_SHIFT                     (29U)
#define AIPS_PACRO_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_WP0_SHIFT)) & AIPS_PACRO_WP0_MASK)
#define AIPS_PACRO_SP0_MASK                      (0x40000000U)
#define AIPS_PACRO_SP0_SHIFT                     (30U)
#define AIPS_PACRO_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRO_SP0_SHIFT)) & AIPS_PACRO_SP0_MASK)

/*! @name PACRP - Peripheral Access Control Register */
#define AIPS_PACRP_TP7_MASK                      (0x1U)
#define AIPS_PACRP_TP7_SHIFT                     (0U)
#define AIPS_PACRP_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP7_SHIFT)) & AIPS_PACRP_TP7_MASK)
#define AIPS_PACRP_WP7_MASK                      (0x2U)
#define AIPS_PACRP_WP7_SHIFT                     (1U)
#define AIPS_PACRP_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP7_SHIFT)) & AIPS_PACRP_WP7_MASK)
#define AIPS_PACRP_SP7_MASK                      (0x4U)
#define AIPS_PACRP_SP7_SHIFT                     (2U)
#define AIPS_PACRP_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP7_SHIFT)) & AIPS_PACRP_SP7_MASK)
#define AIPS_PACRP_TP6_MASK                      (0x10U)
#define AIPS_PACRP_TP6_SHIFT                     (4U)
#define AIPS_PACRP_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP6_SHIFT)) & AIPS_PACRP_TP6_MASK)
#define AIPS_PACRP_WP6_MASK                      (0x20U)
#define AIPS_PACRP_WP6_SHIFT                     (5U)
#define AIPS_PACRP_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP6_SHIFT)) & AIPS_PACRP_WP6_MASK)
#define AIPS_PACRP_SP6_MASK                      (0x40U)
#define AIPS_PACRP_SP6_SHIFT                     (6U)
#define AIPS_PACRP_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP6_SHIFT)) & AIPS_PACRP_SP6_MASK)
#define AIPS_PACRP_TP5_MASK                      (0x100U)
#define AIPS_PACRP_TP5_SHIFT                     (8U)
#define AIPS_PACRP_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP5_SHIFT)) & AIPS_PACRP_TP5_MASK)
#define AIPS_PACRP_WP5_MASK                      (0x200U)
#define AIPS_PACRP_WP5_SHIFT                     (9U)
#define AIPS_PACRP_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP5_SHIFT)) & AIPS_PACRP_WP5_MASK)
#define AIPS_PACRP_SP5_MASK                      (0x400U)
#define AIPS_PACRP_SP5_SHIFT                     (10U)
#define AIPS_PACRP_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP5_SHIFT)) & AIPS_PACRP_SP5_MASK)
#define AIPS_PACRP_TP4_MASK                      (0x1000U)
#define AIPS_PACRP_TP4_SHIFT                     (12U)
#define AIPS_PACRP_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP4_SHIFT)) & AIPS_PACRP_TP4_MASK)
#define AIPS_PACRP_WP4_MASK                      (0x2000U)
#define AIPS_PACRP_WP4_SHIFT                     (13U)
#define AIPS_PACRP_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP4_SHIFT)) & AIPS_PACRP_WP4_MASK)
#define AIPS_PACRP_SP4_MASK                      (0x4000U)
#define AIPS_PACRP_SP4_SHIFT                     (14U)
#define AIPS_PACRP_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP4_SHIFT)) & AIPS_PACRP_SP4_MASK)
#define AIPS_PACRP_TP3_MASK                      (0x10000U)
#define AIPS_PACRP_TP3_SHIFT                     (16U)
#define AIPS_PACRP_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP3_SHIFT)) & AIPS_PACRP_TP3_MASK)
#define AIPS_PACRP_WP3_MASK                      (0x20000U)
#define AIPS_PACRP_WP3_SHIFT                     (17U)
#define AIPS_PACRP_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP3_SHIFT)) & AIPS_PACRP_WP3_MASK)
#define AIPS_PACRP_SP3_MASK                      (0x40000U)
#define AIPS_PACRP_SP3_SHIFT                     (18U)
#define AIPS_PACRP_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP3_SHIFT)) & AIPS_PACRP_SP3_MASK)
#define AIPS_PACRP_TP2_MASK                      (0x100000U)
#define AIPS_PACRP_TP2_SHIFT                     (20U)
#define AIPS_PACRP_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP2_SHIFT)) & AIPS_PACRP_TP2_MASK)
#define AIPS_PACRP_WP2_MASK                      (0x200000U)
#define AIPS_PACRP_WP2_SHIFT                     (21U)
#define AIPS_PACRP_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP2_SHIFT)) & AIPS_PACRP_WP2_MASK)
#define AIPS_PACRP_SP2_MASK                      (0x400000U)
#define AIPS_PACRP_SP2_SHIFT                     (22U)
#define AIPS_PACRP_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP2_SHIFT)) & AIPS_PACRP_SP2_MASK)
#define AIPS_PACRP_TP1_MASK                      (0x1000000U)
#define AIPS_PACRP_TP1_SHIFT                     (24U)
#define AIPS_PACRP_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP1_SHIFT)) & AIPS_PACRP_TP1_MASK)
#define AIPS_PACRP_WP1_MASK                      (0x2000000U)
#define AIPS_PACRP_WP1_SHIFT                     (25U)
#define AIPS_PACRP_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP1_SHIFT)) & AIPS_PACRP_WP1_MASK)
#define AIPS_PACRP_SP1_MASK                      (0x4000000U)
#define AIPS_PACRP_SP1_SHIFT                     (26U)
#define AIPS_PACRP_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP1_SHIFT)) & AIPS_PACRP_SP1_MASK)
#define AIPS_PACRP_TP0_MASK                      (0x10000000U)
#define AIPS_PACRP_TP0_SHIFT                     (28U)
#define AIPS_PACRP_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_TP0_SHIFT)) & AIPS_PACRP_TP0_MASK)
#define AIPS_PACRP_WP0_MASK                      (0x20000000U)
#define AIPS_PACRP_WP0_SHIFT                     (29U)
#define AIPS_PACRP_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_WP0_SHIFT)) & AIPS_PACRP_WP0_MASK)
#define AIPS_PACRP_SP0_MASK                      (0x40000000U)
#define AIPS_PACRP_SP0_SHIFT                     (30U)
#define AIPS_PACRP_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRP_SP0_SHIFT)) & AIPS_PACRP_SP0_MASK)


/*!
 * @}
 */ /* end of group AIPS_Register_Masks */


/* AIPS - Peripheral instance base addresses */
/** Peripheral AIPS0 base address */
#define AIPS0_BASE                               (0x40000000u)
/** Peripheral AIPS0 base pointer */
#define AIPS0                                    ((AIPS_Type *)AIPS0_BASE)
/** Peripheral AIPS1 base address */
#define AIPS1_BASE                               (0x40080000u)
/** Peripheral AIPS1 base pointer */
#define AIPS1                                    ((AIPS_Type *)AIPS1_BASE)
/** Array initializer of AIPS peripheral base addresses */
#define AIPS_BASE_ADDRS                          { AIPS0_BASE, AIPS1_BASE }
/** Array initializer of AIPS peripheral base pointers */
#define AIPS_BASE_PTRS                           { AIPS0, AIPS1 }

/*!
 * @}
 */ /* end of group AIPS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- AXBS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AXBS_Peripheral_Access_Layer AXBS Peripheral Access Layer
 * @{
 */

/** AXBS - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x100 */
    __IO uint32_t PRS;                               /**< Priority Registers Slave, array offset: 0x0, array step: 0x100 */
         uint8_t RESERVED_0[12];
    __IO uint32_t CRS;                               /**< Control Register, array offset: 0x10, array step: 0x100 */
         uint8_t RESERVED_1[236];
  } SLAVE[6];
       uint8_t RESERVED_0[512];
  __IO uint32_t MGPCR0;                            /**< Master General Purpose Control Register, offset: 0x800 */
       uint8_t RESERVED_1[252];
  __IO uint32_t MGPCR1;                            /**< Master General Purpose Control Register, offset: 0x900 */
       uint8_t RESERVED_2[252];
  __IO uint32_t MGPCR2;                            /**< Master General Purpose Control Register, offset: 0xA00 */
       uint8_t RESERVED_3[252];
  __IO uint32_t MGPCR3;                            /**< Master General Purpose Control Register, offset: 0xB00 */
       uint8_t RESERVED_4[252];
  __IO uint32_t MGPCR4;                            /**< Master General Purpose Control Register, offset: 0xC00 */
} AXBS_Type;

/* ----------------------------------------------------------------------------
   -- AXBS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AXBS_Register_Masks AXBS Register Masks
 * @{
 */

/*! @name PRS - Priority Registers Slave */
#define AXBS_PRS_M0_MASK                         (0x7U)
#define AXBS_PRS_M0_SHIFT                        (0U)
#define AXBS_PRS_M0(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_PRS_M0_SHIFT)) & AXBS_PRS_M0_MASK)
#define AXBS_PRS_M1_MASK                         (0x70U)
#define AXBS_PRS_M1_SHIFT                        (4U)
#define AXBS_PRS_M1(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_PRS_M1_SHIFT)) & AXBS_PRS_M1_MASK)
#define AXBS_PRS_M2_MASK                         (0x700U)
#define AXBS_PRS_M2_SHIFT                        (8U)
#define AXBS_PRS_M2(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_PRS_M2_SHIFT)) & AXBS_PRS_M2_MASK)
#define AXBS_PRS_M3_MASK                         (0x7000U)
#define AXBS_PRS_M3_SHIFT                        (12U)
#define AXBS_PRS_M3(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_PRS_M3_SHIFT)) & AXBS_PRS_M3_MASK)
#define AXBS_PRS_M4_MASK                         (0x70000U)
#define AXBS_PRS_M4_SHIFT                        (16U)
#define AXBS_PRS_M4(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_PRS_M4_SHIFT)) & AXBS_PRS_M4_MASK)

/* The count of AXBS_PRS */
#define AXBS_PRS_COUNT                           (6U)

/*! @name CRS - Control Register */
#define AXBS_CRS_PARK_MASK                       (0x7U)
#define AXBS_CRS_PARK_SHIFT                      (0U)
#define AXBS_CRS_PARK(x)                         (((uint32_t)(((uint32_t)(x)) << AXBS_CRS_PARK_SHIFT)) & AXBS_CRS_PARK_MASK)
#define AXBS_CRS_PCTL_MASK                       (0x30U)
#define AXBS_CRS_PCTL_SHIFT                      (4U)
#define AXBS_CRS_PCTL(x)                         (((uint32_t)(((uint32_t)(x)) << AXBS_CRS_PCTL_SHIFT)) & AXBS_CRS_PCTL_MASK)
#define AXBS_CRS_ARB_MASK                        (0x300U)
#define AXBS_CRS_ARB_SHIFT                       (8U)
#define AXBS_CRS_ARB(x)                          (((uint32_t)(((uint32_t)(x)) << AXBS_CRS_ARB_SHIFT)) & AXBS_CRS_ARB_MASK)
#define AXBS_CRS_HLP_MASK                        (0x40000000U)
#define AXBS_CRS_HLP_SHIFT                       (30U)
#define AXBS_CRS_HLP(x)                          (((uint32_t)(((uint32_t)(x)) << AXBS_CRS_HLP_SHIFT)) & AXBS_CRS_HLP_MASK)
#define AXBS_CRS_RO_MASK                         (0x80000000U)
#define AXBS_CRS_RO_SHIFT                        (31U)
#define AXBS_CRS_RO(x)                           (((uint32_t)(((uint32_t)(x)) << AXBS_CRS_RO_SHIFT)) & AXBS_CRS_RO_MASK)

/* The count of AXBS_CRS */
#define AXBS_CRS_COUNT                           (6U)

/*! @name MGPCR0 - Master General Purpose Control Register */
#define AXBS_MGPCR0_AULB_MASK                    (0x7U)
#define AXBS_MGPCR0_AULB_SHIFT                   (0U)
#define AXBS_MGPCR0_AULB(x)                      (((uint32_t)(((uint32_t)(x)) << AXBS_MGPCR0_AULB_SHIFT)) & AXBS_MGPCR0_AULB_MASK)

/*! @name MGPCR1 - Master General Purpose Control Register */
#define AXBS_MGPCR1_AULB_MASK                    (0x7U)
#define AXBS_MGPCR1_AULB_SHIFT                   (0U)
#define AXBS_MGPCR1_AULB(x)                      (((uint32_t)(((uint32_t)(x)) << AXBS_MGPCR1_AULB_SHIFT)) & AXBS_MGPCR1_AULB_MASK)

/*! @name MGPCR2 - Master General Purpose Control Register */
#define AXBS_MGPCR2_AULB_MASK                    (0x7U)
#define AXBS_MGPCR2_AULB_SHIFT                   (0U)
#define AXBS_MGPCR2_AULB(x)                      (((uint32_t)(((uint32_t)(x)) << AXBS_MGPCR2_AULB_SHIFT)) & AXBS_MGPCR2_AULB_MASK)

/*! @name MGPCR3 - Master General Purpose Control Register */
#define AXBS_MGPCR3_AULB_MASK                    (0x7U)
#define AXBS_MGPCR3_AULB_SHIFT                   (0U)
#define AXBS_MGPCR3_AULB(x)                      (((uint32_t)(((uint32_t)(x)) << AXBS_MGPCR3_AULB_SHIFT)) & AXBS_MGPCR3_AULB_MASK)

/*! @name MGPCR4 - Master General Purpose Control Register */
#define AXBS_MGPCR4_AULB_MASK                    (0x7U)
#define AXBS_MGPCR4_AULB_SHIFT                   (0U)
#define AXBS_MGPCR4_AULB(x)                      (((uint32_t)(((uint32_t)(x)) << AXBS_MGPCR4_AULB_SHIFT)) & AXBS_MGPCR4_AULB_MASK)


/*!
 * @}
 */ /* end of group AXBS_Register_Masks */


/* AXBS - Peripheral instance base addresses */
/** Peripheral AXBS base address */
#define AXBS_BASE                                (0x40004000u)
/** Peripheral AXBS base pointer */
#define AXBS                                     ((AXBS_Type *)AXBS_BASE)
/** Array initializer of AXBS peripheral base addresses */
#define AXBS_BASE_ADDRS                          { AXBS_BASE }
/** Array initializer of AXBS peripheral base pointers */
#define AXBS_BASE_PTRS                           { AXBS }

/*!
 * @}
 */ /* end of group AXBS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CAU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAU_Peripheral_Access_Layer CAU Peripheral Access Layer
 * @{
 */

/** CAU - Register Layout Typedef */
typedef struct {
  __O  uint32_t DIRECT[16];                        /**< Direct access register 0..Direct access register 15, array offset: 0x0, array step: 0x4 */
       uint8_t RESERVED_0[2048];
  __O  uint32_t LDR_CASR;                          /**< Status register - Load Register command, offset: 0x840 */
  __O  uint32_t LDR_CAA;                           /**< Accumulator register - Load Register command, offset: 0x844 */
  __O  uint32_t LDR_CA[9];                         /**< General Purpose Register 0 - Load Register command..General Purpose Register 8 - Load Register command, array offset: 0x848, array step: 0x4 */
       uint8_t RESERVED_1[20];
  __I  uint32_t STR_CASR;                          /**< Status register - Store Register command, offset: 0x880 */
  __I  uint32_t STR_CAA;                           /**< Accumulator register - Store Register command, offset: 0x884 */
  __I  uint32_t STR_CA[9];                         /**< General Purpose Register 0 - Store Register command..General Purpose Register 8 - Store Register command, array offset: 0x888, array step: 0x4 */
       uint8_t RESERVED_2[20];
  __O  uint32_t ADR_CASR;                          /**< Status register - Add Register command, offset: 0x8C0 */
  __O  uint32_t ADR_CAA;                           /**< Accumulator register - Add to register command, offset: 0x8C4 */
  __O  uint32_t ADR_CA[9];                         /**< General Purpose Register 0 - Add to register command..General Purpose Register 8 - Add to register command, array offset: 0x8C8, array step: 0x4 */
       uint8_t RESERVED_3[20];
  __O  uint32_t RADR_CASR;                         /**< Status register - Reverse and Add to Register command, offset: 0x900 */
  __O  uint32_t RADR_CAA;                          /**< Accumulator register - Reverse and Add to Register command, offset: 0x904 */
  __O  uint32_t RADR_CA[9];                        /**< General Purpose Register 0 - Reverse and Add to Register command..General Purpose Register 8 - Reverse and Add to Register command, array offset: 0x908, array step: 0x4 */
       uint8_t RESERVED_4[84];
  __O  uint32_t XOR_CASR;                          /**< Status register - Exclusive Or command, offset: 0x980 */
  __O  uint32_t XOR_CAA;                           /**< Accumulator register - Exclusive Or command, offset: 0x984 */
  __O  uint32_t XOR_CA[9];                         /**< General Purpose Register 0 - Exclusive Or command..General Purpose Register 8 - Exclusive Or command, array offset: 0x988, array step: 0x4 */
       uint8_t RESERVED_5[20];
  __O  uint32_t ROTL_CASR;                         /**< Status register - Rotate Left command, offset: 0x9C0 */
  __O  uint32_t ROTL_CAA;                          /**< Accumulator register - Rotate Left command, offset: 0x9C4 */
  __O  uint32_t ROTL_CA[9];                        /**< General Purpose Register 0 - Rotate Left command..General Purpose Register 8 - Rotate Left command, array offset: 0x9C8, array step: 0x4 */
       uint8_t RESERVED_6[276];
  __O  uint32_t AESC_CASR;                         /**< Status register - AES Column Operation command, offset: 0xB00 */
  __O  uint32_t AESC_CAA;                          /**< Accumulator register - AES Column Operation command, offset: 0xB04 */
  __O  uint32_t AESC_CA[9];                        /**< General Purpose Register 0 - AES Column Operation command..General Purpose Register 8 - AES Column Operation command, array offset: 0xB08, array step: 0x4 */
       uint8_t RESERVED_7[20];
  __O  uint32_t AESIC_CASR;                        /**< Status register - AES Inverse Column Operation command, offset: 0xB40 */
  __O  uint32_t AESIC_CAA;                         /**< Accumulator register - AES Inverse Column Operation command, offset: 0xB44 */
  __O  uint32_t AESIC_CA[9];                       /**< General Purpose Register 0 - AES Inverse Column Operation command..General Purpose Register 8 - AES Inverse Column Operation command, array offset: 0xB48, array step: 0x4 */
} CAU_Type;

/* ----------------------------------------------------------------------------
   -- CAU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAU_Register_Masks CAU Register Masks
 * @{
 */

/*! @name DIRECT - Direct access register 0..Direct access register 15 */
#define CAU_DIRECT_CAU_DIRECT0_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT0_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT0(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT0_SHIFT)) & CAU_DIRECT_CAU_DIRECT0_MASK)
#define CAU_DIRECT_CAU_DIRECT1_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT1_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT1(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT1_SHIFT)) & CAU_DIRECT_CAU_DIRECT1_MASK)
#define CAU_DIRECT_CAU_DIRECT2_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT2_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT2(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT2_SHIFT)) & CAU_DIRECT_CAU_DIRECT2_MASK)
#define CAU_DIRECT_CAU_DIRECT3_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT3_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT3(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT3_SHIFT)) & CAU_DIRECT_CAU_DIRECT3_MASK)
#define CAU_DIRECT_CAU_DIRECT4_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT4_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT4(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT4_SHIFT)) & CAU_DIRECT_CAU_DIRECT4_MASK)
#define CAU_DIRECT_CAU_DIRECT5_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT5_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT5(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT5_SHIFT)) & CAU_DIRECT_CAU_DIRECT5_MASK)
#define CAU_DIRECT_CAU_DIRECT6_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT6_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT6(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT6_SHIFT)) & CAU_DIRECT_CAU_DIRECT6_MASK)
#define CAU_DIRECT_CAU_DIRECT7_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT7_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT7(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT7_SHIFT)) & CAU_DIRECT_CAU_DIRECT7_MASK)
#define CAU_DIRECT_CAU_DIRECT8_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT8_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT8(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT8_SHIFT)) & CAU_DIRECT_CAU_DIRECT8_MASK)
#define CAU_DIRECT_CAU_DIRECT9_MASK              (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT9_SHIFT             (0U)
#define CAU_DIRECT_CAU_DIRECT9(x)                (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT9_SHIFT)) & CAU_DIRECT_CAU_DIRECT9_MASK)
#define CAU_DIRECT_CAU_DIRECT10_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT10_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT10(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT10_SHIFT)) & CAU_DIRECT_CAU_DIRECT10_MASK)
#define CAU_DIRECT_CAU_DIRECT11_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT11_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT11(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT11_SHIFT)) & CAU_DIRECT_CAU_DIRECT11_MASK)
#define CAU_DIRECT_CAU_DIRECT12_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT12_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT12(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT12_SHIFT)) & CAU_DIRECT_CAU_DIRECT12_MASK)
#define CAU_DIRECT_CAU_DIRECT13_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT13_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT13(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT13_SHIFT)) & CAU_DIRECT_CAU_DIRECT13_MASK)
#define CAU_DIRECT_CAU_DIRECT14_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT14_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT14(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT14_SHIFT)) & CAU_DIRECT_CAU_DIRECT14_MASK)
#define CAU_DIRECT_CAU_DIRECT15_MASK             (0xFFFFFFFFU)
#define CAU_DIRECT_CAU_DIRECT15_SHIFT            (0U)
#define CAU_DIRECT_CAU_DIRECT15(x)               (((uint32_t)(((uint32_t)(x)) << CAU_DIRECT_CAU_DIRECT15_SHIFT)) & CAU_DIRECT_CAU_DIRECT15_MASK)

/* The count of CAU_DIRECT */
#define CAU_DIRECT_COUNT                         (16U)

/*! @name LDR_CASR - Status register - Load Register command */
#define CAU_LDR_CASR_IC_MASK                     (0x1U)
#define CAU_LDR_CASR_IC_SHIFT                    (0U)
#define CAU_LDR_CASR_IC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CASR_IC_SHIFT)) & CAU_LDR_CASR_IC_MASK)
#define CAU_LDR_CASR_DPE_MASK                    (0x2U)
#define CAU_LDR_CASR_DPE_SHIFT                   (1U)
#define CAU_LDR_CASR_DPE(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CASR_DPE_SHIFT)) & CAU_LDR_CASR_DPE_MASK)
#define CAU_LDR_CASR_VER_MASK                    (0xF0000000U)
#define CAU_LDR_CASR_VER_SHIFT                   (28U)
#define CAU_LDR_CASR_VER(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CASR_VER_SHIFT)) & CAU_LDR_CASR_VER_MASK)

/*! @name LDR_CAA - Accumulator register - Load Register command */
#define CAU_LDR_CAA_ACC_MASK                     (0xFFFFFFFFU)
#define CAU_LDR_CAA_ACC_SHIFT                    (0U)
#define CAU_LDR_CAA_ACC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CAA_ACC_SHIFT)) & CAU_LDR_CAA_ACC_MASK)

/*! @name LDR_CA - General Purpose Register 0 - Load Register command..General Purpose Register 8 - Load Register command */
#define CAU_LDR_CA_CA0_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA0_SHIFT                     (0U)
#define CAU_LDR_CA_CA0(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA0_SHIFT)) & CAU_LDR_CA_CA0_MASK)
#define CAU_LDR_CA_CA1_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA1_SHIFT                     (0U)
#define CAU_LDR_CA_CA1(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA1_SHIFT)) & CAU_LDR_CA_CA1_MASK)
#define CAU_LDR_CA_CA2_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA2_SHIFT                     (0U)
#define CAU_LDR_CA_CA2(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA2_SHIFT)) & CAU_LDR_CA_CA2_MASK)
#define CAU_LDR_CA_CA3_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA3_SHIFT                     (0U)
#define CAU_LDR_CA_CA3(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA3_SHIFT)) & CAU_LDR_CA_CA3_MASK)
#define CAU_LDR_CA_CA4_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA4_SHIFT                     (0U)
#define CAU_LDR_CA_CA4(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA4_SHIFT)) & CAU_LDR_CA_CA4_MASK)
#define CAU_LDR_CA_CA5_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA5_SHIFT                     (0U)
#define CAU_LDR_CA_CA5(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA5_SHIFT)) & CAU_LDR_CA_CA5_MASK)
#define CAU_LDR_CA_CA6_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA6_SHIFT                     (0U)
#define CAU_LDR_CA_CA6(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA6_SHIFT)) & CAU_LDR_CA_CA6_MASK)
#define CAU_LDR_CA_CA7_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA7_SHIFT                     (0U)
#define CAU_LDR_CA_CA7(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA7_SHIFT)) & CAU_LDR_CA_CA7_MASK)
#define CAU_LDR_CA_CA8_MASK                      (0xFFFFFFFFU)
#define CAU_LDR_CA_CA8_SHIFT                     (0U)
#define CAU_LDR_CA_CA8(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_LDR_CA_CA8_SHIFT)) & CAU_LDR_CA_CA8_MASK)

/* The count of CAU_LDR_CA */
#define CAU_LDR_CA_COUNT                         (9U)

/*! @name STR_CASR - Status register - Store Register command */
#define CAU_STR_CASR_IC_MASK                     (0x1U)
#define CAU_STR_CASR_IC_SHIFT                    (0U)
#define CAU_STR_CASR_IC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_STR_CASR_IC_SHIFT)) & CAU_STR_CASR_IC_MASK)
#define CAU_STR_CASR_DPE_MASK                    (0x2U)
#define CAU_STR_CASR_DPE_SHIFT                   (1U)
#define CAU_STR_CASR_DPE(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_STR_CASR_DPE_SHIFT)) & CAU_STR_CASR_DPE_MASK)
#define CAU_STR_CASR_VER_MASK                    (0xF0000000U)
#define CAU_STR_CASR_VER_SHIFT                   (28U)
#define CAU_STR_CASR_VER(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_STR_CASR_VER_SHIFT)) & CAU_STR_CASR_VER_MASK)

/*! @name STR_CAA - Accumulator register - Store Register command */
#define CAU_STR_CAA_ACC_MASK                     (0xFFFFFFFFU)
#define CAU_STR_CAA_ACC_SHIFT                    (0U)
#define CAU_STR_CAA_ACC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_STR_CAA_ACC_SHIFT)) & CAU_STR_CAA_ACC_MASK)

/*! @name STR_CA - General Purpose Register 0 - Store Register command..General Purpose Register 8 - Store Register command */
#define CAU_STR_CA_CA0_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA0_SHIFT                     (0U)
#define CAU_STR_CA_CA0(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA0_SHIFT)) & CAU_STR_CA_CA0_MASK)
#define CAU_STR_CA_CA1_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA1_SHIFT                     (0U)
#define CAU_STR_CA_CA1(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA1_SHIFT)) & CAU_STR_CA_CA1_MASK)
#define CAU_STR_CA_CA2_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA2_SHIFT                     (0U)
#define CAU_STR_CA_CA2(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA2_SHIFT)) & CAU_STR_CA_CA2_MASK)
#define CAU_STR_CA_CA3_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA3_SHIFT                     (0U)
#define CAU_STR_CA_CA3(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA3_SHIFT)) & CAU_STR_CA_CA3_MASK)
#define CAU_STR_CA_CA4_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA4_SHIFT                     (0U)
#define CAU_STR_CA_CA4(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA4_SHIFT)) & CAU_STR_CA_CA4_MASK)
#define CAU_STR_CA_CA5_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA5_SHIFT                     (0U)
#define CAU_STR_CA_CA5(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA5_SHIFT)) & CAU_STR_CA_CA5_MASK)
#define CAU_STR_CA_CA6_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA6_SHIFT                     (0U)
#define CAU_STR_CA_CA6(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA6_SHIFT)) & CAU_STR_CA_CA6_MASK)
#define CAU_STR_CA_CA7_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA7_SHIFT                     (0U)
#define CAU_STR_CA_CA7(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA7_SHIFT)) & CAU_STR_CA_CA7_MASK)
#define CAU_STR_CA_CA8_MASK                      (0xFFFFFFFFU)
#define CAU_STR_CA_CA8_SHIFT                     (0U)
#define CAU_STR_CA_CA8(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_STR_CA_CA8_SHIFT)) & CAU_STR_CA_CA8_MASK)

/* The count of CAU_STR_CA */
#define CAU_STR_CA_COUNT                         (9U)

/*! @name ADR_CASR - Status register - Add Register command */
#define CAU_ADR_CASR_IC_MASK                     (0x1U)
#define CAU_ADR_CASR_IC_SHIFT                    (0U)
#define CAU_ADR_CASR_IC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CASR_IC_SHIFT)) & CAU_ADR_CASR_IC_MASK)
#define CAU_ADR_CASR_DPE_MASK                    (0x2U)
#define CAU_ADR_CASR_DPE_SHIFT                   (1U)
#define CAU_ADR_CASR_DPE(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CASR_DPE_SHIFT)) & CAU_ADR_CASR_DPE_MASK)
#define CAU_ADR_CASR_VER_MASK                    (0xF0000000U)
#define CAU_ADR_CASR_VER_SHIFT                   (28U)
#define CAU_ADR_CASR_VER(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CASR_VER_SHIFT)) & CAU_ADR_CASR_VER_MASK)

/*! @name ADR_CAA - Accumulator register - Add to register command */
#define CAU_ADR_CAA_ACC_MASK                     (0xFFFFFFFFU)
#define CAU_ADR_CAA_ACC_SHIFT                    (0U)
#define CAU_ADR_CAA_ACC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CAA_ACC_SHIFT)) & CAU_ADR_CAA_ACC_MASK)

/*! @name ADR_CA - General Purpose Register 0 - Add to register command..General Purpose Register 8 - Add to register command */
#define CAU_ADR_CA_CA0_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA0_SHIFT                     (0U)
#define CAU_ADR_CA_CA0(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA0_SHIFT)) & CAU_ADR_CA_CA0_MASK)
#define CAU_ADR_CA_CA1_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA1_SHIFT                     (0U)
#define CAU_ADR_CA_CA1(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA1_SHIFT)) & CAU_ADR_CA_CA1_MASK)
#define CAU_ADR_CA_CA2_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA2_SHIFT                     (0U)
#define CAU_ADR_CA_CA2(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA2_SHIFT)) & CAU_ADR_CA_CA2_MASK)
#define CAU_ADR_CA_CA3_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA3_SHIFT                     (0U)
#define CAU_ADR_CA_CA3(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA3_SHIFT)) & CAU_ADR_CA_CA3_MASK)
#define CAU_ADR_CA_CA4_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA4_SHIFT                     (0U)
#define CAU_ADR_CA_CA4(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA4_SHIFT)) & CAU_ADR_CA_CA4_MASK)
#define CAU_ADR_CA_CA5_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA5_SHIFT                     (0U)
#define CAU_ADR_CA_CA5(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA5_SHIFT)) & CAU_ADR_CA_CA5_MASK)
#define CAU_ADR_CA_CA6_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA6_SHIFT                     (0U)
#define CAU_ADR_CA_CA6(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA6_SHIFT)) & CAU_ADR_CA_CA6_MASK)
#define CAU_ADR_CA_CA7_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA7_SHIFT                     (0U)
#define CAU_ADR_CA_CA7(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA7_SHIFT)) & CAU_ADR_CA_CA7_MASK)
#define CAU_ADR_CA_CA8_MASK                      (0xFFFFFFFFU)
#define CAU_ADR_CA_CA8_SHIFT                     (0U)
#define CAU_ADR_CA_CA8(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_ADR_CA_CA8_SHIFT)) & CAU_ADR_CA_CA8_MASK)

/* The count of CAU_ADR_CA */
#define CAU_ADR_CA_COUNT                         (9U)

/*! @name RADR_CASR - Status register - Reverse and Add to Register command */
#define CAU_RADR_CASR_IC_MASK                    (0x1U)
#define CAU_RADR_CASR_IC_SHIFT                   (0U)
#define CAU_RADR_CASR_IC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CASR_IC_SHIFT)) & CAU_RADR_CASR_IC_MASK)
#define CAU_RADR_CASR_DPE_MASK                   (0x2U)
#define CAU_RADR_CASR_DPE_SHIFT                  (1U)
#define CAU_RADR_CASR_DPE(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CASR_DPE_SHIFT)) & CAU_RADR_CASR_DPE_MASK)
#define CAU_RADR_CASR_VER_MASK                   (0xF0000000U)
#define CAU_RADR_CASR_VER_SHIFT                  (28U)
#define CAU_RADR_CASR_VER(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CASR_VER_SHIFT)) & CAU_RADR_CASR_VER_MASK)

/*! @name RADR_CAA - Accumulator register - Reverse and Add to Register command */
#define CAU_RADR_CAA_ACC_MASK                    (0xFFFFFFFFU)
#define CAU_RADR_CAA_ACC_SHIFT                   (0U)
#define CAU_RADR_CAA_ACC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CAA_ACC_SHIFT)) & CAU_RADR_CAA_ACC_MASK)

/*! @name RADR_CA - General Purpose Register 0 - Reverse and Add to Register command..General Purpose Register 8 - Reverse and Add to Register command */
#define CAU_RADR_CA_CA0_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA0_SHIFT                    (0U)
#define CAU_RADR_CA_CA0(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA0_SHIFT)) & CAU_RADR_CA_CA0_MASK)
#define CAU_RADR_CA_CA1_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA1_SHIFT                    (0U)
#define CAU_RADR_CA_CA1(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA1_SHIFT)) & CAU_RADR_CA_CA1_MASK)
#define CAU_RADR_CA_CA2_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA2_SHIFT                    (0U)
#define CAU_RADR_CA_CA2(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA2_SHIFT)) & CAU_RADR_CA_CA2_MASK)
#define CAU_RADR_CA_CA3_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA3_SHIFT                    (0U)
#define CAU_RADR_CA_CA3(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA3_SHIFT)) & CAU_RADR_CA_CA3_MASK)
#define CAU_RADR_CA_CA4_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA4_SHIFT                    (0U)
#define CAU_RADR_CA_CA4(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA4_SHIFT)) & CAU_RADR_CA_CA4_MASK)
#define CAU_RADR_CA_CA5_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA5_SHIFT                    (0U)
#define CAU_RADR_CA_CA5(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA5_SHIFT)) & CAU_RADR_CA_CA5_MASK)
#define CAU_RADR_CA_CA6_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA6_SHIFT                    (0U)
#define CAU_RADR_CA_CA6(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA6_SHIFT)) & CAU_RADR_CA_CA6_MASK)
#define CAU_RADR_CA_CA7_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA7_SHIFT                    (0U)
#define CAU_RADR_CA_CA7(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA7_SHIFT)) & CAU_RADR_CA_CA7_MASK)
#define CAU_RADR_CA_CA8_MASK                     (0xFFFFFFFFU)
#define CAU_RADR_CA_CA8_SHIFT                    (0U)
#define CAU_RADR_CA_CA8(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_RADR_CA_CA8_SHIFT)) & CAU_RADR_CA_CA8_MASK)

/* The count of CAU_RADR_CA */
#define CAU_RADR_CA_COUNT                        (9U)

/*! @name XOR_CASR - Status register - Exclusive Or command */
#define CAU_XOR_CASR_IC_MASK                     (0x1U)
#define CAU_XOR_CASR_IC_SHIFT                    (0U)
#define CAU_XOR_CASR_IC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CASR_IC_SHIFT)) & CAU_XOR_CASR_IC_MASK)
#define CAU_XOR_CASR_DPE_MASK                    (0x2U)
#define CAU_XOR_CASR_DPE_SHIFT                   (1U)
#define CAU_XOR_CASR_DPE(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CASR_DPE_SHIFT)) & CAU_XOR_CASR_DPE_MASK)
#define CAU_XOR_CASR_VER_MASK                    (0xF0000000U)
#define CAU_XOR_CASR_VER_SHIFT                   (28U)
#define CAU_XOR_CASR_VER(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CASR_VER_SHIFT)) & CAU_XOR_CASR_VER_MASK)

/*! @name XOR_CAA - Accumulator register - Exclusive Or command */
#define CAU_XOR_CAA_ACC_MASK                     (0xFFFFFFFFU)
#define CAU_XOR_CAA_ACC_SHIFT                    (0U)
#define CAU_XOR_CAA_ACC(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CAA_ACC_SHIFT)) & CAU_XOR_CAA_ACC_MASK)

/*! @name XOR_CA - General Purpose Register 0 - Exclusive Or command..General Purpose Register 8 - Exclusive Or command */
#define CAU_XOR_CA_CA0_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA0_SHIFT                     (0U)
#define CAU_XOR_CA_CA0(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA0_SHIFT)) & CAU_XOR_CA_CA0_MASK)
#define CAU_XOR_CA_CA1_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA1_SHIFT                     (0U)
#define CAU_XOR_CA_CA1(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA1_SHIFT)) & CAU_XOR_CA_CA1_MASK)
#define CAU_XOR_CA_CA2_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA2_SHIFT                     (0U)
#define CAU_XOR_CA_CA2(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA2_SHIFT)) & CAU_XOR_CA_CA2_MASK)
#define CAU_XOR_CA_CA3_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA3_SHIFT                     (0U)
#define CAU_XOR_CA_CA3(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA3_SHIFT)) & CAU_XOR_CA_CA3_MASK)
#define CAU_XOR_CA_CA4_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA4_SHIFT                     (0U)
#define CAU_XOR_CA_CA4(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA4_SHIFT)) & CAU_XOR_CA_CA4_MASK)
#define CAU_XOR_CA_CA5_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA5_SHIFT                     (0U)
#define CAU_XOR_CA_CA5(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA5_SHIFT)) & CAU_XOR_CA_CA5_MASK)
#define CAU_XOR_CA_CA6_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA6_SHIFT                     (0U)
#define CAU_XOR_CA_CA6(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA6_SHIFT)) & CAU_XOR_CA_CA6_MASK)
#define CAU_XOR_CA_CA7_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA7_SHIFT                     (0U)
#define CAU_XOR_CA_CA7(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA7_SHIFT)) & CAU_XOR_CA_CA7_MASK)
#define CAU_XOR_CA_CA8_MASK                      (0xFFFFFFFFU)
#define CAU_XOR_CA_CA8_SHIFT                     (0U)
#define CAU_XOR_CA_CA8(x)                        (((uint32_t)(((uint32_t)(x)) << CAU_XOR_CA_CA8_SHIFT)) & CAU_XOR_CA_CA8_MASK)

/* The count of CAU_XOR_CA */
#define CAU_XOR_CA_COUNT                         (9U)

/*! @name ROTL_CASR - Status register - Rotate Left command */
#define CAU_ROTL_CASR_IC_MASK                    (0x1U)
#define CAU_ROTL_CASR_IC_SHIFT                   (0U)
#define CAU_ROTL_CASR_IC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CASR_IC_SHIFT)) & CAU_ROTL_CASR_IC_MASK)
#define CAU_ROTL_CASR_DPE_MASK                   (0x2U)
#define CAU_ROTL_CASR_DPE_SHIFT                  (1U)
#define CAU_ROTL_CASR_DPE(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CASR_DPE_SHIFT)) & CAU_ROTL_CASR_DPE_MASK)
#define CAU_ROTL_CASR_VER_MASK                   (0xF0000000U)
#define CAU_ROTL_CASR_VER_SHIFT                  (28U)
#define CAU_ROTL_CASR_VER(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CASR_VER_SHIFT)) & CAU_ROTL_CASR_VER_MASK)

/*! @name ROTL_CAA - Accumulator register - Rotate Left command */
#define CAU_ROTL_CAA_ACC_MASK                    (0xFFFFFFFFU)
#define CAU_ROTL_CAA_ACC_SHIFT                   (0U)
#define CAU_ROTL_CAA_ACC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CAA_ACC_SHIFT)) & CAU_ROTL_CAA_ACC_MASK)

/*! @name ROTL_CA - General Purpose Register 0 - Rotate Left command..General Purpose Register 8 - Rotate Left command */
#define CAU_ROTL_CA_CA0_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA0_SHIFT                    (0U)
#define CAU_ROTL_CA_CA0(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA0_SHIFT)) & CAU_ROTL_CA_CA0_MASK)
#define CAU_ROTL_CA_CA1_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA1_SHIFT                    (0U)
#define CAU_ROTL_CA_CA1(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA1_SHIFT)) & CAU_ROTL_CA_CA1_MASK)
#define CAU_ROTL_CA_CA2_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA2_SHIFT                    (0U)
#define CAU_ROTL_CA_CA2(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA2_SHIFT)) & CAU_ROTL_CA_CA2_MASK)
#define CAU_ROTL_CA_CA3_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA3_SHIFT                    (0U)
#define CAU_ROTL_CA_CA3(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA3_SHIFT)) & CAU_ROTL_CA_CA3_MASK)
#define CAU_ROTL_CA_CA4_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA4_SHIFT                    (0U)
#define CAU_ROTL_CA_CA4(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA4_SHIFT)) & CAU_ROTL_CA_CA4_MASK)
#define CAU_ROTL_CA_CA5_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA5_SHIFT                    (0U)
#define CAU_ROTL_CA_CA5(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA5_SHIFT)) & CAU_ROTL_CA_CA5_MASK)
#define CAU_ROTL_CA_CA6_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA6_SHIFT                    (0U)
#define CAU_ROTL_CA_CA6(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA6_SHIFT)) & CAU_ROTL_CA_CA6_MASK)
#define CAU_ROTL_CA_CA7_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA7_SHIFT                    (0U)
#define CAU_ROTL_CA_CA7(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA7_SHIFT)) & CAU_ROTL_CA_CA7_MASK)
#define CAU_ROTL_CA_CA8_MASK                     (0xFFFFFFFFU)
#define CAU_ROTL_CA_CA8_SHIFT                    (0U)
#define CAU_ROTL_CA_CA8(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_ROTL_CA_CA8_SHIFT)) & CAU_ROTL_CA_CA8_MASK)

/* The count of CAU_ROTL_CA */
#define CAU_ROTL_CA_COUNT                        (9U)

/*! @name AESC_CASR - Status register - AES Column Operation command */
#define CAU_AESC_CASR_IC_MASK                    (0x1U)
#define CAU_AESC_CASR_IC_SHIFT                   (0U)
#define CAU_AESC_CASR_IC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CASR_IC_SHIFT)) & CAU_AESC_CASR_IC_MASK)
#define CAU_AESC_CASR_DPE_MASK                   (0x2U)
#define CAU_AESC_CASR_DPE_SHIFT                  (1U)
#define CAU_AESC_CASR_DPE(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CASR_DPE_SHIFT)) & CAU_AESC_CASR_DPE_MASK)
#define CAU_AESC_CASR_VER_MASK                   (0xF0000000U)
#define CAU_AESC_CASR_VER_SHIFT                  (28U)
#define CAU_AESC_CASR_VER(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CASR_VER_SHIFT)) & CAU_AESC_CASR_VER_MASK)

/*! @name AESC_CAA - Accumulator register - AES Column Operation command */
#define CAU_AESC_CAA_ACC_MASK                    (0xFFFFFFFFU)
#define CAU_AESC_CAA_ACC_SHIFT                   (0U)
#define CAU_AESC_CAA_ACC(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CAA_ACC_SHIFT)) & CAU_AESC_CAA_ACC_MASK)

/*! @name AESC_CA - General Purpose Register 0 - AES Column Operation command..General Purpose Register 8 - AES Column Operation command */
#define CAU_AESC_CA_CA0_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA0_SHIFT                    (0U)
#define CAU_AESC_CA_CA0(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA0_SHIFT)) & CAU_AESC_CA_CA0_MASK)
#define CAU_AESC_CA_CA1_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA1_SHIFT                    (0U)
#define CAU_AESC_CA_CA1(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA1_SHIFT)) & CAU_AESC_CA_CA1_MASK)
#define CAU_AESC_CA_CA2_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA2_SHIFT                    (0U)
#define CAU_AESC_CA_CA2(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA2_SHIFT)) & CAU_AESC_CA_CA2_MASK)
#define CAU_AESC_CA_CA3_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA3_SHIFT                    (0U)
#define CAU_AESC_CA_CA3(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA3_SHIFT)) & CAU_AESC_CA_CA3_MASK)
#define CAU_AESC_CA_CA4_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA4_SHIFT                    (0U)
#define CAU_AESC_CA_CA4(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA4_SHIFT)) & CAU_AESC_CA_CA4_MASK)
#define CAU_AESC_CA_CA5_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA5_SHIFT                    (0U)
#define CAU_AESC_CA_CA5(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA5_SHIFT)) & CAU_AESC_CA_CA5_MASK)
#define CAU_AESC_CA_CA6_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA6_SHIFT                    (0U)
#define CAU_AESC_CA_CA6(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA6_SHIFT)) & CAU_AESC_CA_CA6_MASK)
#define CAU_AESC_CA_CA7_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA7_SHIFT                    (0U)
#define CAU_AESC_CA_CA7(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA7_SHIFT)) & CAU_AESC_CA_CA7_MASK)
#define CAU_AESC_CA_CA8_MASK                     (0xFFFFFFFFU)
#define CAU_AESC_CA_CA8_SHIFT                    (0U)
#define CAU_AESC_CA_CA8(x)                       (((uint32_t)(((uint32_t)(x)) << CAU_AESC_CA_CA8_SHIFT)) & CAU_AESC_CA_CA8_MASK)

/* The count of CAU_AESC_CA */
#define CAU_AESC_CA_COUNT                        (9U)

/*! @name AESIC_CASR - Status register - AES Inverse Column Operation command */
#define CAU_AESIC_CASR_IC_MASK                   (0x1U)
#define CAU_AESIC_CASR_IC_SHIFT                  (0U)
#define CAU_AESIC_CASR_IC(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CASR_IC_SHIFT)) & CAU_AESIC_CASR_IC_MASK)
#define CAU_AESIC_CASR_DPE_MASK                  (0x2U)
#define CAU_AESIC_CASR_DPE_SHIFT                 (1U)
#define CAU_AESIC_CASR_DPE(x)                    (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CASR_DPE_SHIFT)) & CAU_AESIC_CASR_DPE_MASK)
#define CAU_AESIC_CASR_VER_MASK                  (0xF0000000U)
#define CAU_AESIC_CASR_VER_SHIFT                 (28U)
#define CAU_AESIC_CASR_VER(x)                    (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CASR_VER_SHIFT)) & CAU_AESIC_CASR_VER_MASK)

/*! @name AESIC_CAA - Accumulator register - AES Inverse Column Operation command */
#define CAU_AESIC_CAA_ACC_MASK                   (0xFFFFFFFFU)
#define CAU_AESIC_CAA_ACC_SHIFT                  (0U)
#define CAU_AESIC_CAA_ACC(x)                     (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CAA_ACC_SHIFT)) & CAU_AESIC_CAA_ACC_MASK)

/*! @name AESIC_CA - General Purpose Register 0 - AES Inverse Column Operation command..General Purpose Register 8 - AES Inverse Column Operation command */
#define CAU_AESIC_CA_CA0_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA0_SHIFT                   (0U)
#define CAU_AESIC_CA_CA0(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA0_SHIFT)) & CAU_AESIC_CA_CA0_MASK)
#define CAU_AESIC_CA_CA1_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA1_SHIFT                   (0U)
#define CAU_AESIC_CA_CA1(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA1_SHIFT)) & CAU_AESIC_CA_CA1_MASK)
#define CAU_AESIC_CA_CA2_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA2_SHIFT                   (0U)
#define CAU_AESIC_CA_CA2(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA2_SHIFT)) & CAU_AESIC_CA_CA2_MASK)
#define CAU_AESIC_CA_CA3_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA3_SHIFT                   (0U)
#define CAU_AESIC_CA_CA3(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA3_SHIFT)) & CAU_AESIC_CA_CA3_MASK)
#define CAU_AESIC_CA_CA4_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA4_SHIFT                   (0U)
#define CAU_AESIC_CA_CA4(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA4_SHIFT)) & CAU_AESIC_CA_CA4_MASK)
#define CAU_AESIC_CA_CA5_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA5_SHIFT                   (0U)
#define CAU_AESIC_CA_CA5(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA5_SHIFT)) & CAU_AESIC_CA_CA5_MASK)
#define CAU_AESIC_CA_CA6_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA6_SHIFT                   (0U)
#define CAU_AESIC_CA_CA6(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA6_SHIFT)) & CAU_AESIC_CA_CA6_MASK)
#define CAU_AESIC_CA_CA7_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA7_SHIFT                   (0U)
#define CAU_AESIC_CA_CA7(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA7_SHIFT)) & CAU_AESIC_CA_CA7_MASK)
#define CAU_AESIC_CA_CA8_MASK                    (0xFFFFFFFFU)
#define CAU_AESIC_CA_CA8_SHIFT                   (0U)
#define CAU_AESIC_CA_CA8(x)                      (((uint32_t)(((uint32_t)(x)) << CAU_AESIC_CA_CA8_SHIFT)) & CAU_AESIC_CA_CA8_MASK)

/* The count of CAU_AESIC_CA */
#define CAU_AESIC_CA_COUNT                       (9U)


/*!
 * @}
 */ /* end of group CAU_Register_Masks */


/* CAU - Peripheral instance base addresses */
/** Peripheral CAU base address */
#define CAU_BASE                                 (0xE0081000u)
/** Peripheral CAU base pointer */
#define CAU                                      ((CAU_Type *)CAU_BASE)
/** Array initializer of CAU peripheral base addresses */
#define CAU_BASE_ADDRS                           { CAU_BASE }
/** Array initializer of CAU peripheral base pointers */
#define CAU_BASE_PTRS                            { CAU }

/*!
 * @}
 */ /* end of group CAU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR0;                                /**< CMP Control Register 0, offset: 0x0 */
  __IO uint8_t CR1;                                /**< CMP Control Register 1, offset: 0x1 */
  __IO uint8_t FPR;                                /**< CMP Filter Period Register, offset: 0x2 */
  __IO uint8_t SCR;                                /**< CMP Status and Control Register, offset: 0x3 */
  __IO uint8_t DACCR;                              /**< DAC Control Register, offset: 0x4 */
  __IO uint8_t MUXCR;                              /**< MUX Control Register, offset: 0x5 */
} CMP_Type;

/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/*! @name CR0 - CMP Control Register 0 */
#define CMP_CR0_HYSTCTR_MASK                     (0x3U)
#define CMP_CR0_HYSTCTR_SHIFT                    (0U)
#define CMP_CR0_HYSTCTR(x)                       (((uint8_t)(((uint8_t)(x)) << CMP_CR0_HYSTCTR_SHIFT)) & CMP_CR0_HYSTCTR_MASK)
#define CMP_CR0_FILTER_CNT_MASK                  (0x70U)
#define CMP_CR0_FILTER_CNT_SHIFT                 (4U)
#define CMP_CR0_FILTER_CNT(x)                    (((uint8_t)(((uint8_t)(x)) << CMP_CR0_FILTER_CNT_SHIFT)) & CMP_CR0_FILTER_CNT_MASK)

/*! @name CR1 - CMP Control Register 1 */
#define CMP_CR1_EN_MASK                          (0x1U)
#define CMP_CR1_EN_SHIFT                         (0U)
#define CMP_CR1_EN(x)                            (((uint8_t)(((uint8_t)(x)) << CMP_CR1_EN_SHIFT)) & CMP_CR1_EN_MASK)
#define CMP_CR1_OPE_MASK                         (0x2U)
#define CMP_CR1_OPE_SHIFT                        (1U)
#define CMP_CR1_OPE(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_CR1_OPE_SHIFT)) & CMP_CR1_OPE_MASK)
#define CMP_CR1_COS_MASK                         (0x4U)
#define CMP_CR1_COS_SHIFT                        (2U)
#define CMP_CR1_COS(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_CR1_COS_SHIFT)) & CMP_CR1_COS_MASK)
#define CMP_CR1_INV_MASK                         (0x8U)
#define CMP_CR1_INV_SHIFT                        (3U)
#define CMP_CR1_INV(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_CR1_INV_SHIFT)) & CMP_CR1_INV_MASK)
#define CMP_CR1_PMODE_MASK                       (0x10U)
#define CMP_CR1_PMODE_SHIFT                      (4U)
#define CMP_CR1_PMODE(x)                         (((uint8_t)(((uint8_t)(x)) << CMP_CR1_PMODE_SHIFT)) & CMP_CR1_PMODE_MASK)
#define CMP_CR1_TRIGM_MASK                       (0x20U)
#define CMP_CR1_TRIGM_SHIFT                      (5U)
#define CMP_CR1_TRIGM(x)                         (((uint8_t)(((uint8_t)(x)) << CMP_CR1_TRIGM_SHIFT)) & CMP_CR1_TRIGM_MASK)
#define CMP_CR1_WE_MASK                          (0x40U)
#define CMP_CR1_WE_SHIFT                         (6U)
#define CMP_CR1_WE(x)                            (((uint8_t)(((uint8_t)(x)) << CMP_CR1_WE_SHIFT)) & CMP_CR1_WE_MASK)
#define CMP_CR1_SE_MASK                          (0x80U)
#define CMP_CR1_SE_SHIFT                         (7U)
#define CMP_CR1_SE(x)                            (((uint8_t)(((uint8_t)(x)) << CMP_CR1_SE_SHIFT)) & CMP_CR1_SE_MASK)

/*! @name FPR - CMP Filter Period Register */
#define CMP_FPR_FILT_PER_MASK                    (0xFFU)
#define CMP_FPR_FILT_PER_SHIFT                   (0U)
#define CMP_FPR_FILT_PER(x)                      (((uint8_t)(((uint8_t)(x)) << CMP_FPR_FILT_PER_SHIFT)) & CMP_FPR_FILT_PER_MASK)

/*! @name SCR - CMP Status and Control Register */
#define CMP_SCR_COUT_MASK                        (0x1U)
#define CMP_SCR_COUT_SHIFT                       (0U)
#define CMP_SCR_COUT(x)                          (((uint8_t)(((uint8_t)(x)) << CMP_SCR_COUT_SHIFT)) & CMP_SCR_COUT_MASK)
#define CMP_SCR_CFF_MASK                         (0x2U)
#define CMP_SCR_CFF_SHIFT                        (1U)
#define CMP_SCR_CFF(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_SCR_CFF_SHIFT)) & CMP_SCR_CFF_MASK)
#define CMP_SCR_CFR_MASK                         (0x4U)
#define CMP_SCR_CFR_SHIFT                        (2U)
#define CMP_SCR_CFR(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_SCR_CFR_SHIFT)) & CMP_SCR_CFR_MASK)
#define CMP_SCR_IEF_MASK                         (0x8U)
#define CMP_SCR_IEF_SHIFT                        (3U)
#define CMP_SCR_IEF(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_SCR_IEF_SHIFT)) & CMP_SCR_IEF_MASK)
#define CMP_SCR_IER_MASK                         (0x10U)
#define CMP_SCR_IER_SHIFT                        (4U)
#define CMP_SCR_IER(x)                           (((uint8_t)(((uint8_t)(x)) << CMP_SCR_IER_SHIFT)) & CMP_SCR_IER_MASK)
#define CMP_SCR_DMAEN_MASK                       (0x40U)
#define CMP_SCR_DMAEN_SHIFT                      (6U)
#define CMP_SCR_DMAEN(x)                         (((uint8_t)(((uint8_t)(x)) << CMP_SCR_DMAEN_SHIFT)) & CMP_SCR_DMAEN_MASK)

/*! @name DACCR - DAC Control Register */
#define CMP_DACCR_VOSEL_MASK                     (0x3FU)
#define CMP_DACCR_VOSEL_SHIFT                    (0U)
#define CMP_DACCR_VOSEL(x)                       (((uint8_t)(((uint8_t)(x)) << CMP_DACCR_VOSEL_SHIFT)) & CMP_DACCR_VOSEL_MASK)
#define CMP_DACCR_VRSEL_MASK                     (0x40U)
#define CMP_DACCR_VRSEL_SHIFT                    (6U)
#define CMP_DACCR_VRSEL(x)                       (((uint8_t)(((uint8_t)(x)) << CMP_DACCR_VRSEL_SHIFT)) & CMP_DACCR_VRSEL_MASK)
#define CMP_DACCR_DACEN_MASK                     (0x80U)
#define CMP_DACCR_DACEN_SHIFT                    (7U)
#define CMP_DACCR_DACEN(x)                       (((uint8_t)(((uint8_t)(x)) << CMP_DACCR_DACEN_SHIFT)) & CMP_DACCR_DACEN_MASK)

/*! @name MUXCR - MUX Control Register */
#define CMP_MUXCR_MSEL_MASK                      (0x7U)
#define CMP_MUXCR_MSEL_SHIFT                     (0U)
#define CMP_MUXCR_MSEL(x)                        (((uint8_t)(((uint8_t)(x)) << CMP_MUXCR_MSEL_SHIFT)) & CMP_MUXCR_MSEL_MASK)
#define CMP_MUXCR_PSEL_MASK                      (0x38U)
#define CMP_MUXCR_PSEL_SHIFT                     (3U)
#define CMP_MUXCR_PSEL(x)                        (((uint8_t)(((uint8_t)(x)) << CMP_MUXCR_PSEL_SHIFT)) & CMP_MUXCR_PSEL_MASK)


/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
/** Peripheral CMP1 base address */
#define CMP1_BASE                                (0x40073008u)
/** Peripheral CMP1 base pointer */
#define CMP1                                     ((CMP_Type *)CMP1_BASE)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMP0_BASE, CMP1_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMP0, CMP1 }
/** Interrupt vectors for the CMP peripheral type */
#define CMP_IRQS                                 { CMP0_IRQn, CMP1_IRQn }

/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMT_Peripheral_Access_Layer CMT Peripheral Access Layer
 * @{
 */

/** CMT - Register Layout Typedef */
typedef struct {
  __IO uint8_t CGH1;                               /**< CMT Carrier Generator High Data Register 1, offset: 0x0 */
  __IO uint8_t CGL1;                               /**< CMT Carrier Generator Low Data Register 1, offset: 0x1 */
  __IO uint8_t CGH2;                               /**< CMT Carrier Generator High Data Register 2, offset: 0x2 */
  __IO uint8_t CGL2;                               /**< CMT Carrier Generator Low Data Register 2, offset: 0x3 */
  __IO uint8_t OC;                                 /**< CMT Output Control Register, offset: 0x4 */
  __IO uint8_t MSC;                                /**< CMT Modulator Status and Control Register, offset: 0x5 */
  __IO uint8_t CMD1;                               /**< CMT Modulator Data Register Mark High, offset: 0x6 */
  __IO uint8_t CMD2;                               /**< CMT Modulator Data Register Mark Low, offset: 0x7 */
  __IO uint8_t CMD3;                               /**< CMT Modulator Data Register Space High, offset: 0x8 */
  __IO uint8_t CMD4;                               /**< CMT Modulator Data Register Space Low, offset: 0x9 */
  __IO uint8_t PPS;                                /**< CMT Primary Prescaler Register, offset: 0xA */
  __IO uint8_t DMA;                                /**< CMT Direct Memory Access Register, offset: 0xB */
} CMT_Type;

/* ----------------------------------------------------------------------------
   -- CMT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMT_Register_Masks CMT Register Masks
 * @{
 */

/*! @name CGH1 - CMT Carrier Generator High Data Register 1 */
#define CMT_CGH1_PH_MASK                         (0xFFU)
#define CMT_CGH1_PH_SHIFT                        (0U)
#define CMT_CGH1_PH(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CGH1_PH_SHIFT)) & CMT_CGH1_PH_MASK)

/*! @name CGL1 - CMT Carrier Generator Low Data Register 1 */
#define CMT_CGL1_PL_MASK                         (0xFFU)
#define CMT_CGL1_PL_SHIFT                        (0U)
#define CMT_CGL1_PL(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CGL1_PL_SHIFT)) & CMT_CGL1_PL_MASK)

/*! @name CGH2 - CMT Carrier Generator High Data Register 2 */
#define CMT_CGH2_SH_MASK                         (0xFFU)
#define CMT_CGH2_SH_SHIFT                        (0U)
#define CMT_CGH2_SH(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CGH2_SH_SHIFT)) & CMT_CGH2_SH_MASK)

/*! @name CGL2 - CMT Carrier Generator Low Data Register 2 */
#define CMT_CGL2_SL_MASK                         (0xFFU)
#define CMT_CGL2_SL_SHIFT                        (0U)
#define CMT_CGL2_SL(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CGL2_SL_SHIFT)) & CMT_CGL2_SL_MASK)

/*! @name OC - CMT Output Control Register */
#define CMT_OC_IROPEN_MASK                       (0x20U)
#define CMT_OC_IROPEN_SHIFT                      (5U)
#define CMT_OC_IROPEN(x)                         (((uint8_t)(((uint8_t)(x)) << CMT_OC_IROPEN_SHIFT)) & CMT_OC_IROPEN_MASK)
#define CMT_OC_CMTPOL_MASK                       (0x40U)
#define CMT_OC_CMTPOL_SHIFT                      (6U)
#define CMT_OC_CMTPOL(x)                         (((uint8_t)(((uint8_t)(x)) << CMT_OC_CMTPOL_SHIFT)) & CMT_OC_CMTPOL_MASK)
#define CMT_OC_IROL_MASK                         (0x80U)
#define CMT_OC_IROL_SHIFT                        (7U)
#define CMT_OC_IROL(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_OC_IROL_SHIFT)) & CMT_OC_IROL_MASK)

/*! @name MSC - CMT Modulator Status and Control Register */
#define CMT_MSC_MCGEN_MASK                       (0x1U)
#define CMT_MSC_MCGEN_SHIFT                      (0U)
#define CMT_MSC_MCGEN(x)                         (((uint8_t)(((uint8_t)(x)) << CMT_MSC_MCGEN_SHIFT)) & CMT_MSC_MCGEN_MASK)
#define CMT_MSC_EOCIE_MASK                       (0x2U)
#define CMT_MSC_EOCIE_SHIFT                      (1U)
#define CMT_MSC_EOCIE(x)                         (((uint8_t)(((uint8_t)(x)) << CMT_MSC_EOCIE_SHIFT)) & CMT_MSC_EOCIE_MASK)
#define CMT_MSC_FSK_MASK                         (0x4U)
#define CMT_MSC_FSK_SHIFT                        (2U)
#define CMT_MSC_FSK(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_MSC_FSK_SHIFT)) & CMT_MSC_FSK_MASK)
#define CMT_MSC_BASE_MASK                        (0x8U)
#define CMT_MSC_BASE_SHIFT                       (3U)
#define CMT_MSC_BASE(x)                          (((uint8_t)(((uint8_t)(x)) << CMT_MSC_BASE_SHIFT)) & CMT_MSC_BASE_MASK)
#define CMT_MSC_EXSPC_MASK                       (0x10U)
#define CMT_MSC_EXSPC_SHIFT                      (4U)
#define CMT_MSC_EXSPC(x)                         (((uint8_t)(((uint8_t)(x)) << CMT_MSC_EXSPC_SHIFT)) & CMT_MSC_EXSPC_MASK)
#define CMT_MSC_CMTDIV_MASK                      (0x60U)
#define CMT_MSC_CMTDIV_SHIFT                     (5U)
#define CMT_MSC_CMTDIV(x)                        (((uint8_t)(((uint8_t)(x)) << CMT_MSC_CMTDIV_SHIFT)) & CMT_MSC_CMTDIV_MASK)
#define CMT_MSC_EOCF_MASK                        (0x80U)
#define CMT_MSC_EOCF_SHIFT                       (7U)
#define CMT_MSC_EOCF(x)                          (((uint8_t)(((uint8_t)(x)) << CMT_MSC_EOCF_SHIFT)) & CMT_MSC_EOCF_MASK)

/*! @name CMD1 - CMT Modulator Data Register Mark High */
#define CMT_CMD1_MB_MASK                         (0xFFU)
#define CMT_CMD1_MB_SHIFT                        (0U)
#define CMT_CMD1_MB(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CMD1_MB_SHIFT)) & CMT_CMD1_MB_MASK)

/*! @name CMD2 - CMT Modulator Data Register Mark Low */
#define CMT_CMD2_MB_MASK                         (0xFFU)
#define CMT_CMD2_MB_SHIFT                        (0U)
#define CMT_CMD2_MB(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CMD2_MB_SHIFT)) & CMT_CMD2_MB_MASK)

/*! @name CMD3 - CMT Modulator Data Register Space High */
#define CMT_CMD3_SB_MASK                         (0xFFU)
#define CMT_CMD3_SB_SHIFT                        (0U)
#define CMT_CMD3_SB(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CMD3_SB_SHIFT)) & CMT_CMD3_SB_MASK)

/*! @name CMD4 - CMT Modulator Data Register Space Low */
#define CMT_CMD4_SB_MASK                         (0xFFU)
#define CMT_CMD4_SB_SHIFT                        (0U)
#define CMT_CMD4_SB(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_CMD4_SB_SHIFT)) & CMT_CMD4_SB_MASK)

/*! @name PPS - CMT Primary Prescaler Register */
#define CMT_PPS_PPSDIV_MASK                      (0xFU)
#define CMT_PPS_PPSDIV_SHIFT                     (0U)
#define CMT_PPS_PPSDIV(x)                        (((uint8_t)(((uint8_t)(x)) << CMT_PPS_PPSDIV_SHIFT)) & CMT_PPS_PPSDIV_MASK)

/*! @name DMA - CMT Direct Memory Access Register */
#define CMT_DMA_DMA_MASK                         (0x1U)
#define CMT_DMA_DMA_SHIFT                        (0U)
#define CMT_DMA_DMA(x)                           (((uint8_t)(((uint8_t)(x)) << CMT_DMA_DMA_SHIFT)) & CMT_DMA_DMA_MASK)


/*!
 * @}
 */ /* end of group CMT_Register_Masks */


/* CMT - Peripheral instance base addresses */
/** Peripheral CMT base address */
#define CMT_BASE                                 (0x40062000u)
/** Peripheral CMT base pointer */
#define CMT                                      ((CMT_Type *)CMT_BASE)
/** Array initializer of CMT peripheral base addresses */
#define CMT_BASE_ADDRS                           { CMT_BASE }
/** Array initializer of CMT peripheral base pointers */
#define CMT_BASE_PTRS                            { CMT }
/** Interrupt vectors for the CMT peripheral type */
#define CMT_IRQS                                 { CMT_IRQn }

/*!
 * @}
 */ /* end of group CMT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  union {                                          /* offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint16_t DATAL;                             /**< CRC_DATAL register., offset: 0x0 */
      __IO uint16_t DATAH;                             /**< CRC_DATAH register., offset: 0x2 */
    } ACCESS16BIT;
    __IO uint32_t DATA;                              /**< CRC Data register, offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint8_t DATALL;                             /**< CRC_DATALL register., offset: 0x0 */
      __IO uint8_t DATALU;                             /**< CRC_DATALU register., offset: 0x1 */
      __IO uint8_t DATAHL;                             /**< CRC_DATAHL register., offset: 0x2 */
      __IO uint8_t DATAHU;                             /**< CRC_DATAHU register., offset: 0x3 */
    } ACCESS8BIT;
  };
  union {                                          /* offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint16_t GPOLYL;                            /**< CRC_GPOLYL register., offset: 0x4 */
      __IO uint16_t GPOLYH;                            /**< CRC_GPOLYH register., offset: 0x6 */
    } GPOLY_ACCESS16BIT;
    __IO uint32_t GPOLY;                             /**< CRC Polynomial register, offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint8_t GPOLYLL;                            /**< CRC_GPOLYLL register., offset: 0x4 */
      __IO uint8_t GPOLYLU;                            /**< CRC_GPOLYLU register., offset: 0x5 */
      __IO uint8_t GPOLYHL;                            /**< CRC_GPOLYHL register., offset: 0x6 */
      __IO uint8_t GPOLYHU;                            /**< CRC_GPOLYHU register., offset: 0x7 */
    } GPOLY_ACCESS8BIT;
  };
  union {                                          /* offset: 0x8 */
    __IO uint32_t CTRL;                              /**< CRC Control register, offset: 0x8 */
    struct {                                         /* offset: 0x8 */
           uint8_t RESERVED_0[3];
      __IO uint8_t CTRLHU;                             /**< CRC_CTRLHU register., offset: 0xB */
    } CTRL_ACCESS8BIT;
  };
} CRC_Type;

/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/*! @name DATAL - CRC_DATAL register. */
#define CRC_DATAL_DATAL_MASK                     (0xFFFFU)
#define CRC_DATAL_DATAL_SHIFT                    (0U)
#define CRC_DATAL_DATAL(x)                       (((uint16_t)(((uint16_t)(x)) << CRC_DATAL_DATAL_SHIFT)) & CRC_DATAL_DATAL_MASK)

/*! @name DATAH - CRC_DATAH register. */
#define CRC_DATAH_DATAH_MASK                     (0xFFFFU)
#define CRC_DATAH_DATAH_SHIFT                    (0U)
#define CRC_DATAH_DATAH(x)                       (((uint16_t)(((uint16_t)(x)) << CRC_DATAH_DATAH_SHIFT)) & CRC_DATAH_DATAH_MASK)

/*! @name DATA - CRC Data register */
#define CRC_DATA_LL_MASK                         (0xFFU)
#define CRC_DATA_LL_SHIFT                        (0U)
#define CRC_DATA_LL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_LL_SHIFT)) & CRC_DATA_LL_MASK)
#define CRC_DATA_LU_MASK                         (0xFF00U)
#define CRC_DATA_LU_SHIFT                        (8U)
#define CRC_DATA_LU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_LU_SHIFT)) & CRC_DATA_LU_MASK)
#define CRC_DATA_HL_MASK                         (0xFF0000U)
#define CRC_DATA_HL_SHIFT                        (16U)
#define CRC_DATA_HL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_HL_SHIFT)) & CRC_DATA_HL_MASK)
#define CRC_DATA_HU_MASK                         (0xFF000000U)
#define CRC_DATA_HU_SHIFT                        (24U)
#define CRC_DATA_HU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_HU_SHIFT)) & CRC_DATA_HU_MASK)

/*! @name DATALL - CRC_DATALL register. */
#define CRC_DATALL_DATALL_MASK                   (0xFFU)
#define CRC_DATALL_DATALL_SHIFT                  (0U)
#define CRC_DATALL_DATALL(x)                     (((uint8_t)(((uint8_t)(x)) << CRC_DATALL_DATALL_SHIFT)) & CRC_DATALL_DATALL_MASK)

/*! @name DATALU - CRC_DATALU register. */
#define CRC_DATALU_DATALU_MASK                   (0xFFU)
#define CRC_DATALU_DATALU_SHIFT                  (0U)
#define CRC_DATALU_DATALU(x)                     (((uint8_t)(((uint8_t)(x)) << CRC_DATALU_DATALU_SHIFT)) & CRC_DATALU_DATALU_MASK)

/*! @name DATAHL - CRC_DATAHL register. */
#define CRC_DATAHL_DATAHL_MASK                   (0xFFU)
#define CRC_DATAHL_DATAHL_SHIFT                  (0U)
#define CRC_DATAHL_DATAHL(x)                     (((uint8_t)(((uint8_t)(x)) << CRC_DATAHL_DATAHL_SHIFT)) & CRC_DATAHL_DATAHL_MASK)

/*! @name DATAHU - CRC_DATAHU register. */
#define CRC_DATAHU_DATAHU_MASK                   (0xFFU)
#define CRC_DATAHU_DATAHU_SHIFT                  (0U)
#define CRC_DATAHU_DATAHU(x)                     (((uint8_t)(((uint8_t)(x)) << CRC_DATAHU_DATAHU_SHIFT)) & CRC_DATAHU_DATAHU_MASK)

/*! @name GPOLYL - CRC_GPOLYL register. */
#define CRC_GPOLYL_GPOLYL_MASK                   (0xFFFFU)
#define CRC_GPOLYL_GPOLYL_SHIFT                  (0U)
#define CRC_GPOLYL_GPOLYL(x)                     (((uint16_t)(((uint16_t)(x)) << CRC_GPOLYL_GPOLYL_SHIFT)) & CRC_GPOLYL_GPOLYL_MASK)

/*! @name GPOLYH - CRC_GPOLYH register. */
#define CRC_GPOLYH_GPOLYH_MASK                   (0xFFFFU)
#define CRC_GPOLYH_GPOLYH_SHIFT                  (0U)
#define CRC_GPOLYH_GPOLYH(x)                     (((uint16_t)(((uint16_t)(x)) << CRC_GPOLYH_GPOLYH_SHIFT)) & CRC_GPOLYH_GPOLYH_MASK)

/*! @name GPOLY - CRC Polynomial register */
#define CRC_GPOLY_LOW_MASK                       (0xFFFFU)
#define CRC_GPOLY_LOW_SHIFT                      (0U)
#define CRC_GPOLY_LOW(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_LOW_SHIFT)) & CRC_GPOLY_LOW_MASK)
#define CRC_GPOLY_HIGH_MASK                      (0xFFFF0000U)
#define CRC_GPOLY_HIGH_SHIFT                     (16U)
#define CRC_GPOLY_HIGH(x)                        (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_HIGH_SHIFT)) & CRC_GPOLY_HIGH_MASK)

/*! @name GPOLYLL - CRC_GPOLYLL register. */
#define CRC_GPOLYLL_GPOLYLL_MASK                 (0xFFU)
#define CRC_GPOLYLL_GPOLYLL_SHIFT                (0U)
#define CRC_GPOLYLL_GPOLYLL(x)                   (((uint8_t)(((uint8_t)(x)) << CRC_GPOLYLL_GPOLYLL_SHIFT)) & CRC_GPOLYLL_GPOLYLL_MASK)

/*! @name GPOLYLU - CRC_GPOLYLU register. */
#define CRC_GPOLYLU_GPOLYLU_MASK                 (0xFFU)
#define CRC_GPOLYLU_GPOLYLU_SHIFT                (0U)
#define CRC_GPOLYLU_GPOLYLU(x)                   (((uint8_t)(((uint8_t)(x)) << CRC_GPOLYLU_GPOLYLU_SHIFT)) & CRC_GPOLYLU_GPOLYLU_MASK)

/*! @name GPOLYHL - CRC_GPOLYHL register. */
#define CRC_GPOLYHL_GPOLYHL_MASK                 (0xFFU)
#define CRC_GPOLYHL_GPOLYHL_SHIFT                (0U)
#define CRC_GPOLYHL_GPOLYHL(x)                   (((uint8_t)(((uint8_t)(x)) << CRC_GPOLYHL_GPOLYHL_SHIFT)) & CRC_GPOLYHL_GPOLYHL_MASK)

/*! @name GPOLYHU - CRC_GPOLYHU register. */
#define CRC_GPOLYHU_GPOLYHU_MASK                 (0xFFU)
#define CRC_GPOLYHU_GPOLYHU_SHIFT                (0U)
#define CRC_GPOLYHU_GPOLYHU(x)                   (((uint8_t)(((uint8_t)(x)) << CRC_GPOLYHU_GPOLYHU_SHIFT)) & CRC_GPOLYHU_GPOLYHU_MASK)

/*! @name CTRL - CRC Control register */
#define CRC_CTRL_TCRC_MASK                       (0x1000000U)
#define CRC_CTRL_TCRC_SHIFT                      (24U)
#define CRC_CTRL_TCRC(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TCRC_SHIFT)) & CRC_CTRL_TCRC_MASK)
#define CRC_CTRL_WAS_MASK                        (0x2000000U)
#define CRC_CTRL_WAS_SHIFT                       (25U)
#define CRC_CTRL_WAS(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_WAS_SHIFT)) & CRC_CTRL_WAS_MASK)
#define CRC_CTRL_FXOR_MASK                       (0x4000000U)
#define CRC_CTRL_FXOR_SHIFT                      (26U)
#define CRC_CTRL_FXOR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_FXOR_SHIFT)) & CRC_CTRL_FXOR_MASK)
#define CRC_CTRL_TOTR_MASK                       (0x30000000U)
#define CRC_CTRL_TOTR_SHIFT                      (28U)
#define CRC_CTRL_TOTR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOTR_SHIFT)) & CRC_CTRL_TOTR_MASK)
#define CRC_CTRL_TOT_MASK                        (0xC0000000U)
#define CRC_CTRL_TOT_SHIFT                       (30U)
#define CRC_CTRL_TOT(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOT_SHIFT)) & CRC_CTRL_TOT_MASK)

/*! @name CTRLHU - CRC_CTRLHU register. */
#define CRC_CTRLHU_TCRC_MASK                     (0x1U)
#define CRC_CTRLHU_TCRC_SHIFT                    (0U)
#define CRC_CTRLHU_TCRC(x)                       (((uint8_t)(((uint8_t)(x)) << CRC_CTRLHU_TCRC_SHIFT)) & CRC_CTRLHU_TCRC_MASK)
#define CRC_CTRLHU_WAS_MASK                      (0x2U)
#define CRC_CTRLHU_WAS_SHIFT                     (1U)
#define CRC_CTRLHU_WAS(x)                        (((uint8_t)(((uint8_t)(x)) << CRC_CTRLHU_WAS_SHIFT)) & CRC_CTRLHU_WAS_MASK)
#define CRC_CTRLHU_FXOR_MASK                     (0x4U)
#define CRC_CTRLHU_FXOR_SHIFT                    (2U)
#define CRC_CTRLHU_FXOR(x)                       (((uint8_t)(((uint8_t)(x)) << CRC_CTRLHU_FXOR_SHIFT)) & CRC_CTRLHU_FXOR_MASK)
#define CRC_CTRLHU_TOTR_MASK                     (0x30U)
#define CRC_CTRLHU_TOTR_SHIFT                    (4U)
#define CRC_CTRLHU_TOTR(x)                       (((uint8_t)(((uint8_t)(x)) << CRC_CTRLHU_TOTR_SHIFT)) & CRC_CTRLHU_TOTR_MASK)
#define CRC_CTRLHU_TOT_MASK                      (0xC0U)
#define CRC_CTRLHU_TOT_SHIFT                     (6U)
#define CRC_CTRLHU_TOT(x)                        (((uint8_t)(((uint8_t)(x)) << CRC_CTRLHU_TOT_SHIFT)) & CRC_CTRLHU_TOT_MASK)


/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE                                 (0x40032000u)
/** Peripheral CRC base pointer */
#define CRC0                                     ((CRC_Type *)CRC_BASE)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC0 }

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DAC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral_Access_Layer DAC Peripheral Access Layer
 * @{
 */

/** DAC - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x2 */
    __IO uint8_t DATL;                               /**< DAC Data Low Register, array offset: 0x0, array step: 0x2 */
    __IO uint8_t DATH;                               /**< DAC Data High Register, array offset: 0x1, array step: 0x2 */
  } DAT[16];
  __IO uint8_t SR;                                 /**< DAC Status Register, offset: 0x20 */
  __IO uint8_t C0;                                 /**< DAC Control Register, offset: 0x21 */
  __IO uint8_t C1;                                 /**< DAC Control Register 1, offset: 0x22 */
  __IO uint8_t C2;                                 /**< DAC Control Register 2, offset: 0x23 */
} DAC_Type;

/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/*! @name DATL - DAC Data Low Register */
#define DAC_DATL_DATA0_MASK                      (0xFFU)
#define DAC_DATL_DATA0_SHIFT                     (0U)
#define DAC_DATL_DATA0(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_DATL_DATA0_SHIFT)) & DAC_DATL_DATA0_MASK)

/* The count of DAC_DATL */
#define DAC_DATL_COUNT                           (16U)

/*! @name DATH - DAC Data High Register */
#define DAC_DATH_DATA1_MASK                      (0xFU)
#define DAC_DATH_DATA1_SHIFT                     (0U)
#define DAC_DATH_DATA1(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_DATH_DATA1_SHIFT)) & DAC_DATH_DATA1_MASK)

/* The count of DAC_DATH */
#define DAC_DATH_COUNT                           (16U)

/*! @name SR - DAC Status Register */
#define DAC_SR_DACBFRPBF_MASK                    (0x1U)
#define DAC_SR_DACBFRPBF_SHIFT                   (0U)
#define DAC_SR_DACBFRPBF(x)                      (((uint8_t)(((uint8_t)(x)) << DAC_SR_DACBFRPBF_SHIFT)) & DAC_SR_DACBFRPBF_MASK)
#define DAC_SR_DACBFRPTF_MASK                    (0x2U)
#define DAC_SR_DACBFRPTF_SHIFT                   (1U)
#define DAC_SR_DACBFRPTF(x)                      (((uint8_t)(((uint8_t)(x)) << DAC_SR_DACBFRPTF_SHIFT)) & DAC_SR_DACBFRPTF_MASK)
#define DAC_SR_DACBFWMF_MASK                     (0x4U)
#define DAC_SR_DACBFWMF_SHIFT                    (2U)
#define DAC_SR_DACBFWMF(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_SR_DACBFWMF_SHIFT)) & DAC_SR_DACBFWMF_MASK)

/*! @name C0 - DAC Control Register */
#define DAC_C0_DACBBIEN_MASK                     (0x1U)
#define DAC_C0_DACBBIEN_SHIFT                    (0U)
#define DAC_C0_DACBBIEN(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACBBIEN_SHIFT)) & DAC_C0_DACBBIEN_MASK)
#define DAC_C0_DACBTIEN_MASK                     (0x2U)
#define DAC_C0_DACBTIEN_SHIFT                    (1U)
#define DAC_C0_DACBTIEN(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACBTIEN_SHIFT)) & DAC_C0_DACBTIEN_MASK)
#define DAC_C0_DACBWIEN_MASK                     (0x4U)
#define DAC_C0_DACBWIEN_SHIFT                    (2U)
#define DAC_C0_DACBWIEN(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACBWIEN_SHIFT)) & DAC_C0_DACBWIEN_MASK)
#define DAC_C0_LPEN_MASK                         (0x8U)
#define DAC_C0_LPEN_SHIFT                        (3U)
#define DAC_C0_LPEN(x)                           (((uint8_t)(((uint8_t)(x)) << DAC_C0_LPEN_SHIFT)) & DAC_C0_LPEN_MASK)
#define DAC_C0_DACSWTRG_MASK                     (0x10U)
#define DAC_C0_DACSWTRG_SHIFT                    (4U)
#define DAC_C0_DACSWTRG(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACSWTRG_SHIFT)) & DAC_C0_DACSWTRG_MASK)
#define DAC_C0_DACTRGSEL_MASK                    (0x20U)
#define DAC_C0_DACTRGSEL_SHIFT                   (5U)
#define DAC_C0_DACTRGSEL(x)                      (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACTRGSEL_SHIFT)) & DAC_C0_DACTRGSEL_MASK)
#define DAC_C0_DACRFS_MASK                       (0x40U)
#define DAC_C0_DACRFS_SHIFT                      (6U)
#define DAC_C0_DACRFS(x)                         (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACRFS_SHIFT)) & DAC_C0_DACRFS_MASK)
#define DAC_C0_DACEN_MASK                        (0x80U)
#define DAC_C0_DACEN_SHIFT                       (7U)
#define DAC_C0_DACEN(x)                          (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACEN_SHIFT)) & DAC_C0_DACEN_MASK)

/*! @name C1 - DAC Control Register 1 */
#define DAC_C1_DACBFEN_MASK                      (0x1U)
#define DAC_C1_DACBFEN_SHIFT                     (0U)
#define DAC_C1_DACBFEN(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C1_DACBFEN_SHIFT)) & DAC_C1_DACBFEN_MASK)
#define DAC_C1_DACBFMD_MASK                      (0x6U)
#define DAC_C1_DACBFMD_SHIFT                     (1U)
#define DAC_C1_DACBFMD(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C1_DACBFMD_SHIFT)) & DAC_C1_DACBFMD_MASK)
#define DAC_C1_DACBFWM_MASK                      (0x18U)
#define DAC_C1_DACBFWM_SHIFT                     (3U)
#define DAC_C1_DACBFWM(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C1_DACBFWM_SHIFT)) & DAC_C1_DACBFWM_MASK)
#define DAC_C1_DMAEN_MASK                        (0x80U)
#define DAC_C1_DMAEN_SHIFT                       (7U)
#define DAC_C1_DMAEN(x)                          (((uint8_t)(((uint8_t)(x)) << DAC_C1_DMAEN_SHIFT)) & DAC_C1_DMAEN_MASK)

/*! @name C2 - DAC Control Register 2 */
#define DAC_C2_DACBFUP_MASK                      (0xFU)
#define DAC_C2_DACBFUP_SHIFT                     (0U)
#define DAC_C2_DACBFUP(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C2_DACBFUP_SHIFT)) & DAC_C2_DACBFUP_MASK)
#define DAC_C2_DACBFRP_MASK                      (0xF0U)
#define DAC_C2_DACBFRP_SHIFT                     (4U)
#define DAC_C2_DACBFRP(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C2_DACBFRP_SHIFT)) & DAC_C2_DACBFRP_MASK)


/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DAC0 base address */
#define DAC0_BASE                                (0x400CC000u)
/** Peripheral DAC0 base pointer */
#define DAC0                                     ((DAC_Type *)DAC0_BASE)
/** Array initializer of DAC peripheral base addresses */
#define DAC_BASE_ADDRS                           { DAC0_BASE }
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DAC0 }
/** Interrupt vectors for the DAC peripheral type */
#define DAC_IRQS                                 { DAC0_IRQn }

/*!
 * @}
 */ /* end of group DAC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< Control Register, offset: 0x0 */
  __I  uint32_t ES;                                /**< Error Status Register, offset: 0x4 */
       uint8_t RESERVED_0[4];
  __IO uint32_t ERQ;                               /**< Enable Request Register, offset: 0xC */
       uint8_t RESERVED_1[4];
  __IO uint32_t EEI;                               /**< Enable Error Interrupt Register, offset: 0x14 */
  __O  uint8_t CEEI;                               /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  __O  uint8_t SEEI;                               /**< Set Enable Error Interrupt Register, offset: 0x19 */
  __O  uint8_t CERQ;                               /**< Clear Enable Request Register, offset: 0x1A */
  __O  uint8_t SERQ;                               /**< Set Enable Request Register, offset: 0x1B */
  __O  uint8_t CDNE;                               /**< Clear DONE Status Bit Register, offset: 0x1C */
  __O  uint8_t SSRT;                               /**< Set START Bit Register, offset: 0x1D */
  __O  uint8_t CERR;                               /**< Clear Error Register, offset: 0x1E */
  __O  uint8_t CINT;                               /**< Clear Interrupt Request Register, offset: 0x1F */
       uint8_t RESERVED_2[4];
  __IO uint32_t INT;                               /**< Interrupt Request Register, offset: 0x24 */
       uint8_t RESERVED_3[4];
  __IO uint32_t ERR;                               /**< Error Register, offset: 0x2C */
       uint8_t RESERVED_4[4];
  __I  uint32_t HRS;                               /**< Hardware Request Status Register, offset: 0x34 */
       uint8_t RESERVED_5[12];
  __IO uint32_t EARS;                              /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
       uint8_t RESERVED_6[184];
  __IO uint8_t DCHPRI3;                            /**< Channel n Priority Register, offset: 0x100 */
  __IO uint8_t DCHPRI2;                            /**< Channel n Priority Register, offset: 0x101 */
  __IO uint8_t DCHPRI1;                            /**< Channel n Priority Register, offset: 0x102 */
  __IO uint8_t DCHPRI0;                            /**< Channel n Priority Register, offset: 0x103 */
  __IO uint8_t DCHPRI7;                            /**< Channel n Priority Register, offset: 0x104 */
  __IO uint8_t DCHPRI6;                            /**< Channel n Priority Register, offset: 0x105 */
  __IO uint8_t DCHPRI5;                            /**< Channel n Priority Register, offset: 0x106 */
  __IO uint8_t DCHPRI4;                            /**< Channel n Priority Register, offset: 0x107 */
  __IO uint8_t DCHPRI11;                           /**< Channel n Priority Register, offset: 0x108 */
  __IO uint8_t DCHPRI10;                           /**< Channel n Priority Register, offset: 0x109 */
  __IO uint8_t DCHPRI9;                            /**< Channel n Priority Register, offset: 0x10A */
  __IO uint8_t DCHPRI8;                            /**< Channel n Priority Register, offset: 0x10B */
  __IO uint8_t DCHPRI15;                           /**< Channel n Priority Register, offset: 0x10C */
  __IO uint8_t DCHPRI14;                           /**< Channel n Priority Register, offset: 0x10D */
  __IO uint8_t DCHPRI13;                           /**< Channel n Priority Register, offset: 0x10E */
  __IO uint8_t DCHPRI12;                           /**< Channel n Priority Register, offset: 0x10F */
  __IO uint8_t DCHPRI19;                           /**< Channel n Priority Register, offset: 0x110 */
  __IO uint8_t DCHPRI18;                           /**< Channel n Priority Register, offset: 0x111 */
  __IO uint8_t DCHPRI17;                           /**< Channel n Priority Register, offset: 0x112 */
  __IO uint8_t DCHPRI16;                           /**< Channel n Priority Register, offset: 0x113 */
  __IO uint8_t DCHPRI23;                           /**< Channel n Priority Register, offset: 0x114 */
  __IO uint8_t DCHPRI22;                           /**< Channel n Priority Register, offset: 0x115 */
  __IO uint8_t DCHPRI21;                           /**< Channel n Priority Register, offset: 0x116 */
  __IO uint8_t DCHPRI20;                           /**< Channel n Priority Register, offset: 0x117 */
  __IO uint8_t DCHPRI27;                           /**< Channel n Priority Register, offset: 0x118 */
  __IO uint8_t DCHPRI26;                           /**< Channel n Priority Register, offset: 0x119 */
  __IO uint8_t DCHPRI25;                           /**< Channel n Priority Register, offset: 0x11A */
  __IO uint8_t DCHPRI24;                           /**< Channel n Priority Register, offset: 0x11B */
  __IO uint8_t DCHPRI31;                           /**< Channel n Priority Register, offset: 0x11C */
  __IO uint8_t DCHPRI30;                           /**< Channel n Priority Register, offset: 0x11D */
  __IO uint8_t DCHPRI29;                           /**< Channel n Priority Register, offset: 0x11E */
  __IO uint8_t DCHPRI28;                           /**< Channel n Priority Register, offset: 0x11F */
       uint8_t RESERVED_7[3808];
  struct {                                         /* offset: 0x1000, array step: 0x20 */
    __IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    __IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    __IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLNO;                       /**< TCD Minor Byte Count (Minor Loop Mapping Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    };
    __IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    __IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    __IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    };
    __IO uint32_t DLAST_SGA;                         /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    __IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    };
  } TCD[32];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name CR - Control Register */
#define DMA_CR_EDBG_MASK                         (0x2U)
#define DMA_CR_EDBG_SHIFT                        (1U)
#define DMA_CR_EDBG(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EDBG_SHIFT)) & DMA_CR_EDBG_MASK)
#define DMA_CR_ERCA_MASK                         (0x4U)
#define DMA_CR_ERCA_SHIFT                        (2U)
#define DMA_CR_ERCA(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_ERCA_SHIFT)) & DMA_CR_ERCA_MASK)
#define DMA_CR_ERGA_MASK                         (0x8U)
#define DMA_CR_ERGA_SHIFT                        (3U)
#define DMA_CR_ERGA(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_ERGA_SHIFT)) & DMA_CR_ERGA_MASK)
#define DMA_CR_HOE_MASK                          (0x10U)
#define DMA_CR_HOE_SHIFT                         (4U)
#define DMA_CR_HOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_HOE_SHIFT)) & DMA_CR_HOE_MASK)
#define DMA_CR_HALT_MASK                         (0x20U)
#define DMA_CR_HALT_SHIFT                        (5U)
#define DMA_CR_HALT(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_HALT_SHIFT)) & DMA_CR_HALT_MASK)
#define DMA_CR_CLM_MASK                          (0x40U)
#define DMA_CR_CLM_SHIFT                         (6U)
#define DMA_CR_CLM(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_CLM_SHIFT)) & DMA_CR_CLM_MASK)
#define DMA_CR_EMLM_MASK                         (0x80U)
#define DMA_CR_EMLM_SHIFT                        (7U)
#define DMA_CR_EMLM(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EMLM_SHIFT)) & DMA_CR_EMLM_MASK)
#define DMA_CR_GRP0PRI_MASK                      (0x100U)
#define DMA_CR_GRP0PRI_SHIFT                     (8U)
#define DMA_CR_GRP0PRI(x)                        (((uint32_t)(((uint32_t)(x)) << DMA_CR_GRP0PRI_SHIFT)) & DMA_CR_GRP0PRI_MASK)
#define DMA_CR_GRP1PRI_MASK                      (0x400U)
#define DMA_CR_GRP1PRI_SHIFT                     (10U)
#define DMA_CR_GRP1PRI(x)                        (((uint32_t)(((uint32_t)(x)) << DMA_CR_GRP1PRI_SHIFT)) & DMA_CR_GRP1PRI_MASK)
#define DMA_CR_ECX_MASK                          (0x10000U)
#define DMA_CR_ECX_SHIFT                         (16U)
#define DMA_CR_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_ECX_SHIFT)) & DMA_CR_ECX_MASK)
#define DMA_CR_CX_MASK                           (0x20000U)
#define DMA_CR_CX_SHIFT                          (17U)
#define DMA_CR_CX(x)                             (((uint32_t)(((uint32_t)(x)) << DMA_CR_CX_SHIFT)) & DMA_CR_CX_MASK)

/*! @name ES - Error Status Register */
#define DMA_ES_DBE_MASK                          (0x1U)
#define DMA_ES_DBE_SHIFT                         (0U)
#define DMA_ES_DBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DBE_SHIFT)) & DMA_ES_DBE_MASK)
#define DMA_ES_SBE_MASK                          (0x2U)
#define DMA_ES_SBE_SHIFT                         (1U)
#define DMA_ES_SBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SBE_SHIFT)) & DMA_ES_SBE_MASK)
#define DMA_ES_SGE_MASK                          (0x4U)
#define DMA_ES_SGE_SHIFT                         (2U)
#define DMA_ES_SGE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SGE_SHIFT)) & DMA_ES_SGE_MASK)
#define DMA_ES_NCE_MASK                          (0x8U)
#define DMA_ES_NCE_SHIFT                         (3U)
#define DMA_ES_NCE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_NCE_SHIFT)) & DMA_ES_NCE_MASK)
#define DMA_ES_DOE_MASK                          (0x10U)
#define DMA_ES_DOE_SHIFT                         (4U)
#define DMA_ES_DOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DOE_SHIFT)) & DMA_ES_DOE_MASK)
#define DMA_ES_DAE_MASK                          (0x20U)
#define DMA_ES_DAE_SHIFT                         (5U)
#define DMA_ES_DAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DAE_SHIFT)) & DMA_ES_DAE_MASK)
#define DMA_ES_SOE_MASK                          (0x40U)
#define DMA_ES_SOE_SHIFT                         (6U)
#define DMA_ES_SOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SOE_SHIFT)) & DMA_ES_SOE_MASK)
#define DMA_ES_SAE_MASK                          (0x80U)
#define DMA_ES_SAE_SHIFT                         (7U)
#define DMA_ES_SAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SAE_SHIFT)) & DMA_ES_SAE_MASK)
#define DMA_ES_ERRCHN_MASK                       (0x1F00U)
#define DMA_ES_ERRCHN_SHIFT                      (8U)
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ES_ERRCHN_SHIFT)) & DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          (0x4000U)
#define DMA_ES_CPE_SHIFT                         (14U)
#define DMA_ES_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_CPE_SHIFT)) & DMA_ES_CPE_MASK)
#define DMA_ES_GPE_MASK                          (0x8000U)
#define DMA_ES_GPE_SHIFT                         (15U)
#define DMA_ES_GPE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_GPE_SHIFT)) & DMA_ES_GPE_MASK)
#define DMA_ES_ECX_MASK                          (0x10000U)
#define DMA_ES_ECX_SHIFT                         (16U)
#define DMA_ES_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_ECX_SHIFT)) & DMA_ES_ECX_MASK)
#define DMA_ES_VLD_MASK                          (0x80000000U)
#define DMA_ES_VLD_SHIFT                         (31U)
#define DMA_ES_VLD(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_VLD_SHIFT)) & DMA_ES_VLD_MASK)

/*! @name ERQ - Enable Request Register */
#define DMA_ERQ_ERQ0_MASK                        (0x1U)
#define DMA_ERQ_ERQ0_SHIFT                       (0U)
#define DMA_ERQ_ERQ0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ0_SHIFT)) & DMA_ERQ_ERQ0_MASK)
#define DMA_ERQ_ERQ1_MASK                        (0x2U)
#define DMA_ERQ_ERQ1_SHIFT                       (1U)
#define DMA_ERQ_ERQ1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ1_SHIFT)) & DMA_ERQ_ERQ1_MASK)
#define DMA_ERQ_ERQ2_MASK                        (0x4U)
#define DMA_ERQ_ERQ2_SHIFT                       (2U)
#define DMA_ERQ_ERQ2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ2_SHIFT)) & DMA_ERQ_ERQ2_MASK)
#define DMA_ERQ_ERQ3_MASK                        (0x8U)
#define DMA_ERQ_ERQ3_SHIFT                       (3U)
#define DMA_ERQ_ERQ3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ3_SHIFT)) & DMA_ERQ_ERQ3_MASK)
#define DMA_ERQ_ERQ4_MASK                        (0x10U)
#define DMA_ERQ_ERQ4_SHIFT                       (4U)
#define DMA_ERQ_ERQ4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ4_SHIFT)) & DMA_ERQ_ERQ4_MASK)
#define DMA_ERQ_ERQ5_MASK                        (0x20U)
#define DMA_ERQ_ERQ5_SHIFT                       (5U)
#define DMA_ERQ_ERQ5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ5_SHIFT)) & DMA_ERQ_ERQ5_MASK)
#define DMA_ERQ_ERQ6_MASK                        (0x40U)
#define DMA_ERQ_ERQ6_SHIFT                       (6U)
#define DMA_ERQ_ERQ6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ6_SHIFT)) & DMA_ERQ_ERQ6_MASK)
#define DMA_ERQ_ERQ7_MASK                        (0x80U)
#define DMA_ERQ_ERQ7_SHIFT                       (7U)
#define DMA_ERQ_ERQ7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ7_SHIFT)) & DMA_ERQ_ERQ7_MASK)
#define DMA_ERQ_ERQ8_MASK                        (0x100U)
#define DMA_ERQ_ERQ8_SHIFT                       (8U)
#define DMA_ERQ_ERQ8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ8_SHIFT)) & DMA_ERQ_ERQ8_MASK)
#define DMA_ERQ_ERQ9_MASK                        (0x200U)
#define DMA_ERQ_ERQ9_SHIFT                       (9U)
#define DMA_ERQ_ERQ9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ9_SHIFT)) & DMA_ERQ_ERQ9_MASK)
#define DMA_ERQ_ERQ10_MASK                       (0x400U)
#define DMA_ERQ_ERQ10_SHIFT                      (10U)
#define DMA_ERQ_ERQ10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ10_SHIFT)) & DMA_ERQ_ERQ10_MASK)
#define DMA_ERQ_ERQ11_MASK                       (0x800U)
#define DMA_ERQ_ERQ11_SHIFT                      (11U)
#define DMA_ERQ_ERQ11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ11_SHIFT)) & DMA_ERQ_ERQ11_MASK)
#define DMA_ERQ_ERQ12_MASK                       (0x1000U)
#define DMA_ERQ_ERQ12_SHIFT                      (12U)
#define DMA_ERQ_ERQ12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ12_SHIFT)) & DMA_ERQ_ERQ12_MASK)
#define DMA_ERQ_ERQ13_MASK                       (0x2000U)
#define DMA_ERQ_ERQ13_SHIFT                      (13U)
#define DMA_ERQ_ERQ13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ13_SHIFT)) & DMA_ERQ_ERQ13_MASK)
#define DMA_ERQ_ERQ14_MASK                       (0x4000U)
#define DMA_ERQ_ERQ14_SHIFT                      (14U)
#define DMA_ERQ_ERQ14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ14_SHIFT)) & DMA_ERQ_ERQ14_MASK)
#define DMA_ERQ_ERQ15_MASK                       (0x8000U)
#define DMA_ERQ_ERQ15_SHIFT                      (15U)
#define DMA_ERQ_ERQ15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ15_SHIFT)) & DMA_ERQ_ERQ15_MASK)
#define DMA_ERQ_ERQ16_MASK                       (0x10000U)
#define DMA_ERQ_ERQ16_SHIFT                      (16U)
#define DMA_ERQ_ERQ16(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ16_SHIFT)) & DMA_ERQ_ERQ16_MASK)
#define DMA_ERQ_ERQ17_MASK                       (0x20000U)
#define DMA_ERQ_ERQ17_SHIFT                      (17U)
#define DMA_ERQ_ERQ17(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ17_SHIFT)) & DMA_ERQ_ERQ17_MASK)
#define DMA_ERQ_ERQ18_MASK                       (0x40000U)
#define DMA_ERQ_ERQ18_SHIFT                      (18U)
#define DMA_ERQ_ERQ18(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ18_SHIFT)) & DMA_ERQ_ERQ18_MASK)
#define DMA_ERQ_ERQ19_MASK                       (0x80000U)
#define DMA_ERQ_ERQ19_SHIFT                      (19U)
#define DMA_ERQ_ERQ19(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ19_SHIFT)) & DMA_ERQ_ERQ19_MASK)
#define DMA_ERQ_ERQ20_MASK                       (0x100000U)
#define DMA_ERQ_ERQ20_SHIFT                      (20U)
#define DMA_ERQ_ERQ20(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ20_SHIFT)) & DMA_ERQ_ERQ20_MASK)
#define DMA_ERQ_ERQ21_MASK                       (0x200000U)
#define DMA_ERQ_ERQ21_SHIFT                      (21U)
#define DMA_ERQ_ERQ21(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ21_SHIFT)) & DMA_ERQ_ERQ21_MASK)
#define DMA_ERQ_ERQ22_MASK                       (0x400000U)
#define DMA_ERQ_ERQ22_SHIFT                      (22U)
#define DMA_ERQ_ERQ22(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ22_SHIFT)) & DMA_ERQ_ERQ22_MASK)
#define DMA_ERQ_ERQ23_MASK                       (0x800000U)
#define DMA_ERQ_ERQ23_SHIFT                      (23U)
#define DMA_ERQ_ERQ23(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ23_SHIFT)) & DMA_ERQ_ERQ23_MASK)
#define DMA_ERQ_ERQ24_MASK                       (0x1000000U)
#define DMA_ERQ_ERQ24_SHIFT                      (24U)
#define DMA_ERQ_ERQ24(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ24_SHIFT)) & DMA_ERQ_ERQ24_MASK)
#define DMA_ERQ_ERQ25_MASK                       (0x2000000U)
#define DMA_ERQ_ERQ25_SHIFT                      (25U)
#define DMA_ERQ_ERQ25(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ25_SHIFT)) & DMA_ERQ_ERQ25_MASK)
#define DMA_ERQ_ERQ26_MASK                       (0x4000000U)
#define DMA_ERQ_ERQ26_SHIFT                      (26U)
#define DMA_ERQ_ERQ26(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ26_SHIFT)) & DMA_ERQ_ERQ26_MASK)
#define DMA_ERQ_ERQ27_MASK                       (0x8000000U)
#define DMA_ERQ_ERQ27_SHIFT                      (27U)
#define DMA_ERQ_ERQ27(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ27_SHIFT)) & DMA_ERQ_ERQ27_MASK)
#define DMA_ERQ_ERQ28_MASK                       (0x10000000U)
#define DMA_ERQ_ERQ28_SHIFT                      (28U)
#define DMA_ERQ_ERQ28(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ28_SHIFT)) & DMA_ERQ_ERQ28_MASK)
#define DMA_ERQ_ERQ29_MASK                       (0x20000000U)
#define DMA_ERQ_ERQ29_SHIFT                      (29U)
#define DMA_ERQ_ERQ29(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ29_SHIFT)) & DMA_ERQ_ERQ29_MASK)
#define DMA_ERQ_ERQ30_MASK                       (0x40000000U)
#define DMA_ERQ_ERQ30_SHIFT                      (30U)
#define DMA_ERQ_ERQ30(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ30_SHIFT)) & DMA_ERQ_ERQ30_MASK)
#define DMA_ERQ_ERQ31_MASK                       (0x80000000U)
#define DMA_ERQ_ERQ31_SHIFT                      (31U)
#define DMA_ERQ_ERQ31(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ31_SHIFT)) & DMA_ERQ_ERQ31_MASK)

/*! @name EEI - Enable Error Interrupt Register */
#define DMA_EEI_EEI0_MASK                        (0x1U)
#define DMA_EEI_EEI0_SHIFT                       (0U)
#define DMA_EEI_EEI0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI0_SHIFT)) & DMA_EEI_EEI0_MASK)
#define DMA_EEI_EEI1_MASK                        (0x2U)
#define DMA_EEI_EEI1_SHIFT                       (1U)
#define DMA_EEI_EEI1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI1_SHIFT)) & DMA_EEI_EEI1_MASK)
#define DMA_EEI_EEI2_MASK                        (0x4U)
#define DMA_EEI_EEI2_SHIFT                       (2U)
#define DMA_EEI_EEI2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI2_SHIFT)) & DMA_EEI_EEI2_MASK)
#define DMA_EEI_EEI3_MASK                        (0x8U)
#define DMA_EEI_EEI3_SHIFT                       (3U)
#define DMA_EEI_EEI3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI3_SHIFT)) & DMA_EEI_EEI3_MASK)
#define DMA_EEI_EEI4_MASK                        (0x10U)
#define DMA_EEI_EEI4_SHIFT                       (4U)
#define DMA_EEI_EEI4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI4_SHIFT)) & DMA_EEI_EEI4_MASK)
#define DMA_EEI_EEI5_MASK                        (0x20U)
#define DMA_EEI_EEI5_SHIFT                       (5U)
#define DMA_EEI_EEI5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI5_SHIFT)) & DMA_EEI_EEI5_MASK)
#define DMA_EEI_EEI6_MASK                        (0x40U)
#define DMA_EEI_EEI6_SHIFT                       (6U)
#define DMA_EEI_EEI6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI6_SHIFT)) & DMA_EEI_EEI6_MASK)
#define DMA_EEI_EEI7_MASK                        (0x80U)
#define DMA_EEI_EEI7_SHIFT                       (7U)
#define DMA_EEI_EEI7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI7_SHIFT)) & DMA_EEI_EEI7_MASK)
#define DMA_EEI_EEI8_MASK                        (0x100U)
#define DMA_EEI_EEI8_SHIFT                       (8U)
#define DMA_EEI_EEI8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI8_SHIFT)) & DMA_EEI_EEI8_MASK)
#define DMA_EEI_EEI9_MASK                        (0x200U)
#define DMA_EEI_EEI9_SHIFT                       (9U)
#define DMA_EEI_EEI9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI9_SHIFT)) & DMA_EEI_EEI9_MASK)
#define DMA_EEI_EEI10_MASK                       (0x400U)
#define DMA_EEI_EEI10_SHIFT                      (10U)
#define DMA_EEI_EEI10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI10_SHIFT)) & DMA_EEI_EEI10_MASK)
#define DMA_EEI_EEI11_MASK                       (0x800U)
#define DMA_EEI_EEI11_SHIFT                      (11U)
#define DMA_EEI_EEI11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI11_SHIFT)) & DMA_EEI_EEI11_MASK)
#define DMA_EEI_EEI12_MASK                       (0x1000U)
#define DMA_EEI_EEI12_SHIFT                      (12U)
#define DMA_EEI_EEI12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI12_SHIFT)) & DMA_EEI_EEI12_MASK)
#define DMA_EEI_EEI13_MASK                       (0x2000U)
#define DMA_EEI_EEI13_SHIFT                      (13U)
#define DMA_EEI_EEI13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI13_SHIFT)) & DMA_EEI_EEI13_MASK)
#define DMA_EEI_EEI14_MASK                       (0x4000U)
#define DMA_EEI_EEI14_SHIFT                      (14U)
#define DMA_EEI_EEI14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI14_SHIFT)) & DMA_EEI_EEI14_MASK)
#define DMA_EEI_EEI15_MASK                       (0x8000U)
#define DMA_EEI_EEI15_SHIFT                      (15U)
#define DMA_EEI_EEI15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI15_SHIFT)) & DMA_EEI_EEI15_MASK)
#define DMA_EEI_EEI16_MASK                       (0x10000U)
#define DMA_EEI_EEI16_SHIFT                      (16U)
#define DMA_EEI_EEI16(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI16_SHIFT)) & DMA_EEI_EEI16_MASK)
#define DMA_EEI_EEI17_MASK                       (0x20000U)
#define DMA_EEI_EEI17_SHIFT                      (17U)
#define DMA_EEI_EEI17(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI17_SHIFT)) & DMA_EEI_EEI17_MASK)
#define DMA_EEI_EEI18_MASK                       (0x40000U)
#define DMA_EEI_EEI18_SHIFT                      (18U)
#define DMA_EEI_EEI18(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI18_SHIFT)) & DMA_EEI_EEI18_MASK)
#define DMA_EEI_EEI19_MASK                       (0x80000U)
#define DMA_EEI_EEI19_SHIFT                      (19U)
#define DMA_EEI_EEI19(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI19_SHIFT)) & DMA_EEI_EEI19_MASK)
#define DMA_EEI_EEI20_MASK                       (0x100000U)
#define DMA_EEI_EEI20_SHIFT                      (20U)
#define DMA_EEI_EEI20(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI20_SHIFT)) & DMA_EEI_EEI20_MASK)
#define DMA_EEI_EEI21_MASK                       (0x200000U)
#define DMA_EEI_EEI21_SHIFT                      (21U)
#define DMA_EEI_EEI21(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI21_SHIFT)) & DMA_EEI_EEI21_MASK)
#define DMA_EEI_EEI22_MASK                       (0x400000U)
#define DMA_EEI_EEI22_SHIFT                      (22U)
#define DMA_EEI_EEI22(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI22_SHIFT)) & DMA_EEI_EEI22_MASK)
#define DMA_EEI_EEI23_MASK                       (0x800000U)
#define DMA_EEI_EEI23_SHIFT                      (23U)
#define DMA_EEI_EEI23(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI23_SHIFT)) & DMA_EEI_EEI23_MASK)
#define DMA_EEI_EEI24_MASK                       (0x1000000U)
#define DMA_EEI_EEI24_SHIFT                      (24U)
#define DMA_EEI_EEI24(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI24_SHIFT)) & DMA_EEI_EEI24_MASK)
#define DMA_EEI_EEI25_MASK                       (0x2000000U)
#define DMA_EEI_EEI25_SHIFT                      (25U)
#define DMA_EEI_EEI25(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI25_SHIFT)) & DMA_EEI_EEI25_MASK)
#define DMA_EEI_EEI26_MASK                       (0x4000000U)
#define DMA_EEI_EEI26_SHIFT                      (26U)
#define DMA_EEI_EEI26(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI26_SHIFT)) & DMA_EEI_EEI26_MASK)
#define DMA_EEI_EEI27_MASK                       (0x8000000U)
#define DMA_EEI_EEI27_SHIFT                      (27U)
#define DMA_EEI_EEI27(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI27_SHIFT)) & DMA_EEI_EEI27_MASK)
#define DMA_EEI_EEI28_MASK                       (0x10000000U)
#define DMA_EEI_EEI28_SHIFT                      (28U)
#define DMA_EEI_EEI28(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI28_SHIFT)) & DMA_EEI_EEI28_MASK)
#define DMA_EEI_EEI29_MASK                       (0x20000000U)
#define DMA_EEI_EEI29_SHIFT                      (29U)
#define DMA_EEI_EEI29(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI29_SHIFT)) & DMA_EEI_EEI29_MASK)
#define DMA_EEI_EEI30_MASK                       (0x40000000U)
#define DMA_EEI_EEI30_SHIFT                      (30U)
#define DMA_EEI_EEI30(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI30_SHIFT)) & DMA_EEI_EEI30_MASK)
#define DMA_EEI_EEI31_MASK                       (0x80000000U)
#define DMA_EEI_EEI31_SHIFT                      (31U)
#define DMA_EEI_EEI31(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI31_SHIFT)) & DMA_EEI_EEI31_MASK)

/*! @name CEEI - Clear Enable Error Interrupt Register */
#define DMA_CEEI_CEEI_MASK                       (0x1FU)
#define DMA_CEEI_CEEI_SHIFT                      (0U)
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CEEI_SHIFT)) & DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       (0x40U)
#define DMA_CEEI_CAEE_SHIFT                      (6U)
#define DMA_CEEI_CAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CAEE_SHIFT)) & DMA_CEEI_CAEE_MASK)
#define DMA_CEEI_NOP_MASK                        (0x80U)
#define DMA_CEEI_NOP_SHIFT                       (7U)
#define DMA_CEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_NOP_SHIFT)) & DMA_CEEI_NOP_MASK)

/*! @name SEEI - Set Enable Error Interrupt Register */
#define DMA_SEEI_SEEI_MASK                       (0x1FU)
#define DMA_SEEI_SEEI_SHIFT                      (0U)
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SEEI_SHIFT)) & DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       (0x40U)
#define DMA_SEEI_SAEE_SHIFT                      (6U)
#define DMA_SEEI_SAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SAEE_SHIFT)) & DMA_SEEI_SAEE_MASK)
#define DMA_SEEI_NOP_MASK                        (0x80U)
#define DMA_SEEI_NOP_SHIFT                       (7U)
#define DMA_SEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_NOP_SHIFT)) & DMA_SEEI_NOP_MASK)

/*! @name CERQ - Clear Enable Request Register */
#define DMA_CERQ_CERQ_MASK                       (0x1FU)
#define DMA_CERQ_CERQ_SHIFT                      (0U)
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CERQ_SHIFT)) & DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       (0x40U)
#define DMA_CERQ_CAER_SHIFT                      (6U)
#define DMA_CERQ_CAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CAER_SHIFT)) & DMA_CERQ_CAER_MASK)
#define DMA_CERQ_NOP_MASK                        (0x80U)
#define DMA_CERQ_NOP_SHIFT                       (7U)
#define DMA_CERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_NOP_SHIFT)) & DMA_CERQ_NOP_MASK)

/*! @name SERQ - Set Enable Request Register */
#define DMA_SERQ_SERQ_MASK                       (0x1FU)
#define DMA_SERQ_SERQ_SHIFT                      (0U)
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SERQ_SHIFT)) & DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       (0x40U)
#define DMA_SERQ_SAER_SHIFT                      (6U)
#define DMA_SERQ_SAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SAER_SHIFT)) & DMA_SERQ_SAER_MASK)
#define DMA_SERQ_NOP_MASK                        (0x80U)
#define DMA_SERQ_NOP_SHIFT                       (7U)
#define DMA_SERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_NOP_SHIFT)) & DMA_SERQ_NOP_MASK)

/*! @name CDNE - Clear DONE Status Bit Register */
#define DMA_CDNE_CDNE_MASK                       (0x1FU)
#define DMA_CDNE_CDNE_SHIFT                      (0U)
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CDNE_SHIFT)) & DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       (0x40U)
#define DMA_CDNE_CADN_SHIFT                      (6U)
#define DMA_CDNE_CADN(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CADN_SHIFT)) & DMA_CDNE_CADN_MASK)
#define DMA_CDNE_NOP_MASK                        (0x80U)
#define DMA_CDNE_NOP_SHIFT                       (7U)
#define DMA_CDNE_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_NOP_SHIFT)) & DMA_CDNE_NOP_MASK)

/*! @name SSRT - Set START Bit Register */
#define DMA_SSRT_SSRT_MASK                       (0x1FU)
#define DMA_SSRT_SSRT_SHIFT                      (0U)
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SSRT_SHIFT)) & DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       (0x40U)
#define DMA_SSRT_SAST_SHIFT                      (6U)
#define DMA_SSRT_SAST(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SAST_SHIFT)) & DMA_SSRT_SAST_MASK)
#define DMA_SSRT_NOP_MASK                        (0x80U)
#define DMA_SSRT_NOP_SHIFT                       (7U)
#define DMA_SSRT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_NOP_SHIFT)) & DMA_SSRT_NOP_MASK)

/*! @name CERR - Clear Error Register */
#define DMA_CERR_CERR_MASK                       (0x1FU)
#define DMA_CERR_CERR_SHIFT                      (0U)
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CERR_SHIFT)) & DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       (0x40U)
#define DMA_CERR_CAEI_SHIFT                      (6U)
#define DMA_CERR_CAEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CAEI_SHIFT)) & DMA_CERR_CAEI_MASK)
#define DMA_CERR_NOP_MASK                        (0x80U)
#define DMA_CERR_NOP_SHIFT                       (7U)
#define DMA_CERR_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERR_NOP_SHIFT)) & DMA_CERR_NOP_MASK)

/*! @name CINT - Clear Interrupt Request Register */
#define DMA_CINT_CINT_MASK                       (0x1FU)
#define DMA_CINT_CINT_SHIFT                      (0U)
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CINT_SHIFT)) & DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR_MASK                       (0x40U)
#define DMA_CINT_CAIR_SHIFT                      (6U)
#define DMA_CINT_CAIR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CAIR_SHIFT)) & DMA_CINT_CAIR_MASK)
#define DMA_CINT_NOP_MASK                        (0x80U)
#define DMA_CINT_NOP_SHIFT                       (7U)
#define DMA_CINT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CINT_NOP_SHIFT)) & DMA_CINT_NOP_MASK)

/*! @name INT - Interrupt Request Register */
#define DMA_INT_INT0_MASK                        (0x1U)
#define DMA_INT_INT0_SHIFT                       (0U)
#define DMA_INT_INT0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT0_SHIFT)) & DMA_INT_INT0_MASK)
#define DMA_INT_INT1_MASK                        (0x2U)
#define DMA_INT_INT1_SHIFT                       (1U)
#define DMA_INT_INT1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT1_SHIFT)) & DMA_INT_INT1_MASK)
#define DMA_INT_INT2_MASK                        (0x4U)
#define DMA_INT_INT2_SHIFT                       (2U)
#define DMA_INT_INT2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT2_SHIFT)) & DMA_INT_INT2_MASK)
#define DMA_INT_INT3_MASK                        (0x8U)
#define DMA_INT_INT3_SHIFT                       (3U)
#define DMA_INT_INT3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT3_SHIFT)) & DMA_INT_INT3_MASK)
#define DMA_INT_INT4_MASK                        (0x10U)
#define DMA_INT_INT4_SHIFT                       (4U)
#define DMA_INT_INT4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT4_SHIFT)) & DMA_INT_INT4_MASK)
#define DMA_INT_INT5_MASK                        (0x20U)
#define DMA_INT_INT5_SHIFT                       (5U)
#define DMA_INT_INT5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT5_SHIFT)) & DMA_INT_INT5_MASK)
#define DMA_INT_INT6_MASK                        (0x40U)
#define DMA_INT_INT6_SHIFT                       (6U)
#define DMA_INT_INT6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT6_SHIFT)) & DMA_INT_INT6_MASK)
#define DMA_INT_INT7_MASK                        (0x80U)
#define DMA_INT_INT7_SHIFT                       (7U)
#define DMA_INT_INT7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT7_SHIFT)) & DMA_INT_INT7_MASK)
#define DMA_INT_INT8_MASK                        (0x100U)
#define DMA_INT_INT8_SHIFT                       (8U)
#define DMA_INT_INT8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT8_SHIFT)) & DMA_INT_INT8_MASK)
#define DMA_INT_INT9_MASK                        (0x200U)
#define DMA_INT_INT9_SHIFT                       (9U)
#define DMA_INT_INT9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT9_SHIFT)) & DMA_INT_INT9_MASK)
#define DMA_INT_INT10_MASK                       (0x400U)
#define DMA_INT_INT10_SHIFT                      (10U)
#define DMA_INT_INT10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT10_SHIFT)) & DMA_INT_INT10_MASK)
#define DMA_INT_INT11_MASK                       (0x800U)
#define DMA_INT_INT11_SHIFT                      (11U)
#define DMA_INT_INT11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT11_SHIFT)) & DMA_INT_INT11_MASK)
#define DMA_INT_INT12_MASK                       (0x1000U)
#define DMA_INT_INT12_SHIFT                      (12U)
#define DMA_INT_INT12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT12_SHIFT)) & DMA_INT_INT12_MASK)
#define DMA_INT_INT13_MASK                       (0x2000U)
#define DMA_INT_INT13_SHIFT                      (13U)
#define DMA_INT_INT13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT13_SHIFT)) & DMA_INT_INT13_MASK)
#define DMA_INT_INT14_MASK                       (0x4000U)
#define DMA_INT_INT14_SHIFT                      (14U)
#define DMA_INT_INT14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT14_SHIFT)) & DMA_INT_INT14_MASK)
#define DMA_INT_INT15_MASK                       (0x8000U)
#define DMA_INT_INT15_SHIFT                      (15U)
#define DMA_INT_INT15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT15_SHIFT)) & DMA_INT_INT15_MASK)
#define DMA_INT_INT16_MASK                       (0x10000U)
#define DMA_INT_INT16_SHIFT                      (16U)
#define DMA_INT_INT16(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT16_SHIFT)) & DMA_INT_INT16_MASK)
#define DMA_INT_INT17_MASK                       (0x20000U)
#define DMA_INT_INT17_SHIFT                      (17U)
#define DMA_INT_INT17(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT17_SHIFT)) & DMA_INT_INT17_MASK)
#define DMA_INT_INT18_MASK                       (0x40000U)
#define DMA_INT_INT18_SHIFT                      (18U)
#define DMA_INT_INT18(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT18_SHIFT)) & DMA_INT_INT18_MASK)
#define DMA_INT_INT19_MASK                       (0x80000U)
#define DMA_INT_INT19_SHIFT                      (19U)
#define DMA_INT_INT19(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT19_SHIFT)) & DMA_INT_INT19_MASK)
#define DMA_INT_INT20_MASK                       (0x100000U)
#define DMA_INT_INT20_SHIFT                      (20U)
#define DMA_INT_INT20(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT20_SHIFT)) & DMA_INT_INT20_MASK)
#define DMA_INT_INT21_MASK                       (0x200000U)
#define DMA_INT_INT21_SHIFT                      (21U)
#define DMA_INT_INT21(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT21_SHIFT)) & DMA_INT_INT21_MASK)
#define DMA_INT_INT22_MASK                       (0x400000U)
#define DMA_INT_INT22_SHIFT                      (22U)
#define DMA_INT_INT22(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT22_SHIFT)) & DMA_INT_INT22_MASK)
#define DMA_INT_INT23_MASK                       (0x800000U)
#define DMA_INT_INT23_SHIFT                      (23U)
#define DMA_INT_INT23(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT23_SHIFT)) & DMA_INT_INT23_MASK)
#define DMA_INT_INT24_MASK                       (0x1000000U)
#define DMA_INT_INT24_SHIFT                      (24U)
#define DMA_INT_INT24(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT24_SHIFT)) & DMA_INT_INT24_MASK)
#define DMA_INT_INT25_MASK                       (0x2000000U)
#define DMA_INT_INT25_SHIFT                      (25U)
#define DMA_INT_INT25(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT25_SHIFT)) & DMA_INT_INT25_MASK)
#define DMA_INT_INT26_MASK                       (0x4000000U)
#define DMA_INT_INT26_SHIFT                      (26U)
#define DMA_INT_INT26(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT26_SHIFT)) & DMA_INT_INT26_MASK)
#define DMA_INT_INT27_MASK                       (0x8000000U)
#define DMA_INT_INT27_SHIFT                      (27U)
#define DMA_INT_INT27(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT27_SHIFT)) & DMA_INT_INT27_MASK)
#define DMA_INT_INT28_MASK                       (0x10000000U)
#define DMA_INT_INT28_SHIFT                      (28U)
#define DMA_INT_INT28(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT28_SHIFT)) & DMA_INT_INT28_MASK)
#define DMA_INT_INT29_MASK                       (0x20000000U)
#define DMA_INT_INT29_SHIFT                      (29U)
#define DMA_INT_INT29(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT29_SHIFT)) & DMA_INT_INT29_MASK)
#define DMA_INT_INT30_MASK                       (0x40000000U)
#define DMA_INT_INT30_SHIFT                      (30U)
#define DMA_INT_INT30(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT30_SHIFT)) & DMA_INT_INT30_MASK)
#define DMA_INT_INT31_MASK                       (0x80000000U)
#define DMA_INT_INT31_SHIFT                      (31U)
#define DMA_INT_INT31(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT31_SHIFT)) & DMA_INT_INT31_MASK)

/*! @name ERR - Error Register */
#define DMA_ERR_ERR0_MASK                        (0x1U)
#define DMA_ERR_ERR0_SHIFT                       (0U)
#define DMA_ERR_ERR0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR0_SHIFT)) & DMA_ERR_ERR0_MASK)
#define DMA_ERR_ERR1_MASK                        (0x2U)
#define DMA_ERR_ERR1_SHIFT                       (1U)
#define DMA_ERR_ERR1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR1_SHIFT)) & DMA_ERR_ERR1_MASK)
#define DMA_ERR_ERR2_MASK                        (0x4U)
#define DMA_ERR_ERR2_SHIFT                       (2U)
#define DMA_ERR_ERR2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR2_SHIFT)) & DMA_ERR_ERR2_MASK)
#define DMA_ERR_ERR3_MASK                        (0x8U)
#define DMA_ERR_ERR3_SHIFT                       (3U)
#define DMA_ERR_ERR3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR3_SHIFT)) & DMA_ERR_ERR3_MASK)
#define DMA_ERR_ERR4_MASK                        (0x10U)
#define DMA_ERR_ERR4_SHIFT                       (4U)
#define DMA_ERR_ERR4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR4_SHIFT)) & DMA_ERR_ERR4_MASK)
#define DMA_ERR_ERR5_MASK                        (0x20U)
#define DMA_ERR_ERR5_SHIFT                       (5U)
#define DMA_ERR_ERR5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR5_SHIFT)) & DMA_ERR_ERR5_MASK)
#define DMA_ERR_ERR6_MASK                        (0x40U)
#define DMA_ERR_ERR6_SHIFT                       (6U)
#define DMA_ERR_ERR6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR6_SHIFT)) & DMA_ERR_ERR6_MASK)
#define DMA_ERR_ERR7_MASK                        (0x80U)
#define DMA_ERR_ERR7_SHIFT                       (7U)
#define DMA_ERR_ERR7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR7_SHIFT)) & DMA_ERR_ERR7_MASK)
#define DMA_ERR_ERR8_MASK                        (0x100U)
#define DMA_ERR_ERR8_SHIFT                       (8U)
#define DMA_ERR_ERR8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR8_SHIFT)) & DMA_ERR_ERR8_MASK)
#define DMA_ERR_ERR9_MASK                        (0x200U)
#define DMA_ERR_ERR9_SHIFT                       (9U)
#define DMA_ERR_ERR9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR9_SHIFT)) & DMA_ERR_ERR9_MASK)
#define DMA_ERR_ERR10_MASK                       (0x400U)
#define DMA_ERR_ERR10_SHIFT                      (10U)
#define DMA_ERR_ERR10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR10_SHIFT)) & DMA_ERR_ERR10_MASK)
#define DMA_ERR_ERR11_MASK                       (0x800U)
#define DMA_ERR_ERR11_SHIFT                      (11U)
#define DMA_ERR_ERR11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR11_SHIFT)) & DMA_ERR_ERR11_MASK)
#define DMA_ERR_ERR12_MASK                       (0x1000U)
#define DMA_ERR_ERR12_SHIFT                      (12U)
#define DMA_ERR_ERR12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR12_SHIFT)) & DMA_ERR_ERR12_MASK)
#define DMA_ERR_ERR13_MASK                       (0x2000U)
#define DMA_ERR_ERR13_SHIFT                      (13U)
#define DMA_ERR_ERR13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR13_SHIFT)) & DMA_ERR_ERR13_MASK)
#define DMA_ERR_ERR14_MASK                       (0x4000U)
#define DMA_ERR_ERR14_SHIFT                      (14U)
#define DMA_ERR_ERR14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR14_SHIFT)) & DMA_ERR_ERR14_MASK)
#define DMA_ERR_ERR15_MASK                       (0x8000U)
#define DMA_ERR_ERR15_SHIFT                      (15U)
#define DMA_ERR_ERR15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR15_SHIFT)) & DMA_ERR_ERR15_MASK)
#define DMA_ERR_ERR16_MASK                       (0x10000U)
#define DMA_ERR_ERR16_SHIFT                      (16U)
#define DMA_ERR_ERR16(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR16_SHIFT)) & DMA_ERR_ERR16_MASK)
#define DMA_ERR_ERR17_MASK                       (0x20000U)
#define DMA_ERR_ERR17_SHIFT                      (17U)
#define DMA_ERR_ERR17(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR17_SHIFT)) & DMA_ERR_ERR17_MASK)
#define DMA_ERR_ERR18_MASK                       (0x40000U)
#define DMA_ERR_ERR18_SHIFT                      (18U)
#define DMA_ERR_ERR18(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR18_SHIFT)) & DMA_ERR_ERR18_MASK)
#define DMA_ERR_ERR19_MASK                       (0x80000U)
#define DMA_ERR_ERR19_SHIFT                      (19U)
#define DMA_ERR_ERR19(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR19_SHIFT)) & DMA_ERR_ERR19_MASK)
#define DMA_ERR_ERR20_MASK                       (0x100000U)
#define DMA_ERR_ERR20_SHIFT                      (20U)
#define DMA_ERR_ERR20(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR20_SHIFT)) & DMA_ERR_ERR20_MASK)
#define DMA_ERR_ERR21_MASK                       (0x200000U)
#define DMA_ERR_ERR21_SHIFT                      (21U)
#define DMA_ERR_ERR21(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR21_SHIFT)) & DMA_ERR_ERR21_MASK)
#define DMA_ERR_ERR22_MASK                       (0x400000U)
#define DMA_ERR_ERR22_SHIFT                      (22U)
#define DMA_ERR_ERR22(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR22_SHIFT)) & DMA_ERR_ERR22_MASK)
#define DMA_ERR_ERR23_MASK                       (0x800000U)
#define DMA_ERR_ERR23_SHIFT                      (23U)
#define DMA_ERR_ERR23(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR23_SHIFT)) & DMA_ERR_ERR23_MASK)
#define DMA_ERR_ERR24_MASK                       (0x1000000U)
#define DMA_ERR_ERR24_SHIFT                      (24U)
#define DMA_ERR_ERR24(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR24_SHIFT)) & DMA_ERR_ERR24_MASK)
#define DMA_ERR_ERR25_MASK                       (0x2000000U)
#define DMA_ERR_ERR25_SHIFT                      (25U)
#define DMA_ERR_ERR25(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR25_SHIFT)) & DMA_ERR_ERR25_MASK)
#define DMA_ERR_ERR26_MASK                       (0x4000000U)
#define DMA_ERR_ERR26_SHIFT                      (26U)
#define DMA_ERR_ERR26(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR26_SHIFT)) & DMA_ERR_ERR26_MASK)
#define DMA_ERR_ERR27_MASK                       (0x8000000U)
#define DMA_ERR_ERR27_SHIFT                      (27U)
#define DMA_ERR_ERR27(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR27_SHIFT)) & DMA_ERR_ERR27_MASK)
#define DMA_ERR_ERR28_MASK                       (0x10000000U)
#define DMA_ERR_ERR28_SHIFT                      (28U)
#define DMA_ERR_ERR28(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR28_SHIFT)) & DMA_ERR_ERR28_MASK)
#define DMA_ERR_ERR29_MASK                       (0x20000000U)
#define DMA_ERR_ERR29_SHIFT                      (29U)
#define DMA_ERR_ERR29(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR29_SHIFT)) & DMA_ERR_ERR29_MASK)
#define DMA_ERR_ERR30_MASK                       (0x40000000U)
#define DMA_ERR_ERR30_SHIFT                      (30U)
#define DMA_ERR_ERR30(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR30_SHIFT)) & DMA_ERR_ERR30_MASK)
#define DMA_ERR_ERR31_MASK                       (0x80000000U)
#define DMA_ERR_ERR31_SHIFT                      (31U)
#define DMA_ERR_ERR31(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR31_SHIFT)) & DMA_ERR_ERR31_MASK)

/*! @name HRS - Hardware Request Status Register */
#define DMA_HRS_HRS0_MASK                        (0x1U)
#define DMA_HRS_HRS0_SHIFT                       (0U)
#define DMA_HRS_HRS0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS0_SHIFT)) & DMA_HRS_HRS0_MASK)
#define DMA_HRS_HRS1_MASK                        (0x2U)
#define DMA_HRS_HRS1_SHIFT                       (1U)
#define DMA_HRS_HRS1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS1_SHIFT)) & DMA_HRS_HRS1_MASK)
#define DMA_HRS_HRS2_MASK                        (0x4U)
#define DMA_HRS_HRS2_SHIFT                       (2U)
#define DMA_HRS_HRS2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS2_SHIFT)) & DMA_HRS_HRS2_MASK)
#define DMA_HRS_HRS3_MASK                        (0x8U)
#define DMA_HRS_HRS3_SHIFT                       (3U)
#define DMA_HRS_HRS3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS3_SHIFT)) & DMA_HRS_HRS3_MASK)
#define DMA_HRS_HRS4_MASK                        (0x10U)
#define DMA_HRS_HRS4_SHIFT                       (4U)
#define DMA_HRS_HRS4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS4_SHIFT)) & DMA_HRS_HRS4_MASK)
#define DMA_HRS_HRS5_MASK                        (0x20U)
#define DMA_HRS_HRS5_SHIFT                       (5U)
#define DMA_HRS_HRS5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS5_SHIFT)) & DMA_HRS_HRS5_MASK)
#define DMA_HRS_HRS6_MASK                        (0x40U)
#define DMA_HRS_HRS6_SHIFT                       (6U)
#define DMA_HRS_HRS6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS6_SHIFT)) & DMA_HRS_HRS6_MASK)
#define DMA_HRS_HRS7_MASK                        (0x80U)
#define DMA_HRS_HRS7_SHIFT                       (7U)
#define DMA_HRS_HRS7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS7_SHIFT)) & DMA_HRS_HRS7_MASK)
#define DMA_HRS_HRS8_MASK                        (0x100U)
#define DMA_HRS_HRS8_SHIFT                       (8U)
#define DMA_HRS_HRS8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS8_SHIFT)) & DMA_HRS_HRS8_MASK)
#define DMA_HRS_HRS9_MASK                        (0x200U)
#define DMA_HRS_HRS9_SHIFT                       (9U)
#define DMA_HRS_HRS9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS9_SHIFT)) & DMA_HRS_HRS9_MASK)
#define DMA_HRS_HRS10_MASK                       (0x400U)
#define DMA_HRS_HRS10_SHIFT                      (10U)
#define DMA_HRS_HRS10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS10_SHIFT)) & DMA_HRS_HRS10_MASK)
#define DMA_HRS_HRS11_MASK                       (0x800U)
#define DMA_HRS_HRS11_SHIFT                      (11U)
#define DMA_HRS_HRS11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS11_SHIFT)) & DMA_HRS_HRS11_MASK)
#define DMA_HRS_HRS12_MASK                       (0x1000U)
#define DMA_HRS_HRS12_SHIFT                      (12U)
#define DMA_HRS_HRS12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS12_SHIFT)) & DMA_HRS_HRS12_MASK)
#define DMA_HRS_HRS13_MASK                       (0x2000U)
#define DMA_HRS_HRS13_SHIFT                      (13U)
#define DMA_HRS_HRS13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS13_SHIFT)) & DMA_HRS_HRS13_MASK)
#define DMA_HRS_HRS14_MASK                       (0x4000U)
#define DMA_HRS_HRS14_SHIFT                      (14U)
#define DMA_HRS_HRS14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS14_SHIFT)) & DMA_HRS_HRS14_MASK)
#define DMA_HRS_HRS15_MASK                       (0x8000U)
#define DMA_HRS_HRS15_SHIFT                      (15U)
#define DMA_HRS_HRS15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS15_SHIFT)) & DMA_HRS_HRS15_MASK)
#define DMA_HRS_HRS16_MASK                       (0x10000U)
#define DMA_HRS_HRS16_SHIFT                      (16U)
#define DMA_HRS_HRS16(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS16_SHIFT)) & DMA_HRS_HRS16_MASK)
#define DMA_HRS_HRS17_MASK                       (0x20000U)
#define DMA_HRS_HRS17_SHIFT                      (17U)
#define DMA_HRS_HRS17(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS17_SHIFT)) & DMA_HRS_HRS17_MASK)
#define DMA_HRS_HRS18_MASK                       (0x40000U)
#define DMA_HRS_HRS18_SHIFT                      (18U)
#define DMA_HRS_HRS18(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS18_SHIFT)) & DMA_HRS_HRS18_MASK)
#define DMA_HRS_HRS19_MASK                       (0x80000U)
#define DMA_HRS_HRS19_SHIFT                      (19U)
#define DMA_HRS_HRS19(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS19_SHIFT)) & DMA_HRS_HRS19_MASK)
#define DMA_HRS_HRS20_MASK                       (0x100000U)
#define DMA_HRS_HRS20_SHIFT                      (20U)
#define DMA_HRS_HRS20(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS20_SHIFT)) & DMA_HRS_HRS20_MASK)
#define DMA_HRS_HRS21_MASK                       (0x200000U)
#define DMA_HRS_HRS21_SHIFT                      (21U)
#define DMA_HRS_HRS21(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS21_SHIFT)) & DMA_HRS_HRS21_MASK)
#define DMA_HRS_HRS22_MASK                       (0x400000U)
#define DMA_HRS_HRS22_SHIFT                      (22U)
#define DMA_HRS_HRS22(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS22_SHIFT)) & DMA_HRS_HRS22_MASK)
#define DMA_HRS_HRS23_MASK                       (0x800000U)
#define DMA_HRS_HRS23_SHIFT                      (23U)
#define DMA_HRS_HRS23(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS23_SHIFT)) & DMA_HRS_HRS23_MASK)
#define DMA_HRS_HRS24_MASK                       (0x1000000U)
#define DMA_HRS_HRS24_SHIFT                      (24U)
#define DMA_HRS_HRS24(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS24_SHIFT)) & DMA_HRS_HRS24_MASK)
#define DMA_HRS_HRS25_MASK                       (0x2000000U)
#define DMA_HRS_HRS25_SHIFT                      (25U)
#define DMA_HRS_HRS25(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS25_SHIFT)) & DMA_HRS_HRS25_MASK)
#define DMA_HRS_HRS26_MASK                       (0x4000000U)
#define DMA_HRS_HRS26_SHIFT                      (26U)
#define DMA_HRS_HRS26(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS26_SHIFT)) & DMA_HRS_HRS26_MASK)
#define DMA_HRS_HRS27_MASK                       (0x8000000U)
#define DMA_HRS_HRS27_SHIFT                      (27U)
#define DMA_HRS_HRS27(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS27_SHIFT)) & DMA_HRS_HRS27_MASK)
#define DMA_HRS_HRS28_MASK                       (0x10000000U)
#define DMA_HRS_HRS28_SHIFT                      (28U)
#define DMA_HRS_HRS28(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS28_SHIFT)) & DMA_HRS_HRS28_MASK)
#define DMA_HRS_HRS29_MASK                       (0x20000000U)
#define DMA_HRS_HRS29_SHIFT                      (29U)
#define DMA_HRS_HRS29(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS29_SHIFT)) & DMA_HRS_HRS29_MASK)
#define DMA_HRS_HRS30_MASK                       (0x40000000U)
#define DMA_HRS_HRS30_SHIFT                      (30U)
#define DMA_HRS_HRS30(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS30_SHIFT)) & DMA_HRS_HRS30_MASK)
#define DMA_HRS_HRS31_MASK                       (0x80000000U)
#define DMA_HRS_HRS31_SHIFT                      (31U)
#define DMA_HRS_HRS31(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS31_SHIFT)) & DMA_HRS_HRS31_MASK)

/*! @name EARS - Enable Asynchronous Request in Stop Register */
#define DMA_EARS_EDREQ_0_MASK                    (0x1U)
#define DMA_EARS_EDREQ_0_SHIFT                   (0U)
#define DMA_EARS_EDREQ_0(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_0_SHIFT)) & DMA_EARS_EDREQ_0_MASK)
#define DMA_EARS_EDREQ_1_MASK                    (0x2U)
#define DMA_EARS_EDREQ_1_SHIFT                   (1U)
#define DMA_EARS_EDREQ_1(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_1_SHIFT)) & DMA_EARS_EDREQ_1_MASK)
#define DMA_EARS_EDREQ_2_MASK                    (0x4U)
#define DMA_EARS_EDREQ_2_SHIFT                   (2U)
#define DMA_EARS_EDREQ_2(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_2_SHIFT)) & DMA_EARS_EDREQ_2_MASK)
#define DMA_EARS_EDREQ_3_MASK                    (0x8U)
#define DMA_EARS_EDREQ_3_SHIFT                   (3U)
#define DMA_EARS_EDREQ_3(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_3_SHIFT)) & DMA_EARS_EDREQ_3_MASK)
#define DMA_EARS_EDREQ_4_MASK                    (0x10U)
#define DMA_EARS_EDREQ_4_SHIFT                   (4U)
#define DMA_EARS_EDREQ_4(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_4_SHIFT)) & DMA_EARS_EDREQ_4_MASK)
#define DMA_EARS_EDREQ_5_MASK                    (0x20U)
#define DMA_EARS_EDREQ_5_SHIFT                   (5U)
#define DMA_EARS_EDREQ_5(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_5_SHIFT)) & DMA_EARS_EDREQ_5_MASK)
#define DMA_EARS_EDREQ_6_MASK                    (0x40U)
#define DMA_EARS_EDREQ_6_SHIFT                   (6U)
#define DMA_EARS_EDREQ_6(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_6_SHIFT)) & DMA_EARS_EDREQ_6_MASK)
#define DMA_EARS_EDREQ_7_MASK                    (0x80U)
#define DMA_EARS_EDREQ_7_SHIFT                   (7U)
#define DMA_EARS_EDREQ_7(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_7_SHIFT)) & DMA_EARS_EDREQ_7_MASK)
#define DMA_EARS_EDREQ_8_MASK                    (0x100U)
#define DMA_EARS_EDREQ_8_SHIFT                   (8U)
#define DMA_EARS_EDREQ_8(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_8_SHIFT)) & DMA_EARS_EDREQ_8_MASK)
#define DMA_EARS_EDREQ_9_MASK                    (0x200U)
#define DMA_EARS_EDREQ_9_SHIFT                   (9U)
#define DMA_EARS_EDREQ_9(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_9_SHIFT)) & DMA_EARS_EDREQ_9_MASK)
#define DMA_EARS_EDREQ_10_MASK                   (0x400U)
#define DMA_EARS_EDREQ_10_SHIFT                  (10U)
#define DMA_EARS_EDREQ_10(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_10_SHIFT)) & DMA_EARS_EDREQ_10_MASK)
#define DMA_EARS_EDREQ_11_MASK                   (0x800U)
#define DMA_EARS_EDREQ_11_SHIFT                  (11U)
#define DMA_EARS_EDREQ_11(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_11_SHIFT)) & DMA_EARS_EDREQ_11_MASK)
#define DMA_EARS_EDREQ_12_MASK                   (0x1000U)
#define DMA_EARS_EDREQ_12_SHIFT                  (12U)
#define DMA_EARS_EDREQ_12(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_12_SHIFT)) & DMA_EARS_EDREQ_12_MASK)
#define DMA_EARS_EDREQ_13_MASK                   (0x2000U)
#define DMA_EARS_EDREQ_13_SHIFT                  (13U)
#define DMA_EARS_EDREQ_13(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_13_SHIFT)) & DMA_EARS_EDREQ_13_MASK)
#define DMA_EARS_EDREQ_14_MASK                   (0x4000U)
#define DMA_EARS_EDREQ_14_SHIFT                  (14U)
#define DMA_EARS_EDREQ_14(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_14_SHIFT)) & DMA_EARS_EDREQ_14_MASK)
#define DMA_EARS_EDREQ_15_MASK                   (0x8000U)
#define DMA_EARS_EDREQ_15_SHIFT                  (15U)
#define DMA_EARS_EDREQ_15(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_15_SHIFT)) & DMA_EARS_EDREQ_15_MASK)
#define DMA_EARS_EDREQ_16_MASK                   (0x10000U)
#define DMA_EARS_EDREQ_16_SHIFT                  (16U)
#define DMA_EARS_EDREQ_16(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_16_SHIFT)) & DMA_EARS_EDREQ_16_MASK)
#define DMA_EARS_EDREQ_17_MASK                   (0x20000U)
#define DMA_EARS_EDREQ_17_SHIFT                  (17U)
#define DMA_EARS_EDREQ_17(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_17_SHIFT)) & DMA_EARS_EDREQ_17_MASK)
#define DMA_EARS_EDREQ_18_MASK                   (0x40000U)
#define DMA_EARS_EDREQ_18_SHIFT                  (18U)
#define DMA_EARS_EDREQ_18(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_18_SHIFT)) & DMA_EARS_EDREQ_18_MASK)
#define DMA_EARS_EDREQ_19_MASK                   (0x80000U)
#define DMA_EARS_EDREQ_19_SHIFT                  (19U)
#define DMA_EARS_EDREQ_19(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_19_SHIFT)) & DMA_EARS_EDREQ_19_MASK)
#define DMA_EARS_EDREQ_20_MASK                   (0x100000U)
#define DMA_EARS_EDREQ_20_SHIFT                  (20U)
#define DMA_EARS_EDREQ_20(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_20_SHIFT)) & DMA_EARS_EDREQ_20_MASK)
#define DMA_EARS_EDREQ_21_MASK                   (0x200000U)
#define DMA_EARS_EDREQ_21_SHIFT                  (21U)
#define DMA_EARS_EDREQ_21(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_21_SHIFT)) & DMA_EARS_EDREQ_21_MASK)
#define DMA_EARS_EDREQ_22_MASK                   (0x400000U)
#define DMA_EARS_EDREQ_22_SHIFT                  (22U)
#define DMA_EARS_EDREQ_22(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_22_SHIFT)) & DMA_EARS_EDREQ_22_MASK)
#define DMA_EARS_EDREQ_23_MASK                   (0x800000U)
#define DMA_EARS_EDREQ_23_SHIFT                  (23U)
#define DMA_EARS_EDREQ_23(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_23_SHIFT)) & DMA_EARS_EDREQ_23_MASK)
#define DMA_EARS_EDREQ_24_MASK                   (0x1000000U)
#define DMA_EARS_EDREQ_24_SHIFT                  (24U)
#define DMA_EARS_EDREQ_24(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_24_SHIFT)) & DMA_EARS_EDREQ_24_MASK)
#define DMA_EARS_EDREQ_25_MASK                   (0x2000000U)
#define DMA_EARS_EDREQ_25_SHIFT                  (25U)
#define DMA_EARS_EDREQ_25(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_25_SHIFT)) & DMA_EARS_EDREQ_25_MASK)
#define DMA_EARS_EDREQ_26_MASK                   (0x4000000U)
#define DMA_EARS_EDREQ_26_SHIFT                  (26U)
#define DMA_EARS_EDREQ_26(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_26_SHIFT)) & DMA_EARS_EDREQ_26_MASK)
#define DMA_EARS_EDREQ_27_MASK                   (0x8000000U)
#define DMA_EARS_EDREQ_27_SHIFT                  (27U)
#define DMA_EARS_EDREQ_27(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_27_SHIFT)) & DMA_EARS_EDREQ_27_MASK)
#define DMA_EARS_EDREQ_28_MASK                   (0x10000000U)
#define DMA_EARS_EDREQ_28_SHIFT                  (28U)
#define DMA_EARS_EDREQ_28(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_28_SHIFT)) & DMA_EARS_EDREQ_28_MASK)
#define DMA_EARS_EDREQ_29_MASK                   (0x20000000U)
#define DMA_EARS_EDREQ_29_SHIFT                  (29U)
#define DMA_EARS_EDREQ_29(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_29_SHIFT)) & DMA_EARS_EDREQ_29_MASK)
#define DMA_EARS_EDREQ_30_MASK                   (0x40000000U)
#define DMA_EARS_EDREQ_30_SHIFT                  (30U)
#define DMA_EARS_EDREQ_30(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_30_SHIFT)) & DMA_EARS_EDREQ_30_MASK)
#define DMA_EARS_EDREQ_31_MASK                   (0x80000000U)
#define DMA_EARS_EDREQ_31_SHIFT                  (31U)
#define DMA_EARS_EDREQ_31(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_31_SHIFT)) & DMA_EARS_EDREQ_31_MASK)

/*! @name DCHPRI3 - Channel n Priority Register */
#define DMA_DCHPRI3_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI3_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_CHPRI_SHIFT)) & DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI3_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI3_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_GRPPRI_SHIFT)) & DMA_DCHPRI3_GRPPRI_MASK)
#define DMA_DCHPRI3_DPA_MASK                     (0x40U)
#define DMA_DCHPRI3_DPA_SHIFT                    (6U)
#define DMA_DCHPRI3_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_DPA_SHIFT)) & DMA_DCHPRI3_DPA_MASK)
#define DMA_DCHPRI3_ECP_MASK                     (0x80U)
#define DMA_DCHPRI3_ECP_SHIFT                    (7U)
#define DMA_DCHPRI3_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_ECP_SHIFT)) & DMA_DCHPRI3_ECP_MASK)

/*! @name DCHPRI2 - Channel n Priority Register */
#define DMA_DCHPRI2_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI2_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_CHPRI_SHIFT)) & DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI2_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI2_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_GRPPRI_SHIFT)) & DMA_DCHPRI2_GRPPRI_MASK)
#define DMA_DCHPRI2_DPA_MASK                     (0x40U)
#define DMA_DCHPRI2_DPA_SHIFT                    (6U)
#define DMA_DCHPRI2_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_DPA_SHIFT)) & DMA_DCHPRI2_DPA_MASK)
#define DMA_DCHPRI2_ECP_MASK                     (0x80U)
#define DMA_DCHPRI2_ECP_SHIFT                    (7U)
#define DMA_DCHPRI2_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_ECP_SHIFT)) & DMA_DCHPRI2_ECP_MASK)

/*! @name DCHPRI1 - Channel n Priority Register */
#define DMA_DCHPRI1_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI1_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_CHPRI_SHIFT)) & DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI1_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI1_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_GRPPRI_SHIFT)) & DMA_DCHPRI1_GRPPRI_MASK)
#define DMA_DCHPRI1_DPA_MASK                     (0x40U)
#define DMA_DCHPRI1_DPA_SHIFT                    (6U)
#define DMA_DCHPRI1_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_DPA_SHIFT)) & DMA_DCHPRI1_DPA_MASK)
#define DMA_DCHPRI1_ECP_MASK                     (0x80U)
#define DMA_DCHPRI1_ECP_SHIFT                    (7U)
#define DMA_DCHPRI1_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_ECP_SHIFT)) & DMA_DCHPRI1_ECP_MASK)

/*! @name DCHPRI0 - Channel n Priority Register */
#define DMA_DCHPRI0_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI0_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_CHPRI_SHIFT)) & DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI0_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI0_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_GRPPRI_SHIFT)) & DMA_DCHPRI0_GRPPRI_MASK)
#define DMA_DCHPRI0_DPA_MASK                     (0x40U)
#define DMA_DCHPRI0_DPA_SHIFT                    (6U)
#define DMA_DCHPRI0_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_DPA_SHIFT)) & DMA_DCHPRI0_DPA_MASK)
#define DMA_DCHPRI0_ECP_MASK                     (0x80U)
#define DMA_DCHPRI0_ECP_SHIFT                    (7U)
#define DMA_DCHPRI0_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_ECP_SHIFT)) & DMA_DCHPRI0_ECP_MASK)

/*! @name DCHPRI7 - Channel n Priority Register */
#define DMA_DCHPRI7_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI7_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI7_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_CHPRI_SHIFT)) & DMA_DCHPRI7_CHPRI_MASK)
#define DMA_DCHPRI7_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI7_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI7_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_GRPPRI_SHIFT)) & DMA_DCHPRI7_GRPPRI_MASK)
#define DMA_DCHPRI7_DPA_MASK                     (0x40U)
#define DMA_DCHPRI7_DPA_SHIFT                    (6U)
#define DMA_DCHPRI7_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_DPA_SHIFT)) & DMA_DCHPRI7_DPA_MASK)
#define DMA_DCHPRI7_ECP_MASK                     (0x80U)
#define DMA_DCHPRI7_ECP_SHIFT                    (7U)
#define DMA_DCHPRI7_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_ECP_SHIFT)) & DMA_DCHPRI7_ECP_MASK)

/*! @name DCHPRI6 - Channel n Priority Register */
#define DMA_DCHPRI6_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI6_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI6_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_CHPRI_SHIFT)) & DMA_DCHPRI6_CHPRI_MASK)
#define DMA_DCHPRI6_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI6_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI6_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_GRPPRI_SHIFT)) & DMA_DCHPRI6_GRPPRI_MASK)
#define DMA_DCHPRI6_DPA_MASK                     (0x40U)
#define DMA_DCHPRI6_DPA_SHIFT                    (6U)
#define DMA_DCHPRI6_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_DPA_SHIFT)) & DMA_DCHPRI6_DPA_MASK)
#define DMA_DCHPRI6_ECP_MASK                     (0x80U)
#define DMA_DCHPRI6_ECP_SHIFT                    (7U)
#define DMA_DCHPRI6_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_ECP_SHIFT)) & DMA_DCHPRI6_ECP_MASK)

/*! @name DCHPRI5 - Channel n Priority Register */
#define DMA_DCHPRI5_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI5_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI5_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_CHPRI_SHIFT)) & DMA_DCHPRI5_CHPRI_MASK)
#define DMA_DCHPRI5_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI5_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI5_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_GRPPRI_SHIFT)) & DMA_DCHPRI5_GRPPRI_MASK)
#define DMA_DCHPRI5_DPA_MASK                     (0x40U)
#define DMA_DCHPRI5_DPA_SHIFT                    (6U)
#define DMA_DCHPRI5_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_DPA_SHIFT)) & DMA_DCHPRI5_DPA_MASK)
#define DMA_DCHPRI5_ECP_MASK                     (0x80U)
#define DMA_DCHPRI5_ECP_SHIFT                    (7U)
#define DMA_DCHPRI5_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_ECP_SHIFT)) & DMA_DCHPRI5_ECP_MASK)

/*! @name DCHPRI4 - Channel n Priority Register */
#define DMA_DCHPRI4_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI4_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI4_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_CHPRI_SHIFT)) & DMA_DCHPRI4_CHPRI_MASK)
#define DMA_DCHPRI4_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI4_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI4_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_GRPPRI_SHIFT)) & DMA_DCHPRI4_GRPPRI_MASK)
#define DMA_DCHPRI4_DPA_MASK                     (0x40U)
#define DMA_DCHPRI4_DPA_SHIFT                    (6U)
#define DMA_DCHPRI4_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_DPA_SHIFT)) & DMA_DCHPRI4_DPA_MASK)
#define DMA_DCHPRI4_ECP_MASK                     (0x80U)
#define DMA_DCHPRI4_ECP_SHIFT                    (7U)
#define DMA_DCHPRI4_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_ECP_SHIFT)) & DMA_DCHPRI4_ECP_MASK)

/*! @name DCHPRI11 - Channel n Priority Register */
#define DMA_DCHPRI11_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI11_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI11_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI11_CHPRI_SHIFT)) & DMA_DCHPRI11_CHPRI_MASK)
#define DMA_DCHPRI11_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI11_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI11_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI11_GRPPRI_SHIFT)) & DMA_DCHPRI11_GRPPRI_MASK)
#define DMA_DCHPRI11_DPA_MASK                    (0x40U)
#define DMA_DCHPRI11_DPA_SHIFT                   (6U)
#define DMA_DCHPRI11_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI11_DPA_SHIFT)) & DMA_DCHPRI11_DPA_MASK)
#define DMA_DCHPRI11_ECP_MASK                    (0x80U)
#define DMA_DCHPRI11_ECP_SHIFT                   (7U)
#define DMA_DCHPRI11_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI11_ECP_SHIFT)) & DMA_DCHPRI11_ECP_MASK)

/*! @name DCHPRI10 - Channel n Priority Register */
#define DMA_DCHPRI10_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI10_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI10_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI10_CHPRI_SHIFT)) & DMA_DCHPRI10_CHPRI_MASK)
#define DMA_DCHPRI10_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI10_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI10_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI10_GRPPRI_SHIFT)) & DMA_DCHPRI10_GRPPRI_MASK)
#define DMA_DCHPRI10_DPA_MASK                    (0x40U)
#define DMA_DCHPRI10_DPA_SHIFT                   (6U)
#define DMA_DCHPRI10_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI10_DPA_SHIFT)) & DMA_DCHPRI10_DPA_MASK)
#define DMA_DCHPRI10_ECP_MASK                    (0x80U)
#define DMA_DCHPRI10_ECP_SHIFT                   (7U)
#define DMA_DCHPRI10_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI10_ECP_SHIFT)) & DMA_DCHPRI10_ECP_MASK)

/*! @name DCHPRI9 - Channel n Priority Register */
#define DMA_DCHPRI9_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI9_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI9_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI9_CHPRI_SHIFT)) & DMA_DCHPRI9_CHPRI_MASK)
#define DMA_DCHPRI9_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI9_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI9_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI9_GRPPRI_SHIFT)) & DMA_DCHPRI9_GRPPRI_MASK)
#define DMA_DCHPRI9_DPA_MASK                     (0x40U)
#define DMA_DCHPRI9_DPA_SHIFT                    (6U)
#define DMA_DCHPRI9_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI9_DPA_SHIFT)) & DMA_DCHPRI9_DPA_MASK)
#define DMA_DCHPRI9_ECP_MASK                     (0x80U)
#define DMA_DCHPRI9_ECP_SHIFT                    (7U)
#define DMA_DCHPRI9_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI9_ECP_SHIFT)) & DMA_DCHPRI9_ECP_MASK)

/*! @name DCHPRI8 - Channel n Priority Register */
#define DMA_DCHPRI8_CHPRI_MASK                   (0xFU)
#define DMA_DCHPRI8_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI8_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI8_CHPRI_SHIFT)) & DMA_DCHPRI8_CHPRI_MASK)
#define DMA_DCHPRI8_GRPPRI_MASK                  (0x30U)
#define DMA_DCHPRI8_GRPPRI_SHIFT                 (4U)
#define DMA_DCHPRI8_GRPPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI8_GRPPRI_SHIFT)) & DMA_DCHPRI8_GRPPRI_MASK)
#define DMA_DCHPRI8_DPA_MASK                     (0x40U)
#define DMA_DCHPRI8_DPA_SHIFT                    (6U)
#define DMA_DCHPRI8_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI8_DPA_SHIFT)) & DMA_DCHPRI8_DPA_MASK)
#define DMA_DCHPRI8_ECP_MASK                     (0x80U)
#define DMA_DCHPRI8_ECP_SHIFT                    (7U)
#define DMA_DCHPRI8_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI8_ECP_SHIFT)) & DMA_DCHPRI8_ECP_MASK)

/*! @name DCHPRI15 - Channel n Priority Register */
#define DMA_DCHPRI15_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI15_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI15_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI15_CHPRI_SHIFT)) & DMA_DCHPRI15_CHPRI_MASK)
#define DMA_DCHPRI15_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI15_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI15_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI15_GRPPRI_SHIFT)) & DMA_DCHPRI15_GRPPRI_MASK)
#define DMA_DCHPRI15_DPA_MASK                    (0x40U)
#define DMA_DCHPRI15_DPA_SHIFT                   (6U)
#define DMA_DCHPRI15_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI15_DPA_SHIFT)) & DMA_DCHPRI15_DPA_MASK)
#define DMA_DCHPRI15_ECP_MASK                    (0x80U)
#define DMA_DCHPRI15_ECP_SHIFT                   (7U)
#define DMA_DCHPRI15_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI15_ECP_SHIFT)) & DMA_DCHPRI15_ECP_MASK)

/*! @name DCHPRI14 - Channel n Priority Register */
#define DMA_DCHPRI14_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI14_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI14_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI14_CHPRI_SHIFT)) & DMA_DCHPRI14_CHPRI_MASK)
#define DMA_DCHPRI14_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI14_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI14_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI14_GRPPRI_SHIFT)) & DMA_DCHPRI14_GRPPRI_MASK)
#define DMA_DCHPRI14_DPA_MASK                    (0x40U)
#define DMA_DCHPRI14_DPA_SHIFT                   (6U)
#define DMA_DCHPRI14_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI14_DPA_SHIFT)) & DMA_DCHPRI14_DPA_MASK)
#define DMA_DCHPRI14_ECP_MASK                    (0x80U)
#define DMA_DCHPRI14_ECP_SHIFT                   (7U)
#define DMA_DCHPRI14_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI14_ECP_SHIFT)) & DMA_DCHPRI14_ECP_MASK)

/*! @name DCHPRI13 - Channel n Priority Register */
#define DMA_DCHPRI13_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI13_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI13_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI13_CHPRI_SHIFT)) & DMA_DCHPRI13_CHPRI_MASK)
#define DMA_DCHPRI13_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI13_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI13_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI13_GRPPRI_SHIFT)) & DMA_DCHPRI13_GRPPRI_MASK)
#define DMA_DCHPRI13_DPA_MASK                    (0x40U)
#define DMA_DCHPRI13_DPA_SHIFT                   (6U)
#define DMA_DCHPRI13_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI13_DPA_SHIFT)) & DMA_DCHPRI13_DPA_MASK)
#define DMA_DCHPRI13_ECP_MASK                    (0x80U)
#define DMA_DCHPRI13_ECP_SHIFT                   (7U)
#define DMA_DCHPRI13_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI13_ECP_SHIFT)) & DMA_DCHPRI13_ECP_MASK)

/*! @name DCHPRI12 - Channel n Priority Register */
#define DMA_DCHPRI12_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI12_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI12_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI12_CHPRI_SHIFT)) & DMA_DCHPRI12_CHPRI_MASK)
#define DMA_DCHPRI12_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI12_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI12_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI12_GRPPRI_SHIFT)) & DMA_DCHPRI12_GRPPRI_MASK)
#define DMA_DCHPRI12_DPA_MASK                    (0x40U)
#define DMA_DCHPRI12_DPA_SHIFT                   (6U)
#define DMA_DCHPRI12_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI12_DPA_SHIFT)) & DMA_DCHPRI12_DPA_MASK)
#define DMA_DCHPRI12_ECP_MASK                    (0x80U)
#define DMA_DCHPRI12_ECP_SHIFT                   (7U)
#define DMA_DCHPRI12_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI12_ECP_SHIFT)) & DMA_DCHPRI12_ECP_MASK)

/*! @name DCHPRI19 - Channel n Priority Register */
#define DMA_DCHPRI19_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI19_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI19_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI19_CHPRI_SHIFT)) & DMA_DCHPRI19_CHPRI_MASK)
#define DMA_DCHPRI19_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI19_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI19_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI19_GRPPRI_SHIFT)) & DMA_DCHPRI19_GRPPRI_MASK)
#define DMA_DCHPRI19_DPA_MASK                    (0x40U)
#define DMA_DCHPRI19_DPA_SHIFT                   (6U)
#define DMA_DCHPRI19_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI19_DPA_SHIFT)) & DMA_DCHPRI19_DPA_MASK)
#define DMA_DCHPRI19_ECP_MASK                    (0x80U)
#define DMA_DCHPRI19_ECP_SHIFT                   (7U)
#define DMA_DCHPRI19_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI19_ECP_SHIFT)) & DMA_DCHPRI19_ECP_MASK)

/*! @name DCHPRI18 - Channel n Priority Register */
#define DMA_DCHPRI18_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI18_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI18_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI18_CHPRI_SHIFT)) & DMA_DCHPRI18_CHPRI_MASK)
#define DMA_DCHPRI18_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI18_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI18_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI18_GRPPRI_SHIFT)) & DMA_DCHPRI18_GRPPRI_MASK)
#define DMA_DCHPRI18_DPA_MASK                    (0x40U)
#define DMA_DCHPRI18_DPA_SHIFT                   (6U)
#define DMA_DCHPRI18_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI18_DPA_SHIFT)) & DMA_DCHPRI18_DPA_MASK)
#define DMA_DCHPRI18_ECP_MASK                    (0x80U)
#define DMA_DCHPRI18_ECP_SHIFT                   (7U)
#define DMA_DCHPRI18_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI18_ECP_SHIFT)) & DMA_DCHPRI18_ECP_MASK)

/*! @name DCHPRI17 - Channel n Priority Register */
#define DMA_DCHPRI17_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI17_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI17_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI17_CHPRI_SHIFT)) & DMA_DCHPRI17_CHPRI_MASK)
#define DMA_DCHPRI17_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI17_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI17_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI17_GRPPRI_SHIFT)) & DMA_DCHPRI17_GRPPRI_MASK)
#define DMA_DCHPRI17_DPA_MASK                    (0x40U)
#define DMA_DCHPRI17_DPA_SHIFT                   (6U)
#define DMA_DCHPRI17_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI17_DPA_SHIFT)) & DMA_DCHPRI17_DPA_MASK)
#define DMA_DCHPRI17_ECP_MASK                    (0x80U)
#define DMA_DCHPRI17_ECP_SHIFT                   (7U)
#define DMA_DCHPRI17_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI17_ECP_SHIFT)) & DMA_DCHPRI17_ECP_MASK)

/*! @name DCHPRI16 - Channel n Priority Register */
#define DMA_DCHPRI16_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI16_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI16_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI16_CHPRI_SHIFT)) & DMA_DCHPRI16_CHPRI_MASK)
#define DMA_DCHPRI16_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI16_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI16_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI16_GRPPRI_SHIFT)) & DMA_DCHPRI16_GRPPRI_MASK)
#define DMA_DCHPRI16_DPA_MASK                    (0x40U)
#define DMA_DCHPRI16_DPA_SHIFT                   (6U)
#define DMA_DCHPRI16_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI16_DPA_SHIFT)) & DMA_DCHPRI16_DPA_MASK)
#define DMA_DCHPRI16_ECP_MASK                    (0x80U)
#define DMA_DCHPRI16_ECP_SHIFT                   (7U)
#define DMA_DCHPRI16_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI16_ECP_SHIFT)) & DMA_DCHPRI16_ECP_MASK)

/*! @name DCHPRI23 - Channel n Priority Register */
#define DMA_DCHPRI23_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI23_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI23_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI23_CHPRI_SHIFT)) & DMA_DCHPRI23_CHPRI_MASK)
#define DMA_DCHPRI23_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI23_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI23_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI23_GRPPRI_SHIFT)) & DMA_DCHPRI23_GRPPRI_MASK)
#define DMA_DCHPRI23_DPA_MASK                    (0x40U)
#define DMA_DCHPRI23_DPA_SHIFT                   (6U)
#define DMA_DCHPRI23_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI23_DPA_SHIFT)) & DMA_DCHPRI23_DPA_MASK)
#define DMA_DCHPRI23_ECP_MASK                    (0x80U)
#define DMA_DCHPRI23_ECP_SHIFT                   (7U)
#define DMA_DCHPRI23_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI23_ECP_SHIFT)) & DMA_DCHPRI23_ECP_MASK)

/*! @name DCHPRI22 - Channel n Priority Register */
#define DMA_DCHPRI22_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI22_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI22_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI22_CHPRI_SHIFT)) & DMA_DCHPRI22_CHPRI_MASK)
#define DMA_DCHPRI22_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI22_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI22_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI22_GRPPRI_SHIFT)) & DMA_DCHPRI22_GRPPRI_MASK)
#define DMA_DCHPRI22_DPA_MASK                    (0x40U)
#define DMA_DCHPRI22_DPA_SHIFT                   (6U)
#define DMA_DCHPRI22_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI22_DPA_SHIFT)) & DMA_DCHPRI22_DPA_MASK)
#define DMA_DCHPRI22_ECP_MASK                    (0x80U)
#define DMA_DCHPRI22_ECP_SHIFT                   (7U)
#define DMA_DCHPRI22_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI22_ECP_SHIFT)) & DMA_DCHPRI22_ECP_MASK)

/*! @name DCHPRI21 - Channel n Priority Register */
#define DMA_DCHPRI21_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI21_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI21_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI21_CHPRI_SHIFT)) & DMA_DCHPRI21_CHPRI_MASK)
#define DMA_DCHPRI21_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI21_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI21_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI21_GRPPRI_SHIFT)) & DMA_DCHPRI21_GRPPRI_MASK)
#define DMA_DCHPRI21_DPA_MASK                    (0x40U)
#define DMA_DCHPRI21_DPA_SHIFT                   (6U)
#define DMA_DCHPRI21_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI21_DPA_SHIFT)) & DMA_DCHPRI21_DPA_MASK)
#define DMA_DCHPRI21_ECP_MASK                    (0x80U)
#define DMA_DCHPRI21_ECP_SHIFT                   (7U)
#define DMA_DCHPRI21_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI21_ECP_SHIFT)) & DMA_DCHPRI21_ECP_MASK)

/*! @name DCHPRI20 - Channel n Priority Register */
#define DMA_DCHPRI20_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI20_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI20_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI20_CHPRI_SHIFT)) & DMA_DCHPRI20_CHPRI_MASK)
#define DMA_DCHPRI20_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI20_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI20_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI20_GRPPRI_SHIFT)) & DMA_DCHPRI20_GRPPRI_MASK)
#define DMA_DCHPRI20_DPA_MASK                    (0x40U)
#define DMA_DCHPRI20_DPA_SHIFT                   (6U)
#define DMA_DCHPRI20_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI20_DPA_SHIFT)) & DMA_DCHPRI20_DPA_MASK)
#define DMA_DCHPRI20_ECP_MASK                    (0x80U)
#define DMA_DCHPRI20_ECP_SHIFT                   (7U)
#define DMA_DCHPRI20_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI20_ECP_SHIFT)) & DMA_DCHPRI20_ECP_MASK)

/*! @name DCHPRI27 - Channel n Priority Register */
#define DMA_DCHPRI27_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI27_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI27_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI27_CHPRI_SHIFT)) & DMA_DCHPRI27_CHPRI_MASK)
#define DMA_DCHPRI27_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI27_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI27_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI27_GRPPRI_SHIFT)) & DMA_DCHPRI27_GRPPRI_MASK)
#define DMA_DCHPRI27_DPA_MASK                    (0x40U)
#define DMA_DCHPRI27_DPA_SHIFT                   (6U)
#define DMA_DCHPRI27_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI27_DPA_SHIFT)) & DMA_DCHPRI27_DPA_MASK)
#define DMA_DCHPRI27_ECP_MASK                    (0x80U)
#define DMA_DCHPRI27_ECP_SHIFT                   (7U)
#define DMA_DCHPRI27_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI27_ECP_SHIFT)) & DMA_DCHPRI27_ECP_MASK)

/*! @name DCHPRI26 - Channel n Priority Register */
#define DMA_DCHPRI26_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI26_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI26_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI26_CHPRI_SHIFT)) & DMA_DCHPRI26_CHPRI_MASK)
#define DMA_DCHPRI26_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI26_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI26_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI26_GRPPRI_SHIFT)) & DMA_DCHPRI26_GRPPRI_MASK)
#define DMA_DCHPRI26_DPA_MASK                    (0x40U)
#define DMA_DCHPRI26_DPA_SHIFT                   (6U)
#define DMA_DCHPRI26_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI26_DPA_SHIFT)) & DMA_DCHPRI26_DPA_MASK)
#define DMA_DCHPRI26_ECP_MASK                    (0x80U)
#define DMA_DCHPRI26_ECP_SHIFT                   (7U)
#define DMA_DCHPRI26_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI26_ECP_SHIFT)) & DMA_DCHPRI26_ECP_MASK)

/*! @name DCHPRI25 - Channel n Priority Register */
#define DMA_DCHPRI25_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI25_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI25_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI25_CHPRI_SHIFT)) & DMA_DCHPRI25_CHPRI_MASK)
#define DMA_DCHPRI25_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI25_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI25_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI25_GRPPRI_SHIFT)) & DMA_DCHPRI25_GRPPRI_MASK)
#define DMA_DCHPRI25_DPA_MASK                    (0x40U)
#define DMA_DCHPRI25_DPA_SHIFT                   (6U)
#define DMA_DCHPRI25_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI25_DPA_SHIFT)) & DMA_DCHPRI25_DPA_MASK)
#define DMA_DCHPRI25_ECP_MASK                    (0x80U)
#define DMA_DCHPRI25_ECP_SHIFT                   (7U)
#define DMA_DCHPRI25_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI25_ECP_SHIFT)) & DMA_DCHPRI25_ECP_MASK)

/*! @name DCHPRI24 - Channel n Priority Register */
#define DMA_DCHPRI24_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI24_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI24_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI24_CHPRI_SHIFT)) & DMA_DCHPRI24_CHPRI_MASK)
#define DMA_DCHPRI24_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI24_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI24_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI24_GRPPRI_SHIFT)) & DMA_DCHPRI24_GRPPRI_MASK)
#define DMA_DCHPRI24_DPA_MASK                    (0x40U)
#define DMA_DCHPRI24_DPA_SHIFT                   (6U)
#define DMA_DCHPRI24_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI24_DPA_SHIFT)) & DMA_DCHPRI24_DPA_MASK)
#define DMA_DCHPRI24_ECP_MASK                    (0x80U)
#define DMA_DCHPRI24_ECP_SHIFT                   (7U)
#define DMA_DCHPRI24_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI24_ECP_SHIFT)) & DMA_DCHPRI24_ECP_MASK)

/*! @name DCHPRI31 - Channel n Priority Register */
#define DMA_DCHPRI31_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI31_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI31_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI31_CHPRI_SHIFT)) & DMA_DCHPRI31_CHPRI_MASK)
#define DMA_DCHPRI31_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI31_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI31_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI31_GRPPRI_SHIFT)) & DMA_DCHPRI31_GRPPRI_MASK)
#define DMA_DCHPRI31_DPA_MASK                    (0x40U)
#define DMA_DCHPRI31_DPA_SHIFT                   (6U)
#define DMA_DCHPRI31_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI31_DPA_SHIFT)) & DMA_DCHPRI31_DPA_MASK)
#define DMA_DCHPRI31_ECP_MASK                    (0x80U)
#define DMA_DCHPRI31_ECP_SHIFT                   (7U)
#define DMA_DCHPRI31_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI31_ECP_SHIFT)) & DMA_DCHPRI31_ECP_MASK)

/*! @name DCHPRI30 - Channel n Priority Register */
#define DMA_DCHPRI30_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI30_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI30_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI30_CHPRI_SHIFT)) & DMA_DCHPRI30_CHPRI_MASK)
#define DMA_DCHPRI30_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI30_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI30_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI30_GRPPRI_SHIFT)) & DMA_DCHPRI30_GRPPRI_MASK)
#define DMA_DCHPRI30_DPA_MASK                    (0x40U)
#define DMA_DCHPRI30_DPA_SHIFT                   (6U)
#define DMA_DCHPRI30_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI30_DPA_SHIFT)) & DMA_DCHPRI30_DPA_MASK)
#define DMA_DCHPRI30_ECP_MASK                    (0x80U)
#define DMA_DCHPRI30_ECP_SHIFT                   (7U)
#define DMA_DCHPRI30_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI30_ECP_SHIFT)) & DMA_DCHPRI30_ECP_MASK)

/*! @name DCHPRI29 - Channel n Priority Register */
#define DMA_DCHPRI29_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI29_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI29_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI29_CHPRI_SHIFT)) & DMA_DCHPRI29_CHPRI_MASK)
#define DMA_DCHPRI29_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI29_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI29_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI29_GRPPRI_SHIFT)) & DMA_DCHPRI29_GRPPRI_MASK)
#define DMA_DCHPRI29_DPA_MASK                    (0x40U)
#define DMA_DCHPRI29_DPA_SHIFT                   (6U)
#define DMA_DCHPRI29_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI29_DPA_SHIFT)) & DMA_DCHPRI29_DPA_MASK)
#define DMA_DCHPRI29_ECP_MASK                    (0x80U)
#define DMA_DCHPRI29_ECP_SHIFT                   (7U)
#define DMA_DCHPRI29_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI29_ECP_SHIFT)) & DMA_DCHPRI29_ECP_MASK)

/*! @name DCHPRI28 - Channel n Priority Register */
#define DMA_DCHPRI28_CHPRI_MASK                  (0xFU)
#define DMA_DCHPRI28_CHPRI_SHIFT                 (0U)
#define DMA_DCHPRI28_CHPRI(x)                    (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI28_CHPRI_SHIFT)) & DMA_DCHPRI28_CHPRI_MASK)
#define DMA_DCHPRI28_GRPPRI_MASK                 (0x30U)
#define DMA_DCHPRI28_GRPPRI_SHIFT                (4U)
#define DMA_DCHPRI28_GRPPRI(x)                   (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI28_GRPPRI_SHIFT)) & DMA_DCHPRI28_GRPPRI_MASK)
#define DMA_DCHPRI28_DPA_MASK                    (0x40U)
#define DMA_DCHPRI28_DPA_SHIFT                   (6U)
#define DMA_DCHPRI28_DPA(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI28_DPA_SHIFT)) & DMA_DCHPRI28_DPA_MASK)
#define DMA_DCHPRI28_ECP_MASK                    (0x80U)
#define DMA_DCHPRI28_ECP_SHIFT                   (7U)
#define DMA_DCHPRI28_ECP(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI28_ECP_SHIFT)) & DMA_DCHPRI28_ECP_MASK)

/*! @name SADDR - TCD Source Address */
#define DMA_SADDR_SADDR_MASK                     (0xFFFFFFFFU)
#define DMA_SADDR_SADDR_SHIFT                    (0U)
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SADDR_SADDR_SHIFT)) & DMA_SADDR_SADDR_MASK)

/* The count of DMA_SADDR */
#define DMA_SADDR_COUNT                          (32U)

/*! @name SOFF - TCD Signed Source Address Offset */
#define DMA_SOFF_SOFF_MASK                       (0xFFFFU)
#define DMA_SOFF_SOFF_SHIFT                      (0U)
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_SOFF_SOFF_SHIFT)) & DMA_SOFF_SOFF_MASK)

/* The count of DMA_SOFF */
#define DMA_SOFF_COUNT                           (32U)

/*! @name ATTR - TCD Transfer Attributes */
#define DMA_ATTR_DSIZE_MASK                      (0x7U)
#define DMA_ATTR_DSIZE_SHIFT                     (0U)
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DSIZE_SHIFT)) & DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       (0xF8U)
#define DMA_ATTR_DMOD_SHIFT                      (3U)
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DMOD_SHIFT)) & DMA_ATTR_DMOD_MASK)
#define DMA_ATTR_SSIZE_MASK                      (0x700U)
#define DMA_ATTR_SSIZE_SHIFT                     (8U)
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SSIZE_SHIFT)) & DMA_ATTR_SSIZE_MASK)
#define DMA_ATTR_SMOD_MASK                       (0xF800U)
#define DMA_ATTR_SMOD_SHIFT                      (11U)
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SMOD_SHIFT)) & DMA_ATTR_SMOD_MASK)

/* The count of DMA_ATTR */
#define DMA_ATTR_COUNT                           (32U)

/*! @name NBYTES_MLNO - TCD Minor Byte Count (Minor Loop Mapping Disabled) */
#define DMA_NBYTES_MLNO_NBYTES_MASK              (0xFFFFFFFFU)
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             (0U)
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLNO_NBYTES_SHIFT)) & DMA_NBYTES_MLNO_NBYTES_MASK)

/* The count of DMA_NBYTES_MLNO */
#define DMA_NBYTES_MLNO_COUNT                    (32U)

/*! @name NBYTES_MLOFFNO - TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled) */
#define DMA_NBYTES_MLOFFNO_NBYTES_MASK           (0x3FFFFFFFU)
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT          (0U)
#define DMA_NBYTES_MLOFFNO_NBYTES(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFNO_NBYTES_MASK)
#define DMA_NBYTES_MLOFFNO_DMLOE_MASK            (0x40000000U)
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT           (30U)
#define DMA_NBYTES_MLOFFNO_DMLOE(x)              (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_DMLOE_MASK)
#define DMA_NBYTES_MLOFFNO_SMLOE_MASK            (0x80000000U)
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT           (31U)
#define DMA_NBYTES_MLOFFNO_SMLOE(x)              (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_SMLOE_MASK)

/* The count of DMA_NBYTES_MLOFFNO */
#define DMA_NBYTES_MLOFFNO_COUNT                 (32U)

/*! @name NBYTES_MLOFFYES - TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled) */
#define DMA_NBYTES_MLOFFYES_NBYTES_MASK          (0x3FFU)
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT         (0U)
#define DMA_NBYTES_MLOFFYES_NBYTES(x)            (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           (0x3FFFFC00U)
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          (10U)
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_MLOFF_SHIFT)) & DMA_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_NBYTES_MLOFFYES_DMLOE_MASK           (0x40000000U)
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT          (30U)
#define DMA_NBYTES_MLOFFYES_DMLOE(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_DMLOE_MASK)
#define DMA_NBYTES_MLOFFYES_SMLOE_MASK           (0x80000000U)
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT          (31U)
#define DMA_NBYTES_MLOFFYES_SMLOE(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_SMLOE_MASK)

/* The count of DMA_NBYTES_MLOFFYES */
#define DMA_NBYTES_MLOFFYES_COUNT                (32U)

/*! @name SLAST - TCD Last Source Address Adjustment */
#define DMA_SLAST_SLAST_MASK                     (0xFFFFFFFFU)
#define DMA_SLAST_SLAST_SHIFT                    (0U)
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SLAST_SLAST_SHIFT)) & DMA_SLAST_SLAST_MASK)

/* The count of DMA_SLAST */
#define DMA_SLAST_COUNT                          (32U)

/*! @name DADDR - TCD Destination Address */
#define DMA_DADDR_DADDR_MASK                     (0xFFFFFFFFU)
#define DMA_DADDR_DADDR_SHIFT                    (0U)
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DADDR_DADDR_SHIFT)) & DMA_DADDR_DADDR_MASK)

/* The count of DMA_DADDR */
#define DMA_DADDR_COUNT                          (32U)

/*! @name DOFF - TCD Signed Destination Address Offset */
#define DMA_DOFF_DOFF_MASK                       (0xFFFFU)
#define DMA_DOFF_DOFF_SHIFT                      (0U)
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_DOFF_DOFF_SHIFT)) & DMA_DOFF_DOFF_MASK)

/* The count of DMA_DOFF */
#define DMA_DOFF_COUNT                           (32U)

/*! @name CITER_ELINKNO - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
#define DMA_CITER_ELINKNO_CITER_MASK             (0x7FFFU)
#define DMA_CITER_ELINKNO_CITER_SHIFT            (0U)
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_CITER_SHIFT)) & DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_CITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_CITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_ELINK_SHIFT)) & DMA_CITER_ELINKNO_ELINK_MASK)

/* The count of DMA_CITER_ELINKNO */
#define DMA_CITER_ELINKNO_COUNT                  (32U)

/*! @name CITER_ELINKYES - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
#define DMA_CITER_ELINKYES_CITER_MASK            (0x1FFU)
#define DMA_CITER_ELINKYES_CITER_SHIFT           (0U)
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_CITER_SHIFT)) & DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           (0x3E00U)
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_LINKCH_SHIFT)) & DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_CITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_CITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_ELINK_SHIFT)) & DMA_CITER_ELINKYES_ELINK_MASK)

/* The count of DMA_CITER_ELINKYES */
#define DMA_CITER_ELINKYES_COUNT                 (32U)

/*! @name DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address */
#define DMA_DLAST_SGA_DLASTSGA_MASK              (0xFFFFFFFFU)
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             (0U)
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x)) << DMA_DLAST_SGA_DLASTSGA_SHIFT)) & DMA_DLAST_SGA_DLASTSGA_MASK)

/* The count of DMA_DLAST_SGA */
#define DMA_DLAST_SGA_COUNT                      (32U)

/*! @name CSR - TCD Control and Status */
#define DMA_CSR_START_MASK                       (0x1U)
#define DMA_CSR_START_SHIFT                      (0U)
#define DMA_CSR_START(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_CSR_START_SHIFT)) & DMA_CSR_START_MASK)
#define DMA_CSR_INTMAJOR_MASK                    (0x2U)
#define DMA_CSR_INTMAJOR_SHIFT                   (1U)
#define DMA_CSR_INTMAJOR(x)                      (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTMAJOR_SHIFT)) & DMA_CSR_INTMAJOR_MASK)
#define DMA_CSR_INTHALF_MASK                     (0x4U)
#define DMA_CSR_INTHALF_SHIFT                    (2U)
#define DMA_CSR_INTHALF(x)                       (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTHALF_SHIFT)) & DMA_CSR_INTHALF_MASK)
#define DMA_CSR_DREQ_MASK                        (0x8U)
#define DMA_CSR_DREQ_SHIFT                       (3U)
#define DMA_CSR_DREQ(x)                          (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DREQ_SHIFT)) & DMA_CSR_DREQ_MASK)
#define DMA_CSR_ESG_MASK                         (0x10U)
#define DMA_CSR_ESG_SHIFT                        (4U)
#define DMA_CSR_ESG(x)                           (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ESG_SHIFT)) & DMA_CSR_ESG_MASK)
#define DMA_CSR_MAJORELINK_MASK                  (0x20U)
#define DMA_CSR_MAJORELINK_SHIFT                 (5U)
#define DMA_CSR_MAJORELINK(x)                    (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORELINK_SHIFT)) & DMA_CSR_MAJORELINK_MASK)
#define DMA_CSR_ACTIVE_MASK                      (0x40U)
#define DMA_CSR_ACTIVE_SHIFT                     (6U)
#define DMA_CSR_ACTIVE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ACTIVE_SHIFT)) & DMA_CSR_ACTIVE_MASK)
#define DMA_CSR_DONE_MASK                        (0x80U)
#define DMA_CSR_DONE_SHIFT                       (7U)
#define DMA_CSR_DONE(x)                          (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DONE_SHIFT)) & DMA_CSR_DONE_MASK)
#define DMA_CSR_MAJORLINKCH_MASK                 (0x1F00U)
#define DMA_CSR_MAJORLINKCH_SHIFT                (8U)
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORLINKCH_SHIFT)) & DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         (0xC000U)
#define DMA_CSR_BWC_SHIFT                        (14U)
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x)) << DMA_CSR_BWC_SHIFT)) & DMA_CSR_BWC_MASK)

/* The count of DMA_CSR */
#define DMA_CSR_COUNT                            (32U)

/*! @name BITER_ELINKNO - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
#define DMA_BITER_ELINKNO_BITER_MASK             (0x7FFFU)
#define DMA_BITER_ELINKNO_BITER_SHIFT            (0U)
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_BITER_SHIFT)) & DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_BITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_BITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_ELINK_SHIFT)) & DMA_BITER_ELINKNO_ELINK_MASK)

/* The count of DMA_BITER_ELINKNO */
#define DMA_BITER_ELINKNO_COUNT                  (32U)

/*! @name BITER_ELINKYES - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
#define DMA_BITER_ELINKYES_BITER_MASK            (0x1FFU)
#define DMA_BITER_ELINKYES_BITER_SHIFT           (0U)
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_BITER_SHIFT)) & DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           (0x3E00U)
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_LINKCH_SHIFT)) & DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_BITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_BITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_ELINK_SHIFT)) & DMA_BITER_ELINKYES_ELINK_MASK)

/* The count of DMA_BITER_ELINKYES */
#define DMA_BITER_ELINKYES_COUNT                 (32U)


/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base address */
#define DMA_BASE                                 (0x40008000u)
/** Peripheral DMA base pointer */
#define DMA0                                     ((DMA_Type *)DMA_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_CHN_IRQS                             { DMA0_DMA16_IRQn, DMA1_DMA17_IRQn, DMA2_DMA18_IRQn, DMA3_DMA19_IRQn, DMA4_DMA20_IRQn, DMA5_DMA21_IRQn, DMA6_DMA22_IRQn, DMA7_DMA23_IRQn, DMA8_DMA24_IRQn, DMA9_DMA25_IRQn, DMA10_DMA26_IRQn, DMA11_DMA27_IRQn, DMA12_DMA28_IRQn, DMA13_DMA29_IRQn, DMA14_DMA30_IRQn, DMA15_DMA31_IRQn, DMA0_DMA16_IRQn, DMA1_DMA17_IRQn, DMA2_DMA18_IRQn, DMA3_DMA19_IRQn, DMA4_DMA20_IRQn, DMA5_DMA21_IRQn, DMA6_DMA22_IRQn, DMA7_DMA23_IRQn, DMA8_DMA24_IRQn, DMA9_DMA25_IRQn, DMA10_DMA26_IRQn, DMA11_DMA27_IRQn, DMA12_DMA28_IRQn, DMA13_DMA29_IRQn, DMA14_DMA30_IRQn, DMA15_DMA31_IRQn }
#define DMA_ERROR_IRQS                           { DMA_Error_IRQn }

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Register Layout Typedef */
typedef struct {
  __IO uint8_t CHCFG[32];                          /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_Type;

/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/*! @name CHCFG - Channel Configuration register */
#define DMAMUX_CHCFG_SOURCE_MASK                 (0x3FU)
#define DMAMUX_CHCFG_SOURCE_SHIFT                (0U)
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_SOURCE_SHIFT)) & DMAMUX_CHCFG_SOURCE_MASK)
#define DMAMUX_CHCFG_TRIG_MASK                   (0x40U)
#define DMAMUX_CHCFG_TRIG_SHIFT                  (6U)
#define DMAMUX_CHCFG_TRIG(x)                     (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_TRIG_SHIFT)) & DMAMUX_CHCFG_TRIG_MASK)
#define DMAMUX_CHCFG_ENBL_MASK                   (0x80U)
#define DMAMUX_CHCFG_ENBL_SHIFT                  (7U)
#define DMAMUX_CHCFG_ENBL(x)                     (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_ENBL_SHIFT)) & DMAMUX_CHCFG_ENBL_MASK)

/* The count of DMAMUX_CHCFG */
#define DMAMUX_CHCFG_COUNT                       (32U)


/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE                              (0x40021000u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX                                   ((DMAMUX_Type *)DMAMUX_BASE)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX }

/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EMVSIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EMVSIM_Peripheral_Access_Layer EMVSIM Peripheral Access Layer
 * @{
 */

/** EMVSIM - Register Layout Typedef */
typedef struct {
  __I  uint32_t VER_ID;                            /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t CLKCFG;                            /**< Clock Configuration Register, offset: 0x8 */
  __IO uint32_t DIVISOR;                           /**< Baud Rate Divisor Register, offset: 0xC */
  __IO uint32_t CTRL;                              /**< Control Register, offset: 0x10 */
  __IO uint32_t INT_MASK;                          /**< Interrupt Mask Register, offset: 0x14 */
  __IO uint32_t RX_THD;                            /**< Receiver Threshold Register, offset: 0x18 */
  __IO uint32_t TX_THD;                            /**< Transmitter Threshold Register, offset: 0x1C */
  __IO uint32_t RX_STATUS;                         /**< Receive Status Register, offset: 0x20 */
  __IO uint32_t TX_STATUS;                         /**< Transmitter Status Register, offset: 0x24 */
  __IO uint32_t PCSR;                              /**< Port Control and Status Register, offset: 0x28 */
  __I  uint32_t RX_BUF;                            /**< Receive Data Read Buffer, offset: 0x2C */
  __IO uint32_t TX_BUF;                            /**< Transmit Data Buffer, offset: 0x30 */
  __IO uint32_t TX_GETU;                           /**< Transmitter Guard ETU Value Register, offset: 0x34 */
  __IO uint32_t CWT_VAL;                           /**< Character Wait Time Value Register, offset: 0x38 */
  __IO uint32_t BWT_VAL;                           /**< Block Wait Time Value Register, offset: 0x3C */
  __IO uint32_t BGT_VAL;                           /**< Block Guard Time Value Register, offset: 0x40 */
  __IO uint32_t GPCNT0_VAL;                        /**< General Purpose Counter 0 Timeout Value Register, offset: 0x44 */
  __IO uint32_t GPCNT1_VAL;                        /**< General Purpose Counter 1 Timeout Value, offset: 0x48 */
} EMVSIM_Type;

/* ----------------------------------------------------------------------------
   -- EMVSIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EMVSIM_Register_Masks EMVSIM Register Masks
 * @{
 */

/*! @name VER_ID - Version ID Register */
#define EMVSIM_VER_ID_VER_MASK                   (0xFFFFFFFFU)
#define EMVSIM_VER_ID_VER_SHIFT                  (0U)
#define EMVSIM_VER_ID_VER(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_VER_ID_VER_SHIFT)) & EMVSIM_VER_ID_VER_MASK)

/*! @name PARAM - Parameter Register */
#define EMVSIM_PARAM_RX_FIFO_DEPTH_MASK          (0xFFU)
#define EMVSIM_PARAM_RX_FIFO_DEPTH_SHIFT         (0U)
#define EMVSIM_PARAM_RX_FIFO_DEPTH(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_PARAM_RX_FIFO_DEPTH_SHIFT)) & EMVSIM_PARAM_RX_FIFO_DEPTH_MASK)
#define EMVSIM_PARAM_TX_FIFO_DEPTH_MASK          (0xFF00U)
#define EMVSIM_PARAM_TX_FIFO_DEPTH_SHIFT         (8U)
#define EMVSIM_PARAM_TX_FIFO_DEPTH(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_PARAM_TX_FIFO_DEPTH_SHIFT)) & EMVSIM_PARAM_TX_FIFO_DEPTH_MASK)

/*! @name CLKCFG - Clock Configuration Register */
#define EMVSIM_CLKCFG_CLK_PRSC_MASK              (0xFFU)
#define EMVSIM_CLKCFG_CLK_PRSC_SHIFT             (0U)
#define EMVSIM_CLKCFG_CLK_PRSC(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_CLKCFG_CLK_PRSC_SHIFT)) & EMVSIM_CLKCFG_CLK_PRSC_MASK)
#define EMVSIM_CLKCFG_GPCNT1_CLK_SEL_MASK        (0x300U)
#define EMVSIM_CLKCFG_GPCNT1_CLK_SEL_SHIFT       (8U)
#define EMVSIM_CLKCFG_GPCNT1_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << EMVSIM_CLKCFG_GPCNT1_CLK_SEL_SHIFT)) & EMVSIM_CLKCFG_GPCNT1_CLK_SEL_MASK)
#define EMVSIM_CLKCFG_GPCNT0_CLK_SEL_MASK        (0xC00U)
#define EMVSIM_CLKCFG_GPCNT0_CLK_SEL_SHIFT       (10U)
#define EMVSIM_CLKCFG_GPCNT0_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << EMVSIM_CLKCFG_GPCNT0_CLK_SEL_SHIFT)) & EMVSIM_CLKCFG_GPCNT0_CLK_SEL_MASK)

/*! @name DIVISOR - Baud Rate Divisor Register */
#define EMVSIM_DIVISOR_DIVISOR_VALUE_MASK        (0x1FFU)
#define EMVSIM_DIVISOR_DIVISOR_VALUE_SHIFT       (0U)
#define EMVSIM_DIVISOR_DIVISOR_VALUE(x)          (((uint32_t)(((uint32_t)(x)) << EMVSIM_DIVISOR_DIVISOR_VALUE_SHIFT)) & EMVSIM_DIVISOR_DIVISOR_VALUE_MASK)

/*! @name CTRL - Control Register */
#define EMVSIM_CTRL_IC_MASK                      (0x1U)
#define EMVSIM_CTRL_IC_SHIFT                     (0U)
#define EMVSIM_CTRL_IC(x)                        (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_IC_SHIFT)) & EMVSIM_CTRL_IC_MASK)
#define EMVSIM_CTRL_ICM_MASK                     (0x2U)
#define EMVSIM_CTRL_ICM_SHIFT                    (1U)
#define EMVSIM_CTRL_ICM(x)                       (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_ICM_SHIFT)) & EMVSIM_CTRL_ICM_MASK)
#define EMVSIM_CTRL_ANACK_MASK                   (0x4U)
#define EMVSIM_CTRL_ANACK_SHIFT                  (2U)
#define EMVSIM_CTRL_ANACK(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_ANACK_SHIFT)) & EMVSIM_CTRL_ANACK_MASK)
#define EMVSIM_CTRL_ONACK_MASK                   (0x8U)
#define EMVSIM_CTRL_ONACK_SHIFT                  (3U)
#define EMVSIM_CTRL_ONACK(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_ONACK_SHIFT)) & EMVSIM_CTRL_ONACK_MASK)
#define EMVSIM_CTRL_FLSH_RX_MASK                 (0x100U)
#define EMVSIM_CTRL_FLSH_RX_SHIFT                (8U)
#define EMVSIM_CTRL_FLSH_RX(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_FLSH_RX_SHIFT)) & EMVSIM_CTRL_FLSH_RX_MASK)
#define EMVSIM_CTRL_FLSH_TX_MASK                 (0x200U)
#define EMVSIM_CTRL_FLSH_TX_SHIFT                (9U)
#define EMVSIM_CTRL_FLSH_TX(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_FLSH_TX_SHIFT)) & EMVSIM_CTRL_FLSH_TX_MASK)
#define EMVSIM_CTRL_SW_RST_MASK                  (0x400U)
#define EMVSIM_CTRL_SW_RST_SHIFT                 (10U)
#define EMVSIM_CTRL_SW_RST(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_SW_RST_SHIFT)) & EMVSIM_CTRL_SW_RST_MASK)
#define EMVSIM_CTRL_KILL_CLOCKS_MASK             (0x800U)
#define EMVSIM_CTRL_KILL_CLOCKS_SHIFT            (11U)
#define EMVSIM_CTRL_KILL_CLOCKS(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_KILL_CLOCKS_SHIFT)) & EMVSIM_CTRL_KILL_CLOCKS_MASK)
#define EMVSIM_CTRL_DOZE_EN_MASK                 (0x1000U)
#define EMVSIM_CTRL_DOZE_EN_SHIFT                (12U)
#define EMVSIM_CTRL_DOZE_EN(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_DOZE_EN_SHIFT)) & EMVSIM_CTRL_DOZE_EN_MASK)
#define EMVSIM_CTRL_STOP_EN_MASK                 (0x2000U)
#define EMVSIM_CTRL_STOP_EN_SHIFT                (13U)
#define EMVSIM_CTRL_STOP_EN(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_STOP_EN_SHIFT)) & EMVSIM_CTRL_STOP_EN_MASK)
#define EMVSIM_CTRL_RCV_EN_MASK                  (0x10000U)
#define EMVSIM_CTRL_RCV_EN_SHIFT                 (16U)
#define EMVSIM_CTRL_RCV_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_RCV_EN_SHIFT)) & EMVSIM_CTRL_RCV_EN_MASK)
#define EMVSIM_CTRL_XMT_EN_MASK                  (0x20000U)
#define EMVSIM_CTRL_XMT_EN_SHIFT                 (17U)
#define EMVSIM_CTRL_XMT_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_XMT_EN_SHIFT)) & EMVSIM_CTRL_XMT_EN_MASK)
#define EMVSIM_CTRL_RCVR_11_MASK                 (0x40000U)
#define EMVSIM_CTRL_RCVR_11_SHIFT                (18U)
#define EMVSIM_CTRL_RCVR_11(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_RCVR_11_SHIFT)) & EMVSIM_CTRL_RCVR_11_MASK)
#define EMVSIM_CTRL_RX_DMA_EN_MASK               (0x80000U)
#define EMVSIM_CTRL_RX_DMA_EN_SHIFT              (19U)
#define EMVSIM_CTRL_RX_DMA_EN(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_RX_DMA_EN_SHIFT)) & EMVSIM_CTRL_RX_DMA_EN_MASK)
#define EMVSIM_CTRL_TX_DMA_EN_MASK               (0x100000U)
#define EMVSIM_CTRL_TX_DMA_EN_SHIFT              (20U)
#define EMVSIM_CTRL_TX_DMA_EN(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_TX_DMA_EN_SHIFT)) & EMVSIM_CTRL_TX_DMA_EN_MASK)
#define EMVSIM_CTRL_INV_CRC_VAL_MASK             (0x1000000U)
#define EMVSIM_CTRL_INV_CRC_VAL_SHIFT            (24U)
#define EMVSIM_CTRL_INV_CRC_VAL(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_INV_CRC_VAL_SHIFT)) & EMVSIM_CTRL_INV_CRC_VAL_MASK)
#define EMVSIM_CTRL_CRC_OUT_FLIP_MASK            (0x2000000U)
#define EMVSIM_CTRL_CRC_OUT_FLIP_SHIFT           (25U)
#define EMVSIM_CTRL_CRC_OUT_FLIP(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_CRC_OUT_FLIP_SHIFT)) & EMVSIM_CTRL_CRC_OUT_FLIP_MASK)
#define EMVSIM_CTRL_CRC_IN_FLIP_MASK             (0x4000000U)
#define EMVSIM_CTRL_CRC_IN_FLIP_SHIFT            (26U)
#define EMVSIM_CTRL_CRC_IN_FLIP(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_CRC_IN_FLIP_SHIFT)) & EMVSIM_CTRL_CRC_IN_FLIP_MASK)
#define EMVSIM_CTRL_CWT_EN_MASK                  (0x8000000U)
#define EMVSIM_CTRL_CWT_EN_SHIFT                 (27U)
#define EMVSIM_CTRL_CWT_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_CWT_EN_SHIFT)) & EMVSIM_CTRL_CWT_EN_MASK)
#define EMVSIM_CTRL_LRC_EN_MASK                  (0x10000000U)
#define EMVSIM_CTRL_LRC_EN_SHIFT                 (28U)
#define EMVSIM_CTRL_LRC_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_LRC_EN_SHIFT)) & EMVSIM_CTRL_LRC_EN_MASK)
#define EMVSIM_CTRL_CRC_EN_MASK                  (0x20000000U)
#define EMVSIM_CTRL_CRC_EN_SHIFT                 (29U)
#define EMVSIM_CTRL_CRC_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_CRC_EN_SHIFT)) & EMVSIM_CTRL_CRC_EN_MASK)
#define EMVSIM_CTRL_XMT_CRC_LRC_MASK             (0x40000000U)
#define EMVSIM_CTRL_XMT_CRC_LRC_SHIFT            (30U)
#define EMVSIM_CTRL_XMT_CRC_LRC(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_XMT_CRC_LRC_SHIFT)) & EMVSIM_CTRL_XMT_CRC_LRC_MASK)
#define EMVSIM_CTRL_BWT_EN_MASK                  (0x80000000U)
#define EMVSIM_CTRL_BWT_EN_SHIFT                 (31U)
#define EMVSIM_CTRL_BWT_EN(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CTRL_BWT_EN_SHIFT)) & EMVSIM_CTRL_BWT_EN_MASK)

/*! @name INT_MASK - Interrupt Mask Register */
#define EMVSIM_INT_MASK_RDT_IM_MASK              (0x1U)
#define EMVSIM_INT_MASK_RDT_IM_SHIFT             (0U)
#define EMVSIM_INT_MASK_RDT_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_RDT_IM_SHIFT)) & EMVSIM_INT_MASK_RDT_IM_MASK)
#define EMVSIM_INT_MASK_TC_IM_MASK               (0x2U)
#define EMVSIM_INT_MASK_TC_IM_SHIFT              (1U)
#define EMVSIM_INT_MASK_TC_IM(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_TC_IM_SHIFT)) & EMVSIM_INT_MASK_TC_IM_MASK)
#define EMVSIM_INT_MASK_RFO_IM_MASK              (0x4U)
#define EMVSIM_INT_MASK_RFO_IM_SHIFT             (2U)
#define EMVSIM_INT_MASK_RFO_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_RFO_IM_SHIFT)) & EMVSIM_INT_MASK_RFO_IM_MASK)
#define EMVSIM_INT_MASK_ETC_IM_MASK              (0x8U)
#define EMVSIM_INT_MASK_ETC_IM_SHIFT             (3U)
#define EMVSIM_INT_MASK_ETC_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_ETC_IM_SHIFT)) & EMVSIM_INT_MASK_ETC_IM_MASK)
#define EMVSIM_INT_MASK_TFE_IM_MASK              (0x10U)
#define EMVSIM_INT_MASK_TFE_IM_SHIFT             (4U)
#define EMVSIM_INT_MASK_TFE_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_TFE_IM_SHIFT)) & EMVSIM_INT_MASK_TFE_IM_MASK)
#define EMVSIM_INT_MASK_TNACK_IM_MASK            (0x20U)
#define EMVSIM_INT_MASK_TNACK_IM_SHIFT           (5U)
#define EMVSIM_INT_MASK_TNACK_IM(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_TNACK_IM_SHIFT)) & EMVSIM_INT_MASK_TNACK_IM_MASK)
#define EMVSIM_INT_MASK_TFF_IM_MASK              (0x40U)
#define EMVSIM_INT_MASK_TFF_IM_SHIFT             (6U)
#define EMVSIM_INT_MASK_TFF_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_TFF_IM_SHIFT)) & EMVSIM_INT_MASK_TFF_IM_MASK)
#define EMVSIM_INT_MASK_TDT_IM_MASK              (0x80U)
#define EMVSIM_INT_MASK_TDT_IM_SHIFT             (7U)
#define EMVSIM_INT_MASK_TDT_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_TDT_IM_SHIFT)) & EMVSIM_INT_MASK_TDT_IM_MASK)
#define EMVSIM_INT_MASK_GPCNT0_IM_MASK           (0x100U)
#define EMVSIM_INT_MASK_GPCNT0_IM_SHIFT          (8U)
#define EMVSIM_INT_MASK_GPCNT0_IM(x)             (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_GPCNT0_IM_SHIFT)) & EMVSIM_INT_MASK_GPCNT0_IM_MASK)
#define EMVSIM_INT_MASK_CWT_ERR_IM_MASK          (0x200U)
#define EMVSIM_INT_MASK_CWT_ERR_IM_SHIFT         (9U)
#define EMVSIM_INT_MASK_CWT_ERR_IM(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_CWT_ERR_IM_SHIFT)) & EMVSIM_INT_MASK_CWT_ERR_IM_MASK)
#define EMVSIM_INT_MASK_RNACK_IM_MASK            (0x400U)
#define EMVSIM_INT_MASK_RNACK_IM_SHIFT           (10U)
#define EMVSIM_INT_MASK_RNACK_IM(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_RNACK_IM_SHIFT)) & EMVSIM_INT_MASK_RNACK_IM_MASK)
#define EMVSIM_INT_MASK_BWT_ERR_IM_MASK          (0x800U)
#define EMVSIM_INT_MASK_BWT_ERR_IM_SHIFT         (11U)
#define EMVSIM_INT_MASK_BWT_ERR_IM(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_BWT_ERR_IM_SHIFT)) & EMVSIM_INT_MASK_BWT_ERR_IM_MASK)
#define EMVSIM_INT_MASK_BGT_ERR_IM_MASK          (0x1000U)
#define EMVSIM_INT_MASK_BGT_ERR_IM_SHIFT         (12U)
#define EMVSIM_INT_MASK_BGT_ERR_IM(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_BGT_ERR_IM_SHIFT)) & EMVSIM_INT_MASK_BGT_ERR_IM_MASK)
#define EMVSIM_INT_MASK_GPCNT1_IM_MASK           (0x2000U)
#define EMVSIM_INT_MASK_GPCNT1_IM_SHIFT          (13U)
#define EMVSIM_INT_MASK_GPCNT1_IM(x)             (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_GPCNT1_IM_SHIFT)) & EMVSIM_INT_MASK_GPCNT1_IM_MASK)
#define EMVSIM_INT_MASK_RX_DATA_IM_MASK          (0x4000U)
#define EMVSIM_INT_MASK_RX_DATA_IM_SHIFT         (14U)
#define EMVSIM_INT_MASK_RX_DATA_IM(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_RX_DATA_IM_SHIFT)) & EMVSIM_INT_MASK_RX_DATA_IM_MASK)
#define EMVSIM_INT_MASK_PEF_IM_MASK              (0x8000U)
#define EMVSIM_INT_MASK_PEF_IM_SHIFT             (15U)
#define EMVSIM_INT_MASK_PEF_IM(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_INT_MASK_PEF_IM_SHIFT)) & EMVSIM_INT_MASK_PEF_IM_MASK)

/*! @name RX_THD - Receiver Threshold Register */
#define EMVSIM_RX_THD_RDT_MASK                   (0xFU)
#define EMVSIM_RX_THD_RDT_SHIFT                  (0U)
#define EMVSIM_RX_THD_RDT(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_THD_RDT_SHIFT)) & EMVSIM_RX_THD_RDT_MASK)
#define EMVSIM_RX_THD_RNCK_THD_MASK              (0xF00U)
#define EMVSIM_RX_THD_RNCK_THD_SHIFT             (8U)
#define EMVSIM_RX_THD_RNCK_THD(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_THD_RNCK_THD_SHIFT)) & EMVSIM_RX_THD_RNCK_THD_MASK)

/*! @name TX_THD - Transmitter Threshold Register */
#define EMVSIM_TX_THD_TDT_MASK                   (0xFU)
#define EMVSIM_TX_THD_TDT_SHIFT                  (0U)
#define EMVSIM_TX_THD_TDT(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_THD_TDT_SHIFT)) & EMVSIM_TX_THD_TDT_MASK)
#define EMVSIM_TX_THD_TNCK_THD_MASK              (0xF00U)
#define EMVSIM_TX_THD_TNCK_THD_SHIFT             (8U)
#define EMVSIM_TX_THD_TNCK_THD(x)                (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_THD_TNCK_THD_SHIFT)) & EMVSIM_TX_THD_TNCK_THD_MASK)

/*! @name RX_STATUS - Receive Status Register */
#define EMVSIM_RX_STATUS_RFO_MASK                (0x1U)
#define EMVSIM_RX_STATUS_RFO_SHIFT               (0U)
#define EMVSIM_RX_STATUS_RFO(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RFO_SHIFT)) & EMVSIM_RX_STATUS_RFO_MASK)
#define EMVSIM_RX_STATUS_RX_DATA_MASK            (0x10U)
#define EMVSIM_RX_STATUS_RX_DATA_SHIFT           (4U)
#define EMVSIM_RX_STATUS_RX_DATA(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RX_DATA_SHIFT)) & EMVSIM_RX_STATUS_RX_DATA_MASK)
#define EMVSIM_RX_STATUS_RDTF_MASK               (0x20U)
#define EMVSIM_RX_STATUS_RDTF_SHIFT              (5U)
#define EMVSIM_RX_STATUS_RDTF(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RDTF_SHIFT)) & EMVSIM_RX_STATUS_RDTF_MASK)
#define EMVSIM_RX_STATUS_LRC_OK_MASK             (0x40U)
#define EMVSIM_RX_STATUS_LRC_OK_SHIFT            (6U)
#define EMVSIM_RX_STATUS_LRC_OK(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_LRC_OK_SHIFT)) & EMVSIM_RX_STATUS_LRC_OK_MASK)
#define EMVSIM_RX_STATUS_CRC_OK_MASK             (0x80U)
#define EMVSIM_RX_STATUS_CRC_OK_SHIFT            (7U)
#define EMVSIM_RX_STATUS_CRC_OK(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_CRC_OK_SHIFT)) & EMVSIM_RX_STATUS_CRC_OK_MASK)
#define EMVSIM_RX_STATUS_CWT_ERR_MASK            (0x100U)
#define EMVSIM_RX_STATUS_CWT_ERR_SHIFT           (8U)
#define EMVSIM_RX_STATUS_CWT_ERR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_CWT_ERR_SHIFT)) & EMVSIM_RX_STATUS_CWT_ERR_MASK)
#define EMVSIM_RX_STATUS_RTE_MASK                (0x200U)
#define EMVSIM_RX_STATUS_RTE_SHIFT               (9U)
#define EMVSIM_RX_STATUS_RTE(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RTE_SHIFT)) & EMVSIM_RX_STATUS_RTE_MASK)
#define EMVSIM_RX_STATUS_BWT_ERR_MASK            (0x400U)
#define EMVSIM_RX_STATUS_BWT_ERR_SHIFT           (10U)
#define EMVSIM_RX_STATUS_BWT_ERR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_BWT_ERR_SHIFT)) & EMVSIM_RX_STATUS_BWT_ERR_MASK)
#define EMVSIM_RX_STATUS_BGT_ERR_MASK            (0x800U)
#define EMVSIM_RX_STATUS_BGT_ERR_SHIFT           (11U)
#define EMVSIM_RX_STATUS_BGT_ERR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_BGT_ERR_SHIFT)) & EMVSIM_RX_STATUS_BGT_ERR_MASK)
#define EMVSIM_RX_STATUS_PEF_MASK                (0x1000U)
#define EMVSIM_RX_STATUS_PEF_SHIFT               (12U)
#define EMVSIM_RX_STATUS_PEF(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_PEF_SHIFT)) & EMVSIM_RX_STATUS_PEF_MASK)
#define EMVSIM_RX_STATUS_FEF_MASK                (0x2000U)
#define EMVSIM_RX_STATUS_FEF_SHIFT               (13U)
#define EMVSIM_RX_STATUS_FEF(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_FEF_SHIFT)) & EMVSIM_RX_STATUS_FEF_MASK)
#define EMVSIM_RX_STATUS_RX_WPTR_MASK            (0xF0000U)
#define EMVSIM_RX_STATUS_RX_WPTR_SHIFT           (16U)
#define EMVSIM_RX_STATUS_RX_WPTR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RX_WPTR_SHIFT)) & EMVSIM_RX_STATUS_RX_WPTR_MASK)
#define EMVSIM_RX_STATUS_RX_CNT_MASK             (0x1F000000U)
#define EMVSIM_RX_STATUS_RX_CNT_SHIFT            (24U)
#define EMVSIM_RX_STATUS_RX_CNT(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RX_CNT_SHIFT)) & EMVSIM_RX_STATUS_RX_CNT_MASK)

/*! @name TX_STATUS - Transmitter Status Register */
#define EMVSIM_TX_STATUS_TNTE_MASK               (0x1U)
#define EMVSIM_TX_STATUS_TNTE_SHIFT              (0U)
#define EMVSIM_TX_STATUS_TNTE(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TNTE_SHIFT)) & EMVSIM_TX_STATUS_TNTE_MASK)
#define EMVSIM_TX_STATUS_TFE_MASK                (0x8U)
#define EMVSIM_TX_STATUS_TFE_SHIFT               (3U)
#define EMVSIM_TX_STATUS_TFE(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TFE_SHIFT)) & EMVSIM_TX_STATUS_TFE_MASK)
#define EMVSIM_TX_STATUS_ETCF_MASK               (0x10U)
#define EMVSIM_TX_STATUS_ETCF_SHIFT              (4U)
#define EMVSIM_TX_STATUS_ETCF(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_ETCF_SHIFT)) & EMVSIM_TX_STATUS_ETCF_MASK)
#define EMVSIM_TX_STATUS_TCF_MASK                (0x20U)
#define EMVSIM_TX_STATUS_TCF_SHIFT               (5U)
#define EMVSIM_TX_STATUS_TCF(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TCF_SHIFT)) & EMVSIM_TX_STATUS_TCF_MASK)
#define EMVSIM_TX_STATUS_TFF_MASK                (0x40U)
#define EMVSIM_TX_STATUS_TFF_SHIFT               (6U)
#define EMVSIM_TX_STATUS_TFF(x)                  (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TFF_SHIFT)) & EMVSIM_TX_STATUS_TFF_MASK)
#define EMVSIM_TX_STATUS_TDTF_MASK               (0x80U)
#define EMVSIM_TX_STATUS_TDTF_SHIFT              (7U)
#define EMVSIM_TX_STATUS_TDTF(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TDTF_SHIFT)) & EMVSIM_TX_STATUS_TDTF_MASK)
#define EMVSIM_TX_STATUS_GPCNT0_TO_MASK          (0x100U)
#define EMVSIM_TX_STATUS_GPCNT0_TO_SHIFT         (8U)
#define EMVSIM_TX_STATUS_GPCNT0_TO(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_GPCNT0_TO_SHIFT)) & EMVSIM_TX_STATUS_GPCNT0_TO_MASK)
#define EMVSIM_TX_STATUS_GPCNT1_TO_MASK          (0x200U)
#define EMVSIM_TX_STATUS_GPCNT1_TO_SHIFT         (9U)
#define EMVSIM_TX_STATUS_GPCNT1_TO(x)            (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_GPCNT1_TO_SHIFT)) & EMVSIM_TX_STATUS_GPCNT1_TO_MASK)
#define EMVSIM_TX_STATUS_TX_RPTR_MASK            (0xF0000U)
#define EMVSIM_TX_STATUS_TX_RPTR_SHIFT           (16U)
#define EMVSIM_TX_STATUS_TX_RPTR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TX_RPTR_SHIFT)) & EMVSIM_TX_STATUS_TX_RPTR_MASK)
#define EMVSIM_TX_STATUS_TX_CNT_MASK             (0x1F000000U)
#define EMVSIM_TX_STATUS_TX_CNT_SHIFT            (24U)
#define EMVSIM_TX_STATUS_TX_CNT(x)               (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TX_CNT_SHIFT)) & EMVSIM_TX_STATUS_TX_CNT_MASK)

/*! @name PCSR - Port Control and Status Register */
#define EMVSIM_PCSR_SAPD_MASK                    (0x1U)
#define EMVSIM_PCSR_SAPD_SHIFT                   (0U)
#define EMVSIM_PCSR_SAPD(x)                      (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SAPD_SHIFT)) & EMVSIM_PCSR_SAPD_MASK)
#define EMVSIM_PCSR_SVCC_EN_MASK                 (0x2U)
#define EMVSIM_PCSR_SVCC_EN_SHIFT                (1U)
#define EMVSIM_PCSR_SVCC_EN(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SVCC_EN_SHIFT)) & EMVSIM_PCSR_SVCC_EN_MASK)
#define EMVSIM_PCSR_VCCENP_MASK                  (0x4U)
#define EMVSIM_PCSR_VCCENP_SHIFT                 (2U)
#define EMVSIM_PCSR_VCCENP(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_VCCENP_SHIFT)) & EMVSIM_PCSR_VCCENP_MASK)
#define EMVSIM_PCSR_SRST_MASK                    (0x8U)
#define EMVSIM_PCSR_SRST_SHIFT                   (3U)
#define EMVSIM_PCSR_SRST(x)                      (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SRST_SHIFT)) & EMVSIM_PCSR_SRST_MASK)
#define EMVSIM_PCSR_SCEN_MASK                    (0x10U)
#define EMVSIM_PCSR_SCEN_SHIFT                   (4U)
#define EMVSIM_PCSR_SCEN(x)                      (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SCEN_SHIFT)) & EMVSIM_PCSR_SCEN_MASK)
#define EMVSIM_PCSR_SCSP_MASK                    (0x20U)
#define EMVSIM_PCSR_SCSP_SHIFT                   (5U)
#define EMVSIM_PCSR_SCSP(x)                      (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SCSP_SHIFT)) & EMVSIM_PCSR_SCSP_MASK)
#define EMVSIM_PCSR_SPD_MASK                     (0x80U)
#define EMVSIM_PCSR_SPD_SHIFT                    (7U)
#define EMVSIM_PCSR_SPD(x)                       (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SPD_SHIFT)) & EMVSIM_PCSR_SPD_MASK)
#define EMVSIM_PCSR_SPDIM_MASK                   (0x1000000U)
#define EMVSIM_PCSR_SPDIM_SHIFT                  (24U)
#define EMVSIM_PCSR_SPDIM(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SPDIM_SHIFT)) & EMVSIM_PCSR_SPDIM_MASK)
#define EMVSIM_PCSR_SPDIF_MASK                   (0x2000000U)
#define EMVSIM_PCSR_SPDIF_SHIFT                  (25U)
#define EMVSIM_PCSR_SPDIF(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SPDIF_SHIFT)) & EMVSIM_PCSR_SPDIF_MASK)
#define EMVSIM_PCSR_SPDP_MASK                    (0x4000000U)
#define EMVSIM_PCSR_SPDP_SHIFT                   (26U)
#define EMVSIM_PCSR_SPDP(x)                      (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SPDP_SHIFT)) & EMVSIM_PCSR_SPDP_MASK)
#define EMVSIM_PCSR_SPDES_MASK                   (0x8000000U)
#define EMVSIM_PCSR_SPDES_SHIFT                  (27U)
#define EMVSIM_PCSR_SPDES(x)                     (((uint32_t)(((uint32_t)(x)) << EMVSIM_PCSR_SPDES_SHIFT)) & EMVSIM_PCSR_SPDES_MASK)

/*! @name RX_BUF - Receive Data Read Buffer */
#define EMVSIM_RX_BUF_RX_BYTE_MASK               (0xFFU)
#define EMVSIM_RX_BUF_RX_BYTE_SHIFT              (0U)
#define EMVSIM_RX_BUF_RX_BYTE(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_BUF_RX_BYTE_SHIFT)) & EMVSIM_RX_BUF_RX_BYTE_MASK)

/*! @name TX_BUF - Transmit Data Buffer */
#define EMVSIM_TX_BUF_TX_BYTE_MASK               (0xFFU)
#define EMVSIM_TX_BUF_TX_BYTE_SHIFT              (0U)
#define EMVSIM_TX_BUF_TX_BYTE(x)                 (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_BUF_TX_BYTE_SHIFT)) & EMVSIM_TX_BUF_TX_BYTE_MASK)

/*! @name TX_GETU - Transmitter Guard ETU Value Register */
#define EMVSIM_TX_GETU_GETU_MASK                 (0xFFU)
#define EMVSIM_TX_GETU_GETU_SHIFT                (0U)
#define EMVSIM_TX_GETU_GETU(x)                   (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_GETU_GETU_SHIFT)) & EMVSIM_TX_GETU_GETU_MASK)

/*! @name CWT_VAL - Character Wait Time Value Register */
#define EMVSIM_CWT_VAL_CWT_MASK                  (0xFFFFU)
#define EMVSIM_CWT_VAL_CWT_SHIFT                 (0U)
#define EMVSIM_CWT_VAL_CWT(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_CWT_VAL_CWT_SHIFT)) & EMVSIM_CWT_VAL_CWT_MASK)

/*! @name BWT_VAL - Block Wait Time Value Register */
#define EMVSIM_BWT_VAL_BWT_MASK                  (0xFFFFFFFFU)
#define EMVSIM_BWT_VAL_BWT_SHIFT                 (0U)
#define EMVSIM_BWT_VAL_BWT(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_BWT_VAL_BWT_SHIFT)) & EMVSIM_BWT_VAL_BWT_MASK)

/*! @name BGT_VAL - Block Guard Time Value Register */
#define EMVSIM_BGT_VAL_BGT_MASK                  (0xFFFFU)
#define EMVSIM_BGT_VAL_BGT_SHIFT                 (0U)
#define EMVSIM_BGT_VAL_BGT(x)                    (((uint32_t)(((uint32_t)(x)) << EMVSIM_BGT_VAL_BGT_SHIFT)) & EMVSIM_BGT_VAL_BGT_MASK)

/*! @name GPCNT0_VAL - General Purpose Counter 0 Timeout Value Register */
#define EMVSIM_GPCNT0_VAL_GPCNT0_MASK            (0xFFFFU)
#define EMVSIM_GPCNT0_VAL_GPCNT0_SHIFT           (0U)
#define EMVSIM_GPCNT0_VAL_GPCNT0(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_GPCNT0_VAL_GPCNT0_SHIFT)) & EMVSIM_GPCNT0_VAL_GPCNT0_MASK)

/*! @name GPCNT1_VAL - General Purpose Counter 1 Timeout Value */
#define EMVSIM_GPCNT1_VAL_GPCNT1_MASK            (0xFFFFU)
#define EMVSIM_GPCNT1_VAL_GPCNT1_SHIFT           (0U)
#define EMVSIM_GPCNT1_VAL_GPCNT1(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_GPCNT1_VAL_GPCNT1_SHIFT)) & EMVSIM_GPCNT1_VAL_GPCNT1_MASK)


/*!
 * @}
 */ /* end of group EMVSIM_Register_Masks */


/* EMVSIM - Peripheral instance base addresses */
/** Peripheral EMVSIM0 base address */
#define EMVSIM0_BASE                             (0x400D4000u)
/** Peripheral EMVSIM0 base pointer */
#define EMVSIM0                                  ((EMVSIM_Type *)EMVSIM0_BASE)
/** Peripheral EMVSIM1 base address */
#define EMVSIM1_BASE                             (0x400D5000u)
/** Peripheral EMVSIM1 base pointer */
#define EMVSIM1                                  ((EMVSIM_Type *)EMVSIM1_BASE)
/** Array initializer of EMVSIM peripheral base addresses */
#define EMVSIM_BASE_ADDRS                        { EMVSIM0_BASE, EMVSIM1_BASE }
/** Array initializer of EMVSIM peripheral base pointers */
#define EMVSIM_BASE_PTRS                         { EMVSIM0, EMVSIM1 }
/** Interrupt vectors for the EMVSIM peripheral type */
#define EMVSIM_IRQS                              { EMVSIM0_IRQn, EMVSIM1_IRQn }

/*!
 * @}
 */ /* end of group EMVSIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct {
  __IO uint8_t CTRL;                               /**< Control Register, offset: 0x0 */
  __O  uint8_t SERV;                               /**< Service Register, offset: 0x1 */
  __IO uint8_t CMPL;                               /**< Compare Low Register, offset: 0x2 */
  __IO uint8_t CMPH;                               /**< Compare High Register, offset: 0x3 */
  __IO uint8_t CLKCTRL;                            /**< Clock Control Register, offset: 0x4 */
  __IO uint8_t CLKPRESCALER;                       /**< Clock Prescaler Register, offset: 0x5 */
} EWM_Type;

/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
#define EWM_CTRL_EWMEN_MASK                      (0x1U)
#define EWM_CTRL_EWMEN_SHIFT                     (0U)
#define EWM_CTRL_EWMEN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_EWMEN_SHIFT)) & EWM_CTRL_EWMEN_MASK)
#define EWM_CTRL_ASSIN_MASK                      (0x2U)
#define EWM_CTRL_ASSIN_SHIFT                     (1U)
#define EWM_CTRL_ASSIN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_ASSIN_SHIFT)) & EWM_CTRL_ASSIN_MASK)
#define EWM_CTRL_INEN_MASK                       (0x4U)
#define EWM_CTRL_INEN_SHIFT                      (2U)
#define EWM_CTRL_INEN(x)                         (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_INEN_SHIFT)) & EWM_CTRL_INEN_MASK)
#define EWM_CTRL_INTEN_MASK                      (0x8U)
#define EWM_CTRL_INTEN_SHIFT                     (3U)
#define EWM_CTRL_INTEN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_INTEN_SHIFT)) & EWM_CTRL_INTEN_MASK)

/*! @name SERV - Service Register */
#define EWM_SERV_SERVICE_MASK                    (0xFFU)
#define EWM_SERV_SERVICE_SHIFT                   (0U)
#define EWM_SERV_SERVICE(x)                      (((uint8_t)(((uint8_t)(x)) << EWM_SERV_SERVICE_SHIFT)) & EWM_SERV_SERVICE_MASK)

/*! @name CMPL - Compare Low Register */
#define EWM_CMPL_COMPAREL_MASK                   (0xFFU)
#define EWM_CMPL_COMPAREL_SHIFT                  (0U)
#define EWM_CMPL_COMPAREL(x)                     (((uint8_t)(((uint8_t)(x)) << EWM_CMPL_COMPAREL_SHIFT)) & EWM_CMPL_COMPAREL_MASK)

/*! @name CMPH - Compare High Register */
#define EWM_CMPH_COMPAREH_MASK                   (0xFFU)
#define EWM_CMPH_COMPAREH_SHIFT                  (0U)
#define EWM_CMPH_COMPAREH(x)                     (((uint8_t)(((uint8_t)(x)) << EWM_CMPH_COMPAREH_SHIFT)) & EWM_CMPH_COMPAREH_MASK)

/*! @name CLKCTRL - Clock Control Register */
#define EWM_CLKCTRL_CLKSEL_MASK                  (0x3U)
#define EWM_CLKCTRL_CLKSEL_SHIFT                 (0U)
#define EWM_CLKCTRL_CLKSEL(x)                    (((uint8_t)(((uint8_t)(x)) << EWM_CLKCTRL_CLKSEL_SHIFT)) & EWM_CLKCTRL_CLKSEL_MASK)

/*! @name CLKPRESCALER - Clock Prescaler Register */
#define EWM_CLKPRESCALER_CLK_DIV_MASK            (0xFFU)
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT           (0U)
#define EWM_CLKPRESCALER_CLK_DIV(x)              (((uint8_t)(((uint8_t)(x)) << EWM_CLKPRESCALER_CLK_DIV_SHIFT)) & EWM_CLKPRESCALER_CLK_DIV_MASK)


/*!
 * @}
 */ /* end of group EWM_Register_Masks */


/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE                                 (0x40061000u)
/** Peripheral EWM base pointer */
#define EWM                                      ((EWM_Type *)EWM_BASE)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS                           { EWM_BASE }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM }
/** Interrupt vectors for the EWM peripheral type */
#define EWM_IRQS                                 { WDOG_EWM_IRQn }

/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Peripheral_Access_Layer FB Peripheral Access Layer
 * @{
 */

/** FB - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0xC */
    __IO uint32_t CSAR;                              /**< Chip Select Address Register, array offset: 0x0, array step: 0xC */
    __IO uint32_t CSMR;                              /**< Chip Select Mask Register, array offset: 0x4, array step: 0xC */
    __IO uint32_t CSCR;                              /**< Chip Select Control Register, array offset: 0x8, array step: 0xC */
  } CS[6];
       uint8_t RESERVED_0[24];
  __IO uint32_t CSPMCR;                            /**< Chip Select port Multiplexing Control Register, offset: 0x60 */
} FB_Type;

/* ----------------------------------------------------------------------------
   -- FB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Register_Masks FB Register Masks
 * @{
 */

/*! @name CSAR - Chip Select Address Register */
#define FB_CSAR_BA_MASK                          (0xFFFF0000U)
#define FB_CSAR_BA_SHIFT                         (16U)
#define FB_CSAR_BA(x)                            (((uint32_t)(((uint32_t)(x)) << FB_CSAR_BA_SHIFT)) & FB_CSAR_BA_MASK)

/* The count of FB_CSAR */
#define FB_CSAR_COUNT                            (6U)

/*! @name CSMR - Chip Select Mask Register */
#define FB_CSMR_V_MASK                           (0x1U)
#define FB_CSMR_V_SHIFT                          (0U)
#define FB_CSMR_V(x)                             (((uint32_t)(((uint32_t)(x)) << FB_CSMR_V_SHIFT)) & FB_CSMR_V_MASK)
#define FB_CSMR_WP_MASK                          (0x100U)
#define FB_CSMR_WP_SHIFT                         (8U)
#define FB_CSMR_WP(x)                            (((uint32_t)(((uint32_t)(x)) << FB_CSMR_WP_SHIFT)) & FB_CSMR_WP_MASK)
#define FB_CSMR_BAM_MASK                         (0xFFFF0000U)
#define FB_CSMR_BAM_SHIFT                        (16U)
#define FB_CSMR_BAM(x)                           (((uint32_t)(((uint32_t)(x)) << FB_CSMR_BAM_SHIFT)) & FB_CSMR_BAM_MASK)

/* The count of FB_CSMR */
#define FB_CSMR_COUNT                            (6U)

/*! @name CSCR - Chip Select Control Register */
#define FB_CSCR_BSTW_MASK                        (0x8U)
#define FB_CSCR_BSTW_SHIFT                       (3U)
#define FB_CSCR_BSTW(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_BSTW_SHIFT)) & FB_CSCR_BSTW_MASK)
#define FB_CSCR_BSTR_MASK                        (0x10U)
#define FB_CSCR_BSTR_SHIFT                       (4U)
#define FB_CSCR_BSTR(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_BSTR_SHIFT)) & FB_CSCR_BSTR_MASK)
#define FB_CSCR_BEM_MASK                         (0x20U)
#define FB_CSCR_BEM_SHIFT                        (5U)
#define FB_CSCR_BEM(x)                           (((uint32_t)(((uint32_t)(x)) << FB_CSCR_BEM_SHIFT)) & FB_CSCR_BEM_MASK)
#define FB_CSCR_PS_MASK                          (0xC0U)
#define FB_CSCR_PS_SHIFT                         (6U)
#define FB_CSCR_PS(x)                            (((uint32_t)(((uint32_t)(x)) << FB_CSCR_PS_SHIFT)) & FB_CSCR_PS_MASK)
#define FB_CSCR_AA_MASK                          (0x100U)
#define FB_CSCR_AA_SHIFT                         (8U)
#define FB_CSCR_AA(x)                            (((uint32_t)(((uint32_t)(x)) << FB_CSCR_AA_SHIFT)) & FB_CSCR_AA_MASK)
#define FB_CSCR_BLS_MASK                         (0x200U)
#define FB_CSCR_BLS_SHIFT                        (9U)
#define FB_CSCR_BLS(x)                           (((uint32_t)(((uint32_t)(x)) << FB_CSCR_BLS_SHIFT)) & FB_CSCR_BLS_MASK)
#define FB_CSCR_WS_MASK                          (0xFC00U)
#define FB_CSCR_WS_SHIFT                         (10U)
#define FB_CSCR_WS(x)                            (((uint32_t)(((uint32_t)(x)) << FB_CSCR_WS_SHIFT)) & FB_CSCR_WS_MASK)
#define FB_CSCR_WRAH_MASK                        (0x30000U)
#define FB_CSCR_WRAH_SHIFT                       (16U)
#define FB_CSCR_WRAH(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_WRAH_SHIFT)) & FB_CSCR_WRAH_MASK)
#define FB_CSCR_RDAH_MASK                        (0xC0000U)
#define FB_CSCR_RDAH_SHIFT                       (18U)
#define FB_CSCR_RDAH(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_RDAH_SHIFT)) & FB_CSCR_RDAH_MASK)
#define FB_CSCR_ASET_MASK                        (0x300000U)
#define FB_CSCR_ASET_SHIFT                       (20U)
#define FB_CSCR_ASET(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_ASET_SHIFT)) & FB_CSCR_ASET_MASK)
#define FB_CSCR_EXTS_MASK                        (0x400000U)
#define FB_CSCR_EXTS_SHIFT                       (22U)
#define FB_CSCR_EXTS(x)                          (((uint32_t)(((uint32_t)(x)) << FB_CSCR_EXTS_SHIFT)) & FB_CSCR_EXTS_MASK)
#define FB_CSCR_SWSEN_MASK                       (0x800000U)
#define FB_CSCR_SWSEN_SHIFT                      (23U)
#define FB_CSCR_SWSEN(x)                         (((uint32_t)(((uint32_t)(x)) << FB_CSCR_SWSEN_SHIFT)) & FB_CSCR_SWSEN_MASK)
#define FB_CSCR_SWS_MASK                         (0xFC000000U)
#define FB_CSCR_SWS_SHIFT                        (26U)
#define FB_CSCR_SWS(x)                           (((uint32_t)(((uint32_t)(x)) << FB_CSCR_SWS_SHIFT)) & FB_CSCR_SWS_MASK)

/* The count of FB_CSCR */
#define FB_CSCR_COUNT                            (6U)

/*! @name CSPMCR - Chip Select port Multiplexing Control Register */
#define FB_CSPMCR_GROUP5_MASK                    (0xF000U)
#define FB_CSPMCR_GROUP5_SHIFT                   (12U)
#define FB_CSPMCR_GROUP5(x)                      (((uint32_t)(((uint32_t)(x)) << FB_CSPMCR_GROUP5_SHIFT)) & FB_CSPMCR_GROUP5_MASK)
#define FB_CSPMCR_GROUP4_MASK                    (0xF0000U)
#define FB_CSPMCR_GROUP4_SHIFT                   (16U)
#define FB_CSPMCR_GROUP4(x)                      (((uint32_t)(((uint32_t)(x)) << FB_CSPMCR_GROUP4_SHIFT)) & FB_CSPMCR_GROUP4_MASK)
#define FB_CSPMCR_GROUP3_MASK                    (0xF00000U)
#define FB_CSPMCR_GROUP3_SHIFT                   (20U)
#define FB_CSPMCR_GROUP3(x)                      (((uint32_t)(((uint32_t)(x)) << FB_CSPMCR_GROUP3_SHIFT)) & FB_CSPMCR_GROUP3_MASK)
#define FB_CSPMCR_GROUP2_MASK                    (0xF000000U)
#define FB_CSPMCR_GROUP2_SHIFT                   (24U)
#define FB_CSPMCR_GROUP2(x)                      (((uint32_t)(((uint32_t)(x)) << FB_CSPMCR_GROUP2_SHIFT)) & FB_CSPMCR_GROUP2_MASK)
#define FB_CSPMCR_GROUP1_MASK                    (0xF0000000U)
#define FB_CSPMCR_GROUP1_SHIFT                   (28U)
#define FB_CSPMCR_GROUP1(x)                      (((uint32_t)(((uint32_t)(x)) << FB_CSPMCR_GROUP1_SHIFT)) & FB_CSPMCR_GROUP1_MASK)


/*!
 * @}
 */ /* end of group FB_Register_Masks */


/* FB - Peripheral instance base addresses */
/** Peripheral FB base address */
#define FB_BASE                                  (0x4000C000u)
/** Peripheral FB base pointer */
#define FB                                       ((FB_Type *)FB_BASE)
/** Array initializer of FB peripheral base addresses */
#define FB_BASE_ADDRS                            { FB_BASE }
/** Array initializer of FB peripheral base pointers */
#define FB_BASE_PTRS                             { FB }

/*!
 * @}
 */ /* end of group FB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FLEXIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXIO_Peripheral_Access_Layer FLEXIO Peripheral Access Layer
 * @{
 */

/** FLEXIO - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t CTRL;                              /**< FlexIO Control Register, offset: 0x8 */
  __I  uint32_t PIN;                               /**< Pin State Register, offset: 0xC */
  __IO uint32_t SHIFTSTAT;                         /**< Shifter Status Register, offset: 0x10 */
  __IO uint32_t SHIFTERR;                          /**< Shifter Error Register, offset: 0x14 */
  __IO uint32_t TIMSTAT;                           /**< Timer Status Register, offset: 0x18 */
       uint8_t RESERVED_0[4];
  __IO uint32_t SHIFTSIEN;                         /**< Shifter Status Interrupt Enable, offset: 0x20 */
  __IO uint32_t SHIFTEIEN;                         /**< Shifter Error Interrupt Enable, offset: 0x24 */
  __IO uint32_t TIMIEN;                            /**< Timer Interrupt Enable Register, offset: 0x28 */
       uint8_t RESERVED_1[4];
  __IO uint32_t SHIFTSDEN;                         /**< Shifter Status DMA Enable, offset: 0x30 */
       uint8_t RESERVED_2[12];
  __IO uint32_t SHIFTSTATE;                        /**< Shifter State Register, offset: 0x40 */
       uint8_t RESERVED_3[60];
  __IO uint32_t SHIFTCTL[8];                       /**< Shifter Control N Register, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_4[96];
  __IO uint32_t SHIFTCFG[8];                       /**< Shifter Configuration N Register, array offset: 0x100, array step: 0x4 */
       uint8_t RESERVED_5[224];
  __IO uint32_t SHIFTBUF[8];                       /**< Shifter Buffer N Register, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_6[96];
  __IO uint32_t SHIFTBUFBIS[8];                    /**< Shifter Buffer N Bit Swapped Register, array offset: 0x280, array step: 0x4 */
       uint8_t RESERVED_7[96];
  __IO uint32_t SHIFTBUFBYS[8];                    /**< Shifter Buffer N Byte Swapped Register, array offset: 0x300, array step: 0x4 */
       uint8_t RESERVED_8[96];
  __IO uint32_t SHIFTBUFBBS[8];                    /**< Shifter Buffer N Bit Byte Swapped Register, array offset: 0x380, array step: 0x4 */
       uint8_t RESERVED_9[96];
  __IO uint32_t TIMCTL[8];                         /**< Timer Control N Register, array offset: 0x400, array step: 0x4 */
       uint8_t RESERVED_10[96];
  __IO uint32_t TIMCFG[8];                         /**< Timer Configuration N Register, array offset: 0x480, array step: 0x4 */
       uint8_t RESERVED_11[96];
  __IO uint32_t TIMCMP[8];                         /**< Timer Compare N Register, array offset: 0x500, array step: 0x4 */
       uint8_t RESERVED_12[352];
  __IO uint32_t SHIFTBUFNBS[8];                    /**< Shifter Buffer N Nibble Byte Swapped Register, array offset: 0x680, array step: 0x4 */
       uint8_t RESERVED_13[96];
  __IO uint32_t SHIFTBUFHWS[8];                    /**< Shifter Buffer N Half Word Swapped Register, array offset: 0x700, array step: 0x4 */
       uint8_t RESERVED_14[96];
  __IO uint32_t SHIFTBUFNIS[8];                    /**< Shifter Buffer N Nibble Swapped Register, array offset: 0x780, array step: 0x4 */
} FLEXIO_Type;

/* ----------------------------------------------------------------------------
   -- FLEXIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXIO_Register_Masks FLEXIO Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define FLEXIO_VERID_FEATURE_MASK                (0xFFFFU)
#define FLEXIO_VERID_FEATURE_SHIFT               (0U)
#define FLEXIO_VERID_FEATURE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_FEATURE_SHIFT)) & FLEXIO_VERID_FEATURE_MASK)
#define FLEXIO_VERID_MINOR_MASK                  (0xFF0000U)
#define FLEXIO_VERID_MINOR_SHIFT                 (16U)
#define FLEXIO_VERID_MINOR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_MINOR_SHIFT)) & FLEXIO_VERID_MINOR_MASK)
#define FLEXIO_VERID_MAJOR_MASK                  (0xFF000000U)
#define FLEXIO_VERID_MAJOR_SHIFT                 (24U)
#define FLEXIO_VERID_MAJOR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_MAJOR_SHIFT)) & FLEXIO_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define FLEXIO_PARAM_SHIFTER_MASK                (0xFFU)
#define FLEXIO_PARAM_SHIFTER_SHIFT               (0U)
#define FLEXIO_PARAM_SHIFTER(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_SHIFTER_SHIFT)) & FLEXIO_PARAM_SHIFTER_MASK)
#define FLEXIO_PARAM_TIMER_MASK                  (0xFF00U)
#define FLEXIO_PARAM_TIMER_SHIFT                 (8U)
#define FLEXIO_PARAM_TIMER(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_TIMER_SHIFT)) & FLEXIO_PARAM_TIMER_MASK)
#define FLEXIO_PARAM_PIN_MASK                    (0xFF0000U)
#define FLEXIO_PARAM_PIN_SHIFT                   (16U)
#define FLEXIO_PARAM_PIN(x)                      (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_PIN_SHIFT)) & FLEXIO_PARAM_PIN_MASK)
#define FLEXIO_PARAM_TRIGGER_MASK                (0xFF000000U)
#define FLEXIO_PARAM_TRIGGER_SHIFT               (24U)
#define FLEXIO_PARAM_TRIGGER(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_TRIGGER_SHIFT)) & FLEXIO_PARAM_TRIGGER_MASK)

/*! @name CTRL - FlexIO Control Register */
#define FLEXIO_CTRL_FLEXEN_MASK                  (0x1U)
#define FLEXIO_CTRL_FLEXEN_SHIFT                 (0U)
#define FLEXIO_CTRL_FLEXEN(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_FLEXEN_SHIFT)) & FLEXIO_CTRL_FLEXEN_MASK)
#define FLEXIO_CTRL_SWRST_MASK                   (0x2U)
#define FLEXIO_CTRL_SWRST_SHIFT                  (1U)
#define FLEXIO_CTRL_SWRST(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_SWRST_SHIFT)) & FLEXIO_CTRL_SWRST_MASK)
#define FLEXIO_CTRL_FASTACC_MASK                 (0x4U)
#define FLEXIO_CTRL_FASTACC_SHIFT                (2U)
#define FLEXIO_CTRL_FASTACC(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_FASTACC_SHIFT)) & FLEXIO_CTRL_FASTACC_MASK)
#define FLEXIO_CTRL_DBGE_MASK                    (0x40000000U)
#define FLEXIO_CTRL_DBGE_SHIFT                   (30U)
#define FLEXIO_CTRL_DBGE(x)                      (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_DBGE_SHIFT)) & FLEXIO_CTRL_DBGE_MASK)
#define FLEXIO_CTRL_DOZEN_MASK                   (0x80000000U)
#define FLEXIO_CTRL_DOZEN_SHIFT                  (31U)
#define FLEXIO_CTRL_DOZEN(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_DOZEN_SHIFT)) & FLEXIO_CTRL_DOZEN_MASK)

/*! @name PIN - Pin State Register */
#define FLEXIO_PIN_PDI_MASK                      (0xFFFFFFFFU)
#define FLEXIO_PIN_PDI_SHIFT                     (0U)
#define FLEXIO_PIN_PDI(x)                        (((uint32_t)(((uint32_t)(x)) << FLEXIO_PIN_PDI_SHIFT)) & FLEXIO_PIN_PDI_MASK)

/*! @name SHIFTSTAT - Shifter Status Register */
#define FLEXIO_SHIFTSTAT_SSF_MASK                (0xFFU)
#define FLEXIO_SHIFTSTAT_SSF_SHIFT               (0U)
#define FLEXIO_SHIFTSTAT_SSF(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSTAT_SSF_SHIFT)) & FLEXIO_SHIFTSTAT_SSF_MASK)

/*! @name SHIFTERR - Shifter Error Register */
#define FLEXIO_SHIFTERR_SEF_MASK                 (0xFFU)
#define FLEXIO_SHIFTERR_SEF_SHIFT                (0U)
#define FLEXIO_SHIFTERR_SEF(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTERR_SEF_SHIFT)) & FLEXIO_SHIFTERR_SEF_MASK)

/*! @name TIMSTAT - Timer Status Register */
#define FLEXIO_TIMSTAT_TSF_MASK                  (0xFFU)
#define FLEXIO_TIMSTAT_TSF_SHIFT                 (0U)
#define FLEXIO_TIMSTAT_TSF(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMSTAT_TSF_SHIFT)) & FLEXIO_TIMSTAT_TSF_MASK)

/*! @name SHIFTSIEN - Shifter Status Interrupt Enable */
#define FLEXIO_SHIFTSIEN_SSIE_MASK               (0xFFU)
#define FLEXIO_SHIFTSIEN_SSIE_SHIFT              (0U)
#define FLEXIO_SHIFTSIEN_SSIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSIEN_SSIE_SHIFT)) & FLEXIO_SHIFTSIEN_SSIE_MASK)

/*! @name SHIFTEIEN - Shifter Error Interrupt Enable */
#define FLEXIO_SHIFTEIEN_SEIE_MASK               (0xFFU)
#define FLEXIO_SHIFTEIEN_SEIE_SHIFT              (0U)
#define FLEXIO_SHIFTEIEN_SEIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTEIEN_SEIE_SHIFT)) & FLEXIO_SHIFTEIEN_SEIE_MASK)

/*! @name TIMIEN - Timer Interrupt Enable Register */
#define FLEXIO_TIMIEN_TEIE_MASK                  (0xFFU)
#define FLEXIO_TIMIEN_TEIE_SHIFT                 (0U)
#define FLEXIO_TIMIEN_TEIE(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMIEN_TEIE_SHIFT)) & FLEXIO_TIMIEN_TEIE_MASK)

/*! @name SHIFTSDEN - Shifter Status DMA Enable */
#define FLEXIO_SHIFTSDEN_SSDE_MASK               (0xFFU)
#define FLEXIO_SHIFTSDEN_SSDE_SHIFT              (0U)
#define FLEXIO_SHIFTSDEN_SSDE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSDEN_SSDE_SHIFT)) & FLEXIO_SHIFTSDEN_SSDE_MASK)

/*! @name SHIFTSTATE - Shifter State Register */
#define FLEXIO_SHIFTSTATE_STATE_MASK             (0x7U)
#define FLEXIO_SHIFTSTATE_STATE_SHIFT            (0U)
#define FLEXIO_SHIFTSTATE_STATE(x)               (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSTATE_STATE_SHIFT)) & FLEXIO_SHIFTSTATE_STATE_MASK)

/*! @name SHIFTCTL - Shifter Control N Register */
#define FLEXIO_SHIFTCTL_SMOD_MASK                (0x7U)
#define FLEXIO_SHIFTCTL_SMOD_SHIFT               (0U)
#define FLEXIO_SHIFTCTL_SMOD(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_SMOD_SHIFT)) & FLEXIO_SHIFTCTL_SMOD_MASK)
#define FLEXIO_SHIFTCTL_PINPOL_MASK              (0x80U)
#define FLEXIO_SHIFTCTL_PINPOL_SHIFT             (7U)
#define FLEXIO_SHIFTCTL_PINPOL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINPOL_SHIFT)) & FLEXIO_SHIFTCTL_PINPOL_MASK)
#define FLEXIO_SHIFTCTL_PINSEL_MASK              (0x1F00U)
#define FLEXIO_SHIFTCTL_PINSEL_SHIFT             (8U)
#define FLEXIO_SHIFTCTL_PINSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINSEL_SHIFT)) & FLEXIO_SHIFTCTL_PINSEL_MASK)
#define FLEXIO_SHIFTCTL_PINCFG_MASK              (0x30000U)
#define FLEXIO_SHIFTCTL_PINCFG_SHIFT             (16U)
#define FLEXIO_SHIFTCTL_PINCFG(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINCFG_SHIFT)) & FLEXIO_SHIFTCTL_PINCFG_MASK)
#define FLEXIO_SHIFTCTL_TIMPOL_MASK              (0x800000U)
#define FLEXIO_SHIFTCTL_TIMPOL_SHIFT             (23U)
#define FLEXIO_SHIFTCTL_TIMPOL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMPOL_SHIFT)) & FLEXIO_SHIFTCTL_TIMPOL_MASK)
#define FLEXIO_SHIFTCTL_TIMSEL_MASK              (0x7000000U)
#define FLEXIO_SHIFTCTL_TIMSEL_SHIFT             (24U)
#define FLEXIO_SHIFTCTL_TIMSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMSEL_SHIFT)) & FLEXIO_SHIFTCTL_TIMSEL_MASK)

/* The count of FLEXIO_SHIFTCTL */
#define FLEXIO_SHIFTCTL_COUNT                    (8U)

/*! @name SHIFTCFG - Shifter Configuration N Register */
#define FLEXIO_SHIFTCFG_SSTART_MASK              (0x3U)
#define FLEXIO_SHIFTCFG_SSTART_SHIFT             (0U)
#define FLEXIO_SHIFTCFG_SSTART(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTART_SHIFT)) & FLEXIO_SHIFTCFG_SSTART_MASK)
#define FLEXIO_SHIFTCFG_SSTOP_MASK               (0x30U)
#define FLEXIO_SHIFTCFG_SSTOP_SHIFT              (4U)
#define FLEXIO_SHIFTCFG_SSTOP(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTOP_SHIFT)) & FLEXIO_SHIFTCFG_SSTOP_MASK)
#define FLEXIO_SHIFTCFG_INSRC_MASK               (0x100U)
#define FLEXIO_SHIFTCFG_INSRC_SHIFT              (8U)
#define FLEXIO_SHIFTCFG_INSRC(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_INSRC_SHIFT)) & FLEXIO_SHIFTCFG_INSRC_MASK)
#define FLEXIO_SHIFTCFG_PWIDTH_MASK              (0x1F0000U)
#define FLEXIO_SHIFTCFG_PWIDTH_SHIFT             (16U)
#define FLEXIO_SHIFTCFG_PWIDTH(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_PWIDTH_SHIFT)) & FLEXIO_SHIFTCFG_PWIDTH_MASK)

/* The count of FLEXIO_SHIFTCFG */
#define FLEXIO_SHIFTCFG_COUNT                    (8U)

/*! @name SHIFTBUF - Shifter Buffer N Register */
#define FLEXIO_SHIFTBUF_SHIFTBUF_MASK            (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT           (0U)
#define FLEXIO_SHIFTBUF_SHIFTBUF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT)) & FLEXIO_SHIFTBUF_SHIFTBUF_MASK)

/* The count of FLEXIO_SHIFTBUF */
#define FLEXIO_SHIFTBUF_COUNT                    (8U)

/*! @name SHIFTBUFBIS - Shifter Buffer N Bit Swapped Register */
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT)) & FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK)

/* The count of FLEXIO_SHIFTBUFBIS */
#define FLEXIO_SHIFTBUFBIS_COUNT                 (8U)

/*! @name SHIFTBUFBYS - Shifter Buffer N Byte Swapped Register */
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT)) & FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK)

/* The count of FLEXIO_SHIFTBUFBYS */
#define FLEXIO_SHIFTBUFBYS_COUNT                 (8U)

/*! @name SHIFTBUFBBS - Shifter Buffer N Bit Byte Swapped Register */
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT)) & FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK)

/* The count of FLEXIO_SHIFTBUFBBS */
#define FLEXIO_SHIFTBUFBBS_COUNT                 (8U)

/*! @name TIMCTL - Timer Control N Register */
#define FLEXIO_TIMCTL_TIMOD_MASK                 (0x3U)
#define FLEXIO_TIMCTL_TIMOD_SHIFT                (0U)
#define FLEXIO_TIMCTL_TIMOD(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TIMOD_SHIFT)) & FLEXIO_TIMCTL_TIMOD_MASK)
#define FLEXIO_TIMCTL_PINPOL_MASK                (0x80U)
#define FLEXIO_TIMCTL_PINPOL_SHIFT               (7U)
#define FLEXIO_TIMCTL_PINPOL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINPOL_SHIFT)) & FLEXIO_TIMCTL_PINPOL_MASK)
#define FLEXIO_TIMCTL_PINSEL_MASK                (0x1F00U)
#define FLEXIO_TIMCTL_PINSEL_SHIFT               (8U)
#define FLEXIO_TIMCTL_PINSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINSEL_SHIFT)) & FLEXIO_TIMCTL_PINSEL_MASK)
#define FLEXIO_TIMCTL_PINCFG_MASK                (0x30000U)
#define FLEXIO_TIMCTL_PINCFG_SHIFT               (16U)
#define FLEXIO_TIMCTL_PINCFG(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINCFG_SHIFT)) & FLEXIO_TIMCTL_PINCFG_MASK)
#define FLEXIO_TIMCTL_TRGSRC_MASK                (0x400000U)
#define FLEXIO_TIMCTL_TRGSRC_SHIFT               (22U)
#define FLEXIO_TIMCTL_TRGSRC(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSRC_SHIFT)) & FLEXIO_TIMCTL_TRGSRC_MASK)
#define FLEXIO_TIMCTL_TRGPOL_MASK                (0x800000U)
#define FLEXIO_TIMCTL_TRGPOL_SHIFT               (23U)
#define FLEXIO_TIMCTL_TRGPOL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGPOL_SHIFT)) & FLEXIO_TIMCTL_TRGPOL_MASK)
#define FLEXIO_TIMCTL_TRGSEL_MASK                (0x3F000000U)
#define FLEXIO_TIMCTL_TRGSEL_SHIFT               (24U)
#define FLEXIO_TIMCTL_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSEL_SHIFT)) & FLEXIO_TIMCTL_TRGSEL_MASK)

/* The count of FLEXIO_TIMCTL */
#define FLEXIO_TIMCTL_COUNT                      (8U)

/*! @name TIMCFG - Timer Configuration N Register */
#define FLEXIO_TIMCFG_TSTART_MASK                (0x2U)
#define FLEXIO_TIMCFG_TSTART_SHIFT               (1U)
#define FLEXIO_TIMCFG_TSTART(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TSTART_SHIFT)) & FLEXIO_TIMCFG_TSTART_MASK)
#define FLEXIO_TIMCFG_TSTOP_MASK                 (0x30U)
#define FLEXIO_TIMCFG_TSTOP_SHIFT                (4U)
#define FLEXIO_TIMCFG_TSTOP(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TSTOP_SHIFT)) & FLEXIO_TIMCFG_TSTOP_MASK)
#define FLEXIO_TIMCFG_TIMENA_MASK                (0x700U)
#define FLEXIO_TIMCFG_TIMENA_SHIFT               (8U)
#define FLEXIO_TIMCFG_TIMENA(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMENA_SHIFT)) & FLEXIO_TIMCFG_TIMENA_MASK)
#define FLEXIO_TIMCFG_TIMDIS_MASK                (0x7000U)
#define FLEXIO_TIMCFG_TIMDIS_SHIFT               (12U)
#define FLEXIO_TIMCFG_TIMDIS(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDIS_SHIFT)) & FLEXIO_TIMCFG_TIMDIS_MASK)
#define FLEXIO_TIMCFG_TIMRST_MASK                (0x70000U)
#define FLEXIO_TIMCFG_TIMRST_SHIFT               (16U)
#define FLEXIO_TIMCFG_TIMRST(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMRST_SHIFT)) & FLEXIO_TIMCFG_TIMRST_MASK)
#define FLEXIO_TIMCFG_TIMDEC_MASK                (0x300000U)
#define FLEXIO_TIMCFG_TIMDEC_SHIFT               (20U)
#define FLEXIO_TIMCFG_TIMDEC(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDEC_SHIFT)) & FLEXIO_TIMCFG_TIMDEC_MASK)
#define FLEXIO_TIMCFG_TIMOUT_MASK                (0x3000000U)
#define FLEXIO_TIMCFG_TIMOUT_SHIFT               (24U)
#define FLEXIO_TIMCFG_TIMOUT(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMOUT_SHIFT)) & FLEXIO_TIMCFG_TIMOUT_MASK)

/* The count of FLEXIO_TIMCFG */
#define FLEXIO_TIMCFG_COUNT                      (8U)

/*! @name TIMCMP - Timer Compare N Register */
#define FLEXIO_TIMCMP_CMP_MASK                   (0xFFFFU)
#define FLEXIO_TIMCMP_CMP_SHIFT                  (0U)
#define FLEXIO_TIMCMP_CMP(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCMP_CMP_SHIFT)) & FLEXIO_TIMCMP_CMP_MASK)

/* The count of FLEXIO_TIMCMP */
#define FLEXIO_TIMCMP_COUNT                      (8U)

/*! @name SHIFTBUFNBS - Shifter Buffer N Nibble Byte Swapped Register */
#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_SHIFT)) & FLEXIO_SHIFTBUFNBS_SHIFTBUFNBS_MASK)

/* The count of FLEXIO_SHIFTBUFNBS */
#define FLEXIO_SHIFTBUFNBS_COUNT                 (8U)

/*! @name SHIFTBUFHWS - Shifter Buffer N Half Word Swapped Register */
#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_SHIFT)) & FLEXIO_SHIFTBUFHWS_SHIFTBUFHWS_MASK)

/* The count of FLEXIO_SHIFTBUFHWS */
#define FLEXIO_SHIFTBUFHWS_COUNT                 (8U)

/*! @name SHIFTBUFNIS - Shifter Buffer N Nibble Swapped Register */
#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_SHIFT)) & FLEXIO_SHIFTBUFNIS_SHIFTBUFNIS_MASK)

/* The count of FLEXIO_SHIFTBUFNIS */
#define FLEXIO_SHIFTBUFNIS_COUNT                 (8U)


/*!
 * @}
 */ /* end of group FLEXIO_Register_Masks */


/* FLEXIO - Peripheral instance base addresses */
/** Peripheral FLEXIO0 base address */
#define FLEXIO0_BASE                             (0x400DF000u)
/** Peripheral FLEXIO0 base pointer */
#define FLEXIO0                                  ((FLEXIO_Type *)FLEXIO0_BASE)
/** Array initializer of FLEXIO peripheral base addresses */
#define FLEXIO_BASE_ADDRS                        { FLEXIO0_BASE }
/** Array initializer of FLEXIO peripheral base pointers */
#define FLEXIO_BASE_PTRS                         { FLEXIO0 }
/** Interrupt vectors for the FLEXIO peripheral type */
#define FLEXIO_IRQS                              { FLEXIO0_IRQn }

/*!
 * @}
 */ /* end of group FLEXIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Peripheral_Access_Layer FMC Peripheral Access Layer
 * @{
 */

/** FMC - Register Layout Typedef */
typedef struct {
  __IO uint32_t PFAPR;                             /**< Flash Access Protection Register, offset: 0x0 */
  __IO uint32_t PFB0CR;                            /**< Flash Bank 0 Control Register, offset: 0x4 */
  __I  uint32_t RESERVED;                          /**< Reserved, offset: 0x8 */
       uint8_t RESERVED_0[244];
  __IO uint32_t TAGVDW0S[4];                       /**< Cache Tag Storage, array offset: 0x100, array step: 0x4 */
  __IO uint32_t TAGVDW1S[4];                       /**< Cache Tag Storage, array offset: 0x110, array step: 0x4 */
  __IO uint32_t TAGVDW2S[4];                       /**< Cache Tag Storage, array offset: 0x120, array step: 0x4 */
  __IO uint32_t TAGVDW3S[4];                       /**< Cache Tag Storage, array offset: 0x130, array step: 0x4 */
       uint8_t RESERVED_1[192];
  struct {                                         /* offset: 0x200, array step: index*0x40, index2*0x10 */
    __IO uint32_t DATA_UM;                           /**< Cache Data Storage (uppermost word), array offset: 0x200, array step: index*0x40, index2*0x10 */
    __IO uint32_t DATA_MU;                           /**< Cache Data Storage (mid-upper word), array offset: 0x204, array step: index*0x40, index2*0x10 */
    __IO uint32_t DATA_ML;                           /**< Cache Data Storage (mid-lower word), array offset: 0x208, array step: index*0x40, index2*0x10 */
    __IO uint32_t DATA_LM;                           /**< Cache Data Storage (lowermost word), array offset: 0x20C, array step: index*0x40, index2*0x10 */
  } SET[4][4];
} FMC_Type;

/* ----------------------------------------------------------------------------
   -- FMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Masks FMC Register Masks
 * @{
 */

/*! @name PFAPR - Flash Access Protection Register */
#define FMC_PFAPR_M0AP_MASK                      (0x3U)
#define FMC_PFAPR_M0AP_SHIFT                     (0U)
#define FMC_PFAPR_M0AP(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M0AP_SHIFT)) & FMC_PFAPR_M0AP_MASK)
#define FMC_PFAPR_M1AP_MASK                      (0xCU)
#define FMC_PFAPR_M1AP_SHIFT                     (2U)
#define FMC_PFAPR_M1AP(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M1AP_SHIFT)) & FMC_PFAPR_M1AP_MASK)
#define FMC_PFAPR_M2AP_MASK                      (0x30U)
#define FMC_PFAPR_M2AP_SHIFT                     (4U)
#define FMC_PFAPR_M2AP(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M2AP_SHIFT)) & FMC_PFAPR_M2AP_MASK)
#define FMC_PFAPR_M3AP_MASK                      (0xC0U)
#define FMC_PFAPR_M3AP_SHIFT                     (6U)
#define FMC_PFAPR_M3AP(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M3AP_SHIFT)) & FMC_PFAPR_M3AP_MASK)
#define FMC_PFAPR_M4AP_MASK                      (0x300U)
#define FMC_PFAPR_M4AP_SHIFT                     (8U)
#define FMC_PFAPR_M4AP(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M4AP_SHIFT)) & FMC_PFAPR_M4AP_MASK)
#define FMC_PFAPR_M0PFD_MASK                     (0x10000U)
#define FMC_PFAPR_M0PFD_SHIFT                    (16U)
#define FMC_PFAPR_M0PFD(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M0PFD_SHIFT)) & FMC_PFAPR_M0PFD_MASK)
#define FMC_PFAPR_M1PFD_MASK                     (0x20000U)
#define FMC_PFAPR_M1PFD_SHIFT                    (17U)
#define FMC_PFAPR_M1PFD(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M1PFD_SHIFT)) & FMC_PFAPR_M1PFD_MASK)
#define FMC_PFAPR_M2PFD_MASK                     (0x40000U)
#define FMC_PFAPR_M2PFD_SHIFT                    (18U)
#define FMC_PFAPR_M2PFD(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M2PFD_SHIFT)) & FMC_PFAPR_M2PFD_MASK)
#define FMC_PFAPR_M3PFD_MASK                     (0x80000U)
#define FMC_PFAPR_M3PFD_SHIFT                    (19U)
#define FMC_PFAPR_M3PFD(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M3PFD_SHIFT)) & FMC_PFAPR_M3PFD_MASK)
#define FMC_PFAPR_M4PFD_MASK                     (0x100000U)
#define FMC_PFAPR_M4PFD_SHIFT                    (20U)
#define FMC_PFAPR_M4PFD(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFAPR_M4PFD_SHIFT)) & FMC_PFAPR_M4PFD_MASK)

/*! @name PFB0CR - Flash Bank 0 Control Register */
#define FMC_PFB0CR_B0SEBE_MASK                   (0x1U)
#define FMC_PFB0CR_B0SEBE_SHIFT                  (0U)
#define FMC_PFB0CR_B0SEBE(x)                     (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0SEBE_SHIFT)) & FMC_PFB0CR_B0SEBE_MASK)
#define FMC_PFB0CR_B0IPE_MASK                    (0x2U)
#define FMC_PFB0CR_B0IPE_SHIFT                   (1U)
#define FMC_PFB0CR_B0IPE(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0IPE_SHIFT)) & FMC_PFB0CR_B0IPE_MASK)
#define FMC_PFB0CR_B0DPE_MASK                    (0x4U)
#define FMC_PFB0CR_B0DPE_SHIFT                   (2U)
#define FMC_PFB0CR_B0DPE(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0DPE_SHIFT)) & FMC_PFB0CR_B0DPE_MASK)
#define FMC_PFB0CR_B0ICE_MASK                    (0x8U)
#define FMC_PFB0CR_B0ICE_SHIFT                   (3U)
#define FMC_PFB0CR_B0ICE(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0ICE_SHIFT)) & FMC_PFB0CR_B0ICE_MASK)
#define FMC_PFB0CR_B0DCE_MASK                    (0x10U)
#define FMC_PFB0CR_B0DCE_SHIFT                   (4U)
#define FMC_PFB0CR_B0DCE(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0DCE_SHIFT)) & FMC_PFB0CR_B0DCE_MASK)
#define FMC_PFB0CR_CRC_MASK                      (0xE0U)
#define FMC_PFB0CR_CRC_SHIFT                     (5U)
#define FMC_PFB0CR_CRC(x)                        (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_CRC_SHIFT)) & FMC_PFB0CR_CRC_MASK)
#define FMC_PFB0CR_B0MW_MASK                     (0x60000U)
#define FMC_PFB0CR_B0MW_SHIFT                    (17U)
#define FMC_PFB0CR_B0MW(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0MW_SHIFT)) & FMC_PFB0CR_B0MW_MASK)
#define FMC_PFB0CR_S_B_INV_MASK                  (0x80000U)
#define FMC_PFB0CR_S_B_INV_SHIFT                 (19U)
#define FMC_PFB0CR_S_B_INV(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_S_B_INV_SHIFT)) & FMC_PFB0CR_S_B_INV_MASK)
#define FMC_PFB0CR_CINV_WAY_MASK                 (0xF00000U)
#define FMC_PFB0CR_CINV_WAY_SHIFT                (20U)
#define FMC_PFB0CR_CINV_WAY(x)                   (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_CINV_WAY_SHIFT)) & FMC_PFB0CR_CINV_WAY_MASK)
#define FMC_PFB0CR_CLCK_WAY_MASK                 (0xF000000U)
#define FMC_PFB0CR_CLCK_WAY_SHIFT                (24U)
#define FMC_PFB0CR_CLCK_WAY(x)                   (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_CLCK_WAY_SHIFT)) & FMC_PFB0CR_CLCK_WAY_MASK)
#define FMC_PFB0CR_B0RWSC_MASK                   (0xF0000000U)
#define FMC_PFB0CR_B0RWSC_SHIFT                  (28U)
#define FMC_PFB0CR_B0RWSC(x)                     (((uint32_t)(((uint32_t)(x)) << FMC_PFB0CR_B0RWSC_SHIFT)) & FMC_PFB0CR_B0RWSC_MASK)

/*! @name TAGVDW0S - Cache Tag Storage */
#define FMC_TAGVDW0S_valid_MASK                  (0x1U)
#define FMC_TAGVDW0S_valid_SHIFT                 (0U)
#define FMC_TAGVDW0S_valid(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW0S_valid_SHIFT)) & FMC_TAGVDW0S_valid_MASK)
#define FMC_TAGVDW0S_cache_tag_MASK              (0xFFFC0U)
#define FMC_TAGVDW0S_cache_tag_SHIFT             (6U)
#define FMC_TAGVDW0S_cache_tag(x)                (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW0S_cache_tag_SHIFT)) & FMC_TAGVDW0S_cache_tag_MASK)

/* The count of FMC_TAGVDW0S */
#define FMC_TAGVDW0S_COUNT                       (4U)

/*! @name TAGVDW1S - Cache Tag Storage */
#define FMC_TAGVDW1S_valid_MASK                  (0x1U)
#define FMC_TAGVDW1S_valid_SHIFT                 (0U)
#define FMC_TAGVDW1S_valid(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW1S_valid_SHIFT)) & FMC_TAGVDW1S_valid_MASK)
#define FMC_TAGVDW1S_cache_tag_MASK              (0xFFFC0U)
#define FMC_TAGVDW1S_cache_tag_SHIFT             (6U)
#define FMC_TAGVDW1S_cache_tag(x)                (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW1S_cache_tag_SHIFT)) & FMC_TAGVDW1S_cache_tag_MASK)

/* The count of FMC_TAGVDW1S */
#define FMC_TAGVDW1S_COUNT                       (4U)

/*! @name TAGVDW2S - Cache Tag Storage */
#define FMC_TAGVDW2S_valid_MASK                  (0x1U)
#define FMC_TAGVDW2S_valid_SHIFT                 (0U)
#define FMC_TAGVDW2S_valid(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW2S_valid_SHIFT)) & FMC_TAGVDW2S_valid_MASK)
#define FMC_TAGVDW2S_cache_tag_MASK              (0xFFFC0U)
#define FMC_TAGVDW2S_cache_tag_SHIFT             (6U)
#define FMC_TAGVDW2S_cache_tag(x)                (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW2S_cache_tag_SHIFT)) & FMC_TAGVDW2S_cache_tag_MASK)

/* The count of FMC_TAGVDW2S */
#define FMC_TAGVDW2S_COUNT                       (4U)

/*! @name TAGVDW3S - Cache Tag Storage */
#define FMC_TAGVDW3S_valid_MASK                  (0x1U)
#define FMC_TAGVDW3S_valid_SHIFT                 (0U)
#define FMC_TAGVDW3S_valid(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW3S_valid_SHIFT)) & FMC_TAGVDW3S_valid_MASK)
#define FMC_TAGVDW3S_cache_tag_MASK              (0xFFFC0U)
#define FMC_TAGVDW3S_cache_tag_SHIFT             (6U)
#define FMC_TAGVDW3S_cache_tag(x)                (((uint32_t)(((uint32_t)(x)) << FMC_TAGVDW3S_cache_tag_SHIFT)) & FMC_TAGVDW3S_cache_tag_MASK)

/* The count of FMC_TAGVDW3S */
#define FMC_TAGVDW3S_COUNT                       (4U)

/*! @name DATA_UM - Cache Data Storage (uppermost word) */
#define FMC_DATA_UM_data_MASK                    (0xFFFFFFFFU)
#define FMC_DATA_UM_data_SHIFT                   (0U)
#define FMC_DATA_UM_data(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_DATA_UM_data_SHIFT)) & FMC_DATA_UM_data_MASK)

/* The count of FMC_DATA_UM */
#define FMC_DATA_UM_COUNT                        (4U)

/* The count of FMC_DATA_UM */
#define FMC_DATA_UM_COUNT2                       (4U)

/*! @name DATA_MU - Cache Data Storage (mid-upper word) */
#define FMC_DATA_MU_data_MASK                    (0xFFFFFFFFU)
#define FMC_DATA_MU_data_SHIFT                   (0U)
#define FMC_DATA_MU_data(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_DATA_MU_data_SHIFT)) & FMC_DATA_MU_data_MASK)

/* The count of FMC_DATA_MU */
#define FMC_DATA_MU_COUNT                        (4U)

/* The count of FMC_DATA_MU */
#define FMC_DATA_MU_COUNT2                       (4U)

/*! @name DATA_ML - Cache Data Storage (mid-lower word) */
#define FMC_DATA_ML_data_MASK                    (0xFFFFFFFFU)
#define FMC_DATA_ML_data_SHIFT                   (0U)
#define FMC_DATA_ML_data(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_DATA_ML_data_SHIFT)) & FMC_DATA_ML_data_MASK)

/* The count of FMC_DATA_ML */
#define FMC_DATA_ML_COUNT                        (4U)

/* The count of FMC_DATA_ML */
#define FMC_DATA_ML_COUNT2                       (4U)

/*! @name DATA_LM - Cache Data Storage (lowermost word) */
#define FMC_DATA_LM_data_MASK                    (0xFFFFFFFFU)
#define FMC_DATA_LM_data_SHIFT                   (0U)
#define FMC_DATA_LM_data(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_DATA_LM_data_SHIFT)) & FMC_DATA_LM_data_MASK)

/* The count of FMC_DATA_LM */
#define FMC_DATA_LM_COUNT                        (4U)

/* The count of FMC_DATA_LM */
#define FMC_DATA_LM_COUNT2                       (4U)


/*!
 * @}
 */ /* end of group FMC_Register_Masks */


/* FMC - Peripheral instance base addresses */
/** Peripheral FMC base address */
#define FMC_BASE                                 (0x4001F000u)
/** Peripheral FMC base pointer */
#define FMC                                      ((FMC_Type *)FMC_BASE)
/** Array initializer of FMC peripheral base addresses */
#define FMC_BASE_ADDRS                           { FMC_BASE }
/** Array initializer of FMC peripheral base pointers */
#define FMC_BASE_PTRS                            { FMC }

/*!
 * @}
 */ /* end of group FMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Peripheral_Access_Layer FTFA Peripheral Access Layer
 * @{
 */

/** FTFA - Register Layout Typedef */
typedef struct {
  __IO uint8_t FSTAT;                              /**< Flash Status Register, offset: 0x0 */
  __IO uint8_t FCNFG;                              /**< Flash Configuration Register, offset: 0x1 */
  __I  uint8_t FSEC;                               /**< Flash Security Register, offset: 0x2 */
  __I  uint8_t FOPT;                               /**< Flash Option Register, offset: 0x3 */
  __IO uint8_t FCCOB3;                             /**< Flash Common Command Object Registers, offset: 0x4 */
  __IO uint8_t FCCOB2;                             /**< Flash Common Command Object Registers, offset: 0x5 */
  __IO uint8_t FCCOB1;                             /**< Flash Common Command Object Registers, offset: 0x6 */
  __IO uint8_t FCCOB0;                             /**< Flash Common Command Object Registers, offset: 0x7 */
  __IO uint8_t FCCOB7;                             /**< Flash Common Command Object Registers, offset: 0x8 */
  __IO uint8_t FCCOB6;                             /**< Flash Common Command Object Registers, offset: 0x9 */
  __IO uint8_t FCCOB5;                             /**< Flash Common Command Object Registers, offset: 0xA */
  __IO uint8_t FCCOB4;                             /**< Flash Common Command Object Registers, offset: 0xB */
  __IO uint8_t FCCOBB;                             /**< Flash Common Command Object Registers, offset: 0xC */
  __IO uint8_t FCCOBA;                             /**< Flash Common Command Object Registers, offset: 0xD */
  __IO uint8_t FCCOB9;                             /**< Flash Common Command Object Registers, offset: 0xE */
  __IO uint8_t FCCOB8;                             /**< Flash Common Command Object Registers, offset: 0xF */
  __IO uint8_t FPROT3;                             /**< Program Flash Protection Registers, offset: 0x10 */
  __IO uint8_t FPROT2;                             /**< Program Flash Protection Registers, offset: 0x11 */
  __IO uint8_t FPROT1;                             /**< Program Flash Protection Registers, offset: 0x12 */
  __IO uint8_t FPROT0;                             /**< Program Flash Protection Registers, offset: 0x13 */
       uint8_t RESERVED_0[4];
  __I  uint8_t XACCH3;                             /**< Execute-only Access Registers, offset: 0x18 */
  __I  uint8_t XACCH2;                             /**< Execute-only Access Registers, offset: 0x19 */
  __I  uint8_t XACCH1;                             /**< Execute-only Access Registers, offset: 0x1A */
  __I  uint8_t XACCH0;                             /**< Execute-only Access Registers, offset: 0x1B */
  __I  uint8_t XACCL3;                             /**< Execute-only Access Registers, offset: 0x1C */
  __I  uint8_t XACCL2;                             /**< Execute-only Access Registers, offset: 0x1D */
  __I  uint8_t XACCL1;                             /**< Execute-only Access Registers, offset: 0x1E */
  __I  uint8_t XACCL0;                             /**< Execute-only Access Registers, offset: 0x1F */
  __I  uint8_t SACCH3;                             /**< Supervisor-only Access Registers, offset: 0x20 */
  __I  uint8_t SACCH2;                             /**< Supervisor-only Access Registers, offset: 0x21 */
  __I  uint8_t SACCH1;                             /**< Supervisor-only Access Registers, offset: 0x22 */
  __I  uint8_t SACCH0;                             /**< Supervisor-only Access Registers, offset: 0x23 */
  __I  uint8_t SACCL3;                             /**< Supervisor-only Access Registers, offset: 0x24 */
  __I  uint8_t SACCL2;                             /**< Supervisor-only Access Registers, offset: 0x25 */
  __I  uint8_t SACCL1;                             /**< Supervisor-only Access Registers, offset: 0x26 */
  __I  uint8_t SACCL0;                             /**< Supervisor-only Access Registers, offset: 0x27 */
  __I  uint8_t FACSS;                              /**< Flash Access Segment Size Register, offset: 0x28 */
       uint8_t RESERVED_1[2];
  __I  uint8_t FACSN;                              /**< Flash Access Segment Number Register, offset: 0x2B */
} FTFA_Type;

/* ----------------------------------------------------------------------------
   -- FTFA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Masks FTFA Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
#define FTFA_FSTAT_MGSTAT0_MASK                  (0x1U)
#define FTFA_FSTAT_MGSTAT0_SHIFT                 (0U)
#define FTFA_FSTAT_MGSTAT0(x)                    (((uint8_t)(((uint8_t)(x)) << FTFA_FSTAT_MGSTAT0_SHIFT)) & FTFA_FSTAT_MGSTAT0_MASK)
#define FTFA_FSTAT_FPVIOL_MASK                   (0x10U)
#define FTFA_FSTAT_FPVIOL_SHIFT                  (4U)
#define FTFA_FSTAT_FPVIOL(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FSTAT_FPVIOL_SHIFT)) & FTFA_FSTAT_FPVIOL_MASK)
#define FTFA_FSTAT_ACCERR_MASK                   (0x20U)
#define FTFA_FSTAT_ACCERR_SHIFT                  (5U)
#define FTFA_FSTAT_ACCERR(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FSTAT_ACCERR_SHIFT)) & FTFA_FSTAT_ACCERR_MASK)
#define FTFA_FSTAT_RDCOLERR_MASK                 (0x40U)
#define FTFA_FSTAT_RDCOLERR_SHIFT                (6U)
#define FTFA_FSTAT_RDCOLERR(x)                   (((uint8_t)(((uint8_t)(x)) << FTFA_FSTAT_RDCOLERR_SHIFT)) & FTFA_FSTAT_RDCOLERR_MASK)
#define FTFA_FSTAT_CCIF_MASK                     (0x80U)
#define FTFA_FSTAT_CCIF_SHIFT                    (7U)
#define FTFA_FSTAT_CCIF(x)                       (((uint8_t)(((uint8_t)(x)) << FTFA_FSTAT_CCIF_SHIFT)) & FTFA_FSTAT_CCIF_MASK)

/*! @name FCNFG - Flash Configuration Register */
#define FTFA_FCNFG_ERSSUSP_MASK                  (0x10U)
#define FTFA_FCNFG_ERSSUSP_SHIFT                 (4U)
#define FTFA_FCNFG_ERSSUSP(x)                    (((uint8_t)(((uint8_t)(x)) << FTFA_FCNFG_ERSSUSP_SHIFT)) & FTFA_FCNFG_ERSSUSP_MASK)
#define FTFA_FCNFG_ERSAREQ_MASK                  (0x20U)
#define FTFA_FCNFG_ERSAREQ_SHIFT                 (5U)
#define FTFA_FCNFG_ERSAREQ(x)                    (((uint8_t)(((uint8_t)(x)) << FTFA_FCNFG_ERSAREQ_SHIFT)) & FTFA_FCNFG_ERSAREQ_MASK)
#define FTFA_FCNFG_RDCOLLIE_MASK                 (0x40U)
#define FTFA_FCNFG_RDCOLLIE_SHIFT                (6U)
#define FTFA_FCNFG_RDCOLLIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFA_FCNFG_RDCOLLIE_SHIFT)) & FTFA_FCNFG_RDCOLLIE_MASK)
#define FTFA_FCNFG_CCIE_MASK                     (0x80U)
#define FTFA_FCNFG_CCIE_SHIFT                    (7U)
#define FTFA_FCNFG_CCIE(x)                       (((uint8_t)(((uint8_t)(x)) << FTFA_FCNFG_CCIE_SHIFT)) & FTFA_FCNFG_CCIE_MASK)

/*! @name FSEC - Flash Security Register */
#define FTFA_FSEC_SEC_MASK                       (0x3U)
#define FTFA_FSEC_SEC_SHIFT                      (0U)
#define FTFA_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x)) << FTFA_FSEC_SEC_SHIFT)) & FTFA_FSEC_SEC_MASK)
#define FTFA_FSEC_FSLACC_MASK                    (0xCU)
#define FTFA_FSEC_FSLACC_SHIFT                   (2U)
#define FTFA_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FSEC_FSLACC_SHIFT)) & FTFA_FSEC_FSLACC_MASK)
#define FTFA_FSEC_MEEN_MASK                      (0x30U)
#define FTFA_FSEC_MEEN_SHIFT                     (4U)
#define FTFA_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_FSEC_MEEN_SHIFT)) & FTFA_FSEC_MEEN_MASK)
#define FTFA_FSEC_KEYEN_MASK                     (0xC0U)
#define FTFA_FSEC_KEYEN_SHIFT                    (6U)
#define FTFA_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x)) << FTFA_FSEC_KEYEN_SHIFT)) & FTFA_FSEC_KEYEN_MASK)

/*! @name FOPT - Flash Option Register */
#define FTFA_FOPT_OPT_MASK                       (0xFFU)
#define FTFA_FOPT_OPT_SHIFT                      (0U)
#define FTFA_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x)) << FTFA_FOPT_OPT_SHIFT)) & FTFA_FOPT_OPT_MASK)

/*! @name FCCOB3 - Flash Common Command Object Registers */
#define FTFA_FCCOB3_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB3_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB3_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB3_CCOBn_SHIFT)) & FTFA_FCCOB3_CCOBn_MASK)

/*! @name FCCOB2 - Flash Common Command Object Registers */
#define FTFA_FCCOB2_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB2_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB2_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB2_CCOBn_SHIFT)) & FTFA_FCCOB2_CCOBn_MASK)

/*! @name FCCOB1 - Flash Common Command Object Registers */
#define FTFA_FCCOB1_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB1_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB1_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB1_CCOBn_SHIFT)) & FTFA_FCCOB1_CCOBn_MASK)

/*! @name FCCOB0 - Flash Common Command Object Registers */
#define FTFA_FCCOB0_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB0_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB0_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB0_CCOBn_SHIFT)) & FTFA_FCCOB0_CCOBn_MASK)

/*! @name FCCOB7 - Flash Common Command Object Registers */
#define FTFA_FCCOB7_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB7_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB7_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB7_CCOBn_SHIFT)) & FTFA_FCCOB7_CCOBn_MASK)

/*! @name FCCOB6 - Flash Common Command Object Registers */
#define FTFA_FCCOB6_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB6_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB6_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB6_CCOBn_SHIFT)) & FTFA_FCCOB6_CCOBn_MASK)

/*! @name FCCOB5 - Flash Common Command Object Registers */
#define FTFA_FCCOB5_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB5_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB5_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB5_CCOBn_SHIFT)) & FTFA_FCCOB5_CCOBn_MASK)

/*! @name FCCOB4 - Flash Common Command Object Registers */
#define FTFA_FCCOB4_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB4_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB4_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB4_CCOBn_SHIFT)) & FTFA_FCCOB4_CCOBn_MASK)

/*! @name FCCOBB - Flash Common Command Object Registers */
#define FTFA_FCCOBB_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOBB_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOBB_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOBB_CCOBn_SHIFT)) & FTFA_FCCOBB_CCOBn_MASK)

/*! @name FCCOBA - Flash Common Command Object Registers */
#define FTFA_FCCOBA_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOBA_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOBA_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOBA_CCOBn_SHIFT)) & FTFA_FCCOBA_CCOBn_MASK)

/*! @name FCCOB9 - Flash Common Command Object Registers */
#define FTFA_FCCOB9_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB9_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB9_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB9_CCOBn_SHIFT)) & FTFA_FCCOB9_CCOBn_MASK)

/*! @name FCCOB8 - Flash Common Command Object Registers */
#define FTFA_FCCOB8_CCOBn_MASK                   (0xFFU)
#define FTFA_FCCOB8_CCOBn_SHIFT                  (0U)
#define FTFA_FCCOB8_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FCCOB8_CCOBn_SHIFT)) & FTFA_FCCOB8_CCOBn_MASK)

/*! @name FPROT3 - Program Flash Protection Registers */
#define FTFA_FPROT3_PROT_MASK                    (0xFFU)
#define FTFA_FPROT3_PROT_SHIFT                   (0U)
#define FTFA_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FPROT3_PROT_SHIFT)) & FTFA_FPROT3_PROT_MASK)

/*! @name FPROT2 - Program Flash Protection Registers */
#define FTFA_FPROT2_PROT_MASK                    (0xFFU)
#define FTFA_FPROT2_PROT_SHIFT                   (0U)
#define FTFA_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FPROT2_PROT_SHIFT)) & FTFA_FPROT2_PROT_MASK)

/*! @name FPROT1 - Program Flash Protection Registers */
#define FTFA_FPROT1_PROT_MASK                    (0xFFU)
#define FTFA_FPROT1_PROT_SHIFT                   (0U)
#define FTFA_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FPROT1_PROT_SHIFT)) & FTFA_FPROT1_PROT_MASK)

/*! @name FPROT0 - Program Flash Protection Registers */
#define FTFA_FPROT0_PROT_MASK                    (0xFFU)
#define FTFA_FPROT0_PROT_SHIFT                   (0U)
#define FTFA_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FPROT0_PROT_SHIFT)) & FTFA_FPROT0_PROT_MASK)

/*! @name XACCH3 - Execute-only Access Registers */
#define FTFA_XACCH3_XA_MASK                      (0xFFU)
#define FTFA_XACCH3_XA_SHIFT                     (0U)
#define FTFA_XACCH3_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCH3_XA_SHIFT)) & FTFA_XACCH3_XA_MASK)

/*! @name XACCH2 - Execute-only Access Registers */
#define FTFA_XACCH2_XA_MASK                      (0xFFU)
#define FTFA_XACCH2_XA_SHIFT                     (0U)
#define FTFA_XACCH2_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCH2_XA_SHIFT)) & FTFA_XACCH2_XA_MASK)

/*! @name XACCH1 - Execute-only Access Registers */
#define FTFA_XACCH1_XA_MASK                      (0xFFU)
#define FTFA_XACCH1_XA_SHIFT                     (0U)
#define FTFA_XACCH1_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCH1_XA_SHIFT)) & FTFA_XACCH1_XA_MASK)

/*! @name XACCH0 - Execute-only Access Registers */
#define FTFA_XACCH0_XA_MASK                      (0xFFU)
#define FTFA_XACCH0_XA_SHIFT                     (0U)
#define FTFA_XACCH0_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCH0_XA_SHIFT)) & FTFA_XACCH0_XA_MASK)

/*! @name XACCL3 - Execute-only Access Registers */
#define FTFA_XACCL3_XA_MASK                      (0xFFU)
#define FTFA_XACCL3_XA_SHIFT                     (0U)
#define FTFA_XACCL3_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCL3_XA_SHIFT)) & FTFA_XACCL3_XA_MASK)

/*! @name XACCL2 - Execute-only Access Registers */
#define FTFA_XACCL2_XA_MASK                      (0xFFU)
#define FTFA_XACCL2_XA_SHIFT                     (0U)
#define FTFA_XACCL2_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCL2_XA_SHIFT)) & FTFA_XACCL2_XA_MASK)

/*! @name XACCL1 - Execute-only Access Registers */
#define FTFA_XACCL1_XA_MASK                      (0xFFU)
#define FTFA_XACCL1_XA_SHIFT                     (0U)
#define FTFA_XACCL1_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCL1_XA_SHIFT)) & FTFA_XACCL1_XA_MASK)

/*! @name XACCL0 - Execute-only Access Registers */
#define FTFA_XACCL0_XA_MASK                      (0xFFU)
#define FTFA_XACCL0_XA_SHIFT                     (0U)
#define FTFA_XACCL0_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_XACCL0_XA_SHIFT)) & FTFA_XACCL0_XA_MASK)

/*! @name SACCH3 - Supervisor-only Access Registers */
#define FTFA_SACCH3_SA_MASK                      (0xFFU)
#define FTFA_SACCH3_SA_SHIFT                     (0U)
#define FTFA_SACCH3_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCH3_SA_SHIFT)) & FTFA_SACCH3_SA_MASK)

/*! @name SACCH2 - Supervisor-only Access Registers */
#define FTFA_SACCH2_SA_MASK                      (0xFFU)
#define FTFA_SACCH2_SA_SHIFT                     (0U)
#define FTFA_SACCH2_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCH2_SA_SHIFT)) & FTFA_SACCH2_SA_MASK)

/*! @name SACCH1 - Supervisor-only Access Registers */
#define FTFA_SACCH1_SA_MASK                      (0xFFU)
#define FTFA_SACCH1_SA_SHIFT                     (0U)
#define FTFA_SACCH1_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCH1_SA_SHIFT)) & FTFA_SACCH1_SA_MASK)

/*! @name SACCH0 - Supervisor-only Access Registers */
#define FTFA_SACCH0_SA_MASK                      (0xFFU)
#define FTFA_SACCH0_SA_SHIFT                     (0U)
#define FTFA_SACCH0_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCH0_SA_SHIFT)) & FTFA_SACCH0_SA_MASK)

/*! @name SACCL3 - Supervisor-only Access Registers */
#define FTFA_SACCL3_SA_MASK                      (0xFFU)
#define FTFA_SACCL3_SA_SHIFT                     (0U)
#define FTFA_SACCL3_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCL3_SA_SHIFT)) & FTFA_SACCL3_SA_MASK)

/*! @name SACCL2 - Supervisor-only Access Registers */
#define FTFA_SACCL2_SA_MASK                      (0xFFU)
#define FTFA_SACCL2_SA_SHIFT                     (0U)
#define FTFA_SACCL2_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCL2_SA_SHIFT)) & FTFA_SACCL2_SA_MASK)

/*! @name SACCL1 - Supervisor-only Access Registers */
#define FTFA_SACCL1_SA_MASK                      (0xFFU)
#define FTFA_SACCL1_SA_SHIFT                     (0U)
#define FTFA_SACCL1_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCL1_SA_SHIFT)) & FTFA_SACCL1_SA_MASK)

/*! @name SACCL0 - Supervisor-only Access Registers */
#define FTFA_SACCL0_SA_MASK                      (0xFFU)
#define FTFA_SACCL0_SA_SHIFT                     (0U)
#define FTFA_SACCL0_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFA_SACCL0_SA_SHIFT)) & FTFA_SACCL0_SA_MASK)

/*! @name FACSS - Flash Access Segment Size Register */
#define FTFA_FACSS_SGSIZE_MASK                   (0xFFU)
#define FTFA_FACSS_SGSIZE_SHIFT                  (0U)
#define FTFA_FACSS_SGSIZE(x)                     (((uint8_t)(((uint8_t)(x)) << FTFA_FACSS_SGSIZE_SHIFT)) & FTFA_FACSS_SGSIZE_MASK)

/*! @name FACSN - Flash Access Segment Number Register */
#define FTFA_FACSN_NUMSG_MASK                    (0xFFU)
#define FTFA_FACSN_NUMSG_SHIFT                   (0U)
#define FTFA_FACSN_NUMSG(x)                      (((uint8_t)(((uint8_t)(x)) << FTFA_FACSN_NUMSG_SHIFT)) & FTFA_FACSN_NUMSG_MASK)


/*!
 * @}
 */ /* end of group FTFA_Register_Masks */


/* FTFA - Peripheral instance base addresses */
/** Peripheral FTFA base address */
#define FTFA_BASE                                (0x40020000u)
/** Peripheral FTFA base pointer */
#define FTFA                                     ((FTFA_Type *)FTFA_BASE)
/** Array initializer of FTFA peripheral base addresses */
#define FTFA_BASE_ADDRS                          { FTFA_BASE }
/** Array initializer of FTFA peripheral base pointers */
#define FTFA_BASE_PTRS                           { FTFA }
/** Interrupt vectors for the FTFA peripheral type */
#define FTFA_COMMAND_COMPLETE_IRQS               { FTFA_IRQn }
#define FTFA_READ_COLLISION_IRQS                 { Read_Collision_IRQn }

/*!
 * @}
 */ /* end of group FTFA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status And Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[8];
  __IO uint32_t CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  __IO uint32_t STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  __IO uint32_t MODE;                              /**< Features Mode Selection, offset: 0x54 */
  __IO uint32_t SYNC;                              /**< Synchronization, offset: 0x58 */
  __IO uint32_t OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  __IO uint32_t OUTMASK;                           /**< Output Mask, offset: 0x60 */
  __IO uint32_t COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  __IO uint32_t DEADTIME;                          /**< Deadtime Insertion Control, offset: 0x68 */
  __IO uint32_t EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  __IO uint32_t POL;                               /**< Channels Polarity, offset: 0x70 */
  __IO uint32_t FMS;                               /**< Fault Mode Status, offset: 0x74 */
  __IO uint32_t FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  __IO uint32_t FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80 */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
  __IO uint32_t FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  __IO uint32_t SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  __IO uint32_t INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  __IO uint32_t SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  __IO uint32_t PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
} FTM_Type;

/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/*! @name SC - Status And Control */
#define FTM_SC_PS_MASK                           (0x7U)
#define FTM_SC_PS_SHIFT                          (0U)
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_PS_SHIFT)) & FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         (0x18U)
#define FTM_SC_CLKS_SHIFT                        (3U)
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_CLKS_SHIFT)) & FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS_MASK                        (0x20U)
#define FTM_SC_CPWMS_SHIFT                       (5U)
#define FTM_SC_CPWMS(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_SC_CPWMS_SHIFT)) & FTM_SC_CPWMS_MASK)
#define FTM_SC_TOIE_MASK                         (0x40U)
#define FTM_SC_TOIE_SHIFT                        (6U)
#define FTM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOIE_SHIFT)) & FTM_SC_TOIE_MASK)
#define FTM_SC_TOF_MASK                          (0x80U)
#define FTM_SC_TOF_SHIFT                         (7U)
#define FTM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOF_SHIFT)) & FTM_SC_TOF_MASK)

/*! @name CNT - Counter */
#define FTM_CNT_COUNT_MASK                       (0xFFFFU)
#define FTM_CNT_COUNT_SHIFT                      (0U)
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CNT_COUNT_SHIFT)) & FTM_CNT_COUNT_MASK)

/*! @name MOD - Modulo */
#define FTM_MOD_MOD_MASK                         (0xFFFFU)
#define FTM_MOD_MOD_SHIFT                        (0U)
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MOD_SHIFT)) & FTM_MOD_MOD_MASK)

/*! @name CnSC - Channel (n) Status And Control */
#define FTM_CnSC_DMA_MASK                        (0x1U)
#define FTM_CnSC_DMA_SHIFT                       (0U)
#define FTM_CnSC_DMA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_DMA_SHIFT)) & FTM_CnSC_DMA_MASK)
#define FTM_CnSC_ICRST_MASK                      (0x2U)
#define FTM_CnSC_ICRST_SHIFT                     (1U)
#define FTM_CnSC_ICRST(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ICRST_SHIFT)) & FTM_CnSC_ICRST_MASK)
#define FTM_CnSC_ELSA_MASK                       (0x4U)
#define FTM_CnSC_ELSA_SHIFT                      (2U)
#define FTM_CnSC_ELSA(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSA_SHIFT)) & FTM_CnSC_ELSA_MASK)
#define FTM_CnSC_ELSB_MASK                       (0x8U)
#define FTM_CnSC_ELSB_SHIFT                      (3U)
#define FTM_CnSC_ELSB(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSB_SHIFT)) & FTM_CnSC_ELSB_MASK)
#define FTM_CnSC_MSA_MASK                        (0x10U)
#define FTM_CnSC_MSA_SHIFT                       (4U)
#define FTM_CnSC_MSA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSA_SHIFT)) & FTM_CnSC_MSA_MASK)
#define FTM_CnSC_MSB_MASK                        (0x20U)
#define FTM_CnSC_MSB_SHIFT                       (5U)
#define FTM_CnSC_MSB(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSB_SHIFT)) & FTM_CnSC_MSB_MASK)
#define FTM_CnSC_CHIE_MASK                       (0x40U)
#define FTM_CnSC_CHIE_SHIFT                      (6U)
#define FTM_CnSC_CHIE(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIE_SHIFT)) & FTM_CnSC_CHIE_MASK)
#define FTM_CnSC_CHF_MASK                        (0x80U)
#define FTM_CnSC_CHF_SHIFT                       (7U)
#define FTM_CnSC_CHF(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHF_SHIFT)) & FTM_CnSC_CHF_MASK)

/* The count of FTM_CnSC */
#define FTM_CnSC_COUNT                           (8U)

/*! @name CnV - Channel (n) Value */
#define FTM_CnV_VAL_MASK                         (0xFFFFU)
#define FTM_CnV_VAL_SHIFT                        (0U)
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_CnV_VAL_SHIFT)) & FTM_CnV_VAL_MASK)

/* The count of FTM_CnV */
#define FTM_CnV_COUNT                            (8U)

/*! @name CNTIN - Counter Initial Value */
#define FTM_CNTIN_INIT_MASK                      (0xFFFFU)
#define FTM_CNTIN_INIT_SHIFT                     (0U)
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CNTIN_INIT_SHIFT)) & FTM_CNTIN_INIT_MASK)

/*! @name STATUS - Capture And Compare Status */
#define FTM_STATUS_CH0F_MASK                     (0x1U)
#define FTM_STATUS_CH0F_SHIFT                    (0U)
#define FTM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH0F_SHIFT)) & FTM_STATUS_CH0F_MASK)
#define FTM_STATUS_CH1F_MASK                     (0x2U)
#define FTM_STATUS_CH1F_SHIFT                    (1U)
#define FTM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH1F_SHIFT)) & FTM_STATUS_CH1F_MASK)
#define FTM_STATUS_CH2F_MASK                     (0x4U)
#define FTM_STATUS_CH2F_SHIFT                    (2U)
#define FTM_STATUS_CH2F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH2F_SHIFT)) & FTM_STATUS_CH2F_MASK)
#define FTM_STATUS_CH3F_MASK                     (0x8U)
#define FTM_STATUS_CH3F_SHIFT                    (3U)
#define FTM_STATUS_CH3F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH3F_SHIFT)) & FTM_STATUS_CH3F_MASK)
#define FTM_STATUS_CH4F_MASK                     (0x10U)
#define FTM_STATUS_CH4F_SHIFT                    (4U)
#define FTM_STATUS_CH4F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH4F_SHIFT)) & FTM_STATUS_CH4F_MASK)
#define FTM_STATUS_CH5F_MASK                     (0x20U)
#define FTM_STATUS_CH5F_SHIFT                    (5U)
#define FTM_STATUS_CH5F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH5F_SHIFT)) & FTM_STATUS_CH5F_MASK)
#define FTM_STATUS_CH6F_MASK                     (0x40U)
#define FTM_STATUS_CH6F_SHIFT                    (6U)
#define FTM_STATUS_CH6F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH6F_SHIFT)) & FTM_STATUS_CH6F_MASK)
#define FTM_STATUS_CH7F_MASK                     (0x80U)
#define FTM_STATUS_CH7F_SHIFT                    (7U)
#define FTM_STATUS_CH7F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH7F_SHIFT)) & FTM_STATUS_CH7F_MASK)

/*! @name MODE - Features Mode Selection */
#define FTM_MODE_FTMEN_MASK                      (0x1U)
#define FTM_MODE_FTMEN_SHIFT                     (0U)
#define FTM_MODE_FTMEN(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FTMEN_SHIFT)) & FTM_MODE_FTMEN_MASK)
#define FTM_MODE_INIT_MASK                       (0x2U)
#define FTM_MODE_INIT_SHIFT                      (1U)
#define FTM_MODE_INIT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_MODE_INIT_SHIFT)) & FTM_MODE_INIT_MASK)
#define FTM_MODE_WPDIS_MASK                      (0x4U)
#define FTM_MODE_WPDIS_SHIFT                     (2U)
#define FTM_MODE_WPDIS(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_WPDIS_SHIFT)) & FTM_MODE_WPDIS_MASK)
#define FTM_MODE_PWMSYNC_MASK                    (0x8U)
#define FTM_MODE_PWMSYNC_SHIFT                   (3U)
#define FTM_MODE_PWMSYNC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_PWMSYNC_SHIFT)) & FTM_MODE_PWMSYNC_MASK)
#define FTM_MODE_CAPTEST_MASK                    (0x10U)
#define FTM_MODE_CAPTEST_SHIFT                   (4U)
#define FTM_MODE_CAPTEST(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_CAPTEST_SHIFT)) & FTM_MODE_CAPTEST_MASK)
#define FTM_MODE_FAULTM_MASK                     (0x60U)
#define FTM_MODE_FAULTM_SHIFT                    (5U)
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTM_SHIFT)) & FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE_MASK                    (0x80U)
#define FTM_MODE_FAULTIE_SHIFT                   (7U)
#define FTM_MODE_FAULTIE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTIE_SHIFT)) & FTM_MODE_FAULTIE_MASK)

/*! @name SYNC - Synchronization */
#define FTM_SYNC_CNTMIN_MASK                     (0x1U)
#define FTM_SYNC_CNTMIN_SHIFT                    (0U)
#define FTM_SYNC_CNTMIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMIN_SHIFT)) & FTM_SYNC_CNTMIN_MASK)
#define FTM_SYNC_CNTMAX_MASK                     (0x2U)
#define FTM_SYNC_CNTMAX_SHIFT                    (1U)
#define FTM_SYNC_CNTMAX(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMAX_SHIFT)) & FTM_SYNC_CNTMAX_MASK)
#define FTM_SYNC_REINIT_MASK                     (0x4U)
#define FTM_SYNC_REINIT_SHIFT                    (2U)
#define FTM_SYNC_REINIT(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_REINIT_SHIFT)) & FTM_SYNC_REINIT_MASK)
#define FTM_SYNC_SYNCHOM_MASK                    (0x8U)
#define FTM_SYNC_SYNCHOM_SHIFT                   (3U)
#define FTM_SYNC_SYNCHOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SYNCHOM_SHIFT)) & FTM_SYNC_SYNCHOM_MASK)
#define FTM_SYNC_TRIG0_MASK                      (0x10U)
#define FTM_SYNC_TRIG0_SHIFT                     (4U)
#define FTM_SYNC_TRIG0(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG0_SHIFT)) & FTM_SYNC_TRIG0_MASK)
#define FTM_SYNC_TRIG1_MASK                      (0x20U)
#define FTM_SYNC_TRIG1_SHIFT                     (5U)
#define FTM_SYNC_TRIG1(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG1_SHIFT)) & FTM_SYNC_TRIG1_MASK)
#define FTM_SYNC_TRIG2_MASK                      (0x40U)
#define FTM_SYNC_TRIG2_SHIFT                     (6U)
#define FTM_SYNC_TRIG2(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG2_SHIFT)) & FTM_SYNC_TRIG2_MASK)
#define FTM_SYNC_SWSYNC_MASK                     (0x80U)
#define FTM_SYNC_SWSYNC_SHIFT                    (7U)
#define FTM_SYNC_SWSYNC(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SWSYNC_SHIFT)) & FTM_SYNC_SWSYNC_MASK)

/*! @name OUTINIT - Initial State For Channels Output */
#define FTM_OUTINIT_CH0OI_MASK                   (0x1U)
#define FTM_OUTINIT_CH0OI_SHIFT                  (0U)
#define FTM_OUTINIT_CH0OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH0OI_SHIFT)) & FTM_OUTINIT_CH0OI_MASK)
#define FTM_OUTINIT_CH1OI_MASK                   (0x2U)
#define FTM_OUTINIT_CH1OI_SHIFT                  (1U)
#define FTM_OUTINIT_CH1OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH1OI_SHIFT)) & FTM_OUTINIT_CH1OI_MASK)
#define FTM_OUTINIT_CH2OI_MASK                   (0x4U)
#define FTM_OUTINIT_CH2OI_SHIFT                  (2U)
#define FTM_OUTINIT_CH2OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH2OI_SHIFT)) & FTM_OUTINIT_CH2OI_MASK)
#define FTM_OUTINIT_CH3OI_MASK                   (0x8U)
#define FTM_OUTINIT_CH3OI_SHIFT                  (3U)
#define FTM_OUTINIT_CH3OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH3OI_SHIFT)) & FTM_OUTINIT_CH3OI_MASK)
#define FTM_OUTINIT_CH4OI_MASK                   (0x10U)
#define FTM_OUTINIT_CH4OI_SHIFT                  (4U)
#define FTM_OUTINIT_CH4OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH4OI_SHIFT)) & FTM_OUTINIT_CH4OI_MASK)
#define FTM_OUTINIT_CH5OI_MASK                   (0x20U)
#define FTM_OUTINIT_CH5OI_SHIFT                  (5U)
#define FTM_OUTINIT_CH5OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH5OI_SHIFT)) & FTM_OUTINIT_CH5OI_MASK)
#define FTM_OUTINIT_CH6OI_MASK                   (0x40U)
#define FTM_OUTINIT_CH6OI_SHIFT                  (6U)
#define FTM_OUTINIT_CH6OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH6OI_SHIFT)) & FTM_OUTINIT_CH6OI_MASK)
#define FTM_OUTINIT_CH7OI_MASK                   (0x80U)
#define FTM_OUTINIT_CH7OI_SHIFT                  (7U)
#define FTM_OUTINIT_CH7OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH7OI_SHIFT)) & FTM_OUTINIT_CH7OI_MASK)

/*! @name OUTMASK - Output Mask */
#define FTM_OUTMASK_CH0OM_MASK                   (0x1U)
#define FTM_OUTMASK_CH0OM_SHIFT                  (0U)
#define FTM_OUTMASK_CH0OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH0OM_SHIFT)) & FTM_OUTMASK_CH0OM_MASK)
#define FTM_OUTMASK_CH1OM_MASK                   (0x2U)
#define FTM_OUTMASK_CH1OM_SHIFT                  (1U)
#define FTM_OUTMASK_CH1OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH1OM_SHIFT)) & FTM_OUTMASK_CH1OM_MASK)
#define FTM_OUTMASK_CH2OM_MASK                   (0x4U)
#define FTM_OUTMASK_CH2OM_SHIFT                  (2U)
#define FTM_OUTMASK_CH2OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH2OM_SHIFT)) & FTM_OUTMASK_CH2OM_MASK)
#define FTM_OUTMASK_CH3OM_MASK                   (0x8U)
#define FTM_OUTMASK_CH3OM_SHIFT                  (3U)
#define FTM_OUTMASK_CH3OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH3OM_SHIFT)) & FTM_OUTMASK_CH3OM_MASK)
#define FTM_OUTMASK_CH4OM_MASK                   (0x10U)
#define FTM_OUTMASK_CH4OM_SHIFT                  (4U)
#define FTM_OUTMASK_CH4OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH4OM_SHIFT)) & FTM_OUTMASK_CH4OM_MASK)
#define FTM_OUTMASK_CH5OM_MASK                   (0x20U)
#define FTM_OUTMASK_CH5OM_SHIFT                  (5U)
#define FTM_OUTMASK_CH5OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH5OM_SHIFT)) & FTM_OUTMASK_CH5OM_MASK)
#define FTM_OUTMASK_CH6OM_MASK                   (0x40U)
#define FTM_OUTMASK_CH6OM_SHIFT                  (6U)
#define FTM_OUTMASK_CH6OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH6OM_SHIFT)) & FTM_OUTMASK_CH6OM_MASK)
#define FTM_OUTMASK_CH7OM_MASK                   (0x80U)
#define FTM_OUTMASK_CH7OM_SHIFT                  (7U)
#define FTM_OUTMASK_CH7OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH7OM_SHIFT)) & FTM_OUTMASK_CH7OM_MASK)

/*! @name COMBINE - Function For Linked Channels */
#define FTM_COMBINE_COMBINE0_MASK                (0x1U)
#define FTM_COMBINE_COMBINE0_SHIFT               (0U)
#define FTM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE0_SHIFT)) & FTM_COMBINE_COMBINE0_MASK)
#define FTM_COMBINE_COMP0_MASK                   (0x2U)
#define FTM_COMBINE_COMP0_SHIFT                  (1U)
#define FTM_COMBINE_COMP0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP0_SHIFT)) & FTM_COMBINE_COMP0_MASK)
#define FTM_COMBINE_DECAPEN0_MASK                (0x4U)
#define FTM_COMBINE_DECAPEN0_SHIFT               (2U)
#define FTM_COMBINE_DECAPEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN0_SHIFT)) & FTM_COMBINE_DECAPEN0_MASK)
#define FTM_COMBINE_DECAP0_MASK                  (0x8U)
#define FTM_COMBINE_DECAP0_SHIFT                 (3U)
#define FTM_COMBINE_DECAP0(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP0_SHIFT)) & FTM_COMBINE_DECAP0_MASK)
#define FTM_COMBINE_DTEN0_MASK                   (0x10U)
#define FTM_COMBINE_DTEN0_SHIFT                  (4U)
#define FTM_COMBINE_DTEN0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN0_SHIFT)) & FTM_COMBINE_DTEN0_MASK)
#define FTM_COMBINE_SYNCEN0_MASK                 (0x20U)
#define FTM_COMBINE_SYNCEN0_SHIFT                (5U)
#define FTM_COMBINE_SYNCEN0(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN0_SHIFT)) & FTM_COMBINE_SYNCEN0_MASK)
#define FTM_COMBINE_FAULTEN0_MASK                (0x40U)
#define FTM_COMBINE_FAULTEN0_SHIFT               (6U)
#define FTM_COMBINE_FAULTEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN0_SHIFT)) & FTM_COMBINE_FAULTEN0_MASK)
#define FTM_COMBINE_COMBINE1_MASK                (0x100U)
#define FTM_COMBINE_COMBINE1_SHIFT               (8U)
#define FTM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE1_SHIFT)) & FTM_COMBINE_COMBINE1_MASK)
#define FTM_COMBINE_COMP1_MASK                   (0x200U)
#define FTM_COMBINE_COMP1_SHIFT                  (9U)
#define FTM_COMBINE_COMP1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP1_SHIFT)) & FTM_COMBINE_COMP1_MASK)
#define FTM_COMBINE_DECAPEN1_MASK                (0x400U)
#define FTM_COMBINE_DECAPEN1_SHIFT               (10U)
#define FTM_COMBINE_DECAPEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN1_SHIFT)) & FTM_COMBINE_DECAPEN1_MASK)
#define FTM_COMBINE_DECAP1_MASK                  (0x800U)
#define FTM_COMBINE_DECAP1_SHIFT                 (11U)
#define FTM_COMBINE_DECAP1(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP1_SHIFT)) & FTM_COMBINE_DECAP1_MASK)
#define FTM_COMBINE_DTEN1_MASK                   (0x1000U)
#define FTM_COMBINE_DTEN1_SHIFT                  (12U)
#define FTM_COMBINE_DTEN1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN1_SHIFT)) & FTM_COMBINE_DTEN1_MASK)
#define FTM_COMBINE_SYNCEN1_MASK                 (0x2000U)
#define FTM_COMBINE_SYNCEN1_SHIFT                (13U)
#define FTM_COMBINE_SYNCEN1(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN1_SHIFT)) & FTM_COMBINE_SYNCEN1_MASK)
#define FTM_COMBINE_FAULTEN1_MASK                (0x4000U)
#define FTM_COMBINE_FAULTEN1_SHIFT               (14U)
#define FTM_COMBINE_FAULTEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN1_SHIFT)) & FTM_COMBINE_FAULTEN1_MASK)
#define FTM_COMBINE_COMBINE2_MASK                (0x10000U)
#define FTM_COMBINE_COMBINE2_SHIFT               (16U)
#define FTM_COMBINE_COMBINE2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE2_SHIFT)) & FTM_COMBINE_COMBINE2_MASK)
#define FTM_COMBINE_COMP2_MASK                   (0x20000U)
#define FTM_COMBINE_COMP2_SHIFT                  (17U)
#define FTM_COMBINE_COMP2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP2_SHIFT)) & FTM_COMBINE_COMP2_MASK)
#define FTM_COMBINE_DECAPEN2_MASK                (0x40000U)
#define FTM_COMBINE_DECAPEN2_SHIFT               (18U)
#define FTM_COMBINE_DECAPEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN2_SHIFT)) & FTM_COMBINE_DECAPEN2_MASK)
#define FTM_COMBINE_DECAP2_MASK                  (0x80000U)
#define FTM_COMBINE_DECAP2_SHIFT                 (19U)
#define FTM_COMBINE_DECAP2(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP2_SHIFT)) & FTM_COMBINE_DECAP2_MASK)
#define FTM_COMBINE_DTEN2_MASK                   (0x100000U)
#define FTM_COMBINE_DTEN2_SHIFT                  (20U)
#define FTM_COMBINE_DTEN2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN2_SHIFT)) & FTM_COMBINE_DTEN2_MASK)
#define FTM_COMBINE_SYNCEN2_MASK                 (0x200000U)
#define FTM_COMBINE_SYNCEN2_SHIFT                (21U)
#define FTM_COMBINE_SYNCEN2(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN2_SHIFT)) & FTM_COMBINE_SYNCEN2_MASK)
#define FTM_COMBINE_FAULTEN2_MASK                (0x400000U)
#define FTM_COMBINE_FAULTEN2_SHIFT               (22U)
#define FTM_COMBINE_FAULTEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN2_SHIFT)) & FTM_COMBINE_FAULTEN2_MASK)
#define FTM_COMBINE_COMBINE3_MASK                (0x1000000U)
#define FTM_COMBINE_COMBINE3_SHIFT               (24U)
#define FTM_COMBINE_COMBINE3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE3_SHIFT)) & FTM_COMBINE_COMBINE3_MASK)
#define FTM_COMBINE_COMP3_MASK                   (0x2000000U)
#define FTM_COMBINE_COMP3_SHIFT                  (25U)
#define FTM_COMBINE_COMP3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP3_SHIFT)) & FTM_COMBINE_COMP3_MASK)
#define FTM_COMBINE_DECAPEN3_MASK                (0x4000000U)
#define FTM_COMBINE_DECAPEN3_SHIFT               (26U)
#define FTM_COMBINE_DECAPEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN3_SHIFT)) & FTM_COMBINE_DECAPEN3_MASK)
#define FTM_COMBINE_DECAP3_MASK                  (0x8000000U)
#define FTM_COMBINE_DECAP3_SHIFT                 (27U)
#define FTM_COMBINE_DECAP3(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP3_SHIFT)) & FTM_COMBINE_DECAP3_MASK)
#define FTM_COMBINE_DTEN3_MASK                   (0x10000000U)
#define FTM_COMBINE_DTEN3_SHIFT                  (28U)
#define FTM_COMBINE_DTEN3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN3_SHIFT)) & FTM_COMBINE_DTEN3_MASK)
#define FTM_COMBINE_SYNCEN3_MASK                 (0x20000000U)
#define FTM_COMBINE_SYNCEN3_SHIFT                (29U)
#define FTM_COMBINE_SYNCEN3(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN3_SHIFT)) & FTM_COMBINE_SYNCEN3_MASK)
#define FTM_COMBINE_FAULTEN3_MASK                (0x40000000U)
#define FTM_COMBINE_FAULTEN3_SHIFT               (30U)
#define FTM_COMBINE_FAULTEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN3_SHIFT)) & FTM_COMBINE_FAULTEN3_MASK)

/*! @name DEADTIME - Deadtime Insertion Control */
#define FTM_DEADTIME_DTVAL_MASK                  (0x3FU)
#define FTM_DEADTIME_DTVAL_SHIFT                 (0U)
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTVAL_SHIFT)) & FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   (0xC0U)
#define FTM_DEADTIME_DTPS_SHIFT                  (6U)
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTPS_SHIFT)) & FTM_DEADTIME_DTPS_MASK)

/*! @name EXTTRIG - FTM External Trigger */
#define FTM_EXTTRIG_CH2TRIG_MASK                 (0x1U)
#define FTM_EXTTRIG_CH2TRIG_SHIFT                (0U)
#define FTM_EXTTRIG_CH2TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH2TRIG_SHIFT)) & FTM_EXTTRIG_CH2TRIG_MASK)
#define FTM_EXTTRIG_CH3TRIG_MASK                 (0x2U)
#define FTM_EXTTRIG_CH3TRIG_SHIFT                (1U)
#define FTM_EXTTRIG_CH3TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH3TRIG_SHIFT)) & FTM_EXTTRIG_CH3TRIG_MASK)
#define FTM_EXTTRIG_CH4TRIG_MASK                 (0x4U)
#define FTM_EXTTRIG_CH4TRIG_SHIFT                (2U)
#define FTM_EXTTRIG_CH4TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH4TRIG_SHIFT)) & FTM_EXTTRIG_CH4TRIG_MASK)
#define FTM_EXTTRIG_CH5TRIG_MASK                 (0x8U)
#define FTM_EXTTRIG_CH5TRIG_SHIFT                (3U)
#define FTM_EXTTRIG_CH5TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH5TRIG_SHIFT)) & FTM_EXTTRIG_CH5TRIG_MASK)
#define FTM_EXTTRIG_CH0TRIG_MASK                 (0x10U)
#define FTM_EXTTRIG_CH0TRIG_SHIFT                (4U)
#define FTM_EXTTRIG_CH0TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH0TRIG_SHIFT)) & FTM_EXTTRIG_CH0TRIG_MASK)
#define FTM_EXTTRIG_CH1TRIG_MASK                 (0x20U)
#define FTM_EXTTRIG_CH1TRIG_SHIFT                (5U)
#define FTM_EXTTRIG_CH1TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH1TRIG_SHIFT)) & FTM_EXTTRIG_CH1TRIG_MASK)
#define FTM_EXTTRIG_INITTRIGEN_MASK              (0x40U)
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             (6U)
#define FTM_EXTTRIG_INITTRIGEN(x)                (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_INITTRIGEN_SHIFT)) & FTM_EXTTRIG_INITTRIGEN_MASK)
#define FTM_EXTTRIG_TRIGF_MASK                   (0x80U)
#define FTM_EXTTRIG_TRIGF_SHIFT                  (7U)
#define FTM_EXTTRIG_TRIGF(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_TRIGF_SHIFT)) & FTM_EXTTRIG_TRIGF_MASK)

/*! @name POL - Channels Polarity */
#define FTM_POL_POL0_MASK                        (0x1U)
#define FTM_POL_POL0_SHIFT                       (0U)
#define FTM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL0_SHIFT)) & FTM_POL_POL0_MASK)
#define FTM_POL_POL1_MASK                        (0x2U)
#define FTM_POL_POL1_SHIFT                       (1U)
#define FTM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL1_SHIFT)) & FTM_POL_POL1_MASK)
#define FTM_POL_POL2_MASK                        (0x4U)
#define FTM_POL_POL2_SHIFT                       (2U)
#define FTM_POL_POL2(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL2_SHIFT)) & FTM_POL_POL2_MASK)
#define FTM_POL_POL3_MASK                        (0x8U)
#define FTM_POL_POL3_SHIFT                       (3U)
#define FTM_POL_POL3(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL3_SHIFT)) & FTM_POL_POL3_MASK)
#define FTM_POL_POL4_MASK                        (0x10U)
#define FTM_POL_POL4_SHIFT                       (4U)
#define FTM_POL_POL4(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL4_SHIFT)) & FTM_POL_POL4_MASK)
#define FTM_POL_POL5_MASK                        (0x20U)
#define FTM_POL_POL5_SHIFT                       (5U)
#define FTM_POL_POL5(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL5_SHIFT)) & FTM_POL_POL5_MASK)
#define FTM_POL_POL6_MASK                        (0x40U)
#define FTM_POL_POL6_SHIFT                       (6U)
#define FTM_POL_POL6(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL6_SHIFT)) & FTM_POL_POL6_MASK)
#define FTM_POL_POL7_MASK                        (0x80U)
#define FTM_POL_POL7_SHIFT                       (7U)
#define FTM_POL_POL7(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL7_SHIFT)) & FTM_POL_POL7_MASK)

/*! @name FMS - Fault Mode Status */
#define FTM_FMS_FAULTF0_MASK                     (0x1U)
#define FTM_FMS_FAULTF0_SHIFT                    (0U)
#define FTM_FMS_FAULTF0(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF0_SHIFT)) & FTM_FMS_FAULTF0_MASK)
#define FTM_FMS_FAULTF1_MASK                     (0x2U)
#define FTM_FMS_FAULTF1_SHIFT                    (1U)
#define FTM_FMS_FAULTF1(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF1_SHIFT)) & FTM_FMS_FAULTF1_MASK)
#define FTM_FMS_FAULTF2_MASK                     (0x4U)
#define FTM_FMS_FAULTF2_SHIFT                    (2U)
#define FTM_FMS_FAULTF2(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF2_SHIFT)) & FTM_FMS_FAULTF2_MASK)
#define FTM_FMS_FAULTF3_MASK                     (0x8U)
#define FTM_FMS_FAULTF3_SHIFT                    (3U)
#define FTM_FMS_FAULTF3(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF3_SHIFT)) & FTM_FMS_FAULTF3_MASK)
#define FTM_FMS_FAULTIN_MASK                     (0x20U)
#define FTM_FMS_FAULTIN_SHIFT                    (5U)
#define FTM_FMS_FAULTIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTIN_SHIFT)) & FTM_FMS_FAULTIN_MASK)
#define FTM_FMS_WPEN_MASK                        (0x40U)
#define FTM_FMS_WPEN_SHIFT                       (6U)
#define FTM_FMS_WPEN(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_FMS_WPEN_SHIFT)) & FTM_FMS_WPEN_MASK)
#define FTM_FMS_FAULTF_MASK                      (0x80U)
#define FTM_FMS_FAULTF_SHIFT                     (7U)
#define FTM_FMS_FAULTF(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF_SHIFT)) & FTM_FMS_FAULTF_MASK)

/*! @name FILTER - Input Capture Filter Control */
#define FTM_FILTER_CH0FVAL_MASK                  (0xFU)
#define FTM_FILTER_CH0FVAL_SHIFT                 (0U)
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH0FVAL_SHIFT)) & FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define FTM_FILTER_CH1FVAL_SHIFT                 (4U)
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH1FVAL_SHIFT)) & FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  (0xF00U)
#define FTM_FILTER_CH2FVAL_SHIFT                 (8U)
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH2FVAL_SHIFT)) & FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  (0xF000U)
#define FTM_FILTER_CH3FVAL_SHIFT                 (12U)
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH3FVAL_SHIFT)) & FTM_FILTER_CH3FVAL_MASK)

/*! @name FLTCTRL - Fault Control */
#define FTM_FLTCTRL_FAULT0EN_MASK                (0x1U)
#define FTM_FLTCTRL_FAULT0EN_SHIFT               (0U)
#define FTM_FLTCTRL_FAULT0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT0EN_SHIFT)) & FTM_FLTCTRL_FAULT0EN_MASK)
#define FTM_FLTCTRL_FAULT1EN_MASK                (0x2U)
#define FTM_FLTCTRL_FAULT1EN_SHIFT               (1U)
#define FTM_FLTCTRL_FAULT1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT1EN_SHIFT)) & FTM_FLTCTRL_FAULT1EN_MASK)
#define FTM_FLTCTRL_FAULT2EN_MASK                (0x4U)
#define FTM_FLTCTRL_FAULT2EN_SHIFT               (2U)
#define FTM_FLTCTRL_FAULT2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT2EN_SHIFT)) & FTM_FLTCTRL_FAULT2EN_MASK)
#define FTM_FLTCTRL_FAULT3EN_MASK                (0x8U)
#define FTM_FLTCTRL_FAULT3EN_SHIFT               (3U)
#define FTM_FLTCTRL_FAULT3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT3EN_SHIFT)) & FTM_FLTCTRL_FAULT3EN_MASK)
#define FTM_FLTCTRL_FFLTR0EN_MASK                (0x10U)
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               (4U)
#define FTM_FLTCTRL_FFLTR0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR0EN_SHIFT)) & FTM_FLTCTRL_FFLTR0EN_MASK)
#define FTM_FLTCTRL_FFLTR1EN_MASK                (0x20U)
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               (5U)
#define FTM_FLTCTRL_FFLTR1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR1EN_SHIFT)) & FTM_FLTCTRL_FFLTR1EN_MASK)
#define FTM_FLTCTRL_FFLTR2EN_MASK                (0x40U)
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               (6U)
#define FTM_FLTCTRL_FFLTR2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR2EN_SHIFT)) & FTM_FLTCTRL_FFLTR2EN_MASK)
#define FTM_FLTCTRL_FFLTR3EN_MASK                (0x80U)
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               (7U)
#define FTM_FLTCTRL_FFLTR3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR3EN_SHIFT)) & FTM_FLTCTRL_FFLTR3EN_MASK)
#define FTM_FLTCTRL_FFVAL_MASK                   (0xF00U)
#define FTM_FLTCTRL_FFVAL_SHIFT                  (8U)
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFVAL_SHIFT)) & FTM_FLTCTRL_FFVAL_MASK)

/*! @name QDCTRL - Quadrature Decoder Control And Status */
#define FTM_QDCTRL_QUADEN_MASK                   (0x1U)
#define FTM_QDCTRL_QUADEN_SHIFT                  (0U)
#define FTM_QDCTRL_QUADEN(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADEN_SHIFT)) & FTM_QDCTRL_QUADEN_MASK)
#define FTM_QDCTRL_TOFDIR_MASK                   (0x2U)
#define FTM_QDCTRL_TOFDIR_SHIFT                  (1U)
#define FTM_QDCTRL_TOFDIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_TOFDIR_SHIFT)) & FTM_QDCTRL_TOFDIR_MASK)
#define FTM_QDCTRL_QUADIR_MASK                   (0x4U)
#define FTM_QDCTRL_QUADIR_SHIFT                  (2U)
#define FTM_QDCTRL_QUADIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADIR_SHIFT)) & FTM_QDCTRL_QUADIR_MASK)
#define FTM_QDCTRL_QUADMODE_MASK                 (0x8U)
#define FTM_QDCTRL_QUADMODE_SHIFT                (3U)
#define FTM_QDCTRL_QUADMODE(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADMODE_SHIFT)) & FTM_QDCTRL_QUADMODE_MASK)
#define FTM_QDCTRL_PHBPOL_MASK                   (0x10U)
#define FTM_QDCTRL_PHBPOL_SHIFT                  (4U)
#define FTM_QDCTRL_PHBPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBPOL_SHIFT)) & FTM_QDCTRL_PHBPOL_MASK)
#define FTM_QDCTRL_PHAPOL_MASK                   (0x20U)
#define FTM_QDCTRL_PHAPOL_SHIFT                  (5U)
#define FTM_QDCTRL_PHAPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAPOL_SHIFT)) & FTM_QDCTRL_PHAPOL_MASK)
#define FTM_QDCTRL_PHBFLTREN_MASK                (0x40U)
#define FTM_QDCTRL_PHBFLTREN_SHIFT               (6U)
#define FTM_QDCTRL_PHBFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBFLTREN_SHIFT)) & FTM_QDCTRL_PHBFLTREN_MASK)
#define FTM_QDCTRL_PHAFLTREN_MASK                (0x80U)
#define FTM_QDCTRL_PHAFLTREN_SHIFT               (7U)
#define FTM_QDCTRL_PHAFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAFLTREN_SHIFT)) & FTM_QDCTRL_PHAFLTREN_MASK)

/*! @name CONF - Configuration */
#define FTM_CONF_NUMTOF_MASK                     (0x1FU)
#define FTM_CONF_NUMTOF_SHIFT                    (0U)
#define FTM_CONF_NUMTOF(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_NUMTOF_SHIFT)) & FTM_CONF_NUMTOF_MASK)
#define FTM_CONF_BDMMODE_MASK                    (0xC0U)
#define FTM_CONF_BDMMODE_SHIFT                   (6U)
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_BDMMODE_SHIFT)) & FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     (0x200U)
#define FTM_CONF_GTBEEN_SHIFT                    (9U)
#define FTM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEEN_SHIFT)) & FTM_CONF_GTBEEN_MASK)
#define FTM_CONF_GTBEOUT_MASK                    (0x400U)
#define FTM_CONF_GTBEOUT_SHIFT                   (10U)
#define FTM_CONF_GTBEOUT(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEOUT_SHIFT)) & FTM_CONF_GTBEOUT_MASK)

/*! @name FLTPOL - FTM Fault Input Polarity */
#define FTM_FLTPOL_FLT0POL_MASK                  (0x1U)
#define FTM_FLTPOL_FLT0POL_SHIFT                 (0U)
#define FTM_FLTPOL_FLT0POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT0POL_SHIFT)) & FTM_FLTPOL_FLT0POL_MASK)
#define FTM_FLTPOL_FLT1POL_MASK                  (0x2U)
#define FTM_FLTPOL_FLT1POL_SHIFT                 (1U)
#define FTM_FLTPOL_FLT1POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT1POL_SHIFT)) & FTM_FLTPOL_FLT1POL_MASK)
#define FTM_FLTPOL_FLT2POL_MASK                  (0x4U)
#define FTM_FLTPOL_FLT2POL_SHIFT                 (2U)
#define FTM_FLTPOL_FLT2POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT2POL_SHIFT)) & FTM_FLTPOL_FLT2POL_MASK)
#define FTM_FLTPOL_FLT3POL_MASK                  (0x8U)
#define FTM_FLTPOL_FLT3POL_SHIFT                 (3U)
#define FTM_FLTPOL_FLT3POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT3POL_SHIFT)) & FTM_FLTPOL_FLT3POL_MASK)

/*! @name SYNCONF - Synchronization Configuration */
#define FTM_SYNCONF_HWTRIGMODE_MASK              (0x1U)
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             (0U)
#define FTM_SYNCONF_HWTRIGMODE(x)                (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWTRIGMODE_SHIFT)) & FTM_SYNCONF_HWTRIGMODE_MASK)
#define FTM_SYNCONF_CNTINC_MASK                  (0x4U)
#define FTM_SYNCONF_CNTINC_SHIFT                 (2U)
#define FTM_SYNCONF_CNTINC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_CNTINC_SHIFT)) & FTM_SYNCONF_CNTINC_MASK)
#define FTM_SYNCONF_INVC_MASK                    (0x10U)
#define FTM_SYNCONF_INVC_SHIFT                   (4U)
#define FTM_SYNCONF_INVC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_INVC_SHIFT)) & FTM_SYNCONF_INVC_MASK)
#define FTM_SYNCONF_SWOC_MASK                    (0x20U)
#define FTM_SYNCONF_SWOC_SHIFT                   (5U)
#define FTM_SYNCONF_SWOC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOC_SHIFT)) & FTM_SYNCONF_SWOC_MASK)
#define FTM_SYNCONF_SYNCMODE_MASK                (0x80U)
#define FTM_SYNCONF_SYNCMODE_SHIFT               (7U)
#define FTM_SYNCONF_SYNCMODE(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SYNCMODE_SHIFT)) & FTM_SYNCONF_SYNCMODE_MASK)
#define FTM_SYNCONF_SWRSTCNT_MASK                (0x100U)
#define FTM_SYNCONF_SWRSTCNT_SHIFT               (8U)
#define FTM_SYNCONF_SWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWRSTCNT_SHIFT)) & FTM_SYNCONF_SWRSTCNT_MASK)
#define FTM_SYNCONF_SWWRBUF_MASK                 (0x200U)
#define FTM_SYNCONF_SWWRBUF_SHIFT                (9U)
#define FTM_SYNCONF_SWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWWRBUF_SHIFT)) & FTM_SYNCONF_SWWRBUF_MASK)
#define FTM_SYNCONF_SWOM_MASK                    (0x400U)
#define FTM_SYNCONF_SWOM_SHIFT                   (10U)
#define FTM_SYNCONF_SWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOM_SHIFT)) & FTM_SYNCONF_SWOM_MASK)
#define FTM_SYNCONF_SWINVC_MASK                  (0x800U)
#define FTM_SYNCONF_SWINVC_SHIFT                 (11U)
#define FTM_SYNCONF_SWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWINVC_SHIFT)) & FTM_SYNCONF_SWINVC_MASK)
#define FTM_SYNCONF_SWSOC_MASK                   (0x1000U)
#define FTM_SYNCONF_SWSOC_SHIFT                  (12U)
#define FTM_SYNCONF_SWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWSOC_SHIFT)) & FTM_SYNCONF_SWSOC_MASK)
#define FTM_SYNCONF_HWRSTCNT_MASK                (0x10000U)
#define FTM_SYNCONF_HWRSTCNT_SHIFT               (16U)
#define FTM_SYNCONF_HWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWRSTCNT_SHIFT)) & FTM_SYNCONF_HWRSTCNT_MASK)
#define FTM_SYNCONF_HWWRBUF_MASK                 (0x20000U)
#define FTM_SYNCONF_HWWRBUF_SHIFT                (17U)
#define FTM_SYNCONF_HWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWWRBUF_SHIFT)) & FTM_SYNCONF_HWWRBUF_MASK)
#define FTM_SYNCONF_HWOM_MASK                    (0x40000U)
#define FTM_SYNCONF_HWOM_SHIFT                   (18U)
#define FTM_SYNCONF_HWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWOM_SHIFT)) & FTM_SYNCONF_HWOM_MASK)
#define FTM_SYNCONF_HWINVC_MASK                  (0x80000U)
#define FTM_SYNCONF_HWINVC_SHIFT                 (19U)
#define FTM_SYNCONF_HWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWINVC_SHIFT)) & FTM_SYNCONF_HWINVC_MASK)
#define FTM_SYNCONF_HWSOC_MASK                   (0x100000U)
#define FTM_SYNCONF_HWSOC_SHIFT                  (20U)
#define FTM_SYNCONF_HWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWSOC_SHIFT)) & FTM_SYNCONF_HWSOC_MASK)

/*! @name INVCTRL - FTM Inverting Control */
#define FTM_INVCTRL_INV0EN_MASK                  (0x1U)
#define FTM_INVCTRL_INV0EN_SHIFT                 (0U)
#define FTM_INVCTRL_INV0EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV0EN_SHIFT)) & FTM_INVCTRL_INV0EN_MASK)
#define FTM_INVCTRL_INV1EN_MASK                  (0x2U)
#define FTM_INVCTRL_INV1EN_SHIFT                 (1U)
#define FTM_INVCTRL_INV1EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV1EN_SHIFT)) & FTM_INVCTRL_INV1EN_MASK)
#define FTM_INVCTRL_INV2EN_MASK                  (0x4U)
#define FTM_INVCTRL_INV2EN_SHIFT                 (2U)
#define FTM_INVCTRL_INV2EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV2EN_SHIFT)) & FTM_INVCTRL_INV2EN_MASK)
#define FTM_INVCTRL_INV3EN_MASK                  (0x8U)
#define FTM_INVCTRL_INV3EN_SHIFT                 (3U)
#define FTM_INVCTRL_INV3EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV3EN_SHIFT)) & FTM_INVCTRL_INV3EN_MASK)

/*! @name SWOCTRL - FTM Software Output Control */
#define FTM_SWOCTRL_CH0OC_MASK                   (0x1U)
#define FTM_SWOCTRL_CH0OC_SHIFT                  (0U)
#define FTM_SWOCTRL_CH0OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OC_SHIFT)) & FTM_SWOCTRL_CH0OC_MASK)
#define FTM_SWOCTRL_CH1OC_MASK                   (0x2U)
#define FTM_SWOCTRL_CH1OC_SHIFT                  (1U)
#define FTM_SWOCTRL_CH1OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OC_SHIFT)) & FTM_SWOCTRL_CH1OC_MASK)
#define FTM_SWOCTRL_CH2OC_MASK                   (0x4U)
#define FTM_SWOCTRL_CH2OC_SHIFT                  (2U)
#define FTM_SWOCTRL_CH2OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OC_SHIFT)) & FTM_SWOCTRL_CH2OC_MASK)
#define FTM_SWOCTRL_CH3OC_MASK                   (0x8U)
#define FTM_SWOCTRL_CH3OC_SHIFT                  (3U)
#define FTM_SWOCTRL_CH3OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OC_SHIFT)) & FTM_SWOCTRL_CH3OC_MASK)
#define FTM_SWOCTRL_CH4OC_MASK                   (0x10U)
#define FTM_SWOCTRL_CH4OC_SHIFT                  (4U)
#define FTM_SWOCTRL_CH4OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OC_SHIFT)) & FTM_SWOCTRL_CH4OC_MASK)
#define FTM_SWOCTRL_CH5OC_MASK                   (0x20U)
#define FTM_SWOCTRL_CH5OC_SHIFT                  (5U)
#define FTM_SWOCTRL_CH5OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OC_SHIFT)) & FTM_SWOCTRL_CH5OC_MASK)
#define FTM_SWOCTRL_CH6OC_MASK                   (0x40U)
#define FTM_SWOCTRL_CH6OC_SHIFT                  (6U)
#define FTM_SWOCTRL_CH6OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OC_SHIFT)) & FTM_SWOCTRL_CH6OC_MASK)
#define FTM_SWOCTRL_CH7OC_MASK                   (0x80U)
#define FTM_SWOCTRL_CH7OC_SHIFT                  (7U)
#define FTM_SWOCTRL_CH7OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OC_SHIFT)) & FTM_SWOCTRL_CH7OC_MASK)
#define FTM_SWOCTRL_CH0OCV_MASK                  (0x100U)
#define FTM_SWOCTRL_CH0OCV_SHIFT                 (8U)
#define FTM_SWOCTRL_CH0OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OCV_SHIFT)) & FTM_SWOCTRL_CH0OCV_MASK)
#define FTM_SWOCTRL_CH1OCV_MASK                  (0x200U)
#define FTM_SWOCTRL_CH1OCV_SHIFT                 (9U)
#define FTM_SWOCTRL_CH1OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OCV_SHIFT)) & FTM_SWOCTRL_CH1OCV_MASK)
#define FTM_SWOCTRL_CH2OCV_MASK                  (0x400U)
#define FTM_SWOCTRL_CH2OCV_SHIFT                 (10U)
#define FTM_SWOCTRL_CH2OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OCV_SHIFT)) & FTM_SWOCTRL_CH2OCV_MASK)
#define FTM_SWOCTRL_CH3OCV_MASK                  (0x800U)
#define FTM_SWOCTRL_CH3OCV_SHIFT                 (11U)
#define FTM_SWOCTRL_CH3OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OCV_SHIFT)) & FTM_SWOCTRL_CH3OCV_MASK)
#define FTM_SWOCTRL_CH4OCV_MASK                  (0x1000U)
#define FTM_SWOCTRL_CH4OCV_SHIFT                 (12U)
#define FTM_SWOCTRL_CH4OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OCV_SHIFT)) & FTM_SWOCTRL_CH4OCV_MASK)
#define FTM_SWOCTRL_CH5OCV_MASK                  (0x2000U)
#define FTM_SWOCTRL_CH5OCV_SHIFT                 (13U)
#define FTM_SWOCTRL_CH5OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OCV_SHIFT)) & FTM_SWOCTRL_CH5OCV_MASK)
#define FTM_SWOCTRL_CH6OCV_MASK                  (0x4000U)
#define FTM_SWOCTRL_CH6OCV_SHIFT                 (14U)
#define FTM_SWOCTRL_CH6OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OCV_SHIFT)) & FTM_SWOCTRL_CH6OCV_MASK)
#define FTM_SWOCTRL_CH7OCV_MASK                  (0x8000U)
#define FTM_SWOCTRL_CH7OCV_SHIFT                 (15U)
#define FTM_SWOCTRL_CH7OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OCV_SHIFT)) & FTM_SWOCTRL_CH7OCV_MASK)

/*! @name PWMLOAD - FTM PWM Load */
#define FTM_PWMLOAD_CH0SEL_MASK                  (0x1U)
#define FTM_PWMLOAD_CH0SEL_SHIFT                 (0U)
#define FTM_PWMLOAD_CH0SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH0SEL_SHIFT)) & FTM_PWMLOAD_CH0SEL_MASK)
#define FTM_PWMLOAD_CH1SEL_MASK                  (0x2U)
#define FTM_PWMLOAD_CH1SEL_SHIFT                 (1U)
#define FTM_PWMLOAD_CH1SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH1SEL_SHIFT)) & FTM_PWMLOAD_CH1SEL_MASK)
#define FTM_PWMLOAD_CH2SEL_MASK                  (0x4U)
#define FTM_PWMLOAD_CH2SEL_SHIFT                 (2U)
#define FTM_PWMLOAD_CH2SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH2SEL_SHIFT)) & FTM_PWMLOAD_CH2SEL_MASK)
#define FTM_PWMLOAD_CH3SEL_MASK                  (0x8U)
#define FTM_PWMLOAD_CH3SEL_SHIFT                 (3U)
#define FTM_PWMLOAD_CH3SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH3SEL_SHIFT)) & FTM_PWMLOAD_CH3SEL_MASK)
#define FTM_PWMLOAD_CH4SEL_MASK                  (0x10U)
#define FTM_PWMLOAD_CH4SEL_SHIFT                 (4U)
#define FTM_PWMLOAD_CH4SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH4SEL_SHIFT)) & FTM_PWMLOAD_CH4SEL_MASK)
#define FTM_PWMLOAD_CH5SEL_MASK                  (0x20U)
#define FTM_PWMLOAD_CH5SEL_SHIFT                 (5U)
#define FTM_PWMLOAD_CH5SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH5SEL_SHIFT)) & FTM_PWMLOAD_CH5SEL_MASK)
#define FTM_PWMLOAD_CH6SEL_MASK                  (0x40U)
#define FTM_PWMLOAD_CH6SEL_SHIFT                 (6U)
#define FTM_PWMLOAD_CH6SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH6SEL_SHIFT)) & FTM_PWMLOAD_CH6SEL_MASK)
#define FTM_PWMLOAD_CH7SEL_MASK                  (0x80U)
#define FTM_PWMLOAD_CH7SEL_SHIFT                 (7U)
#define FTM_PWMLOAD_CH7SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH7SEL_SHIFT)) & FTM_PWMLOAD_CH7SEL_MASK)
#define FTM_PWMLOAD_LDOK_MASK                    (0x200U)
#define FTM_PWMLOAD_LDOK_SHIFT                   (9U)
#define FTM_PWMLOAD_LDOK(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_LDOK_SHIFT)) & FTM_PWMLOAD_LDOK_MASK)


/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base address */
#define FTM0_BASE                                (0x40038000u)
/** Peripheral FTM0 base pointer */
#define FTM0                                     ((FTM_Type *)FTM0_BASE)
/** Peripheral FTM1 base address */
#define FTM1_BASE                                (0x40039000u)
/** Peripheral FTM1 base pointer */
#define FTM1                                     ((FTM_Type *)FTM1_BASE)
/** Peripheral FTM2 base address */
#define FTM2_BASE                                (0x4003A000u)
/** Peripheral FTM2 base pointer */
#define FTM2                                     ((FTM_Type *)FTM2_BASE)
/** Peripheral FTM3 base address */
#define FTM3_BASE                                (0x400B9000u)
/** Peripheral FTM3 base pointer */
#define FTM3                                     ((FTM_Type *)FTM3_BASE)
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM2_BASE, FTM3_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM2, FTM3 }
/** Interrupt vectors for the FTM peripheral type */
#define FTM_IRQS                                 { FTM0_IRQn, FTM1_IRQn, FTM2_IRQn, FTM3_IRQn }

/*!
 * @}
 */ /* end of group FTM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name PDOR - Port Data Output Register */
#define GPIO_PDOR_PDO_MASK                       (0xFFFFFFFFU)
#define GPIO_PDOR_PDO_SHIFT                      (0U)
#define GPIO_PDOR_PDO(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO_SHIFT)) & GPIO_PDOR_PDO_MASK)

/*! @name PSOR - Port Set Output Register */
#define GPIO_PSOR_PTSO_MASK                      (0xFFFFFFFFU)
#define GPIO_PSOR_PTSO_SHIFT                     (0U)
#define GPIO_PSOR_PTSO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO_SHIFT)) & GPIO_PSOR_PTSO_MASK)

/*! @name PCOR - Port Clear Output Register */
#define GPIO_PCOR_PTCO_MASK                      (0xFFFFFFFFU)
#define GPIO_PCOR_PTCO_SHIFT                     (0U)
#define GPIO_PCOR_PTCO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO_SHIFT)) & GPIO_PCOR_PTCO_MASK)

/*! @name PTOR - Port Toggle Output Register */
#define GPIO_PTOR_PTTO_MASK                      (0xFFFFFFFFU)
#define GPIO_PTOR_PTTO_SHIFT                     (0U)
#define GPIO_PTOR_PTTO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO_SHIFT)) & GPIO_PTOR_PTTO_MASK)

/*! @name PDIR - Port Data Input Register */
#define GPIO_PDIR_PDI_MASK                       (0xFFFFFFFFU)
#define GPIO_PDIR_PDI_SHIFT                      (0U)
#define GPIO_PDIR_PDI(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI_SHIFT)) & GPIO_PDIR_PDI_MASK)

/*! @name PDDR - Port Data Direction Register */
#define GPIO_PDDR_PDD_MASK                       (0xFFFFFFFFU)
#define GPIO_PDDR_PDD_SHIFT                      (0U)
#define GPIO_PDDR_PDD(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD_SHIFT)) & GPIO_PDDR_PDD_MASK)


/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIOA base address */
#define GPIOA_BASE                               (0x400FF000u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0x400FF040u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0x400FF080u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0x400FF0C0u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0x400FF100u)
/** Peripheral GPIOE base pointer */
#define GPIOE                                    ((GPIO_Type *)GPIOE_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE }

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct {
  __IO uint8_t A1;                                 /**< I2C Address Register 1, offset: 0x0 */
  __IO uint8_t F;                                  /**< I2C Frequency Divider register, offset: 0x1 */
  __IO uint8_t C1;                                 /**< I2C Control Register 1, offset: 0x2 */
  __IO uint8_t S;                                  /**< I2C Status register, offset: 0x3 */
  __IO uint8_t D;                                  /**< I2C Data I/O register, offset: 0x4 */
  __IO uint8_t C2;                                 /**< I2C Control Register 2, offset: 0x5 */
  __IO uint8_t FLT;                                /**< I2C Programmable Input Glitch Filter Register, offset: 0x6 */
  __IO uint8_t RA;                                 /**< I2C Range Address register, offset: 0x7 */
  __IO uint8_t SMB;                                /**< I2C SMBus Control and Status register, offset: 0x8 */
  __IO uint8_t A2;                                 /**< I2C Address Register 2, offset: 0x9 */
  __IO uint8_t SLTH;                               /**< I2C SCL Low Timeout Register High, offset: 0xA */
  __IO uint8_t SLTL;                               /**< I2C SCL Low Timeout Register Low, offset: 0xB */
  __IO uint8_t S2;                                 /**< I2C Status register 2, offset: 0xC */
} I2C_Type;

/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/*! @name A1 - I2C Address Register 1 */
#define I2C_A1_AD_MASK                           (0xFEU)
#define I2C_A1_AD_SHIFT                          (1U)
#define I2C_A1_AD(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_A1_AD_SHIFT)) & I2C_A1_AD_MASK)

/*! @name F - I2C Frequency Divider register */
#define I2C_F_ICR_MASK                           (0x3FU)
#define I2C_F_ICR_SHIFT                          (0U)
#define I2C_F_ICR(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_F_ICR_SHIFT)) & I2C_F_ICR_MASK)
#define I2C_F_MULT_MASK                          (0xC0U)
#define I2C_F_MULT_SHIFT                         (6U)
#define I2C_F_MULT(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_F_MULT_SHIFT)) & I2C_F_MULT_MASK)

/*! @name C1 - I2C Control Register 1 */
#define I2C_C1_DMAEN_MASK                        (0x1U)
#define I2C_C1_DMAEN_SHIFT                       (0U)
#define I2C_C1_DMAEN(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_C1_DMAEN_SHIFT)) & I2C_C1_DMAEN_MASK)
#define I2C_C1_WUEN_MASK                         (0x2U)
#define I2C_C1_WUEN_SHIFT                        (1U)
#define I2C_C1_WUEN(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C1_WUEN_SHIFT)) & I2C_C1_WUEN_MASK)
#define I2C_C1_RSTA_MASK                         (0x4U)
#define I2C_C1_RSTA_SHIFT                        (2U)
#define I2C_C1_RSTA(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C1_RSTA_SHIFT)) & I2C_C1_RSTA_MASK)
#define I2C_C1_TXAK_MASK                         (0x8U)
#define I2C_C1_TXAK_SHIFT                        (3U)
#define I2C_C1_TXAK(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C1_TXAK_SHIFT)) & I2C_C1_TXAK_MASK)
#define I2C_C1_TX_MASK                           (0x10U)
#define I2C_C1_TX_SHIFT                          (4U)
#define I2C_C1_TX(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_C1_TX_SHIFT)) & I2C_C1_TX_MASK)
#define I2C_C1_MST_MASK                          (0x20U)
#define I2C_C1_MST_SHIFT                         (5U)
#define I2C_C1_MST(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_C1_MST_SHIFT)) & I2C_C1_MST_MASK)
#define I2C_C1_IICIE_MASK                        (0x40U)
#define I2C_C1_IICIE_SHIFT                       (6U)
#define I2C_C1_IICIE(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_C1_IICIE_SHIFT)) & I2C_C1_IICIE_MASK)
#define I2C_C1_IICEN_MASK                        (0x80U)
#define I2C_C1_IICEN_SHIFT                       (7U)
#define I2C_C1_IICEN(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_C1_IICEN_SHIFT)) & I2C_C1_IICEN_MASK)

/*! @name S - I2C Status register */
#define I2C_S_RXAK_MASK                          (0x1U)
#define I2C_S_RXAK_SHIFT                         (0U)
#define I2C_S_RXAK(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_S_RXAK_SHIFT)) & I2C_S_RXAK_MASK)
#define I2C_S_IICIF_MASK                         (0x2U)
#define I2C_S_IICIF_SHIFT                        (1U)
#define I2C_S_IICIF(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_S_IICIF_SHIFT)) & I2C_S_IICIF_MASK)
#define I2C_S_SRW_MASK                           (0x4U)
#define I2C_S_SRW_SHIFT                          (2U)
#define I2C_S_SRW(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_S_SRW_SHIFT)) & I2C_S_SRW_MASK)
#define I2C_S_RAM_MASK                           (0x8U)
#define I2C_S_RAM_SHIFT                          (3U)
#define I2C_S_RAM(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_S_RAM_SHIFT)) & I2C_S_RAM_MASK)
#define I2C_S_ARBL_MASK                          (0x10U)
#define I2C_S_ARBL_SHIFT                         (4U)
#define I2C_S_ARBL(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_S_ARBL_SHIFT)) & I2C_S_ARBL_MASK)
#define I2C_S_BUSY_MASK                          (0x20U)
#define I2C_S_BUSY_SHIFT                         (5U)
#define I2C_S_BUSY(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_S_BUSY_SHIFT)) & I2C_S_BUSY_MASK)
#define I2C_S_IAAS_MASK                          (0x40U)
#define I2C_S_IAAS_SHIFT                         (6U)
#define I2C_S_IAAS(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_S_IAAS_SHIFT)) & I2C_S_IAAS_MASK)
#define I2C_S_TCF_MASK                           (0x80U)
#define I2C_S_TCF_SHIFT                          (7U)
#define I2C_S_TCF(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_S_TCF_SHIFT)) & I2C_S_TCF_MASK)

/*! @name D - I2C Data I/O register */
#define I2C_D_DATA_MASK                          (0xFFU)
#define I2C_D_DATA_SHIFT                         (0U)
#define I2C_D_DATA(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_D_DATA_SHIFT)) & I2C_D_DATA_MASK)

/*! @name C2 - I2C Control Register 2 */
#define I2C_C2_AD_MASK                           (0x7U)
#define I2C_C2_AD_SHIFT                          (0U)
#define I2C_C2_AD(x)                             (((uint8_t)(((uint8_t)(x)) << I2C_C2_AD_SHIFT)) & I2C_C2_AD_MASK)
#define I2C_C2_RMEN_MASK                         (0x8U)
#define I2C_C2_RMEN_SHIFT                        (3U)
#define I2C_C2_RMEN(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C2_RMEN_SHIFT)) & I2C_C2_RMEN_MASK)
#define I2C_C2_SBRC_MASK                         (0x10U)
#define I2C_C2_SBRC_SHIFT                        (4U)
#define I2C_C2_SBRC(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C2_SBRC_SHIFT)) & I2C_C2_SBRC_MASK)
#define I2C_C2_HDRS_MASK                         (0x20U)
#define I2C_C2_HDRS_SHIFT                        (5U)
#define I2C_C2_HDRS(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_C2_HDRS_SHIFT)) & I2C_C2_HDRS_MASK)
#define I2C_C2_ADEXT_MASK                        (0x40U)
#define I2C_C2_ADEXT_SHIFT                       (6U)
#define I2C_C2_ADEXT(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_C2_ADEXT_SHIFT)) & I2C_C2_ADEXT_MASK)
#define I2C_C2_GCAEN_MASK                        (0x80U)
#define I2C_C2_GCAEN_SHIFT                       (7U)
#define I2C_C2_GCAEN(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_C2_GCAEN_SHIFT)) & I2C_C2_GCAEN_MASK)

/*! @name FLT - I2C Programmable Input Glitch Filter Register */
#define I2C_FLT_FLT_MASK                         (0xFU)
#define I2C_FLT_FLT_SHIFT                        (0U)
#define I2C_FLT_FLT(x)                           (((uint8_t)(((uint8_t)(x)) << I2C_FLT_FLT_SHIFT)) & I2C_FLT_FLT_MASK)
#define I2C_FLT_STARTF_MASK                      (0x10U)
#define I2C_FLT_STARTF_SHIFT                     (4U)
#define I2C_FLT_STARTF(x)                        (((uint8_t)(((uint8_t)(x)) << I2C_FLT_STARTF_SHIFT)) & I2C_FLT_STARTF_MASK)
#define I2C_FLT_SSIE_MASK                        (0x20U)
#define I2C_FLT_SSIE_SHIFT                       (5U)
#define I2C_FLT_SSIE(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_FLT_SSIE_SHIFT)) & I2C_FLT_SSIE_MASK)
#define I2C_FLT_STOPF_MASK                       (0x40U)
#define I2C_FLT_STOPF_SHIFT                      (6U)
#define I2C_FLT_STOPF(x)                         (((uint8_t)(((uint8_t)(x)) << I2C_FLT_STOPF_SHIFT)) & I2C_FLT_STOPF_MASK)
#define I2C_FLT_SHEN_MASK                        (0x80U)
#define I2C_FLT_SHEN_SHIFT                       (7U)
#define I2C_FLT_SHEN(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_FLT_SHEN_SHIFT)) & I2C_FLT_SHEN_MASK)

/*! @name RA - I2C Range Address register */
#define I2C_RA_RAD_MASK                          (0xFEU)
#define I2C_RA_RAD_SHIFT                         (1U)
#define I2C_RA_RAD(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_RA_RAD_SHIFT)) & I2C_RA_RAD_MASK)

/*! @name SMB - I2C SMBus Control and Status register */
#define I2C_SMB_SHTF2IE_MASK                     (0x1U)
#define I2C_SMB_SHTF2IE_SHIFT                    (0U)
#define I2C_SMB_SHTF2IE(x)                       (((uint8_t)(((uint8_t)(x)) << I2C_SMB_SHTF2IE_SHIFT)) & I2C_SMB_SHTF2IE_MASK)
#define I2C_SMB_SHTF2_MASK                       (0x2U)
#define I2C_SMB_SHTF2_SHIFT                      (1U)
#define I2C_SMB_SHTF2(x)                         (((uint8_t)(((uint8_t)(x)) << I2C_SMB_SHTF2_SHIFT)) & I2C_SMB_SHTF2_MASK)
#define I2C_SMB_SHTF1_MASK                       (0x4U)
#define I2C_SMB_SHTF1_SHIFT                      (2U)
#define I2C_SMB_SHTF1(x)                         (((uint8_t)(((uint8_t)(x)) << I2C_SMB_SHTF1_SHIFT)) & I2C_SMB_SHTF1_MASK)
#define I2C_SMB_SLTF_MASK                        (0x8U)
#define I2C_SMB_SLTF_SHIFT                       (3U)
#define I2C_SMB_SLTF(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_SMB_SLTF_SHIFT)) & I2C_SMB_SLTF_MASK)
#define I2C_SMB_TCKSEL_MASK                      (0x10U)
#define I2C_SMB_TCKSEL_SHIFT                     (4U)
#define I2C_SMB_TCKSEL(x)                        (((uint8_t)(((uint8_t)(x)) << I2C_SMB_TCKSEL_SHIFT)) & I2C_SMB_TCKSEL_MASK)
#define I2C_SMB_SIICAEN_MASK                     (0x20U)
#define I2C_SMB_SIICAEN_SHIFT                    (5U)
#define I2C_SMB_SIICAEN(x)                       (((uint8_t)(((uint8_t)(x)) << I2C_SMB_SIICAEN_SHIFT)) & I2C_SMB_SIICAEN_MASK)
#define I2C_SMB_ALERTEN_MASK                     (0x40U)
#define I2C_SMB_ALERTEN_SHIFT                    (6U)
#define I2C_SMB_ALERTEN(x)                       (((uint8_t)(((uint8_t)(x)) << I2C_SMB_ALERTEN_SHIFT)) & I2C_SMB_ALERTEN_MASK)
#define I2C_SMB_FACK_MASK                        (0x80U)
#define I2C_SMB_FACK_SHIFT                       (7U)
#define I2C_SMB_FACK(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_SMB_FACK_SHIFT)) & I2C_SMB_FACK_MASK)

/*! @name A2 - I2C Address Register 2 */
#define I2C_A2_SAD_MASK                          (0xFEU)
#define I2C_A2_SAD_SHIFT                         (1U)
#define I2C_A2_SAD(x)                            (((uint8_t)(((uint8_t)(x)) << I2C_A2_SAD_SHIFT)) & I2C_A2_SAD_MASK)

/*! @name SLTH - I2C SCL Low Timeout Register High */
#define I2C_SLTH_SSLT_MASK                       (0xFFU)
#define I2C_SLTH_SSLT_SHIFT                      (0U)
#define I2C_SLTH_SSLT(x)                         (((uint8_t)(((uint8_t)(x)) << I2C_SLTH_SSLT_SHIFT)) & I2C_SLTH_SSLT_MASK)

/*! @name SLTL - I2C SCL Low Timeout Register Low */
#define I2C_SLTL_SSLT_MASK                       (0xFFU)
#define I2C_SLTL_SSLT_SHIFT                      (0U)
#define I2C_SLTL_SSLT(x)                         (((uint8_t)(((uint8_t)(x)) << I2C_SLTL_SSLT_SHIFT)) & I2C_SLTL_SSLT_MASK)

/*! @name S2 - I2C Status register 2 */
#define I2C_S2_EMPTY_MASK                        (0x1U)
#define I2C_S2_EMPTY_SHIFT                       (0U)
#define I2C_S2_EMPTY(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_S2_EMPTY_SHIFT)) & I2C_S2_EMPTY_MASK)
#define I2C_S2_ERROR_MASK                        (0x2U)
#define I2C_S2_ERROR_SHIFT                       (1U)
#define I2C_S2_ERROR(x)                          (((uint8_t)(((uint8_t)(x)) << I2C_S2_ERROR_SHIFT)) & I2C_S2_ERROR_MASK)


/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE                                (0x40066000u)
/** Peripheral I2C0 base pointer */
#define I2C0                                     ((I2C_Type *)I2C0_BASE)
/** Peripheral I2C1 base address */
#define I2C1_BASE                                (0x40067000u)
/** Peripheral I2C1 base pointer */
#define I2C1                                     ((I2C_Type *)I2C1_BASE)
/** Peripheral I2C2 base address */
#define I2C2_BASE                                (0x400E6000u)
/** Peripheral I2C2 base pointer */
#define I2C2                                     ((I2C_Type *)I2C2_BASE)
/** Peripheral I2C3 base address */
#define I2C3_BASE                                (0x400E7000u)
/** Peripheral I2C3 base pointer */
#define I2C3                                     ((I2C_Type *)I2C3_BASE)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0, I2C1, I2C2, I2C3 }
/** Interrupt vectors for the I2C peripheral type */
#define I2C_IRQS                                 { I2C0_IRQn, I2C1_IRQn, I2C2_IRQn, I2C3_IRQn }

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2S Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Peripheral_Access_Layer I2S Peripheral Access Layer
 * @{
 */

/** I2S - Register Layout Typedef */
typedef struct {
  __IO uint32_t TCSR;                              /**< SAI Transmit Control Register, offset: 0x0 */
  __IO uint32_t TCR1;                              /**< SAI Transmit Configuration 1 Register, offset: 0x4 */
  __IO uint32_t TCR2;                              /**< SAI Transmit Configuration 2 Register, offset: 0x8 */
  __IO uint32_t TCR3;                              /**< SAI Transmit Configuration 3 Register, offset: 0xC */
  __IO uint32_t TCR4;                              /**< SAI Transmit Configuration 4 Register, offset: 0x10 */
  __IO uint32_t TCR5;                              /**< SAI Transmit Configuration 5 Register, offset: 0x14 */
       uint8_t RESERVED_0[8];
  __O  uint32_t TDR[2];                            /**< SAI Transmit Data Register, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[24];
  __I  uint32_t TFR[2];                            /**< SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4 */
       uint8_t RESERVED_2[24];
  __IO uint32_t TMR;                               /**< SAI Transmit Mask Register, offset: 0x60 */
       uint8_t RESERVED_3[28];
  __IO uint32_t RCSR;                              /**< SAI Receive Control Register, offset: 0x80 */
  __IO uint32_t RCR1;                              /**< SAI Receive Configuration 1 Register, offset: 0x84 */
  __IO uint32_t RCR2;                              /**< SAI Receive Configuration 2 Register, offset: 0x88 */
  __IO uint32_t RCR3;                              /**< SAI Receive Configuration 3 Register, offset: 0x8C */
  __IO uint32_t RCR4;                              /**< SAI Receive Configuration 4 Register, offset: 0x90 */
  __IO uint32_t RCR5;                              /**< SAI Receive Configuration 5 Register, offset: 0x94 */
       uint8_t RESERVED_4[8];
  __I  uint32_t RDR[2];                            /**< SAI Receive Data Register, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_5[24];
  __I  uint32_t RFR[2];                            /**< SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_6[24];
  __IO uint32_t RMR;                               /**< SAI Receive Mask Register, offset: 0xE0 */
       uint8_t RESERVED_7[28];
  __IO uint32_t MCR;                               /**< SAI MCLK Control Register, offset: 0x100 */
  __IO uint32_t MDR;                               /**< SAI MCLK Divide Register, offset: 0x104 */
} I2S_Type;

/* ----------------------------------------------------------------------------
   -- I2S Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Masks I2S Register Masks
 * @{
 */

/*! @name TCSR - SAI Transmit Control Register */
#define I2S_TCSR_FRDE_MASK                       (0x1U)
#define I2S_TCSR_FRDE_SHIFT                      (0U)
#define I2S_TCSR_FRDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRDE_SHIFT)) & I2S_TCSR_FRDE_MASK)
#define I2S_TCSR_FWDE_MASK                       (0x2U)
#define I2S_TCSR_FWDE_SHIFT                      (1U)
#define I2S_TCSR_FWDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWDE_SHIFT)) & I2S_TCSR_FWDE_MASK)
#define I2S_TCSR_FRIE_MASK                       (0x100U)
#define I2S_TCSR_FRIE_SHIFT                      (8U)
#define I2S_TCSR_FRIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRIE_SHIFT)) & I2S_TCSR_FRIE_MASK)
#define I2S_TCSR_FWIE_MASK                       (0x200U)
#define I2S_TCSR_FWIE_SHIFT                      (9U)
#define I2S_TCSR_FWIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWIE_SHIFT)) & I2S_TCSR_FWIE_MASK)
#define I2S_TCSR_FEIE_MASK                       (0x400U)
#define I2S_TCSR_FEIE_SHIFT                      (10U)
#define I2S_TCSR_FEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FEIE_SHIFT)) & I2S_TCSR_FEIE_MASK)
#define I2S_TCSR_SEIE_MASK                       (0x800U)
#define I2S_TCSR_SEIE_SHIFT                      (11U)
#define I2S_TCSR_SEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SEIE_SHIFT)) & I2S_TCSR_SEIE_MASK)
#define I2S_TCSR_WSIE_MASK                       (0x1000U)
#define I2S_TCSR_WSIE_SHIFT                      (12U)
#define I2S_TCSR_WSIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_WSIE_SHIFT)) & I2S_TCSR_WSIE_MASK)
#define I2S_TCSR_FRF_MASK                        (0x10000U)
#define I2S_TCSR_FRF_SHIFT                       (16U)
#define I2S_TCSR_FRF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRF_SHIFT)) & I2S_TCSR_FRF_MASK)
#define I2S_TCSR_FWF_MASK                        (0x20000U)
#define I2S_TCSR_FWF_SHIFT                       (17U)
#define I2S_TCSR_FWF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWF_SHIFT)) & I2S_TCSR_FWF_MASK)
#define I2S_TCSR_FEF_MASK                        (0x40000U)
#define I2S_TCSR_FEF_SHIFT                       (18U)
#define I2S_TCSR_FEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FEF_SHIFT)) & I2S_TCSR_FEF_MASK)
#define I2S_TCSR_SEF_MASK                        (0x80000U)
#define I2S_TCSR_SEF_SHIFT                       (19U)
#define I2S_TCSR_SEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SEF_SHIFT)) & I2S_TCSR_SEF_MASK)
#define I2S_TCSR_WSF_MASK                        (0x100000U)
#define I2S_TCSR_WSF_SHIFT                       (20U)
#define I2S_TCSR_WSF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_WSF_SHIFT)) & I2S_TCSR_WSF_MASK)
#define I2S_TCSR_SR_MASK                         (0x1000000U)
#define I2S_TCSR_SR_SHIFT                        (24U)
#define I2S_TCSR_SR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SR_SHIFT)) & I2S_TCSR_SR_MASK)
#define I2S_TCSR_FR_MASK                         (0x2000000U)
#define I2S_TCSR_FR_SHIFT                        (25U)
#define I2S_TCSR_FR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FR_SHIFT)) & I2S_TCSR_FR_MASK)
#define I2S_TCSR_BCE_MASK                        (0x10000000U)
#define I2S_TCSR_BCE_SHIFT                       (28U)
#define I2S_TCSR_BCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_BCE_SHIFT)) & I2S_TCSR_BCE_MASK)
#define I2S_TCSR_DBGE_MASK                       (0x20000000U)
#define I2S_TCSR_DBGE_SHIFT                      (29U)
#define I2S_TCSR_DBGE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_DBGE_SHIFT)) & I2S_TCSR_DBGE_MASK)
#define I2S_TCSR_STOPE_MASK                      (0x40000000U)
#define I2S_TCSR_STOPE_SHIFT                     (30U)
#define I2S_TCSR_STOPE(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_STOPE_SHIFT)) & I2S_TCSR_STOPE_MASK)
#define I2S_TCSR_TE_MASK                         (0x80000000U)
#define I2S_TCSR_TE_SHIFT                        (31U)
#define I2S_TCSR_TE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_TE_SHIFT)) & I2S_TCSR_TE_MASK)

/*! @name TCR1 - SAI Transmit Configuration 1 Register */
#define I2S_TCR1_TFW_MASK                        (0x7U)
#define I2S_TCR1_TFW_SHIFT                       (0U)
#define I2S_TCR1_TFW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR1_TFW_SHIFT)) & I2S_TCR1_TFW_MASK)

/*! @name TCR2 - SAI Transmit Configuration 2 Register */
#define I2S_TCR2_DIV_MASK                        (0xFFU)
#define I2S_TCR2_DIV_SHIFT                       (0U)
#define I2S_TCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_DIV_SHIFT)) & I2S_TCR2_DIV_MASK)
#define I2S_TCR2_BCD_MASK                        (0x1000000U)
#define I2S_TCR2_BCD_SHIFT                       (24U)
#define I2S_TCR2_BCD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCD_SHIFT)) & I2S_TCR2_BCD_MASK)
#define I2S_TCR2_BCP_MASK                        (0x2000000U)
#define I2S_TCR2_BCP_SHIFT                       (25U)
#define I2S_TCR2_BCP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCP_SHIFT)) & I2S_TCR2_BCP_MASK)
#define I2S_TCR2_MSEL_MASK                       (0xC000000U)
#define I2S_TCR2_MSEL_SHIFT                      (26U)
#define I2S_TCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_MSEL_SHIFT)) & I2S_TCR2_MSEL_MASK)
#define I2S_TCR2_BCI_MASK                        (0x10000000U)
#define I2S_TCR2_BCI_SHIFT                       (28U)
#define I2S_TCR2_BCI(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCI_SHIFT)) & I2S_TCR2_BCI_MASK)
#define I2S_TCR2_BCS_MASK                        (0x20000000U)
#define I2S_TCR2_BCS_SHIFT                       (29U)
#define I2S_TCR2_BCS(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCS_SHIFT)) & I2S_TCR2_BCS_MASK)
#define I2S_TCR2_SYNC_MASK                       (0xC0000000U)
#define I2S_TCR2_SYNC_SHIFT                      (30U)
#define I2S_TCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_SYNC_SHIFT)) & I2S_TCR2_SYNC_MASK)

/*! @name TCR3 - SAI Transmit Configuration 3 Register */
#define I2S_TCR3_WDFL_MASK                       (0x1FU)
#define I2S_TCR3_WDFL_SHIFT                      (0U)
#define I2S_TCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_WDFL_SHIFT)) & I2S_TCR3_WDFL_MASK)
#define I2S_TCR3_TCE_MASK                        (0x30000U)
#define I2S_TCR3_TCE_SHIFT                       (16U)
#define I2S_TCR3_TCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_TCE_SHIFT)) & I2S_TCR3_TCE_MASK)
#define I2S_TCR3_CFR_MASK                        (0x3000000U)
#define I2S_TCR3_CFR_SHIFT                       (24U)
#define I2S_TCR3_CFR(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_CFR_SHIFT)) & I2S_TCR3_CFR_MASK)

/*! @name TCR4 - SAI Transmit Configuration 4 Register */
#define I2S_TCR4_FSD_MASK                        (0x1U)
#define I2S_TCR4_FSD_SHIFT                       (0U)
#define I2S_TCR4_FSD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSD_SHIFT)) & I2S_TCR4_FSD_MASK)
#define I2S_TCR4_FSP_MASK                        (0x2U)
#define I2S_TCR4_FSP_SHIFT                       (1U)
#define I2S_TCR4_FSP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSP_SHIFT)) & I2S_TCR4_FSP_MASK)
#define I2S_TCR4_ONDEM_MASK                      (0x4U)
#define I2S_TCR4_ONDEM_SHIFT                     (2U)
#define I2S_TCR4_ONDEM(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_ONDEM_SHIFT)) & I2S_TCR4_ONDEM_MASK)
#define I2S_TCR4_FSE_MASK                        (0x8U)
#define I2S_TCR4_FSE_SHIFT                       (3U)
#define I2S_TCR4_FSE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSE_SHIFT)) & I2S_TCR4_FSE_MASK)
#define I2S_TCR4_MF_MASK                         (0x10U)
#define I2S_TCR4_MF_SHIFT                        (4U)
#define I2S_TCR4_MF(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_MF_SHIFT)) & I2S_TCR4_MF_MASK)
#define I2S_TCR4_SYWD_MASK                       (0x1F00U)
#define I2S_TCR4_SYWD_SHIFT                      (8U)
#define I2S_TCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_SYWD_SHIFT)) & I2S_TCR4_SYWD_MASK)
#define I2S_TCR4_FRSZ_MASK                       (0x1F0000U)
#define I2S_TCR4_FRSZ_SHIFT                      (16U)
#define I2S_TCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FRSZ_SHIFT)) & I2S_TCR4_FRSZ_MASK)
#define I2S_TCR4_FPACK_MASK                      (0x3000000U)
#define I2S_TCR4_FPACK_SHIFT                     (24U)
#define I2S_TCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FPACK_SHIFT)) & I2S_TCR4_FPACK_MASK)
#define I2S_TCR4_FCOMB_MASK                      (0xC000000U)
#define I2S_TCR4_FCOMB_SHIFT                     (26U)
#define I2S_TCR4_FCOMB(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FCOMB_SHIFT)) & I2S_TCR4_FCOMB_MASK)
#define I2S_TCR4_FCONT_MASK                      (0x10000000U)
#define I2S_TCR4_FCONT_SHIFT                     (28U)
#define I2S_TCR4_FCONT(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FCONT_SHIFT)) & I2S_TCR4_FCONT_MASK)

/*! @name TCR5 - SAI Transmit Configuration 5 Register */
#define I2S_TCR5_FBT_MASK                        (0x1F00U)
#define I2S_TCR5_FBT_SHIFT                       (8U)
#define I2S_TCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_FBT_SHIFT)) & I2S_TCR5_FBT_MASK)
#define I2S_TCR5_W0W_MASK                        (0x1F0000U)
#define I2S_TCR5_W0W_SHIFT                       (16U)
#define I2S_TCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_W0W_SHIFT)) & I2S_TCR5_W0W_MASK)
#define I2S_TCR5_WNW_MASK                        (0x1F000000U)
#define I2S_TCR5_WNW_SHIFT                       (24U)
#define I2S_TCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_WNW_SHIFT)) & I2S_TCR5_WNW_MASK)

/*! @name TDR - SAI Transmit Data Register */
#define I2S_TDR_TDR_MASK                         (0xFFFFFFFFU)
#define I2S_TDR_TDR_SHIFT                        (0U)
#define I2S_TDR_TDR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TDR_TDR_SHIFT)) & I2S_TDR_TDR_MASK)

/* The count of I2S_TDR */
#define I2S_TDR_COUNT                            (2U)

/*! @name TFR - SAI Transmit FIFO Register */
#define I2S_TFR_RFP_MASK                         (0xFU)
#define I2S_TFR_RFP_SHIFT                        (0U)
#define I2S_TFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_RFP_SHIFT)) & I2S_TFR_RFP_MASK)
#define I2S_TFR_WFP_MASK                         (0xF0000U)
#define I2S_TFR_WFP_SHIFT                        (16U)
#define I2S_TFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_WFP_SHIFT)) & I2S_TFR_WFP_MASK)
#define I2S_TFR_WCP_MASK                         (0x80000000U)
#define I2S_TFR_WCP_SHIFT                        (31U)
#define I2S_TFR_WCP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_WCP_SHIFT)) & I2S_TFR_WCP_MASK)

/* The count of I2S_TFR */
#define I2S_TFR_COUNT                            (2U)

/*! @name TMR - SAI Transmit Mask Register */
#define I2S_TMR_TWM_MASK                         (0xFFFFFFFFU)
#define I2S_TMR_TWM_SHIFT                        (0U)
#define I2S_TMR_TWM(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TMR_TWM_SHIFT)) & I2S_TMR_TWM_MASK)

/*! @name RCSR - SAI Receive Control Register */
#define I2S_RCSR_FRDE_MASK                       (0x1U)
#define I2S_RCSR_FRDE_SHIFT                      (0U)
#define I2S_RCSR_FRDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRDE_SHIFT)) & I2S_RCSR_FRDE_MASK)
#define I2S_RCSR_FWDE_MASK                       (0x2U)
#define I2S_RCSR_FWDE_SHIFT                      (1U)
#define I2S_RCSR_FWDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWDE_SHIFT)) & I2S_RCSR_FWDE_MASK)
#define I2S_RCSR_FRIE_MASK                       (0x100U)
#define I2S_RCSR_FRIE_SHIFT                      (8U)
#define I2S_RCSR_FRIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRIE_SHIFT)) & I2S_RCSR_FRIE_MASK)
#define I2S_RCSR_FWIE_MASK                       (0x200U)
#define I2S_RCSR_FWIE_SHIFT                      (9U)
#define I2S_RCSR_FWIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWIE_SHIFT)) & I2S_RCSR_FWIE_MASK)
#define I2S_RCSR_FEIE_MASK                       (0x400U)
#define I2S_RCSR_FEIE_SHIFT                      (10U)
#define I2S_RCSR_FEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FEIE_SHIFT)) & I2S_RCSR_FEIE_MASK)
#define I2S_RCSR_SEIE_MASK                       (0x800U)
#define I2S_RCSR_SEIE_SHIFT                      (11U)
#define I2S_RCSR_SEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SEIE_SHIFT)) & I2S_RCSR_SEIE_MASK)
#define I2S_RCSR_WSIE_MASK                       (0x1000U)
#define I2S_RCSR_WSIE_SHIFT                      (12U)
#define I2S_RCSR_WSIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_WSIE_SHIFT)) & I2S_RCSR_WSIE_MASK)
#define I2S_RCSR_FRF_MASK                        (0x10000U)
#define I2S_RCSR_FRF_SHIFT                       (16U)
#define I2S_RCSR_FRF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRF_SHIFT)) & I2S_RCSR_FRF_MASK)
#define I2S_RCSR_FWF_MASK                        (0x20000U)
#define I2S_RCSR_FWF_SHIFT                       (17U)
#define I2S_RCSR_FWF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWF_SHIFT)) & I2S_RCSR_FWF_MASK)
#define I2S_RCSR_FEF_MASK                        (0x40000U)
#define I2S_RCSR_FEF_SHIFT                       (18U)
#define I2S_RCSR_FEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FEF_SHIFT)) & I2S_RCSR_FEF_MASK)
#define I2S_RCSR_SEF_MASK                        (0x80000U)
#define I2S_RCSR_SEF_SHIFT                       (19U)
#define I2S_RCSR_SEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SEF_SHIFT)) & I2S_RCSR_SEF_MASK)
#define I2S_RCSR_WSF_MASK                        (0x100000U)
#define I2S_RCSR_WSF_SHIFT                       (20U)
#define I2S_RCSR_WSF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_WSF_SHIFT)) & I2S_RCSR_WSF_MASK)
#define I2S_RCSR_SR_MASK                         (0x1000000U)
#define I2S_RCSR_SR_SHIFT                        (24U)
#define I2S_RCSR_SR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SR_SHIFT)) & I2S_RCSR_SR_MASK)
#define I2S_RCSR_FR_MASK                         (0x2000000U)
#define I2S_RCSR_FR_SHIFT                        (25U)
#define I2S_RCSR_FR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FR_SHIFT)) & I2S_RCSR_FR_MASK)
#define I2S_RCSR_BCE_MASK                        (0x10000000U)
#define I2S_RCSR_BCE_SHIFT                       (28U)
#define I2S_RCSR_BCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_BCE_SHIFT)) & I2S_RCSR_BCE_MASK)
#define I2S_RCSR_DBGE_MASK                       (0x20000000U)
#define I2S_RCSR_DBGE_SHIFT                      (29U)
#define I2S_RCSR_DBGE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_DBGE_SHIFT)) & I2S_RCSR_DBGE_MASK)
#define I2S_RCSR_STOPE_MASK                      (0x40000000U)
#define I2S_RCSR_STOPE_SHIFT                     (30U)
#define I2S_RCSR_STOPE(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_STOPE_SHIFT)) & I2S_RCSR_STOPE_MASK)
#define I2S_RCSR_RE_MASK                         (0x80000000U)
#define I2S_RCSR_RE_SHIFT                        (31U)
#define I2S_RCSR_RE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_RE_SHIFT)) & I2S_RCSR_RE_MASK)

/*! @name RCR1 - SAI Receive Configuration 1 Register */
#define I2S_RCR1_RFW_MASK                        (0x7U)
#define I2S_RCR1_RFW_SHIFT                       (0U)
#define I2S_RCR1_RFW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR1_RFW_SHIFT)) & I2S_RCR1_RFW_MASK)

/*! @name RCR2 - SAI Receive Configuration 2 Register */
#define I2S_RCR2_DIV_MASK                        (0xFFU)
#define I2S_RCR2_DIV_SHIFT                       (0U)
#define I2S_RCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_DIV_SHIFT)) & I2S_RCR2_DIV_MASK)
#define I2S_RCR2_BCD_MASK                        (0x1000000U)
#define I2S_RCR2_BCD_SHIFT                       (24U)
#define I2S_RCR2_BCD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCD_SHIFT)) & I2S_RCR2_BCD_MASK)
#define I2S_RCR2_BCP_MASK                        (0x2000000U)
#define I2S_RCR2_BCP_SHIFT                       (25U)
#define I2S_RCR2_BCP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCP_SHIFT)) & I2S_RCR2_BCP_MASK)
#define I2S_RCR2_MSEL_MASK                       (0xC000000U)
#define I2S_RCR2_MSEL_SHIFT                      (26U)
#define I2S_RCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_MSEL_SHIFT)) & I2S_RCR2_MSEL_MASK)
#define I2S_RCR2_BCI_MASK                        (0x10000000U)
#define I2S_RCR2_BCI_SHIFT                       (28U)
#define I2S_RCR2_BCI(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCI_SHIFT)) & I2S_RCR2_BCI_MASK)
#define I2S_RCR2_BCS_MASK                        (0x20000000U)
#define I2S_RCR2_BCS_SHIFT                       (29U)
#define I2S_RCR2_BCS(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCS_SHIFT)) & I2S_RCR2_BCS_MASK)
#define I2S_RCR2_SYNC_MASK                       (0xC0000000U)
#define I2S_RCR2_SYNC_SHIFT                      (30U)
#define I2S_RCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_SYNC_SHIFT)) & I2S_RCR2_SYNC_MASK)

/*! @name RCR3 - SAI Receive Configuration 3 Register */
#define I2S_RCR3_WDFL_MASK                       (0x1FU)
#define I2S_RCR3_WDFL_SHIFT                      (0U)
#define I2S_RCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_WDFL_SHIFT)) & I2S_RCR3_WDFL_MASK)
#define I2S_RCR3_RCE_MASK                        (0x30000U)
#define I2S_RCR3_RCE_SHIFT                       (16U)
#define I2S_RCR3_RCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_RCE_SHIFT)) & I2S_RCR3_RCE_MASK)
#define I2S_RCR3_CFR_MASK                        (0x3000000U)
#define I2S_RCR3_CFR_SHIFT                       (24U)
#define I2S_RCR3_CFR(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_CFR_SHIFT)) & I2S_RCR3_CFR_MASK)

/*! @name RCR4 - SAI Receive Configuration 4 Register */
#define I2S_RCR4_FSD_MASK                        (0x1U)
#define I2S_RCR4_FSD_SHIFT                       (0U)
#define I2S_RCR4_FSD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSD_SHIFT)) & I2S_RCR4_FSD_MASK)
#define I2S_RCR4_FSP_MASK                        (0x2U)
#define I2S_RCR4_FSP_SHIFT                       (1U)
#define I2S_RCR4_FSP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSP_SHIFT)) & I2S_RCR4_FSP_MASK)
#define I2S_RCR4_ONDEM_MASK                      (0x4U)
#define I2S_RCR4_ONDEM_SHIFT                     (2U)
#define I2S_RCR4_ONDEM(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_ONDEM_SHIFT)) & I2S_RCR4_ONDEM_MASK)
#define I2S_RCR4_FSE_MASK                        (0x8U)
#define I2S_RCR4_FSE_SHIFT                       (3U)
#define I2S_RCR4_FSE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSE_SHIFT)) & I2S_RCR4_FSE_MASK)
#define I2S_RCR4_MF_MASK                         (0x10U)
#define I2S_RCR4_MF_SHIFT                        (4U)
#define I2S_RCR4_MF(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_MF_SHIFT)) & I2S_RCR4_MF_MASK)
#define I2S_RCR4_SYWD_MASK                       (0x1F00U)
#define I2S_RCR4_SYWD_SHIFT                      (8U)
#define I2S_RCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_SYWD_SHIFT)) & I2S_RCR4_SYWD_MASK)
#define I2S_RCR4_FRSZ_MASK                       (0x1F0000U)
#define I2S_RCR4_FRSZ_SHIFT                      (16U)
#define I2S_RCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FRSZ_SHIFT)) & I2S_RCR4_FRSZ_MASK)
#define I2S_RCR4_FPACK_MASK                      (0x3000000U)
#define I2S_RCR4_FPACK_SHIFT                     (24U)
#define I2S_RCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FPACK_SHIFT)) & I2S_RCR4_FPACK_MASK)
#define I2S_RCR4_FCOMB_MASK                      (0xC000000U)
#define I2S_RCR4_FCOMB_SHIFT                     (26U)
#define I2S_RCR4_FCOMB(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FCOMB_SHIFT)) & I2S_RCR4_FCOMB_MASK)
#define I2S_RCR4_FCONT_MASK                      (0x10000000U)
#define I2S_RCR4_FCONT_SHIFT                     (28U)
#define I2S_RCR4_FCONT(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FCONT_SHIFT)) & I2S_RCR4_FCONT_MASK)

/*! @name RCR5 - SAI Receive Configuration 5 Register */
#define I2S_RCR5_FBT_MASK                        (0x1F00U)
#define I2S_RCR5_FBT_SHIFT                       (8U)
#define I2S_RCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_FBT_SHIFT)) & I2S_RCR5_FBT_MASK)
#define I2S_RCR5_W0W_MASK                        (0x1F0000U)
#define I2S_RCR5_W0W_SHIFT                       (16U)
#define I2S_RCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_W0W_SHIFT)) & I2S_RCR5_W0W_MASK)
#define I2S_RCR5_WNW_MASK                        (0x1F000000U)
#define I2S_RCR5_WNW_SHIFT                       (24U)
#define I2S_RCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_WNW_SHIFT)) & I2S_RCR5_WNW_MASK)

/*! @name RDR - SAI Receive Data Register */
#define I2S_RDR_RDR_MASK                         (0xFFFFFFFFU)
#define I2S_RDR_RDR_SHIFT                        (0U)
#define I2S_RDR_RDR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RDR_RDR_SHIFT)) & I2S_RDR_RDR_MASK)

/* The count of I2S_RDR */
#define I2S_RDR_COUNT                            (2U)

/*! @name RFR - SAI Receive FIFO Register */
#define I2S_RFR_RFP_MASK                         (0xFU)
#define I2S_RFR_RFP_SHIFT                        (0U)
#define I2S_RFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_RFP_SHIFT)) & I2S_RFR_RFP_MASK)
#define I2S_RFR_RCP_MASK                         (0x8000U)
#define I2S_RFR_RCP_SHIFT                        (15U)
#define I2S_RFR_RCP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_RCP_SHIFT)) & I2S_RFR_RCP_MASK)
#define I2S_RFR_WFP_MASK                         (0xF0000U)
#define I2S_RFR_WFP_SHIFT                        (16U)
#define I2S_RFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_WFP_SHIFT)) & I2S_RFR_WFP_MASK)

/* The count of I2S_RFR */
#define I2S_RFR_COUNT                            (2U)

/*! @name RMR - SAI Receive Mask Register */
#define I2S_RMR_RWM_MASK                         (0xFFFFFFFFU)
#define I2S_RMR_RWM_SHIFT                        (0U)
#define I2S_RMR_RWM(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RMR_RWM_SHIFT)) & I2S_RMR_RWM_MASK)

/*! @name MCR - SAI MCLK Control Register */
#define I2S_MCR_MICS_MASK                        (0x3000000U)
#define I2S_MCR_MICS_SHIFT                       (24U)
#define I2S_MCR_MICS(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_MCR_MICS_SHIFT)) & I2S_MCR_MICS_MASK)
#define I2S_MCR_MOE_MASK                         (0x40000000U)
#define I2S_MCR_MOE_SHIFT                        (30U)
#define I2S_MCR_MOE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_MCR_MOE_SHIFT)) & I2S_MCR_MOE_MASK)
#define I2S_MCR_DUF_MASK                         (0x80000000U)
#define I2S_MCR_DUF_SHIFT                        (31U)
#define I2S_MCR_DUF(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_MCR_DUF_SHIFT)) & I2S_MCR_DUF_MASK)

/*! @name MDR - SAI MCLK Divide Register */
#define I2S_MDR_DIVIDE_MASK                      (0xFFFU)
#define I2S_MDR_DIVIDE_SHIFT                     (0U)
#define I2S_MDR_DIVIDE(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_MDR_DIVIDE_SHIFT)) & I2S_MDR_DIVIDE_MASK)
#define I2S_MDR_FRACT_MASK                       (0xFF000U)
#define I2S_MDR_FRACT_SHIFT                      (12U)
#define I2S_MDR_FRACT(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_MDR_FRACT_SHIFT)) & I2S_MDR_FRACT_MASK)


/*!
 * @}
 */ /* end of group I2S_Register_Masks */


/* I2S - Peripheral instance base addresses */
/** Peripheral I2S0 base address */
#define I2S0_BASE                                (0x4002F000u)
/** Peripheral I2S0 base pointer */
#define I2S0                                     ((I2S_Type *)I2S0_BASE)
/** Array initializer of I2S peripheral base addresses */
#define I2S_BASE_ADDRS                           { I2S0_BASE }
/** Array initializer of I2S peripheral base pointers */
#define I2S_BASE_PTRS                            { I2S0 }
/** Interrupt vectors for the I2S peripheral type */
#define I2S_RX_IRQS                              { I2S0_Rx_IRQn }
#define I2S_TX_IRQS                              { I2S0_Tx_IRQn }

/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LLWU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Peripheral_Access_Layer LLWU Peripheral Access Layer
 * @{
 */

/** LLWU - Register Layout Typedef */
typedef struct {
  __IO uint8_t PE1;                                /**< LLWU Pin Enable 1 register, offset: 0x0 */
  __IO uint8_t PE2;                                /**< LLWU Pin Enable 2 register, offset: 0x1 */
  __IO uint8_t PE3;                                /**< LLWU Pin Enable 3 register, offset: 0x2 */
  __IO uint8_t PE4;                                /**< LLWU Pin Enable 4 register, offset: 0x3 */
  __IO uint8_t PE5;                                /**< LLWU Pin Enable 5 register, offset: 0x4 */
  __IO uint8_t PE6;                                /**< LLWU Pin Enable 6 register, offset: 0x5 */
  __IO uint8_t PE7;                                /**< LLWU Pin Enable 7 register, offset: 0x6 */
  __IO uint8_t PE8;                                /**< LLWU Pin Enable 8 register, offset: 0x7 */
  __IO uint8_t ME;                                 /**< LLWU Module Enable register, offset: 0x8 */
  __IO uint8_t PF1;                                /**< LLWU Pin Flag 1 register, offset: 0x9 */
  __IO uint8_t PF2;                                /**< LLWU Pin Flag 2 register, offset: 0xA */
  __IO uint8_t PF3;                                /**< LLWU Pin Flag 3 register, offset: 0xB */
  __IO uint8_t PF4;                                /**< LLWU Pin Flag 4 register, offset: 0xC */
  __I  uint8_t MF5;                                /**< LLWU Module Flag 5 register, offset: 0xD */
  __IO uint8_t FILT1;                              /**< LLWU Pin Filter 1 register, offset: 0xE */
  __IO uint8_t FILT2;                              /**< LLWU Pin Filter 2 register, offset: 0xF */
  __IO uint8_t FILT3;                              /**< LLWU Pin Filter 3 register, offset: 0x10 */
  __IO uint8_t FILT4;                              /**< LLWU Pin Filter 4 register, offset: 0x11 */
} LLWU_Type;

/* ----------------------------------------------------------------------------
   -- LLWU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Masks LLWU Register Masks
 * @{
 */

/*! @name PE1 - LLWU Pin Enable 1 register */
#define LLWU_PE1_WUPE0_MASK                      (0x3U)
#define LLWU_PE1_WUPE0_SHIFT                     (0U)
#define LLWU_PE1_WUPE0(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE1_WUPE0_SHIFT)) & LLWU_PE1_WUPE0_MASK)
#define LLWU_PE1_WUPE1_MASK                      (0xCU)
#define LLWU_PE1_WUPE1_SHIFT                     (2U)
#define LLWU_PE1_WUPE1(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE1_WUPE1_SHIFT)) & LLWU_PE1_WUPE1_MASK)
#define LLWU_PE1_WUPE2_MASK                      (0x30U)
#define LLWU_PE1_WUPE2_SHIFT                     (4U)
#define LLWU_PE1_WUPE2(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE1_WUPE2_SHIFT)) & LLWU_PE1_WUPE2_MASK)
#define LLWU_PE1_WUPE3_MASK                      (0xC0U)
#define LLWU_PE1_WUPE3_SHIFT                     (6U)
#define LLWU_PE1_WUPE3(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE1_WUPE3_SHIFT)) & LLWU_PE1_WUPE3_MASK)

/*! @name PE2 - LLWU Pin Enable 2 register */
#define LLWU_PE2_WUPE4_MASK                      (0x3U)
#define LLWU_PE2_WUPE4_SHIFT                     (0U)
#define LLWU_PE2_WUPE4(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE2_WUPE4_SHIFT)) & LLWU_PE2_WUPE4_MASK)
#define LLWU_PE2_WUPE5_MASK                      (0xCU)
#define LLWU_PE2_WUPE5_SHIFT                     (2U)
#define LLWU_PE2_WUPE5(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE2_WUPE5_SHIFT)) & LLWU_PE2_WUPE5_MASK)
#define LLWU_PE2_WUPE6_MASK                      (0x30U)
#define LLWU_PE2_WUPE6_SHIFT                     (4U)
#define LLWU_PE2_WUPE6(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE2_WUPE6_SHIFT)) & LLWU_PE2_WUPE6_MASK)
#define LLWU_PE2_WUPE7_MASK                      (0xC0U)
#define LLWU_PE2_WUPE7_SHIFT                     (6U)
#define LLWU_PE2_WUPE7(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE2_WUPE7_SHIFT)) & LLWU_PE2_WUPE7_MASK)

/*! @name PE3 - LLWU Pin Enable 3 register */
#define LLWU_PE3_WUPE8_MASK                      (0x3U)
#define LLWU_PE3_WUPE8_SHIFT                     (0U)
#define LLWU_PE3_WUPE8(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE3_WUPE8_SHIFT)) & LLWU_PE3_WUPE8_MASK)
#define LLWU_PE3_WUPE9_MASK                      (0xCU)
#define LLWU_PE3_WUPE9_SHIFT                     (2U)
#define LLWU_PE3_WUPE9(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PE3_WUPE9_SHIFT)) & LLWU_PE3_WUPE9_MASK)
#define LLWU_PE3_WUPE10_MASK                     (0x30U)
#define LLWU_PE3_WUPE10_SHIFT                    (4U)
#define LLWU_PE3_WUPE10(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE3_WUPE10_SHIFT)) & LLWU_PE3_WUPE10_MASK)
#define LLWU_PE3_WUPE11_MASK                     (0xC0U)
#define LLWU_PE3_WUPE11_SHIFT                    (6U)
#define LLWU_PE3_WUPE11(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE3_WUPE11_SHIFT)) & LLWU_PE3_WUPE11_MASK)

/*! @name PE4 - LLWU Pin Enable 4 register */
#define LLWU_PE4_WUPE12_MASK                     (0x3U)
#define LLWU_PE4_WUPE12_SHIFT                    (0U)
#define LLWU_PE4_WUPE12(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE4_WUPE12_SHIFT)) & LLWU_PE4_WUPE12_MASK)
#define LLWU_PE4_WUPE13_MASK                     (0xCU)
#define LLWU_PE4_WUPE13_SHIFT                    (2U)
#define LLWU_PE4_WUPE13(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE4_WUPE13_SHIFT)) & LLWU_PE4_WUPE13_MASK)
#define LLWU_PE4_WUPE14_MASK                     (0x30U)
#define LLWU_PE4_WUPE14_SHIFT                    (4U)
#define LLWU_PE4_WUPE14(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE4_WUPE14_SHIFT)) & LLWU_PE4_WUPE14_MASK)
#define LLWU_PE4_WUPE15_MASK                     (0xC0U)
#define LLWU_PE4_WUPE15_SHIFT                    (6U)
#define LLWU_PE4_WUPE15(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE4_WUPE15_SHIFT)) & LLWU_PE4_WUPE15_MASK)

/*! @name PE5 - LLWU Pin Enable 5 register */
#define LLWU_PE5_WUPE16_MASK                     (0x3U)
#define LLWU_PE5_WUPE16_SHIFT                    (0U)
#define LLWU_PE5_WUPE16(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE5_WUPE16_SHIFT)) & LLWU_PE5_WUPE16_MASK)
#define LLWU_PE5_WUPE17_MASK                     (0xCU)
#define LLWU_PE5_WUPE17_SHIFT                    (2U)
#define LLWU_PE5_WUPE17(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE5_WUPE17_SHIFT)) & LLWU_PE5_WUPE17_MASK)
#define LLWU_PE5_WUPE18_MASK                     (0x30U)
#define LLWU_PE5_WUPE18_SHIFT                    (4U)
#define LLWU_PE5_WUPE18(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE5_WUPE18_SHIFT)) & LLWU_PE5_WUPE18_MASK)
#define LLWU_PE5_WUPE19_MASK                     (0xC0U)
#define LLWU_PE5_WUPE19_SHIFT                    (6U)
#define LLWU_PE5_WUPE19(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE5_WUPE19_SHIFT)) & LLWU_PE5_WUPE19_MASK)

/*! @name PE6 - LLWU Pin Enable 6 register */
#define LLWU_PE6_WUPE20_MASK                     (0x3U)
#define LLWU_PE6_WUPE20_SHIFT                    (0U)
#define LLWU_PE6_WUPE20(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE6_WUPE20_SHIFT)) & LLWU_PE6_WUPE20_MASK)
#define LLWU_PE6_WUPE21_MASK                     (0xCU)
#define LLWU_PE6_WUPE21_SHIFT                    (2U)
#define LLWU_PE6_WUPE21(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE6_WUPE21_SHIFT)) & LLWU_PE6_WUPE21_MASK)
#define LLWU_PE6_WUPE22_MASK                     (0x30U)
#define LLWU_PE6_WUPE22_SHIFT                    (4U)
#define LLWU_PE6_WUPE22(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE6_WUPE22_SHIFT)) & LLWU_PE6_WUPE22_MASK)
#define LLWU_PE6_WUPE23_MASK                     (0xC0U)
#define LLWU_PE6_WUPE23_SHIFT                    (6U)
#define LLWU_PE6_WUPE23(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE6_WUPE23_SHIFT)) & LLWU_PE6_WUPE23_MASK)

/*! @name PE7 - LLWU Pin Enable 7 register */
#define LLWU_PE7_WUPE24_MASK                     (0x3U)
#define LLWU_PE7_WUPE24_SHIFT                    (0U)
#define LLWU_PE7_WUPE24(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE7_WUPE24_SHIFT)) & LLWU_PE7_WUPE24_MASK)
#define LLWU_PE7_WUPE25_MASK                     (0xCU)
#define LLWU_PE7_WUPE25_SHIFT                    (2U)
#define LLWU_PE7_WUPE25(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE7_WUPE25_SHIFT)) & LLWU_PE7_WUPE25_MASK)
#define LLWU_PE7_WUPE26_MASK                     (0x30U)
#define LLWU_PE7_WUPE26_SHIFT                    (4U)
#define LLWU_PE7_WUPE26(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE7_WUPE26_SHIFT)) & LLWU_PE7_WUPE26_MASK)
#define LLWU_PE7_WUPE27_MASK                     (0xC0U)
#define LLWU_PE7_WUPE27_SHIFT                    (6U)
#define LLWU_PE7_WUPE27(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE7_WUPE27_SHIFT)) & LLWU_PE7_WUPE27_MASK)

/*! @name PE8 - LLWU Pin Enable 8 register */
#define LLWU_PE8_WUPE28_MASK                     (0x3U)
#define LLWU_PE8_WUPE28_SHIFT                    (0U)
#define LLWU_PE8_WUPE28(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE8_WUPE28_SHIFT)) & LLWU_PE8_WUPE28_MASK)
#define LLWU_PE8_WUPE29_MASK                     (0xCU)
#define LLWU_PE8_WUPE29_SHIFT                    (2U)
#define LLWU_PE8_WUPE29(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE8_WUPE29_SHIFT)) & LLWU_PE8_WUPE29_MASK)
#define LLWU_PE8_WUPE30_MASK                     (0x30U)
#define LLWU_PE8_WUPE30_SHIFT                    (4U)
#define LLWU_PE8_WUPE30(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE8_WUPE30_SHIFT)) & LLWU_PE8_WUPE30_MASK)
#define LLWU_PE8_WUPE31_MASK                     (0xC0U)
#define LLWU_PE8_WUPE31_SHIFT                    (6U)
#define LLWU_PE8_WUPE31(x)                       (((uint8_t)(((uint8_t)(x)) << LLWU_PE8_WUPE31_SHIFT)) & LLWU_PE8_WUPE31_MASK)

/*! @name ME - LLWU Module Enable register */
#define LLWU_ME_WUME0_MASK                       (0x1U)
#define LLWU_ME_WUME0_SHIFT                      (0U)
#define LLWU_ME_WUME0(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME0_SHIFT)) & LLWU_ME_WUME0_MASK)
#define LLWU_ME_WUME1_MASK                       (0x2U)
#define LLWU_ME_WUME1_SHIFT                      (1U)
#define LLWU_ME_WUME1(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME1_SHIFT)) & LLWU_ME_WUME1_MASK)
#define LLWU_ME_WUME2_MASK                       (0x4U)
#define LLWU_ME_WUME2_SHIFT                      (2U)
#define LLWU_ME_WUME2(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME2_SHIFT)) & LLWU_ME_WUME2_MASK)
#define LLWU_ME_WUME3_MASK                       (0x8U)
#define LLWU_ME_WUME3_SHIFT                      (3U)
#define LLWU_ME_WUME3(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME3_SHIFT)) & LLWU_ME_WUME3_MASK)
#define LLWU_ME_WUME4_MASK                       (0x10U)
#define LLWU_ME_WUME4_SHIFT                      (4U)
#define LLWU_ME_WUME4(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME4_SHIFT)) & LLWU_ME_WUME4_MASK)
#define LLWU_ME_WUME5_MASK                       (0x20U)
#define LLWU_ME_WUME5_SHIFT                      (5U)
#define LLWU_ME_WUME5(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME5_SHIFT)) & LLWU_ME_WUME5_MASK)
#define LLWU_ME_WUME6_MASK                       (0x40U)
#define LLWU_ME_WUME6_SHIFT                      (6U)
#define LLWU_ME_WUME6(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME6_SHIFT)) & LLWU_ME_WUME6_MASK)
#define LLWU_ME_WUME7_MASK                       (0x80U)
#define LLWU_ME_WUME7_SHIFT                      (7U)
#define LLWU_ME_WUME7(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_ME_WUME7_SHIFT)) & LLWU_ME_WUME7_MASK)

/*! @name PF1 - LLWU Pin Flag 1 register */
#define LLWU_PF1_WUF0_MASK                       (0x1U)
#define LLWU_PF1_WUF0_SHIFT                      (0U)
#define LLWU_PF1_WUF0(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF0_SHIFT)) & LLWU_PF1_WUF0_MASK)
#define LLWU_PF1_WUF1_MASK                       (0x2U)
#define LLWU_PF1_WUF1_SHIFT                      (1U)
#define LLWU_PF1_WUF1(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF1_SHIFT)) & LLWU_PF1_WUF1_MASK)
#define LLWU_PF1_WUF2_MASK                       (0x4U)
#define LLWU_PF1_WUF2_SHIFT                      (2U)
#define LLWU_PF1_WUF2(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF2_SHIFT)) & LLWU_PF1_WUF2_MASK)
#define LLWU_PF1_WUF3_MASK                       (0x8U)
#define LLWU_PF1_WUF3_SHIFT                      (3U)
#define LLWU_PF1_WUF3(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF3_SHIFT)) & LLWU_PF1_WUF3_MASK)
#define LLWU_PF1_WUF4_MASK                       (0x10U)
#define LLWU_PF1_WUF4_SHIFT                      (4U)
#define LLWU_PF1_WUF4(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF4_SHIFT)) & LLWU_PF1_WUF4_MASK)
#define LLWU_PF1_WUF5_MASK                       (0x20U)
#define LLWU_PF1_WUF5_SHIFT                      (5U)
#define LLWU_PF1_WUF5(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF5_SHIFT)) & LLWU_PF1_WUF5_MASK)
#define LLWU_PF1_WUF6_MASK                       (0x40U)
#define LLWU_PF1_WUF6_SHIFT                      (6U)
#define LLWU_PF1_WUF6(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF6_SHIFT)) & LLWU_PF1_WUF6_MASK)
#define LLWU_PF1_WUF7_MASK                       (0x80U)
#define LLWU_PF1_WUF7_SHIFT                      (7U)
#define LLWU_PF1_WUF7(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF1_WUF7_SHIFT)) & LLWU_PF1_WUF7_MASK)

/*! @name PF2 - LLWU Pin Flag 2 register */
#define LLWU_PF2_WUF8_MASK                       (0x1U)
#define LLWU_PF2_WUF8_SHIFT                      (0U)
#define LLWU_PF2_WUF8(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF8_SHIFT)) & LLWU_PF2_WUF8_MASK)
#define LLWU_PF2_WUF9_MASK                       (0x2U)
#define LLWU_PF2_WUF9_SHIFT                      (1U)
#define LLWU_PF2_WUF9(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF9_SHIFT)) & LLWU_PF2_WUF9_MASK)
#define LLWU_PF2_WUF10_MASK                      (0x4U)
#define LLWU_PF2_WUF10_SHIFT                     (2U)
#define LLWU_PF2_WUF10(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF10_SHIFT)) & LLWU_PF2_WUF10_MASK)
#define LLWU_PF2_WUF11_MASK                      (0x8U)
#define LLWU_PF2_WUF11_SHIFT                     (3U)
#define LLWU_PF2_WUF11(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF11_SHIFT)) & LLWU_PF2_WUF11_MASK)
#define LLWU_PF2_WUF12_MASK                      (0x10U)
#define LLWU_PF2_WUF12_SHIFT                     (4U)
#define LLWU_PF2_WUF12(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF12_SHIFT)) & LLWU_PF2_WUF12_MASK)
#define LLWU_PF2_WUF13_MASK                      (0x20U)
#define LLWU_PF2_WUF13_SHIFT                     (5U)
#define LLWU_PF2_WUF13(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF13_SHIFT)) & LLWU_PF2_WUF13_MASK)
#define LLWU_PF2_WUF14_MASK                      (0x40U)
#define LLWU_PF2_WUF14_SHIFT                     (6U)
#define LLWU_PF2_WUF14(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF14_SHIFT)) & LLWU_PF2_WUF14_MASK)
#define LLWU_PF2_WUF15_MASK                      (0x80U)
#define LLWU_PF2_WUF15_SHIFT                     (7U)
#define LLWU_PF2_WUF15(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF2_WUF15_SHIFT)) & LLWU_PF2_WUF15_MASK)

/*! @name PF3 - LLWU Pin Flag 3 register */
#define LLWU_PF3_WUF16_MASK                      (0x1U)
#define LLWU_PF3_WUF16_SHIFT                     (0U)
#define LLWU_PF3_WUF16(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF16_SHIFT)) & LLWU_PF3_WUF16_MASK)
#define LLWU_PF3_WUF17_MASK                      (0x2U)
#define LLWU_PF3_WUF17_SHIFT                     (1U)
#define LLWU_PF3_WUF17(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF17_SHIFT)) & LLWU_PF3_WUF17_MASK)
#define LLWU_PF3_WUF18_MASK                      (0x4U)
#define LLWU_PF3_WUF18_SHIFT                     (2U)
#define LLWU_PF3_WUF18(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF18_SHIFT)) & LLWU_PF3_WUF18_MASK)
#define LLWU_PF3_WUF19_MASK                      (0x8U)
#define LLWU_PF3_WUF19_SHIFT                     (3U)
#define LLWU_PF3_WUF19(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF19_SHIFT)) & LLWU_PF3_WUF19_MASK)
#define LLWU_PF3_WUF20_MASK                      (0x10U)
#define LLWU_PF3_WUF20_SHIFT                     (4U)
#define LLWU_PF3_WUF20(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF20_SHIFT)) & LLWU_PF3_WUF20_MASK)
#define LLWU_PF3_WUF21_MASK                      (0x20U)
#define LLWU_PF3_WUF21_SHIFT                     (5U)
#define LLWU_PF3_WUF21(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF21_SHIFT)) & LLWU_PF3_WUF21_MASK)
#define LLWU_PF3_WUF22_MASK                      (0x40U)
#define LLWU_PF3_WUF22_SHIFT                     (6U)
#define LLWU_PF3_WUF22(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF22_SHIFT)) & LLWU_PF3_WUF22_MASK)
#define LLWU_PF3_WUF23_MASK                      (0x80U)
#define LLWU_PF3_WUF23_SHIFT                     (7U)
#define LLWU_PF3_WUF23(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF3_WUF23_SHIFT)) & LLWU_PF3_WUF23_MASK)

/*! @name PF4 - LLWU Pin Flag 4 register */
#define LLWU_PF4_WUF24_MASK                      (0x1U)
#define LLWU_PF4_WUF24_SHIFT                     (0U)
#define LLWU_PF4_WUF24(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF24_SHIFT)) & LLWU_PF4_WUF24_MASK)
#define LLWU_PF4_WUF25_MASK                      (0x2U)
#define LLWU_PF4_WUF25_SHIFT                     (1U)
#define LLWU_PF4_WUF25(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF25_SHIFT)) & LLWU_PF4_WUF25_MASK)
#define LLWU_PF4_WUF26_MASK                      (0x4U)
#define LLWU_PF4_WUF26_SHIFT                     (2U)
#define LLWU_PF4_WUF26(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF26_SHIFT)) & LLWU_PF4_WUF26_MASK)
#define LLWU_PF4_WUF27_MASK                      (0x8U)
#define LLWU_PF4_WUF27_SHIFT                     (3U)
#define LLWU_PF4_WUF27(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF27_SHIFT)) & LLWU_PF4_WUF27_MASK)
#define LLWU_PF4_WUF28_MASK                      (0x10U)
#define LLWU_PF4_WUF28_SHIFT                     (4U)
#define LLWU_PF4_WUF28(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF28_SHIFT)) & LLWU_PF4_WUF28_MASK)
#define LLWU_PF4_WUF29_MASK                      (0x20U)
#define LLWU_PF4_WUF29_SHIFT                     (5U)
#define LLWU_PF4_WUF29(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF29_SHIFT)) & LLWU_PF4_WUF29_MASK)
#define LLWU_PF4_WUF30_MASK                      (0x40U)
#define LLWU_PF4_WUF30_SHIFT                     (6U)
#define LLWU_PF4_WUF30(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF30_SHIFT)) & LLWU_PF4_WUF30_MASK)
#define LLWU_PF4_WUF31_MASK                      (0x80U)
#define LLWU_PF4_WUF31_SHIFT                     (7U)
#define LLWU_PF4_WUF31(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_PF4_WUF31_SHIFT)) & LLWU_PF4_WUF31_MASK)

/*! @name MF5 - LLWU Module Flag 5 register */
#define LLWU_MF5_MWUF0_MASK                      (0x1U)
#define LLWU_MF5_MWUF0_SHIFT                     (0U)
#define LLWU_MF5_MWUF0(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF0_SHIFT)) & LLWU_MF5_MWUF0_MASK)
#define LLWU_MF5_MWUF1_MASK                      (0x2U)
#define LLWU_MF5_MWUF1_SHIFT                     (1U)
#define LLWU_MF5_MWUF1(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF1_SHIFT)) & LLWU_MF5_MWUF1_MASK)
#define LLWU_MF5_MWUF2_MASK                      (0x4U)
#define LLWU_MF5_MWUF2_SHIFT                     (2U)
#define LLWU_MF5_MWUF2(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF2_SHIFT)) & LLWU_MF5_MWUF2_MASK)
#define LLWU_MF5_MWUF3_MASK                      (0x8U)
#define LLWU_MF5_MWUF3_SHIFT                     (3U)
#define LLWU_MF5_MWUF3(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF3_SHIFT)) & LLWU_MF5_MWUF3_MASK)
#define LLWU_MF5_MWUF4_MASK                      (0x10U)
#define LLWU_MF5_MWUF4_SHIFT                     (4U)
#define LLWU_MF5_MWUF4(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF4_SHIFT)) & LLWU_MF5_MWUF4_MASK)
#define LLWU_MF5_MWUF5_MASK                      (0x20U)
#define LLWU_MF5_MWUF5_SHIFT                     (5U)
#define LLWU_MF5_MWUF5(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF5_SHIFT)) & LLWU_MF5_MWUF5_MASK)
#define LLWU_MF5_MWUF6_MASK                      (0x40U)
#define LLWU_MF5_MWUF6_SHIFT                     (6U)
#define LLWU_MF5_MWUF6(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF6_SHIFT)) & LLWU_MF5_MWUF6_MASK)
#define LLWU_MF5_MWUF7_MASK                      (0x80U)
#define LLWU_MF5_MWUF7_SHIFT                     (7U)
#define LLWU_MF5_MWUF7(x)                        (((uint8_t)(((uint8_t)(x)) << LLWU_MF5_MWUF7_SHIFT)) & LLWU_MF5_MWUF7_MASK)

/*! @name FILT1 - LLWU Pin Filter 1 register */
#define LLWU_FILT1_FILTSEL_MASK                  (0x1FU)
#define LLWU_FILT1_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT1_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTSEL_SHIFT)) & LLWU_FILT1_FILTSEL_MASK)
#define LLWU_FILT1_FILTE_MASK                    (0x60U)
#define LLWU_FILT1_FILTE_SHIFT                   (5U)
#define LLWU_FILT1_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTE_SHIFT)) & LLWU_FILT1_FILTE_MASK)
#define LLWU_FILT1_FILTF_MASK                    (0x80U)
#define LLWU_FILT1_FILTF_SHIFT                   (7U)
#define LLWU_FILT1_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTF_SHIFT)) & LLWU_FILT1_FILTF_MASK)

/*! @name FILT2 - LLWU Pin Filter 2 register */
#define LLWU_FILT2_FILTSEL_MASK                  (0x1FU)
#define LLWU_FILT2_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT2_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTSEL_SHIFT)) & LLWU_FILT2_FILTSEL_MASK)
#define LLWU_FILT2_FILTE_MASK                    (0x60U)
#define LLWU_FILT2_FILTE_SHIFT                   (5U)
#define LLWU_FILT2_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTE_SHIFT)) & LLWU_FILT2_FILTE_MASK)
#define LLWU_FILT2_FILTF_MASK                    (0x80U)
#define LLWU_FILT2_FILTF_SHIFT                   (7U)
#define LLWU_FILT2_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTF_SHIFT)) & LLWU_FILT2_FILTF_MASK)

/*! @name FILT3 - LLWU Pin Filter 3 register */
#define LLWU_FILT3_FILTSEL_MASK                  (0x1FU)
#define LLWU_FILT3_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT3_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT3_FILTSEL_SHIFT)) & LLWU_FILT3_FILTSEL_MASK)
#define LLWU_FILT3_FILTE_MASK                    (0x60U)
#define LLWU_FILT3_FILTE_SHIFT                   (5U)
#define LLWU_FILT3_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT3_FILTE_SHIFT)) & LLWU_FILT3_FILTE_MASK)
#define LLWU_FILT3_FILTF_MASK                    (0x80U)
#define LLWU_FILT3_FILTF_SHIFT                   (7U)
#define LLWU_FILT3_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT3_FILTF_SHIFT)) & LLWU_FILT3_FILTF_MASK)

/*! @name FILT4 - LLWU Pin Filter 4 register */
#define LLWU_FILT4_FILTSEL_MASK                  (0x1FU)
#define LLWU_FILT4_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT4_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT4_FILTSEL_SHIFT)) & LLWU_FILT4_FILTSEL_MASK)
#define LLWU_FILT4_FILTE_MASK                    (0x60U)
#define LLWU_FILT4_FILTE_SHIFT                   (5U)
#define LLWU_FILT4_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT4_FILTE_SHIFT)) & LLWU_FILT4_FILTE_MASK)
#define LLWU_FILT4_FILTF_MASK                    (0x80U)
#define LLWU_FILT4_FILTF_SHIFT                   (7U)
#define LLWU_FILT4_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT4_FILTF_SHIFT)) & LLWU_FILT4_FILTF_MASK)


/*!
 * @}
 */ /* end of group LLWU_Register_Masks */


/* LLWU - Peripheral instance base addresses */
/** Peripheral LLWU base address */
#define LLWU_BASE                                (0x4007C000u)
/** Peripheral LLWU base pointer */
#define LLWU                                     ((LLWU_Type *)LLWU_BASE)
/** Array initializer of LLWU peripheral base addresses */
#define LLWU_BASE_ADDRS                          { LLWU_BASE }
/** Array initializer of LLWU peripheral base pointers */
#define LLWU_BASE_PTRS                           { LLWU }
/** Interrupt vectors for the LLWU peripheral type */
#define LLWU_IRQS                                { LLWU_IRQn }

/*!
 * @}
 */ /* end of group LLWU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LMEM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LMEM_Peripheral_Access_Layer LMEM Peripheral Access Layer
 * @{
 */

/** LMEM - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCCCR;                             /**< Cache control register, offset: 0x0 */
  __IO uint32_t PCCLCR;                            /**< Cache line control register, offset: 0x4 */
  __IO uint32_t PCCSAR;                            /**< Cache search address register, offset: 0x8 */
  __IO uint32_t PCCCVR;                            /**< Cache read/write value register, offset: 0xC */
       uint8_t RESERVED_0[16];
  __IO uint32_t PCCRMR;                            /**< Cache regions mode register, offset: 0x20 */
       uint8_t RESERVED_1[2012];
  __IO uint32_t PSCCR;                             /**< Cache control register, offset: 0x800 */
  __IO uint32_t PSCLCR;                            /**< Cache line control register, offset: 0x804 */
  __IO uint32_t PSCSAR;                            /**< Cache search address register, offset: 0x808 */
  __IO uint32_t PSCCVR;                            /**< Cache read/write value register, offset: 0x80C */
       uint8_t RESERVED_2[16];
  __IO uint32_t PSCRMR;                            /**< Cache regions mode register, offset: 0x820 */
} LMEM_Type;

/* ----------------------------------------------------------------------------
   -- LMEM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LMEM_Register_Masks LMEM Register Masks
 * @{
 */

/*! @name PCCCR - Cache control register */
#define LMEM_PCCCR_ENCACHE_MASK                  (0x1U)
#define LMEM_PCCCR_ENCACHE_SHIFT                 (0U)
#define LMEM_PCCCR_ENCACHE(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_ENCACHE_SHIFT)) & LMEM_PCCCR_ENCACHE_MASK)
#define LMEM_PCCCR_ENWRBUF_MASK                  (0x2U)
#define LMEM_PCCCR_ENWRBUF_SHIFT                 (1U)
#define LMEM_PCCCR_ENWRBUF(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_ENWRBUF_SHIFT)) & LMEM_PCCCR_ENWRBUF_MASK)
#define LMEM_PCCCR_PCCR2_MASK                    (0x4U)
#define LMEM_PCCCR_PCCR2_SHIFT                   (2U)
#define LMEM_PCCCR_PCCR2(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PCCR2_SHIFT)) & LMEM_PCCCR_PCCR2_MASK)
#define LMEM_PCCCR_PCCR3_MASK                    (0x8U)
#define LMEM_PCCCR_PCCR3_SHIFT                   (3U)
#define LMEM_PCCCR_PCCR3(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PCCR3_SHIFT)) & LMEM_PCCCR_PCCR3_MASK)
#define LMEM_PCCCR_INVW0_MASK                    (0x1000000U)
#define LMEM_PCCCR_INVW0_SHIFT                   (24U)
#define LMEM_PCCCR_INVW0(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_INVW0_SHIFT)) & LMEM_PCCCR_INVW0_MASK)
#define LMEM_PCCCR_PUSHW0_MASK                   (0x2000000U)
#define LMEM_PCCCR_PUSHW0_SHIFT                  (25U)
#define LMEM_PCCCR_PUSHW0(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PUSHW0_SHIFT)) & LMEM_PCCCR_PUSHW0_MASK)
#define LMEM_PCCCR_INVW1_MASK                    (0x4000000U)
#define LMEM_PCCCR_INVW1_SHIFT                   (26U)
#define LMEM_PCCCR_INVW1(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_INVW1_SHIFT)) & LMEM_PCCCR_INVW1_MASK)
#define LMEM_PCCCR_PUSHW1_MASK                   (0x8000000U)
#define LMEM_PCCCR_PUSHW1_SHIFT                  (27U)
#define LMEM_PCCCR_PUSHW1(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PUSHW1_SHIFT)) & LMEM_PCCCR_PUSHW1_MASK)
#define LMEM_PCCCR_GO_MASK                       (0x80000000U)
#define LMEM_PCCCR_GO_SHIFT                      (31U)
#define LMEM_PCCCR_GO(x)                         (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_GO_SHIFT)) & LMEM_PCCCR_GO_MASK)

/*! @name PCCLCR - Cache line control register */
#define LMEM_PCCLCR_LGO_MASK                     (0x1U)
#define LMEM_PCCLCR_LGO_SHIFT                    (0U)
#define LMEM_PCCLCR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LGO_SHIFT)) & LMEM_PCCLCR_LGO_MASK)
#define LMEM_PCCLCR_CACHEADDR_MASK               (0xFFCU)
#define LMEM_PCCLCR_CACHEADDR_SHIFT              (2U)
#define LMEM_PCCLCR_CACHEADDR(x)                 (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_CACHEADDR_SHIFT)) & LMEM_PCCLCR_CACHEADDR_MASK)
#define LMEM_PCCLCR_WSEL_MASK                    (0x4000U)
#define LMEM_PCCLCR_WSEL_SHIFT                   (14U)
#define LMEM_PCCLCR_WSEL(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_WSEL_SHIFT)) & LMEM_PCCLCR_WSEL_MASK)
#define LMEM_PCCLCR_TDSEL_MASK                   (0x10000U)
#define LMEM_PCCLCR_TDSEL_SHIFT                  (16U)
#define LMEM_PCCLCR_TDSEL(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_TDSEL_SHIFT)) & LMEM_PCCLCR_TDSEL_MASK)
#define LMEM_PCCLCR_LCIVB_MASK                   (0x100000U)
#define LMEM_PCCLCR_LCIVB_SHIFT                  (20U)
#define LMEM_PCCLCR_LCIVB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCIVB_SHIFT)) & LMEM_PCCLCR_LCIVB_MASK)
#define LMEM_PCCLCR_LCIMB_MASK                   (0x200000U)
#define LMEM_PCCLCR_LCIMB_SHIFT                  (21U)
#define LMEM_PCCLCR_LCIMB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCIMB_SHIFT)) & LMEM_PCCLCR_LCIMB_MASK)
#define LMEM_PCCLCR_LCWAY_MASK                   (0x400000U)
#define LMEM_PCCLCR_LCWAY_SHIFT                  (22U)
#define LMEM_PCCLCR_LCWAY(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCWAY_SHIFT)) & LMEM_PCCLCR_LCWAY_MASK)
#define LMEM_PCCLCR_LCMD_MASK                    (0x3000000U)
#define LMEM_PCCLCR_LCMD_SHIFT                   (24U)
#define LMEM_PCCLCR_LCMD(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCMD_SHIFT)) & LMEM_PCCLCR_LCMD_MASK)
#define LMEM_PCCLCR_LADSEL_MASK                  (0x4000000U)
#define LMEM_PCCLCR_LADSEL_SHIFT                 (26U)
#define LMEM_PCCLCR_LADSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LADSEL_SHIFT)) & LMEM_PCCLCR_LADSEL_MASK)
#define LMEM_PCCLCR_LACC_MASK                    (0x8000000U)
#define LMEM_PCCLCR_LACC_SHIFT                   (27U)
#define LMEM_PCCLCR_LACC(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LACC_SHIFT)) & LMEM_PCCLCR_LACC_MASK)

/*! @name PCCSAR - Cache search address register */
#define LMEM_PCCSAR_LGO_MASK                     (0x1U)
#define LMEM_PCCSAR_LGO_SHIFT                    (0U)
#define LMEM_PCCSAR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCSAR_LGO_SHIFT)) & LMEM_PCCSAR_LGO_MASK)
#define LMEM_PCCSAR_PHYADDR_MASK                 (0xFFFFFFFCU)
#define LMEM_PCCSAR_PHYADDR_SHIFT                (2U)
#define LMEM_PCCSAR_PHYADDR(x)                   (((uint32_t)(((uint32_t)(x)) << LMEM_PCCSAR_PHYADDR_SHIFT)) & LMEM_PCCSAR_PHYADDR_MASK)

/*! @name PCCCVR - Cache read/write value register */
#define LMEM_PCCCVR_DATA_MASK                    (0xFFFFFFFFU)
#define LMEM_PCCCVR_DATA_SHIFT                   (0U)
#define LMEM_PCCCVR_DATA(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCVR_DATA_SHIFT)) & LMEM_PCCCVR_DATA_MASK)

/*! @name PCCRMR - Cache regions mode register */
#define LMEM_PCCRMR_R15_MASK                     (0x3U)
#define LMEM_PCCRMR_R15_SHIFT                    (0U)
#define LMEM_PCCRMR_R15(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R15_SHIFT)) & LMEM_PCCRMR_R15_MASK)
#define LMEM_PCCRMR_R14_MASK                     (0xCU)
#define LMEM_PCCRMR_R14_SHIFT                    (2U)
#define LMEM_PCCRMR_R14(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R14_SHIFT)) & LMEM_PCCRMR_R14_MASK)
#define LMEM_PCCRMR_R13_MASK                     (0x30U)
#define LMEM_PCCRMR_R13_SHIFT                    (4U)
#define LMEM_PCCRMR_R13(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R13_SHIFT)) & LMEM_PCCRMR_R13_MASK)
#define LMEM_PCCRMR_R12_MASK                     (0xC0U)
#define LMEM_PCCRMR_R12_SHIFT                    (6U)
#define LMEM_PCCRMR_R12(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R12_SHIFT)) & LMEM_PCCRMR_R12_MASK)
#define LMEM_PCCRMR_R11_MASK                     (0x300U)
#define LMEM_PCCRMR_R11_SHIFT                    (8U)
#define LMEM_PCCRMR_R11(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R11_SHIFT)) & LMEM_PCCRMR_R11_MASK)
#define LMEM_PCCRMR_R10_MASK                     (0xC00U)
#define LMEM_PCCRMR_R10_SHIFT                    (10U)
#define LMEM_PCCRMR_R10(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R10_SHIFT)) & LMEM_PCCRMR_R10_MASK)
#define LMEM_PCCRMR_R9_MASK                      (0x3000U)
#define LMEM_PCCRMR_R9_SHIFT                     (12U)
#define LMEM_PCCRMR_R9(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R9_SHIFT)) & LMEM_PCCRMR_R9_MASK)
#define LMEM_PCCRMR_R8_MASK                      (0xC000U)
#define LMEM_PCCRMR_R8_SHIFT                     (14U)
#define LMEM_PCCRMR_R8(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R8_SHIFT)) & LMEM_PCCRMR_R8_MASK)
#define LMEM_PCCRMR_R7_MASK                      (0x30000U)
#define LMEM_PCCRMR_R7_SHIFT                     (16U)
#define LMEM_PCCRMR_R7(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R7_SHIFT)) & LMEM_PCCRMR_R7_MASK)
#define LMEM_PCCRMR_R6_MASK                      (0xC0000U)
#define LMEM_PCCRMR_R6_SHIFT                     (18U)
#define LMEM_PCCRMR_R6(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R6_SHIFT)) & LMEM_PCCRMR_R6_MASK)
#define LMEM_PCCRMR_R5_MASK                      (0x300000U)
#define LMEM_PCCRMR_R5_SHIFT                     (20U)
#define LMEM_PCCRMR_R5(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R5_SHIFT)) & LMEM_PCCRMR_R5_MASK)
#define LMEM_PCCRMR_R4_MASK                      (0xC00000U)
#define LMEM_PCCRMR_R4_SHIFT                     (22U)
#define LMEM_PCCRMR_R4(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R4_SHIFT)) & LMEM_PCCRMR_R4_MASK)
#define LMEM_PCCRMR_R3_MASK                      (0x3000000U)
#define LMEM_PCCRMR_R3_SHIFT                     (24U)
#define LMEM_PCCRMR_R3(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R3_SHIFT)) & LMEM_PCCRMR_R3_MASK)
#define LMEM_PCCRMR_R2_MASK                      (0xC000000U)
#define LMEM_PCCRMR_R2_SHIFT                     (26U)
#define LMEM_PCCRMR_R2(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R2_SHIFT)) & LMEM_PCCRMR_R2_MASK)
#define LMEM_PCCRMR_R1_MASK                      (0x30000000U)
#define LMEM_PCCRMR_R1_SHIFT                     (28U)
#define LMEM_PCCRMR_R1(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R1_SHIFT)) & LMEM_PCCRMR_R1_MASK)
#define LMEM_PCCRMR_R0_MASK                      (0xC0000000U)
#define LMEM_PCCRMR_R0_SHIFT                     (30U)
#define LMEM_PCCRMR_R0(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R0_SHIFT)) & LMEM_PCCRMR_R0_MASK)

/*! @name PSCCR - Cache control register */
#define LMEM_PSCCR_ENCACHE_MASK                  (0x1U)
#define LMEM_PSCCR_ENCACHE_SHIFT                 (0U)
#define LMEM_PSCCR_ENCACHE(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_ENCACHE_SHIFT)) & LMEM_PSCCR_ENCACHE_MASK)
#define LMEM_PSCCR_ENWRBUF_MASK                  (0x2U)
#define LMEM_PSCCR_ENWRBUF_SHIFT                 (1U)
#define LMEM_PSCCR_ENWRBUF(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_ENWRBUF_SHIFT)) & LMEM_PSCCR_ENWRBUF_MASK)
#define LMEM_PSCCR_INVW0_MASK                    (0x1000000U)
#define LMEM_PSCCR_INVW0_SHIFT                   (24U)
#define LMEM_PSCCR_INVW0(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_INVW0_SHIFT)) & LMEM_PSCCR_INVW0_MASK)
#define LMEM_PSCCR_PUSHW0_MASK                   (0x2000000U)
#define LMEM_PSCCR_PUSHW0_SHIFT                  (25U)
#define LMEM_PSCCR_PUSHW0(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_PUSHW0_SHIFT)) & LMEM_PSCCR_PUSHW0_MASK)
#define LMEM_PSCCR_INVW1_MASK                    (0x4000000U)
#define LMEM_PSCCR_INVW1_SHIFT                   (26U)
#define LMEM_PSCCR_INVW1(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_INVW1_SHIFT)) & LMEM_PSCCR_INVW1_MASK)
#define LMEM_PSCCR_PUSHW1_MASK                   (0x8000000U)
#define LMEM_PSCCR_PUSHW1_SHIFT                  (27U)
#define LMEM_PSCCR_PUSHW1(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_PUSHW1_SHIFT)) & LMEM_PSCCR_PUSHW1_MASK)
#define LMEM_PSCCR_GO_MASK                       (0x80000000U)
#define LMEM_PSCCR_GO_SHIFT                      (31U)
#define LMEM_PSCCR_GO(x)                         (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCR_GO_SHIFT)) & LMEM_PSCCR_GO_MASK)

/*! @name PSCLCR - Cache line control register */
#define LMEM_PSCLCR_LGO_MASK                     (0x1U)
#define LMEM_PSCLCR_LGO_SHIFT                    (0U)
#define LMEM_PSCLCR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LGO_SHIFT)) & LMEM_PSCLCR_LGO_MASK)
#define LMEM_PSCLCR_CACHEADDR_MASK               (0xFFCU)
#define LMEM_PSCLCR_CACHEADDR_SHIFT              (2U)
#define LMEM_PSCLCR_CACHEADDR(x)                 (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_CACHEADDR_SHIFT)) & LMEM_PSCLCR_CACHEADDR_MASK)
#define LMEM_PSCLCR_WSEL_MASK                    (0x4000U)
#define LMEM_PSCLCR_WSEL_SHIFT                   (14U)
#define LMEM_PSCLCR_WSEL(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_WSEL_SHIFT)) & LMEM_PSCLCR_WSEL_MASK)
#define LMEM_PSCLCR_TDSEL_MASK                   (0x10000U)
#define LMEM_PSCLCR_TDSEL_SHIFT                  (16U)
#define LMEM_PSCLCR_TDSEL(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_TDSEL_SHIFT)) & LMEM_PSCLCR_TDSEL_MASK)
#define LMEM_PSCLCR_LCIVB_MASK                   (0x100000U)
#define LMEM_PSCLCR_LCIVB_SHIFT                  (20U)
#define LMEM_PSCLCR_LCIVB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LCIVB_SHIFT)) & LMEM_PSCLCR_LCIVB_MASK)
#define LMEM_PSCLCR_LCIMB_MASK                   (0x200000U)
#define LMEM_PSCLCR_LCIMB_SHIFT                  (21U)
#define LMEM_PSCLCR_LCIMB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LCIMB_SHIFT)) & LMEM_PSCLCR_LCIMB_MASK)
#define LMEM_PSCLCR_LCWAY_MASK                   (0x400000U)
#define LMEM_PSCLCR_LCWAY_SHIFT                  (22U)
#define LMEM_PSCLCR_LCWAY(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LCWAY_SHIFT)) & LMEM_PSCLCR_LCWAY_MASK)
#define LMEM_PSCLCR_LCMD_MASK                    (0x3000000U)
#define LMEM_PSCLCR_LCMD_SHIFT                   (24U)
#define LMEM_PSCLCR_LCMD(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LCMD_SHIFT)) & LMEM_PSCLCR_LCMD_MASK)
#define LMEM_PSCLCR_LADSEL_MASK                  (0x4000000U)
#define LMEM_PSCLCR_LADSEL_SHIFT                 (26U)
#define LMEM_PSCLCR_LADSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LADSEL_SHIFT)) & LMEM_PSCLCR_LADSEL_MASK)
#define LMEM_PSCLCR_LACC_MASK                    (0x8000000U)
#define LMEM_PSCLCR_LACC_SHIFT                   (27U)
#define LMEM_PSCLCR_LACC(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCLCR_LACC_SHIFT)) & LMEM_PSCLCR_LACC_MASK)

/*! @name PSCSAR - Cache search address register */
#define LMEM_PSCSAR_LGO_MASK                     (0x1U)
#define LMEM_PSCSAR_LGO_SHIFT                    (0U)
#define LMEM_PSCSAR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCSAR_LGO_SHIFT)) & LMEM_PSCSAR_LGO_MASK)
#define LMEM_PSCSAR_PHYADDR_MASK                 (0xFFFFFFFCU)
#define LMEM_PSCSAR_PHYADDR_SHIFT                (2U)
#define LMEM_PSCSAR_PHYADDR(x)                   (((uint32_t)(((uint32_t)(x)) << LMEM_PSCSAR_PHYADDR_SHIFT)) & LMEM_PSCSAR_PHYADDR_MASK)

/*! @name PSCCVR - Cache read/write value register */
#define LMEM_PSCCVR_DATA_MASK                    (0xFFFFFFFFU)
#define LMEM_PSCCVR_DATA_SHIFT                   (0U)
#define LMEM_PSCCVR_DATA(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PSCCVR_DATA_SHIFT)) & LMEM_PSCCVR_DATA_MASK)

/*! @name PSCRMR - Cache regions mode register */
#define LMEM_PSCRMR_R15_MASK                     (0x3U)
#define LMEM_PSCRMR_R15_SHIFT                    (0U)
#define LMEM_PSCRMR_R15(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R15_SHIFT)) & LMEM_PSCRMR_R15_MASK)
#define LMEM_PSCRMR_R14_MASK                     (0xCU)
#define LMEM_PSCRMR_R14_SHIFT                    (2U)
#define LMEM_PSCRMR_R14(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R14_SHIFT)) & LMEM_PSCRMR_R14_MASK)
#define LMEM_PSCRMR_R13_MASK                     (0x30U)
#define LMEM_PSCRMR_R13_SHIFT                    (4U)
#define LMEM_PSCRMR_R13(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R13_SHIFT)) & LMEM_PSCRMR_R13_MASK)
#define LMEM_PSCRMR_R12_MASK                     (0xC0U)
#define LMEM_PSCRMR_R12_SHIFT                    (6U)
#define LMEM_PSCRMR_R12(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R12_SHIFT)) & LMEM_PSCRMR_R12_MASK)
#define LMEM_PSCRMR_R11_MASK                     (0x300U)
#define LMEM_PSCRMR_R11_SHIFT                    (8U)
#define LMEM_PSCRMR_R11(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R11_SHIFT)) & LMEM_PSCRMR_R11_MASK)
#define LMEM_PSCRMR_R10_MASK                     (0xC00U)
#define LMEM_PSCRMR_R10_SHIFT                    (10U)
#define LMEM_PSCRMR_R10(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R10_SHIFT)) & LMEM_PSCRMR_R10_MASK)
#define LMEM_PSCRMR_R9_MASK                      (0x3000U)
#define LMEM_PSCRMR_R9_SHIFT                     (12U)
#define LMEM_PSCRMR_R9(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R9_SHIFT)) & LMEM_PSCRMR_R9_MASK)
#define LMEM_PSCRMR_R8_MASK                      (0xC000U)
#define LMEM_PSCRMR_R8_SHIFT                     (14U)
#define LMEM_PSCRMR_R8(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R8_SHIFT)) & LMEM_PSCRMR_R8_MASK)
#define LMEM_PSCRMR_R7_MASK                      (0x30000U)
#define LMEM_PSCRMR_R7_SHIFT                     (16U)
#define LMEM_PSCRMR_R7(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R7_SHIFT)) & LMEM_PSCRMR_R7_MASK)
#define LMEM_PSCRMR_R6_MASK                      (0xC0000U)
#define LMEM_PSCRMR_R6_SHIFT                     (18U)
#define LMEM_PSCRMR_R6(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R6_SHIFT)) & LMEM_PSCRMR_R6_MASK)
#define LMEM_PSCRMR_R5_MASK                      (0x300000U)
#define LMEM_PSCRMR_R5_SHIFT                     (20U)
#define LMEM_PSCRMR_R5(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R5_SHIFT)) & LMEM_PSCRMR_R5_MASK)
#define LMEM_PSCRMR_R4_MASK                      (0xC00000U)
#define LMEM_PSCRMR_R4_SHIFT                     (22U)
#define LMEM_PSCRMR_R4(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R4_SHIFT)) & LMEM_PSCRMR_R4_MASK)
#define LMEM_PSCRMR_R3_MASK                      (0x3000000U)
#define LMEM_PSCRMR_R3_SHIFT                     (24U)
#define LMEM_PSCRMR_R3(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R3_SHIFT)) & LMEM_PSCRMR_R3_MASK)
#define LMEM_PSCRMR_R2_MASK                      (0xC000000U)
#define LMEM_PSCRMR_R2_SHIFT                     (26U)
#define LMEM_PSCRMR_R2(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R2_SHIFT)) & LMEM_PSCRMR_R2_MASK)
#define LMEM_PSCRMR_R1_MASK                      (0x30000000U)
#define LMEM_PSCRMR_R1_SHIFT                     (28U)
#define LMEM_PSCRMR_R1(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R1_SHIFT)) & LMEM_PSCRMR_R1_MASK)
#define LMEM_PSCRMR_R0_MASK                      (0xC0000000U)
#define LMEM_PSCRMR_R0_SHIFT                     (30U)
#define LMEM_PSCRMR_R0(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PSCRMR_R0_SHIFT)) & LMEM_PSCRMR_R0_MASK)


/*!
 * @}
 */ /* end of group LMEM_Register_Masks */


/* LMEM - Peripheral instance base addresses */
/** Peripheral LMEM base address */
#define LMEM_BASE                                (0xE0082000u)
/** Peripheral LMEM base pointer */
#define LMEM                                     ((LMEM_Type *)LMEM_BASE)
/** Array initializer of LMEM peripheral base addresses */
#define LMEM_BASE_ADDRS                          { LMEM_BASE }
/** Array initializer of LMEM peripheral base pointers */
#define LMEM_BASE_PTRS                           { LMEM }

/*!
 * @}
 */ /* end of group LMEM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPTMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Peripheral_Access_Layer LPTMR Peripheral Access Layer
 * @{
 */

/** LPTMR - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSR;                               /**< Low Power Timer Control Status Register, offset: 0x0 */
  __IO uint32_t PSR;                               /**< Low Power Timer Prescale Register, offset: 0x4 */
  __IO uint32_t CMR;                               /**< Low Power Timer Compare Register, offset: 0x8 */
  __IO uint32_t CNR;                               /**< Low Power Timer Counter Register, offset: 0xC */
} LPTMR_Type;

/* ----------------------------------------------------------------------------
   -- LPTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Masks LPTMR Register Masks
 * @{
 */

/*! @name CSR - Low Power Timer Control Status Register */
#define LPTMR_CSR_TEN_MASK                       (0x1U)
#define LPTMR_CSR_TEN_SHIFT                      (0U)
#define LPTMR_CSR_TEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TEN_SHIFT)) & LPTMR_CSR_TEN_MASK)
#define LPTMR_CSR_TMS_MASK                       (0x2U)
#define LPTMR_CSR_TMS_SHIFT                      (1U)
#define LPTMR_CSR_TMS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TMS_SHIFT)) & LPTMR_CSR_TMS_MASK)
#define LPTMR_CSR_TFC_MASK                       (0x4U)
#define LPTMR_CSR_TFC_SHIFT                      (2U)
#define LPTMR_CSR_TFC(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TFC_SHIFT)) & LPTMR_CSR_TFC_MASK)
#define LPTMR_CSR_TPP_MASK                       (0x8U)
#define LPTMR_CSR_TPP_SHIFT                      (3U)
#define LPTMR_CSR_TPP(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TPP_SHIFT)) & LPTMR_CSR_TPP_MASK)
#define LPTMR_CSR_TPS_MASK                       (0x30U)
#define LPTMR_CSR_TPS_SHIFT                      (4U)
#define LPTMR_CSR_TPS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TPS_SHIFT)) & LPTMR_CSR_TPS_MASK)
#define LPTMR_CSR_TIE_MASK                       (0x40U)
#define LPTMR_CSR_TIE_SHIFT                      (6U)
#define LPTMR_CSR_TIE(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TIE_SHIFT)) & LPTMR_CSR_TIE_MASK)
#define LPTMR_CSR_TCF_MASK                       (0x80U)
#define LPTMR_CSR_TCF_SHIFT                      (7U)
#define LPTMR_CSR_TCF(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TCF_SHIFT)) & LPTMR_CSR_TCF_MASK)

/*! @name PSR - Low Power Timer Prescale Register */
#define LPTMR_PSR_PCS_MASK                       (0x3U)
#define LPTMR_PSR_PCS_SHIFT                      (0U)
#define LPTMR_PSR_PCS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PCS_SHIFT)) & LPTMR_PSR_PCS_MASK)
#define LPTMR_PSR_PBYP_MASK                      (0x4U)
#define LPTMR_PSR_PBYP_SHIFT                     (2U)
#define LPTMR_PSR_PBYP(x)                        (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PBYP_SHIFT)) & LPTMR_PSR_PBYP_MASK)
#define LPTMR_PSR_PRESCALE_MASK                  (0x78U)
#define LPTMR_PSR_PRESCALE_SHIFT                 (3U)
#define LPTMR_PSR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PRESCALE_SHIFT)) & LPTMR_PSR_PRESCALE_MASK)

/*! @name CMR - Low Power Timer Compare Register */
#define LPTMR_CMR_COMPARE_MASK                   (0xFFFFU)
#define LPTMR_CMR_COMPARE_SHIFT                  (0U)
#define LPTMR_CMR_COMPARE(x)                     (((uint32_t)(((uint32_t)(x)) << LPTMR_CMR_COMPARE_SHIFT)) & LPTMR_CMR_COMPARE_MASK)

/*! @name CNR - Low Power Timer Counter Register */
#define LPTMR_CNR_COUNTER_MASK                   (0xFFFFU)
#define LPTMR_CNR_COUNTER_SHIFT                  (0U)
#define LPTMR_CNR_COUNTER(x)                     (((uint32_t)(((uint32_t)(x)) << LPTMR_CNR_COUNTER_SHIFT)) & LPTMR_CNR_COUNTER_MASK)


/*!
 * @}
 */ /* end of group LPTMR_Register_Masks */


/* LPTMR - Peripheral instance base addresses */
/** Peripheral LPTMR0 base address */
#define LPTMR0_BASE                              (0x40040000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_Type *)LPTMR0_BASE)
/** Peripheral LPTMR1 base address */
#define LPTMR1_BASE                              (0x40044000u)
/** Peripheral LPTMR1 base pointer */
#define LPTMR1                                   ((LPTMR_Type *)LPTMR1_BASE)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE, LPTMR1_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0, LPTMR1 }
/** Interrupt vectors for the LPTMR peripheral type */
#define LPTMR_IRQS                               { LPTMR0_LPTMR1_IRQn, LPTMR0_LPTMR1_IRQn }

/*!
 * @}
 */ /* end of group LPTMR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPUART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Peripheral_Access_Layer LPUART Peripheral Access Layer
 * @{
 */

/** LPUART - Register Layout Typedef */
typedef struct {
  __IO uint32_t BAUD;                              /**< LPUART Baud Rate Register, offset: 0x0 */
  __IO uint32_t STAT;                              /**< LPUART Status Register, offset: 0x4 */
  __IO uint32_t CTRL;                              /**< LPUART Control Register, offset: 0x8 */
  __IO uint32_t DATA;                              /**< LPUART Data Register, offset: 0xC */
  __IO uint32_t MATCH;                             /**< LPUART Match Address Register, offset: 0x10 */
  __IO uint32_t MODIR;                             /**< LPUART Modem IrDA Register, offset: 0x14 */
  __IO uint32_t FIFO;                              /**< LPUART FIFO Register, offset: 0x18 */
  __IO uint32_t WATER;                             /**< LPUART Watermark Register, offset: 0x1C */
} LPUART_Type;

/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/*! @name BAUD - LPUART Baud Rate Register */
#define LPUART_BAUD_SBR_MASK                     (0x1FFFU)
#define LPUART_BAUD_SBR_SHIFT                    (0U)
#define LPUART_BAUD_SBR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBR_SHIFT)) & LPUART_BAUD_SBR_MASK)
#define LPUART_BAUD_SBNS_MASK                    (0x2000U)
#define LPUART_BAUD_SBNS_SHIFT                   (13U)
#define LPUART_BAUD_SBNS(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBNS_SHIFT)) & LPUART_BAUD_SBNS_MASK)
#define LPUART_BAUD_RXEDGIE_MASK                 (0x4000U)
#define LPUART_BAUD_RXEDGIE_SHIFT                (14U)
#define LPUART_BAUD_RXEDGIE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RXEDGIE_SHIFT)) & LPUART_BAUD_RXEDGIE_MASK)
#define LPUART_BAUD_LBKDIE_MASK                  (0x8000U)
#define LPUART_BAUD_LBKDIE_SHIFT                 (15U)
#define LPUART_BAUD_LBKDIE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_LBKDIE_SHIFT)) & LPUART_BAUD_LBKDIE_MASK)
#define LPUART_BAUD_RESYNCDIS_MASK               (0x10000U)
#define LPUART_BAUD_RESYNCDIS_SHIFT              (16U)
#define LPUART_BAUD_RESYNCDIS(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RESYNCDIS_SHIFT)) & LPUART_BAUD_RESYNCDIS_MASK)
#define LPUART_BAUD_BOTHEDGE_MASK                (0x20000U)
#define LPUART_BAUD_BOTHEDGE_SHIFT               (17U)
#define LPUART_BAUD_BOTHEDGE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_BOTHEDGE_SHIFT)) & LPUART_BAUD_BOTHEDGE_MASK)
#define LPUART_BAUD_MATCFG_MASK                  (0xC0000U)
#define LPUART_BAUD_MATCFG_SHIFT                 (18U)
#define LPUART_BAUD_MATCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MATCFG_SHIFT)) & LPUART_BAUD_MATCFG_MASK)
#define LPUART_BAUD_RDMAE_MASK                   (0x200000U)
#define LPUART_BAUD_RDMAE_SHIFT                  (21U)
#define LPUART_BAUD_RDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RDMAE_SHIFT)) & LPUART_BAUD_RDMAE_MASK)
#define LPUART_BAUD_TDMAE_MASK                   (0x800000U)
#define LPUART_BAUD_TDMAE_SHIFT                  (23U)
#define LPUART_BAUD_TDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_TDMAE_SHIFT)) & LPUART_BAUD_TDMAE_MASK)
#define LPUART_BAUD_OSR_MASK                     (0x1F000000U)
#define LPUART_BAUD_OSR_SHIFT                    (24U)
#define LPUART_BAUD_OSR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_OSR_SHIFT)) & LPUART_BAUD_OSR_MASK)
#define LPUART_BAUD_M10_MASK                     (0x20000000U)
#define LPUART_BAUD_M10_SHIFT                    (29U)
#define LPUART_BAUD_M10(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_M10_SHIFT)) & LPUART_BAUD_M10_MASK)
#define LPUART_BAUD_MAEN2_MASK                   (0x40000000U)
#define LPUART_BAUD_MAEN2_SHIFT                  (30U)
#define LPUART_BAUD_MAEN2(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN2_SHIFT)) & LPUART_BAUD_MAEN2_MASK)
#define LPUART_BAUD_MAEN1_MASK                   (0x80000000U)
#define LPUART_BAUD_MAEN1_SHIFT                  (31U)
#define LPUART_BAUD_MAEN1(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN1_SHIFT)) & LPUART_BAUD_MAEN1_MASK)

/*! @name STAT - LPUART Status Register */
#define LPUART_STAT_MA2F_MASK                    (0x4000U)
#define LPUART_STAT_MA2F_SHIFT                   (14U)
#define LPUART_STAT_MA2F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA2F_SHIFT)) & LPUART_STAT_MA2F_MASK)
#define LPUART_STAT_MA1F_MASK                    (0x8000U)
#define LPUART_STAT_MA1F_SHIFT                   (15U)
#define LPUART_STAT_MA1F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA1F_SHIFT)) & LPUART_STAT_MA1F_MASK)
#define LPUART_STAT_PF_MASK                      (0x10000U)
#define LPUART_STAT_PF_SHIFT                     (16U)
#define LPUART_STAT_PF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_PF_SHIFT)) & LPUART_STAT_PF_MASK)
#define LPUART_STAT_FE_MASK                      (0x20000U)
#define LPUART_STAT_FE_SHIFT                     (17U)
#define LPUART_STAT_FE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_FE_SHIFT)) & LPUART_STAT_FE_MASK)
#define LPUART_STAT_NF_MASK                      (0x40000U)
#define LPUART_STAT_NF_SHIFT                     (18U)
#define LPUART_STAT_NF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_NF_SHIFT)) & LPUART_STAT_NF_MASK)
#define LPUART_STAT_OR_MASK                      (0x80000U)
#define LPUART_STAT_OR_SHIFT                     (19U)
#define LPUART_STAT_OR(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_OR_SHIFT)) & LPUART_STAT_OR_MASK)
#define LPUART_STAT_IDLE_MASK                    (0x100000U)
#define LPUART_STAT_IDLE_SHIFT                   (20U)
#define LPUART_STAT_IDLE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_IDLE_SHIFT)) & LPUART_STAT_IDLE_MASK)
#define LPUART_STAT_RDRF_MASK                    (0x200000U)
#define LPUART_STAT_RDRF_SHIFT                   (21U)
#define LPUART_STAT_RDRF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RDRF_SHIFT)) & LPUART_STAT_RDRF_MASK)
#define LPUART_STAT_TC_MASK                      (0x400000U)
#define LPUART_STAT_TC_SHIFT                     (22U)
#define LPUART_STAT_TC(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TC_SHIFT)) & LPUART_STAT_TC_MASK)
#define LPUART_STAT_TDRE_MASK                    (0x800000U)
#define LPUART_STAT_TDRE_SHIFT                   (23U)
#define LPUART_STAT_TDRE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TDRE_SHIFT)) & LPUART_STAT_TDRE_MASK)
#define LPUART_STAT_RAF_MASK                     (0x1000000U)
#define LPUART_STAT_RAF_SHIFT                    (24U)
#define LPUART_STAT_RAF(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RAF_SHIFT)) & LPUART_STAT_RAF_MASK)
#define LPUART_STAT_LBKDE_MASK                   (0x2000000U)
#define LPUART_STAT_LBKDE_SHIFT                  (25U)
#define LPUART_STAT_LBKDE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDE_SHIFT)) & LPUART_STAT_LBKDE_MASK)
#define LPUART_STAT_BRK13_MASK                   (0x4000000U)
#define LPUART_STAT_BRK13_SHIFT                  (26U)
#define LPUART_STAT_BRK13(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_BRK13_SHIFT)) & LPUART_STAT_BRK13_MASK)
#define LPUART_STAT_RWUID_MASK                   (0x8000000U)
#define LPUART_STAT_RWUID_SHIFT                  (27U)
#define LPUART_STAT_RWUID(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RWUID_SHIFT)) & LPUART_STAT_RWUID_MASK)
#define LPUART_STAT_RXINV_MASK                   (0x10000000U)
#define LPUART_STAT_RXINV_SHIFT                  (28U)
#define LPUART_STAT_RXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXINV_SHIFT)) & LPUART_STAT_RXINV_MASK)
#define LPUART_STAT_MSBF_MASK                    (0x20000000U)
#define LPUART_STAT_MSBF_SHIFT                   (29U)
#define LPUART_STAT_MSBF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MSBF_SHIFT)) & LPUART_STAT_MSBF_MASK)
#define LPUART_STAT_RXEDGIF_MASK                 (0x40000000U)
#define LPUART_STAT_RXEDGIF_SHIFT                (30U)
#define LPUART_STAT_RXEDGIF(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXEDGIF_SHIFT)) & LPUART_STAT_RXEDGIF_MASK)
#define LPUART_STAT_LBKDIF_MASK                  (0x80000000U)
#define LPUART_STAT_LBKDIF_SHIFT                 (31U)
#define LPUART_STAT_LBKDIF(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDIF_SHIFT)) & LPUART_STAT_LBKDIF_MASK)

/*! @name CTRL - LPUART Control Register */
#define LPUART_CTRL_PT_MASK                      (0x1U)
#define LPUART_CTRL_PT_SHIFT                     (0U)
#define LPUART_CTRL_PT(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PT_SHIFT)) & LPUART_CTRL_PT_MASK)
#define LPUART_CTRL_PE_MASK                      (0x2U)
#define LPUART_CTRL_PE_SHIFT                     (1U)
#define LPUART_CTRL_PE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PE_SHIFT)) & LPUART_CTRL_PE_MASK)
#define LPUART_CTRL_ILT_MASK                     (0x4U)
#define LPUART_CTRL_ILT_SHIFT                    (2U)
#define LPUART_CTRL_ILT(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILT_SHIFT)) & LPUART_CTRL_ILT_MASK)
#define LPUART_CTRL_WAKE_MASK                    (0x8U)
#define LPUART_CTRL_WAKE_SHIFT                   (3U)
#define LPUART_CTRL_WAKE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_WAKE_SHIFT)) & LPUART_CTRL_WAKE_MASK)
#define LPUART_CTRL_M_MASK                       (0x10U)
#define LPUART_CTRL_M_SHIFT                      (4U)
#define LPUART_CTRL_M(x)                         (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M_SHIFT)) & LPUART_CTRL_M_MASK)
#define LPUART_CTRL_RSRC_MASK                    (0x20U)
#define LPUART_CTRL_RSRC_SHIFT                   (5U)
#define LPUART_CTRL_RSRC(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RSRC_SHIFT)) & LPUART_CTRL_RSRC_MASK)
#define LPUART_CTRL_DOZEEN_MASK                  (0x40U)
#define LPUART_CTRL_DOZEEN_SHIFT                 (6U)
#define LPUART_CTRL_DOZEEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_DOZEEN_SHIFT)) & LPUART_CTRL_DOZEEN_MASK)
#define LPUART_CTRL_LOOPS_MASK                   (0x80U)
#define LPUART_CTRL_LOOPS_SHIFT                  (7U)
#define LPUART_CTRL_LOOPS(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_LOOPS_SHIFT)) & LPUART_CTRL_LOOPS_MASK)
#define LPUART_CTRL_IDLECFG_MASK                 (0x700U)
#define LPUART_CTRL_IDLECFG_SHIFT                (8U)
#define LPUART_CTRL_IDLECFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_IDLECFG_SHIFT)) & LPUART_CTRL_IDLECFG_MASK)
#define LPUART_CTRL_MA2IE_MASK                   (0x4000U)
#define LPUART_CTRL_MA2IE_SHIFT                  (14U)
#define LPUART_CTRL_MA2IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA2IE_SHIFT)) & LPUART_CTRL_MA2IE_MASK)
#define LPUART_CTRL_MA1IE_MASK                   (0x8000U)
#define LPUART_CTRL_MA1IE_SHIFT                  (15U)
#define LPUART_CTRL_MA1IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA1IE_SHIFT)) & LPUART_CTRL_MA1IE_MASK)
#define LPUART_CTRL_SBK_MASK                     (0x10000U)
#define LPUART_CTRL_SBK_SHIFT                    (16U)
#define LPUART_CTRL_SBK(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_SBK_SHIFT)) & LPUART_CTRL_SBK_MASK)
#define LPUART_CTRL_RWU_MASK                     (0x20000U)
#define LPUART_CTRL_RWU_SHIFT                    (17U)
#define LPUART_CTRL_RWU(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RWU_SHIFT)) & LPUART_CTRL_RWU_MASK)
#define LPUART_CTRL_RE_MASK                      (0x40000U)
#define LPUART_CTRL_RE_SHIFT                     (18U)
#define LPUART_CTRL_RE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RE_SHIFT)) & LPUART_CTRL_RE_MASK)
#define LPUART_CTRL_TE_MASK                      (0x80000U)
#define LPUART_CTRL_TE_SHIFT                     (19U)
#define LPUART_CTRL_TE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TE_SHIFT)) & LPUART_CTRL_TE_MASK)
#define LPUART_CTRL_ILIE_MASK                    (0x100000U)
#define LPUART_CTRL_ILIE_SHIFT                   (20U)
#define LPUART_CTRL_ILIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILIE_SHIFT)) & LPUART_CTRL_ILIE_MASK)
#define LPUART_CTRL_RIE_MASK                     (0x200000U)
#define LPUART_CTRL_RIE_SHIFT                    (21U)
#define LPUART_CTRL_RIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RIE_SHIFT)) & LPUART_CTRL_RIE_MASK)
#define LPUART_CTRL_TCIE_MASK                    (0x400000U)
#define LPUART_CTRL_TCIE_SHIFT                   (22U)
#define LPUART_CTRL_TCIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TCIE_SHIFT)) & LPUART_CTRL_TCIE_MASK)
#define LPUART_CTRL_TIE_MASK                     (0x800000U)
#define LPUART_CTRL_TIE_SHIFT                    (23U)
#define LPUART_CTRL_TIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TIE_SHIFT)) & LPUART_CTRL_TIE_MASK)
#define LPUART_CTRL_PEIE_MASK                    (0x1000000U)
#define LPUART_CTRL_PEIE_SHIFT                   (24U)
#define LPUART_CTRL_PEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PEIE_SHIFT)) & LPUART_CTRL_PEIE_MASK)
#define LPUART_CTRL_FEIE_MASK                    (0x2000000U)
#define LPUART_CTRL_FEIE_SHIFT                   (25U)
#define LPUART_CTRL_FEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_FEIE_SHIFT)) & LPUART_CTRL_FEIE_MASK)
#define LPUART_CTRL_NEIE_MASK                    (0x4000000U)
#define LPUART_CTRL_NEIE_SHIFT                   (26U)
#define LPUART_CTRL_NEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_NEIE_SHIFT)) & LPUART_CTRL_NEIE_MASK)
#define LPUART_CTRL_ORIE_MASK                    (0x8000000U)
#define LPUART_CTRL_ORIE_SHIFT                   (27U)
#define LPUART_CTRL_ORIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ORIE_SHIFT)) & LPUART_CTRL_ORIE_MASK)
#define LPUART_CTRL_TXINV_MASK                   (0x10000000U)
#define LPUART_CTRL_TXINV_SHIFT                  (28U)
#define LPUART_CTRL_TXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXINV_SHIFT)) & LPUART_CTRL_TXINV_MASK)
#define LPUART_CTRL_TXDIR_MASK                   (0x20000000U)
#define LPUART_CTRL_TXDIR_SHIFT                  (29U)
#define LPUART_CTRL_TXDIR(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXDIR_SHIFT)) & LPUART_CTRL_TXDIR_MASK)
#define LPUART_CTRL_R9T8_MASK                    (0x40000000U)
#define LPUART_CTRL_R9T8_SHIFT                   (30U)
#define LPUART_CTRL_R9T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R9T8_SHIFT)) & LPUART_CTRL_R9T8_MASK)
#define LPUART_CTRL_R8T9_MASK                    (0x80000000U)
#define LPUART_CTRL_R8T9_SHIFT                   (31U)
#define LPUART_CTRL_R8T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R8T9_SHIFT)) & LPUART_CTRL_R8T9_MASK)

/*! @name DATA - LPUART Data Register */
#define LPUART_DATA_R0T0_MASK                    (0x1U)
#define LPUART_DATA_R0T0_SHIFT                   (0U)
#define LPUART_DATA_R0T0(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R0T0_SHIFT)) & LPUART_DATA_R0T0_MASK)
#define LPUART_DATA_R1T1_MASK                    (0x2U)
#define LPUART_DATA_R1T1_SHIFT                   (1U)
#define LPUART_DATA_R1T1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R1T1_SHIFT)) & LPUART_DATA_R1T1_MASK)
#define LPUART_DATA_R2T2_MASK                    (0x4U)
#define LPUART_DATA_R2T2_SHIFT                   (2U)
#define LPUART_DATA_R2T2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R2T2_SHIFT)) & LPUART_DATA_R2T2_MASK)
#define LPUART_DATA_R3T3_MASK                    (0x8U)
#define LPUART_DATA_R3T3_SHIFT                   (3U)
#define LPUART_DATA_R3T3(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R3T3_SHIFT)) & LPUART_DATA_R3T3_MASK)
#define LPUART_DATA_R4T4_MASK                    (0x10U)
#define LPUART_DATA_R4T4_SHIFT                   (4U)
#define LPUART_DATA_R4T4(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R4T4_SHIFT)) & LPUART_DATA_R4T4_MASK)
#define LPUART_DATA_R5T5_MASK                    (0x20U)
#define LPUART_DATA_R5T5_SHIFT                   (5U)
#define LPUART_DATA_R5T5(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R5T5_SHIFT)) & LPUART_DATA_R5T5_MASK)
#define LPUART_DATA_R6T6_MASK                    (0x40U)
#define LPUART_DATA_R6T6_SHIFT                   (6U)
#define LPUART_DATA_R6T6(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R6T6_SHIFT)) & LPUART_DATA_R6T6_MASK)
#define LPUART_DATA_R7T7_MASK                    (0x80U)
#define LPUART_DATA_R7T7_SHIFT                   (7U)
#define LPUART_DATA_R7T7(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R7T7_SHIFT)) & LPUART_DATA_R7T7_MASK)
#define LPUART_DATA_R8T8_MASK                    (0x100U)
#define LPUART_DATA_R8T8_SHIFT                   (8U)
#define LPUART_DATA_R8T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R8T8_SHIFT)) & LPUART_DATA_R8T8_MASK)
#define LPUART_DATA_R9T9_MASK                    (0x200U)
#define LPUART_DATA_R9T9_SHIFT                   (9U)
#define LPUART_DATA_R9T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R9T9_SHIFT)) & LPUART_DATA_R9T9_MASK)
#define LPUART_DATA_IDLINE_MASK                  (0x800U)
#define LPUART_DATA_IDLINE_SHIFT                 (11U)
#define LPUART_DATA_IDLINE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_IDLINE_SHIFT)) & LPUART_DATA_IDLINE_MASK)
#define LPUART_DATA_RXEMPT_MASK                  (0x1000U)
#define LPUART_DATA_RXEMPT_SHIFT                 (12U)
#define LPUART_DATA_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_RXEMPT_SHIFT)) & LPUART_DATA_RXEMPT_MASK)
#define LPUART_DATA_FRETSC_MASK                  (0x2000U)
#define LPUART_DATA_FRETSC_SHIFT                 (13U)
#define LPUART_DATA_FRETSC(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_FRETSC_SHIFT)) & LPUART_DATA_FRETSC_MASK)
#define LPUART_DATA_PARITYE_MASK                 (0x4000U)
#define LPUART_DATA_PARITYE_SHIFT                (14U)
#define LPUART_DATA_PARITYE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_PARITYE_SHIFT)) & LPUART_DATA_PARITYE_MASK)
#define LPUART_DATA_NOISY_MASK                   (0x8000U)
#define LPUART_DATA_NOISY_SHIFT                  (15U)
#define LPUART_DATA_NOISY(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_NOISY_SHIFT)) & LPUART_DATA_NOISY_MASK)

/*! @name MATCH - LPUART Match Address Register */
#define LPUART_MATCH_MA1_MASK                    (0x3FFU)
#define LPUART_MATCH_MA1_SHIFT                   (0U)
#define LPUART_MATCH_MA1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA1_SHIFT)) & LPUART_MATCH_MA1_MASK)
#define LPUART_MATCH_MA2_MASK                    (0x3FF0000U)
#define LPUART_MATCH_MA2_SHIFT                   (16U)
#define LPUART_MATCH_MA2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA2_SHIFT)) & LPUART_MATCH_MA2_MASK)

/*! @name MODIR - LPUART Modem IrDA Register */
#define LPUART_MODIR_TXCTSE_MASK                 (0x1U)
#define LPUART_MODIR_TXCTSE_SHIFT                (0U)
#define LPUART_MODIR_TXCTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSE_SHIFT)) & LPUART_MODIR_TXCTSE_MASK)
#define LPUART_MODIR_TXRTSE_MASK                 (0x2U)
#define LPUART_MODIR_TXRTSE_SHIFT                (1U)
#define LPUART_MODIR_TXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSE_SHIFT)) & LPUART_MODIR_TXRTSE_MASK)
#define LPUART_MODIR_TXRTSPOL_MASK               (0x4U)
#define LPUART_MODIR_TXRTSPOL_SHIFT              (2U)
#define LPUART_MODIR_TXRTSPOL(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSPOL_SHIFT)) & LPUART_MODIR_TXRTSPOL_MASK)
#define LPUART_MODIR_RXRTSE_MASK                 (0x8U)
#define LPUART_MODIR_RXRTSE_SHIFT                (3U)
#define LPUART_MODIR_RXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RXRTSE_SHIFT)) & LPUART_MODIR_RXRTSE_MASK)
#define LPUART_MODIR_TXCTSC_MASK                 (0x10U)
#define LPUART_MODIR_TXCTSC_SHIFT                (4U)
#define LPUART_MODIR_TXCTSC(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSC_SHIFT)) & LPUART_MODIR_TXCTSC_MASK)
#define LPUART_MODIR_TXCTSSRC_MASK               (0x20U)
#define LPUART_MODIR_TXCTSSRC_SHIFT              (5U)
#define LPUART_MODIR_TXCTSSRC(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSSRC_SHIFT)) & LPUART_MODIR_TXCTSSRC_MASK)
#define LPUART_MODIR_RTSWATER_MASK               (0xFF00U)
#define LPUART_MODIR_RTSWATER_SHIFT              (8U)
#define LPUART_MODIR_RTSWATER(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RTSWATER_SHIFT)) & LPUART_MODIR_RTSWATER_MASK)
#define LPUART_MODIR_TNP_MASK                    (0x30000U)
#define LPUART_MODIR_TNP_SHIFT                   (16U)
#define LPUART_MODIR_TNP(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TNP_SHIFT)) & LPUART_MODIR_TNP_MASK)
#define LPUART_MODIR_IREN_MASK                   (0x40000U)
#define LPUART_MODIR_IREN_SHIFT                  (18U)
#define LPUART_MODIR_IREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_IREN_SHIFT)) & LPUART_MODIR_IREN_MASK)

/*! @name FIFO - LPUART FIFO Register */
#define LPUART_FIFO_RXFIFOSIZE_MASK              (0x7U)
#define LPUART_FIFO_RXFIFOSIZE_SHIFT             (0U)
#define LPUART_FIFO_RXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFIFOSIZE_SHIFT)) & LPUART_FIFO_RXFIFOSIZE_MASK)
#define LPUART_FIFO_RXFE_MASK                    (0x8U)
#define LPUART_FIFO_RXFE_SHIFT                   (3U)
#define LPUART_FIFO_RXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFE_SHIFT)) & LPUART_FIFO_RXFE_MASK)
#define LPUART_FIFO_TXFIFOSIZE_MASK              (0x70U)
#define LPUART_FIFO_TXFIFOSIZE_SHIFT             (4U)
#define LPUART_FIFO_TXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFIFOSIZE_SHIFT)) & LPUART_FIFO_TXFIFOSIZE_MASK)
#define LPUART_FIFO_TXFE_MASK                    (0x80U)
#define LPUART_FIFO_TXFE_SHIFT                   (7U)
#define LPUART_FIFO_TXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFE_SHIFT)) & LPUART_FIFO_TXFE_MASK)
#define LPUART_FIFO_RXUFE_MASK                   (0x100U)
#define LPUART_FIFO_RXUFE_SHIFT                  (8U)
#define LPUART_FIFO_RXUFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUFE_SHIFT)) & LPUART_FIFO_RXUFE_MASK)
#define LPUART_FIFO_TXOFE_MASK                   (0x200U)
#define LPUART_FIFO_TXOFE_SHIFT                  (9U)
#define LPUART_FIFO_TXOFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOFE_SHIFT)) & LPUART_FIFO_TXOFE_MASK)
#define LPUART_FIFO_RXIDEN_MASK                  (0x1C00U)
#define LPUART_FIFO_RXIDEN_SHIFT                 (10U)
#define LPUART_FIFO_RXIDEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXIDEN_SHIFT)) & LPUART_FIFO_RXIDEN_MASK)
#define LPUART_FIFO_RXFLUSH_MASK                 (0x4000U)
#define LPUART_FIFO_RXFLUSH_SHIFT                (14U)
#define LPUART_FIFO_RXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFLUSH_SHIFT)) & LPUART_FIFO_RXFLUSH_MASK)
#define LPUART_FIFO_TXFLUSH_MASK                 (0x8000U)
#define LPUART_FIFO_TXFLUSH_SHIFT                (15U)
#define LPUART_FIFO_TXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFLUSH_SHIFT)) & LPUART_FIFO_TXFLUSH_MASK)
#define LPUART_FIFO_RXUF_MASK                    (0x10000U)
#define LPUART_FIFO_RXUF_SHIFT                   (16U)
#define LPUART_FIFO_RXUF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUF_SHIFT)) & LPUART_FIFO_RXUF_MASK)
#define LPUART_FIFO_TXOF_MASK                    (0x20000U)
#define LPUART_FIFO_TXOF_SHIFT                   (17U)
#define LPUART_FIFO_TXOF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOF_SHIFT)) & LPUART_FIFO_TXOF_MASK)
#define LPUART_FIFO_RXEMPT_MASK                  (0x400000U)
#define LPUART_FIFO_RXEMPT_SHIFT                 (22U)
#define LPUART_FIFO_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXEMPT_SHIFT)) & LPUART_FIFO_RXEMPT_MASK)
#define LPUART_FIFO_TXEMPT_MASK                  (0x800000U)
#define LPUART_FIFO_TXEMPT_SHIFT                 (23U)
#define LPUART_FIFO_TXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXEMPT_SHIFT)) & LPUART_FIFO_TXEMPT_MASK)

/*! @name WATER - LPUART Watermark Register */
#define LPUART_WATER_TXWATER_MASK                (0xFFU)
#define LPUART_WATER_TXWATER_SHIFT               (0U)
#define LPUART_WATER_TXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXWATER_SHIFT)) & LPUART_WATER_TXWATER_MASK)
#define LPUART_WATER_TXCOUNT_MASK                (0xFF00U)
#define LPUART_WATER_TXCOUNT_SHIFT               (8U)
#define LPUART_WATER_TXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXCOUNT_SHIFT)) & LPUART_WATER_TXCOUNT_MASK)
#define LPUART_WATER_RXWATER_MASK                (0xFF0000U)
#define LPUART_WATER_RXWATER_SHIFT               (16U)
#define LPUART_WATER_RXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXWATER_SHIFT)) & LPUART_WATER_RXWATER_MASK)
#define LPUART_WATER_RXCOUNT_MASK                (0xFF000000U)
#define LPUART_WATER_RXCOUNT_SHIFT               (24U)
#define LPUART_WATER_RXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXCOUNT_SHIFT)) & LPUART_WATER_RXCOUNT_MASK)


/*!
 * @}
 */ /* end of group LPUART_Register_Masks */


/* LPUART - Peripheral instance base addresses */
/** Peripheral LPUART0 base address */
#define LPUART0_BASE                             (0x400C4000u)
/** Peripheral LPUART0 base pointer */
#define LPUART0                                  ((LPUART_Type *)LPUART0_BASE)
/** Peripheral LPUART1 base address */
#define LPUART1_BASE                             (0x400C5000u)
/** Peripheral LPUART1 base pointer */
#define LPUART1                                  ((LPUART_Type *)LPUART1_BASE)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE                             (0x400C6000u)
/** Peripheral LPUART2 base pointer */
#define LPUART2                                  ((LPUART_Type *)LPUART2_BASE)
/** Peripheral LPUART3 base address */
#define LPUART3_BASE                             (0x400C7000u)
/** Peripheral LPUART3 base pointer */
#define LPUART3                                  ((LPUART_Type *)LPUART3_BASE)
/** Peripheral LPUART4 base address */
#define LPUART4_BASE                             (0x400D6000u)
/** Peripheral LPUART4 base pointer */
#define LPUART4                                  ((LPUART_Type *)LPUART4_BASE)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                        { LPUART0_BASE, LPUART1_BASE, LPUART2_BASE, LPUART3_BASE, LPUART4_BASE }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                         { LPUART0, LPUART1, LPUART2, LPUART3, LPUART4 }
/** Interrupt vectors for the LPUART peripheral type */
#define LPUART_RX_TX_IRQS                        { LPUART0_IRQn, LPUART1_IRQn, LPUART2_IRQn, LPUART3_IRQn, LPUART4_IRQn }
#define LPUART_ERR_IRQS                          { LPUART0_IRQn, LPUART1_IRQn, LPUART2_IRQn, LPUART3_IRQn, LPUART4_IRQn }

/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Peripheral_Access_Layer MCG Peripheral Access Layer
 * @{
 */

/** MCG - Register Layout Typedef */
typedef struct {
  __IO uint8_t C1;                                 /**< MCG Control 1 Register, offset: 0x0 */
  __IO uint8_t C2;                                 /**< MCG Control 2 Register, offset: 0x1 */
  __IO uint8_t C3;                                 /**< MCG Control 3 Register, offset: 0x2 */
  __IO uint8_t C4;                                 /**< MCG Control 4 Register, offset: 0x3 */
  __IO uint8_t C5;                                 /**< MCG Control 5 Register, offset: 0x4 */
  __IO uint8_t C6;                                 /**< MCG Control 6 Register, offset: 0x5 */
  __IO uint8_t S;                                  /**< MCG Status Register, offset: 0x6 */
       uint8_t RESERVED_0[1];
  __IO uint8_t SC;                                 /**< MCG Status and Control Register, offset: 0x8 */
       uint8_t RESERVED_1[1];
  __IO uint8_t ATCVH;                              /**< MCG Auto Trim Compare Value High Register, offset: 0xA */
  __IO uint8_t ATCVL;                              /**< MCG Auto Trim Compare Value Low Register, offset: 0xB */
  __IO uint8_t C7;                                 /**< MCG Control 7 Register, offset: 0xC */
  __IO uint8_t C8;                                 /**< MCG Control 8 Register, offset: 0xD */
} MCG_Type;

/* ----------------------------------------------------------------------------
   -- MCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Masks MCG Register Masks
 * @{
 */

/*! @name C1 - MCG Control 1 Register */
#define MCG_C1_IREFSTEN_MASK                     (0x1U)
#define MCG_C1_IREFSTEN_SHIFT                    (0U)
#define MCG_C1_IREFSTEN(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_C1_IREFSTEN_SHIFT)) & MCG_C1_IREFSTEN_MASK)
#define MCG_C1_IRCLKEN_MASK                      (0x2U)
#define MCG_C1_IRCLKEN_SHIFT                     (1U)
#define MCG_C1_IRCLKEN(x)                        (((uint8_t)(((uint8_t)(x)) << MCG_C1_IRCLKEN_SHIFT)) & MCG_C1_IRCLKEN_MASK)
#define MCG_C1_IREFS_MASK                        (0x4U)
#define MCG_C1_IREFS_SHIFT                       (2U)
#define MCG_C1_IREFS(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C1_IREFS_SHIFT)) & MCG_C1_IREFS_MASK)
#define MCG_C1_FRDIV_MASK                        (0x38U)
#define MCG_C1_FRDIV_SHIFT                       (3U)
#define MCG_C1_FRDIV(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C1_FRDIV_SHIFT)) & MCG_C1_FRDIV_MASK)
#define MCG_C1_CLKS_MASK                         (0xC0U)
#define MCG_C1_CLKS_SHIFT                        (6U)
#define MCG_C1_CLKS(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C1_CLKS_SHIFT)) & MCG_C1_CLKS_MASK)

/*! @name C2 - MCG Control 2 Register */
#define MCG_C2_IRCS_MASK                         (0x1U)
#define MCG_C2_IRCS_SHIFT                        (0U)
#define MCG_C2_IRCS(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C2_IRCS_SHIFT)) & MCG_C2_IRCS_MASK)
#define MCG_C2_LP_MASK                           (0x2U)
#define MCG_C2_LP_SHIFT                          (1U)
#define MCG_C2_LP(x)                             (((uint8_t)(((uint8_t)(x)) << MCG_C2_LP_SHIFT)) & MCG_C2_LP_MASK)
#define MCG_C2_EREFS_MASK                        (0x4U)
#define MCG_C2_EREFS_SHIFT                       (2U)
#define MCG_C2_EREFS(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C2_EREFS_SHIFT)) & MCG_C2_EREFS_MASK)
#define MCG_C2_HGO_MASK                          (0x8U)
#define MCG_C2_HGO_SHIFT                         (3U)
#define MCG_C2_HGO(x)                            (((uint8_t)(((uint8_t)(x)) << MCG_C2_HGO_SHIFT)) & MCG_C2_HGO_MASK)
#define MCG_C2_RANGE_MASK                        (0x30U)
#define MCG_C2_RANGE_SHIFT                       (4U)
#define MCG_C2_RANGE(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C2_RANGE_SHIFT)) & MCG_C2_RANGE_MASK)
#define MCG_C2_FCFTRIM_MASK                      (0x40U)
#define MCG_C2_FCFTRIM_SHIFT                     (6U)
#define MCG_C2_FCFTRIM(x)                        (((uint8_t)(((uint8_t)(x)) << MCG_C2_FCFTRIM_SHIFT)) & MCG_C2_FCFTRIM_MASK)
#define MCG_C2_LOCRE0_MASK                       (0x80U)
#define MCG_C2_LOCRE0_SHIFT                      (7U)
#define MCG_C2_LOCRE0(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C2_LOCRE0_SHIFT)) & MCG_C2_LOCRE0_MASK)

/*! @name C3 - MCG Control 3 Register */
#define MCG_C3_SCTRIM_MASK                       (0xFFU)
#define MCG_C3_SCTRIM_SHIFT                      (0U)
#define MCG_C3_SCTRIM(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C3_SCTRIM_SHIFT)) & MCG_C3_SCTRIM_MASK)

/*! @name C4 - MCG Control 4 Register */
#define MCG_C4_SCFTRIM_MASK                      (0x1U)
#define MCG_C4_SCFTRIM_SHIFT                     (0U)
#define MCG_C4_SCFTRIM(x)                        (((uint8_t)(((uint8_t)(x)) << MCG_C4_SCFTRIM_SHIFT)) & MCG_C4_SCFTRIM_MASK)
#define MCG_C4_FCTRIM_MASK                       (0x1EU)
#define MCG_C4_FCTRIM_SHIFT                      (1U)
#define MCG_C4_FCTRIM(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C4_FCTRIM_SHIFT)) & MCG_C4_FCTRIM_MASK)
#define MCG_C4_DRST_DRS_MASK                     (0x60U)
#define MCG_C4_DRST_DRS_SHIFT                    (5U)
#define MCG_C4_DRST_DRS(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_C4_DRST_DRS_SHIFT)) & MCG_C4_DRST_DRS_MASK)
#define MCG_C4_DMX32_MASK                        (0x80U)
#define MCG_C4_DMX32_SHIFT                       (7U)
#define MCG_C4_DMX32(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C4_DMX32_SHIFT)) & MCG_C4_DMX32_MASK)

/*! @name C5 - MCG Control 5 Register */
#define MCG_C5_PRDIV_MASK                        (0x7U)
#define MCG_C5_PRDIV_SHIFT                       (0U)
#define MCG_C5_PRDIV(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C5_PRDIV_SHIFT)) & MCG_C5_PRDIV_MASK)
#define MCG_C5_PLLSTEN_MASK                      (0x20U)
#define MCG_C5_PLLSTEN_SHIFT                     (5U)
#define MCG_C5_PLLSTEN(x)                        (((uint8_t)(((uint8_t)(x)) << MCG_C5_PLLSTEN_SHIFT)) & MCG_C5_PLLSTEN_MASK)
#define MCG_C5_PLLCLKEN_MASK                     (0x40U)
#define MCG_C5_PLLCLKEN_SHIFT                    (6U)
#define MCG_C5_PLLCLKEN(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_C5_PLLCLKEN_SHIFT)) & MCG_C5_PLLCLKEN_MASK)

/*! @name C6 - MCG Control 6 Register */
#define MCG_C6_VDIV_MASK                         (0x1FU)
#define MCG_C6_VDIV_SHIFT                        (0U)
#define MCG_C6_VDIV(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C6_VDIV_SHIFT)) & MCG_C6_VDIV_MASK)
#define MCG_C6_CME0_MASK                         (0x20U)
#define MCG_C6_CME0_SHIFT                        (5U)
#define MCG_C6_CME0(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C6_CME0_SHIFT)) & MCG_C6_CME0_MASK)
#define MCG_C6_PLLS_MASK                         (0x40U)
#define MCG_C6_PLLS_SHIFT                        (6U)
#define MCG_C6_PLLS(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C6_PLLS_SHIFT)) & MCG_C6_PLLS_MASK)
#define MCG_C6_LOLIE0_MASK                       (0x80U)
#define MCG_C6_LOLIE0_SHIFT                      (7U)
#define MCG_C6_LOLIE0(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C6_LOLIE0_SHIFT)) & MCG_C6_LOLIE0_MASK)

/*! @name S - MCG Status Register */
#define MCG_S_IRCST_MASK                         (0x1U)
#define MCG_S_IRCST_SHIFT                        (0U)
#define MCG_S_IRCST(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_S_IRCST_SHIFT)) & MCG_S_IRCST_MASK)
#define MCG_S_OSCINIT0_MASK                      (0x2U)
#define MCG_S_OSCINIT0_SHIFT                     (1U)
#define MCG_S_OSCINIT0(x)                        (((uint8_t)(((uint8_t)(x)) << MCG_S_OSCINIT0_SHIFT)) & MCG_S_OSCINIT0_MASK)
#define MCG_S_CLKST_MASK                         (0xCU)
#define MCG_S_CLKST_SHIFT                        (2U)
#define MCG_S_CLKST(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_S_CLKST_SHIFT)) & MCG_S_CLKST_MASK)
#define MCG_S_IREFST_MASK                        (0x10U)
#define MCG_S_IREFST_SHIFT                       (4U)
#define MCG_S_IREFST(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_S_IREFST_SHIFT)) & MCG_S_IREFST_MASK)
#define MCG_S_PLLST_MASK                         (0x20U)
#define MCG_S_PLLST_SHIFT                        (5U)
#define MCG_S_PLLST(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_S_PLLST_SHIFT)) & MCG_S_PLLST_MASK)
#define MCG_S_LOCK0_MASK                         (0x40U)
#define MCG_S_LOCK0_SHIFT                        (6U)
#define MCG_S_LOCK0(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_S_LOCK0_SHIFT)) & MCG_S_LOCK0_MASK)
#define MCG_S_LOLS0_MASK                         (0x80U)
#define MCG_S_LOLS0_SHIFT                        (7U)
#define MCG_S_LOLS0(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_S_LOLS0_SHIFT)) & MCG_S_LOLS0_MASK)

/*! @name SC - MCG Status and Control Register */
#define MCG_SC_LOCS0_MASK                        (0x1U)
#define MCG_SC_LOCS0_SHIFT                       (0U)
#define MCG_SC_LOCS0(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_SC_LOCS0_SHIFT)) & MCG_SC_LOCS0_MASK)
#define MCG_SC_FCRDIV_MASK                       (0xEU)
#define MCG_SC_FCRDIV_SHIFT                      (1U)
#define MCG_SC_FCRDIV(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_SC_FCRDIV_SHIFT)) & MCG_SC_FCRDIV_MASK)
#define MCG_SC_FLTPRSRV_MASK                     (0x10U)
#define MCG_SC_FLTPRSRV_SHIFT                    (4U)
#define MCG_SC_FLTPRSRV(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_SC_FLTPRSRV_SHIFT)) & MCG_SC_FLTPRSRV_MASK)
#define MCG_SC_ATMF_MASK                         (0x20U)
#define MCG_SC_ATMF_SHIFT                        (5U)
#define MCG_SC_ATMF(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_SC_ATMF_SHIFT)) & MCG_SC_ATMF_MASK)
#define MCG_SC_ATMS_MASK                         (0x40U)
#define MCG_SC_ATMS_SHIFT                        (6U)
#define MCG_SC_ATMS(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_SC_ATMS_SHIFT)) & MCG_SC_ATMS_MASK)
#define MCG_SC_ATME_MASK                         (0x80U)
#define MCG_SC_ATME_SHIFT                        (7U)
#define MCG_SC_ATME(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_SC_ATME_SHIFT)) & MCG_SC_ATME_MASK)

/*! @name ATCVH - MCG Auto Trim Compare Value High Register */
#define MCG_ATCVH_ATCVH_MASK                     (0xFFU)
#define MCG_ATCVH_ATCVH_SHIFT                    (0U)
#define MCG_ATCVH_ATCVH(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_ATCVH_ATCVH_SHIFT)) & MCG_ATCVH_ATCVH_MASK)

/*! @name ATCVL - MCG Auto Trim Compare Value Low Register */
#define MCG_ATCVL_ATCVL_MASK                     (0xFFU)
#define MCG_ATCVL_ATCVL_SHIFT                    (0U)
#define MCG_ATCVL_ATCVL(x)                       (((uint8_t)(((uint8_t)(x)) << MCG_ATCVL_ATCVL_SHIFT)) & MCG_ATCVL_ATCVL_MASK)

/*! @name C7 - MCG Control 7 Register */
#define MCG_C7_OSCSEL_MASK                       (0x3U)
#define MCG_C7_OSCSEL_SHIFT                      (0U)
#define MCG_C7_OSCSEL(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C7_OSCSEL_SHIFT)) & MCG_C7_OSCSEL_MASK)

/*! @name C8 - MCG Control 8 Register */
#define MCG_C8_LOCS1_MASK                        (0x1U)
#define MCG_C8_LOCS1_SHIFT                       (0U)
#define MCG_C8_LOCS1(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C8_LOCS1_SHIFT)) & MCG_C8_LOCS1_MASK)
#define MCG_C8_CME1_MASK                         (0x20U)
#define MCG_C8_CME1_SHIFT                        (5U)
#define MCG_C8_CME1(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C8_CME1_SHIFT)) & MCG_C8_CME1_MASK)
#define MCG_C8_LOLRE_MASK                        (0x40U)
#define MCG_C8_LOLRE_SHIFT                       (6U)
#define MCG_C8_LOLRE(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C8_LOLRE_SHIFT)) & MCG_C8_LOLRE_MASK)
#define MCG_C8_LOCRE1_MASK                       (0x80U)
#define MCG_C8_LOCRE1_SHIFT                      (7U)
#define MCG_C8_LOCRE1(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C8_LOCRE1_SHIFT)) & MCG_C8_LOCRE1_MASK)


/*!
 * @}
 */ /* end of group MCG_Register_Masks */


/* MCG - Peripheral instance base addresses */
/** Peripheral MCG base address */
#define MCG_BASE                                 (0x40064000u)
/** Peripheral MCG base pointer */
#define MCG                                      ((MCG_Type *)MCG_BASE)
/** Array initializer of MCG peripheral base addresses */
#define MCG_BASE_ADDRS                           { MCG_BASE }
/** Array initializer of MCG peripheral base pointers */
#define MCG_BASE_PTRS                            { MCG }
/** Interrupt vectors for the MCG peripheral type */
#define MCG_IRQS                                 { MCG_IRQn }
/* MCG C5[PLLCLKEN0] backward compatibility */
#define MCG_C5_PLLCLKEN0_MASK         (MCG_C5_PLLCLKEN_MASK)
#define MCG_C5_PLLCLKEN0_SHIFT        (MCG_C5_PLLCLKEN_SHIFT)
#define MCG_C5_PLLCLKEN0_WIDTH        (MCG_C5_PLLCLKEN_WIDTH)
#define MCG_C5_PLLCLKEN0(x)           (MCG_C5_PLLCLKEN(x))

/* MCG C5[PLLSTEN0] backward compatibility */
#define MCG_C5_PLLSTEN0_MASK         (MCG_C5_PLLSTEN_MASK)
#define MCG_C5_PLLSTEN0_SHIFT        (MCG_C5_PLLSTEN_SHIFT)
#define MCG_C5_PLLSTEN0_WIDTH        (MCG_C5_PLLSTEN_WIDTH)
#define MCG_C5_PLLSTEN0(x)           (MCG_C5_PLLSTEN(x))

/* MCG C5[PRDIV0] backward compatibility */
#define MCG_C5_PRDIV0_MASK         (MCG_C5_PRDIV_MASK)
#define MCG_C5_PRDIV0_SHIFT        (MCG_C5_PRDIV_SHIFT)
#define MCG_C5_PRDIV0_WIDTH        (MCG_C5_PRDIV_WIDTH)
#define MCG_C5_PRDIV0(x)           (MCG_C5_PRDIV(x))

/* MCG C6[VDIV0] backward compatibility */
#define MCG_C6_VDIV0_MASK         (MCG_C6_VDIV_MASK)
#define MCG_C6_VDIV0_SHIFT        (MCG_C6_VDIV_SHIFT)
#define MCG_C6_VDIV0_WIDTH        (MCG_C6_VDIV_WIDTH)
#define MCG_C6_VDIV0(x)           (MCG_C6_VDIV(x))


/*!
 * @}
 */ /* end of group MCG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[8];
  __I  uint16_t PLASC;                             /**< Crossbar Switch (AXBS) Slave Configuration, offset: 0x8 */
  __I  uint16_t PLAMC;                             /**< Crossbar Switch (AXBS) Master Configuration, offset: 0xA */
  __IO uint32_t CR;                                /**< Control Register, offset: 0xC */
  __IO uint32_t ISCR;                              /**< Interrupt Status Register, offset: 0x10 */
       uint8_t RESERVED_1[12];
  __I  uint32_t FADR;                              /**< Fault address register, offset: 0x20 */
  __I  uint32_t FATR;                              /**< Fault attributes register, offset: 0x24 */
  __I  uint32_t FDR;                               /**< Fault data register, offset: 0x28 */
       uint8_t RESERVED_2[4];
  __IO uint32_t PID;                               /**< Process ID register, offset: 0x30 */
       uint8_t RESERVED_3[12];
  __IO uint32_t CPO;                               /**< Compute Operation Control Register, offset: 0x40 */
} MCM_Type;

/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/*! @name PLASC - Crossbar Switch (AXBS) Slave Configuration */
#define MCM_PLASC_ASC_MASK                       (0xFFU)
#define MCM_PLASC_ASC_SHIFT                      (0U)
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_ASC_SHIFT)) & MCM_PLASC_ASC_MASK)

/*! @name PLAMC - Crossbar Switch (AXBS) Master Configuration */
#define MCM_PLAMC_AMC_MASK                       (0xFFU)
#define MCM_PLAMC_AMC_SHIFT                      (0U)
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLAMC_AMC_SHIFT)) & MCM_PLAMC_AMC_MASK)

/*! @name CR - Control Register */
#define MCM_CR_SRAMUAP_MASK                      (0x3000000U)
#define MCM_CR_SRAMUAP_SHIFT                     (24U)
#define MCM_CR_SRAMUAP(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CR_SRAMUAP_SHIFT)) & MCM_CR_SRAMUAP_MASK)
#define MCM_CR_SRAMUWP_MASK                      (0x4000000U)
#define MCM_CR_SRAMUWP_SHIFT                     (26U)
#define MCM_CR_SRAMUWP(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CR_SRAMUWP_SHIFT)) & MCM_CR_SRAMUWP_MASK)
#define MCM_CR_SRAMLAP_MASK                      (0x30000000U)
#define MCM_CR_SRAMLAP_SHIFT                     (28U)
#define MCM_CR_SRAMLAP(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CR_SRAMLAP_SHIFT)) & MCM_CR_SRAMLAP_MASK)
#define MCM_CR_SRAMLWP_MASK                      (0x40000000U)
#define MCM_CR_SRAMLWP_SHIFT                     (30U)
#define MCM_CR_SRAMLWP(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CR_SRAMLWP_SHIFT)) & MCM_CR_SRAMLWP_MASK)

/*! @name ISCR - Interrupt Status Register */
#define MCM_ISCR_FIOC_MASK                       (0x100U)
#define MCM_ISCR_FIOC_SHIFT                      (8U)
#define MCM_ISCR_FIOC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIOC_SHIFT)) & MCM_ISCR_FIOC_MASK)
#define MCM_ISCR_FDZC_MASK                       (0x200U)
#define MCM_ISCR_FDZC_SHIFT                      (9U)
#define MCM_ISCR_FDZC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FDZC_SHIFT)) & MCM_ISCR_FDZC_MASK)
#define MCM_ISCR_FOFC_MASK                       (0x400U)
#define MCM_ISCR_FOFC_SHIFT                      (10U)
#define MCM_ISCR_FOFC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FOFC_SHIFT)) & MCM_ISCR_FOFC_MASK)
#define MCM_ISCR_FUFC_MASK                       (0x800U)
#define MCM_ISCR_FUFC_SHIFT                      (11U)
#define MCM_ISCR_FUFC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FUFC_SHIFT)) & MCM_ISCR_FUFC_MASK)
#define MCM_ISCR_FIXC_MASK                       (0x1000U)
#define MCM_ISCR_FIXC_SHIFT                      (12U)
#define MCM_ISCR_FIXC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIXC_SHIFT)) & MCM_ISCR_FIXC_MASK)
#define MCM_ISCR_FIDC_MASK                       (0x8000U)
#define MCM_ISCR_FIDC_SHIFT                      (15U)
#define MCM_ISCR_FIDC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIDC_SHIFT)) & MCM_ISCR_FIDC_MASK)
#define MCM_ISCR_FIOCE_MASK                      (0x1000000U)
#define MCM_ISCR_FIOCE_SHIFT                     (24U)
#define MCM_ISCR_FIOCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIOCE_SHIFT)) & MCM_ISCR_FIOCE_MASK)
#define MCM_ISCR_FDZCE_MASK                      (0x2000000U)
#define MCM_ISCR_FDZCE_SHIFT                     (25U)
#define MCM_ISCR_FDZCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FDZCE_SHIFT)) & MCM_ISCR_FDZCE_MASK)
#define MCM_ISCR_FOFCE_MASK                      (0x4000000U)
#define MCM_ISCR_FOFCE_SHIFT                     (26U)
#define MCM_ISCR_FOFCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FOFCE_SHIFT)) & MCM_ISCR_FOFCE_MASK)
#define MCM_ISCR_FUFCE_MASK                      (0x8000000U)
#define MCM_ISCR_FUFCE_SHIFT                     (27U)
#define MCM_ISCR_FUFCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FUFCE_SHIFT)) & MCM_ISCR_FUFCE_MASK)
#define MCM_ISCR_FIXCE_MASK                      (0x10000000U)
#define MCM_ISCR_FIXCE_SHIFT                     (28U)
#define MCM_ISCR_FIXCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIXCE_SHIFT)) & MCM_ISCR_FIXCE_MASK)
#define MCM_ISCR_FIDCE_MASK                      (0x80000000U)
#define MCM_ISCR_FIDCE_SHIFT                     (31U)
#define MCM_ISCR_FIDCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIDCE_SHIFT)) & MCM_ISCR_FIDCE_MASK)

/*! @name FADR - Fault address register */
#define MCM_FADR_ADDRESS_MASK                    (0xFFFFFFFFU)
#define MCM_FADR_ADDRESS_SHIFT                   (0U)
#define MCM_FADR_ADDRESS(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_FADR_ADDRESS_SHIFT)) & MCM_FADR_ADDRESS_MASK)

/*! @name FATR - Fault attributes register */
#define MCM_FATR_BEDA_MASK                       (0x1U)
#define MCM_FATR_BEDA_SHIFT                      (0U)
#define MCM_FATR_BEDA(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BEDA_SHIFT)) & MCM_FATR_BEDA_MASK)
#define MCM_FATR_BEMD_MASK                       (0x2U)
#define MCM_FATR_BEMD_SHIFT                      (1U)
#define MCM_FATR_BEMD(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BEMD_SHIFT)) & MCM_FATR_BEMD_MASK)
#define MCM_FATR_BESZ_MASK                       (0x30U)
#define MCM_FATR_BESZ_SHIFT                      (4U)
#define MCM_FATR_BESZ(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BESZ_SHIFT)) & MCM_FATR_BESZ_MASK)
#define MCM_FATR_BEWT_MASK                       (0x80U)
#define MCM_FATR_BEWT_SHIFT                      (7U)
#define MCM_FATR_BEWT(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BEWT_SHIFT)) & MCM_FATR_BEWT_MASK)
#define MCM_FATR_BEMN_MASK                       (0xF00U)
#define MCM_FATR_BEMN_SHIFT                      (8U)
#define MCM_FATR_BEMN(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BEMN_SHIFT)) & MCM_FATR_BEMN_MASK)
#define MCM_FATR_BEOVR_MASK                      (0x80000000U)
#define MCM_FATR_BEOVR_SHIFT                     (31U)
#define MCM_FATR_BEOVR(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_FATR_BEOVR_SHIFT)) & MCM_FATR_BEOVR_MASK)

/*! @name FDR - Fault data register */
#define MCM_FDR_DATA_MASK                        (0xFFFFFFFFU)
#define MCM_FDR_DATA_SHIFT                       (0U)
#define MCM_FDR_DATA(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_FDR_DATA_SHIFT)) & MCM_FDR_DATA_MASK)

/*! @name PID - Process ID register */
#define MCM_PID_PID_MASK                         (0xFFU)
#define MCM_PID_PID_SHIFT                        (0U)
#define MCM_PID_PID(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_PID_PID_SHIFT)) & MCM_PID_PID_MASK)

/*! @name CPO - Compute Operation Control Register */
#define MCM_CPO_CPOREQ_MASK                      (0x1U)
#define MCM_CPO_CPOREQ_SHIFT                     (0U)
#define MCM_CPO_CPOREQ(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOREQ_SHIFT)) & MCM_CPO_CPOREQ_MASK)
#define MCM_CPO_CPOACK_MASK                      (0x2U)
#define MCM_CPO_CPOACK_SHIFT                     (1U)
#define MCM_CPO_CPOACK(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOACK_SHIFT)) & MCM_CPO_CPOACK_MASK)
#define MCM_CPO_CPOWOI_MASK                      (0x4U)
#define MCM_CPO_CPOWOI_SHIFT                     (2U)
#define MCM_CPO_CPOWOI(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOWOI_SHIFT)) & MCM_CPO_CPOWOI_MASK)


/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xE0080000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_Type *)MCM_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }
/** Interrupt vectors for the MCM peripheral type */
#define MCM_IRQS                                 { MCM_IRQn }

/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MPU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MPU_Peripheral_Access_Layer MPU Peripheral Access Layer
 * @{
 */

/** MPU - Register Layout Typedef */
typedef struct {
  __IO uint32_t CESR;                              /**< Control/Error Status Register, offset: 0x0 */
       uint8_t RESERVED_0[12];
  struct {                                         /* offset: 0x10, array step: 0x8 */
    __I  uint32_t EAR;                               /**< Error Address Register, slave port n, array offset: 0x10, array step: 0x8 */
    __I  uint32_t EDR;                               /**< Error Detail Register, slave port n, array offset: 0x14, array step: 0x8 */
  } SP[5];
       uint8_t RESERVED_1[968];
  __IO uint32_t WORD[12][4];                       /**< Region Descriptor n, Word 0..Region Descriptor n, Word 3, array offset: 0x400, array step: index*0x10, index2*0x4 */
       uint8_t RESERVED_2[832];
  __IO uint32_t RGDAAC[12];                        /**< Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4 */
} MPU_Type;

/* ----------------------------------------------------------------------------
   -- MPU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MPU_Register_Masks MPU Register Masks
 * @{
 */

/*! @name CESR - Control/Error Status Register */
#define MPU_CESR_VLD_MASK                        (0x1U)
#define MPU_CESR_VLD_SHIFT                       (0U)
#define MPU_CESR_VLD(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_VLD_SHIFT)) & MPU_CESR_VLD_MASK)
#define MPU_CESR_NRGD_MASK                       (0xF00U)
#define MPU_CESR_NRGD_SHIFT                      (8U)
#define MPU_CESR_NRGD(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_CESR_NRGD_SHIFT)) & MPU_CESR_NRGD_MASK)
#define MPU_CESR_NSP_MASK                        (0xF000U)
#define MPU_CESR_NSP_SHIFT                       (12U)
#define MPU_CESR_NSP(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_NSP_SHIFT)) & MPU_CESR_NSP_MASK)
#define MPU_CESR_HRL_MASK                        (0xF0000U)
#define MPU_CESR_HRL_SHIFT                       (16U)
#define MPU_CESR_HRL(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_HRL_SHIFT)) & MPU_CESR_HRL_MASK)
#define MPU_CESR_SPERR_MASK                      (0xF8000000U)
#define MPU_CESR_SPERR_SHIFT                     (27U)
#define MPU_CESR_SPERR(x)                        (((uint32_t)(((uint32_t)(x)) << MPU_CESR_SPERR_SHIFT)) & MPU_CESR_SPERR_MASK)

/*! @name EAR - Error Address Register, slave port n */
#define MPU_EAR_EADDR_MASK                       (0xFFFFFFFFU)
#define MPU_EAR_EADDR_SHIFT                      (0U)
#define MPU_EAR_EADDR(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_EAR_EADDR_SHIFT)) & MPU_EAR_EADDR_MASK)

/* The count of MPU_EAR */
#define MPU_EAR_COUNT                            (5U)

/*! @name EDR - Error Detail Register, slave port n */
#define MPU_EDR_ERW_MASK                         (0x1U)
#define MPU_EDR_ERW_SHIFT                        (0U)
#define MPU_EDR_ERW(x)                           (((uint32_t)(((uint32_t)(x)) << MPU_EDR_ERW_SHIFT)) & MPU_EDR_ERW_MASK)
#define MPU_EDR_EATTR_MASK                       (0xEU)
#define MPU_EDR_EATTR_SHIFT                      (1U)
#define MPU_EDR_EATTR(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EATTR_SHIFT)) & MPU_EDR_EATTR_MASK)
#define MPU_EDR_EMN_MASK                         (0xF0U)
#define MPU_EDR_EMN_SHIFT                        (4U)
#define MPU_EDR_EMN(x)                           (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EMN_SHIFT)) & MPU_EDR_EMN_MASK)
#define MPU_EDR_EPID_MASK                        (0xFF00U)
#define MPU_EDR_EPID_SHIFT                       (8U)
#define MPU_EDR_EPID(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EPID_SHIFT)) & MPU_EDR_EPID_MASK)
#define MPU_EDR_EACD_MASK                        (0xFFFF0000U)
#define MPU_EDR_EACD_SHIFT                       (16U)
#define MPU_EDR_EACD(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EACD_SHIFT)) & MPU_EDR_EACD_MASK)

/* The count of MPU_EDR */
#define MPU_EDR_COUNT                            (5U)

/*! @name WORD - Region Descriptor n, Word 0..Region Descriptor n, Word 3 */
#define MPU_WORD_VLD_MASK                        (0x1U)
#define MPU_WORD_VLD_SHIFT                       (0U)
#define MPU_WORD_VLD(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_WORD_VLD_SHIFT)) & MPU_WORD_VLD_MASK)
#define MPU_WORD_M0UM_MASK                       (0x7U)
#define MPU_WORD_M0UM_SHIFT                      (0U)
#define MPU_WORD_M0UM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M0UM_SHIFT)) & MPU_WORD_M0UM_MASK)
#define MPU_WORD_M0SM_MASK                       (0x18U)
#define MPU_WORD_M0SM_SHIFT                      (3U)
#define MPU_WORD_M0SM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M0SM_SHIFT)) & MPU_WORD_M0SM_MASK)
#define MPU_WORD_M0PE_MASK                       (0x20U)
#define MPU_WORD_M0PE_SHIFT                      (5U)
#define MPU_WORD_M0PE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M0PE_SHIFT)) & MPU_WORD_M0PE_MASK)
#define MPU_WORD_ENDADDR_MASK                    (0xFFFFFFE0U)
#define MPU_WORD_ENDADDR_SHIFT                   (5U)
#define MPU_WORD_ENDADDR(x)                      (((uint32_t)(((uint32_t)(x)) << MPU_WORD_ENDADDR_SHIFT)) & MPU_WORD_ENDADDR_MASK)
#define MPU_WORD_SRTADDR_MASK                    (0xFFFFFFE0U)
#define MPU_WORD_SRTADDR_SHIFT                   (5U)
#define MPU_WORD_SRTADDR(x)                      (((uint32_t)(((uint32_t)(x)) << MPU_WORD_SRTADDR_SHIFT)) & MPU_WORD_SRTADDR_MASK)
#define MPU_WORD_M1UM_MASK                       (0x1C0U)
#define MPU_WORD_M1UM_SHIFT                      (6U)
#define MPU_WORD_M1UM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M1UM_SHIFT)) & MPU_WORD_M1UM_MASK)
#define MPU_WORD_M1SM_MASK                       (0x600U)
#define MPU_WORD_M1SM_SHIFT                      (9U)
#define MPU_WORD_M1SM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M1SM_SHIFT)) & MPU_WORD_M1SM_MASK)
#define MPU_WORD_M1PE_MASK                       (0x800U)
#define MPU_WORD_M1PE_SHIFT                      (11U)
#define MPU_WORD_M1PE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M1PE_SHIFT)) & MPU_WORD_M1PE_MASK)
#define MPU_WORD_M2UM_MASK                       (0x7000U)
#define MPU_WORD_M2UM_SHIFT                      (12U)
#define MPU_WORD_M2UM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M2UM_SHIFT)) & MPU_WORD_M2UM_MASK)
#define MPU_WORD_M2SM_MASK                       (0x18000U)
#define MPU_WORD_M2SM_SHIFT                      (15U)
#define MPU_WORD_M2SM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M2SM_SHIFT)) & MPU_WORD_M2SM_MASK)
#define MPU_WORD_PIDMASK_MASK                    (0xFF0000U)
#define MPU_WORD_PIDMASK_SHIFT                   (16U)
#define MPU_WORD_PIDMASK(x)                      (((uint32_t)(((uint32_t)(x)) << MPU_WORD_PIDMASK_SHIFT)) & MPU_WORD_PIDMASK_MASK)
#define MPU_WORD_M2PE_MASK                       (0x20000U)
#define MPU_WORD_M2PE_SHIFT                      (17U)
#define MPU_WORD_M2PE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M2PE_SHIFT)) & MPU_WORD_M2PE_MASK)
#define MPU_WORD_M3UM_MASK                       (0x1C0000U)
#define MPU_WORD_M3UM_SHIFT                      (18U)
#define MPU_WORD_M3UM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M3UM_SHIFT)) & MPU_WORD_M3UM_MASK)
#define MPU_WORD_M3SM_MASK                       (0x600000U)
#define MPU_WORD_M3SM_SHIFT                      (21U)
#define MPU_WORD_M3SM(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M3SM_SHIFT)) & MPU_WORD_M3SM_MASK)
#define MPU_WORD_M3PE_MASK                       (0x800000U)
#define MPU_WORD_M3PE_SHIFT                      (23U)
#define MPU_WORD_M3PE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M3PE_SHIFT)) & MPU_WORD_M3PE_MASK)
#define MPU_WORD_PID_MASK                        (0xFF000000U)
#define MPU_WORD_PID_SHIFT                       (24U)
#define MPU_WORD_PID(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_WORD_PID_SHIFT)) & MPU_WORD_PID_MASK)
#define MPU_WORD_M4WE_MASK                       (0x1000000U)
#define MPU_WORD_M4WE_SHIFT                      (24U)
#define MPU_WORD_M4WE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M4WE_SHIFT)) & MPU_WORD_M4WE_MASK)
#define MPU_WORD_M4RE_MASK                       (0x2000000U)
#define MPU_WORD_M4RE_SHIFT                      (25U)
#define MPU_WORD_M4RE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M4RE_SHIFT)) & MPU_WORD_M4RE_MASK)
#define MPU_WORD_M5WE_MASK                       (0x4000000U)
#define MPU_WORD_M5WE_SHIFT                      (26U)
#define MPU_WORD_M5WE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M5WE_SHIFT)) & MPU_WORD_M5WE_MASK)
#define MPU_WORD_M5RE_MASK                       (0x8000000U)
#define MPU_WORD_M5RE_SHIFT                      (27U)
#define MPU_WORD_M5RE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M5RE_SHIFT)) & MPU_WORD_M5RE_MASK)
#define MPU_WORD_M6WE_MASK                       (0x10000000U)
#define MPU_WORD_M6WE_SHIFT                      (28U)
#define MPU_WORD_M6WE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M6WE_SHIFT)) & MPU_WORD_M6WE_MASK)
#define MPU_WORD_M6RE_MASK                       (0x20000000U)
#define MPU_WORD_M6RE_SHIFT                      (29U)
#define MPU_WORD_M6RE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M6RE_SHIFT)) & MPU_WORD_M6RE_MASK)
#define MPU_WORD_M7WE_MASK                       (0x40000000U)
#define MPU_WORD_M7WE_SHIFT                      (30U)
#define MPU_WORD_M7WE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M7WE_SHIFT)) & MPU_WORD_M7WE_MASK)
#define MPU_WORD_M7RE_MASK                       (0x80000000U)
#define MPU_WORD_M7RE_SHIFT                      (31U)
#define MPU_WORD_M7RE(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_WORD_M7RE_SHIFT)) & MPU_WORD_M7RE_MASK)

/* The count of MPU_WORD */
#define MPU_WORD_COUNT                           (12U)

/* The count of MPU_WORD */
#define MPU_WORD_COUNT2                          (4U)

/*! @name RGDAAC - Region Descriptor Alternate Access Control n */
#define MPU_RGDAAC_M0UM_MASK                     (0x7U)
#define MPU_RGDAAC_M0UM_SHIFT                    (0U)
#define MPU_RGDAAC_M0UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0UM_SHIFT)) & MPU_RGDAAC_M0UM_MASK)
#define MPU_RGDAAC_M0SM_MASK                     (0x18U)
#define MPU_RGDAAC_M0SM_SHIFT                    (3U)
#define MPU_RGDAAC_M0SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0SM_SHIFT)) & MPU_RGDAAC_M0SM_MASK)
#define MPU_RGDAAC_M0PE_MASK                     (0x20U)
#define MPU_RGDAAC_M0PE_SHIFT                    (5U)
#define MPU_RGDAAC_M0PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0PE_SHIFT)) & MPU_RGDAAC_M0PE_MASK)
#define MPU_RGDAAC_M1UM_MASK                     (0x1C0U)
#define MPU_RGDAAC_M1UM_SHIFT                    (6U)
#define MPU_RGDAAC_M1UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1UM_SHIFT)) & MPU_RGDAAC_M1UM_MASK)
#define MPU_RGDAAC_M1SM_MASK                     (0x600U)
#define MPU_RGDAAC_M1SM_SHIFT                    (9U)
#define MPU_RGDAAC_M1SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1SM_SHIFT)) & MPU_RGDAAC_M1SM_MASK)
#define MPU_RGDAAC_M1PE_MASK                     (0x800U)
#define MPU_RGDAAC_M1PE_SHIFT                    (11U)
#define MPU_RGDAAC_M1PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1PE_SHIFT)) & MPU_RGDAAC_M1PE_MASK)
#define MPU_RGDAAC_M2UM_MASK                     (0x7000U)
#define MPU_RGDAAC_M2UM_SHIFT                    (12U)
#define MPU_RGDAAC_M2UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M2UM_SHIFT)) & MPU_RGDAAC_M2UM_MASK)
#define MPU_RGDAAC_M2SM_MASK                     (0x18000U)
#define MPU_RGDAAC_M2SM_SHIFT                    (15U)
#define MPU_RGDAAC_M2SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M2SM_SHIFT)) & MPU_RGDAAC_M2SM_MASK)
#define MPU_RGDAAC_M2PE_MASK                     (0x20000U)
#define MPU_RGDAAC_M2PE_SHIFT                    (17U)
#define MPU_RGDAAC_M2PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M2PE_SHIFT)) & MPU_RGDAAC_M2PE_MASK)
#define MPU_RGDAAC_M3UM_MASK                     (0x1C0000U)
#define MPU_RGDAAC_M3UM_SHIFT                    (18U)
#define MPU_RGDAAC_M3UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M3UM_SHIFT)) & MPU_RGDAAC_M3UM_MASK)
#define MPU_RGDAAC_M3SM_MASK                     (0x600000U)
#define MPU_RGDAAC_M3SM_SHIFT                    (21U)
#define MPU_RGDAAC_M3SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M3SM_SHIFT)) & MPU_RGDAAC_M3SM_MASK)
#define MPU_RGDAAC_M3PE_MASK                     (0x800000U)
#define MPU_RGDAAC_M3PE_SHIFT                    (23U)
#define MPU_RGDAAC_M3PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M3PE_SHIFT)) & MPU_RGDAAC_M3PE_MASK)
#define MPU_RGDAAC_M4WE_MASK                     (0x1000000U)
#define MPU_RGDAAC_M4WE_SHIFT                    (24U)
#define MPU_RGDAAC_M4WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M4WE_SHIFT)) & MPU_RGDAAC_M4WE_MASK)
#define MPU_RGDAAC_M4RE_MASK                     (0x2000000U)
#define MPU_RGDAAC_M4RE_SHIFT                    (25U)
#define MPU_RGDAAC_M4RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M4RE_SHIFT)) & MPU_RGDAAC_M4RE_MASK)
#define MPU_RGDAAC_M5WE_MASK                     (0x4000000U)
#define MPU_RGDAAC_M5WE_SHIFT                    (26U)
#define MPU_RGDAAC_M5WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M5WE_SHIFT)) & MPU_RGDAAC_M5WE_MASK)
#define MPU_RGDAAC_M5RE_MASK                     (0x8000000U)
#define MPU_RGDAAC_M5RE_SHIFT                    (27U)
#define MPU_RGDAAC_M5RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M5RE_SHIFT)) & MPU_RGDAAC_M5RE_MASK)
#define MPU_RGDAAC_M6WE_MASK                     (0x10000000U)
#define MPU_RGDAAC_M6WE_SHIFT                    (28U)
#define MPU_RGDAAC_M6WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M6WE_SHIFT)) & MPU_RGDAAC_M6WE_MASK)
#define MPU_RGDAAC_M6RE_MASK                     (0x20000000U)
#define MPU_RGDAAC_M6RE_SHIFT                    (29U)
#define MPU_RGDAAC_M6RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M6RE_SHIFT)) & MPU_RGDAAC_M6RE_MASK)
#define MPU_RGDAAC_M7WE_MASK                     (0x40000000U)
#define MPU_RGDAAC_M7WE_SHIFT                    (30U)
#define MPU_RGDAAC_M7WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M7WE_SHIFT)) & MPU_RGDAAC_M7WE_MASK)
#define MPU_RGDAAC_M7RE_MASK                     (0x80000000U)
#define MPU_RGDAAC_M7RE_SHIFT                    (31U)
#define MPU_RGDAAC_M7RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M7RE_SHIFT)) & MPU_RGDAAC_M7RE_MASK)

/* The count of MPU_RGDAAC */
#define MPU_RGDAAC_COUNT                         (12U)


/*!
 * @}
 */ /* end of group MPU_Register_Masks */


/* MPU - Peripheral instance base addresses */
/** Peripheral MPU base address */
#define MPU_BASE                                 (0x4000D000u)
/** Peripheral MPU base pointer */
#define MPU                                      ((MPU_Type *)MPU_BASE)
/** Array initializer of MPU peripheral base addresses */
#define MPU_BASE_ADDRS                           { MPU_BASE }
/** Array initializer of MPU peripheral base pointers */
#define MPU_BASE_PTRS                            { MPU }

/*!
 * @}
 */ /* end of group MPU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- NV Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Peripheral_Access_Layer NV Peripheral Access Layer
 * @{
 */

/** NV - Register Layout Typedef */
typedef struct {
  __I  uint8_t BACKKEY3;                           /**< Backdoor Comparison Key 3., offset: 0x0 */
  __I  uint8_t BACKKEY2;                           /**< Backdoor Comparison Key 2., offset: 0x1 */
  __I  uint8_t BACKKEY1;                           /**< Backdoor Comparison Key 1., offset: 0x2 */
  __I  uint8_t BACKKEY0;                           /**< Backdoor Comparison Key 0., offset: 0x3 */
  __I  uint8_t BACKKEY7;                           /**< Backdoor Comparison Key 7., offset: 0x4 */
  __I  uint8_t BACKKEY6;                           /**< Backdoor Comparison Key 6., offset: 0x5 */
  __I  uint8_t BACKKEY5;                           /**< Backdoor Comparison Key 5., offset: 0x6 */
  __I  uint8_t BACKKEY4;                           /**< Backdoor Comparison Key 4., offset: 0x7 */
  __I  uint8_t FPROT3;                             /**< Non-volatile P-Flash Protection 1 - Low Register, offset: 0x8 */
  __I  uint8_t FPROT2;                             /**< Non-volatile P-Flash Protection 1 - High Register, offset: 0x9 */
  __I  uint8_t FPROT1;                             /**< Non-volatile P-Flash Protection 0 - Low Register, offset: 0xA */
  __I  uint8_t FPROT0;                             /**< Non-volatile P-Flash Protection 0 - High Register, offset: 0xB */
  __I  uint8_t FSEC;                               /**< Non-volatile Flash Security Register, offset: 0xC */
  __I  uint8_t FOPT;                               /**< Non-volatile Flash Option Register, offset: 0xD */
} NV_Type;

/* ----------------------------------------------------------------------------
   -- NV Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Masks NV Register Masks
 * @{
 */

/*! @name BACKKEY3 - Backdoor Comparison Key 3. */
#define NV_BACKKEY3_KEY_MASK                     (0xFFU)
#define NV_BACKKEY3_KEY_SHIFT                    (0U)
#define NV_BACKKEY3_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY3_KEY_SHIFT)) & NV_BACKKEY3_KEY_MASK)

/*! @name BACKKEY2 - Backdoor Comparison Key 2. */
#define NV_BACKKEY2_KEY_MASK                     (0xFFU)
#define NV_BACKKEY2_KEY_SHIFT                    (0U)
#define NV_BACKKEY2_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY2_KEY_SHIFT)) & NV_BACKKEY2_KEY_MASK)

/*! @name BACKKEY1 - Backdoor Comparison Key 1. */
#define NV_BACKKEY1_KEY_MASK                     (0xFFU)
#define NV_BACKKEY1_KEY_SHIFT                    (0U)
#define NV_BACKKEY1_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY1_KEY_SHIFT)) & NV_BACKKEY1_KEY_MASK)

/*! @name BACKKEY0 - Backdoor Comparison Key 0. */
#define NV_BACKKEY0_KEY_MASK                     (0xFFU)
#define NV_BACKKEY0_KEY_SHIFT                    (0U)
#define NV_BACKKEY0_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY0_KEY_SHIFT)) & NV_BACKKEY0_KEY_MASK)

/*! @name BACKKEY7 - Backdoor Comparison Key 7. */
#define NV_BACKKEY7_KEY_MASK                     (0xFFU)
#define NV_BACKKEY7_KEY_SHIFT                    (0U)
#define NV_BACKKEY7_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY7_KEY_SHIFT)) & NV_BACKKEY7_KEY_MASK)

/*! @name BACKKEY6 - Backdoor Comparison Key 6. */
#define NV_BACKKEY6_KEY_MASK                     (0xFFU)
#define NV_BACKKEY6_KEY_SHIFT                    (0U)
#define NV_BACKKEY6_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY6_KEY_SHIFT)) & NV_BACKKEY6_KEY_MASK)

/*! @name BACKKEY5 - Backdoor Comparison Key 5. */
#define NV_BACKKEY5_KEY_MASK                     (0xFFU)
#define NV_BACKKEY5_KEY_SHIFT                    (0U)
#define NV_BACKKEY5_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY5_KEY_SHIFT)) & NV_BACKKEY5_KEY_MASK)

/*! @name BACKKEY4 - Backdoor Comparison Key 4. */
#define NV_BACKKEY4_KEY_MASK                     (0xFFU)
#define NV_BACKKEY4_KEY_SHIFT                    (0U)
#define NV_BACKKEY4_KEY(x)                       (((uint8_t)(((uint8_t)(x)) << NV_BACKKEY4_KEY_SHIFT)) & NV_BACKKEY4_KEY_MASK)

/*! @name FPROT3 - Non-volatile P-Flash Protection 1 - Low Register */
#define NV_FPROT3_PROT_MASK                      (0xFFU)
#define NV_FPROT3_PROT_SHIFT                     (0U)
#define NV_FPROT3_PROT(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FPROT3_PROT_SHIFT)) & NV_FPROT3_PROT_MASK)

/*! @name FPROT2 - Non-volatile P-Flash Protection 1 - High Register */
#define NV_FPROT2_PROT_MASK                      (0xFFU)
#define NV_FPROT2_PROT_SHIFT                     (0U)
#define NV_FPROT2_PROT(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FPROT2_PROT_SHIFT)) & NV_FPROT2_PROT_MASK)

/*! @name FPROT1 - Non-volatile P-Flash Protection 0 - Low Register */
#define NV_FPROT1_PROT_MASK                      (0xFFU)
#define NV_FPROT1_PROT_SHIFT                     (0U)
#define NV_FPROT1_PROT(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FPROT1_PROT_SHIFT)) & NV_FPROT1_PROT_MASK)

/*! @name FPROT0 - Non-volatile P-Flash Protection 0 - High Register */
#define NV_FPROT0_PROT_MASK                      (0xFFU)
#define NV_FPROT0_PROT_SHIFT                     (0U)
#define NV_FPROT0_PROT(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FPROT0_PROT_SHIFT)) & NV_FPROT0_PROT_MASK)

/*! @name FSEC - Non-volatile Flash Security Register */
#define NV_FSEC_SEC_MASK                         (0x3U)
#define NV_FSEC_SEC_SHIFT                        (0U)
#define NV_FSEC_SEC(x)                           (((uint8_t)(((uint8_t)(x)) << NV_FSEC_SEC_SHIFT)) & NV_FSEC_SEC_MASK)
#define NV_FSEC_FSLACC_MASK                      (0xCU)
#define NV_FSEC_FSLACC_SHIFT                     (2U)
#define NV_FSEC_FSLACC(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FSEC_FSLACC_SHIFT)) & NV_FSEC_FSLACC_MASK)
#define NV_FSEC_MEEN_MASK                        (0x30U)
#define NV_FSEC_MEEN_SHIFT                       (4U)
#define NV_FSEC_MEEN(x)                          (((uint8_t)(((uint8_t)(x)) << NV_FSEC_MEEN_SHIFT)) & NV_FSEC_MEEN_MASK)
#define NV_FSEC_KEYEN_MASK                       (0xC0U)
#define NV_FSEC_KEYEN_SHIFT                      (6U)
#define NV_FSEC_KEYEN(x)                         (((uint8_t)(((uint8_t)(x)) << NV_FSEC_KEYEN_SHIFT)) & NV_FSEC_KEYEN_MASK)

/*! @name FOPT - Non-volatile Flash Option Register */
#define NV_FOPT_LPBOOT_MASK                      (0x1U)
#define NV_FOPT_LPBOOT_SHIFT                     (0U)
#define NV_FOPT_LPBOOT(x)                        (((uint8_t)(((uint8_t)(x)) << NV_FOPT_LPBOOT_SHIFT)) & NV_FOPT_LPBOOT_MASK)
#define NV_FOPT_BOOTPIN_OPT_MASK                 (0x2U)
#define NV_FOPT_BOOTPIN_OPT_SHIFT                (1U)
#define NV_FOPT_BOOTPIN_OPT(x)                   (((uint8_t)(((uint8_t)(x)) << NV_FOPT_BOOTPIN_OPT_SHIFT)) & NV_FOPT_BOOTPIN_OPT_MASK)
#define NV_FOPT_NMI_DIS_MASK                     (0x4U)
#define NV_FOPT_NMI_DIS_SHIFT                    (2U)
#define NV_FOPT_NMI_DIS(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FOPT_NMI_DIS_SHIFT)) & NV_FOPT_NMI_DIS_MASK)
#define NV_FOPT_FAST_INIT_MASK                   (0x20U)
#define NV_FOPT_FAST_INIT_SHIFT                  (5U)
#define NV_FOPT_FAST_INIT(x)                     (((uint8_t)(((uint8_t)(x)) << NV_FOPT_FAST_INIT_SHIFT)) & NV_FOPT_FAST_INIT_MASK)
#define NV_FOPT_BOOTSRC_SEL_MASK                 (0xC0U)
#define NV_FOPT_BOOTSRC_SEL_SHIFT                (6U)
#define NV_FOPT_BOOTSRC_SEL(x)                   (((uint8_t)(((uint8_t)(x)) << NV_FOPT_BOOTSRC_SEL_SHIFT)) & NV_FOPT_BOOTSRC_SEL_MASK)


/*!
 * @}
 */ /* end of group NV_Register_Masks */


/* NV - Peripheral instance base addresses */
/** Peripheral FTFA_FlashConfig base address */
#define FTFA_FlashConfig_BASE                    (0x400u)
/** Peripheral FTFA_FlashConfig base pointer */
#define FTFA_FlashConfig                         ((NV_Type *)FTFA_FlashConfig_BASE)
/** Array initializer of NV peripheral base addresses */
#define NV_BASE_ADDRS                            { FTFA_FlashConfig_BASE }
/** Array initializer of NV peripheral base pointers */
#define NV_BASE_PTRS                             { FTFA_FlashConfig }

/*!
 * @}
 */ /* end of group NV_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- OSC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Peripheral_Access_Layer OSC Peripheral Access Layer
 * @{
 */

/** OSC - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR;                                 /**< OSC Control Register, offset: 0x0 */
       uint8_t RESERVED_0[1];
  __IO uint8_t DIV;                                /**< OSC_DIV, offset: 0x2 */
} OSC_Type;

/* ----------------------------------------------------------------------------
   -- OSC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Masks OSC Register Masks
 * @{
 */

/*! @name CR - OSC Control Register */
#define OSC_CR_SC16P_MASK                        (0x1U)
#define OSC_CR_SC16P_SHIFT                       (0U)
#define OSC_CR_SC16P(x)                          (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC16P_SHIFT)) & OSC_CR_SC16P_MASK)
#define OSC_CR_SC8P_MASK                         (0x2U)
#define OSC_CR_SC8P_SHIFT                        (1U)
#define OSC_CR_SC8P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC8P_SHIFT)) & OSC_CR_SC8P_MASK)
#define OSC_CR_SC4P_MASK                         (0x4U)
#define OSC_CR_SC4P_SHIFT                        (2U)
#define OSC_CR_SC4P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC4P_SHIFT)) & OSC_CR_SC4P_MASK)
#define OSC_CR_SC2P_MASK                         (0x8U)
#define OSC_CR_SC2P_SHIFT                        (3U)
#define OSC_CR_SC2P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC2P_SHIFT)) & OSC_CR_SC2P_MASK)
#define OSC_CR_EREFSTEN_MASK                     (0x20U)
#define OSC_CR_EREFSTEN_SHIFT                    (5U)
#define OSC_CR_EREFSTEN(x)                       (((uint8_t)(((uint8_t)(x)) << OSC_CR_EREFSTEN_SHIFT)) & OSC_CR_EREFSTEN_MASK)
#define OSC_CR_ERCLKEN_MASK                      (0x80U)
#define OSC_CR_ERCLKEN_SHIFT                     (7U)
#define OSC_CR_ERCLKEN(x)                        (((uint8_t)(((uint8_t)(x)) << OSC_CR_ERCLKEN_SHIFT)) & OSC_CR_ERCLKEN_MASK)

/*! @name DIV - OSC_DIV */
#define OSC_DIV_ERPS_MASK                        (0xC0U)
#define OSC_DIV_ERPS_SHIFT                       (6U)
#define OSC_DIV_ERPS(x)                          (((uint8_t)(((uint8_t)(x)) << OSC_DIV_ERPS_SHIFT)) & OSC_DIV_ERPS_MASK)


/*!
 * @}
 */ /* end of group OSC_Register_Masks */


/* OSC - Peripheral instance base addresses */
/** Peripheral OSC base address */
#define OSC_BASE                                 (0x40065000u)
/** Peripheral OSC base pointer */
#define OSC                                      ((OSC_Type *)OSC_BASE)
/** Array initializer of OSC peripheral base addresses */
#define OSC_BASE_ADDRS                           { OSC_BASE }
/** Array initializer of OSC peripheral base pointers */
#define OSC_BASE_PTRS                            { OSC }

/*!
 * @}
 */ /* end of group OSC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PDB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Peripheral_Access_Layer PDB Peripheral Access Layer
 * @{
 */

/** PDB - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status and Control register, offset: 0x0 */
  __IO uint32_t MOD;                               /**< Modulus register, offset: 0x4 */
  __I  uint32_t CNT;                               /**< Counter register, offset: 0x8 */
  __IO uint32_t IDLY;                              /**< Interrupt Delay register, offset: 0xC */
  struct {                                         /* offset: 0x10, array step: 0x10 */
    __IO uint32_t C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x10 */
    __IO uint32_t S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x10 */
    __IO uint32_t DLY[2];                            /**< Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x10, index2*0x4 */
  } CH[1];
       uint8_t RESERVED_0[304];
  struct {                                         /* offset: 0x150, array step: 0x8 */
    __IO uint32_t INTC;                              /**< DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8 */
    __IO uint32_t INT;                               /**< DAC Interval n register, array offset: 0x154, array step: 0x8 */
  } DAC[1];
       uint8_t RESERVED_1[56];
  __IO uint32_t POEN;                              /**< Pulse-Out n Enable register, offset: 0x190 */
  __IO uint32_t PODLY[2];                          /**< Pulse-Out n Delay register, array offset: 0x194, array step: 0x4 */
} PDB_Type;

/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Masks PDB Register Masks
 * @{
 */

/*! @name SC - Status and Control register */
#define PDB_SC_LDOK_MASK                         (0x1U)
#define PDB_SC_LDOK_SHIFT                        (0U)
#define PDB_SC_LDOK(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_LDOK_SHIFT)) & PDB_SC_LDOK_MASK)
#define PDB_SC_CONT_MASK                         (0x2U)
#define PDB_SC_CONT_SHIFT                        (1U)
#define PDB_SC_CONT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_CONT_SHIFT)) & PDB_SC_CONT_MASK)
#define PDB_SC_MULT_MASK                         (0xCU)
#define PDB_SC_MULT_SHIFT                        (2U)
#define PDB_SC_MULT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_MULT_SHIFT)) & PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE_MASK                        (0x20U)
#define PDB_SC_PDBIE_SHIFT                       (5U)
#define PDB_SC_PDBIE(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBIE_SHIFT)) & PDB_SC_PDBIE_MASK)
#define PDB_SC_PDBIF_MASK                        (0x40U)
#define PDB_SC_PDBIF_SHIFT                       (6U)
#define PDB_SC_PDBIF(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBIF_SHIFT)) & PDB_SC_PDBIF_MASK)
#define PDB_SC_PDBEN_MASK                        (0x80U)
#define PDB_SC_PDBEN_SHIFT                       (7U)
#define PDB_SC_PDBEN(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBEN_SHIFT)) & PDB_SC_PDBEN_MASK)
#define PDB_SC_TRGSEL_MASK                       (0xF00U)
#define PDB_SC_TRGSEL_SHIFT                      (8U)
#define PDB_SC_TRGSEL(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_TRGSEL_SHIFT)) & PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    (0x7000U)
#define PDB_SC_PRESCALER_SHIFT                   (12U)
#define PDB_SC_PRESCALER(x)                      (((uint32_t)(((uint32_t)(x)) << PDB_SC_PRESCALER_SHIFT)) & PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN_MASK                        (0x8000U)
#define PDB_SC_DMAEN_SHIFT                       (15U)
#define PDB_SC_DMAEN(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_DMAEN_SHIFT)) & PDB_SC_DMAEN_MASK)
#define PDB_SC_SWTRIG_MASK                       (0x10000U)
#define PDB_SC_SWTRIG_SHIFT                      (16U)
#define PDB_SC_SWTRIG(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_SWTRIG_SHIFT)) & PDB_SC_SWTRIG_MASK)
#define PDB_SC_PDBEIE_MASK                       (0x20000U)
#define PDB_SC_PDBEIE_SHIFT                      (17U)
#define PDB_SC_PDBEIE(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBEIE_SHIFT)) & PDB_SC_PDBEIE_MASK)
#define PDB_SC_LDMOD_MASK                        (0xC0000U)
#define PDB_SC_LDMOD_SHIFT                       (18U)
#define PDB_SC_LDMOD(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_LDMOD_SHIFT)) & PDB_SC_LDMOD_MASK)

/*! @name MOD - Modulus register */
#define PDB_MOD_MOD_MASK                         (0xFFFFU)
#define PDB_MOD_MOD_SHIFT                        (0U)
#define PDB_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_MOD_MOD_SHIFT)) & PDB_MOD_MOD_MASK)

/*! @name CNT - Counter register */
#define PDB_CNT_CNT_MASK                         (0xFFFFU)
#define PDB_CNT_CNT_SHIFT                        (0U)
#define PDB_CNT_CNT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_CNT_CNT_SHIFT)) & PDB_CNT_CNT_MASK)

/*! @name IDLY - Interrupt Delay register */
#define PDB_IDLY_IDLY_MASK                       (0xFFFFU)
#define PDB_IDLY_IDLY_SHIFT                      (0U)
#define PDB_IDLY_IDLY(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_IDLY_IDLY_SHIFT)) & PDB_IDLY_IDLY_MASK)

/*! @name C1 - Channel n Control register 1 */
#define PDB_C1_EN_MASK                           (0xFFU)
#define PDB_C1_EN_SHIFT                          (0U)
#define PDB_C1_EN(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_C1_EN_SHIFT)) & PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          (0xFF00U)
#define PDB_C1_TOS_SHIFT                         (8U)
#define PDB_C1_TOS(x)                            (((uint32_t)(((uint32_t)(x)) << PDB_C1_TOS_SHIFT)) & PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           (0xFF0000U)
#define PDB_C1_BB_SHIFT                          (16U)
#define PDB_C1_BB(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_C1_BB_SHIFT)) & PDB_C1_BB_MASK)

/* The count of PDB_C1 */
#define PDB_C1_COUNT                             (1U)

/*! @name S - Channel n Status register */
#define PDB_S_ERR_MASK                           (0xFFU)
#define PDB_S_ERR_SHIFT                          (0U)
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_S_ERR_SHIFT)) & PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            (0xFF0000U)
#define PDB_S_CF_SHIFT                           (16U)
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x)) << PDB_S_CF_SHIFT)) & PDB_S_CF_MASK)

/* The count of PDB_S */
#define PDB_S_COUNT                              (1U)

/*! @name DLY - Channel n Delay 0 register..Channel n Delay 1 register */
#define PDB_DLY_DLY_MASK                         (0xFFFFU)
#define PDB_DLY_DLY_SHIFT                        (0U)
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_DLY_DLY_SHIFT)) & PDB_DLY_DLY_MASK)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT                            (1U)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT2                           (2U)

/*! @name INTC - DAC Interval Trigger n Control register */
#define PDB_INTC_TOE_MASK                        (0x1U)
#define PDB_INTC_TOE_SHIFT                       (0U)
#define PDB_INTC_TOE(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_INTC_TOE_SHIFT)) & PDB_INTC_TOE_MASK)
#define PDB_INTC_EXT_MASK                        (0x2U)
#define PDB_INTC_EXT_SHIFT                       (1U)
#define PDB_INTC_EXT(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_INTC_EXT_SHIFT)) & PDB_INTC_EXT_MASK)

/* The count of PDB_INTC */
#define PDB_INTC_COUNT                           (1U)

/*! @name INT - DAC Interval n register */
#define PDB_INT_INT_MASK                         (0xFFFFU)
#define PDB_INT_INT_SHIFT                        (0U)
#define PDB_INT_INT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_INT_INT_SHIFT)) & PDB_INT_INT_MASK)

/* The count of PDB_INT */
#define PDB_INT_COUNT                            (1U)

/*! @name POEN - Pulse-Out n Enable register */
#define PDB_POEN_POEN_MASK                       (0xFFU)
#define PDB_POEN_POEN_SHIFT                      (0U)
#define PDB_POEN_POEN(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_POEN_POEN_SHIFT)) & PDB_POEN_POEN_MASK)

/*! @name PODLY - Pulse-Out n Delay register */
#define PDB_PODLY_DLY2_MASK                      (0xFFFFU)
#define PDB_PODLY_DLY2_SHIFT                     (0U)
#define PDB_PODLY_DLY2(x)                        (((uint32_t)(((uint32_t)(x)) << PDB_PODLY_DLY2_SHIFT)) & PDB_PODLY_DLY2_MASK)
#define PDB_PODLY_DLY1_MASK                      (0xFFFF0000U)
#define PDB_PODLY_DLY1_SHIFT                     (16U)
#define PDB_PODLY_DLY1(x)                        (((uint32_t)(((uint32_t)(x)) << PDB_PODLY_DLY1_SHIFT)) & PDB_PODLY_DLY1_MASK)

/* The count of PDB_PODLY */
#define PDB_PODLY_COUNT                          (2U)


/*!
 * @}
 */ /* end of group PDB_Register_Masks */


/* PDB - Peripheral instance base addresses */
/** Peripheral PDB0 base address */
#define PDB0_BASE                                (0x40036000u)
/** Peripheral PDB0 base pointer */
#define PDB0                                     ((PDB_Type *)PDB0_BASE)
/** Array initializer of PDB peripheral base addresses */
#define PDB_BASE_ADDRS                           { PDB0_BASE }
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB0 }
/** Interrupt vectors for the PDB peripheral type */
#define PDB_IRQS                                 { PDB0_IRQn }

/*!
 * @}
 */ /* end of group PDB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< PIT Module Control Register, offset: 0x0 */
       uint8_t RESERVED_0[252];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __IO uint32_t LDVAL;                             /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    __IO uint32_t TFLG;                              /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[4];
} PIT_Type;

/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/*! @name MCR - PIT Module Control Register */
#define PIT_MCR_FRZ_MASK                         (0x1U)
#define PIT_MCR_FRZ_SHIFT                        (0U)
#define PIT_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x)) << PIT_MCR_FRZ_SHIFT)) & PIT_MCR_FRZ_MASK)
#define PIT_MCR_MDIS_MASK                        (0x2U)
#define PIT_MCR_MDIS_SHIFT                       (1U)
#define PIT_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x)) << PIT_MCR_MDIS_SHIFT)) & PIT_MCR_MDIS_MASK)

/*! @name LDVAL - Timer Load Value Register */
#define PIT_LDVAL_TSV_MASK                       (0xFFFFFFFFU)
#define PIT_LDVAL_TSV_SHIFT                      (0U)
#define PIT_LDVAL_TSV(x)                         (((uint32_t)(((uint32_t)(x)) << PIT_LDVAL_TSV_SHIFT)) & PIT_LDVAL_TSV_MASK)

/* The count of PIT_LDVAL */
#define PIT_LDVAL_COUNT                          (4U)

/*! @name CVAL - Current Timer Value Register */
#define PIT_CVAL_TVL_MASK                        (0xFFFFFFFFU)
#define PIT_CVAL_TVL_SHIFT                       (0U)
#define PIT_CVAL_TVL(x)                          (((uint32_t)(((uint32_t)(x)) << PIT_CVAL_TVL_SHIFT)) & PIT_CVAL_TVL_MASK)

/* The count of PIT_CVAL */
#define PIT_CVAL_COUNT                           (4U)

/*! @name TCTRL - Timer Control Register */
#define PIT_TCTRL_TEN_MASK                       (0x1U)
#define PIT_TCTRL_TEN_SHIFT                      (0U)
#define PIT_TCTRL_TEN(x)                         (((uint32_t)(((uint32_t)(x)) << PIT_TCTRL_TEN_SHIFT)) & PIT_TCTRL_TEN_MASK)
#define PIT_TCTRL_TIE_MASK                       (0x2U)
#define PIT_TCTRL_TIE_SHIFT                      (1U)
#define PIT_TCTRL_TIE(x)                         (((uint32_t)(((uint32_t)(x)) << PIT_TCTRL_TIE_SHIFT)) & PIT_TCTRL_TIE_MASK)
#define PIT_TCTRL_CHN_MASK                       (0x4U)
#define PIT_TCTRL_CHN_SHIFT                      (2U)
#define PIT_TCTRL_CHN(x)                         (((uint32_t)(((uint32_t)(x)) << PIT_TCTRL_CHN_SHIFT)) & PIT_TCTRL_CHN_MASK)

/* The count of PIT_TCTRL */
#define PIT_TCTRL_COUNT                          (4U)

/*! @name TFLG - Timer Flag Register */
#define PIT_TFLG_TIF_MASK                        (0x1U)
#define PIT_TFLG_TIF_SHIFT                       (0U)
#define PIT_TFLG_TIF(x)                          (((uint32_t)(((uint32_t)(x)) << PIT_TFLG_TIF_SHIFT)) & PIT_TFLG_TIF_MASK)

/* The count of PIT_TFLG */
#define PIT_TFLG_COUNT                           (4U)


/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT0 base address */
#define PIT0_BASE                                (0x40037000u)
/** Peripheral PIT0 base pointer */
#define PIT0                                     ((PIT_Type *)PIT0_BASE)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS                           { PIT0_BASE }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT0 }
/** Interrupt vectors for the PIT peripheral type */
#define PIT_IRQS                                 { PIT0CH0_IRQn, PIT0CH1_IRQn, PIT0CH2_IRQn, PIT0CH3_IRQn }

/*!
 * @}
 */ /* end of group PIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t LVDSC1;                             /**< Low Voltage Detect Status And Control 1 register, offset: 0x0 */
  __IO uint8_t LVDSC2;                             /**< Low Voltage Detect Status And Control 2 register, offset: 0x1 */
  __IO uint8_t REGSC;                              /**< Regulator Status And Control register, offset: 0x2 */
       uint8_t RESERVED_0[8];
  __IO uint8_t HVDSC1;                             /**< High Voltage Detect Status And Control 1 register, offset: 0xB */
} PMC_Type;

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name LVDSC1 - Low Voltage Detect Status And Control 1 register */
#define PMC_LVDSC1_LVDV_MASK                     (0x3U)
#define PMC_LVDSC1_LVDV_SHIFT                    (0U)
#define PMC_LVDSC1_LVDV(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC1_LVDV_SHIFT)) & PMC_LVDSC1_LVDV_MASK)
#define PMC_LVDSC1_LVDRE_MASK                    (0x10U)
#define PMC_LVDSC1_LVDRE_SHIFT                   (4U)
#define PMC_LVDSC1_LVDRE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC1_LVDRE_SHIFT)) & PMC_LVDSC1_LVDRE_MASK)
#define PMC_LVDSC1_LVDIE_MASK                    (0x20U)
#define PMC_LVDSC1_LVDIE_SHIFT                   (5U)
#define PMC_LVDSC1_LVDIE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC1_LVDIE_SHIFT)) & PMC_LVDSC1_LVDIE_MASK)
#define PMC_LVDSC1_LVDACK_MASK                   (0x40U)
#define PMC_LVDSC1_LVDACK_SHIFT                  (6U)
#define PMC_LVDSC1_LVDACK(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC1_LVDACK_SHIFT)) & PMC_LVDSC1_LVDACK_MASK)
#define PMC_LVDSC1_LVDF_MASK                     (0x80U)
#define PMC_LVDSC1_LVDF_SHIFT                    (7U)
#define PMC_LVDSC1_LVDF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC1_LVDF_SHIFT)) & PMC_LVDSC1_LVDF_MASK)

/*! @name LVDSC2 - Low Voltage Detect Status And Control 2 register */
#define PMC_LVDSC2_LVWV_MASK                     (0x3U)
#define PMC_LVDSC2_LVWV_SHIFT                    (0U)
#define PMC_LVDSC2_LVWV(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWV_SHIFT)) & PMC_LVDSC2_LVWV_MASK)
#define PMC_LVDSC2_LVWIE_MASK                    (0x20U)
#define PMC_LVDSC2_LVWIE_SHIFT                   (5U)
#define PMC_LVDSC2_LVWIE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWIE_SHIFT)) & PMC_LVDSC2_LVWIE_MASK)
#define PMC_LVDSC2_LVWACK_MASK                   (0x40U)
#define PMC_LVDSC2_LVWACK_SHIFT                  (6U)
#define PMC_LVDSC2_LVWACK(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWACK_SHIFT)) & PMC_LVDSC2_LVWACK_MASK)
#define PMC_LVDSC2_LVWF_MASK                     (0x80U)
#define PMC_LVDSC2_LVWF_SHIFT                    (7U)
#define PMC_LVDSC2_LVWF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWF_SHIFT)) & PMC_LVDSC2_LVWF_MASK)

/*! @name REGSC - Regulator Status And Control register */
#define PMC_REGSC_BGBE_MASK                      (0x1U)
#define PMC_REGSC_BGBE_SHIFT                     (0U)
#define PMC_REGSC_BGBE(x)                        (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_BGBE_SHIFT)) & PMC_REGSC_BGBE_MASK)
#define PMC_REGSC_REGONS_MASK                    (0x4U)
#define PMC_REGSC_REGONS_SHIFT                   (2U)
#define PMC_REGSC_REGONS(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_REGONS_SHIFT)) & PMC_REGSC_REGONS_MASK)
#define PMC_REGSC_ACKISO_MASK                    (0x8U)
#define PMC_REGSC_ACKISO_SHIFT                   (3U)
#define PMC_REGSC_ACKISO(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_ACKISO_SHIFT)) & PMC_REGSC_ACKISO_MASK)
#define PMC_REGSC_BGEN_MASK                      (0x10U)
#define PMC_REGSC_BGEN_SHIFT                     (4U)
#define PMC_REGSC_BGEN(x)                        (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_BGEN_SHIFT)) & PMC_REGSC_BGEN_MASK)

/*! @name HVDSC1 - High Voltage Detect Status And Control 1 register */
#define PMC_HVDSC1_HVDV_MASK                     (0x1U)
#define PMC_HVDSC1_HVDV_SHIFT                    (0U)
#define PMC_HVDSC1_HVDV(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_HVDSC1_HVDV_SHIFT)) & PMC_HVDSC1_HVDV_MASK)
#define PMC_HVDSC1_HVDRE_MASK                    (0x10U)
#define PMC_HVDSC1_HVDRE_SHIFT                   (4U)
#define PMC_HVDSC1_HVDRE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_HVDSC1_HVDRE_SHIFT)) & PMC_HVDSC1_HVDRE_MASK)
#define PMC_HVDSC1_HVDIE_MASK                    (0x20U)
#define PMC_HVDSC1_HVDIE_SHIFT                   (5U)
#define PMC_HVDSC1_HVDIE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_HVDSC1_HVDIE_SHIFT)) & PMC_HVDSC1_HVDIE_MASK)
#define PMC_HVDSC1_HVDACK_MASK                   (0x40U)
#define PMC_HVDSC1_HVDACK_SHIFT                  (6U)
#define PMC_HVDSC1_HVDACK(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_HVDSC1_HVDACK_SHIFT)) & PMC_HVDSC1_HVDACK_MASK)
#define PMC_HVDSC1_HVDF_MASK                     (0x80U)
#define PMC_HVDSC1_HVDF_SHIFT                    (7U)
#define PMC_HVDSC1_HVDF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_HVDSC1_HVDF_SHIFT)) & PMC_HVDSC1_HVDF_MASK)


/*!
 * @}
 */ /* end of group PMC_Register_Masks */


/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE                                 (0x4007D000u)
/** Peripheral PMC base pointer */
#define PMC                                      ((PMC_Type *)PMC_BASE)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS                           { PMC_BASE }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC }
/** Interrupt vectors for the PMC peripheral type */
#define PMC_IRQS                                 { LVD_LVW_IRQn }

/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCR[32];                           /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  __O  uint32_t GPCLR;                             /**< Global Pin Control Low Register, offset: 0x80 */
  __O  uint32_t GPCHR;                             /**< Global Pin Control High Register, offset: 0x84 */
       uint8_t RESERVED_0[24];
  __IO uint32_t ISFR;                              /**< Interrupt Status Flag Register, offset: 0xA0 */
       uint8_t RESERVED_1[28];
  __IO uint32_t DFER;                              /**< Digital Filter Enable Register, offset: 0xC0 */
  __IO uint32_t DFCR;                              /**< Digital Filter Clock Register, offset: 0xC4 */
  __IO uint32_t DFWR;                              /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_Type;

/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/*! @name PCR - Pin Control Register n */
#define PORT_PCR_PS_MASK                         (0x1U)
#define PORT_PCR_PS_SHIFT                        (0U)
#define PORT_PCR_PS(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PS_SHIFT)) & PORT_PCR_PS_MASK)
#define PORT_PCR_PE_MASK                         (0x2U)
#define PORT_PCR_PE_SHIFT                        (1U)
#define PORT_PCR_PE(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PE_SHIFT)) & PORT_PCR_PE_MASK)
#define PORT_PCR_SRE_MASK                        (0x4U)
#define PORT_PCR_SRE_SHIFT                       (2U)
#define PORT_PCR_SRE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_SRE_SHIFT)) & PORT_PCR_SRE_MASK)
#define PORT_PCR_PFE_MASK                        (0x10U)
#define PORT_PCR_PFE_SHIFT                       (4U)
#define PORT_PCR_PFE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PFE_SHIFT)) & PORT_PCR_PFE_MASK)
#define PORT_PCR_ODE_MASK                        (0x20U)
#define PORT_PCR_ODE_SHIFT                       (5U)
#define PORT_PCR_ODE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ODE_SHIFT)) & PORT_PCR_ODE_MASK)
#define PORT_PCR_DSE_MASK                        (0x40U)
#define PORT_PCR_DSE_SHIFT                       (6U)
#define PORT_PCR_DSE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_DSE_SHIFT)) & PORT_PCR_DSE_MASK)
#define PORT_PCR_MUX_MASK                        (0x700U)
#define PORT_PCR_MUX_SHIFT                       (8U)
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_MUX_SHIFT)) & PORT_PCR_MUX_MASK)
#define PORT_PCR_LK_MASK                         (0x8000U)
#define PORT_PCR_LK_SHIFT                        (15U)
#define PORT_PCR_LK(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_LK_SHIFT)) & PORT_PCR_LK_MASK)
#define PORT_PCR_IRQC_MASK                       (0xF0000U)
#define PORT_PCR_IRQC_SHIFT                      (16U)
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_PCR_IRQC_SHIFT)) & PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF_MASK                        (0x1000000U)
#define PORT_PCR_ISF_SHIFT                       (24U)
#define PORT_PCR_ISF(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ISF_SHIFT)) & PORT_PCR_ISF_MASK)

/* The count of PORT_PCR */
#define PORT_PCR_COUNT                           (32U)

/*! @name GPCLR - Global Pin Control Low Register */
#define PORT_GPCLR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCLR_GPWD_SHIFT                    (0U)
#define PORT_GPCLR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWD_SHIFT)) & PORT_GPCLR_GPWD_MASK)
#define PORT_GPCLR_GPWE_MASK                     (0xFFFF0000U)
#define PORT_GPCLR_GPWE_SHIFT                    (16U)
#define PORT_GPCLR_GPWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE_SHIFT)) & PORT_GPCLR_GPWE_MASK)

/*! @name GPCHR - Global Pin Control High Register */
#define PORT_GPCHR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCHR_GPWD_SHIFT                    (0U)
#define PORT_GPCHR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWD_SHIFT)) & PORT_GPCHR_GPWD_MASK)
#define PORT_GPCHR_GPWE_MASK                     (0xFFFF0000U)
#define PORT_GPCHR_GPWE_SHIFT                    (16U)
#define PORT_GPCHR_GPWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE_SHIFT)) & PORT_GPCHR_GPWE_MASK)

/*! @name ISFR - Interrupt Status Flag Register */
#define PORT_ISFR_ISF_MASK                       (0xFFFFFFFFU)
#define PORT_ISFR_ISF_SHIFT                      (0U)
#define PORT_ISFR_ISF(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_ISFR_ISF_SHIFT)) & PORT_ISFR_ISF_MASK)

/*! @name DFER - Digital Filter Enable Register */
#define PORT_DFER_DFE_MASK                       (0xFFFFFFFFU)
#define PORT_DFER_DFE_SHIFT                      (0U)
#define PORT_DFER_DFE(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_DFER_DFE_SHIFT)) & PORT_DFER_DFE_MASK)

/*! @name DFCR - Digital Filter Clock Register */
#define PORT_DFCR_CS_MASK                        (0x1U)
#define PORT_DFCR_CS_SHIFT                       (0U)
#define PORT_DFCR_CS(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_DFCR_CS_SHIFT)) & PORT_DFCR_CS_MASK)

/*! @name DFWR - Digital Filter Width Register */
#define PORT_DFWR_FILT_MASK                      (0x1FU)
#define PORT_DFWR_FILT_SHIFT                     (0U)
#define PORT_DFWR_FILT(x)                        (((uint32_t)(((uint32_t)(x)) << PORT_DFWR_FILT_SHIFT)) & PORT_DFWR_FILT_MASK)


/*!
 * @}
 */ /* end of group PORT_Register_Masks */


/* PORT - Peripheral instance base addresses */
/** Peripheral PORTA base address */
#define PORTA_BASE                               (0x40049000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_Type *)PORTA_BASE)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4004A000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_Type *)PORTB_BASE)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_Type *)PORTC_BASE)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4004C000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_Type *)PORTD_BASE)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4004D000u)
/** Peripheral PORTE base pointer */
#define PORTE                                    ((PORT_Type *)PORTE_BASE)
/** Array initializer of PORT peripheral base addresses */
#define PORT_BASE_ADDRS                          { PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE }
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORTA, PORTB, PORTC, PORTD, PORTE }
/** Interrupt vectors for the PORT peripheral type */
#define PORT_IRQS                                { PORTA_IRQn, PORTB_IRQn, PORTC_IRQn, PORTD_IRQn, PORTE_IRQn }

/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- QuadSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QuadSPI_Peripheral_Access_Layer QuadSPI Peripheral Access Layer
 * @{
 */

/** QuadSPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t IPCR;                              /**< IP Configuration Register, offset: 0x8 */
  __IO uint32_t FLSHCR;                            /**< Flash Configuration Register, offset: 0xC */
  __IO uint32_t BUF0CR;                            /**< Buffer0 Configuration Register, offset: 0x10 */
  __IO uint32_t BUF1CR;                            /**< Buffer1 Configuration Register, offset: 0x14 */
  __IO uint32_t BUF2CR;                            /**< Buffer2 Configuration Register, offset: 0x18 */
  __IO uint32_t BUF3CR;                            /**< Buffer3 Configuration Register, offset: 0x1C */
  __IO uint32_t BFGENCR;                           /**< Buffer Generic Configuration Register, offset: 0x20 */
  __IO uint32_t SOCCR;                             /**< SOC Configuration Register, offset: 0x24 */
       uint8_t RESERVED_1[8];
  __IO uint32_t BUF0IND;                           /**< Buffer0 Top Index Register, offset: 0x30 */
  __IO uint32_t BUF1IND;                           /**< Buffer1 Top Index Register, offset: 0x34 */
  __IO uint32_t BUF2IND;                           /**< Buffer2 Top Index Register, offset: 0x38 */
       uint8_t RESERVED_2[196];
  __IO uint32_t SFAR;                              /**< Serial Flash Address Register, offset: 0x100 */
  __IO uint32_t SFACR;                             /**< Serial Flash Address Configuration Register, offset: 0x104 */
  __IO uint32_t SMPR;                              /**< Sampling Register, offset: 0x108 */
  __I  uint32_t RBSR;                              /**< RX Buffer Status Register, offset: 0x10C */
  __IO uint32_t RBCT;                              /**< RX Buffer Control Register, offset: 0x110 */
       uint8_t RESERVED_3[60];
  __I  uint32_t TBSR;                              /**< TX Buffer Status Register, offset: 0x150 */
  __IO uint32_t TBDR;                              /**< TX Buffer Data Register, offset: 0x154 */
  __IO uint32_t TBCT;                              /**< Tx Buffer Control Register, offset: 0x158 */
  __I  uint32_t SR;                                /**< Status Register, offset: 0x15C */
  __IO uint32_t FR;                                /**< Flag Register, offset: 0x160 */
  __IO uint32_t RSER;                              /**< Interrupt and DMA Request Select and Enable Register, offset: 0x164 */
  __I  uint32_t SPNDST;                            /**< Sequence Suspend Status Register, offset: 0x168 */
  __IO uint32_t SPTRCLR;                           /**< Sequence Pointer Clear Register, offset: 0x16C */
       uint8_t RESERVED_4[16];
  __IO uint32_t SFA1AD;                            /**< Serial Flash A1 Top Address, offset: 0x180 */
  __IO uint32_t SFA2AD;                            /**< Serial Flash A2 Top Address, offset: 0x184 */
  __IO uint32_t SFB1AD;                            /**< Serial Flash B1Top Address, offset: 0x188 */
  __IO uint32_t SFB2AD;                            /**< Serial Flash B2Top Address, offset: 0x18C */
  __IO uint32_t DLPR;                              /**< Data Learn Pattern Register, offset: 0x190 */
       uint8_t RESERVED_5[108];
  __I  uint32_t RBDR[16];                          /**< RX Buffer Data Register, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_6[192];
  __IO uint32_t LUTKEY;                            /**< LUT Key Register, offset: 0x300 */
  __IO uint32_t LCKCR;                             /**< LUT Lock Configuration Register, offset: 0x304 */
       uint8_t RESERVED_7[8];
  __IO uint32_t LUT[64];                           /**< Look-up Table register, array offset: 0x310, array step: 0x4 */
} QuadSPI_Type;

/* ----------------------------------------------------------------------------
   -- QuadSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QuadSPI_Register_Masks QuadSPI Register Masks
 * @{
 */

/*! @name MCR - Module Configuration Register */
#define QuadSPI_MCR_SWRSTSD_MASK                 (0x1U)
#define QuadSPI_MCR_SWRSTSD_SHIFT                (0U)
#define QuadSPI_MCR_SWRSTSD(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SWRSTSD_SHIFT)) & QuadSPI_MCR_SWRSTSD_MASK)
#define QuadSPI_MCR_SWRSTHD_MASK                 (0x2U)
#define QuadSPI_MCR_SWRSTHD_SHIFT                (1U)
#define QuadSPI_MCR_SWRSTHD(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SWRSTHD_SHIFT)) & QuadSPI_MCR_SWRSTHD_MASK)
#define QuadSPI_MCR_END_CFG_MASK                 (0xCU)
#define QuadSPI_MCR_END_CFG_SHIFT                (2U)
#define QuadSPI_MCR_END_CFG(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_END_CFG_SHIFT)) & QuadSPI_MCR_END_CFG_MASK)
#define QuadSPI_MCR_DQS_LAT_EN_MASK              (0x20U)
#define QuadSPI_MCR_DQS_LAT_EN_SHIFT             (5U)
#define QuadSPI_MCR_DQS_LAT_EN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DQS_LAT_EN_SHIFT)) & QuadSPI_MCR_DQS_LAT_EN_MASK)
#define QuadSPI_MCR_DQS_EN_MASK                  (0x40U)
#define QuadSPI_MCR_DQS_EN_SHIFT                 (6U)
#define QuadSPI_MCR_DQS_EN(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DQS_EN_SHIFT)) & QuadSPI_MCR_DQS_EN_MASK)
#define QuadSPI_MCR_DDR_EN_MASK                  (0x80U)
#define QuadSPI_MCR_DDR_EN_SHIFT                 (7U)
#define QuadSPI_MCR_DDR_EN(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DDR_EN_SHIFT)) & QuadSPI_MCR_DDR_EN_MASK)
#define QuadSPI_MCR_CLR_RXF_MASK                 (0x400U)
#define QuadSPI_MCR_CLR_RXF_SHIFT                (10U)
#define QuadSPI_MCR_CLR_RXF(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_CLR_RXF_SHIFT)) & QuadSPI_MCR_CLR_RXF_MASK)
#define QuadSPI_MCR_CLR_TXF_MASK                 (0x800U)
#define QuadSPI_MCR_CLR_TXF_SHIFT                (11U)
#define QuadSPI_MCR_CLR_TXF(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_CLR_TXF_SHIFT)) & QuadSPI_MCR_CLR_TXF_MASK)
#define QuadSPI_MCR_MDIS_MASK                    (0x4000U)
#define QuadSPI_MCR_MDIS_SHIFT                   (14U)
#define QuadSPI_MCR_MDIS(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_MDIS_SHIFT)) & QuadSPI_MCR_MDIS_MASK)
#define QuadSPI_MCR_SCLKCFG_MASK                 (0xFF000000U)
#define QuadSPI_MCR_SCLKCFG_SHIFT                (24U)
#define QuadSPI_MCR_SCLKCFG(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SCLKCFG_SHIFT)) & QuadSPI_MCR_SCLKCFG_MASK)

/*! @name IPCR - IP Configuration Register */
#define QuadSPI_IPCR_IDATSZ_MASK                 (0xFFFFU)
#define QuadSPI_IPCR_IDATSZ_SHIFT                (0U)
#define QuadSPI_IPCR_IDATSZ(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_IPCR_IDATSZ_SHIFT)) & QuadSPI_IPCR_IDATSZ_MASK)
#define QuadSPI_IPCR_PAR_EN_MASK                 (0x10000U)
#define QuadSPI_IPCR_PAR_EN_SHIFT                (16U)
#define QuadSPI_IPCR_PAR_EN(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_IPCR_PAR_EN_SHIFT)) & QuadSPI_IPCR_PAR_EN_MASK)
#define QuadSPI_IPCR_SEQID_MASK                  (0xF000000U)
#define QuadSPI_IPCR_SEQID_SHIFT                 (24U)
#define QuadSPI_IPCR_SEQID(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_IPCR_SEQID_SHIFT)) & QuadSPI_IPCR_SEQID_MASK)

/*! @name FLSHCR - Flash Configuration Register */
#define QuadSPI_FLSHCR_TCSS_MASK                 (0xFU)
#define QuadSPI_FLSHCR_TCSS_SHIFT                (0U)
#define QuadSPI_FLSHCR_TCSS(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TCSS_SHIFT)) & QuadSPI_FLSHCR_TCSS_MASK)
#define QuadSPI_FLSHCR_TCSH_MASK                 (0xF00U)
#define QuadSPI_FLSHCR_TCSH_SHIFT                (8U)
#define QuadSPI_FLSHCR_TCSH(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TCSH_SHIFT)) & QuadSPI_FLSHCR_TCSH_MASK)
#define QuadSPI_FLSHCR_TDH_MASK                  (0x30000U)
#define QuadSPI_FLSHCR_TDH_SHIFT                 (16U)
#define QuadSPI_FLSHCR_TDH(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TDH_SHIFT)) & QuadSPI_FLSHCR_TDH_MASK)

/*! @name BUF0CR - Buffer0 Configuration Register */
#define QuadSPI_BUF0CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF0CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF0CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_MSTRID_SHIFT)) & QuadSPI_BUF0CR_MSTRID_MASK)
#define QuadSPI_BUF0CR_ADATSZ_MASK               (0x7F00U)
#define QuadSPI_BUF0CR_ADATSZ_SHIFT              (8U)
#define QuadSPI_BUF0CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_ADATSZ_SHIFT)) & QuadSPI_BUF0CR_ADATSZ_MASK)
#define QuadSPI_BUF0CR_HP_EN_MASK                (0x80000000U)
#define QuadSPI_BUF0CR_HP_EN_SHIFT               (31U)
#define QuadSPI_BUF0CR_HP_EN(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_HP_EN_SHIFT)) & QuadSPI_BUF0CR_HP_EN_MASK)

/*! @name BUF1CR - Buffer1 Configuration Register */
#define QuadSPI_BUF1CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF1CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF1CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1CR_MSTRID_SHIFT)) & QuadSPI_BUF1CR_MSTRID_MASK)
#define QuadSPI_BUF1CR_ADATSZ_MASK               (0x7F00U)
#define QuadSPI_BUF1CR_ADATSZ_SHIFT              (8U)
#define QuadSPI_BUF1CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1CR_ADATSZ_SHIFT)) & QuadSPI_BUF1CR_ADATSZ_MASK)

/*! @name BUF2CR - Buffer2 Configuration Register */
#define QuadSPI_BUF2CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF2CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF2CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2CR_MSTRID_SHIFT)) & QuadSPI_BUF2CR_MSTRID_MASK)
#define QuadSPI_BUF2CR_ADATSZ_MASK               (0x7F00U)
#define QuadSPI_BUF2CR_ADATSZ_SHIFT              (8U)
#define QuadSPI_BUF2CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2CR_ADATSZ_SHIFT)) & QuadSPI_BUF2CR_ADATSZ_MASK)

/*! @name BUF3CR - Buffer3 Configuration Register */
#define QuadSPI_BUF3CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF3CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF3CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_MSTRID_SHIFT)) & QuadSPI_BUF3CR_MSTRID_MASK)
#define QuadSPI_BUF3CR_ADATSZ_MASK               (0x7F00U)
#define QuadSPI_BUF3CR_ADATSZ_SHIFT              (8U)
#define QuadSPI_BUF3CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_ADATSZ_SHIFT)) & QuadSPI_BUF3CR_ADATSZ_MASK)
#define QuadSPI_BUF3CR_ALLMST_MASK               (0x80000000U)
#define QuadSPI_BUF3CR_ALLMST_SHIFT              (31U)
#define QuadSPI_BUF3CR_ALLMST(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_ALLMST_SHIFT)) & QuadSPI_BUF3CR_ALLMST_MASK)

/*! @name BFGENCR - Buffer Generic Configuration Register */
#define QuadSPI_BFGENCR_SEQID_MASK               (0xF000U)
#define QuadSPI_BFGENCR_SEQID_SHIFT              (12U)
#define QuadSPI_BFGENCR_SEQID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BFGENCR_SEQID_SHIFT)) & QuadSPI_BFGENCR_SEQID_MASK)
#define QuadSPI_BFGENCR_PAR_EN_MASK              (0x10000U)
#define QuadSPI_BFGENCR_PAR_EN_SHIFT             (16U)
#define QuadSPI_BFGENCR_PAR_EN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_BFGENCR_PAR_EN_SHIFT)) & QuadSPI_BFGENCR_PAR_EN_MASK)

/*! @name SOCCR - SOC Configuration Register */
#define QuadSPI_SOCCR_QSPISRC_MASK               (0x7U)
#define QuadSPI_SOCCR_QSPISRC_SHIFT              (0U)
#define QuadSPI_SOCCR_QSPISRC(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_QSPISRC_SHIFT)) & QuadSPI_SOCCR_QSPISRC_MASK)
#define QuadSPI_SOCCR_DQSLPEN_MASK               (0x100U)
#define QuadSPI_SOCCR_DQSLPEN_SHIFT              (8U)
#define QuadSPI_SOCCR_DQSLPEN(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSLPEN_SHIFT)) & QuadSPI_SOCCR_DQSLPEN_MASK)
#define QuadSPI_SOCCR_DQSPADLPEN_MASK            (0x200U)
#define QuadSPI_SOCCR_DQSPADLPEN_SHIFT           (9U)
#define QuadSPI_SOCCR_DQSPADLPEN(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSPADLPEN_SHIFT)) & QuadSPI_SOCCR_DQSPADLPEN_MASK)
#define QuadSPI_SOCCR_DQSPHASEL_MASK             (0xC00U)
#define QuadSPI_SOCCR_DQSPHASEL_SHIFT            (10U)
#define QuadSPI_SOCCR_DQSPHASEL(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSPHASEL_SHIFT)) & QuadSPI_SOCCR_DQSPHASEL_MASK)
#define QuadSPI_SOCCR_DQSINVSEL_MASK             (0x1000U)
#define QuadSPI_SOCCR_DQSINVSEL_SHIFT            (12U)
#define QuadSPI_SOCCR_DQSINVSEL(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSINVSEL_SHIFT)) & QuadSPI_SOCCR_DQSINVSEL_MASK)
#define QuadSPI_SOCCR_CK2EN_MASK                 (0x2000U)
#define QuadSPI_SOCCR_CK2EN_SHIFT                (13U)
#define QuadSPI_SOCCR_CK2EN(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_CK2EN_SHIFT)) & QuadSPI_SOCCR_CK2EN_MASK)
#define QuadSPI_SOCCR_DIFFCKEN_MASK              (0x4000U)
#define QuadSPI_SOCCR_DIFFCKEN_SHIFT             (14U)
#define QuadSPI_SOCCR_DIFFCKEN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DIFFCKEN_SHIFT)) & QuadSPI_SOCCR_DIFFCKEN_MASK)
#define QuadSPI_SOCCR_OCTEN_MASK                 (0x8000U)
#define QuadSPI_SOCCR_OCTEN_SHIFT                (15U)
#define QuadSPI_SOCCR_OCTEN(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_OCTEN_SHIFT)) & QuadSPI_SOCCR_OCTEN_MASK)
#define QuadSPI_SOCCR_DLYTAPSELA_MASK            (0x3F0000U)
#define QuadSPI_SOCCR_DLYTAPSELA_SHIFT           (16U)
#define QuadSPI_SOCCR_DLYTAPSELA(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DLYTAPSELA_SHIFT)) & QuadSPI_SOCCR_DLYTAPSELA_MASK)
#define QuadSPI_SOCCR_DLYTAPSELB_MASK            (0x3F000000U)
#define QuadSPI_SOCCR_DLYTAPSELB_SHIFT           (24U)
#define QuadSPI_SOCCR_DLYTAPSELB(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DLYTAPSELB_SHIFT)) & QuadSPI_SOCCR_DLYTAPSELB_MASK)

/*! @name BUF0IND - Buffer0 Top Index Register */
#define QuadSPI_BUF0IND_TPINDX0_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF0IND_TPINDX0_SHIFT            (3U)
#define QuadSPI_BUF0IND_TPINDX0(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0IND_TPINDX0_SHIFT)) & QuadSPI_BUF0IND_TPINDX0_MASK)

/*! @name BUF1IND - Buffer1 Top Index Register */
#define QuadSPI_BUF1IND_TPINDX1_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF1IND_TPINDX1_SHIFT            (3U)
#define QuadSPI_BUF1IND_TPINDX1(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1IND_TPINDX1_SHIFT)) & QuadSPI_BUF1IND_TPINDX1_MASK)

/*! @name BUF2IND - Buffer2 Top Index Register */
#define QuadSPI_BUF2IND_TPINDX2_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF2IND_TPINDX2_SHIFT            (3U)
#define QuadSPI_BUF2IND_TPINDX2(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2IND_TPINDX2_SHIFT)) & QuadSPI_BUF2IND_TPINDX2_MASK)

/*! @name SFAR - Serial Flash Address Register */
#define QuadSPI_SFAR_SFADR_MASK                  (0xFFFFFFFFU)
#define QuadSPI_SFAR_SFADR_SHIFT                 (0U)
#define QuadSPI_SFAR_SFADR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFAR_SFADR_SHIFT)) & QuadSPI_SFAR_SFADR_MASK)

/*! @name SFACR - Serial Flash Address Configuration Register */
#define QuadSPI_SFACR_CAS_MASK                   (0xFU)
#define QuadSPI_SFACR_CAS_SHIFT                  (0U)
#define QuadSPI_SFACR_CAS(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFACR_CAS_SHIFT)) & QuadSPI_SFACR_CAS_MASK)
#define QuadSPI_SFACR_WA_MASK                    (0x10000U)
#define QuadSPI_SFACR_WA_SHIFT                   (16U)
#define QuadSPI_SFACR_WA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFACR_WA_SHIFT)) & QuadSPI_SFACR_WA_MASK)

/*! @name SMPR - Sampling Register */
#define QuadSPI_SMPR_HSENA_MASK                  (0x1U)
#define QuadSPI_SMPR_HSENA_SHIFT                 (0U)
#define QuadSPI_SMPR_HSENA(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_HSENA_SHIFT)) & QuadSPI_SMPR_HSENA_MASK)
#define QuadSPI_SMPR_HSPHS_MASK                  (0x2U)
#define QuadSPI_SMPR_HSPHS_SHIFT                 (1U)
#define QuadSPI_SMPR_HSPHS(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_HSPHS_SHIFT)) & QuadSPI_SMPR_HSPHS_MASK)
#define QuadSPI_SMPR_HSDLY_MASK                  (0x4U)
#define QuadSPI_SMPR_HSDLY_SHIFT                 (2U)
#define QuadSPI_SMPR_HSDLY(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_HSDLY_SHIFT)) & QuadSPI_SMPR_HSDLY_MASK)
#define QuadSPI_SMPR_FSPHS_MASK                  (0x20U)
#define QuadSPI_SMPR_FSPHS_SHIFT                 (5U)
#define QuadSPI_SMPR_FSPHS(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_FSPHS_SHIFT)) & QuadSPI_SMPR_FSPHS_MASK)
#define QuadSPI_SMPR_FSDLY_MASK                  (0x40U)
#define QuadSPI_SMPR_FSDLY_SHIFT                 (6U)
#define QuadSPI_SMPR_FSDLY(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_FSDLY_SHIFT)) & QuadSPI_SMPR_FSDLY_MASK)
#define QuadSPI_SMPR_DDRSMP_MASK                 (0x70000U)
#define QuadSPI_SMPR_DDRSMP_SHIFT                (16U)
#define QuadSPI_SMPR_DDRSMP(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_DDRSMP_SHIFT)) & QuadSPI_SMPR_DDRSMP_MASK)

/*! @name RBSR - RX Buffer Status Register */
#define QuadSPI_RBSR_RDBFL_MASK                  (0x1F00U)
#define QuadSPI_RBSR_RDBFL_SHIFT                 (8U)
#define QuadSPI_RBSR_RDBFL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBSR_RDBFL_SHIFT)) & QuadSPI_RBSR_RDBFL_MASK)
#define QuadSPI_RBSR_RDCTR_MASK                  (0xFFFF0000U)
#define QuadSPI_RBSR_RDCTR_SHIFT                 (16U)
#define QuadSPI_RBSR_RDCTR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBSR_RDCTR_SHIFT)) & QuadSPI_RBSR_RDCTR_MASK)

/*! @name RBCT - RX Buffer Control Register */
#define QuadSPI_RBCT_WMRK_MASK                   (0xFU)
#define QuadSPI_RBCT_WMRK_SHIFT                  (0U)
#define QuadSPI_RBCT_WMRK(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBCT_WMRK_SHIFT)) & QuadSPI_RBCT_WMRK_MASK)
#define QuadSPI_RBCT_RXBRD_MASK                  (0x100U)
#define QuadSPI_RBCT_RXBRD_SHIFT                 (8U)
#define QuadSPI_RBCT_RXBRD(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBCT_RXBRD_SHIFT)) & QuadSPI_RBCT_RXBRD_MASK)

/*! @name TBSR - TX Buffer Status Register */
#define QuadSPI_TBSR_TRBFL_MASK                  (0x1F00U)
#define QuadSPI_TBSR_TRBFL_SHIFT                 (8U)
#define QuadSPI_TBSR_TRBFL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBSR_TRBFL_SHIFT)) & QuadSPI_TBSR_TRBFL_MASK)
#define QuadSPI_TBSR_TRCTR_MASK                  (0xFFFF0000U)
#define QuadSPI_TBSR_TRCTR_SHIFT                 (16U)
#define QuadSPI_TBSR_TRCTR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBSR_TRCTR_SHIFT)) & QuadSPI_TBSR_TRCTR_MASK)

/*! @name TBDR - TX Buffer Data Register */
#define QuadSPI_TBDR_TXDATA_MASK                 (0xFFFFFFFFU)
#define QuadSPI_TBDR_TXDATA_SHIFT                (0U)
#define QuadSPI_TBDR_TXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBDR_TXDATA_SHIFT)) & QuadSPI_TBDR_TXDATA_MASK)

/*! @name TBCT - Tx Buffer Control Register */
#define QuadSPI_TBCT_WMRK_MASK                   (0xFU)
#define QuadSPI_TBCT_WMRK_SHIFT                  (0U)
#define QuadSPI_TBCT_WMRK(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBCT_WMRK_SHIFT)) & QuadSPI_TBCT_WMRK_MASK)

/*! @name SR - Status Register */
#define QuadSPI_SR_BUSY_MASK                     (0x1U)
#define QuadSPI_SR_BUSY_SHIFT                    (0U)
#define QuadSPI_SR_BUSY(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_BUSY_SHIFT)) & QuadSPI_SR_BUSY_MASK)
#define QuadSPI_SR_IP_ACC_MASK                   (0x2U)
#define QuadSPI_SR_IP_ACC_SHIFT                  (1U)
#define QuadSPI_SR_IP_ACC(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_IP_ACC_SHIFT)) & QuadSPI_SR_IP_ACC_MASK)
#define QuadSPI_SR_AHB_ACC_MASK                  (0x4U)
#define QuadSPI_SR_AHB_ACC_SHIFT                 (2U)
#define QuadSPI_SR_AHB_ACC(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB_ACC_SHIFT)) & QuadSPI_SR_AHB_ACC_MASK)
#define QuadSPI_SR_AHBGNT_MASK                   (0x20U)
#define QuadSPI_SR_AHBGNT_SHIFT                  (5U)
#define QuadSPI_SR_AHBGNT(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHBGNT_SHIFT)) & QuadSPI_SR_AHBGNT_MASK)
#define QuadSPI_SR_AHBTRN_MASK                   (0x40U)
#define QuadSPI_SR_AHBTRN_SHIFT                  (6U)
#define QuadSPI_SR_AHBTRN(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHBTRN_SHIFT)) & QuadSPI_SR_AHBTRN_MASK)
#define QuadSPI_SR_AHB0NE_MASK                   (0x80U)
#define QuadSPI_SR_AHB0NE_SHIFT                  (7U)
#define QuadSPI_SR_AHB0NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB0NE_SHIFT)) & QuadSPI_SR_AHB0NE_MASK)
#define QuadSPI_SR_AHB1NE_MASK                   (0x100U)
#define QuadSPI_SR_AHB1NE_SHIFT                  (8U)
#define QuadSPI_SR_AHB1NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB1NE_SHIFT)) & QuadSPI_SR_AHB1NE_MASK)
#define QuadSPI_SR_AHB2NE_MASK                   (0x200U)
#define QuadSPI_SR_AHB2NE_SHIFT                  (9U)
#define QuadSPI_SR_AHB2NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB2NE_SHIFT)) & QuadSPI_SR_AHB2NE_MASK)
#define QuadSPI_SR_AHB3NE_MASK                   (0x400U)
#define QuadSPI_SR_AHB3NE_SHIFT                  (10U)
#define QuadSPI_SR_AHB3NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB3NE_SHIFT)) & QuadSPI_SR_AHB3NE_MASK)
#define QuadSPI_SR_AHB0FUL_MASK                  (0x800U)
#define QuadSPI_SR_AHB0FUL_SHIFT                 (11U)
#define QuadSPI_SR_AHB0FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB0FUL_SHIFT)) & QuadSPI_SR_AHB0FUL_MASK)
#define QuadSPI_SR_AHB1FUL_MASK                  (0x1000U)
#define QuadSPI_SR_AHB1FUL_SHIFT                 (12U)
#define QuadSPI_SR_AHB1FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB1FUL_SHIFT)) & QuadSPI_SR_AHB1FUL_MASK)
#define QuadSPI_SR_AHB2FUL_MASK                  (0x2000U)
#define QuadSPI_SR_AHB2FUL_SHIFT                 (13U)
#define QuadSPI_SR_AHB2FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB2FUL_SHIFT)) & QuadSPI_SR_AHB2FUL_MASK)
#define QuadSPI_SR_AHB3FUL_MASK                  (0x4000U)
#define QuadSPI_SR_AHB3FUL_SHIFT                 (14U)
#define QuadSPI_SR_AHB3FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB3FUL_SHIFT)) & QuadSPI_SR_AHB3FUL_MASK)
#define QuadSPI_SR_RXWE_MASK                     (0x10000U)
#define QuadSPI_SR_RXWE_SHIFT                    (16U)
#define QuadSPI_SR_RXWE(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXWE_SHIFT)) & QuadSPI_SR_RXWE_MASK)
#define QuadSPI_SR_RXFULL_MASK                   (0x80000U)
#define QuadSPI_SR_RXFULL_SHIFT                  (19U)
#define QuadSPI_SR_RXFULL(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXFULL_SHIFT)) & QuadSPI_SR_RXFULL_MASK)
#define QuadSPI_SR_RXDMA_MASK                    (0x800000U)
#define QuadSPI_SR_RXDMA_SHIFT                   (23U)
#define QuadSPI_SR_RXDMA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXDMA_SHIFT)) & QuadSPI_SR_RXDMA_MASK)
#define QuadSPI_SR_TXEDA_MASK                    (0x1000000U)
#define QuadSPI_SR_TXEDA_SHIFT                   (24U)
#define QuadSPI_SR_TXEDA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXEDA_SHIFT)) & QuadSPI_SR_TXEDA_MASK)
#define QuadSPI_SR_TXWA_MASK                     (0x2000000U)
#define QuadSPI_SR_TXWA_SHIFT                    (25U)
#define QuadSPI_SR_TXWA(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXWA_SHIFT)) & QuadSPI_SR_TXWA_MASK)
#define QuadSPI_SR_TXDMA_MASK                    (0x4000000U)
#define QuadSPI_SR_TXDMA_SHIFT                   (26U)
#define QuadSPI_SR_TXDMA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXDMA_SHIFT)) & QuadSPI_SR_TXDMA_MASK)
#define QuadSPI_SR_TXFULL_MASK                   (0x8000000U)
#define QuadSPI_SR_TXFULL_SHIFT                  (27U)
#define QuadSPI_SR_TXFULL(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXFULL_SHIFT)) & QuadSPI_SR_TXFULL_MASK)
#define QuadSPI_SR_DLPSMP_MASK                   (0xE0000000U)
#define QuadSPI_SR_DLPSMP_SHIFT                  (29U)
#define QuadSPI_SR_DLPSMP(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_DLPSMP_SHIFT)) & QuadSPI_SR_DLPSMP_MASK)

/*! @name FR - Flag Register */
#define QuadSPI_FR_TFF_MASK                      (0x1U)
#define QuadSPI_FR_TFF_SHIFT                     (0U)
#define QuadSPI_FR_TFF(x)                        (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TFF_SHIFT)) & QuadSPI_FR_TFF_MASK)
#define QuadSPI_FR_IPGEF_MASK                    (0x10U)
#define QuadSPI_FR_IPGEF_SHIFT                   (4U)
#define QuadSPI_FR_IPGEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IPGEF_SHIFT)) & QuadSPI_FR_IPGEF_MASK)
#define QuadSPI_FR_IPIEF_MASK                    (0x40U)
#define QuadSPI_FR_IPIEF_SHIFT                   (6U)
#define QuadSPI_FR_IPIEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IPIEF_SHIFT)) & QuadSPI_FR_IPIEF_MASK)
#define QuadSPI_FR_IPAEF_MASK                    (0x80U)
#define QuadSPI_FR_IPAEF_SHIFT                   (7U)
#define QuadSPI_FR_IPAEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IPAEF_SHIFT)) & QuadSPI_FR_IPAEF_MASK)
#define QuadSPI_FR_IUEF_MASK                     (0x800U)
#define QuadSPI_FR_IUEF_SHIFT                    (11U)
#define QuadSPI_FR_IUEF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IUEF_SHIFT)) & QuadSPI_FR_IUEF_MASK)
#define QuadSPI_FR_ABOF_MASK                     (0x1000U)
#define QuadSPI_FR_ABOF_SHIFT                    (12U)
#define QuadSPI_FR_ABOF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ABOF_SHIFT)) & QuadSPI_FR_ABOF_MASK)
#define QuadSPI_FR_AIBSEF_MASK                   (0x2000U)
#define QuadSPI_FR_AIBSEF_SHIFT                  (13U)
#define QuadSPI_FR_AIBSEF(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_AIBSEF_SHIFT)) & QuadSPI_FR_AIBSEF_MASK)
#define QuadSPI_FR_AITEF_MASK                    (0x4000U)
#define QuadSPI_FR_AITEF_SHIFT                   (14U)
#define QuadSPI_FR_AITEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_AITEF_SHIFT)) & QuadSPI_FR_AITEF_MASK)
#define QuadSPI_FR_ABSEF_MASK                    (0x8000U)
#define QuadSPI_FR_ABSEF_SHIFT                   (15U)
#define QuadSPI_FR_ABSEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ABSEF_SHIFT)) & QuadSPI_FR_ABSEF_MASK)
#define QuadSPI_FR_RBDF_MASK                     (0x10000U)
#define QuadSPI_FR_RBDF_SHIFT                    (16U)
#define QuadSPI_FR_RBDF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_RBDF_SHIFT)) & QuadSPI_FR_RBDF_MASK)
#define QuadSPI_FR_RBOF_MASK                     (0x20000U)
#define QuadSPI_FR_RBOF_SHIFT                    (17U)
#define QuadSPI_FR_RBOF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_RBOF_SHIFT)) & QuadSPI_FR_RBOF_MASK)
#define QuadSPI_FR_ILLINE_MASK                   (0x800000U)
#define QuadSPI_FR_ILLINE_SHIFT                  (23U)
#define QuadSPI_FR_ILLINE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ILLINE_SHIFT)) & QuadSPI_FR_ILLINE_MASK)
#define QuadSPI_FR_TBUF_MASK                     (0x4000000U)
#define QuadSPI_FR_TBUF_SHIFT                    (26U)
#define QuadSPI_FR_TBUF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TBUF_SHIFT)) & QuadSPI_FR_TBUF_MASK)
#define QuadSPI_FR_TBFF_MASK                     (0x8000000U)
#define QuadSPI_FR_TBFF_SHIFT                    (27U)
#define QuadSPI_FR_TBFF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TBFF_SHIFT)) & QuadSPI_FR_TBFF_MASK)
#define QuadSPI_FR_DLPFF_MASK                    (0x80000000U)
#define QuadSPI_FR_DLPFF_SHIFT                   (31U)
#define QuadSPI_FR_DLPFF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_DLPFF_SHIFT)) & QuadSPI_FR_DLPFF_MASK)

/*! @name RSER - Interrupt and DMA Request Select and Enable Register */
#define QuadSPI_RSER_TFIE_MASK                   (0x1U)
#define QuadSPI_RSER_TFIE_SHIFT                  (0U)
#define QuadSPI_RSER_TFIE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TFIE_SHIFT)) & QuadSPI_RSER_TFIE_MASK)
#define QuadSPI_RSER_IPGEIE_MASK                 (0x10U)
#define QuadSPI_RSER_IPGEIE_SHIFT                (4U)
#define QuadSPI_RSER_IPGEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IPGEIE_SHIFT)) & QuadSPI_RSER_IPGEIE_MASK)
#define QuadSPI_RSER_IPIEIE_MASK                 (0x40U)
#define QuadSPI_RSER_IPIEIE_SHIFT                (6U)
#define QuadSPI_RSER_IPIEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IPIEIE_SHIFT)) & QuadSPI_RSER_IPIEIE_MASK)
#define QuadSPI_RSER_IPAEIE_MASK                 (0x80U)
#define QuadSPI_RSER_IPAEIE_SHIFT                (7U)
#define QuadSPI_RSER_IPAEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IPAEIE_SHIFT)) & QuadSPI_RSER_IPAEIE_MASK)
#define QuadSPI_RSER_IUEIE_MASK                  (0x800U)
#define QuadSPI_RSER_IUEIE_SHIFT                 (11U)
#define QuadSPI_RSER_IUEIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IUEIE_SHIFT)) & QuadSPI_RSER_IUEIE_MASK)
#define QuadSPI_RSER_ABOIE_MASK                  (0x1000U)
#define QuadSPI_RSER_ABOIE_SHIFT                 (12U)
#define QuadSPI_RSER_ABOIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ABOIE_SHIFT)) & QuadSPI_RSER_ABOIE_MASK)
#define QuadSPI_RSER_AIBSIE_MASK                 (0x2000U)
#define QuadSPI_RSER_AIBSIE_SHIFT                (13U)
#define QuadSPI_RSER_AIBSIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_AIBSIE_SHIFT)) & QuadSPI_RSER_AIBSIE_MASK)
#define QuadSPI_RSER_AITIE_MASK                  (0x4000U)
#define QuadSPI_RSER_AITIE_SHIFT                 (14U)
#define QuadSPI_RSER_AITIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_AITIE_SHIFT)) & QuadSPI_RSER_AITIE_MASK)
#define QuadSPI_RSER_ABSEIE_MASK                 (0x8000U)
#define QuadSPI_RSER_ABSEIE_SHIFT                (15U)
#define QuadSPI_RSER_ABSEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ABSEIE_SHIFT)) & QuadSPI_RSER_ABSEIE_MASK)
#define QuadSPI_RSER_RBDIE_MASK                  (0x10000U)
#define QuadSPI_RSER_RBDIE_SHIFT                 (16U)
#define QuadSPI_RSER_RBDIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBDIE_SHIFT)) & QuadSPI_RSER_RBDIE_MASK)
#define QuadSPI_RSER_RBOIE_MASK                  (0x20000U)
#define QuadSPI_RSER_RBOIE_SHIFT                 (17U)
#define QuadSPI_RSER_RBOIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBOIE_SHIFT)) & QuadSPI_RSER_RBOIE_MASK)
#define QuadSPI_RSER_RBDDE_MASK                  (0x200000U)
#define QuadSPI_RSER_RBDDE_SHIFT                 (21U)
#define QuadSPI_RSER_RBDDE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBDDE_SHIFT)) & QuadSPI_RSER_RBDDE_MASK)
#define QuadSPI_RSER_ILLINIE_MASK                (0x800000U)
#define QuadSPI_RSER_ILLINIE_SHIFT               (23U)
#define QuadSPI_RSER_ILLINIE(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ILLINIE_SHIFT)) & QuadSPI_RSER_ILLINIE_MASK)
#define QuadSPI_RSER_TBFDE_MASK                  (0x2000000U)
#define QuadSPI_RSER_TBFDE_SHIFT                 (25U)
#define QuadSPI_RSER_TBFDE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBFDE_SHIFT)) & QuadSPI_RSER_TBFDE_MASK)
#define QuadSPI_RSER_TBUIE_MASK                  (0x4000000U)
#define QuadSPI_RSER_TBUIE_SHIFT                 (26U)
#define QuadSPI_RSER_TBUIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBUIE_SHIFT)) & QuadSPI_RSER_TBUIE_MASK)
#define QuadSPI_RSER_TBFIE_MASK                  (0x8000000U)
#define QuadSPI_RSER_TBFIE_SHIFT                 (27U)
#define QuadSPI_RSER_TBFIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBFIE_SHIFT)) & QuadSPI_RSER_TBFIE_MASK)
#define QuadSPI_RSER_DLPFIE_MASK                 (0x80000000U)
#define QuadSPI_RSER_DLPFIE_SHIFT                (31U)
#define QuadSPI_RSER_DLPFIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_DLPFIE_SHIFT)) & QuadSPI_RSER_DLPFIE_MASK)

/*! @name SPNDST - Sequence Suspend Status Register */
#define QuadSPI_SPNDST_SUSPND_MASK               (0x1U)
#define QuadSPI_SPNDST_SUSPND_SHIFT              (0U)
#define QuadSPI_SPNDST_SUSPND(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_SUSPND_SHIFT)) & QuadSPI_SPNDST_SUSPND_MASK)
#define QuadSPI_SPNDST_SPDBUF_MASK               (0xC0U)
#define QuadSPI_SPNDST_SPDBUF_SHIFT              (6U)
#define QuadSPI_SPNDST_SPDBUF(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_SPDBUF_SHIFT)) & QuadSPI_SPNDST_SPDBUF_MASK)
#define QuadSPI_SPNDST_DATLFT_MASK               (0x7E00U)
#define QuadSPI_SPNDST_DATLFT_SHIFT              (9U)
#define QuadSPI_SPNDST_DATLFT(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_DATLFT_SHIFT)) & QuadSPI_SPNDST_DATLFT_MASK)

/*! @name SPTRCLR - Sequence Pointer Clear Register */
#define QuadSPI_SPTRCLR_BFPTRC_MASK              (0x1U)
#define QuadSPI_SPTRCLR_BFPTRC_SHIFT             (0U)
#define QuadSPI_SPTRCLR_BFPTRC(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPTRCLR_BFPTRC_SHIFT)) & QuadSPI_SPTRCLR_BFPTRC_MASK)
#define QuadSPI_SPTRCLR_IPPTRC_MASK              (0x100U)
#define QuadSPI_SPTRCLR_IPPTRC_SHIFT             (8U)
#define QuadSPI_SPTRCLR_IPPTRC(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPTRCLR_IPPTRC_SHIFT)) & QuadSPI_SPTRCLR_IPPTRC_MASK)

/*! @name SFA1AD - Serial Flash A1 Top Address */
#define QuadSPI_SFA1AD_TPADA1_MASK               (0xFFFFFC00U)
#define QuadSPI_SFA1AD_TPADA1_SHIFT              (10U)
#define QuadSPI_SFA1AD_TPADA1(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFA1AD_TPADA1_SHIFT)) & QuadSPI_SFA1AD_TPADA1_MASK)

/*! @name SFA2AD - Serial Flash A2 Top Address */
#define QuadSPI_SFA2AD_TPADA2_MASK               (0xFFFFFC00U)
#define QuadSPI_SFA2AD_TPADA2_SHIFT              (10U)
#define QuadSPI_SFA2AD_TPADA2(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFA2AD_TPADA2_SHIFT)) & QuadSPI_SFA2AD_TPADA2_MASK)

/*! @name SFB1AD - Serial Flash B1Top Address */
#define QuadSPI_SFB1AD_TPADB1_MASK               (0xFFFFFC00U)
#define QuadSPI_SFB1AD_TPADB1_SHIFT              (10U)
#define QuadSPI_SFB1AD_TPADB1(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFB1AD_TPADB1_SHIFT)) & QuadSPI_SFB1AD_TPADB1_MASK)

/*! @name SFB2AD - Serial Flash B2Top Address */
#define QuadSPI_SFB2AD_TPADB2_MASK               (0xFFFFFC00U)
#define QuadSPI_SFB2AD_TPADB2_SHIFT              (10U)
#define QuadSPI_SFB2AD_TPADB2(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFB2AD_TPADB2_SHIFT)) & QuadSPI_SFB2AD_TPADB2_MASK)

/*! @name DLPR - Data Learn Pattern Register */
#define QuadSPI_DLPR_DLPV_MASK                   (0xFFFFFFFFU)
#define QuadSPI_DLPR_DLPV_SHIFT                  (0U)
#define QuadSPI_DLPR_DLPV(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_DLPR_DLPV_SHIFT)) & QuadSPI_DLPR_DLPV_MASK)

/*! @name RBDR - RX Buffer Data Register */
#define QuadSPI_RBDR_RXDATA_MASK                 (0xFFFFFFFFU)
#define QuadSPI_RBDR_RXDATA_SHIFT                (0U)
#define QuadSPI_RBDR_RXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBDR_RXDATA_SHIFT)) & QuadSPI_RBDR_RXDATA_MASK)

/* The count of QuadSPI_RBDR */
#define QuadSPI_RBDR_COUNT                       (16U)

/*! @name LUTKEY - LUT Key Register */
#define QuadSPI_LUTKEY_KEY_MASK                  (0xFFFFFFFFU)
#define QuadSPI_LUTKEY_KEY_SHIFT                 (0U)
#define QuadSPI_LUTKEY_KEY(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUTKEY_KEY_SHIFT)) & QuadSPI_LUTKEY_KEY_MASK)

/*! @name LCKCR - LUT Lock Configuration Register */
#define QuadSPI_LCKCR_LOCK_MASK                  (0x1U)
#define QuadSPI_LCKCR_LOCK_SHIFT                 (0U)
#define QuadSPI_LCKCR_LOCK(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LCKCR_LOCK_SHIFT)) & QuadSPI_LCKCR_LOCK_MASK)
#define QuadSPI_LCKCR_UNLOCK_MASK                (0x2U)
#define QuadSPI_LCKCR_UNLOCK_SHIFT               (1U)
#define QuadSPI_LCKCR_UNLOCK(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_LCKCR_UNLOCK_SHIFT)) & QuadSPI_LCKCR_UNLOCK_MASK)

/*! @name LUT - Look-up Table register */
#define QuadSPI_LUT_OPRND0_MASK                  (0xFFU)
#define QuadSPI_LUT_OPRND0_SHIFT                 (0U)
#define QuadSPI_LUT_OPRND0(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_OPRND0_SHIFT)) & QuadSPI_LUT_OPRND0_MASK)
#define QuadSPI_LUT_PAD0_MASK                    (0x300U)
#define QuadSPI_LUT_PAD0_SHIFT                   (8U)
#define QuadSPI_LUT_PAD0(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_PAD0_SHIFT)) & QuadSPI_LUT_PAD0_MASK)
#define QuadSPI_LUT_INSTR0_MASK                  (0xFC00U)
#define QuadSPI_LUT_INSTR0_SHIFT                 (10U)
#define QuadSPI_LUT_INSTR0(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_INSTR0_SHIFT)) & QuadSPI_LUT_INSTR0_MASK)
#define QuadSPI_LUT_OPRND1_MASK                  (0xFF0000U)
#define QuadSPI_LUT_OPRND1_SHIFT                 (16U)
#define QuadSPI_LUT_OPRND1(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_OPRND1_SHIFT)) & QuadSPI_LUT_OPRND1_MASK)
#define QuadSPI_LUT_PAD1_MASK                    (0x3000000U)
#define QuadSPI_LUT_PAD1_SHIFT                   (24U)
#define QuadSPI_LUT_PAD1(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_PAD1_SHIFT)) & QuadSPI_LUT_PAD1_MASK)
#define QuadSPI_LUT_INSTR1_MASK                  (0xFC000000U)
#define QuadSPI_LUT_INSTR1_SHIFT                 (26U)
#define QuadSPI_LUT_INSTR1(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_INSTR1_SHIFT)) & QuadSPI_LUT_INSTR1_MASK)

/* The count of QuadSPI_LUT */
#define QuadSPI_LUT_COUNT                        (64U)


/*!
 * @}
 */ /* end of group QuadSPI_Register_Masks */


/* QuadSPI - Peripheral instance base addresses */
/** Peripheral QuadSPI0 base address */
#define QuadSPI0_BASE                            (0x400DA000u)
/** Peripheral QuadSPI0 base pointer */
#define QuadSPI0                                 ((QuadSPI_Type *)QuadSPI0_BASE)
/** Array initializer of QuadSPI peripheral base addresses */
#define QuadSPI_BASE_ADDRS                       { QuadSPI0_BASE }
/** Array initializer of QuadSPI peripheral base pointers */
#define QuadSPI_BASE_PTRS                        { QuadSPI0 }
/** Interrupt vectors for the QuadSPI peripheral type */
#define QuadSPI_IRQS                             { QuadSPI0_IRQn }

/*!
 * @}
 */ /* end of group QuadSPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Peripheral_Access_Layer RCM Peripheral Access Layer
 * @{
 */

/** RCM - Register Layout Typedef */
typedef struct {
  __I  uint8_t SRS0;                               /**< System Reset Status Register 0, offset: 0x0 */
  __I  uint8_t SRS1;                               /**< System Reset Status Register 1, offset: 0x1 */
       uint8_t RESERVED_0[2];
  __IO uint8_t RPFC;                               /**< Reset Pin Filter Control register, offset: 0x4 */
  __IO uint8_t RPFW;                               /**< Reset Pin Filter Width register, offset: 0x5 */
  __IO uint8_t FM;                                 /**< Force Mode Register, offset: 0x6 */
  __IO uint8_t MR;                                 /**< Mode Register, offset: 0x7 */
  __IO uint8_t SSRS0;                              /**< Sticky System Reset Status Register 0, offset: 0x8 */
  __IO uint8_t SSRS1;                              /**< Sticky System Reset Status Register 1, offset: 0x9 */
} RCM_Type;

/* ----------------------------------------------------------------------------
   -- RCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Masks RCM Register Masks
 * @{
 */

/*! @name SRS0 - System Reset Status Register 0 */
#define RCM_SRS0_WAKEUP_MASK                     (0x1U)
#define RCM_SRS0_WAKEUP_SHIFT                    (0U)
#define RCM_SRS0_WAKEUP(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_WAKEUP_SHIFT)) & RCM_SRS0_WAKEUP_MASK)
#define RCM_SRS0_LVD_MASK                        (0x2U)
#define RCM_SRS0_LVD_SHIFT                       (1U)
#define RCM_SRS0_LVD(x)                          (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_LVD_SHIFT)) & RCM_SRS0_LVD_MASK)
#define RCM_SRS0_LOC_MASK                        (0x4U)
#define RCM_SRS0_LOC_SHIFT                       (2U)
#define RCM_SRS0_LOC(x)                          (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_LOC_SHIFT)) & RCM_SRS0_LOC_MASK)
#define RCM_SRS0_LOL_MASK                        (0x8U)
#define RCM_SRS0_LOL_SHIFT                       (3U)
#define RCM_SRS0_LOL(x)                          (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_LOL_SHIFT)) & RCM_SRS0_LOL_MASK)
#define RCM_SRS0_WDOG_MASK                       (0x20U)
#define RCM_SRS0_WDOG_SHIFT                      (5U)
#define RCM_SRS0_WDOG(x)                         (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_WDOG_SHIFT)) & RCM_SRS0_WDOG_MASK)
#define RCM_SRS0_PIN_MASK                        (0x40U)
#define RCM_SRS0_PIN_SHIFT                       (6U)
#define RCM_SRS0_PIN(x)                          (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_PIN_SHIFT)) & RCM_SRS0_PIN_MASK)
#define RCM_SRS0_POR_MASK                        (0x80U)
#define RCM_SRS0_POR_SHIFT                       (7U)
#define RCM_SRS0_POR(x)                          (((uint8_t)(((uint8_t)(x)) << RCM_SRS0_POR_SHIFT)) & RCM_SRS0_POR_MASK)

/*! @name SRS1 - System Reset Status Register 1 */
#define RCM_SRS1_JTAG_MASK                       (0x1U)
#define RCM_SRS1_JTAG_SHIFT                      (0U)
#define RCM_SRS1_JTAG(x)                         (((uint8_t)(((uint8_t)(x)) << RCM_SRS1_JTAG_SHIFT)) & RCM_SRS1_JTAG_MASK)
#define RCM_SRS1_LOCKUP_MASK                     (0x2U)
#define RCM_SRS1_LOCKUP_SHIFT                    (1U)
#define RCM_SRS1_LOCKUP(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_SRS1_LOCKUP_SHIFT)) & RCM_SRS1_LOCKUP_MASK)
#define RCM_SRS1_SW_MASK                         (0x4U)
#define RCM_SRS1_SW_SHIFT                        (2U)
#define RCM_SRS1_SW(x)                           (((uint8_t)(((uint8_t)(x)) << RCM_SRS1_SW_SHIFT)) & RCM_SRS1_SW_MASK)
#define RCM_SRS1_MDM_AP_MASK                     (0x8U)
#define RCM_SRS1_MDM_AP_SHIFT                    (3U)
#define RCM_SRS1_MDM_AP(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_SRS1_MDM_AP_SHIFT)) & RCM_SRS1_MDM_AP_MASK)
#define RCM_SRS1_SACKERR_MASK                    (0x20U)
#define RCM_SRS1_SACKERR_SHIFT                   (5U)
#define RCM_SRS1_SACKERR(x)                      (((uint8_t)(((uint8_t)(x)) << RCM_SRS1_SACKERR_SHIFT)) & RCM_SRS1_SACKERR_MASK)

/*! @name RPFC - Reset Pin Filter Control register */
#define RCM_RPFC_RSTFLTSRW_MASK                  (0x3U)
#define RCM_RPFC_RSTFLTSRW_SHIFT                 (0U)
#define RCM_RPFC_RSTFLTSRW(x)                    (((uint8_t)(((uint8_t)(x)) << RCM_RPFC_RSTFLTSRW_SHIFT)) & RCM_RPFC_RSTFLTSRW_MASK)
#define RCM_RPFC_RSTFLTSS_MASK                   (0x4U)
#define RCM_RPFC_RSTFLTSS_SHIFT                  (2U)
#define RCM_RPFC_RSTFLTSS(x)                     (((uint8_t)(((uint8_t)(x)) << RCM_RPFC_RSTFLTSS_SHIFT)) & RCM_RPFC_RSTFLTSS_MASK)

/*! @name RPFW - Reset Pin Filter Width register */
#define RCM_RPFW_RSTFLTSEL_MASK                  (0x1FU)
#define RCM_RPFW_RSTFLTSEL_SHIFT                 (0U)
#define RCM_RPFW_RSTFLTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << RCM_RPFW_RSTFLTSEL_SHIFT)) & RCM_RPFW_RSTFLTSEL_MASK)

/*! @name FM - Force Mode Register */
#define RCM_FM_FORCEROM_MASK                     (0x6U)
#define RCM_FM_FORCEROM_SHIFT                    (1U)
#define RCM_FM_FORCEROM(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_FM_FORCEROM_SHIFT)) & RCM_FM_FORCEROM_MASK)

/*! @name MR - Mode Register */
#define RCM_MR_BOOTROM_MASK                      (0x6U)
#define RCM_MR_BOOTROM_SHIFT                     (1U)
#define RCM_MR_BOOTROM(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_MR_BOOTROM_SHIFT)) & RCM_MR_BOOTROM_MASK)

/*! @name SSRS0 - Sticky System Reset Status Register 0 */
#define RCM_SSRS0_SWAKEUP_MASK                   (0x1U)
#define RCM_SSRS0_SWAKEUP_SHIFT                  (0U)
#define RCM_SSRS0_SWAKEUP(x)                     (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SWAKEUP_SHIFT)) & RCM_SSRS0_SWAKEUP_MASK)
#define RCM_SSRS0_SLVD_MASK                      (0x2U)
#define RCM_SSRS0_SLVD_SHIFT                     (1U)
#define RCM_SSRS0_SLVD(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SLVD_SHIFT)) & RCM_SSRS0_SLVD_MASK)
#define RCM_SSRS0_SLOC_MASK                      (0x4U)
#define RCM_SSRS0_SLOC_SHIFT                     (2U)
#define RCM_SSRS0_SLOC(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SLOC_SHIFT)) & RCM_SSRS0_SLOC_MASK)
#define RCM_SSRS0_SLOL_MASK                      (0x8U)
#define RCM_SSRS0_SLOL_SHIFT                     (3U)
#define RCM_SSRS0_SLOL(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SLOL_SHIFT)) & RCM_SSRS0_SLOL_MASK)
#define RCM_SSRS0_SWDOG_MASK                     (0x20U)
#define RCM_SSRS0_SWDOG_SHIFT                    (5U)
#define RCM_SSRS0_SWDOG(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SWDOG_SHIFT)) & RCM_SSRS0_SWDOG_MASK)
#define RCM_SSRS0_SPIN_MASK                      (0x40U)
#define RCM_SSRS0_SPIN_SHIFT                     (6U)
#define RCM_SSRS0_SPIN(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SPIN_SHIFT)) & RCM_SSRS0_SPIN_MASK)
#define RCM_SSRS0_SPOR_MASK                      (0x80U)
#define RCM_SSRS0_SPOR_SHIFT                     (7U)
#define RCM_SSRS0_SPOR(x)                        (((uint8_t)(((uint8_t)(x)) << RCM_SSRS0_SPOR_SHIFT)) & RCM_SSRS0_SPOR_MASK)

/*! @name SSRS1 - Sticky System Reset Status Register 1 */
#define RCM_SSRS1_SJTAG_MASK                     (0x1U)
#define RCM_SSRS1_SJTAG_SHIFT                    (0U)
#define RCM_SSRS1_SJTAG(x)                       (((uint8_t)(((uint8_t)(x)) << RCM_SSRS1_SJTAG_SHIFT)) & RCM_SSRS1_SJTAG_MASK)
#define RCM_SSRS1_SLOCKUP_MASK                   (0x2U)
#define RCM_SSRS1_SLOCKUP_SHIFT                  (1U)
#define RCM_SSRS1_SLOCKUP(x)                     (((uint8_t)(((uint8_t)(x)) << RCM_SSRS1_SLOCKUP_SHIFT)) & RCM_SSRS1_SLOCKUP_MASK)
#define RCM_SSRS1_SSW_MASK                       (0x4U)
#define RCM_SSRS1_SSW_SHIFT                      (2U)
#define RCM_SSRS1_SSW(x)                         (((uint8_t)(((uint8_t)(x)) << RCM_SSRS1_SSW_SHIFT)) & RCM_SSRS1_SSW_MASK)
#define RCM_SSRS1_SMDM_AP_MASK                   (0x8U)
#define RCM_SSRS1_SMDM_AP_SHIFT                  (3U)
#define RCM_SSRS1_SMDM_AP(x)                     (((uint8_t)(((uint8_t)(x)) << RCM_SSRS1_SMDM_AP_SHIFT)) & RCM_SSRS1_SMDM_AP_MASK)
#define RCM_SSRS1_SSACKERR_MASK                  (0x20U)
#define RCM_SSRS1_SSACKERR_SHIFT                 (5U)
#define RCM_SSRS1_SSACKERR(x)                    (((uint8_t)(((uint8_t)(x)) << RCM_SSRS1_SSACKERR_SHIFT)) & RCM_SSRS1_SSACKERR_MASK)


/*!
 * @}
 */ /* end of group RCM_Register_Masks */


/* RCM - Peripheral instance base addresses */
/** Peripheral RCM base address */
#define RCM_BASE                                 (0x4007F000u)
/** Peripheral RCM base pointer */
#define RCM                                      ((RCM_Type *)RCM_BASE)
/** Array initializer of RCM peripheral base addresses */
#define RCM_BASE_ADDRS                           { RCM_BASE }
/** Array initializer of RCM peripheral base pointers */
#define RCM_BASE_PTRS                            { RCM }

/*!
 * @}
 */ /* end of group RCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RFSYS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Peripheral_Access_Layer RFSYS Peripheral Access Layer
 * @{
 */

/** RFSYS - Register Layout Typedef */
typedef struct {
  __IO uint32_t REG[8];                            /**< Register file register, array offset: 0x0, array step: 0x4 */
} RFSYS_Type;

/* ----------------------------------------------------------------------------
   -- RFSYS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Register_Masks RFSYS Register Masks
 * @{
 */

/*! @name REG - Register file register */
#define RFSYS_REG_LL_MASK                        (0xFFU)
#define RFSYS_REG_LL_SHIFT                       (0U)
#define RFSYS_REG_LL(x)                          (((uint32_t)(((uint32_t)(x)) << RFSYS_REG_LL_SHIFT)) & RFSYS_REG_LL_MASK)
#define RFSYS_REG_LH_MASK                        (0xFF00U)
#define RFSYS_REG_LH_SHIFT                       (8U)
#define RFSYS_REG_LH(x)                          (((uint32_t)(((uint32_t)(x)) << RFSYS_REG_LH_SHIFT)) & RFSYS_REG_LH_MASK)
#define RFSYS_REG_HL_MASK                        (0xFF0000U)
#define RFSYS_REG_HL_SHIFT                       (16U)
#define RFSYS_REG_HL(x)                          (((uint32_t)(((uint32_t)(x)) << RFSYS_REG_HL_SHIFT)) & RFSYS_REG_HL_MASK)
#define RFSYS_REG_HH_MASK                        (0xFF000000U)
#define RFSYS_REG_HH_SHIFT                       (24U)
#define RFSYS_REG_HH(x)                          (((uint32_t)(((uint32_t)(x)) << RFSYS_REG_HH_SHIFT)) & RFSYS_REG_HH_MASK)

/* The count of RFSYS_REG */
#define RFSYS_REG_COUNT                          (8U)


/*!
 * @}
 */ /* end of group RFSYS_Register_Masks */


/* RFSYS - Peripheral instance base addresses */
/** Peripheral RFSYS base address */
#define RFSYS_BASE                               (0x40041000u)
/** Peripheral RFSYS base pointer */
#define RFSYS                                    ((RFSYS_Type *)RFSYS_BASE)
/** Array initializer of RFSYS peripheral base addresses */
#define RFSYS_BASE_ADDRS                         { RFSYS_BASE }
/** Array initializer of RFSYS peripheral base pointers */
#define RFSYS_BASE_PTRS                          { RFSYS }

/*!
 * @}
 */ /* end of group RFSYS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RFVBAT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Peripheral_Access_Layer RFVBAT Peripheral Access Layer
 * @{
 */

/** RFVBAT - Register Layout Typedef */
typedef struct {
  __IO uint32_t REG[8];                            /**< VBAT register file register, array offset: 0x0, array step: 0x4 */
} RFVBAT_Type;

/* ----------------------------------------------------------------------------
   -- RFVBAT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Register_Masks RFVBAT Register Masks
 * @{
 */

/*! @name REG - VBAT register file register */
#define RFVBAT_REG_LL_MASK                       (0xFFU)
#define RFVBAT_REG_LL_SHIFT                      (0U)
#define RFVBAT_REG_LL(x)                         (((uint32_t)(((uint32_t)(x)) << RFVBAT_REG_LL_SHIFT)) & RFVBAT_REG_LL_MASK)
#define RFVBAT_REG_LH_MASK                       (0xFF00U)
#define RFVBAT_REG_LH_SHIFT                      (8U)
#define RFVBAT_REG_LH(x)                         (((uint32_t)(((uint32_t)(x)) << RFVBAT_REG_LH_SHIFT)) & RFVBAT_REG_LH_MASK)
#define RFVBAT_REG_HL_MASK                       (0xFF0000U)
#define RFVBAT_REG_HL_SHIFT                      (16U)
#define RFVBAT_REG_HL(x)                         (((uint32_t)(((uint32_t)(x)) << RFVBAT_REG_HL_SHIFT)) & RFVBAT_REG_HL_MASK)
#define RFVBAT_REG_HH_MASK                       (0xFF000000U)
#define RFVBAT_REG_HH_SHIFT                      (24U)
#define RFVBAT_REG_HH(x)                         (((uint32_t)(((uint32_t)(x)) << RFVBAT_REG_HH_SHIFT)) & RFVBAT_REG_HH_MASK)

/* The count of RFVBAT_REG */
#define RFVBAT_REG_COUNT                         (8U)


/*!
 * @}
 */ /* end of group RFVBAT_Register_Masks */


/* RFVBAT - Peripheral instance base addresses */
/** Peripheral RFVBAT base address */
#define RFVBAT_BASE                              (0x4003E000u)
/** Peripheral RFVBAT base pointer */
#define RFVBAT                                   ((RFVBAT_Type *)RFVBAT_BASE)
/** Array initializer of RFVBAT peripheral base addresses */
#define RFVBAT_BASE_ADDRS                        { RFVBAT_BASE }
/** Array initializer of RFVBAT peripheral base pointers */
#define RFVBAT_BASE_PTRS                         { RFVBAT }

/*!
 * @}
 */ /* end of group RFVBAT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Peripheral_Access_Layer RTC Peripheral Access Layer
 * @{
 */

/** RTC - Register Layout Typedef */
typedef struct {
  __IO uint32_t TSR;                               /**< RTC Time Seconds Register, offset: 0x0 */
  __IO uint32_t TPR;                               /**< RTC Time Prescaler Register, offset: 0x4 */
  __IO uint32_t TAR;                               /**< RTC Time Alarm Register, offset: 0x8 */
  __IO uint32_t TCR;                               /**< RTC Time Compensation Register, offset: 0xC */
  __IO uint32_t CR;                                /**< RTC Control Register, offset: 0x10 */
  __IO uint32_t SR;                                /**< RTC Status Register, offset: 0x14 */
  __IO uint32_t LR;                                /**< RTC Lock Register, offset: 0x18 */
  __IO uint32_t IER;                               /**< RTC Interrupt Enable Register, offset: 0x1C */
       uint8_t RESERVED_0[2016];
  __IO uint32_t WAR;                               /**< RTC Write Access Register, offset: 0x800 */
  __IO uint32_t RAR;                               /**< RTC Read Access Register, offset: 0x804 */
} RTC_Type;

/* ----------------------------------------------------------------------------
   -- RTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Masks RTC Register Masks
 * @{
 */

/*! @name TSR - RTC Time Seconds Register */
#define RTC_TSR_TSR_MASK                         (0xFFFFFFFFU)
#define RTC_TSR_TSR_SHIFT                        (0U)
#define RTC_TSR_TSR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TSR_TSR_SHIFT)) & RTC_TSR_TSR_MASK)

/*! @name TPR - RTC Time Prescaler Register */
#define RTC_TPR_TPR_MASK                         (0xFFFFU)
#define RTC_TPR_TPR_SHIFT                        (0U)
#define RTC_TPR_TPR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TPR_TPR_SHIFT)) & RTC_TPR_TPR_MASK)

/*! @name TAR - RTC Time Alarm Register */
#define RTC_TAR_TAR_MASK                         (0xFFFFFFFFU)
#define RTC_TAR_TAR_SHIFT                        (0U)
#define RTC_TAR_TAR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TAR_TAR_SHIFT)) & RTC_TAR_TAR_MASK)

/*! @name TCR - RTC Time Compensation Register */
#define RTC_TCR_TCR_MASK                         (0xFFU)
#define RTC_TCR_TCR_SHIFT                        (0U)
#define RTC_TCR_TCR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_TCR_SHIFT)) & RTC_TCR_TCR_MASK)
#define RTC_TCR_CIR_MASK                         (0xFF00U)
#define RTC_TCR_CIR_SHIFT                        (8U)
#define RTC_TCR_CIR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_CIR_SHIFT)) & RTC_TCR_CIR_MASK)
#define RTC_TCR_TCV_MASK                         (0xFF0000U)
#define RTC_TCR_TCV_SHIFT                        (16U)
#define RTC_TCR_TCV(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_TCV_SHIFT)) & RTC_TCR_TCV_MASK)
#define RTC_TCR_CIC_MASK                         (0xFF000000U)
#define RTC_TCR_CIC_SHIFT                        (24U)
#define RTC_TCR_CIC(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_CIC_SHIFT)) & RTC_TCR_CIC_MASK)

/*! @name CR - RTC Control Register */
#define RTC_CR_SWR_MASK                          (0x1U)
#define RTC_CR_SWR_SHIFT                         (0U)
#define RTC_CR_SWR(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_SWR_SHIFT)) & RTC_CR_SWR_MASK)
#define RTC_CR_WPE_MASK                          (0x2U)
#define RTC_CR_WPE_SHIFT                         (1U)
#define RTC_CR_WPE(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_WPE_SHIFT)) & RTC_CR_WPE_MASK)
#define RTC_CR_SUP_MASK                          (0x4U)
#define RTC_CR_SUP_SHIFT                         (2U)
#define RTC_CR_SUP(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_SUP_SHIFT)) & RTC_CR_SUP_MASK)
#define RTC_CR_UM_MASK                           (0x8U)
#define RTC_CR_UM_SHIFT                          (3U)
#define RTC_CR_UM(x)                             (((uint32_t)(((uint32_t)(x)) << RTC_CR_UM_SHIFT)) & RTC_CR_UM_MASK)
#define RTC_CR_WPS_MASK                          (0x10U)
#define RTC_CR_WPS_SHIFT                         (4U)
#define RTC_CR_WPS(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_WPS_SHIFT)) & RTC_CR_WPS_MASK)
#define RTC_CR_OSCE_MASK                         (0x100U)
#define RTC_CR_OSCE_SHIFT                        (8U)
#define RTC_CR_OSCE(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_OSCE_SHIFT)) & RTC_CR_OSCE_MASK)
#define RTC_CR_CLKO_MASK                         (0x200U)
#define RTC_CR_CLKO_SHIFT                        (9U)
#define RTC_CR_CLKO(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_CLKO_SHIFT)) & RTC_CR_CLKO_MASK)
#define RTC_CR_SC16P_MASK                        (0x400U)
#define RTC_CR_SC16P_SHIFT                       (10U)
#define RTC_CR_SC16P(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC16P_SHIFT)) & RTC_CR_SC16P_MASK)
#define RTC_CR_SC8P_MASK                         (0x800U)
#define RTC_CR_SC8P_SHIFT                        (11U)
#define RTC_CR_SC8P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC8P_SHIFT)) & RTC_CR_SC8P_MASK)
#define RTC_CR_SC4P_MASK                         (0x1000U)
#define RTC_CR_SC4P_SHIFT                        (12U)
#define RTC_CR_SC4P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC4P_SHIFT)) & RTC_CR_SC4P_MASK)
#define RTC_CR_SC2P_MASK                         (0x2000U)
#define RTC_CR_SC2P_SHIFT                        (13U)
#define RTC_CR_SC2P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC2P_SHIFT)) & RTC_CR_SC2P_MASK)

/*! @name SR - RTC Status Register */
#define RTC_SR_TIF_MASK                          (0x1U)
#define RTC_SR_TIF_SHIFT                         (0U)
#define RTC_SR_TIF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TIF_SHIFT)) & RTC_SR_TIF_MASK)
#define RTC_SR_TOF_MASK                          (0x2U)
#define RTC_SR_TOF_SHIFT                         (1U)
#define RTC_SR_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TOF_SHIFT)) & RTC_SR_TOF_MASK)
#define RTC_SR_TAF_MASK                          (0x4U)
#define RTC_SR_TAF_SHIFT                         (2U)
#define RTC_SR_TAF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TAF_SHIFT)) & RTC_SR_TAF_MASK)
#define RTC_SR_TCE_MASK                          (0x10U)
#define RTC_SR_TCE_SHIFT                         (4U)
#define RTC_SR_TCE(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TCE_SHIFT)) & RTC_SR_TCE_MASK)

/*! @name LR - RTC Lock Register */
#define RTC_LR_TCL_MASK                          (0x8U)
#define RTC_LR_TCL_SHIFT                         (3U)
#define RTC_LR_TCL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_TCL_SHIFT)) & RTC_LR_TCL_MASK)
#define RTC_LR_CRL_MASK                          (0x10U)
#define RTC_LR_CRL_SHIFT                         (4U)
#define RTC_LR_CRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_CRL_SHIFT)) & RTC_LR_CRL_MASK)
#define RTC_LR_SRL_MASK                          (0x20U)
#define RTC_LR_SRL_SHIFT                         (5U)
#define RTC_LR_SRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_SRL_SHIFT)) & RTC_LR_SRL_MASK)
#define RTC_LR_LRL_MASK                          (0x40U)
#define RTC_LR_LRL_SHIFT                         (6U)
#define RTC_LR_LRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_LRL_SHIFT)) & RTC_LR_LRL_MASK)

/*! @name IER - RTC Interrupt Enable Register */
#define RTC_IER_TIIE_MASK                        (0x1U)
#define RTC_IER_TIIE_SHIFT                       (0U)
#define RTC_IER_TIIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TIIE_SHIFT)) & RTC_IER_TIIE_MASK)
#define RTC_IER_TOIE_MASK                        (0x2U)
#define RTC_IER_TOIE_SHIFT                       (1U)
#define RTC_IER_TOIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TOIE_SHIFT)) & RTC_IER_TOIE_MASK)
#define RTC_IER_TAIE_MASK                        (0x4U)
#define RTC_IER_TAIE_SHIFT                       (2U)
#define RTC_IER_TAIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TAIE_SHIFT)) & RTC_IER_TAIE_MASK)
#define RTC_IER_TSIE_MASK                        (0x10U)
#define RTC_IER_TSIE_SHIFT                       (4U)
#define RTC_IER_TSIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TSIE_SHIFT)) & RTC_IER_TSIE_MASK)
#define RTC_IER_WPON_MASK                        (0x80U)
#define RTC_IER_WPON_SHIFT                       (7U)
#define RTC_IER_WPON(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_WPON_SHIFT)) & RTC_IER_WPON_MASK)

/*! @name WAR - RTC Write Access Register */
#define RTC_WAR_TSRW_MASK                        (0x1U)
#define RTC_WAR_TSRW_SHIFT                       (0U)
#define RTC_WAR_TSRW(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAR_TSRW_SHIFT)) & RTC_WAR_TSRW_MASK)
#define RTC_WAR_TPRW_MASK                        (0x2U)
#define RTC_WAR_TPRW_SHIFT                       (1U)
#define RTC_WAR_TPRW(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAR_TPRW_SHIFT)) & RTC_WAR_TPRW_MASK)
#define RTC_WAR_TARW_MASK                        (0x4U)
#define RTC_WAR_TARW_SHIFT                       (2U)
#define RTC_WAR_TARW(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAR_TARW_SHIFT)) & RTC_WAR_TARW_MASK)
#define RTC_WAR_TCRW_MASK                        (0x8U)
#define RTC_WAR_TCRW_SHIFT                       (3U)
#define RTC_WAR_TCRW(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAR_TCRW_SHIFT)) & RTC_WAR_TCRW_MASK)
#define RTC_WAR_CRW_MASK                         (0x10U)
#define RTC_WAR_CRW_SHIFT                        (4U)
#define RTC_WAR_CRW(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_WAR_CRW_SHIFT)) & RTC_WAR_CRW_MASK)
#define RTC_WAR_SRW_MASK                         (0x20U)
#define RTC_WAR_SRW_SHIFT                        (5U)
#define RTC_WAR_SRW(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_WAR_SRW_SHIFT)) & RTC_WAR_SRW_MASK)
#define RTC_WAR_LRW_MASK                         (0x40U)
#define RTC_WAR_LRW_SHIFT                        (6U)
#define RTC_WAR_LRW(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_WAR_LRW_SHIFT)) & RTC_WAR_LRW_MASK)
#define RTC_WAR_IERW_MASK                        (0x80U)
#define RTC_WAR_IERW_SHIFT                       (7U)
#define RTC_WAR_IERW(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAR_IERW_SHIFT)) & RTC_WAR_IERW_MASK)

/*! @name RAR - RTC Read Access Register */
#define RTC_RAR_TSRR_MASK                        (0x1U)
#define RTC_RAR_TSRR_SHIFT                       (0U)
#define RTC_RAR_TSRR(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_RAR_TSRR_SHIFT)) & RTC_RAR_TSRR_MASK)
#define RTC_RAR_TPRR_MASK                        (0x2U)
#define RTC_RAR_TPRR_SHIFT                       (1U)
#define RTC_RAR_TPRR(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_RAR_TPRR_SHIFT)) & RTC_RAR_TPRR_MASK)
#define RTC_RAR_TARR_MASK                        (0x4U)
#define RTC_RAR_TARR_SHIFT                       (2U)
#define RTC_RAR_TARR(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_RAR_TARR_SHIFT)) & RTC_RAR_TARR_MASK)
#define RTC_RAR_TCRR_MASK                        (0x8U)
#define RTC_RAR_TCRR_SHIFT                       (3U)
#define RTC_RAR_TCRR(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_RAR_TCRR_SHIFT)) & RTC_RAR_TCRR_MASK)
#define RTC_RAR_CRR_MASK                         (0x10U)
#define RTC_RAR_CRR_SHIFT                        (4U)
#define RTC_RAR_CRR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_RAR_CRR_SHIFT)) & RTC_RAR_CRR_MASK)
#define RTC_RAR_SRR_MASK                         (0x20U)
#define RTC_RAR_SRR_SHIFT                        (5U)
#define RTC_RAR_SRR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_RAR_SRR_SHIFT)) & RTC_RAR_SRR_MASK)
#define RTC_RAR_LRR_MASK                         (0x40U)
#define RTC_RAR_LRR_SHIFT                        (6U)
#define RTC_RAR_LRR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_RAR_LRR_SHIFT)) & RTC_RAR_LRR_MASK)
#define RTC_RAR_IERR_MASK                        (0x80U)
#define RTC_RAR_IERR_SHIFT                       (7U)
#define RTC_RAR_IERR(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_RAR_IERR_SHIFT)) & RTC_RAR_IERR_MASK)


/*!
 * @}
 */ /* end of group RTC_Register_Masks */


/* RTC - Peripheral instance base addresses */
/** Peripheral RTC base address */
#define RTC_BASE                                 (0x4003D000u)
/** Peripheral RTC base pointer */
#define RTC                                      ((RTC_Type *)RTC_BASE)
/** Array initializer of RTC peripheral base addresses */
#define RTC_BASE_ADDRS                           { RTC_BASE }
/** Array initializer of RTC peripheral base pointers */
#define RTC_BASE_PTRS                            { RTC }
/** Interrupt vectors for the RTC peripheral type */
#define RTC_IRQS                                 { RTC_IRQn }
#define RTC_SECONDS_IRQS                         { RTC_Seconds_IRQn }

/*!
 * @}
 */ /* end of group RTC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SDHC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDHC_Peripheral_Access_Layer SDHC Peripheral Access Layer
 * @{
 */

/** SDHC - Register Layout Typedef */
typedef struct {
  __IO uint32_t DSADDR;                            /**< DMA System Address register, offset: 0x0 */
  __IO uint32_t BLKATTR;                           /**< Block Attributes register, offset: 0x4 */
  __IO uint32_t CMDARG;                            /**< Command Argument register, offset: 0x8 */
  __IO uint32_t XFERTYP;                           /**< Transfer Type register, offset: 0xC */
  __I  uint32_t CMDRSP[4];                         /**< Command Response 0..Command Response 3, array offset: 0x10, array step: 0x4 */
  __IO uint32_t DATPORT;                           /**< Buffer Data Port register, offset: 0x20 */
  __I  uint32_t PRSSTAT;                           /**< Present State register, offset: 0x24 */
  __IO uint32_t PROCTL;                            /**< Protocol Control register, offset: 0x28 */
  __IO uint32_t SYSCTL;                            /**< System Control register, offset: 0x2C */
  __IO uint32_t IRQSTAT;                           /**< Interrupt Status register, offset: 0x30 */
  __IO uint32_t IRQSTATEN;                         /**< Interrupt Status Enable register, offset: 0x34 */
  __IO uint32_t IRQSIGEN;                          /**< Interrupt Signal Enable register, offset: 0x38 */
  __I  uint32_t AC12ERR;                           /**< Auto CMD12 Error Status Register, offset: 0x3C */
  __I  uint32_t HTCAPBLT;                          /**< Host Controller Capabilities, offset: 0x40 */
  __IO uint32_t WML;                               /**< Watermark Level Register, offset: 0x44 */
       uint8_t RESERVED_0[8];
  __O  uint32_t FEVT;                              /**< Force Event register, offset: 0x50 */
  __I  uint32_t ADMAES;                            /**< ADMA Error Status register, offset: 0x54 */
  __IO uint32_t ADSADDR;                           /**< ADMA System Addressregister, offset: 0x58 */
       uint8_t RESERVED_1[100];
  __IO uint32_t VENDOR;                            /**< Vendor Specific register, offset: 0xC0 */
  __IO uint32_t MMCBOOT;                           /**< MMC Boot register, offset: 0xC4 */
       uint8_t RESERVED_2[52];
  __I  uint32_t HOSTVER;                           /**< Host Controller Version, offset: 0xFC */
} SDHC_Type;

/* ----------------------------------------------------------------------------
   -- SDHC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDHC_Register_Masks SDHC Register Masks
 * @{
 */

/*! @name DSADDR - DMA System Address register */
#define SDHC_DSADDR_DSADDR_MASK                  (0xFFFFFFFCU)
#define SDHC_DSADDR_DSADDR_SHIFT                 (2U)
#define SDHC_DSADDR_DSADDR(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_DSADDR_DSADDR_SHIFT)) & SDHC_DSADDR_DSADDR_MASK)

/*! @name BLKATTR - Block Attributes register */
#define SDHC_BLKATTR_BLKSIZE_MASK                (0x1FFFU)
#define SDHC_BLKATTR_BLKSIZE_SHIFT               (0U)
#define SDHC_BLKATTR_BLKSIZE(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_BLKATTR_BLKSIZE_SHIFT)) & SDHC_BLKATTR_BLKSIZE_MASK)
#define SDHC_BLKATTR_BLKCNT_MASK                 (0xFFFF0000U)
#define SDHC_BLKATTR_BLKCNT_SHIFT                (16U)
#define SDHC_BLKATTR_BLKCNT(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_BLKATTR_BLKCNT_SHIFT)) & SDHC_BLKATTR_BLKCNT_MASK)

/*! @name CMDARG - Command Argument register */
#define SDHC_CMDARG_CMDARG_MASK                  (0xFFFFFFFFU)
#define SDHC_CMDARG_CMDARG_SHIFT                 (0U)
#define SDHC_CMDARG_CMDARG(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_CMDARG_CMDARG_SHIFT)) & SDHC_CMDARG_CMDARG_MASK)

/*! @name XFERTYP - Transfer Type register */
#define SDHC_XFERTYP_DMAEN_MASK                  (0x1U)
#define SDHC_XFERTYP_DMAEN_SHIFT                 (0U)
#define SDHC_XFERTYP_DMAEN(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_DMAEN_SHIFT)) & SDHC_XFERTYP_DMAEN_MASK)
#define SDHC_XFERTYP_BCEN_MASK                   (0x2U)
#define SDHC_XFERTYP_BCEN_SHIFT                  (1U)
#define SDHC_XFERTYP_BCEN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_BCEN_SHIFT)) & SDHC_XFERTYP_BCEN_MASK)
#define SDHC_XFERTYP_AC12EN_MASK                 (0x4U)
#define SDHC_XFERTYP_AC12EN_SHIFT                (2U)
#define SDHC_XFERTYP_AC12EN(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_AC12EN_SHIFT)) & SDHC_XFERTYP_AC12EN_MASK)
#define SDHC_XFERTYP_DTDSEL_MASK                 (0x10U)
#define SDHC_XFERTYP_DTDSEL_SHIFT                (4U)
#define SDHC_XFERTYP_DTDSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_DTDSEL_SHIFT)) & SDHC_XFERTYP_DTDSEL_MASK)
#define SDHC_XFERTYP_MSBSEL_MASK                 (0x20U)
#define SDHC_XFERTYP_MSBSEL_SHIFT                (5U)
#define SDHC_XFERTYP_MSBSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_MSBSEL_SHIFT)) & SDHC_XFERTYP_MSBSEL_MASK)
#define SDHC_XFERTYP_RSPTYP_MASK                 (0x30000U)
#define SDHC_XFERTYP_RSPTYP_SHIFT                (16U)
#define SDHC_XFERTYP_RSPTYP(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_RSPTYP_SHIFT)) & SDHC_XFERTYP_RSPTYP_MASK)
#define SDHC_XFERTYP_CCCEN_MASK                  (0x80000U)
#define SDHC_XFERTYP_CCCEN_SHIFT                 (19U)
#define SDHC_XFERTYP_CCCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_CCCEN_SHIFT)) & SDHC_XFERTYP_CCCEN_MASK)
#define SDHC_XFERTYP_CICEN_MASK                  (0x100000U)
#define SDHC_XFERTYP_CICEN_SHIFT                 (20U)
#define SDHC_XFERTYP_CICEN(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_CICEN_SHIFT)) & SDHC_XFERTYP_CICEN_MASK)
#define SDHC_XFERTYP_DPSEL_MASK                  (0x200000U)
#define SDHC_XFERTYP_DPSEL_SHIFT                 (21U)
#define SDHC_XFERTYP_DPSEL(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_DPSEL_SHIFT)) & SDHC_XFERTYP_DPSEL_MASK)
#define SDHC_XFERTYP_CMDTYP_MASK                 (0xC00000U)
#define SDHC_XFERTYP_CMDTYP_SHIFT                (22U)
#define SDHC_XFERTYP_CMDTYP(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_CMDTYP_SHIFT)) & SDHC_XFERTYP_CMDTYP_MASK)
#define SDHC_XFERTYP_CMDINX_MASK                 (0x3F000000U)
#define SDHC_XFERTYP_CMDINX_SHIFT                (24U)
#define SDHC_XFERTYP_CMDINX(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_XFERTYP_CMDINX_SHIFT)) & SDHC_XFERTYP_CMDINX_MASK)

/*! @name CMDRSP - Command Response 0..Command Response 3 */
#define SDHC_CMDRSP_CMDRSP0_MASK                 (0xFFFFFFFFU)
#define SDHC_CMDRSP_CMDRSP0_SHIFT                (0U)
#define SDHC_CMDRSP_CMDRSP0(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_CMDRSP_CMDRSP0_SHIFT)) & SDHC_CMDRSP_CMDRSP0_MASK)
#define SDHC_CMDRSP_CMDRSP1_MASK                 (0xFFFFFFFFU)
#define SDHC_CMDRSP_CMDRSP1_SHIFT                (0U)
#define SDHC_CMDRSP_CMDRSP1(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_CMDRSP_CMDRSP1_SHIFT)) & SDHC_CMDRSP_CMDRSP1_MASK)
#define SDHC_CMDRSP_CMDRSP2_MASK                 (0xFFFFFFFFU)
#define SDHC_CMDRSP_CMDRSP2_SHIFT                (0U)
#define SDHC_CMDRSP_CMDRSP2(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_CMDRSP_CMDRSP2_SHIFT)) & SDHC_CMDRSP_CMDRSP2_MASK)
#define SDHC_CMDRSP_CMDRSP3_MASK                 (0xFFFFFFFFU)
#define SDHC_CMDRSP_CMDRSP3_SHIFT                (0U)
#define SDHC_CMDRSP_CMDRSP3(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_CMDRSP_CMDRSP3_SHIFT)) & SDHC_CMDRSP_CMDRSP3_MASK)

/* The count of SDHC_CMDRSP */
#define SDHC_CMDRSP_COUNT                        (4U)

/*! @name DATPORT - Buffer Data Port register */
#define SDHC_DATPORT_DATCONT_MASK                (0xFFFFFFFFU)
#define SDHC_DATPORT_DATCONT_SHIFT               (0U)
#define SDHC_DATPORT_DATCONT(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_DATPORT_DATCONT_SHIFT)) & SDHC_DATPORT_DATCONT_MASK)

/*! @name PRSSTAT - Present State register */
#define SDHC_PRSSTAT_CIHB_MASK                   (0x1U)
#define SDHC_PRSSTAT_CIHB_SHIFT                  (0U)
#define SDHC_PRSSTAT_CIHB(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_CIHB_SHIFT)) & SDHC_PRSSTAT_CIHB_MASK)
#define SDHC_PRSSTAT_CDIHB_MASK                  (0x2U)
#define SDHC_PRSSTAT_CDIHB_SHIFT                 (1U)
#define SDHC_PRSSTAT_CDIHB(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_CDIHB_SHIFT)) & SDHC_PRSSTAT_CDIHB_MASK)
#define SDHC_PRSSTAT_DLA_MASK                    (0x4U)
#define SDHC_PRSSTAT_DLA_SHIFT                   (2U)
#define SDHC_PRSSTAT_DLA(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_DLA_SHIFT)) & SDHC_PRSSTAT_DLA_MASK)
#define SDHC_PRSSTAT_SDSTB_MASK                  (0x8U)
#define SDHC_PRSSTAT_SDSTB_SHIFT                 (3U)
#define SDHC_PRSSTAT_SDSTB(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_SDSTB_SHIFT)) & SDHC_PRSSTAT_SDSTB_MASK)
#define SDHC_PRSSTAT_IPGOFF_MASK                 (0x10U)
#define SDHC_PRSSTAT_IPGOFF_SHIFT                (4U)
#define SDHC_PRSSTAT_IPGOFF(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_IPGOFF_SHIFT)) & SDHC_PRSSTAT_IPGOFF_MASK)
#define SDHC_PRSSTAT_HCKOFF_MASK                 (0x20U)
#define SDHC_PRSSTAT_HCKOFF_SHIFT                (5U)
#define SDHC_PRSSTAT_HCKOFF(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_HCKOFF_SHIFT)) & SDHC_PRSSTAT_HCKOFF_MASK)
#define SDHC_PRSSTAT_PEROFF_MASK                 (0x40U)
#define SDHC_PRSSTAT_PEROFF_SHIFT                (6U)
#define SDHC_PRSSTAT_PEROFF(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_PEROFF_SHIFT)) & SDHC_PRSSTAT_PEROFF_MASK)
#define SDHC_PRSSTAT_SDOFF_MASK                  (0x80U)
#define SDHC_PRSSTAT_SDOFF_SHIFT                 (7U)
#define SDHC_PRSSTAT_SDOFF(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_SDOFF_SHIFT)) & SDHC_PRSSTAT_SDOFF_MASK)
#define SDHC_PRSSTAT_WTA_MASK                    (0x100U)
#define SDHC_PRSSTAT_WTA_SHIFT                   (8U)
#define SDHC_PRSSTAT_WTA(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_WTA_SHIFT)) & SDHC_PRSSTAT_WTA_MASK)
#define SDHC_PRSSTAT_RTA_MASK                    (0x200U)
#define SDHC_PRSSTAT_RTA_SHIFT                   (9U)
#define SDHC_PRSSTAT_RTA(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_RTA_SHIFT)) & SDHC_PRSSTAT_RTA_MASK)
#define SDHC_PRSSTAT_BWEN_MASK                   (0x400U)
#define SDHC_PRSSTAT_BWEN_SHIFT                  (10U)
#define SDHC_PRSSTAT_BWEN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_BWEN_SHIFT)) & SDHC_PRSSTAT_BWEN_MASK)
#define SDHC_PRSSTAT_BREN_MASK                   (0x800U)
#define SDHC_PRSSTAT_BREN_SHIFT                  (11U)
#define SDHC_PRSSTAT_BREN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_BREN_SHIFT)) & SDHC_PRSSTAT_BREN_MASK)
#define SDHC_PRSSTAT_CINS_MASK                   (0x10000U)
#define SDHC_PRSSTAT_CINS_SHIFT                  (16U)
#define SDHC_PRSSTAT_CINS(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_CINS_SHIFT)) & SDHC_PRSSTAT_CINS_MASK)
#define SDHC_PRSSTAT_CLSL_MASK                   (0x800000U)
#define SDHC_PRSSTAT_CLSL_SHIFT                  (23U)
#define SDHC_PRSSTAT_CLSL(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_CLSL_SHIFT)) & SDHC_PRSSTAT_CLSL_MASK)
#define SDHC_PRSSTAT_DLSL_MASK                   (0xFF000000U)
#define SDHC_PRSSTAT_DLSL_SHIFT                  (24U)
#define SDHC_PRSSTAT_DLSL(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PRSSTAT_DLSL_SHIFT)) & SDHC_PRSSTAT_DLSL_MASK)

/*! @name PROCTL - Protocol Control register */
#define SDHC_PROCTL_LCTL_MASK                    (0x1U)
#define SDHC_PROCTL_LCTL_SHIFT                   (0U)
#define SDHC_PROCTL_LCTL(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_LCTL_SHIFT)) & SDHC_PROCTL_LCTL_MASK)
#define SDHC_PROCTL_DTW_MASK                     (0x6U)
#define SDHC_PROCTL_DTW_SHIFT                    (1U)
#define SDHC_PROCTL_DTW(x)                       (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_DTW_SHIFT)) & SDHC_PROCTL_DTW_MASK)
#define SDHC_PROCTL_D3CD_MASK                    (0x8U)
#define SDHC_PROCTL_D3CD_SHIFT                   (3U)
#define SDHC_PROCTL_D3CD(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_D3CD_SHIFT)) & SDHC_PROCTL_D3CD_MASK)
#define SDHC_PROCTL_EMODE_MASK                   (0x30U)
#define SDHC_PROCTL_EMODE_SHIFT                  (4U)
#define SDHC_PROCTL_EMODE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_EMODE_SHIFT)) & SDHC_PROCTL_EMODE_MASK)
#define SDHC_PROCTL_CDTL_MASK                    (0x40U)
#define SDHC_PROCTL_CDTL_SHIFT                   (6U)
#define SDHC_PROCTL_CDTL(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_CDTL_SHIFT)) & SDHC_PROCTL_CDTL_MASK)
#define SDHC_PROCTL_CDSS_MASK                    (0x80U)
#define SDHC_PROCTL_CDSS_SHIFT                   (7U)
#define SDHC_PROCTL_CDSS(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_CDSS_SHIFT)) & SDHC_PROCTL_CDSS_MASK)
#define SDHC_PROCTL_DMAS_MASK                    (0x300U)
#define SDHC_PROCTL_DMAS_SHIFT                   (8U)
#define SDHC_PROCTL_DMAS(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_DMAS_SHIFT)) & SDHC_PROCTL_DMAS_MASK)
#define SDHC_PROCTL_SABGREQ_MASK                 (0x10000U)
#define SDHC_PROCTL_SABGREQ_SHIFT                (16U)
#define SDHC_PROCTL_SABGREQ(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_SABGREQ_SHIFT)) & SDHC_PROCTL_SABGREQ_MASK)
#define SDHC_PROCTL_CREQ_MASK                    (0x20000U)
#define SDHC_PROCTL_CREQ_SHIFT                   (17U)
#define SDHC_PROCTL_CREQ(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_CREQ_SHIFT)) & SDHC_PROCTL_CREQ_MASK)
#define SDHC_PROCTL_RWCTL_MASK                   (0x40000U)
#define SDHC_PROCTL_RWCTL_SHIFT                  (18U)
#define SDHC_PROCTL_RWCTL(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_RWCTL_SHIFT)) & SDHC_PROCTL_RWCTL_MASK)
#define SDHC_PROCTL_IABG_MASK                    (0x80000U)
#define SDHC_PROCTL_IABG_SHIFT                   (19U)
#define SDHC_PROCTL_IABG(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_IABG_SHIFT)) & SDHC_PROCTL_IABG_MASK)
#define SDHC_PROCTL_WECINT_MASK                  (0x1000000U)
#define SDHC_PROCTL_WECINT_SHIFT                 (24U)
#define SDHC_PROCTL_WECINT(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_WECINT_SHIFT)) & SDHC_PROCTL_WECINT_MASK)
#define SDHC_PROCTL_WECINS_MASK                  (0x2000000U)
#define SDHC_PROCTL_WECINS_SHIFT                 (25U)
#define SDHC_PROCTL_WECINS(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_WECINS_SHIFT)) & SDHC_PROCTL_WECINS_MASK)
#define SDHC_PROCTL_WECRM_MASK                   (0x4000000U)
#define SDHC_PROCTL_WECRM_SHIFT                  (26U)
#define SDHC_PROCTL_WECRM(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_PROCTL_WECRM_SHIFT)) & SDHC_PROCTL_WECRM_MASK)

/*! @name SYSCTL - System Control register */
#define SDHC_SYSCTL_IPGEN_MASK                   (0x1U)
#define SDHC_SYSCTL_IPGEN_SHIFT                  (0U)
#define SDHC_SYSCTL_IPGEN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_IPGEN_SHIFT)) & SDHC_SYSCTL_IPGEN_MASK)
#define SDHC_SYSCTL_HCKEN_MASK                   (0x2U)
#define SDHC_SYSCTL_HCKEN_SHIFT                  (1U)
#define SDHC_SYSCTL_HCKEN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_HCKEN_SHIFT)) & SDHC_SYSCTL_HCKEN_MASK)
#define SDHC_SYSCTL_PEREN_MASK                   (0x4U)
#define SDHC_SYSCTL_PEREN_SHIFT                  (2U)
#define SDHC_SYSCTL_PEREN(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_PEREN_SHIFT)) & SDHC_SYSCTL_PEREN_MASK)
#define SDHC_SYSCTL_SDCLKEN_MASK                 (0x8U)
#define SDHC_SYSCTL_SDCLKEN_SHIFT                (3U)
#define SDHC_SYSCTL_SDCLKEN(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_SDCLKEN_SHIFT)) & SDHC_SYSCTL_SDCLKEN_MASK)
#define SDHC_SYSCTL_DVS_MASK                     (0xF0U)
#define SDHC_SYSCTL_DVS_SHIFT                    (4U)
#define SDHC_SYSCTL_DVS(x)                       (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_DVS_SHIFT)) & SDHC_SYSCTL_DVS_MASK)
#define SDHC_SYSCTL_SDCLKFS_MASK                 (0xFF00U)
#define SDHC_SYSCTL_SDCLKFS_SHIFT                (8U)
#define SDHC_SYSCTL_SDCLKFS(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_SDCLKFS_SHIFT)) & SDHC_SYSCTL_SDCLKFS_MASK)
#define SDHC_SYSCTL_DTOCV_MASK                   (0xF0000U)
#define SDHC_SYSCTL_DTOCV_SHIFT                  (16U)
#define SDHC_SYSCTL_DTOCV(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_DTOCV_SHIFT)) & SDHC_SYSCTL_DTOCV_MASK)
#define SDHC_SYSCTL_RSTA_MASK                    (0x1000000U)
#define SDHC_SYSCTL_RSTA_SHIFT                   (24U)
#define SDHC_SYSCTL_RSTA(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_RSTA_SHIFT)) & SDHC_SYSCTL_RSTA_MASK)
#define SDHC_SYSCTL_RSTC_MASK                    (0x2000000U)
#define SDHC_SYSCTL_RSTC_SHIFT                   (25U)
#define SDHC_SYSCTL_RSTC(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_RSTC_SHIFT)) & SDHC_SYSCTL_RSTC_MASK)
#define SDHC_SYSCTL_RSTD_MASK                    (0x4000000U)
#define SDHC_SYSCTL_RSTD_SHIFT                   (26U)
#define SDHC_SYSCTL_RSTD(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_RSTD_SHIFT)) & SDHC_SYSCTL_RSTD_MASK)
#define SDHC_SYSCTL_INITA_MASK                   (0x8000000U)
#define SDHC_SYSCTL_INITA_SHIFT                  (27U)
#define SDHC_SYSCTL_INITA(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_SYSCTL_INITA_SHIFT)) & SDHC_SYSCTL_INITA_MASK)

/*! @name IRQSTAT - Interrupt Status register */
#define SDHC_IRQSTAT_CC_MASK                     (0x1U)
#define SDHC_IRQSTAT_CC_SHIFT                    (0U)
#define SDHC_IRQSTAT_CC(x)                       (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CC_SHIFT)) & SDHC_IRQSTAT_CC_MASK)
#define SDHC_IRQSTAT_TC_MASK                     (0x2U)
#define SDHC_IRQSTAT_TC_SHIFT                    (1U)
#define SDHC_IRQSTAT_TC(x)                       (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_TC_SHIFT)) & SDHC_IRQSTAT_TC_MASK)
#define SDHC_IRQSTAT_BGE_MASK                    (0x4U)
#define SDHC_IRQSTAT_BGE_SHIFT                   (2U)
#define SDHC_IRQSTAT_BGE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_BGE_SHIFT)) & SDHC_IRQSTAT_BGE_MASK)
#define SDHC_IRQSTAT_DINT_MASK                   (0x8U)
#define SDHC_IRQSTAT_DINT_SHIFT                  (3U)
#define SDHC_IRQSTAT_DINT(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_DINT_SHIFT)) & SDHC_IRQSTAT_DINT_MASK)
#define SDHC_IRQSTAT_BWR_MASK                    (0x10U)
#define SDHC_IRQSTAT_BWR_SHIFT                   (4U)
#define SDHC_IRQSTAT_BWR(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_BWR_SHIFT)) & SDHC_IRQSTAT_BWR_MASK)
#define SDHC_IRQSTAT_BRR_MASK                    (0x20U)
#define SDHC_IRQSTAT_BRR_SHIFT                   (5U)
#define SDHC_IRQSTAT_BRR(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_BRR_SHIFT)) & SDHC_IRQSTAT_BRR_MASK)
#define SDHC_IRQSTAT_CINS_MASK                   (0x40U)
#define SDHC_IRQSTAT_CINS_SHIFT                  (6U)
#define SDHC_IRQSTAT_CINS(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CINS_SHIFT)) & SDHC_IRQSTAT_CINS_MASK)
#define SDHC_IRQSTAT_CRM_MASK                    (0x80U)
#define SDHC_IRQSTAT_CRM_SHIFT                   (7U)
#define SDHC_IRQSTAT_CRM(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CRM_SHIFT)) & SDHC_IRQSTAT_CRM_MASK)
#define SDHC_IRQSTAT_CINT_MASK                   (0x100U)
#define SDHC_IRQSTAT_CINT_SHIFT                  (8U)
#define SDHC_IRQSTAT_CINT(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CINT_SHIFT)) & SDHC_IRQSTAT_CINT_MASK)
#define SDHC_IRQSTAT_CTOE_MASK                   (0x10000U)
#define SDHC_IRQSTAT_CTOE_SHIFT                  (16U)
#define SDHC_IRQSTAT_CTOE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CTOE_SHIFT)) & SDHC_IRQSTAT_CTOE_MASK)
#define SDHC_IRQSTAT_CCE_MASK                    (0x20000U)
#define SDHC_IRQSTAT_CCE_SHIFT                   (17U)
#define SDHC_IRQSTAT_CCE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CCE_SHIFT)) & SDHC_IRQSTAT_CCE_MASK)
#define SDHC_IRQSTAT_CEBE_MASK                   (0x40000U)
#define SDHC_IRQSTAT_CEBE_SHIFT                  (18U)
#define SDHC_IRQSTAT_CEBE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CEBE_SHIFT)) & SDHC_IRQSTAT_CEBE_MASK)
#define SDHC_IRQSTAT_CIE_MASK                    (0x80000U)
#define SDHC_IRQSTAT_CIE_SHIFT                   (19U)
#define SDHC_IRQSTAT_CIE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_CIE_SHIFT)) & SDHC_IRQSTAT_CIE_MASK)
#define SDHC_IRQSTAT_DTOE_MASK                   (0x100000U)
#define SDHC_IRQSTAT_DTOE_SHIFT                  (20U)
#define SDHC_IRQSTAT_DTOE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_DTOE_SHIFT)) & SDHC_IRQSTAT_DTOE_MASK)
#define SDHC_IRQSTAT_DCE_MASK                    (0x200000U)
#define SDHC_IRQSTAT_DCE_SHIFT                   (21U)
#define SDHC_IRQSTAT_DCE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_DCE_SHIFT)) & SDHC_IRQSTAT_DCE_MASK)
#define SDHC_IRQSTAT_DEBE_MASK                   (0x400000U)
#define SDHC_IRQSTAT_DEBE_SHIFT                  (22U)
#define SDHC_IRQSTAT_DEBE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_DEBE_SHIFT)) & SDHC_IRQSTAT_DEBE_MASK)
#define SDHC_IRQSTAT_AC12E_MASK                  (0x1000000U)
#define SDHC_IRQSTAT_AC12E_SHIFT                 (24U)
#define SDHC_IRQSTAT_AC12E(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_AC12E_SHIFT)) & SDHC_IRQSTAT_AC12E_MASK)
#define SDHC_IRQSTAT_DMAE_MASK                   (0x10000000U)
#define SDHC_IRQSTAT_DMAE_SHIFT                  (28U)
#define SDHC_IRQSTAT_DMAE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTAT_DMAE_SHIFT)) & SDHC_IRQSTAT_DMAE_MASK)

/*! @name IRQSTATEN - Interrupt Status Enable register */
#define SDHC_IRQSTATEN_CCSEN_MASK                (0x1U)
#define SDHC_IRQSTATEN_CCSEN_SHIFT               (0U)
#define SDHC_IRQSTATEN_CCSEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CCSEN_SHIFT)) & SDHC_IRQSTATEN_CCSEN_MASK)
#define SDHC_IRQSTATEN_TCSEN_MASK                (0x2U)
#define SDHC_IRQSTATEN_TCSEN_SHIFT               (1U)
#define SDHC_IRQSTATEN_TCSEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_TCSEN_SHIFT)) & SDHC_IRQSTATEN_TCSEN_MASK)
#define SDHC_IRQSTATEN_BGESEN_MASK               (0x4U)
#define SDHC_IRQSTATEN_BGESEN_SHIFT              (2U)
#define SDHC_IRQSTATEN_BGESEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_BGESEN_SHIFT)) & SDHC_IRQSTATEN_BGESEN_MASK)
#define SDHC_IRQSTATEN_DINTSEN_MASK              (0x8U)
#define SDHC_IRQSTATEN_DINTSEN_SHIFT             (3U)
#define SDHC_IRQSTATEN_DINTSEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_DINTSEN_SHIFT)) & SDHC_IRQSTATEN_DINTSEN_MASK)
#define SDHC_IRQSTATEN_BWRSEN_MASK               (0x10U)
#define SDHC_IRQSTATEN_BWRSEN_SHIFT              (4U)
#define SDHC_IRQSTATEN_BWRSEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_BWRSEN_SHIFT)) & SDHC_IRQSTATEN_BWRSEN_MASK)
#define SDHC_IRQSTATEN_BRRSEN_MASK               (0x20U)
#define SDHC_IRQSTATEN_BRRSEN_SHIFT              (5U)
#define SDHC_IRQSTATEN_BRRSEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_BRRSEN_SHIFT)) & SDHC_IRQSTATEN_BRRSEN_MASK)
#define SDHC_IRQSTATEN_CINSEN_MASK               (0x40U)
#define SDHC_IRQSTATEN_CINSEN_SHIFT              (6U)
#define SDHC_IRQSTATEN_CINSEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CINSEN_SHIFT)) & SDHC_IRQSTATEN_CINSEN_MASK)
#define SDHC_IRQSTATEN_CRMSEN_MASK               (0x80U)
#define SDHC_IRQSTATEN_CRMSEN_SHIFT              (7U)
#define SDHC_IRQSTATEN_CRMSEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CRMSEN_SHIFT)) & SDHC_IRQSTATEN_CRMSEN_MASK)
#define SDHC_IRQSTATEN_CINTSEN_MASK              (0x100U)
#define SDHC_IRQSTATEN_CINTSEN_SHIFT             (8U)
#define SDHC_IRQSTATEN_CINTSEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CINTSEN_SHIFT)) & SDHC_IRQSTATEN_CINTSEN_MASK)
#define SDHC_IRQSTATEN_CTOESEN_MASK              (0x10000U)
#define SDHC_IRQSTATEN_CTOESEN_SHIFT             (16U)
#define SDHC_IRQSTATEN_CTOESEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CTOESEN_SHIFT)) & SDHC_IRQSTATEN_CTOESEN_MASK)
#define SDHC_IRQSTATEN_CCESEN_MASK               (0x20000U)
#define SDHC_IRQSTATEN_CCESEN_SHIFT              (17U)
#define SDHC_IRQSTATEN_CCESEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CCESEN_SHIFT)) & SDHC_IRQSTATEN_CCESEN_MASK)
#define SDHC_IRQSTATEN_CEBESEN_MASK              (0x40000U)
#define SDHC_IRQSTATEN_CEBESEN_SHIFT             (18U)
#define SDHC_IRQSTATEN_CEBESEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CEBESEN_SHIFT)) & SDHC_IRQSTATEN_CEBESEN_MASK)
#define SDHC_IRQSTATEN_CIESEN_MASK               (0x80000U)
#define SDHC_IRQSTATEN_CIESEN_SHIFT              (19U)
#define SDHC_IRQSTATEN_CIESEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_CIESEN_SHIFT)) & SDHC_IRQSTATEN_CIESEN_MASK)
#define SDHC_IRQSTATEN_DTOESEN_MASK              (0x100000U)
#define SDHC_IRQSTATEN_DTOESEN_SHIFT             (20U)
#define SDHC_IRQSTATEN_DTOESEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_DTOESEN_SHIFT)) & SDHC_IRQSTATEN_DTOESEN_MASK)
#define SDHC_IRQSTATEN_DCESEN_MASK               (0x200000U)
#define SDHC_IRQSTATEN_DCESEN_SHIFT              (21U)
#define SDHC_IRQSTATEN_DCESEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_DCESEN_SHIFT)) & SDHC_IRQSTATEN_DCESEN_MASK)
#define SDHC_IRQSTATEN_DEBESEN_MASK              (0x400000U)
#define SDHC_IRQSTATEN_DEBESEN_SHIFT             (22U)
#define SDHC_IRQSTATEN_DEBESEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_DEBESEN_SHIFT)) & SDHC_IRQSTATEN_DEBESEN_MASK)
#define SDHC_IRQSTATEN_AC12ESEN_MASK             (0x1000000U)
#define SDHC_IRQSTATEN_AC12ESEN_SHIFT            (24U)
#define SDHC_IRQSTATEN_AC12ESEN(x)               (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_AC12ESEN_SHIFT)) & SDHC_IRQSTATEN_AC12ESEN_MASK)
#define SDHC_IRQSTATEN_DMAESEN_MASK              (0x10000000U)
#define SDHC_IRQSTATEN_DMAESEN_SHIFT             (28U)
#define SDHC_IRQSTATEN_DMAESEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSTATEN_DMAESEN_SHIFT)) & SDHC_IRQSTATEN_DMAESEN_MASK)

/*! @name IRQSIGEN - Interrupt Signal Enable register */
#define SDHC_IRQSIGEN_CCIEN_MASK                 (0x1U)
#define SDHC_IRQSIGEN_CCIEN_SHIFT                (0U)
#define SDHC_IRQSIGEN_CCIEN(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CCIEN_SHIFT)) & SDHC_IRQSIGEN_CCIEN_MASK)
#define SDHC_IRQSIGEN_TCIEN_MASK                 (0x2U)
#define SDHC_IRQSIGEN_TCIEN_SHIFT                (1U)
#define SDHC_IRQSIGEN_TCIEN(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_TCIEN_SHIFT)) & SDHC_IRQSIGEN_TCIEN_MASK)
#define SDHC_IRQSIGEN_BGEIEN_MASK                (0x4U)
#define SDHC_IRQSIGEN_BGEIEN_SHIFT               (2U)
#define SDHC_IRQSIGEN_BGEIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_BGEIEN_SHIFT)) & SDHC_IRQSIGEN_BGEIEN_MASK)
#define SDHC_IRQSIGEN_DINTIEN_MASK               (0x8U)
#define SDHC_IRQSIGEN_DINTIEN_SHIFT              (3U)
#define SDHC_IRQSIGEN_DINTIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_DINTIEN_SHIFT)) & SDHC_IRQSIGEN_DINTIEN_MASK)
#define SDHC_IRQSIGEN_BWRIEN_MASK                (0x10U)
#define SDHC_IRQSIGEN_BWRIEN_SHIFT               (4U)
#define SDHC_IRQSIGEN_BWRIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_BWRIEN_SHIFT)) & SDHC_IRQSIGEN_BWRIEN_MASK)
#define SDHC_IRQSIGEN_BRRIEN_MASK                (0x20U)
#define SDHC_IRQSIGEN_BRRIEN_SHIFT               (5U)
#define SDHC_IRQSIGEN_BRRIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_BRRIEN_SHIFT)) & SDHC_IRQSIGEN_BRRIEN_MASK)
#define SDHC_IRQSIGEN_CINSIEN_MASK               (0x40U)
#define SDHC_IRQSIGEN_CINSIEN_SHIFT              (6U)
#define SDHC_IRQSIGEN_CINSIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CINSIEN_SHIFT)) & SDHC_IRQSIGEN_CINSIEN_MASK)
#define SDHC_IRQSIGEN_CRMIEN_MASK                (0x80U)
#define SDHC_IRQSIGEN_CRMIEN_SHIFT               (7U)
#define SDHC_IRQSIGEN_CRMIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CRMIEN_SHIFT)) & SDHC_IRQSIGEN_CRMIEN_MASK)
#define SDHC_IRQSIGEN_CINTIEN_MASK               (0x100U)
#define SDHC_IRQSIGEN_CINTIEN_SHIFT              (8U)
#define SDHC_IRQSIGEN_CINTIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CINTIEN_SHIFT)) & SDHC_IRQSIGEN_CINTIEN_MASK)
#define SDHC_IRQSIGEN_CTOEIEN_MASK               (0x10000U)
#define SDHC_IRQSIGEN_CTOEIEN_SHIFT              (16U)
#define SDHC_IRQSIGEN_CTOEIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CTOEIEN_SHIFT)) & SDHC_IRQSIGEN_CTOEIEN_MASK)
#define SDHC_IRQSIGEN_CCEIEN_MASK                (0x20000U)
#define SDHC_IRQSIGEN_CCEIEN_SHIFT               (17U)
#define SDHC_IRQSIGEN_CCEIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CCEIEN_SHIFT)) & SDHC_IRQSIGEN_CCEIEN_MASK)
#define SDHC_IRQSIGEN_CEBEIEN_MASK               (0x40000U)
#define SDHC_IRQSIGEN_CEBEIEN_SHIFT              (18U)
#define SDHC_IRQSIGEN_CEBEIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CEBEIEN_SHIFT)) & SDHC_IRQSIGEN_CEBEIEN_MASK)
#define SDHC_IRQSIGEN_CIEIEN_MASK                (0x80000U)
#define SDHC_IRQSIGEN_CIEIEN_SHIFT               (19U)
#define SDHC_IRQSIGEN_CIEIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_CIEIEN_SHIFT)) & SDHC_IRQSIGEN_CIEIEN_MASK)
#define SDHC_IRQSIGEN_DTOEIEN_MASK               (0x100000U)
#define SDHC_IRQSIGEN_DTOEIEN_SHIFT              (20U)
#define SDHC_IRQSIGEN_DTOEIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_DTOEIEN_SHIFT)) & SDHC_IRQSIGEN_DTOEIEN_MASK)
#define SDHC_IRQSIGEN_DCEIEN_MASK                (0x200000U)
#define SDHC_IRQSIGEN_DCEIEN_SHIFT               (21U)
#define SDHC_IRQSIGEN_DCEIEN(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_DCEIEN_SHIFT)) & SDHC_IRQSIGEN_DCEIEN_MASK)
#define SDHC_IRQSIGEN_DEBEIEN_MASK               (0x400000U)
#define SDHC_IRQSIGEN_DEBEIEN_SHIFT              (22U)
#define SDHC_IRQSIGEN_DEBEIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_DEBEIEN_SHIFT)) & SDHC_IRQSIGEN_DEBEIEN_MASK)
#define SDHC_IRQSIGEN_AC12EIEN_MASK              (0x1000000U)
#define SDHC_IRQSIGEN_AC12EIEN_SHIFT             (24U)
#define SDHC_IRQSIGEN_AC12EIEN(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_AC12EIEN_SHIFT)) & SDHC_IRQSIGEN_AC12EIEN_MASK)
#define SDHC_IRQSIGEN_DMAEIEN_MASK               (0x10000000U)
#define SDHC_IRQSIGEN_DMAEIEN_SHIFT              (28U)
#define SDHC_IRQSIGEN_DMAEIEN(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_IRQSIGEN_DMAEIEN_SHIFT)) & SDHC_IRQSIGEN_DMAEIEN_MASK)

/*! @name AC12ERR - Auto CMD12 Error Status Register */
#define SDHC_AC12ERR_AC12NE_MASK                 (0x1U)
#define SDHC_AC12ERR_AC12NE_SHIFT                (0U)
#define SDHC_AC12ERR_AC12NE(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_AC12NE_SHIFT)) & SDHC_AC12ERR_AC12NE_MASK)
#define SDHC_AC12ERR_AC12TOE_MASK                (0x2U)
#define SDHC_AC12ERR_AC12TOE_SHIFT               (1U)
#define SDHC_AC12ERR_AC12TOE(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_AC12TOE_SHIFT)) & SDHC_AC12ERR_AC12TOE_MASK)
#define SDHC_AC12ERR_AC12EBE_MASK                (0x4U)
#define SDHC_AC12ERR_AC12EBE_SHIFT               (2U)
#define SDHC_AC12ERR_AC12EBE(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_AC12EBE_SHIFT)) & SDHC_AC12ERR_AC12EBE_MASK)
#define SDHC_AC12ERR_AC12CE_MASK                 (0x8U)
#define SDHC_AC12ERR_AC12CE_SHIFT                (3U)
#define SDHC_AC12ERR_AC12CE(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_AC12CE_SHIFT)) & SDHC_AC12ERR_AC12CE_MASK)
#define SDHC_AC12ERR_AC12IE_MASK                 (0x10U)
#define SDHC_AC12ERR_AC12IE_SHIFT                (4U)
#define SDHC_AC12ERR_AC12IE(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_AC12IE_SHIFT)) & SDHC_AC12ERR_AC12IE_MASK)
#define SDHC_AC12ERR_CNIBAC12E_MASK              (0x80U)
#define SDHC_AC12ERR_CNIBAC12E_SHIFT             (7U)
#define SDHC_AC12ERR_CNIBAC12E(x)                (((uint32_t)(((uint32_t)(x)) << SDHC_AC12ERR_CNIBAC12E_SHIFT)) & SDHC_AC12ERR_CNIBAC12E_MASK)

/*! @name HTCAPBLT - Host Controller Capabilities */
#define SDHC_HTCAPBLT_MBL_MASK                   (0x70000U)
#define SDHC_HTCAPBLT_MBL_SHIFT                  (16U)
#define SDHC_HTCAPBLT_MBL(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_MBL_SHIFT)) & SDHC_HTCAPBLT_MBL_MASK)
#define SDHC_HTCAPBLT_ADMAS_MASK                 (0x100000U)
#define SDHC_HTCAPBLT_ADMAS_SHIFT                (20U)
#define SDHC_HTCAPBLT_ADMAS(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_ADMAS_SHIFT)) & SDHC_HTCAPBLT_ADMAS_MASK)
#define SDHC_HTCAPBLT_HSS_MASK                   (0x200000U)
#define SDHC_HTCAPBLT_HSS_SHIFT                  (21U)
#define SDHC_HTCAPBLT_HSS(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_HSS_SHIFT)) & SDHC_HTCAPBLT_HSS_MASK)
#define SDHC_HTCAPBLT_DMAS_MASK                  (0x400000U)
#define SDHC_HTCAPBLT_DMAS_SHIFT                 (22U)
#define SDHC_HTCAPBLT_DMAS(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_DMAS_SHIFT)) & SDHC_HTCAPBLT_DMAS_MASK)
#define SDHC_HTCAPBLT_SRS_MASK                   (0x800000U)
#define SDHC_HTCAPBLT_SRS_SHIFT                  (23U)
#define SDHC_HTCAPBLT_SRS(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_SRS_SHIFT)) & SDHC_HTCAPBLT_SRS_MASK)
#define SDHC_HTCAPBLT_VS33_MASK                  (0x1000000U)
#define SDHC_HTCAPBLT_VS33_SHIFT                 (24U)
#define SDHC_HTCAPBLT_VS33(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_HTCAPBLT_VS33_SHIFT)) & SDHC_HTCAPBLT_VS33_MASK)

/*! @name WML - Watermark Level Register */
#define SDHC_WML_RDWML_MASK                      (0xFFU)
#define SDHC_WML_RDWML_SHIFT                     (0U)
#define SDHC_WML_RDWML(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_WML_RDWML_SHIFT)) & SDHC_WML_RDWML_MASK)
#define SDHC_WML_WRWML_MASK                      (0xFF0000U)
#define SDHC_WML_WRWML_SHIFT                     (16U)
#define SDHC_WML_WRWML(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_WML_WRWML_SHIFT)) & SDHC_WML_WRWML_MASK)

/*! @name FEVT - Force Event register */
#define SDHC_FEVT_AC12NE_MASK                    (0x1U)
#define SDHC_FEVT_AC12NE_SHIFT                   (0U)
#define SDHC_FEVT_AC12NE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12NE_SHIFT)) & SDHC_FEVT_AC12NE_MASK)
#define SDHC_FEVT_AC12TOE_MASK                   (0x2U)
#define SDHC_FEVT_AC12TOE_SHIFT                  (1U)
#define SDHC_FEVT_AC12TOE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12TOE_SHIFT)) & SDHC_FEVT_AC12TOE_MASK)
#define SDHC_FEVT_AC12CE_MASK                    (0x4U)
#define SDHC_FEVT_AC12CE_SHIFT                   (2U)
#define SDHC_FEVT_AC12CE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12CE_SHIFT)) & SDHC_FEVT_AC12CE_MASK)
#define SDHC_FEVT_AC12EBE_MASK                   (0x8U)
#define SDHC_FEVT_AC12EBE_SHIFT                  (3U)
#define SDHC_FEVT_AC12EBE(x)                     (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12EBE_SHIFT)) & SDHC_FEVT_AC12EBE_MASK)
#define SDHC_FEVT_AC12IE_MASK                    (0x10U)
#define SDHC_FEVT_AC12IE_SHIFT                   (4U)
#define SDHC_FEVT_AC12IE(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12IE_SHIFT)) & SDHC_FEVT_AC12IE_MASK)
#define SDHC_FEVT_CNIBAC12E_MASK                 (0x80U)
#define SDHC_FEVT_CNIBAC12E_SHIFT                (7U)
#define SDHC_FEVT_CNIBAC12E(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CNIBAC12E_SHIFT)) & SDHC_FEVT_CNIBAC12E_MASK)
#define SDHC_FEVT_CTOE_MASK                      (0x10000U)
#define SDHC_FEVT_CTOE_SHIFT                     (16U)
#define SDHC_FEVT_CTOE(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CTOE_SHIFT)) & SDHC_FEVT_CTOE_MASK)
#define SDHC_FEVT_CCE_MASK                       (0x20000U)
#define SDHC_FEVT_CCE_SHIFT                      (17U)
#define SDHC_FEVT_CCE(x)                         (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CCE_SHIFT)) & SDHC_FEVT_CCE_MASK)
#define SDHC_FEVT_CEBE_MASK                      (0x40000U)
#define SDHC_FEVT_CEBE_SHIFT                     (18U)
#define SDHC_FEVT_CEBE(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CEBE_SHIFT)) & SDHC_FEVT_CEBE_MASK)
#define SDHC_FEVT_CIE_MASK                       (0x80000U)
#define SDHC_FEVT_CIE_SHIFT                      (19U)
#define SDHC_FEVT_CIE(x)                         (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CIE_SHIFT)) & SDHC_FEVT_CIE_MASK)
#define SDHC_FEVT_DTOE_MASK                      (0x100000U)
#define SDHC_FEVT_DTOE_SHIFT                     (20U)
#define SDHC_FEVT_DTOE(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_DTOE_SHIFT)) & SDHC_FEVT_DTOE_MASK)
#define SDHC_FEVT_DCE_MASK                       (0x200000U)
#define SDHC_FEVT_DCE_SHIFT                      (21U)
#define SDHC_FEVT_DCE(x)                         (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_DCE_SHIFT)) & SDHC_FEVT_DCE_MASK)
#define SDHC_FEVT_DEBE_MASK                      (0x400000U)
#define SDHC_FEVT_DEBE_SHIFT                     (22U)
#define SDHC_FEVT_DEBE(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_DEBE_SHIFT)) & SDHC_FEVT_DEBE_MASK)
#define SDHC_FEVT_AC12E_MASK                     (0x1000000U)
#define SDHC_FEVT_AC12E_SHIFT                    (24U)
#define SDHC_FEVT_AC12E(x)                       (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_AC12E_SHIFT)) & SDHC_FEVT_AC12E_MASK)
#define SDHC_FEVT_DMAE_MASK                      (0x10000000U)
#define SDHC_FEVT_DMAE_SHIFT                     (28U)
#define SDHC_FEVT_DMAE(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_DMAE_SHIFT)) & SDHC_FEVT_DMAE_MASK)
#define SDHC_FEVT_CINT_MASK                      (0x80000000U)
#define SDHC_FEVT_CINT_SHIFT                     (31U)
#define SDHC_FEVT_CINT(x)                        (((uint32_t)(((uint32_t)(x)) << SDHC_FEVT_CINT_SHIFT)) & SDHC_FEVT_CINT_MASK)

/*! @name ADMAES - ADMA Error Status register */
#define SDHC_ADMAES_ADMAES_MASK                  (0x3U)
#define SDHC_ADMAES_ADMAES_SHIFT                 (0U)
#define SDHC_ADMAES_ADMAES(x)                    (((uint32_t)(((uint32_t)(x)) << SDHC_ADMAES_ADMAES_SHIFT)) & SDHC_ADMAES_ADMAES_MASK)
#define SDHC_ADMAES_ADMALME_MASK                 (0x4U)
#define SDHC_ADMAES_ADMALME_SHIFT                (2U)
#define SDHC_ADMAES_ADMALME(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_ADMAES_ADMALME_SHIFT)) & SDHC_ADMAES_ADMALME_MASK)
#define SDHC_ADMAES_ADMADCE_MASK                 (0x8U)
#define SDHC_ADMAES_ADMADCE_SHIFT                (3U)
#define SDHC_ADMAES_ADMADCE(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_ADMAES_ADMADCE_SHIFT)) & SDHC_ADMAES_ADMADCE_MASK)

/*! @name ADSADDR - ADMA System Addressregister */
#define SDHC_ADSADDR_ADSADDR_MASK                (0xFFFFFFFCU)
#define SDHC_ADSADDR_ADSADDR_SHIFT               (2U)
#define SDHC_ADSADDR_ADSADDR(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_ADSADDR_ADSADDR_SHIFT)) & SDHC_ADSADDR_ADSADDR_MASK)

/*! @name VENDOR - Vendor Specific register */
#define SDHC_VENDOR_EXBLKNU_MASK                 (0x2U)
#define SDHC_VENDOR_EXBLKNU_SHIFT                (1U)
#define SDHC_VENDOR_EXBLKNU(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_VENDOR_EXBLKNU_SHIFT)) & SDHC_VENDOR_EXBLKNU_MASK)
#define SDHC_VENDOR_INTSTVAL_MASK                (0xFF0000U)
#define SDHC_VENDOR_INTSTVAL_SHIFT               (16U)
#define SDHC_VENDOR_INTSTVAL(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_VENDOR_INTSTVAL_SHIFT)) & SDHC_VENDOR_INTSTVAL_MASK)

/*! @name MMCBOOT - MMC Boot register */
#define SDHC_MMCBOOT_DTOCVACK_MASK               (0xFU)
#define SDHC_MMCBOOT_DTOCVACK_SHIFT              (0U)
#define SDHC_MMCBOOT_DTOCVACK(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_DTOCVACK_SHIFT)) & SDHC_MMCBOOT_DTOCVACK_MASK)
#define SDHC_MMCBOOT_BOOTACK_MASK                (0x10U)
#define SDHC_MMCBOOT_BOOTACK_SHIFT               (4U)
#define SDHC_MMCBOOT_BOOTACK(x)                  (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_BOOTACK_SHIFT)) & SDHC_MMCBOOT_BOOTACK_MASK)
#define SDHC_MMCBOOT_BOOTMODE_MASK               (0x20U)
#define SDHC_MMCBOOT_BOOTMODE_SHIFT              (5U)
#define SDHC_MMCBOOT_BOOTMODE(x)                 (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_BOOTMODE_SHIFT)) & SDHC_MMCBOOT_BOOTMODE_MASK)
#define SDHC_MMCBOOT_BOOTEN_MASK                 (0x40U)
#define SDHC_MMCBOOT_BOOTEN_SHIFT                (6U)
#define SDHC_MMCBOOT_BOOTEN(x)                   (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_BOOTEN_SHIFT)) & SDHC_MMCBOOT_BOOTEN_MASK)
#define SDHC_MMCBOOT_AUTOSABGEN_MASK             (0x80U)
#define SDHC_MMCBOOT_AUTOSABGEN_SHIFT            (7U)
#define SDHC_MMCBOOT_AUTOSABGEN(x)               (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_AUTOSABGEN_SHIFT)) & SDHC_MMCBOOT_AUTOSABGEN_MASK)
#define SDHC_MMCBOOT_BOOTBLKCNT_MASK             (0xFFFF0000U)
#define SDHC_MMCBOOT_BOOTBLKCNT_SHIFT            (16U)
#define SDHC_MMCBOOT_BOOTBLKCNT(x)               (((uint32_t)(((uint32_t)(x)) << SDHC_MMCBOOT_BOOTBLKCNT_SHIFT)) & SDHC_MMCBOOT_BOOTBLKCNT_MASK)

/*! @name HOSTVER - Host Controller Version */
#define SDHC_HOSTVER_SVN_MASK                    (0xFFU)
#define SDHC_HOSTVER_SVN_SHIFT                   (0U)
#define SDHC_HOSTVER_SVN(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_HOSTVER_SVN_SHIFT)) & SDHC_HOSTVER_SVN_MASK)
#define SDHC_HOSTVER_VVN_MASK                    (0xFF00U)
#define SDHC_HOSTVER_VVN_SHIFT                   (8U)
#define SDHC_HOSTVER_VVN(x)                      (((uint32_t)(((uint32_t)(x)) << SDHC_HOSTVER_VVN_SHIFT)) & SDHC_HOSTVER_VVN_MASK)


/*!
 * @}
 */ /* end of group SDHC_Register_Masks */


/* SDHC - Peripheral instance base addresses */
/** Peripheral SDHC base address */
#define SDHC_BASE                                (0x400B1000u)
/** Peripheral SDHC base pointer */
#define SDHC                                     ((SDHC_Type *)SDHC_BASE)
/** Array initializer of SDHC peripheral base addresses */
#define SDHC_BASE_ADDRS                          { SDHC_BASE }
/** Array initializer of SDHC peripheral base pointers */
#define SDHC_BASE_PTRS                           { SDHC }
/** Interrupt vectors for the SDHC peripheral type */
#define SDHC_IRQS                                { SDHC_IRQn }

/*!
 * @}
 */ /* end of group SDHC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SDRAM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDRAM_Peripheral_Access_Layer SDRAM Peripheral Access Layer
 * @{
 */

/** SDRAM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[66];
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x42 */
       uint8_t RESERVED_1[4];
  struct {                                         /* offset: 0x48, array step: 0x8 */
    __IO uint32_t AC;                                /**< Address and Control Register, array offset: 0x48, array step: 0x8 */
    __IO uint32_t CM;                                /**< Control Mask, array offset: 0x4C, array step: 0x8 */
  } BLOCK[2];
} SDRAM_Type;

/* ----------------------------------------------------------------------------
   -- SDRAM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDRAM_Register_Masks SDRAM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
#define SDRAM_CTRL_RC_MASK                       (0x1FFU)
#define SDRAM_CTRL_RC_SHIFT                      (0U)
#define SDRAM_CTRL_RC(x)                         (((uint16_t)(((uint16_t)(x)) << SDRAM_CTRL_RC_SHIFT)) & SDRAM_CTRL_RC_MASK)
#define SDRAM_CTRL_RTIM_MASK                     (0x600U)
#define SDRAM_CTRL_RTIM_SHIFT                    (9U)
#define SDRAM_CTRL_RTIM(x)                       (((uint16_t)(((uint16_t)(x)) << SDRAM_CTRL_RTIM_SHIFT)) & SDRAM_CTRL_RTIM_MASK)
#define SDRAM_CTRL_IS_MASK                       (0x800U)
#define SDRAM_CTRL_IS_SHIFT                      (11U)
#define SDRAM_CTRL_IS(x)                         (((uint16_t)(((uint16_t)(x)) << SDRAM_CTRL_IS_SHIFT)) & SDRAM_CTRL_IS_MASK)

/*! @name AC - Address and Control Register */
#define SDRAM_AC_IP_MASK                         (0x8U)
#define SDRAM_AC_IP_SHIFT                        (3U)
#define SDRAM_AC_IP(x)                           (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_IP_SHIFT)) & SDRAM_AC_IP_MASK)
#define SDRAM_AC_PS_MASK                         (0x30U)
#define SDRAM_AC_PS_SHIFT                        (4U)
#define SDRAM_AC_PS(x)                           (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_PS_SHIFT)) & SDRAM_AC_PS_MASK)
#define SDRAM_AC_IMRS_MASK                       (0x40U)
#define SDRAM_AC_IMRS_SHIFT                      (6U)
#define SDRAM_AC_IMRS(x)                         (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_IMRS_SHIFT)) & SDRAM_AC_IMRS_MASK)
#define SDRAM_AC_CBM_MASK                        (0x700U)
#define SDRAM_AC_CBM_SHIFT                       (8U)
#define SDRAM_AC_CBM(x)                          (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_CBM_SHIFT)) & SDRAM_AC_CBM_MASK)
#define SDRAM_AC_CASL_MASK                       (0x3000U)
#define SDRAM_AC_CASL_SHIFT                      (12U)
#define SDRAM_AC_CASL(x)                         (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_CASL_SHIFT)) & SDRAM_AC_CASL_MASK)
#define SDRAM_AC_RE_MASK                         (0x8000U)
#define SDRAM_AC_RE_SHIFT                        (15U)
#define SDRAM_AC_RE(x)                           (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_RE_SHIFT)) & SDRAM_AC_RE_MASK)
#define SDRAM_AC_BA_MASK                         (0xFFFC0000U)
#define SDRAM_AC_BA_SHIFT                        (18U)
#define SDRAM_AC_BA(x)                           (((uint32_t)(((uint32_t)(x)) << SDRAM_AC_BA_SHIFT)) & SDRAM_AC_BA_MASK)

/* The count of SDRAM_AC */
#define SDRAM_AC_COUNT                           (2U)

/*! @name CM - Control Mask */
#define SDRAM_CM_V_MASK                          (0x1U)
#define SDRAM_CM_V_SHIFT                         (0U)
#define SDRAM_CM_V(x)                            (((uint32_t)(((uint32_t)(x)) << SDRAM_CM_V_SHIFT)) & SDRAM_CM_V_MASK)
#define SDRAM_CM_WP_MASK                         (0x100U)
#define SDRAM_CM_WP_SHIFT                        (8U)
#define SDRAM_CM_WP(x)                           (((uint32_t)(((uint32_t)(x)) << SDRAM_CM_WP_SHIFT)) & SDRAM_CM_WP_MASK)
#define SDRAM_CM_BAM_MASK                        (0xFFFC0000U)
#define SDRAM_CM_BAM_SHIFT                       (18U)
#define SDRAM_CM_BAM(x)                          (((uint32_t)(((uint32_t)(x)) << SDRAM_CM_BAM_SHIFT)) & SDRAM_CM_BAM_MASK)

/* The count of SDRAM_CM */
#define SDRAM_CM_COUNT                           (2U)


/*!
 * @}
 */ /* end of group SDRAM_Register_Masks */


/* SDRAM - Peripheral instance base addresses */
/** Peripheral SDRAM base address */
#define SDRAM_BASE                               (0x4000F000u)
/** Peripheral SDRAM base pointer */
#define SDRAM                                    ((SDRAM_Type *)SDRAM_BASE)
/** Array initializer of SDRAM peripheral base addresses */
#define SDRAM_BASE_ADDRS                         { SDRAM_BASE }
/** Array initializer of SDRAM peripheral base pointers */
#define SDRAM_BASE_PTRS                          { SDRAM }

/*!
 * @}
 */ /* end of group SDRAM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SOPT1;                             /**< System Options Register 1, offset: 0x0 */
  __IO uint32_t SOPT1CFG;                          /**< SOPT1 Configuration Register, offset: 0x4 */
       uint8_t RESERVED_0[4092];
  __IO uint32_t SOPT2;                             /**< System Options Register 2, offset: 0x1004 */
       uint8_t RESERVED_1[4];
  __IO uint32_t SOPT4;                             /**< System Options Register 4, offset: 0x100C */
  __IO uint32_t SOPT5;                             /**< System Options Register 5, offset: 0x1010 */
       uint8_t RESERVED_2[4];
  __IO uint32_t SOPT7;                             /**< System Options Register 7, offset: 0x1018 */
  __IO uint32_t SOPT8;                             /**< System Options Register 8, offset: 0x101C */
  __IO uint32_t SOPT9;                             /**< System Options Register 9, offset: 0x1020 */
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x1024 */
  __IO uint32_t SCGC1;                             /**< System Clock Gating Control Register 1, offset: 0x1028 */
  __IO uint32_t SCGC2;                             /**< System Clock Gating Control Register 2, offset: 0x102C */
  __IO uint32_t SCGC3;                             /**< System Clock Gating Control Register 3, offset: 0x1030 */
  __IO uint32_t SCGC4;                             /**< System Clock Gating Control Register 4, offset: 0x1034 */
  __IO uint32_t SCGC5;                             /**< System Clock Gating Control Register 5, offset: 0x1038 */
  __IO uint32_t SCGC6;                             /**< System Clock Gating Control Register 6, offset: 0x103C */
  __IO uint32_t SCGC7;                             /**< System Clock Gating Control Register 7, offset: 0x1040 */
  __IO uint32_t CLKDIV1;                           /**< System Clock Divider Register 1, offset: 0x1044 */
  __IO uint32_t CLKDIV2;                           /**< System Clock Divider Register 2, offset: 0x1048 */
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x104C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x1050 */
  __I  uint32_t UIDH;                              /**< Unique Identification Register High, offset: 0x1054 */
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x1058 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x105C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x1060 */
  __IO uint32_t CLKDIV3;                           /**< System Clock Divider Register 3, offset: 0x1064 */
  __IO uint32_t CLKDIV4;                           /**< System Clock Divider Register 4, offset: 0x1068 */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name SOPT1 - System Options Register 1 */
#define SIM_SOPT1_RAMSIZE_MASK                   (0xF000U)
#define SIM_SOPT1_RAMSIZE_SHIFT                  (12U)
#define SIM_SOPT1_RAMSIZE(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_RAMSIZE_SHIFT)) & SIM_SOPT1_RAMSIZE_MASK)
#define SIM_SOPT1_OSC32KSEL_MASK                 (0xC0000U)
#define SIM_SOPT1_OSC32KSEL_SHIFT                (18U)
#define SIM_SOPT1_OSC32KSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_OSC32KSEL_SHIFT)) & SIM_SOPT1_OSC32KSEL_MASK)
#define SIM_SOPT1_USBVSTBY_MASK                  (0x20000000U)
#define SIM_SOPT1_USBVSTBY_SHIFT                 (29U)
#define SIM_SOPT1_USBVSTBY(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_USBVSTBY_SHIFT)) & SIM_SOPT1_USBVSTBY_MASK)
#define SIM_SOPT1_USBSSTBY_MASK                  (0x40000000U)
#define SIM_SOPT1_USBSSTBY_SHIFT                 (30U)
#define SIM_SOPT1_USBSSTBY(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_USBSSTBY_SHIFT)) & SIM_SOPT1_USBSSTBY_MASK)
#define SIM_SOPT1_USBREGEN_MASK                  (0x80000000U)
#define SIM_SOPT1_USBREGEN_SHIFT                 (31U)
#define SIM_SOPT1_USBREGEN(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_USBREGEN_SHIFT)) & SIM_SOPT1_USBREGEN_MASK)

/*! @name SOPT1CFG - SOPT1 Configuration Register */
#define SIM_SOPT1CFG_URWE_MASK                   (0x1000000U)
#define SIM_SOPT1CFG_URWE_SHIFT                  (24U)
#define SIM_SOPT1CFG_URWE(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1CFG_URWE_SHIFT)) & SIM_SOPT1CFG_URWE_MASK)
#define SIM_SOPT1CFG_UVSWE_MASK                  (0x2000000U)
#define SIM_SOPT1CFG_UVSWE_SHIFT                 (25U)
#define SIM_SOPT1CFG_UVSWE(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1CFG_UVSWE_SHIFT)) & SIM_SOPT1CFG_UVSWE_MASK)
#define SIM_SOPT1CFG_USSWE_MASK                  (0x4000000U)
#define SIM_SOPT1CFG_USSWE_SHIFT                 (26U)
#define SIM_SOPT1CFG_USSWE(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1CFG_USSWE_SHIFT)) & SIM_SOPT1CFG_USSWE_MASK)

/*! @name SOPT2 - System Options Register 2 */
#define SIM_SOPT2_RTCCLKOUTSEL_MASK              (0x10U)
#define SIM_SOPT2_RTCCLKOUTSEL_SHIFT             (4U)
#define SIM_SOPT2_RTCCLKOUTSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_RTCCLKOUTSEL_SHIFT)) & SIM_SOPT2_RTCCLKOUTSEL_MASK)
#define SIM_SOPT2_CLKOUTSEL_MASK                 (0xE0U)
#define SIM_SOPT2_CLKOUTSEL_SHIFT                (5U)
#define SIM_SOPT2_CLKOUTSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_CLKOUTSEL_SHIFT)) & SIM_SOPT2_CLKOUTSEL_MASK)
#define SIM_SOPT2_FBSL_MASK                      (0x300U)
#define SIM_SOPT2_FBSL_SHIFT                     (8U)
#define SIM_SOPT2_FBSL(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_FBSL_SHIFT)) & SIM_SOPT2_FBSL_MASK)
#define SIM_SOPT2_TRACECLKSEL_MASK               (0x1000U)
#define SIM_SOPT2_TRACECLKSEL_SHIFT              (12U)
#define SIM_SOPT2_TRACECLKSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_TRACECLKSEL_SHIFT)) & SIM_SOPT2_TRACECLKSEL_MASK)
#define SIM_SOPT2_PLLFLLSEL_MASK                 (0x30000U)
#define SIM_SOPT2_PLLFLLSEL_SHIFT                (16U)
#define SIM_SOPT2_PLLFLLSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_PLLFLLSEL_SHIFT)) & SIM_SOPT2_PLLFLLSEL_MASK)
#define SIM_SOPT2_USBSRC_MASK                    (0x40000U)
#define SIM_SOPT2_USBSRC_SHIFT                   (18U)
#define SIM_SOPT2_USBSRC(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_USBSRC_SHIFT)) & SIM_SOPT2_USBSRC_MASK)
#define SIM_SOPT2_FLEXIOSRC_MASK                 (0xC00000U)
#define SIM_SOPT2_FLEXIOSRC_SHIFT                (22U)
#define SIM_SOPT2_FLEXIOSRC(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_FLEXIOSRC_SHIFT)) & SIM_SOPT2_FLEXIOSRC_MASK)
#define SIM_SOPT2_TPMSRC_MASK                    (0x3000000U)
#define SIM_SOPT2_TPMSRC_SHIFT                   (24U)
#define SIM_SOPT2_TPMSRC(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_TPMSRC_SHIFT)) & SIM_SOPT2_TPMSRC_MASK)
#define SIM_SOPT2_LPUARTSRC_MASK                 (0xC000000U)
#define SIM_SOPT2_LPUARTSRC_SHIFT                (26U)
#define SIM_SOPT2_LPUARTSRC(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_LPUARTSRC_SHIFT)) & SIM_SOPT2_LPUARTSRC_MASK)
#define SIM_SOPT2_SDHCSRC_MASK                   (0x30000000U)
#define SIM_SOPT2_SDHCSRC_SHIFT                  (28U)
#define SIM_SOPT2_SDHCSRC(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_SDHCSRC_SHIFT)) & SIM_SOPT2_SDHCSRC_MASK)
#define SIM_SOPT2_EMVSIMSRC_MASK                 (0xC0000000U)
#define SIM_SOPT2_EMVSIMSRC_SHIFT                (30U)
#define SIM_SOPT2_EMVSIMSRC(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_EMVSIMSRC_SHIFT)) & SIM_SOPT2_EMVSIMSRC_MASK)

/*! @name SOPT4 - System Options Register 4 */
#define SIM_SOPT4_FTM0FLT0_MASK                  (0x1U)
#define SIM_SOPT4_FTM0FLT0_SHIFT                 (0U)
#define SIM_SOPT4_FTM0FLT0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM0FLT0_SHIFT)) & SIM_SOPT4_FTM0FLT0_MASK)
#define SIM_SOPT4_FTM0FLT1_MASK                  (0x2U)
#define SIM_SOPT4_FTM0FLT1_SHIFT                 (1U)
#define SIM_SOPT4_FTM0FLT1(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM0FLT1_SHIFT)) & SIM_SOPT4_FTM0FLT1_MASK)
#define SIM_SOPT4_FTM1FLT0_MASK                  (0x10U)
#define SIM_SOPT4_FTM1FLT0_SHIFT                 (4U)
#define SIM_SOPT4_FTM1FLT0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM1FLT0_SHIFT)) & SIM_SOPT4_FTM1FLT0_MASK)
#define SIM_SOPT4_FTM2FLT0_MASK                  (0x100U)
#define SIM_SOPT4_FTM2FLT0_SHIFT                 (8U)
#define SIM_SOPT4_FTM2FLT0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM2FLT0_SHIFT)) & SIM_SOPT4_FTM2FLT0_MASK)
#define SIM_SOPT4_FTM3FLT0_MASK                  (0x1000U)
#define SIM_SOPT4_FTM3FLT0_SHIFT                 (12U)
#define SIM_SOPT4_FTM3FLT0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM3FLT0_SHIFT)) & SIM_SOPT4_FTM3FLT0_MASK)
#define SIM_SOPT4_FTM1CH0SRC_MASK                (0xC0000U)
#define SIM_SOPT4_FTM1CH0SRC_SHIFT               (18U)
#define SIM_SOPT4_FTM1CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM1CH0SRC_SHIFT)) & SIM_SOPT4_FTM1CH0SRC_MASK)
#define SIM_SOPT4_FTM2CH0SRC_MASK                (0x300000U)
#define SIM_SOPT4_FTM2CH0SRC_SHIFT               (20U)
#define SIM_SOPT4_FTM2CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM2CH0SRC_SHIFT)) & SIM_SOPT4_FTM2CH0SRC_MASK)
#define SIM_SOPT4_FTM2CH1SRC_MASK                (0x400000U)
#define SIM_SOPT4_FTM2CH1SRC_SHIFT               (22U)
#define SIM_SOPT4_FTM2CH1SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM2CH1SRC_SHIFT)) & SIM_SOPT4_FTM2CH1SRC_MASK)
#define SIM_SOPT4_FTM0CLKSEL_MASK                (0x1000000U)
#define SIM_SOPT4_FTM0CLKSEL_SHIFT               (24U)
#define SIM_SOPT4_FTM0CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM0CLKSEL_SHIFT)) & SIM_SOPT4_FTM0CLKSEL_MASK)
#define SIM_SOPT4_FTM1CLKSEL_MASK                (0x2000000U)
#define SIM_SOPT4_FTM1CLKSEL_SHIFT               (25U)
#define SIM_SOPT4_FTM1CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM1CLKSEL_SHIFT)) & SIM_SOPT4_FTM1CLKSEL_MASK)
#define SIM_SOPT4_FTM2CLKSEL_MASK                (0x4000000U)
#define SIM_SOPT4_FTM2CLKSEL_SHIFT               (26U)
#define SIM_SOPT4_FTM2CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM2CLKSEL_SHIFT)) & SIM_SOPT4_FTM2CLKSEL_MASK)
#define SIM_SOPT4_FTM3CLKSEL_MASK                (0x8000000U)
#define SIM_SOPT4_FTM3CLKSEL_SHIFT               (27U)
#define SIM_SOPT4_FTM3CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM3CLKSEL_SHIFT)) & SIM_SOPT4_FTM3CLKSEL_MASK)
#define SIM_SOPT4_FTM0TRG0SRC_MASK               (0x10000000U)
#define SIM_SOPT4_FTM0TRG0SRC_SHIFT              (28U)
#define SIM_SOPT4_FTM0TRG0SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM0TRG0SRC_SHIFT)) & SIM_SOPT4_FTM0TRG0SRC_MASK)
#define SIM_SOPT4_FTM0TRG1SRC_MASK               (0x20000000U)
#define SIM_SOPT4_FTM0TRG1SRC_SHIFT              (29U)
#define SIM_SOPT4_FTM0TRG1SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM0TRG1SRC_SHIFT)) & SIM_SOPT4_FTM0TRG1SRC_MASK)
#define SIM_SOPT4_FTM3TRG0SRC_MASK               (0x40000000U)
#define SIM_SOPT4_FTM3TRG0SRC_SHIFT              (30U)
#define SIM_SOPT4_FTM3TRG0SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM3TRG0SRC_SHIFT)) & SIM_SOPT4_FTM3TRG0SRC_MASK)
#define SIM_SOPT4_FTM3TRG1SRC_MASK               (0x80000000U)
#define SIM_SOPT4_FTM3TRG1SRC_SHIFT              (31U)
#define SIM_SOPT4_FTM3TRG1SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_FTM3TRG1SRC_SHIFT)) & SIM_SOPT4_FTM3TRG1SRC_MASK)

/*! @name SOPT5 - System Options Register 5 */
#define SIM_SOPT5_LPUART0TXSRC_MASK              (0x30000U)
#define SIM_SOPT5_LPUART0TXSRC_SHIFT             (16U)
#define SIM_SOPT5_LPUART0TXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART0TXSRC_SHIFT)) & SIM_SOPT5_LPUART0TXSRC_MASK)
#define SIM_SOPT5_LPUART0RXSRC_MASK              (0xC0000U)
#define SIM_SOPT5_LPUART0RXSRC_SHIFT             (18U)
#define SIM_SOPT5_LPUART0RXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART0RXSRC_SHIFT)) & SIM_SOPT5_LPUART0RXSRC_MASK)
#define SIM_SOPT5_LPUART1TXSRC_MASK              (0x300000U)
#define SIM_SOPT5_LPUART1TXSRC_SHIFT             (20U)
#define SIM_SOPT5_LPUART1TXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART1TXSRC_SHIFT)) & SIM_SOPT5_LPUART1TXSRC_MASK)
#define SIM_SOPT5_LPUART1RXSRC_MASK              (0xC00000U)
#define SIM_SOPT5_LPUART1RXSRC_SHIFT             (22U)
#define SIM_SOPT5_LPUART1RXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART1RXSRC_SHIFT)) & SIM_SOPT5_LPUART1RXSRC_MASK)

/*! @name SOPT7 - System Options Register 7 */
#define SIM_SOPT7_ADC0TRGSEL_MASK                (0xFU)
#define SIM_SOPT7_ADC0TRGSEL_SHIFT               (0U)
#define SIM_SOPT7_ADC0TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT7_ADC0TRGSEL_SHIFT)) & SIM_SOPT7_ADC0TRGSEL_MASK)
#define SIM_SOPT7_ADC0PRETRGSEL_MASK             (0x10U)
#define SIM_SOPT7_ADC0PRETRGSEL_SHIFT            (4U)
#define SIM_SOPT7_ADC0PRETRGSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_SOPT7_ADC0PRETRGSEL_SHIFT)) & SIM_SOPT7_ADC0PRETRGSEL_MASK)
#define SIM_SOPT7_ADC0ALTTRGEN_MASK              (0x80U)
#define SIM_SOPT7_ADC0ALTTRGEN_SHIFT             (7U)
#define SIM_SOPT7_ADC0ALTTRGEN(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT7_ADC0ALTTRGEN_SHIFT)) & SIM_SOPT7_ADC0ALTTRGEN_MASK)

/*! @name SOPT8 - System Options Register 8 */
#define SIM_SOPT8_FTM0SYNCBIT_MASK               (0x1U)
#define SIM_SOPT8_FTM0SYNCBIT_SHIFT              (0U)
#define SIM_SOPT8_FTM0SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0SYNCBIT_SHIFT)) & SIM_SOPT8_FTM0SYNCBIT_MASK)
#define SIM_SOPT8_FTM1SYNCBIT_MASK               (0x2U)
#define SIM_SOPT8_FTM1SYNCBIT_SHIFT              (1U)
#define SIM_SOPT8_FTM1SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM1SYNCBIT_SHIFT)) & SIM_SOPT8_FTM1SYNCBIT_MASK)
#define SIM_SOPT8_FTM2SYNCBIT_MASK               (0x4U)
#define SIM_SOPT8_FTM2SYNCBIT_SHIFT              (2U)
#define SIM_SOPT8_FTM2SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM2SYNCBIT_SHIFT)) & SIM_SOPT8_FTM2SYNCBIT_MASK)
#define SIM_SOPT8_FTM3SYNCBIT_MASK               (0x8U)
#define SIM_SOPT8_FTM3SYNCBIT_SHIFT              (3U)
#define SIM_SOPT8_FTM3SYNCBIT(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3SYNCBIT_SHIFT)) & SIM_SOPT8_FTM3SYNCBIT_MASK)
#define SIM_SOPT8_FTM0OCH0SRC_MASK               (0x10000U)
#define SIM_SOPT8_FTM0OCH0SRC_SHIFT              (16U)
#define SIM_SOPT8_FTM0OCH0SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH0SRC_SHIFT)) & SIM_SOPT8_FTM0OCH0SRC_MASK)
#define SIM_SOPT8_FTM0OCH1SRC_MASK               (0x20000U)
#define SIM_SOPT8_FTM0OCH1SRC_SHIFT              (17U)
#define SIM_SOPT8_FTM0OCH1SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH1SRC_SHIFT)) & SIM_SOPT8_FTM0OCH1SRC_MASK)
#define SIM_SOPT8_FTM0OCH2SRC_MASK               (0x40000U)
#define SIM_SOPT8_FTM0OCH2SRC_SHIFT              (18U)
#define SIM_SOPT8_FTM0OCH2SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH2SRC_SHIFT)) & SIM_SOPT8_FTM0OCH2SRC_MASK)
#define SIM_SOPT8_FTM0OCH3SRC_MASK               (0x80000U)
#define SIM_SOPT8_FTM0OCH3SRC_SHIFT              (19U)
#define SIM_SOPT8_FTM0OCH3SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH3SRC_SHIFT)) & SIM_SOPT8_FTM0OCH3SRC_MASK)
#define SIM_SOPT8_FTM0OCH4SRC_MASK               (0x100000U)
#define SIM_SOPT8_FTM0OCH4SRC_SHIFT              (20U)
#define SIM_SOPT8_FTM0OCH4SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH4SRC_SHIFT)) & SIM_SOPT8_FTM0OCH4SRC_MASK)
#define SIM_SOPT8_FTM0OCH5SRC_MASK               (0x200000U)
#define SIM_SOPT8_FTM0OCH5SRC_SHIFT              (21U)
#define SIM_SOPT8_FTM0OCH5SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH5SRC_SHIFT)) & SIM_SOPT8_FTM0OCH5SRC_MASK)
#define SIM_SOPT8_FTM0OCH6SRC_MASK               (0x400000U)
#define SIM_SOPT8_FTM0OCH6SRC_SHIFT              (22U)
#define SIM_SOPT8_FTM0OCH6SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH6SRC_SHIFT)) & SIM_SOPT8_FTM0OCH6SRC_MASK)
#define SIM_SOPT8_FTM0OCH7SRC_MASK               (0x800000U)
#define SIM_SOPT8_FTM0OCH7SRC_SHIFT              (23U)
#define SIM_SOPT8_FTM0OCH7SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM0OCH7SRC_SHIFT)) & SIM_SOPT8_FTM0OCH7SRC_MASK)
#define SIM_SOPT8_FTM3OCH0SRC_MASK               (0x1000000U)
#define SIM_SOPT8_FTM3OCH0SRC_SHIFT              (24U)
#define SIM_SOPT8_FTM3OCH0SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH0SRC_SHIFT)) & SIM_SOPT8_FTM3OCH0SRC_MASK)
#define SIM_SOPT8_FTM3OCH1SRC_MASK               (0x2000000U)
#define SIM_SOPT8_FTM3OCH1SRC_SHIFT              (25U)
#define SIM_SOPT8_FTM3OCH1SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH1SRC_SHIFT)) & SIM_SOPT8_FTM3OCH1SRC_MASK)
#define SIM_SOPT8_FTM3OCH2SRC_MASK               (0x4000000U)
#define SIM_SOPT8_FTM3OCH2SRC_SHIFT              (26U)
#define SIM_SOPT8_FTM3OCH2SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH2SRC_SHIFT)) & SIM_SOPT8_FTM3OCH2SRC_MASK)
#define SIM_SOPT8_FTM3OCH3SRC_MASK               (0x8000000U)
#define SIM_SOPT8_FTM3OCH3SRC_SHIFT              (27U)
#define SIM_SOPT8_FTM3OCH3SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH3SRC_SHIFT)) & SIM_SOPT8_FTM3OCH3SRC_MASK)
#define SIM_SOPT8_FTM3OCH4SRC_MASK               (0x10000000U)
#define SIM_SOPT8_FTM3OCH4SRC_SHIFT              (28U)
#define SIM_SOPT8_FTM3OCH4SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH4SRC_SHIFT)) & SIM_SOPT8_FTM3OCH4SRC_MASK)
#define SIM_SOPT8_FTM3OCH5SRC_MASK               (0x20000000U)
#define SIM_SOPT8_FTM3OCH5SRC_SHIFT              (29U)
#define SIM_SOPT8_FTM3OCH5SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH5SRC_SHIFT)) & SIM_SOPT8_FTM3OCH5SRC_MASK)
#define SIM_SOPT8_FTM3OCH6SRC_MASK               (0x40000000U)
#define SIM_SOPT8_FTM3OCH6SRC_SHIFT              (30U)
#define SIM_SOPT8_FTM3OCH6SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH6SRC_SHIFT)) & SIM_SOPT8_FTM3OCH6SRC_MASK)
#define SIM_SOPT8_FTM3OCH7SRC_MASK               (0x80000000U)
#define SIM_SOPT8_FTM3OCH7SRC_SHIFT              (31U)
#define SIM_SOPT8_FTM3OCH7SRC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_SOPT8_FTM3OCH7SRC_SHIFT)) & SIM_SOPT8_FTM3OCH7SRC_MASK)

/*! @name SOPT9 - System Options Register 9 */
#define SIM_SOPT9_TPM1CH0SRC_MASK                (0xC0000U)
#define SIM_SOPT9_TPM1CH0SRC_SHIFT               (18U)
#define SIM_SOPT9_TPM1CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT9_TPM1CH0SRC_SHIFT)) & SIM_SOPT9_TPM1CH0SRC_MASK)
#define SIM_SOPT9_TPM2CH0SRC_MASK                (0x300000U)
#define SIM_SOPT9_TPM2CH0SRC_SHIFT               (20U)
#define SIM_SOPT9_TPM2CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT9_TPM2CH0SRC_SHIFT)) & SIM_SOPT9_TPM2CH0SRC_MASK)
#define SIM_SOPT9_TPM1CLKSEL_MASK                (0x2000000U)
#define SIM_SOPT9_TPM1CLKSEL_SHIFT               (25U)
#define SIM_SOPT9_TPM1CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT9_TPM1CLKSEL_SHIFT)) & SIM_SOPT9_TPM1CLKSEL_MASK)
#define SIM_SOPT9_TPM2CLKSEL_MASK                (0x4000000U)
#define SIM_SOPT9_TPM2CLKSEL_SHIFT               (26U)
#define SIM_SOPT9_TPM2CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT9_TPM2CLKSEL_SHIFT)) & SIM_SOPT9_TPM2CLKSEL_MASK)

/*! @name SDID - System Device Identification Register */
#define SIM_SDID_PINID_MASK                      (0xFU)
#define SIM_SDID_PINID_SHIFT                     (0U)
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PINID_SHIFT)) & SIM_SDID_PINID_MASK)
#define SIM_SDID_FAMID_MASK                      (0x70U)
#define SIM_SDID_FAMID_SHIFT                     (4U)
#define SIM_SDID_FAMID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FAMID_SHIFT)) & SIM_SDID_FAMID_MASK)
#define SIM_SDID_DIEID_MASK                      (0xF80U)
#define SIM_SDID_DIEID_SHIFT                     (7U)
#define SIM_SDID_DIEID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_DIEID_SHIFT)) & SIM_SDID_DIEID_MASK)
#define SIM_SDID_REVID_MASK                      (0xF000U)
#define SIM_SDID_REVID_SHIFT                     (12U)
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_REVID_SHIFT)) & SIM_SDID_REVID_MASK)
#define SIM_SDID_SERIESID_MASK                   (0xF00000U)
#define SIM_SDID_SERIESID_SHIFT                  (20U)
#define SIM_SDID_SERIESID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SERIESID_SHIFT)) & SIM_SDID_SERIESID_MASK)
#define SIM_SDID_SUBFAMID_MASK                   (0xF000000U)
#define SIM_SDID_SUBFAMID_SHIFT                  (24U)
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SUBFAMID_SHIFT)) & SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMILYID_MASK                   (0xF0000000U)
#define SIM_SDID_FAMILYID_SHIFT                  (28U)
#define SIM_SDID_FAMILYID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FAMILYID_SHIFT)) & SIM_SDID_FAMILYID_MASK)

/*! @name SCGC1 - System Clock Gating Control Register 1 */
#define SIM_SCGC1_I2C2_MASK                      (0x40U)
#define SIM_SCGC1_I2C2_SHIFT                     (6U)
#define SIM_SCGC1_I2C2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC1_I2C2_SHIFT)) & SIM_SCGC1_I2C2_MASK)
#define SIM_SCGC1_I2C3_MASK                      (0x80U)
#define SIM_SCGC1_I2C3_SHIFT                     (7U)
#define SIM_SCGC1_I2C3(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC1_I2C3_SHIFT)) & SIM_SCGC1_I2C3_MASK)

/*! @name SCGC2 - System Clock Gating Control Register 2 */
#define SIM_SCGC2_LPUART0_MASK                   (0x10U)
#define SIM_SCGC2_LPUART0_SHIFT                  (4U)
#define SIM_SCGC2_LPUART0(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_LPUART0_SHIFT)) & SIM_SCGC2_LPUART0_MASK)
#define SIM_SCGC2_LPUART1_MASK                   (0x20U)
#define SIM_SCGC2_LPUART1_SHIFT                  (5U)
#define SIM_SCGC2_LPUART1(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_LPUART1_SHIFT)) & SIM_SCGC2_LPUART1_MASK)
#define SIM_SCGC2_LPUART2_MASK                   (0x40U)
#define SIM_SCGC2_LPUART2_SHIFT                  (6U)
#define SIM_SCGC2_LPUART2(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_LPUART2_SHIFT)) & SIM_SCGC2_LPUART2_MASK)
#define SIM_SCGC2_LPUART3_MASK                   (0x80U)
#define SIM_SCGC2_LPUART3_SHIFT                  (7U)
#define SIM_SCGC2_LPUART3(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_LPUART3_SHIFT)) & SIM_SCGC2_LPUART3_MASK)
#define SIM_SCGC2_TPM1_MASK                      (0x200U)
#define SIM_SCGC2_TPM1_SHIFT                     (9U)
#define SIM_SCGC2_TPM1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_TPM1_SHIFT)) & SIM_SCGC2_TPM1_MASK)
#define SIM_SCGC2_TPM2_MASK                      (0x400U)
#define SIM_SCGC2_TPM2_SHIFT                     (10U)
#define SIM_SCGC2_TPM2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_TPM2_SHIFT)) & SIM_SCGC2_TPM2_MASK)
#define SIM_SCGC2_DAC0_MASK                      (0x1000U)
#define SIM_SCGC2_DAC0_SHIFT                     (12U)
#define SIM_SCGC2_DAC0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_DAC0_SHIFT)) & SIM_SCGC2_DAC0_MASK)
#define SIM_SCGC2_EMVSIM0_MASK                   (0x100000U)
#define SIM_SCGC2_EMVSIM0_SHIFT                  (20U)
#define SIM_SCGC2_EMVSIM0(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_EMVSIM0_SHIFT)) & SIM_SCGC2_EMVSIM0_MASK)
#define SIM_SCGC2_EMVSIM1_MASK                   (0x200000U)
#define SIM_SCGC2_EMVSIM1_SHIFT                  (21U)
#define SIM_SCGC2_EMVSIM1(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_EMVSIM1_SHIFT)) & SIM_SCGC2_EMVSIM1_MASK)
#define SIM_SCGC2_LPUART4_MASK                   (0x400000U)
#define SIM_SCGC2_LPUART4_SHIFT                  (22U)
#define SIM_SCGC2_LPUART4(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_LPUART4_SHIFT)) & SIM_SCGC2_LPUART4_MASK)
#define SIM_SCGC2_QSPI_MASK                      (0x4000000U)
#define SIM_SCGC2_QSPI_SHIFT                     (26U)
#define SIM_SCGC2_QSPI(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_QSPI_SHIFT)) & SIM_SCGC2_QSPI_MASK)
#define SIM_SCGC2_FLEXIO_MASK                    (0x80000000U)
#define SIM_SCGC2_FLEXIO_SHIFT                   (31U)
#define SIM_SCGC2_FLEXIO(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC2_FLEXIO_SHIFT)) & SIM_SCGC2_FLEXIO_MASK)

/*! @name SCGC3 - System Clock Gating Control Register 3 */
#define SIM_SCGC3_TRNG_MASK                      (0x1U)
#define SIM_SCGC3_TRNG_SHIFT                     (0U)
#define SIM_SCGC3_TRNG(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC3_TRNG_SHIFT)) & SIM_SCGC3_TRNG_MASK)
#define SIM_SCGC3_SPI2_MASK                      (0x1000U)
#define SIM_SCGC3_SPI2_SHIFT                     (12U)
#define SIM_SCGC3_SPI2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC3_SPI2_SHIFT)) & SIM_SCGC3_SPI2_MASK)
#define SIM_SCGC3_SDHC_MASK                      (0x20000U)
#define SIM_SCGC3_SDHC_SHIFT                     (17U)
#define SIM_SCGC3_SDHC(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC3_SDHC_SHIFT)) & SIM_SCGC3_SDHC_MASK)
#define SIM_SCGC3_FTM2_MASK                      (0x1000000U)
#define SIM_SCGC3_FTM2_SHIFT                     (24U)
#define SIM_SCGC3_FTM2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC3_FTM2_SHIFT)) & SIM_SCGC3_FTM2_MASK)
#define SIM_SCGC3_FTM3_MASK                      (0x2000000U)
#define SIM_SCGC3_FTM3_SHIFT                     (25U)
#define SIM_SCGC3_FTM3(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC3_FTM3_SHIFT)) & SIM_SCGC3_FTM3_MASK)

/*! @name SCGC4 - System Clock Gating Control Register 4 */
#define SIM_SCGC4_EWM_MASK                       (0x2U)
#define SIM_SCGC4_EWM_SHIFT                      (1U)
#define SIM_SCGC4_EWM(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_EWM_SHIFT)) & SIM_SCGC4_EWM_MASK)
#define SIM_SCGC4_CMT_MASK                       (0x4U)
#define SIM_SCGC4_CMT_SHIFT                      (2U)
#define SIM_SCGC4_CMT(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_CMT_SHIFT)) & SIM_SCGC4_CMT_MASK)
#define SIM_SCGC4_I2C0_MASK                      (0x40U)
#define SIM_SCGC4_I2C0_SHIFT                     (6U)
#define SIM_SCGC4_I2C0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_I2C0_SHIFT)) & SIM_SCGC4_I2C0_MASK)
#define SIM_SCGC4_I2C1_MASK                      (0x80U)
#define SIM_SCGC4_I2C1_SHIFT                     (7U)
#define SIM_SCGC4_I2C1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_I2C1_SHIFT)) & SIM_SCGC4_I2C1_MASK)
#define SIM_SCGC4_USBOTG_MASK                    (0x40000U)
#define SIM_SCGC4_USBOTG_SHIFT                   (18U)
#define SIM_SCGC4_USBOTG(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_USBOTG_SHIFT)) & SIM_SCGC4_USBOTG_MASK)
#define SIM_SCGC4_CMP_MASK                       (0x80000U)
#define SIM_SCGC4_CMP_SHIFT                      (19U)
#define SIM_SCGC4_CMP(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_CMP_SHIFT)) & SIM_SCGC4_CMP_MASK)
#define SIM_SCGC4_VREF_MASK                      (0x100000U)
#define SIM_SCGC4_VREF_SHIFT                     (20U)
#define SIM_SCGC4_VREF(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_VREF_SHIFT)) & SIM_SCGC4_VREF_MASK)

/*! @name SCGC5 - System Clock Gating Control Register 5 */
#define SIM_SCGC5_LPTMR_MASK                     (0x1U)
#define SIM_SCGC5_LPTMR_SHIFT                    (0U)
#define SIM_SCGC5_LPTMR(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_LPTMR_SHIFT)) & SIM_SCGC5_LPTMR_MASK)
#define SIM_SCGC5_LPTMR1_MASK                    (0x10U)
#define SIM_SCGC5_LPTMR1_SHIFT                   (4U)
#define SIM_SCGC5_LPTMR1(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_LPTMR1_SHIFT)) & SIM_SCGC5_LPTMR1_MASK)
#define SIM_SCGC5_TSI_MASK                       (0x20U)
#define SIM_SCGC5_TSI_SHIFT                      (5U)
#define SIM_SCGC5_TSI(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_TSI_SHIFT)) & SIM_SCGC5_TSI_MASK)
#define SIM_SCGC5_PORTA_MASK                     (0x200U)
#define SIM_SCGC5_PORTA_SHIFT                    (9U)
#define SIM_SCGC5_PORTA(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PORTA_SHIFT)) & SIM_SCGC5_PORTA_MASK)
#define SIM_SCGC5_PORTB_MASK                     (0x400U)
#define SIM_SCGC5_PORTB_SHIFT                    (10U)
#define SIM_SCGC5_PORTB(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PORTB_SHIFT)) & SIM_SCGC5_PORTB_MASK)
#define SIM_SCGC5_PORTC_MASK                     (0x800U)
#define SIM_SCGC5_PORTC_SHIFT                    (11U)
#define SIM_SCGC5_PORTC(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PORTC_SHIFT)) & SIM_SCGC5_PORTC_MASK)
#define SIM_SCGC5_PORTD_MASK                     (0x1000U)
#define SIM_SCGC5_PORTD_SHIFT                    (12U)
#define SIM_SCGC5_PORTD(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PORTD_SHIFT)) & SIM_SCGC5_PORTD_MASK)
#define SIM_SCGC5_PORTE_MASK                     (0x2000U)
#define SIM_SCGC5_PORTE_SHIFT                    (13U)
#define SIM_SCGC5_PORTE(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PORTE_SHIFT)) & SIM_SCGC5_PORTE_MASK)

/*! @name SCGC6 - System Clock Gating Control Register 6 */
#define SIM_SCGC6_FTF_MASK                       (0x1U)
#define SIM_SCGC6_FTF_SHIFT                      (0U)
#define SIM_SCGC6_FTF(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_FTF_SHIFT)) & SIM_SCGC6_FTF_MASK)
#define SIM_SCGC6_DMAMUX_MASK                    (0x2U)
#define SIM_SCGC6_DMAMUX_SHIFT                   (1U)
#define SIM_SCGC6_DMAMUX(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_DMAMUX_SHIFT)) & SIM_SCGC6_DMAMUX_MASK)
#define SIM_SCGC6_SPI0_MASK                      (0x1000U)
#define SIM_SCGC6_SPI0_SHIFT                     (12U)
#define SIM_SCGC6_SPI0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_SPI0_SHIFT)) & SIM_SCGC6_SPI0_MASK)
#define SIM_SCGC6_SPI1_MASK                      (0x2000U)
#define SIM_SCGC6_SPI1_SHIFT                     (13U)
#define SIM_SCGC6_SPI1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_SPI1_SHIFT)) & SIM_SCGC6_SPI1_MASK)
#define SIM_SCGC6_I2S_MASK                       (0x8000U)
#define SIM_SCGC6_I2S_SHIFT                      (15U)
#define SIM_SCGC6_I2S(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_I2S_SHIFT)) & SIM_SCGC6_I2S_MASK)
#define SIM_SCGC6_CRC_MASK                       (0x40000U)
#define SIM_SCGC6_CRC_SHIFT                      (18U)
#define SIM_SCGC6_CRC(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_CRC_SHIFT)) & SIM_SCGC6_CRC_MASK)
#define SIM_SCGC6_USBDCD_MASK                    (0x200000U)
#define SIM_SCGC6_USBDCD_SHIFT                   (21U)
#define SIM_SCGC6_USBDCD(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_USBDCD_SHIFT)) & SIM_SCGC6_USBDCD_MASK)
#define SIM_SCGC6_PDB_MASK                       (0x400000U)
#define SIM_SCGC6_PDB_SHIFT                      (22U)
#define SIM_SCGC6_PDB(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_PDB_SHIFT)) & SIM_SCGC6_PDB_MASK)
#define SIM_SCGC6_PIT_MASK                       (0x800000U)
#define SIM_SCGC6_PIT_SHIFT                      (23U)
#define SIM_SCGC6_PIT(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_PIT_SHIFT)) & SIM_SCGC6_PIT_MASK)
#define SIM_SCGC6_FTM0_MASK                      (0x1000000U)
#define SIM_SCGC6_FTM0_SHIFT                     (24U)
#define SIM_SCGC6_FTM0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_FTM0_SHIFT)) & SIM_SCGC6_FTM0_MASK)
#define SIM_SCGC6_FTM1_MASK                      (0x2000000U)
#define SIM_SCGC6_FTM1_SHIFT                     (25U)
#define SIM_SCGC6_FTM1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_FTM1_SHIFT)) & SIM_SCGC6_FTM1_MASK)
#define SIM_SCGC6_FTM2_MASK                      (0x4000000U)
#define SIM_SCGC6_FTM2_SHIFT                     (26U)
#define SIM_SCGC6_FTM2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_FTM2_SHIFT)) & SIM_SCGC6_FTM2_MASK)
#define SIM_SCGC6_ADC0_MASK                      (0x8000000U)
#define SIM_SCGC6_ADC0_SHIFT                     (27U)
#define SIM_SCGC6_ADC0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_ADC0_SHIFT)) & SIM_SCGC6_ADC0_MASK)
#define SIM_SCGC6_RTC_MASK                       (0x20000000U)
#define SIM_SCGC6_RTC_SHIFT                      (29U)
#define SIM_SCGC6_RTC(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_RTC_SHIFT)) & SIM_SCGC6_RTC_MASK)
#define SIM_SCGC6_DAC0_MASK                      (0x80000000U)
#define SIM_SCGC6_DAC0_SHIFT                     (31U)
#define SIM_SCGC6_DAC0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_DAC0_SHIFT)) & SIM_SCGC6_DAC0_MASK)

/*! @name SCGC7 - System Clock Gating Control Register 7 */
#define SIM_SCGC7_FLEXBUS_MASK                   (0x1U)
#define SIM_SCGC7_FLEXBUS_SHIFT                  (0U)
#define SIM_SCGC7_FLEXBUS(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC7_FLEXBUS_SHIFT)) & SIM_SCGC7_FLEXBUS_MASK)
#define SIM_SCGC7_DMA_MASK                       (0x2U)
#define SIM_SCGC7_DMA_SHIFT                      (1U)
#define SIM_SCGC7_DMA(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC7_DMA_SHIFT)) & SIM_SCGC7_DMA_MASK)
#define SIM_SCGC7_MPU_MASK                       (0x4U)
#define SIM_SCGC7_MPU_SHIFT                      (2U)
#define SIM_SCGC7_MPU(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC7_MPU_SHIFT)) & SIM_SCGC7_MPU_MASK)
#define SIM_SCGC7_SDRAMC_MASK                    (0x8U)
#define SIM_SCGC7_SDRAMC_SHIFT                   (3U)
#define SIM_SCGC7_SDRAMC(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC7_SDRAMC_SHIFT)) & SIM_SCGC7_SDRAMC_MASK)

/*! @name CLKDIV1 - System Clock Divider Register 1 */
#define SIM_CLKDIV1_OUTDIV4_MASK                 (0xF0000U)
#define SIM_CLKDIV1_OUTDIV4_SHIFT                (16U)
#define SIM_CLKDIV1_OUTDIV4(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV4_SHIFT)) & SIM_CLKDIV1_OUTDIV4_MASK)
#define SIM_CLKDIV1_OUTDIV3_MASK                 (0xF00000U)
#define SIM_CLKDIV1_OUTDIV3_SHIFT                (20U)
#define SIM_CLKDIV1_OUTDIV3(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV3_SHIFT)) & SIM_CLKDIV1_OUTDIV3_MASK)
#define SIM_CLKDIV1_OUTDIV2_MASK                 (0xF000000U)
#define SIM_CLKDIV1_OUTDIV2_SHIFT                (24U)
#define SIM_CLKDIV1_OUTDIV2(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV2_SHIFT)) & SIM_CLKDIV1_OUTDIV2_MASK)
#define SIM_CLKDIV1_OUTDIV1_MASK                 (0xF0000000U)
#define SIM_CLKDIV1_OUTDIV1_SHIFT                (28U)
#define SIM_CLKDIV1_OUTDIV1(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV1_SHIFT)) & SIM_CLKDIV1_OUTDIV1_MASK)

/*! @name CLKDIV2 - System Clock Divider Register 2 */
#define SIM_CLKDIV2_USBFRAC_MASK                 (0x1U)
#define SIM_CLKDIV2_USBFRAC_SHIFT                (0U)
#define SIM_CLKDIV2_USBFRAC(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV2_USBFRAC_SHIFT)) & SIM_CLKDIV2_USBFRAC_MASK)
#define SIM_CLKDIV2_USBDIV_MASK                  (0xEU)
#define SIM_CLKDIV2_USBDIV_SHIFT                 (1U)
#define SIM_CLKDIV2_USBDIV(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV2_USBDIV_SHIFT)) & SIM_CLKDIV2_USBDIV_MASK)

/*! @name FCFG1 - Flash Configuration Register 1 */
#define SIM_FCFG1_FLASHDIS_MASK                  (0x1U)
#define SIM_FCFG1_FLASHDIS_SHIFT                 (0U)
#define SIM_FCFG1_FLASHDIS(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_FLASHDIS_SHIFT)) & SIM_FCFG1_FLASHDIS_MASK)
#define SIM_FCFG1_FLASHDOZE_MASK                 (0x2U)
#define SIM_FCFG1_FLASHDOZE_SHIFT                (1U)
#define SIM_FCFG1_FLASHDOZE(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_FLASHDOZE_SHIFT)) & SIM_FCFG1_FLASHDOZE_MASK)
#define SIM_FCFG1_PFSIZE_MASK                    (0xF000000U)
#define SIM_FCFG1_PFSIZE_SHIFT                   (24U)
#define SIM_FCFG1_PFSIZE(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_PFSIZE_SHIFT)) & SIM_FCFG1_PFSIZE_MASK)

/*! @name FCFG2 - Flash Configuration Register 2 */
#define SIM_FCFG2_MAXADDR1_MASK                  (0x7F0000U)
#define SIM_FCFG2_MAXADDR1_SHIFT                 (16U)
#define SIM_FCFG2_MAXADDR1(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG2_MAXADDR1_SHIFT)) & SIM_FCFG2_MAXADDR1_MASK)
#define SIM_FCFG2_MAXADDR0_MASK                  (0x7F000000U)
#define SIM_FCFG2_MAXADDR0_SHIFT                 (24U)
#define SIM_FCFG2_MAXADDR0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG2_MAXADDR0_SHIFT)) & SIM_FCFG2_MAXADDR0_MASK)

/*! @name UIDH - Unique Identification Register High */
#define SIM_UIDH_UID_MASK                        (0xFFFFFFFFU)
#define SIM_UIDH_UID_SHIFT                       (0U)
#define SIM_UIDH_UID(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_UIDH_UID_SHIFT)) & SIM_UIDH_UID_MASK)

/*! @name UIDMH - Unique Identification Register Mid-High */
#define SIM_UIDMH_UID_MASK                       (0xFFFFFFFFU)
#define SIM_UIDMH_UID_SHIFT                      (0U)
#define SIM_UIDMH_UID(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_UIDMH_UID_SHIFT)) & SIM_UIDMH_UID_MASK)

/*! @name UIDML - Unique Identification Register Mid Low */
#define SIM_UIDML_UID_MASK                       (0xFFFFFFFFU)
#define SIM_UIDML_UID_SHIFT                      (0U)
#define SIM_UIDML_UID(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_UIDML_UID_SHIFT)) & SIM_UIDML_UID_MASK)

/*! @name UIDL - Unique Identification Register Low */
#define SIM_UIDL_UID_MASK                        (0xFFFFFFFFU)
#define SIM_UIDL_UID_SHIFT                       (0U)
#define SIM_UIDL_UID(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_UIDL_UID_SHIFT)) & SIM_UIDL_UID_MASK)

/*! @name CLKDIV3 - System Clock Divider Register 3 */
#define SIM_CLKDIV3_PLLFLLFRAC_MASK              (0x1U)
#define SIM_CLKDIV3_PLLFLLFRAC_SHIFT             (0U)
#define SIM_CLKDIV3_PLLFLLFRAC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV3_PLLFLLFRAC_SHIFT)) & SIM_CLKDIV3_PLLFLLFRAC_MASK)
#define SIM_CLKDIV3_PLLFLLDIV_MASK               (0xEU)
#define SIM_CLKDIV3_PLLFLLDIV_SHIFT              (1U)
#define SIM_CLKDIV3_PLLFLLDIV(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV3_PLLFLLDIV_SHIFT)) & SIM_CLKDIV3_PLLFLLDIV_MASK)

/*! @name CLKDIV4 - System Clock Divider Register 4 */
#define SIM_CLKDIV4_TRACEFRAC_MASK               (0x1U)
#define SIM_CLKDIV4_TRACEFRAC_SHIFT              (0U)
#define SIM_CLKDIV4_TRACEFRAC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV4_TRACEFRAC_SHIFT)) & SIM_CLKDIV4_TRACEFRAC_MASK)
#define SIM_CLKDIV4_TRACEDIV_MASK                (0xEU)
#define SIM_CLKDIV4_TRACEDIV_SHIFT               (1U)
#define SIM_CLKDIV4_TRACEDIV(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV4_TRACEDIV_SHIFT)) & SIM_CLKDIV4_TRACEDIV_MASK)


/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40047000u)
/** Peripheral SIM base pointer */
#define SIM                                      ((SIM_Type *)SIM_BASE)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS                           { SIM_BASE }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM }

/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Peripheral_Access_Layer SMC Peripheral Access Layer
 * @{
 */

/** SMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t PMPROT;                             /**< Power Mode Protection register, offset: 0x0 */
  __IO uint8_t PMCTRL;                             /**< Power Mode Control register, offset: 0x1 */
  __IO uint8_t STOPCTRL;                           /**< Stop Control Register, offset: 0x2 */
  __I  uint8_t PMSTAT;                             /**< Power Mode Status register, offset: 0x3 */
} SMC_Type;

/* ----------------------------------------------------------------------------
   -- SMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Masks SMC Register Masks
 * @{
 */

/*! @name PMPROT - Power Mode Protection register */
#define SMC_PMPROT_AVLLS_MASK                    (0x2U)
#define SMC_PMPROT_AVLLS_SHIFT                   (1U)
#define SMC_PMPROT_AVLLS(x)                      (((uint8_t)(((uint8_t)(x)) << SMC_PMPROT_AVLLS_SHIFT)) & SMC_PMPROT_AVLLS_MASK)
#define SMC_PMPROT_ALLS_MASK                     (0x8U)
#define SMC_PMPROT_ALLS_SHIFT                    (3U)
#define SMC_PMPROT_ALLS(x)                       (((uint8_t)(((uint8_t)(x)) << SMC_PMPROT_ALLS_SHIFT)) & SMC_PMPROT_ALLS_MASK)
#define SMC_PMPROT_AVLP_MASK                     (0x20U)
#define SMC_PMPROT_AVLP_SHIFT                    (5U)
#define SMC_PMPROT_AVLP(x)                       (((uint8_t)(((uint8_t)(x)) << SMC_PMPROT_AVLP_SHIFT)) & SMC_PMPROT_AVLP_MASK)
#define SMC_PMPROT_AHSRUN_MASK                   (0x80U)
#define SMC_PMPROT_AHSRUN_SHIFT                  (7U)
#define SMC_PMPROT_AHSRUN(x)                     (((uint8_t)(((uint8_t)(x)) << SMC_PMPROT_AHSRUN_SHIFT)) & SMC_PMPROT_AHSRUN_MASK)

/*! @name PMCTRL - Power Mode Control register */
#define SMC_PMCTRL_STOPM_MASK                    (0x7U)
#define SMC_PMCTRL_STOPM_SHIFT                   (0U)
#define SMC_PMCTRL_STOPM(x)                      (((uint8_t)(((uint8_t)(x)) << SMC_PMCTRL_STOPM_SHIFT)) & SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_STOPA_MASK                    (0x8U)
#define SMC_PMCTRL_STOPA_SHIFT                   (3U)
#define SMC_PMCTRL_STOPA(x)                      (((uint8_t)(((uint8_t)(x)) << SMC_PMCTRL_STOPA_SHIFT)) & SMC_PMCTRL_STOPA_MASK)
#define SMC_PMCTRL_RUNM_MASK                     (0x60U)
#define SMC_PMCTRL_RUNM_SHIFT                    (5U)
#define SMC_PMCTRL_RUNM(x)                       (((uint8_t)(((uint8_t)(x)) << SMC_PMCTRL_RUNM_SHIFT)) & SMC_PMCTRL_RUNM_MASK)

/*! @name STOPCTRL - Stop Control Register */
#define SMC_STOPCTRL_LLSM_MASK                   (0x7U)
#define SMC_STOPCTRL_LLSM_SHIFT                  (0U)
#define SMC_STOPCTRL_LLSM(x)                     (((uint8_t)(((uint8_t)(x)) << SMC_STOPCTRL_LLSM_SHIFT)) & SMC_STOPCTRL_LLSM_MASK)
#define SMC_STOPCTRL_LPOPO_MASK                  (0x8U)
#define SMC_STOPCTRL_LPOPO_SHIFT                 (3U)
#define SMC_STOPCTRL_LPOPO(x)                    (((uint8_t)(((uint8_t)(x)) << SMC_STOPCTRL_LPOPO_SHIFT)) & SMC_STOPCTRL_LPOPO_MASK)
#define SMC_STOPCTRL_RAM2PO_MASK                 (0x10U)
#define SMC_STOPCTRL_RAM2PO_SHIFT                (4U)
#define SMC_STOPCTRL_RAM2PO(x)                   (((uint8_t)(((uint8_t)(x)) << SMC_STOPCTRL_RAM2PO_SHIFT)) & SMC_STOPCTRL_RAM2PO_MASK)
#define SMC_STOPCTRL_PORPO_MASK                  (0x20U)
#define SMC_STOPCTRL_PORPO_SHIFT                 (5U)
#define SMC_STOPCTRL_PORPO(x)                    (((uint8_t)(((uint8_t)(x)) << SMC_STOPCTRL_PORPO_SHIFT)) & SMC_STOPCTRL_PORPO_MASK)
#define SMC_STOPCTRL_PSTOPO_MASK                 (0xC0U)
#define SMC_STOPCTRL_PSTOPO_SHIFT                (6U)
#define SMC_STOPCTRL_PSTOPO(x)                   (((uint8_t)(((uint8_t)(x)) << SMC_STOPCTRL_PSTOPO_SHIFT)) & SMC_STOPCTRL_PSTOPO_MASK)

/*! @name PMSTAT - Power Mode Status register */
#define SMC_PMSTAT_PMSTAT_MASK                   (0xFFU)
#define SMC_PMSTAT_PMSTAT_SHIFT                  (0U)
#define SMC_PMSTAT_PMSTAT(x)                     (((uint8_t)(((uint8_t)(x)) << SMC_PMSTAT_PMSTAT_SHIFT)) & SMC_PMSTAT_PMSTAT_MASK)


/*!
 * @}
 */ /* end of group SMC_Register_Masks */


/* SMC - Peripheral instance base addresses */
/** Peripheral SMC base address */
#define SMC_BASE                                 (0x4007E000u)
/** Peripheral SMC base pointer */
#define SMC                                      ((SMC_Type *)SMC_BASE)
/** Array initializer of SMC peripheral base addresses */
#define SMC_BASE_ADDRS                           { SMC_BASE }
/** Array initializer of SMC peripheral base pointers */
#define SMC_BASE_PTRS                            { SMC }

/*!
 * @}
 */ /* end of group SMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 * @{
 */

/** SPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t TCR;                               /**< Transfer Count Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t CTAR[2];                           /**< Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];                     /**< Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint8_t RESERVED_1[24];
  __IO uint32_t SR;                                /**< Status Register, offset: 0x2C */
  __IO uint32_t RSER;                              /**< DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                                          /* offset: 0x34 */
    __IO uint32_t PUSHR;                             /**< PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;                       /**< PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;                              /**< POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR0;                             /**< Transmit FIFO Registers, offset: 0x3C */
  __I  uint32_t TXFR1;                             /**< Transmit FIFO Registers, offset: 0x40 */
  __I  uint32_t TXFR2;                             /**< Transmit FIFO Registers, offset: 0x44 */
  __I  uint32_t TXFR3;                             /**< Transmit FIFO Registers, offset: 0x48 */
       uint8_t RESERVED_2[48];
  __I  uint32_t RXFR0;                             /**< Receive FIFO Registers, offset: 0x7C */
  __I  uint32_t RXFR1;                             /**< Receive FIFO Registers, offset: 0x80 */
  __I  uint32_t RXFR2;                             /**< Receive FIFO Registers, offset: 0x84 */
  __I  uint32_t RXFR3;                             /**< Receive FIFO Registers, offset: 0x88 */
} SPI_Type;

/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Masks SPI Register Masks
 * @{
 */

/*! @name MCR - Module Configuration Register */
#define SPI_MCR_HALT_MASK                        (0x1U)
#define SPI_MCR_HALT_SHIFT                       (0U)
#define SPI_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_HALT_SHIFT)) & SPI_MCR_HALT_MASK)
#define SPI_MCR_SMPL_PT_MASK                     (0x300U)
#define SPI_MCR_SMPL_PT_SHIFT                    (8U)
#define SPI_MCR_SMPL_PT(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_MCR_SMPL_PT_SHIFT)) & SPI_MCR_SMPL_PT_MASK)
#define SPI_MCR_CLR_RXF_MASK                     (0x400U)
#define SPI_MCR_CLR_RXF_SHIFT                    (10U)
#define SPI_MCR_CLR_RXF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_MCR_CLR_RXF_SHIFT)) & SPI_MCR_CLR_RXF_MASK)
#define SPI_MCR_CLR_TXF_MASK                     (0x800U)
#define SPI_MCR_CLR_TXF_SHIFT                    (11U)
#define SPI_MCR_CLR_TXF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_MCR_CLR_TXF_SHIFT)) & SPI_MCR_CLR_TXF_MASK)
#define SPI_MCR_DIS_RXF_MASK                     (0x1000U)
#define SPI_MCR_DIS_RXF_SHIFT                    (12U)
#define SPI_MCR_DIS_RXF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_MCR_DIS_RXF_SHIFT)) & SPI_MCR_DIS_RXF_MASK)
#define SPI_MCR_DIS_TXF_MASK                     (0x2000U)
#define SPI_MCR_DIS_TXF_SHIFT                    (13U)
#define SPI_MCR_DIS_TXF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_MCR_DIS_TXF_SHIFT)) & SPI_MCR_DIS_TXF_MASK)
#define SPI_MCR_MDIS_MASK                        (0x4000U)
#define SPI_MCR_MDIS_SHIFT                       (14U)
#define SPI_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_MDIS_SHIFT)) & SPI_MCR_MDIS_MASK)
#define SPI_MCR_DOZE_MASK                        (0x8000U)
#define SPI_MCR_DOZE_SHIFT                       (15U)
#define SPI_MCR_DOZE(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_DOZE_SHIFT)) & SPI_MCR_DOZE_MASK)
#define SPI_MCR_PCSIS_MASK                       (0x3F0000U)
#define SPI_MCR_PCSIS_SHIFT                      (16U)
#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_MCR_PCSIS_SHIFT)) & SPI_MCR_PCSIS_MASK)
#define SPI_MCR_ROOE_MASK                        (0x1000000U)
#define SPI_MCR_ROOE_SHIFT                       (24U)
#define SPI_MCR_ROOE(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_ROOE_SHIFT)) & SPI_MCR_ROOE_MASK)
#define SPI_MCR_PCSSE_MASK                       (0x2000000U)
#define SPI_MCR_PCSSE_SHIFT                      (25U)
#define SPI_MCR_PCSSE(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_MCR_PCSSE_SHIFT)) & SPI_MCR_PCSSE_MASK)
#define SPI_MCR_MTFE_MASK                        (0x4000000U)
#define SPI_MCR_MTFE_SHIFT                       (26U)
#define SPI_MCR_MTFE(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_MTFE_SHIFT)) & SPI_MCR_MTFE_MASK)
#define SPI_MCR_FRZ_MASK                         (0x8000000U)
#define SPI_MCR_FRZ_SHIFT                        (27U)
#define SPI_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_MCR_FRZ_SHIFT)) & SPI_MCR_FRZ_MASK)
#define SPI_MCR_DCONF_MASK                       (0x30000000U)
#define SPI_MCR_DCONF_SHIFT                      (28U)
#define SPI_MCR_DCONF(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_MCR_DCONF_SHIFT)) & SPI_MCR_DCONF_MASK)
#define SPI_MCR_CONT_SCKE_MASK                   (0x40000000U)
#define SPI_MCR_CONT_SCKE_SHIFT                  (30U)
#define SPI_MCR_CONT_SCKE(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_MCR_CONT_SCKE_SHIFT)) & SPI_MCR_CONT_SCKE_MASK)
#define SPI_MCR_MSTR_MASK                        (0x80000000U)
#define SPI_MCR_MSTR_SHIFT                       (31U)
#define SPI_MCR_MSTR(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_MSTR_SHIFT)) & SPI_MCR_MSTR_MASK)

/*! @name TCR - Transfer Count Register */
#define SPI_TCR_SPI_TCNT_MASK                    (0xFFFF0000U)
#define SPI_TCR_SPI_TCNT_SHIFT                   (16U)
#define SPI_TCR_SPI_TCNT(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_TCR_SPI_TCNT_SHIFT)) & SPI_TCR_SPI_TCNT_MASK)

/*! @name CTAR - Clock and Transfer Attributes Register (In Master Mode) */
#define SPI_CTAR_BR_MASK                         (0xFU)
#define SPI_CTAR_BR_SHIFT                        (0U)
#define SPI_CTAR_BR(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_BR_SHIFT)) & SPI_CTAR_BR_MASK)
#define SPI_CTAR_DT_MASK                         (0xF0U)
#define SPI_CTAR_DT_SHIFT                        (4U)
#define SPI_CTAR_DT(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_DT_SHIFT)) & SPI_CTAR_DT_MASK)
#define SPI_CTAR_ASC_MASK                        (0xF00U)
#define SPI_CTAR_ASC_SHIFT                       (8U)
#define SPI_CTAR_ASC(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_ASC_SHIFT)) & SPI_CTAR_ASC_MASK)
#define SPI_CTAR_CSSCK_MASK                      (0xF000U)
#define SPI_CTAR_CSSCK_SHIFT                     (12U)
#define SPI_CTAR_CSSCK(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_CSSCK_SHIFT)) & SPI_CTAR_CSSCK_MASK)
#define SPI_CTAR_PBR_MASK                        (0x30000U)
#define SPI_CTAR_PBR_SHIFT                       (16U)
#define SPI_CTAR_PBR(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_PBR_SHIFT)) & SPI_CTAR_PBR_MASK)
#define SPI_CTAR_PDT_MASK                        (0xC0000U)
#define SPI_CTAR_PDT_SHIFT                       (18U)
#define SPI_CTAR_PDT(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_PDT_SHIFT)) & SPI_CTAR_PDT_MASK)
#define SPI_CTAR_PASC_MASK                       (0x300000U)
#define SPI_CTAR_PASC_SHIFT                      (20U)
#define SPI_CTAR_PASC(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_PASC_SHIFT)) & SPI_CTAR_PASC_MASK)
#define SPI_CTAR_PCSSCK_MASK                     (0xC00000U)
#define SPI_CTAR_PCSSCK_SHIFT                    (22U)
#define SPI_CTAR_PCSSCK(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_PCSSCK_SHIFT)) & SPI_CTAR_PCSSCK_MASK)
#define SPI_CTAR_LSBFE_MASK                      (0x1000000U)
#define SPI_CTAR_LSBFE_SHIFT                     (24U)
#define SPI_CTAR_LSBFE(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_LSBFE_SHIFT)) & SPI_CTAR_LSBFE_MASK)
#define SPI_CTAR_CPHA_MASK                       (0x2000000U)
#define SPI_CTAR_CPHA_SHIFT                      (25U)
#define SPI_CTAR_CPHA(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_CPHA_SHIFT)) & SPI_CTAR_CPHA_MASK)
#define SPI_CTAR_CPOL_MASK                       (0x4000000U)
#define SPI_CTAR_CPOL_SHIFT                      (26U)
#define SPI_CTAR_CPOL(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_CPOL_SHIFT)) & SPI_CTAR_CPOL_MASK)
#define SPI_CTAR_FMSZ_MASK                       (0x78000000U)
#define SPI_CTAR_FMSZ_SHIFT                      (27U)
#define SPI_CTAR_FMSZ(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_FMSZ_SHIFT)) & SPI_CTAR_FMSZ_MASK)
#define SPI_CTAR_DBR_MASK                        (0x80000000U)
#define SPI_CTAR_DBR_SHIFT                       (31U)
#define SPI_CTAR_DBR(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_DBR_SHIFT)) & SPI_CTAR_DBR_MASK)

/* The count of SPI_CTAR */
#define SPI_CTAR_COUNT                           (2U)

/*! @name CTAR_SLAVE - Clock and Transfer Attributes Register (In Slave Mode) */
#define SPI_CTAR_SLAVE_CPHA_MASK                 (0x2000000U)
#define SPI_CTAR_SLAVE_CPHA_SHIFT                (25U)
#define SPI_CTAR_SLAVE_CPHA(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_SLAVE_CPHA_SHIFT)) & SPI_CTAR_SLAVE_CPHA_MASK)
#define SPI_CTAR_SLAVE_CPOL_MASK                 (0x4000000U)
#define SPI_CTAR_SLAVE_CPOL_SHIFT                (26U)
#define SPI_CTAR_SLAVE_CPOL(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_SLAVE_CPOL_SHIFT)) & SPI_CTAR_SLAVE_CPOL_MASK)
#define SPI_CTAR_SLAVE_FMSZ_MASK                 (0x78000000U)
#define SPI_CTAR_SLAVE_FMSZ_SHIFT                (27U)
#define SPI_CTAR_SLAVE_FMSZ(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_CTAR_SLAVE_FMSZ_SHIFT)) & SPI_CTAR_SLAVE_FMSZ_MASK)

/* The count of SPI_CTAR_SLAVE */
#define SPI_CTAR_SLAVE_COUNT                     (1U)

/*! @name SR - Status Register */
#define SPI_SR_POPNXTPTR_MASK                    (0xFU)
#define SPI_SR_POPNXTPTR_SHIFT                   (0U)
#define SPI_SR_POPNXTPTR(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_SR_POPNXTPTR_SHIFT)) & SPI_SR_POPNXTPTR_MASK)
#define SPI_SR_RXCTR_MASK                        (0xF0U)
#define SPI_SR_RXCTR_SHIFT                       (4U)
#define SPI_SR_RXCTR(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_SR_RXCTR_SHIFT)) & SPI_SR_RXCTR_MASK)
#define SPI_SR_TXNXTPTR_MASK                     (0xF00U)
#define SPI_SR_TXNXTPTR_SHIFT                    (8U)
#define SPI_SR_TXNXTPTR(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_SR_TXNXTPTR_SHIFT)) & SPI_SR_TXNXTPTR_MASK)
#define SPI_SR_TXCTR_MASK                        (0xF000U)
#define SPI_SR_TXCTR_SHIFT                       (12U)
#define SPI_SR_TXCTR(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_SR_TXCTR_SHIFT)) & SPI_SR_TXCTR_MASK)
#define SPI_SR_RFDF_MASK                         (0x20000U)
#define SPI_SR_RFDF_SHIFT                        (17U)
#define SPI_SR_RFDF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_SR_RFDF_SHIFT)) & SPI_SR_RFDF_MASK)
#define SPI_SR_RFOF_MASK                         (0x80000U)
#define SPI_SR_RFOF_SHIFT                        (19U)
#define SPI_SR_RFOF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_SR_RFOF_SHIFT)) & SPI_SR_RFOF_MASK)
#define SPI_SR_TFFF_MASK                         (0x2000000U)
#define SPI_SR_TFFF_SHIFT                        (25U)
#define SPI_SR_TFFF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_SR_TFFF_SHIFT)) & SPI_SR_TFFF_MASK)
#define SPI_SR_TFUF_MASK                         (0x8000000U)
#define SPI_SR_TFUF_SHIFT                        (27U)
#define SPI_SR_TFUF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_SR_TFUF_SHIFT)) & SPI_SR_TFUF_MASK)
#define SPI_SR_EOQF_MASK                         (0x10000000U)
#define SPI_SR_EOQF_SHIFT                        (28U)
#define SPI_SR_EOQF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_SR_EOQF_SHIFT)) & SPI_SR_EOQF_MASK)
#define SPI_SR_TXRXS_MASK                        (0x40000000U)
#define SPI_SR_TXRXS_SHIFT                       (30U)
#define SPI_SR_TXRXS(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_SR_TXRXS_SHIFT)) & SPI_SR_TXRXS_MASK)
#define SPI_SR_TCF_MASK                          (0x80000000U)
#define SPI_SR_TCF_SHIFT                         (31U)
#define SPI_SR_TCF(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_SR_TCF_SHIFT)) & SPI_SR_TCF_MASK)

/*! @name RSER - DMA/Interrupt Request Select and Enable Register */
#define SPI_RSER_RFDF_DIRS_MASK                  (0x10000U)
#define SPI_RSER_RFDF_DIRS_SHIFT                 (16U)
#define SPI_RSER_RFDF_DIRS(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_RSER_RFDF_DIRS_SHIFT)) & SPI_RSER_RFDF_DIRS_MASK)
#define SPI_RSER_RFDF_RE_MASK                    (0x20000U)
#define SPI_RSER_RFDF_RE_SHIFT                   (17U)
#define SPI_RSER_RFDF_RE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RSER_RFDF_RE_SHIFT)) & SPI_RSER_RFDF_RE_MASK)
#define SPI_RSER_RFOF_RE_MASK                    (0x80000U)
#define SPI_RSER_RFOF_RE_SHIFT                   (19U)
#define SPI_RSER_RFOF_RE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RSER_RFOF_RE_SHIFT)) & SPI_RSER_RFOF_RE_MASK)
#define SPI_RSER_TFFF_DIRS_MASK                  (0x1000000U)
#define SPI_RSER_TFFF_DIRS_SHIFT                 (24U)
#define SPI_RSER_TFFF_DIRS(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_RSER_TFFF_DIRS_SHIFT)) & SPI_RSER_TFFF_DIRS_MASK)
#define SPI_RSER_TFFF_RE_MASK                    (0x2000000U)
#define SPI_RSER_TFFF_RE_SHIFT                   (25U)
#define SPI_RSER_TFFF_RE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RSER_TFFF_RE_SHIFT)) & SPI_RSER_TFFF_RE_MASK)
#define SPI_RSER_TFUF_RE_MASK                    (0x8000000U)
#define SPI_RSER_TFUF_RE_SHIFT                   (27U)
#define SPI_RSER_TFUF_RE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RSER_TFUF_RE_SHIFT)) & SPI_RSER_TFUF_RE_MASK)
#define SPI_RSER_EOQF_RE_MASK                    (0x10000000U)
#define SPI_RSER_EOQF_RE_SHIFT                   (28U)
#define SPI_RSER_EOQF_RE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RSER_EOQF_RE_SHIFT)) & SPI_RSER_EOQF_RE_MASK)
#define SPI_RSER_TCF_RE_MASK                     (0x80000000U)
#define SPI_RSER_TCF_RE_SHIFT                    (31U)
#define SPI_RSER_TCF_RE(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_RSER_TCF_RE_SHIFT)) & SPI_RSER_TCF_RE_MASK)

/*! @name PUSHR - PUSH TX FIFO Register In Master Mode */
#define SPI_PUSHR_TXDATA_MASK                    (0xFFFFU)
#define SPI_PUSHR_TXDATA_SHIFT                   (0U)
#define SPI_PUSHR_TXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_TXDATA_SHIFT)) & SPI_PUSHR_TXDATA_MASK)
#define SPI_PUSHR_PCS_MASK                       (0x3F0000U)
#define SPI_PUSHR_PCS_SHIFT                      (16U)
#define SPI_PUSHR_PCS(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_PCS_SHIFT)) & SPI_PUSHR_PCS_MASK)
#define SPI_PUSHR_CTCNT_MASK                     (0x4000000U)
#define SPI_PUSHR_CTCNT_SHIFT                    (26U)
#define SPI_PUSHR_CTCNT(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_CTCNT_SHIFT)) & SPI_PUSHR_CTCNT_MASK)
#define SPI_PUSHR_EOQ_MASK                       (0x8000000U)
#define SPI_PUSHR_EOQ_SHIFT                      (27U)
#define SPI_PUSHR_EOQ(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_EOQ_SHIFT)) & SPI_PUSHR_EOQ_MASK)
#define SPI_PUSHR_CTAS_MASK                      (0x70000000U)
#define SPI_PUSHR_CTAS_SHIFT                     (28U)
#define SPI_PUSHR_CTAS(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_CTAS_SHIFT)) & SPI_PUSHR_CTAS_MASK)
#define SPI_PUSHR_CONT_MASK                      (0x80000000U)
#define SPI_PUSHR_CONT_SHIFT                     (31U)
#define SPI_PUSHR_CONT(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_CONT_SHIFT)) & SPI_PUSHR_CONT_MASK)

/*! @name PUSHR_SLAVE - PUSH TX FIFO Register In Slave Mode */
#define SPI_PUSHR_SLAVE_TXDATA_MASK              (0xFFFFU)
#define SPI_PUSHR_SLAVE_TXDATA_SHIFT             (0U)
#define SPI_PUSHR_SLAVE_TXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_PUSHR_SLAVE_TXDATA_SHIFT)) & SPI_PUSHR_SLAVE_TXDATA_MASK)

/*! @name POPR - POP RX FIFO Register */
#define SPI_POPR_RXDATA_MASK                     (0xFFFFFFFFU)
#define SPI_POPR_RXDATA_SHIFT                    (0U)
#define SPI_POPR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_POPR_RXDATA_SHIFT)) & SPI_POPR_RXDATA_MASK)

/*! @name TXFR0 - Transmit FIFO Registers */
#define SPI_TXFR0_TXDATA_MASK                    (0xFFFFU)
#define SPI_TXFR0_TXDATA_SHIFT                   (0U)
#define SPI_TXFR0_TXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_TXFR0_TXDATA_SHIFT)) & SPI_TXFR0_TXDATA_MASK)
#define SPI_TXFR0_TXCMD_TXDATA_MASK              (0xFFFF0000U)
#define SPI_TXFR0_TXCMD_TXDATA_SHIFT             (16U)
#define SPI_TXFR0_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_TXFR0_TXCMD_TXDATA_SHIFT)) & SPI_TXFR0_TXCMD_TXDATA_MASK)

/*! @name TXFR1 - Transmit FIFO Registers */
#define SPI_TXFR1_TXDATA_MASK                    (0xFFFFU)
#define SPI_TXFR1_TXDATA_SHIFT                   (0U)
#define SPI_TXFR1_TXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_TXFR1_TXDATA_SHIFT)) & SPI_TXFR1_TXDATA_MASK)
#define SPI_TXFR1_TXCMD_TXDATA_MASK              (0xFFFF0000U)
#define SPI_TXFR1_TXCMD_TXDATA_SHIFT             (16U)
#define SPI_TXFR1_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_TXFR1_TXCMD_TXDATA_SHIFT)) & SPI_TXFR1_TXCMD_TXDATA_MASK)

/*! @name TXFR2 - Transmit FIFO Registers */
#define SPI_TXFR2_TXDATA_MASK                    (0xFFFFU)
#define SPI_TXFR2_TXDATA_SHIFT                   (0U)
#define SPI_TXFR2_TXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_TXFR2_TXDATA_SHIFT)) & SPI_TXFR2_TXDATA_MASK)
#define SPI_TXFR2_TXCMD_TXDATA_MASK              (0xFFFF0000U)
#define SPI_TXFR2_TXCMD_TXDATA_SHIFT             (16U)
#define SPI_TXFR2_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_TXFR2_TXCMD_TXDATA_SHIFT)) & SPI_TXFR2_TXCMD_TXDATA_MASK)

/*! @name TXFR3 - Transmit FIFO Registers */
#define SPI_TXFR3_TXDATA_MASK                    (0xFFFFU)
#define SPI_TXFR3_TXDATA_SHIFT                   (0U)
#define SPI_TXFR3_TXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_TXFR3_TXDATA_SHIFT)) & SPI_TXFR3_TXDATA_MASK)
#define SPI_TXFR3_TXCMD_TXDATA_MASK              (0xFFFF0000U)
#define SPI_TXFR3_TXCMD_TXDATA_SHIFT             (16U)
#define SPI_TXFR3_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_TXFR3_TXCMD_TXDATA_SHIFT)) & SPI_TXFR3_TXCMD_TXDATA_MASK)

/*! @name RXFR0 - Receive FIFO Registers */
#define SPI_RXFR0_RXDATA_MASK                    (0xFFFFFFFFU)
#define SPI_RXFR0_RXDATA_SHIFT                   (0U)
#define SPI_RXFR0_RXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RXFR0_RXDATA_SHIFT)) & SPI_RXFR0_RXDATA_MASK)

/*! @name RXFR1 - Receive FIFO Registers */
#define SPI_RXFR1_RXDATA_MASK                    (0xFFFFFFFFU)
#define SPI_RXFR1_RXDATA_SHIFT                   (0U)
#define SPI_RXFR1_RXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RXFR1_RXDATA_SHIFT)) & SPI_RXFR1_RXDATA_MASK)

/*! @name RXFR2 - Receive FIFO Registers */
#define SPI_RXFR2_RXDATA_MASK                    (0xFFFFFFFFU)
#define SPI_RXFR2_RXDATA_SHIFT                   (0U)
#define SPI_RXFR2_RXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RXFR2_RXDATA_SHIFT)) & SPI_RXFR2_RXDATA_MASK)

/*! @name RXFR3 - Receive FIFO Registers */
#define SPI_RXFR3_RXDATA_MASK                    (0xFFFFFFFFU)
#define SPI_RXFR3_RXDATA_SHIFT                   (0U)
#define SPI_RXFR3_RXDATA(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_RXFR3_RXDATA_SHIFT)) & SPI_RXFR3_RXDATA_MASK)


/*!
 * @}
 */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base address */
#define SPI0_BASE                                (0x4002C000u)
/** Peripheral SPI0 base pointer */
#define SPI0                                     ((SPI_Type *)SPI0_BASE)
/** Peripheral SPI1 base address */
#define SPI1_BASE                                (0x4002D000u)
/** Peripheral SPI1 base pointer */
#define SPI1                                     ((SPI_Type *)SPI1_BASE)
/** Peripheral SPI2 base address */
#define SPI2_BASE                                (0x400AC000u)
/** Peripheral SPI2 base pointer */
#define SPI2                                     ((SPI_Type *)SPI2_BASE)
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE, SPI2_BASE }
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0, SPI1, SPI2 }
/** Interrupt vectors for the SPI peripheral type */
#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn, SPI2_IRQn }

/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TPM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TPM_Peripheral_Access_Layer TPM Peripheral Access Layer
 * @{
 */

/** TPM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status and Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status and Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[2];
       uint8_t RESERVED_0[52];
  __IO uint32_t STATUS;                            /**< Capture and Compare Status, offset: 0x50 */
       uint8_t RESERVED_1[16];
  __IO uint32_t COMBINE;                           /**< Combine Channel Register, offset: 0x64 */
       uint8_t RESERVED_2[8];
  __IO uint32_t POL;                               /**< Channel Polarity, offset: 0x70 */
       uint8_t RESERVED_3[4];
  __IO uint32_t FILTER;                            /**< Filter Control, offset: 0x78 */
       uint8_t RESERVED_4[4];
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control and Status, offset: 0x80 */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
} TPM_Type;

/* ----------------------------------------------------------------------------
   -- TPM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TPM_Register_Masks TPM Register Masks
 * @{
 */

/*! @name SC - Status and Control */
#define TPM_SC_PS_MASK                           (0x7U)
#define TPM_SC_PS_SHIFT                          (0U)
#define TPM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x)) << TPM_SC_PS_SHIFT)) & TPM_SC_PS_MASK)
#define TPM_SC_CMOD_MASK                         (0x18U)
#define TPM_SC_CMOD_SHIFT                        (3U)
#define TPM_SC_CMOD(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_SC_CMOD_SHIFT)) & TPM_SC_CMOD_MASK)
#define TPM_SC_CPWMS_MASK                        (0x20U)
#define TPM_SC_CPWMS_SHIFT                       (5U)
#define TPM_SC_CPWMS(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_SC_CPWMS_SHIFT)) & TPM_SC_CPWMS_MASK)
#define TPM_SC_TOIE_MASK                         (0x40U)
#define TPM_SC_TOIE_SHIFT                        (6U)
#define TPM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_SC_TOIE_SHIFT)) & TPM_SC_TOIE_MASK)
#define TPM_SC_TOF_MASK                          (0x80U)
#define TPM_SC_TOF_SHIFT                         (7U)
#define TPM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << TPM_SC_TOF_SHIFT)) & TPM_SC_TOF_MASK)
#define TPM_SC_DMA_MASK                          (0x100U)
#define TPM_SC_DMA_SHIFT                         (8U)
#define TPM_SC_DMA(x)                            (((uint32_t)(((uint32_t)(x)) << TPM_SC_DMA_SHIFT)) & TPM_SC_DMA_MASK)

/*! @name CNT - Counter */
#define TPM_CNT_COUNT_MASK                       (0xFFFFU)
#define TPM_CNT_COUNT_SHIFT                      (0U)
#define TPM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CNT_COUNT_SHIFT)) & TPM_CNT_COUNT_MASK)

/*! @name MOD - Modulo */
#define TPM_MOD_MOD_MASK                         (0xFFFFU)
#define TPM_MOD_MOD_SHIFT                        (0U)
#define TPM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_MOD_MOD_SHIFT)) & TPM_MOD_MOD_MASK)

/*! @name CnSC - Channel (n) Status and Control */
#define TPM_CnSC_DMA_MASK                        (0x1U)
#define TPM_CnSC_DMA_SHIFT                       (0U)
#define TPM_CnSC_DMA(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_DMA_SHIFT)) & TPM_CnSC_DMA_MASK)
#define TPM_CnSC_ELSA_MASK                       (0x4U)
#define TPM_CnSC_ELSA_SHIFT                      (2U)
#define TPM_CnSC_ELSA(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_ELSA_SHIFT)) & TPM_CnSC_ELSA_MASK)
#define TPM_CnSC_ELSB_MASK                       (0x8U)
#define TPM_CnSC_ELSB_SHIFT                      (3U)
#define TPM_CnSC_ELSB(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_ELSB_SHIFT)) & TPM_CnSC_ELSB_MASK)
#define TPM_CnSC_MSA_MASK                        (0x10U)
#define TPM_CnSC_MSA_SHIFT                       (4U)
#define TPM_CnSC_MSA(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_MSA_SHIFT)) & TPM_CnSC_MSA_MASK)
#define TPM_CnSC_MSB_MASK                        (0x20U)
#define TPM_CnSC_MSB_SHIFT                       (5U)
#define TPM_CnSC_MSB(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_MSB_SHIFT)) & TPM_CnSC_MSB_MASK)
#define TPM_CnSC_CHIE_MASK                       (0x40U)
#define TPM_CnSC_CHIE_SHIFT                      (6U)
#define TPM_CnSC_CHIE(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_CHIE_SHIFT)) & TPM_CnSC_CHIE_MASK)
#define TPM_CnSC_CHF_MASK                        (0x80U)
#define TPM_CnSC_CHF_SHIFT                       (7U)
#define TPM_CnSC_CHF(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_CnSC_CHF_SHIFT)) & TPM_CnSC_CHF_MASK)

/* The count of TPM_CnSC */
#define TPM_CnSC_COUNT                           (2U)

/*! @name CnV - Channel (n) Value */
#define TPM_CnV_VAL_MASK                         (0xFFFFU)
#define TPM_CnV_VAL_SHIFT                        (0U)
#define TPM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_CnV_VAL_SHIFT)) & TPM_CnV_VAL_MASK)

/* The count of TPM_CnV */
#define TPM_CnV_COUNT                            (2U)

/*! @name STATUS - Capture and Compare Status */
#define TPM_STATUS_CH0F_MASK                     (0x1U)
#define TPM_STATUS_CH0F_SHIFT                    (0U)
#define TPM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH0F_SHIFT)) & TPM_STATUS_CH0F_MASK)
#define TPM_STATUS_CH1F_MASK                     (0x2U)
#define TPM_STATUS_CH1F_SHIFT                    (1U)
#define TPM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH1F_SHIFT)) & TPM_STATUS_CH1F_MASK)
#define TPM_STATUS_TOF_MASK                      (0x100U)
#define TPM_STATUS_TOF_SHIFT                     (8U)
#define TPM_STATUS_TOF(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_TOF_SHIFT)) & TPM_STATUS_TOF_MASK)

/*! @name COMBINE - Combine Channel Register */
#define TPM_COMBINE_COMBINE0_MASK                (0x1U)
#define TPM_COMBINE_COMBINE0_SHIFT               (0U)
#define TPM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMBINE0_SHIFT)) & TPM_COMBINE_COMBINE0_MASK)
#define TPM_COMBINE_COMSWAP0_MASK                (0x2U)
#define TPM_COMBINE_COMSWAP0_SHIFT               (1U)
#define TPM_COMBINE_COMSWAP0(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMSWAP0_SHIFT)) & TPM_COMBINE_COMSWAP0_MASK)

/*! @name POL - Channel Polarity */
#define TPM_POL_POL0_MASK                        (0x1U)
#define TPM_POL_POL0_SHIFT                       (0U)
#define TPM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL0_SHIFT)) & TPM_POL_POL0_MASK)
#define TPM_POL_POL1_MASK                        (0x2U)
#define TPM_POL_POL1_SHIFT                       (1U)
#define TPM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL1_SHIFT)) & TPM_POL_POL1_MASK)

/*! @name FILTER - Filter Control */
#define TPM_FILTER_CH0FVAL_MASK                  (0xFU)
#define TPM_FILTER_CH0FVAL_SHIFT                 (0U)
#define TPM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH0FVAL_SHIFT)) & TPM_FILTER_CH0FVAL_MASK)
#define TPM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define TPM_FILTER_CH1FVAL_SHIFT                 (4U)
#define TPM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH1FVAL_SHIFT)) & TPM_FILTER_CH1FVAL_MASK)

/*! @name QDCTRL - Quadrature Decoder Control and Status */
#define TPM_QDCTRL_QUADEN_MASK                   (0x1U)
#define TPM_QDCTRL_QUADEN_SHIFT                  (0U)
#define TPM_QDCTRL_QUADEN(x)                     (((uint32_t)(((uint32_t)(x)) << TPM_QDCTRL_QUADEN_SHIFT)) & TPM_QDCTRL_QUADEN_MASK)
#define TPM_QDCTRL_TOFDIR_MASK                   (0x2U)
#define TPM_QDCTRL_TOFDIR_SHIFT                  (1U)
#define TPM_QDCTRL_TOFDIR(x)                     (((uint32_t)(((uint32_t)(x)) << TPM_QDCTRL_TOFDIR_SHIFT)) & TPM_QDCTRL_TOFDIR_MASK)
#define TPM_QDCTRL_QUADIR_MASK                   (0x4U)
#define TPM_QDCTRL_QUADIR_SHIFT                  (2U)
#define TPM_QDCTRL_QUADIR(x)                     (((uint32_t)(((uint32_t)(x)) << TPM_QDCTRL_QUADIR_SHIFT)) & TPM_QDCTRL_QUADIR_MASK)
#define TPM_QDCTRL_QUADMODE_MASK                 (0x8U)
#define TPM_QDCTRL_QUADMODE_SHIFT                (3U)
#define TPM_QDCTRL_QUADMODE(x)                   (((uint32_t)(((uint32_t)(x)) << TPM_QDCTRL_QUADMODE_SHIFT)) & TPM_QDCTRL_QUADMODE_MASK)

/*! @name CONF - Configuration */
#define TPM_CONF_DOZEEN_MASK                     (0x20U)
#define TPM_CONF_DOZEEN_SHIFT                    (5U)
#define TPM_CONF_DOZEEN(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_DOZEEN_SHIFT)) & TPM_CONF_DOZEEN_MASK)
#define TPM_CONF_DBGMODE_MASK                    (0xC0U)
#define TPM_CONF_DBGMODE_SHIFT                   (6U)
#define TPM_CONF_DBGMODE(x)                      (((uint32_t)(((uint32_t)(x)) << TPM_CONF_DBGMODE_SHIFT)) & TPM_CONF_DBGMODE_MASK)
#define TPM_CONF_GTBSYNC_MASK                    (0x100U)
#define TPM_CONF_GTBSYNC_SHIFT                   (8U)
#define TPM_CONF_GTBSYNC(x)                      (((uint32_t)(((uint32_t)(x)) << TPM_CONF_GTBSYNC_SHIFT)) & TPM_CONF_GTBSYNC_MASK)
#define TPM_CONF_GTBEEN_MASK                     (0x200U)
#define TPM_CONF_GTBEEN_SHIFT                    (9U)
#define TPM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_GTBEEN_SHIFT)) & TPM_CONF_GTBEEN_MASK)
#define TPM_CONF_CSOT_MASK                       (0x10000U)
#define TPM_CONF_CSOT_SHIFT                      (16U)
#define TPM_CONF_CSOT(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CONF_CSOT_SHIFT)) & TPM_CONF_CSOT_MASK)
#define TPM_CONF_CSOO_MASK                       (0x20000U)
#define TPM_CONF_CSOO_SHIFT                      (17U)
#define TPM_CONF_CSOO(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CONF_CSOO_SHIFT)) & TPM_CONF_CSOO_MASK)
#define TPM_CONF_CROT_MASK                       (0x40000U)
#define TPM_CONF_CROT_SHIFT                      (18U)
#define TPM_CONF_CROT(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CONF_CROT_SHIFT)) & TPM_CONF_CROT_MASK)
#define TPM_CONF_CPOT_MASK                       (0x80000U)
#define TPM_CONF_CPOT_SHIFT                      (19U)
#define TPM_CONF_CPOT(x)                         (((uint32_t)(((uint32_t)(x)) << TPM_CONF_CPOT_SHIFT)) & TPM_CONF_CPOT_MASK)
#define TPM_CONF_TRGPOL_MASK                     (0x400000U)
#define TPM_CONF_TRGPOL_SHIFT                    (22U)
#define TPM_CONF_TRGPOL(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_TRGPOL_SHIFT)) & TPM_CONF_TRGPOL_MASK)
#define TPM_CONF_TRGSRC_MASK                     (0x800000U)
#define TPM_CONF_TRGSRC_SHIFT                    (23U)
#define TPM_CONF_TRGSRC(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_TRGSRC_SHIFT)) & TPM_CONF_TRGSRC_MASK)
#define TPM_CONF_TRGSEL_MASK                     (0xF000000U)
#define TPM_CONF_TRGSEL_SHIFT                    (24U)
#define TPM_CONF_TRGSEL(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_TRGSEL_SHIFT)) & TPM_CONF_TRGSEL_MASK)


/*!
 * @}
 */ /* end of group TPM_Register_Masks */


/* TPM - Peripheral instance base addresses */
/** Peripheral TPM1 base address */
#define TPM1_BASE                                (0x400C9000u)
/** Peripheral TPM1 base pointer */
#define TPM1                                     ((TPM_Type *)TPM1_BASE)
/** Peripheral TPM2 base address */
#define TPM2_BASE                                (0x400CA000u)
/** Peripheral TPM2 base pointer */
#define TPM2                                     ((TPM_Type *)TPM2_BASE)
/** Array initializer of TPM peripheral base addresses */
#define TPM_BASE_ADDRS                           { 0u, TPM1_BASE, TPM2_BASE }
/** Array initializer of TPM peripheral base pointers */
#define TPM_BASE_PTRS                            { (TPM_Type *)0u, TPM1, TPM2 }
/** Interrupt vectors for the TPM peripheral type */
#define TPM_IRQS                                 { NotAvail_IRQn, TPM1_IRQn, TPM2_IRQn }

/*!
 * @}
 */ /* end of group TPM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TRNG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRNG_Peripheral_Access_Layer TRNG Peripheral Access Layer
 * @{
 */

/** TRNG - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCTL;                              /**< RNG Miscellaneous Control Register, offset: 0x0 */
  __IO uint32_t SCMISC;                            /**< RNG Statistical Check Miscellaneous Register, offset: 0x4 */
  __IO uint32_t PKRRNG;                            /**< RNG Poker Range Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t PKRMAX;                            /**< RNG Poker Maximum Limit Register, offset: 0xC */
    __I  uint32_t PKRSQ;                             /**< RNG Poker Square Calculation Result Register, offset: 0xC */
  };
  __IO uint32_t SDCTL;                             /**< RNG Seed Control Register, offset: 0x10 */
  union {                                          /* offset: 0x14 */
    __IO uint32_t SBLIM;                             /**< RNG Sparse Bit Limit Register, offset: 0x14 */
    __I  uint32_t TOTSAM;                            /**< RNG Total Samples Register, offset: 0x14 */
  };
  __IO uint32_t FRQMIN;                            /**< RNG Frequency Count Minimum Limit Register, offset: 0x18 */
  union {                                          /* offset: 0x1C */
    __I  uint32_t FRQCNT;                            /**< RNG Frequency Count Register, offset: 0x1C */
    __IO uint32_t FRQMAX;                            /**< RNG Frequency Count Maximum Limit Register, offset: 0x1C */
  };
  union {                                          /* offset: 0x20 */
    __I  uint32_t SCMC;                              /**< RNG Statistical Check Monobit Count Register, offset: 0x20 */
    __IO uint32_t SCML;                              /**< RNG Statistical Check Monobit Limit Register, offset: 0x20 */
  };
  union {                                          /* offset: 0x24 */
    __I  uint32_t SCR1C;                             /**< RNG Statistical Check Run Length 1 Count Register, offset: 0x24 */
    __IO uint32_t SCR1L;                             /**< RNG Statistical Check Run Length 1 Limit Register, offset: 0x24 */
  };
  union {                                          /* offset: 0x28 */
    __I  uint32_t SCR2C;                             /**< RNG Statistical Check Run Length 2 Count Register, offset: 0x28 */
    __IO uint32_t SCR2L;                             /**< RNG Statistical Check Run Length 2 Limit Register, offset: 0x28 */
  };
  union {                                          /* offset: 0x2C */
    __I  uint32_t SCR3C;                             /**< RNG Statistical Check Run Length 3 Count Register, offset: 0x2C */
    __IO uint32_t SCR3L;                             /**< RNG Statistical Check Run Length 3 Limit Register, offset: 0x2C */
  };
  union {                                          /* offset: 0x30 */
    __I  uint32_t SCR4C;                             /**< RNG Statistical Check Run Length 4 Count Register, offset: 0x30 */
    __IO uint32_t SCR4L;                             /**< RNG Statistical Check Run Length 4 Limit Register, offset: 0x30 */
  };
  union {                                          /* offset: 0x34 */
    __I  uint32_t SCR5C;                             /**< RNG Statistical Check Run Length 5 Count Register, offset: 0x34 */
    __IO uint32_t SCR5L;                             /**< RNG Statistical Check Run Length 5 Limit Register, offset: 0x34 */
  };
  union {                                          /* offset: 0x38 */
    __I  uint32_t SCR6PC;                            /**< RNG Statistical Check Run Length 6+ Count Register, offset: 0x38 */
    __IO uint32_t SCR6PL;                            /**< RNG Statistical Check Run Length 6+ Limit Register, offset: 0x38 */
  };
  __I  uint32_t STATUS;                            /**< RNG Status Register, offset: 0x3C */
  __I  uint32_t ENT[16];                           /**< RNG TRNG Entropy Read Register, array offset: 0x40, array step: 0x4 */
  __I  uint32_t PKRCNT10;                          /**< RNG Statistical Check Poker Count 1 and 0 Register, offset: 0x80 */
  __I  uint32_t PKRCNT32;                          /**< RNG Statistical Check Poker Count 3 and 2 Register, offset: 0x84 */
  __I  uint32_t PKRCNT54;                          /**< RNG Statistical Check Poker Count 5 and 4 Register, offset: 0x88 */
  __I  uint32_t PKRCNT76;                          /**< RNG Statistical Check Poker Count 7 and 6 Register, offset: 0x8C */
  __I  uint32_t PKRCNT98;                          /**< RNG Statistical Check Poker Count 9 and 8 Register, offset: 0x90 */
  __I  uint32_t PKRCNTBA;                          /**< RNG Statistical Check Poker Count B and A Register, offset: 0x94 */
  __I  uint32_t PKRCNTDC;                          /**< RNG Statistical Check Poker Count D and C Register, offset: 0x98 */
  __I  uint32_t PKRCNTFE;                          /**< RNG Statistical Check Poker Count F and E Register, offset: 0x9C */
       uint8_t RESERVED_0[16];
  __IO uint32_t SEC_CFG;                           /**< RNG Security Configuration Register, offset: 0xB0 */
  __IO uint32_t INT_CTRL;                          /**< RNG Interrupt Control Register, offset: 0xB4 */
  __IO uint32_t INT_MASK;                          /**< RNG Mask Register, offset: 0xB8 */
  __IO uint32_t INT_STATUS;                        /**< RNG Interrupt Status Register, offset: 0xBC */
       uint8_t RESERVED_1[48];
  __I  uint32_t VID1;                              /**< RNG Version ID Register (MS), offset: 0xF0 */
  __I  uint32_t VID2;                              /**< RNG Version ID Register (LS), offset: 0xF4 */
} TRNG_Type;

/* ----------------------------------------------------------------------------
   -- TRNG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRNG_Register_Masks TRNG Register Masks
 * @{
 */

/*! @name MCTL - RNG Miscellaneous Control Register */
#define TRNG_MCTL_SAMP_MODE_MASK                 (0x3U)
#define TRNG_MCTL_SAMP_MODE_SHIFT                (0U)
#define TRNG_MCTL_SAMP_MODE(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_SAMP_MODE_SHIFT)) & TRNG_MCTL_SAMP_MODE_MASK)
#define TRNG_MCTL_OSC_DIV_MASK                   (0xCU)
#define TRNG_MCTL_OSC_DIV_SHIFT                  (2U)
#define TRNG_MCTL_OSC_DIV(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_OSC_DIV_SHIFT)) & TRNG_MCTL_OSC_DIV_MASK)
#define TRNG_MCTL_UNUSED_MASK                    (0x10U)
#define TRNG_MCTL_UNUSED_SHIFT                   (4U)
#define TRNG_MCTL_UNUSED(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_UNUSED_SHIFT)) & TRNG_MCTL_UNUSED_MASK)
#define TRNG_MCTL_TRNG_ACC_MASK                  (0x20U)
#define TRNG_MCTL_TRNG_ACC_SHIFT                 (5U)
#define TRNG_MCTL_TRNG_ACC(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_TRNG_ACC_SHIFT)) & TRNG_MCTL_TRNG_ACC_MASK)
#define TRNG_MCTL_RST_DEF_MASK                   (0x40U)
#define TRNG_MCTL_RST_DEF_SHIFT                  (6U)
#define TRNG_MCTL_RST_DEF(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_RST_DEF_SHIFT)) & TRNG_MCTL_RST_DEF_MASK)
#define TRNG_MCTL_FOR_SCLK_MASK                  (0x80U)
#define TRNG_MCTL_FOR_SCLK_SHIFT                 (7U)
#define TRNG_MCTL_FOR_SCLK(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_FOR_SCLK_SHIFT)) & TRNG_MCTL_FOR_SCLK_MASK)
#define TRNG_MCTL_FCT_FAIL_MASK                  (0x100U)
#define TRNG_MCTL_FCT_FAIL_SHIFT                 (8U)
#define TRNG_MCTL_FCT_FAIL(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_FCT_FAIL_SHIFT)) & TRNG_MCTL_FCT_FAIL_MASK)
#define TRNG_MCTL_FCT_VAL_MASK                   (0x200U)
#define TRNG_MCTL_FCT_VAL_SHIFT                  (9U)
#define TRNG_MCTL_FCT_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_FCT_VAL_SHIFT)) & TRNG_MCTL_FCT_VAL_MASK)
#define TRNG_MCTL_ENT_VAL_MASK                   (0x400U)
#define TRNG_MCTL_ENT_VAL_SHIFT                  (10U)
#define TRNG_MCTL_ENT_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_ENT_VAL_SHIFT)) & TRNG_MCTL_ENT_VAL_MASK)
#define TRNG_MCTL_TST_OUT_MASK                   (0x800U)
#define TRNG_MCTL_TST_OUT_SHIFT                  (11U)
#define TRNG_MCTL_TST_OUT(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_TST_OUT_SHIFT)) & TRNG_MCTL_TST_OUT_MASK)
#define TRNG_MCTL_ERR_MASK                       (0x1000U)
#define TRNG_MCTL_ERR_SHIFT                      (12U)
#define TRNG_MCTL_ERR(x)                         (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_ERR_SHIFT)) & TRNG_MCTL_ERR_MASK)
#define TRNG_MCTL_TSTOP_OK_MASK                  (0x2000U)
#define TRNG_MCTL_TSTOP_OK_SHIFT                 (13U)
#define TRNG_MCTL_TSTOP_OK(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_TSTOP_OK_SHIFT)) & TRNG_MCTL_TSTOP_OK_MASK)
#define TRNG_MCTL_PRGM_MASK                      (0x10000U)
#define TRNG_MCTL_PRGM_SHIFT                     (16U)
#define TRNG_MCTL_PRGM(x)                        (((uint32_t)(((uint32_t)(x)) << TRNG_MCTL_PRGM_SHIFT)) & TRNG_MCTL_PRGM_MASK)

/*! @name SCMISC - RNG Statistical Check Miscellaneous Register */
#define TRNG_SCMISC_LRUN_MAX_MASK                (0xFFU)
#define TRNG_SCMISC_LRUN_MAX_SHIFT               (0U)
#define TRNG_SCMISC_LRUN_MAX(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCMISC_LRUN_MAX_SHIFT)) & TRNG_SCMISC_LRUN_MAX_MASK)
#define TRNG_SCMISC_RTY_CT_MASK                  (0xF0000U)
#define TRNG_SCMISC_RTY_CT_SHIFT                 (16U)
#define TRNG_SCMISC_RTY_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCMISC_RTY_CT_SHIFT)) & TRNG_SCMISC_RTY_CT_MASK)

/*! @name PKRRNG - RNG Poker Range Register */
#define TRNG_PKRRNG_PKR_RNG_MASK                 (0xFFFFU)
#define TRNG_PKRRNG_PKR_RNG_SHIFT                (0U)
#define TRNG_PKRRNG_PKR_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_PKRRNG_PKR_RNG_SHIFT)) & TRNG_PKRRNG_PKR_RNG_MASK)

/*! @name PKRMAX - RNG Poker Maximum Limit Register */
#define TRNG_PKRMAX_PKR_MAX_MASK                 (0xFFFFFFU)
#define TRNG_PKRMAX_PKR_MAX_SHIFT                (0U)
#define TRNG_PKRMAX_PKR_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_PKRMAX_PKR_MAX_SHIFT)) & TRNG_PKRMAX_PKR_MAX_MASK)

/*! @name PKRSQ - RNG Poker Square Calculation Result Register */
#define TRNG_PKRSQ_PKR_SQ_MASK                   (0xFFFFFFU)
#define TRNG_PKRSQ_PKR_SQ_SHIFT                  (0U)
#define TRNG_PKRSQ_PKR_SQ(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_PKRSQ_PKR_SQ_SHIFT)) & TRNG_PKRSQ_PKR_SQ_MASK)

/*! @name SDCTL - RNG Seed Control Register */
#define TRNG_SDCTL_SAMP_SIZE_MASK                (0xFFFFU)
#define TRNG_SDCTL_SAMP_SIZE_SHIFT               (0U)
#define TRNG_SDCTL_SAMP_SIZE(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SDCTL_SAMP_SIZE_SHIFT)) & TRNG_SDCTL_SAMP_SIZE_MASK)
#define TRNG_SDCTL_ENT_DLY_MASK                  (0xFFFF0000U)
#define TRNG_SDCTL_ENT_DLY_SHIFT                 (16U)
#define TRNG_SDCTL_ENT_DLY(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SDCTL_ENT_DLY_SHIFT)) & TRNG_SDCTL_ENT_DLY_MASK)

/*! @name SBLIM - RNG Sparse Bit Limit Register */
#define TRNG_SBLIM_SB_LIM_MASK                   (0x3FFU)
#define TRNG_SBLIM_SB_LIM_SHIFT                  (0U)
#define TRNG_SBLIM_SB_LIM(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_SBLIM_SB_LIM_SHIFT)) & TRNG_SBLIM_SB_LIM_MASK)

/*! @name TOTSAM - RNG Total Samples Register */
#define TRNG_TOTSAM_TOT_SAM_MASK                 (0xFFFFFU)
#define TRNG_TOTSAM_TOT_SAM_SHIFT                (0U)
#define TRNG_TOTSAM_TOT_SAM(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_TOTSAM_TOT_SAM_SHIFT)) & TRNG_TOTSAM_TOT_SAM_MASK)

/*! @name FRQMIN - RNG Frequency Count Minimum Limit Register */
#define TRNG_FRQMIN_FRQ_MIN_MASK                 (0x3FFFFFU)
#define TRNG_FRQMIN_FRQ_MIN_SHIFT                (0U)
#define TRNG_FRQMIN_FRQ_MIN(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_FRQMIN_FRQ_MIN_SHIFT)) & TRNG_FRQMIN_FRQ_MIN_MASK)

/*! @name FRQCNT - RNG Frequency Count Register */
#define TRNG_FRQCNT_FRQ_CT_MASK                  (0x3FFFFFU)
#define TRNG_FRQCNT_FRQ_CT_SHIFT                 (0U)
#define TRNG_FRQCNT_FRQ_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_FRQCNT_FRQ_CT_SHIFT)) & TRNG_FRQCNT_FRQ_CT_MASK)

/*! @name FRQMAX - RNG Frequency Count Maximum Limit Register */
#define TRNG_FRQMAX_FRQ_MAX_MASK                 (0x3FFFFFU)
#define TRNG_FRQMAX_FRQ_MAX_SHIFT                (0U)
#define TRNG_FRQMAX_FRQ_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_FRQMAX_FRQ_MAX_SHIFT)) & TRNG_FRQMAX_FRQ_MAX_MASK)

/*! @name SCMC - RNG Statistical Check Monobit Count Register */
#define TRNG_SCMC_MONO_CT_MASK                   (0xFFFFU)
#define TRNG_SCMC_MONO_CT_SHIFT                  (0U)
#define TRNG_SCMC_MONO_CT(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_SCMC_MONO_CT_SHIFT)) & TRNG_SCMC_MONO_CT_MASK)

/*! @name SCML - RNG Statistical Check Monobit Limit Register */
#define TRNG_SCML_MONO_MAX_MASK                  (0xFFFFU)
#define TRNG_SCML_MONO_MAX_SHIFT                 (0U)
#define TRNG_SCML_MONO_MAX(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCML_MONO_MAX_SHIFT)) & TRNG_SCML_MONO_MAX_MASK)
#define TRNG_SCML_MONO_RNG_MASK                  (0xFFFF0000U)
#define TRNG_SCML_MONO_RNG_SHIFT                 (16U)
#define TRNG_SCML_MONO_RNG(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCML_MONO_RNG_SHIFT)) & TRNG_SCML_MONO_RNG_MASK)

/*! @name SCR1C - RNG Statistical Check Run Length 1 Count Register */
#define TRNG_SCR1C_R1_0_CT_MASK                  (0x7FFFU)
#define TRNG_SCR1C_R1_0_CT_SHIFT                 (0U)
#define TRNG_SCR1C_R1_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1C_R1_0_CT_SHIFT)) & TRNG_SCR1C_R1_0_CT_MASK)
#define TRNG_SCR1C_R1_1_CT_MASK                  (0x7FFF0000U)
#define TRNG_SCR1C_R1_1_CT_SHIFT                 (16U)
#define TRNG_SCR1C_R1_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1C_R1_1_CT_SHIFT)) & TRNG_SCR1C_R1_1_CT_MASK)

/*! @name SCR1L - RNG Statistical Check Run Length 1 Limit Register */
#define TRNG_SCR1L_RUN1_MAX_MASK                 (0x7FFFU)
#define TRNG_SCR1L_RUN1_MAX_SHIFT                (0U)
#define TRNG_SCR1L_RUN1_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1L_RUN1_MAX_SHIFT)) & TRNG_SCR1L_RUN1_MAX_MASK)
#define TRNG_SCR1L_RUN1_RNG_MASK                 (0x7FFF0000U)
#define TRNG_SCR1L_RUN1_RNG_SHIFT                (16U)
#define TRNG_SCR1L_RUN1_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1L_RUN1_RNG_SHIFT)) & TRNG_SCR1L_RUN1_RNG_MASK)

/*! @name SCR2C - RNG Statistical Check Run Length 2 Count Register */
#define TRNG_SCR2C_R2_0_CT_MASK                  (0x3FFFU)
#define TRNG_SCR2C_R2_0_CT_SHIFT                 (0U)
#define TRNG_SCR2C_R2_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2C_R2_0_CT_SHIFT)) & TRNG_SCR2C_R2_0_CT_MASK)
#define TRNG_SCR2C_R2_1_CT_MASK                  (0x3FFF0000U)
#define TRNG_SCR2C_R2_1_CT_SHIFT                 (16U)
#define TRNG_SCR2C_R2_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2C_R2_1_CT_SHIFT)) & TRNG_SCR2C_R2_1_CT_MASK)

/*! @name SCR2L - RNG Statistical Check Run Length 2 Limit Register */
#define TRNG_SCR2L_RUN2_MAX_MASK                 (0x3FFFU)
#define TRNG_SCR2L_RUN2_MAX_SHIFT                (0U)
#define TRNG_SCR2L_RUN2_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2L_RUN2_MAX_SHIFT)) & TRNG_SCR2L_RUN2_MAX_MASK)
#define TRNG_SCR2L_RUN2_RNG_MASK                 (0x3FFF0000U)
#define TRNG_SCR2L_RUN2_RNG_SHIFT                (16U)
#define TRNG_SCR2L_RUN2_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2L_RUN2_RNG_SHIFT)) & TRNG_SCR2L_RUN2_RNG_MASK)

/*! @name SCR3C - RNG Statistical Check Run Length 3 Count Register */
#define TRNG_SCR3C_R3_0_CT_MASK                  (0x1FFFU)
#define TRNG_SCR3C_R3_0_CT_SHIFT                 (0U)
#define TRNG_SCR3C_R3_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3C_R3_0_CT_SHIFT)) & TRNG_SCR3C_R3_0_CT_MASK)
#define TRNG_SCR3C_R3_1_CT_MASK                  (0x1FFF0000U)
#define TRNG_SCR3C_R3_1_CT_SHIFT                 (16U)
#define TRNG_SCR3C_R3_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3C_R3_1_CT_SHIFT)) & TRNG_SCR3C_R3_1_CT_MASK)

/*! @name SCR3L - RNG Statistical Check Run Length 3 Limit Register */
#define TRNG_SCR3L_RUN3_MAX_MASK                 (0x1FFFU)
#define TRNG_SCR3L_RUN3_MAX_SHIFT                (0U)
#define TRNG_SCR3L_RUN3_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3L_RUN3_MAX_SHIFT)) & TRNG_SCR3L_RUN3_MAX_MASK)
#define TRNG_SCR3L_RUN3_RNG_MASK                 (0x1FFF0000U)
#define TRNG_SCR3L_RUN3_RNG_SHIFT                (16U)
#define TRNG_SCR3L_RUN3_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3L_RUN3_RNG_SHIFT)) & TRNG_SCR3L_RUN3_RNG_MASK)

/*! @name SCR4C - RNG Statistical Check Run Length 4 Count Register */
#define TRNG_SCR4C_R4_0_CT_MASK                  (0xFFFU)
#define TRNG_SCR4C_R4_0_CT_SHIFT                 (0U)
#define TRNG_SCR4C_R4_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4C_R4_0_CT_SHIFT)) & TRNG_SCR4C_R4_0_CT_MASK)
#define TRNG_SCR4C_R4_1_CT_MASK                  (0xFFF0000U)
#define TRNG_SCR4C_R4_1_CT_SHIFT                 (16U)
#define TRNG_SCR4C_R4_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4C_R4_1_CT_SHIFT)) & TRNG_SCR4C_R4_1_CT_MASK)

/*! @name SCR4L - RNG Statistical Check Run Length 4 Limit Register */
#define TRNG_SCR4L_RUN4_MAX_MASK                 (0xFFFU)
#define TRNG_SCR4L_RUN4_MAX_SHIFT                (0U)
#define TRNG_SCR4L_RUN4_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4L_RUN4_MAX_SHIFT)) & TRNG_SCR4L_RUN4_MAX_MASK)
#define TRNG_SCR4L_RUN4_RNG_MASK                 (0xFFF0000U)
#define TRNG_SCR4L_RUN4_RNG_SHIFT                (16U)
#define TRNG_SCR4L_RUN4_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4L_RUN4_RNG_SHIFT)) & TRNG_SCR4L_RUN4_RNG_MASK)

/*! @name SCR5C - RNG Statistical Check Run Length 5 Count Register */
#define TRNG_SCR5C_R5_0_CT_MASK                  (0x7FFU)
#define TRNG_SCR5C_R5_0_CT_SHIFT                 (0U)
#define TRNG_SCR5C_R5_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5C_R5_0_CT_SHIFT)) & TRNG_SCR5C_R5_0_CT_MASK)
#define TRNG_SCR5C_R5_1_CT_MASK                  (0x7FF0000U)
#define TRNG_SCR5C_R5_1_CT_SHIFT                 (16U)
#define TRNG_SCR5C_R5_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5C_R5_1_CT_SHIFT)) & TRNG_SCR5C_R5_1_CT_MASK)

/*! @name SCR5L - RNG Statistical Check Run Length 5 Limit Register */
#define TRNG_SCR5L_RUN5_MAX_MASK                 (0x7FFU)
#define TRNG_SCR5L_RUN5_MAX_SHIFT                (0U)
#define TRNG_SCR5L_RUN5_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5L_RUN5_MAX_SHIFT)) & TRNG_SCR5L_RUN5_MAX_MASK)
#define TRNG_SCR5L_RUN5_RNG_MASK                 (0x7FF0000U)
#define TRNG_SCR5L_RUN5_RNG_SHIFT                (16U)
#define TRNG_SCR5L_RUN5_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5L_RUN5_RNG_SHIFT)) & TRNG_SCR5L_RUN5_RNG_MASK)

/*! @name SCR6PC - RNG Statistical Check Run Length 6+ Count Register */
#define TRNG_SCR6PC_R6P_0_CT_MASK                (0x7FFU)
#define TRNG_SCR6PC_R6P_0_CT_SHIFT               (0U)
#define TRNG_SCR6PC_R6P_0_CT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PC_R6P_0_CT_SHIFT)) & TRNG_SCR6PC_R6P_0_CT_MASK)
#define TRNG_SCR6PC_R6P_1_CT_MASK                (0x7FF0000U)
#define TRNG_SCR6PC_R6P_1_CT_SHIFT               (16U)
#define TRNG_SCR6PC_R6P_1_CT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PC_R6P_1_CT_SHIFT)) & TRNG_SCR6PC_R6P_1_CT_MASK)

/*! @name SCR6PL - RNG Statistical Check Run Length 6+ Limit Register */
#define TRNG_SCR6PL_RUN6P_MAX_MASK               (0x7FFU)
#define TRNG_SCR6PL_RUN6P_MAX_SHIFT              (0U)
#define TRNG_SCR6PL_RUN6P_MAX(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PL_RUN6P_MAX_SHIFT)) & TRNG_SCR6PL_RUN6P_MAX_MASK)
#define TRNG_SCR6PL_RUN6P_RNG_MASK               (0x7FF0000U)
#define TRNG_SCR6PL_RUN6P_RNG_SHIFT              (16U)
#define TRNG_SCR6PL_RUN6P_RNG(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PL_RUN6P_RNG_SHIFT)) & TRNG_SCR6PL_RUN6P_RNG_MASK)

/*! @name STATUS - RNG Status Register */
#define TRNG_STATUS_TF1BR0_MASK                  (0x1U)
#define TRNG_STATUS_TF1BR0_SHIFT                 (0U)
#define TRNG_STATUS_TF1BR0(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF1BR0_SHIFT)) & TRNG_STATUS_TF1BR0_MASK)
#define TRNG_STATUS_TF1BR1_MASK                  (0x2U)
#define TRNG_STATUS_TF1BR1_SHIFT                 (1U)
#define TRNG_STATUS_TF1BR1(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF1BR1_SHIFT)) & TRNG_STATUS_TF1BR1_MASK)
#define TRNG_STATUS_TF2BR0_MASK                  (0x4U)
#define TRNG_STATUS_TF2BR0_SHIFT                 (2U)
#define TRNG_STATUS_TF2BR0(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF2BR0_SHIFT)) & TRNG_STATUS_TF2BR0_MASK)
#define TRNG_STATUS_TF2BR1_MASK                  (0x8U)
#define TRNG_STATUS_TF2BR1_SHIFT                 (3U)
#define TRNG_STATUS_TF2BR1(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF2BR1_SHIFT)) & TRNG_STATUS_TF2BR1_MASK)
#define TRNG_STATUS_TF3BR0_MASK                  (0x10U)
#define TRNG_STATUS_TF3BR0_SHIFT                 (4U)
#define TRNG_STATUS_TF3BR0(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF3BR0_SHIFT)) & TRNG_STATUS_TF3BR0_MASK)
#define TRNG_STATUS_TF3BR1_MASK                  (0x20U)
#define TRNG_STATUS_TF3BR1_SHIFT                 (5U)
#define TRNG_STATUS_TF3BR1(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF3BR1_SHIFT)) & TRNG_STATUS_TF3BR1_MASK)
#define TRNG_STATUS_TF4BR0_MASK                  (0x40U)
#define TRNG_STATUS_TF4BR0_SHIFT                 (6U)
#define TRNG_STATUS_TF4BR0(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF4BR0_SHIFT)) & TRNG_STATUS_TF4BR0_MASK)
#define TRNG_STATUS_TF4BR1_MASK                  (0x80U)
#define TRNG_STATUS_TF4BR1_SHIFT                 (7U)
#define TRNG_STATUS_TF4BR1(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF4BR1_SHIFT)) & TRNG_STATUS_TF4BR1_MASK)
#define TRNG_STATUS_TF5BR0_MASK                  (0x100U)
#define TRNG_STATUS_TF5BR0_SHIFT                 (8U)
#define TRNG_STATUS_TF5BR0(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF5BR0_SHIFT)) & TRNG_STATUS_TF5BR0_MASK)
#define TRNG_STATUS_TF5BR1_MASK                  (0x200U)
#define TRNG_STATUS_TF5BR1_SHIFT                 (9U)
#define TRNG_STATUS_TF5BR1(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF5BR1_SHIFT)) & TRNG_STATUS_TF5BR1_MASK)
#define TRNG_STATUS_TF6PBR0_MASK                 (0x400U)
#define TRNG_STATUS_TF6PBR0_SHIFT                (10U)
#define TRNG_STATUS_TF6PBR0(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF6PBR0_SHIFT)) & TRNG_STATUS_TF6PBR0_MASK)
#define TRNG_STATUS_TF6PBR1_MASK                 (0x800U)
#define TRNG_STATUS_TF6PBR1_SHIFT                (11U)
#define TRNG_STATUS_TF6PBR1(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TF6PBR1_SHIFT)) & TRNG_STATUS_TF6PBR1_MASK)
#define TRNG_STATUS_TFSB_MASK                    (0x1000U)
#define TRNG_STATUS_TFSB_SHIFT                   (12U)
#define TRNG_STATUS_TFSB(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TFSB_SHIFT)) & TRNG_STATUS_TFSB_MASK)
#define TRNG_STATUS_TFLR_MASK                    (0x2000U)
#define TRNG_STATUS_TFLR_SHIFT                   (13U)
#define TRNG_STATUS_TFLR(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TFLR_SHIFT)) & TRNG_STATUS_TFLR_MASK)
#define TRNG_STATUS_TFP_MASK                     (0x4000U)
#define TRNG_STATUS_TFP_SHIFT                    (14U)
#define TRNG_STATUS_TFP(x)                       (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TFP_SHIFT)) & TRNG_STATUS_TFP_MASK)
#define TRNG_STATUS_TFMB_MASK                    (0x8000U)
#define TRNG_STATUS_TFMB_SHIFT                   (15U)
#define TRNG_STATUS_TFMB(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_TFMB_SHIFT)) & TRNG_STATUS_TFMB_MASK)
#define TRNG_STATUS_RETRY_CT_MASK                (0xF0000U)
#define TRNG_STATUS_RETRY_CT_SHIFT               (16U)
#define TRNG_STATUS_RETRY_CT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_STATUS_RETRY_CT_SHIFT)) & TRNG_STATUS_RETRY_CT_MASK)

/*! @name ENT - RNG TRNG Entropy Read Register */
#define TRNG_ENT_ENT_MASK                        (0xFFFFFFFFU)
#define TRNG_ENT_ENT_SHIFT                       (0U)
#define TRNG_ENT_ENT(x)                          (((uint32_t)(((uint32_t)(x)) << TRNG_ENT_ENT_SHIFT)) & TRNG_ENT_ENT_MASK)

/* The count of TRNG_ENT */
#define TRNG_ENT_COUNT                           (16U)

/*! @name PKRCNT10 - RNG Statistical Check Poker Count 1 and 0 Register */
#define TRNG_PKRCNT10_PKR_0_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT10_PKR_0_CT_SHIFT             (0U)
#define TRNG_PKRCNT10_PKR_0_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT10_PKR_0_CT_SHIFT)) & TRNG_PKRCNT10_PKR_0_CT_MASK)
#define TRNG_PKRCNT10_PKR_1_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT10_PKR_1_CT_SHIFT             (16U)
#define TRNG_PKRCNT10_PKR_1_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT10_PKR_1_CT_SHIFT)) & TRNG_PKRCNT10_PKR_1_CT_MASK)

/*! @name PKRCNT32 - RNG Statistical Check Poker Count 3 and 2 Register */
#define TRNG_PKRCNT32_PKR_2_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT32_PKR_2_CT_SHIFT             (0U)
#define TRNG_PKRCNT32_PKR_2_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT32_PKR_2_CT_SHIFT)) & TRNG_PKRCNT32_PKR_2_CT_MASK)
#define TRNG_PKRCNT32_PKR_3_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT32_PKR_3_CT_SHIFT             (16U)
#define TRNG_PKRCNT32_PKR_3_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT32_PKR_3_CT_SHIFT)) & TRNG_PKRCNT32_PKR_3_CT_MASK)

/*! @name PKRCNT54 - RNG Statistical Check Poker Count 5 and 4 Register */
#define TRNG_PKRCNT54_PKR_4_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT54_PKR_4_CT_SHIFT             (0U)
#define TRNG_PKRCNT54_PKR_4_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT54_PKR_4_CT_SHIFT)) & TRNG_PKRCNT54_PKR_4_CT_MASK)
#define TRNG_PKRCNT54_PKR_5_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT54_PKR_5_CT_SHIFT             (16U)
#define TRNG_PKRCNT54_PKR_5_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT54_PKR_5_CT_SHIFT)) & TRNG_PKRCNT54_PKR_5_CT_MASK)

/*! @name PKRCNT76 - RNG Statistical Check Poker Count 7 and 6 Register */
#define TRNG_PKRCNT76_PKR_6_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT76_PKR_6_CT_SHIFT             (0U)
#define TRNG_PKRCNT76_PKR_6_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT76_PKR_6_CT_SHIFT)) & TRNG_PKRCNT76_PKR_6_CT_MASK)
#define TRNG_PKRCNT76_PKR_7_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT76_PKR_7_CT_SHIFT             (16U)
#define TRNG_PKRCNT76_PKR_7_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT76_PKR_7_CT_SHIFT)) & TRNG_PKRCNT76_PKR_7_CT_MASK)

/*! @name PKRCNT98 - RNG Statistical Check Poker Count 9 and 8 Register */
#define TRNG_PKRCNT98_PKR_8_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT98_PKR_8_CT_SHIFT             (0U)
#define TRNG_PKRCNT98_PKR_8_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT98_PKR_8_CT_SHIFT)) & TRNG_PKRCNT98_PKR_8_CT_MASK)
#define TRNG_PKRCNT98_PKR_9_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT98_PKR_9_CT_SHIFT             (16U)
#define TRNG_PKRCNT98_PKR_9_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT98_PKR_9_CT_SHIFT)) & TRNG_PKRCNT98_PKR_9_CT_MASK)

/*! @name PKRCNTBA - RNG Statistical Check Poker Count B and A Register */
#define TRNG_PKRCNTBA_PKR_A_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTBA_PKR_A_CT_SHIFT             (0U)
#define TRNG_PKRCNTBA_PKR_A_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTBA_PKR_A_CT_SHIFT)) & TRNG_PKRCNTBA_PKR_A_CT_MASK)
#define TRNG_PKRCNTBA_PKR_B_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTBA_PKR_B_CT_SHIFT             (16U)
#define TRNG_PKRCNTBA_PKR_B_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTBA_PKR_B_CT_SHIFT)) & TRNG_PKRCNTBA_PKR_B_CT_MASK)

/*! @name PKRCNTDC - RNG Statistical Check Poker Count D and C Register */
#define TRNG_PKRCNTDC_PKR_C_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTDC_PKR_C_CT_SHIFT             (0U)
#define TRNG_PKRCNTDC_PKR_C_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTDC_PKR_C_CT_SHIFT)) & TRNG_PKRCNTDC_PKR_C_CT_MASK)
#define TRNG_PKRCNTDC_PKR_D_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTDC_PKR_D_CT_SHIFT             (16U)
#define TRNG_PKRCNTDC_PKR_D_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTDC_PKR_D_CT_SHIFT)) & TRNG_PKRCNTDC_PKR_D_CT_MASK)

/*! @name PKRCNTFE - RNG Statistical Check Poker Count F and E Register */
#define TRNG_PKRCNTFE_PKR_E_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTFE_PKR_E_CT_SHIFT             (0U)
#define TRNG_PKRCNTFE_PKR_E_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTFE_PKR_E_CT_SHIFT)) & TRNG_PKRCNTFE_PKR_E_CT_MASK)
#define TRNG_PKRCNTFE_PKR_F_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTFE_PKR_F_CT_SHIFT             (16U)
#define TRNG_PKRCNTFE_PKR_F_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTFE_PKR_F_CT_SHIFT)) & TRNG_PKRCNTFE_PKR_F_CT_MASK)

/*! @name SEC_CFG - RNG Security Configuration Register */
#define TRNG_SEC_CFG_SH0_MASK                    (0x1U)
#define TRNG_SEC_CFG_SH0_SHIFT                   (0U)
#define TRNG_SEC_CFG_SH0(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_SH0_SHIFT)) & TRNG_SEC_CFG_SH0_MASK)
#define TRNG_SEC_CFG_NO_PRGM_MASK                (0x2U)
#define TRNG_SEC_CFG_NO_PRGM_SHIFT               (1U)
#define TRNG_SEC_CFG_NO_PRGM(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_NO_PRGM_SHIFT)) & TRNG_SEC_CFG_NO_PRGM_MASK)
#define TRNG_SEC_CFG_SK_VAL_MASK                 (0x4U)
#define TRNG_SEC_CFG_SK_VAL_SHIFT                (2U)
#define TRNG_SEC_CFG_SK_VAL(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_SK_VAL_SHIFT)) & TRNG_SEC_CFG_SK_VAL_MASK)

/*! @name INT_CTRL - RNG Interrupt Control Register */
#define TRNG_INT_CTRL_HW_ERR_MASK                (0x1U)
#define TRNG_INT_CTRL_HW_ERR_SHIFT               (0U)
#define TRNG_INT_CTRL_HW_ERR(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_INT_CTRL_HW_ERR_SHIFT)) & TRNG_INT_CTRL_HW_ERR_MASK)
#define TRNG_INT_CTRL_ENT_VAL_MASK               (0x2U)
#define TRNG_INT_CTRL_ENT_VAL_SHIFT              (1U)
#define TRNG_INT_CTRL_ENT_VAL(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_INT_CTRL_ENT_VAL_SHIFT)) & TRNG_INT_CTRL_ENT_VAL_MASK)
#define TRNG_INT_CTRL_FRQ_CT_FAIL_MASK           (0x4U)
#define TRNG_INT_CTRL_FRQ_CT_FAIL_SHIFT          (2U)
#define TRNG_INT_CTRL_FRQ_CT_FAIL(x)             (((uint32_t)(((uint32_t)(x)) << TRNG_INT_CTRL_FRQ_CT_FAIL_SHIFT)) & TRNG_INT_CTRL_FRQ_CT_FAIL_MASK)
#define TRNG_INT_CTRL_UNUSED_MASK                (0xFFFFFFF8U)
#define TRNG_INT_CTRL_UNUSED_SHIFT               (3U)
#define TRNG_INT_CTRL_UNUSED(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_INT_CTRL_UNUSED_SHIFT)) & TRNG_INT_CTRL_UNUSED_MASK)

/*! @name INT_MASK - RNG Mask Register */
#define TRNG_INT_MASK_HW_ERR_MASK                (0x1U)
#define TRNG_INT_MASK_HW_ERR_SHIFT               (0U)
#define TRNG_INT_MASK_HW_ERR(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_HW_ERR_SHIFT)) & TRNG_INT_MASK_HW_ERR_MASK)
#define TRNG_INT_MASK_ENT_VAL_MASK               (0x2U)
#define TRNG_INT_MASK_ENT_VAL_SHIFT              (1U)
#define TRNG_INT_MASK_ENT_VAL(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_ENT_VAL_SHIFT)) & TRNG_INT_MASK_ENT_VAL_MASK)
#define TRNG_INT_MASK_FRQ_CT_FAIL_MASK           (0x4U)
#define TRNG_INT_MASK_FRQ_CT_FAIL_SHIFT          (2U)
#define TRNG_INT_MASK_FRQ_CT_FAIL(x)             (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_FRQ_CT_FAIL_SHIFT)) & TRNG_INT_MASK_FRQ_CT_FAIL_MASK)

/*! @name INT_STATUS - RNG Interrupt Status Register */
#define TRNG_INT_STATUS_HW_ERR_MASK              (0x1U)
#define TRNG_INT_STATUS_HW_ERR_SHIFT             (0U)
#define TRNG_INT_STATUS_HW_ERR(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_HW_ERR_SHIFT)) & TRNG_INT_STATUS_HW_ERR_MASK)
#define TRNG_INT_STATUS_ENT_VAL_MASK             (0x2U)
#define TRNG_INT_STATUS_ENT_VAL_SHIFT            (1U)
#define TRNG_INT_STATUS_ENT_VAL(x)               (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_ENT_VAL_SHIFT)) & TRNG_INT_STATUS_ENT_VAL_MASK)
#define TRNG_INT_STATUS_FRQ_CT_FAIL_MASK         (0x4U)
#define TRNG_INT_STATUS_FRQ_CT_FAIL_SHIFT        (2U)
#define TRNG_INT_STATUS_FRQ_CT_FAIL(x)           (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_FRQ_CT_FAIL_SHIFT)) & TRNG_INT_STATUS_FRQ_CT_FAIL_MASK)

/*! @name VID1 - RNG Version ID Register (MS) */
#define TRNG_VID1_RNG_MIN_REV_MASK               (0xFFU)
#define TRNG_VID1_RNG_MIN_REV_SHIFT              (0U)
#define TRNG_VID1_RNG_MIN_REV(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_RNG_MIN_REV_SHIFT)) & TRNG_VID1_RNG_MIN_REV_MASK)
#define TRNG_VID1_RNG_MAJ_REV_MASK               (0xFF00U)
#define TRNG_VID1_RNG_MAJ_REV_SHIFT              (8U)
#define TRNG_VID1_RNG_MAJ_REV(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_RNG_MAJ_REV_SHIFT)) & TRNG_VID1_RNG_MAJ_REV_MASK)
#define TRNG_VID1_RNG_IP_ID_MASK                 (0xFFFF0000U)
#define TRNG_VID1_RNG_IP_ID_SHIFT                (16U)
#define TRNG_VID1_RNG_IP_ID(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_RNG_IP_ID_SHIFT)) & TRNG_VID1_RNG_IP_ID_MASK)

/*! @name VID2 - RNG Version ID Register (LS) */
#define TRNG_VID2_RNG_CONFIG_OPT_MASK            (0xFFU)
#define TRNG_VID2_RNG_CONFIG_OPT_SHIFT           (0U)
#define TRNG_VID2_RNG_CONFIG_OPT(x)              (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_RNG_CONFIG_OPT_SHIFT)) & TRNG_VID2_RNG_CONFIG_OPT_MASK)
#define TRNG_VID2_RNG_ECO_REV_MASK               (0xFF00U)
#define TRNG_VID2_RNG_ECO_REV_SHIFT              (8U)
#define TRNG_VID2_RNG_ECO_REV(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_RNG_ECO_REV_SHIFT)) & TRNG_VID2_RNG_ECO_REV_MASK)
#define TRNG_VID2_RNG_INTG_OPT_MASK              (0xFF0000U)
#define TRNG_VID2_RNG_INTG_OPT_SHIFT             (16U)
#define TRNG_VID2_RNG_INTG_OPT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_RNG_INTG_OPT_SHIFT)) & TRNG_VID2_RNG_INTG_OPT_MASK)
#define TRNG_VID2_RNG_ERA_MASK                   (0xFF000000U)
#define TRNG_VID2_RNG_ERA_SHIFT                  (24U)
#define TRNG_VID2_RNG_ERA(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_RNG_ERA_SHIFT)) & TRNG_VID2_RNG_ERA_MASK)


/*!
 * @}
 */ /* end of group TRNG_Register_Masks */


/* TRNG - Peripheral instance base addresses */
/** Peripheral TRNG0 base address */
#define TRNG0_BASE                               (0x400A0000u)
/** Peripheral TRNG0 base pointer */
#define TRNG0                                    ((TRNG_Type *)TRNG0_BASE)
/** Array initializer of TRNG peripheral base addresses */
#define TRNG_BASE_ADDRS                          { TRNG0_BASE }
/** Array initializer of TRNG peripheral base pointers */
#define TRNG_BASE_PTRS                           { TRNG0 }
/** Interrupt vectors for the TRNG peripheral type */
#define TRNG_IRQS                                { TRNG0_IRQn }

/*!
 * @}
 */ /* end of group TRNG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TSI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TSI_Peripheral_Access_Layer TSI Peripheral Access Layer
 * @{
 */

/** TSI - Register Layout Typedef */
typedef struct {
  __IO uint32_t GENCS;                             /**< TSI General Control and Status Register, offset: 0x0 */
  __IO uint32_t DATA;                              /**< TSI DATA Register, offset: 0x4 */
  __IO uint32_t TSHD;                              /**< TSI Threshold Register, offset: 0x8 */
} TSI_Type;

/* ----------------------------------------------------------------------------
   -- TSI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TSI_Register_Masks TSI Register Masks
 * @{
 */

/*! @name GENCS - TSI General Control and Status Register */
#define TSI_GENCS_EOSDMEO_MASK                   (0x1U)
#define TSI_GENCS_EOSDMEO_SHIFT                  (0U)
#define TSI_GENCS_EOSDMEO(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_EOSDMEO_SHIFT)) & TSI_GENCS_EOSDMEO_MASK)
#define TSI_GENCS_CURSW_MASK                     (0x2U)
#define TSI_GENCS_CURSW_SHIFT                    (1U)
#define TSI_GENCS_CURSW(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_CURSW_SHIFT)) & TSI_GENCS_CURSW_MASK)
#define TSI_GENCS_EOSF_MASK                      (0x4U)
#define TSI_GENCS_EOSF_SHIFT                     (2U)
#define TSI_GENCS_EOSF(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_EOSF_SHIFT)) & TSI_GENCS_EOSF_MASK)
#define TSI_GENCS_SCNIP_MASK                     (0x8U)
#define TSI_GENCS_SCNIP_SHIFT                    (3U)
#define TSI_GENCS_SCNIP(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_SCNIP_SHIFT)) & TSI_GENCS_SCNIP_MASK)
#define TSI_GENCS_STM_MASK                       (0x10U)
#define TSI_GENCS_STM_SHIFT                      (4U)
#define TSI_GENCS_STM(x)                         (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_STM_SHIFT)) & TSI_GENCS_STM_MASK)
#define TSI_GENCS_STPE_MASK                      (0x20U)
#define TSI_GENCS_STPE_SHIFT                     (5U)
#define TSI_GENCS_STPE(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_STPE_SHIFT)) & TSI_GENCS_STPE_MASK)
#define TSI_GENCS_TSIIEN_MASK                    (0x40U)
#define TSI_GENCS_TSIIEN_SHIFT                   (6U)
#define TSI_GENCS_TSIIEN(x)                      (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_TSIIEN_SHIFT)) & TSI_GENCS_TSIIEN_MASK)
#define TSI_GENCS_TSIEN_MASK                     (0x80U)
#define TSI_GENCS_TSIEN_SHIFT                    (7U)
#define TSI_GENCS_TSIEN(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_TSIEN_SHIFT)) & TSI_GENCS_TSIEN_MASK)
#define TSI_GENCS_NSCN_MASK                      (0x1F00U)
#define TSI_GENCS_NSCN_SHIFT                     (8U)
#define TSI_GENCS_NSCN(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_NSCN_SHIFT)) & TSI_GENCS_NSCN_MASK)
#define TSI_GENCS_PS_MASK                        (0xE000U)
#define TSI_GENCS_PS_SHIFT                       (13U)
#define TSI_GENCS_PS(x)                          (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_PS_SHIFT)) & TSI_GENCS_PS_MASK)
#define TSI_GENCS_EXTCHRG_MASK                   (0x70000U)
#define TSI_GENCS_EXTCHRG_SHIFT                  (16U)
#define TSI_GENCS_EXTCHRG(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_EXTCHRG_SHIFT)) & TSI_GENCS_EXTCHRG_MASK)
#define TSI_GENCS_DVOLT_MASK                     (0x180000U)
#define TSI_GENCS_DVOLT_SHIFT                    (19U)
#define TSI_GENCS_DVOLT(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_DVOLT_SHIFT)) & TSI_GENCS_DVOLT_MASK)
#define TSI_GENCS_REFCHRG_MASK                   (0xE00000U)
#define TSI_GENCS_REFCHRG_SHIFT                  (21U)
#define TSI_GENCS_REFCHRG(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_REFCHRG_SHIFT)) & TSI_GENCS_REFCHRG_MASK)
#define TSI_GENCS_MODE_MASK                      (0xF000000U)
#define TSI_GENCS_MODE_SHIFT                     (24U)
#define TSI_GENCS_MODE(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_MODE_SHIFT)) & TSI_GENCS_MODE_MASK)
#define TSI_GENCS_ESOR_MASK                      (0x10000000U)
#define TSI_GENCS_ESOR_SHIFT                     (28U)
#define TSI_GENCS_ESOR(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_ESOR_SHIFT)) & TSI_GENCS_ESOR_MASK)
#define TSI_GENCS_OUTRGF_MASK                    (0x80000000U)
#define TSI_GENCS_OUTRGF_SHIFT                   (31U)
#define TSI_GENCS_OUTRGF(x)                      (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_OUTRGF_SHIFT)) & TSI_GENCS_OUTRGF_MASK)

/*! @name DATA - TSI DATA Register */
#define TSI_DATA_TSICNT_MASK                     (0xFFFFU)
#define TSI_DATA_TSICNT_SHIFT                    (0U)
#define TSI_DATA_TSICNT(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_DATA_TSICNT_SHIFT)) & TSI_DATA_TSICNT_MASK)
#define TSI_DATA_SWTS_MASK                       (0x400000U)
#define TSI_DATA_SWTS_SHIFT                      (22U)
#define TSI_DATA_SWTS(x)                         (((uint32_t)(((uint32_t)(x)) << TSI_DATA_SWTS_SHIFT)) & TSI_DATA_SWTS_MASK)
#define TSI_DATA_DMAEN_MASK                      (0x800000U)
#define TSI_DATA_DMAEN_SHIFT                     (23U)
#define TSI_DATA_DMAEN(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_DATA_DMAEN_SHIFT)) & TSI_DATA_DMAEN_MASK)
#define TSI_DATA_TSICH_MASK                      (0xF0000000U)
#define TSI_DATA_TSICH_SHIFT                     (28U)
#define TSI_DATA_TSICH(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_DATA_TSICH_SHIFT)) & TSI_DATA_TSICH_MASK)

/*! @name TSHD - TSI Threshold Register */
#define TSI_TSHD_THRESL_MASK                     (0xFFFFU)
#define TSI_TSHD_THRESL_SHIFT                    (0U)
#define TSI_TSHD_THRESL(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_TSHD_THRESL_SHIFT)) & TSI_TSHD_THRESL_MASK)
#define TSI_TSHD_THRESH_MASK                     (0xFFFF0000U)
#define TSI_TSHD_THRESH_SHIFT                    (16U)
#define TSI_TSHD_THRESH(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_TSHD_THRESH_SHIFT)) & TSI_TSHD_THRESH_MASK)


/*!
 * @}
 */ /* end of group TSI_Register_Masks */


/* TSI - Peripheral instance base addresses */
/** Peripheral TSI0 base address */
#define TSI0_BASE                                (0x40045000u)
/** Peripheral TSI0 base pointer */
#define TSI0                                     ((TSI_Type *)TSI0_BASE)
/** Array initializer of TSI peripheral base addresses */
#define TSI_BASE_ADDRS                           { TSI0_BASE }
/** Array initializer of TSI peripheral base pointers */
#define TSI_BASE_PTRS                            { TSI0 }
/** Interrupt vectors for the TSI peripheral type */
#define TSI_IRQS                                 { TSI0_IRQn }

/*!
 * @}
 */ /* end of group TSI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- USB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Peripheral_Access_Layer USB Peripheral Access Layer
 * @{
 */

/** USB - Register Layout Typedef */
typedef struct {
  __I  uint8_t PERID;                              /**< Peripheral ID register, offset: 0x0 */
       uint8_t RESERVED_0[3];
  __I  uint8_t IDCOMP;                             /**< Peripheral ID Complement register, offset: 0x4 */
       uint8_t RESERVED_1[3];
  __I  uint8_t REV;                                /**< Peripheral Revision register, offset: 0x8 */
       uint8_t RESERVED_2[3];
  __I  uint8_t ADDINFO;                            /**< Peripheral Additional Info register, offset: 0xC */
       uint8_t RESERVED_3[3];
  __IO uint8_t OTGISTAT;                           /**< OTG Interrupt Status register, offset: 0x10 */
       uint8_t RESERVED_4[3];
  __IO uint8_t OTGICR;                             /**< OTG Interrupt Control register, offset: 0x14 */
       uint8_t RESERVED_5[3];
  __IO uint8_t OTGSTAT;                            /**< OTG Status register, offset: 0x18 */
       uint8_t RESERVED_6[3];
  __IO uint8_t OTGCTL;                             /**< OTG Control register, offset: 0x1C */
       uint8_t RESERVED_7[99];
  __IO uint8_t ISTAT;                              /**< Interrupt Status register, offset: 0x80 */
       uint8_t RESERVED_8[3];
  __IO uint8_t INTEN;                              /**< Interrupt Enable register, offset: 0x84 */
       uint8_t RESERVED_9[3];
  __IO uint8_t ERRSTAT;                            /**< Error Interrupt Status register, offset: 0x88 */
       uint8_t RESERVED_10[3];
  __IO uint8_t ERREN;                              /**< Error Interrupt Enable register, offset: 0x8C */
       uint8_t RESERVED_11[3];
  __I  uint8_t STAT;                               /**< Status register, offset: 0x90 */
       uint8_t RESERVED_12[3];
  __IO uint8_t CTL;                                /**< Control register, offset: 0x94 */
       uint8_t RESERVED_13[3];
  __IO uint8_t ADDR;                               /**< Address register, offset: 0x98 */
       uint8_t RESERVED_14[3];
  __IO uint8_t BDTPAGE1;                           /**< BDT Page register 1, offset: 0x9C */
       uint8_t RESERVED_15[3];
  __IO uint8_t FRMNUML;                            /**< Frame Number register Low, offset: 0xA0 */
       uint8_t RESERVED_16[3];
  __IO uint8_t FRMNUMH;                            /**< Frame Number register High, offset: 0xA4 */
       uint8_t RESERVED_17[3];
  __IO uint8_t TOKEN;                              /**< Token register, offset: 0xA8 */
       uint8_t RESERVED_18[3];
  __IO uint8_t SOFTHLD;                            /**< SOF Threshold register, offset: 0xAC */
       uint8_t RESERVED_19[3];
  __IO uint8_t BDTPAGE2;                           /**< BDT Page Register 2, offset: 0xB0 */
       uint8_t RESERVED_20[3];
  __IO uint8_t BDTPAGE3;                           /**< BDT Page Register 3, offset: 0xB4 */
       uint8_t RESERVED_21[11];
  struct {                                         /* offset: 0xC0, array step: 0x4 */
    __IO uint8_t ENDPT;                              /**< Endpoint Control register, array offset: 0xC0, array step: 0x4 */
         uint8_t RESERVED_0[3];
  } ENDPOINT[16];
  __IO uint8_t USBCTRL;                            /**< USB Control register, offset: 0x100 */
       uint8_t RESERVED_22[3];
  __I  uint8_t OBSERVE;                            /**< USB OTG Observe register, offset: 0x104 */
       uint8_t RESERVED_23[3];
  __IO uint8_t CONTROL;                            /**< USB OTG Control register, offset: 0x108 */
       uint8_t RESERVED_24[3];
  __IO uint8_t USBTRC0;                            /**< USB Transceiver Control register 0, offset: 0x10C */
       uint8_t RESERVED_25[7];
  __IO uint8_t USBFRMADJUST;                       /**< Frame Adjust Register, offset: 0x114 */
       uint8_t RESERVED_26[23];
  __IO uint8_t MISCCTRL;                           /**< Miscellaneous Control register, offset: 0x12C */
       uint8_t RESERVED_27[19];
  __IO uint8_t CLK_RECOVER_CTRL;                   /**< USB Clock recovery control, offset: 0x140 */
       uint8_t RESERVED_28[3];
  __IO uint8_t CLK_RECOVER_IRC_EN;                 /**< IRC48M oscillator enable register, offset: 0x144 */
       uint8_t RESERVED_29[15];
  __IO uint8_t CLK_RECOVER_INT_EN;                 /**< Clock recovery combined interrupt enable, offset: 0x154 */
       uint8_t RESERVED_30[7];
  __IO uint8_t CLK_RECOVER_INT_STATUS;             /**< Clock recovery separated interrupt status, offset: 0x15C */
} USB_Type;

/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/*! @name PERID - Peripheral ID register */
#define USB_PERID_ID_MASK                        (0x3FU)
#define USB_PERID_ID_SHIFT                       (0U)
#define USB_PERID_ID(x)                          (((uint8_t)(((uint8_t)(x)) << USB_PERID_ID_SHIFT)) & USB_PERID_ID_MASK)

/*! @name IDCOMP - Peripheral ID Complement register */
#define USB_IDCOMP_NID_MASK                      (0x3FU)
#define USB_IDCOMP_NID_SHIFT                     (0U)
#define USB_IDCOMP_NID(x)                        (((uint8_t)(((uint8_t)(x)) << USB_IDCOMP_NID_SHIFT)) & USB_IDCOMP_NID_MASK)

/*! @name REV - Peripheral Revision register */
#define USB_REV_REV_MASK                         (0xFFU)
#define USB_REV_REV_SHIFT                        (0U)
#define USB_REV_REV(x)                           (((uint8_t)(((uint8_t)(x)) << USB_REV_REV_SHIFT)) & USB_REV_REV_MASK)

/*! @name ADDINFO - Peripheral Additional Info register */
#define USB_ADDINFO_IEHOST_MASK                  (0x1U)
#define USB_ADDINFO_IEHOST_SHIFT                 (0U)
#define USB_ADDINFO_IEHOST(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ADDINFO_IEHOST_SHIFT)) & USB_ADDINFO_IEHOST_MASK)

/*! @name OTGISTAT - OTG Interrupt Status register */
#define USB_OTGISTAT_AVBUSCHG_MASK               (0x1U)
#define USB_OTGISTAT_AVBUSCHG_SHIFT              (0U)
#define USB_OTGISTAT_AVBUSCHG(x)                 (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_AVBUSCHG_SHIFT)) & USB_OTGISTAT_AVBUSCHG_MASK)
#define USB_OTGISTAT_B_SESS_CHG_MASK             (0x4U)
#define USB_OTGISTAT_B_SESS_CHG_SHIFT            (2U)
#define USB_OTGISTAT_B_SESS_CHG(x)               (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_B_SESS_CHG_SHIFT)) & USB_OTGISTAT_B_SESS_CHG_MASK)
#define USB_OTGISTAT_SESSVLDCHG_MASK             (0x8U)
#define USB_OTGISTAT_SESSVLDCHG_SHIFT            (3U)
#define USB_OTGISTAT_SESSVLDCHG(x)               (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_SESSVLDCHG_SHIFT)) & USB_OTGISTAT_SESSVLDCHG_MASK)
#define USB_OTGISTAT_LINE_STATE_CHG_MASK         (0x20U)
#define USB_OTGISTAT_LINE_STATE_CHG_SHIFT        (5U)
#define USB_OTGISTAT_LINE_STATE_CHG(x)           (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_LINE_STATE_CHG_SHIFT)) & USB_OTGISTAT_LINE_STATE_CHG_MASK)
#define USB_OTGISTAT_ONEMSEC_MASK                (0x40U)
#define USB_OTGISTAT_ONEMSEC_SHIFT               (6U)
#define USB_OTGISTAT_ONEMSEC(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_ONEMSEC_SHIFT)) & USB_OTGISTAT_ONEMSEC_MASK)
#define USB_OTGISTAT_IDCHG_MASK                  (0x80U)
#define USB_OTGISTAT_IDCHG_SHIFT                 (7U)
#define USB_OTGISTAT_IDCHG(x)                    (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_IDCHG_SHIFT)) & USB_OTGISTAT_IDCHG_MASK)

/*! @name OTGICR - OTG Interrupt Control register */
#define USB_OTGICR_AVBUSEN_MASK                  (0x1U)
#define USB_OTGICR_AVBUSEN_SHIFT                 (0U)
#define USB_OTGICR_AVBUSEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_AVBUSEN_SHIFT)) & USB_OTGICR_AVBUSEN_MASK)
#define USB_OTGICR_BSESSEN_MASK                  (0x4U)
#define USB_OTGICR_BSESSEN_SHIFT                 (2U)
#define USB_OTGICR_BSESSEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_BSESSEN_SHIFT)) & USB_OTGICR_BSESSEN_MASK)
#define USB_OTGICR_SESSVLDEN_MASK                (0x8U)
#define USB_OTGICR_SESSVLDEN_SHIFT               (3U)
#define USB_OTGICR_SESSVLDEN(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_SESSVLDEN_SHIFT)) & USB_OTGICR_SESSVLDEN_MASK)
#define USB_OTGICR_LINESTATEEN_MASK              (0x20U)
#define USB_OTGICR_LINESTATEEN_SHIFT             (5U)
#define USB_OTGICR_LINESTATEEN(x)                (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_LINESTATEEN_SHIFT)) & USB_OTGICR_LINESTATEEN_MASK)
#define USB_OTGICR_ONEMSECEN_MASK                (0x40U)
#define USB_OTGICR_ONEMSECEN_SHIFT               (6U)
#define USB_OTGICR_ONEMSECEN(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_ONEMSECEN_SHIFT)) & USB_OTGICR_ONEMSECEN_MASK)
#define USB_OTGICR_IDEN_MASK                     (0x80U)
#define USB_OTGICR_IDEN_SHIFT                    (7U)
#define USB_OTGICR_IDEN(x)                       (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_IDEN_SHIFT)) & USB_OTGICR_IDEN_MASK)

/*! @name OTGSTAT - OTG Status register */
#define USB_OTGSTAT_AVBUSVLD_MASK                (0x1U)
#define USB_OTGSTAT_AVBUSVLD_SHIFT               (0U)
#define USB_OTGSTAT_AVBUSVLD(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_AVBUSVLD_SHIFT)) & USB_OTGSTAT_AVBUSVLD_MASK)
#define USB_OTGSTAT_BSESSEND_MASK                (0x4U)
#define USB_OTGSTAT_BSESSEND_SHIFT               (2U)
#define USB_OTGSTAT_BSESSEND(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_BSESSEND_SHIFT)) & USB_OTGSTAT_BSESSEND_MASK)
#define USB_OTGSTAT_SESS_VLD_MASK                (0x8U)
#define USB_OTGSTAT_SESS_VLD_SHIFT               (3U)
#define USB_OTGSTAT_SESS_VLD(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_SESS_VLD_SHIFT)) & USB_OTGSTAT_SESS_VLD_MASK)
#define USB_OTGSTAT_LINESTATESTABLE_MASK         (0x20U)
#define USB_OTGSTAT_LINESTATESTABLE_SHIFT        (5U)
#define USB_OTGSTAT_LINESTATESTABLE(x)           (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_LINESTATESTABLE_SHIFT)) & USB_OTGSTAT_LINESTATESTABLE_MASK)
#define USB_OTGSTAT_ONEMSECEN_MASK               (0x40U)
#define USB_OTGSTAT_ONEMSECEN_SHIFT              (6U)
#define USB_OTGSTAT_ONEMSECEN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_ONEMSECEN_SHIFT)) & USB_OTGSTAT_ONEMSECEN_MASK)
#define USB_OTGSTAT_ID_MASK                      (0x80U)
#define USB_OTGSTAT_ID_SHIFT                     (7U)
#define USB_OTGSTAT_ID(x)                        (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_ID_SHIFT)) & USB_OTGSTAT_ID_MASK)

/*! @name OTGCTL - OTG Control register */
#define USB_OTGCTL_OTGEN_MASK                    (0x4U)
#define USB_OTGCTL_OTGEN_SHIFT                   (2U)
#define USB_OTGCTL_OTGEN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OTGCTL_OTGEN_SHIFT)) & USB_OTGCTL_OTGEN_MASK)
#define USB_OTGCTL_DMLOW_MASK                    (0x10U)
#define USB_OTGCTL_DMLOW_SHIFT                   (4U)
#define USB_OTGCTL_DMLOW(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OTGCTL_DMLOW_SHIFT)) & USB_OTGCTL_DMLOW_MASK)
#define USB_OTGCTL_DPLOW_MASK                    (0x20U)
#define USB_OTGCTL_DPLOW_SHIFT                   (5U)
#define USB_OTGCTL_DPLOW(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OTGCTL_DPLOW_SHIFT)) & USB_OTGCTL_DPLOW_MASK)
#define USB_OTGCTL_DPHIGH_MASK                   (0x80U)
#define USB_OTGCTL_DPHIGH_SHIFT                  (7U)
#define USB_OTGCTL_DPHIGH(x)                     (((uint8_t)(((uint8_t)(x)) << USB_OTGCTL_DPHIGH_SHIFT)) & USB_OTGCTL_DPHIGH_MASK)

/*! @name ISTAT - Interrupt Status register */
#define USB_ISTAT_USBRST_MASK                    (0x1U)
#define USB_ISTAT_USBRST_SHIFT                   (0U)
#define USB_ISTAT_USBRST(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_USBRST_SHIFT)) & USB_ISTAT_USBRST_MASK)
#define USB_ISTAT_ERROR_MASK                     (0x2U)
#define USB_ISTAT_ERROR_SHIFT                    (1U)
#define USB_ISTAT_ERROR(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_ERROR_SHIFT)) & USB_ISTAT_ERROR_MASK)
#define USB_ISTAT_SOFTOK_MASK                    (0x4U)
#define USB_ISTAT_SOFTOK_SHIFT                   (2U)
#define USB_ISTAT_SOFTOK(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_SOFTOK_SHIFT)) & USB_ISTAT_SOFTOK_MASK)
#define USB_ISTAT_TOKDNE_MASK                    (0x8U)
#define USB_ISTAT_TOKDNE_SHIFT                   (3U)
#define USB_ISTAT_TOKDNE(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_TOKDNE_SHIFT)) & USB_ISTAT_TOKDNE_MASK)
#define USB_ISTAT_SLEEP_MASK                     (0x10U)
#define USB_ISTAT_SLEEP_SHIFT                    (4U)
#define USB_ISTAT_SLEEP(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_SLEEP_SHIFT)) & USB_ISTAT_SLEEP_MASK)
#define USB_ISTAT_RESUME_MASK                    (0x20U)
#define USB_ISTAT_RESUME_SHIFT                   (5U)
#define USB_ISTAT_RESUME(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_RESUME_SHIFT)) & USB_ISTAT_RESUME_MASK)
#define USB_ISTAT_ATTACH_MASK                    (0x40U)
#define USB_ISTAT_ATTACH_SHIFT                   (6U)
#define USB_ISTAT_ATTACH(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_ATTACH_SHIFT)) & USB_ISTAT_ATTACH_MASK)
#define USB_ISTAT_STALL_MASK                     (0x80U)
#define USB_ISTAT_STALL_SHIFT                    (7U)
#define USB_ISTAT_STALL(x)                       (((uint8_t)(((uint8_t)(x)) << USB_ISTAT_STALL_SHIFT)) & USB_ISTAT_STALL_MASK)

/*! @name INTEN - Interrupt Enable register */
#define USB_INTEN_USBRSTEN_MASK                  (0x1U)
#define USB_INTEN_USBRSTEN_SHIFT                 (0U)
#define USB_INTEN_USBRSTEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_USBRSTEN_SHIFT)) & USB_INTEN_USBRSTEN_MASK)
#define USB_INTEN_ERROREN_MASK                   (0x2U)
#define USB_INTEN_ERROREN_SHIFT                  (1U)
#define USB_INTEN_ERROREN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_ERROREN_SHIFT)) & USB_INTEN_ERROREN_MASK)
#define USB_INTEN_SOFTOKEN_MASK                  (0x4U)
#define USB_INTEN_SOFTOKEN_SHIFT                 (2U)
#define USB_INTEN_SOFTOKEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_SOFTOKEN_SHIFT)) & USB_INTEN_SOFTOKEN_MASK)
#define USB_INTEN_TOKDNEEN_MASK                  (0x8U)
#define USB_INTEN_TOKDNEEN_SHIFT                 (3U)
#define USB_INTEN_TOKDNEEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_TOKDNEEN_SHIFT)) & USB_INTEN_TOKDNEEN_MASK)
#define USB_INTEN_SLEEPEN_MASK                   (0x10U)
#define USB_INTEN_SLEEPEN_SHIFT                  (4U)
#define USB_INTEN_SLEEPEN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_SLEEPEN_SHIFT)) & USB_INTEN_SLEEPEN_MASK)
#define USB_INTEN_RESUMEEN_MASK                  (0x20U)
#define USB_INTEN_RESUMEEN_SHIFT                 (5U)
#define USB_INTEN_RESUMEEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_RESUMEEN_SHIFT)) & USB_INTEN_RESUMEEN_MASK)
#define USB_INTEN_ATTACHEN_MASK                  (0x40U)
#define USB_INTEN_ATTACHEN_SHIFT                 (6U)
#define USB_INTEN_ATTACHEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_INTEN_ATTACHEN_SHIFT)) & USB_INTEN_ATTACHEN_MASK)
#define USB_INTEN_STALLEN_MASK                   (0x80U)
#define USB_INTEN_STALLEN_SHIFT                  (7U)
#define USB_INTEN_STALLEN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_INTEN_STALLEN_SHIFT)) & USB_INTEN_STALLEN_MASK)

/*! @name ERRSTAT - Error Interrupt Status register */
#define USB_ERRSTAT_PIDERR_MASK                  (0x1U)
#define USB_ERRSTAT_PIDERR_SHIFT                 (0U)
#define USB_ERRSTAT_PIDERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_PIDERR_SHIFT)) & USB_ERRSTAT_PIDERR_MASK)
#define USB_ERRSTAT_CRC5EOF_MASK                 (0x2U)
#define USB_ERRSTAT_CRC5EOF_SHIFT                (1U)
#define USB_ERRSTAT_CRC5EOF(x)                   (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_CRC5EOF_SHIFT)) & USB_ERRSTAT_CRC5EOF_MASK)
#define USB_ERRSTAT_CRC16_MASK                   (0x4U)
#define USB_ERRSTAT_CRC16_SHIFT                  (2U)
#define USB_ERRSTAT_CRC16(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_CRC16_SHIFT)) & USB_ERRSTAT_CRC16_MASK)
#define USB_ERRSTAT_DFN8_MASK                    (0x8U)
#define USB_ERRSTAT_DFN8_SHIFT                   (3U)
#define USB_ERRSTAT_DFN8(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_DFN8_SHIFT)) & USB_ERRSTAT_DFN8_MASK)
#define USB_ERRSTAT_BTOERR_MASK                  (0x10U)
#define USB_ERRSTAT_BTOERR_SHIFT                 (4U)
#define USB_ERRSTAT_BTOERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_BTOERR_SHIFT)) & USB_ERRSTAT_BTOERR_MASK)
#define USB_ERRSTAT_DMAERR_MASK                  (0x20U)
#define USB_ERRSTAT_DMAERR_SHIFT                 (5U)
#define USB_ERRSTAT_DMAERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_DMAERR_SHIFT)) & USB_ERRSTAT_DMAERR_MASK)
#define USB_ERRSTAT_OWNERR_MASK                  (0x40U)
#define USB_ERRSTAT_OWNERR_SHIFT                 (6U)
#define USB_ERRSTAT_OWNERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_OWNERR_SHIFT)) & USB_ERRSTAT_OWNERR_MASK)
#define USB_ERRSTAT_BTSERR_MASK                  (0x80U)
#define USB_ERRSTAT_BTSERR_SHIFT                 (7U)
#define USB_ERRSTAT_BTSERR(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERRSTAT_BTSERR_SHIFT)) & USB_ERRSTAT_BTSERR_MASK)

/*! @name ERREN - Error Interrupt Enable register */
#define USB_ERREN_PIDERREN_MASK                  (0x1U)
#define USB_ERREN_PIDERREN_SHIFT                 (0U)
#define USB_ERREN_PIDERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_PIDERREN_SHIFT)) & USB_ERREN_PIDERREN_MASK)
#define USB_ERREN_CRC5EOFEN_MASK                 (0x2U)
#define USB_ERREN_CRC5EOFEN_SHIFT                (1U)
#define USB_ERREN_CRC5EOFEN(x)                   (((uint8_t)(((uint8_t)(x)) << USB_ERREN_CRC5EOFEN_SHIFT)) & USB_ERREN_CRC5EOFEN_MASK)
#define USB_ERREN_CRC16EN_MASK                   (0x4U)
#define USB_ERREN_CRC16EN_SHIFT                  (2U)
#define USB_ERREN_CRC16EN(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ERREN_CRC16EN_SHIFT)) & USB_ERREN_CRC16EN_MASK)
#define USB_ERREN_DFN8EN_MASK                    (0x8U)
#define USB_ERREN_DFN8EN_SHIFT                   (3U)
#define USB_ERREN_DFN8EN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ERREN_DFN8EN_SHIFT)) & USB_ERREN_DFN8EN_MASK)
#define USB_ERREN_BTOERREN_MASK                  (0x10U)
#define USB_ERREN_BTOERREN_SHIFT                 (4U)
#define USB_ERREN_BTOERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_BTOERREN_SHIFT)) & USB_ERREN_BTOERREN_MASK)
#define USB_ERREN_DMAERREN_MASK                  (0x20U)
#define USB_ERREN_DMAERREN_SHIFT                 (5U)
#define USB_ERREN_DMAERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_DMAERREN_SHIFT)) & USB_ERREN_DMAERREN_MASK)
#define USB_ERREN_OWNERREN_MASK                  (0x40U)
#define USB_ERREN_OWNERREN_SHIFT                 (6U)
#define USB_ERREN_OWNERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_OWNERREN_SHIFT)) & USB_ERREN_OWNERREN_MASK)
#define USB_ERREN_BTSERREN_MASK                  (0x80U)
#define USB_ERREN_BTSERREN_SHIFT                 (7U)
#define USB_ERREN_BTSERREN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ERREN_BTSERREN_SHIFT)) & USB_ERREN_BTSERREN_MASK)

/*! @name STAT - Status register */
#define USB_STAT_ODD_MASK                        (0x4U)
#define USB_STAT_ODD_SHIFT                       (2U)
#define USB_STAT_ODD(x)                          (((uint8_t)(((uint8_t)(x)) << USB_STAT_ODD_SHIFT)) & USB_STAT_ODD_MASK)
#define USB_STAT_TX_MASK                         (0x8U)
#define USB_STAT_TX_SHIFT                        (3U)
#define USB_STAT_TX(x)                           (((uint8_t)(((uint8_t)(x)) << USB_STAT_TX_SHIFT)) & USB_STAT_TX_MASK)
#define USB_STAT_ENDP_MASK                       (0xF0U)
#define USB_STAT_ENDP_SHIFT                      (4U)
#define USB_STAT_ENDP(x)                         (((uint8_t)(((uint8_t)(x)) << USB_STAT_ENDP_SHIFT)) & USB_STAT_ENDP_MASK)

/*! @name CTL - Control register */
#define USB_CTL_USBENSOFEN_MASK                  (0x1U)
#define USB_CTL_USBENSOFEN_SHIFT                 (0U)
#define USB_CTL_USBENSOFEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_CTL_USBENSOFEN_SHIFT)) & USB_CTL_USBENSOFEN_MASK)
#define USB_CTL_ODDRST_MASK                      (0x2U)
#define USB_CTL_ODDRST_SHIFT                     (1U)
#define USB_CTL_ODDRST(x)                        (((uint8_t)(((uint8_t)(x)) << USB_CTL_ODDRST_SHIFT)) & USB_CTL_ODDRST_MASK)
#define USB_CTL_RESUME_MASK                      (0x4U)
#define USB_CTL_RESUME_SHIFT                     (2U)
#define USB_CTL_RESUME(x)                        (((uint8_t)(((uint8_t)(x)) << USB_CTL_RESUME_SHIFT)) & USB_CTL_RESUME_MASK)
#define USB_CTL_HOSTMODEEN_MASK                  (0x8U)
#define USB_CTL_HOSTMODEEN_SHIFT                 (3U)
#define USB_CTL_HOSTMODEEN(x)                    (((uint8_t)(((uint8_t)(x)) << USB_CTL_HOSTMODEEN_SHIFT)) & USB_CTL_HOSTMODEEN_MASK)
#define USB_CTL_RESET_MASK                       (0x10U)
#define USB_CTL_RESET_SHIFT                      (4U)
#define USB_CTL_RESET(x)                         (((uint8_t)(((uint8_t)(x)) << USB_CTL_RESET_SHIFT)) & USB_CTL_RESET_MASK)
#define USB_CTL_TXSUSPENDTOKENBUSY_MASK          (0x20U)
#define USB_CTL_TXSUSPENDTOKENBUSY_SHIFT         (5U)
#define USB_CTL_TXSUSPENDTOKENBUSY(x)            (((uint8_t)(((uint8_t)(x)) << USB_CTL_TXSUSPENDTOKENBUSY_SHIFT)) & USB_CTL_TXSUSPENDTOKENBUSY_MASK)
#define USB_CTL_SE0_MASK                         (0x40U)
#define USB_CTL_SE0_SHIFT                        (6U)
#define USB_CTL_SE0(x)                           (((uint8_t)(((uint8_t)(x)) << USB_CTL_SE0_SHIFT)) & USB_CTL_SE0_MASK)
#define USB_CTL_JSTATE_MASK                      (0x80U)
#define USB_CTL_JSTATE_SHIFT                     (7U)
#define USB_CTL_JSTATE(x)                        (((uint8_t)(((uint8_t)(x)) << USB_CTL_JSTATE_SHIFT)) & USB_CTL_JSTATE_MASK)

/*! @name ADDR - Address register */
#define USB_ADDR_ADDR_MASK                       (0x7FU)
#define USB_ADDR_ADDR_SHIFT                      (0U)
#define USB_ADDR_ADDR(x)                         (((uint8_t)(((uint8_t)(x)) << USB_ADDR_ADDR_SHIFT)) & USB_ADDR_ADDR_MASK)
#define USB_ADDR_LSEN_MASK                       (0x80U)
#define USB_ADDR_LSEN_SHIFT                      (7U)
#define USB_ADDR_LSEN(x)                         (((uint8_t)(((uint8_t)(x)) << USB_ADDR_LSEN_SHIFT)) & USB_ADDR_LSEN_MASK)

/*! @name BDTPAGE1 - BDT Page register 1 */
#define USB_BDTPAGE1_BDTBA_MASK                  (0xFEU)
#define USB_BDTPAGE1_BDTBA_SHIFT                 (1U)
#define USB_BDTPAGE1_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE1_BDTBA_SHIFT)) & USB_BDTPAGE1_BDTBA_MASK)

/*! @name FRMNUML - Frame Number register Low */
#define USB_FRMNUML_FRM_MASK                     (0xFFU)
#define USB_FRMNUML_FRM_SHIFT                    (0U)
#define USB_FRMNUML_FRM(x)                       (((uint8_t)(((uint8_t)(x)) << USB_FRMNUML_FRM_SHIFT)) & USB_FRMNUML_FRM_MASK)

/*! @name FRMNUMH - Frame Number register High */
#define USB_FRMNUMH_FRM_MASK                     (0x7U)
#define USB_FRMNUMH_FRM_SHIFT                    (0U)
#define USB_FRMNUMH_FRM(x)                       (((uint8_t)(((uint8_t)(x)) << USB_FRMNUMH_FRM_SHIFT)) & USB_FRMNUMH_FRM_MASK)

/*! @name TOKEN - Token register */
#define USB_TOKEN_TOKENENDPT_MASK                (0xFU)
#define USB_TOKEN_TOKENENDPT_SHIFT               (0U)
#define USB_TOKEN_TOKENENDPT(x)                  (((uint8_t)(((uint8_t)(x)) << USB_TOKEN_TOKENENDPT_SHIFT)) & USB_TOKEN_TOKENENDPT_MASK)
#define USB_TOKEN_TOKENPID_MASK                  (0xF0U)
#define USB_TOKEN_TOKENPID_SHIFT                 (4U)
#define USB_TOKEN_TOKENPID(x)                    (((uint8_t)(((uint8_t)(x)) << USB_TOKEN_TOKENPID_SHIFT)) & USB_TOKEN_TOKENPID_MASK)

/*! @name SOFTHLD - SOF Threshold register */
#define USB_SOFTHLD_CNT_MASK                     (0xFFU)
#define USB_SOFTHLD_CNT_SHIFT                    (0U)
#define USB_SOFTHLD_CNT(x)                       (((uint8_t)(((uint8_t)(x)) << USB_SOFTHLD_CNT_SHIFT)) & USB_SOFTHLD_CNT_MASK)

/*! @name BDTPAGE2 - BDT Page Register 2 */
#define USB_BDTPAGE2_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE2_BDTBA_SHIFT                 (0U)
#define USB_BDTPAGE2_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE2_BDTBA_SHIFT)) & USB_BDTPAGE2_BDTBA_MASK)

/*! @name BDTPAGE3 - BDT Page Register 3 */
#define USB_BDTPAGE3_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE3_BDTBA_SHIFT                 (0U)
#define USB_BDTPAGE3_BDTBA(x)                    (((uint8_t)(((uint8_t)(x)) << USB_BDTPAGE3_BDTBA_SHIFT)) & USB_BDTPAGE3_BDTBA_MASK)

/*! @name ENDPT - Endpoint Control register */
#define USB_ENDPT_EPHSHK_MASK                    (0x1U)
#define USB_ENDPT_EPHSHK_SHIFT                   (0U)
#define USB_ENDPT_EPHSHK(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPHSHK_SHIFT)) & USB_ENDPT_EPHSHK_MASK)
#define USB_ENDPT_EPSTALL_MASK                   (0x2U)
#define USB_ENDPT_EPSTALL_SHIFT                  (1U)
#define USB_ENDPT_EPSTALL(x)                     (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPSTALL_SHIFT)) & USB_ENDPT_EPSTALL_MASK)
#define USB_ENDPT_EPTXEN_MASK                    (0x4U)
#define USB_ENDPT_EPTXEN_SHIFT                   (2U)
#define USB_ENDPT_EPTXEN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPTXEN_SHIFT)) & USB_ENDPT_EPTXEN_MASK)
#define USB_ENDPT_EPRXEN_MASK                    (0x8U)
#define USB_ENDPT_EPRXEN_SHIFT                   (3U)
#define USB_ENDPT_EPRXEN(x)                      (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPRXEN_SHIFT)) & USB_ENDPT_EPRXEN_MASK)
#define USB_ENDPT_EPCTLDIS_MASK                  (0x10U)
#define USB_ENDPT_EPCTLDIS_SHIFT                 (4U)
#define USB_ENDPT_EPCTLDIS(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_EPCTLDIS_SHIFT)) & USB_ENDPT_EPCTLDIS_MASK)
#define USB_ENDPT_RETRYDIS_MASK                  (0x40U)
#define USB_ENDPT_RETRYDIS_SHIFT                 (6U)
#define USB_ENDPT_RETRYDIS(x)                    (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_RETRYDIS_SHIFT)) & USB_ENDPT_RETRYDIS_MASK)
#define USB_ENDPT_HOSTWOHUB_MASK                 (0x80U)
#define USB_ENDPT_HOSTWOHUB_SHIFT                (7U)
#define USB_ENDPT_HOSTWOHUB(x)                   (((uint8_t)(((uint8_t)(x)) << USB_ENDPT_HOSTWOHUB_SHIFT)) & USB_ENDPT_HOSTWOHUB_MASK)

/* The count of USB_ENDPT */
#define USB_ENDPT_COUNT                          (16U)

/*! @name USBCTRL - USB Control register */
#define USB_USBCTRL_UARTSEL_MASK                 (0x10U)
#define USB_USBCTRL_UARTSEL_SHIFT                (4U)
#define USB_USBCTRL_UARTSEL(x)                   (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_UARTSEL_SHIFT)) & USB_USBCTRL_UARTSEL_MASK)
#define USB_USBCTRL_UARTCHLS_MASK                (0x20U)
#define USB_USBCTRL_UARTCHLS_SHIFT               (5U)
#define USB_USBCTRL_UARTCHLS(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_UARTCHLS_SHIFT)) & USB_USBCTRL_UARTCHLS_MASK)
#define USB_USBCTRL_PDE_MASK                     (0x40U)
#define USB_USBCTRL_PDE_SHIFT                    (6U)
#define USB_USBCTRL_PDE(x)                       (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_PDE_SHIFT)) & USB_USBCTRL_PDE_MASK)
#define USB_USBCTRL_SUSP_MASK                    (0x80U)
#define USB_USBCTRL_SUSP_SHIFT                   (7U)
#define USB_USBCTRL_SUSP(x)                      (((uint8_t)(((uint8_t)(x)) << USB_USBCTRL_SUSP_SHIFT)) & USB_USBCTRL_SUSP_MASK)

/*! @name OBSERVE - USB OTG Observe register */
#define USB_OBSERVE_DMPD_MASK                    (0x10U)
#define USB_OBSERVE_DMPD_SHIFT                   (4U)
#define USB_OBSERVE_DMPD(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DMPD_SHIFT)) & USB_OBSERVE_DMPD_MASK)
#define USB_OBSERVE_DPPD_MASK                    (0x40U)
#define USB_OBSERVE_DPPD_SHIFT                   (6U)
#define USB_OBSERVE_DPPD(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DPPD_SHIFT)) & USB_OBSERVE_DPPD_MASK)
#define USB_OBSERVE_DPPU_MASK                    (0x80U)
#define USB_OBSERVE_DPPU_SHIFT                   (7U)
#define USB_OBSERVE_DPPU(x)                      (((uint8_t)(((uint8_t)(x)) << USB_OBSERVE_DPPU_SHIFT)) & USB_OBSERVE_DPPU_MASK)

/*! @name CONTROL - USB OTG Control register */
#define USB_CONTROL_DPPULLUPNONOTG_MASK          (0x10U)
#define USB_CONTROL_DPPULLUPNONOTG_SHIFT         (4U)
#define USB_CONTROL_DPPULLUPNONOTG(x)            (((uint8_t)(((uint8_t)(x)) << USB_CONTROL_DPPULLUPNONOTG_SHIFT)) & USB_CONTROL_DPPULLUPNONOTG_MASK)

/*! @name USBTRC0 - USB Transceiver Control register 0 */
#define USB_USBTRC0_USB_RESUME_INT_MASK          (0x1U)
#define USB_USBTRC0_USB_RESUME_INT_SHIFT         (0U)
#define USB_USBTRC0_USB_RESUME_INT(x)            (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USB_RESUME_INT_SHIFT)) & USB_USBTRC0_USB_RESUME_INT_MASK)
#define USB_USBTRC0_SYNC_DET_MASK                (0x2U)
#define USB_USBTRC0_SYNC_DET_SHIFT               (1U)
#define USB_USBTRC0_SYNC_DET(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_SYNC_DET_SHIFT)) & USB_USBTRC0_SYNC_DET_MASK)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK    (0x4U)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT   (2U)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT(x)      (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT)) & USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK)
#define USB_USBTRC0_VREDG_DET_MASK               (0x8U)
#define USB_USBTRC0_VREDG_DET_SHIFT              (3U)
#define USB_USBTRC0_VREDG_DET(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_VREDG_DET_SHIFT)) & USB_USBTRC0_VREDG_DET_MASK)
#define USB_USBTRC0_VFEDG_DET_MASK               (0x10U)
#define USB_USBTRC0_VFEDG_DET_SHIFT              (4U)
#define USB_USBTRC0_VFEDG_DET(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_VFEDG_DET_SHIFT)) & USB_USBTRC0_VFEDG_DET_MASK)
#define USB_USBTRC0_USBRESMEN_MASK               (0x20U)
#define USB_USBTRC0_USBRESMEN_SHIFT              (5U)
#define USB_USBTRC0_USBRESMEN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USBRESMEN_SHIFT)) & USB_USBTRC0_USBRESMEN_MASK)
#define USB_USBTRC0_USBRESET_MASK                (0x80U)
#define USB_USBTRC0_USBRESET_SHIFT               (7U)
#define USB_USBTRC0_USBRESET(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBTRC0_USBRESET_SHIFT)) & USB_USBTRC0_USBRESET_MASK)

/*! @name USBFRMADJUST - Frame Adjust Register */
#define USB_USBFRMADJUST_ADJ_MASK                (0xFFU)
#define USB_USBFRMADJUST_ADJ_SHIFT               (0U)
#define USB_USBFRMADJUST_ADJ(x)                  (((uint8_t)(((uint8_t)(x)) << USB_USBFRMADJUST_ADJ_SHIFT)) & USB_USBFRMADJUST_ADJ_MASK)

/*! @name MISCCTRL - Miscellaneous Control register */
#define USB_MISCCTRL_SOFDYNTHLD_MASK             (0x1U)
#define USB_MISCCTRL_SOFDYNTHLD_SHIFT            (0U)
#define USB_MISCCTRL_SOFDYNTHLD(x)               (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_SOFDYNTHLD_SHIFT)) & USB_MISCCTRL_SOFDYNTHLD_MASK)
#define USB_MISCCTRL_SOFBUSSET_MASK              (0x2U)
#define USB_MISCCTRL_SOFBUSSET_SHIFT             (1U)
#define USB_MISCCTRL_SOFBUSSET(x)                (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_SOFBUSSET_SHIFT)) & USB_MISCCTRL_SOFBUSSET_MASK)
#define USB_MISCCTRL_OWNERRISODIS_MASK           (0x4U)
#define USB_MISCCTRL_OWNERRISODIS_SHIFT          (2U)
#define USB_MISCCTRL_OWNERRISODIS(x)             (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_OWNERRISODIS_SHIFT)) & USB_MISCCTRL_OWNERRISODIS_MASK)
#define USB_MISCCTRL_VREDG_EN_MASK               (0x8U)
#define USB_MISCCTRL_VREDG_EN_SHIFT              (3U)
#define USB_MISCCTRL_VREDG_EN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_VREDG_EN_SHIFT)) & USB_MISCCTRL_VREDG_EN_MASK)
#define USB_MISCCTRL_VFEDG_EN_MASK               (0x10U)
#define USB_MISCCTRL_VFEDG_EN_SHIFT              (4U)
#define USB_MISCCTRL_VFEDG_EN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_MISCCTRL_VFEDG_EN_SHIFT)) & USB_MISCCTRL_VFEDG_EN_MASK)

/*! @name CLK_RECOVER_CTRL - USB Clock recovery control */
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK (0x20U)
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT (5U)
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK (0x40U)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT (6U)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK (0x80U)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT (7U)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN(x) (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK)

/*! @name CLK_RECOVER_IRC_EN - IRC48M oscillator enable register */
#define USB_CLK_RECOVER_IRC_EN_REG_EN_MASK       (0x1U)
#define USB_CLK_RECOVER_IRC_EN_REG_EN_SHIFT      (0U)
#define USB_CLK_RECOVER_IRC_EN_REG_EN(x)         (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_IRC_EN_REG_EN_SHIFT)) & USB_CLK_RECOVER_IRC_EN_REG_EN_MASK)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK       (0x2U)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT      (1U)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN(x)         (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT)) & USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK)

/*! @name CLK_RECOVER_INT_EN - Clock recovery combined interrupt enable */
#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_MASK (0x10U)
#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_SHIFT (4U)
#define USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN(x)   (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_SHIFT)) & USB_CLK_RECOVER_INT_EN_OVF_ERROR_EN_MASK)

/*! @name CLK_RECOVER_INT_STATUS - Clock recovery separated interrupt status */
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK (0x10U)
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT (4U)
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR(x)  (((uint8_t)(((uint8_t)(x)) << USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT)) & USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK)


/*!
 * @}
 */ /* end of group USB_Register_Masks */


/* USB - Peripheral instance base addresses */
/** Peripheral USB0 base address */
#define USB0_BASE                                (0x40072000u)
/** Peripheral USB0 base pointer */
#define USB0                                     ((USB_Type *)USB0_BASE)
/** Array initializer of USB peripheral base addresses */
#define USB_BASE_ADDRS                           { USB0_BASE }
/** Array initializer of USB peripheral base pointers */
#define USB_BASE_PTRS                            { USB0 }
/** Interrupt vectors for the USB peripheral type */
#define USB_IRQS                                 { USB0_IRQn }

/*!
 * @}
 */ /* end of group USB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- USBDCD Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USBDCD_Peripheral_Access_Layer USBDCD Peripheral Access Layer
 * @{
 */

/** USBDCD - Register Layout Typedef */
typedef struct {
  __IO uint32_t CONTROL;                           /**< Control register, offset: 0x0 */
  __IO uint32_t CLOCK;                             /**< Clock register, offset: 0x4 */
  __I  uint32_t STATUS;                            /**< Status register, offset: 0x8 */
  __IO uint32_t SIGNAL_OVERRIDE;                   /**< Signal Override Register, offset: 0xC */
  __IO uint32_t TIMER0;                            /**< TIMER0 register, offset: 0x10 */
  __IO uint32_t TIMER1;                            /**< TIMER1 register, offset: 0x14 */
  union {                                          /* offset: 0x18 */
    __IO uint32_t TIMER2_BC11;                       /**< TIMER2_BC11 register, offset: 0x18 */
    __IO uint32_t TIMER2_BC12;                       /**< TIMER2_BC12 register, offset: 0x18 */
  };
} USBDCD_Type;

/* ----------------------------------------------------------------------------
   -- USBDCD Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USBDCD_Register_Masks USBDCD Register Masks
 * @{
 */

/*! @name CONTROL - Control register */
#define USBDCD_CONTROL_IACK_MASK                 (0x1U)
#define USBDCD_CONTROL_IACK_SHIFT                (0U)
#define USBDCD_CONTROL_IACK(x)                   (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_IACK_SHIFT)) & USBDCD_CONTROL_IACK_MASK)
#define USBDCD_CONTROL_IF_MASK                   (0x100U)
#define USBDCD_CONTROL_IF_SHIFT                  (8U)
#define USBDCD_CONTROL_IF(x)                     (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_IF_SHIFT)) & USBDCD_CONTROL_IF_MASK)
#define USBDCD_CONTROL_IE_MASK                   (0x10000U)
#define USBDCD_CONTROL_IE_SHIFT                  (16U)
#define USBDCD_CONTROL_IE(x)                     (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_IE_SHIFT)) & USBDCD_CONTROL_IE_MASK)
#define USBDCD_CONTROL_BC12_MASK                 (0x20000U)
#define USBDCD_CONTROL_BC12_SHIFT                (17U)
#define USBDCD_CONTROL_BC12(x)                   (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_BC12_SHIFT)) & USBDCD_CONTROL_BC12_MASK)
#define USBDCD_CONTROL_START_MASK                (0x1000000U)
#define USBDCD_CONTROL_START_SHIFT               (24U)
#define USBDCD_CONTROL_START(x)                  (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_START_SHIFT)) & USBDCD_CONTROL_START_MASK)
#define USBDCD_CONTROL_SR_MASK                   (0x2000000U)
#define USBDCD_CONTROL_SR_SHIFT                  (25U)
#define USBDCD_CONTROL_SR(x)                     (((uint32_t)(((uint32_t)(x)) << USBDCD_CONTROL_SR_SHIFT)) & USBDCD_CONTROL_SR_MASK)

/*! @name CLOCK - Clock register */
#define USBDCD_CLOCK_CLOCK_UNIT_MASK             (0x1U)
#define USBDCD_CLOCK_CLOCK_UNIT_SHIFT            (0U)
#define USBDCD_CLOCK_CLOCK_UNIT(x)               (((uint32_t)(((uint32_t)(x)) << USBDCD_CLOCK_CLOCK_UNIT_SHIFT)) & USBDCD_CLOCK_CLOCK_UNIT_MASK)
#define USBDCD_CLOCK_CLOCK_SPEED_MASK            (0xFFCU)
#define USBDCD_CLOCK_CLOCK_SPEED_SHIFT           (2U)
#define USBDCD_CLOCK_CLOCK_SPEED(x)              (((uint32_t)(((uint32_t)(x)) << USBDCD_CLOCK_CLOCK_SPEED_SHIFT)) & USBDCD_CLOCK_CLOCK_SPEED_MASK)

/*! @name STATUS - Status register */
#define USBDCD_STATUS_SEQ_RES_MASK               (0x30000U)
#define USBDCD_STATUS_SEQ_RES_SHIFT              (16U)
#define USBDCD_STATUS_SEQ_RES(x)                 (((uint32_t)(((uint32_t)(x)) << USBDCD_STATUS_SEQ_RES_SHIFT)) & USBDCD_STATUS_SEQ_RES_MASK)
#define USBDCD_STATUS_SEQ_STAT_MASK              (0xC0000U)
#define USBDCD_STATUS_SEQ_STAT_SHIFT             (18U)
#define USBDCD_STATUS_SEQ_STAT(x)                (((uint32_t)(((uint32_t)(x)) << USBDCD_STATUS_SEQ_STAT_SHIFT)) & USBDCD_STATUS_SEQ_STAT_MASK)
#define USBDCD_STATUS_ERR_MASK                   (0x100000U)
#define USBDCD_STATUS_ERR_SHIFT                  (20U)
#define USBDCD_STATUS_ERR(x)                     (((uint32_t)(((uint32_t)(x)) << USBDCD_STATUS_ERR_SHIFT)) & USBDCD_STATUS_ERR_MASK)
#define USBDCD_STATUS_TO_MASK                    (0x200000U)
#define USBDCD_STATUS_TO_SHIFT                   (21U)
#define USBDCD_STATUS_TO(x)                      (((uint32_t)(((uint32_t)(x)) << USBDCD_STATUS_TO_SHIFT)) & USBDCD_STATUS_TO_MASK)
#define USBDCD_STATUS_ACTIVE_MASK                (0x400000U)
#define USBDCD_STATUS_ACTIVE_SHIFT               (22U)
#define USBDCD_STATUS_ACTIVE(x)                  (((uint32_t)(((uint32_t)(x)) << USBDCD_STATUS_ACTIVE_SHIFT)) & USBDCD_STATUS_ACTIVE_MASK)

/*! @name SIGNAL_OVERRIDE - Signal Override Register */
#define USBDCD_SIGNAL_OVERRIDE_PS_MASK           (0x3U)
#define USBDCD_SIGNAL_OVERRIDE_PS_SHIFT          (0U)
#define USBDCD_SIGNAL_OVERRIDE_PS(x)             (((uint32_t)(((uint32_t)(x)) << USBDCD_SIGNAL_OVERRIDE_PS_SHIFT)) & USBDCD_SIGNAL_OVERRIDE_PS_MASK)

/*! @name TIMER0 - TIMER0 register */
#define USBDCD_TIMER0_TUNITCON_MASK              (0xFFFU)
#define USBDCD_TIMER0_TUNITCON_SHIFT             (0U)
#define USBDCD_TIMER0_TUNITCON(x)                (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER0_TUNITCON_SHIFT)) & USBDCD_TIMER0_TUNITCON_MASK)
#define USBDCD_TIMER0_TSEQ_INIT_MASK             (0x3FF0000U)
#define USBDCD_TIMER0_TSEQ_INIT_SHIFT            (16U)
#define USBDCD_TIMER0_TSEQ_INIT(x)               (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER0_TSEQ_INIT_SHIFT)) & USBDCD_TIMER0_TSEQ_INIT_MASK)

/*! @name TIMER1 - TIMER1 register */
#define USBDCD_TIMER1_TVDPSRC_ON_MASK            (0x3FFU)
#define USBDCD_TIMER1_TVDPSRC_ON_SHIFT           (0U)
#define USBDCD_TIMER1_TVDPSRC_ON(x)              (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER1_TVDPSRC_ON_SHIFT)) & USBDCD_TIMER1_TVDPSRC_ON_MASK)
#define USBDCD_TIMER1_TDCD_DBNC_MASK             (0x3FF0000U)
#define USBDCD_TIMER1_TDCD_DBNC_SHIFT            (16U)
#define USBDCD_TIMER1_TDCD_DBNC(x)               (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER1_TDCD_DBNC_SHIFT)) & USBDCD_TIMER1_TDCD_DBNC_MASK)

/*! @name TIMER2_BC11 - TIMER2_BC11 register */
#define USBDCD_TIMER2_BC11_CHECK_DM_MASK         (0xFU)
#define USBDCD_TIMER2_BC11_CHECK_DM_SHIFT        (0U)
#define USBDCD_TIMER2_BC11_CHECK_DM(x)           (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER2_BC11_CHECK_DM_SHIFT)) & USBDCD_TIMER2_BC11_CHECK_DM_MASK)
#define USBDCD_TIMER2_BC11_TVDPSRC_CON_MASK      (0x3FF0000U)
#define USBDCD_TIMER2_BC11_TVDPSRC_CON_SHIFT     (16U)
#define USBDCD_TIMER2_BC11_TVDPSRC_CON(x)        (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER2_BC11_TVDPSRC_CON_SHIFT)) & USBDCD_TIMER2_BC11_TVDPSRC_CON_MASK)

/*! @name TIMER2_BC12 - TIMER2_BC12 register */
#define USBDCD_TIMER2_BC12_TVDMSRC_ON_MASK       (0x3FFU)
#define USBDCD_TIMER2_BC12_TVDMSRC_ON_SHIFT      (0U)
#define USBDCD_TIMER2_BC12_TVDMSRC_ON(x)         (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER2_BC12_TVDMSRC_ON_SHIFT)) & USBDCD_TIMER2_BC12_TVDMSRC_ON_MASK)
#define USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_MASK  (0x3FF0000U)
#define USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_SHIFT (16U)
#define USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD(x)    (((uint32_t)(((uint32_t)(x)) << USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_SHIFT)) & USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_MASK)


/*!
 * @}
 */ /* end of group USBDCD_Register_Masks */


/* USBDCD - Peripheral instance base addresses */
/** Peripheral USBDCD base address */
#define USBDCD_BASE                              (0x40035000u)
/** Peripheral USBDCD base pointer */
#define USBDCD                                   ((USBDCD_Type *)USBDCD_BASE)
/** Array initializer of USBDCD peripheral base addresses */
#define USBDCD_BASE_ADDRS                        { USBDCD_BASE }
/** Array initializer of USBDCD peripheral base pointers */
#define USBDCD_BASE_PTRS                         { USBDCD }
/** Interrupt vectors for the USBDCD peripheral type */
#define USBDCD_IRQS                              { USBDCD_IRQn }

/*!
 * @}
 */ /* end of group USBDCD_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- VREF Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Peripheral_Access_Layer VREF Peripheral Access Layer
 * @{
 */

/** VREF - Register Layout Typedef */
typedef struct {
  __IO uint8_t TRM;                                /**< VREF Trim Register, offset: 0x0 */
  __IO uint8_t SC;                                 /**< VREF Status and Control Register, offset: 0x1 */
} VREF_Type;

/* ----------------------------------------------------------------------------
   -- VREF Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Register_Masks VREF Register Masks
 * @{
 */

/*! @name TRM - VREF Trim Register */
#define VREF_TRM_TRIM_MASK                       (0x3FU)
#define VREF_TRM_TRIM_SHIFT                      (0U)
#define VREF_TRM_TRIM(x)                         (((uint8_t)(((uint8_t)(x)) << VREF_TRM_TRIM_SHIFT)) & VREF_TRM_TRIM_MASK)
#define VREF_TRM_CHOPEN_MASK                     (0x40U)
#define VREF_TRM_CHOPEN_SHIFT                    (6U)
#define VREF_TRM_CHOPEN(x)                       (((uint8_t)(((uint8_t)(x)) << VREF_TRM_CHOPEN_SHIFT)) & VREF_TRM_CHOPEN_MASK)

/*! @name SC - VREF Status and Control Register */
#define VREF_SC_MODE_LV_MASK                     (0x3U)
#define VREF_SC_MODE_LV_SHIFT                    (0U)
#define VREF_SC_MODE_LV(x)                       (((uint8_t)(((uint8_t)(x)) << VREF_SC_MODE_LV_SHIFT)) & VREF_SC_MODE_LV_MASK)
#define VREF_SC_VREFST_MASK                      (0x4U)
#define VREF_SC_VREFST_SHIFT                     (2U)
#define VREF_SC_VREFST(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_VREFST_SHIFT)) & VREF_SC_VREFST_MASK)
#define VREF_SC_ICOMPEN_MASK                     (0x20U)
#define VREF_SC_ICOMPEN_SHIFT                    (5U)
#define VREF_SC_ICOMPEN(x)                       (((uint8_t)(((uint8_t)(x)) << VREF_SC_ICOMPEN_SHIFT)) & VREF_SC_ICOMPEN_MASK)
#define VREF_SC_REGEN_MASK                       (0x40U)
#define VREF_SC_REGEN_SHIFT                      (6U)
#define VREF_SC_REGEN(x)                         (((uint8_t)(((uint8_t)(x)) << VREF_SC_REGEN_SHIFT)) & VREF_SC_REGEN_MASK)
#define VREF_SC_VREFEN_MASK                      (0x80U)
#define VREF_SC_VREFEN_SHIFT                     (7U)
#define VREF_SC_VREFEN(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_VREFEN_SHIFT)) & VREF_SC_VREFEN_MASK)


/*!
 * @}
 */ /* end of group VREF_Register_Masks */


/* VREF - Peripheral instance base addresses */
/** Peripheral VREF base address */
#define VREF_BASE                                (0x40074000u)
/** Peripheral VREF base pointer */
#define VREF                                     ((VREF_Type *)VREF_BASE)
/** Array initializer of VREF peripheral base addresses */
#define VREF_BASE_ADDRS                          { VREF_BASE }
/** Array initializer of VREF peripheral base pointers */
#define VREF_BASE_PTRS                           { VREF }

/*!
 * @}
 */ /* end of group VREF_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- WDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral_Access_Layer WDOG Peripheral Access Layer
 * @{
 */

/** WDOG - Register Layout Typedef */
typedef struct {
  __IO uint16_t STCTRLH;                           /**< Watchdog Status and Control Register High, offset: 0x0 */
  __IO uint16_t STCTRLL;                           /**< Watchdog Status and Control Register Low, offset: 0x2 */
  __IO uint16_t TOVALH;                            /**< Watchdog Time-out Value Register High, offset: 0x4 */
  __IO uint16_t TOVALL;                            /**< Watchdog Time-out Value Register Low, offset: 0x6 */
  __IO uint16_t WINH;                              /**< Watchdog Window Register High, offset: 0x8 */
  __IO uint16_t WINL;                              /**< Watchdog Window Register Low, offset: 0xA */
  __IO uint16_t REFRESH;                           /**< Watchdog Refresh register, offset: 0xC */
  __IO uint16_t UNLOCK;                            /**< Watchdog Unlock register, offset: 0xE */
  __IO uint16_t TMROUTH;                           /**< Watchdog Timer Output Register High, offset: 0x10 */
  __IO uint16_t TMROUTL;                           /**< Watchdog Timer Output Register Low, offset: 0x12 */
  __IO uint16_t RSTCNT;                            /**< Watchdog Reset Count register, offset: 0x14 */
  __IO uint16_t PRESC;                             /**< Watchdog Prescaler register, offset: 0x16 */
} WDOG_Type;

/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/*! @name STCTRLH - Watchdog Status and Control Register High */
#define WDOG_STCTRLH_WDOGEN_MASK                 (0x1U)
#define WDOG_STCTRLH_WDOGEN_SHIFT                (0U)
#define WDOG_STCTRLH_WDOGEN(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_WDOGEN_SHIFT)) & WDOG_STCTRLH_WDOGEN_MASK)
#define WDOG_STCTRLH_CLKSRC_MASK                 (0x2U)
#define WDOG_STCTRLH_CLKSRC_SHIFT                (1U)
#define WDOG_STCTRLH_CLKSRC(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_CLKSRC_SHIFT)) & WDOG_STCTRLH_CLKSRC_MASK)
#define WDOG_STCTRLH_IRQRSTEN_MASK               (0x4U)
#define WDOG_STCTRLH_IRQRSTEN_SHIFT              (2U)
#define WDOG_STCTRLH_IRQRSTEN(x)                 (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_IRQRSTEN_SHIFT)) & WDOG_STCTRLH_IRQRSTEN_MASK)
#define WDOG_STCTRLH_WINEN_MASK                  (0x8U)
#define WDOG_STCTRLH_WINEN_SHIFT                 (3U)
#define WDOG_STCTRLH_WINEN(x)                    (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_WINEN_SHIFT)) & WDOG_STCTRLH_WINEN_MASK)
#define WDOG_STCTRLH_ALLOWUPDATE_MASK            (0x10U)
#define WDOG_STCTRLH_ALLOWUPDATE_SHIFT           (4U)
#define WDOG_STCTRLH_ALLOWUPDATE(x)              (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_ALLOWUPDATE_SHIFT)) & WDOG_STCTRLH_ALLOWUPDATE_MASK)
#define WDOG_STCTRLH_DBGEN_MASK                  (0x20U)
#define WDOG_STCTRLH_DBGEN_SHIFT                 (5U)
#define WDOG_STCTRLH_DBGEN(x)                    (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_DBGEN_SHIFT)) & WDOG_STCTRLH_DBGEN_MASK)
#define WDOG_STCTRLH_STOPEN_MASK                 (0x40U)
#define WDOG_STCTRLH_STOPEN_SHIFT                (6U)
#define WDOG_STCTRLH_STOPEN(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_STOPEN_SHIFT)) & WDOG_STCTRLH_STOPEN_MASK)
#define WDOG_STCTRLH_WAITEN_MASK                 (0x80U)
#define WDOG_STCTRLH_WAITEN_SHIFT                (7U)
#define WDOG_STCTRLH_WAITEN(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_WAITEN_SHIFT)) & WDOG_STCTRLH_WAITEN_MASK)
#define WDOG_STCTRLH_TESTWDOG_MASK               (0x400U)
#define WDOG_STCTRLH_TESTWDOG_SHIFT              (10U)
#define WDOG_STCTRLH_TESTWDOG(x)                 (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_TESTWDOG_SHIFT)) & WDOG_STCTRLH_TESTWDOG_MASK)
#define WDOG_STCTRLH_TESTSEL_MASK                (0x800U)
#define WDOG_STCTRLH_TESTSEL_SHIFT               (11U)
#define WDOG_STCTRLH_TESTSEL(x)                  (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_TESTSEL_SHIFT)) & WDOG_STCTRLH_TESTSEL_MASK)
#define WDOG_STCTRLH_BYTESEL_MASK                (0x3000U)
#define WDOG_STCTRLH_BYTESEL_SHIFT               (12U)
#define WDOG_STCTRLH_BYTESEL(x)                  (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_BYTESEL_SHIFT)) & WDOG_STCTRLH_BYTESEL_MASK)
#define WDOG_STCTRLH_DISTESTWDOG_MASK            (0x4000U)
#define WDOG_STCTRLH_DISTESTWDOG_SHIFT           (14U)
#define WDOG_STCTRLH_DISTESTWDOG(x)              (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLH_DISTESTWDOG_SHIFT)) & WDOG_STCTRLH_DISTESTWDOG_MASK)

/*! @name STCTRLL - Watchdog Status and Control Register Low */
#define WDOG_STCTRLL_INTFLG_MASK                 (0x8000U)
#define WDOG_STCTRLL_INTFLG_SHIFT                (15U)
#define WDOG_STCTRLL_INTFLG(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_STCTRLL_INTFLG_SHIFT)) & WDOG_STCTRLL_INTFLG_MASK)

/*! @name TOVALH - Watchdog Time-out Value Register High */
#define WDOG_TOVALH_TOVALHIGH_MASK               (0xFFFFU)
#define WDOG_TOVALH_TOVALHIGH_SHIFT              (0U)
#define WDOG_TOVALH_TOVALHIGH(x)                 (((uint16_t)(((uint16_t)(x)) << WDOG_TOVALH_TOVALHIGH_SHIFT)) & WDOG_TOVALH_TOVALHIGH_MASK)

/*! @name TOVALL - Watchdog Time-out Value Register Low */
#define WDOG_TOVALL_TOVALLOW_MASK                (0xFFFFU)
#define WDOG_TOVALL_TOVALLOW_SHIFT               (0U)
#define WDOG_TOVALL_TOVALLOW(x)                  (((uint16_t)(((uint16_t)(x)) << WDOG_TOVALL_TOVALLOW_SHIFT)) & WDOG_TOVALL_TOVALLOW_MASK)

/*! @name WINH - Watchdog Window Register High */
#define WDOG_WINH_WINHIGH_MASK                   (0xFFFFU)
#define WDOG_WINH_WINHIGH_SHIFT                  (0U)
#define WDOG_WINH_WINHIGH(x)                     (((uint16_t)(((uint16_t)(x)) << WDOG_WINH_WINHIGH_SHIFT)) & WDOG_WINH_WINHIGH_MASK)

/*! @name WINL - Watchdog Window Register Low */
#define WDOG_WINL_WINLOW_MASK                    (0xFFFFU)
#define WDOG_WINL_WINLOW_SHIFT                   (0U)
#define WDOG_WINL_WINLOW(x)                      (((uint16_t)(((uint16_t)(x)) << WDOG_WINL_WINLOW_SHIFT)) & WDOG_WINL_WINLOW_MASK)

/*! @name REFRESH - Watchdog Refresh register */
#define WDOG_REFRESH_WDOGREFRESH_MASK            (0xFFFFU)
#define WDOG_REFRESH_WDOGREFRESH_SHIFT           (0U)
#define WDOG_REFRESH_WDOGREFRESH(x)              (((uint16_t)(((uint16_t)(x)) << WDOG_REFRESH_WDOGREFRESH_SHIFT)) & WDOG_REFRESH_WDOGREFRESH_MASK)

/*! @name UNLOCK - Watchdog Unlock register */
#define WDOG_UNLOCK_WDOGUNLOCK_MASK              (0xFFFFU)
#define WDOG_UNLOCK_WDOGUNLOCK_SHIFT             (0U)
#define WDOG_UNLOCK_WDOGUNLOCK(x)                (((uint16_t)(((uint16_t)(x)) << WDOG_UNLOCK_WDOGUNLOCK_SHIFT)) & WDOG_UNLOCK_WDOGUNLOCK_MASK)

/*! @name TMROUTH - Watchdog Timer Output Register High */
#define WDOG_TMROUTH_TIMEROUTHIGH_MASK           (0xFFFFU)
#define WDOG_TMROUTH_TIMEROUTHIGH_SHIFT          (0U)
#define WDOG_TMROUTH_TIMEROUTHIGH(x)             (((uint16_t)(((uint16_t)(x)) << WDOG_TMROUTH_TIMEROUTHIGH_SHIFT)) & WDOG_TMROUTH_TIMEROUTHIGH_MASK)

/*! @name TMROUTL - Watchdog Timer Output Register Low */
#define WDOG_TMROUTL_TIMEROUTLOW_MASK            (0xFFFFU)
#define WDOG_TMROUTL_TIMEROUTLOW_SHIFT           (0U)
#define WDOG_TMROUTL_TIMEROUTLOW(x)              (((uint16_t)(((uint16_t)(x)) << WDOG_TMROUTL_TIMEROUTLOW_SHIFT)) & WDOG_TMROUTL_TIMEROUTLOW_MASK)

/*! @name RSTCNT - Watchdog Reset Count register */
#define WDOG_RSTCNT_RSTCNT_MASK                  (0xFFFFU)
#define WDOG_RSTCNT_RSTCNT_SHIFT                 (0U)
#define WDOG_RSTCNT_RSTCNT(x)                    (((uint16_t)(((uint16_t)(x)) << WDOG_RSTCNT_RSTCNT_SHIFT)) & WDOG_RSTCNT_RSTCNT_MASK)

/*! @name PRESC - Watchdog Prescaler register */
#define WDOG_PRESC_PRESCVAL_MASK                 (0x700U)
#define WDOG_PRESC_PRESCVAL_SHIFT                (8U)
#define WDOG_PRESC_PRESCVAL(x)                   (((uint16_t)(((uint16_t)(x)) << WDOG_PRESC_PRESCVAL_SHIFT)) & WDOG_PRESC_PRESCVAL_MASK)


/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG base address */
#define WDOG_BASE                                (0x40052000u)
/** Peripheral WDOG base pointer */
#define WDOG                                     ((WDOG_Type *)WDOG_BASE)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { WDOG_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG }
/** Interrupt vectors for the WDOG peripheral type */
#define WDOG_IRQS                                { WDOG_EWM_IRQn }

/*!
 * @}
 */ /* end of group WDOG_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

#define PIT0_IRQn                                PIT0CH0_IRQn
#define PIT1_IRQn                                PIT0CH1_IRQn
#define PIT2_IRQn                                PIT0CH2_IRQn
#define PIT3_IRQn                                PIT0CH3_IRQn
#define PIT_BASE                                 PIT0_BASE
#define PIT                                      PIT0
#define PIT_MCR                                  PIT0_MCR
#define PIT_LDVAL0                               PIT0_LDVAL0
#define PIT_CVAL0                                PIT0_CVAL0
#define PIT_TCTRL0                               PIT0_TCTRL0
#define PIT_TFLG0                                PIT0_TFLG0
#define PIT_LDVAL1                               PIT0_LDVAL1
#define PIT_CVAL1                                PIT0_CVAL1
#define PIT_TCTRL1                               PIT0_TCTRL1
#define PIT_TFLG1                                PIT0_TFLG1
#define PIT_LDVAL2                               PIT0_LDVAL2
#define PIT_CVAL2                                PIT0_CVAL2
#define PIT_TCTRL2                               PIT0_TCTRL2
#define PIT_TFLG2                                PIT0_TFLG2
#define PIT_LDVAL3                               PIT0_LDVAL3
#define PIT_CVAL3                                PIT0_CVAL3
#define PIT_TCTRL3                               PIT0_TCTRL3
#define PIT_TFLG3                                PIT0_TFLG3
#define PIT_LDVAL(index)                         PIT0_LDVAL(index)
#define PIT_CVAL(index)                          PIT0_CVAL(index)
#define PIT_TCTRL(index)                         PIT0_TCTRL(index)
#define PIT_TFLG(index)                          PIT0_TFLG(index)
#define PIT0_IRQHandler                          PIT0CH0_IRQHandler
#define PIT1_IRQHandler                          PIT0CH1_IRQHandler
#define PIT2_IRQHandler                          PIT0CH2_IRQHandler
#define PIT3_IRQHandler                          PIT0CH3_IRQHandler
#define DSPI0                                    SPI0
#define DSPI1                                    SPI1
#define DSPI2                                    SPI2
#define DMAMUX0                                  DMAMUX

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MK80F25615_H_ */

