/*
** ###################################################################
**     Processors:          MKL28Z512VDC7
**                          MKL28Z512VLL7
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    MKL28ZRM, Rev. 2, March, 2016
**     Version:             rev. 1.12, 2015-06-03
**     Build:               b160314
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MKL28Z7
**
**     Copyright (c) 1997 - 2016 Freescale Semiconductor, Inc.
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
**     - rev. 1.0 (2014-12-29)
**         Initial version.
**     - rev. 1.1 (2015-03-19)
**         Make array for registers in XRDC, PCC and TRGMUX
**     - rev. 1.2 (2015-03-20)
**         Add vector table for intmux0
**         Add IRQS for SCG, RCM, DAC, TSI
**     - rev. 1.3 (2015-03-26)
**         Update FGPIO and GPIO
**         Update IRQ for LPIT
**     - rev. 1.4 (2015-04-08)
**         Group channel registers for LPIT
**     - rev. 1.5 (2015-04-15)
**         Group channel registers for INTMUX
**     - rev. 1.6 (2015-05-06)
**         Correct FOPT reset value
**         Correct vector table size
**         Remove parts with 256K flash size
**     - rev. 1.7 (2015-05-08)
**         Reduce register for XRDC
**     - rev. 1.8 (2015-05-12)
**         Add CMP
**         DMAMUX channel count to 8
**         Add PIDR for GPIO/FGPIO
**         Rename PIT to LPIT in PCC
**         SCG, USB register update
**         SIM, TRGMUX1, TRNG, TSTMR0/1 base address update
**         Add KEY related macros for WDOG
**     - rev. 1.9 (2015-05-16)
**         Add IRQS
**     - rev. 1.10 (2015-05-30)
**         Remove MTB1, MCM1 and MMDVSQ1
**     - rev. 1.11 (2015-06-01)
**         Remove registers for modules which are not available on single in PCC and TRGMUX
**     - rev. 1.12 (2015-06-03)
**         Bit 31 of TRGMUX changed to RW from RO
**
** ###################################################################
*/

/*!
 * @file MKL28Z7.h
 * @version 1.12
 * @date 2015-06-03
 * @brief CMSIS Peripheral Access Layer for MKL28Z7
 *
 * CMSIS Peripheral Access Layer for MKL28Z7
 */

#ifndef _MKL28Z7_H_
#define _MKL28Z7_H_                              /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0100U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x000CU


/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 80                 /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M0 SV Hard Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M0 SV Call Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M0 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M0 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_04_IRQn                 = 0,                /**< DMA0 channel 0/4 transfer complete */
  DMA0_15_IRQn                 = 1,                /**< DMA0 channel 1/5 transfer complete */
  DMA0_26_IRQn                 = 2,                /**< DMA0 channel 2/6 transfer complete */
  DMA0_37_IRQn                 = 3,                /**< DMA0 channel 3/7 transfer complete */
  CTI0_DMA0_Error_IRQn         = 4,                /**< CTI0 or DMA0 error */
  FLEXIO0_IRQn                 = 5,                /**< FLEXIO0 */
  TPM0_IRQn                    = 6,                /**< TPM0 single interrupt vector for all sources */
  TPM1_IRQn                    = 7,                /**< TPM1 single interrupt vector for all sources */
  TPM2_IRQn                    = 8,                /**< TPM2 single interrupt vector for all sources */
  LPIT0_IRQn                   = 9,                /**< LPIT0 interrupt */
  LPSPI0_IRQn                  = 10,               /**< LPSPI0 single interrupt vector for all sources */
  LPSPI1_IRQn                  = 11,               /**< LPSPI1 single interrupt vector for all sources */
  LPUART0_IRQn                 = 12,               /**< LPUART0 status and error */
  LPUART1_IRQn                 = 13,               /**< LPUART1 status and error */
  LPI2C0_IRQn                  = 14,               /**< LPI2C0 interrupt */
  LPI2C1_IRQn                  = 15,               /**< LPI2C1 interrupt */
  Reserved32_IRQn              = 16,               /**< Reserved interrupt */
  PORTA_IRQn                   = 17,               /**< PORTA Pin detect */
  PORTB_IRQn                   = 18,               /**< PORTB Pin detect */
  PORTC_IRQn                   = 19,               /**< PORTC Pin detect */
  PORTD_IRQn                   = 20,               /**< PORTD Pin detect */
  PORTE_IRQn                   = 21,               /**< PORTE Pin detect */
  LLWU0_IRQn                   = 22,               /**< Low leakage wakeup 0 */
  I2S0_IRQn                    = 23,               /**< I2S0 interrupt */
  USB0_IRQn                    = 24,               /**< USB0 interrupt */
  ADC0_IRQn                    = 25,               /**< ADC0 interrupt */
  LPTMR0_IRQn                  = 26,               /**< LPTMR0 interrupt */
  RTC_Seconds_IRQn             = 27,               /**< RTC seconds */
  INTMUX0_0_IRQn               = 28,               /**< INTMUX0 channel 0 interrupt */
  INTMUX0_1_IRQn               = 29,               /**< INTMUX0 channel 1 interrupt */
  INTMUX0_2_IRQn               = 30,               /**< INTMUX0 channel 2 interrupt */
  INTMUX0_3_IRQn               = 31,               /**< INTMUX0 channel 3 interrupt */
  LPTMR1_IRQn                  = 32,               /**< LPTMR1 interrupt  (INTMUX source IRQ0) */
  Reserved49_IRQn              = 33,               /**< Reserved interrupt */
  Reserved50_IRQn              = 34,               /**< Reserved interrupt */
  Reserved51_IRQn              = 35,               /**< Reserved interrupt */
  LPSPI2_IRQn                  = 36,               /**< LPSPI2 single interrupt vector for all sources (INTMUX source IRQ4) */
  LPUART2_IRQn                 = 37,               /**< LPUART2 status and error (INTMUX source IRQ5) */
  EMVSIM0_IRQn                 = 38,               /**< EMVSIM0 interrupt (INTMUX source IRQ6) */
  LPI2C2_IRQn                  = 39,               /**< LPI2C2 interrupt (INTMUX source IRQ7) */
  TSI0_IRQn                    = 40,               /**< TSI0 interrupt (INTMUX source IRQ8) */
  PMC_IRQn                     = 41,               /**< PMC interrupt (INTMUX source IRQ9) */
  FTFA_IRQn                    = 42,               /**< FTFA interrupt (INTMUX source IRQ10) */
  SCG_IRQn                     = 43,               /**< SCG interrupt (INTMUX source IRQ11) */
  WDOG0_IRQn                   = 44,               /**< WDOG0 interrupt (INTMUX source IRQ12) */
  DAC0_IRQn                    = 45,               /**< DAC0 interrupt (INTMUX source IRQ13) */
  TRNG_IRQn                    = 46,               /**< TRNG interrupt (INTMUX source IRQ14) */
  RCM_IRQn                     = 47,               /**< RCM interrupt (INTMUX source IRQ15) */
  CMP0_IRQn                    = 48,               /**< CMP0 interrupt (INTMUX source IRQ16) */
  CMP1_IRQn                    = 49,               /**< CMP1 interrupt (INTMUX source IRQ17) */
  RTC_IRQn                     = 50,               /**< RTC Alarm interrupt (INTMUX source IRQ18) */
  Reserved67_IRQn              = 51,               /**< Reserved interrupt */
  Reserved68_IRQn              = 52,               /**< Reserved interrupt */
  Reserved69_IRQn              = 53,               /**< Reserved interrupt */
  Reserved70_IRQn              = 54,               /**< Reserved interrupt */
  Reserved71_IRQn              = 55,               /**< Reserved interrupt */
  Reserved72_IRQn              = 56,               /**< Reserved interrupt */
  Reserved73_IRQn              = 57,               /**< Reserved interrupt */
  Reserved74_IRQn              = 58,               /**< Reserved interrupt */
  Reserved75_IRQn              = 59,               /**< Reserved interrupt */
  Reserved76_IRQn              = 60,               /**< Reserved interrupt */
  Reserved77_IRQn              = 61,               /**< Reserved interrupt */
  Reserved78_IRQn              = 62,               /**< Reserved interrupt */
  Reserved79_IRQn              = 63                /**< Reserved interrupt */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M0 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M0 Core Configuration
 * @{
 */

#define __CM0PLUS_REV                  0x0000    /**< Core revision r0p0 */
#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __VTOR_PRESENT                 1         /**< Defines if VTOR is present or not */
#define __NVIC_PRIO_BITS               2         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */

#include "core_cm0plus.h"              /* Core Peripheral Access Layer */
#include "system_MKL28Z7.h"            /* Device specific configuration file */

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
    kDmaRequestMux0FlexIO0Channel0  = 1|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel1  = 2|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel2  = 3|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel3  = 4|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel4  = 5|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel5  = 6|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel6  = 7|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0FlexIO0Channel7  = 8|0x100U,    /**< FLEXIO0. */
    kDmaRequestMux0LPI2C0Rx         = 9|0x100U,    /**< LPI2C0 Receive. */
    kDmaRequestMux0LPI2C0Tx         = 10|0x100U,   /**< LPI2C0 Transmit. */
    kDmaRequestMux0LPI2C1Rx         = 11|0x100U,   /**< LPI2C1 Receive. */
    kDmaRequestMux0LPI2C1Tx         = 12|0x100U,   /**< LPI2C1 Transmit. */
    kDmaRequestMux0LPI2C2Rx         = 13|0x100U,   /**< LPI2C2 Receive. */
    kDmaRequestMux0LPI2C2Tx         = 14|0x100U,   /**< LPI2C2 Transmit. */
    kDmaRequestMux0LPUART0Rx        = 15|0x100U,   /**< LPUART0 Receive. */
    kDmaRequestMux0LPUART0Tx        = 16|0x100U,   /**< LPUART0 Transmit. */
    kDmaRequestMux0LPUART1Rx        = 17|0x100U,   /**< LPUART1 Receive. */
    kDmaRequestMux0LPUART1Tx        = 18|0x100U,   /**< LPUART1 Transmit. */
    kDmaRequestMux0LPUART2Rx        = 19|0x100U,   /**< LPUART2 Receive. */
    kDmaRequestMux0LPUART2Tx        = 20|0x100U,   /**< LPUART2 Transmit. */
    kDmaRequestMux0LPSPI0Rx         = 21|0x100U,   /**< LPSPI0 Receive. */
    kDmaRequestMux0LPSPI0Tx         = 22|0x100U,   /**< LPSPI0 Transmit. */
    kDmaRequestMux0LPSPI1Rx         = 23|0x100U,   /**< LPSPI1 Receive. */
    kDmaRequestMux0LPSPI1Tx         = 24|0x100U,   /**< LPSPI1 Transmit. */
    kDmaRequestMux0LPSPI2Rx         = 25|0x100U,   /**< LPSPI2 Receive. */
    kDmaRequestMux0LPSPI2Tx         = 26|0x100U,   /**< LPSPI2 Transmit. */
    kDmaRequestMux0TPM0Channel0     = 27|0x100U,   /**< TPM0 C0V Transmit. */
    kDmaRequestMux0TPM0Channel1     = 28|0x100U,   /**< TPM0 C1V Transmit. */
    kDmaRequestMux0TPM0Channel2     = 29|0x100U,   /**< TPM0 C2V Transmit. */
    kDmaRequestMux0TPM0Channel3     = 30|0x100U,   /**< TPM0 C3V Transmit. */
    kDmaRequestMux0TPM0Channel4     = 31|0x100U,   /**< TPM0 C4V Transmit. */
    kDmaRequestMux0TPM0Channel5     = 32|0x100U,   /**< TPM0 C5V Transmit. */
    kDmaRequestMux0Reserved33       = 33|0x100U,   /**< Reserved33 */
    kDmaRequestMux0Reserved34       = 34|0x100U,   /**< Reserved34 */
    kDmaRequestMux0TPM0Overflow     = 35|0x100U,   /**< TPM0. */
    kDmaRequestMux0TPM1Channel0     = 36|0x100U,   /**< TPM1 C0V Transmit. */
    kDmaRequestMux0TPM1Channel1     = 37|0x100U,   /**< TPM1 C1V Transmit. */
    kDmaRequestMux0TPM1Overflow     = 38|0x100U,   /**< TPM1. */
    kDmaRequestMux0TPM2Channel0     = 39|0x100U,   /**< TPM2 C0V Transmit. */
    kDmaRequestMux0TPM2Channel1     = 40|0x100U,   /**< TPM2 C1V Transmit. */
    kDmaRequestMux0TPM2Overflow     = 41|0x100U,   /**< TPM2. */
    kDmaRequestMux0Reserved42       = 42|0x100U,   /**< Reserved42 */
    kDmaRequestMux0EMVSIM0Rx        = 43|0x100U,   /**< EMVSIM0 Receive. */
    kDmaRequestMux0EMVSIM0Tx        = 44|0x100U,   /**< EMVSIM0 Transmit. */
    kDmaRequestMux0I2S0Rx           = 45|0x100U,   /**< I2S0 Receive. */
    kDmaRequestMux0I2S0Tx           = 46|0x100U,   /**< I2S0 Transmit. */
    kDmaRequestMux0PortA            = 47|0x100U,   /**< PTA. */
    kDmaRequestMux0PortB            = 48|0x100U,   /**< PTB. */
    kDmaRequestMux0PortC            = 49|0x100U,   /**< PTC. */
    kDmaRequestMux0PortD            = 50|0x100U,   /**< PTD. */
    kDmaRequestMux0PortE            = 51|0x100U,   /**< PTE. */
    kDmaRequestMux0ADC0             = 52|0x100U,   /**< ADC0. */
    kDmaRequestMux0Reserved53       = 53|0x100U,   /**< Reserved53 */
    kDmaRequestMux0DAC0             = 54|0x100U,   /**< DAC0. */
    kDmaRequestMux0Reserved55       = 55|0x100U,   /**< Reserved55 */
    kDmaRequestMux0CMP0             = 56|0x100U,   /**< CMP0. */
    kDmaRequestMux0CMP1             = 57|0x100U,   /**< CMP1. */
    kDmaRequestMux0Reserved58       = 58|0x100U,   /**< Reserved58 */
    kDmaRequestMux0Reserved59       = 59|0x100U,   /**< Reserved59 */
    kDmaRequestMux0TSI0             = 60|0x100U,   /**< TSI0. */
    kDmaRequestMux0LPTMR0           = 61|0x100U,   /**< LPTMR0. */
    kDmaRequestMux0LPTMR1           = 62|0x100U,   /**< LPTMR1. */
    kDmaRequestMux0AlwaysOn63       = 63|0x100U,   /**< DMAMUX Always Enabled slot. */
} dma_request_source_t;

/* @} */

/*!
 * @addtogroup trgmux_source
 * @{ */

/*******************************************************************************
 * Definitions
*******************************************************************************/

/*!
 * @brief Structure for the TRGMUX source
 *
 * Defines the structure for the TRGMUX source collections.
 */
typedef enum _trgmux_source
{
    kTRGMUX_SourceDisabled          = 0U,          /**< Trigger function is disabled */
    kTRGMUX_SourcePortPin           = 1U,          /**< Port pin trigger intput is selected */
    kTRGMUX_SourceFlexIOTimer0      = 2U,          /**< FlexIO Timer 0 input is selected */
    kTRGMUX_SourceFlexIOTimer1      = 3U,          /**< FlexIO Timer 1 input is selected */
    kTRGMUX_SourceFlexIOTimer2      = 4U,          /**< FlexIO Timer 2 input is selected */
    kTRGMUX_SourceFlexIOTimer3      = 5U,          /**< FlexIO Timer 3 input is selected */
    kTRGMUX_SourceFlexIOTimer4      = 6U,          /**< FlexIO Timer 4 input is selected */
    kTRGMUX_SourceFlexIOTimer5      = 7U,          /**< FlexIO Timer 5 input is selected */
    kTRGMUX_SourceFlexIOTimer6      = 8U,          /**< FlexIO Timer 6 input is selected */
    kTRGMUX_SourceFlexIOTimer7      = 9U,          /**< FlexIO Timer 7 input is selected */
    kTRGMUX_SourceTpm0Overflow      = 10U,         /**< TPM0 Overflow is selected */
    kTRGMUX_SourceTpm0Ch0           = 11U,         /**< TPM0 Channel 0 is selected */
    kTRGMUX_SourceTpm0Ch1           = 12U,         /**< TPM0 Channel 1 is selected */
    kTRGMUX_SourceTpm1Overflow      = 13U,         /**< TPM1 Overflow is selected */
    kTRGMUX_SourceTpm1Ch0           = 14U,         /**< TPM1 Channel 0 is selected */
    kTRGMUX_SourceTpm1Ch1           = 15U,         /**< TPM1 Channel 1 is selected */
    kTRGMUX_SourceLpit1Ch0          = 16U,         /**< LPIT1 Channel 0 is selected */
    kTRGMUX_SourceLpit1Ch1          = 17U,         /**< LPIT1 Channel 1 is selected */
    kTRGMUX_SourceLpit1Ch2          = 18U,         /**< LPIT1 Channel 2 is selected */
    kTRGMUX_SourceLpit1Ch3          = 19U,         /**< LPIT1 Channel 3 is selected */
    kTRGMUX_SourceLpuart0RxData     = 20U,         /**< LPUART0 RX Data is selected */
    kTRGMUX_SourceLpuart0TxData     = 21U,         /**< LPUART0 TX Data is selected */
    kTRGMUX_SourceLpuart0RxIdle     = 22U,         /**< LPUART0 RX Idle is selected */
    kTRGMUX_SourceLpuart1RxData     = 23U,         /**< LPUART1 RX Data is selected */
    kTRGMUX_SourceLpuart1TxData     = 24U,         /**< LPUART1 TX Data is selected */
    kTRGMUX_SourceLpuart1RxIdle     = 25U,         /**< LPUART1 RX Idle is selected */
    kTRGMUX_SourceLpi2c0MasterStop  = 26U,         /**< LPI2C0 Master STOP is selected */
    kTRGMUX_SourceLpi2c0SlaveStop   = 27U,         /**< LPI2C0 Slave STOP is selected */
    kTRGMUX_SourceLpi2c1MasterStop  = 28U,         /**< LPI2C1 Master STOP is selected */
    kTRGMUX_SourceLpi2c1SlaveStop   = 29U,         /**< LPI2C1 Slave STOP is selected */
    kTRGMUX_SourceLpspi0Frame       = 30U,         /**< LPSPI0 Frame is selected */
    kTRGMUX_SourceLpspi0RxData      = 31U,         /**< LPSPI0 RX Data is selected */
    kTRGMUX_SourceLpspi1Frame       = 32U,         /**< LPSPI1 Frame is selected */
    kTRGMUX_SourceLpspi1RxData      = 33U,         /**< LPSPI1 RX Data is selected */
    kTRGMUX_SourceRtcSecCount       = 34U,         /**< RTC Seconds Counter is selected */
    kTRGMUX_SourceRtcAlarm          = 35U,         /**< RTC Alarm is selected */
    kTRGMUX_SourceLptmr0Trg         = 36U,         /**< LPTMR0 Trigger is selected */
    kTRGMUX_SourceLptmr1Trg         = 37U,         /**< LPTMR1 Trigger is selected */
    kTRGMUX_SourceCmp0Output        = 38U,         /**< CMP0 Output is selected */
    kTRGMUX_SourceCmp1Output        = 39U,         /**< CMP1 Output is selected */
    kTRGMUX_SourceAdc0ConvAComplete = 40U,         /**< ADC0 Conversion A Complete is selected */
    kTRGMUX_SourceAdc0ConvBComplete = 41U,         /**< ADC0 Conversion B Complete is selected */
    kTRGMUX_SourcePortAPinTrg       = 42U,         /**< Port A Pin Trigger is selected */
    kTRGMUX_SourcePortBPinTrg       = 43U,         /**< Port B Pin Trigger is selected */
    kTRGMUX_SourcePortCPinTrg       = 44U,         /**< Port C Pin Trigger is selected */
    kTRGMUX_SourcePortDPinTrg       = 45U,         /**< Port D Pin Trigger is selected */
    kTRGMUX_SourcePortEPinTrg       = 46U,         /**< Port E Pin Trigger is selected */
    kTRGMUX_SourceTpm2Overflow      = 47U,         /**< TPM2 Overflow is selected */
    kTRGMUX_SourceTpm2Ch0           = 48U,         /**< TPM2 Channel 0 is selected */
    kTRGMUX_SourceTpm2Ch1           = 49U,         /**< TPM2 Channel 1 is selected */
    kTRGMUX_SourceLpit0Ch0          = 50U,         /**< LPIT0 Channel 0 is selected */
    kTRGMUX_SourceLpit0Ch1          = 51U,         /**< LPIT0 Channel 1 is selected */
    kTRGMUX_SourceLpit0Ch2          = 52U,         /**< LPIT0 Channel 2 is selected */
    kTRGMUX_SourceLpit0Ch3          = 53U,         /**< LPIT0 Channel 3 is selected */
    kTRGMUX_SourceUsbSof            = 54U,         /**< USB Start-of-Frame is selected */
    kTRGMUX_SourceLpuart2RxData     = 55U,         /**< LPUART2 RX Data is selected */
    kTRGMUX_SourceLpuart2TxData     = 56U,         /**< LPUART2 TX Data is selected */
    kTRGMUX_SourceLpuart2RxIdle     = 57U,         /**< LPUART2 RX Idle is selected */
    kTRGMUX_SourceLpi2c2MasterStop  = 58U,         /**< LPI2C2 Master STOP is selected */
    kTRGMUX_SourceLpi2c2SlaveStop   = 59U,         /**< LPI2C2 Slave STOP is selected */
    kTRGMUX_SourceLpspi2Frame       = 60U,         /**< LPSPI2 Frame is selected */
    kTRGMUX_SourceLpspi2RxData      = 61U,         /**< LPSPI2 RX Data is selected */
    kTRGMUX_SourceI2c0TxFrameSync   = 62U,         /**< I2C0 TX Frame Sync is selected */
    kTRGMUX_SourceI2c0RxFrameSync   = 63U,         /**< I2C0 RX Frame Sync is selected */
} trgmux_source_t;

/*!
 * @brief Structure for the TRGMUX device
 *
 * Defines the structure for the TRGMUX device collections.
 */
typedef enum _trgmux_device
{
    kTRGMUX_Trgmux0Dmamux0          = 0U,          /**< DMAMUX0 device trigger input */
    kTRGMUX_Trgmux0Lpit0            = 1U,          /**< LPIT0 device trigger input */
    kTRGMUX_Trgmux0Tpm2             = 2U,          /**< TPM2 device trigger input */
    kTRGMUX_Trgmux1Tpm0             = 2U,          /**< TPM0 device trigger input */
    kTRGMUX_Trgmux0Adc0             = 4U,          /**< ADC0 device trigger input */
    kTRGMUX_Trgmux1Flexio           = 4U,          /**< FLEXIO device trigger input */
    kTRGMUX_Trgmux0Lpuart2          = 5U,          /**< LPUART2 device trigger input */
    kTRGMUX_Trgmux1Lpuart0          = 5U,          /**< LPUART0 device trigger input */
    kTRGMUX_Trgmux0Lpi2c2           = 7U,          /**< LPI2C2 device trigger input */
    kTRGMUX_Trgmux1Lpi2c0           = 7U,          /**< LPI2C0 device trigger input */
    kTRGMUX_Trgmux0Lpspi2           = 9U,          /**< LPSPI2 device trigger input */
    kTRGMUX_Trgmux1Lpsi0            = 9U,          /**< LPSPI0 device trigger input */
    kTRGMUX_Trgmux0Cmp0             = 11U,         /**< CMP0 device trigger input */
    kTRGMUX_Trgmux0Cmp1             = 12U,         /**< CMP1 device trigger input */
    kTRGMUX_Trgmux0Dac0             = 13U,         /**< DAC0 device trigger input */
    kTRGMUX_Trgmux1Tpm1             = 3U,          /**< TPM1 device trigger input */
    kTRGMUX_Trgmux1Lpuart1          = 6U,          /**< LPUART1 device trigger input */
    kTRGMUX_Trgmux1Lpi2c1           = 8U,          /**< LPI2C1 device trigger input */
    kTRGMUX_Trgmux1Lpspi1           = 10U,         /**< LPSPI1 device trigger input */
} trgmux_device_t;

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
#define ADC0_BASE                                (0x40066000u)
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
/** Peripheral CAU0 base address */
#define CAU0_BASE                                (0xF0005000u)
/** Peripheral CAU0 base pointer */
#define CAU0                                     ((CAU_Type *)CAU0_BASE)
/** Array initializer of CAU peripheral base addresses */
#define CAU_BASE_ADDRS                           { CAU0_BASE }
/** Array initializer of CAU peripheral base pointers */
#define CAU_BASE_PTRS                            { CAU0 }

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
#define CMP_MUXCR_PSTM_MASK                      (0x80U)
#define CMP_MUXCR_PSTM_SHIFT                     (7U)
#define CMP_MUXCR_PSTM(x)                        (((uint8_t)(((uint8_t)(x)) << CMP_MUXCR_PSTM_SHIFT)) & CMP_MUXCR_PSTM_MASK)


/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x4006E000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
/** Peripheral CMP1 base address */
#define CMP1_BASE                                (0x400EF000u)
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
#define CRC_BASE                                 (0x40078000u)
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
#define DAC0_BASE                                (0x4006A000u)
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
       uint8_t RESERVED_7[3832];
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
  } TCD[8];
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
#define DMA_CR_ECX_MASK                          (0x10000U)
#define DMA_CR_ECX_SHIFT                         (16U)
#define DMA_CR_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_ECX_SHIFT)) & DMA_CR_ECX_MASK)
#define DMA_CR_CX_MASK                           (0x20000U)
#define DMA_CR_CX_SHIFT                          (17U)
#define DMA_CR_CX(x)                             (((uint32_t)(((uint32_t)(x)) << DMA_CR_CX_SHIFT)) & DMA_CR_CX_MASK)
#define DMA_CR_ACTIVE_MASK                       (0x80000000U)
#define DMA_CR_ACTIVE_SHIFT                      (31U)
#define DMA_CR_ACTIVE(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_CR_ACTIVE_SHIFT)) & DMA_CR_ACTIVE_MASK)

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
#define DMA_ES_ERRCHN_MASK                       (0x700U)
#define DMA_ES_ERRCHN_SHIFT                      (8U)
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ES_ERRCHN_SHIFT)) & DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          (0x4000U)
#define DMA_ES_CPE_SHIFT                         (14U)
#define DMA_ES_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_CPE_SHIFT)) & DMA_ES_CPE_MASK)
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

/*! @name CEEI - Clear Enable Error Interrupt Register */
#define DMA_CEEI_CEEI_MASK                       (0x7U)
#define DMA_CEEI_CEEI_SHIFT                      (0U)
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CEEI_SHIFT)) & DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       (0x40U)
#define DMA_CEEI_CAEE_SHIFT                      (6U)
#define DMA_CEEI_CAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CAEE_SHIFT)) & DMA_CEEI_CAEE_MASK)
#define DMA_CEEI_NOP_MASK                        (0x80U)
#define DMA_CEEI_NOP_SHIFT                       (7U)
#define DMA_CEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_NOP_SHIFT)) & DMA_CEEI_NOP_MASK)

/*! @name SEEI - Set Enable Error Interrupt Register */
#define DMA_SEEI_SEEI_MASK                       (0x7U)
#define DMA_SEEI_SEEI_SHIFT                      (0U)
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SEEI_SHIFT)) & DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       (0x40U)
#define DMA_SEEI_SAEE_SHIFT                      (6U)
#define DMA_SEEI_SAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SAEE_SHIFT)) & DMA_SEEI_SAEE_MASK)
#define DMA_SEEI_NOP_MASK                        (0x80U)
#define DMA_SEEI_NOP_SHIFT                       (7U)
#define DMA_SEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_NOP_SHIFT)) & DMA_SEEI_NOP_MASK)

/*! @name CERQ - Clear Enable Request Register */
#define DMA_CERQ_CERQ_MASK                       (0x7U)
#define DMA_CERQ_CERQ_SHIFT                      (0U)
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CERQ_SHIFT)) & DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       (0x40U)
#define DMA_CERQ_CAER_SHIFT                      (6U)
#define DMA_CERQ_CAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CAER_SHIFT)) & DMA_CERQ_CAER_MASK)
#define DMA_CERQ_NOP_MASK                        (0x80U)
#define DMA_CERQ_NOP_SHIFT                       (7U)
#define DMA_CERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_NOP_SHIFT)) & DMA_CERQ_NOP_MASK)

/*! @name SERQ - Set Enable Request Register */
#define DMA_SERQ_SERQ_MASK                       (0x7U)
#define DMA_SERQ_SERQ_SHIFT                      (0U)
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SERQ_SHIFT)) & DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       (0x40U)
#define DMA_SERQ_SAER_SHIFT                      (6U)
#define DMA_SERQ_SAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SAER_SHIFT)) & DMA_SERQ_SAER_MASK)
#define DMA_SERQ_NOP_MASK                        (0x80U)
#define DMA_SERQ_NOP_SHIFT                       (7U)
#define DMA_SERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_NOP_SHIFT)) & DMA_SERQ_NOP_MASK)

/*! @name CDNE - Clear DONE Status Bit Register */
#define DMA_CDNE_CDNE_MASK                       (0x7U)
#define DMA_CDNE_CDNE_SHIFT                      (0U)
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CDNE_SHIFT)) & DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       (0x40U)
#define DMA_CDNE_CADN_SHIFT                      (6U)
#define DMA_CDNE_CADN(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CADN_SHIFT)) & DMA_CDNE_CADN_MASK)
#define DMA_CDNE_NOP_MASK                        (0x80U)
#define DMA_CDNE_NOP_SHIFT                       (7U)
#define DMA_CDNE_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_NOP_SHIFT)) & DMA_CDNE_NOP_MASK)

/*! @name SSRT - Set START Bit Register */
#define DMA_SSRT_SSRT_MASK                       (0x7U)
#define DMA_SSRT_SSRT_SHIFT                      (0U)
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SSRT_SHIFT)) & DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       (0x40U)
#define DMA_SSRT_SAST_SHIFT                      (6U)
#define DMA_SSRT_SAST(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SAST_SHIFT)) & DMA_SSRT_SAST_MASK)
#define DMA_SSRT_NOP_MASK                        (0x80U)
#define DMA_SSRT_NOP_SHIFT                       (7U)
#define DMA_SSRT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_NOP_SHIFT)) & DMA_SSRT_NOP_MASK)

/*! @name CERR - Clear Error Register */
#define DMA_CERR_CERR_MASK                       (0x7U)
#define DMA_CERR_CERR_SHIFT                      (0U)
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CERR_SHIFT)) & DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       (0x40U)
#define DMA_CERR_CAEI_SHIFT                      (6U)
#define DMA_CERR_CAEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CAEI_SHIFT)) & DMA_CERR_CAEI_MASK)
#define DMA_CERR_NOP_MASK                        (0x80U)
#define DMA_CERR_NOP_SHIFT                       (7U)
#define DMA_CERR_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERR_NOP_SHIFT)) & DMA_CERR_NOP_MASK)

/*! @name CINT - Clear Interrupt Request Register */
#define DMA_CINT_CINT_MASK                       (0x7U)
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

/*! @name DCHPRI3 - Channel n Priority Register */
#define DMA_DCHPRI3_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI3_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_CHPRI_SHIFT)) & DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_DPA_MASK                     (0x40U)
#define DMA_DCHPRI3_DPA_SHIFT                    (6U)
#define DMA_DCHPRI3_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_DPA_SHIFT)) & DMA_DCHPRI3_DPA_MASK)
#define DMA_DCHPRI3_ECP_MASK                     (0x80U)
#define DMA_DCHPRI3_ECP_SHIFT                    (7U)
#define DMA_DCHPRI3_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_ECP_SHIFT)) & DMA_DCHPRI3_ECP_MASK)

/*! @name DCHPRI2 - Channel n Priority Register */
#define DMA_DCHPRI2_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI2_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_CHPRI_SHIFT)) & DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_DPA_MASK                     (0x40U)
#define DMA_DCHPRI2_DPA_SHIFT                    (6U)
#define DMA_DCHPRI2_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_DPA_SHIFT)) & DMA_DCHPRI2_DPA_MASK)
#define DMA_DCHPRI2_ECP_MASK                     (0x80U)
#define DMA_DCHPRI2_ECP_SHIFT                    (7U)
#define DMA_DCHPRI2_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_ECP_SHIFT)) & DMA_DCHPRI2_ECP_MASK)

/*! @name DCHPRI1 - Channel n Priority Register */
#define DMA_DCHPRI1_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI1_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_CHPRI_SHIFT)) & DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_DPA_MASK                     (0x40U)
#define DMA_DCHPRI1_DPA_SHIFT                    (6U)
#define DMA_DCHPRI1_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_DPA_SHIFT)) & DMA_DCHPRI1_DPA_MASK)
#define DMA_DCHPRI1_ECP_MASK                     (0x80U)
#define DMA_DCHPRI1_ECP_SHIFT                    (7U)
#define DMA_DCHPRI1_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_ECP_SHIFT)) & DMA_DCHPRI1_ECP_MASK)

/*! @name DCHPRI0 - Channel n Priority Register */
#define DMA_DCHPRI0_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI0_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_CHPRI_SHIFT)) & DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_DPA_MASK                     (0x40U)
#define DMA_DCHPRI0_DPA_SHIFT                    (6U)
#define DMA_DCHPRI0_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_DPA_SHIFT)) & DMA_DCHPRI0_DPA_MASK)
#define DMA_DCHPRI0_ECP_MASK                     (0x80U)
#define DMA_DCHPRI0_ECP_SHIFT                    (7U)
#define DMA_DCHPRI0_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_ECP_SHIFT)) & DMA_DCHPRI0_ECP_MASK)

/*! @name DCHPRI7 - Channel n Priority Register */
#define DMA_DCHPRI7_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI7_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI7_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_CHPRI_SHIFT)) & DMA_DCHPRI7_CHPRI_MASK)
#define DMA_DCHPRI7_DPA_MASK                     (0x40U)
#define DMA_DCHPRI7_DPA_SHIFT                    (6U)
#define DMA_DCHPRI7_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_DPA_SHIFT)) & DMA_DCHPRI7_DPA_MASK)
#define DMA_DCHPRI7_ECP_MASK                     (0x80U)
#define DMA_DCHPRI7_ECP_SHIFT                    (7U)
#define DMA_DCHPRI7_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI7_ECP_SHIFT)) & DMA_DCHPRI7_ECP_MASK)

/*! @name DCHPRI6 - Channel n Priority Register */
#define DMA_DCHPRI6_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI6_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI6_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_CHPRI_SHIFT)) & DMA_DCHPRI6_CHPRI_MASK)
#define DMA_DCHPRI6_DPA_MASK                     (0x40U)
#define DMA_DCHPRI6_DPA_SHIFT                    (6U)
#define DMA_DCHPRI6_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_DPA_SHIFT)) & DMA_DCHPRI6_DPA_MASK)
#define DMA_DCHPRI6_ECP_MASK                     (0x80U)
#define DMA_DCHPRI6_ECP_SHIFT                    (7U)
#define DMA_DCHPRI6_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI6_ECP_SHIFT)) & DMA_DCHPRI6_ECP_MASK)

/*! @name DCHPRI5 - Channel n Priority Register */
#define DMA_DCHPRI5_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI5_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI5_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_CHPRI_SHIFT)) & DMA_DCHPRI5_CHPRI_MASK)
#define DMA_DCHPRI5_DPA_MASK                     (0x40U)
#define DMA_DCHPRI5_DPA_SHIFT                    (6U)
#define DMA_DCHPRI5_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_DPA_SHIFT)) & DMA_DCHPRI5_DPA_MASK)
#define DMA_DCHPRI5_ECP_MASK                     (0x80U)
#define DMA_DCHPRI5_ECP_SHIFT                    (7U)
#define DMA_DCHPRI5_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI5_ECP_SHIFT)) & DMA_DCHPRI5_ECP_MASK)

/*! @name DCHPRI4 - Channel n Priority Register */
#define DMA_DCHPRI4_CHPRI_MASK                   (0x7U)
#define DMA_DCHPRI4_CHPRI_SHIFT                  (0U)
#define DMA_DCHPRI4_CHPRI(x)                     (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_CHPRI_SHIFT)) & DMA_DCHPRI4_CHPRI_MASK)
#define DMA_DCHPRI4_DPA_MASK                     (0x40U)
#define DMA_DCHPRI4_DPA_SHIFT                    (6U)
#define DMA_DCHPRI4_DPA(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_DPA_SHIFT)) & DMA_DCHPRI4_DPA_MASK)
#define DMA_DCHPRI4_ECP_MASK                     (0x80U)
#define DMA_DCHPRI4_ECP_SHIFT                    (7U)
#define DMA_DCHPRI4_ECP(x)                       (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI4_ECP_SHIFT)) & DMA_DCHPRI4_ECP_MASK)

/*! @name SADDR - TCD Source Address */
#define DMA_SADDR_SADDR_MASK                     (0xFFFFFFFFU)
#define DMA_SADDR_SADDR_SHIFT                    (0U)
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SADDR_SADDR_SHIFT)) & DMA_SADDR_SADDR_MASK)

/* The count of DMA_SADDR */
#define DMA_SADDR_COUNT                          (8U)

/*! @name SOFF - TCD Signed Source Address Offset */
#define DMA_SOFF_SOFF_MASK                       (0xFFFFU)
#define DMA_SOFF_SOFF_SHIFT                      (0U)
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_SOFF_SOFF_SHIFT)) & DMA_SOFF_SOFF_MASK)

/* The count of DMA_SOFF */
#define DMA_SOFF_COUNT                           (8U)

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
#define DMA_ATTR_COUNT                           (8U)

/*! @name NBYTES_MLNO - TCD Minor Byte Count (Minor Loop Mapping Disabled) */
#define DMA_NBYTES_MLNO_NBYTES_MASK              (0xFFFFFFFFU)
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             (0U)
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLNO_NBYTES_SHIFT)) & DMA_NBYTES_MLNO_NBYTES_MASK)

/* The count of DMA_NBYTES_MLNO */
#define DMA_NBYTES_MLNO_COUNT                    (8U)

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
#define DMA_NBYTES_MLOFFNO_COUNT                 (8U)

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
#define DMA_NBYTES_MLOFFYES_COUNT                (8U)

/*! @name SLAST - TCD Last Source Address Adjustment */
#define DMA_SLAST_SLAST_MASK                     (0xFFFFFFFFU)
#define DMA_SLAST_SLAST_SHIFT                    (0U)
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SLAST_SLAST_SHIFT)) & DMA_SLAST_SLAST_MASK)

/* The count of DMA_SLAST */
#define DMA_SLAST_COUNT                          (8U)

/*! @name DADDR - TCD Destination Address */
#define DMA_DADDR_DADDR_MASK                     (0xFFFFFFFFU)
#define DMA_DADDR_DADDR_SHIFT                    (0U)
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DADDR_DADDR_SHIFT)) & DMA_DADDR_DADDR_MASK)

/* The count of DMA_DADDR */
#define DMA_DADDR_COUNT                          (8U)

/*! @name DOFF - TCD Signed Destination Address Offset */
#define DMA_DOFF_DOFF_MASK                       (0xFFFFU)
#define DMA_DOFF_DOFF_SHIFT                      (0U)
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_DOFF_DOFF_SHIFT)) & DMA_DOFF_DOFF_MASK)

/* The count of DMA_DOFF */
#define DMA_DOFF_COUNT                           (8U)

/*! @name CITER_ELINKNO - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
#define DMA_CITER_ELINKNO_CITER_MASK             (0x7FFFU)
#define DMA_CITER_ELINKNO_CITER_SHIFT            (0U)
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_CITER_SHIFT)) & DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_CITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_CITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_ELINK_SHIFT)) & DMA_CITER_ELINKNO_ELINK_MASK)

/* The count of DMA_CITER_ELINKNO */
#define DMA_CITER_ELINKNO_COUNT                  (8U)

/*! @name CITER_ELINKYES - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
#define DMA_CITER_ELINKYES_CITER_MASK            (0x1FFU)
#define DMA_CITER_ELINKYES_CITER_SHIFT           (0U)
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_CITER_SHIFT)) & DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           (0xE00U)
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_LINKCH_SHIFT)) & DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_CITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_CITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_ELINK_SHIFT)) & DMA_CITER_ELINKYES_ELINK_MASK)

/* The count of DMA_CITER_ELINKYES */
#define DMA_CITER_ELINKYES_COUNT                 (8U)

/*! @name DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address */
#define DMA_DLAST_SGA_DLASTSGA_MASK              (0xFFFFFFFFU)
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             (0U)
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x)) << DMA_DLAST_SGA_DLASTSGA_SHIFT)) & DMA_DLAST_SGA_DLASTSGA_MASK)

/* The count of DMA_DLAST_SGA */
#define DMA_DLAST_SGA_COUNT                      (8U)

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
#define DMA_CSR_MAJORLINKCH_MASK                 (0x700U)
#define DMA_CSR_MAJORLINKCH_SHIFT                (8U)
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORLINKCH_SHIFT)) & DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         (0xC000U)
#define DMA_CSR_BWC_SHIFT                        (14U)
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x)) << DMA_CSR_BWC_SHIFT)) & DMA_CSR_BWC_MASK)

/* The count of DMA_CSR */
#define DMA_CSR_COUNT                            (8U)

/*! @name BITER_ELINKNO - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
#define DMA_BITER_ELINKNO_BITER_MASK             (0x7FFFU)
#define DMA_BITER_ELINKNO_BITER_SHIFT            (0U)
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_BITER_SHIFT)) & DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_BITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_BITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_ELINK_SHIFT)) & DMA_BITER_ELINKNO_ELINK_MASK)

/* The count of DMA_BITER_ELINKNO */
#define DMA_BITER_ELINKNO_COUNT                  (8U)

/*! @name BITER_ELINKYES - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
#define DMA_BITER_ELINKYES_BITER_MASK            (0x1FFU)
#define DMA_BITER_ELINKYES_BITER_SHIFT           (0U)
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_BITER_SHIFT)) & DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           (0xE00U)
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_LINKCH_SHIFT)) & DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_BITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_BITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_ELINK_SHIFT)) & DMA_BITER_ELINKYES_ELINK_MASK)

/* The count of DMA_BITER_ELINKYES */
#define DMA_BITER_ELINKYES_COUNT                 (8U)


/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA0 base address */
#define DMA0_BASE                                (0x40008000u)
/** Peripheral DMA0 base pointer */
#define DMA0                                     ((DMA_Type *)DMA0_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA0_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_CHN_IRQS                             { DMA0_04_IRQn, DMA0_15_IRQn, DMA0_26_IRQn, DMA0_37_IRQn, DMA0_04_IRQn, DMA0_15_IRQn, DMA0_26_IRQn, DMA0_37_IRQn }
#define DMA_ERROR_IRQS                           { CTI0_DMA0_Error_IRQn }

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
  __IO uint8_t CHCFG[8];                           /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
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
#define DMAMUX_CHCFG_COUNT                       (8U)


/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX0 base address */
#define DMAMUX0_BASE                             (0x40021000u)
/** Peripheral DMAMUX0 base pointer */
#define DMAMUX0                                  ((DMAMUX_Type *)DMAMUX0_BASE)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX0_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX0 }

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
#define EMVSIM_RX_STATUS_RX_WPTR_MASK            (0x30000U)
#define EMVSIM_RX_STATUS_RX_WPTR_SHIFT           (16U)
#define EMVSIM_RX_STATUS_RX_WPTR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_RX_STATUS_RX_WPTR_SHIFT)) & EMVSIM_RX_STATUS_RX_WPTR_MASK)
#define EMVSIM_RX_STATUS_RX_CNT_MASK             (0x1C00000U)
#define EMVSIM_RX_STATUS_RX_CNT_SHIFT            (22U)
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
#define EMVSIM_TX_STATUS_TX_RPTR_MASK            (0x30000U)
#define EMVSIM_TX_STATUS_TX_RPTR_SHIFT           (16U)
#define EMVSIM_TX_STATUS_TX_RPTR(x)              (((uint32_t)(((uint32_t)(x)) << EMVSIM_TX_STATUS_TX_RPTR_SHIFT)) & EMVSIM_TX_STATUS_TX_RPTR_MASK)
#define EMVSIM_TX_STATUS_TX_CNT_MASK             (0x1C00000U)
#define EMVSIM_TX_STATUS_TX_CNT_SHIFT            (22U)
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
#define EMVSIM0_BASE                             (0x4004E000u)
/** Peripheral EMVSIM0 base pointer */
#define EMVSIM0                                  ((EMVSIM_Type *)EMVSIM0_BASE)
/** Array initializer of EMVSIM peripheral base addresses */
#define EMVSIM_BASE_ADDRS                        { EMVSIM0_BASE }
/** Array initializer of EMVSIM peripheral base pointers */
#define EMVSIM_BASE_PTRS                         { EMVSIM0 }
/** Interrupt vectors for the EMVSIM peripheral type */
#define EMVSIM_IRQS                              { EMVSIM0_IRQn }

/*!
 * @}
 */ /* end of group EMVSIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FGPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FGPIO_Peripheral_Access_Layer FGPIO Peripheral Access Layer
 * @{
 */

/** FGPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} FGPIO_Type;

/* ----------------------------------------------------------------------------
   -- FGPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FGPIO_Register_Masks FGPIO Register Masks
 * @{
 */

/*! @name PDOR - Port Data Output Register */
#define FGPIO_PDOR_PDO_MASK                      (0xFFFFFFFFU)
#define FGPIO_PDOR_PDO_SHIFT                     (0U)
#define FGPIO_PDOR_PDO(x)                        (((uint32_t)(((uint32_t)(x)) << FGPIO_PDOR_PDO_SHIFT)) & FGPIO_PDOR_PDO_MASK)

/*! @name PSOR - Port Set Output Register */
#define FGPIO_PSOR_PTSO_MASK                     (0xFFFFFFFFU)
#define FGPIO_PSOR_PTSO_SHIFT                    (0U)
#define FGPIO_PSOR_PTSO(x)                       (((uint32_t)(((uint32_t)(x)) << FGPIO_PSOR_PTSO_SHIFT)) & FGPIO_PSOR_PTSO_MASK)

/*! @name PCOR - Port Clear Output Register */
#define FGPIO_PCOR_PTCO_MASK                     (0xFFFFFFFFU)
#define FGPIO_PCOR_PTCO_SHIFT                    (0U)
#define FGPIO_PCOR_PTCO(x)                       (((uint32_t)(((uint32_t)(x)) << FGPIO_PCOR_PTCO_SHIFT)) & FGPIO_PCOR_PTCO_MASK)

/*! @name PTOR - Port Toggle Output Register */
#define FGPIO_PTOR_PTTO_MASK                     (0xFFFFFFFFU)
#define FGPIO_PTOR_PTTO_SHIFT                    (0U)
#define FGPIO_PTOR_PTTO(x)                       (((uint32_t)(((uint32_t)(x)) << FGPIO_PTOR_PTTO_SHIFT)) & FGPIO_PTOR_PTTO_MASK)

/*! @name PDIR - Port Data Input Register */
#define FGPIO_PDIR_PDI_MASK                      (0xFFFFFFFFU)
#define FGPIO_PDIR_PDI_SHIFT                     (0U)
#define FGPIO_PDIR_PDI(x)                        (((uint32_t)(((uint32_t)(x)) << FGPIO_PDIR_PDI_SHIFT)) & FGPIO_PDIR_PDI_MASK)

/*! @name PDDR - Port Data Direction Register */
#define FGPIO_PDDR_PDD_MASK                      (0xFFFFFFFFU)
#define FGPIO_PDDR_PDD_SHIFT                     (0U)
#define FGPIO_PDDR_PDD(x)                        (((uint32_t)(((uint32_t)(x)) << FGPIO_PDDR_PDD_SHIFT)) & FGPIO_PDDR_PDD_MASK)


/*!
 * @}
 */ /* end of group FGPIO_Register_Masks */


/* FGPIO - Peripheral instance base addresses */
/** Peripheral FGPIOA base address */
#define FGPIOA_BASE                              (0xF8000000u)
/** Peripheral FGPIOA base pointer */
#define FGPIOA                                   ((FGPIO_Type *)FGPIOA_BASE)
/** Array initializer of FGPIO peripheral base addresses */
#define FGPIO_BASE_ADDRS                         { FGPIOA_BASE }
/** Array initializer of FGPIO peripheral base pointers */
#define FGPIO_BASE_PTRS                          { FGPIOA }

/*!
 * @}
 */ /* end of group FGPIO_Peripheral_Access_Layer */


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
#define FLEXIO0_BASE                             (0x400CA000u)
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

/*!
 * @}
 */ /* end of group FTFA_Peripheral_Access_Layer */


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
#define GPIOA_BASE                               (0x4000F000u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0x4000F040u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0x4000F080u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0x4000F0C0u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0x4000F100u)
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
  __O  uint32_t TDR[1];                            /**< SAI Transmit Data Register, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[28];
  __I  uint32_t TFR[1];                            /**< SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4 */
       uint8_t RESERVED_2[28];
  __IO uint32_t TMR;                               /**< SAI Transmit Mask Register, offset: 0x60 */
       uint8_t RESERVED_3[28];
  __IO uint32_t RCSR;                              /**< SAI Receive Control Register, offset: 0x80 */
  __IO uint32_t RCR1;                              /**< SAI Receive Configuration 1 Register, offset: 0x84 */
  __IO uint32_t RCR2;                              /**< SAI Receive Configuration 2 Register, offset: 0x88 */
  __IO uint32_t RCR3;                              /**< SAI Receive Configuration 3 Register, offset: 0x8C */
  __IO uint32_t RCR4;                              /**< SAI Receive Configuration 4 Register, offset: 0x90 */
  __IO uint32_t RCR5;                              /**< SAI Receive Configuration 5 Register, offset: 0x94 */
       uint8_t RESERVED_4[8];
  __I  uint32_t RDR[1];                            /**< SAI Receive Data Register, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_5[28];
  __I  uint32_t RFR[1];                            /**< SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_6[28];
  __IO uint32_t RMR;                               /**< SAI Receive Mask Register, offset: 0xE0 */
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
#define I2S_TCR1_TFW_MASK                        (0x3U)
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
#define I2S_TCR3_WDFL_MASK                       (0xFU)
#define I2S_TCR3_WDFL_SHIFT                      (0U)
#define I2S_TCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_WDFL_SHIFT)) & I2S_TCR3_WDFL_MASK)
#define I2S_TCR3_TCE_MASK                        (0x10000U)
#define I2S_TCR3_TCE_SHIFT                       (16U)
#define I2S_TCR3_TCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_TCE_SHIFT)) & I2S_TCR3_TCE_MASK)

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
#define I2S_TCR4_FRSZ_MASK                       (0xF0000U)
#define I2S_TCR4_FRSZ_SHIFT                      (16U)
#define I2S_TCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FRSZ_SHIFT)) & I2S_TCR4_FRSZ_MASK)
#define I2S_TCR4_FPACK_MASK                      (0x3000000U)
#define I2S_TCR4_FPACK_SHIFT                     (24U)
#define I2S_TCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FPACK_SHIFT)) & I2S_TCR4_FPACK_MASK)
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
#define I2S_TDR_COUNT                            (1U)

/*! @name TFR - SAI Transmit FIFO Register */
#define I2S_TFR_RFP_MASK                         (0x7U)
#define I2S_TFR_RFP_SHIFT                        (0U)
#define I2S_TFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_RFP_SHIFT)) & I2S_TFR_RFP_MASK)
#define I2S_TFR_WFP_MASK                         (0x70000U)
#define I2S_TFR_WFP_SHIFT                        (16U)
#define I2S_TFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_WFP_SHIFT)) & I2S_TFR_WFP_MASK)

/* The count of I2S_TFR */
#define I2S_TFR_COUNT                            (1U)

/*! @name TMR - SAI Transmit Mask Register */
#define I2S_TMR_TWM_MASK                         (0xFFFFU)
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
#define I2S_RCR1_RFW_MASK                        (0x3U)
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
#define I2S_RCR3_WDFL_MASK                       (0xFU)
#define I2S_RCR3_WDFL_SHIFT                      (0U)
#define I2S_RCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_WDFL_SHIFT)) & I2S_RCR3_WDFL_MASK)
#define I2S_RCR3_RCE_MASK                        (0x10000U)
#define I2S_RCR3_RCE_SHIFT                       (16U)
#define I2S_RCR3_RCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_RCE_SHIFT)) & I2S_RCR3_RCE_MASK)

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
#define I2S_RCR4_FRSZ_MASK                       (0xF0000U)
#define I2S_RCR4_FRSZ_SHIFT                      (16U)
#define I2S_RCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FRSZ_SHIFT)) & I2S_RCR4_FRSZ_MASK)
#define I2S_RCR4_FPACK_MASK                      (0x3000000U)
#define I2S_RCR4_FPACK_SHIFT                     (24U)
#define I2S_RCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FPACK_SHIFT)) & I2S_RCR4_FPACK_MASK)
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
#define I2S_RDR_COUNT                            (1U)

/*! @name RFR - SAI Receive FIFO Register */
#define I2S_RFR_RFP_MASK                         (0x7U)
#define I2S_RFR_RFP_SHIFT                        (0U)
#define I2S_RFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_RFP_SHIFT)) & I2S_RFR_RFP_MASK)
#define I2S_RFR_WFP_MASK                         (0x70000U)
#define I2S_RFR_WFP_SHIFT                        (16U)
#define I2S_RFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_WFP_SHIFT)) & I2S_RFR_WFP_MASK)

/* The count of I2S_RFR */
#define I2S_RFR_COUNT                            (1U)

/*! @name RMR - SAI Receive Mask Register */
#define I2S_RMR_RWM_MASK                         (0xFFFFU)
#define I2S_RMR_RWM_SHIFT                        (0U)
#define I2S_RMR_RWM(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RMR_RWM_SHIFT)) & I2S_RMR_RWM_MASK)


/*!
 * @}
 */ /* end of group I2S_Register_Masks */


/* I2S - Peripheral instance base addresses */
/** Peripheral I2S0 base address */
#define I2S0_BASE                                (0x4004C000u)
/** Peripheral I2S0 base pointer */
#define I2S0                                     ((I2S_Type *)I2S0_BASE)
/** Array initializer of I2S peripheral base addresses */
#define I2S_BASE_ADDRS                           { I2S0_BASE }
/** Array initializer of I2S peripheral base pointers */
#define I2S_BASE_PTRS                            { I2S0 }
/** Interrupt vectors for the I2S peripheral type */
#define I2S_RX_IRQS                              { I2S0_IRQn }
#define I2S_TX_IRQS                              { I2S0_IRQn }

/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- INTMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTMUX_Peripheral_Access_Layer INTMUX Peripheral Access Layer
 * @{
 */

/** INTMUX - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x40 */
    __IO uint32_t CHn_CSR;                           /**< Channel n Control Status Register, array offset: 0x0, array step: 0x40 */
    __I  uint32_t CHn_VEC;                           /**< Channel n Vector Number Register, array offset: 0x4, array step: 0x40 */
         uint8_t RESERVED_0[8];
    __IO uint32_t CHn_IER_31_0;                      /**< Channel n Interrupt Enable Register, array offset: 0x10, array step: 0x40 */
         uint8_t RESERVED_1[12];
    __I  uint32_t CHn_IPR_31_0;                      /**< Channel n Interrupt Pending Register, array offset: 0x20, array step: 0x40 */
         uint8_t RESERVED_2[28];
  } CHANNEL[4];
} INTMUX_Type;

/* ----------------------------------------------------------------------------
   -- INTMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTMUX_Register_Masks INTMUX Register Masks
 * @{
 */

/*! @name CHn_CSR - Channel n Control Status Register */
#define INTMUX_CHn_CSR_RST_MASK                  (0x1U)
#define INTMUX_CHn_CSR_RST_SHIFT                 (0U)
#define INTMUX_CHn_CSR_RST(x)                    (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_CSR_RST_SHIFT)) & INTMUX_CHn_CSR_RST_MASK)
#define INTMUX_CHn_CSR_AND_MASK                  (0x2U)
#define INTMUX_CHn_CSR_AND_SHIFT                 (1U)
#define INTMUX_CHn_CSR_AND(x)                    (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_CSR_AND_SHIFT)) & INTMUX_CHn_CSR_AND_MASK)
#define INTMUX_CHn_CSR_IRQN_MASK                 (0x30U)
#define INTMUX_CHn_CSR_IRQN_SHIFT                (4U)
#define INTMUX_CHn_CSR_IRQN(x)                   (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_CSR_IRQN_SHIFT)) & INTMUX_CHn_CSR_IRQN_MASK)
#define INTMUX_CHn_CSR_CHIN_MASK                 (0xF00U)
#define INTMUX_CHn_CSR_CHIN_SHIFT                (8U)
#define INTMUX_CHn_CSR_CHIN(x)                   (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_CSR_CHIN_SHIFT)) & INTMUX_CHn_CSR_CHIN_MASK)
#define INTMUX_CHn_CSR_IRQP_MASK                 (0x80000000U)
#define INTMUX_CHn_CSR_IRQP_SHIFT                (31U)
#define INTMUX_CHn_CSR_IRQP(x)                   (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_CSR_IRQP_SHIFT)) & INTMUX_CHn_CSR_IRQP_MASK)

/* The count of INTMUX_CHn_CSR */
#define INTMUX_CHn_CSR_COUNT                     (4U)

/*! @name CHn_VEC - Channel n Vector Number Register */
#define INTMUX_CHn_VEC_VECN_MASK                 (0x3FFCU)
#define INTMUX_CHn_VEC_VECN_SHIFT                (2U)
#define INTMUX_CHn_VEC_VECN(x)                   (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_VEC_VECN_SHIFT)) & INTMUX_CHn_VEC_VECN_MASK)

/* The count of INTMUX_CHn_VEC */
#define INTMUX_CHn_VEC_COUNT                     (4U)

/*! @name CHn_IER_31_0 - Channel n Interrupt Enable Register */
#define INTMUX_CHn_IER_31_0_INTE_MASK            (0xFFFFFFFFU)
#define INTMUX_CHn_IER_31_0_INTE_SHIFT           (0U)
#define INTMUX_CHn_IER_31_0_INTE(x)              (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_IER_31_0_INTE_SHIFT)) & INTMUX_CHn_IER_31_0_INTE_MASK)

/* The count of INTMUX_CHn_IER_31_0 */
#define INTMUX_CHn_IER_31_0_COUNT                (4U)

/*! @name CHn_IPR_31_0 - Channel n Interrupt Pending Register */
#define INTMUX_CHn_IPR_31_0_INTP_MASK            (0xFFFFFFFFU)
#define INTMUX_CHn_IPR_31_0_INTP_SHIFT           (0U)
#define INTMUX_CHn_IPR_31_0_INTP(x)              (((uint32_t)(((uint32_t)(x)) << INTMUX_CHn_IPR_31_0_INTP_SHIFT)) & INTMUX_CHn_IPR_31_0_INTP_MASK)

/* The count of INTMUX_CHn_IPR_31_0 */
#define INTMUX_CHn_IPR_31_0_COUNT                (4U)


/*!
 * @}
 */ /* end of group INTMUX_Register_Masks */


/* INTMUX - Peripheral instance base addresses */
/** Peripheral INTMUX0 base address */
#define INTMUX0_BASE                             (0x40024000u)
/** Peripheral INTMUX0 base pointer */
#define INTMUX0                                  ((INTMUX_Type *)INTMUX0_BASE)
/** Array initializer of INTMUX peripheral base addresses */
#define INTMUX_BASE_ADDRS                        { INTMUX0_BASE }
/** Array initializer of INTMUX peripheral base pointers */
#define INTMUX_BASE_PTRS                         { INTMUX0 }
/** Interrupt vectors for the INTMUX peripheral type */
#define INTMUX_IRQS                              { INTMUX0_0_IRQn, INTMUX0_1_IRQn, INTMUX0_2_IRQn, INTMUX0_3_IRQn }

/*!
 * @}
 */ /* end of group INTMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LLWU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Peripheral_Access_Layer LLWU Peripheral Access Layer
 * @{
 */

/** LLWU - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t PE1;                               /**< LLWU Pin Enable 1 register, offset: 0x8 */
  __IO uint32_t PE2;                               /**< LLWU Pin Enable 2 register, offset: 0xC */
       uint8_t RESERVED_0[8];
  __IO uint32_t ME;                                /**< LLWU Module Interrupt Enable register, offset: 0x18 */
  __IO uint32_t DE;                                /**< LLWU Module DMA Enable register, offset: 0x1C */
  __IO uint32_t PF;                                /**< LLWU Pin Flag register, offset: 0x20 */
       uint8_t RESERVED_1[4];
  __I  uint32_t MF;                                /**< LLWU Module Interrupt Flag register, offset: 0x28 */
       uint8_t RESERVED_2[4];
  __IO uint32_t FILT;                              /**< LLWU Pin Filter register, offset: 0x30 */
} LLWU_Type;

/* ----------------------------------------------------------------------------
   -- LLWU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Masks LLWU Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define LLWU_VERID_FEATURE_MASK                  (0xFFFFU)
#define LLWU_VERID_FEATURE_SHIFT                 (0U)
#define LLWU_VERID_FEATURE(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_VERID_FEATURE_SHIFT)) & LLWU_VERID_FEATURE_MASK)
#define LLWU_VERID_MINOR_MASK                    (0xFF0000U)
#define LLWU_VERID_MINOR_SHIFT                   (16U)
#define LLWU_VERID_MINOR(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_VERID_MINOR_SHIFT)) & LLWU_VERID_MINOR_MASK)
#define LLWU_VERID_MAJOR_MASK                    (0xFF000000U)
#define LLWU_VERID_MAJOR_SHIFT                   (24U)
#define LLWU_VERID_MAJOR(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_VERID_MAJOR_SHIFT)) & LLWU_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define LLWU_PARAM_FILTERS_MASK                  (0xFFU)
#define LLWU_PARAM_FILTERS_SHIFT                 (0U)
#define LLWU_PARAM_FILTERS(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_PARAM_FILTERS_SHIFT)) & LLWU_PARAM_FILTERS_MASK)
#define LLWU_PARAM_DMAS_MASK                     (0xFF00U)
#define LLWU_PARAM_DMAS_SHIFT                    (8U)
#define LLWU_PARAM_DMAS(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PARAM_DMAS_SHIFT)) & LLWU_PARAM_DMAS_MASK)
#define LLWU_PARAM_MODULES_MASK                  (0xFF0000U)
#define LLWU_PARAM_MODULES_SHIFT                 (16U)
#define LLWU_PARAM_MODULES(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_PARAM_MODULES_SHIFT)) & LLWU_PARAM_MODULES_MASK)
#define LLWU_PARAM_PINS_MASK                     (0xFF000000U)
#define LLWU_PARAM_PINS_SHIFT                    (24U)
#define LLWU_PARAM_PINS(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PARAM_PINS_SHIFT)) & LLWU_PARAM_PINS_MASK)

/*! @name PE1 - LLWU Pin Enable 1 register */
#define LLWU_PE1_WUPE0_MASK                      (0x3U)
#define LLWU_PE1_WUPE0_SHIFT                     (0U)
#define LLWU_PE1_WUPE0(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE0_SHIFT)) & LLWU_PE1_WUPE0_MASK)
#define LLWU_PE1_WUPE1_MASK                      (0xCU)
#define LLWU_PE1_WUPE1_SHIFT                     (2U)
#define LLWU_PE1_WUPE1(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE1_SHIFT)) & LLWU_PE1_WUPE1_MASK)
#define LLWU_PE1_WUPE2_MASK                      (0x30U)
#define LLWU_PE1_WUPE2_SHIFT                     (4U)
#define LLWU_PE1_WUPE2(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE2_SHIFT)) & LLWU_PE1_WUPE2_MASK)
#define LLWU_PE1_WUPE3_MASK                      (0xC0U)
#define LLWU_PE1_WUPE3_SHIFT                     (6U)
#define LLWU_PE1_WUPE3(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE3_SHIFT)) & LLWU_PE1_WUPE3_MASK)
#define LLWU_PE1_WUPE4_MASK                      (0x300U)
#define LLWU_PE1_WUPE4_SHIFT                     (8U)
#define LLWU_PE1_WUPE4(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE4_SHIFT)) & LLWU_PE1_WUPE4_MASK)
#define LLWU_PE1_WUPE5_MASK                      (0xC00U)
#define LLWU_PE1_WUPE5_SHIFT                     (10U)
#define LLWU_PE1_WUPE5(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE5_SHIFT)) & LLWU_PE1_WUPE5_MASK)
#define LLWU_PE1_WUPE6_MASK                      (0x3000U)
#define LLWU_PE1_WUPE6_SHIFT                     (12U)
#define LLWU_PE1_WUPE6(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE6_SHIFT)) & LLWU_PE1_WUPE6_MASK)
#define LLWU_PE1_WUPE7_MASK                      (0xC000U)
#define LLWU_PE1_WUPE7_SHIFT                     (14U)
#define LLWU_PE1_WUPE7(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE7_SHIFT)) & LLWU_PE1_WUPE7_MASK)
#define LLWU_PE1_WUPE8_MASK                      (0x30000U)
#define LLWU_PE1_WUPE8_SHIFT                     (16U)
#define LLWU_PE1_WUPE8(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE8_SHIFT)) & LLWU_PE1_WUPE8_MASK)
#define LLWU_PE1_WUPE9_MASK                      (0xC0000U)
#define LLWU_PE1_WUPE9_SHIFT                     (18U)
#define LLWU_PE1_WUPE9(x)                        (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE9_SHIFT)) & LLWU_PE1_WUPE9_MASK)
#define LLWU_PE1_WUPE10_MASK                     (0x300000U)
#define LLWU_PE1_WUPE10_SHIFT                    (20U)
#define LLWU_PE1_WUPE10(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE10_SHIFT)) & LLWU_PE1_WUPE10_MASK)
#define LLWU_PE1_WUPE11_MASK                     (0xC00000U)
#define LLWU_PE1_WUPE11_SHIFT                    (22U)
#define LLWU_PE1_WUPE11(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE11_SHIFT)) & LLWU_PE1_WUPE11_MASK)
#define LLWU_PE1_WUPE12_MASK                     (0x3000000U)
#define LLWU_PE1_WUPE12_SHIFT                    (24U)
#define LLWU_PE1_WUPE12(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE12_SHIFT)) & LLWU_PE1_WUPE12_MASK)
#define LLWU_PE1_WUPE13_MASK                     (0xC000000U)
#define LLWU_PE1_WUPE13_SHIFT                    (26U)
#define LLWU_PE1_WUPE13(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE13_SHIFT)) & LLWU_PE1_WUPE13_MASK)
#define LLWU_PE1_WUPE14_MASK                     (0x30000000U)
#define LLWU_PE1_WUPE14_SHIFT                    (28U)
#define LLWU_PE1_WUPE14(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE14_SHIFT)) & LLWU_PE1_WUPE14_MASK)
#define LLWU_PE1_WUPE15_MASK                     (0xC0000000U)
#define LLWU_PE1_WUPE15_SHIFT                    (30U)
#define LLWU_PE1_WUPE15(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE1_WUPE15_SHIFT)) & LLWU_PE1_WUPE15_MASK)

/*! @name PE2 - LLWU Pin Enable 2 register */
#define LLWU_PE2_WUPE16_MASK                     (0x3U)
#define LLWU_PE2_WUPE16_SHIFT                    (0U)
#define LLWU_PE2_WUPE16(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE16_SHIFT)) & LLWU_PE2_WUPE16_MASK)
#define LLWU_PE2_WUPE17_MASK                     (0xCU)
#define LLWU_PE2_WUPE17_SHIFT                    (2U)
#define LLWU_PE2_WUPE17(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE17_SHIFT)) & LLWU_PE2_WUPE17_MASK)
#define LLWU_PE2_WUPE18_MASK                     (0x30U)
#define LLWU_PE2_WUPE18_SHIFT                    (4U)
#define LLWU_PE2_WUPE18(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE18_SHIFT)) & LLWU_PE2_WUPE18_MASK)
#define LLWU_PE2_WUPE19_MASK                     (0xC0U)
#define LLWU_PE2_WUPE19_SHIFT                    (6U)
#define LLWU_PE2_WUPE19(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE19_SHIFT)) & LLWU_PE2_WUPE19_MASK)
#define LLWU_PE2_WUPE20_MASK                     (0x300U)
#define LLWU_PE2_WUPE20_SHIFT                    (8U)
#define LLWU_PE2_WUPE20(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE20_SHIFT)) & LLWU_PE2_WUPE20_MASK)
#define LLWU_PE2_WUPE21_MASK                     (0xC00U)
#define LLWU_PE2_WUPE21_SHIFT                    (10U)
#define LLWU_PE2_WUPE21(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE21_SHIFT)) & LLWU_PE2_WUPE21_MASK)
#define LLWU_PE2_WUPE22_MASK                     (0x3000U)
#define LLWU_PE2_WUPE22_SHIFT                    (12U)
#define LLWU_PE2_WUPE22(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE22_SHIFT)) & LLWU_PE2_WUPE22_MASK)
#define LLWU_PE2_WUPE23_MASK                     (0xC000U)
#define LLWU_PE2_WUPE23_SHIFT                    (14U)
#define LLWU_PE2_WUPE23(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE23_SHIFT)) & LLWU_PE2_WUPE23_MASK)
#define LLWU_PE2_WUPE24_MASK                     (0x30000U)
#define LLWU_PE2_WUPE24_SHIFT                    (16U)
#define LLWU_PE2_WUPE24(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE24_SHIFT)) & LLWU_PE2_WUPE24_MASK)
#define LLWU_PE2_WUPE25_MASK                     (0xC0000U)
#define LLWU_PE2_WUPE25_SHIFT                    (18U)
#define LLWU_PE2_WUPE25(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE25_SHIFT)) & LLWU_PE2_WUPE25_MASK)
#define LLWU_PE2_WUPE26_MASK                     (0x300000U)
#define LLWU_PE2_WUPE26_SHIFT                    (20U)
#define LLWU_PE2_WUPE26(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE26_SHIFT)) & LLWU_PE2_WUPE26_MASK)
#define LLWU_PE2_WUPE27_MASK                     (0xC00000U)
#define LLWU_PE2_WUPE27_SHIFT                    (22U)
#define LLWU_PE2_WUPE27(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE27_SHIFT)) & LLWU_PE2_WUPE27_MASK)
#define LLWU_PE2_WUPE28_MASK                     (0x3000000U)
#define LLWU_PE2_WUPE28_SHIFT                    (24U)
#define LLWU_PE2_WUPE28(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE28_SHIFT)) & LLWU_PE2_WUPE28_MASK)
#define LLWU_PE2_WUPE29_MASK                     (0xC000000U)
#define LLWU_PE2_WUPE29_SHIFT                    (26U)
#define LLWU_PE2_WUPE29(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE29_SHIFT)) & LLWU_PE2_WUPE29_MASK)
#define LLWU_PE2_WUPE30_MASK                     (0x30000000U)
#define LLWU_PE2_WUPE30_SHIFT                    (28U)
#define LLWU_PE2_WUPE30(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE30_SHIFT)) & LLWU_PE2_WUPE30_MASK)
#define LLWU_PE2_WUPE31_MASK                     (0xC0000000U)
#define LLWU_PE2_WUPE31_SHIFT                    (30U)
#define LLWU_PE2_WUPE31(x)                       (((uint32_t)(((uint32_t)(x)) << LLWU_PE2_WUPE31_SHIFT)) & LLWU_PE2_WUPE31_MASK)

/*! @name ME - LLWU Module Interrupt Enable register */
#define LLWU_ME_WUME0_MASK                       (0x1U)
#define LLWU_ME_WUME0_SHIFT                      (0U)
#define LLWU_ME_WUME0(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME0_SHIFT)) & LLWU_ME_WUME0_MASK)
#define LLWU_ME_WUME1_MASK                       (0x2U)
#define LLWU_ME_WUME1_SHIFT                      (1U)
#define LLWU_ME_WUME1(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME1_SHIFT)) & LLWU_ME_WUME1_MASK)
#define LLWU_ME_WUME2_MASK                       (0x4U)
#define LLWU_ME_WUME2_SHIFT                      (2U)
#define LLWU_ME_WUME2(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME2_SHIFT)) & LLWU_ME_WUME2_MASK)
#define LLWU_ME_WUME3_MASK                       (0x8U)
#define LLWU_ME_WUME3_SHIFT                      (3U)
#define LLWU_ME_WUME3(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME3_SHIFT)) & LLWU_ME_WUME3_MASK)
#define LLWU_ME_WUME4_MASK                       (0x10U)
#define LLWU_ME_WUME4_SHIFT                      (4U)
#define LLWU_ME_WUME4(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME4_SHIFT)) & LLWU_ME_WUME4_MASK)
#define LLWU_ME_WUME5_MASK                       (0x20U)
#define LLWU_ME_WUME5_SHIFT                      (5U)
#define LLWU_ME_WUME5(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME5_SHIFT)) & LLWU_ME_WUME5_MASK)
#define LLWU_ME_WUME6_MASK                       (0x40U)
#define LLWU_ME_WUME6_SHIFT                      (6U)
#define LLWU_ME_WUME6(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME6_SHIFT)) & LLWU_ME_WUME6_MASK)
#define LLWU_ME_WUME7_MASK                       (0x80U)
#define LLWU_ME_WUME7_SHIFT                      (7U)
#define LLWU_ME_WUME7(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_ME_WUME7_SHIFT)) & LLWU_ME_WUME7_MASK)

/*! @name DE - LLWU Module DMA Enable register */
#define LLWU_DE_WUDE0_MASK                       (0x1U)
#define LLWU_DE_WUDE0_SHIFT                      (0U)
#define LLWU_DE_WUDE0(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE0_SHIFT)) & LLWU_DE_WUDE0_MASK)
#define LLWU_DE_WUDE1_MASK                       (0x2U)
#define LLWU_DE_WUDE1_SHIFT                      (1U)
#define LLWU_DE_WUDE1(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE1_SHIFT)) & LLWU_DE_WUDE1_MASK)
#define LLWU_DE_WUDE2_MASK                       (0x4U)
#define LLWU_DE_WUDE2_SHIFT                      (2U)
#define LLWU_DE_WUDE2(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE2_SHIFT)) & LLWU_DE_WUDE2_MASK)
#define LLWU_DE_WUDE3_MASK                       (0x8U)
#define LLWU_DE_WUDE3_SHIFT                      (3U)
#define LLWU_DE_WUDE3(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE3_SHIFT)) & LLWU_DE_WUDE3_MASK)
#define LLWU_DE_WUDE4_MASK                       (0x10U)
#define LLWU_DE_WUDE4_SHIFT                      (4U)
#define LLWU_DE_WUDE4(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE4_SHIFT)) & LLWU_DE_WUDE4_MASK)
#define LLWU_DE_WUDE5_MASK                       (0x20U)
#define LLWU_DE_WUDE5_SHIFT                      (5U)
#define LLWU_DE_WUDE5(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE5_SHIFT)) & LLWU_DE_WUDE5_MASK)
#define LLWU_DE_WUDE6_MASK                       (0x40U)
#define LLWU_DE_WUDE6_SHIFT                      (6U)
#define LLWU_DE_WUDE6(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE6_SHIFT)) & LLWU_DE_WUDE6_MASK)
#define LLWU_DE_WUDE7_MASK                       (0x80U)
#define LLWU_DE_WUDE7_SHIFT                      (7U)
#define LLWU_DE_WUDE7(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_DE_WUDE7_SHIFT)) & LLWU_DE_WUDE7_MASK)

/*! @name PF - LLWU Pin Flag register */
#define LLWU_PF_WUF0_MASK                        (0x1U)
#define LLWU_PF_WUF0_SHIFT                       (0U)
#define LLWU_PF_WUF0(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF0_SHIFT)) & LLWU_PF_WUF0_MASK)
#define LLWU_PF_WUF1_MASK                        (0x2U)
#define LLWU_PF_WUF1_SHIFT                       (1U)
#define LLWU_PF_WUF1(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF1_SHIFT)) & LLWU_PF_WUF1_MASK)
#define LLWU_PF_WUF2_MASK                        (0x4U)
#define LLWU_PF_WUF2_SHIFT                       (2U)
#define LLWU_PF_WUF2(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF2_SHIFT)) & LLWU_PF_WUF2_MASK)
#define LLWU_PF_WUF3_MASK                        (0x8U)
#define LLWU_PF_WUF3_SHIFT                       (3U)
#define LLWU_PF_WUF3(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF3_SHIFT)) & LLWU_PF_WUF3_MASK)
#define LLWU_PF_WUF4_MASK                        (0x10U)
#define LLWU_PF_WUF4_SHIFT                       (4U)
#define LLWU_PF_WUF4(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF4_SHIFT)) & LLWU_PF_WUF4_MASK)
#define LLWU_PF_WUF5_MASK                        (0x20U)
#define LLWU_PF_WUF5_SHIFT                       (5U)
#define LLWU_PF_WUF5(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF5_SHIFT)) & LLWU_PF_WUF5_MASK)
#define LLWU_PF_WUF6_MASK                        (0x40U)
#define LLWU_PF_WUF6_SHIFT                       (6U)
#define LLWU_PF_WUF6(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF6_SHIFT)) & LLWU_PF_WUF6_MASK)
#define LLWU_PF_WUF7_MASK                        (0x80U)
#define LLWU_PF_WUF7_SHIFT                       (7U)
#define LLWU_PF_WUF7(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF7_SHIFT)) & LLWU_PF_WUF7_MASK)
#define LLWU_PF_WUF8_MASK                        (0x100U)
#define LLWU_PF_WUF8_SHIFT                       (8U)
#define LLWU_PF_WUF8(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF8_SHIFT)) & LLWU_PF_WUF8_MASK)
#define LLWU_PF_WUF9_MASK                        (0x200U)
#define LLWU_PF_WUF9_SHIFT                       (9U)
#define LLWU_PF_WUF9(x)                          (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF9_SHIFT)) & LLWU_PF_WUF9_MASK)
#define LLWU_PF_WUF10_MASK                       (0x400U)
#define LLWU_PF_WUF10_SHIFT                      (10U)
#define LLWU_PF_WUF10(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF10_SHIFT)) & LLWU_PF_WUF10_MASK)
#define LLWU_PF_WUF11_MASK                       (0x800U)
#define LLWU_PF_WUF11_SHIFT                      (11U)
#define LLWU_PF_WUF11(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF11_SHIFT)) & LLWU_PF_WUF11_MASK)
#define LLWU_PF_WUF12_MASK                       (0x1000U)
#define LLWU_PF_WUF12_SHIFT                      (12U)
#define LLWU_PF_WUF12(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF12_SHIFT)) & LLWU_PF_WUF12_MASK)
#define LLWU_PF_WUF13_MASK                       (0x2000U)
#define LLWU_PF_WUF13_SHIFT                      (13U)
#define LLWU_PF_WUF13(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF13_SHIFT)) & LLWU_PF_WUF13_MASK)
#define LLWU_PF_WUF14_MASK                       (0x4000U)
#define LLWU_PF_WUF14_SHIFT                      (14U)
#define LLWU_PF_WUF14(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF14_SHIFT)) & LLWU_PF_WUF14_MASK)
#define LLWU_PF_WUF15_MASK                       (0x8000U)
#define LLWU_PF_WUF15_SHIFT                      (15U)
#define LLWU_PF_WUF15(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF15_SHIFT)) & LLWU_PF_WUF15_MASK)
#define LLWU_PF_WUF16_MASK                       (0x10000U)
#define LLWU_PF_WUF16_SHIFT                      (16U)
#define LLWU_PF_WUF16(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF16_SHIFT)) & LLWU_PF_WUF16_MASK)
#define LLWU_PF_WUF17_MASK                       (0x20000U)
#define LLWU_PF_WUF17_SHIFT                      (17U)
#define LLWU_PF_WUF17(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF17_SHIFT)) & LLWU_PF_WUF17_MASK)
#define LLWU_PF_WUF18_MASK                       (0x40000U)
#define LLWU_PF_WUF18_SHIFT                      (18U)
#define LLWU_PF_WUF18(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF18_SHIFT)) & LLWU_PF_WUF18_MASK)
#define LLWU_PF_WUF19_MASK                       (0x80000U)
#define LLWU_PF_WUF19_SHIFT                      (19U)
#define LLWU_PF_WUF19(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF19_SHIFT)) & LLWU_PF_WUF19_MASK)
#define LLWU_PF_WUF20_MASK                       (0x100000U)
#define LLWU_PF_WUF20_SHIFT                      (20U)
#define LLWU_PF_WUF20(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF20_SHIFT)) & LLWU_PF_WUF20_MASK)
#define LLWU_PF_WUF21_MASK                       (0x200000U)
#define LLWU_PF_WUF21_SHIFT                      (21U)
#define LLWU_PF_WUF21(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF21_SHIFT)) & LLWU_PF_WUF21_MASK)
#define LLWU_PF_WUF22_MASK                       (0x400000U)
#define LLWU_PF_WUF22_SHIFT                      (22U)
#define LLWU_PF_WUF22(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF22_SHIFT)) & LLWU_PF_WUF22_MASK)
#define LLWU_PF_WUF23_MASK                       (0x800000U)
#define LLWU_PF_WUF23_SHIFT                      (23U)
#define LLWU_PF_WUF23(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF23_SHIFT)) & LLWU_PF_WUF23_MASK)
#define LLWU_PF_WUF24_MASK                       (0x1000000U)
#define LLWU_PF_WUF24_SHIFT                      (24U)
#define LLWU_PF_WUF24(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF24_SHIFT)) & LLWU_PF_WUF24_MASK)
#define LLWU_PF_WUF25_MASK                       (0x2000000U)
#define LLWU_PF_WUF25_SHIFT                      (25U)
#define LLWU_PF_WUF25(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF25_SHIFT)) & LLWU_PF_WUF25_MASK)
#define LLWU_PF_WUF26_MASK                       (0x4000000U)
#define LLWU_PF_WUF26_SHIFT                      (26U)
#define LLWU_PF_WUF26(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF26_SHIFT)) & LLWU_PF_WUF26_MASK)
#define LLWU_PF_WUF27_MASK                       (0x8000000U)
#define LLWU_PF_WUF27_SHIFT                      (27U)
#define LLWU_PF_WUF27(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF27_SHIFT)) & LLWU_PF_WUF27_MASK)
#define LLWU_PF_WUF28_MASK                       (0x10000000U)
#define LLWU_PF_WUF28_SHIFT                      (28U)
#define LLWU_PF_WUF28(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF28_SHIFT)) & LLWU_PF_WUF28_MASK)
#define LLWU_PF_WUF29_MASK                       (0x20000000U)
#define LLWU_PF_WUF29_SHIFT                      (29U)
#define LLWU_PF_WUF29(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF29_SHIFT)) & LLWU_PF_WUF29_MASK)
#define LLWU_PF_WUF30_MASK                       (0x40000000U)
#define LLWU_PF_WUF30_SHIFT                      (30U)
#define LLWU_PF_WUF30(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF30_SHIFT)) & LLWU_PF_WUF30_MASK)
#define LLWU_PF_WUF31_MASK                       (0x80000000U)
#define LLWU_PF_WUF31_SHIFT                      (31U)
#define LLWU_PF_WUF31(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_PF_WUF31_SHIFT)) & LLWU_PF_WUF31_MASK)

/*! @name MF - LLWU Module Interrupt Flag register */
#define LLWU_MF_MWUF0_MASK                       (0x1U)
#define LLWU_MF_MWUF0_SHIFT                      (0U)
#define LLWU_MF_MWUF0(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF0_SHIFT)) & LLWU_MF_MWUF0_MASK)
#define LLWU_MF_MWUF1_MASK                       (0x2U)
#define LLWU_MF_MWUF1_SHIFT                      (1U)
#define LLWU_MF_MWUF1(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF1_SHIFT)) & LLWU_MF_MWUF1_MASK)
#define LLWU_MF_MWUF2_MASK                       (0x4U)
#define LLWU_MF_MWUF2_SHIFT                      (2U)
#define LLWU_MF_MWUF2(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF2_SHIFT)) & LLWU_MF_MWUF2_MASK)
#define LLWU_MF_MWUF3_MASK                       (0x8U)
#define LLWU_MF_MWUF3_SHIFT                      (3U)
#define LLWU_MF_MWUF3(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF3_SHIFT)) & LLWU_MF_MWUF3_MASK)
#define LLWU_MF_MWUF4_MASK                       (0x10U)
#define LLWU_MF_MWUF4_SHIFT                      (4U)
#define LLWU_MF_MWUF4(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF4_SHIFT)) & LLWU_MF_MWUF4_MASK)
#define LLWU_MF_MWUF5_MASK                       (0x20U)
#define LLWU_MF_MWUF5_SHIFT                      (5U)
#define LLWU_MF_MWUF5(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF5_SHIFT)) & LLWU_MF_MWUF5_MASK)
#define LLWU_MF_MWUF6_MASK                       (0x40U)
#define LLWU_MF_MWUF6_SHIFT                      (6U)
#define LLWU_MF_MWUF6(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF6_SHIFT)) & LLWU_MF_MWUF6_MASK)
#define LLWU_MF_MWUF7_MASK                       (0x80U)
#define LLWU_MF_MWUF7_SHIFT                      (7U)
#define LLWU_MF_MWUF7(x)                         (((uint32_t)(((uint32_t)(x)) << LLWU_MF_MWUF7_SHIFT)) & LLWU_MF_MWUF7_MASK)

/*! @name FILT - LLWU Pin Filter register */
#define LLWU_FILT_FILTSEL1_MASK                  (0x1FU)
#define LLWU_FILT_FILTSEL1_SHIFT                 (0U)
#define LLWU_FILT_FILTSEL1(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTSEL1_SHIFT)) & LLWU_FILT_FILTSEL1_MASK)
#define LLWU_FILT_FILTE1_MASK                    (0x60U)
#define LLWU_FILT_FILTE1_SHIFT                   (5U)
#define LLWU_FILT_FILTE1(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTE1_SHIFT)) & LLWU_FILT_FILTE1_MASK)
#define LLWU_FILT_FILTF1_MASK                    (0x80U)
#define LLWU_FILT_FILTF1_SHIFT                   (7U)
#define LLWU_FILT_FILTF1(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTF1_SHIFT)) & LLWU_FILT_FILTF1_MASK)
#define LLWU_FILT_FILTSEL2_MASK                  (0x1F00U)
#define LLWU_FILT_FILTSEL2_SHIFT                 (8U)
#define LLWU_FILT_FILTSEL2(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTSEL2_SHIFT)) & LLWU_FILT_FILTSEL2_MASK)
#define LLWU_FILT_FILTE2_MASK                    (0x6000U)
#define LLWU_FILT_FILTE2_SHIFT                   (13U)
#define LLWU_FILT_FILTE2(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTE2_SHIFT)) & LLWU_FILT_FILTE2_MASK)
#define LLWU_FILT_FILTF2_MASK                    (0x8000U)
#define LLWU_FILT_FILTF2_SHIFT                   (15U)
#define LLWU_FILT_FILTF2(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTF2_SHIFT)) & LLWU_FILT_FILTF2_MASK)
#define LLWU_FILT_FILTSEL3_MASK                  (0x1F0000U)
#define LLWU_FILT_FILTSEL3_SHIFT                 (16U)
#define LLWU_FILT_FILTSEL3(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTSEL3_SHIFT)) & LLWU_FILT_FILTSEL3_MASK)
#define LLWU_FILT_FILTE3_MASK                    (0x600000U)
#define LLWU_FILT_FILTE3_SHIFT                   (21U)
#define LLWU_FILT_FILTE3(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTE3_SHIFT)) & LLWU_FILT_FILTE3_MASK)
#define LLWU_FILT_FILTF3_MASK                    (0x800000U)
#define LLWU_FILT_FILTF3_SHIFT                   (23U)
#define LLWU_FILT_FILTF3(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTF3_SHIFT)) & LLWU_FILT_FILTF3_MASK)
#define LLWU_FILT_FILTSEL4_MASK                  (0x1F000000U)
#define LLWU_FILT_FILTSEL4_SHIFT                 (24U)
#define LLWU_FILT_FILTSEL4(x)                    (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTSEL4_SHIFT)) & LLWU_FILT_FILTSEL4_MASK)
#define LLWU_FILT_FILTE4_MASK                    (0x60000000U)
#define LLWU_FILT_FILTE4_SHIFT                   (29U)
#define LLWU_FILT_FILTE4(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTE4_SHIFT)) & LLWU_FILT_FILTE4_MASK)
#define LLWU_FILT_FILTF4_MASK                    (0x80000000U)
#define LLWU_FILT_FILTF4_SHIFT                   (31U)
#define LLWU_FILT_FILTF4(x)                      (((uint32_t)(((uint32_t)(x)) << LLWU_FILT_FILTF4_SHIFT)) & LLWU_FILT_FILTF4_MASK)


/*!
 * @}
 */ /* end of group LLWU_Register_Masks */


/* LLWU - Peripheral instance base addresses */
/** Peripheral LLWU0 base address */
#define LLWU0_BASE                               (0x40061000u)
/** Peripheral LLWU0 base pointer */
#define LLWU0                                    ((LLWU_Type *)LLWU0_BASE)
/** Array initializer of LLWU peripheral base addresses */
#define LLWU_BASE_ADDRS                          { LLWU0_BASE }
/** Array initializer of LLWU peripheral base pointers */
#define LLWU_BASE_PTRS                           { LLWU0 }
/** Interrupt vectors for the LLWU peripheral type */
#define LLWU_IRQS                                { LLWU0_IRQn }

/*!
 * @}
 */ /* end of group LLWU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPI2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Peripheral_Access_Layer LPI2C Peripheral Access Layer
 * @{
 */

/** LPI2C - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
       uint8_t RESERVED_0[8];
  __IO uint32_t MCR;                               /**< Master Control Register, offset: 0x10 */
  __IO uint32_t MSR;                               /**< Master Status Register, offset: 0x14 */
  __IO uint32_t MIER;                              /**< Master Interrupt Enable Register, offset: 0x18 */
  __IO uint32_t MDER;                              /**< Master DMA Enable Register, offset: 0x1C */
  __IO uint32_t MCFGR0;                            /**< Master Configuration Register 0, offset: 0x20 */
  __IO uint32_t MCFGR1;                            /**< Master Configuration Register 1, offset: 0x24 */
  __IO uint32_t MCFGR2;                            /**< Master Configuration Register 2, offset: 0x28 */
  __IO uint32_t MCFGR3;                            /**< Master Configuration Register 3, offset: 0x2C */
       uint8_t RESERVED_1[16];
  __IO uint32_t MDMR;                              /**< Master Data Match Register, offset: 0x40 */
       uint8_t RESERVED_2[4];
  __IO uint32_t MCCR0;                             /**< Master Clock Configuration Register 0, offset: 0x48 */
       uint8_t RESERVED_3[4];
  __IO uint32_t MCCR1;                             /**< Master Clock Configuration Register 1, offset: 0x50 */
       uint8_t RESERVED_4[4];
  __IO uint32_t MFCR;                              /**< Master FIFO Control Register, offset: 0x58 */
  __I  uint32_t MFSR;                              /**< Master FIFO Status Register, offset: 0x5C */
  __O  uint32_t MTDR;                              /**< Master Transmit Data Register, offset: 0x60 */
       uint8_t RESERVED_5[12];
  __I  uint32_t MRDR;                              /**< Master Receive Data Register, offset: 0x70 */
       uint8_t RESERVED_6[156];
  __IO uint32_t SCR;                               /**< Slave Control Register, offset: 0x110 */
  __IO uint32_t SSR;                               /**< Slave Status Register, offset: 0x114 */
  __IO uint32_t SIER;                              /**< Slave Interrupt Enable Register, offset: 0x118 */
  __IO uint32_t SDER;                              /**< Slave DMA Enable Register, offset: 0x11C */
       uint8_t RESERVED_7[4];
  __IO uint32_t SCFGR1;                            /**< Slave Configuration Register 1, offset: 0x124 */
  __IO uint32_t SCFGR2;                            /**< Slave Configuration Register 2, offset: 0x128 */
       uint8_t RESERVED_8[20];
  __IO uint32_t SAMR;                              /**< Slave Address Match Register, offset: 0x140 */
       uint8_t RESERVED_9[12];
  __I  uint32_t SASR;                              /**< Slave Address Status Register, offset: 0x150 */
  __IO uint32_t STAR;                              /**< Slave Transmit ACK Register, offset: 0x154 */
       uint8_t RESERVED_10[8];
  __O  uint32_t STDR;                              /**< Slave Transmit Data Register, offset: 0x160 */
       uint8_t RESERVED_11[12];
  __I  uint32_t SRDR;                              /**< Slave Receive Data Register, offset: 0x170 */
} LPI2C_Type;

/* ----------------------------------------------------------------------------
   -- LPI2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Register_Masks LPI2C Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define LPI2C_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPI2C_VERID_FEATURE_SHIFT                (0U)
#define LPI2C_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_FEATURE_SHIFT)) & LPI2C_VERID_FEATURE_MASK)
#define LPI2C_VERID_MINOR_MASK                   (0xFF0000U)
#define LPI2C_VERID_MINOR_SHIFT                  (16U)
#define LPI2C_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MINOR_SHIFT)) & LPI2C_VERID_MINOR_MASK)
#define LPI2C_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPI2C_VERID_MAJOR_SHIFT                  (24U)
#define LPI2C_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MAJOR_SHIFT)) & LPI2C_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define LPI2C_PARAM_MTXFIFO_MASK                 (0xFU)
#define LPI2C_PARAM_MTXFIFO_SHIFT                (0U)
#define LPI2C_PARAM_MTXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MTXFIFO_SHIFT)) & LPI2C_PARAM_MTXFIFO_MASK)
#define LPI2C_PARAM_MRXFIFO_MASK                 (0xF00U)
#define LPI2C_PARAM_MRXFIFO_SHIFT                (8U)
#define LPI2C_PARAM_MRXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MRXFIFO_SHIFT)) & LPI2C_PARAM_MRXFIFO_MASK)

/*! @name MCR - Master Control Register */
#define LPI2C_MCR_MEN_MASK                       (0x1U)
#define LPI2C_MCR_MEN_SHIFT                      (0U)
#define LPI2C_MCR_MEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_MEN_SHIFT)) & LPI2C_MCR_MEN_MASK)
#define LPI2C_MCR_RST_MASK                       (0x2U)
#define LPI2C_MCR_RST_SHIFT                      (1U)
#define LPI2C_MCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RST_SHIFT)) & LPI2C_MCR_RST_MASK)
#define LPI2C_MCR_DOZEN_MASK                     (0x4U)
#define LPI2C_MCR_DOZEN_SHIFT                    (2U)
#define LPI2C_MCR_DOZEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DOZEN_SHIFT)) & LPI2C_MCR_DOZEN_MASK)
#define LPI2C_MCR_DBGEN_MASK                     (0x8U)
#define LPI2C_MCR_DBGEN_SHIFT                    (3U)
#define LPI2C_MCR_DBGEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DBGEN_SHIFT)) & LPI2C_MCR_DBGEN_MASK)
#define LPI2C_MCR_RTF_MASK                       (0x100U)
#define LPI2C_MCR_RTF_SHIFT                      (8U)
#define LPI2C_MCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RTF_SHIFT)) & LPI2C_MCR_RTF_MASK)
#define LPI2C_MCR_RRF_MASK                       (0x200U)
#define LPI2C_MCR_RRF_SHIFT                      (9U)
#define LPI2C_MCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RRF_SHIFT)) & LPI2C_MCR_RRF_MASK)

/*! @name MSR - Master Status Register */
#define LPI2C_MSR_TDF_MASK                       (0x1U)
#define LPI2C_MSR_TDF_SHIFT                      (0U)
#define LPI2C_MSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_TDF_SHIFT)) & LPI2C_MSR_TDF_MASK)
#define LPI2C_MSR_RDF_MASK                       (0x2U)
#define LPI2C_MSR_RDF_SHIFT                      (1U)
#define LPI2C_MSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_RDF_SHIFT)) & LPI2C_MSR_RDF_MASK)
#define LPI2C_MSR_EPF_MASK                       (0x100U)
#define LPI2C_MSR_EPF_SHIFT                      (8U)
#define LPI2C_MSR_EPF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_EPF_SHIFT)) & LPI2C_MSR_EPF_MASK)
#define LPI2C_MSR_SDF_MASK                       (0x200U)
#define LPI2C_MSR_SDF_SHIFT                      (9U)
#define LPI2C_MSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_SDF_SHIFT)) & LPI2C_MSR_SDF_MASK)
#define LPI2C_MSR_NDF_MASK                       (0x400U)
#define LPI2C_MSR_NDF_SHIFT                      (10U)
#define LPI2C_MSR_NDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_NDF_SHIFT)) & LPI2C_MSR_NDF_MASK)
#define LPI2C_MSR_ALF_MASK                       (0x800U)
#define LPI2C_MSR_ALF_SHIFT                      (11U)
#define LPI2C_MSR_ALF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_ALF_SHIFT)) & LPI2C_MSR_ALF_MASK)
#define LPI2C_MSR_FEF_MASK                       (0x1000U)
#define LPI2C_MSR_FEF_SHIFT                      (12U)
#define LPI2C_MSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_FEF_SHIFT)) & LPI2C_MSR_FEF_MASK)
#define LPI2C_MSR_PLTF_MASK                      (0x2000U)
#define LPI2C_MSR_PLTF_SHIFT                     (13U)
#define LPI2C_MSR_PLTF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_PLTF_SHIFT)) & LPI2C_MSR_PLTF_MASK)
#define LPI2C_MSR_DMF_MASK                       (0x4000U)
#define LPI2C_MSR_DMF_SHIFT                      (14U)
#define LPI2C_MSR_DMF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_DMF_SHIFT)) & LPI2C_MSR_DMF_MASK)
#define LPI2C_MSR_MBF_MASK                       (0x1000000U)
#define LPI2C_MSR_MBF_SHIFT                      (24U)
#define LPI2C_MSR_MBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_MBF_SHIFT)) & LPI2C_MSR_MBF_MASK)
#define LPI2C_MSR_BBF_MASK                       (0x2000000U)
#define LPI2C_MSR_BBF_SHIFT                      (25U)
#define LPI2C_MSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_BBF_SHIFT)) & LPI2C_MSR_BBF_MASK)

/*! @name MIER - Master Interrupt Enable Register */
#define LPI2C_MIER_TDIE_MASK                     (0x1U)
#define LPI2C_MIER_TDIE_SHIFT                    (0U)
#define LPI2C_MIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_TDIE_SHIFT)) & LPI2C_MIER_TDIE_MASK)
#define LPI2C_MIER_RDIE_MASK                     (0x2U)
#define LPI2C_MIER_RDIE_SHIFT                    (1U)
#define LPI2C_MIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_RDIE_SHIFT)) & LPI2C_MIER_RDIE_MASK)
#define LPI2C_MIER_EPIE_MASK                     (0x100U)
#define LPI2C_MIER_EPIE_SHIFT                    (8U)
#define LPI2C_MIER_EPIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_EPIE_SHIFT)) & LPI2C_MIER_EPIE_MASK)
#define LPI2C_MIER_SDIE_MASK                     (0x200U)
#define LPI2C_MIER_SDIE_SHIFT                    (9U)
#define LPI2C_MIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_SDIE_SHIFT)) & LPI2C_MIER_SDIE_MASK)
#define LPI2C_MIER_NDIE_MASK                     (0x400U)
#define LPI2C_MIER_NDIE_SHIFT                    (10U)
#define LPI2C_MIER_NDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_NDIE_SHIFT)) & LPI2C_MIER_NDIE_MASK)
#define LPI2C_MIER_ALIE_MASK                     (0x800U)
#define LPI2C_MIER_ALIE_SHIFT                    (11U)
#define LPI2C_MIER_ALIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_ALIE_SHIFT)) & LPI2C_MIER_ALIE_MASK)
#define LPI2C_MIER_FEIE_MASK                     (0x1000U)
#define LPI2C_MIER_FEIE_SHIFT                    (12U)
#define LPI2C_MIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_FEIE_SHIFT)) & LPI2C_MIER_FEIE_MASK)
#define LPI2C_MIER_PLTIE_MASK                    (0x2000U)
#define LPI2C_MIER_PLTIE_SHIFT                   (13U)
#define LPI2C_MIER_PLTIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_PLTIE_SHIFT)) & LPI2C_MIER_PLTIE_MASK)
#define LPI2C_MIER_DMIE_MASK                     (0x4000U)
#define LPI2C_MIER_DMIE_SHIFT                    (14U)
#define LPI2C_MIER_DMIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_DMIE_SHIFT)) & LPI2C_MIER_DMIE_MASK)

/*! @name MDER - Master DMA Enable Register */
#define LPI2C_MDER_TDDE_MASK                     (0x1U)
#define LPI2C_MDER_TDDE_SHIFT                    (0U)
#define LPI2C_MDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_TDDE_SHIFT)) & LPI2C_MDER_TDDE_MASK)
#define LPI2C_MDER_RDDE_MASK                     (0x2U)
#define LPI2C_MDER_RDDE_SHIFT                    (1U)
#define LPI2C_MDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_RDDE_SHIFT)) & LPI2C_MDER_RDDE_MASK)

/*! @name MCFGR0 - Master Configuration Register 0 */
#define LPI2C_MCFGR0_HREN_MASK                   (0x1U)
#define LPI2C_MCFGR0_HREN_SHIFT                  (0U)
#define LPI2C_MCFGR0_HREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HREN_SHIFT)) & LPI2C_MCFGR0_HREN_MASK)
#define LPI2C_MCFGR0_HRPOL_MASK                  (0x2U)
#define LPI2C_MCFGR0_HRPOL_SHIFT                 (1U)
#define LPI2C_MCFGR0_HRPOL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRPOL_SHIFT)) & LPI2C_MCFGR0_HRPOL_MASK)
#define LPI2C_MCFGR0_HRSEL_MASK                  (0x4U)
#define LPI2C_MCFGR0_HRSEL_SHIFT                 (2U)
#define LPI2C_MCFGR0_HRSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRSEL_SHIFT)) & LPI2C_MCFGR0_HRSEL_MASK)
#define LPI2C_MCFGR0_CIRFIFO_MASK                (0x100U)
#define LPI2C_MCFGR0_CIRFIFO_SHIFT               (8U)
#define LPI2C_MCFGR0_CIRFIFO(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_CIRFIFO_SHIFT)) & LPI2C_MCFGR0_CIRFIFO_MASK)
#define LPI2C_MCFGR0_RDMO_MASK                   (0x200U)
#define LPI2C_MCFGR0_RDMO_SHIFT                  (9U)
#define LPI2C_MCFGR0_RDMO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_RDMO_SHIFT)) & LPI2C_MCFGR0_RDMO_MASK)

/*! @name MCFGR1 - Master Configuration Register 1 */
#define LPI2C_MCFGR1_PRESCALE_MASK               (0x7U)
#define LPI2C_MCFGR1_PRESCALE_SHIFT              (0U)
#define LPI2C_MCFGR1_PRESCALE(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PRESCALE_SHIFT)) & LPI2C_MCFGR1_PRESCALE_MASK)
#define LPI2C_MCFGR1_AUTOSTOP_MASK               (0x100U)
#define LPI2C_MCFGR1_AUTOSTOP_SHIFT              (8U)
#define LPI2C_MCFGR1_AUTOSTOP(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_AUTOSTOP_SHIFT)) & LPI2C_MCFGR1_AUTOSTOP_MASK)
#define LPI2C_MCFGR1_IGNACK_MASK                 (0x200U)
#define LPI2C_MCFGR1_IGNACK_SHIFT                (9U)
#define LPI2C_MCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_IGNACK_SHIFT)) & LPI2C_MCFGR1_IGNACK_MASK)
#define LPI2C_MCFGR1_TIMECFG_MASK                (0x400U)
#define LPI2C_MCFGR1_TIMECFG_SHIFT               (10U)
#define LPI2C_MCFGR1_TIMECFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_TIMECFG_SHIFT)) & LPI2C_MCFGR1_TIMECFG_MASK)
#define LPI2C_MCFGR1_MATCFG_MASK                 (0x70000U)
#define LPI2C_MCFGR1_MATCFG_SHIFT                (16U)
#define LPI2C_MCFGR1_MATCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_MATCFG_SHIFT)) & LPI2C_MCFGR1_MATCFG_MASK)
#define LPI2C_MCFGR1_PINCFG_MASK                 (0x7000000U)
#define LPI2C_MCFGR1_PINCFG_SHIFT                (24U)
#define LPI2C_MCFGR1_PINCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PINCFG_SHIFT)) & LPI2C_MCFGR1_PINCFG_MASK)

/*! @name MCFGR2 - Master Configuration Register 2 */
#define LPI2C_MCFGR2_BUSIDLE_MASK                (0xFFFU)
#define LPI2C_MCFGR2_BUSIDLE_SHIFT               (0U)
#define LPI2C_MCFGR2_BUSIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_BUSIDLE_SHIFT)) & LPI2C_MCFGR2_BUSIDLE_MASK)
#define LPI2C_MCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_MCFGR2_FILTSCL_SHIFT               (16U)
#define LPI2C_MCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSCL_SHIFT)) & LPI2C_MCFGR2_FILTSCL_MASK)
#define LPI2C_MCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_MCFGR2_FILTSDA_SHIFT               (24U)
#define LPI2C_MCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSDA_SHIFT)) & LPI2C_MCFGR2_FILTSDA_MASK)

/*! @name MCFGR3 - Master Configuration Register 3 */
#define LPI2C_MCFGR3_PINLOW_MASK                 (0xFFF00U)
#define LPI2C_MCFGR3_PINLOW_SHIFT                (8U)
#define LPI2C_MCFGR3_PINLOW(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR3_PINLOW_SHIFT)) & LPI2C_MCFGR3_PINLOW_MASK)

/*! @name MDMR - Master Data Match Register */
#define LPI2C_MDMR_MATCH0_MASK                   (0xFFU)
#define LPI2C_MDMR_MATCH0_SHIFT                  (0U)
#define LPI2C_MDMR_MATCH0(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH0_SHIFT)) & LPI2C_MDMR_MATCH0_MASK)
#define LPI2C_MDMR_MATCH1_MASK                   (0xFF0000U)
#define LPI2C_MDMR_MATCH1_SHIFT                  (16U)
#define LPI2C_MDMR_MATCH1(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH1_SHIFT)) & LPI2C_MDMR_MATCH1_MASK)

/*! @name MCCR0 - Master Clock Configuration Register 0 */
#define LPI2C_MCCR0_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR0_CLKLO_SHIFT                  (0U)
#define LPI2C_MCCR0_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKLO_SHIFT)) & LPI2C_MCCR0_CLKLO_MASK)
#define LPI2C_MCCR0_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR0_CLKHI_SHIFT                  (8U)
#define LPI2C_MCCR0_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKHI_SHIFT)) & LPI2C_MCCR0_CLKHI_MASK)
#define LPI2C_MCCR0_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR0_SETHOLD_SHIFT                (16U)
#define LPI2C_MCCR0_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_SETHOLD_SHIFT)) & LPI2C_MCCR0_SETHOLD_MASK)
#define LPI2C_MCCR0_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR0_DATAVD_SHIFT                 (24U)
#define LPI2C_MCCR0_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_DATAVD_SHIFT)) & LPI2C_MCCR0_DATAVD_MASK)

/*! @name MCCR1 - Master Clock Configuration Register 1 */
#define LPI2C_MCCR1_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR1_CLKLO_SHIFT                  (0U)
#define LPI2C_MCCR1_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKLO_SHIFT)) & LPI2C_MCCR1_CLKLO_MASK)
#define LPI2C_MCCR1_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR1_CLKHI_SHIFT                  (8U)
#define LPI2C_MCCR1_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKHI_SHIFT)) & LPI2C_MCCR1_CLKHI_MASK)
#define LPI2C_MCCR1_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR1_SETHOLD_SHIFT                (16U)
#define LPI2C_MCCR1_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_SETHOLD_SHIFT)) & LPI2C_MCCR1_SETHOLD_MASK)
#define LPI2C_MCCR1_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR1_DATAVD_SHIFT                 (24U)
#define LPI2C_MCCR1_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_DATAVD_SHIFT)) & LPI2C_MCCR1_DATAVD_MASK)

/*! @name MFCR - Master FIFO Control Register */
#define LPI2C_MFCR_TXWATER_MASK                  (0xFFU)
#define LPI2C_MFCR_TXWATER_SHIFT                 (0U)
#define LPI2C_MFCR_TXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_TXWATER_SHIFT)) & LPI2C_MFCR_TXWATER_MASK)
#define LPI2C_MFCR_RXWATER_MASK                  (0xFF0000U)
#define LPI2C_MFCR_RXWATER_SHIFT                 (16U)
#define LPI2C_MFCR_RXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_RXWATER_SHIFT)) & LPI2C_MFCR_RXWATER_MASK)

/*! @name MFSR - Master FIFO Status Register */
#define LPI2C_MFSR_TXCOUNT_MASK                  (0xFFU)
#define LPI2C_MFSR_TXCOUNT_SHIFT                 (0U)
#define LPI2C_MFSR_TXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_TXCOUNT_SHIFT)) & LPI2C_MFSR_TXCOUNT_MASK)
#define LPI2C_MFSR_RXCOUNT_MASK                  (0xFF0000U)
#define LPI2C_MFSR_RXCOUNT_SHIFT                 (16U)
#define LPI2C_MFSR_RXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_RXCOUNT_SHIFT)) & LPI2C_MFSR_RXCOUNT_MASK)

/*! @name MTDR - Master Transmit Data Register */
#define LPI2C_MTDR_DATA_MASK                     (0xFFU)
#define LPI2C_MTDR_DATA_SHIFT                    (0U)
#define LPI2C_MTDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_DATA_SHIFT)) & LPI2C_MTDR_DATA_MASK)
#define LPI2C_MTDR_CMD_MASK                      (0x700U)
#define LPI2C_MTDR_CMD_SHIFT                     (8U)
#define LPI2C_MTDR_CMD(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_CMD_SHIFT)) & LPI2C_MTDR_CMD_MASK)

/*! @name MRDR - Master Receive Data Register */
#define LPI2C_MRDR_DATA_MASK                     (0xFFU)
#define LPI2C_MRDR_DATA_SHIFT                    (0U)
#define LPI2C_MRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_DATA_SHIFT)) & LPI2C_MRDR_DATA_MASK)
#define LPI2C_MRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_MRDR_RXEMPTY_SHIFT                 (14U)
#define LPI2C_MRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_RXEMPTY_SHIFT)) & LPI2C_MRDR_RXEMPTY_MASK)

/*! @name SCR - Slave Control Register */
#define LPI2C_SCR_SEN_MASK                       (0x1U)
#define LPI2C_SCR_SEN_SHIFT                      (0U)
#define LPI2C_SCR_SEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_SEN_SHIFT)) & LPI2C_SCR_SEN_MASK)
#define LPI2C_SCR_RST_MASK                       (0x2U)
#define LPI2C_SCR_RST_SHIFT                      (1U)
#define LPI2C_SCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RST_SHIFT)) & LPI2C_SCR_RST_MASK)
#define LPI2C_SCR_FILTEN_MASK                    (0x10U)
#define LPI2C_SCR_FILTEN_SHIFT                   (4U)
#define LPI2C_SCR_FILTEN(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTEN_SHIFT)) & LPI2C_SCR_FILTEN_MASK)
#define LPI2C_SCR_FILTDZ_MASK                    (0x20U)
#define LPI2C_SCR_FILTDZ_SHIFT                   (5U)
#define LPI2C_SCR_FILTDZ(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTDZ_SHIFT)) & LPI2C_SCR_FILTDZ_MASK)
#define LPI2C_SCR_RTF_MASK                       (0x100U)
#define LPI2C_SCR_RTF_SHIFT                      (8U)
#define LPI2C_SCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RTF_SHIFT)) & LPI2C_SCR_RTF_MASK)
#define LPI2C_SCR_RRF_MASK                       (0x200U)
#define LPI2C_SCR_RRF_SHIFT                      (9U)
#define LPI2C_SCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RRF_SHIFT)) & LPI2C_SCR_RRF_MASK)

/*! @name SSR - Slave Status Register */
#define LPI2C_SSR_TDF_MASK                       (0x1U)
#define LPI2C_SSR_TDF_SHIFT                      (0U)
#define LPI2C_SSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TDF_SHIFT)) & LPI2C_SSR_TDF_MASK)
#define LPI2C_SSR_RDF_MASK                       (0x2U)
#define LPI2C_SSR_RDF_SHIFT                      (1U)
#define LPI2C_SSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RDF_SHIFT)) & LPI2C_SSR_RDF_MASK)
#define LPI2C_SSR_AVF_MASK                       (0x4U)
#define LPI2C_SSR_AVF_SHIFT                      (2U)
#define LPI2C_SSR_AVF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AVF_SHIFT)) & LPI2C_SSR_AVF_MASK)
#define LPI2C_SSR_TAF_MASK                       (0x8U)
#define LPI2C_SSR_TAF_SHIFT                      (3U)
#define LPI2C_SSR_TAF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TAF_SHIFT)) & LPI2C_SSR_TAF_MASK)
#define LPI2C_SSR_RSF_MASK                       (0x100U)
#define LPI2C_SSR_RSF_SHIFT                      (8U)
#define LPI2C_SSR_RSF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RSF_SHIFT)) & LPI2C_SSR_RSF_MASK)
#define LPI2C_SSR_SDF_MASK                       (0x200U)
#define LPI2C_SSR_SDF_SHIFT                      (9U)
#define LPI2C_SSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SDF_SHIFT)) & LPI2C_SSR_SDF_MASK)
#define LPI2C_SSR_BEF_MASK                       (0x400U)
#define LPI2C_SSR_BEF_SHIFT                      (10U)
#define LPI2C_SSR_BEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BEF_SHIFT)) & LPI2C_SSR_BEF_MASK)
#define LPI2C_SSR_FEF_MASK                       (0x800U)
#define LPI2C_SSR_FEF_SHIFT                      (11U)
#define LPI2C_SSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_FEF_SHIFT)) & LPI2C_SSR_FEF_MASK)
#define LPI2C_SSR_AM0F_MASK                      (0x1000U)
#define LPI2C_SSR_AM0F_SHIFT                     (12U)
#define LPI2C_SSR_AM0F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM0F_SHIFT)) & LPI2C_SSR_AM0F_MASK)
#define LPI2C_SSR_AM1F_MASK                      (0x2000U)
#define LPI2C_SSR_AM1F_SHIFT                     (13U)
#define LPI2C_SSR_AM1F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM1F_SHIFT)) & LPI2C_SSR_AM1F_MASK)
#define LPI2C_SSR_GCF_MASK                       (0x4000U)
#define LPI2C_SSR_GCF_SHIFT                      (14U)
#define LPI2C_SSR_GCF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_GCF_SHIFT)) & LPI2C_SSR_GCF_MASK)
#define LPI2C_SSR_SARF_MASK                      (0x8000U)
#define LPI2C_SSR_SARF_SHIFT                     (15U)
#define LPI2C_SSR_SARF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SARF_SHIFT)) & LPI2C_SSR_SARF_MASK)
#define LPI2C_SSR_SBF_MASK                       (0x1000000U)
#define LPI2C_SSR_SBF_SHIFT                      (24U)
#define LPI2C_SSR_SBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SBF_SHIFT)) & LPI2C_SSR_SBF_MASK)
#define LPI2C_SSR_BBF_MASK                       (0x2000000U)
#define LPI2C_SSR_BBF_SHIFT                      (25U)
#define LPI2C_SSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BBF_SHIFT)) & LPI2C_SSR_BBF_MASK)

/*! @name SIER - Slave Interrupt Enable Register */
#define LPI2C_SIER_TDIE_MASK                     (0x1U)
#define LPI2C_SIER_TDIE_SHIFT                    (0U)
#define LPI2C_SIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TDIE_SHIFT)) & LPI2C_SIER_TDIE_MASK)
#define LPI2C_SIER_RDIE_MASK                     (0x2U)
#define LPI2C_SIER_RDIE_SHIFT                    (1U)
#define LPI2C_SIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RDIE_SHIFT)) & LPI2C_SIER_RDIE_MASK)
#define LPI2C_SIER_AVIE_MASK                     (0x4U)
#define LPI2C_SIER_AVIE_SHIFT                    (2U)
#define LPI2C_SIER_AVIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AVIE_SHIFT)) & LPI2C_SIER_AVIE_MASK)
#define LPI2C_SIER_TAIE_MASK                     (0x8U)
#define LPI2C_SIER_TAIE_SHIFT                    (3U)
#define LPI2C_SIER_TAIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TAIE_SHIFT)) & LPI2C_SIER_TAIE_MASK)
#define LPI2C_SIER_RSIE_MASK                     (0x100U)
#define LPI2C_SIER_RSIE_SHIFT                    (8U)
#define LPI2C_SIER_RSIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RSIE_SHIFT)) & LPI2C_SIER_RSIE_MASK)
#define LPI2C_SIER_SDIE_MASK                     (0x200U)
#define LPI2C_SIER_SDIE_SHIFT                    (9U)
#define LPI2C_SIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SDIE_SHIFT)) & LPI2C_SIER_SDIE_MASK)
#define LPI2C_SIER_BEIE_MASK                     (0x400U)
#define LPI2C_SIER_BEIE_SHIFT                    (10U)
#define LPI2C_SIER_BEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_BEIE_SHIFT)) & LPI2C_SIER_BEIE_MASK)
#define LPI2C_SIER_FEIE_MASK                     (0x800U)
#define LPI2C_SIER_FEIE_SHIFT                    (11U)
#define LPI2C_SIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_FEIE_SHIFT)) & LPI2C_SIER_FEIE_MASK)
#define LPI2C_SIER_AM0IE_MASK                    (0x1000U)
#define LPI2C_SIER_AM0IE_SHIFT                   (12U)
#define LPI2C_SIER_AM0IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM0IE_SHIFT)) & LPI2C_SIER_AM0IE_MASK)
#define LPI2C_SIER_AM1F_MASK                     (0x2000U)
#define LPI2C_SIER_AM1F_SHIFT                    (13U)
#define LPI2C_SIER_AM1F(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM1F_SHIFT)) & LPI2C_SIER_AM1F_MASK)
#define LPI2C_SIER_GCIE_MASK                     (0x4000U)
#define LPI2C_SIER_GCIE_SHIFT                    (14U)
#define LPI2C_SIER_GCIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_GCIE_SHIFT)) & LPI2C_SIER_GCIE_MASK)
#define LPI2C_SIER_SARIE_MASK                    (0x8000U)
#define LPI2C_SIER_SARIE_SHIFT                   (15U)
#define LPI2C_SIER_SARIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SARIE_SHIFT)) & LPI2C_SIER_SARIE_MASK)

/*! @name SDER - Slave DMA Enable Register */
#define LPI2C_SDER_TDDE_MASK                     (0x1U)
#define LPI2C_SDER_TDDE_SHIFT                    (0U)
#define LPI2C_SDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_TDDE_SHIFT)) & LPI2C_SDER_TDDE_MASK)
#define LPI2C_SDER_RDDE_MASK                     (0x2U)
#define LPI2C_SDER_RDDE_SHIFT                    (1U)
#define LPI2C_SDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_RDDE_SHIFT)) & LPI2C_SDER_RDDE_MASK)
#define LPI2C_SDER_AVDE_MASK                     (0x4U)
#define LPI2C_SDER_AVDE_SHIFT                    (2U)
#define LPI2C_SDER_AVDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_AVDE_SHIFT)) & LPI2C_SDER_AVDE_MASK)

/*! @name SCFGR1 - Slave Configuration Register 1 */
#define LPI2C_SCFGR1_ADRSTALL_MASK               (0x1U)
#define LPI2C_SCFGR1_ADRSTALL_SHIFT              (0U)
#define LPI2C_SCFGR1_ADRSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADRSTALL_SHIFT)) & LPI2C_SCFGR1_ADRSTALL_MASK)
#define LPI2C_SCFGR1_RXSTALL_MASK                (0x2U)
#define LPI2C_SCFGR1_RXSTALL_SHIFT               (1U)
#define LPI2C_SCFGR1_RXSTALL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXSTALL_SHIFT)) & LPI2C_SCFGR1_RXSTALL_MASK)
#define LPI2C_SCFGR1_TXDSTALL_MASK               (0x4U)
#define LPI2C_SCFGR1_TXDSTALL_SHIFT              (2U)
#define LPI2C_SCFGR1_TXDSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXDSTALL_SHIFT)) & LPI2C_SCFGR1_TXDSTALL_MASK)
#define LPI2C_SCFGR1_ACKSTALL_MASK               (0x8U)
#define LPI2C_SCFGR1_ACKSTALL_SHIFT              (3U)
#define LPI2C_SCFGR1_ACKSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ACKSTALL_SHIFT)) & LPI2C_SCFGR1_ACKSTALL_MASK)
#define LPI2C_SCFGR1_GCEN_MASK                   (0x100U)
#define LPI2C_SCFGR1_GCEN_SHIFT                  (8U)
#define LPI2C_SCFGR1_GCEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_GCEN_SHIFT)) & LPI2C_SCFGR1_GCEN_MASK)
#define LPI2C_SCFGR1_SAEN_MASK                   (0x200U)
#define LPI2C_SCFGR1_SAEN_SHIFT                  (9U)
#define LPI2C_SCFGR1_SAEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_SAEN_SHIFT)) & LPI2C_SCFGR1_SAEN_MASK)
#define LPI2C_SCFGR1_TXCFG_MASK                  (0x400U)
#define LPI2C_SCFGR1_TXCFG_SHIFT                 (10U)
#define LPI2C_SCFGR1_TXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXCFG_SHIFT)) & LPI2C_SCFGR1_TXCFG_MASK)
#define LPI2C_SCFGR1_RXCFG_MASK                  (0x800U)
#define LPI2C_SCFGR1_RXCFG_SHIFT                 (11U)
#define LPI2C_SCFGR1_RXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXCFG_SHIFT)) & LPI2C_SCFGR1_RXCFG_MASK)
#define LPI2C_SCFGR1_IGNACK_MASK                 (0x1000U)
#define LPI2C_SCFGR1_IGNACK_SHIFT                (12U)
#define LPI2C_SCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_IGNACK_SHIFT)) & LPI2C_SCFGR1_IGNACK_MASK)
#define LPI2C_SCFGR1_HSMEN_MASK                  (0x2000U)
#define LPI2C_SCFGR1_HSMEN_SHIFT                 (13U)
#define LPI2C_SCFGR1_HSMEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_HSMEN_SHIFT)) & LPI2C_SCFGR1_HSMEN_MASK)
#define LPI2C_SCFGR1_ADDRCFG_MASK                (0x70000U)
#define LPI2C_SCFGR1_ADDRCFG_SHIFT               (16U)
#define LPI2C_SCFGR1_ADDRCFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADDRCFG_SHIFT)) & LPI2C_SCFGR1_ADDRCFG_MASK)

/*! @name SCFGR2 - Slave Configuration Register 2 */
#define LPI2C_SCFGR2_CLKHOLD_MASK                (0xFU)
#define LPI2C_SCFGR2_CLKHOLD_SHIFT               (0U)
#define LPI2C_SCFGR2_CLKHOLD(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_CLKHOLD_SHIFT)) & LPI2C_SCFGR2_CLKHOLD_MASK)
#define LPI2C_SCFGR2_DATAVD_MASK                 (0x3F00U)
#define LPI2C_SCFGR2_DATAVD_SHIFT                (8U)
#define LPI2C_SCFGR2_DATAVD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_DATAVD_SHIFT)) & LPI2C_SCFGR2_DATAVD_MASK)
#define LPI2C_SCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_SCFGR2_FILTSCL_SHIFT               (16U)
#define LPI2C_SCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSCL_SHIFT)) & LPI2C_SCFGR2_FILTSCL_MASK)
#define LPI2C_SCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_SCFGR2_FILTSDA_SHIFT               (24U)
#define LPI2C_SCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSDA_SHIFT)) & LPI2C_SCFGR2_FILTSDA_MASK)

/*! @name SAMR - Slave Address Match Register */
#define LPI2C_SAMR_ADDR0_MASK                    (0x7FEU)
#define LPI2C_SAMR_ADDR0_SHIFT                   (1U)
#define LPI2C_SAMR_ADDR0(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR0_SHIFT)) & LPI2C_SAMR_ADDR0_MASK)
#define LPI2C_SAMR_ADDR1_MASK                    (0x7FE0000U)
#define LPI2C_SAMR_ADDR1_SHIFT                   (17U)
#define LPI2C_SAMR_ADDR1(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR1_SHIFT)) & LPI2C_SAMR_ADDR1_MASK)

/*! @name SASR - Slave Address Status Register */
#define LPI2C_SASR_RADDR_MASK                    (0x7FFU)
#define LPI2C_SASR_RADDR_SHIFT                   (0U)
#define LPI2C_SASR_RADDR(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_RADDR_SHIFT)) & LPI2C_SASR_RADDR_MASK)
#define LPI2C_SASR_ANV_MASK                      (0x4000U)
#define LPI2C_SASR_ANV_SHIFT                     (14U)
#define LPI2C_SASR_ANV(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_ANV_SHIFT)) & LPI2C_SASR_ANV_MASK)

/*! @name STAR - Slave Transmit ACK Register */
#define LPI2C_STAR_TXNACK_MASK                   (0x1U)
#define LPI2C_STAR_TXNACK_SHIFT                  (0U)
#define LPI2C_STAR_TXNACK(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_STAR_TXNACK_SHIFT)) & LPI2C_STAR_TXNACK_MASK)

/*! @name STDR - Slave Transmit Data Register */
#define LPI2C_STDR_DATA_MASK                     (0xFFU)
#define LPI2C_STDR_DATA_SHIFT                    (0U)
#define LPI2C_STDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_STDR_DATA_SHIFT)) & LPI2C_STDR_DATA_MASK)

/*! @name SRDR - Slave Receive Data Register */
#define LPI2C_SRDR_DATA_MASK                     (0xFFU)
#define LPI2C_SRDR_DATA_SHIFT                    (0U)
#define LPI2C_SRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_DATA_SHIFT)) & LPI2C_SRDR_DATA_MASK)
#define LPI2C_SRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_SRDR_RXEMPTY_SHIFT                 (14U)
#define LPI2C_SRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_RXEMPTY_SHIFT)) & LPI2C_SRDR_RXEMPTY_MASK)
#define LPI2C_SRDR_SOF_MASK                      (0x8000U)
#define LPI2C_SRDR_SOF_SHIFT                     (15U)
#define LPI2C_SRDR_SOF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_SOF_SHIFT)) & LPI2C_SRDR_SOF_MASK)


/*!
 * @}
 */ /* end of group LPI2C_Register_Masks */


/* LPI2C - Peripheral instance base addresses */
/** Peripheral LPI2C0 base address */
#define LPI2C0_BASE                              (0x400C0000u)
/** Peripheral LPI2C0 base pointer */
#define LPI2C0                                   ((LPI2C_Type *)LPI2C0_BASE)
/** Peripheral LPI2C1 base address */
#define LPI2C1_BASE                              (0x400C1000u)
/** Peripheral LPI2C1 base pointer */
#define LPI2C1                                   ((LPI2C_Type *)LPI2C1_BASE)
/** Peripheral LPI2C2 base address */
#define LPI2C2_BASE                              (0x40042000u)
/** Peripheral LPI2C2 base pointer */
#define LPI2C2                                   ((LPI2C_Type *)LPI2C2_BASE)
/** Array initializer of LPI2C peripheral base addresses */
#define LPI2C_BASE_ADDRS                         { LPI2C0_BASE, LPI2C1_BASE, LPI2C2_BASE }
/** Array initializer of LPI2C peripheral base pointers */
#define LPI2C_BASE_PTRS                          { LPI2C0, LPI2C1, LPI2C2 }
/** Interrupt vectors for the LPI2C peripheral type */
#define LPI2C_IRQS                               { LPI2C0_IRQn, LPI2C1_IRQn, LPI2C2_IRQn }

/*!
 * @}
 */ /* end of group LPI2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPIT_Peripheral_Access_Layer LPIT Peripheral Access Layer
 * @{
 */

/** LPIT - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t MCR;                               /**< Module Control Register, offset: 0x8 */
  __IO uint32_t MSR;                               /**< Module Status Register, offset: 0xC */
  __IO uint32_t MIER;                              /**< Module Interrupt Enable Register, offset: 0x10 */
  __IO uint32_t SETTEN;                            /**< Set Timer Enable Register, offset: 0x14 */
  __IO uint32_t CLRTEN;                            /**< Clear Timer Enable Register, offset: 0x18 */
       uint8_t RESERVED_0[4];
  struct {                                         /* offset: 0x20, array step: 0x10 */
    __IO uint32_t TVAL;                              /**< Timer Value Register, array offset: 0x20, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value, array offset: 0x24, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x28, array step: 0x10 */
         uint8_t RESERVED_0[4];
  } CHANNEL[4];
} LPIT_Type;

/* ----------------------------------------------------------------------------
   -- LPIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPIT_Register_Masks LPIT Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define LPIT_VERID_FEATURE_MASK                  (0xFFFFU)
#define LPIT_VERID_FEATURE_SHIFT                 (0U)
#define LPIT_VERID_FEATURE(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_FEATURE_SHIFT)) & LPIT_VERID_FEATURE_MASK)
#define LPIT_VERID_MINOR_MASK                    (0xFF0000U)
#define LPIT_VERID_MINOR_SHIFT                   (16U)
#define LPIT_VERID_MINOR(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_MINOR_SHIFT)) & LPIT_VERID_MINOR_MASK)
#define LPIT_VERID_MAJOR_MASK                    (0xFF000000U)
#define LPIT_VERID_MAJOR_SHIFT                   (24U)
#define LPIT_VERID_MAJOR(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_MAJOR_SHIFT)) & LPIT_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define LPIT_PARAM_CHANNEL_MASK                  (0xFFU)
#define LPIT_PARAM_CHANNEL_SHIFT                 (0U)
#define LPIT_PARAM_CHANNEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_PARAM_CHANNEL_SHIFT)) & LPIT_PARAM_CHANNEL_MASK)
#define LPIT_PARAM_EXT_TRIG_MASK                 (0xFF00U)
#define LPIT_PARAM_EXT_TRIG_SHIFT                (8U)
#define LPIT_PARAM_EXT_TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_PARAM_EXT_TRIG_SHIFT)) & LPIT_PARAM_EXT_TRIG_MASK)

/*! @name MCR - Module Control Register */
#define LPIT_MCR_M_CEN_MASK                      (0x1U)
#define LPIT_MCR_M_CEN_SHIFT                     (0U)
#define LPIT_MCR_M_CEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_M_CEN_SHIFT)) & LPIT_MCR_M_CEN_MASK)
#define LPIT_MCR_SW_RST_MASK                     (0x2U)
#define LPIT_MCR_SW_RST_SHIFT                    (1U)
#define LPIT_MCR_SW_RST(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_SW_RST_SHIFT)) & LPIT_MCR_SW_RST_MASK)
#define LPIT_MCR_DOZE_EN_MASK                    (0x4U)
#define LPIT_MCR_DOZE_EN_SHIFT                   (2U)
#define LPIT_MCR_DOZE_EN(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_DOZE_EN_SHIFT)) & LPIT_MCR_DOZE_EN_MASK)
#define LPIT_MCR_DBG_EN_MASK                     (0x8U)
#define LPIT_MCR_DBG_EN_SHIFT                    (3U)
#define LPIT_MCR_DBG_EN(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_DBG_EN_SHIFT)) & LPIT_MCR_DBG_EN_MASK)

/*! @name MSR - Module Status Register */
#define LPIT_MSR_TIF0_MASK                       (0x1U)
#define LPIT_MSR_TIF0_SHIFT                      (0U)
#define LPIT_MSR_TIF0(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF0_SHIFT)) & LPIT_MSR_TIF0_MASK)
#define LPIT_MSR_TIF1_MASK                       (0x2U)
#define LPIT_MSR_TIF1_SHIFT                      (1U)
#define LPIT_MSR_TIF1(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF1_SHIFT)) & LPIT_MSR_TIF1_MASK)
#define LPIT_MSR_TIF2_MASK                       (0x4U)
#define LPIT_MSR_TIF2_SHIFT                      (2U)
#define LPIT_MSR_TIF2(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF2_SHIFT)) & LPIT_MSR_TIF2_MASK)
#define LPIT_MSR_TIF3_MASK                       (0x8U)
#define LPIT_MSR_TIF3_SHIFT                      (3U)
#define LPIT_MSR_TIF3(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF3_SHIFT)) & LPIT_MSR_TIF3_MASK)

/*! @name MIER - Module Interrupt Enable Register */
#define LPIT_MIER_TIE0_MASK                      (0x1U)
#define LPIT_MIER_TIE0_SHIFT                     (0U)
#define LPIT_MIER_TIE0(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE0_SHIFT)) & LPIT_MIER_TIE0_MASK)
#define LPIT_MIER_TIE1_MASK                      (0x2U)
#define LPIT_MIER_TIE1_SHIFT                     (1U)
#define LPIT_MIER_TIE1(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE1_SHIFT)) & LPIT_MIER_TIE1_MASK)
#define LPIT_MIER_TIE2_MASK                      (0x4U)
#define LPIT_MIER_TIE2_SHIFT                     (2U)
#define LPIT_MIER_TIE2(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE2_SHIFT)) & LPIT_MIER_TIE2_MASK)
#define LPIT_MIER_TIE3_MASK                      (0x8U)
#define LPIT_MIER_TIE3_SHIFT                     (3U)
#define LPIT_MIER_TIE3(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE3_SHIFT)) & LPIT_MIER_TIE3_MASK)

/*! @name SETTEN - Set Timer Enable Register */
#define LPIT_SETTEN_SET_T_EN_0_MASK              (0x1U)
#define LPIT_SETTEN_SET_T_EN_0_SHIFT             (0U)
#define LPIT_SETTEN_SET_T_EN_0(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_0_SHIFT)) & LPIT_SETTEN_SET_T_EN_0_MASK)
#define LPIT_SETTEN_SET_T_EN_1_MASK              (0x2U)
#define LPIT_SETTEN_SET_T_EN_1_SHIFT             (1U)
#define LPIT_SETTEN_SET_T_EN_1(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_1_SHIFT)) & LPIT_SETTEN_SET_T_EN_1_MASK)
#define LPIT_SETTEN_SET_T_EN_2_MASK              (0x4U)
#define LPIT_SETTEN_SET_T_EN_2_SHIFT             (2U)
#define LPIT_SETTEN_SET_T_EN_2(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_2_SHIFT)) & LPIT_SETTEN_SET_T_EN_2_MASK)
#define LPIT_SETTEN_SET_T_EN_3_MASK              (0x8U)
#define LPIT_SETTEN_SET_T_EN_3_SHIFT             (3U)
#define LPIT_SETTEN_SET_T_EN_3(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_3_SHIFT)) & LPIT_SETTEN_SET_T_EN_3_MASK)

/*! @name CLRTEN - Clear Timer Enable Register */
#define LPIT_CLRTEN_CLR_T_EN_0_MASK              (0x1U)
#define LPIT_CLRTEN_CLR_T_EN_0_SHIFT             (0U)
#define LPIT_CLRTEN_CLR_T_EN_0(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_0_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_0_MASK)
#define LPIT_CLRTEN_CLR_T_EN_1_MASK              (0x2U)
#define LPIT_CLRTEN_CLR_T_EN_1_SHIFT             (1U)
#define LPIT_CLRTEN_CLR_T_EN_1(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_1_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_1_MASK)
#define LPIT_CLRTEN_CLR_T_EN_2_MASK              (0x4U)
#define LPIT_CLRTEN_CLR_T_EN_2_SHIFT             (2U)
#define LPIT_CLRTEN_CLR_T_EN_2(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_2_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_2_MASK)
#define LPIT_CLRTEN_CLR_T_EN_3_MASK              (0x8U)
#define LPIT_CLRTEN_CLR_T_EN_3_SHIFT             (3U)
#define LPIT_CLRTEN_CLR_T_EN_3(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_3_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_3_MASK)

/*! @name TVAL - Timer Value Register */
#define LPIT_TVAL_TMR_VAL_MASK                   (0xFFFFFFFFU)
#define LPIT_TVAL_TMR_VAL_SHIFT                  (0U)
#define LPIT_TVAL_TMR_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << LPIT_TVAL_TMR_VAL_SHIFT)) & LPIT_TVAL_TMR_VAL_MASK)

/* The count of LPIT_TVAL */
#define LPIT_TVAL_COUNT                          (4U)

/*! @name CVAL - Current Timer Value */
#define LPIT_CVAL_TMR_CUR_VAL_MASK               (0xFFFFFFFFU)
#define LPIT_CVAL_TMR_CUR_VAL_SHIFT              (0U)
#define LPIT_CVAL_TMR_CUR_VAL(x)                 (((uint32_t)(((uint32_t)(x)) << LPIT_CVAL_TMR_CUR_VAL_SHIFT)) & LPIT_CVAL_TMR_CUR_VAL_MASK)

/* The count of LPIT_CVAL */
#define LPIT_CVAL_COUNT                          (4U)

/*! @name TCTRL - Timer Control Register */
#define LPIT_TCTRL_T_EN_MASK                     (0x1U)
#define LPIT_TCTRL_T_EN_SHIFT                    (0U)
#define LPIT_TCTRL_T_EN(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_T_EN_SHIFT)) & LPIT_TCTRL_T_EN_MASK)
#define LPIT_TCTRL_CHAIN_MASK                    (0x2U)
#define LPIT_TCTRL_CHAIN_SHIFT                   (1U)
#define LPIT_TCTRL_CHAIN(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_CHAIN_SHIFT)) & LPIT_TCTRL_CHAIN_MASK)
#define LPIT_TCTRL_MODE_MASK                     (0xCU)
#define LPIT_TCTRL_MODE_SHIFT                    (2U)
#define LPIT_TCTRL_MODE(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_MODE_SHIFT)) & LPIT_TCTRL_MODE_MASK)
#define LPIT_TCTRL_TSOT_MASK                     (0x10000U)
#define LPIT_TCTRL_TSOT_SHIFT                    (16U)
#define LPIT_TCTRL_TSOT(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_TSOT_SHIFT)) & LPIT_TCTRL_TSOT_MASK)
#define LPIT_TCTRL_TSOI_MASK                     (0x20000U)
#define LPIT_TCTRL_TSOI_SHIFT                    (17U)
#define LPIT_TCTRL_TSOI(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_TSOI_SHIFT)) & LPIT_TCTRL_TSOI_MASK)
#define LPIT_TCTRL_TROT_MASK                     (0x40000U)
#define LPIT_TCTRL_TROT_SHIFT                    (18U)
#define LPIT_TCTRL_TROT(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_TROT_SHIFT)) & LPIT_TCTRL_TROT_MASK)
#define LPIT_TCTRL_TRG_SRC_MASK                  (0x800000U)
#define LPIT_TCTRL_TRG_SRC_SHIFT                 (23U)
#define LPIT_TCTRL_TRG_SRC(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_TRG_SRC_SHIFT)) & LPIT_TCTRL_TRG_SRC_MASK)
#define LPIT_TCTRL_TRG_SEL_MASK                  (0xF000000U)
#define LPIT_TCTRL_TRG_SEL_SHIFT                 (24U)
#define LPIT_TCTRL_TRG_SEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_TCTRL_TRG_SEL_SHIFT)) & LPIT_TCTRL_TRG_SEL_MASK)

/* The count of LPIT_TCTRL */
#define LPIT_TCTRL_COUNT                         (4U)


/*!
 * @}
 */ /* end of group LPIT_Register_Masks */


/* LPIT - Peripheral instance base addresses */
/** Peripheral LPIT0 base address */
#define LPIT0_BASE                               (0x40030000u)
/** Peripheral LPIT0 base pointer */
#define LPIT0                                    ((LPIT_Type *)LPIT0_BASE)
/** Array initializer of LPIT peripheral base addresses */
#define LPIT_BASE_ADDRS                          { LPIT0_BASE }
/** Array initializer of LPIT peripheral base pointers */
#define LPIT_BASE_PTRS                           { LPIT0 }
/** Interrupt vectors for the LPIT peripheral type */
#define LPIT_IRQS                                { LPIT0_IRQn }

/*!
 * @}
 */ /* end of group LPIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPSPI_Peripheral_Access_Layer LPSPI Peripheral Access Layer
 * @{
 */

/** LPSPI - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
       uint8_t RESERVED_0[8];
  __IO uint32_t CR;                                /**< Control Register, offset: 0x10 */
  __IO uint32_t SR;                                /**< Status Register, offset: 0x14 */
  __IO uint32_t IER;                               /**< Interrupt Enable Register, offset: 0x18 */
  __IO uint32_t DER;                               /**< DMA Enable Register, offset: 0x1C */
  __IO uint32_t CFGR0;                             /**< Configuration Register 0, offset: 0x20 */
  __IO uint32_t CFGR1;                             /**< Configuration Register 1, offset: 0x24 */
       uint8_t RESERVED_1[8];
  __IO uint32_t DMR0;                              /**< Data Match Register 0, offset: 0x30 */
  __IO uint32_t DMR1;                              /**< Data Match Register 1, offset: 0x34 */
       uint8_t RESERVED_2[8];
  __IO uint32_t CCR;                               /**< Clock Configuration Register, offset: 0x40 */
       uint8_t RESERVED_3[20];
  __IO uint32_t FCR;                               /**< FIFO Control Register, offset: 0x58 */
  __I  uint32_t FSR;                               /**< FIFO Status Register, offset: 0x5C */
  __IO uint32_t TCR;                               /**< Transmit Command Register, offset: 0x60 */
  __O  uint32_t TDR;                               /**< Transmit Data Register, offset: 0x64 */
       uint8_t RESERVED_4[8];
  __I  uint32_t RSR;                               /**< Receive Status Register, offset: 0x70 */
  __I  uint32_t RDR;                               /**< Receive Data Register, offset: 0x74 */
} LPSPI_Type;

/* ----------------------------------------------------------------------------
   -- LPSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPSPI_Register_Masks LPSPI Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define LPSPI_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPSPI_VERID_FEATURE_SHIFT                (0U)
#define LPSPI_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_FEATURE_SHIFT)) & LPSPI_VERID_FEATURE_MASK)
#define LPSPI_VERID_MINOR_MASK                   (0xFF0000U)
#define LPSPI_VERID_MINOR_SHIFT                  (16U)
#define LPSPI_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_MINOR_SHIFT)) & LPSPI_VERID_MINOR_MASK)
#define LPSPI_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPSPI_VERID_MAJOR_SHIFT                  (24U)
#define LPSPI_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_MAJOR_SHIFT)) & LPSPI_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define LPSPI_PARAM_TXFIFO_MASK                  (0xFFU)
#define LPSPI_PARAM_TXFIFO_SHIFT                 (0U)
#define LPSPI_PARAM_TXFIFO(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_PARAM_TXFIFO_SHIFT)) & LPSPI_PARAM_TXFIFO_MASK)
#define LPSPI_PARAM_RXFIFO_MASK                  (0xFF00U)
#define LPSPI_PARAM_RXFIFO_SHIFT                 (8U)
#define LPSPI_PARAM_RXFIFO(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_PARAM_RXFIFO_SHIFT)) & LPSPI_PARAM_RXFIFO_MASK)

/*! @name CR - Control Register */
#define LPSPI_CR_MEN_MASK                        (0x1U)
#define LPSPI_CR_MEN_SHIFT                       (0U)
#define LPSPI_CR_MEN(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_MEN_SHIFT)) & LPSPI_CR_MEN_MASK)
#define LPSPI_CR_RST_MASK                        (0x2U)
#define LPSPI_CR_RST_SHIFT                       (1U)
#define LPSPI_CR_RST(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RST_SHIFT)) & LPSPI_CR_RST_MASK)
#define LPSPI_CR_DOZEN_MASK                      (0x4U)
#define LPSPI_CR_DOZEN_SHIFT                     (2U)
#define LPSPI_CR_DOZEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_DOZEN_SHIFT)) & LPSPI_CR_DOZEN_MASK)
#define LPSPI_CR_DBGEN_MASK                      (0x8U)
#define LPSPI_CR_DBGEN_SHIFT                     (3U)
#define LPSPI_CR_DBGEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_DBGEN_SHIFT)) & LPSPI_CR_DBGEN_MASK)
#define LPSPI_CR_RTF_MASK                        (0x100U)
#define LPSPI_CR_RTF_SHIFT                       (8U)
#define LPSPI_CR_RTF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RTF_SHIFT)) & LPSPI_CR_RTF_MASK)
#define LPSPI_CR_RRF_MASK                        (0x200U)
#define LPSPI_CR_RRF_SHIFT                       (9U)
#define LPSPI_CR_RRF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RRF_SHIFT)) & LPSPI_CR_RRF_MASK)

/*! @name SR - Status Register */
#define LPSPI_SR_TDF_MASK                        (0x1U)
#define LPSPI_SR_TDF_SHIFT                       (0U)
#define LPSPI_SR_TDF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TDF_SHIFT)) & LPSPI_SR_TDF_MASK)
#define LPSPI_SR_RDF_MASK                        (0x2U)
#define LPSPI_SR_RDF_SHIFT                       (1U)
#define LPSPI_SR_RDF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_RDF_SHIFT)) & LPSPI_SR_RDF_MASK)
#define LPSPI_SR_WCF_MASK                        (0x100U)
#define LPSPI_SR_WCF_SHIFT                       (8U)
#define LPSPI_SR_WCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_WCF_SHIFT)) & LPSPI_SR_WCF_MASK)
#define LPSPI_SR_FCF_MASK                        (0x200U)
#define LPSPI_SR_FCF_SHIFT                       (9U)
#define LPSPI_SR_FCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_FCF_SHIFT)) & LPSPI_SR_FCF_MASK)
#define LPSPI_SR_TCF_MASK                        (0x400U)
#define LPSPI_SR_TCF_SHIFT                       (10U)
#define LPSPI_SR_TCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TCF_SHIFT)) & LPSPI_SR_TCF_MASK)
#define LPSPI_SR_TEF_MASK                        (0x800U)
#define LPSPI_SR_TEF_SHIFT                       (11U)
#define LPSPI_SR_TEF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TEF_SHIFT)) & LPSPI_SR_TEF_MASK)
#define LPSPI_SR_REF_MASK                        (0x1000U)
#define LPSPI_SR_REF_SHIFT                       (12U)
#define LPSPI_SR_REF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_REF_SHIFT)) & LPSPI_SR_REF_MASK)
#define LPSPI_SR_DMF_MASK                        (0x2000U)
#define LPSPI_SR_DMF_SHIFT                       (13U)
#define LPSPI_SR_DMF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_DMF_SHIFT)) & LPSPI_SR_DMF_MASK)
#define LPSPI_SR_MBF_MASK                        (0x1000000U)
#define LPSPI_SR_MBF_SHIFT                       (24U)
#define LPSPI_SR_MBF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_MBF_SHIFT)) & LPSPI_SR_MBF_MASK)

/*! @name IER - Interrupt Enable Register */
#define LPSPI_IER_TDIE_MASK                      (0x1U)
#define LPSPI_IER_TDIE_SHIFT                     (0U)
#define LPSPI_IER_TDIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TDIE_SHIFT)) & LPSPI_IER_TDIE_MASK)
#define LPSPI_IER_RDIE_MASK                      (0x2U)
#define LPSPI_IER_RDIE_SHIFT                     (1U)
#define LPSPI_IER_RDIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_RDIE_SHIFT)) & LPSPI_IER_RDIE_MASK)
#define LPSPI_IER_WCIE_MASK                      (0x100U)
#define LPSPI_IER_WCIE_SHIFT                     (8U)
#define LPSPI_IER_WCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_WCIE_SHIFT)) & LPSPI_IER_WCIE_MASK)
#define LPSPI_IER_FCIE_MASK                      (0x200U)
#define LPSPI_IER_FCIE_SHIFT                     (9U)
#define LPSPI_IER_FCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_FCIE_SHIFT)) & LPSPI_IER_FCIE_MASK)
#define LPSPI_IER_TCIE_MASK                      (0x400U)
#define LPSPI_IER_TCIE_SHIFT                     (10U)
#define LPSPI_IER_TCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TCIE_SHIFT)) & LPSPI_IER_TCIE_MASK)
#define LPSPI_IER_TEIE_MASK                      (0x800U)
#define LPSPI_IER_TEIE_SHIFT                     (11U)
#define LPSPI_IER_TEIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TEIE_SHIFT)) & LPSPI_IER_TEIE_MASK)
#define LPSPI_IER_REIE_MASK                      (0x1000U)
#define LPSPI_IER_REIE_SHIFT                     (12U)
#define LPSPI_IER_REIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_REIE_SHIFT)) & LPSPI_IER_REIE_MASK)
#define LPSPI_IER_DMIE_MASK                      (0x2000U)
#define LPSPI_IER_DMIE_SHIFT                     (13U)
#define LPSPI_IER_DMIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_DMIE_SHIFT)) & LPSPI_IER_DMIE_MASK)

/*! @name DER - DMA Enable Register */
#define LPSPI_DER_TDDE_MASK                      (0x1U)
#define LPSPI_DER_TDDE_SHIFT                     (0U)
#define LPSPI_DER_TDDE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_DER_TDDE_SHIFT)) & LPSPI_DER_TDDE_MASK)
#define LPSPI_DER_RDDE_MASK                      (0x2U)
#define LPSPI_DER_RDDE_SHIFT                     (1U)
#define LPSPI_DER_RDDE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_DER_RDDE_SHIFT)) & LPSPI_DER_RDDE_MASK)

/*! @name CFGR0 - Configuration Register 0 */
#define LPSPI_CFGR0_HREN_MASK                    (0x1U)
#define LPSPI_CFGR0_HREN_SHIFT                   (0U)
#define LPSPI_CFGR0_HREN(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HREN_SHIFT)) & LPSPI_CFGR0_HREN_MASK)
#define LPSPI_CFGR0_HRPOL_MASK                   (0x2U)
#define LPSPI_CFGR0_HRPOL_SHIFT                  (1U)
#define LPSPI_CFGR0_HRPOL(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HRPOL_SHIFT)) & LPSPI_CFGR0_HRPOL_MASK)
#define LPSPI_CFGR0_HRSEL_MASK                   (0x4U)
#define LPSPI_CFGR0_HRSEL_SHIFT                  (2U)
#define LPSPI_CFGR0_HRSEL(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HRSEL_SHIFT)) & LPSPI_CFGR0_HRSEL_MASK)
#define LPSPI_CFGR0_CIRFIFO_MASK                 (0x100U)
#define LPSPI_CFGR0_CIRFIFO_SHIFT                (8U)
#define LPSPI_CFGR0_CIRFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_CIRFIFO_SHIFT)) & LPSPI_CFGR0_CIRFIFO_MASK)
#define LPSPI_CFGR0_RDMO_MASK                    (0x200U)
#define LPSPI_CFGR0_RDMO_SHIFT                   (9U)
#define LPSPI_CFGR0_RDMO(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_RDMO_SHIFT)) & LPSPI_CFGR0_RDMO_MASK)

/*! @name CFGR1 - Configuration Register 1 */
#define LPSPI_CFGR1_MASTER_MASK                  (0x1U)
#define LPSPI_CFGR1_MASTER_SHIFT                 (0U)
#define LPSPI_CFGR1_MASTER(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_MASTER_SHIFT)) & LPSPI_CFGR1_MASTER_MASK)
#define LPSPI_CFGR1_SAMPLE_MASK                  (0x2U)
#define LPSPI_CFGR1_SAMPLE_SHIFT                 (1U)
#define LPSPI_CFGR1_SAMPLE(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_SAMPLE_SHIFT)) & LPSPI_CFGR1_SAMPLE_MASK)
#define LPSPI_CFGR1_AUTOPCS_MASK                 (0x4U)
#define LPSPI_CFGR1_AUTOPCS_SHIFT                (2U)
#define LPSPI_CFGR1_AUTOPCS(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_AUTOPCS_SHIFT)) & LPSPI_CFGR1_AUTOPCS_MASK)
#define LPSPI_CFGR1_NOSTALL_MASK                 (0x8U)
#define LPSPI_CFGR1_NOSTALL_SHIFT                (3U)
#define LPSPI_CFGR1_NOSTALL(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_NOSTALL_SHIFT)) & LPSPI_CFGR1_NOSTALL_MASK)
#define LPSPI_CFGR1_PCSPOL_MASK                  (0xF00U)
#define LPSPI_CFGR1_PCSPOL_SHIFT                 (8U)
#define LPSPI_CFGR1_PCSPOL(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PCSPOL_SHIFT)) & LPSPI_CFGR1_PCSPOL_MASK)
#define LPSPI_CFGR1_MATCFG_MASK                  (0x70000U)
#define LPSPI_CFGR1_MATCFG_SHIFT                 (16U)
#define LPSPI_CFGR1_MATCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_MATCFG_SHIFT)) & LPSPI_CFGR1_MATCFG_MASK)
#define LPSPI_CFGR1_PINCFG_MASK                  (0x3000000U)
#define LPSPI_CFGR1_PINCFG_SHIFT                 (24U)
#define LPSPI_CFGR1_PINCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PINCFG_SHIFT)) & LPSPI_CFGR1_PINCFG_MASK)
#define LPSPI_CFGR1_OUTCFG_MASK                  (0x4000000U)
#define LPSPI_CFGR1_OUTCFG_SHIFT                 (26U)
#define LPSPI_CFGR1_OUTCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_OUTCFG_SHIFT)) & LPSPI_CFGR1_OUTCFG_MASK)
#define LPSPI_CFGR1_PCSCFG_MASK                  (0x8000000U)
#define LPSPI_CFGR1_PCSCFG_SHIFT                 (27U)
#define LPSPI_CFGR1_PCSCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PCSCFG_SHIFT)) & LPSPI_CFGR1_PCSCFG_MASK)

/*! @name DMR0 - Data Match Register 0 */
#define LPSPI_DMR0_MATCH0_MASK                   (0xFFFFFFFFU)
#define LPSPI_DMR0_MATCH0_SHIFT                  (0U)
#define LPSPI_DMR0_MATCH0(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_DMR0_MATCH0_SHIFT)) & LPSPI_DMR0_MATCH0_MASK)

/*! @name DMR1 - Data Match Register 1 */
#define LPSPI_DMR1_MATCH1_MASK                   (0xFFFFFFFFU)
#define LPSPI_DMR1_MATCH1_SHIFT                  (0U)
#define LPSPI_DMR1_MATCH1(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_DMR1_MATCH1_SHIFT)) & LPSPI_DMR1_MATCH1_MASK)

/*! @name CCR - Clock Configuration Register */
#define LPSPI_CCR_SCKDIV_MASK                    (0xFFU)
#define LPSPI_CCR_SCKDIV_SHIFT                   (0U)
#define LPSPI_CCR_SCKDIV(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_SCKDIV_SHIFT)) & LPSPI_CCR_SCKDIV_MASK)
#define LPSPI_CCR_DBT_MASK                       (0xFF00U)
#define LPSPI_CCR_DBT_SHIFT                      (8U)
#define LPSPI_CCR_DBT(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_DBT_SHIFT)) & LPSPI_CCR_DBT_MASK)
#define LPSPI_CCR_PCSSCK_MASK                    (0xFF0000U)
#define LPSPI_CCR_PCSSCK_SHIFT                   (16U)
#define LPSPI_CCR_PCSSCK(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_PCSSCK_SHIFT)) & LPSPI_CCR_PCSSCK_MASK)
#define LPSPI_CCR_SCKPCS_MASK                    (0xFF000000U)
#define LPSPI_CCR_SCKPCS_SHIFT                   (24U)
#define LPSPI_CCR_SCKPCS(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_SCKPCS_SHIFT)) & LPSPI_CCR_SCKPCS_MASK)

/*! @name FCR - FIFO Control Register */
#define LPSPI_FCR_TXWATER_MASK                   (0xFFU)
#define LPSPI_FCR_TXWATER_SHIFT                  (0U)
#define LPSPI_FCR_TXWATER(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FCR_TXWATER_SHIFT)) & LPSPI_FCR_TXWATER_MASK)
#define LPSPI_FCR_RXWATER_MASK                   (0xFF0000U)
#define LPSPI_FCR_RXWATER_SHIFT                  (16U)
#define LPSPI_FCR_RXWATER(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FCR_RXWATER_SHIFT)) & LPSPI_FCR_RXWATER_MASK)

/*! @name FSR - FIFO Status Register */
#define LPSPI_FSR_TXCOUNT_MASK                   (0xFFU)
#define LPSPI_FSR_TXCOUNT_SHIFT                  (0U)
#define LPSPI_FSR_TXCOUNT(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FSR_TXCOUNT_SHIFT)) & LPSPI_FSR_TXCOUNT_MASK)
#define LPSPI_FSR_RXCOUNT_MASK                   (0xFF0000U)
#define LPSPI_FSR_RXCOUNT_SHIFT                  (16U)
#define LPSPI_FSR_RXCOUNT(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FSR_RXCOUNT_SHIFT)) & LPSPI_FSR_RXCOUNT_MASK)

/*! @name TCR - Transmit Command Register */
#define LPSPI_TCR_FRAMESZ_MASK                   (0xFFFU)
#define LPSPI_TCR_FRAMESZ_SHIFT                  (0U)
#define LPSPI_TCR_FRAMESZ(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_FRAMESZ_SHIFT)) & LPSPI_TCR_FRAMESZ_MASK)
#define LPSPI_TCR_WIDTH_MASK                     (0x30000U)
#define LPSPI_TCR_WIDTH_SHIFT                    (16U)
#define LPSPI_TCR_WIDTH(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_WIDTH_SHIFT)) & LPSPI_TCR_WIDTH_MASK)
#define LPSPI_TCR_TXMSK_MASK                     (0x40000U)
#define LPSPI_TCR_TXMSK_SHIFT                    (18U)
#define LPSPI_TCR_TXMSK(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_TXMSK_SHIFT)) & LPSPI_TCR_TXMSK_MASK)
#define LPSPI_TCR_RXMSK_MASK                     (0x80000U)
#define LPSPI_TCR_RXMSK_SHIFT                    (19U)
#define LPSPI_TCR_RXMSK(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_RXMSK_SHIFT)) & LPSPI_TCR_RXMSK_MASK)
#define LPSPI_TCR_CONTC_MASK                     (0x100000U)
#define LPSPI_TCR_CONTC_SHIFT                    (20U)
#define LPSPI_TCR_CONTC(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CONTC_SHIFT)) & LPSPI_TCR_CONTC_MASK)
#define LPSPI_TCR_CONT_MASK                      (0x200000U)
#define LPSPI_TCR_CONT_SHIFT                     (21U)
#define LPSPI_TCR_CONT(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CONT_SHIFT)) & LPSPI_TCR_CONT_MASK)
#define LPSPI_TCR_BYSW_MASK                      (0x400000U)
#define LPSPI_TCR_BYSW_SHIFT                     (22U)
#define LPSPI_TCR_BYSW(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_BYSW_SHIFT)) & LPSPI_TCR_BYSW_MASK)
#define LPSPI_TCR_LSBF_MASK                      (0x800000U)
#define LPSPI_TCR_LSBF_SHIFT                     (23U)
#define LPSPI_TCR_LSBF(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_LSBF_SHIFT)) & LPSPI_TCR_LSBF_MASK)
#define LPSPI_TCR_PCS_MASK                       (0x3000000U)
#define LPSPI_TCR_PCS_SHIFT                      (24U)
#define LPSPI_TCR_PCS(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_PCS_SHIFT)) & LPSPI_TCR_PCS_MASK)
#define LPSPI_TCR_PRESCALE_MASK                  (0x38000000U)
#define LPSPI_TCR_PRESCALE_SHIFT                 (27U)
#define LPSPI_TCR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_PRESCALE_SHIFT)) & LPSPI_TCR_PRESCALE_MASK)
#define LPSPI_TCR_CPHA_MASK                      (0x40000000U)
#define LPSPI_TCR_CPHA_SHIFT                     (30U)
#define LPSPI_TCR_CPHA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CPHA_SHIFT)) & LPSPI_TCR_CPHA_MASK)
#define LPSPI_TCR_CPOL_MASK                      (0x80000000U)
#define LPSPI_TCR_CPOL_SHIFT                     (31U)
#define LPSPI_TCR_CPOL(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CPOL_SHIFT)) & LPSPI_TCR_CPOL_MASK)

/*! @name TDR - Transmit Data Register */
#define LPSPI_TDR_DATA_MASK                      (0xFFFFFFFFU)
#define LPSPI_TDR_DATA_SHIFT                     (0U)
#define LPSPI_TDR_DATA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TDR_DATA_SHIFT)) & LPSPI_TDR_DATA_MASK)

/*! @name RSR - Receive Status Register */
#define LPSPI_RSR_SOF_MASK                       (0x1U)
#define LPSPI_RSR_SOF_SHIFT                      (0U)
#define LPSPI_RSR_SOF(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_RSR_SOF_SHIFT)) & LPSPI_RSR_SOF_MASK)
#define LPSPI_RSR_RXEMPTY_MASK                   (0x2U)
#define LPSPI_RSR_RXEMPTY_SHIFT                  (1U)
#define LPSPI_RSR_RXEMPTY(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_RSR_RXEMPTY_SHIFT)) & LPSPI_RSR_RXEMPTY_MASK)

/*! @name RDR - Receive Data Register */
#define LPSPI_RDR_DATA_MASK                      (0xFFFFFFFFU)
#define LPSPI_RDR_DATA_SHIFT                     (0U)
#define LPSPI_RDR_DATA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_RDR_DATA_SHIFT)) & LPSPI_RDR_DATA_MASK)


/*!
 * @}
 */ /* end of group LPSPI_Register_Masks */


/* LPSPI - Peripheral instance base addresses */
/** Peripheral LPSPI0 base address */
#define LPSPI0_BASE                              (0x400BC000u)
/** Peripheral LPSPI0 base pointer */
#define LPSPI0                                   ((LPSPI_Type *)LPSPI0_BASE)
/** Peripheral LPSPI1 base address */
#define LPSPI1_BASE                              (0x400BD000u)
/** Peripheral LPSPI1 base pointer */
#define LPSPI1                                   ((LPSPI_Type *)LPSPI1_BASE)
/** Peripheral LPSPI2 base address */
#define LPSPI2_BASE                              (0x4003E000u)
/** Peripheral LPSPI2 base pointer */
#define LPSPI2                                   ((LPSPI_Type *)LPSPI2_BASE)
/** Array initializer of LPSPI peripheral base addresses */
#define LPSPI_BASE_ADDRS                         { LPSPI0_BASE, LPSPI1_BASE, LPSPI2_BASE }
/** Array initializer of LPSPI peripheral base pointers */
#define LPSPI_BASE_PTRS                          { LPSPI0, LPSPI1, LPSPI2 }
/** Interrupt vectors for the LPSPI peripheral type */
#define LPSPI_IRQS                               { LPSPI0_IRQn, LPSPI1_IRQn, LPSPI2_IRQn }

/*!
 * @}
 */ /* end of group LPSPI_Peripheral_Access_Layer */


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
#define LPTMR_CSR_TDRE_MASK                      (0x100U)
#define LPTMR_CSR_TDRE_SHIFT                     (8U)
#define LPTMR_CSR_TDRE(x)                        (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TDRE_SHIFT)) & LPTMR_CSR_TDRE_MASK)

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
#define LPTMR0_BASE                              (0x40034000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_Type *)LPTMR0_BASE)
/** Peripheral LPTMR1 base address */
#define LPTMR1_BASE                              (0x400B5000u)
/** Peripheral LPTMR1 base pointer */
#define LPTMR1                                   ((LPTMR_Type *)LPTMR1_BASE)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE, LPTMR1_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0, LPTMR1 }
/** Interrupt vectors for the LPTMR peripheral type */
#define LPTMR_IRQS                               { LPTMR0_IRQn, LPTMR1_IRQn }

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
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t GLOBAL;                            /**< LPUART Global Register, offset: 0x8 */
  __IO uint32_t PINCFG;                            /**< LPUART Pin Configuration Register, offset: 0xC */
  __IO uint32_t BAUD;                              /**< LPUART Baud Rate Register, offset: 0x10 */
  __IO uint32_t STAT;                              /**< LPUART Status Register, offset: 0x14 */
  __IO uint32_t CTRL;                              /**< LPUART Control Register, offset: 0x18 */
  __IO uint32_t DATA;                              /**< LPUART Data Register, offset: 0x1C */
  __IO uint32_t MATCH;                             /**< LPUART Match Address Register, offset: 0x20 */
  __IO uint32_t MODIR;                             /**< LPUART Modem IrDA Register, offset: 0x24 */
  __IO uint32_t FIFO;                              /**< LPUART FIFO Register, offset: 0x28 */
  __IO uint32_t WATER;                             /**< LPUART Watermark Register, offset: 0x2C */
} LPUART_Type;

/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define LPUART_VERID_FEATURE_MASK                (0xFFFFU)
#define LPUART_VERID_FEATURE_SHIFT               (0U)
#define LPUART_VERID_FEATURE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_FEATURE_SHIFT)) & LPUART_VERID_FEATURE_MASK)
#define LPUART_VERID_MINOR_MASK                  (0xFF0000U)
#define LPUART_VERID_MINOR_SHIFT                 (16U)
#define LPUART_VERID_MINOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MINOR_SHIFT)) & LPUART_VERID_MINOR_MASK)
#define LPUART_VERID_MAJOR_MASK                  (0xFF000000U)
#define LPUART_VERID_MAJOR_SHIFT                 (24U)
#define LPUART_VERID_MAJOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MAJOR_SHIFT)) & LPUART_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define LPUART_PARAM_TXFIFO_MASK                 (0xFFU)
#define LPUART_PARAM_TXFIFO_SHIFT                (0U)
#define LPUART_PARAM_TXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_TXFIFO_SHIFT)) & LPUART_PARAM_TXFIFO_MASK)
#define LPUART_PARAM_RXFIFO_MASK                 (0xFF00U)
#define LPUART_PARAM_RXFIFO_SHIFT                (8U)
#define LPUART_PARAM_RXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_RXFIFO_SHIFT)) & LPUART_PARAM_RXFIFO_MASK)

/*! @name GLOBAL - LPUART Global Register */
#define LPUART_GLOBAL_RST_MASK                   (0x2U)
#define LPUART_GLOBAL_RST_SHIFT                  (1U)
#define LPUART_GLOBAL_RST(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_GLOBAL_RST_SHIFT)) & LPUART_GLOBAL_RST_MASK)

/*! @name PINCFG - LPUART Pin Configuration Register */
#define LPUART_PINCFG_TRGSEL_MASK                (0x3U)
#define LPUART_PINCFG_TRGSEL_SHIFT               (0U)
#define LPUART_PINCFG_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_PINCFG_TRGSEL_SHIFT)) & LPUART_PINCFG_TRGSEL_MASK)

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
#define LPUART2_BASE                             (0x40046000u)
/** Peripheral LPUART2 base pointer */
#define LPUART2                                  ((LPUART_Type *)LPUART2_BASE)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                        { LPUART0_BASE, LPUART1_BASE, LPUART2_BASE }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                         { LPUART0, LPUART1, LPUART2 }
/** Interrupt vectors for the LPUART peripheral type */
#define LPUART_RX_TX_IRQS                        { LPUART0_IRQn, LPUART1_IRQn, LPUART2_IRQn }
#define LPUART_ERR_IRQS                          { LPUART0_IRQn, LPUART1_IRQn, LPUART2_IRQn }

/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


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
  __IO uint32_t PLACR;                             /**< Platform Control Register, offset: 0xC */
       uint8_t RESERVED_1[48];
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

/*! @name PLACR - Platform Control Register */
#define MCM_PLACR_MMCAU_MASK                     (0x100U)
#define MCM_PLACR_MMCAU_SHIFT                    (8U)
#define MCM_PLACR_MMCAU(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_MMCAU_SHIFT)) & MCM_PLACR_MMCAU_MASK)
#define MCM_PLACR_ARB_MASK                       (0x200U)
#define MCM_PLACR_ARB_SHIFT                      (9U)
#define MCM_PLACR_ARB(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_ARB_SHIFT)) & MCM_PLACR_ARB_MASK)
#define MCM_PLACR_CFCC_MASK                      (0x400U)
#define MCM_PLACR_CFCC_SHIFT                     (10U)
#define MCM_PLACR_CFCC(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_CFCC_SHIFT)) & MCM_PLACR_CFCC_MASK)
#define MCM_PLACR_DFCDA_MASK                     (0x800U)
#define MCM_PLACR_DFCDA_SHIFT                    (11U)
#define MCM_PLACR_DFCDA(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_DFCDA_SHIFT)) & MCM_PLACR_DFCDA_MASK)
#define MCM_PLACR_DFCIC_MASK                     (0x1000U)
#define MCM_PLACR_DFCIC_SHIFT                    (12U)
#define MCM_PLACR_DFCIC(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_DFCIC_SHIFT)) & MCM_PLACR_DFCIC_MASK)
#define MCM_PLACR_DFCC_MASK                      (0x2000U)
#define MCM_PLACR_DFCC_SHIFT                     (13U)
#define MCM_PLACR_DFCC(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_DFCC_SHIFT)) & MCM_PLACR_DFCC_MASK)
#define MCM_PLACR_EFDS_MASK                      (0x4000U)
#define MCM_PLACR_EFDS_SHIFT                     (14U)
#define MCM_PLACR_EFDS(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_EFDS_SHIFT)) & MCM_PLACR_EFDS_MASK)
#define MCM_PLACR_DFCS_MASK                      (0x8000U)
#define MCM_PLACR_DFCS_SHIFT                     (15U)
#define MCM_PLACR_DFCS(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_DFCS_SHIFT)) & MCM_PLACR_DFCS_MASK)
#define MCM_PLACR_ESFC_MASK                      (0x10000U)
#define MCM_PLACR_ESFC_SHIFT                     (16U)
#define MCM_PLACR_ESFC(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_PLACR_ESFC_SHIFT)) & MCM_PLACR_ESFC_MASK)

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
/** Peripheral MCM0 base address */
#define MCM0_BASE                                (0xF0003000u)
/** Peripheral MCM0 base pointer */
#define MCM0                                     ((MCM_Type *)MCM0_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM0_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM0 }

/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MMDVSQ Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MMDVSQ_Peripheral_Access_Layer MMDVSQ Peripheral Access Layer
 * @{
 */

/** MMDVSQ - Register Layout Typedef */
typedef struct {
  __IO uint32_t DEND;                              /**< Dividend Register, offset: 0x0 */
  __IO uint32_t DSOR;                              /**< Divisor Register, offset: 0x4 */
  __IO uint32_t CSR;                               /**< Control/Status Register, offset: 0x8 */
  __IO uint32_t RES;                               /**< Result Register, offset: 0xC */
  __IO uint32_t RCND;                              /**< Radicand Register, offset: 0x10 */
} MMDVSQ_Type;

/* ----------------------------------------------------------------------------
   -- MMDVSQ Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MMDVSQ_Register_Masks MMDVSQ Register Masks
 * @{
 */

/*! @name DEND - Dividend Register */
#define MMDVSQ_DEND_DIVIDEND_MASK                (0xFFFFFFFFU)
#define MMDVSQ_DEND_DIVIDEND_SHIFT               (0U)
#define MMDVSQ_DEND_DIVIDEND(x)                  (((uint32_t)(((uint32_t)(x)) << MMDVSQ_DEND_DIVIDEND_SHIFT)) & MMDVSQ_DEND_DIVIDEND_MASK)

/*! @name DSOR - Divisor Register */
#define MMDVSQ_DSOR_DIVISOR_MASK                 (0xFFFFFFFFU)
#define MMDVSQ_DSOR_DIVISOR_SHIFT                (0U)
#define MMDVSQ_DSOR_DIVISOR(x)                   (((uint32_t)(((uint32_t)(x)) << MMDVSQ_DSOR_DIVISOR_SHIFT)) & MMDVSQ_DSOR_DIVISOR_MASK)

/*! @name CSR - Control/Status Register */
#define MMDVSQ_CSR_SRT_MASK                      (0x1U)
#define MMDVSQ_CSR_SRT_SHIFT                     (0U)
#define MMDVSQ_CSR_SRT(x)                        (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_SRT_SHIFT)) & MMDVSQ_CSR_SRT_MASK)
#define MMDVSQ_CSR_USGN_MASK                     (0x2U)
#define MMDVSQ_CSR_USGN_SHIFT                    (1U)
#define MMDVSQ_CSR_USGN(x)                       (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_USGN_SHIFT)) & MMDVSQ_CSR_USGN_MASK)
#define MMDVSQ_CSR_REM_MASK                      (0x4U)
#define MMDVSQ_CSR_REM_SHIFT                     (2U)
#define MMDVSQ_CSR_REM(x)                        (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_REM_SHIFT)) & MMDVSQ_CSR_REM_MASK)
#define MMDVSQ_CSR_DZE_MASK                      (0x8U)
#define MMDVSQ_CSR_DZE_SHIFT                     (3U)
#define MMDVSQ_CSR_DZE(x)                        (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_DZE_SHIFT)) & MMDVSQ_CSR_DZE_MASK)
#define MMDVSQ_CSR_DZ_MASK                       (0x10U)
#define MMDVSQ_CSR_DZ_SHIFT                      (4U)
#define MMDVSQ_CSR_DZ(x)                         (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_DZ_SHIFT)) & MMDVSQ_CSR_DZ_MASK)
#define MMDVSQ_CSR_DFS_MASK                      (0x20U)
#define MMDVSQ_CSR_DFS_SHIFT                     (5U)
#define MMDVSQ_CSR_DFS(x)                        (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_DFS_SHIFT)) & MMDVSQ_CSR_DFS_MASK)
#define MMDVSQ_CSR_SQRT_MASK                     (0x20000000U)
#define MMDVSQ_CSR_SQRT_SHIFT                    (29U)
#define MMDVSQ_CSR_SQRT(x)                       (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_SQRT_SHIFT)) & MMDVSQ_CSR_SQRT_MASK)
#define MMDVSQ_CSR_DIV_MASK                      (0x40000000U)
#define MMDVSQ_CSR_DIV_SHIFT                     (30U)
#define MMDVSQ_CSR_DIV(x)                        (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_DIV_SHIFT)) & MMDVSQ_CSR_DIV_MASK)
#define MMDVSQ_CSR_BUSY_MASK                     (0x80000000U)
#define MMDVSQ_CSR_BUSY_SHIFT                    (31U)
#define MMDVSQ_CSR_BUSY(x)                       (((uint32_t)(((uint32_t)(x)) << MMDVSQ_CSR_BUSY_SHIFT)) & MMDVSQ_CSR_BUSY_MASK)

/*! @name RES - Result Register */
#define MMDVSQ_RES_RESULT_MASK                   (0xFFFFFFFFU)
#define MMDVSQ_RES_RESULT_SHIFT                  (0U)
#define MMDVSQ_RES_RESULT(x)                     (((uint32_t)(((uint32_t)(x)) << MMDVSQ_RES_RESULT_SHIFT)) & MMDVSQ_RES_RESULT_MASK)

/*! @name RCND - Radicand Register */
#define MMDVSQ_RCND_RADICAND_MASK                (0xFFFFFFFFU)
#define MMDVSQ_RCND_RADICAND_SHIFT               (0U)
#define MMDVSQ_RCND_RADICAND(x)                  (((uint32_t)(((uint32_t)(x)) << MMDVSQ_RCND_RADICAND_SHIFT)) & MMDVSQ_RCND_RADICAND_MASK)


/*!
 * @}
 */ /* end of group MMDVSQ_Register_Masks */


/* MMDVSQ - Peripheral instance base addresses */
/** Peripheral MMDVSQ0 base address */
#define MMDVSQ0_BASE                             (0xF0004000u)
/** Peripheral MMDVSQ0 base pointer */
#define MMDVSQ0                                  ((MMDVSQ_Type *)MMDVSQ0_BASE)
/** Array initializer of MMDVSQ peripheral base addresses */
#define MMDVSQ_BASE_ADDRS                        { MMDVSQ0_BASE }
/** Array initializer of MMDVSQ peripheral base pointers */
#define MMDVSQ_BASE_PTRS                         { MMDVSQ0 }

/*!
 * @}
 */ /* end of group MMDVSQ_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MSCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Peripheral_Access_Layer MSCM Peripheral Access Layer
 * @{
 */

/** MSCM - Register Layout Typedef */
typedef struct {
  __I  uint32_t CPxTYPE;                           /**< Processor X Type Register, offset: 0x0 */
  __I  uint32_t CPxNUM;                            /**< Processor X Number Register, offset: 0x4 */
  __I  uint32_t CPxMASTER;                         /**< Processor X Master Register, offset: 0x8 */
  __I  uint32_t CPxCOUNT;                          /**< Processor X Count Register, offset: 0xC */
  __I  uint32_t CPxCFG0;                           /**< Processor X Configuration Register, offset: 0x10 */
  __I  uint32_t CPxCFG1;                           /**< Processor X Configuration Register, offset: 0x14 */
  __I  uint32_t CPxCFG2;                           /**< Processor X Configuration Register, offset: 0x18 */
  __I  uint32_t CPxCFG3;                           /**< Processor X Configuration Register, offset: 0x1C */
  struct {                                         /* offset: 0x20, array step: 0x20 */
    __I  uint32_t TYPE;                              /**< Processor 0 Type Register, array offset: 0x20, array step: 0x20 */
    __I  uint32_t NUM;                               /**< Processor 0 Number Register, array offset: 0x24, array step: 0x20 */
    __I  uint32_t MASTER;                            /**< Processor 0 Master Register, array offset: 0x28, array step: 0x20 */
    __I  uint32_t COUNT;                             /**< Processor 0 Count Register, array offset: 0x2C, array step: 0x20 */
    __I  uint32_t CFG0;                              /**< Processor 0 Configuration Register, array offset: 0x30, array step: 0x20 */
    __I  uint32_t CFG1;                              /**< Processor 0 Configuration Register, array offset: 0x34, array step: 0x20 */
    __I  uint32_t CFG2;                              /**< Processor 0 Configuration Register, array offset: 0x38, array step: 0x20 */
    __I  uint32_t CFG3;                              /**< Processor 0 Configuration Register, array offset: 0x3C, array step: 0x20 */
  } CP[1];
       uint8_t RESERVED_0[960];
  __I  uint32_t OCMDR[3];                          /**< On-Chip Memory Descriptor Register, array offset: 0x400, array step: 0x4 */
} MSCM_Type;

/* ----------------------------------------------------------------------------
   -- MSCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Register_Masks MSCM Register Masks
 * @{
 */

/*! @name CPxTYPE - Processor X Type Register */
#define MSCM_CPxTYPE_RYPZ_MASK                   (0xFFU)
#define MSCM_CPxTYPE_RYPZ_SHIFT                  (0U)
#define MSCM_CPxTYPE_RYPZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxTYPE_RYPZ_SHIFT)) & MSCM_CPxTYPE_RYPZ_MASK)
#define MSCM_CPxTYPE_PERSONALITY_MASK            (0xFFFFFF00U)
#define MSCM_CPxTYPE_PERSONALITY_SHIFT           (8U)
#define MSCM_CPxTYPE_PERSONALITY(x)              (((uint32_t)(((uint32_t)(x)) << MSCM_CPxTYPE_PERSONALITY_SHIFT)) & MSCM_CPxTYPE_PERSONALITY_MASK)

/*! @name CPxNUM - Processor X Number Register */
#define MSCM_CPxNUM_CPN_MASK                     (0x1U)
#define MSCM_CPxNUM_CPN_SHIFT                    (0U)
#define MSCM_CPxNUM_CPN(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CPxNUM_CPN_SHIFT)) & MSCM_CPxNUM_CPN_MASK)

/*! @name CPxMASTER - Processor X Master Register */
#define MSCM_CPxMASTER_PPN_MASK                  (0x3FU)
#define MSCM_CPxMASTER_PPN_SHIFT                 (0U)
#define MSCM_CPxMASTER_PPN(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CPxMASTER_PPN_SHIFT)) & MSCM_CPxMASTER_PPN_MASK)

/*! @name CPxCOUNT - Processor X Count Register */
#define MSCM_CPxCOUNT_PCNT_MASK                  (0x3U)
#define MSCM_CPxCOUNT_PCNT_SHIFT                 (0U)
#define MSCM_CPxCOUNT_PCNT(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCOUNT_PCNT_SHIFT)) & MSCM_CPxCOUNT_PCNT_MASK)

/*! @name CPxCFG0 - Processor X Configuration Register */
#define MSCM_CPxCFG0_DCWY_MASK                   (0xFFU)
#define MSCM_CPxCFG0_DCWY_SHIFT                  (0U)
#define MSCM_CPxCFG0_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_DCWY_SHIFT)) & MSCM_CPxCFG0_DCWY_MASK)
#define MSCM_CPxCFG0_DCSZ_MASK                   (0xFF00U)
#define MSCM_CPxCFG0_DCSZ_SHIFT                  (8U)
#define MSCM_CPxCFG0_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_DCSZ_SHIFT)) & MSCM_CPxCFG0_DCSZ_MASK)
#define MSCM_CPxCFG0_ICWY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG0_ICWY_SHIFT                  (16U)
#define MSCM_CPxCFG0_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_ICWY_SHIFT)) & MSCM_CPxCFG0_ICWY_MASK)
#define MSCM_CPxCFG0_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG0_ICSZ_SHIFT                  (24U)
#define MSCM_CPxCFG0_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_ICSZ_SHIFT)) & MSCM_CPxCFG0_ICSZ_MASK)

/*! @name CPxCFG1 - Processor X Configuration Register */
#define MSCM_CPxCFG1_DCWY_MASK                   (0xFFU)
#define MSCM_CPxCFG1_DCWY_SHIFT                  (0U)
#define MSCM_CPxCFG1_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_DCWY_SHIFT)) & MSCM_CPxCFG1_DCWY_MASK)
#define MSCM_CPxCFG1_DCSZ_MASK                   (0xFF00U)
#define MSCM_CPxCFG1_DCSZ_SHIFT                  (8U)
#define MSCM_CPxCFG1_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_DCSZ_SHIFT)) & MSCM_CPxCFG1_DCSZ_MASK)
#define MSCM_CPxCFG1_ICWY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG1_ICWY_SHIFT                  (16U)
#define MSCM_CPxCFG1_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_ICWY_SHIFT)) & MSCM_CPxCFG1_ICWY_MASK)
#define MSCM_CPxCFG1_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG1_ICSZ_SHIFT                  (24U)
#define MSCM_CPxCFG1_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_ICSZ_SHIFT)) & MSCM_CPxCFG1_ICSZ_MASK)

/*! @name CPxCFG2 - Processor X Configuration Register */
#define MSCM_CPxCFG2_DCWY_MASK                   (0xFFU)
#define MSCM_CPxCFG2_DCWY_SHIFT                  (0U)
#define MSCM_CPxCFG2_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_DCWY_SHIFT)) & MSCM_CPxCFG2_DCWY_MASK)
#define MSCM_CPxCFG2_DCSZ_MASK                   (0xFF00U)
#define MSCM_CPxCFG2_DCSZ_SHIFT                  (8U)
#define MSCM_CPxCFG2_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_DCSZ_SHIFT)) & MSCM_CPxCFG2_DCSZ_MASK)
#define MSCM_CPxCFG2_ICWY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG2_ICWY_SHIFT                  (16U)
#define MSCM_CPxCFG2_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_ICWY_SHIFT)) & MSCM_CPxCFG2_ICWY_MASK)
#define MSCM_CPxCFG2_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG2_ICSZ_SHIFT                  (24U)
#define MSCM_CPxCFG2_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_ICSZ_SHIFT)) & MSCM_CPxCFG2_ICSZ_MASK)

/*! @name CPxCFG3 - Processor X Configuration Register */
#define MSCM_CPxCFG3_DCWY_MASK                   (0xFFU)
#define MSCM_CPxCFG3_DCWY_SHIFT                  (0U)
#define MSCM_CPxCFG3_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_DCWY_SHIFT)) & MSCM_CPxCFG3_DCWY_MASK)
#define MSCM_CPxCFG3_DCSZ_MASK                   (0xFF00U)
#define MSCM_CPxCFG3_DCSZ_SHIFT                  (8U)
#define MSCM_CPxCFG3_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_DCSZ_SHIFT)) & MSCM_CPxCFG3_DCSZ_MASK)
#define MSCM_CPxCFG3_ICWY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG3_ICWY_SHIFT                  (16U)
#define MSCM_CPxCFG3_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_ICWY_SHIFT)) & MSCM_CPxCFG3_ICWY_MASK)
#define MSCM_CPxCFG3_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG3_ICSZ_SHIFT                  (24U)
#define MSCM_CPxCFG3_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_ICSZ_SHIFT)) & MSCM_CPxCFG3_ICSZ_MASK)

/*! @name TYPE - Processor 0 Type Register */
#define MSCM_TYPE_RYPZ_MASK                      (0xFFU)
#define MSCM_TYPE_RYPZ_SHIFT                     (0U)
#define MSCM_TYPE_RYPZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_TYPE_RYPZ_SHIFT)) & MSCM_TYPE_RYPZ_MASK)
#define MSCM_TYPE_PERSONALITY_MASK               (0xFFFFFF00U)
#define MSCM_TYPE_PERSONALITY_SHIFT              (8U)
#define MSCM_TYPE_PERSONALITY(x)                 (((uint32_t)(((uint32_t)(x)) << MSCM_TYPE_PERSONALITY_SHIFT)) & MSCM_TYPE_PERSONALITY_MASK)

/* The count of MSCM_TYPE */
#define MSCM_TYPE_COUNT                          (1U)

/*! @name NUM - Processor 0 Number Register */
#define MSCM_NUM_CPN_MASK                        (0x1U)
#define MSCM_NUM_CPN_SHIFT                       (0U)
#define MSCM_NUM_CPN(x)                          (((uint32_t)(((uint32_t)(x)) << MSCM_NUM_CPN_SHIFT)) & MSCM_NUM_CPN_MASK)

/* The count of MSCM_NUM */
#define MSCM_NUM_COUNT                           (1U)

/*! @name MASTER - Processor 0 Master Register */
#define MSCM_MASTER_PPN_MASK                     (0x3FU)
#define MSCM_MASTER_PPN_SHIFT                    (0U)
#define MSCM_MASTER_PPN(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_MASTER_PPN_SHIFT)) & MSCM_MASTER_PPN_MASK)

/* The count of MSCM_MASTER */
#define MSCM_MASTER_COUNT                        (1U)

/*! @name COUNT - Processor 0 Count Register */
#define MSCM_COUNT_PCNT_MASK                     (0x3U)
#define MSCM_COUNT_PCNT_SHIFT                    (0U)
#define MSCM_COUNT_PCNT(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_COUNT_PCNT_SHIFT)) & MSCM_COUNT_PCNT_MASK)

/* The count of MSCM_COUNT */
#define MSCM_COUNT_COUNT                         (1U)

/*! @name CFG0 - Processor 0 Configuration Register */
#define MSCM_CFG0_DCWY_MASK                      (0xFFU)
#define MSCM_CFG0_DCWY_SHIFT                     (0U)
#define MSCM_CFG0_DCWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG0_DCWY_SHIFT)) & MSCM_CFG0_DCWY_MASK)
#define MSCM_CFG0_DCSZ_MASK                      (0xFF00U)
#define MSCM_CFG0_DCSZ_SHIFT                     (8U)
#define MSCM_CFG0_DCSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG0_DCSZ_SHIFT)) & MSCM_CFG0_DCSZ_MASK)
#define MSCM_CFG0_ICWY_MASK                      (0xFF0000U)
#define MSCM_CFG0_ICWY_SHIFT                     (16U)
#define MSCM_CFG0_ICWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG0_ICWY_SHIFT)) & MSCM_CFG0_ICWY_MASK)
#define MSCM_CFG0_ICSZ_MASK                      (0xFF000000U)
#define MSCM_CFG0_ICSZ_SHIFT                     (24U)
#define MSCM_CFG0_ICSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG0_ICSZ_SHIFT)) & MSCM_CFG0_ICSZ_MASK)

/* The count of MSCM_CFG0 */
#define MSCM_CFG0_COUNT                          (1U)

/*! @name CFG1 - Processor 0 Configuration Register */
#define MSCM_CFG1_DCWY_MASK                      (0xFFU)
#define MSCM_CFG1_DCWY_SHIFT                     (0U)
#define MSCM_CFG1_DCWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG1_DCWY_SHIFT)) & MSCM_CFG1_DCWY_MASK)
#define MSCM_CFG1_DCSZ_MASK                      (0xFF00U)
#define MSCM_CFG1_DCSZ_SHIFT                     (8U)
#define MSCM_CFG1_DCSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG1_DCSZ_SHIFT)) & MSCM_CFG1_DCSZ_MASK)
#define MSCM_CFG1_ICWY_MASK                      (0xFF0000U)
#define MSCM_CFG1_ICWY_SHIFT                     (16U)
#define MSCM_CFG1_ICWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG1_ICWY_SHIFT)) & MSCM_CFG1_ICWY_MASK)
#define MSCM_CFG1_ICSZ_MASK                      (0xFF000000U)
#define MSCM_CFG1_ICSZ_SHIFT                     (24U)
#define MSCM_CFG1_ICSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG1_ICSZ_SHIFT)) & MSCM_CFG1_ICSZ_MASK)

/* The count of MSCM_CFG1 */
#define MSCM_CFG1_COUNT                          (1U)

/*! @name CFG2 - Processor 0 Configuration Register */
#define MSCM_CFG2_DCWY_MASK                      (0xFFU)
#define MSCM_CFG2_DCWY_SHIFT                     (0U)
#define MSCM_CFG2_DCWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG2_DCWY_SHIFT)) & MSCM_CFG2_DCWY_MASK)
#define MSCM_CFG2_DCSZ_MASK                      (0xFF00U)
#define MSCM_CFG2_DCSZ_SHIFT                     (8U)
#define MSCM_CFG2_DCSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG2_DCSZ_SHIFT)) & MSCM_CFG2_DCSZ_MASK)
#define MSCM_CFG2_ICWY_MASK                      (0xFF0000U)
#define MSCM_CFG2_ICWY_SHIFT                     (16U)
#define MSCM_CFG2_ICWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG2_ICWY_SHIFT)) & MSCM_CFG2_ICWY_MASK)
#define MSCM_CFG2_ICSZ_MASK                      (0xFF000000U)
#define MSCM_CFG2_ICSZ_SHIFT                     (24U)
#define MSCM_CFG2_ICSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG2_ICSZ_SHIFT)) & MSCM_CFG2_ICSZ_MASK)

/* The count of MSCM_CFG2 */
#define MSCM_CFG2_COUNT                          (1U)

/*! @name CFG3 - Processor 0 Configuration Register */
#define MSCM_CFG3_DCWY_MASK                      (0xFFU)
#define MSCM_CFG3_DCWY_SHIFT                     (0U)
#define MSCM_CFG3_DCWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG3_DCWY_SHIFT)) & MSCM_CFG3_DCWY_MASK)
#define MSCM_CFG3_DCSZ_MASK                      (0xFF00U)
#define MSCM_CFG3_DCSZ_SHIFT                     (8U)
#define MSCM_CFG3_DCSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG3_DCSZ_SHIFT)) & MSCM_CFG3_DCSZ_MASK)
#define MSCM_CFG3_ICWY_MASK                      (0xFF0000U)
#define MSCM_CFG3_ICWY_SHIFT                     (16U)
#define MSCM_CFG3_ICWY(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG3_ICWY_SHIFT)) & MSCM_CFG3_ICWY_MASK)
#define MSCM_CFG3_ICSZ_MASK                      (0xFF000000U)
#define MSCM_CFG3_ICSZ_SHIFT                     (24U)
#define MSCM_CFG3_ICSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MSCM_CFG3_ICSZ_SHIFT)) & MSCM_CFG3_ICSZ_MASK)

/* The count of MSCM_CFG3 */
#define MSCM_CFG3_COUNT                          (1U)

/*! @name OCMDR - On-Chip Memory Descriptor Register */
#define MSCM_OCMDR_OCMPU_MASK                    (0x1000U)
#define MSCM_OCMDR_OCMPU_SHIFT                   (12U)
#define MSCM_OCMDR_OCMPU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMPU_SHIFT)) & MSCM_OCMDR_OCMPU_MASK)
#define MSCM_OCMDR_OCMT_MASK                     (0xE000U)
#define MSCM_OCMDR_OCMT_SHIFT                    (13U)
#define MSCM_OCMDR_OCMT(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMT_SHIFT)) & MSCM_OCMDR_OCMT_MASK)
#define MSCM_OCMDR_RO_MASK                       (0x10000U)
#define MSCM_OCMDR_RO_SHIFT                      (16U)
#define MSCM_OCMDR_RO(x)                         (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_RO_SHIFT)) & MSCM_OCMDR_RO_MASK)
#define MSCM_OCMDR_OCMW_MASK                     (0xE0000U)
#define MSCM_OCMDR_OCMW_SHIFT                    (17U)
#define MSCM_OCMDR_OCMW(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMW_SHIFT)) & MSCM_OCMDR_OCMW_MASK)
#define MSCM_OCMDR_OCMSZ_MASK                    (0xF000000U)
#define MSCM_OCMDR_OCMSZ_SHIFT                   (24U)
#define MSCM_OCMDR_OCMSZ(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMSZ_SHIFT)) & MSCM_OCMDR_OCMSZ_MASK)
#define MSCM_OCMDR_OCMSZH_MASK                   (0x10000000U)
#define MSCM_OCMDR_OCMSZH_SHIFT                  (28U)
#define MSCM_OCMDR_OCMSZH(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMSZH_SHIFT)) & MSCM_OCMDR_OCMSZH_MASK)
#define MSCM_OCMDR_V_MASK                        (0x80000000U)
#define MSCM_OCMDR_V_SHIFT                       (31U)
#define MSCM_OCMDR_V(x)                          (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_V_SHIFT)) & MSCM_OCMDR_V_MASK)

/* The count of MSCM_OCMDR */
#define MSCM_OCMDR_COUNT                         (3U)


/*!
 * @}
 */ /* end of group MSCM_Register_Masks */


/* MSCM - Peripheral instance base addresses */
/** Peripheral MSCM base address */
#define MSCM_BASE                                (0x40001000u)
/** Peripheral MSCM base pointer */
#define MSCM                                     ((MSCM_Type *)MSCM_BASE)
/** Array initializer of MSCM peripheral base addresses */
#define MSCM_BASE_ADDRS                          { MSCM_BASE }
/** Array initializer of MSCM peripheral base pointers */
#define MSCM_BASE_PTRS                           { MSCM }

/*!
 * @}
 */ /* end of group MSCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MTB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_Peripheral_Access_Layer MTB Peripheral Access Layer
 * @{
 */

/** MTB - Register Layout Typedef */
typedef struct {
  __IO uint32_t POSITION;                          /**< MTB Position Register, offset: 0x0 */
  __IO uint32_t MASTER;                            /**< MTB Master Register, offset: 0x4 */
  __IO uint32_t FLOW;                              /**< MTB Flow Register, offset: 0x8 */
  __I  uint32_t BASE;                              /**< MTB Base Register, offset: 0xC */
       uint8_t RESERVED_0[3824];
  __I  uint32_t MODECTRL;                          /**< Integration Mode Control Register, offset: 0xF00 */
       uint8_t RESERVED_1[156];
  __I  uint32_t TAGSET;                            /**< Claim TAG Set Register, offset: 0xFA0 */
  __I  uint32_t TAGCLEAR;                          /**< Claim TAG Clear Register, offset: 0xFA4 */
       uint8_t RESERVED_2[8];
  __I  uint32_t LOCKACCESS;                        /**< Lock Access Register, offset: 0xFB0 */
  __I  uint32_t LOCKSTAT;                          /**< Lock Status Register, offset: 0xFB4 */
  __I  uint32_t AUTHSTAT;                          /**< Authentication Status Register, offset: 0xFB8 */
  __I  uint32_t DEVICEARCH;                        /**< Device Architecture Register, offset: 0xFBC */
       uint8_t RESERVED_3[8];
  __I  uint32_t DEVICECFG;                         /**< Device Configuration Register, offset: 0xFC8 */
  __I  uint32_t DEVICETYPID;                       /**< Device Type Identifier Register, offset: 0xFCC */
  __I  uint32_t PERIPHID4;                         /**< Peripheral ID Register, offset: 0xFD0 */
  __I  uint32_t PERIPHID5;                         /**< Peripheral ID Register, offset: 0xFD4 */
  __I  uint32_t PERIPHID6;                         /**< Peripheral ID Register, offset: 0xFD8 */
  __I  uint32_t PERIPHID7;                         /**< Peripheral ID Register, offset: 0xFDC */
  __I  uint32_t PERIPHID0;                         /**< Peripheral ID Register, offset: 0xFE0 */
  __I  uint32_t PERIPHID1;                         /**< Peripheral ID Register, offset: 0xFE4 */
  __I  uint32_t PERIPHID2;                         /**< Peripheral ID Register, offset: 0xFE8 */
  __I  uint32_t PERIPHID3;                         /**< Peripheral ID Register, offset: 0xFEC */
  __I  uint32_t COMPID[4];                         /**< Component ID Register, array offset: 0xFF0, array step: 0x4 */
} MTB_Type;

/* ----------------------------------------------------------------------------
   -- MTB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_Register_Masks MTB Register Masks
 * @{
 */

/*! @name POSITION - MTB Position Register */
#define MTB_POSITION_WRAP_MASK                   (0x4U)
#define MTB_POSITION_WRAP_SHIFT                  (2U)
#define MTB_POSITION_WRAP(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_POSITION_WRAP_SHIFT)) & MTB_POSITION_WRAP_MASK)
#define MTB_POSITION_POINTER_MASK                (0xFFFFFFF8U)
#define MTB_POSITION_POINTER_SHIFT               (3U)
#define MTB_POSITION_POINTER(x)                  (((uint32_t)(((uint32_t)(x)) << MTB_POSITION_POINTER_SHIFT)) & MTB_POSITION_POINTER_MASK)

/*! @name MASTER - MTB Master Register */
#define MTB_MASTER_MASK_MASK                     (0x1FU)
#define MTB_MASTER_MASK_SHIFT                    (0U)
#define MTB_MASTER_MASK(x)                       (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_MASK_SHIFT)) & MTB_MASTER_MASK_MASK)
#define MTB_MASTER_TSTARTEN_MASK                 (0x20U)
#define MTB_MASTER_TSTARTEN_SHIFT                (5U)
#define MTB_MASTER_TSTARTEN(x)                   (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_TSTARTEN_SHIFT)) & MTB_MASTER_TSTARTEN_MASK)
#define MTB_MASTER_TSTOPEN_MASK                  (0x40U)
#define MTB_MASTER_TSTOPEN_SHIFT                 (6U)
#define MTB_MASTER_TSTOPEN(x)                    (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_TSTOPEN_SHIFT)) & MTB_MASTER_TSTOPEN_MASK)
#define MTB_MASTER_SFRWPRIV_MASK                 (0x80U)
#define MTB_MASTER_SFRWPRIV_SHIFT                (7U)
#define MTB_MASTER_SFRWPRIV(x)                   (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_SFRWPRIV_SHIFT)) & MTB_MASTER_SFRWPRIV_MASK)
#define MTB_MASTER_RAMPRIV_MASK                  (0x100U)
#define MTB_MASTER_RAMPRIV_SHIFT                 (8U)
#define MTB_MASTER_RAMPRIV(x)                    (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_RAMPRIV_SHIFT)) & MTB_MASTER_RAMPRIV_MASK)
#define MTB_MASTER_HALTREQ_MASK                  (0x200U)
#define MTB_MASTER_HALTREQ_SHIFT                 (9U)
#define MTB_MASTER_HALTREQ(x)                    (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_HALTREQ_SHIFT)) & MTB_MASTER_HALTREQ_MASK)
#define MTB_MASTER_EN_MASK                       (0x80000000U)
#define MTB_MASTER_EN_SHIFT                      (31U)
#define MTB_MASTER_EN(x)                         (((uint32_t)(((uint32_t)(x)) << MTB_MASTER_EN_SHIFT)) & MTB_MASTER_EN_MASK)

/*! @name FLOW - MTB Flow Register */
#define MTB_FLOW_AUTOSTOP_MASK                   (0x1U)
#define MTB_FLOW_AUTOSTOP_SHIFT                  (0U)
#define MTB_FLOW_AUTOSTOP(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_FLOW_AUTOSTOP_SHIFT)) & MTB_FLOW_AUTOSTOP_MASK)
#define MTB_FLOW_AUTOHALT_MASK                   (0x2U)
#define MTB_FLOW_AUTOHALT_SHIFT                  (1U)
#define MTB_FLOW_AUTOHALT(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_FLOW_AUTOHALT_SHIFT)) & MTB_FLOW_AUTOHALT_MASK)
#define MTB_FLOW_WATERMARK_MASK                  (0xFFFFFFF8U)
#define MTB_FLOW_WATERMARK_SHIFT                 (3U)
#define MTB_FLOW_WATERMARK(x)                    (((uint32_t)(((uint32_t)(x)) << MTB_FLOW_WATERMARK_SHIFT)) & MTB_FLOW_WATERMARK_MASK)

/*! @name BASE - MTB Base Register */
#define MTB_BASE_BASEADDR_MASK                   (0xFFFFFFFFU)
#define MTB_BASE_BASEADDR_SHIFT                  (0U)
#define MTB_BASE_BASEADDR(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_BASE_BASEADDR_SHIFT)) & MTB_BASE_BASEADDR_MASK)

/*! @name MODECTRL - Integration Mode Control Register */
#define MTB_MODECTRL_MODECTRL_MASK               (0xFFFFFFFFU)
#define MTB_MODECTRL_MODECTRL_SHIFT              (0U)
#define MTB_MODECTRL_MODECTRL(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_MODECTRL_MODECTRL_SHIFT)) & MTB_MODECTRL_MODECTRL_MASK)

/*! @name TAGSET - Claim TAG Set Register */
#define MTB_TAGSET_TAGSET_MASK                   (0xFFFFFFFFU)
#define MTB_TAGSET_TAGSET_SHIFT                  (0U)
#define MTB_TAGSET_TAGSET(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_TAGSET_TAGSET_SHIFT)) & MTB_TAGSET_TAGSET_MASK)

/*! @name TAGCLEAR - Claim TAG Clear Register */
#define MTB_TAGCLEAR_TAGCLEAR_MASK               (0xFFFFFFFFU)
#define MTB_TAGCLEAR_TAGCLEAR_SHIFT              (0U)
#define MTB_TAGCLEAR_TAGCLEAR(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_TAGCLEAR_TAGCLEAR_SHIFT)) & MTB_TAGCLEAR_TAGCLEAR_MASK)

/*! @name LOCKACCESS - Lock Access Register */
#define MTB_LOCKACCESS_LOCKACCESS_MASK           (0xFFFFFFFFU)
#define MTB_LOCKACCESS_LOCKACCESS_SHIFT          (0U)
#define MTB_LOCKACCESS_LOCKACCESS(x)             (((uint32_t)(((uint32_t)(x)) << MTB_LOCKACCESS_LOCKACCESS_SHIFT)) & MTB_LOCKACCESS_LOCKACCESS_MASK)

/*! @name LOCKSTAT - Lock Status Register */
#define MTB_LOCKSTAT_LOCKSTAT_MASK               (0xFFFFFFFFU)
#define MTB_LOCKSTAT_LOCKSTAT_SHIFT              (0U)
#define MTB_LOCKSTAT_LOCKSTAT(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_LOCKSTAT_LOCKSTAT_SHIFT)) & MTB_LOCKSTAT_LOCKSTAT_MASK)

/*! @name AUTHSTAT - Authentication Status Register */
#define MTB_AUTHSTAT_BIT0_MASK                   (0x1U)
#define MTB_AUTHSTAT_BIT0_SHIFT                  (0U)
#define MTB_AUTHSTAT_BIT0(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_AUTHSTAT_BIT0_SHIFT)) & MTB_AUTHSTAT_BIT0_MASK)
#define MTB_AUTHSTAT_BIT1_MASK                   (0x2U)
#define MTB_AUTHSTAT_BIT1_SHIFT                  (1U)
#define MTB_AUTHSTAT_BIT1(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_AUTHSTAT_BIT1_SHIFT)) & MTB_AUTHSTAT_BIT1_MASK)
#define MTB_AUTHSTAT_BIT2_MASK                   (0x4U)
#define MTB_AUTHSTAT_BIT2_SHIFT                  (2U)
#define MTB_AUTHSTAT_BIT2(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_AUTHSTAT_BIT2_SHIFT)) & MTB_AUTHSTAT_BIT2_MASK)
#define MTB_AUTHSTAT_BIT3_MASK                   (0x8U)
#define MTB_AUTHSTAT_BIT3_SHIFT                  (3U)
#define MTB_AUTHSTAT_BIT3(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_AUTHSTAT_BIT3_SHIFT)) & MTB_AUTHSTAT_BIT3_MASK)

/*! @name DEVICEARCH - Device Architecture Register */
#define MTB_DEVICEARCH_DEVICEARCH_MASK           (0xFFFFFFFFU)
#define MTB_DEVICEARCH_DEVICEARCH_SHIFT          (0U)
#define MTB_DEVICEARCH_DEVICEARCH(x)             (((uint32_t)(((uint32_t)(x)) << MTB_DEVICEARCH_DEVICEARCH_SHIFT)) & MTB_DEVICEARCH_DEVICEARCH_MASK)

/*! @name DEVICECFG - Device Configuration Register */
#define MTB_DEVICECFG_DEVICECFG_MASK             (0xFFFFFFFFU)
#define MTB_DEVICECFG_DEVICECFG_SHIFT            (0U)
#define MTB_DEVICECFG_DEVICECFG(x)               (((uint32_t)(((uint32_t)(x)) << MTB_DEVICECFG_DEVICECFG_SHIFT)) & MTB_DEVICECFG_DEVICECFG_MASK)

/*! @name DEVICETYPID - Device Type Identifier Register */
#define MTB_DEVICETYPID_DEVICETYPID_MASK         (0xFFFFFFFFU)
#define MTB_DEVICETYPID_DEVICETYPID_SHIFT        (0U)
#define MTB_DEVICETYPID_DEVICETYPID(x)           (((uint32_t)(((uint32_t)(x)) << MTB_DEVICETYPID_DEVICETYPID_SHIFT)) & MTB_DEVICETYPID_DEVICETYPID_MASK)

/*! @name PERIPHID4 - Peripheral ID Register */
#define MTB_PERIPHID4_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID4_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID4_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID4_PERIPHID_SHIFT)) & MTB_PERIPHID4_PERIPHID_MASK)

/*! @name PERIPHID5 - Peripheral ID Register */
#define MTB_PERIPHID5_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID5_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID5_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID5_PERIPHID_SHIFT)) & MTB_PERIPHID5_PERIPHID_MASK)

/*! @name PERIPHID6 - Peripheral ID Register */
#define MTB_PERIPHID6_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID6_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID6_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID6_PERIPHID_SHIFT)) & MTB_PERIPHID6_PERIPHID_MASK)

/*! @name PERIPHID7 - Peripheral ID Register */
#define MTB_PERIPHID7_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID7_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID7_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID7_PERIPHID_SHIFT)) & MTB_PERIPHID7_PERIPHID_MASK)

/*! @name PERIPHID0 - Peripheral ID Register */
#define MTB_PERIPHID0_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID0_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID0_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID0_PERIPHID_SHIFT)) & MTB_PERIPHID0_PERIPHID_MASK)

/*! @name PERIPHID1 - Peripheral ID Register */
#define MTB_PERIPHID1_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID1_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID1_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID1_PERIPHID_SHIFT)) & MTB_PERIPHID1_PERIPHID_MASK)

/*! @name PERIPHID2 - Peripheral ID Register */
#define MTB_PERIPHID2_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID2_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID2_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID2_PERIPHID_SHIFT)) & MTB_PERIPHID2_PERIPHID_MASK)

/*! @name PERIPHID3 - Peripheral ID Register */
#define MTB_PERIPHID3_PERIPHID_MASK              (0xFFFFFFFFU)
#define MTB_PERIPHID3_PERIPHID_SHIFT             (0U)
#define MTB_PERIPHID3_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << MTB_PERIPHID3_PERIPHID_SHIFT)) & MTB_PERIPHID3_PERIPHID_MASK)

/*! @name COMPID - Component ID Register */
#define MTB_COMPID_COMPID_MASK                   (0xFFFFFFFFU)
#define MTB_COMPID_COMPID_SHIFT                  (0U)
#define MTB_COMPID_COMPID(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_COMPID_COMPID_SHIFT)) & MTB_COMPID_COMPID_MASK)

/* The count of MTB_COMPID */
#define MTB_COMPID_COUNT                         (4U)


/*!
 * @}
 */ /* end of group MTB_Register_Masks */


/* MTB - Peripheral instance base addresses */
/** Peripheral MTB0 base address */
#define MTB0_BASE                                (0xF0000000u)
/** Peripheral MTB0 base pointer */
#define MTB0                                     ((MTB_Type *)MTB0_BASE)
/** Array initializer of MTB peripheral base addresses */
#define MTB_BASE_ADDRS                           { MTB0_BASE }
/** Array initializer of MTB peripheral base pointers */
#define MTB_BASE_PTRS                            { MTB0 }

/*!
 * @}
 */ /* end of group MTB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MTB_DWT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_DWT_Peripheral_Access_Layer MTB_DWT Peripheral Access Layer
 * @{
 */

/** MTB_DWT - Register Layout Typedef */
typedef struct {
  __I  uint32_t CTRL;                              /**< MTB DWT Control Register, offset: 0x0 */
       uint8_t RESERVED_0[28];
  struct {                                         /* offset: 0x20, array step: 0x10 */
    __IO uint32_t COMP;                              /**< MTB_DWT Comparator Register, array offset: 0x20, array step: 0x10 */
    __IO uint32_t MASK;                              /**< MTB_DWT Comparator Mask Register, array offset: 0x24, array step: 0x10 */
    __IO uint32_t FCT;                               /**< MTB_DWT Comparator Function Register 0..MTB_DWT Comparator Function Register 1, array offset: 0x28, array step: 0x10 */
         uint8_t RESERVED_0[4];
  } COMPARATOR[2];
       uint8_t RESERVED_1[448];
  __IO uint32_t TBCTRL;                            /**< MTB_DWT Trace Buffer Control Register, offset: 0x200 */
       uint8_t RESERVED_2[3524];
  __I  uint32_t DEVICECFG;                         /**< Device Configuration Register, offset: 0xFC8 */
  __I  uint32_t DEVICETYPID;                       /**< Device Type Identifier Register, offset: 0xFCC */
  __I  uint32_t PERIPHID4;                         /**< Peripheral ID Register, offset: 0xFD0 */
  __I  uint32_t PERIPHID5;                         /**< Peripheral ID Register, offset: 0xFD4 */
  __I  uint32_t PERIPHID6;                         /**< Peripheral ID Register, offset: 0xFD8 */
  __I  uint32_t PERIPHID7;                         /**< Peripheral ID Register, offset: 0xFDC */
  __I  uint32_t PERIPHID0;                         /**< Peripheral ID Register, offset: 0xFE0 */
  __I  uint32_t PERIPHID1;                         /**< Peripheral ID Register, offset: 0xFE4 */
  __I  uint32_t PERIPHID2;                         /**< Peripheral ID Register, offset: 0xFE8 */
  __I  uint32_t PERIPHID3;                         /**< Peripheral ID Register, offset: 0xFEC */
  __I  uint32_t COMPID[4];                         /**< Component ID Register, array offset: 0xFF0, array step: 0x4 */
} MTB_DWT_Type;

/* ----------------------------------------------------------------------------
   -- MTB_DWT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_DWT_Register_Masks MTB_DWT Register Masks
 * @{
 */

/*! @name CTRL - MTB DWT Control Register */
#define MTB_DWT_CTRL_DWTCFGCTRL_MASK             (0xFFFFFFFU)
#define MTB_DWT_CTRL_DWTCFGCTRL_SHIFT            (0U)
#define MTB_DWT_CTRL_DWTCFGCTRL(x)               (((uint32_t)(((uint32_t)(x)) << MTB_DWT_CTRL_DWTCFGCTRL_SHIFT)) & MTB_DWT_CTRL_DWTCFGCTRL_MASK)
#define MTB_DWT_CTRL_NUMCMP_MASK                 (0xF0000000U)
#define MTB_DWT_CTRL_NUMCMP_SHIFT                (28U)
#define MTB_DWT_CTRL_NUMCMP(x)                   (((uint32_t)(((uint32_t)(x)) << MTB_DWT_CTRL_NUMCMP_SHIFT)) & MTB_DWT_CTRL_NUMCMP_MASK)

/*! @name COMP - MTB_DWT Comparator Register */
#define MTB_DWT_COMP_COMP_MASK                   (0xFFFFFFFFU)
#define MTB_DWT_COMP_COMP_SHIFT                  (0U)
#define MTB_DWT_COMP_COMP(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_DWT_COMP_COMP_SHIFT)) & MTB_DWT_COMP_COMP_MASK)

/* The count of MTB_DWT_COMP */
#define MTB_DWT_COMP_COUNT                       (2U)

/*! @name MASK - MTB_DWT Comparator Mask Register */
#define MTB_DWT_MASK_MASK_MASK                   (0x1FU)
#define MTB_DWT_MASK_MASK_SHIFT                  (0U)
#define MTB_DWT_MASK_MASK(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_DWT_MASK_MASK_SHIFT)) & MTB_DWT_MASK_MASK_MASK)

/* The count of MTB_DWT_MASK */
#define MTB_DWT_MASK_COUNT                       (2U)

/*! @name FCT - MTB_DWT Comparator Function Register 0..MTB_DWT Comparator Function Register 1 */
#define MTB_DWT_FCT_FUNCTION_MASK                (0xFU)
#define MTB_DWT_FCT_FUNCTION_SHIFT               (0U)
#define MTB_DWT_FCT_FUNCTION(x)                  (((uint32_t)(((uint32_t)(x)) << MTB_DWT_FCT_FUNCTION_SHIFT)) & MTB_DWT_FCT_FUNCTION_MASK)
#define MTB_DWT_FCT_DATAVMATCH_MASK              (0x100U)
#define MTB_DWT_FCT_DATAVMATCH_SHIFT             (8U)
#define MTB_DWT_FCT_DATAVMATCH(x)                (((uint32_t)(((uint32_t)(x)) << MTB_DWT_FCT_DATAVMATCH_SHIFT)) & MTB_DWT_FCT_DATAVMATCH_MASK)
#define MTB_DWT_FCT_DATAVSIZE_MASK               (0xC00U)
#define MTB_DWT_FCT_DATAVSIZE_SHIFT              (10U)
#define MTB_DWT_FCT_DATAVSIZE(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_DWT_FCT_DATAVSIZE_SHIFT)) & MTB_DWT_FCT_DATAVSIZE_MASK)
#define MTB_DWT_FCT_DATAVADDR0_MASK              (0xF000U)
#define MTB_DWT_FCT_DATAVADDR0_SHIFT             (12U)
#define MTB_DWT_FCT_DATAVADDR0(x)                (((uint32_t)(((uint32_t)(x)) << MTB_DWT_FCT_DATAVADDR0_SHIFT)) & MTB_DWT_FCT_DATAVADDR0_MASK)
#define MTB_DWT_FCT_MATCHED_MASK                 (0x1000000U)
#define MTB_DWT_FCT_MATCHED_SHIFT                (24U)
#define MTB_DWT_FCT_MATCHED(x)                   (((uint32_t)(((uint32_t)(x)) << MTB_DWT_FCT_MATCHED_SHIFT)) & MTB_DWT_FCT_MATCHED_MASK)

/* The count of MTB_DWT_FCT */
#define MTB_DWT_FCT_COUNT                        (2U)

/*! @name TBCTRL - MTB_DWT Trace Buffer Control Register */
#define MTB_DWT_TBCTRL_ACOMP0_MASK               (0x1U)
#define MTB_DWT_TBCTRL_ACOMP0_SHIFT              (0U)
#define MTB_DWT_TBCTRL_ACOMP0(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_DWT_TBCTRL_ACOMP0_SHIFT)) & MTB_DWT_TBCTRL_ACOMP0_MASK)
#define MTB_DWT_TBCTRL_ACOMP1_MASK               (0x2U)
#define MTB_DWT_TBCTRL_ACOMP1_SHIFT              (1U)
#define MTB_DWT_TBCTRL_ACOMP1(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_DWT_TBCTRL_ACOMP1_SHIFT)) & MTB_DWT_TBCTRL_ACOMP1_MASK)
#define MTB_DWT_TBCTRL_NUMCOMP_MASK              (0xF0000000U)
#define MTB_DWT_TBCTRL_NUMCOMP_SHIFT             (28U)
#define MTB_DWT_TBCTRL_NUMCOMP(x)                (((uint32_t)(((uint32_t)(x)) << MTB_DWT_TBCTRL_NUMCOMP_SHIFT)) & MTB_DWT_TBCTRL_NUMCOMP_MASK)

/*! @name DEVICECFG - Device Configuration Register */
#define MTB_DWT_DEVICECFG_DEVICECFG_MASK         (0xFFFFFFFFU)
#define MTB_DWT_DEVICECFG_DEVICECFG_SHIFT        (0U)
#define MTB_DWT_DEVICECFG_DEVICECFG(x)           (((uint32_t)(((uint32_t)(x)) << MTB_DWT_DEVICECFG_DEVICECFG_SHIFT)) & MTB_DWT_DEVICECFG_DEVICECFG_MASK)

/*! @name DEVICETYPID - Device Type Identifier Register */
#define MTB_DWT_DEVICETYPID_DEVICETYPID_MASK     (0xFFFFFFFFU)
#define MTB_DWT_DEVICETYPID_DEVICETYPID_SHIFT    (0U)
#define MTB_DWT_DEVICETYPID_DEVICETYPID(x)       (((uint32_t)(((uint32_t)(x)) << MTB_DWT_DEVICETYPID_DEVICETYPID_SHIFT)) & MTB_DWT_DEVICETYPID_DEVICETYPID_MASK)

/*! @name PERIPHID4 - Peripheral ID Register */
#define MTB_DWT_PERIPHID4_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID4_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID4_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID4_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID4_PERIPHID_MASK)

/*! @name PERIPHID5 - Peripheral ID Register */
#define MTB_DWT_PERIPHID5_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID5_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID5_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID5_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID5_PERIPHID_MASK)

/*! @name PERIPHID6 - Peripheral ID Register */
#define MTB_DWT_PERIPHID6_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID6_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID6_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID6_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID6_PERIPHID_MASK)

/*! @name PERIPHID7 - Peripheral ID Register */
#define MTB_DWT_PERIPHID7_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID7_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID7_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID7_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID7_PERIPHID_MASK)

/*! @name PERIPHID0 - Peripheral ID Register */
#define MTB_DWT_PERIPHID0_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID0_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID0_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID0_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID0_PERIPHID_MASK)

/*! @name PERIPHID1 - Peripheral ID Register */
#define MTB_DWT_PERIPHID1_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID1_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID1_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID1_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID1_PERIPHID_MASK)

/*! @name PERIPHID2 - Peripheral ID Register */
#define MTB_DWT_PERIPHID2_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID2_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID2_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID2_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID2_PERIPHID_MASK)

/*! @name PERIPHID3 - Peripheral ID Register */
#define MTB_DWT_PERIPHID3_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_DWT_PERIPHID3_PERIPHID_SHIFT         (0U)
#define MTB_DWT_PERIPHID3_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_DWT_PERIPHID3_PERIPHID_SHIFT)) & MTB_DWT_PERIPHID3_PERIPHID_MASK)

/*! @name COMPID - Component ID Register */
#define MTB_DWT_COMPID_COMPID_MASK               (0xFFFFFFFFU)
#define MTB_DWT_COMPID_COMPID_SHIFT              (0U)
#define MTB_DWT_COMPID_COMPID(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_DWT_COMPID_COMPID_SHIFT)) & MTB_DWT_COMPID_COMPID_MASK)

/* The count of MTB_DWT_COMPID */
#define MTB_DWT_COMPID_COUNT                     (4U)


/*!
 * @}
 */ /* end of group MTB_DWT_Register_Masks */


/* MTB_DWT - Peripheral instance base addresses */
/** Peripheral MTB0_DWT base address */
#define MTB0_DWT_BASE                            (0xF0001000u)
/** Peripheral MTB0_DWT base pointer */
#define MTB0_DWT                                 ((MTB_DWT_Type *)MTB0_DWT_BASE)
/** Array initializer of MTB_DWT peripheral base addresses */
#define MTB_DWT_BASE_ADDRS                       { MTB0_DWT_BASE }
/** Array initializer of MTB_DWT peripheral base pointers */
#define MTB_DWT_BASE_PTRS                        { MTB0_DWT }

/*!
 * @}
 */ /* end of group MTB_DWT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MTB_ROM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_ROM_Peripheral_Access_Layer MTB_ROM Peripheral Access Layer
 * @{
 */

/** MTB_ROM - Register Layout Typedef */
typedef struct {
  __I  uint32_t ENTRY[4];                          /**< Entry, array offset: 0x0, array step: 0x4 */
  __I  uint32_t TABLEMARK;                         /**< End of Table Marker Register, offset: 0x10 */
       uint8_t RESERVED_0[4024];
  __I  uint32_t SYSACCESS;                         /**< System Access Register, offset: 0xFCC */
  __I  uint32_t PERIPHID4;                         /**< Peripheral ID Register, offset: 0xFD0 */
  __I  uint32_t PERIPHID5;                         /**< Peripheral ID Register, offset: 0xFD4 */
  __I  uint32_t PERIPHID6;                         /**< Peripheral ID Register, offset: 0xFD8 */
  __I  uint32_t PERIPHID7;                         /**< Peripheral ID Register, offset: 0xFDC */
  __I  uint32_t PERIPHID0;                         /**< Peripheral ID Register, offset: 0xFE0 */
  __I  uint32_t PERIPHID1;                         /**< Peripheral ID Register, offset: 0xFE4 */
  __I  uint32_t PERIPHID2;                         /**< Peripheral ID Register, offset: 0xFE8 */
  __I  uint32_t PERIPHID3;                         /**< Peripheral ID Register, offset: 0xFEC */
  __I  uint32_t COMPID[4];                         /**< Component ID Register, array offset: 0xFF0, array step: 0x4 */
} MTB_ROM_Type;

/* ----------------------------------------------------------------------------
   -- MTB_ROM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTB_ROM_Register_Masks MTB_ROM Register Masks
 * @{
 */

/*! @name ENTRY - Entry */
#define MTB_ROM_ENTRY_ENTRY_MASK                 (0xFFFFFFFFU)
#define MTB_ROM_ENTRY_ENTRY_SHIFT                (0U)
#define MTB_ROM_ENTRY_ENTRY(x)                   (((uint32_t)(((uint32_t)(x)) << MTB_ROM_ENTRY_ENTRY_SHIFT)) & MTB_ROM_ENTRY_ENTRY_MASK)

/* The count of MTB_ROM_ENTRY */
#define MTB_ROM_ENTRY_COUNT                      (4U)

/*! @name TABLEMARK - End of Table Marker Register */
#define MTB_ROM_TABLEMARK_MARK_MASK              (0xFFFFFFFFU)
#define MTB_ROM_TABLEMARK_MARK_SHIFT             (0U)
#define MTB_ROM_TABLEMARK_MARK(x)                (((uint32_t)(((uint32_t)(x)) << MTB_ROM_TABLEMARK_MARK_SHIFT)) & MTB_ROM_TABLEMARK_MARK_MASK)

/*! @name SYSACCESS - System Access Register */
#define MTB_ROM_SYSACCESS_SYSACCESS_MASK         (0xFFFFFFFFU)
#define MTB_ROM_SYSACCESS_SYSACCESS_SHIFT        (0U)
#define MTB_ROM_SYSACCESS_SYSACCESS(x)           (((uint32_t)(((uint32_t)(x)) << MTB_ROM_SYSACCESS_SYSACCESS_SHIFT)) & MTB_ROM_SYSACCESS_SYSACCESS_MASK)

/*! @name PERIPHID4 - Peripheral ID Register */
#define MTB_ROM_PERIPHID4_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID4_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID4_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID4_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID4_PERIPHID_MASK)

/*! @name PERIPHID5 - Peripheral ID Register */
#define MTB_ROM_PERIPHID5_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID5_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID5_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID5_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID5_PERIPHID_MASK)

/*! @name PERIPHID6 - Peripheral ID Register */
#define MTB_ROM_PERIPHID6_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID6_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID6_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID6_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID6_PERIPHID_MASK)

/*! @name PERIPHID7 - Peripheral ID Register */
#define MTB_ROM_PERIPHID7_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID7_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID7_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID7_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID7_PERIPHID_MASK)

/*! @name PERIPHID0 - Peripheral ID Register */
#define MTB_ROM_PERIPHID0_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID0_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID0_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID0_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID0_PERIPHID_MASK)

/*! @name PERIPHID1 - Peripheral ID Register */
#define MTB_ROM_PERIPHID1_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID1_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID1_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID1_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID1_PERIPHID_MASK)

/*! @name PERIPHID2 - Peripheral ID Register */
#define MTB_ROM_PERIPHID2_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID2_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID2_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID2_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID2_PERIPHID_MASK)

/*! @name PERIPHID3 - Peripheral ID Register */
#define MTB_ROM_PERIPHID3_PERIPHID_MASK          (0xFFFFFFFFU)
#define MTB_ROM_PERIPHID3_PERIPHID_SHIFT         (0U)
#define MTB_ROM_PERIPHID3_PERIPHID(x)            (((uint32_t)(((uint32_t)(x)) << MTB_ROM_PERIPHID3_PERIPHID_SHIFT)) & MTB_ROM_PERIPHID3_PERIPHID_MASK)

/*! @name COMPID - Component ID Register */
#define MTB_ROM_COMPID_COMPID_MASK               (0xFFFFFFFFU)
#define MTB_ROM_COMPID_COMPID_SHIFT              (0U)
#define MTB_ROM_COMPID_COMPID(x)                 (((uint32_t)(((uint32_t)(x)) << MTB_ROM_COMPID_COMPID_SHIFT)) & MTB_ROM_COMPID_COMPID_MASK)

/* The count of MTB_ROM_COMPID */
#define MTB_ROM_COMPID_COUNT                     (4U)


/*!
 * @}
 */ /* end of group MTB_ROM_Register_Masks */


/* MTB_ROM - Peripheral instance base addresses */
/** Peripheral MTB0_ROM base address */
#define MTB0_ROM_BASE                            (0xF0002000u)
/** Peripheral MTB0_ROM base pointer */
#define MTB0_ROM                                 ((MTB_ROM_Type *)MTB0_ROM_BASE)
/** Array initializer of MTB_ROM peripheral base addresses */
#define MTB_ROM_BASE_ADDRS                       { MTB0_ROM_BASE }
/** Array initializer of MTB_ROM peripheral base pointers */
#define MTB_ROM_BASE_PTRS                        { MTB0_ROM }

/*!
 * @}
 */ /* end of group MTB_ROM_Peripheral_Access_Layer */


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
#define NV_FOPT_LPBOOT0_MASK                     (0x1U)
#define NV_FOPT_LPBOOT0_SHIFT                    (0U)
#define NV_FOPT_LPBOOT0(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FOPT_LPBOOT0_SHIFT)) & NV_FOPT_LPBOOT0_MASK)
#define NV_FOPT_BOOTPIN_OPT_MASK                 (0x2U)
#define NV_FOPT_BOOTPIN_OPT_SHIFT                (1U)
#define NV_FOPT_BOOTPIN_OPT(x)                   (((uint8_t)(((uint8_t)(x)) << NV_FOPT_BOOTPIN_OPT_SHIFT)) & NV_FOPT_BOOTPIN_OPT_MASK)
#define NV_FOPT_NMI_DIS_MASK                     (0x4U)
#define NV_FOPT_NMI_DIS_SHIFT                    (2U)
#define NV_FOPT_NMI_DIS(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FOPT_NMI_DIS_SHIFT)) & NV_FOPT_NMI_DIS_MASK)
#define NV_FOPT_RESET_PIN_CFG_MASK               (0x8U)
#define NV_FOPT_RESET_PIN_CFG_SHIFT              (3U)
#define NV_FOPT_RESET_PIN_CFG(x)                 (((uint8_t)(((uint8_t)(x)) << NV_FOPT_RESET_PIN_CFG_SHIFT)) & NV_FOPT_RESET_PIN_CFG_MASK)
#define NV_FOPT_LPBOOT1_MASK                     (0x10U)
#define NV_FOPT_LPBOOT1_SHIFT                    (4U)
#define NV_FOPT_LPBOOT1(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FOPT_LPBOOT1_SHIFT)) & NV_FOPT_LPBOOT1_MASK)
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
   -- PCC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Peripheral_Access_Layer PCC Peripheral Access Layer
 * @{
 */

/** PCC - Register Layout Typedef */
typedef struct {
  __IO uint32_t CLKCFG[121];                       /**< PCC CLKCFG Register, array offset: 0x0, array step: 0x4 */
} PCC_Type;

/* ----------------------------------------------------------------------------
   -- PCC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Register_Masks PCC Register Masks
 * @{
 */

/*! @name CLKCFG - PCC CLKCFG Register */
#define PCC_CLKCFG_PCD_MASK                      (0x7U)
#define PCC_CLKCFG_PCD_SHIFT                     (0U)
#define PCC_CLKCFG_PCD(x)                        (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_PCD_SHIFT)) & PCC_CLKCFG_PCD_MASK)
#define PCC_CLKCFG_FRAC_MASK                     (0x8U)
#define PCC_CLKCFG_FRAC_SHIFT                    (3U)
#define PCC_CLKCFG_FRAC(x)                       (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_FRAC_SHIFT)) & PCC_CLKCFG_FRAC_MASK)
#define PCC_CLKCFG_PCS_MASK                      (0x7000000U)
#define PCC_CLKCFG_PCS_SHIFT                     (24U)
#define PCC_CLKCFG_PCS(x)                        (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_PCS_SHIFT)) & PCC_CLKCFG_PCS_MASK)
#define PCC_CLKCFG_INUSE_MASK                    (0x20000000U)
#define PCC_CLKCFG_INUSE_SHIFT                   (29U)
#define PCC_CLKCFG_INUSE(x)                      (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_INUSE_SHIFT)) & PCC_CLKCFG_INUSE_MASK)
#define PCC_CLKCFG_CGC_MASK                      (0x40000000U)
#define PCC_CLKCFG_CGC_SHIFT                     (30U)
#define PCC_CLKCFG_CGC(x)                        (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_CGC_SHIFT)) & PCC_CLKCFG_CGC_MASK)
#define PCC_CLKCFG_PR_MASK                       (0x80000000U)
#define PCC_CLKCFG_PR_SHIFT                      (31U)
#define PCC_CLKCFG_PR(x)                         (((uint32_t)(((uint32_t)(x)) << PCC_CLKCFG_PR_SHIFT)) & PCC_CLKCFG_PR_MASK)

/* The count of PCC_CLKCFG */
#define PCC_CLKCFG_COUNT                         (121U)


/*!
 * @}
 */ /* end of group PCC_Register_Masks */


/* PCC - Peripheral instance base addresses */
/** Peripheral PCC0 base address */
#define PCC0_BASE                                (0x4007A000u)
/** Peripheral PCC0 base pointer */
#define PCC0                                     ((PCC_Type *)PCC0_BASE)
/** Peripheral PCC1 base address */
#define PCC1_BASE                                (0x400FA000u)
/** Peripheral PCC1 base pointer */
#define PCC1                                     ((PCC_Type *)PCC1_BASE)
/** Array initializer of PCC peripheral base addresses */
#define PCC_BASE_ADDRS                           { PCC0_BASE, PCC1_BASE }
/** Array initializer of PCC peripheral base pointers */
#define PCC_BASE_PTRS                            { PCC0, PCC1 }
#define PCC_INSTANCE_MASK      0xF
#define PCC_INSTANCE_SHIFT     12
#define PCC_PERIPHERAL_MASK    0xFFF
#define PCC_PERIPHERAL_SHIFT   0
#define PCC_INSTANCE_0    0
#define PCC_INSTANCE_1    1

#define PCC_DMA0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 8)
#define PCC_FLASH_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 32)
#define PCC_DMAMUX0_INDEX   ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 33)
#define PCC_INTMUX0_INDEX   ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 36)
#define PCC_TPM2_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 46)
#define PCC_LPIT0_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 48)
#define PCC_LPTMR0_INDEX    ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 52)
#define PCC_RTC_INDEX       ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 56)
#define PCC_LPSPI2_INDEX    ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 62)
#define PCC_LPI2C2_INDEX    ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 66)
#define PCC_LPUART2_INDEX   ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 70)
#define PCC_SAI0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 76)
#define PCC_EMVSIM0_INDEX   ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 78)
#define PCC_USB0FS_INDEX    ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 85)
#define PCC_PORTA_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 90)
#define PCC_PORTB_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 91)
#define PCC_PORTC_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 92)
#define PCC_PORTD_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 93)
#define PCC_PORTE_INDEX     ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 94)
#define PCC_TSI0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 98)
#define PCC_ADC0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 102)
#define PCC_DAC0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 106)
#define PCC_CMP0_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 110)
#define PCC_VREF_INDEX      ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 114)
#define PCC_CRC_INDEX       ((uint16_t)((PCC_INSTANCE_0 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 120)
#define PCC_DMA0            (PCC0->CLKCFG[8])
#define PCC_FLASH           (PCC0->CLKCFG[32])
#define PCC_DMAMUX0         (PCC0->CLKCFG[33])
#define PCC_INTMUX0         (PCC0->CLKCFG[36])
#define PCC_TPM2            (PCC0->CLKCFG[46])
#define PCC_LPIT0           (PCC0->CLKCFG[48])
#define PCC_LPTMR0          (PCC0->CLKCFG[52])
#define PCC_RTC             (PCC0->CLKCFG[56])
#define PCC_LPSPI2          (PCC0->CLKCFG[62])
#define PCC_LPI2C2          (PCC0->CLKCFG[66])
#define PCC_LPUART2         (PCC0->CLKCFG[70])
#define PCC_SAI0            (PCC0->CLKCFG[76])
#define PCC_EMVSIM0         (PCC0->CLKCFG[78])
#define PCC_USB0FS          (PCC0->CLKCFG[85])
#define PCC_PORTA           (PCC0->CLKCFG[90])
#define PCC_PORTB           (PCC0->CLKCFG[91])
#define PCC_PORTC           (PCC0->CLKCFG[92])
#define PCC_PORTD           (PCC0->CLKCFG[93])
#define PCC_PORTE           (PCC0->CLKCFG[94])
#define PCC_TSI0            (PCC0->CLKCFG[98])
#define PCC_ADC0            (PCC0->CLKCFG[102])
#define PCC_DAC0            (PCC0->CLKCFG[106])
#define PCC_CMP0            (PCC0->CLKCFG[110])
#define PCC_VREF            (PCC0->CLKCFG[114])
#define PCC_CRC             (PCC0->CLKCFG[120])

#define PCC_TRNG_INDEX      ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 37)
#define PCC_TPM0_INDEX      ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 44)
#define PCC_TPM1_INDEX      ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 45)
#define PCC_LPTMR1_INDEX    ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 53)
#define PCC_LPSPI0_INDEX    ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 60)
#define PCC_LPSPI1_INDEX    ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 61)
#define PCC_LPI2C0_INDEX    ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 64)
#define PCC_LPI2C1_INDEX    ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 65)
#define PCC_LPUART0_INDEX   ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 68)
#define PCC_LPUART1_INDEX   ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 69)
#define PCC_FLEXIO0_INDEX   ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 74)
#define PCC_PORTM_INDEX     ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 96)
#define PCC_CMP1_INDEX      ((uint16_t)((PCC_INSTANCE_1 & PCC_INSTANCE_MASK) << PCC_INSTANCE_SHIFT) | 111)
#define PCC_TRNG            (PCC1->CLKCFG[37])
#define PCC_TPM0            (PCC1->CLKCFG[44])
#define PCC_TPM1            (PCC1->CLKCFG[45])
#define PCC_LPTMR1          (PCC1->CLKCFG[53])
#define PCC_LPSPI0          (PCC1->CLKCFG[60])
#define PCC_LPSPI1          (PCC1->CLKCFG[61])
#define PCC_LPI2C0          (PCC1->CLKCFG[64])
#define PCC_LPI2C1          (PCC1->CLKCFG[65])
#define PCC_LPUART0         (PCC1->CLKCFG[68])
#define PCC_LPUART1         (PCC1->CLKCFG[69])
#define PCC_FLEXIO0         (PCC1->CLKCFG[74])
#define PCC_CMP1            (PCC1->CLKCFG[111])


/*!
 * @}
 */ /* end of group PCC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter register, offset: 0x4 */
  __IO uint32_t LVDSC1;                            /**< Low Voltage Detect Status And Control 1 register, offset: 0x8 */
  __IO uint32_t LVDSC2;                            /**< Low Voltage Detect Status And Control 2 register, offset: 0xC */
  __IO uint32_t REGSC;                             /**< Regulator Status And Control register, offset: 0x10 */
       uint8_t RESERVED_0[32];
  __IO uint32_t HVDSC1;                            /**< High Voltage Detect Status And Control 1 register, offset: 0x34 */
} PMC_Type;

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name VERID - Version ID register */
#define PMC_VERID_FEATURE_MASK                   (0xFFFFU)
#define PMC_VERID_FEATURE_SHIFT                  (0U)
#define PMC_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << PMC_VERID_FEATURE_SHIFT)) & PMC_VERID_FEATURE_MASK)
#define PMC_VERID_MINOR_MASK                     (0xFF0000U)
#define PMC_VERID_MINOR_SHIFT                    (16U)
#define PMC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_VERID_MINOR_SHIFT)) & PMC_VERID_MINOR_MASK)
#define PMC_VERID_MAJOR_MASK                     (0xFF000000U)
#define PMC_VERID_MAJOR_SHIFT                    (24U)
#define PMC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_VERID_MAJOR_SHIFT)) & PMC_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter register */
#define PMC_PARAM_VLPOE_MASK                     (0x1U)
#define PMC_PARAM_VLPOE_SHIFT                    (0U)
#define PMC_PARAM_VLPOE(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_PARAM_VLPOE_SHIFT)) & PMC_PARAM_VLPOE_MASK)
#define PMC_PARAM_HVDE_MASK                      (0x2U)
#define PMC_PARAM_HVDE_SHIFT                     (1U)
#define PMC_PARAM_HVDE(x)                        (((uint32_t)(((uint32_t)(x)) << PMC_PARAM_HVDE_SHIFT)) & PMC_PARAM_HVDE_MASK)

/*! @name LVDSC1 - Low Voltage Detect Status And Control 1 register */
#define PMC_LVDSC1_LVDV_MASK                     (0x3U)
#define PMC_LVDSC1_LVDV_SHIFT                    (0U)
#define PMC_LVDSC1_LVDV(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC1_LVDV_SHIFT)) & PMC_LVDSC1_LVDV_MASK)
#define PMC_LVDSC1_LVDRE_MASK                    (0x10U)
#define PMC_LVDSC1_LVDRE_SHIFT                   (4U)
#define PMC_LVDSC1_LVDRE(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC1_LVDRE_SHIFT)) & PMC_LVDSC1_LVDRE_MASK)
#define PMC_LVDSC1_LVDIE_MASK                    (0x20U)
#define PMC_LVDSC1_LVDIE_SHIFT                   (5U)
#define PMC_LVDSC1_LVDIE(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC1_LVDIE_SHIFT)) & PMC_LVDSC1_LVDIE_MASK)
#define PMC_LVDSC1_LVDACK_MASK                   (0x40U)
#define PMC_LVDSC1_LVDACK_SHIFT                  (6U)
#define PMC_LVDSC1_LVDACK(x)                     (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC1_LVDACK_SHIFT)) & PMC_LVDSC1_LVDACK_MASK)
#define PMC_LVDSC1_LVDF_MASK                     (0x80U)
#define PMC_LVDSC1_LVDF_SHIFT                    (7U)
#define PMC_LVDSC1_LVDF(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC1_LVDF_SHIFT)) & PMC_LVDSC1_LVDF_MASK)

/*! @name LVDSC2 - Low Voltage Detect Status And Control 2 register */
#define PMC_LVDSC2_LVWV_MASK                     (0x3U)
#define PMC_LVDSC2_LVWV_SHIFT                    (0U)
#define PMC_LVDSC2_LVWV(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC2_LVWV_SHIFT)) & PMC_LVDSC2_LVWV_MASK)
#define PMC_LVDSC2_LVWIE_MASK                    (0x20U)
#define PMC_LVDSC2_LVWIE_SHIFT                   (5U)
#define PMC_LVDSC2_LVWIE(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC2_LVWIE_SHIFT)) & PMC_LVDSC2_LVWIE_MASK)
#define PMC_LVDSC2_LVWACK_MASK                   (0x40U)
#define PMC_LVDSC2_LVWACK_SHIFT                  (6U)
#define PMC_LVDSC2_LVWACK(x)                     (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC2_LVWACK_SHIFT)) & PMC_LVDSC2_LVWACK_MASK)
#define PMC_LVDSC2_LVWF_MASK                     (0x80U)
#define PMC_LVDSC2_LVWF_SHIFT                    (7U)
#define PMC_LVDSC2_LVWF(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_LVDSC2_LVWF_SHIFT)) & PMC_LVDSC2_LVWF_MASK)

/*! @name REGSC - Regulator Status And Control register */
#define PMC_REGSC_BGBE_MASK                      (0x1U)
#define PMC_REGSC_BGBE_SHIFT                     (0U)
#define PMC_REGSC_BGBE(x)                        (((uint32_t)(((uint32_t)(x)) << PMC_REGSC_BGBE_SHIFT)) & PMC_REGSC_BGBE_MASK)
#define PMC_REGSC_REGONS_MASK                    (0x4U)
#define PMC_REGSC_REGONS_SHIFT                   (2U)
#define PMC_REGSC_REGONS(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_REGSC_REGONS_SHIFT)) & PMC_REGSC_REGONS_MASK)
#define PMC_REGSC_ACKISO_MASK                    (0x8U)
#define PMC_REGSC_ACKISO_SHIFT                   (3U)
#define PMC_REGSC_ACKISO(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_REGSC_ACKISO_SHIFT)) & PMC_REGSC_ACKISO_MASK)
#define PMC_REGSC_BGEN_MASK                      (0x10U)
#define PMC_REGSC_BGEN_SHIFT                     (4U)
#define PMC_REGSC_BGEN(x)                        (((uint32_t)(((uint32_t)(x)) << PMC_REGSC_BGEN_SHIFT)) & PMC_REGSC_BGEN_MASK)
#define PMC_REGSC_VLPO_MASK                      (0x40U)
#define PMC_REGSC_VLPO_SHIFT                     (6U)
#define PMC_REGSC_VLPO(x)                        (((uint32_t)(((uint32_t)(x)) << PMC_REGSC_VLPO_SHIFT)) & PMC_REGSC_VLPO_MASK)

/*! @name HVDSC1 - High Voltage Detect Status And Control 1 register */
#define PMC_HVDSC1_HVDV_MASK                     (0x1U)
#define PMC_HVDSC1_HVDV_SHIFT                    (0U)
#define PMC_HVDSC1_HVDV(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_HVDSC1_HVDV_SHIFT)) & PMC_HVDSC1_HVDV_MASK)
#define PMC_HVDSC1_HVDRE_MASK                    (0x10U)
#define PMC_HVDSC1_HVDRE_SHIFT                   (4U)
#define PMC_HVDSC1_HVDRE(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_HVDSC1_HVDRE_SHIFT)) & PMC_HVDSC1_HVDRE_MASK)
#define PMC_HVDSC1_HVDIE_MASK                    (0x20U)
#define PMC_HVDSC1_HVDIE_SHIFT                   (5U)
#define PMC_HVDSC1_HVDIE(x)                      (((uint32_t)(((uint32_t)(x)) << PMC_HVDSC1_HVDIE_SHIFT)) & PMC_HVDSC1_HVDIE_MASK)
#define PMC_HVDSC1_HVDACK_MASK                   (0x40U)
#define PMC_HVDSC1_HVDACK_SHIFT                  (6U)
#define PMC_HVDSC1_HVDACK(x)                     (((uint32_t)(((uint32_t)(x)) << PMC_HVDSC1_HVDACK_SHIFT)) & PMC_HVDSC1_HVDACK_MASK)
#define PMC_HVDSC1_HVDF_MASK                     (0x80U)
#define PMC_HVDSC1_HVDF_SHIFT                    (7U)
#define PMC_HVDSC1_HVDF(x)                       (((uint32_t)(((uint32_t)(x)) << PMC_HVDSC1_HVDF_SHIFT)) & PMC_HVDSC1_HVDF_MASK)


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
#define PMC_IRQS                                 { PMC_IRQn }

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
  __O  uint32_t GICLR;                             /**< Global Interrupt Control Low Register, offset: 0x88 */
  __O  uint32_t GICHR;                             /**< Global Interrupt Control High Register, offset: 0x8C */
       uint8_t RESERVED_0[16];
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

/*! @name GICLR - Global Interrupt Control Low Register */
#define PORT_GICLR_GIWE_MASK                     (0xFFFFU)
#define PORT_GICLR_GIWE_SHIFT                    (0U)
#define PORT_GICLR_GIWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICLR_GIWE_SHIFT)) & PORT_GICLR_GIWE_MASK)
#define PORT_GICLR_GIWD_MASK                     (0xFFFF0000U)
#define PORT_GICLR_GIWD_SHIFT                    (16U)
#define PORT_GICLR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICLR_GIWD_SHIFT)) & PORT_GICLR_GIWD_MASK)

/*! @name GICHR - Global Interrupt Control High Register */
#define PORT_GICHR_GIWE_MASK                     (0xFFFFU)
#define PORT_GICHR_GIWE_SHIFT                    (0U)
#define PORT_GICHR_GIWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICHR_GIWE_SHIFT)) & PORT_GICHR_GIWE_MASK)
#define PORT_GICHR_GIWD_MASK                     (0xFFFF0000U)
#define PORT_GICHR_GIWD_SHIFT                    (16U)
#define PORT_GICHR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICHR_GIWD_SHIFT)) & PORT_GICHR_GIWD_MASK)

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
#define PORTA_BASE                               (0x4005A000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_Type *)PORTA_BASE)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4005B000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_Type *)PORTB_BASE)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4005C000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_Type *)PORTC_BASE)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4005D000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_Type *)PORTD_BASE)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4005E000u)
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
   -- RCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Peripheral_Access_Layer RCM Peripheral Access Layer
 * @{
 */

/** RCM - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __I  uint32_t SRS;                               /**< System Reset Status Register, offset: 0x8 */
  __IO uint32_t RPC;                               /**< Reset Pin Control register, offset: 0xC */
  __IO uint32_t MR;                                /**< Mode Register, offset: 0x10 */
  __IO uint32_t FM;                                /**< Force Mode Register, offset: 0x14 */
  __IO uint32_t SSRS;                              /**< Sticky System Reset Status Register, offset: 0x18 */
  __IO uint32_t SRIE;                              /**< System Reset Interrupt Enable Register, offset: 0x1C */
} RCM_Type;

/* ----------------------------------------------------------------------------
   -- RCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Masks RCM Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define RCM_VERID_FEATURE_MASK                   (0xFFFFU)
#define RCM_VERID_FEATURE_SHIFT                  (0U)
#define RCM_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_VERID_FEATURE_SHIFT)) & RCM_VERID_FEATURE_MASK)
#define RCM_VERID_MINOR_MASK                     (0xFF0000U)
#define RCM_VERID_MINOR_SHIFT                    (16U)
#define RCM_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_VERID_MINOR_SHIFT)) & RCM_VERID_MINOR_MASK)
#define RCM_VERID_MAJOR_MASK                     (0xFF000000U)
#define RCM_VERID_MAJOR_SHIFT                    (24U)
#define RCM_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_VERID_MAJOR_SHIFT)) & RCM_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define RCM_PARAM_EWAKEUP_MASK                   (0x1U)
#define RCM_PARAM_EWAKEUP_SHIFT                  (0U)
#define RCM_PARAM_EWAKEUP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EWAKEUP_SHIFT)) & RCM_PARAM_EWAKEUP_MASK)
#define RCM_PARAM_ELVD_MASK                      (0x2U)
#define RCM_PARAM_ELVD_SHIFT                     (1U)
#define RCM_PARAM_ELVD(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELVD_SHIFT)) & RCM_PARAM_ELVD_MASK)
#define RCM_PARAM_ELOC_MASK                      (0x4U)
#define RCM_PARAM_ELOC_SHIFT                     (2U)
#define RCM_PARAM_ELOC(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOC_SHIFT)) & RCM_PARAM_ELOC_MASK)
#define RCM_PARAM_ELOL_MASK                      (0x8U)
#define RCM_PARAM_ELOL_SHIFT                     (3U)
#define RCM_PARAM_ELOL(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOL_SHIFT)) & RCM_PARAM_ELOL_MASK)
#define RCM_PARAM_EWDOG_MASK                     (0x20U)
#define RCM_PARAM_EWDOG_SHIFT                    (5U)
#define RCM_PARAM_EWDOG(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EWDOG_SHIFT)) & RCM_PARAM_EWDOG_MASK)
#define RCM_PARAM_EPIN_MASK                      (0x40U)
#define RCM_PARAM_EPIN_SHIFT                     (6U)
#define RCM_PARAM_EPIN(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EPIN_SHIFT)) & RCM_PARAM_EPIN_MASK)
#define RCM_PARAM_EPOR_MASK                      (0x80U)
#define RCM_PARAM_EPOR_SHIFT                     (7U)
#define RCM_PARAM_EPOR(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EPOR_SHIFT)) & RCM_PARAM_EPOR_MASK)
#define RCM_PARAM_EJTAG_MASK                     (0x100U)
#define RCM_PARAM_EJTAG_SHIFT                    (8U)
#define RCM_PARAM_EJTAG(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EJTAG_SHIFT)) & RCM_PARAM_EJTAG_MASK)
#define RCM_PARAM_ELOCKUP_MASK                   (0x200U)
#define RCM_PARAM_ELOCKUP_SHIFT                  (9U)
#define RCM_PARAM_ELOCKUP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOCKUP_SHIFT)) & RCM_PARAM_ELOCKUP_MASK)
#define RCM_PARAM_ESW_MASK                       (0x400U)
#define RCM_PARAM_ESW_SHIFT                      (10U)
#define RCM_PARAM_ESW(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ESW_SHIFT)) & RCM_PARAM_ESW_MASK)
#define RCM_PARAM_EMDM_AP_MASK                   (0x800U)
#define RCM_PARAM_EMDM_AP_SHIFT                  (11U)
#define RCM_PARAM_EMDM_AP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EMDM_AP_SHIFT)) & RCM_PARAM_EMDM_AP_MASK)
#define RCM_PARAM_ESACKERR_MASK                  (0x2000U)
#define RCM_PARAM_ESACKERR_SHIFT                 (13U)
#define RCM_PARAM_ESACKERR(x)                    (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ESACKERR_SHIFT)) & RCM_PARAM_ESACKERR_MASK)
#define RCM_PARAM_ETAMPER_MASK                   (0x8000U)
#define RCM_PARAM_ETAMPER_SHIFT                  (15U)
#define RCM_PARAM_ETAMPER(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ETAMPER_SHIFT)) & RCM_PARAM_ETAMPER_MASK)
#define RCM_PARAM_ECORE1_MASK                    (0x10000U)
#define RCM_PARAM_ECORE1_SHIFT                   (16U)
#define RCM_PARAM_ECORE1(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ECORE1_SHIFT)) & RCM_PARAM_ECORE1_MASK)

/*! @name SRS - System Reset Status Register */
#define RCM_SRS_WAKEUP_MASK                      (0x1U)
#define RCM_SRS_WAKEUP_SHIFT                     (0U)
#define RCM_SRS_WAKEUP(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRS_WAKEUP_SHIFT)) & RCM_SRS_WAKEUP_MASK)
#define RCM_SRS_LVD_MASK                         (0x2U)
#define RCM_SRS_LVD_SHIFT                        (1U)
#define RCM_SRS_LVD(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LVD_SHIFT)) & RCM_SRS_LVD_MASK)
#define RCM_SRS_LOC_MASK                         (0x4U)
#define RCM_SRS_LOC_SHIFT                        (2U)
#define RCM_SRS_LOC(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOC_SHIFT)) & RCM_SRS_LOC_MASK)
#define RCM_SRS_LOL_MASK                         (0x8U)
#define RCM_SRS_LOL_SHIFT                        (3U)
#define RCM_SRS_LOL(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOL_SHIFT)) & RCM_SRS_LOL_MASK)
#define RCM_SRS_WDOG_MASK                        (0x20U)
#define RCM_SRS_WDOG_SHIFT                       (5U)
#define RCM_SRS_WDOG(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRS_WDOG_SHIFT)) & RCM_SRS_WDOG_MASK)
#define RCM_SRS_PIN_MASK                         (0x40U)
#define RCM_SRS_PIN_SHIFT                        (6U)
#define RCM_SRS_PIN(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_PIN_SHIFT)) & RCM_SRS_PIN_MASK)
#define RCM_SRS_POR_MASK                         (0x80U)
#define RCM_SRS_POR_SHIFT                        (7U)
#define RCM_SRS_POR(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_POR_SHIFT)) & RCM_SRS_POR_MASK)
#define RCM_SRS_LOCKUP_MASK                      (0x200U)
#define RCM_SRS_LOCKUP_SHIFT                     (9U)
#define RCM_SRS_LOCKUP(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOCKUP_SHIFT)) & RCM_SRS_LOCKUP_MASK)
#define RCM_SRS_SW_MASK                          (0x400U)
#define RCM_SRS_SW_SHIFT                         (10U)
#define RCM_SRS_SW(x)                            (((uint32_t)(((uint32_t)(x)) << RCM_SRS_SW_SHIFT)) & RCM_SRS_SW_MASK)
#define RCM_SRS_MDM_AP_MASK                      (0x800U)
#define RCM_SRS_MDM_AP_SHIFT                     (11U)
#define RCM_SRS_MDM_AP(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRS_MDM_AP_SHIFT)) & RCM_SRS_MDM_AP_MASK)
#define RCM_SRS_SACKERR_MASK                     (0x2000U)
#define RCM_SRS_SACKERR_SHIFT                    (13U)
#define RCM_SRS_SACKERR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRS_SACKERR_SHIFT)) & RCM_SRS_SACKERR_MASK)

/*! @name RPC - Reset Pin Control register */
#define RCM_RPC_RSTFLTSRW_MASK                   (0x3U)
#define RCM_RPC_RSTFLTSRW_SHIFT                  (0U)
#define RCM_RPC_RSTFLTSRW(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSRW_SHIFT)) & RCM_RPC_RSTFLTSRW_MASK)
#define RCM_RPC_RSTFLTSS_MASK                    (0x4U)
#define RCM_RPC_RSTFLTSS_SHIFT                   (2U)
#define RCM_RPC_RSTFLTSS(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSS_SHIFT)) & RCM_RPC_RSTFLTSS_MASK)
#define RCM_RPC_RSTFLTSEL_MASK                   (0x1F00U)
#define RCM_RPC_RSTFLTSEL_SHIFT                  (8U)
#define RCM_RPC_RSTFLTSEL(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSEL_SHIFT)) & RCM_RPC_RSTFLTSEL_MASK)

/*! @name MR - Mode Register */
#define RCM_MR_BOOTROM_MASK                      (0x6U)
#define RCM_MR_BOOTROM_SHIFT                     (1U)
#define RCM_MR_BOOTROM(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_MR_BOOTROM_SHIFT)) & RCM_MR_BOOTROM_MASK)

/*! @name FM - Force Mode Register */
#define RCM_FM_FORCEROM_MASK                     (0x6U)
#define RCM_FM_FORCEROM_SHIFT                    (1U)
#define RCM_FM_FORCEROM(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_FM_FORCEROM_SHIFT)) & RCM_FM_FORCEROM_MASK)

/*! @name SSRS - Sticky System Reset Status Register */
#define RCM_SSRS_SWAKEUP_MASK                    (0x1U)
#define RCM_SSRS_SWAKEUP_SHIFT                   (0U)
#define RCM_SSRS_SWAKEUP(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SWAKEUP_SHIFT)) & RCM_SSRS_SWAKEUP_MASK)
#define RCM_SSRS_SLVD_MASK                       (0x2U)
#define RCM_SSRS_SLVD_SHIFT                      (1U)
#define RCM_SSRS_SLVD(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLVD_SHIFT)) & RCM_SSRS_SLVD_MASK)
#define RCM_SSRS_SLOC_MASK                       (0x4U)
#define RCM_SSRS_SLOC_SHIFT                      (2U)
#define RCM_SSRS_SLOC(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOC_SHIFT)) & RCM_SSRS_SLOC_MASK)
#define RCM_SSRS_SLOL_MASK                       (0x8U)
#define RCM_SSRS_SLOL_SHIFT                      (3U)
#define RCM_SSRS_SLOL(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOL_SHIFT)) & RCM_SSRS_SLOL_MASK)
#define RCM_SSRS_SWDOG_MASK                      (0x20U)
#define RCM_SSRS_SWDOG_SHIFT                     (5U)
#define RCM_SSRS_SWDOG(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SWDOG_SHIFT)) & RCM_SSRS_SWDOG_MASK)
#define RCM_SSRS_SPIN_MASK                       (0x40U)
#define RCM_SSRS_SPIN_SHIFT                      (6U)
#define RCM_SSRS_SPIN(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SPIN_SHIFT)) & RCM_SSRS_SPIN_MASK)
#define RCM_SSRS_SPOR_MASK                       (0x80U)
#define RCM_SSRS_SPOR_SHIFT                      (7U)
#define RCM_SSRS_SPOR(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SPOR_SHIFT)) & RCM_SSRS_SPOR_MASK)
#define RCM_SSRS_SLOCKUP_MASK                    (0x200U)
#define RCM_SSRS_SLOCKUP_SHIFT                   (9U)
#define RCM_SSRS_SLOCKUP(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOCKUP_SHIFT)) & RCM_SSRS_SLOCKUP_MASK)
#define RCM_SSRS_SSW_MASK                        (0x400U)
#define RCM_SSRS_SSW_SHIFT                       (10U)
#define RCM_SSRS_SSW(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SSW_SHIFT)) & RCM_SSRS_SSW_MASK)
#define RCM_SSRS_SMDM_AP_MASK                    (0x800U)
#define RCM_SSRS_SMDM_AP_SHIFT                   (11U)
#define RCM_SSRS_SMDM_AP(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SMDM_AP_SHIFT)) & RCM_SSRS_SMDM_AP_MASK)
#define RCM_SSRS_SSACKERR_MASK                   (0x2000U)
#define RCM_SSRS_SSACKERR_SHIFT                  (13U)
#define RCM_SSRS_SSACKERR(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SSACKERR_SHIFT)) & RCM_SSRS_SSACKERR_MASK)

/*! @name SRIE - System Reset Interrupt Enable Register */
#define RCM_SRIE_DELAY_MASK                      (0x3U)
#define RCM_SRIE_DELAY_SHIFT                     (0U)
#define RCM_SRIE_DELAY(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_DELAY_SHIFT)) & RCM_SRIE_DELAY_MASK)
#define RCM_SRIE_LOC_MASK                        (0x4U)
#define RCM_SRIE_LOC_SHIFT                       (2U)
#define RCM_SRIE_LOC(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOC_SHIFT)) & RCM_SRIE_LOC_MASK)
#define RCM_SRIE_LOL_MASK                        (0x8U)
#define RCM_SRIE_LOL_SHIFT                       (3U)
#define RCM_SRIE_LOL(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOL_SHIFT)) & RCM_SRIE_LOL_MASK)
#define RCM_SRIE_WDOG_MASK                       (0x20U)
#define RCM_SRIE_WDOG_SHIFT                      (5U)
#define RCM_SRIE_WDOG(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_WDOG_SHIFT)) & RCM_SRIE_WDOG_MASK)
#define RCM_SRIE_PIN_MASK                        (0x40U)
#define RCM_SRIE_PIN_SHIFT                       (6U)
#define RCM_SRIE_PIN(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_PIN_SHIFT)) & RCM_SRIE_PIN_MASK)
#define RCM_SRIE_GIE_MASK                        (0x80U)
#define RCM_SRIE_GIE_SHIFT                       (7U)
#define RCM_SRIE_GIE(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_GIE_SHIFT)) & RCM_SRIE_GIE_MASK)
#define RCM_SRIE_LOCKUP_MASK                     (0x200U)
#define RCM_SRIE_LOCKUP_SHIFT                    (9U)
#define RCM_SRIE_LOCKUP(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOCKUP_SHIFT)) & RCM_SRIE_LOCKUP_MASK)
#define RCM_SRIE_SW_MASK                         (0x400U)
#define RCM_SRIE_SW_SHIFT                        (10U)
#define RCM_SRIE_SW(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_SW_SHIFT)) & RCM_SRIE_SW_MASK)
#define RCM_SRIE_MDM_AP_MASK                     (0x800U)
#define RCM_SRIE_MDM_AP_SHIFT                    (11U)
#define RCM_SRIE_MDM_AP(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_MDM_AP_SHIFT)) & RCM_SRIE_MDM_AP_MASK)
#define RCM_SRIE_SACKERR_MASK                    (0x2000U)
#define RCM_SRIE_SACKERR_SHIFT                   (13U)
#define RCM_SRIE_SACKERR(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_SACKERR_SHIFT)) & RCM_SRIE_SACKERR_MASK)


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
/** Interrupt vectors for the RCM peripheral type */
#define RCM_IRQS                                 { RCM_IRQn }

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
#define RFSYS_BASE                               (0x4007C000u)
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
#define RTC_CR_CPS_MASK                          (0x20U)
#define RTC_CR_CPS_SHIFT                         (5U)
#define RTC_CR_CPS(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_CPS_SHIFT)) & RTC_CR_CPS_MASK)
#define RTC_CR_LPOS_MASK                         (0x80U)
#define RTC_CR_LPOS_SHIFT                        (7U)
#define RTC_CR_LPOS(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_LPOS_SHIFT)) & RTC_CR_LPOS_MASK)
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
#define RTC_CR_CPE_MASK                          (0x3000000U)
#define RTC_CR_CPE_SHIFT                         (24U)
#define RTC_CR_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_CPE_SHIFT)) & RTC_CR_CPE_MASK)

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
#define RTC_IER_TSIC_MASK                        (0x70000U)
#define RTC_IER_TSIC_SHIFT                       (16U)
#define RTC_IER_TSIC(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TSIC_SHIFT)) & RTC_IER_TSIC_MASK)


/*!
 * @}
 */ /* end of group RTC_Register_Masks */


/* RTC - Peripheral instance base addresses */
/** Peripheral RTC base address */
#define RTC_BASE                                 (0x40038000u)
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
   -- SCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Peripheral_Access_Layer SCG Peripheral Access Layer
 * @{
 */

/** SCG - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
       uint8_t RESERVED_0[8];
  __I  uint32_t CSR;                               /**< Clock Status Register, offset: 0x10 */
  __IO uint32_t RCCR;                              /**< Run Clock Control Register, offset: 0x14 */
  __IO uint32_t VCCR;                              /**< VLPR Clock Control Register, offset: 0x18 */
  __IO uint32_t HCCR;                              /**< HSRUN Clock Control Register, offset: 0x1C */
  __IO uint32_t CLKOUTCNFG;                        /**< SCG CLKOUT Configuration Register, offset: 0x20 */
       uint8_t RESERVED_1[220];
  __IO uint32_t SOSCCSR;                           /**< System OSC Control Status Register, offset: 0x100 */
  __IO uint32_t SOSCDIV;                           /**< System OSC Divide Register, offset: 0x104 */
  __IO uint32_t SOSCCFG;                           /**< System Oscillator Configuration Register, offset: 0x108 */
       uint8_t RESERVED_2[244];
  __IO uint32_t SIRCCSR;                           /**< Slow IRC Control Status Register, offset: 0x200 */
  __IO uint32_t SIRCDIV;                           /**< Slow IRC Divide Register, offset: 0x204 */
  __IO uint32_t SIRCCFG;                           /**< Slow IRC Configuration Register, offset: 0x208 */
       uint8_t RESERVED_3[244];
  __IO uint32_t FIRCCSR;                           /**< Fast IRC Control Status Register, offset: 0x300 */
  __IO uint32_t FIRCDIV;                           /**< Fast IRC Divide Register, offset: 0x304 */
  __IO uint32_t FIRCCFG;                           /**< Fast IRC Configuration Register, offset: 0x308 */
  __IO uint32_t FIRCTCFG;                          /**< Fast IRC Trim Configuration Register, offset: 0x30C */
       uint8_t RESERVED_4[8];
  __IO uint32_t FIRCSTAT;                          /**< Fast IRC Status Register, offset: 0x318 */
       uint8_t RESERVED_5[740];
  __IO uint32_t SPLLCSR;                           /**< System PLL Control Status Register, offset: 0x600 */
  __IO uint32_t SPLLDIV;                           /**< System PLL Divide Register, offset: 0x604 */
  __IO uint32_t SPLLCFG;                           /**< System PLL Configuration Register, offset: 0x608 */
} SCG_Type;

/* ----------------------------------------------------------------------------
   -- SCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Register_Masks SCG Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
#define SCG_VERID_VERSION_MASK                   (0xFFFFFFFFU)
#define SCG_VERID_VERSION_SHIFT                  (0U)
#define SCG_VERID_VERSION(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_VERID_VERSION_SHIFT)) & SCG_VERID_VERSION_MASK)

/*! @name PARAM - Parameter Register */
#define SCG_PARAM_CLKPRES_MASK                   (0xFFU)
#define SCG_PARAM_CLKPRES_SHIFT                  (0U)
#define SCG_PARAM_CLKPRES(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_CLKPRES_SHIFT)) & SCG_PARAM_CLKPRES_MASK)
#define SCG_PARAM_DIVPRES_MASK                   (0xF8000000U)
#define SCG_PARAM_DIVPRES_SHIFT                  (27U)
#define SCG_PARAM_DIVPRES(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_DIVPRES_SHIFT)) & SCG_PARAM_DIVPRES_MASK)

/*! @name CSR - Clock Status Register */
#define SCG_CSR_DIVSLOW_MASK                     (0xFU)
#define SCG_CSR_DIVSLOW_SHIFT                    (0U)
#define SCG_CSR_DIVSLOW(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_CSR_DIVSLOW_SHIFT)) & SCG_CSR_DIVSLOW_MASK)
#define SCG_CSR_DIVCORE_MASK                     (0xF0000U)
#define SCG_CSR_DIVCORE_SHIFT                    (16U)
#define SCG_CSR_DIVCORE(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_CSR_DIVCORE_SHIFT)) & SCG_CSR_DIVCORE_MASK)
#define SCG_CSR_SCS_MASK                         (0xF000000U)
#define SCG_CSR_SCS_SHIFT                        (24U)
#define SCG_CSR_SCS(x)                           (((uint32_t)(((uint32_t)(x)) << SCG_CSR_SCS_SHIFT)) & SCG_CSR_SCS_MASK)

/*! @name RCCR - Run Clock Control Register */
#define SCG_RCCR_DIVSLOW_MASK                    (0xFU)
#define SCG_RCCR_DIVSLOW_SHIFT                   (0U)
#define SCG_RCCR_DIVSLOW(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_DIVSLOW_SHIFT)) & SCG_RCCR_DIVSLOW_MASK)
#define SCG_RCCR_DIVCORE_MASK                    (0xF0000U)
#define SCG_RCCR_DIVCORE_SHIFT                   (16U)
#define SCG_RCCR_DIVCORE(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_DIVCORE_SHIFT)) & SCG_RCCR_DIVCORE_MASK)
#define SCG_RCCR_SCS_MASK                        (0xF000000U)
#define SCG_RCCR_SCS_SHIFT                       (24U)
#define SCG_RCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_SCS_SHIFT)) & SCG_RCCR_SCS_MASK)

/*! @name VCCR - VLPR Clock Control Register */
#define SCG_VCCR_DIVSLOW_MASK                    (0xFU)
#define SCG_VCCR_DIVSLOW_SHIFT                   (0U)
#define SCG_VCCR_DIVSLOW(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_DIVSLOW_SHIFT)) & SCG_VCCR_DIVSLOW_MASK)
#define SCG_VCCR_DIVCORE_MASK                    (0xF0000U)
#define SCG_VCCR_DIVCORE_SHIFT                   (16U)
#define SCG_VCCR_DIVCORE(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_DIVCORE_SHIFT)) & SCG_VCCR_DIVCORE_MASK)
#define SCG_VCCR_SCS_MASK                        (0xF000000U)
#define SCG_VCCR_SCS_SHIFT                       (24U)
#define SCG_VCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_SCS_SHIFT)) & SCG_VCCR_SCS_MASK)

/*! @name HCCR - HSRUN Clock Control Register */
#define SCG_HCCR_DIVSLOW_MASK                    (0xFU)
#define SCG_HCCR_DIVSLOW_SHIFT                   (0U)
#define SCG_HCCR_DIVSLOW(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_HCCR_DIVSLOW_SHIFT)) & SCG_HCCR_DIVSLOW_MASK)
#define SCG_HCCR_DIVCORE_MASK                    (0xF0000U)
#define SCG_HCCR_DIVCORE_SHIFT                   (16U)
#define SCG_HCCR_DIVCORE(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_HCCR_DIVCORE_SHIFT)) & SCG_HCCR_DIVCORE_MASK)
#define SCG_HCCR_SCS_MASK                        (0xF000000U)
#define SCG_HCCR_SCS_SHIFT                       (24U)
#define SCG_HCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_HCCR_SCS_SHIFT)) & SCG_HCCR_SCS_MASK)

/*! @name CLKOUTCNFG - SCG CLKOUT Configuration Register */
#define SCG_CLKOUTCNFG_CLKOUTSEL_MASK            (0xF000000U)
#define SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT           (24U)
#define SCG_CLKOUTCNFG_CLKOUTSEL(x)              (((uint32_t)(((uint32_t)(x)) << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)) & SCG_CLKOUTCNFG_CLKOUTSEL_MASK)

/*! @name SOSCCSR - System OSC Control Status Register */
#define SCG_SOSCCSR_SOSCEN_MASK                  (0x1U)
#define SCG_SOSCCSR_SOSCEN_SHIFT                 (0U)
#define SCG_SOSCCSR_SOSCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCEN_SHIFT)) & SCG_SOSCCSR_SOSCEN_MASK)
#define SCG_SOSCCSR_SOSCSTEN_MASK                (0x2U)
#define SCG_SOSCCSR_SOSCSTEN_SHIFT               (1U)
#define SCG_SOSCCSR_SOSCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCSTEN_SHIFT)) & SCG_SOSCCSR_SOSCSTEN_MASK)
#define SCG_SOSCCSR_SOSCLPEN_MASK                (0x4U)
#define SCG_SOSCCSR_SOSCLPEN_SHIFT               (2U)
#define SCG_SOSCCSR_SOSCLPEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCLPEN_SHIFT)) & SCG_SOSCCSR_SOSCLPEN_MASK)
#define SCG_SOSCCSR_SOSCERCLKEN_MASK             (0x8U)
#define SCG_SOSCCSR_SOSCERCLKEN_SHIFT            (3U)
#define SCG_SOSCCSR_SOSCERCLKEN(x)               (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCERCLKEN_SHIFT)) & SCG_SOSCCSR_SOSCERCLKEN_MASK)
#define SCG_SOSCCSR_SOSCCM_MASK                  (0x10000U)
#define SCG_SOSCCSR_SOSCCM_SHIFT                 (16U)
#define SCG_SOSCCSR_SOSCCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCM_SHIFT)) & SCG_SOSCCSR_SOSCCM_MASK)
#define SCG_SOSCCSR_SOSCCMRE_MASK                (0x20000U)
#define SCG_SOSCCSR_SOSCCMRE_SHIFT               (17U)
#define SCG_SOSCCSR_SOSCCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCMRE_SHIFT)) & SCG_SOSCCSR_SOSCCMRE_MASK)
#define SCG_SOSCCSR_LK_MASK                      (0x800000U)
#define SCG_SOSCCSR_LK_SHIFT                     (23U)
#define SCG_SOSCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_LK_SHIFT)) & SCG_SOSCCSR_LK_MASK)
#define SCG_SOSCCSR_SOSCVLD_MASK                 (0x1000000U)
#define SCG_SOSCCSR_SOSCVLD_SHIFT                (24U)
#define SCG_SOSCCSR_SOSCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCVLD_SHIFT)) & SCG_SOSCCSR_SOSCVLD_MASK)
#define SCG_SOSCCSR_SOSCSEL_MASK                 (0x2000000U)
#define SCG_SOSCCSR_SOSCSEL_SHIFT                (25U)
#define SCG_SOSCCSR_SOSCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCSEL_SHIFT)) & SCG_SOSCCSR_SOSCSEL_MASK)
#define SCG_SOSCCSR_SOSCERR_MASK                 (0x4000000U)
#define SCG_SOSCCSR_SOSCERR_SHIFT                (26U)
#define SCG_SOSCCSR_SOSCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCERR_SHIFT)) & SCG_SOSCCSR_SOSCERR_MASK)

/*! @name SOSCDIV - System OSC Divide Register */
#define SCG_SOSCDIV_SOSCDIV1_MASK                (0x7U)
#define SCG_SOSCDIV_SOSCDIV1_SHIFT               (0U)
#define SCG_SOSCDIV_SOSCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCDIV_SOSCDIV1_SHIFT)) & SCG_SOSCDIV_SOSCDIV1_MASK)
#define SCG_SOSCDIV_SOSCDIV2_MASK                (0x700U)
#define SCG_SOSCDIV_SOSCDIV2_SHIFT               (8U)
#define SCG_SOSCDIV_SOSCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCDIV_SOSCDIV2_SHIFT)) & SCG_SOSCDIV_SOSCDIV2_MASK)
#define SCG_SOSCDIV_SOSCDIV3_MASK                (0x70000U)
#define SCG_SOSCDIV_SOSCDIV3_SHIFT               (16U)
#define SCG_SOSCDIV_SOSCDIV3(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCDIV_SOSCDIV3_SHIFT)) & SCG_SOSCDIV_SOSCDIV3_MASK)

/*! @name SOSCCFG - System Oscillator Configuration Register */
#define SCG_SOSCCFG_EREFS_MASK                   (0x4U)
#define SCG_SOSCCFG_EREFS_SHIFT                  (2U)
#define SCG_SOSCCFG_EREFS(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_EREFS_SHIFT)) & SCG_SOSCCFG_EREFS_MASK)
#define SCG_SOSCCFG_HGO_MASK                     (0x8U)
#define SCG_SOSCCFG_HGO_SHIFT                    (3U)
#define SCG_SOSCCFG_HGO(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_HGO_SHIFT)) & SCG_SOSCCFG_HGO_MASK)
#define SCG_SOSCCFG_RANGE_MASK                   (0x30U)
#define SCG_SOSCCFG_RANGE_SHIFT                  (4U)
#define SCG_SOSCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_RANGE_SHIFT)) & SCG_SOSCCFG_RANGE_MASK)
#define SCG_SOSCCFG_SC16P_MASK                   (0x100U)
#define SCG_SOSCCFG_SC16P_SHIFT                  (8U)
#define SCG_SOSCCFG_SC16P(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_SC16P_SHIFT)) & SCG_SOSCCFG_SC16P_MASK)
#define SCG_SOSCCFG_SC8P_MASK                    (0x200U)
#define SCG_SOSCCFG_SC8P_SHIFT                   (9U)
#define SCG_SOSCCFG_SC8P(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_SC8P_SHIFT)) & SCG_SOSCCFG_SC8P_MASK)
#define SCG_SOSCCFG_SC4P_MASK                    (0x400U)
#define SCG_SOSCCFG_SC4P_SHIFT                   (10U)
#define SCG_SOSCCFG_SC4P(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_SC4P_SHIFT)) & SCG_SOSCCFG_SC4P_MASK)
#define SCG_SOSCCFG_SC2P_MASK                    (0x800U)
#define SCG_SOSCCFG_SC2P_SHIFT                   (11U)
#define SCG_SOSCCFG_SC2P(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_SC2P_SHIFT)) & SCG_SOSCCFG_SC2P_MASK)

/*! @name SIRCCSR - Slow IRC Control Status Register */
#define SCG_SIRCCSR_SIRCEN_MASK                  (0x1U)
#define SCG_SIRCCSR_SIRCEN_SHIFT                 (0U)
#define SCG_SIRCCSR_SIRCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCEN_SHIFT)) & SCG_SIRCCSR_SIRCEN_MASK)
#define SCG_SIRCCSR_SIRCSTEN_MASK                (0x2U)
#define SCG_SIRCCSR_SIRCSTEN_SHIFT               (1U)
#define SCG_SIRCCSR_SIRCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSTEN_SHIFT)) & SCG_SIRCCSR_SIRCSTEN_MASK)
#define SCG_SIRCCSR_SIRCLPEN_MASK                (0x4U)
#define SCG_SIRCCSR_SIRCLPEN_SHIFT               (2U)
#define SCG_SIRCCSR_SIRCLPEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCLPEN_SHIFT)) & SCG_SIRCCSR_SIRCLPEN_MASK)
#define SCG_SIRCCSR_LK_MASK                      (0x800000U)
#define SCG_SIRCCSR_LK_SHIFT                     (23U)
#define SCG_SIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_LK_SHIFT)) & SCG_SIRCCSR_LK_MASK)
#define SCG_SIRCCSR_SIRCVLD_MASK                 (0x1000000U)
#define SCG_SIRCCSR_SIRCVLD_SHIFT                (24U)
#define SCG_SIRCCSR_SIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCVLD_SHIFT)) & SCG_SIRCCSR_SIRCVLD_MASK)
#define SCG_SIRCCSR_SIRCSEL_MASK                 (0x2000000U)
#define SCG_SIRCCSR_SIRCSEL_SHIFT                (25U)
#define SCG_SIRCCSR_SIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSEL_SHIFT)) & SCG_SIRCCSR_SIRCSEL_MASK)

/*! @name SIRCDIV - Slow IRC Divide Register */
#define SCG_SIRCDIV_SIRCDIV1_MASK                (0x7U)
#define SCG_SIRCDIV_SIRCDIV1_SHIFT               (0U)
#define SCG_SIRCDIV_SIRCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCDIV_SIRCDIV1_SHIFT)) & SCG_SIRCDIV_SIRCDIV1_MASK)
#define SCG_SIRCDIV_SIRCDIV2_MASK                (0x700U)
#define SCG_SIRCDIV_SIRCDIV2_SHIFT               (8U)
#define SCG_SIRCDIV_SIRCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCDIV_SIRCDIV2_SHIFT)) & SCG_SIRCDIV_SIRCDIV2_MASK)
#define SCG_SIRCDIV_SIRCDIV3_MASK                (0x70000U)
#define SCG_SIRCDIV_SIRCDIV3_SHIFT               (16U)
#define SCG_SIRCDIV_SIRCDIV3(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCDIV_SIRCDIV3_SHIFT)) & SCG_SIRCDIV_SIRCDIV3_MASK)

/*! @name SIRCCFG - Slow IRC Configuration Register */
#define SCG_SIRCCFG_RANGE_MASK                   (0x1U)
#define SCG_SIRCCFG_RANGE_SHIFT                  (0U)
#define SCG_SIRCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCFG_RANGE_SHIFT)) & SCG_SIRCCFG_RANGE_MASK)

/*! @name FIRCCSR - Fast IRC Control Status Register */
#define SCG_FIRCCSR_FIRCEN_MASK                  (0x1U)
#define SCG_FIRCCSR_FIRCEN_SHIFT                 (0U)
#define SCG_FIRCCSR_FIRCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCEN_SHIFT)) & SCG_FIRCCSR_FIRCEN_MASK)
#define SCG_FIRCCSR_FIRCSTEN_MASK                (0x2U)
#define SCG_FIRCCSR_FIRCSTEN_SHIFT               (1U)
#define SCG_FIRCCSR_FIRCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCSTEN_SHIFT)) & SCG_FIRCCSR_FIRCSTEN_MASK)
#define SCG_FIRCCSR_FIRCLPEN_MASK                (0x4U)
#define SCG_FIRCCSR_FIRCLPEN_SHIFT               (2U)
#define SCG_FIRCCSR_FIRCLPEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCLPEN_SHIFT)) & SCG_FIRCCSR_FIRCLPEN_MASK)
#define SCG_FIRCCSR_FIRCREGOFF_MASK              (0x8U)
#define SCG_FIRCCSR_FIRCREGOFF_SHIFT             (3U)
#define SCG_FIRCCSR_FIRCREGOFF(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCREGOFF_SHIFT)) & SCG_FIRCCSR_FIRCREGOFF_MASK)
#define SCG_FIRCCSR_FIRCTREN_MASK                (0x100U)
#define SCG_FIRCCSR_FIRCTREN_SHIFT               (8U)
#define SCG_FIRCCSR_FIRCTREN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCTREN_SHIFT)) & SCG_FIRCCSR_FIRCTREN_MASK)
#define SCG_FIRCCSR_FIRCTRUP_MASK                (0x200U)
#define SCG_FIRCCSR_FIRCTRUP_SHIFT               (9U)
#define SCG_FIRCCSR_FIRCTRUP(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCTRUP_SHIFT)) & SCG_FIRCCSR_FIRCTRUP_MASK)
#define SCG_FIRCCSR_LK_MASK                      (0x800000U)
#define SCG_FIRCCSR_LK_SHIFT                     (23U)
#define SCG_FIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_LK_SHIFT)) & SCG_FIRCCSR_LK_MASK)
#define SCG_FIRCCSR_FIRCVLD_MASK                 (0x1000000U)
#define SCG_FIRCCSR_FIRCVLD_SHIFT                (24U)
#define SCG_FIRCCSR_FIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCVLD_SHIFT)) & SCG_FIRCCSR_FIRCVLD_MASK)
#define SCG_FIRCCSR_FIRCSEL_MASK                 (0x2000000U)
#define SCG_FIRCCSR_FIRCSEL_SHIFT                (25U)
#define SCG_FIRCCSR_FIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCSEL_SHIFT)) & SCG_FIRCCSR_FIRCSEL_MASK)
#define SCG_FIRCCSR_FIRCERR_MASK                 (0x4000000U)
#define SCG_FIRCCSR_FIRCERR_SHIFT                (26U)
#define SCG_FIRCCSR_FIRCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCERR_SHIFT)) & SCG_FIRCCSR_FIRCERR_MASK)

/*! @name FIRCDIV - Fast IRC Divide Register */
#define SCG_FIRCDIV_FIRCDIV1_MASK                (0x7U)
#define SCG_FIRCDIV_FIRCDIV1_SHIFT               (0U)
#define SCG_FIRCDIV_FIRCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCDIV_FIRCDIV1_SHIFT)) & SCG_FIRCDIV_FIRCDIV1_MASK)
#define SCG_FIRCDIV_FIRCDIV2_MASK                (0x700U)
#define SCG_FIRCDIV_FIRCDIV2_SHIFT               (8U)
#define SCG_FIRCDIV_FIRCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCDIV_FIRCDIV2_SHIFT)) & SCG_FIRCDIV_FIRCDIV2_MASK)
#define SCG_FIRCDIV_FIRCDIV3_MASK                (0x70000U)
#define SCG_FIRCDIV_FIRCDIV3_SHIFT               (16U)
#define SCG_FIRCDIV_FIRCDIV3(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCDIV_FIRCDIV3_SHIFT)) & SCG_FIRCDIV_FIRCDIV3_MASK)

/*! @name FIRCCFG - Fast IRC Configuration Register */
#define SCG_FIRCCFG_RANGE_MASK                   (0x3U)
#define SCG_FIRCCFG_RANGE_SHIFT                  (0U)
#define SCG_FIRCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCFG_RANGE_SHIFT)) & SCG_FIRCCFG_RANGE_MASK)

/*! @name FIRCTCFG - Fast IRC Trim Configuration Register */
#define SCG_FIRCTCFG_TRIMSRC_MASK                (0x3U)
#define SCG_FIRCTCFG_TRIMSRC_SHIFT               (0U)
#define SCG_FIRCTCFG_TRIMSRC(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTCFG_TRIMSRC_SHIFT)) & SCG_FIRCTCFG_TRIMSRC_MASK)
#define SCG_FIRCTCFG_TRIMDIV_MASK                (0x700U)
#define SCG_FIRCTCFG_TRIMDIV_SHIFT               (8U)
#define SCG_FIRCTCFG_TRIMDIV(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCTCFG_TRIMDIV_SHIFT)) & SCG_FIRCTCFG_TRIMDIV_MASK)

/*! @name FIRCSTAT - Fast IRC Status Register */
#define SCG_FIRCSTAT_TRIMFINE_MASK               (0x7FU)
#define SCG_FIRCSTAT_TRIMFINE_SHIFT              (0U)
#define SCG_FIRCSTAT_TRIMFINE(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCSTAT_TRIMFINE_SHIFT)) & SCG_FIRCSTAT_TRIMFINE_MASK)
#define SCG_FIRCSTAT_TRIMCOAR_MASK               (0x3F00U)
#define SCG_FIRCSTAT_TRIMCOAR_SHIFT              (8U)
#define SCG_FIRCSTAT_TRIMCOAR(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_FIRCSTAT_TRIMCOAR_SHIFT)) & SCG_FIRCSTAT_TRIMCOAR_MASK)

/*! @name SPLLCSR - System PLL Control Status Register */
#define SCG_SPLLCSR_SPLLEN_MASK                  (0x1U)
#define SCG_SPLLCSR_SPLLEN_SHIFT                 (0U)
#define SCG_SPLLCSR_SPLLEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLEN_SHIFT)) & SCG_SPLLCSR_SPLLEN_MASK)
#define SCG_SPLLCSR_SPLLSTEN_MASK                (0x2U)
#define SCG_SPLLCSR_SPLLSTEN_SHIFT               (1U)
#define SCG_SPLLCSR_SPLLSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLSTEN_SHIFT)) & SCG_SPLLCSR_SPLLSTEN_MASK)
#define SCG_SPLLCSR_SPLLCM_MASK                  (0x10000U)
#define SCG_SPLLCSR_SPLLCM_SHIFT                 (16U)
#define SCG_SPLLCSR_SPLLCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLCM_SHIFT)) & SCG_SPLLCSR_SPLLCM_MASK)
#define SCG_SPLLCSR_SPLLCMRE_MASK                (0x20000U)
#define SCG_SPLLCSR_SPLLCMRE_SHIFT               (17U)
#define SCG_SPLLCSR_SPLLCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLCMRE_SHIFT)) & SCG_SPLLCSR_SPLLCMRE_MASK)
#define SCG_SPLLCSR_LK_MASK                      (0x800000U)
#define SCG_SPLLCSR_LK_SHIFT                     (23U)
#define SCG_SPLLCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_LK_SHIFT)) & SCG_SPLLCSR_LK_MASK)
#define SCG_SPLLCSR_SPLLVLD_MASK                 (0x1000000U)
#define SCG_SPLLCSR_SPLLVLD_SHIFT                (24U)
#define SCG_SPLLCSR_SPLLVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLVLD_SHIFT)) & SCG_SPLLCSR_SPLLVLD_MASK)
#define SCG_SPLLCSR_SPLLSEL_MASK                 (0x2000000U)
#define SCG_SPLLCSR_SPLLSEL_SHIFT                (25U)
#define SCG_SPLLCSR_SPLLSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLSEL_SHIFT)) & SCG_SPLLCSR_SPLLSEL_MASK)
#define SCG_SPLLCSR_SPLLERR_MASK                 (0x4000000U)
#define SCG_SPLLCSR_SPLLERR_SHIFT                (26U)
#define SCG_SPLLCSR_SPLLERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLERR_SHIFT)) & SCG_SPLLCSR_SPLLERR_MASK)

/*! @name SPLLDIV - System PLL Divide Register */
#define SCG_SPLLDIV_SPLLDIV1_MASK                (0x7U)
#define SCG_SPLLDIV_SPLLDIV1_SHIFT               (0U)
#define SCG_SPLLDIV_SPLLDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLDIV_SPLLDIV1_SHIFT)) & SCG_SPLLDIV_SPLLDIV1_MASK)
#define SCG_SPLLDIV_SPLLDIV2_MASK                (0x700U)
#define SCG_SPLLDIV_SPLLDIV2_SHIFT               (8U)
#define SCG_SPLLDIV_SPLLDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLDIV_SPLLDIV2_SHIFT)) & SCG_SPLLDIV_SPLLDIV2_MASK)
#define SCG_SPLLDIV_SPLLDIV3_MASK                (0x70000U)
#define SCG_SPLLDIV_SPLLDIV3_SHIFT               (16U)
#define SCG_SPLLDIV_SPLLDIV3(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLDIV_SPLLDIV3_SHIFT)) & SCG_SPLLDIV_SPLLDIV3_MASK)

/*! @name SPLLCFG - System PLL Configuration Register */
#define SCG_SPLLCFG_SOURCE_MASK                  (0x1U)
#define SCG_SPLLCFG_SOURCE_SHIFT                 (0U)
#define SCG_SPLLCFG_SOURCE(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_SOURCE_SHIFT)) & SCG_SPLLCFG_SOURCE_MASK)
#define SCG_SPLLCFG_PREDIV_MASK                  (0x700U)
#define SCG_SPLLCFG_PREDIV_SHIFT                 (8U)
#define SCG_SPLLCFG_PREDIV(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_PREDIV_SHIFT)) & SCG_SPLLCFG_PREDIV_MASK)
#define SCG_SPLLCFG_MULT_MASK                    (0x1F0000U)
#define SCG_SPLLCFG_MULT_SHIFT                   (16U)
#define SCG_SPLLCFG_MULT(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_MULT_SHIFT)) & SCG_SPLLCFG_MULT_MASK)


/*!
 * @}
 */ /* end of group SCG_Register_Masks */


/* SCG - Peripheral instance base addresses */
/** Peripheral SCG base address */
#define SCG_BASE                                 (0x4007B000u)
/** Peripheral SCG base pointer */
#define SCG                                      ((SCG_Type *)SCG_BASE)
/** Array initializer of SCG peripheral base addresses */
#define SCG_BASE_ADDRS                           { SCG_BASE }
/** Array initializer of SCG peripheral base pointers */
#define SCG_BASE_PTRS                            { SCG }
/** Interrupt vectors for the SCG peripheral type */
#define SCG_IRQS                                 { SCG_IRQn }

/*!
 * @}
 */ /* end of group SCG_Peripheral_Access_Layer */


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
       uint8_t RESERVED_0[4124];
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x1024 */
       uint8_t RESERVED_1[36];
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x104C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x1050 */
       uint8_t RESERVED_2[4];
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x1058 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x105C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x1060 */
       uint8_t RESERVED_3[136];
  __I  uint32_t PCSR;                              /**< Peripheral Clock Status Register, offset: 0x10EC */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name SOPT1 - System Options Register 1 */
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

/*! @name SDID - System Device Identification Register */
#define SIM_SDID_PINID_MASK                      (0xFU)
#define SIM_SDID_PINID_SHIFT                     (0U)
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PINID_SHIFT)) & SIM_SDID_PINID_MASK)
#define SIM_SDID_KEYATT_MASK                     (0x70U)
#define SIM_SDID_KEYATT_SHIFT                    (4U)
#define SIM_SDID_KEYATT(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SDID_KEYATT_SHIFT)) & SIM_SDID_KEYATT_MASK)
#define SIM_SDID_DIEID_MASK                      (0xF80U)
#define SIM_SDID_DIEID_SHIFT                     (7U)
#define SIM_SDID_DIEID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_DIEID_SHIFT)) & SIM_SDID_DIEID_MASK)
#define SIM_SDID_REVID_MASK                      (0xF000U)
#define SIM_SDID_REVID_SHIFT                     (12U)
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_REVID_SHIFT)) & SIM_SDID_REVID_MASK)
#define SIM_SDID_SRAMSIZE_MASK                   (0xF0000U)
#define SIM_SDID_SRAMSIZE_SHIFT                  (16U)
#define SIM_SDID_SRAMSIZE(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SRAMSIZE_SHIFT)) & SIM_SDID_SRAMSIZE_MASK)
#define SIM_SDID_SERIESID_MASK                   (0xF00000U)
#define SIM_SDID_SERIESID_SHIFT                  (20U)
#define SIM_SDID_SERIESID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SERIESID_SHIFT)) & SIM_SDID_SERIESID_MASK)
#define SIM_SDID_SUBFAMID_MASK                   (0xF000000U)
#define SIM_SDID_SUBFAMID_SHIFT                  (24U)
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SUBFAMID_SHIFT)) & SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMID_MASK                      (0xF0000000U)
#define SIM_SDID_FAMID_SHIFT                     (28U)
#define SIM_SDID_FAMID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FAMID_SHIFT)) & SIM_SDID_FAMID_MASK)

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
#define SIM_FCFG2_MAXADDR0_MASK                  (0x7F000000U)
#define SIM_FCFG2_MAXADDR0_SHIFT                 (24U)
#define SIM_FCFG2_MAXADDR0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG2_MAXADDR0_SHIFT)) & SIM_FCFG2_MAXADDR0_MASK)

/*! @name UIDMH - Unique Identification Register Mid-High */
#define SIM_UIDMH_UID_MASK                       (0xFFFFU)
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

/*! @name PCSR - Peripheral Clock Status Register */
#define SIM_PCSR_CS1_MASK                        (0x2U)
#define SIM_PCSR_CS1_SHIFT                       (1U)
#define SIM_PCSR_CS1(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS1_SHIFT)) & SIM_PCSR_CS1_MASK)
#define SIM_PCSR_CS2_MASK                        (0x4U)
#define SIM_PCSR_CS2_SHIFT                       (2U)
#define SIM_PCSR_CS2(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS2_SHIFT)) & SIM_PCSR_CS2_MASK)
#define SIM_PCSR_CS3_MASK                        (0x8U)
#define SIM_PCSR_CS3_SHIFT                       (3U)
#define SIM_PCSR_CS3(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS3_SHIFT)) & SIM_PCSR_CS3_MASK)
#define SIM_PCSR_CS4_MASK                        (0x10U)
#define SIM_PCSR_CS4_SHIFT                       (4U)
#define SIM_PCSR_CS4(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS4_SHIFT)) & SIM_PCSR_CS4_MASK)
#define SIM_PCSR_CS5_MASK                        (0x20U)
#define SIM_PCSR_CS5_SHIFT                       (5U)
#define SIM_PCSR_CS5(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS5_SHIFT)) & SIM_PCSR_CS5_MASK)
#define SIM_PCSR_CS6_MASK                        (0x40U)
#define SIM_PCSR_CS6_SHIFT                       (6U)
#define SIM_PCSR_CS6(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS6_SHIFT)) & SIM_PCSR_CS6_MASK)
#define SIM_PCSR_CS7_MASK                        (0x80U)
#define SIM_PCSR_CS7_SHIFT                       (7U)
#define SIM_PCSR_CS7(x)                          (((uint32_t)(((uint32_t)(x)) << SIM_PCSR_CS7_SHIFT)) & SIM_PCSR_CS7_MASK)


/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40074000u)
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
  __I  uint32_t VERID;                             /**< SMC Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< SMC Parameter Register, offset: 0x4 */
  __IO uint32_t PMPROT;                            /**< Power Mode Protection register, offset: 0x8 */
  __IO uint32_t PMCTRL;                            /**< Power Mode Control register, offset: 0xC */
  __IO uint32_t STOPCTRL;                          /**< Stop Control Register, offset: 0x10 */
  __I  uint32_t PMSTAT;                            /**< Power Mode Status register, offset: 0x14 */
} SMC_Type;

/* ----------------------------------------------------------------------------
   -- SMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Masks SMC Register Masks
 * @{
 */

/*! @name VERID - SMC Version ID Register */
#define SMC_VERID_FEATURE_MASK                   (0xFFFFU)
#define SMC_VERID_FEATURE_SHIFT                  (0U)
#define SMC_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_VERID_FEATURE_SHIFT)) & SMC_VERID_FEATURE_MASK)
#define SMC_VERID_MINOR_MASK                     (0xFF0000U)
#define SMC_VERID_MINOR_SHIFT                    (16U)
#define SMC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_VERID_MINOR_SHIFT)) & SMC_VERID_MINOR_MASK)
#define SMC_VERID_MAJOR_MASK                     (0xFF000000U)
#define SMC_VERID_MAJOR_SHIFT                    (24U)
#define SMC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_VERID_MAJOR_SHIFT)) & SMC_VERID_MAJOR_MASK)

/*! @name PARAM - SMC Parameter Register */
#define SMC_PARAM_EHSRUN_MASK                    (0x1U)
#define SMC_PARAM_EHSRUN_SHIFT                   (0U)
#define SMC_PARAM_EHSRUN(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_EHSRUN_SHIFT)) & SMC_PARAM_EHSRUN_MASK)
#define SMC_PARAM_ELLS_MASK                      (0x8U)
#define SMC_PARAM_ELLS_SHIFT                     (3U)
#define SMC_PARAM_ELLS(x)                        (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_ELLS_SHIFT)) & SMC_PARAM_ELLS_MASK)
#define SMC_PARAM_ELLS2_MASK                     (0x20U)
#define SMC_PARAM_ELLS2_SHIFT                    (5U)
#define SMC_PARAM_ELLS2(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_ELLS2_SHIFT)) & SMC_PARAM_ELLS2_MASK)
#define SMC_PARAM_EVLLS0_MASK                    (0x40U)
#define SMC_PARAM_EVLLS0_SHIFT                   (6U)
#define SMC_PARAM_EVLLS0(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_EVLLS0_SHIFT)) & SMC_PARAM_EVLLS0_MASK)

/*! @name PMPROT - Power Mode Protection register */
#define SMC_PMPROT_AVLLS_MASK                    (0x2U)
#define SMC_PMPROT_AVLLS_SHIFT                   (1U)
#define SMC_PMPROT_AVLLS(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_AVLLS_SHIFT)) & SMC_PMPROT_AVLLS_MASK)
#define SMC_PMPROT_ALLS_MASK                     (0x8U)
#define SMC_PMPROT_ALLS_SHIFT                    (3U)
#define SMC_PMPROT_ALLS(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_ALLS_SHIFT)) & SMC_PMPROT_ALLS_MASK)
#define SMC_PMPROT_AVLP_MASK                     (0x20U)
#define SMC_PMPROT_AVLP_SHIFT                    (5U)
#define SMC_PMPROT_AVLP(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_AVLP_SHIFT)) & SMC_PMPROT_AVLP_MASK)
#define SMC_PMPROT_AHSRUN_MASK                   (0x80U)
#define SMC_PMPROT_AHSRUN_SHIFT                  (7U)
#define SMC_PMPROT_AHSRUN(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_AHSRUN_SHIFT)) & SMC_PMPROT_AHSRUN_MASK)

/*! @name PMCTRL - Power Mode Control register */
#define SMC_PMCTRL_STOPM_MASK                    (0x7U)
#define SMC_PMCTRL_STOPM_SHIFT                   (0U)
#define SMC_PMCTRL_STOPM(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_STOPM_SHIFT)) & SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_STOPA_MASK                    (0x8U)
#define SMC_PMCTRL_STOPA_SHIFT                   (3U)
#define SMC_PMCTRL_STOPA(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_STOPA_SHIFT)) & SMC_PMCTRL_STOPA_MASK)
#define SMC_PMCTRL_RUNM_MASK                     (0x60U)
#define SMC_PMCTRL_RUNM_SHIFT                    (5U)
#define SMC_PMCTRL_RUNM(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_RUNM_SHIFT)) & SMC_PMCTRL_RUNM_MASK)

/*! @name STOPCTRL - Stop Control Register */
#define SMC_STOPCTRL_LLSM_MASK                   (0x7U)
#define SMC_STOPCTRL_LLSM_SHIFT                  (0U)
#define SMC_STOPCTRL_LLSM(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_STOPCTRL_LLSM_SHIFT)) & SMC_STOPCTRL_LLSM_MASK)
#define SMC_STOPCTRL_LPOPO_MASK                  (0x8U)
#define SMC_STOPCTRL_LPOPO_SHIFT                 (3U)
#define SMC_STOPCTRL_LPOPO(x)                    (((uint32_t)(((uint32_t)(x)) << SMC_STOPCTRL_LPOPO_SHIFT)) & SMC_STOPCTRL_LPOPO_MASK)
#define SMC_STOPCTRL_PORPO_MASK                  (0x20U)
#define SMC_STOPCTRL_PORPO_SHIFT                 (5U)
#define SMC_STOPCTRL_PORPO(x)                    (((uint32_t)(((uint32_t)(x)) << SMC_STOPCTRL_PORPO_SHIFT)) & SMC_STOPCTRL_PORPO_MASK)
#define SMC_STOPCTRL_PSTOPO_MASK                 (0xC0U)
#define SMC_STOPCTRL_PSTOPO_SHIFT                (6U)
#define SMC_STOPCTRL_PSTOPO(x)                   (((uint32_t)(((uint32_t)(x)) << SMC_STOPCTRL_PSTOPO_SHIFT)) & SMC_STOPCTRL_PSTOPO_MASK)

/*! @name PMSTAT - Power Mode Status register */
#define SMC_PMSTAT_PMSTAT_MASK                   (0xFFU)
#define SMC_PMSTAT_PMSTAT_SHIFT                  (0U)
#define SMC_PMSTAT_PMSTAT(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_PMSTAT_PMSTAT_SHIFT)) & SMC_PMSTAT_PMSTAT_MASK)


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
   -- TPM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TPM_Peripheral_Access_Layer TPM Peripheral Access Layer
 * @{
 */

/** TPM - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t GLOBAL;                            /**< TPM Global Register, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t SC;                                /**< Status and Control, offset: 0x10 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x14 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x18 */
  __IO uint32_t STATUS;                            /**< Capture and Compare Status, offset: 0x1C */
  struct {                                         /* offset: 0x20, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status and Control, array offset: 0x20, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x24, array step: 0x8 */
  } CONTROLS[6];
       uint8_t RESERVED_1[20];
  __IO uint32_t COMBINE;                           /**< Combine Channel Register, offset: 0x64 */
       uint8_t RESERVED_2[4];
  __IO uint32_t TRIG;                              /**< Channel Trigger, offset: 0x6C */
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

/*! @name VERID - Version ID Register */
#define TPM_VERID_FEATURE_MASK                   (0xFFFFU)
#define TPM_VERID_FEATURE_SHIFT                  (0U)
#define TPM_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << TPM_VERID_FEATURE_SHIFT)) & TPM_VERID_FEATURE_MASK)
#define TPM_VERID_MINOR_MASK                     (0xFF0000U)
#define TPM_VERID_MINOR_SHIFT                    (16U)
#define TPM_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_VERID_MINOR_SHIFT)) & TPM_VERID_MINOR_MASK)
#define TPM_VERID_MAJOR_MASK                     (0xFF000000U)
#define TPM_VERID_MAJOR_SHIFT                    (24U)
#define TPM_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_VERID_MAJOR_SHIFT)) & TPM_VERID_MAJOR_MASK)

/*! @name PARAM - Parameter Register */
#define TPM_PARAM_CHAN_MASK                      (0xFFU)
#define TPM_PARAM_CHAN_SHIFT                     (0U)
#define TPM_PARAM_CHAN(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_PARAM_CHAN_SHIFT)) & TPM_PARAM_CHAN_MASK)
#define TPM_PARAM_TRIG_MASK                      (0xFF00U)
#define TPM_PARAM_TRIG_SHIFT                     (8U)
#define TPM_PARAM_TRIG(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_PARAM_TRIG_SHIFT)) & TPM_PARAM_TRIG_MASK)
#define TPM_PARAM_WIDTH_MASK                     (0xFF0000U)
#define TPM_PARAM_WIDTH_SHIFT                    (16U)
#define TPM_PARAM_WIDTH(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_PARAM_WIDTH_SHIFT)) & TPM_PARAM_WIDTH_MASK)

/*! @name GLOBAL - TPM Global Register */
#define TPM_GLOBAL_RST_MASK                      (0x2U)
#define TPM_GLOBAL_RST_SHIFT                     (1U)
#define TPM_GLOBAL_RST(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_GLOBAL_RST_SHIFT)) & TPM_GLOBAL_RST_MASK)

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

/*! @name STATUS - Capture and Compare Status */
#define TPM_STATUS_CH0F_MASK                     (0x1U)
#define TPM_STATUS_CH0F_SHIFT                    (0U)
#define TPM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH0F_SHIFT)) & TPM_STATUS_CH0F_MASK)
#define TPM_STATUS_CH1F_MASK                     (0x2U)
#define TPM_STATUS_CH1F_SHIFT                    (1U)
#define TPM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH1F_SHIFT)) & TPM_STATUS_CH1F_MASK)
#define TPM_STATUS_CH2F_MASK                     (0x4U)
#define TPM_STATUS_CH2F_SHIFT                    (2U)
#define TPM_STATUS_CH2F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH2F_SHIFT)) & TPM_STATUS_CH2F_MASK)
#define TPM_STATUS_CH3F_MASK                     (0x8U)
#define TPM_STATUS_CH3F_SHIFT                    (3U)
#define TPM_STATUS_CH3F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH3F_SHIFT)) & TPM_STATUS_CH3F_MASK)
#define TPM_STATUS_CH4F_MASK                     (0x10U)
#define TPM_STATUS_CH4F_SHIFT                    (4U)
#define TPM_STATUS_CH4F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH4F_SHIFT)) & TPM_STATUS_CH4F_MASK)
#define TPM_STATUS_CH5F_MASK                     (0x20U)
#define TPM_STATUS_CH5F_SHIFT                    (5U)
#define TPM_STATUS_CH5F(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_CH5F_SHIFT)) & TPM_STATUS_CH5F_MASK)
#define TPM_STATUS_TOF_MASK                      (0x100U)
#define TPM_STATUS_TOF_SHIFT                     (8U)
#define TPM_STATUS_TOF(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_STATUS_TOF_SHIFT)) & TPM_STATUS_TOF_MASK)

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
#define TPM_CnSC_COUNT                           (6U)

/*! @name CnV - Channel (n) Value */
#define TPM_CnV_VAL_MASK                         (0xFFFFU)
#define TPM_CnV_VAL_SHIFT                        (0U)
#define TPM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_CnV_VAL_SHIFT)) & TPM_CnV_VAL_MASK)

/* The count of TPM_CnV */
#define TPM_CnV_COUNT                            (6U)

/*! @name COMBINE - Combine Channel Register */
#define TPM_COMBINE_COMBINE0_MASK                (0x1U)
#define TPM_COMBINE_COMBINE0_SHIFT               (0U)
#define TPM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMBINE0_SHIFT)) & TPM_COMBINE_COMBINE0_MASK)
#define TPM_COMBINE_COMSWAP0_MASK                (0x2U)
#define TPM_COMBINE_COMSWAP0_SHIFT               (1U)
#define TPM_COMBINE_COMSWAP0(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMSWAP0_SHIFT)) & TPM_COMBINE_COMSWAP0_MASK)
#define TPM_COMBINE_COMBINE1_MASK                (0x100U)
#define TPM_COMBINE_COMBINE1_SHIFT               (8U)
#define TPM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMBINE1_SHIFT)) & TPM_COMBINE_COMBINE1_MASK)
#define TPM_COMBINE_COMSWAP1_MASK                (0x200U)
#define TPM_COMBINE_COMSWAP1_SHIFT               (9U)
#define TPM_COMBINE_COMSWAP1(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMSWAP1_SHIFT)) & TPM_COMBINE_COMSWAP1_MASK)
#define TPM_COMBINE_COMBINE2_MASK                (0x10000U)
#define TPM_COMBINE_COMBINE2_SHIFT               (16U)
#define TPM_COMBINE_COMBINE2(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMBINE2_SHIFT)) & TPM_COMBINE_COMBINE2_MASK)
#define TPM_COMBINE_COMSWAP2_MASK                (0x20000U)
#define TPM_COMBINE_COMSWAP2_SHIFT               (17U)
#define TPM_COMBINE_COMSWAP2(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMSWAP2_SHIFT)) & TPM_COMBINE_COMSWAP2_MASK)

/*! @name TRIG - Channel Trigger */
#define TPM_TRIG_TRIG0_MASK                      (0x1U)
#define TPM_TRIG_TRIG0_SHIFT                     (0U)
#define TPM_TRIG_TRIG0(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG0_SHIFT)) & TPM_TRIG_TRIG0_MASK)
#define TPM_TRIG_TRIG1_MASK                      (0x2U)
#define TPM_TRIG_TRIG1_SHIFT                     (1U)
#define TPM_TRIG_TRIG1(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG1_SHIFT)) & TPM_TRIG_TRIG1_MASK)
#define TPM_TRIG_TRIG2_MASK                      (0x4U)
#define TPM_TRIG_TRIG2_SHIFT                     (2U)
#define TPM_TRIG_TRIG2(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG2_SHIFT)) & TPM_TRIG_TRIG2_MASK)
#define TPM_TRIG_TRIG3_MASK                      (0x8U)
#define TPM_TRIG_TRIG3_SHIFT                     (3U)
#define TPM_TRIG_TRIG3(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG3_SHIFT)) & TPM_TRIG_TRIG3_MASK)
#define TPM_TRIG_TRIG4_MASK                      (0x10U)
#define TPM_TRIG_TRIG4_SHIFT                     (4U)
#define TPM_TRIG_TRIG4(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG4_SHIFT)) & TPM_TRIG_TRIG4_MASK)
#define TPM_TRIG_TRIG5_MASK                      (0x20U)
#define TPM_TRIG_TRIG5_SHIFT                     (5U)
#define TPM_TRIG_TRIG5(x)                        (((uint32_t)(((uint32_t)(x)) << TPM_TRIG_TRIG5_SHIFT)) & TPM_TRIG_TRIG5_MASK)

/*! @name POL - Channel Polarity */
#define TPM_POL_POL0_MASK                        (0x1U)
#define TPM_POL_POL0_SHIFT                       (0U)
#define TPM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL0_SHIFT)) & TPM_POL_POL0_MASK)
#define TPM_POL_POL1_MASK                        (0x2U)
#define TPM_POL_POL1_SHIFT                       (1U)
#define TPM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL1_SHIFT)) & TPM_POL_POL1_MASK)
#define TPM_POL_POL2_MASK                        (0x4U)
#define TPM_POL_POL2_SHIFT                       (2U)
#define TPM_POL_POL2(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL2_SHIFT)) & TPM_POL_POL2_MASK)
#define TPM_POL_POL3_MASK                        (0x8U)
#define TPM_POL_POL3_SHIFT                       (3U)
#define TPM_POL_POL3(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL3_SHIFT)) & TPM_POL_POL3_MASK)
#define TPM_POL_POL4_MASK                        (0x10U)
#define TPM_POL_POL4_SHIFT                       (4U)
#define TPM_POL_POL4(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL4_SHIFT)) & TPM_POL_POL4_MASK)
#define TPM_POL_POL5_MASK                        (0x20U)
#define TPM_POL_POL5_SHIFT                       (5U)
#define TPM_POL_POL5(x)                          (((uint32_t)(((uint32_t)(x)) << TPM_POL_POL5_SHIFT)) & TPM_POL_POL5_MASK)

/*! @name FILTER - Filter Control */
#define TPM_FILTER_CH0FVAL_MASK                  (0xFU)
#define TPM_FILTER_CH0FVAL_SHIFT                 (0U)
#define TPM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH0FVAL_SHIFT)) & TPM_FILTER_CH0FVAL_MASK)
#define TPM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define TPM_FILTER_CH1FVAL_SHIFT                 (4U)
#define TPM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH1FVAL_SHIFT)) & TPM_FILTER_CH1FVAL_MASK)
#define TPM_FILTER_CH2FVAL_MASK                  (0xF00U)
#define TPM_FILTER_CH2FVAL_SHIFT                 (8U)
#define TPM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH2FVAL_SHIFT)) & TPM_FILTER_CH2FVAL_MASK)
#define TPM_FILTER_CH3FVAL_MASK                  (0xF000U)
#define TPM_FILTER_CH3FVAL_SHIFT                 (12U)
#define TPM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH3FVAL_SHIFT)) & TPM_FILTER_CH3FVAL_MASK)
#define TPM_FILTER_CH4FVAL_MASK                  (0xF0000U)
#define TPM_FILTER_CH4FVAL_SHIFT                 (16U)
#define TPM_FILTER_CH4FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH4FVAL_SHIFT)) & TPM_FILTER_CH4FVAL_MASK)
#define TPM_FILTER_CH5FVAL_MASK                  (0xF00000U)
#define TPM_FILTER_CH5FVAL_SHIFT                 (20U)
#define TPM_FILTER_CH5FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << TPM_FILTER_CH5FVAL_SHIFT)) & TPM_FILTER_CH5FVAL_MASK)

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
#define TPM_CONF_TRGSEL_MASK                     (0x3000000U)
#define TPM_CONF_TRGSEL_SHIFT                    (24U)
#define TPM_CONF_TRGSEL(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_TRGSEL_SHIFT)) & TPM_CONF_TRGSEL_MASK)


/*!
 * @}
 */ /* end of group TPM_Register_Masks */


/* TPM - Peripheral instance base addresses */
/** Peripheral TPM0 base address */
#define TPM0_BASE                                (0x400AC000u)
/** Peripheral TPM0 base pointer */
#define TPM0                                     ((TPM_Type *)TPM0_BASE)
/** Peripheral TPM1 base address */
#define TPM1_BASE                                (0x400AD000u)
/** Peripheral TPM1 base pointer */
#define TPM1                                     ((TPM_Type *)TPM1_BASE)
/** Peripheral TPM2 base address */
#define TPM2_BASE                                (0x4002E000u)
/** Peripheral TPM2 base pointer */
#define TPM2                                     ((TPM_Type *)TPM2_BASE)
/** Array initializer of TPM peripheral base addresses */
#define TPM_BASE_ADDRS                           { TPM0_BASE, TPM1_BASE, TPM2_BASE }
/** Array initializer of TPM peripheral base pointers */
#define TPM_BASE_PTRS                            { TPM0, TPM1, TPM2 }
/** Interrupt vectors for the TPM peripheral type */
#define TPM_IRQS                                 { TPM0_IRQn, TPM1_IRQn, TPM2_IRQn }

/*!
 * @}
 */ /* end of group TPM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TRGMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Peripheral_Access_Layer TRGMUX Peripheral Access Layer
 * @{
 */

/** TRGMUX - Register Layout Typedef */
typedef struct {
  __IO uint32_t TRGCFG[14];                        /**< TRGMUX TRGCFG Register, array offset: 0x0, array step: 0x4 */
} TRGMUX_Type;

/* ----------------------------------------------------------------------------
   -- TRGMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Register_Masks TRGMUX Register Masks
 * @{
 */

/*! @name TRGCFG - TRGMUX TRGCFG Register */
#define TRGMUX_TRGCFG_SEL0_MASK                  (0x3FU)
#define TRGMUX_TRGCFG_SEL0_SHIFT                 (0U)
#define TRGMUX_TRGCFG_SEL0(x)                    (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGCFG_SEL0_SHIFT)) & TRGMUX_TRGCFG_SEL0_MASK)
#define TRGMUX_TRGCFG_SEL1_MASK                  (0x3F00U)
#define TRGMUX_TRGCFG_SEL1_SHIFT                 (8U)
#define TRGMUX_TRGCFG_SEL1(x)                    (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGCFG_SEL1_SHIFT)) & TRGMUX_TRGCFG_SEL1_MASK)
#define TRGMUX_TRGCFG_SEL2_MASK                  (0x3F0000U)
#define TRGMUX_TRGCFG_SEL2_SHIFT                 (16U)
#define TRGMUX_TRGCFG_SEL2(x)                    (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGCFG_SEL2_SHIFT)) & TRGMUX_TRGCFG_SEL2_MASK)
#define TRGMUX_TRGCFG_SEL3_MASK                  (0x3F000000U)
#define TRGMUX_TRGCFG_SEL3_SHIFT                 (24U)
#define TRGMUX_TRGCFG_SEL3(x)                    (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGCFG_SEL3_SHIFT)) & TRGMUX_TRGCFG_SEL3_MASK)
#define TRGMUX_TRGCFG_LK_MASK                    (0x80000000U)
#define TRGMUX_TRGCFG_LK_SHIFT                   (31U)
#define TRGMUX_TRGCFG_LK(x)                      (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGCFG_LK_SHIFT)) & TRGMUX_TRGCFG_LK_MASK)

/* The count of TRGMUX_TRGCFG */
#define TRGMUX_TRGCFG_COUNT                      (14U)


/*!
 * @}
 */ /* end of group TRGMUX_Register_Masks */


/* TRGMUX - Peripheral instance base addresses */
/** Peripheral TRGMUX0 base address */
#define TRGMUX0_BASE                             (0x40027000u)
/** Peripheral TRGMUX0 base pointer */
#define TRGMUX0                                  ((TRGMUX_Type *)TRGMUX0_BASE)
/** Peripheral TRGMUX1 base address */
#define TRGMUX1_BASE                             (0x400A7000u)
/** Peripheral TRGMUX1 base pointer */
#define TRGMUX1                                  ((TRGMUX_Type *)TRGMUX1_BASE)
/** Array initializer of TRGMUX peripheral base addresses */
#define TRGMUX_BASE_ADDRS                        { TRGMUX0_BASE, TRGMUX1_BASE }
/** Array initializer of TRGMUX peripheral base pointers */
#define TRGMUX_BASE_PTRS                         { TRGMUX0, TRGMUX1 }
#define TRGMUX_INSTANCE_MASK      0xF
#define TRGMUX_INSTANCE_SHIFT     12
#define TRGMUX_PERIPHERAL_MASK    0xFFF
#define TRGMUX_PERIPHERAL_SHIFT   0
#define TRGMUX_INSTANCE_0    0
#define TRGMUX_INSTANCE_1    1

#define TRGMUX_DMAMUX0_INDEX   ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 0)
#define TRGMUX_LPIT0_INDEX     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 1)
#define TRGMUX_TPM2_INDEX      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 2)
#define TRGMUX_ADC0_INDEX      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 4)
#define TRGMUX_LPUART2_INDEX   ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 5)
#define TRGMUX_LPI2C2_INDEX    ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 7)
#define TRGMUX_LPSPI2_INDEX    ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 9)
#define TRGMUX_CMP0_INDEX      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 11)
#define TRGMUX_CMP1_INDEX      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 12)
#define TRGMUX_DAC0_INDEX      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 13)
#define TRGMUX_DMAMUX0         (TRGMUX0->TRGCFG[0])
#define TRGMUX_LPIT0           (TRGMUX0->TRGCFG[1])
#define TRGMUX_TPM2            (TRGMUX0->TRGCFG[2])
#define TRGMUX_ADC0            (TRGMUX0->TRGCFG[4])
#define TRGMUX_LPUART2         (TRGMUX0->TRGCFG[5])
#define TRGMUX_LPI2C2          (TRGMUX0->TRGCFG[7])
#define TRGMUX_LPSPI2          (TRGMUX0->TRGCFG[9])
#define TRGMUX_CMP0            (TRGMUX0->TRGCFG[11])
#define TRGMUX_CMP1            (TRGMUX0->TRGCFG[12])
#define TRGMUX_DAC0            (TRGMUX0->TRGCFG[13])

#define TRGMUX_TPM0_INDEX      ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 2)
#define TRGMUX_TPM1_INDEX      ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 3)
#define TRGMUX_FLEXIO_INDEX    ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 4)
#define TRGMUX_LPUART0_INDEX   ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 5)
#define TRGMUX_LPUART1_INDEX   ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 6)
#define TRGMUX_LPI2C0_INDEX    ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 7)
#define TRGMUX_LPI2C1_INDEX    ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 8)
#define TRGMUX_LPSPI0_INDEX    ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 9)
#define TRGMUX_LPSPI1_INDEX    ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 10)
#define TRGMUX_TPM0            (TRGMUX1->TRGCFG[2])
#define TRGMUX_TPM1            (TRGMUX1->TRGCFG[3])
#define TRGMUX_FLEXIO          (TRGMUX1->TRGCFG[4])
#define TRGMUX_LPUART0         (TRGMUX1->TRGCFG[5])
#define TRGMUX_LPUART1         (TRGMUX1->TRGCFG[6])
#define TRGMUX_LPI2C0          (TRGMUX1->TRGCFG[7])
#define TRGMUX_LPI2C1          (TRGMUX1->TRGCFG[8])
#define TRGMUX_LPSPI0          (TRGMUX1->TRGCFG[9])
#define TRGMUX_LPSPI1          (TRGMUX1->TRGCFG[10])


/*!
 * @}
 */ /* end of group TRGMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TRNG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRNG_Peripheral_Access_Layer TRNG Peripheral Access Layer
 * @{
 */

/** TRNG - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCTL;                              /**< TRNG Miscellaneous Control Register, offset: 0x0 */
  __IO uint32_t SCMISC;                            /**< TRNG Statistical Check Miscellaneous Register, offset: 0x4 */
  __IO uint32_t PKRRNG;                            /**< TRNG Poker Range Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t PKRMAX;                            /**< TRNG Poker Maximum Limit Register, offset: 0xC */
    __I  uint32_t PKRSQ;                             /**< TRNG Poker Square Calculation Result Register, offset: 0xC */
  };
  __IO uint32_t SDCTL;                             /**< TRNG Seed Control Register, offset: 0x10 */
  union {                                          /* offset: 0x14 */
    __IO uint32_t SBLIM;                             /**< TRNG Sparse Bit Limit Register, offset: 0x14 */
    __I  uint32_t TOTSAM;                            /**< TRNG Total Samples Register, offset: 0x14 */
  };
  __IO uint32_t FRQMIN;                            /**< TRNG Frequency Count Minimum Limit Register, offset: 0x18 */
  union {                                          /* offset: 0x1C */
    __I  uint32_t FRQCNT;                            /**< TRNG Frequency Count Register, offset: 0x1C */
    __IO uint32_t FRQMAX;                            /**< TRNG Frequency Count Maximum Limit Register, offset: 0x1C */
  };
  union {                                          /* offset: 0x20 */
    __I  uint32_t SCMC;                              /**< TRNG Statistical Check Monobit Count Register, offset: 0x20 */
    __IO uint32_t SCML;                              /**< TRNG Statistical Check Monobit Limit Register, offset: 0x20 */
  };
  union {                                          /* offset: 0x24 */
    __I  uint32_t SCR1C;                             /**< TRNG Statistical Check Run Length 1 Count Register, offset: 0x24 */
    __IO uint32_t SCR1L;                             /**< TRNG Statistical Check Run Length 1 Limit Register, offset: 0x24 */
  };
  union {                                          /* offset: 0x28 */
    __I  uint32_t SCR2C;                             /**< TRNG Statistical Check Run Length 2 Count Register, offset: 0x28 */
    __IO uint32_t SCR2L;                             /**< TRNG Statistical Check Run Length 2 Limit Register, offset: 0x28 */
  };
  union {                                          /* offset: 0x2C */
    __I  uint32_t SCR3C;                             /**< TRNG Statistical Check Run Length 3 Count Register, offset: 0x2C */
    __IO uint32_t SCR3L;                             /**< TRNG Statistical Check Run Length 3 Limit Register, offset: 0x2C */
  };
  union {                                          /* offset: 0x30 */
    __I  uint32_t SCR4C;                             /**< TRNG Statistical Check Run Length 4 Count Register, offset: 0x30 */
    __IO uint32_t SCR4L;                             /**< TRNG Statistical Check Run Length 4 Limit Register, offset: 0x30 */
  };
  union {                                          /* offset: 0x34 */
    __I  uint32_t SCR5C;                             /**< TRNG Statistical Check Run Length 5 Count Register, offset: 0x34 */
    __IO uint32_t SCR5L;                             /**< TRNG Statistical Check Run Length 5 Limit Register, offset: 0x34 */
  };
  union {                                          /* offset: 0x38 */
    __I  uint32_t SCR6PC;                            /**< TRNG Statistical Check Run Length 6+ Count Register, offset: 0x38 */
    __IO uint32_t SCR6PL;                            /**< TRNG Statistical Check Run Length 6+ Limit Register, offset: 0x38 */
  };
  __I  uint32_t STATUS;                            /**< TRNG Status Register, offset: 0x3C */
  __I  uint32_t ENT[16];                           /**< TRNG Entropy Read Register, array offset: 0x40, array step: 0x4 */
  __I  uint32_t PKRCNT10;                          /**< TRNG Statistical Check Poker Count 1 and 0 Register, offset: 0x80 */
  __I  uint32_t PKRCNT32;                          /**< TRNG Statistical Check Poker Count 3 and 2 Register, offset: 0x84 */
  __I  uint32_t PKRCNT54;                          /**< TRNG Statistical Check Poker Count 5 and 4 Register, offset: 0x88 */
  __I  uint32_t PKRCNT76;                          /**< TRNG Statistical Check Poker Count 7 and 6 Register, offset: 0x8C */
  __I  uint32_t PKRCNT98;                          /**< TRNG Statistical Check Poker Count 9 and 8 Register, offset: 0x90 */
  __I  uint32_t PKRCNTBA;                          /**< TRNG Statistical Check Poker Count B and A Register, offset: 0x94 */
  __I  uint32_t PKRCNTDC;                          /**< TRNG Statistical Check Poker Count D and C Register, offset: 0x98 */
  __I  uint32_t PKRCNTFE;                          /**< TRNG Statistical Check Poker Count F and E Register, offset: 0x9C */
  __IO uint32_t SEC_CFG;                           /**< TRNG Security Configuration Register, offset: 0xA0 */
  __IO uint32_t INT_CTRL;                          /**< TRNG Interrupt Control Register, offset: 0xA4 */
  __IO uint32_t INT_MASK;                          /**< TRNG Mask Register, offset: 0xA8 */
  __IO uint32_t INT_STATUS;                        /**< TRNG Interrupt Status Register, offset: 0xAC */
       uint8_t RESERVED_0[64];
  __I  uint32_t VID1;                              /**< TRNG Version ID Register (MS), offset: 0xF0 */
  __I  uint32_t VID2;                              /**< TRNG Version ID Register (LS), offset: 0xF4 */
} TRNG_Type;

/* ----------------------------------------------------------------------------
   -- TRNG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRNG_Register_Masks TRNG Register Masks
 * @{
 */

/*! @name MCTL - TRNG Miscellaneous Control Register */
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

/*! @name SCMISC - TRNG Statistical Check Miscellaneous Register */
#define TRNG_SCMISC_LRUN_MAX_MASK                (0xFFU)
#define TRNG_SCMISC_LRUN_MAX_SHIFT               (0U)
#define TRNG_SCMISC_LRUN_MAX(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCMISC_LRUN_MAX_SHIFT)) & TRNG_SCMISC_LRUN_MAX_MASK)
#define TRNG_SCMISC_RTY_CT_MASK                  (0xF0000U)
#define TRNG_SCMISC_RTY_CT_SHIFT                 (16U)
#define TRNG_SCMISC_RTY_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCMISC_RTY_CT_SHIFT)) & TRNG_SCMISC_RTY_CT_MASK)

/*! @name PKRRNG - TRNG Poker Range Register */
#define TRNG_PKRRNG_PKR_RNG_MASK                 (0xFFFFU)
#define TRNG_PKRRNG_PKR_RNG_SHIFT                (0U)
#define TRNG_PKRRNG_PKR_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_PKRRNG_PKR_RNG_SHIFT)) & TRNG_PKRRNG_PKR_RNG_MASK)

/*! @name PKRMAX - TRNG Poker Maximum Limit Register */
#define TRNG_PKRMAX_PKR_MAX_MASK                 (0xFFFFFFU)
#define TRNG_PKRMAX_PKR_MAX_SHIFT                (0U)
#define TRNG_PKRMAX_PKR_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_PKRMAX_PKR_MAX_SHIFT)) & TRNG_PKRMAX_PKR_MAX_MASK)

/*! @name PKRSQ - TRNG Poker Square Calculation Result Register */
#define TRNG_PKRSQ_PKR_SQ_MASK                   (0xFFFFFFU)
#define TRNG_PKRSQ_PKR_SQ_SHIFT                  (0U)
#define TRNG_PKRSQ_PKR_SQ(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_PKRSQ_PKR_SQ_SHIFT)) & TRNG_PKRSQ_PKR_SQ_MASK)

/*! @name SDCTL - TRNG Seed Control Register */
#define TRNG_SDCTL_SAMP_SIZE_MASK                (0xFFFFU)
#define TRNG_SDCTL_SAMP_SIZE_SHIFT               (0U)
#define TRNG_SDCTL_SAMP_SIZE(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SDCTL_SAMP_SIZE_SHIFT)) & TRNG_SDCTL_SAMP_SIZE_MASK)
#define TRNG_SDCTL_ENT_DLY_MASK                  (0xFFFF0000U)
#define TRNG_SDCTL_ENT_DLY_SHIFT                 (16U)
#define TRNG_SDCTL_ENT_DLY(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SDCTL_ENT_DLY_SHIFT)) & TRNG_SDCTL_ENT_DLY_MASK)

/*! @name SBLIM - TRNG Sparse Bit Limit Register */
#define TRNG_SBLIM_SB_LIM_MASK                   (0x3FFU)
#define TRNG_SBLIM_SB_LIM_SHIFT                  (0U)
#define TRNG_SBLIM_SB_LIM(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_SBLIM_SB_LIM_SHIFT)) & TRNG_SBLIM_SB_LIM_MASK)

/*! @name TOTSAM - TRNG Total Samples Register */
#define TRNG_TOTSAM_TOT_SAM_MASK                 (0xFFFFFU)
#define TRNG_TOTSAM_TOT_SAM_SHIFT                (0U)
#define TRNG_TOTSAM_TOT_SAM(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_TOTSAM_TOT_SAM_SHIFT)) & TRNG_TOTSAM_TOT_SAM_MASK)

/*! @name FRQMIN - TRNG Frequency Count Minimum Limit Register */
#define TRNG_FRQMIN_FRQ_MIN_MASK                 (0x3FFFFFU)
#define TRNG_FRQMIN_FRQ_MIN_SHIFT                (0U)
#define TRNG_FRQMIN_FRQ_MIN(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_FRQMIN_FRQ_MIN_SHIFT)) & TRNG_FRQMIN_FRQ_MIN_MASK)

/*! @name FRQCNT - TRNG Frequency Count Register */
#define TRNG_FRQCNT_FRQ_CT_MASK                  (0x3FFFFFU)
#define TRNG_FRQCNT_FRQ_CT_SHIFT                 (0U)
#define TRNG_FRQCNT_FRQ_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_FRQCNT_FRQ_CT_SHIFT)) & TRNG_FRQCNT_FRQ_CT_MASK)

/*! @name FRQMAX - TRNG Frequency Count Maximum Limit Register */
#define TRNG_FRQMAX_FRQ_MAX_MASK                 (0x3FFFFFU)
#define TRNG_FRQMAX_FRQ_MAX_SHIFT                (0U)
#define TRNG_FRQMAX_FRQ_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_FRQMAX_FRQ_MAX_SHIFT)) & TRNG_FRQMAX_FRQ_MAX_MASK)

/*! @name SCMC - TRNG Statistical Check Monobit Count Register */
#define TRNG_SCMC_MONO_CT_MASK                   (0xFFFFU)
#define TRNG_SCMC_MONO_CT_SHIFT                  (0U)
#define TRNG_SCMC_MONO_CT(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_SCMC_MONO_CT_SHIFT)) & TRNG_SCMC_MONO_CT_MASK)

/*! @name SCML - TRNG Statistical Check Monobit Limit Register */
#define TRNG_SCML_MONO_MAX_MASK                  (0xFFFFU)
#define TRNG_SCML_MONO_MAX_SHIFT                 (0U)
#define TRNG_SCML_MONO_MAX(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCML_MONO_MAX_SHIFT)) & TRNG_SCML_MONO_MAX_MASK)
#define TRNG_SCML_MONO_RNG_MASK                  (0xFFFF0000U)
#define TRNG_SCML_MONO_RNG_SHIFT                 (16U)
#define TRNG_SCML_MONO_RNG(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCML_MONO_RNG_SHIFT)) & TRNG_SCML_MONO_RNG_MASK)

/*! @name SCR1C - TRNG Statistical Check Run Length 1 Count Register */
#define TRNG_SCR1C_R1_0_CT_MASK                  (0x7FFFU)
#define TRNG_SCR1C_R1_0_CT_SHIFT                 (0U)
#define TRNG_SCR1C_R1_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1C_R1_0_CT_SHIFT)) & TRNG_SCR1C_R1_0_CT_MASK)
#define TRNG_SCR1C_R1_1_CT_MASK                  (0x7FFF0000U)
#define TRNG_SCR1C_R1_1_CT_SHIFT                 (16U)
#define TRNG_SCR1C_R1_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1C_R1_1_CT_SHIFT)) & TRNG_SCR1C_R1_1_CT_MASK)

/*! @name SCR1L - TRNG Statistical Check Run Length 1 Limit Register */
#define TRNG_SCR1L_RUN1_MAX_MASK                 (0x7FFFU)
#define TRNG_SCR1L_RUN1_MAX_SHIFT                (0U)
#define TRNG_SCR1L_RUN1_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1L_RUN1_MAX_SHIFT)) & TRNG_SCR1L_RUN1_MAX_MASK)
#define TRNG_SCR1L_RUN1_RNG_MASK                 (0x7FFF0000U)
#define TRNG_SCR1L_RUN1_RNG_SHIFT                (16U)
#define TRNG_SCR1L_RUN1_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR1L_RUN1_RNG_SHIFT)) & TRNG_SCR1L_RUN1_RNG_MASK)

/*! @name SCR2C - TRNG Statistical Check Run Length 2 Count Register */
#define TRNG_SCR2C_R2_0_CT_MASK                  (0x3FFFU)
#define TRNG_SCR2C_R2_0_CT_SHIFT                 (0U)
#define TRNG_SCR2C_R2_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2C_R2_0_CT_SHIFT)) & TRNG_SCR2C_R2_0_CT_MASK)
#define TRNG_SCR2C_R2_1_CT_MASK                  (0x3FFF0000U)
#define TRNG_SCR2C_R2_1_CT_SHIFT                 (16U)
#define TRNG_SCR2C_R2_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2C_R2_1_CT_SHIFT)) & TRNG_SCR2C_R2_1_CT_MASK)

/*! @name SCR2L - TRNG Statistical Check Run Length 2 Limit Register */
#define TRNG_SCR2L_RUN2_MAX_MASK                 (0x3FFFU)
#define TRNG_SCR2L_RUN2_MAX_SHIFT                (0U)
#define TRNG_SCR2L_RUN2_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2L_RUN2_MAX_SHIFT)) & TRNG_SCR2L_RUN2_MAX_MASK)
#define TRNG_SCR2L_RUN2_RNG_MASK                 (0x3FFF0000U)
#define TRNG_SCR2L_RUN2_RNG_SHIFT                (16U)
#define TRNG_SCR2L_RUN2_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR2L_RUN2_RNG_SHIFT)) & TRNG_SCR2L_RUN2_RNG_MASK)

/*! @name SCR3C - TRNG Statistical Check Run Length 3 Count Register */
#define TRNG_SCR3C_R3_0_CT_MASK                  (0x1FFFU)
#define TRNG_SCR3C_R3_0_CT_SHIFT                 (0U)
#define TRNG_SCR3C_R3_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3C_R3_0_CT_SHIFT)) & TRNG_SCR3C_R3_0_CT_MASK)
#define TRNG_SCR3C_R3_1_CT_MASK                  (0x1FFF0000U)
#define TRNG_SCR3C_R3_1_CT_SHIFT                 (16U)
#define TRNG_SCR3C_R3_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3C_R3_1_CT_SHIFT)) & TRNG_SCR3C_R3_1_CT_MASK)

/*! @name SCR3L - TRNG Statistical Check Run Length 3 Limit Register */
#define TRNG_SCR3L_RUN3_MAX_MASK                 (0x1FFFU)
#define TRNG_SCR3L_RUN3_MAX_SHIFT                (0U)
#define TRNG_SCR3L_RUN3_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3L_RUN3_MAX_SHIFT)) & TRNG_SCR3L_RUN3_MAX_MASK)
#define TRNG_SCR3L_RUN3_RNG_MASK                 (0x1FFF0000U)
#define TRNG_SCR3L_RUN3_RNG_SHIFT                (16U)
#define TRNG_SCR3L_RUN3_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR3L_RUN3_RNG_SHIFT)) & TRNG_SCR3L_RUN3_RNG_MASK)

/*! @name SCR4C - TRNG Statistical Check Run Length 4 Count Register */
#define TRNG_SCR4C_R4_0_CT_MASK                  (0xFFFU)
#define TRNG_SCR4C_R4_0_CT_SHIFT                 (0U)
#define TRNG_SCR4C_R4_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4C_R4_0_CT_SHIFT)) & TRNG_SCR4C_R4_0_CT_MASK)
#define TRNG_SCR4C_R4_1_CT_MASK                  (0xFFF0000U)
#define TRNG_SCR4C_R4_1_CT_SHIFT                 (16U)
#define TRNG_SCR4C_R4_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4C_R4_1_CT_SHIFT)) & TRNG_SCR4C_R4_1_CT_MASK)

/*! @name SCR4L - TRNG Statistical Check Run Length 4 Limit Register */
#define TRNG_SCR4L_RUN4_MAX_MASK                 (0xFFFU)
#define TRNG_SCR4L_RUN4_MAX_SHIFT                (0U)
#define TRNG_SCR4L_RUN4_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4L_RUN4_MAX_SHIFT)) & TRNG_SCR4L_RUN4_MAX_MASK)
#define TRNG_SCR4L_RUN4_RNG_MASK                 (0xFFF0000U)
#define TRNG_SCR4L_RUN4_RNG_SHIFT                (16U)
#define TRNG_SCR4L_RUN4_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR4L_RUN4_RNG_SHIFT)) & TRNG_SCR4L_RUN4_RNG_MASK)

/*! @name SCR5C - TRNG Statistical Check Run Length 5 Count Register */
#define TRNG_SCR5C_R5_0_CT_MASK                  (0x7FFU)
#define TRNG_SCR5C_R5_0_CT_SHIFT                 (0U)
#define TRNG_SCR5C_R5_0_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5C_R5_0_CT_SHIFT)) & TRNG_SCR5C_R5_0_CT_MASK)
#define TRNG_SCR5C_R5_1_CT_MASK                  (0x7FF0000U)
#define TRNG_SCR5C_R5_1_CT_SHIFT                 (16U)
#define TRNG_SCR5C_R5_1_CT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5C_R5_1_CT_SHIFT)) & TRNG_SCR5C_R5_1_CT_MASK)

/*! @name SCR5L - TRNG Statistical Check Run Length 5 Limit Register */
#define TRNG_SCR5L_RUN5_MAX_MASK                 (0x7FFU)
#define TRNG_SCR5L_RUN5_MAX_SHIFT                (0U)
#define TRNG_SCR5L_RUN5_MAX(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5L_RUN5_MAX_SHIFT)) & TRNG_SCR5L_RUN5_MAX_MASK)
#define TRNG_SCR5L_RUN5_RNG_MASK                 (0x7FF0000U)
#define TRNG_SCR5L_RUN5_RNG_SHIFT                (16U)
#define TRNG_SCR5L_RUN5_RNG(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SCR5L_RUN5_RNG_SHIFT)) & TRNG_SCR5L_RUN5_RNG_MASK)

/*! @name SCR6PC - TRNG Statistical Check Run Length 6+ Count Register */
#define TRNG_SCR6PC_R6P_0_CT_MASK                (0x7FFU)
#define TRNG_SCR6PC_R6P_0_CT_SHIFT               (0U)
#define TRNG_SCR6PC_R6P_0_CT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PC_R6P_0_CT_SHIFT)) & TRNG_SCR6PC_R6P_0_CT_MASK)
#define TRNG_SCR6PC_R6P_1_CT_MASK                (0x7FF0000U)
#define TRNG_SCR6PC_R6P_1_CT_SHIFT               (16U)
#define TRNG_SCR6PC_R6P_1_CT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PC_R6P_1_CT_SHIFT)) & TRNG_SCR6PC_R6P_1_CT_MASK)

/*! @name SCR6PL - TRNG Statistical Check Run Length 6+ Limit Register */
#define TRNG_SCR6PL_RUN6P_MAX_MASK               (0x7FFU)
#define TRNG_SCR6PL_RUN6P_MAX_SHIFT              (0U)
#define TRNG_SCR6PL_RUN6P_MAX(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PL_RUN6P_MAX_SHIFT)) & TRNG_SCR6PL_RUN6P_MAX_MASK)
#define TRNG_SCR6PL_RUN6P_RNG_MASK               (0x7FF0000U)
#define TRNG_SCR6PL_RUN6P_RNG_SHIFT              (16U)
#define TRNG_SCR6PL_RUN6P_RNG(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_SCR6PL_RUN6P_RNG_SHIFT)) & TRNG_SCR6PL_RUN6P_RNG_MASK)

/*! @name STATUS - TRNG Status Register */
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

/*! @name ENT - TRNG Entropy Read Register */
#define TRNG_ENT_ENT_MASK                        (0xFFFFFFFFU)
#define TRNG_ENT_ENT_SHIFT                       (0U)
#define TRNG_ENT_ENT(x)                          (((uint32_t)(((uint32_t)(x)) << TRNG_ENT_ENT_SHIFT)) & TRNG_ENT_ENT_MASK)

/* The count of TRNG_ENT */
#define TRNG_ENT_COUNT                           (16U)

/*! @name PKRCNT10 - TRNG Statistical Check Poker Count 1 and 0 Register */
#define TRNG_PKRCNT10_PKR_0_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT10_PKR_0_CT_SHIFT             (0U)
#define TRNG_PKRCNT10_PKR_0_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT10_PKR_0_CT_SHIFT)) & TRNG_PKRCNT10_PKR_0_CT_MASK)
#define TRNG_PKRCNT10_PKR_1_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT10_PKR_1_CT_SHIFT             (16U)
#define TRNG_PKRCNT10_PKR_1_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT10_PKR_1_CT_SHIFT)) & TRNG_PKRCNT10_PKR_1_CT_MASK)

/*! @name PKRCNT32 - TRNG Statistical Check Poker Count 3 and 2 Register */
#define TRNG_PKRCNT32_PKR_2_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT32_PKR_2_CT_SHIFT             (0U)
#define TRNG_PKRCNT32_PKR_2_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT32_PKR_2_CT_SHIFT)) & TRNG_PKRCNT32_PKR_2_CT_MASK)
#define TRNG_PKRCNT32_PKR_3_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT32_PKR_3_CT_SHIFT             (16U)
#define TRNG_PKRCNT32_PKR_3_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT32_PKR_3_CT_SHIFT)) & TRNG_PKRCNT32_PKR_3_CT_MASK)

/*! @name PKRCNT54 - TRNG Statistical Check Poker Count 5 and 4 Register */
#define TRNG_PKRCNT54_PKR_4_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT54_PKR_4_CT_SHIFT             (0U)
#define TRNG_PKRCNT54_PKR_4_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT54_PKR_4_CT_SHIFT)) & TRNG_PKRCNT54_PKR_4_CT_MASK)
#define TRNG_PKRCNT54_PKR_5_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT54_PKR_5_CT_SHIFT             (16U)
#define TRNG_PKRCNT54_PKR_5_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT54_PKR_5_CT_SHIFT)) & TRNG_PKRCNT54_PKR_5_CT_MASK)

/*! @name PKRCNT76 - TRNG Statistical Check Poker Count 7 and 6 Register */
#define TRNG_PKRCNT76_PKR_6_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT76_PKR_6_CT_SHIFT             (0U)
#define TRNG_PKRCNT76_PKR_6_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT76_PKR_6_CT_SHIFT)) & TRNG_PKRCNT76_PKR_6_CT_MASK)
#define TRNG_PKRCNT76_PKR_7_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT76_PKR_7_CT_SHIFT             (16U)
#define TRNG_PKRCNT76_PKR_7_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT76_PKR_7_CT_SHIFT)) & TRNG_PKRCNT76_PKR_7_CT_MASK)

/*! @name PKRCNT98 - TRNG Statistical Check Poker Count 9 and 8 Register */
#define TRNG_PKRCNT98_PKR_8_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNT98_PKR_8_CT_SHIFT             (0U)
#define TRNG_PKRCNT98_PKR_8_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT98_PKR_8_CT_SHIFT)) & TRNG_PKRCNT98_PKR_8_CT_MASK)
#define TRNG_PKRCNT98_PKR_9_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNT98_PKR_9_CT_SHIFT             (16U)
#define TRNG_PKRCNT98_PKR_9_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNT98_PKR_9_CT_SHIFT)) & TRNG_PKRCNT98_PKR_9_CT_MASK)

/*! @name PKRCNTBA - TRNG Statistical Check Poker Count B and A Register */
#define TRNG_PKRCNTBA_PKR_A_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTBA_PKR_A_CT_SHIFT             (0U)
#define TRNG_PKRCNTBA_PKR_A_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTBA_PKR_A_CT_SHIFT)) & TRNG_PKRCNTBA_PKR_A_CT_MASK)
#define TRNG_PKRCNTBA_PKR_B_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTBA_PKR_B_CT_SHIFT             (16U)
#define TRNG_PKRCNTBA_PKR_B_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTBA_PKR_B_CT_SHIFT)) & TRNG_PKRCNTBA_PKR_B_CT_MASK)

/*! @name PKRCNTDC - TRNG Statistical Check Poker Count D and C Register */
#define TRNG_PKRCNTDC_PKR_C_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTDC_PKR_C_CT_SHIFT             (0U)
#define TRNG_PKRCNTDC_PKR_C_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTDC_PKR_C_CT_SHIFT)) & TRNG_PKRCNTDC_PKR_C_CT_MASK)
#define TRNG_PKRCNTDC_PKR_D_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTDC_PKR_D_CT_SHIFT             (16U)
#define TRNG_PKRCNTDC_PKR_D_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTDC_PKR_D_CT_SHIFT)) & TRNG_PKRCNTDC_PKR_D_CT_MASK)

/*! @name PKRCNTFE - TRNG Statistical Check Poker Count F and E Register */
#define TRNG_PKRCNTFE_PKR_E_CT_MASK              (0xFFFFU)
#define TRNG_PKRCNTFE_PKR_E_CT_SHIFT             (0U)
#define TRNG_PKRCNTFE_PKR_E_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTFE_PKR_E_CT_SHIFT)) & TRNG_PKRCNTFE_PKR_E_CT_MASK)
#define TRNG_PKRCNTFE_PKR_F_CT_MASK              (0xFFFF0000U)
#define TRNG_PKRCNTFE_PKR_F_CT_SHIFT             (16U)
#define TRNG_PKRCNTFE_PKR_F_CT(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_PKRCNTFE_PKR_F_CT_SHIFT)) & TRNG_PKRCNTFE_PKR_F_CT_MASK)

/*! @name SEC_CFG - TRNG Security Configuration Register */
#define TRNG_SEC_CFG_SH0_MASK                    (0x1U)
#define TRNG_SEC_CFG_SH0_SHIFT                   (0U)
#define TRNG_SEC_CFG_SH0(x)                      (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_SH0_SHIFT)) & TRNG_SEC_CFG_SH0_MASK)
#define TRNG_SEC_CFG_NO_PRGM_MASK                (0x2U)
#define TRNG_SEC_CFG_NO_PRGM_SHIFT               (1U)
#define TRNG_SEC_CFG_NO_PRGM(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_NO_PRGM_SHIFT)) & TRNG_SEC_CFG_NO_PRGM_MASK)
#define TRNG_SEC_CFG_SK_VAL_MASK                 (0x4U)
#define TRNG_SEC_CFG_SK_VAL_SHIFT                (2U)
#define TRNG_SEC_CFG_SK_VAL(x)                   (((uint32_t)(((uint32_t)(x)) << TRNG_SEC_CFG_SK_VAL_SHIFT)) & TRNG_SEC_CFG_SK_VAL_MASK)

/*! @name INT_CTRL - TRNG Interrupt Control Register */
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

/*! @name INT_MASK - TRNG Mask Register */
#define TRNG_INT_MASK_HW_ERR_MASK                (0x1U)
#define TRNG_INT_MASK_HW_ERR_SHIFT               (0U)
#define TRNG_INT_MASK_HW_ERR(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_HW_ERR_SHIFT)) & TRNG_INT_MASK_HW_ERR_MASK)
#define TRNG_INT_MASK_ENT_VAL_MASK               (0x2U)
#define TRNG_INT_MASK_ENT_VAL_SHIFT              (1U)
#define TRNG_INT_MASK_ENT_VAL(x)                 (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_ENT_VAL_SHIFT)) & TRNG_INT_MASK_ENT_VAL_MASK)
#define TRNG_INT_MASK_FRQ_CT_FAIL_MASK           (0x4U)
#define TRNG_INT_MASK_FRQ_CT_FAIL_SHIFT          (2U)
#define TRNG_INT_MASK_FRQ_CT_FAIL(x)             (((uint32_t)(((uint32_t)(x)) << TRNG_INT_MASK_FRQ_CT_FAIL_SHIFT)) & TRNG_INT_MASK_FRQ_CT_FAIL_MASK)

/*! @name INT_STATUS - TRNG Interrupt Status Register */
#define TRNG_INT_STATUS_HW_ERR_MASK              (0x1U)
#define TRNG_INT_STATUS_HW_ERR_SHIFT             (0U)
#define TRNG_INT_STATUS_HW_ERR(x)                (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_HW_ERR_SHIFT)) & TRNG_INT_STATUS_HW_ERR_MASK)
#define TRNG_INT_STATUS_ENT_VAL_MASK             (0x2U)
#define TRNG_INT_STATUS_ENT_VAL_SHIFT            (1U)
#define TRNG_INT_STATUS_ENT_VAL(x)               (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_ENT_VAL_SHIFT)) & TRNG_INT_STATUS_ENT_VAL_MASK)
#define TRNG_INT_STATUS_FRQ_CT_FAIL_MASK         (0x4U)
#define TRNG_INT_STATUS_FRQ_CT_FAIL_SHIFT        (2U)
#define TRNG_INT_STATUS_FRQ_CT_FAIL(x)           (((uint32_t)(((uint32_t)(x)) << TRNG_INT_STATUS_FRQ_CT_FAIL_SHIFT)) & TRNG_INT_STATUS_FRQ_CT_FAIL_MASK)

/*! @name VID1 - TRNG Version ID Register (MS) */
#define TRNG_VID1_MIN_REV_MASK                   (0xFFU)
#define TRNG_VID1_MIN_REV_SHIFT                  (0U)
#define TRNG_VID1_MIN_REV(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_MIN_REV_SHIFT)) & TRNG_VID1_MIN_REV_MASK)
#define TRNG_VID1_MAJ_REV_MASK                   (0xFF00U)
#define TRNG_VID1_MAJ_REV_SHIFT                  (8U)
#define TRNG_VID1_MAJ_REV(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_MAJ_REV_SHIFT)) & TRNG_VID1_MAJ_REV_MASK)
#define TRNG_VID1_IP_ID_MASK                     (0xFFFF0000U)
#define TRNG_VID1_IP_ID_SHIFT                    (16U)
#define TRNG_VID1_IP_ID(x)                       (((uint32_t)(((uint32_t)(x)) << TRNG_VID1_IP_ID_SHIFT)) & TRNG_VID1_IP_ID_MASK)

/*! @name VID2 - TRNG Version ID Register (LS) */
#define TRNG_VID2_CONFIG_OPT_MASK                (0xFFU)
#define TRNG_VID2_CONFIG_OPT_SHIFT               (0U)
#define TRNG_VID2_CONFIG_OPT(x)                  (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_CONFIG_OPT_SHIFT)) & TRNG_VID2_CONFIG_OPT_MASK)
#define TRNG_VID2_ECO_REV_MASK                   (0xFF00U)
#define TRNG_VID2_ECO_REV_SHIFT                  (8U)
#define TRNG_VID2_ECO_REV(x)                     (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_ECO_REV_SHIFT)) & TRNG_VID2_ECO_REV_MASK)
#define TRNG_VID2_INTG_OPT_MASK                  (0xFF0000U)
#define TRNG_VID2_INTG_OPT_SHIFT                 (16U)
#define TRNG_VID2_INTG_OPT(x)                    (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_INTG_OPT_SHIFT)) & TRNG_VID2_INTG_OPT_MASK)
#define TRNG_VID2_ERA_MASK                       (0xFF000000U)
#define TRNG_VID2_ERA_SHIFT                      (24U)
#define TRNG_VID2_ERA(x)                         (((uint32_t)(((uint32_t)(x)) << TRNG_VID2_ERA_SHIFT)) & TRNG_VID2_ERA_MASK)


/*!
 * @}
 */ /* end of group TRNG_Register_Masks */


/* TRNG - Peripheral instance base addresses */
/** Peripheral TRNG base address */
#define TRNG_BASE                                (0x400A5000u)
/** Peripheral TRNG base pointer */
#define TRNG                                     ((TRNG_Type *)TRNG_BASE)
/** Array initializer of TRNG peripheral base addresses */
#define TRNG_BASE_ADDRS                          { TRNG_BASE }
/** Array initializer of TRNG peripheral base pointers */
#define TRNG_BASE_PTRS                           { TRNG }
/** Interrupt vectors for the TRNG peripheral type */
#define TRNG_IRQS                                { TRNG_IRQn }

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
#define TSI0_BASE                                (0x40062000u)
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
   -- TSTMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TSTMR_Peripheral_Access_Layer TSTMR Peripheral Access Layer
 * @{
 */

/** TSTMR - Register Layout Typedef */
typedef struct {
  __I  uint32_t L;                                 /**< Time Stamp Timer Register Low, offset: 0x0 */
  __I  uint32_t H;                                 /**< Time Stamp Timer Register High, offset: 0x4 */
} TSTMR_Type;

/* ----------------------------------------------------------------------------
   -- TSTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TSTMR_Register_Masks TSTMR Register Masks
 * @{
 */

/*! @name L - Time Stamp Timer Register Low */
#define TSTMR_L_VALUE_MASK                       (0xFFFFFFFFU)
#define TSTMR_L_VALUE_SHIFT                      (0U)
#define TSTMR_L_VALUE(x)                         (((uint32_t)(((uint32_t)(x)) << TSTMR_L_VALUE_SHIFT)) & TSTMR_L_VALUE_MASK)

/*! @name H - Time Stamp Timer Register High */
#define TSTMR_H_VALUE_MASK                       (0xFFFFFFU)
#define TSTMR_H_VALUE_SHIFT                      (0U)
#define TSTMR_H_VALUE(x)                         (((uint32_t)(((uint32_t)(x)) << TSTMR_H_VALUE_SHIFT)) & TSTMR_H_VALUE_MASK)


/*!
 * @}
 */ /* end of group TSTMR_Register_Masks */


/* TSTMR - Peripheral instance base addresses */
/** Peripheral TSTMR0 base address */
#define TSTMR0_BASE                              (0x400750F0u)
/** Peripheral TSTMR0 base pointer */
#define TSTMR0                                   ((TSTMR_Type *)TSTMR0_BASE)
/** Array initializer of TSTMR peripheral base addresses */
#define TSTMR_BASE_ADDRS                         { TSTMR0_BASE }
/** Array initializer of TSTMR peripheral base pointers */
#define TSTMR_BASE_PTRS                          { TSTMR0 }

/*!
 * @}
 */ /* end of group TSTMR_Peripheral_Access_Layer */


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
       uint8_t RESERVED_26[15];
  __IO uint8_t KEEP_ALIVE_CTRL;                    /**< Keep Alive mode control, offset: 0x124 */
       uint8_t RESERVED_27[3];
  __IO uint8_t KEEP_ALIVE_WKCTRL;                  /**< Keep Alive mode wakeup control, offset: 0x128 */
       uint8_t RESERVED_28[3];
  __IO uint8_t MISCCTRL;                           /**< Miscellaneous Control register, offset: 0x12C */
       uint8_t RESERVED_29[19];
  __IO uint8_t CLK_RECOVER_CTRL;                   /**< USB Clock recovery control, offset: 0x140 */
       uint8_t RESERVED_30[19];
  __IO uint8_t CLK_RECOVER_INT_EN;                 /**< Clock recovery combined interrupt enable, offset: 0x154 */
       uint8_t RESERVED_31[7];
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
#define USB_OTGISTAT_LINE_STATE_CHG_MASK         (0x20U)
#define USB_OTGISTAT_LINE_STATE_CHG_SHIFT        (5U)
#define USB_OTGISTAT_LINE_STATE_CHG(x)           (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_LINE_STATE_CHG_SHIFT)) & USB_OTGISTAT_LINE_STATE_CHG_MASK)
#define USB_OTGISTAT_ONEMSEC_MASK                (0x40U)
#define USB_OTGISTAT_ONEMSEC_SHIFT               (6U)
#define USB_OTGISTAT_ONEMSEC(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGISTAT_ONEMSEC_SHIFT)) & USB_OTGISTAT_ONEMSEC_MASK)

/*! @name OTGICR - OTG Interrupt Control register */
#define USB_OTGICR_LINESTATEEN_MASK              (0x20U)
#define USB_OTGICR_LINESTATEEN_SHIFT             (5U)
#define USB_OTGICR_LINESTATEEN(x)                (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_LINESTATEEN_SHIFT)) & USB_OTGICR_LINESTATEEN_MASK)
#define USB_OTGICR_ONEMSECEN_MASK                (0x40U)
#define USB_OTGICR_ONEMSECEN_SHIFT               (6U)
#define USB_OTGICR_ONEMSECEN(x)                  (((uint8_t)(((uint8_t)(x)) << USB_OTGICR_ONEMSECEN_SHIFT)) & USB_OTGICR_ONEMSECEN_MASK)

/*! @name OTGSTAT - OTG Status register */
#define USB_OTGSTAT_LINESTATESTABLE_MASK         (0x20U)
#define USB_OTGSTAT_LINESTATESTABLE_SHIFT        (5U)
#define USB_OTGSTAT_LINESTATESTABLE(x)           (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_LINESTATESTABLE_SHIFT)) & USB_OTGSTAT_LINESTATESTABLE_MASK)
#define USB_OTGSTAT_ONEMSECEN_MASK               (0x40U)
#define USB_OTGSTAT_ONEMSECEN_SHIFT              (6U)
#define USB_OTGSTAT_ONEMSECEN(x)                 (((uint8_t)(((uint8_t)(x)) << USB_OTGSTAT_ONEMSECEN_SHIFT)) & USB_OTGSTAT_ONEMSECEN_MASK)

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

/*! @name KEEP_ALIVE_CTRL - Keep Alive mode control */
#define USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_MASK   (0x1U)
#define USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_SHIFT  (0U)
#define USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN(x)     (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_SHIFT)) & USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_MASK)
#define USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_MASK   (0x2U)
#define USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_SHIFT  (1U)
#define USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN(x)     (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_SHIFT)) & USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_MASK)
#define USB_KEEP_ALIVE_CTRL_WAKE_REQ_EN_MASK     (0x8U)
#define USB_KEEP_ALIVE_CTRL_WAKE_REQ_EN_SHIFT    (3U)
#define USB_KEEP_ALIVE_CTRL_WAKE_REQ_EN(x)       (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_CTRL_WAKE_REQ_EN_SHIFT)) & USB_KEEP_ALIVE_CTRL_WAKE_REQ_EN_MASK)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_MASK     (0x10U)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_SHIFT    (4U)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_EN(x)       (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_SHIFT)) & USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_MASK)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK    (0x80U)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_SHIFT   (7U)
#define USB_KEEP_ALIVE_CTRL_WAKE_INT_STS(x)      (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_SHIFT)) & USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK)

/*! @name KEEP_ALIVE_WKCTRL - Keep Alive mode wakeup control */
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ON_THIS_MASK  (0xFU)
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ON_THIS_SHIFT (0U)
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ON_THIS(x)    (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_WKCTRL_WAKE_ON_THIS_SHIFT)) & USB_KEEP_ALIVE_WKCTRL_WAKE_ON_THIS_MASK)
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ENDPT_MASK    (0xF0U)
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ENDPT_SHIFT   (4U)
#define USB_KEEP_ALIVE_WKCTRL_WAKE_ENDPT(x)      (((uint8_t)(((uint8_t)(x)) << USB_KEEP_ALIVE_WKCTRL_WAKE_ENDPT_SHIFT)) & USB_KEEP_ALIVE_WKCTRL_WAKE_ENDPT_MASK)

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
#define USB0_BASE                                (0x40055000u)
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
       uint8_t RESERVED_0[3];
  __IO uint8_t TRM4;                               /**< VREF Trim Register 4, offset: 0x5 */
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
#define VREF_TRM_FLIP_MASK                       (0x80U)
#define VREF_TRM_FLIP_SHIFT                      (7U)
#define VREF_TRM_FLIP(x)                         (((uint8_t)(((uint8_t)(x)) << VREF_TRM_FLIP_SHIFT)) & VREF_TRM_FLIP_MASK)

/*! @name SC - VREF Status and Control Register */
#define VREF_SC_MODE_LV_MASK                     (0x3U)
#define VREF_SC_MODE_LV_SHIFT                    (0U)
#define VREF_SC_MODE_LV(x)                       (((uint8_t)(((uint8_t)(x)) << VREF_SC_MODE_LV_SHIFT)) & VREF_SC_MODE_LV_MASK)
#define VREF_SC_VREFST_MASK                      (0x4U)
#define VREF_SC_VREFST_SHIFT                     (2U)
#define VREF_SC_VREFST(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_VREFST_SHIFT)) & VREF_SC_VREFST_MASK)
#define VREF_SC_TMUXEN_MASK                      (0x8U)
#define VREF_SC_TMUXEN_SHIFT                     (3U)
#define VREF_SC_TMUXEN(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_TMUXEN_SHIFT)) & VREF_SC_TMUXEN_MASK)
#define VREF_SC_TRESEN_MASK                      (0x10U)
#define VREF_SC_TRESEN_SHIFT                     (4U)
#define VREF_SC_TRESEN(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_TRESEN_SHIFT)) & VREF_SC_TRESEN_MASK)
#define VREF_SC_ICOMPEN_MASK                     (0x20U)
#define VREF_SC_ICOMPEN_SHIFT                    (5U)
#define VREF_SC_ICOMPEN(x)                       (((uint8_t)(((uint8_t)(x)) << VREF_SC_ICOMPEN_SHIFT)) & VREF_SC_ICOMPEN_MASK)
#define VREF_SC_REGEN_MASK                       (0x40U)
#define VREF_SC_REGEN_SHIFT                      (6U)
#define VREF_SC_REGEN(x)                         (((uint8_t)(((uint8_t)(x)) << VREF_SC_REGEN_SHIFT)) & VREF_SC_REGEN_MASK)
#define VREF_SC_VREFEN_MASK                      (0x80U)
#define VREF_SC_VREFEN_SHIFT                     (7U)
#define VREF_SC_VREFEN(x)                        (((uint8_t)(((uint8_t)(x)) << VREF_SC_VREFEN_SHIFT)) & VREF_SC_VREFEN_MASK)

/*! @name TRM4 - VREF Trim Register 4 */
#define VREF_TRM4_TRIM2V1_MASK                   (0x3FU)
#define VREF_TRM4_TRIM2V1_SHIFT                  (0U)
#define VREF_TRM4_TRIM2V1(x)                     (((uint8_t)(((uint8_t)(x)) << VREF_TRM4_TRIM2V1_SHIFT)) & VREF_TRM4_TRIM2V1_MASK)
#define VREF_TRM4_VREF2V1_EN_MASK                (0x80U)
#define VREF_TRM4_VREF2V1_EN_SHIFT               (7U)
#define VREF_TRM4_VREF2V1_EN(x)                  (((uint8_t)(((uint8_t)(x)) << VREF_TRM4_VREF2V1_EN_SHIFT)) & VREF_TRM4_VREF2V1_EN_MASK)


/*!
 * @}
 */ /* end of group VREF_Register_Masks */


/* VREF - Peripheral instance base addresses */
/** Peripheral VREF base address */
#define VREF_BASE                                (0x40072000u)
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
  __IO uint32_t CS;                                /**< Watchdog Control and Status Register, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Watchdog Counter Register, offset: 0x4 */
  __IO uint32_t TOVAL;                             /**< Watchdog Timeout Value Register, offset: 0x8 */
  __IO uint32_t WIN;                               /**< Watchdog Window Register, offset: 0xC */
} WDOG_Type;

/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/*! @name CS - Watchdog Control and Status Register */
#define WDOG_CS_STOP_MASK                        (0x1U)
#define WDOG_CS_STOP_SHIFT                       (0U)
#define WDOG_CS_STOP(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_STOP_SHIFT)) & WDOG_CS_STOP_MASK)
#define WDOG_CS_WAIT_MASK                        (0x2U)
#define WDOG_CS_WAIT_SHIFT                       (1U)
#define WDOG_CS_WAIT(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_WAIT_SHIFT)) & WDOG_CS_WAIT_MASK)
#define WDOG_CS_DBG_MASK                         (0x4U)
#define WDOG_CS_DBG_SHIFT                        (2U)
#define WDOG_CS_DBG(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_DBG_SHIFT)) & WDOG_CS_DBG_MASK)
#define WDOG_CS_TST_MASK                         (0x18U)
#define WDOG_CS_TST_SHIFT                        (3U)
#define WDOG_CS_TST(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_TST_SHIFT)) & WDOG_CS_TST_MASK)
#define WDOG_CS_UPDATE_MASK                      (0x20U)
#define WDOG_CS_UPDATE_SHIFT                     (5U)
#define WDOG_CS_UPDATE(x)                        (((uint32_t)(((uint32_t)(x)) << WDOG_CS_UPDATE_SHIFT)) & WDOG_CS_UPDATE_MASK)
#define WDOG_CS_INT_MASK                         (0x40U)
#define WDOG_CS_INT_SHIFT                        (6U)
#define WDOG_CS_INT(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_INT_SHIFT)) & WDOG_CS_INT_MASK)
#define WDOG_CS_EN_MASK                          (0x80U)
#define WDOG_CS_EN_SHIFT                         (7U)
#define WDOG_CS_EN(x)                            (((uint32_t)(((uint32_t)(x)) << WDOG_CS_EN_SHIFT)) & WDOG_CS_EN_MASK)
#define WDOG_CS_CLK_MASK                         (0x300U)
#define WDOG_CS_CLK_SHIFT                        (8U)
#define WDOG_CS_CLK(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_CLK_SHIFT)) & WDOG_CS_CLK_MASK)
#define WDOG_CS_PRES_MASK                        (0x1000U)
#define WDOG_CS_PRES_SHIFT                       (12U)
#define WDOG_CS_PRES(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_PRES_SHIFT)) & WDOG_CS_PRES_MASK)
#define WDOG_CS_CMD32EN_MASK                     (0x2000U)
#define WDOG_CS_CMD32EN_SHIFT                    (13U)
#define WDOG_CS_CMD32EN(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_CS_CMD32EN_SHIFT)) & WDOG_CS_CMD32EN_MASK)
#define WDOG_CS_FLG_MASK                         (0x4000U)
#define WDOG_CS_FLG_SHIFT                        (14U)
#define WDOG_CS_FLG(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_FLG_SHIFT)) & WDOG_CS_FLG_MASK)
#define WDOG_CS_WIN_MASK                         (0x8000U)
#define WDOG_CS_WIN_SHIFT                        (15U)
#define WDOG_CS_WIN(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_WIN_SHIFT)) & WDOG_CS_WIN_MASK)

/*! @name CNT - Watchdog Counter Register */
#define WDOG_CNT_CNTLOW_MASK                     (0xFFU)
#define WDOG_CNT_CNTLOW_SHIFT                    (0U)
#define WDOG_CNT_CNTLOW(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_CNT_CNTLOW_SHIFT)) & WDOG_CNT_CNTLOW_MASK)
#define WDOG_CNT_CNTHIGH_MASK                    (0xFF00U)
#define WDOG_CNT_CNTHIGH_SHIFT                   (8U)
#define WDOG_CNT_CNTHIGH(x)                      (((uint32_t)(((uint32_t)(x)) << WDOG_CNT_CNTHIGH_SHIFT)) & WDOG_CNT_CNTHIGH_MASK)

/*! @name TOVAL - Watchdog Timeout Value Register */
#define WDOG_TOVAL_TOVALLOW_MASK                 (0xFFU)
#define WDOG_TOVAL_TOVALLOW_SHIFT                (0U)
#define WDOG_TOVAL_TOVALLOW(x)                   (((uint32_t)(((uint32_t)(x)) << WDOG_TOVAL_TOVALLOW_SHIFT)) & WDOG_TOVAL_TOVALLOW_MASK)
#define WDOG_TOVAL_TOVALHIGH_MASK                (0xFF00U)
#define WDOG_TOVAL_TOVALHIGH_SHIFT               (8U)
#define WDOG_TOVAL_TOVALHIGH(x)                  (((uint32_t)(((uint32_t)(x)) << WDOG_TOVAL_TOVALHIGH_SHIFT)) & WDOG_TOVAL_TOVALHIGH_MASK)

/*! @name WIN - Watchdog Window Register */
#define WDOG_WIN_WINLOW_MASK                     (0xFFU)
#define WDOG_WIN_WINLOW_SHIFT                    (0U)
#define WDOG_WIN_WINLOW(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_WIN_WINLOW_SHIFT)) & WDOG_WIN_WINLOW_MASK)
#define WDOG_WIN_WINHIGH_MASK                    (0xFF00U)
#define WDOG_WIN_WINHIGH_SHIFT                   (8U)
#define WDOG_WIN_WINHIGH(x)                      (((uint32_t)(((uint32_t)(x)) << WDOG_WIN_WINHIGH_SHIFT)) & WDOG_WIN_WINHIGH_MASK)


/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG0 base address */
#define WDOG0_BASE                               (0x40076000u)
/** Peripheral WDOG0 base pointer */
#define WDOG0                                    ((WDOG_Type *)WDOG0_BASE)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { WDOG0_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG0 }
/** Interrupt vectors for the WDOG peripheral type */
#define WDOG_IRQS                                { WDOG0_IRQn }
#define WDOG_UPDATE_KEY                          (0xD928C520U)
#define WDOG_REFRESH_KEY                         (0xB480A602U)


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

#define I2S_TCR2_CLKMODE_MASK                    I2S_TCR2_MSEL_MASK
#define I2S_TCR2_CLKMODE_SHIFT                   I2S_TCR2_MSEL_SHIFT
#define I2S_TCR2_CLKMODE(x)                      I2S_TCR2_MSEL(x)
#define I2S_RCR2_CLKMODE_MASK                    I2S_RCR2_MSEL_MASK
#define I2S_RCR2_CLKMODE_SHIFT                   I2S_RCR2_MSEL_SHIFT
#define I2S_RCR2_CLKMODE(x)                      I2S_RCR2_MSEL(x)
#define FPTA_BASE                                FGPIOA_BASE
#define FPTA                                     FGPIOA
#define PTA_BASE                                 GPIOA_BASE
#define PTA                                      GPIOA
#define PTB_BASE                                 GPIOB_BASE
#define PTB                                      GPIOB
#define PTC_BASE                                 GPIOC_BASE
#define PTC                                      GPIOC
#define PTD_BASE                                 GPIOD_BASE
#define PTD                                      GPIOD
#define PTE_BASE                                 GPIOE_BASE
#define PTE                                      GPIOE
#define I2C_FLT_STOPIE_MASK                      This_symbol_has_been_deprecated
#define I2C_FLT_STOPIE_SHIFT                     This_symbol_has_been_deprecated
#define I2S_RCR2_CLKMODE_MASK                    I2S_RCR2_MSEL_MASK
#define I2S_RCR2_CLKMODE_SHIFT                   I2S_RCR2_MSEL_SHIFT
#define I2S_RCR2_CLKMODE(x)                      I2S_RCR2_MSEL(x)
#define I2S_TCR2_CLKMODE_MASK                    I2S_TCR2_MSEL_MASK
#define I2S_TCR2_CLKMODE_SHIFT                   I2S_TCR2_MSEL_SHIFT
#define I2S_TCR2_CLKMODE(x)                      I2S_TCR2_MSEL(x)
#define LPTimer_IRQn                             LPTMR0_IRQn
#define LPTimer_IRQHandler                       LPTMR0_IRQHandler
#define TRNG0                                    TRNG
#define CAU                                      CAU0
#define CAU_BASE                                 CAU0_BASE

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MKL28Z7_H_ */

