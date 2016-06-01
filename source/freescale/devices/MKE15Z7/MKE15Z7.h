/*
** ###################################################################
**     Processors:          MKE15Z128VLH7
**                          MKE15Z128VLL7
**                          MKE15Z256VLH7
**                          MKE15Z256VLL7
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    KE1xZP100M72SF0RM, Rev. 1, Nov. 2015
**     Version:             rev. 4.0, 2016-02-19
**     Build:               b160329
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MKE15Z7
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
**     - rev. 1.0 (2015-08-19)
**         Initial version.
**     - rev. 2.0 (2015-09-22)
**         Based on rev0final RDP, add PCC/TRGMUX.
**     - rev. 3.0 (2015-12-29)
**         Align LPFLL register names.
**     - rev. 4.0 (2016-02-19)
**         Based on rev1final RDP.
**
** ###################################################################
*/

/*!
 * @file MKE15Z7.h
 * @version 4.0
 * @date 2016-02-19
 * @brief CMSIS Peripheral Access Layer for MKE15Z7
 *
 * CMSIS Peripheral Access Layer for MKE15Z7
 */

#ifndef _MKE15Z7_H_
#define _MKE15Z7_H_                              /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0400U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0000U


/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 48                 /**< Number of interrupts in the Vector table */

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
  DMA0_04_IRQn                 = 0,                /**< DMA channel 0 or 4 transfer complete */
  DMA0_15_IRQn                 = 1,                /**< DMA channel 1 or 5 transfer complete */
  DMA0_26_IRQn                 = 2,                /**< DMA channel 2 or 6 transfer complete */
  DMA0_37_IRQn                 = 3,                /**< DMA channel 3 or 7 transfer complete */
  DMA_Error_IRQn               = 4,                /**< DMA error interrupt channels 0-7 */
  FTFE_IRQn                    = 5,                /**< Single interrupt vector for all sources */
  LVD_LVW_IRQn                 = 6,                /**< Low-voltage detect, low-voltage warning */
  PORTAE_IRQn                  = 7,                /**< Pin detect (Port A, E) */
  LPI2C0_IRQn                  = 8,                /**< Inter-integrated circuit 0 */
  LPI2C1_IRQn                  = 9,                /**< Inter-integrated circuit 1 */
  LPSPI0_IRQn                  = 10,               /**< Serial peripheral Interface 0 */
  LPSPI1_IRQn                  = 11,               /**< Serial peripheral Interface 1 */
  LPUART0_IRQn                 = 12,               /**< Single interrupt vector for all sources */
  LPUART1_IRQn                 = 13,               /**< Single interrupt vector for all sources */
  LPUART2_IRQn                 = 14,               /**< Single interrupt vector for all sources */
  ADC0_IRQn                    = 15,               /**< ADC0 conversion complete interrupt */
  CMP0_IRQn                    = 16,               /**< CMP0 interrupt */
  FTM0_IRQn                    = 17,               /**< FTM0 single interrupt vector for all sources */
  FTM1_IRQn                    = 18,               /**< FTM1 single interrupt vector for all sources */
  FTM2_IRQn                    = 19,               /**< FTM2 single interrupt vector for all sources */
  RTC_IRQn                     = 20,               /**< Single interrupt vector for all sources */
  CMP1_IRQn                    = 21,               /**< CMP1 interrupt */
  LPIT0_IRQn                   = 22,               /**< LPIT interrupt */
  FLEXIO_IRQn                  = 23,               /**< FLEXIO interrupt */
  TSI_IRQn                     = 24,               /**< TSI interrupt */
  PDB0_IRQn                    = 25,               /**< Programmable delay block */
  PORTBCD_IRQn                 = 26,               /**< Pin detect (Port B, C, D) */
  SCG_IRQn                     = 27,               /**< Multipurpose clock generator */
  WDOG_EWM_IRQn                = 28,               /**< Single interrupt vector for WDOG and EWM */
  PWT_LPTMR0_IRQn              = 29,               /**< Single interrupt vector for PWT and LPTMR0 */
  ADC1_IRQn                    = 30,               /**< ADC1 conversion complete interrupt */
  RCM_IRQn                     = 31                /**< RCM interrupt */
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
#include "system_MKE15Z7.h"            /* Device specific configuration file */

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
    kDmaRequestMux0TSI              = 1|0x100U,    /**< TSI. */
    kDmaRequestMux0LPUART0Rx        = 2|0x100U,    /**< LPUART0 Receive. */
    kDmaRequestMux0LPUART0Tx        = 3|0x100U,    /**< LPUART0 Transmit. */
    kDmaRequestMux0LPUART1Rx        = 4|0x100U,    /**< LPUART1 Receive. */
    kDmaRequestMux0LPUART1Tx        = 5|0x100U,    /**< LPUART1 Transmit. */
    kDmaRequestMux0LPUART2Rx        = 6|0x100U,    /**< LPUART2 Receive. */
    kDmaRequestMux0LPUART2Tx        = 7|0x100U,    /**< LPUART2 Transmit. */
    kDmaRequestMux0Reserved8        = 8|0x100U,    /**< Reserved8 */
    kDmaRequestMux0Reserved9        = 9|0x100U,    /**< Reserved9 */
    kDmaRequestMux0FlexIOChannel0   = 10|0x100U,   /**< FLEXIO Shifter0. */
    kDmaRequestMux0FlexIOChannel1   = 11|0x100U,   /**< FLEXIO Shifter1. */
    kDmaRequestMux0FlexIOChannel2   = 12|0x100U,   /**< FLEXIO Shifter2. */
    kDmaRequestMux0FlexIOChannel3   = 13|0x100U,   /**< FLEXIO Shifter3. */
    kDmaRequestMux0LPSPI0Rx         = 14|0x100U,   /**< LPSPI0 Receive. */
    kDmaRequestMux0LPSPI0Tx         = 15|0x100U,   /**< LPSPI0 Transmit. */
    kDmaRequestMux0LPSPI1Rx         = 16|0x100U,   /**< LPSPI1 Receive. */
    kDmaRequestMux0LPSPI1Tx         = 17|0x100U,   /**< LPSPI1 Transmit. */
    kDmaRequestMux0LPI2C0Rx         = 18|0x100U,   /**< LPI2C0 Receive. */
    kDmaRequestMux0LPI2C0Tx         = 19|0x100U,   /**< LPI2C0 Transmit. */
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
    kDmaRequestMux0LPI2C1Rx         = 32|0x100U,   /**< LPI2C1 Receive. */
    kDmaRequestMux0LPI2C1Tx         = 33|0x100U,   /**< LPI2C1 Transmit. */
    kDmaRequestMux0Reserved34       = 34|0x100U,   /**< Reserved34 */
    kDmaRequestMux0Reserved35       = 35|0x100U,   /**< Reserved35 */
    kDmaRequestMux0Reserved36       = 36|0x100U,   /**< Reserved36 */
    kDmaRequestMux0Reserved37       = 37|0x100U,   /**< Reserved37 */
    kDmaRequestMux0Reserved38       = 38|0x100U,   /**< Reserved38 */
    kDmaRequestMux0Reserved39       = 39|0x100U,   /**< Reserved39 */
    kDmaRequestMux0ADC0             = 40|0x100U,   /**< ADC0 COCO. */
    kDmaRequestMux0ADC1             = 41|0x100U,   /**< ADC1 COCO. */
    kDmaRequestMux0Reserved42       = 42|0x100U,   /**< Reserved42 */
    kDmaRequestMux0CMP0             = 43|0x100U,   /**< CMP0. */
    kDmaRequestMux0CMP1             = 44|0x100U,   /**< CMP1. */
    kDmaRequestMux0Reserved45       = 45|0x100U,   /**< Reserved45 */
    kDmaRequestMux0PDB0             = 46|0x100U,   /**< PDB0. */
    kDmaRequestMux0Reserved47       = 47|0x100U,   /**< Reserved47 */
    kDmaRequestMux0Reserved48       = 48|0x100U,   /**< Reserved48 */
    kDmaRequestMux0PortA            = 49|0x100U,   /**< PTA PortA. */
    kDmaRequestMux0PortB            = 50|0x100U,   /**< PTB PortB. */
    kDmaRequestMux0PortC            = 51|0x100U,   /**< PTC PortC. */
    kDmaRequestMux0PortD            = 52|0x100U,   /**< PTD PortD. */
    kDmaRequestMux0PortE            = 53|0x100U,   /**< PTE PortE. */
    kDmaRequestMux0Reserved54       = 54|0x100U,   /**< Reserved54 */
    kDmaRequestMux0Reserved55       = 55|0x100U,   /**< Reserved55 */
    kDmaRequestMux0Reserved56       = 56|0x100U,   /**< Reserved56 */
    kDmaRequestMux0FTM1Channel2     = 57|0x100U,   /**< FTM1 Channel2 or 3. */
    kDmaRequestMux0FTM1Channel3     = 57|0x100U,   /**< FTM1 Channel2 or 3. */
    kDmaRequestMux0FTM2Channel2     = 58|0x100U,   /**< FTM2 Channel2 or 3. */
    kDmaRequestMux0FTM2Channel3     = 58|0x100U,   /**< FTM2 Channel2 or 3. */
    kDmaRequestMux0LPTMR0           = 59|0x100U,   /**< LPTMR0. */
    kDmaRequestMux0AlwaysOn60       = 60|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn61       = 61|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn62       = 62|0x100U,   /**< DMAMUX Always Enabled slot. */
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
    kTRGMUX_SourceDisabled0         = 0U,          /**< Trigger function is disabled */
    kTRGMUX_SourceTriggerMuxInput0  = 0U,          /**< Trigger Mux input 0 is selected */
    kTRGMUX_SourceDisabled1         = 1U,          /**< Trigger function is disabled */
    kTRGMUX_SourceTriggerMuxInput1  = 1U,          /**< Trigger Mux input 1 is selected */
    kTRGMUX_SimSoftwareTrigger      = 2U,          /**< SIM Software Trigger is selected */
    kTRGMUX_SourceTriggerMuxInput2  = 2U,          /**< Trigger Mux input 2 is selected */
    kTRGMUX_SourceTriggerMuxInput3  = 3U,          /**< Trigger Mux input 3 is selected */
    kTRGMUX_SourceTriggerMuxInput4  = 3U,          /**< Trigger Mux input 4 is selected */
    kTRGMUX_SourceTriggerMuxInput5  = 4U,          /**< Trigger Mux input 5 is selected */
    kTRGMUX_SourceRtcSeconds        = 4U,          /**< RTC Seconds input is selected */
    kTRGMUX_SourceTriggerMuxInput6  = 5U,          /**< Trigger Mux input 6 is selected */
    kTRGMUX_SourceRtcAlarm          = 5U,          /**< RTC Alarm input is selected */
    kTRGMUX_SourceTriggerMuxInput7  = 6U,          /**< Trigger Mux input 7 is selected */
    kTRGMUX_SourceLptmr0            = 6U,          /**< LPTMR0 input is selected */
    kTRGMUX_SourceLpuart0RxData     = 7U,          /**< LPUART0 RX Data is selected */
    kTRGMUX_SourceLpit0Ch0          = 7U,          /**< LPIT0 Channel0 is selected */
    kTRGMUX_SourceLpuart0TxData     = 8U,          /**< LPUART0 TX Data is selected */
    kTRGMUX_SourceLpit0Ch1          = 8U,          /**< LPIT0 Channel1 is selected */
    kTRGMUX_SourceLpuart0Idle       = 9U,          /**< LPUART0 Idle is selected */
    kTRGMUX_SourceLpit0Ch2          = 9U,          /**< LPIT0 Channel2 is selected */
    kTRGMUX_SourceLpi2c0MasterStop  = 10U,         /**< LPI2C0 Master STOP is selected */
    kTRGMUX_SourceLpit0Ch3          = 10U,         /**< LPIT0 Channel3 is selected */
    kTRGMUX_SourceLpi2c0SlaveStop   = 11U,         /**< LPI2C0 Slave STOP is selected */
    kTRGMUX_SourceFtm0              = 11U,         /**< FTM0 is selected */
    kTRGMUX_SourceLpspi0Frame       = 12U,         /**< LPSPI0 Frame is selected */
    kTRGMUX_SourceFtm1              = 12U,         /**< FTM1 is selected */
    kTRGMUX_SourceLpspi0RxData      = 13U,         /**< LPSPI0 RX Data is selected */
    kTRGMUX_SourceFtm2              = 13U,         /**< FTM2 is selected */
    kTRGMUX_SourceLpuart1RxData     = 14U,         /**< LPUART1 RX Data is selected */
    kTRGMUX_SourceLpuart1TxData     = 15U,         /**< LPUART1 TX Data is selected */
    kTRGMUX_SourceAdc0CocoA         = 15U,         /**< ADC0 COCOA is selected */
    kTRGMUX_SourceLpuart1RxIdle     = 16U,         /**< LPUART1 RX Idle is selected */
    kTRGMUX_SourceAdc0CocoB         = 16U,         /**< ADC0 COCOB is selected */
    kTRGMUX_SourceLpi2c1MasterStop  = 17U,         /**< LPI2C1 Master STOP is selected */
    kTRGMUX_SourceCmp0Output        = 17U,         /**< CMP0 Output is selected */
    kTRGMUX_SourceLpi2c1SlaveStop   = 18U,         /**< LPI2C1 Slave STOP is selected */
    kTRGMUX_SourceCmp1Output        = 18U,         /**< CMP1 Output is selected */
    kTRGMUX_SourceLpspi1Frame       = 19U,         /**< LPSPI1 Frame is selected */
    kTRGMUX_SourceLpspi1RxData      = 20U,         /**< LPSPI1 RX Data is selected */
    kTRGMUX_SourceFlexIOTimer0      = 20U,         /**< FLEXIO Trigger 0 is selected */
    kTRGMUX_SourceAdc1CocoA         = 21U,         /**< ADC1 COCOA is selected */
    kTRGMUX_SourceFlexIOTimer1      = 21U,         /**< FLEXIO Trigger 1 is selected */
    kTRGMUX_SourceAdc1CocoB         = 22U,         /**< ADC1 COCOB is selected */
    kTRGMUX_SourceFlexIOTimer2      = 22U,         /**< FLEXIO Trigger 2 is selected */
    kTRGMUX_SourcePdb0Pulse0        = 23U,         /**< PDB0 Pulse0 is selected */
    kTRGMUX_SourceFlexIOTimer3      = 23U,         /**< FLEXIO Trigger 3 is selected */
    kTRGMUX_SourcePdb0Pulse1        = 24U,         /**< PDB0 Pulse1 is selected */
    kTRGMUX_SourceTrgmux1Output0    = 24U,         /**< TRGMUX1 Output 0 is selected */
    kTRGMUX_SourceTrgmux1Output1    = 25U,         /**< TRGMUX1 Output 1 is selected */
    kTRGMUX_SourceTrgmux1Output2    = 26U,         /**< TRGMUX1 Output 2 is selected */
    kTRGMUX_SourceTrgmux1Output3    = 27U,         /**< TRGMUX1 Output 3 is selected */
    kTRGMUX_SourceTrgmux1Output4    = 28U,         /**< TRGMUX1 Output 4 is selected */
    kTRGMUX_SourceTrgmux1Output5    = 29U,         /**< TRGMUX1 Output 5 is selected */
    kTRGMUX_SourceTrgmux1Output6    = 30U,         /**< TRGMUX1 Output 6 is selected */
    kTRGMUX_SourceTrgmux1Output7    = 31U,         /**< TRGMUX1 Output 7 is selected */
} trgmux_source_t;

/*!
 * @brief Structure for the TRGMUX device
 *
 * Defines the structure for the TRGMUX device collections.
 */
typedef enum _trgmux_device
{
    kTRGMUX_Dmamux0                 = 0U,          /**< DMAMUX0 device trigger input */
    kTRGMUX_Ctrl0                   = 0U,          /**< CTRL0 device trigger input */
    kTRGMUX_ExtOut0_3               = 1U,          /**< EXTOUT0 device trigger input */
    kTRGMUX_Ctrl1                   = 1U,          /**< CTRL1 device trigger input */
    kTRGMUX_ExtOut4_7               = 2U,          /**< EXTOUT1 device trigger input */
    kTRGMUX_Adc0                    = 3U,          /**< ADC0 device trigger input */
    kTRGMUX_Adc1                    = 4U,          /**< ADC1 device trigger input */
    kTRGMUX_Cmp0                    = 7U,          /**< CMP0 device trigger input */
    kTRGMUX_Cmp1                    = 8U,          /**< CMP1 device trigger input */
    kTRGMUX_Ftm0                    = 10U,         /**< FTM0 device trigger input */
    kTRGMUX_Ftm1                    = 11U,         /**< FTM1 device trigger input */
    kTRGMUX_Ftm2                    = 12U,         /**< FTM2 device trigger input */
    kTRGMUX_Pdb0                    = 14U,         /**< PDB0 device trigger input */
    kTRGMUX_Flexio                  = 17U,         /**< FLEXIO device trigger input */
    kTRGMUX_Lpit                    = 18U,         /**< LPIT device trigger input */
    kTRGMUX_Lpuart0                 = 19U,         /**< LPUART0 device trigger input */
    kTRGMUX_Lpuart1                 = 20U,         /**< LPUART1 device trigger input */
    kTRGMUX_Lpi2c0                  = 21U,         /**< LPI2C0 device trigger input */
    kTRGMUX_Lpi2c1                  = 22U,         /**< LPI2C1 device trigger input */
    kTRGMUX_Lpspi0                  = 23U,         /**< LPSPI0 device trigger input */
    kTRGMUX_Lpspi1                  = 24U,         /**< LPSPI1 device trigger input */
    kTRGMUX_Lptmr0                  = 25U,         /**< LPTMR0 device trigger input */
    kTRGMUX_Tsi                     = 26U,         /**< TSI device trigger input */
    kTRGMUX_Pwt                     = 27U,         /**< PWT device trigger input */
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
       uint8_t RESERVED_0[56];
  __IO uint32_t CFG1;                              /**< ADC Configuration Register 1, offset: 0x40 */
  __IO uint32_t CFG2;                              /**< ADC Configuration Register 2, offset: 0x44 */
  __I  uint32_t R[2];                              /**< ADC Data Result Register, array offset: 0x48, array step: 0x4 */
       uint8_t RESERVED_1[56];
  __IO uint32_t CV1;                               /**< Compare Value Registers, offset: 0x88 */
  __IO uint32_t CV2;                               /**< Compare Value Registers, offset: 0x8C */
  __IO uint32_t SC2;                               /**< Status and Control Register 2, offset: 0x90 */
  __IO uint32_t SC3;                               /**< Status and Control Register 3, offset: 0x94 */
  __IO uint32_t BASE_OFS;                          /**< BASE Offset Register, offset: 0x98 */
  __IO uint32_t OFS;                               /**< ADC Offset Correction Register, offset: 0x9C */
  __IO uint32_t USR_OFS;                           /**< USER Offset Correction Register, offset: 0xA0 */
  __IO uint32_t XOFS;                              /**< ADC X Offset Correction Register, offset: 0xA4 */
  __IO uint32_t YOFS;                              /**< ADC Y Offset Correction Register, offset: 0xA8 */
  __IO uint32_t G;                                 /**< ADC Gain Register, offset: 0xAC */
  __IO uint32_t UG;                                /**< ADC User Gain Register, offset: 0xB0 */
  __IO uint32_t CLPS;                              /**< ADC General Calibration Value Register, offset: 0xB4 */
  __IO uint32_t CLP3;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xB8 */
  __IO uint32_t CLP2;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xBC */
  __IO uint32_t CLP1;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xC0 */
  __IO uint32_t CLP0;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xC4 */
  __IO uint32_t CLPX;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xC8 */
  __IO uint32_t CLP9;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0xCC */
  __IO uint32_t CLPS_OFS;                          /**< ADC General Calibration Value Register, offset: 0xD0 */
  __IO uint32_t CLP3_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xD4 */
  __IO uint32_t CLP2_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xD8 */
  __IO uint32_t CLP1_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xDC */
  __IO uint32_t CLP0_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xE0 */
  __IO uint32_t CLPX_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xE4 */
  __IO uint32_t CLP9_OFS;                          /**< ADC Plus-Side General Calibration Value Register, offset: 0xE8 */
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
#define ADC_CFG1_ADIV_MASK                       (0x60U)
#define ADC_CFG1_ADIV_SHIFT                      (5U)
#define ADC_CFG1_ADIV(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADIV_SHIFT)) & ADC_CFG1_ADIV_MASK)

/*! @name CFG2 - ADC Configuration Register 2 */
#define ADC_CFG2_SMPLTS_MASK                     (0xFFU)
#define ADC_CFG2_SMPLTS_SHIFT                    (0U)
#define ADC_CFG2_SMPLTS(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_SMPLTS_SHIFT)) & ADC_CFG2_SMPLTS_MASK)

/*! @name R - ADC Data Result Register */
#define ADC_R_D_MASK                             (0xFFFU)
#define ADC_R_D_SHIFT                            (0U)
#define ADC_R_D(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_R_D_SHIFT)) & ADC_R_D_MASK)

/* The count of ADC_R */
#define ADC_R_COUNT                              (2U)

/*! @name CV1 - Compare Value Registers */
#define ADC_CV1_CV_MASK                          (0xFFFU)
#define ADC_CV1_CV_SHIFT                         (0U)
#define ADC_CV1_CV(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_CV1_CV_SHIFT)) & ADC_CV1_CV_MASK)

/*! @name CV2 - Compare Value Registers */
#define ADC_CV2_CV_MASK                          (0xFFFU)
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
#define ADC_SC3_CAL_MASK                         (0x80U)
#define ADC_SC3_CAL_SHIFT                        (7U)
#define ADC_SC3_CAL(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_SC3_CAL_SHIFT)) & ADC_SC3_CAL_MASK)

/*! @name BASE_OFS - BASE Offset Register */
#define ADC_BASE_OFS_BA_OFS_MASK                 (0xFFU)
#define ADC_BASE_OFS_BA_OFS_SHIFT                (0U)
#define ADC_BASE_OFS_BA_OFS(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_BASE_OFS_BA_OFS_SHIFT)) & ADC_BASE_OFS_BA_OFS_MASK)

/*! @name OFS - ADC Offset Correction Register */
#define ADC_OFS_OFS_MASK                         (0xFFFFU)
#define ADC_OFS_OFS_SHIFT                        (0U)
#define ADC_OFS_OFS(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_OFS_OFS_SHIFT)) & ADC_OFS_OFS_MASK)

/*! @name USR_OFS - USER Offset Correction Register */
#define ADC_USR_OFS_USR_OFS_MASK                 (0xFFU)
#define ADC_USR_OFS_USR_OFS_SHIFT                (0U)
#define ADC_USR_OFS_USR_OFS(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_USR_OFS_USR_OFS_SHIFT)) & ADC_USR_OFS_USR_OFS_MASK)

/*! @name XOFS - ADC X Offset Correction Register */
#define ADC_XOFS_OFS_MASK                        (0x3FU)
#define ADC_XOFS_OFS_SHIFT                       (0U)
#define ADC_XOFS_OFS(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_XOFS_OFS_SHIFT)) & ADC_XOFS_OFS_MASK)

/*! @name YOFS - ADC Y Offset Correction Register */
#define ADC_YOFS_YOFS_MASK                       (0xFFU)
#define ADC_YOFS_YOFS_SHIFT                      (0U)
#define ADC_YOFS_YOFS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_YOFS_YOFS_SHIFT)) & ADC_YOFS_YOFS_MASK)

/*! @name G - ADC Gain Register */
#define ADC_G_G_MASK                             (0x7FFU)
#define ADC_G_G_SHIFT                            (0U)
#define ADC_G_G(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_G_G_SHIFT)) & ADC_G_G_MASK)

/*! @name UG - ADC User Gain Register */
#define ADC_UG_UG_MASK                           (0x3FFU)
#define ADC_UG_UG_SHIFT                          (0U)
#define ADC_UG_UG(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_UG_UG_SHIFT)) & ADC_UG_UG_MASK)

/*! @name CLPS - ADC General Calibration Value Register */
#define ADC_CLPS_CLPS_MASK                       (0x7FU)
#define ADC_CLPS_CLPS_SHIFT                      (0U)
#define ADC_CLPS_CLPS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPS_CLPS_SHIFT)) & ADC_CLPS_CLPS_MASK)

/*! @name CLP3 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP3_CLP3_MASK                       (0x3FFU)
#define ADC_CLP3_CLP3_SHIFT                      (0U)
#define ADC_CLP3_CLP3(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP3_CLP3_SHIFT)) & ADC_CLP3_CLP3_MASK)

/*! @name CLP2 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP2_CLP2_MASK                       (0x3FFU)
#define ADC_CLP2_CLP2_SHIFT                      (0U)
#define ADC_CLP2_CLP2(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP2_CLP2_SHIFT)) & ADC_CLP2_CLP2_MASK)

/*! @name CLP1 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP1_CLP1_MASK                       (0x1FFU)
#define ADC_CLP1_CLP1_SHIFT                      (0U)
#define ADC_CLP1_CLP1(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP1_CLP1_SHIFT)) & ADC_CLP1_CLP1_MASK)

/*! @name CLP0 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP0_CLP0_MASK                       (0xFFU)
#define ADC_CLP0_CLP0_SHIFT                      (0U)
#define ADC_CLP0_CLP0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP0_CLP0_SHIFT)) & ADC_CLP0_CLP0_MASK)

/*! @name CLPX - ADC Plus-Side General Calibration Value Register */
#define ADC_CLPX_CLPX_MASK                       (0x7FU)
#define ADC_CLPX_CLPX_SHIFT                      (0U)
#define ADC_CLPX_CLPX(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPX_CLPX_SHIFT)) & ADC_CLPX_CLPX_MASK)
#define ADC_CLPX_CLPXEN_MASK                     (0x80U)
#define ADC_CLPX_CLPXEN_SHIFT                    (7U)
#define ADC_CLPX_CLPXEN(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CLPX_CLPXEN_SHIFT)) & ADC_CLPX_CLPXEN_MASK)

/*! @name CLP9 - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP9_CLP9_MASK                       (0x7FU)
#define ADC_CLP9_CLP9_SHIFT                      (0U)
#define ADC_CLP9_CLP9(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP9_CLP9_SHIFT)) & ADC_CLP9_CLP9_MASK)
#define ADC_CLP9_CLP9EN_MASK                     (0x80U)
#define ADC_CLP9_CLP9EN_SHIFT                    (7U)
#define ADC_CLP9_CLP9EN(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CLP9_CLP9EN_SHIFT)) & ADC_CLP9_CLP9EN_MASK)

/*! @name CLPS_OFS - ADC General Calibration Value Register */
#define ADC_CLPS_OFS_CLPS_OFS_MASK               (0xFU)
#define ADC_CLPS_OFS_CLPS_OFS_SHIFT              (0U)
#define ADC_CLPS_OFS_CLPS_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLPS_OFS_CLPS_OFS_SHIFT)) & ADC_CLPS_OFS_CLPS_OFS_MASK)

/*! @name CLP3_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP3_OFS_CLP3_OFS_MASK               (0xFU)
#define ADC_CLP3_OFS_CLP3_OFS_SHIFT              (0U)
#define ADC_CLP3_OFS_CLP3_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP3_OFS_CLP3_OFS_SHIFT)) & ADC_CLP3_OFS_CLP3_OFS_MASK)

/*! @name CLP2_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP2_OFS_CLP2_OFS_MASK               (0xFU)
#define ADC_CLP2_OFS_CLP2_OFS_SHIFT              (0U)
#define ADC_CLP2_OFS_CLP2_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP2_OFS_CLP2_OFS_SHIFT)) & ADC_CLP2_OFS_CLP2_OFS_MASK)

/*! @name CLP1_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP1_OFS_CLP1_OFS_MASK               (0xFU)
#define ADC_CLP1_OFS_CLP1_OFS_SHIFT              (0U)
#define ADC_CLP1_OFS_CLP1_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP1_OFS_CLP1_OFS_SHIFT)) & ADC_CLP1_OFS_CLP1_OFS_MASK)

/*! @name CLP0_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP0_OFS_CLP0_OFS_MASK               (0xFU)
#define ADC_CLP0_OFS_CLP0_OFS_SHIFT              (0U)
#define ADC_CLP0_OFS_CLP0_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP0_OFS_CLP0_OFS_SHIFT)) & ADC_CLP0_OFS_CLP0_OFS_MASK)

/*! @name CLPX_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLPX_OFS_CLPX_OFS_MASK               (0xFFFU)
#define ADC_CLPX_OFS_CLPX_OFS_SHIFT              (0U)
#define ADC_CLPX_OFS_CLPX_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLPX_OFS_CLPX_OFS_SHIFT)) & ADC_CLPX_OFS_CLPX_OFS_MASK)

/*! @name CLP9_OFS - ADC Plus-Side General Calibration Value Register */
#define ADC_CLP9_OFS_CLP9_OFS_MASK               (0xFFFU)
#define ADC_CLP9_OFS_CLP9_OFS_SHIFT              (0U)
#define ADC_CLP9_OFS_CLP9_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP9_OFS_CLP9_OFS_SHIFT)) & ADC_CLP9_OFS_CLP9_OFS_MASK)


/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base address */
#define ADC0_BASE                                (0x4003B000u)
/** Peripheral ADC0 base pointer */
#define ADC0                                     ((ADC_Type *)ADC0_BASE)
/** Peripheral ADC1 base address */
#define ADC1_BASE                                (0x40027000u)
/** Peripheral ADC1 base pointer */
#define ADC1                                     ((ADC_Type *)ADC1_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC0_BASE, ADC1_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0, ADC1 }
/** Interrupt vectors for the ADC peripheral type */
#define ADC_IRQS                                 { ADC0_IRQn, ADC1_IRQn }

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
  __IO uint32_t PACR[4];                           /**< Peripheral Access Control Register, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[16];
  __IO uint32_t OPACR[16];                         /**< Off-Platform Peripheral Access Control Register, array offset: 0x40, array step: 0x4 */
  __IO uint32_t PACRU;                             /**< Peripheral Access Control Register, offset: 0x80 */
} AIPS_Type;

/* ----------------------------------------------------------------------------
   -- AIPS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Masks AIPS Register Masks
 * @{
 */

/*! @name MPRA - Master Privilege Register A */
#define AIPS_MPRA_MPL2_MASK                      (0x100000U)
#define AIPS_MPRA_MPL2_SHIFT                     (20U)
#define AIPS_MPRA_MPL2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL2_SHIFT)) & AIPS_MPRA_MPL2_MASK)
#define AIPS_MPRA_MTW2_MASK                      (0x200000U)
#define AIPS_MPRA_MTW2_SHIFT                     (21U)
#define AIPS_MPRA_MTW2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW2_SHIFT)) & AIPS_MPRA_MTW2_MASK)
#define AIPS_MPRA_MTR2_MASK                      (0x400000U)
#define AIPS_MPRA_MTR2_SHIFT                     (22U)
#define AIPS_MPRA_MTR2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR2_SHIFT)) & AIPS_MPRA_MTR2_MASK)
#define AIPS_MPRA_MPL0_MASK                      (0x10000000U)
#define AIPS_MPRA_MPL0_SHIFT                     (28U)
#define AIPS_MPRA_MPL0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL0_SHIFT)) & AIPS_MPRA_MPL0_MASK)
#define AIPS_MPRA_MTW0_MASK                      (0x20000000U)
#define AIPS_MPRA_MTW0_SHIFT                     (29U)
#define AIPS_MPRA_MTW0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW0_SHIFT)) & AIPS_MPRA_MTW0_MASK)
#define AIPS_MPRA_MTR0_MASK                      (0x40000000U)
#define AIPS_MPRA_MTR0_SHIFT                     (30U)
#define AIPS_MPRA_MTR0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR0_SHIFT)) & AIPS_MPRA_MTR0_MASK)

/*! @name PACR - Peripheral Access Control Register */
#define AIPS_PACR_TP7_MASK                       (0x1U)
#define AIPS_PACR_TP7_SHIFT                      (0U)
#define AIPS_PACR_TP7(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP7_SHIFT)) & AIPS_PACR_TP7_MASK)
#define AIPS_PACR_WP7_MASK                       (0x2U)
#define AIPS_PACR_WP7_SHIFT                      (1U)
#define AIPS_PACR_WP7(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP7_SHIFT)) & AIPS_PACR_WP7_MASK)
#define AIPS_PACR_SP7_MASK                       (0x4U)
#define AIPS_PACR_SP7_SHIFT                      (2U)
#define AIPS_PACR_SP7(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP7_SHIFT)) & AIPS_PACR_SP7_MASK)
#define AIPS_PACR_TP6_MASK                       (0x10U)
#define AIPS_PACR_TP6_SHIFT                      (4U)
#define AIPS_PACR_TP6(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP6_SHIFT)) & AIPS_PACR_TP6_MASK)
#define AIPS_PACR_WP6_MASK                       (0x20U)
#define AIPS_PACR_WP6_SHIFT                      (5U)
#define AIPS_PACR_WP6(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP6_SHIFT)) & AIPS_PACR_WP6_MASK)
#define AIPS_PACR_SP6_MASK                       (0x40U)
#define AIPS_PACR_SP6_SHIFT                      (6U)
#define AIPS_PACR_SP6(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP6_SHIFT)) & AIPS_PACR_SP6_MASK)
#define AIPS_PACR_TP5_MASK                       (0x100U)
#define AIPS_PACR_TP5_SHIFT                      (8U)
#define AIPS_PACR_TP5(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP5_SHIFT)) & AIPS_PACR_TP5_MASK)
#define AIPS_PACR_WP5_MASK                       (0x200U)
#define AIPS_PACR_WP5_SHIFT                      (9U)
#define AIPS_PACR_WP5(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP5_SHIFT)) & AIPS_PACR_WP5_MASK)
#define AIPS_PACR_SP5_MASK                       (0x400U)
#define AIPS_PACR_SP5_SHIFT                      (10U)
#define AIPS_PACR_SP5(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP5_SHIFT)) & AIPS_PACR_SP5_MASK)
#define AIPS_PACR_TP4_MASK                       (0x1000U)
#define AIPS_PACR_TP4_SHIFT                      (12U)
#define AIPS_PACR_TP4(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP4_SHIFT)) & AIPS_PACR_TP4_MASK)
#define AIPS_PACR_WP4_MASK                       (0x2000U)
#define AIPS_PACR_WP4_SHIFT                      (13U)
#define AIPS_PACR_WP4(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP4_SHIFT)) & AIPS_PACR_WP4_MASK)
#define AIPS_PACR_SP4_MASK                       (0x4000U)
#define AIPS_PACR_SP4_SHIFT                      (14U)
#define AIPS_PACR_SP4(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP4_SHIFT)) & AIPS_PACR_SP4_MASK)
#define AIPS_PACR_TP3_MASK                       (0x10000U)
#define AIPS_PACR_TP3_SHIFT                      (16U)
#define AIPS_PACR_TP3(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP3_SHIFT)) & AIPS_PACR_TP3_MASK)
#define AIPS_PACR_WP3_MASK                       (0x20000U)
#define AIPS_PACR_WP3_SHIFT                      (17U)
#define AIPS_PACR_WP3(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP3_SHIFT)) & AIPS_PACR_WP3_MASK)
#define AIPS_PACR_SP3_MASK                       (0x40000U)
#define AIPS_PACR_SP3_SHIFT                      (18U)
#define AIPS_PACR_SP3(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP3_SHIFT)) & AIPS_PACR_SP3_MASK)
#define AIPS_PACR_TP2_MASK                       (0x100000U)
#define AIPS_PACR_TP2_SHIFT                      (20U)
#define AIPS_PACR_TP2(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP2_SHIFT)) & AIPS_PACR_TP2_MASK)
#define AIPS_PACR_WP2_MASK                       (0x200000U)
#define AIPS_PACR_WP2_SHIFT                      (21U)
#define AIPS_PACR_WP2(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP2_SHIFT)) & AIPS_PACR_WP2_MASK)
#define AIPS_PACR_SP2_MASK                       (0x400000U)
#define AIPS_PACR_SP2_SHIFT                      (22U)
#define AIPS_PACR_SP2(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP2_SHIFT)) & AIPS_PACR_SP2_MASK)
#define AIPS_PACR_TP1_MASK                       (0x1000000U)
#define AIPS_PACR_TP1_SHIFT                      (24U)
#define AIPS_PACR_TP1(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP1_SHIFT)) & AIPS_PACR_TP1_MASK)
#define AIPS_PACR_WP1_MASK                       (0x2000000U)
#define AIPS_PACR_WP1_SHIFT                      (25U)
#define AIPS_PACR_WP1(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP1_SHIFT)) & AIPS_PACR_WP1_MASK)
#define AIPS_PACR_SP1_MASK                       (0x4000000U)
#define AIPS_PACR_SP1_SHIFT                      (26U)
#define AIPS_PACR_SP1(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP1_SHIFT)) & AIPS_PACR_SP1_MASK)
#define AIPS_PACR_TP0_MASK                       (0x10000000U)
#define AIPS_PACR_TP0_SHIFT                      (28U)
#define AIPS_PACR_TP0(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_TP0_SHIFT)) & AIPS_PACR_TP0_MASK)
#define AIPS_PACR_WP0_MASK                       (0x20000000U)
#define AIPS_PACR_WP0_SHIFT                      (29U)
#define AIPS_PACR_WP0(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_WP0_SHIFT)) & AIPS_PACR_WP0_MASK)
#define AIPS_PACR_SP0_MASK                       (0x40000000U)
#define AIPS_PACR_SP0_SHIFT                      (30U)
#define AIPS_PACR_SP0(x)                         (((uint32_t)(((uint32_t)(x)) << AIPS_PACR_SP0_SHIFT)) & AIPS_PACR_SP0_MASK)

/* The count of AIPS_PACR */
#define AIPS_PACR_COUNT                          (4U)

/*! @name OPACR - Off-Platform Peripheral Access Control Register */
#define AIPS_OPACR_TP7_MASK                      (0x1U)
#define AIPS_OPACR_TP7_SHIFT                     (0U)
#define AIPS_OPACR_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP7_SHIFT)) & AIPS_OPACR_TP7_MASK)
#define AIPS_OPACR_WP7_MASK                      (0x2U)
#define AIPS_OPACR_WP7_SHIFT                     (1U)
#define AIPS_OPACR_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP7_SHIFT)) & AIPS_OPACR_WP7_MASK)
#define AIPS_OPACR_SP7_MASK                      (0x4U)
#define AIPS_OPACR_SP7_SHIFT                     (2U)
#define AIPS_OPACR_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP7_SHIFT)) & AIPS_OPACR_SP7_MASK)
#define AIPS_OPACR_TP6_MASK                      (0x10U)
#define AIPS_OPACR_TP6_SHIFT                     (4U)
#define AIPS_OPACR_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP6_SHIFT)) & AIPS_OPACR_TP6_MASK)
#define AIPS_OPACR_WP6_MASK                      (0x20U)
#define AIPS_OPACR_WP6_SHIFT                     (5U)
#define AIPS_OPACR_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP6_SHIFT)) & AIPS_OPACR_WP6_MASK)
#define AIPS_OPACR_SP6_MASK                      (0x40U)
#define AIPS_OPACR_SP6_SHIFT                     (6U)
#define AIPS_OPACR_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP6_SHIFT)) & AIPS_OPACR_SP6_MASK)
#define AIPS_OPACR_TP5_MASK                      (0x100U)
#define AIPS_OPACR_TP5_SHIFT                     (8U)
#define AIPS_OPACR_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP5_SHIFT)) & AIPS_OPACR_TP5_MASK)
#define AIPS_OPACR_WP5_MASK                      (0x200U)
#define AIPS_OPACR_WP5_SHIFT                     (9U)
#define AIPS_OPACR_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP5_SHIFT)) & AIPS_OPACR_WP5_MASK)
#define AIPS_OPACR_SP5_MASK                      (0x400U)
#define AIPS_OPACR_SP5_SHIFT                     (10U)
#define AIPS_OPACR_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP5_SHIFT)) & AIPS_OPACR_SP5_MASK)
#define AIPS_OPACR_TP4_MASK                      (0x1000U)
#define AIPS_OPACR_TP4_SHIFT                     (12U)
#define AIPS_OPACR_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP4_SHIFT)) & AIPS_OPACR_TP4_MASK)
#define AIPS_OPACR_WP4_MASK                      (0x2000U)
#define AIPS_OPACR_WP4_SHIFT                     (13U)
#define AIPS_OPACR_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP4_SHIFT)) & AIPS_OPACR_WP4_MASK)
#define AIPS_OPACR_SP4_MASK                      (0x4000U)
#define AIPS_OPACR_SP4_SHIFT                     (14U)
#define AIPS_OPACR_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP4_SHIFT)) & AIPS_OPACR_SP4_MASK)
#define AIPS_OPACR_TP3_MASK                      (0x10000U)
#define AIPS_OPACR_TP3_SHIFT                     (16U)
#define AIPS_OPACR_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP3_SHIFT)) & AIPS_OPACR_TP3_MASK)
#define AIPS_OPACR_WP3_MASK                      (0x20000U)
#define AIPS_OPACR_WP3_SHIFT                     (17U)
#define AIPS_OPACR_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP3_SHIFT)) & AIPS_OPACR_WP3_MASK)
#define AIPS_OPACR_SP3_MASK                      (0x40000U)
#define AIPS_OPACR_SP3_SHIFT                     (18U)
#define AIPS_OPACR_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP3_SHIFT)) & AIPS_OPACR_SP3_MASK)
#define AIPS_OPACR_TP2_MASK                      (0x100000U)
#define AIPS_OPACR_TP2_SHIFT                     (20U)
#define AIPS_OPACR_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP2_SHIFT)) & AIPS_OPACR_TP2_MASK)
#define AIPS_OPACR_WP2_MASK                      (0x200000U)
#define AIPS_OPACR_WP2_SHIFT                     (21U)
#define AIPS_OPACR_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP2_SHIFT)) & AIPS_OPACR_WP2_MASK)
#define AIPS_OPACR_SP2_MASK                      (0x400000U)
#define AIPS_OPACR_SP2_SHIFT                     (22U)
#define AIPS_OPACR_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP2_SHIFT)) & AIPS_OPACR_SP2_MASK)
#define AIPS_OPACR_TP1_MASK                      (0x1000000U)
#define AIPS_OPACR_TP1_SHIFT                     (24U)
#define AIPS_OPACR_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP1_SHIFT)) & AIPS_OPACR_TP1_MASK)
#define AIPS_OPACR_WP1_MASK                      (0x2000000U)
#define AIPS_OPACR_WP1_SHIFT                     (25U)
#define AIPS_OPACR_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP1_SHIFT)) & AIPS_OPACR_WP1_MASK)
#define AIPS_OPACR_SP1_MASK                      (0x4000000U)
#define AIPS_OPACR_SP1_SHIFT                     (26U)
#define AIPS_OPACR_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP1_SHIFT)) & AIPS_OPACR_SP1_MASK)
#define AIPS_OPACR_TP0_MASK                      (0x10000000U)
#define AIPS_OPACR_TP0_SHIFT                     (28U)
#define AIPS_OPACR_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP0_SHIFT)) & AIPS_OPACR_TP0_MASK)
#define AIPS_OPACR_WP0_MASK                      (0x20000000U)
#define AIPS_OPACR_WP0_SHIFT                     (29U)
#define AIPS_OPACR_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP0_SHIFT)) & AIPS_OPACR_WP0_MASK)
#define AIPS_OPACR_SP0_MASK                      (0x40000000U)
#define AIPS_OPACR_SP0_SHIFT                     (30U)
#define AIPS_OPACR_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP0_SHIFT)) & AIPS_OPACR_SP0_MASK)

/* The count of AIPS_OPACR */
#define AIPS_OPACR_COUNT                         (16U)

/*! @name PACRU - Peripheral Access Control Register */
#define AIPS_PACRU_TP1_MASK                      (0x1000000U)
#define AIPS_PACRU_TP1_SHIFT                     (24U)
#define AIPS_PACRU_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_TP1_SHIFT)) & AIPS_PACRU_TP1_MASK)
#define AIPS_PACRU_WP1_MASK                      (0x2000000U)
#define AIPS_PACRU_WP1_SHIFT                     (25U)
#define AIPS_PACRU_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_WP1_SHIFT)) & AIPS_PACRU_WP1_MASK)
#define AIPS_PACRU_SP1_MASK                      (0x4000000U)
#define AIPS_PACRU_SP1_SHIFT                     (26U)
#define AIPS_PACRU_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_SP1_SHIFT)) & AIPS_PACRU_SP1_MASK)
#define AIPS_PACRU_TP0_MASK                      (0x10000000U)
#define AIPS_PACRU_TP0_SHIFT                     (28U)
#define AIPS_PACRU_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_TP0_SHIFT)) & AIPS_PACRU_TP0_MASK)
#define AIPS_PACRU_WP0_MASK                      (0x20000000U)
#define AIPS_PACRU_WP0_SHIFT                     (29U)
#define AIPS_PACRU_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_WP0_SHIFT)) & AIPS_PACRU_WP0_MASK)
#define AIPS_PACRU_SP0_MASK                      (0x40000000U)
#define AIPS_PACRU_SP0_SHIFT                     (30U)
#define AIPS_PACRU_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRU_SP0_SHIFT)) & AIPS_PACRU_SP0_MASK)


/*!
 * @}
 */ /* end of group AIPS_Register_Masks */


/* AIPS - Peripheral instance base addresses */
/** Peripheral AIPS base address */
#define AIPS_BASE                                (0x40000000u)
/** Peripheral AIPS base pointer */
#define AIPS                                     ((AIPS_Type *)AIPS_BASE)
/** Array initializer of AIPS peripheral base addresses */
#define AIPS_BASE_ADDRS                          { AIPS_BASE }
/** Array initializer of AIPS peripheral base pointers */
#define AIPS_BASE_PTRS                           { AIPS }

/*!
 * @}
 */ /* end of group AIPS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint32_t C0;                                /**< CMP Control Register 0, offset: 0x0 */
  __IO uint32_t C1;                                /**< CMP Control Register 1, offset: 0x4 */
  __IO uint32_t C2;                                /**< CMP Control Register 2, offset: 0x8 */
} CMP_Type;

/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/*! @name C0 - CMP Control Register 0 */
#define CMP_C0_HYSTCTR_MASK                      (0x3U)
#define CMP_C0_HYSTCTR_SHIFT                     (0U)
#define CMP_C0_HYSTCTR(x)                        (((uint32_t)(((uint32_t)(x)) << CMP_C0_HYSTCTR_SHIFT)) & CMP_C0_HYSTCTR_MASK)
#define CMP_C0_OFFSET_MASK                       (0x4U)
#define CMP_C0_OFFSET_SHIFT                      (2U)
#define CMP_C0_OFFSET(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C0_OFFSET_SHIFT)) & CMP_C0_OFFSET_MASK)
#define CMP_C0_FILTER_CNT_MASK                   (0x70U)
#define CMP_C0_FILTER_CNT_SHIFT                  (4U)
#define CMP_C0_FILTER_CNT(x)                     (((uint32_t)(((uint32_t)(x)) << CMP_C0_FILTER_CNT_SHIFT)) & CMP_C0_FILTER_CNT_MASK)
#define CMP_C0_EN_MASK                           (0x100U)
#define CMP_C0_EN_SHIFT                          (8U)
#define CMP_C0_EN(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_EN_SHIFT)) & CMP_C0_EN_MASK)
#define CMP_C0_OPE_MASK                          (0x200U)
#define CMP_C0_OPE_SHIFT                         (9U)
#define CMP_C0_OPE(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_OPE_SHIFT)) & CMP_C0_OPE_MASK)
#define CMP_C0_COS_MASK                          (0x400U)
#define CMP_C0_COS_SHIFT                         (10U)
#define CMP_C0_COS(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_COS_SHIFT)) & CMP_C0_COS_MASK)
#define CMP_C0_INVT_MASK                         (0x800U)
#define CMP_C0_INVT_SHIFT                        (11U)
#define CMP_C0_INVT(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C0_INVT_SHIFT)) & CMP_C0_INVT_MASK)
#define CMP_C0_PMODE_MASK                        (0x1000U)
#define CMP_C0_PMODE_SHIFT                       (12U)
#define CMP_C0_PMODE(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C0_PMODE_SHIFT)) & CMP_C0_PMODE_MASK)
#define CMP_C0_WE_MASK                           (0x4000U)
#define CMP_C0_WE_SHIFT                          (14U)
#define CMP_C0_WE(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_WE_SHIFT)) & CMP_C0_WE_MASK)
#define CMP_C0_SE_MASK                           (0x8000U)
#define CMP_C0_SE_SHIFT                          (15U)
#define CMP_C0_SE(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_SE_SHIFT)) & CMP_C0_SE_MASK)
#define CMP_C0_FPR_MASK                          (0xFF0000U)
#define CMP_C0_FPR_SHIFT                         (16U)
#define CMP_C0_FPR(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_FPR_SHIFT)) & CMP_C0_FPR_MASK)
#define CMP_C0_COUT_MASK                         (0x1000000U)
#define CMP_C0_COUT_SHIFT                        (24U)
#define CMP_C0_COUT(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C0_COUT_SHIFT)) & CMP_C0_COUT_MASK)
#define CMP_C0_CFF_MASK                          (0x2000000U)
#define CMP_C0_CFF_SHIFT                         (25U)
#define CMP_C0_CFF(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_CFF_SHIFT)) & CMP_C0_CFF_MASK)
#define CMP_C0_CFR_MASK                          (0x4000000U)
#define CMP_C0_CFR_SHIFT                         (26U)
#define CMP_C0_CFR(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_CFR_SHIFT)) & CMP_C0_CFR_MASK)
#define CMP_C0_IEF_MASK                          (0x8000000U)
#define CMP_C0_IEF_SHIFT                         (27U)
#define CMP_C0_IEF(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_IEF_SHIFT)) & CMP_C0_IEF_MASK)
#define CMP_C0_IER_MASK                          (0x10000000U)
#define CMP_C0_IER_SHIFT                         (28U)
#define CMP_C0_IER(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_IER_SHIFT)) & CMP_C0_IER_MASK)
#define CMP_C0_DMAEN_MASK                        (0x40000000U)
#define CMP_C0_DMAEN_SHIFT                       (30U)
#define CMP_C0_DMAEN(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C0_DMAEN_SHIFT)) & CMP_C0_DMAEN_MASK)

/*! @name C1 - CMP Control Register 1 */
#define CMP_C1_VOSEL_MASK                        (0xFFU)
#define CMP_C1_VOSEL_SHIFT                       (0U)
#define CMP_C1_VOSEL(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_VOSEL_SHIFT)) & CMP_C1_VOSEL_MASK)
#define CMP_C1_MSEL_MASK                         (0x700U)
#define CMP_C1_MSEL_SHIFT                        (8U)
#define CMP_C1_MSEL(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_MSEL_SHIFT)) & CMP_C1_MSEL_MASK)
#define CMP_C1_PSEL_MASK                         (0x3800U)
#define CMP_C1_PSEL_SHIFT                        (11U)
#define CMP_C1_PSEL(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_PSEL_SHIFT)) & CMP_C1_PSEL_MASK)
#define CMP_C1_VRSEL_MASK                        (0x4000U)
#define CMP_C1_VRSEL_SHIFT                       (14U)
#define CMP_C1_VRSEL(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_VRSEL_SHIFT)) & CMP_C1_VRSEL_MASK)
#define CMP_C1_DACEN_MASK                        (0x8000U)
#define CMP_C1_DACEN_SHIFT                       (15U)
#define CMP_C1_DACEN(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_DACEN_SHIFT)) & CMP_C1_DACEN_MASK)
#define CMP_C1_CHN0_MASK                         (0x10000U)
#define CMP_C1_CHN0_SHIFT                        (16U)
#define CMP_C1_CHN0(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN0_SHIFT)) & CMP_C1_CHN0_MASK)
#define CMP_C1_CHN1_MASK                         (0x20000U)
#define CMP_C1_CHN1_SHIFT                        (17U)
#define CMP_C1_CHN1(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN1_SHIFT)) & CMP_C1_CHN1_MASK)
#define CMP_C1_CHN2_MASK                         (0x40000U)
#define CMP_C1_CHN2_SHIFT                        (18U)
#define CMP_C1_CHN2(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN2_SHIFT)) & CMP_C1_CHN2_MASK)
#define CMP_C1_CHN3_MASK                         (0x80000U)
#define CMP_C1_CHN3_SHIFT                        (19U)
#define CMP_C1_CHN3(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN3_SHIFT)) & CMP_C1_CHN3_MASK)
#define CMP_C1_CHN4_MASK                         (0x100000U)
#define CMP_C1_CHN4_SHIFT                        (20U)
#define CMP_C1_CHN4(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN4_SHIFT)) & CMP_C1_CHN4_MASK)
#define CMP_C1_CHN5_MASK                         (0x200000U)
#define CMP_C1_CHN5_SHIFT                        (21U)
#define CMP_C1_CHN5(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN5_SHIFT)) & CMP_C1_CHN5_MASK)
#define CMP_C1_CHN6_MASK                         (0x400000U)
#define CMP_C1_CHN6_SHIFT                        (22U)
#define CMP_C1_CHN6(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN6_SHIFT)) & CMP_C1_CHN6_MASK)
#define CMP_C1_CHN7_MASK                         (0x800000U)
#define CMP_C1_CHN7_SHIFT                        (23U)
#define CMP_C1_CHN7(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN7_SHIFT)) & CMP_C1_CHN7_MASK)
#define CMP_C1_INNSEL_MASK                       (0x3000000U)
#define CMP_C1_INNSEL_SHIFT                      (24U)
#define CMP_C1_INNSEL(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C1_INNSEL_SHIFT)) & CMP_C1_INNSEL_MASK)
#define CMP_C1_INPSEL_MASK                       (0x18000000U)
#define CMP_C1_INPSEL_SHIFT                      (27U)
#define CMP_C1_INPSEL(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C1_INPSEL_SHIFT)) & CMP_C1_INPSEL_MASK)
#define CMP_C1_DACOE_MASK                        (0x20000000U)
#define CMP_C1_DACOE_SHIFT                       (29U)
#define CMP_C1_DACOE(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_DACOE_SHIFT)) & CMP_C1_DACOE_MASK)

/*! @name C2 - CMP Control Register 2 */
#define CMP_C2_ACOn_MASK                         (0xFFU)
#define CMP_C2_ACOn_SHIFT                        (0U)
#define CMP_C2_ACOn(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_ACOn_SHIFT)) & CMP_C2_ACOn_MASK)
#define CMP_C2_INITMOD_MASK                      (0x3F00U)
#define CMP_C2_INITMOD_SHIFT                     (8U)
#define CMP_C2_INITMOD(x)                        (((uint32_t)(((uint32_t)(x)) << CMP_C2_INITMOD_SHIFT)) & CMP_C2_INITMOD_MASK)
#define CMP_C2_NSAM_MASK                         (0xC000U)
#define CMP_C2_NSAM_SHIFT                        (14U)
#define CMP_C2_NSAM(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_NSAM_SHIFT)) & CMP_C2_NSAM_MASK)
#define CMP_C2_CH0F_MASK                         (0x10000U)
#define CMP_C2_CH0F_SHIFT                        (16U)
#define CMP_C2_CH0F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH0F_SHIFT)) & CMP_C2_CH0F_MASK)
#define CMP_C2_CH1F_MASK                         (0x20000U)
#define CMP_C2_CH1F_SHIFT                        (17U)
#define CMP_C2_CH1F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH1F_SHIFT)) & CMP_C2_CH1F_MASK)
#define CMP_C2_CH2F_MASK                         (0x40000U)
#define CMP_C2_CH2F_SHIFT                        (18U)
#define CMP_C2_CH2F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH2F_SHIFT)) & CMP_C2_CH2F_MASK)
#define CMP_C2_CH3F_MASK                         (0x80000U)
#define CMP_C2_CH3F_SHIFT                        (19U)
#define CMP_C2_CH3F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH3F_SHIFT)) & CMP_C2_CH3F_MASK)
#define CMP_C2_CH4F_MASK                         (0x100000U)
#define CMP_C2_CH4F_SHIFT                        (20U)
#define CMP_C2_CH4F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH4F_SHIFT)) & CMP_C2_CH4F_MASK)
#define CMP_C2_CH5F_MASK                         (0x200000U)
#define CMP_C2_CH5F_SHIFT                        (21U)
#define CMP_C2_CH5F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH5F_SHIFT)) & CMP_C2_CH5F_MASK)
#define CMP_C2_CH6F_MASK                         (0x400000U)
#define CMP_C2_CH6F_SHIFT                        (22U)
#define CMP_C2_CH6F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH6F_SHIFT)) & CMP_C2_CH6F_MASK)
#define CMP_C2_CH7F_MASK                         (0x800000U)
#define CMP_C2_CH7F_SHIFT                        (23U)
#define CMP_C2_CH7F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH7F_SHIFT)) & CMP_C2_CH7F_MASK)
#define CMP_C2_FXMXCH_MASK                       (0xE000000U)
#define CMP_C2_FXMXCH_SHIFT                      (25U)
#define CMP_C2_FXMXCH(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C2_FXMXCH_SHIFT)) & CMP_C2_FXMXCH_MASK)
#define CMP_C2_FXMP_MASK                         (0x20000000U)
#define CMP_C2_FXMP_SHIFT                        (29U)
#define CMP_C2_FXMP(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_FXMP_SHIFT)) & CMP_C2_FXMP_MASK)
#define CMP_C2_RRIE_MASK                         (0x40000000U)
#define CMP_C2_RRIE_SHIFT                        (30U)
#define CMP_C2_RRIE(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_RRIE_SHIFT)) & CMP_C2_RRIE_MASK)
#define CMP_C2_RRE_MASK                          (0x80000000U)
#define CMP_C2_RRE_SHIFT                         (31U)
#define CMP_C2_RRE(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C2_RRE_SHIFT)) & CMP_C2_RRE_MASK)


/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
/** Peripheral CMP1 base address */
#define CMP1_BASE                                (0x40074000u)
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
/** Peripheral DMA base address */
#define DMA_BASE                                 (0x40008000u)
/** Peripheral DMA base pointer */
#define DMA0                                     ((DMA_Type *)DMA_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_CHN_IRQS                             { DMA0_04_IRQn, DMA0_15_IRQn, DMA0_26_IRQn, DMA0_37_IRQn, DMA0_04_IRQn, DMA0_15_IRQn, DMA0_26_IRQn, DMA0_37_IRQn }
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
       uint8_t RESERVED_0[1];
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
  __IO uint32_t SHIFTCTL[4];                       /**< Shifter Control N Register, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_4[112];
  __IO uint32_t SHIFTCFG[4];                       /**< Shifter Configuration N Register, array offset: 0x100, array step: 0x4 */
       uint8_t RESERVED_5[240];
  __IO uint32_t SHIFTBUF[4];                       /**< Shifter Buffer N Register, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_6[112];
  __IO uint32_t SHIFTBUFBIS[4];                    /**< Shifter Buffer N Bit Swapped Register, array offset: 0x280, array step: 0x4 */
       uint8_t RESERVED_7[112];
  __IO uint32_t SHIFTBUFBYS[4];                    /**< Shifter Buffer N Byte Swapped Register, array offset: 0x300, array step: 0x4 */
       uint8_t RESERVED_8[112];
  __IO uint32_t SHIFTBUFBBS[4];                    /**< Shifter Buffer N Bit Byte Swapped Register, array offset: 0x380, array step: 0x4 */
       uint8_t RESERVED_9[112];
  __IO uint32_t TIMCTL[4];                         /**< Timer Control N Register, array offset: 0x400, array step: 0x4 */
       uint8_t RESERVED_10[112];
  __IO uint32_t TIMCFG[4];                         /**< Timer Configuration N Register, array offset: 0x480, array step: 0x4 */
       uint8_t RESERVED_11[112];
  __IO uint32_t TIMCMP[4];                         /**< Timer Compare N Register, array offset: 0x500, array step: 0x4 */
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
#define FLEXIO_PIN_PDI_MASK                      (0xFFU)
#define FLEXIO_PIN_PDI_SHIFT                     (0U)
#define FLEXIO_PIN_PDI(x)                        (((uint32_t)(((uint32_t)(x)) << FLEXIO_PIN_PDI_SHIFT)) & FLEXIO_PIN_PDI_MASK)

/*! @name SHIFTSTAT - Shifter Status Register */
#define FLEXIO_SHIFTSTAT_SSF_MASK                (0xFU)
#define FLEXIO_SHIFTSTAT_SSF_SHIFT               (0U)
#define FLEXIO_SHIFTSTAT_SSF(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSTAT_SSF_SHIFT)) & FLEXIO_SHIFTSTAT_SSF_MASK)

/*! @name SHIFTERR - Shifter Error Register */
#define FLEXIO_SHIFTERR_SEF_MASK                 (0xFU)
#define FLEXIO_SHIFTERR_SEF_SHIFT                (0U)
#define FLEXIO_SHIFTERR_SEF(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTERR_SEF_SHIFT)) & FLEXIO_SHIFTERR_SEF_MASK)

/*! @name TIMSTAT - Timer Status Register */
#define FLEXIO_TIMSTAT_TSF_MASK                  (0xFU)
#define FLEXIO_TIMSTAT_TSF_SHIFT                 (0U)
#define FLEXIO_TIMSTAT_TSF(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMSTAT_TSF_SHIFT)) & FLEXIO_TIMSTAT_TSF_MASK)

/*! @name SHIFTSIEN - Shifter Status Interrupt Enable */
#define FLEXIO_SHIFTSIEN_SSIE_MASK               (0xFU)
#define FLEXIO_SHIFTSIEN_SSIE_SHIFT              (0U)
#define FLEXIO_SHIFTSIEN_SSIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSIEN_SSIE_SHIFT)) & FLEXIO_SHIFTSIEN_SSIE_MASK)

/*! @name SHIFTEIEN - Shifter Error Interrupt Enable */
#define FLEXIO_SHIFTEIEN_SEIE_MASK               (0xFU)
#define FLEXIO_SHIFTEIEN_SEIE_SHIFT              (0U)
#define FLEXIO_SHIFTEIEN_SEIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTEIEN_SEIE_SHIFT)) & FLEXIO_SHIFTEIEN_SEIE_MASK)

/*! @name TIMIEN - Timer Interrupt Enable Register */
#define FLEXIO_TIMIEN_TEIE_MASK                  (0xFU)
#define FLEXIO_TIMIEN_TEIE_SHIFT                 (0U)
#define FLEXIO_TIMIEN_TEIE(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMIEN_TEIE_SHIFT)) & FLEXIO_TIMIEN_TEIE_MASK)

/*! @name SHIFTSDEN - Shifter Status DMA Enable */
#define FLEXIO_SHIFTSDEN_SSDE_MASK               (0xFU)
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
#define FLEXIO_SHIFTCTL_PINSEL_MASK              (0x700U)
#define FLEXIO_SHIFTCTL_PINSEL_SHIFT             (8U)
#define FLEXIO_SHIFTCTL_PINSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINSEL_SHIFT)) & FLEXIO_SHIFTCTL_PINSEL_MASK)
#define FLEXIO_SHIFTCTL_PINCFG_MASK              (0x30000U)
#define FLEXIO_SHIFTCTL_PINCFG_SHIFT             (16U)
#define FLEXIO_SHIFTCTL_PINCFG(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINCFG_SHIFT)) & FLEXIO_SHIFTCTL_PINCFG_MASK)
#define FLEXIO_SHIFTCTL_TIMPOL_MASK              (0x800000U)
#define FLEXIO_SHIFTCTL_TIMPOL_SHIFT             (23U)
#define FLEXIO_SHIFTCTL_TIMPOL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMPOL_SHIFT)) & FLEXIO_SHIFTCTL_TIMPOL_MASK)
#define FLEXIO_SHIFTCTL_TIMSEL_MASK              (0x3000000U)
#define FLEXIO_SHIFTCTL_TIMSEL_SHIFT             (24U)
#define FLEXIO_SHIFTCTL_TIMSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMSEL_SHIFT)) & FLEXIO_SHIFTCTL_TIMSEL_MASK)

/* The count of FLEXIO_SHIFTCTL */
#define FLEXIO_SHIFTCTL_COUNT                    (4U)

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

/* The count of FLEXIO_SHIFTCFG */
#define FLEXIO_SHIFTCFG_COUNT                    (4U)

/*! @name SHIFTBUF - Shifter Buffer N Register */
#define FLEXIO_SHIFTBUF_SHIFTBUF_MASK            (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT           (0U)
#define FLEXIO_SHIFTBUF_SHIFTBUF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT)) & FLEXIO_SHIFTBUF_SHIFTBUF_MASK)

/* The count of FLEXIO_SHIFTBUF */
#define FLEXIO_SHIFTBUF_COUNT                    (4U)

/*! @name SHIFTBUFBIS - Shifter Buffer N Bit Swapped Register */
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT)) & FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK)

/* The count of FLEXIO_SHIFTBUFBIS */
#define FLEXIO_SHIFTBUFBIS_COUNT                 (4U)

/*! @name SHIFTBUFBYS - Shifter Buffer N Byte Swapped Register */
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT)) & FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK)

/* The count of FLEXIO_SHIFTBUFBYS */
#define FLEXIO_SHIFTBUFBYS_COUNT                 (4U)

/*! @name SHIFTBUFBBS - Shifter Buffer N Bit Byte Swapped Register */
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT)) & FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK)

/* The count of FLEXIO_SHIFTBUFBBS */
#define FLEXIO_SHIFTBUFBBS_COUNT                 (4U)

/*! @name TIMCTL - Timer Control N Register */
#define FLEXIO_TIMCTL_TIMOD_MASK                 (0x3U)
#define FLEXIO_TIMCTL_TIMOD_SHIFT                (0U)
#define FLEXIO_TIMCTL_TIMOD(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TIMOD_SHIFT)) & FLEXIO_TIMCTL_TIMOD_MASK)
#define FLEXIO_TIMCTL_PINPOL_MASK                (0x80U)
#define FLEXIO_TIMCTL_PINPOL_SHIFT               (7U)
#define FLEXIO_TIMCTL_PINPOL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINPOL_SHIFT)) & FLEXIO_TIMCTL_PINPOL_MASK)
#define FLEXIO_TIMCTL_PINSEL_MASK                (0x700U)
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
#define FLEXIO_TIMCTL_TRGSEL_MASK                (0xF000000U)
#define FLEXIO_TIMCTL_TRGSEL_SHIFT               (24U)
#define FLEXIO_TIMCTL_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSEL_SHIFT)) & FLEXIO_TIMCTL_TRGSEL_MASK)

/* The count of FLEXIO_TIMCTL */
#define FLEXIO_TIMCTL_COUNT                      (4U)

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
#define FLEXIO_TIMCFG_COUNT                      (4U)

/*! @name TIMCMP - Timer Compare N Register */
#define FLEXIO_TIMCMP_CMP_MASK                   (0xFFFFU)
#define FLEXIO_TIMCMP_CMP_SHIFT                  (0U)
#define FLEXIO_TIMCMP_CMP(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCMP_CMP_SHIFT)) & FLEXIO_TIMCMP_CMP_MASK)

/* The count of FLEXIO_TIMCMP */
#define FLEXIO_TIMCMP_COUNT                      (4U)


/*!
 * @}
 */ /* end of group FLEXIO_Register_Masks */


/* FLEXIO - Peripheral instance base addresses */
/** Peripheral FLEXIO base address */
#define FLEXIO_BASE                              (0x4005A000u)
/** Peripheral FLEXIO base pointer */
#define FLEXIO                                   ((FLEXIO_Type *)FLEXIO_BASE)
/** Array initializer of FLEXIO peripheral base addresses */
#define FLEXIO_BASE_ADDRS                        { FLEXIO_BASE }
/** Array initializer of FLEXIO peripheral base pointers */
#define FLEXIO_BASE_PTRS                         { FLEXIO }
/** Interrupt vectors for the FLEXIO peripheral type */
#define FLEXIO_IRQS                              { FLEXIO_IRQn }

/*!
 * @}
 */ /* end of group FLEXIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFE Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Peripheral_Access_Layer FTFE Peripheral Access Layer
 * @{
 */

/** FTFE - Register Layout Typedef */
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
       uint8_t RESERVED_0[2];
  __IO uint8_t FEPROT;                             /**< EEPROM Protection Register, offset: 0x16 */
  __IO uint8_t FDPROT;                             /**< Data Flash Protection Register, offset: 0x17 */
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
       uint8_t RESERVED_2[2];
  __IO uint8_t FERSTAT;                            /**< Flash Error Status Register, offset: 0x2E */
  __IO uint8_t FERCNFG;                            /**< Flash Error Configuration Register, offset: 0x2F */
} FTFE_Type;

/* ----------------------------------------------------------------------------
   -- FTFE Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Register_Masks FTFE Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
#define FTFE_FSTAT_MGSTAT0_MASK                  (0x1U)
#define FTFE_FSTAT_MGSTAT0_SHIFT                 (0U)
#define FTFE_FSTAT_MGSTAT0(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_MGSTAT0_SHIFT)) & FTFE_FSTAT_MGSTAT0_MASK)
#define FTFE_FSTAT_FPVIOL_MASK                   (0x10U)
#define FTFE_FSTAT_FPVIOL_SHIFT                  (4U)
#define FTFE_FSTAT_FPVIOL(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_FPVIOL_SHIFT)) & FTFE_FSTAT_FPVIOL_MASK)
#define FTFE_FSTAT_ACCERR_MASK                   (0x20U)
#define FTFE_FSTAT_ACCERR_SHIFT                  (5U)
#define FTFE_FSTAT_ACCERR(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_ACCERR_SHIFT)) & FTFE_FSTAT_ACCERR_MASK)
#define FTFE_FSTAT_RDCOLERR_MASK                 (0x40U)
#define FTFE_FSTAT_RDCOLERR_SHIFT                (6U)
#define FTFE_FSTAT_RDCOLERR(x)                   (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_RDCOLERR_SHIFT)) & FTFE_FSTAT_RDCOLERR_MASK)
#define FTFE_FSTAT_CCIF_MASK                     (0x80U)
#define FTFE_FSTAT_CCIF_SHIFT                    (7U)
#define FTFE_FSTAT_CCIF(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_CCIF_SHIFT)) & FTFE_FSTAT_CCIF_MASK)

/*! @name FCNFG - Flash Configuration Register */
#define FTFE_FCNFG_EEERDY_MASK                   (0x1U)
#define FTFE_FCNFG_EEERDY_SHIFT                  (0U)
#define FTFE_FCNFG_EEERDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_EEERDY_SHIFT)) & FTFE_FCNFG_EEERDY_MASK)
#define FTFE_FCNFG_RAMRDY_MASK                   (0x2U)
#define FTFE_FCNFG_RAMRDY_SHIFT                  (1U)
#define FTFE_FCNFG_RAMRDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RAMRDY_SHIFT)) & FTFE_FCNFG_RAMRDY_MASK)
#define FTFE_FCNFG_PFLSH_MASK                    (0x4U)
#define FTFE_FCNFG_PFLSH_SHIFT                   (2U)
#define FTFE_FCNFG_PFLSH(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_PFLSH_SHIFT)) & FTFE_FCNFG_PFLSH_MASK)
#define FTFE_FCNFG_ERSSUSP_MASK                  (0x10U)
#define FTFE_FCNFG_ERSSUSP_SHIFT                 (4U)
#define FTFE_FCNFG_ERSSUSP(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSSUSP_SHIFT)) & FTFE_FCNFG_ERSSUSP_MASK)
#define FTFE_FCNFG_ERSAREQ_MASK                  (0x20U)
#define FTFE_FCNFG_ERSAREQ_SHIFT                 (5U)
#define FTFE_FCNFG_ERSAREQ(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSAREQ_SHIFT)) & FTFE_FCNFG_ERSAREQ_MASK)
#define FTFE_FCNFG_RDCOLLIE_MASK                 (0x40U)
#define FTFE_FCNFG_RDCOLLIE_SHIFT                (6U)
#define FTFE_FCNFG_RDCOLLIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RDCOLLIE_SHIFT)) & FTFE_FCNFG_RDCOLLIE_MASK)
#define FTFE_FCNFG_CCIE_MASK                     (0x80U)
#define FTFE_FCNFG_CCIE_SHIFT                    (7U)
#define FTFE_FCNFG_CCIE(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_CCIE_SHIFT)) & FTFE_FCNFG_CCIE_MASK)

/*! @name FSEC - Flash Security Register */
#define FTFE_FSEC_SEC_MASK                       (0x3U)
#define FTFE_FSEC_SEC_SHIFT                      (0U)
#define FTFE_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_SEC_SHIFT)) & FTFE_FSEC_SEC_MASK)
#define FTFE_FSEC_FSLACC_MASK                    (0xCU)
#define FTFE_FSEC_FSLACC_SHIFT                   (2U)
#define FTFE_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_FSLACC_SHIFT)) & FTFE_FSEC_FSLACC_MASK)
#define FTFE_FSEC_MEEN_MASK                      (0x30U)
#define FTFE_FSEC_MEEN_SHIFT                     (4U)
#define FTFE_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_MEEN_SHIFT)) & FTFE_FSEC_MEEN_MASK)
#define FTFE_FSEC_KEYEN_MASK                     (0xC0U)
#define FTFE_FSEC_KEYEN_SHIFT                    (6U)
#define FTFE_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_KEYEN_SHIFT)) & FTFE_FSEC_KEYEN_MASK)

/*! @name FOPT - Flash Option Register */
#define FTFE_FOPT_OPT_MASK                       (0xFFU)
#define FTFE_FOPT_OPT_SHIFT                      (0U)
#define FTFE_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x)) << FTFE_FOPT_OPT_SHIFT)) & FTFE_FOPT_OPT_MASK)

/*! @name FCCOB3 - Flash Common Command Object Registers */
#define FTFE_FCCOB3_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB3_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB3_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB3_CCOBn_SHIFT)) & FTFE_FCCOB3_CCOBn_MASK)

/*! @name FCCOB2 - Flash Common Command Object Registers */
#define FTFE_FCCOB2_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB2_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB2_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB2_CCOBn_SHIFT)) & FTFE_FCCOB2_CCOBn_MASK)

/*! @name FCCOB1 - Flash Common Command Object Registers */
#define FTFE_FCCOB1_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB1_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB1_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB1_CCOBn_SHIFT)) & FTFE_FCCOB1_CCOBn_MASK)

/*! @name FCCOB0 - Flash Common Command Object Registers */
#define FTFE_FCCOB0_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB0_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB0_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB0_CCOBn_SHIFT)) & FTFE_FCCOB0_CCOBn_MASK)

/*! @name FCCOB7 - Flash Common Command Object Registers */
#define FTFE_FCCOB7_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB7_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB7_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB7_CCOBn_SHIFT)) & FTFE_FCCOB7_CCOBn_MASK)

/*! @name FCCOB6 - Flash Common Command Object Registers */
#define FTFE_FCCOB6_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB6_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB6_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB6_CCOBn_SHIFT)) & FTFE_FCCOB6_CCOBn_MASK)

/*! @name FCCOB5 - Flash Common Command Object Registers */
#define FTFE_FCCOB5_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB5_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB5_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB5_CCOBn_SHIFT)) & FTFE_FCCOB5_CCOBn_MASK)

/*! @name FCCOB4 - Flash Common Command Object Registers */
#define FTFE_FCCOB4_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB4_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB4_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB4_CCOBn_SHIFT)) & FTFE_FCCOB4_CCOBn_MASK)

/*! @name FCCOBB - Flash Common Command Object Registers */
#define FTFE_FCCOBB_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOBB_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOBB_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBB_CCOBn_SHIFT)) & FTFE_FCCOBB_CCOBn_MASK)

/*! @name FCCOBA - Flash Common Command Object Registers */
#define FTFE_FCCOBA_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOBA_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOBA_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBA_CCOBn_SHIFT)) & FTFE_FCCOBA_CCOBn_MASK)

/*! @name FCCOB9 - Flash Common Command Object Registers */
#define FTFE_FCCOB9_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB9_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB9_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB9_CCOBn_SHIFT)) & FTFE_FCCOB9_CCOBn_MASK)

/*! @name FCCOB8 - Flash Common Command Object Registers */
#define FTFE_FCCOB8_CCOBn_MASK                   (0xFFU)
#define FTFE_FCCOB8_CCOBn_SHIFT                  (0U)
#define FTFE_FCCOB8_CCOBn(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB8_CCOBn_SHIFT)) & FTFE_FCCOB8_CCOBn_MASK)

/*! @name FPROT3 - Program Flash Protection Registers */
#define FTFE_FPROT3_PROT_MASK                    (0xFFU)
#define FTFE_FPROT3_PROT_SHIFT                   (0U)
#define FTFE_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT3_PROT_SHIFT)) & FTFE_FPROT3_PROT_MASK)

/*! @name FPROT2 - Program Flash Protection Registers */
#define FTFE_FPROT2_PROT_MASK                    (0xFFU)
#define FTFE_FPROT2_PROT_SHIFT                   (0U)
#define FTFE_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT2_PROT_SHIFT)) & FTFE_FPROT2_PROT_MASK)

/*! @name FPROT1 - Program Flash Protection Registers */
#define FTFE_FPROT1_PROT_MASK                    (0xFFU)
#define FTFE_FPROT1_PROT_SHIFT                   (0U)
#define FTFE_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT1_PROT_SHIFT)) & FTFE_FPROT1_PROT_MASK)

/*! @name FPROT0 - Program Flash Protection Registers */
#define FTFE_FPROT0_PROT_MASK                    (0xFFU)
#define FTFE_FPROT0_PROT_SHIFT                   (0U)
#define FTFE_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT0_PROT_SHIFT)) & FTFE_FPROT0_PROT_MASK)

/*! @name FEPROT - EEPROM Protection Register */
#define FTFE_FEPROT_EPROT_MASK                   (0xFFU)
#define FTFE_FEPROT_EPROT_SHIFT                  (0U)
#define FTFE_FEPROT_EPROT(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FEPROT_EPROT_SHIFT)) & FTFE_FEPROT_EPROT_MASK)

/*! @name FDPROT - Data Flash Protection Register */
#define FTFE_FDPROT_DPROT_MASK                   (0xFFU)
#define FTFE_FDPROT_DPROT_SHIFT                  (0U)
#define FTFE_FDPROT_DPROT(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FDPROT_DPROT_SHIFT)) & FTFE_FDPROT_DPROT_MASK)

/*! @name XACCH3 - Execute-only Access Registers */
#define FTFE_XACCH3_XA_MASK                      (0xFFU)
#define FTFE_XACCH3_XA_SHIFT                     (0U)
#define FTFE_XACCH3_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCH3_XA_SHIFT)) & FTFE_XACCH3_XA_MASK)

/*! @name XACCH2 - Execute-only Access Registers */
#define FTFE_XACCH2_XA_MASK                      (0xFFU)
#define FTFE_XACCH2_XA_SHIFT                     (0U)
#define FTFE_XACCH2_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCH2_XA_SHIFT)) & FTFE_XACCH2_XA_MASK)

/*! @name XACCH1 - Execute-only Access Registers */
#define FTFE_XACCH1_XA_MASK                      (0xFFU)
#define FTFE_XACCH1_XA_SHIFT                     (0U)
#define FTFE_XACCH1_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCH1_XA_SHIFT)) & FTFE_XACCH1_XA_MASK)

/*! @name XACCH0 - Execute-only Access Registers */
#define FTFE_XACCH0_XA_MASK                      (0xFFU)
#define FTFE_XACCH0_XA_SHIFT                     (0U)
#define FTFE_XACCH0_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCH0_XA_SHIFT)) & FTFE_XACCH0_XA_MASK)

/*! @name XACCL3 - Execute-only Access Registers */
#define FTFE_XACCL3_XA_MASK                      (0xFFU)
#define FTFE_XACCL3_XA_SHIFT                     (0U)
#define FTFE_XACCL3_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCL3_XA_SHIFT)) & FTFE_XACCL3_XA_MASK)

/*! @name XACCL2 - Execute-only Access Registers */
#define FTFE_XACCL2_XA_MASK                      (0xFFU)
#define FTFE_XACCL2_XA_SHIFT                     (0U)
#define FTFE_XACCL2_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCL2_XA_SHIFT)) & FTFE_XACCL2_XA_MASK)

/*! @name XACCL1 - Execute-only Access Registers */
#define FTFE_XACCL1_XA_MASK                      (0xFFU)
#define FTFE_XACCL1_XA_SHIFT                     (0U)
#define FTFE_XACCL1_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCL1_XA_SHIFT)) & FTFE_XACCL1_XA_MASK)

/*! @name XACCL0 - Execute-only Access Registers */
#define FTFE_XACCL0_XA_MASK                      (0xFFU)
#define FTFE_XACCL0_XA_SHIFT                     (0U)
#define FTFE_XACCL0_XA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_XACCL0_XA_SHIFT)) & FTFE_XACCL0_XA_MASK)

/*! @name SACCH3 - Supervisor-only Access Registers */
#define FTFE_SACCH3_SA_MASK                      (0xFFU)
#define FTFE_SACCH3_SA_SHIFT                     (0U)
#define FTFE_SACCH3_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCH3_SA_SHIFT)) & FTFE_SACCH3_SA_MASK)

/*! @name SACCH2 - Supervisor-only Access Registers */
#define FTFE_SACCH2_SA_MASK                      (0xFFU)
#define FTFE_SACCH2_SA_SHIFT                     (0U)
#define FTFE_SACCH2_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCH2_SA_SHIFT)) & FTFE_SACCH2_SA_MASK)

/*! @name SACCH1 - Supervisor-only Access Registers */
#define FTFE_SACCH1_SA_MASK                      (0xFFU)
#define FTFE_SACCH1_SA_SHIFT                     (0U)
#define FTFE_SACCH1_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCH1_SA_SHIFT)) & FTFE_SACCH1_SA_MASK)

/*! @name SACCH0 - Supervisor-only Access Registers */
#define FTFE_SACCH0_SA_MASK                      (0xFFU)
#define FTFE_SACCH0_SA_SHIFT                     (0U)
#define FTFE_SACCH0_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCH0_SA_SHIFT)) & FTFE_SACCH0_SA_MASK)

/*! @name SACCL3 - Supervisor-only Access Registers */
#define FTFE_SACCL3_SA_MASK                      (0xFFU)
#define FTFE_SACCL3_SA_SHIFT                     (0U)
#define FTFE_SACCL3_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCL3_SA_SHIFT)) & FTFE_SACCL3_SA_MASK)

/*! @name SACCL2 - Supervisor-only Access Registers */
#define FTFE_SACCL2_SA_MASK                      (0xFFU)
#define FTFE_SACCL2_SA_SHIFT                     (0U)
#define FTFE_SACCL2_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCL2_SA_SHIFT)) & FTFE_SACCL2_SA_MASK)

/*! @name SACCL1 - Supervisor-only Access Registers */
#define FTFE_SACCL1_SA_MASK                      (0xFFU)
#define FTFE_SACCL1_SA_SHIFT                     (0U)
#define FTFE_SACCL1_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCL1_SA_SHIFT)) & FTFE_SACCL1_SA_MASK)

/*! @name SACCL0 - Supervisor-only Access Registers */
#define FTFE_SACCL0_SA_MASK                      (0xFFU)
#define FTFE_SACCL0_SA_SHIFT                     (0U)
#define FTFE_SACCL0_SA(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_SACCL0_SA_SHIFT)) & FTFE_SACCL0_SA_MASK)

/*! @name FACSS - Flash Access Segment Size Register */
#define FTFE_FACSS_SGSIZE_MASK                   (0xFFU)
#define FTFE_FACSS_SGSIZE_SHIFT                  (0U)
#define FTFE_FACSS_SGSIZE(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FACSS_SGSIZE_SHIFT)) & FTFE_FACSS_SGSIZE_MASK)

/*! @name FACSN - Flash Access Segment Number Register */
#define FTFE_FACSN_NUMSG_MASK                    (0xFFU)
#define FTFE_FACSN_NUMSG_SHIFT                   (0U)
#define FTFE_FACSN_NUMSG(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FACSN_NUMSG_SHIFT)) & FTFE_FACSN_NUMSG_MASK)

/*! @name FERSTAT - Flash Error Status Register */
#define FTFE_FERSTAT_DFDIF_MASK                  (0x2U)
#define FTFE_FERSTAT_DFDIF_SHIFT                 (1U)
#define FTFE_FERSTAT_DFDIF(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FERSTAT_DFDIF_SHIFT)) & FTFE_FERSTAT_DFDIF_MASK)

/*! @name FERCNFG - Flash Error Configuration Register */
#define FTFE_FERCNFG_DFDIE_MASK                  (0x2U)
#define FTFE_FERCNFG_DFDIE_SHIFT                 (1U)
#define FTFE_FERCNFG_DFDIE(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_DFDIE_SHIFT)) & FTFE_FERCNFG_DFDIE_MASK)
#define FTFE_FERCNFG_FDFD_MASK                   (0x20U)
#define FTFE_FERCNFG_FDFD_SHIFT                  (5U)
#define FTFE_FERCNFG_FDFD(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_FDFD_SHIFT)) & FTFE_FERCNFG_FDFD_MASK)


/*!
 * @}
 */ /* end of group FTFE_Register_Masks */


/* FTFE - Peripheral instance base addresses */
/** Peripheral FTFE base address */
#define FTFE_BASE                                (0x40020000u)
/** Peripheral FTFE base pointer */
#define FTFE                                     ((FTFE_Type *)FTFE_BASE)
/** Array initializer of FTFE peripheral base addresses */
#define FTFE_BASE_ADDRS                          { FTFE_BASE }
/** Array initializer of FTFE peripheral base pointers */
#define FTFE_BASE_PTRS                           { FTFE }
/** Interrupt vectors for the FTFE peripheral type */
#define FTFE_COMMAND_COMPLETE_IRQS               { FTFE_IRQn }
#define FTFE_READ_COLLISION_IRQS                 { FTFE_IRQn }

/*!
 * @}
 */ /* end of group FTFE_Peripheral_Access_Layer */


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
  __IO uint32_t HCR;                               /**< Half Cycle Register, offset: 0x9C */
       uint8_t RESERVED_0[352];
  __IO uint32_t MOD_MIRROR;                        /**< Modulo, offset: 0x200 */
  __IO uint32_t CV_MIRROR[8];                      /**< Channel (n) Value, array offset: 0x204, array step: 0x4 */
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
#define FTM_SC_RIE_MASK                          (0x40U)
#define FTM_SC_RIE_SHIFT                         (6U)
#define FTM_SC_RIE(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_RIE_SHIFT)) & FTM_SC_RIE_MASK)
#define FTM_SC_RF_MASK                           (0x80U)
#define FTM_SC_RF_SHIFT                          (7U)
#define FTM_SC_RF(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_RF_SHIFT)) & FTM_SC_RF_MASK)
#define FTM_SC_TOIE_MASK                         (0x100U)
#define FTM_SC_TOIE_SHIFT                        (8U)
#define FTM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOIE_SHIFT)) & FTM_SC_TOIE_MASK)
#define FTM_SC_TOF_MASK                          (0x200U)
#define FTM_SC_TOF_SHIFT                         (9U)
#define FTM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOF_SHIFT)) & FTM_SC_TOF_MASK)
#define FTM_SC_PWMEN0_MASK                       (0x10000U)
#define FTM_SC_PWMEN0_SHIFT                      (16U)
#define FTM_SC_PWMEN0(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN0_SHIFT)) & FTM_SC_PWMEN0_MASK)
#define FTM_SC_PWMEN1_MASK                       (0x20000U)
#define FTM_SC_PWMEN1_SHIFT                      (17U)
#define FTM_SC_PWMEN1(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN1_SHIFT)) & FTM_SC_PWMEN1_MASK)
#define FTM_SC_PWMEN2_MASK                       (0x40000U)
#define FTM_SC_PWMEN2_SHIFT                      (18U)
#define FTM_SC_PWMEN2(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN2_SHIFT)) & FTM_SC_PWMEN2_MASK)
#define FTM_SC_PWMEN3_MASK                       (0x80000U)
#define FTM_SC_PWMEN3_SHIFT                      (19U)
#define FTM_SC_PWMEN3(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN3_SHIFT)) & FTM_SC_PWMEN3_MASK)
#define FTM_SC_PWMEN4_MASK                       (0x100000U)
#define FTM_SC_PWMEN4_SHIFT                      (20U)
#define FTM_SC_PWMEN4(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN4_SHIFT)) & FTM_SC_PWMEN4_MASK)
#define FTM_SC_PWMEN5_MASK                       (0x200000U)
#define FTM_SC_PWMEN5_SHIFT                      (21U)
#define FTM_SC_PWMEN5(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN5_SHIFT)) & FTM_SC_PWMEN5_MASK)
#define FTM_SC_PWMEN6_MASK                       (0x400000U)
#define FTM_SC_PWMEN6_SHIFT                      (22U)
#define FTM_SC_PWMEN6(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN6_SHIFT)) & FTM_SC_PWMEN6_MASK)
#define FTM_SC_PWMEN7_MASK                       (0x800000U)
#define FTM_SC_PWMEN7_SHIFT                      (23U)
#define FTM_SC_PWMEN7(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN7_SHIFT)) & FTM_SC_PWMEN7_MASK)

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
#define FTM_CnSC_TRIGMODE_MASK                   (0x100U)
#define FTM_CnSC_TRIGMODE_SHIFT                  (8U)
#define FTM_CnSC_TRIGMODE(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_TRIGMODE_SHIFT)) & FTM_CnSC_TRIGMODE_MASK)
#define FTM_CnSC_CHIS_MASK                       (0x200U)
#define FTM_CnSC_CHIS_SHIFT                      (9U)
#define FTM_CnSC_CHIS(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIS_SHIFT)) & FTM_CnSC_CHIS_MASK)

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
#define FTM_EXTTRIG_CH6TRIG_MASK                 (0x100U)
#define FTM_EXTTRIG_CH6TRIG_SHIFT                (8U)
#define FTM_EXTTRIG_CH6TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH6TRIG_SHIFT)) & FTM_EXTTRIG_CH6TRIG_MASK)
#define FTM_EXTTRIG_CH7TRIG_MASK                 (0x200U)
#define FTM_EXTTRIG_CH7TRIG_SHIFT                (9U)
#define FTM_EXTTRIG_CH7TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH7TRIG_SHIFT)) & FTM_EXTTRIG_CH7TRIG_MASK)

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
#define FTM_FLTCTRL_FSTATE_MASK                  (0x8000U)
#define FTM_FLTCTRL_FSTATE_SHIFT                 (15U)
#define FTM_FLTCTRL_FSTATE(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FSTATE_SHIFT)) & FTM_FLTCTRL_FSTATE_MASK)

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
#define FTM_CONF_LDFQ_MASK                       (0x1FU)
#define FTM_CONF_LDFQ_SHIFT                      (0U)
#define FTM_CONF_LDFQ(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CONF_LDFQ_SHIFT)) & FTM_CONF_LDFQ_MASK)
#define FTM_CONF_BDMMODE_MASK                    (0xC0U)
#define FTM_CONF_BDMMODE_SHIFT                   (6U)
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_BDMMODE_SHIFT)) & FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     (0x200U)
#define FTM_CONF_GTBEEN_SHIFT                    (9U)
#define FTM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEEN_SHIFT)) & FTM_CONF_GTBEEN_MASK)
#define FTM_CONF_GTBEOUT_MASK                    (0x400U)
#define FTM_CONF_GTBEOUT_SHIFT                   (10U)
#define FTM_CONF_GTBEOUT(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEOUT_SHIFT)) & FTM_CONF_GTBEOUT_MASK)
#define FTM_CONF_ITRIGR_MASK                     (0x800U)
#define FTM_CONF_ITRIGR_SHIFT                    (11U)
#define FTM_CONF_ITRIGR(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_ITRIGR_SHIFT)) & FTM_CONF_ITRIGR_MASK)

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
#define FTM_PWMLOAD_HCSEL_MASK                   (0x100U)
#define FTM_PWMLOAD_HCSEL_SHIFT                  (8U)
#define FTM_PWMLOAD_HCSEL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_HCSEL_SHIFT)) & FTM_PWMLOAD_HCSEL_MASK)
#define FTM_PWMLOAD_LDOK_MASK                    (0x200U)
#define FTM_PWMLOAD_LDOK_SHIFT                   (9U)
#define FTM_PWMLOAD_LDOK(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_LDOK_SHIFT)) & FTM_PWMLOAD_LDOK_MASK)
#define FTM_PWMLOAD_GLEN_MASK                    (0x400U)
#define FTM_PWMLOAD_GLEN_SHIFT                   (10U)
#define FTM_PWMLOAD_GLEN(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLEN_SHIFT)) & FTM_PWMLOAD_GLEN_MASK)
#define FTM_PWMLOAD_GLDOK_MASK                   (0x800U)
#define FTM_PWMLOAD_GLDOK_SHIFT                  (11U)
#define FTM_PWMLOAD_GLDOK(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLDOK_SHIFT)) & FTM_PWMLOAD_GLDOK_MASK)

/*! @name HCR - Half Cycle Register */
#define FTM_HCR_HCVAL_MASK                       (0xFFFFU)
#define FTM_HCR_HCVAL_SHIFT                      (0U)
#define FTM_HCR_HCVAL(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_HCR_HCVAL_SHIFT)) & FTM_HCR_HCVAL_MASK)

/*! @name MOD_MIRROR - Modulo */
#define FTM_MOD_MIRROR_FRACMOD_MASK              (0xF800U)
#define FTM_MOD_MIRROR_FRACMOD_SHIFT             (11U)
#define FTM_MOD_MIRROR_FRACMOD(x)                (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_FRACMOD_SHIFT)) & FTM_MOD_MIRROR_FRACMOD_MASK)
#define FTM_MOD_MIRROR_MOD_MASK                  (0xFFFF0000U)
#define FTM_MOD_MIRROR_MOD_SHIFT                 (16U)
#define FTM_MOD_MIRROR_MOD(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_MOD_SHIFT)) & FTM_MOD_MIRROR_MOD_MASK)

/*! @name CV_MIRROR - Channel (n) Value */
#define FTM_CV_MIRROR_FRACVAL_MASK               (0xF800U)
#define FTM_CV_MIRROR_FRACVAL_SHIFT              (11U)
#define FTM_CV_MIRROR_FRACVAL(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_FRACVAL_SHIFT)) & FTM_CV_MIRROR_FRACVAL_MASK)
#define FTM_CV_MIRROR_VAL_MASK                   (0xFFFF0000U)
#define FTM_CV_MIRROR_VAL_SHIFT                  (16U)
#define FTM_CV_MIRROR_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_VAL_SHIFT)) & FTM_CV_MIRROR_VAL_MASK)

/* The count of FTM_CV_MIRROR */
#define FTM_CV_MIRROR_COUNT                      (8U)


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
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM2_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM2 }
/** Interrupt vectors for the FTM peripheral type */
#define FTM_IRQS                                 { FTM0_IRQn, FTM1_IRQn, FTM2_IRQn }

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
#define LPI2C0_BASE                              (0x40066000u)
/** Peripheral LPI2C0 base pointer */
#define LPI2C0                                   ((LPI2C_Type *)LPI2C0_BASE)
/** Peripheral LPI2C1 base address */
#define LPI2C1_BASE                              (0x40067000u)
/** Peripheral LPI2C1 base pointer */
#define LPI2C1                                   ((LPI2C_Type *)LPI2C1_BASE)
/** Array initializer of LPI2C peripheral base addresses */
#define LPI2C_BASE_ADDRS                         { LPI2C0_BASE, LPI2C1_BASE }
/** Array initializer of LPI2C peripheral base pointers */
#define LPI2C_BASE_PTRS                          { LPI2C0, LPI2C1 }
/** Interrupt vectors for the LPI2C peripheral type */
#define LPI2C_IRQS                               { LPI2C0_IRQn, LPI2C1_IRQn }

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
#define LPIT0_BASE                               (0x40037000u)
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
#define LPSPI0_BASE                              (0x4002C000u)
/** Peripheral LPSPI0 base pointer */
#define LPSPI0                                   ((LPSPI_Type *)LPSPI0_BASE)
/** Peripheral LPSPI1 base address */
#define LPSPI1_BASE                              (0x4002D000u)
/** Peripheral LPSPI1 base pointer */
#define LPSPI1                                   ((LPSPI_Type *)LPSPI1_BASE)
/** Array initializer of LPSPI peripheral base addresses */
#define LPSPI_BASE_ADDRS                         { LPSPI0_BASE, LPSPI1_BASE }
/** Array initializer of LPSPI peripheral base pointers */
#define LPSPI_BASE_PTRS                          { LPSPI0, LPSPI1 }
/** Interrupt vectors for the LPSPI peripheral type */
#define LPSPI_IRQS                               { LPSPI0_IRQn, LPSPI1_IRQn }

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
#define LPTMR0_BASE                              (0x40040000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_Type *)LPTMR0_BASE)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0 }
/** Interrupt vectors for the LPTMR peripheral type */
#define LPTMR_IRQS                               { PWT_LPTMR0_IRQn }

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
#define LPUART_BAUD_RIDMAE_MASK                  (0x100000U)
#define LPUART_BAUD_RIDMAE_SHIFT                 (20U)
#define LPUART_BAUD_RIDMAE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RIDMAE_SHIFT)) & LPUART_BAUD_RIDMAE_MASK)
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
#define LPUART_CTRL_M7_MASK                      (0x800U)
#define LPUART_CTRL_M7_SHIFT                     (11U)
#define LPUART_CTRL_M7(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M7_SHIFT)) & LPUART_CTRL_M7_MASK)
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
#define LPUART0_BASE                             (0x4006A000u)
/** Peripheral LPUART0 base pointer */
#define LPUART0                                  ((LPUART_Type *)LPUART0_BASE)
/** Peripheral LPUART1 base address */
#define LPUART1_BASE                             (0x4006B000u)
/** Peripheral LPUART1 base pointer */
#define LPUART1                                  ((LPUART_Type *)LPUART1_BASE)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE                             (0x4006C000u)
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
       uint8_t RESERVED_1[32];
  __IO uint32_t PID;                               /**< Process ID register, offset: 0x30 */
       uint8_t RESERVED_2[12];
  __IO uint32_t CPO;                               /**< Compute Operation Control Register, offset: 0x40 */
       uint8_t RESERVED_3[60];
  __IO uint32_t MATCR[1];                          /**< Master Attribute Configuration Register, array offset: 0x80, array step: 0x4 */
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

/*! @name MATCR - Master Attribute Configuration Register */
#define MCM_MATCR_ATC0_MASK                      (0x7U)
#define MCM_MATCR_ATC0_SHIFT                     (0U)
#define MCM_MATCR_ATC0(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_MATCR_ATC0_SHIFT)) & MCM_MATCR_ATC0_MASK)
#define MCM_MATCR_RO0_MASK                       (0x80U)
#define MCM_MATCR_RO0_SHIFT                      (7U)
#define MCM_MATCR_RO0(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_MATCR_RO0_SHIFT)) & MCM_MATCR_RO0_MASK)
#define MCM_MATCR_ATC2_MASK                      (0x70000U)
#define MCM_MATCR_ATC2_SHIFT                     (16U)
#define MCM_MATCR_ATC2(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_MATCR_ATC2_SHIFT)) & MCM_MATCR_ATC2_MASK)
#define MCM_MATCR_RO2_MASK                       (0x800000U)
#define MCM_MATCR_RO2_SHIFT                      (23U)
#define MCM_MATCR_RO2(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_MATCR_RO2_SHIFT)) & MCM_MATCR_RO2_MASK)

/* The count of MCM_MATCR */
#define MCM_MATCR_COUNT                          (1U)


/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xF0003000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_Type *)MCM_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }

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
/** Peripheral MMDVSQ base address */
#define MMDVSQ_BASE                              (0xF0004000u)
/** Peripheral MMDVSQ base pointer */
#define MMDVSQ                                   ((MMDVSQ_Type *)MMDVSQ_BASE)
/** Array initializer of MMDVSQ peripheral base addresses */
#define MMDVSQ_BASE_ADDRS                        { MMDVSQ_BASE }
/** Array initializer of MMDVSQ peripheral base pointers */
#define MMDVSQ_BASE_PTRS                         { MMDVSQ }

/*!
 * @}
 */ /* end of group MMDVSQ_Peripheral_Access_Layer */


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
#define MTB_AUTHSTAT_BIT2_MASK                   (0x4U)
#define MTB_AUTHSTAT_BIT2_SHIFT                  (2U)
#define MTB_AUTHSTAT_BIT2(x)                     (((uint32_t)(((uint32_t)(x)) << MTB_AUTHSTAT_BIT2_SHIFT)) & MTB_AUTHSTAT_BIT2_MASK)

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
/** Peripheral MTB base address */
#define MTB_BASE                                 (0xF0000000u)
/** Peripheral MTB base pointer */
#define MTB                                      ((MTB_Type *)MTB_BASE)
/** Array initializer of MTB peripheral base addresses */
#define MTB_BASE_ADDRS                           { MTB_BASE }
/** Array initializer of MTB peripheral base pointers */
#define MTB_BASE_PTRS                            { MTB }

/*!
 * @}
 */ /* end of group MTB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MTBDWT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTBDWT_Peripheral_Access_Layer MTBDWT Peripheral Access Layer
 * @{
 */

/** MTBDWT - Register Layout Typedef */
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
} MTBDWT_Type;

/* ----------------------------------------------------------------------------
   -- MTBDWT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MTBDWT_Register_Masks MTBDWT Register Masks
 * @{
 */

/*! @name CTRL - MTB DWT Control Register */
#define MTBDWT_CTRL_DWTCFGCTRL_MASK              (0xFFFFFFFU)
#define MTBDWT_CTRL_DWTCFGCTRL_SHIFT             (0U)
#define MTBDWT_CTRL_DWTCFGCTRL(x)                (((uint32_t)(((uint32_t)(x)) << MTBDWT_CTRL_DWTCFGCTRL_SHIFT)) & MTBDWT_CTRL_DWTCFGCTRL_MASK)
#define MTBDWT_CTRL_NUMCMP_MASK                  (0xF0000000U)
#define MTBDWT_CTRL_NUMCMP_SHIFT                 (28U)
#define MTBDWT_CTRL_NUMCMP(x)                    (((uint32_t)(((uint32_t)(x)) << MTBDWT_CTRL_NUMCMP_SHIFT)) & MTBDWT_CTRL_NUMCMP_MASK)

/*! @name COMP - MTB_DWT Comparator Register */
#define MTBDWT_COMP_COMP_MASK                    (0xFFFFFFFFU)
#define MTBDWT_COMP_COMP_SHIFT                   (0U)
#define MTBDWT_COMP_COMP(x)                      (((uint32_t)(((uint32_t)(x)) << MTBDWT_COMP_COMP_SHIFT)) & MTBDWT_COMP_COMP_MASK)

/* The count of MTBDWT_COMP */
#define MTBDWT_COMP_COUNT                        (2U)

/*! @name MASK - MTB_DWT Comparator Mask Register */
#define MTBDWT_MASK_MASK_MASK                    (0x1FU)
#define MTBDWT_MASK_MASK_SHIFT                   (0U)
#define MTBDWT_MASK_MASK(x)                      (((uint32_t)(((uint32_t)(x)) << MTBDWT_MASK_MASK_SHIFT)) & MTBDWT_MASK_MASK_MASK)

/* The count of MTBDWT_MASK */
#define MTBDWT_MASK_COUNT                        (2U)

/*! @name FCT - MTB_DWT Comparator Function Register 0..MTB_DWT Comparator Function Register 1 */
#define MTBDWT_FCT_FUNCTION_MASK                 (0xFU)
#define MTBDWT_FCT_FUNCTION_SHIFT                (0U)
#define MTBDWT_FCT_FUNCTION(x)                   (((uint32_t)(((uint32_t)(x)) << MTBDWT_FCT_FUNCTION_SHIFT)) & MTBDWT_FCT_FUNCTION_MASK)
#define MTBDWT_FCT_DATAVMATCH_MASK               (0x100U)
#define MTBDWT_FCT_DATAVMATCH_SHIFT              (8U)
#define MTBDWT_FCT_DATAVMATCH(x)                 (((uint32_t)(((uint32_t)(x)) << MTBDWT_FCT_DATAVMATCH_SHIFT)) & MTBDWT_FCT_DATAVMATCH_MASK)
#define MTBDWT_FCT_DATAVSIZE_MASK                (0xC00U)
#define MTBDWT_FCT_DATAVSIZE_SHIFT               (10U)
#define MTBDWT_FCT_DATAVSIZE(x)                  (((uint32_t)(((uint32_t)(x)) << MTBDWT_FCT_DATAVSIZE_SHIFT)) & MTBDWT_FCT_DATAVSIZE_MASK)
#define MTBDWT_FCT_DATAVADDR0_MASK               (0xF000U)
#define MTBDWT_FCT_DATAVADDR0_SHIFT              (12U)
#define MTBDWT_FCT_DATAVADDR0(x)                 (((uint32_t)(((uint32_t)(x)) << MTBDWT_FCT_DATAVADDR0_SHIFT)) & MTBDWT_FCT_DATAVADDR0_MASK)
#define MTBDWT_FCT_MATCHED_MASK                  (0x1000000U)
#define MTBDWT_FCT_MATCHED_SHIFT                 (24U)
#define MTBDWT_FCT_MATCHED(x)                    (((uint32_t)(((uint32_t)(x)) << MTBDWT_FCT_MATCHED_SHIFT)) & MTBDWT_FCT_MATCHED_MASK)

/* The count of MTBDWT_FCT */
#define MTBDWT_FCT_COUNT                         (2U)

/*! @name TBCTRL - MTB_DWT Trace Buffer Control Register */
#define MTBDWT_TBCTRL_ACOMP0_MASK                (0x1U)
#define MTBDWT_TBCTRL_ACOMP0_SHIFT               (0U)
#define MTBDWT_TBCTRL_ACOMP0(x)                  (((uint32_t)(((uint32_t)(x)) << MTBDWT_TBCTRL_ACOMP0_SHIFT)) & MTBDWT_TBCTRL_ACOMP0_MASK)
#define MTBDWT_TBCTRL_ACOMP1_MASK                (0x2U)
#define MTBDWT_TBCTRL_ACOMP1_SHIFT               (1U)
#define MTBDWT_TBCTRL_ACOMP1(x)                  (((uint32_t)(((uint32_t)(x)) << MTBDWT_TBCTRL_ACOMP1_SHIFT)) & MTBDWT_TBCTRL_ACOMP1_MASK)
#define MTBDWT_TBCTRL_NUMCOMP_MASK               (0xF0000000U)
#define MTBDWT_TBCTRL_NUMCOMP_SHIFT              (28U)
#define MTBDWT_TBCTRL_NUMCOMP(x)                 (((uint32_t)(((uint32_t)(x)) << MTBDWT_TBCTRL_NUMCOMP_SHIFT)) & MTBDWT_TBCTRL_NUMCOMP_MASK)

/*! @name DEVICECFG - Device Configuration Register */
#define MTBDWT_DEVICECFG_DEVICECFG_MASK          (0xFFFFFFFFU)
#define MTBDWT_DEVICECFG_DEVICECFG_SHIFT         (0U)
#define MTBDWT_DEVICECFG_DEVICECFG(x)            (((uint32_t)(((uint32_t)(x)) << MTBDWT_DEVICECFG_DEVICECFG_SHIFT)) & MTBDWT_DEVICECFG_DEVICECFG_MASK)

/*! @name DEVICETYPID - Device Type Identifier Register */
#define MTBDWT_DEVICETYPID_DEVICETYPID_MASK      (0xFFFFFFFFU)
#define MTBDWT_DEVICETYPID_DEVICETYPID_SHIFT     (0U)
#define MTBDWT_DEVICETYPID_DEVICETYPID(x)        (((uint32_t)(((uint32_t)(x)) << MTBDWT_DEVICETYPID_DEVICETYPID_SHIFT)) & MTBDWT_DEVICETYPID_DEVICETYPID_MASK)

/*! @name PERIPHID4 - Peripheral ID Register */
#define MTBDWT_PERIPHID4_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID4_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID4_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID4_PERIPHID_SHIFT)) & MTBDWT_PERIPHID4_PERIPHID_MASK)

/*! @name PERIPHID5 - Peripheral ID Register */
#define MTBDWT_PERIPHID5_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID5_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID5_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID5_PERIPHID_SHIFT)) & MTBDWT_PERIPHID5_PERIPHID_MASK)

/*! @name PERIPHID6 - Peripheral ID Register */
#define MTBDWT_PERIPHID6_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID6_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID6_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID6_PERIPHID_SHIFT)) & MTBDWT_PERIPHID6_PERIPHID_MASK)

/*! @name PERIPHID7 - Peripheral ID Register */
#define MTBDWT_PERIPHID7_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID7_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID7_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID7_PERIPHID_SHIFT)) & MTBDWT_PERIPHID7_PERIPHID_MASK)

/*! @name PERIPHID0 - Peripheral ID Register */
#define MTBDWT_PERIPHID0_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID0_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID0_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID0_PERIPHID_SHIFT)) & MTBDWT_PERIPHID0_PERIPHID_MASK)

/*! @name PERIPHID1 - Peripheral ID Register */
#define MTBDWT_PERIPHID1_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID1_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID1_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID1_PERIPHID_SHIFT)) & MTBDWT_PERIPHID1_PERIPHID_MASK)

/*! @name PERIPHID2 - Peripheral ID Register */
#define MTBDWT_PERIPHID2_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID2_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID2_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID2_PERIPHID_SHIFT)) & MTBDWT_PERIPHID2_PERIPHID_MASK)

/*! @name PERIPHID3 - Peripheral ID Register */
#define MTBDWT_PERIPHID3_PERIPHID_MASK           (0xFFFFFFFFU)
#define MTBDWT_PERIPHID3_PERIPHID_SHIFT          (0U)
#define MTBDWT_PERIPHID3_PERIPHID(x)             (((uint32_t)(((uint32_t)(x)) << MTBDWT_PERIPHID3_PERIPHID_SHIFT)) & MTBDWT_PERIPHID3_PERIPHID_MASK)

/*! @name COMPID - Component ID Register */
#define MTBDWT_COMPID_COMPID_MASK                (0xFFFFFFFFU)
#define MTBDWT_COMPID_COMPID_SHIFT               (0U)
#define MTBDWT_COMPID_COMPID(x)                  (((uint32_t)(((uint32_t)(x)) << MTBDWT_COMPID_COMPID_SHIFT)) & MTBDWT_COMPID_COMPID_MASK)

/* The count of MTBDWT_COMPID */
#define MTBDWT_COMPID_COUNT                      (4U)


/*!
 * @}
 */ /* end of group MTBDWT_Register_Masks */


/* MTBDWT - Peripheral instance base addresses */
/** Peripheral MTBDWT base address */
#define MTBDWT_BASE                              (0xF0001000u)
/** Peripheral MTBDWT base pointer */
#define MTBDWT                                   ((MTBDWT_Type *)MTBDWT_BASE)
/** Array initializer of MTBDWT peripheral base addresses */
#define MTBDWT_BASE_ADDRS                        { MTBDWT_BASE }
/** Array initializer of MTBDWT peripheral base pointers */
#define MTBDWT_BASE_PTRS                         { MTBDWT }

/*!
 * @}
 */ /* end of group MTBDWT_Peripheral_Access_Layer */


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
  __I  uint8_t FEPROT;                             /**< Non-volatile EERAM Protection Register, offset: 0xE */
  __I  uint8_t FDPROT;                             /**< Non-volatile D-Flash Protection Register, offset: 0xF */
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

/*! @name FEPROT - Non-volatile EERAM Protection Register */
#define NV_FEPROT_EPROT_MASK                     (0xFFU)
#define NV_FEPROT_EPROT_SHIFT                    (0U)
#define NV_FEPROT_EPROT(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FEPROT_EPROT_SHIFT)) & NV_FEPROT_EPROT_MASK)

/*! @name FDPROT - Non-volatile D-Flash Protection Register */
#define NV_FDPROT_DPROT_MASK                     (0xFFU)
#define NV_FDPROT_DPROT_SHIFT                    (0U)
#define NV_FDPROT_DPROT(x)                       (((uint8_t)(((uint8_t)(x)) << NV_FDPROT_DPROT_SHIFT)) & NV_FDPROT_DPROT_MASK)


/*!
 * @}
 */ /* end of group NV_Register_Masks */


/* NV - Peripheral instance base addresses */
/** Peripheral FTFE_FlashConfig base address */
#define FTFE_FlashConfig_BASE                    (0x400u)
/** Peripheral FTFE_FlashConfig base pointer */
#define FTFE_FlashConfig                         ((NV_Type *)FTFE_FlashConfig_BASE)
/** Array initializer of NV peripheral base addresses */
#define NV_BASE_ADDRS                            { FTFE_FlashConfig_BASE }
/** Array initializer of NV peripheral base pointers */
#define NV_BASE_PTRS                             { FTFE_FlashConfig }

/*!
 * @}
 */ /* end of group NV_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- OSC32 Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC32_Peripheral_Access_Layer OSC32 Peripheral Access Layer
 * @{
 */

/** OSC32 - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR;                                 /**< RTC Oscillator Control Register, offset: 0x0 */
} OSC32_Type;

/* ----------------------------------------------------------------------------
   -- OSC32 Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC32_Register_Masks OSC32 Register Masks
 * @{
 */

/*! @name CR - RTC Oscillator Control Register */
#define OSC32_CR_ROSCEREFS_MASK                  (0x10U)
#define OSC32_CR_ROSCEREFS_SHIFT                 (4U)
#define OSC32_CR_ROSCEREFS(x)                    (((uint8_t)(((uint8_t)(x)) << OSC32_CR_ROSCEREFS_SHIFT)) & OSC32_CR_ROSCEREFS_MASK)
#define OSC32_CR_ROSCSTB_MASK                    (0x20U)
#define OSC32_CR_ROSCSTB_SHIFT                   (5U)
#define OSC32_CR_ROSCSTB(x)                      (((uint8_t)(((uint8_t)(x)) << OSC32_CR_ROSCSTB_SHIFT)) & OSC32_CR_ROSCSTB_MASK)
#define OSC32_CR_ROSCSTPEN_MASK                  (0x40U)
#define OSC32_CR_ROSCSTPEN_SHIFT                 (6U)
#define OSC32_CR_ROSCSTPEN(x)                    (((uint8_t)(((uint8_t)(x)) << OSC32_CR_ROSCSTPEN_SHIFT)) & OSC32_CR_ROSCSTPEN_MASK)
#define OSC32_CR_ROSCEN_MASK                     (0x80U)
#define OSC32_CR_ROSCEN_SHIFT                    (7U)
#define OSC32_CR_ROSCEN(x)                       (((uint8_t)(((uint8_t)(x)) << OSC32_CR_ROSCEN_SHIFT)) & OSC32_CR_ROSCEN_MASK)


/*!
 * @}
 */ /* end of group OSC32_Register_Masks */


/* OSC32 - Peripheral instance base addresses */
/** Peripheral OSC32 base address */
#define OSC32_BASE                               (0x40060000u)
/** Peripheral OSC32 base pointer */
#define OSC32                                    ((OSC32_Type *)OSC32_BASE)
/** Array initializer of OSC32 peripheral base addresses */
#define OSC32_BASE_ADDRS                         { OSC32_BASE }
/** Array initializer of OSC32 peripheral base pointers */
#define OSC32_BASE_PTRS                          { OSC32 }

/*!
 * @}
 */ /* end of group OSC32_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PCC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Peripheral_Access_Layer PCC Peripheral Access Layer
 * @{
 */

/** PCC - Register Layout Typedef */
typedef struct {
  __IO uint32_t CLKCFG[117];                       /**< PCC Register, array offset: 0x0, array step: 0x4 */
} PCC_Type;

/* ----------------------------------------------------------------------------
   -- PCC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Register_Masks PCC Register Masks
 * @{
 */

/*! @name CLKCFG - PCC Register */
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
#define PCC_CLKCFG_COUNT                         (117U)


/*!
 * @}
 */ /* end of group PCC_Register_Masks */


/* PCC - Peripheral instance base addresses */
/** Peripheral PCC base address */
#define PCC_BASE                                 (0x40065000u)
/** Peripheral PCC base pointer */
#define PCC                                      ((PCC_Type *)PCC_BASE)
/** Array initializer of PCC peripheral base addresses */
#define PCC_BASE_ADDRS                           { PCC_BASE }
/** Array initializer of PCC peripheral base pointers */
#define PCC_BASE_PTRS                            { PCC }
#define PCC_INSTANCE_MASK      0xF
#define PCC_INSTANCE_SHIFT     12
#define PCC_PERIPHERAL_MASK    0xFFF
#define PCC_PERIPHERAL_SHIFT   0
#define PCC_INSTANCE_0         0

#define PCC_DMA0_INDEX                           8
#define PCC_FLASH_INDEX                          32
#define PCC_DMAMUX0_INDEX                        33
#define PCC_ADC1_INDEX                           39
#define PCC_LPSPI0_INDEX                         44
#define PCC_LPSPI1_INDEX                         45
#define PCC_CRC_INDEX                            50
#define PCC_PDB0_INDEX                           54
#define PCC_LPIT0_INDEX                          55
#define PCC_FLEXTMR0_INDEX                       56
#define PCC_FLEXTMR1_INDEX                       57
#define PCC_FLEXTMR2_INDEX                       58
#define PCC_ADC0_INDEX                           59
#define PCC_RTC_INDEX                            61
#define PCC_LPTMR0_INDEX                         64
#define PCC_TSI0_INDEX                           69
#define PCC_PORTA_INDEX                          73
#define PCC_PORTB_INDEX                          74
#define PCC_PORTC_INDEX                          75
#define PCC_PORTD_INDEX                          76
#define PCC_PORTE_INDEX                          77
#define PCC_PWT_INDEX                            86
#define PCC_FLEXIO_INDEX                         90
#define PCC_RTC_OSC_INDEX                        96
#define PCC_EWM_INDEX                            97
#define PCC_LPI2C0_INDEX                         102
#define PCC_LPI2C1_INDEX                         103
#define PCC_LPUART0_INDEX                        106
#define PCC_LPUART1_INDEX                        107
#define PCC_LPUART2_INDEX                        108
#define PCC_CMP0_INDEX                           115
#define PCC_CMP1_INDEX                           116


/*!
 * @}
 */ /* end of group PCC_Peripheral_Access_Layer */


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
  struct {                                         /* offset: 0x10, array step: 0x28 */
    __IO uint32_t C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x28 */
    __IO uint32_t S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x28 */
    __IO uint32_t DLY[2];                            /**< Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4 */
         uint8_t RESERVED_0[24];
  } CH[2];
       uint8_t RESERVED_0[304];
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
#define PDB_C1_COUNT                             (2U)

/*! @name S - Channel n Status register */
#define PDB_S_ERR_MASK                           (0xFFU)
#define PDB_S_ERR_SHIFT                          (0U)
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_S_ERR_SHIFT)) & PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            (0xFF0000U)
#define PDB_S_CF_SHIFT                           (16U)
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x)) << PDB_S_CF_SHIFT)) & PDB_S_CF_MASK)

/* The count of PDB_S */
#define PDB_S_COUNT                              (2U)

/*! @name DLY - Channel n Delay 0 register..Channel n Delay 1 register */
#define PDB_DLY_DLY_MASK                         (0xFFFFU)
#define PDB_DLY_DLY_SHIFT                        (0U)
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_DLY_DLY_SHIFT)) & PDB_DLY_DLY_MASK)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT                            (2U)

/* The count of PDB_DLY */
#define PDB_DLY_COUNT2                           (2U)

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
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t LVDSC1;                             /**< Low Voltage Detect Status and Control 1 Register, offset: 0x0 */
  __IO uint8_t LVDSC2;                             /**< Low Voltage Detect Status and Control 2 Register, offset: 0x1 */
  __IO uint8_t REGSC;                              /**< Regulator Status and Control Register, offset: 0x2 */
       uint8_t RESERVED_0[1];
  __IO uint8_t LPOTRIM;                            /**< Low Power Oscillator Trim Register, offset: 0x4 */
} PMC_Type;

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name LVDSC1 - Low Voltage Detect Status and Control 1 Register */
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

/*! @name LVDSC2 - Low Voltage Detect Status and Control 2 Register */
#define PMC_LVDSC2_LVWIE_MASK                    (0x20U)
#define PMC_LVDSC2_LVWIE_SHIFT                   (5U)
#define PMC_LVDSC2_LVWIE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWIE_SHIFT)) & PMC_LVDSC2_LVWIE_MASK)
#define PMC_LVDSC2_LVWACK_MASK                   (0x40U)
#define PMC_LVDSC2_LVWACK_SHIFT                  (6U)
#define PMC_LVDSC2_LVWACK(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWACK_SHIFT)) & PMC_LVDSC2_LVWACK_MASK)
#define PMC_LVDSC2_LVWF_MASK                     (0x80U)
#define PMC_LVDSC2_LVWF_SHIFT                    (7U)
#define PMC_LVDSC2_LVWF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWF_SHIFT)) & PMC_LVDSC2_LVWF_MASK)

/*! @name REGSC - Regulator Status and Control Register */
#define PMC_REGSC_BIASEN_MASK                    (0x1U)
#define PMC_REGSC_BIASEN_SHIFT                   (0U)
#define PMC_REGSC_BIASEN(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_BIASEN_SHIFT)) & PMC_REGSC_BIASEN_MASK)
#define PMC_REGSC_REGFPM_MASK                    (0x4U)
#define PMC_REGSC_REGFPM_SHIFT                   (2U)
#define PMC_REGSC_REGFPM(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_REGFPM_SHIFT)) & PMC_REGSC_REGFPM_MASK)
#define PMC_REGSC_LPOSTAT_MASK                   (0x40U)
#define PMC_REGSC_LPOSTAT_SHIFT                  (6U)
#define PMC_REGSC_LPOSTAT(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_LPOSTAT_SHIFT)) & PMC_REGSC_LPOSTAT_MASK)
#define PMC_REGSC_LPODIS_MASK                    (0x80U)
#define PMC_REGSC_LPODIS_SHIFT                   (7U)
#define PMC_REGSC_LPODIS(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_LPODIS_SHIFT)) & PMC_REGSC_LPODIS_MASK)

/*! @name LPOTRIM - Low Power Oscillator Trim Register */
#define PMC_LPOTRIM_LPOTRIM_MASK                 (0x1FU)
#define PMC_LPOTRIM_LPOTRIM_SHIFT                (0U)
#define PMC_LPOTRIM_LPOTRIM(x)                   (((uint8_t)(((uint8_t)(x)) << PMC_LPOTRIM_LPOTRIM_SHIFT)) & PMC_LPOTRIM_LPOTRIM_MASK)


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
#define PORT_IRQS                                { PORTAE_IRQn, PORTBCD_IRQn, PORTBCD_IRQn, PORTBCD_IRQn, PORTAE_IRQn }

/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PWT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWT_Peripheral_Access_Layer PWT Peripheral Access Layer
 * @{
 */

/** PWT - Register Layout Typedef */
typedef struct {
  __IO uint8_t CS;                                 /**< Pulse Width Timer Control and Status Register, offset: 0x0 */
  __IO uint8_t CR;                                 /**< Pulse Width Timer Control Register, offset: 0x1 */
  __I  uint8_t PPH;                                /**< Pulse Width Timer Positive Pulse Width Register: High, offset: 0x2 */
  __I  uint8_t PPL;                                /**< Pulse Width Timer Positive Pulse Width Register: Loq, offset: 0x3 */
  __I  uint8_t NPH;                                /**< Pulse Width Timer Negative Pulse Width Register: High, offset: 0x4 */
  __I  uint8_t NPL;                                /**< Pulse Width Timer Negative Pulse Width Register: Low, offset: 0x5 */
  __I  uint8_t CNTH;                               /**< Pulse Width Timer Counter Register: High, offset: 0x6 */
  __I  uint8_t CNTL;                               /**< Pulse Width Timer Counter Register: Low, offset: 0x7 */
} PWT_Type;

/* ----------------------------------------------------------------------------
   -- PWT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWT_Register_Masks PWT Register Masks
 * @{
 */

/*! @name CS - Pulse Width Timer Control and Status Register */
#define PWT_CS_PWTOV_MASK                        (0x1U)
#define PWT_CS_PWTOV_SHIFT                       (0U)
#define PWT_CS_PWTOV(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_PWTOV_SHIFT)) & PWT_CS_PWTOV_MASK)
#define PWT_CS_PWTRDY_MASK                       (0x2U)
#define PWT_CS_PWTRDY_SHIFT                      (1U)
#define PWT_CS_PWTRDY(x)                         (((uint8_t)(((uint8_t)(x)) << PWT_CS_PWTRDY_SHIFT)) & PWT_CS_PWTRDY_MASK)
#define PWT_CS_FCTLE_MASK                        (0x4U)
#define PWT_CS_FCTLE_SHIFT                       (2U)
#define PWT_CS_FCTLE(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_FCTLE_SHIFT)) & PWT_CS_FCTLE_MASK)
#define PWT_CS_PWTSR_MASK                        (0x8U)
#define PWT_CS_PWTSR_SHIFT                       (3U)
#define PWT_CS_PWTSR(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_PWTSR_SHIFT)) & PWT_CS_PWTSR_MASK)
#define PWT_CS_POVIE_MASK                        (0x10U)
#define PWT_CS_POVIE_SHIFT                       (4U)
#define PWT_CS_POVIE(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_POVIE_SHIFT)) & PWT_CS_POVIE_MASK)
#define PWT_CS_PRDYIE_MASK                       (0x20U)
#define PWT_CS_PRDYIE_SHIFT                      (5U)
#define PWT_CS_PRDYIE(x)                         (((uint8_t)(((uint8_t)(x)) << PWT_CS_PRDYIE_SHIFT)) & PWT_CS_PRDYIE_MASK)
#define PWT_CS_PWTIE_MASK                        (0x40U)
#define PWT_CS_PWTIE_SHIFT                       (6U)
#define PWT_CS_PWTIE(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_PWTIE_SHIFT)) & PWT_CS_PWTIE_MASK)
#define PWT_CS_PWTEN_MASK                        (0x80U)
#define PWT_CS_PWTEN_SHIFT                       (7U)
#define PWT_CS_PWTEN(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CS_PWTEN_SHIFT)) & PWT_CS_PWTEN_MASK)

/*! @name CR - Pulse Width Timer Control Register */
#define PWT_CR_PRE_MASK                          (0x7U)
#define PWT_CR_PRE_SHIFT                         (0U)
#define PWT_CR_PRE(x)                            (((uint8_t)(((uint8_t)(x)) << PWT_CR_PRE_SHIFT)) & PWT_CR_PRE_MASK)
#define PWT_CR_LVL_MASK                          (0x8U)
#define PWT_CR_LVL_SHIFT                         (3U)
#define PWT_CR_LVL(x)                            (((uint8_t)(((uint8_t)(x)) << PWT_CR_LVL_SHIFT)) & PWT_CR_LVL_MASK)
#define PWT_CR_TGL_MASK                          (0x10U)
#define PWT_CR_TGL_SHIFT                         (4U)
#define PWT_CR_TGL(x)                            (((uint8_t)(((uint8_t)(x)) << PWT_CR_TGL_SHIFT)) & PWT_CR_TGL_MASK)
#define PWT_CR_PINSEL_MASK                       (0x60U)
#define PWT_CR_PINSEL_SHIFT                      (5U)
#define PWT_CR_PINSEL(x)                         (((uint8_t)(((uint8_t)(x)) << PWT_CR_PINSEL_SHIFT)) & PWT_CR_PINSEL_MASK)
#define PWT_CR_PCLKS_MASK                        (0x80U)
#define PWT_CR_PCLKS_SHIFT                       (7U)
#define PWT_CR_PCLKS(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_CR_PCLKS_SHIFT)) & PWT_CR_PCLKS_MASK)

/*! @name PPH - Pulse Width Timer Positive Pulse Width Register: High */
#define PWT_PPH_PPWH_MASK                        (0xFFU)
#define PWT_PPH_PPWH_SHIFT                       (0U)
#define PWT_PPH_PPWH(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_PPH_PPWH_SHIFT)) & PWT_PPH_PPWH_MASK)

/*! @name PPL - Pulse Width Timer Positive Pulse Width Register: Loq */
#define PWT_PPL_PPWL_MASK                        (0xFFU)
#define PWT_PPL_PPWL_SHIFT                       (0U)
#define PWT_PPL_PPWL(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_PPL_PPWL_SHIFT)) & PWT_PPL_PPWL_MASK)

/*! @name NPH - Pulse Width Timer Negative Pulse Width Register: High */
#define PWT_NPH_NPWH_MASK                        (0xFFU)
#define PWT_NPH_NPWH_SHIFT                       (0U)
#define PWT_NPH_NPWH(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_NPH_NPWH_SHIFT)) & PWT_NPH_NPWH_MASK)

/*! @name NPL - Pulse Width Timer Negative Pulse Width Register: Low */
#define PWT_NPL_NPWL_MASK                        (0xFFU)
#define PWT_NPL_NPWL_SHIFT                       (0U)
#define PWT_NPL_NPWL(x)                          (((uint8_t)(((uint8_t)(x)) << PWT_NPL_NPWL_SHIFT)) & PWT_NPL_NPWL_MASK)

/*! @name CNTH - Pulse Width Timer Counter Register: High */
#define PWT_CNTH_PWTH_MASK                       (0xFFU)
#define PWT_CNTH_PWTH_SHIFT                      (0U)
#define PWT_CNTH_PWTH(x)                         (((uint8_t)(((uint8_t)(x)) << PWT_CNTH_PWTH_SHIFT)) & PWT_CNTH_PWTH_MASK)

/*! @name CNTL - Pulse Width Timer Counter Register: Low */
#define PWT_CNTL_PWTL_MASK                       (0xFFU)
#define PWT_CNTL_PWTL_SHIFT                      (0U)
#define PWT_CNTL_PWTL(x)                         (((uint8_t)(((uint8_t)(x)) << PWT_CNTL_PWTL_SHIFT)) & PWT_CNTL_PWTL_MASK)


/*!
 * @}
 */ /* end of group PWT_Register_Masks */


/* PWT - Peripheral instance base addresses */
/** Peripheral PWT base address */
#define PWT_BASE                                 (0x40056000u)
/** Peripheral PWT base pointer */
#define PWT                                      ((PWT_Type *)PWT_BASE)
/** Array initializer of PWT peripheral base addresses */
#define PWT_BASE_ADDRS                           { PWT_BASE }
/** Array initializer of PWT peripheral base pointers */
#define PWT_BASE_PTRS                            { PWT }
/** Interrupt vectors for the PWT peripheral type */
#define PWT_IRQS                                 { PWT_LPTMR0_IRQn }

/*!
 * @}
 */ /* end of group PWT_Peripheral_Access_Layer */


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
   -- ROM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ROM_Peripheral_Access_Layer ROM Peripheral Access Layer
 * @{
 */

/** ROM - Register Layout Typedef */
typedef struct {
  __I  uint32_t ENTRY[3];                          /**< Entry, array offset: 0x0, array step: 0x4 */
  __I  uint32_t TABLEMARK;                         /**< End of Table Marker Register, offset: 0xC */
       uint8_t RESERVED_0[4028];
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
} ROM_Type;

/* ----------------------------------------------------------------------------
   -- ROM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ROM_Register_Masks ROM Register Masks
 * @{
 */

/*! @name ENTRY - Entry */
#define ROM_ENTRY_ENTRY_MASK                     (0xFFFFFFFFU)
#define ROM_ENTRY_ENTRY_SHIFT                    (0U)
#define ROM_ENTRY_ENTRY(x)                       (((uint32_t)(((uint32_t)(x)) << ROM_ENTRY_ENTRY_SHIFT)) & ROM_ENTRY_ENTRY_MASK)

/* The count of ROM_ENTRY */
#define ROM_ENTRY_COUNT                          (3U)

/*! @name TABLEMARK - End of Table Marker Register */
#define ROM_TABLEMARK_MARK_MASK                  (0xFFFFFFFFU)
#define ROM_TABLEMARK_MARK_SHIFT                 (0U)
#define ROM_TABLEMARK_MARK(x)                    (((uint32_t)(((uint32_t)(x)) << ROM_TABLEMARK_MARK_SHIFT)) & ROM_TABLEMARK_MARK_MASK)

/*! @name SYSACCESS - System Access Register */
#define ROM_SYSACCESS_SYSACCESS_MASK             (0xFFFFFFFFU)
#define ROM_SYSACCESS_SYSACCESS_SHIFT            (0U)
#define ROM_SYSACCESS_SYSACCESS(x)               (((uint32_t)(((uint32_t)(x)) << ROM_SYSACCESS_SYSACCESS_SHIFT)) & ROM_SYSACCESS_SYSACCESS_MASK)

/*! @name PERIPHID4 - Peripheral ID Register */
#define ROM_PERIPHID4_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID4_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID4_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID4_PERIPHID_SHIFT)) & ROM_PERIPHID4_PERIPHID_MASK)

/*! @name PERIPHID5 - Peripheral ID Register */
#define ROM_PERIPHID5_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID5_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID5_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID5_PERIPHID_SHIFT)) & ROM_PERIPHID5_PERIPHID_MASK)

/*! @name PERIPHID6 - Peripheral ID Register */
#define ROM_PERIPHID6_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID6_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID6_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID6_PERIPHID_SHIFT)) & ROM_PERIPHID6_PERIPHID_MASK)

/*! @name PERIPHID7 - Peripheral ID Register */
#define ROM_PERIPHID7_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID7_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID7_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID7_PERIPHID_SHIFT)) & ROM_PERIPHID7_PERIPHID_MASK)

/*! @name PERIPHID0 - Peripheral ID Register */
#define ROM_PERIPHID0_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID0_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID0_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID0_PERIPHID_SHIFT)) & ROM_PERIPHID0_PERIPHID_MASK)

/*! @name PERIPHID1 - Peripheral ID Register */
#define ROM_PERIPHID1_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID1_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID1_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID1_PERIPHID_SHIFT)) & ROM_PERIPHID1_PERIPHID_MASK)

/*! @name PERIPHID2 - Peripheral ID Register */
#define ROM_PERIPHID2_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID2_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID2_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID2_PERIPHID_SHIFT)) & ROM_PERIPHID2_PERIPHID_MASK)

/*! @name PERIPHID3 - Peripheral ID Register */
#define ROM_PERIPHID3_PERIPHID_MASK              (0xFFFFFFFFU)
#define ROM_PERIPHID3_PERIPHID_SHIFT             (0U)
#define ROM_PERIPHID3_PERIPHID(x)                (((uint32_t)(((uint32_t)(x)) << ROM_PERIPHID3_PERIPHID_SHIFT)) & ROM_PERIPHID3_PERIPHID_MASK)

/*! @name COMPID - Component ID Register */
#define ROM_COMPID_COMPID_MASK                   (0xFFFFFFFFU)
#define ROM_COMPID_COMPID_SHIFT                  (0U)
#define ROM_COMPID_COMPID(x)                     (((uint32_t)(((uint32_t)(x)) << ROM_COMPID_COMPID_SHIFT)) & ROM_COMPID_COMPID_MASK)

/* The count of ROM_COMPID */
#define ROM_COMPID_COUNT                         (4U)


/*!
 * @}
 */ /* end of group ROM_Register_Masks */


/* ROM - Peripheral instance base addresses */
/** Peripheral ROM base address */
#define ROM_BASE                                 (0xF0002000u)
/** Peripheral ROM base pointer */
#define ROM                                      ((ROM_Type *)ROM_BASE)
/** Array initializer of ROM peripheral base addresses */
#define ROM_BASE_ADDRS                           { ROM_BASE }
/** Array initializer of ROM peripheral base pointers */
#define ROM_BASE_PTRS                            { ROM }

/*!
 * @}
 */ /* end of group ROM_Peripheral_Access_Layer */


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
#define RTC_SECONDS_IRQS                         { RTC_IRQn }

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
       uint8_t RESERVED_1[4];
  __IO uint32_t CLKOUTCNFG;                        /**< SCG CLKOUT Configuration Register, offset: 0x20 */
       uint8_t RESERVED_2[220];
  __IO uint32_t SOSCCSR;                           /**< System OSC Control Status Register, offset: 0x100 */
  __IO uint32_t SOSCDIV;                           /**< System OSC Divide Register, offset: 0x104 */
  __IO uint32_t SOSCCFG;                           /**< System Oscillator Configuration Register, offset: 0x108 */
       uint8_t RESERVED_3[244];
  __IO uint32_t SIRCCSR;                           /**< Slow IRC Control Status Register, offset: 0x200 */
  __IO uint32_t SIRCDIV;                           /**< Slow IRC Divide Register, offset: 0x204 */
  __IO uint32_t SIRCCFG;                           /**< Slow IRC Configuration Register, offset: 0x208 */
       uint8_t RESERVED_4[244];
  __IO uint32_t FIRCCSR;                           /**< Fast IRC Control Status Register, offset: 0x300 */
  __IO uint32_t FIRCDIV;                           /**< Fast IRC Divide Register, offset: 0x304 */
  __IO uint32_t FIRCCFG;                           /**< Fast IRC Configuration Register, offset: 0x308 */
  __IO uint32_t FIRCTCFG;                          /**< Fast IRC Trim Configuration Register, offset: 0x30C */
       uint8_t RESERVED_5[8];
  __IO uint32_t FIRCSTAT;                          /**< Fast IRC Status Register, offset: 0x318 */
       uint8_t RESERVED_6[228];
  __IO uint32_t ROSCCSR;                           /**< RTC OSC Control Status Register, offset: 0x400 */
       uint8_t RESERVED_7[252];
  __IO uint32_t LPFLLCSR;                          /**< Low Power FLL Control Status Register, offset: 0x500 */
  __IO uint32_t LPFLLDIV;                          /**< Low Power FLL Divide Register, offset: 0x504 */
  __IO uint32_t LPFLLCFG;                          /**< Low Power FLL Configuration Register, offset: 0x508 */
  __IO uint32_t LPFLLTCFG;                         /**< Low Power FLL Trim Configuration Register, offset: 0x50C */
       uint8_t RESERVED_8[4];
  __IO uint32_t LPFLLSTAT;                         /**< Low Power FLL Status Register, offset: 0x514 */
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

/*! @name ROSCCSR - RTC OSC Control Status Register */
#define SCG_ROSCCSR_ROSCCM_MASK                  (0x10000U)
#define SCG_ROSCCSR_ROSCCM_SHIFT                 (16U)
#define SCG_ROSCCSR_ROSCCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCCM_SHIFT)) & SCG_ROSCCSR_ROSCCM_MASK)
#define SCG_ROSCCSR_ROSCCMRE_MASK                (0x20000U)
#define SCG_ROSCCSR_ROSCCMRE_SHIFT               (17U)
#define SCG_ROSCCSR_ROSCCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCCMRE_SHIFT)) & SCG_ROSCCSR_ROSCCMRE_MASK)
#define SCG_ROSCCSR_LK_MASK                      (0x800000U)
#define SCG_ROSCCSR_LK_SHIFT                     (23U)
#define SCG_ROSCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_LK_SHIFT)) & SCG_ROSCCSR_LK_MASK)
#define SCG_ROSCCSR_ROSCVLD_MASK                 (0x1000000U)
#define SCG_ROSCCSR_ROSCVLD_SHIFT                (24U)
#define SCG_ROSCCSR_ROSCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCVLD_SHIFT)) & SCG_ROSCCSR_ROSCVLD_MASK)
#define SCG_ROSCCSR_ROSCERR_MASK                 (0x4000000U)
#define SCG_ROSCCSR_ROSCERR_SHIFT                (26U)
#define SCG_ROSCCSR_ROSCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_ROSCCSR_ROSCERR_SHIFT)) & SCG_ROSCCSR_ROSCERR_MASK)

/*! @name LPFLLCSR - Low Power FLL Control Status Register */
#define SCG_LPFLLCSR_LPFLLEN_MASK                (0x1U)
#define SCG_LPFLLCSR_LPFLLEN_SHIFT               (0U)
#define SCG_LPFLLCSR_LPFLLEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLEN_SHIFT)) & SCG_LPFLLCSR_LPFLLEN_MASK)
#define SCG_LPFLLCSR_LPFLLTREN_MASK              (0x100U)
#define SCG_LPFLLCSR_LPFLLTREN_SHIFT             (8U)
#define SCG_LPFLLCSR_LPFLLTREN(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLTREN_SHIFT)) & SCG_LPFLLCSR_LPFLLTREN_MASK)
#define SCG_LPFLLCSR_LPFLLTRUP_MASK              (0x200U)
#define SCG_LPFLLCSR_LPFLLTRUP_SHIFT             (9U)
#define SCG_LPFLLCSR_LPFLLTRUP(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLTRUP_SHIFT)) & SCG_LPFLLCSR_LPFLLTRUP_MASK)
#define SCG_LPFLLCSR_LPFLLTRMLOCK_MASK           (0x400U)
#define SCG_LPFLLCSR_LPFLLTRMLOCK_SHIFT          (10U)
#define SCG_LPFLLCSR_LPFLLTRMLOCK(x)             (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLTRMLOCK_SHIFT)) & SCG_LPFLLCSR_LPFLLTRMLOCK_MASK)
#define SCG_LPFLLCSR_LPFLLCM_MASK                (0x10000U)
#define SCG_LPFLLCSR_LPFLLCM_SHIFT               (16U)
#define SCG_LPFLLCSR_LPFLLCM(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLCM_SHIFT)) & SCG_LPFLLCSR_LPFLLCM_MASK)
#define SCG_LPFLLCSR_LPFLLCMRE_MASK              (0x20000U)
#define SCG_LPFLLCSR_LPFLLCMRE_SHIFT             (17U)
#define SCG_LPFLLCSR_LPFLLCMRE(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLCMRE_SHIFT)) & SCG_LPFLLCSR_LPFLLCMRE_MASK)
#define SCG_LPFLLCSR_LK_MASK                     (0x800000U)
#define SCG_LPFLLCSR_LK_SHIFT                    (23U)
#define SCG_LPFLLCSR_LK(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LK_SHIFT)) & SCG_LPFLLCSR_LK_MASK)
#define SCG_LPFLLCSR_LPFLLVLD_MASK               (0x1000000U)
#define SCG_LPFLLCSR_LPFLLVLD_SHIFT              (24U)
#define SCG_LPFLLCSR_LPFLLVLD(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLVLD_SHIFT)) & SCG_LPFLLCSR_LPFLLVLD_MASK)
#define SCG_LPFLLCSR_LPFLLSEL_MASK               (0x2000000U)
#define SCG_LPFLLCSR_LPFLLSEL_SHIFT              (25U)
#define SCG_LPFLLCSR_LPFLLSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLSEL_SHIFT)) & SCG_LPFLLCSR_LPFLLSEL_MASK)
#define SCG_LPFLLCSR_LPFLLERR_MASK               (0x4000000U)
#define SCG_LPFLLCSR_LPFLLERR_SHIFT              (26U)
#define SCG_LPFLLCSR_LPFLLERR(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCSR_LPFLLERR_SHIFT)) & SCG_LPFLLCSR_LPFLLERR_MASK)

/*! @name LPFLLDIV - Low Power FLL Divide Register */
#define SCG_LPFLLDIV_LPFLLDIV1_MASK              (0x7U)
#define SCG_LPFLLDIV_LPFLLDIV1_SHIFT             (0U)
#define SCG_LPFLLDIV_LPFLLDIV1(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLDIV_LPFLLDIV1_SHIFT)) & SCG_LPFLLDIV_LPFLLDIV1_MASK)
#define SCG_LPFLLDIV_LPFLLDIV2_MASK              (0x700U)
#define SCG_LPFLLDIV_LPFLLDIV2_SHIFT             (8U)
#define SCG_LPFLLDIV_LPFLLDIV2(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLDIV_LPFLLDIV2_SHIFT)) & SCG_LPFLLDIV_LPFLLDIV2_MASK)

/*! @name LPFLLCFG - Low Power FLL Configuration Register */
#define SCG_LPFLLCFG_FSEL_MASK                   (0x3U)
#define SCG_LPFLLCFG_FSEL_SHIFT                  (0U)
#define SCG_LPFLLCFG_FSEL(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLCFG_FSEL_SHIFT)) & SCG_LPFLLCFG_FSEL_MASK)

/*! @name LPFLLTCFG - Low Power FLL Trim Configuration Register */
#define SCG_LPFLLTCFG_TRIMSRC_MASK               (0x3U)
#define SCG_LPFLLTCFG_TRIMSRC_SHIFT              (0U)
#define SCG_LPFLLTCFG_TRIMSRC(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLTCFG_TRIMSRC_SHIFT)) & SCG_LPFLLTCFG_TRIMSRC_MASK)
#define SCG_LPFLLTCFG_TRIMDIV_MASK               (0x1F00U)
#define SCG_LPFLLTCFG_TRIMDIV_SHIFT              (8U)
#define SCG_LPFLLTCFG_TRIMDIV(x)                 (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLTCFG_TRIMDIV_SHIFT)) & SCG_LPFLLTCFG_TRIMDIV_MASK)
#define SCG_LPFLLTCFG_LOCKW2LSB_MASK             (0x10000U)
#define SCG_LPFLLTCFG_LOCKW2LSB_SHIFT            (16U)
#define SCG_LPFLLTCFG_LOCKW2LSB(x)               (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLTCFG_LOCKW2LSB_SHIFT)) & SCG_LPFLLTCFG_LOCKW2LSB_MASK)

/*! @name LPFLLSTAT - Low Power FLL Status Register */
#define SCG_LPFLLSTAT_AUTOTRIM_MASK              (0xFFU)
#define SCG_LPFLLSTAT_AUTOTRIM_SHIFT             (0U)
#define SCG_LPFLLSTAT_AUTOTRIM(x)                (((uint32_t)(((uint32_t)(x)) << SCG_LPFLLSTAT_AUTOTRIM_SHIFT)) & SCG_LPFLLSTAT_AUTOTRIM_MASK)


/*!
 * @}
 */ /* end of group SCG_Register_Masks */


/* SCG - Peripheral instance base addresses */
/** Peripheral SCG base address */
#define SCG_BASE                                 (0x40064000u)
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
       uint8_t RESERVED_0[4];
  __IO uint32_t CHIPCTL;                           /**< Chip Control register, offset: 0x4 */
       uint8_t RESERVED_1[4];
  __IO uint32_t FTMOPT0;                           /**< FTM Option Register 0, offset: 0xC */
       uint8_t RESERVED_2[8];
  __IO uint32_t ADCOPT;                            /**< ADC Options Register, offset: 0x18 */
  __IO uint32_t FTMOPT1;                           /**< FTM Option Register 1, offset: 0x1C */
       uint8_t RESERVED_3[4];
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x24 */
       uint8_t RESERVED_4[36];
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x4C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x50 */
  __I  uint32_t UIDH;                              /**< Unique Identification Register High, offset: 0x54 */
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x58 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x5C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x60 */
       uint8_t RESERVED_5[8];
  __IO uint32_t MISCTRL;                           /**< Miscellaneous Control register, offset: 0x6C */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name CHIPCTL - Chip Control register */
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK       (0x3U)
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT      (0U)
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN(x)         (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)) & SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK)
#define SIM_CHIPCTL_CLKOUTDIV_MASK               (0x30U)
#define SIM_CHIPCTL_CLKOUTDIV_SHIFT              (4U)
#define SIM_CHIPCTL_CLKOUTDIV(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_CLKOUTDIV_SHIFT)) & SIM_CHIPCTL_CLKOUTDIV_MASK)
#define SIM_CHIPCTL_CLKOUTSEL_MASK               (0xC0U)
#define SIM_CHIPCTL_CLKOUTSEL_SHIFT              (6U)
#define SIM_CHIPCTL_CLKOUTSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_CLKOUTSEL_SHIFT)) & SIM_CHIPCTL_CLKOUTSEL_MASK)
#define SIM_CHIPCTL_PDB_BB_SEL_MASK              (0x2000U)
#define SIM_CHIPCTL_PDB_BB_SEL_SHIFT             (13U)
#define SIM_CHIPCTL_PDB_BB_SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_PDB_BB_SEL_SHIFT)) & SIM_CHIPCTL_PDB_BB_SEL_MASK)
#define SIM_CHIPCTL_PWTCLKSEL_MASK               (0x30000U)
#define SIM_CHIPCTL_PWTCLKSEL_SHIFT              (16U)
#define SIM_CHIPCTL_PWTCLKSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_PWTCLKSEL_SHIFT)) & SIM_CHIPCTL_PWTCLKSEL_MASK)
#define SIM_CHIPCTL_RTC32KCLKSEL_MASK            (0xC0000U)
#define SIM_CHIPCTL_RTC32KCLKSEL_SHIFT           (18U)
#define SIM_CHIPCTL_RTC32KCLKSEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_RTC32KCLKSEL_SHIFT)) & SIM_CHIPCTL_RTC32KCLKSEL_MASK)

/*! @name FTMOPT0 - FTM Option Register 0 */
#define SIM_FTMOPT0_FTM0FLTxSEL_MASK             (0x7U)
#define SIM_FTMOPT0_FTM0FLTxSEL_SHIFT            (0U)
#define SIM_FTMOPT0_FTM0FLTxSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM0FLTxSEL_SHIFT)) & SIM_FTMOPT0_FTM0FLTxSEL_MASK)
#define SIM_FTMOPT0_FTM0CLKSEL_MASK              (0x3000000U)
#define SIM_FTMOPT0_FTM0CLKSEL_SHIFT             (24U)
#define SIM_FTMOPT0_FTM0CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM0CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM0CLKSEL_MASK)
#define SIM_FTMOPT0_FTM1CLKSEL_MASK              (0xC000000U)
#define SIM_FTMOPT0_FTM1CLKSEL_SHIFT             (26U)
#define SIM_FTMOPT0_FTM1CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM1CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM1CLKSEL_MASK)
#define SIM_FTMOPT0_FTM2CLKSEL_MASK              (0x30000000U)
#define SIM_FTMOPT0_FTM2CLKSEL_SHIFT             (28U)
#define SIM_FTMOPT0_FTM2CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM2CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM2CLKSEL_MASK)

/*! @name ADCOPT - ADC Options Register */
#define SIM_ADCOPT_ADC0TRGSEL_MASK               (0x1U)
#define SIM_ADCOPT_ADC0TRGSEL_SHIFT              (0U)
#define SIM_ADCOPT_ADC0TRGSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0TRGSEL_SHIFT)) & SIM_ADCOPT_ADC0TRGSEL_MASK)
#define SIM_ADCOPT_ADC0SWPRETRG_MASK             (0x6U)
#define SIM_ADCOPT_ADC0SWPRETRG_SHIFT            (1U)
#define SIM_ADCOPT_ADC0SWPRETRG(x)               (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0SWPRETRG_SHIFT)) & SIM_ADCOPT_ADC0SWPRETRG_MASK)
#define SIM_ADCOPT_ADC0PRETRGSEL_MASK            (0x30U)
#define SIM_ADCOPT_ADC0PRETRGSEL_SHIFT           (4U)
#define SIM_ADCOPT_ADC0PRETRGSEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT)) & SIM_ADCOPT_ADC0PRETRGSEL_MASK)
#define SIM_ADCOPT_ADC1TRGSEL_MASK               (0x100U)
#define SIM_ADCOPT_ADC1TRGSEL_SHIFT              (8U)
#define SIM_ADCOPT_ADC1TRGSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1TRGSEL_SHIFT)) & SIM_ADCOPT_ADC1TRGSEL_MASK)
#define SIM_ADCOPT_ADC1SWPRETRG_MASK             (0x600U)
#define SIM_ADCOPT_ADC1SWPRETRG_SHIFT            (9U)
#define SIM_ADCOPT_ADC1SWPRETRG(x)               (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1SWPRETRG_SHIFT)) & SIM_ADCOPT_ADC1SWPRETRG_MASK)
#define SIM_ADCOPT_ADC1PRETRGSEL_MASK            (0x3000U)
#define SIM_ADCOPT_ADC1PRETRGSEL_SHIFT           (12U)
#define SIM_ADCOPT_ADC1PRETRGSEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT)) & SIM_ADCOPT_ADC1PRETRGSEL_MASK)

/*! @name FTMOPT1 - FTM Option Register 1 */
#define SIM_FTMOPT1_FTM0SYNCBIT_MASK             (0x1U)
#define SIM_FTMOPT1_FTM0SYNCBIT_SHIFT            (0U)
#define SIM_FTMOPT1_FTM0SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM0SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM0SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM1SYNCBIT_MASK             (0x2U)
#define SIM_FTMOPT1_FTM1SYNCBIT_SHIFT            (1U)
#define SIM_FTMOPT1_FTM1SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM1SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM1SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM2SYNCBIT_MASK             (0x4U)
#define SIM_FTMOPT1_FTM2SYNCBIT_SHIFT            (2U)
#define SIM_FTMOPT1_FTM2SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM2SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM1CH0SEL_MASK              (0x30U)
#define SIM_FTMOPT1_FTM1CH0SEL_SHIFT             (4U)
#define SIM_FTMOPT1_FTM1CH0SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM1CH0SEL_SHIFT)) & SIM_FTMOPT1_FTM1CH0SEL_MASK)
#define SIM_FTMOPT1_FTM2CH0SEL_MASK              (0xC0U)
#define SIM_FTMOPT1_FTM2CH0SEL_SHIFT             (6U)
#define SIM_FTMOPT1_FTM2CH0SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2CH0SEL_SHIFT)) & SIM_FTMOPT1_FTM2CH0SEL_MASK)
#define SIM_FTMOPT1_FTM2CH1SEL_MASK              (0x100U)
#define SIM_FTMOPT1_FTM2CH1SEL_SHIFT             (8U)
#define SIM_FTMOPT1_FTM2CH1SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2CH1SEL_SHIFT)) & SIM_FTMOPT1_FTM2CH1SEL_MASK)
#define SIM_FTMOPT1_FTM0_OUTSEL_MASK             (0xFF0000U)
#define SIM_FTMOPT1_FTM0_OUTSEL_SHIFT            (16U)
#define SIM_FTMOPT1_FTM0_OUTSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)) & SIM_FTMOPT1_FTM0_OUTSEL_MASK)

/*! @name SDID - System Device Identification Register */
#define SIM_SDID_PINID_MASK                      (0x7FU)
#define SIM_SDID_PINID_SHIFT                     (0U)
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PINID_SHIFT)) & SIM_SDID_PINID_MASK)
#define SIM_SDID_PROJECTID_MASK                  (0xF80U)
#define SIM_SDID_PROJECTID_SHIFT                 (7U)
#define SIM_SDID_PROJECTID(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PROJECTID_SHIFT)) & SIM_SDID_PROJECTID_MASK)
#define SIM_SDID_REVID_MASK                      (0xF000U)
#define SIM_SDID_REVID_SHIFT                     (12U)
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_REVID_SHIFT)) & SIM_SDID_REVID_MASK)
#define SIM_SDID_RAMSIZE_MASK                    (0xF0000U)
#define SIM_SDID_RAMSIZE_SHIFT                   (16U)
#define SIM_SDID_RAMSIZE(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SDID_RAMSIZE_SHIFT)) & SIM_SDID_RAMSIZE_MASK)
#define SIM_SDID_SERIESID_MASK                   (0xF00000U)
#define SIM_SDID_SERIESID_SHIFT                  (20U)
#define SIM_SDID_SERIESID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SERIESID_SHIFT)) & SIM_SDID_SERIESID_MASK)
#define SIM_SDID_SUBFAMID_MASK                   (0xF000000U)
#define SIM_SDID_SUBFAMID_SHIFT                  (24U)
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SUBFAMID_SHIFT)) & SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMILYID_MASK                   (0xF0000000U)
#define SIM_SDID_FAMILYID_SHIFT                  (28U)
#define SIM_SDID_FAMILYID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FAMILYID_SHIFT)) & SIM_SDID_FAMILYID_MASK)

/*! @name FCFG1 - Flash Configuration Register 1 */
#define SIM_FCFG1_FLASHDIS_MASK                  (0x1U)
#define SIM_FCFG1_FLASHDIS_SHIFT                 (0U)
#define SIM_FCFG1_FLASHDIS(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_FLASHDIS_SHIFT)) & SIM_FCFG1_FLASHDIS_MASK)
#define SIM_FCFG1_FLASHDOZE_MASK                 (0x2U)
#define SIM_FCFG1_FLASHDOZE_SHIFT                (1U)
#define SIM_FCFG1_FLASHDOZE(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_FLASHDOZE_SHIFT)) & SIM_FCFG1_FLASHDOZE_MASK)
#define SIM_FCFG1_DEPART_MASK                    (0xF000U)
#define SIM_FCFG1_DEPART_SHIFT                   (12U)
#define SIM_FCFG1_DEPART(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_DEPART_SHIFT)) & SIM_FCFG1_DEPART_MASK)
#define SIM_FCFG1_EEERAMSIZE_MASK                (0xF0000U)
#define SIM_FCFG1_EEERAMSIZE_SHIFT               (16U)
#define SIM_FCFG1_EEERAMSIZE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_EEERAMSIZE_SHIFT)) & SIM_FCFG1_EEERAMSIZE_MASK)
#define SIM_FCFG1_PFSIZE_MASK                    (0xF000000U)
#define SIM_FCFG1_PFSIZE_SHIFT                   (24U)
#define SIM_FCFG1_PFSIZE(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_PFSIZE_SHIFT)) & SIM_FCFG1_PFSIZE_MASK)
#define SIM_FCFG1_NVMSIZE_MASK                   (0xF0000000U)
#define SIM_FCFG1_NVMSIZE_SHIFT                  (28U)
#define SIM_FCFG1_NVMSIZE(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_NVMSIZE_SHIFT)) & SIM_FCFG1_NVMSIZE_MASK)

/*! @name FCFG2 - Flash Configuration Register 2 */
#define SIM_FCFG2_MAXADDR1_MASK                  (0x7F0000U)
#define SIM_FCFG2_MAXADDR1_SHIFT                 (16U)
#define SIM_FCFG2_MAXADDR1(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG2_MAXADDR1_SHIFT)) & SIM_FCFG2_MAXADDR1_MASK)
#define SIM_FCFG2_MAXADDR0_MASK                  (0x7F000000U)
#define SIM_FCFG2_MAXADDR0_SHIFT                 (24U)
#define SIM_FCFG2_MAXADDR0(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_FCFG2_MAXADDR0_SHIFT)) & SIM_FCFG2_MAXADDR0_MASK)

/*! @name UIDH - Unique Identification Register High */
#define SIM_UIDH_UID127_96_MASK                  (0xFFFFFFFFU)
#define SIM_UIDH_UID127_96_SHIFT                 (0U)
#define SIM_UIDH_UID127_96(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDH_UID127_96_SHIFT)) & SIM_UIDH_UID127_96_MASK)

/*! @name UIDMH - Unique Identification Register Mid-High */
#define SIM_UIDMH_UID95_64_MASK                  (0xFFFFFFFFU)
#define SIM_UIDMH_UID95_64_SHIFT                 (0U)
#define SIM_UIDMH_UID95_64(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDMH_UID95_64_SHIFT)) & SIM_UIDMH_UID95_64_MASK)

/*! @name UIDML - Unique Identification Register Mid Low */
#define SIM_UIDML_UID63_32_MASK                  (0xFFFFFFFFU)
#define SIM_UIDML_UID63_32_SHIFT                 (0U)
#define SIM_UIDML_UID63_32(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDML_UID63_32_SHIFT)) & SIM_UIDML_UID63_32_MASK)

/*! @name UIDL - Unique Identification Register Low */
#define SIM_UIDL_UID31_0_MASK                    (0xFFFFFFFFU)
#define SIM_UIDL_UID31_0_SHIFT                   (0U)
#define SIM_UIDL_UID31_0(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_UIDL_UID31_0_SHIFT)) & SIM_UIDL_UID31_0_MASK)

/*! @name MISCTRL - Miscellaneous Control register */
#define SIM_MISCTRL_SW_TRG_MASK                  (0x1U)
#define SIM_MISCTRL_SW_TRG_SHIFT                 (0U)
#define SIM_MISCTRL_SW_TRG(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL_SW_TRG_SHIFT)) & SIM_MISCTRL_SW_TRG_MASK)
#define SIM_MISCTRL_DMA_INT_SEL_MASK             (0xF0U)
#define SIM_MISCTRL_DMA_INT_SEL_SHIFT            (4U)
#define SIM_MISCTRL_DMA_INT_SEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL_DMA_INT_SEL_SHIFT)) & SIM_MISCTRL_DMA_INT_SEL_MASK)
#define SIM_MISCTRL_UART0ODE_MASK                (0x10000U)
#define SIM_MISCTRL_UART0ODE_SHIFT               (16U)
#define SIM_MISCTRL_UART0ODE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL_UART0ODE_SHIFT)) & SIM_MISCTRL_UART0ODE_MASK)
#define SIM_MISCTRL_UART1ODE_MASK                (0x20000U)
#define SIM_MISCTRL_UART1ODE_SHIFT               (17U)
#define SIM_MISCTRL_UART1ODE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL_UART1ODE_SHIFT)) & SIM_MISCTRL_UART1ODE_MASK)
#define SIM_MISCTRL_UART2ODE_MASK                (0x40000U)
#define SIM_MISCTRL_UART2ODE_SHIFT               (18U)
#define SIM_MISCTRL_UART2ODE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL_UART2ODE_SHIFT)) & SIM_MISCTRL_UART2ODE_MASK)


/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40048000u)
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
#define SMC_PMPROT_AVLP_MASK                     (0x20U)
#define SMC_PMPROT_AVLP_SHIFT                    (5U)
#define SMC_PMPROT_AVLP(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_AVLP_SHIFT)) & SMC_PMPROT_AVLP_MASK)

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
   -- TRGMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Peripheral_Access_Layer TRGMUX Peripheral Access Layer
 * @{
 */

/** TRGMUX - Register Layout Typedef */
typedef struct {
  __IO uint32_t TRGCFG[32];                        /**< TRGMUX Register, array offset: 0x0, array step: 0x4 */
} TRGMUX_Type;

/* ----------------------------------------------------------------------------
   -- TRGMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Register_Masks TRGMUX Register Masks
 * @{
 */

/*! @name TRGCFG - TRGMUX Register */
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
#define TRGMUX_TRGCFG_COUNT                      (32U)


/*!
 * @}
 */ /* end of group TRGMUX_Register_Masks */


/* TRGMUX - Peripheral instance base addresses */
/** Peripheral TRGMUX0 base address */
#define TRGMUX0_BASE                             (0x40062000u)
/** Peripheral TRGMUX0 base pointer */
#define TRGMUX0                                  ((TRGMUX_Type *)TRGMUX0_BASE)
/** Peripheral TRGMUX1 base address */
#define TRGMUX1_BASE                             (0x40063000u)
/** Peripheral TRGMUX1 base pointer */
#define TRGMUX1                                  ((TRGMUX_Type *)TRGMUX1_BASE)
/** Array initializer of TRGMUX peripheral base addresses */
#define TRGMUX_BASE_ADDRS                        { TRGMUX0_BASE, TRGMUX1_BASE }
/** Array initializer of TRGMUX peripheral base pointers */
#define TRGMUX_BASE_PTRS                         { TRGMUX0, TRGMUX1 }
#define TRGMUX_INSTANCE_MASK                      0xF
#define TRGMUX_INSTANCE_SHIFT                     12
#define TRGMUX_PERIPHERAL_MASK                    0xFFF
#define TRGMUX_PERIPHERAL_SHIFT                   0
#define TRGMUX_INSTANCE_0                         0
#define TRGMUX_INSTANCE_1                         1

#define TRGMUX_DMAMUX0_INDEX                     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 0)
#define TRGMUX_EXTOUT0_INDEX                     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 1)
#define TRGMUX_EXTOUT1_INDEX                     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 2)
#define TRGMUX_ADC0_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 3)
#define TRGMUX_ADC1_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 4)
#define TRGMUX_CMP0_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 7)
#define TRGMUX_CMP1_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 8)
#define TRGMUX_FTM0_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 10)
#define TRGMUX_FTM1_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 11)
#define TRGMUX_FTM2_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 12)
#define TRGMUX_PDB0_INDEX                        ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 14)
#define TRGMUX_FLEXIO_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 17)
#define TRGMUX_LPIT0_INDEX                       ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 18)
#define TRGMUX_LPUART0_INDEX                     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 19)
#define TRGMUX_LPUART1_INDEX                     ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 20)
#define TRGMUX_LPI2C0_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 21)
#define TRGMUX_LPI2C1_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 22)
#define TRGMUX_LPSPI0_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 23)
#define TRGMUX_LPSPI1_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 24)
#define TRGMUX_LPTMR0_INDEX                      ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 25)
#define TRGMUX_TSI_INDEX                         ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 26)
#define TRGMUX_PWT_INDEX                         ((uint16_t)((TRGMUX_INSTANCE_0 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 27)
#define TRGMUX_CTRL0_INDEX                       ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 0)
#define TRGMUX_CTRL1_INDEX                       ((uint16_t)((TRGMUX_INSTANCE_1 & TRGMUX_INSTANCE_MASK) << TRGMUX_INSTANCE_SHIFT) | 1)


/*!
 * @}
 */ /* end of group TRGMUX_Peripheral_Access_Layer */


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
  __IO uint32_t MODE;                              /**< TSI MODE Register, offset: 0xC */
  __IO uint32_t MUL0;                              /**< TSI MUTUAL-CAP Register 0, offset: 0x10 */
  __IO uint32_t MUL1;                              /**< TSI MUTUAL-CAP Register 1, offset: 0x14 */
  __IO uint32_t SINC;                              /**< TSI SINC filter Register, offset: 0x18 */
  __IO uint32_t SSC0;                              /**< TSI SSC Register 0, offset: 0x1C */
  __IO uint32_t SSC1;                              /**< TSI SSC Register 0, offset: 0x20 */
  __IO uint32_t SSC2;                              /**< TSI SSC Register 2, offset: 0x24 */
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
#define TSI_GENCS_CLKSOC_SEL_MASK                (0x800U)
#define TSI_GENCS_CLKSOC_SEL_SHIFT               (11U)
#define TSI_GENCS_CLKSOC_SEL(x)                  (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_CLKSOC_SEL_SHIFT)) & TSI_GENCS_CLKSOC_SEL_MASK)
#define TSI_GENCS_RUN_CTRL_MASK                  (0x1000U)
#define TSI_GENCS_RUN_CTRL_SHIFT                 (12U)
#define TSI_GENCS_RUN_CTRL(x)                    (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_RUN_CTRL_SHIFT)) & TSI_GENCS_RUN_CTRL_MASK)
#define TSI_GENCS_TSI_ANA_TEST_MASK              (0xE000U)
#define TSI_GENCS_TSI_ANA_TEST_SHIFT             (13U)
#define TSI_GENCS_TSI_ANA_TEST(x)                (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_TSI_ANA_TEST_SHIFT)) & TSI_GENCS_TSI_ANA_TEST_MASK)
#define TSI_GENCS_DVOLT_MASK                     (0x180000U)
#define TSI_GENCS_DVOLT_SHIFT                    (19U)
#define TSI_GENCS_DVOLT(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_GENCS_DVOLT_SHIFT)) & TSI_GENCS_DVOLT_MASK)
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
#define TSI_DATA_TSICH_MASK                      (0xF8000000U)
#define TSI_DATA_TSICH_SHIFT                     (27U)
#define TSI_DATA_TSICH(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_DATA_TSICH_SHIFT)) & TSI_DATA_TSICH_MASK)

/*! @name TSHD - TSI Threshold Register */
#define TSI_TSHD_THRESL_MASK                     (0xFFFFU)
#define TSI_TSHD_THRESL_SHIFT                    (0U)
#define TSI_TSHD_THRESL(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_TSHD_THRESL_SHIFT)) & TSI_TSHD_THRESL_MASK)
#define TSI_TSHD_THRESH_MASK                     (0xFFFF0000U)
#define TSI_TSHD_THRESH_SHIFT                    (16U)
#define TSI_TSHD_THRESH(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_TSHD_THRESH_SHIFT)) & TSI_TSHD_THRESH_MASK)

/*! @name MODE - TSI MODE Register */
#define TSI_MODE_S_NOISE_MASK                    (0x1U)
#define TSI_MODE_S_NOISE_SHIFT                   (0U)
#define TSI_MODE_S_NOISE(x)                      (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_NOISE_SHIFT)) & TSI_MODE_S_NOISE_MASK)
#define TSI_MODE_MODE_MASK                       (0x2U)
#define TSI_MODE_MODE_SHIFT                      (1U)
#define TSI_MODE_MODE(x)                         (((uint32_t)(((uint32_t)(x)) << TSI_MODE_MODE_SHIFT)) & TSI_MODE_MODE_MASK)
#define TSI_MODE_SETCLK_MASK                     (0x60U)
#define TSI_MODE_SETCLK_SHIFT                    (5U)
#define TSI_MODE_SETCLK(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_MODE_SETCLK_SHIFT)) & TSI_MODE_SETCLK_MASK)
#define TSI_MODE_S_XCH_MASK                      (0x7000U)
#define TSI_MODE_S_XCH_SHIFT                     (12U)
#define TSI_MODE_S_XCH(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_XCH_SHIFT)) & TSI_MODE_S_XCH_MASK)
#define TSI_MODE_S_XIN_MASK                      (0x40000U)
#define TSI_MODE_S_XIN_SHIFT                     (18U)
#define TSI_MODE_S_XIN(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_XIN_SHIFT)) & TSI_MODE_S_XIN_MASK)
#define TSI_MODE_S_CTRIM_MASK                    (0x380000U)
#define TSI_MODE_S_CTRIM_SHIFT                   (19U)
#define TSI_MODE_S_CTRIM(x)                      (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_CTRIM_SHIFT)) & TSI_MODE_S_CTRIM_MASK)
#define TSI_MODE_S_SEN_MASK                      (0x400000U)
#define TSI_MODE_S_SEN_SHIFT                     (22U)
#define TSI_MODE_S_SEN(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_SEN_SHIFT)) & TSI_MODE_S_SEN_MASK)
#define TSI_MODE_S_W_SHIELD_MASK                 (0x800000U)
#define TSI_MODE_S_W_SHIELD_SHIFT                (23U)
#define TSI_MODE_S_W_SHIELD(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_W_SHIELD_SHIFT)) & TSI_MODE_S_W_SHIELD_MASK)
#define TSI_MODE_S_XDN_MASK                      (0x70000000U)
#define TSI_MODE_S_XDN_SHIFT                     (28U)
#define TSI_MODE_S_XDN(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_MODE_S_XDN_SHIFT)) & TSI_MODE_S_XDN_MASK)

/*! @name MUL0 - TSI MUTUAL-CAP Register 0 */
#define TSI_MUL0_M_SEL_RX_MASK                   (0x7U)
#define TSI_MUL0_M_SEL_RX_SHIFT                  (0U)
#define TSI_MUL0_M_SEL_RX(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_MUL0_M_SEL_RX_SHIFT)) & TSI_MUL0_M_SEL_RX_MASK)
#define TSI_MUL0_M_SEL_TX_MASK                   (0x70U)
#define TSI_MUL0_M_SEL_TX_SHIFT                  (4U)
#define TSI_MUL0_M_SEL_TX(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_MUL0_M_SEL_TX_SHIFT)) & TSI_MUL0_M_SEL_TX_MASK)
#define TSI_MUL0_M_SEN_RES_MASK                  (0xF00U)
#define TSI_MUL0_M_SEN_RES_SHIFT                 (8U)
#define TSI_MUL0_M_SEN_RES(x)                    (((uint32_t)(((uint32_t)(x)) << TSI_MUL0_M_SEN_RES_SHIFT)) & TSI_MUL0_M_SEN_RES_MASK)
#define TSI_MUL0_M_PRE_RES_MASK                  (0xE000U)
#define TSI_MUL0_M_PRE_RES_SHIFT                 (13U)
#define TSI_MUL0_M_PRE_RES(x)                    (((uint32_t)(((uint32_t)(x)) << TSI_MUL0_M_PRE_RES_SHIFT)) & TSI_MUL0_M_PRE_RES_MASK)
#define TSI_MUL0_M_PRE_CURRENT_MASK              (0xE0000000U)
#define TSI_MUL0_M_PRE_CURRENT_SHIFT             (29U)
#define TSI_MUL0_M_PRE_CURRENT(x)                (((uint32_t)(((uint32_t)(x)) << TSI_MUL0_M_PRE_CURRENT_SHIFT)) & TSI_MUL0_M_PRE_CURRENT_MASK)

/*! @name MUL1 - TSI MUTUAL-CAP Register 1 */
#define TSI_MUL1_M_NMIR_CTRL_MASK                (0x1U)
#define TSI_MUL1_M_NMIR_CTRL_SHIFT               (0U)
#define TSI_MUL1_M_NMIR_CTRL(x)                  (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_NMIR_CTRL_SHIFT)) & TSI_MUL1_M_NMIR_CTRL_MASK)
#define TSI_MUL1_M_NMIRROR_MASK                  (0x6U)
#define TSI_MUL1_M_NMIRROR_SHIFT                 (1U)
#define TSI_MUL1_M_NMIRROR(x)                    (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_NMIRROR_SHIFT)) & TSI_MUL1_M_NMIRROR_MASK)
#define TSI_MUL1_M_PMIRRORR_MASK                 (0x18U)
#define TSI_MUL1_M_PMIRRORR_SHIFT                (3U)
#define TSI_MUL1_M_PMIRRORR(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_PMIRRORR_SHIFT)) & TSI_MUL1_M_PMIRRORR_MASK)
#define TSI_MUL1_M_PMIRRORL_MASK                 (0xE0U)
#define TSI_MUL1_M_PMIRRORL_SHIFT                (5U)
#define TSI_MUL1_M_PMIRRORL(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_PMIRRORL_SHIFT)) & TSI_MUL1_M_PMIRRORL_MASK)
#define TSI_MUL1_M_VPRE_CHOOSE_MASK              (0x10000U)
#define TSI_MUL1_M_VPRE_CHOOSE_SHIFT             (16U)
#define TSI_MUL1_M_VPRE_CHOOSE(x)                (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_VPRE_CHOOSE_SHIFT)) & TSI_MUL1_M_VPRE_CHOOSE_MASK)
#define TSI_MUL1_M_MODE_MASK                     (0x40000U)
#define TSI_MUL1_M_MODE_SHIFT                    (18U)
#define TSI_MUL1_M_MODE(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_MODE_SHIFT)) & TSI_MUL1_M_MODE_MASK)
#define TSI_MUL1_M_SEN_BOOST_MASK                (0xF80000U)
#define TSI_MUL1_M_SEN_BOOST_SHIFT               (19U)
#define TSI_MUL1_M_SEN_BOOST(x)                  (((uint32_t)(((uint32_t)(x)) << TSI_MUL1_M_SEN_BOOST_SHIFT)) & TSI_MUL1_M_SEN_BOOST_MASK)

/*! @name SINC - TSI SINC filter Register */
#define TSI_SINC_SSC_CONTROL_OUT_MASK            (0x1U)
#define TSI_SINC_SSC_CONTROL_OUT_SHIFT           (0U)
#define TSI_SINC_SSC_CONTROL_OUT(x)              (((uint32_t)(((uint32_t)(x)) << TSI_SINC_SSC_CONTROL_OUT_SHIFT)) & TSI_SINC_SSC_CONTROL_OUT_MASK)
#define TSI_SINC_SINC_VALID_MASK                 (0x2U)
#define TSI_SINC_SINC_VALID_SHIFT                (1U)
#define TSI_SINC_SINC_VALID(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_SINC_SINC_VALID_SHIFT)) & TSI_SINC_SINC_VALID_MASK)
#define TSI_SINC_SINC_OVERFLOW_FLAG_MASK         (0x4U)
#define TSI_SINC_SINC_OVERFLOW_FLAG_SHIFT        (2U)
#define TSI_SINC_SINC_OVERFLOW_FLAG(x)           (((uint32_t)(((uint32_t)(x)) << TSI_SINC_SINC_OVERFLOW_FLAG_SHIFT)) & TSI_SINC_SINC_OVERFLOW_FLAG_MASK)
#define TSI_SINC_SWITCH_ENABLE_MASK              (0x8U)
#define TSI_SINC_SWITCH_ENABLE_SHIFT             (3U)
#define TSI_SINC_SWITCH_ENABLE(x)                (((uint32_t)(((uint32_t)(x)) << TSI_SINC_SWITCH_ENABLE_SHIFT)) & TSI_SINC_SWITCH_ENABLE_MASK)
#define TSI_SINC_DECIMATION_MASK                 (0x1F0000U)
#define TSI_SINC_DECIMATION_SHIFT                (16U)
#define TSI_SINC_DECIMATION(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_SINC_DECIMATION_SHIFT)) & TSI_SINC_DECIMATION_MASK)
#define TSI_SINC_ORDER_MASK                      (0x200000U)
#define TSI_SINC_ORDER_SHIFT                     (21U)
#define TSI_SINC_ORDER(x)                        (((uint32_t)(((uint32_t)(x)) << TSI_SINC_ORDER_SHIFT)) & TSI_SINC_ORDER_MASK)
#define TSI_SINC_CUTOFF_MASK                     (0xF000000U)
#define TSI_SINC_CUTOFF_SHIFT                    (24U)
#define TSI_SINC_CUTOFF(x)                       (((uint32_t)(((uint32_t)(x)) << TSI_SINC_CUTOFF_SHIFT)) & TSI_SINC_CUTOFF_MASK)

/*! @name SSC0 - TSI SSC Register 0 */
#define TSI_SSC0_SSC_PRESCALE_NUM_MASK           (0xFFU)
#define TSI_SSC0_SSC_PRESCALE_NUM_SHIFT          (0U)
#define TSI_SSC0_SSC_PRESCALE_NUM(x)             (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_SSC_PRESCALE_NUM_SHIFT)) & TSI_SSC0_SSC_PRESCALE_NUM_MASK)
#define TSI_SSC0_BASE_NOCHARGE_NUM_MASK          (0xF0000U)
#define TSI_SSC0_BASE_NOCHARGE_NUM_SHIFT         (16U)
#define TSI_SSC0_BASE_NOCHARGE_NUM(x)            (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_BASE_NOCHARGE_NUM_SHIFT)) & TSI_SSC0_BASE_NOCHARGE_NUM_MASK)
#define TSI_SSC0_CHARGE_NUM_MASK                 (0xF00000U)
#define TSI_SSC0_CHARGE_NUM_SHIFT                (20U)
#define TSI_SSC0_CHARGE_NUM(x)                   (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_CHARGE_NUM_SHIFT)) & TSI_SSC0_CHARGE_NUM_MASK)
#define TSI_SSC0_SSC_CONTROL_REVERSE_MASK        (0x1000000U)
#define TSI_SSC0_SSC_CONTROL_REVERSE_SHIFT       (24U)
#define TSI_SSC0_SSC_CONTROL_REVERSE(x)          (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_SSC_CONTROL_REVERSE_SHIFT)) & TSI_SSC0_SSC_CONTROL_REVERSE_MASK)
#define TSI_SSC0_SSC_MODE_MASK                   (0x6000000U)
#define TSI_SSC0_SSC_MODE_SHIFT                  (25U)
#define TSI_SSC0_SSC_MODE(x)                     (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_SSC_MODE_SHIFT)) & TSI_SSC0_SSC_MODE_MASK)
#define TSI_SSC0_PRBS_OUTSEL_MASK                (0xF0000000U)
#define TSI_SSC0_PRBS_OUTSEL_SHIFT               (28U)
#define TSI_SSC0_PRBS_OUTSEL(x)                  (((uint32_t)(((uint32_t)(x)) << TSI_SSC0_PRBS_OUTSEL_SHIFT)) & TSI_SSC0_PRBS_OUTSEL_MASK)

/*! @name SSC1 - TSI SSC Register 0 */
#define TSI_SSC1_PRBS_SEED_LO_MASK               (0xFFU)
#define TSI_SSC1_PRBS_SEED_LO_SHIFT              (0U)
#define TSI_SSC1_PRBS_SEED_LO(x)                 (((uint32_t)(((uint32_t)(x)) << TSI_SSC1_PRBS_SEED_LO_SHIFT)) & TSI_SSC1_PRBS_SEED_LO_MASK)
#define TSI_SSC1_PRBS_SEED_HI_MASK               (0xFF00U)
#define TSI_SSC1_PRBS_SEED_HI_SHIFT              (8U)
#define TSI_SSC1_PRBS_SEED_HI(x)                 (((uint32_t)(((uint32_t)(x)) << TSI_SSC1_PRBS_SEED_HI_SHIFT)) & TSI_SSC1_PRBS_SEED_HI_MASK)
#define TSI_SSC1_PRBS_WEIGHT_LO_MASK             (0xFF0000U)
#define TSI_SSC1_PRBS_WEIGHT_LO_SHIFT            (16U)
#define TSI_SSC1_PRBS_WEIGHT_LO(x)               (((uint32_t)(((uint32_t)(x)) << TSI_SSC1_PRBS_WEIGHT_LO_SHIFT)) & TSI_SSC1_PRBS_WEIGHT_LO_MASK)
#define TSI_SSC1_PRBS_WEIGHT_HI_MASK             (0xFF000000U)
#define TSI_SSC1_PRBS_WEIGHT_HI_SHIFT            (24U)
#define TSI_SSC1_PRBS_WEIGHT_HI(x)               (((uint32_t)(((uint32_t)(x)) << TSI_SSC1_PRBS_WEIGHT_HI_SHIFT)) & TSI_SSC1_PRBS_WEIGHT_HI_MASK)

/*! @name SSC2 - TSI SSC Register 2 */
#define TSI_SSC2_MOVE_REPEAT_NUM_MASK            (0x1FU)
#define TSI_SSC2_MOVE_REPEAT_NUM_SHIFT           (0U)
#define TSI_SSC2_MOVE_REPEAT_NUM(x)              (((uint32_t)(((uint32_t)(x)) << TSI_SSC2_MOVE_REPEAT_NUM_SHIFT)) & TSI_SSC2_MOVE_REPEAT_NUM_MASK)
#define TSI_SSC2_MOVE_STEPS_NUM_MASK             (0x700U)
#define TSI_SSC2_MOVE_STEPS_NUM_SHIFT            (8U)
#define TSI_SSC2_MOVE_STEPS_NUM(x)               (((uint32_t)(((uint32_t)(x)) << TSI_SSC2_MOVE_STEPS_NUM_SHIFT)) & TSI_SSC2_MOVE_STEPS_NUM_MASK)
#define TSI_SSC2_MOVE_NOCHARGE_MAX_MASK          (0x3F0000U)
#define TSI_SSC2_MOVE_NOCHARGE_MAX_SHIFT         (16U)
#define TSI_SSC2_MOVE_NOCHARGE_MAX(x)            (((uint32_t)(((uint32_t)(x)) << TSI_SSC2_MOVE_NOCHARGE_MAX_SHIFT)) & TSI_SSC2_MOVE_NOCHARGE_MAX_MASK)
#define TSI_SSC2_MOVE_NOCHARGE_MIN_MASK          (0xF0000000U)
#define TSI_SSC2_MOVE_NOCHARGE_MIN_SHIFT         (28U)
#define TSI_SSC2_MOVE_NOCHARGE_MIN(x)            (((uint32_t)(((uint32_t)(x)) << TSI_SSC2_MOVE_NOCHARGE_MIN_SHIFT)) & TSI_SSC2_MOVE_NOCHARGE_MIN_MASK)


/*!
 * @}
 */ /* end of group TSI_Register_Masks */


/* TSI - Peripheral instance base addresses */
/** Peripheral TSI base address */
#define TSI_BASE                                 (0x40045000u)
/** Peripheral TSI base pointer */
#define TSI                                      ((TSI_Type *)TSI_BASE)
/** Array initializer of TSI peripheral base addresses */
#define TSI_BASE_ADDRS                           { TSI_BASE }
/** Array initializer of TSI peripheral base pointers */
#define TSI_BASE_PTRS                            { TSI }
/** Interrupt vectors for the TSI peripheral type */
#define TSI_IRQS                                 { TSI_IRQn }

/*!
 * @}
 */ /* end of group TSI_Peripheral_Access_Layer */


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
#define WDOG_CS_RCS_MASK                         (0x400U)
#define WDOG_CS_RCS_SHIFT                        (10U)
#define WDOG_CS_RCS(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_RCS_SHIFT)) & WDOG_CS_RCS_MASK)
#define WDOG_CS_ULK_MASK                         (0x800U)
#define WDOG_CS_ULK_SHIFT                        (11U)
#define WDOG_CS_ULK(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_ULK_SHIFT)) & WDOG_CS_ULK_MASK)
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

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MKE15Z7_H_ */

