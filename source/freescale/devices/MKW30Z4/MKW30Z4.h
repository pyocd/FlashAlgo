/*
** ###################################################################
**     Processor:           MKW30Z160VHM4
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    MKW40Z160RM, Rev. 1.1, 4/2015
**     Version:             rev. 1.2, 2015-05-07
**     Build:               b160126
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MKW30Z4
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
**     - rev. 1.0 (2014-08-27)
**         Initial version.
**     - rev. 1.1 (2015-03-05)
**         Update with reference manual rev 1.0
**     - rev. 1.2 (2015-05-07)
**         Update with reference manual rev 1.1
**
** ###################################################################
*/

/*!
 * @file MKW30Z4.h
 * @version 1.2
 * @date 2015-05-07
 * @brief CMSIS Peripheral Access Layer for MKW30Z4
 *
 * CMSIS Peripheral Access Layer for MKW30Z4
 */

#ifndef _MKW30Z4_H_
#define _MKW30Z4_H_                              /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0100U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0002U


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
  DMA0_IRQn                    = 0,                /**< DMA channel 0 transfer complete */
  DMA1_IRQn                    = 1,                /**< DMA channel 1 transfer complete */
  DMA2_IRQn                    = 2,                /**< DMA channel 2 transfer complete */
  DMA3_IRQn                    = 3,                /**< DMA channel 3 transfer complete */
  Reserved20_IRQn              = 4,                /**< Reserved interrupt */
  FTFA_IRQn                    = 5,                /**< Command complete and read collision */
  LVD_LVW_DCDC_IRQn            = 6,                /**< Low-voltage detect, low-voltage warning, DCDC */
  LLWU_IRQn                    = 7,                /**< Low leakage wakeup Unit */
  I2C0_IRQn                    = 8,                /**< I2C0 interrupt */
  I2C1_IRQn                    = 9,                /**< I2C1 interrupt */
  SPI0_IRQn                    = 10,               /**< SPI0 single interrupt vector for all sources */
  TSI0_IRQn                    = 11,               /**< TSI0 single interrupt vector for all sources */
  LPUART0_IRQn                 = 12,               /**< LPUART0 status and error */
  TRNG0_IRQn                   = 13,               /**< TRNG0 interrupt */
  CMT_IRQn                     = 14,               /**< CMT interrupt */
  ADC0_IRQn                    = 15,               /**< ADC0 interrupt */
  CMP0_IRQn                    = 16,               /**< CMP0 interrupt */
  TPM0_IRQn                    = 17,               /**< TPM0 single interrupt vector for all sources */
  TPM1_IRQn                    = 18,               /**< TPM1 single interrupt vector for all sources */
  TPM2_IRQn                    = 19,               /**< TPM2 single interrupt vector for all sources */
  RTC_IRQn                     = 20,               /**< RTC alarm */
  RTC_Seconds_IRQn             = 21,               /**< RTC seconds */
  PIT_IRQn                     = 22,               /**< PIT interrupt */
  LTC0_IRQn                    = 23,               /**< LTC0 interrupt */
  BTLL_RSIM_IRQn               = 24,               /**< BTLL and RSIM interrupt */
  DAC0_IRQn                    = 25,               /**< DAC0 interrupt */
  ZigBee_IRQn                  = 26,               /**< ZigBee interrupt */
  MCG_IRQn                     = 27,               /**< MCG interrupt */
  LPTMR0_IRQn                  = 28,               /**< LPTMR0 interrupt */
  SPI1_IRQn                    = 29,               /**< SPI1 single interrupt vector for all sources */
  PORTA_IRQn                   = 30,               /**< PORTA Pin detect */
  PORTB_PORTC_IRQn             = 31                /**< PORTB and PORTC Pin detect */
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
#include "system_MKW30Z4.h"            /* Device specific configuration file */

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
    kDmaRequestMux0Reserved1        = 1|0x100U,    /**< Reserved1 */
    kDmaRequestMux0LPUART0Rx        = 2|0x100U,    /**< LPUART0 Receive. */
    kDmaRequestMux0LPUART0Tx        = 3|0x100U,    /**< LPUART0 Transmit. */
    kDmaRequestMux0Reserved4        = 4|0x100U,    /**< Reserved4 */
    kDmaRequestMux0Reserved5        = 5|0x100U,    /**< Reserved5 */
    kDmaRequestMux0Reserved6        = 6|0x100U,    /**< Reserved6 */
    kDmaRequestMux0Reserved7        = 7|0x100U,    /**< Reserved7 */
    kDmaRequestMux0Reserved8        = 8|0x100U,    /**< Reserved8 */
    kDmaRequestMux0Reserved9        = 9|0x100U,    /**< Reserved9 */
    kDmaRequestMux0Reserved10       = 10|0x100U,   /**< Reserved10 */
    kDmaRequestMux0Reserved11       = 11|0x100U,   /**< Reserved11 */
    kDmaRequestMux0Reserved12       = 12|0x100U,   /**< Reserved12 */
    kDmaRequestMux0Reserved13       = 13|0x100U,   /**< Reserved13 */
    kDmaRequestMux0Reserved14       = 14|0x100U,   /**< Reserved14 */
    kDmaRequestMux0Reserved15       = 15|0x100U,   /**< Reserved15 */
    kDmaRequestMux0SPI0Rx           = 16|0x100U,   /**< SPI0 Receive. */
    kDmaRequestMux0SPI0Tx           = 17|0x100U,   /**< SPI0 Transmit. */
    kDmaRequestMux0SPI1Rx           = 18|0x100U,   /**< SPI1 Receive. */
    kDmaRequestMux0SPI1Tx           = 19|0x100U,   /**< SPI1 Transmit. */
    kDmaRequestMux0AESAInputFIFO    = 20|0x100U,   /**< AESA Input FIFO. */
    kDmaRequestMux0AESAOutputFIFO   = 21|0x100U,   /**< AESA Output FIFO. */
    kDmaRequestMux0I2C0             = 22|0x100U,   /**< I2C0. */
    kDmaRequestMux0I2C1             = 23|0x100U,   /**< I2C1. */
    kDmaRequestMux0TPM0Channel0     = 24|0x100U,   /**< TPM0 C0V. */
    kDmaRequestMux0TPM0Channel1     = 25|0x100U,   /**< TPM0 C1V. */
    kDmaRequestMux0TPM0Channel2     = 26|0x100U,   /**< TPM0 C2V. */
    kDmaRequestMux0TPM0Channel3     = 27|0x100U,   /**< TPM0 C3V. */
    kDmaRequestMux0Reserved28       = 28|0x100U,   /**< Reserved28 */
    kDmaRequestMux0Reserved29       = 29|0x100U,   /**< Reserved29 */
    kDmaRequestMux0Reserved30       = 30|0x100U,   /**< Reserved30 */
    kDmaRequestMux0Reserved31       = 31|0x100U,   /**< Reserved31 */
    kDmaRequestMux0TPM1Channel0     = 32|0x100U,   /**< TPM1 C0V. */
    kDmaRequestMux0TPM1Channel1     = 33|0x100U,   /**< TPM1 C1V. */
    kDmaRequestMux0TPM2Channel0     = 34|0x100U,   /**< TPM2 C0V. */
    kDmaRequestMux0TPM2Channel1     = 35|0x100U,   /**< TPM2 C1V. */
    kDmaRequestMux0Reserved36       = 36|0x100U,   /**< Reserved36 */
    kDmaRequestMux0Reserved37       = 37|0x100U,   /**< Reserved37 */
    kDmaRequestMux0Reserved38       = 38|0x100U,   /**< Reserved38 */
    kDmaRequestMux0Reserved39       = 39|0x100U,   /**< Reserved39 */
    kDmaRequestMux0ADC0             = 40|0x100U,   /**< ADC0. */
    kDmaRequestMux0Reserved41       = 41|0x100U,   /**< Reserved41 */
    kDmaRequestMux0CMP0             = 42|0x100U,   /**< CMP0. */
    kDmaRequestMux0Reserved43       = 43|0x100U,   /**< Reserved43 */
    kDmaRequestMux0Reserved44       = 44|0x100U,   /**< Reserved44 */
    kDmaRequestMux0DAC0             = 45|0x100U,   /**< DAC0. */
    kDmaRequestMux0Reserved46       = 46|0x100U,   /**< Reserved46 */
    kDmaRequestMux0CMT              = 47|0x100U,   /**< CMT. */
    kDmaRequestMux0Reserved48       = 48|0x100U,   /**< Reserved48 */
    kDmaRequestMux0PortA            = 49|0x100U,   /**< PTA. */
    kDmaRequestMux0PortB            = 50|0x100U,   /**< PTB. */
    kDmaRequestMux0PortC            = 51|0x100U,   /**< PTC. */
    kDmaRequestMux0Reserved52       = 52|0x100U,   /**< Reserved52 */
    kDmaRequestMux0Reserved53       = 53|0x100U,   /**< Reserved53 */
    kDmaRequestMux0TPM0Overflow     = 54|0x100U,   /**< TPM0. */
    kDmaRequestMux0TPM1Overflow     = 55|0x100U,   /**< TPM1. */
    kDmaRequestMux0TPM2Overflow     = 56|0x100U,   /**< TPM2. */
    kDmaRequestMux0TSI0             = 57|0x100U,   /**< TSI0. */
    kDmaRequestMux0Reserved58       = 58|0x100U,   /**< Reserved58 */
    kDmaRequestMux0Reserved59       = 59|0x100U,   /**< Reserved59 */
    kDmaRequestMux0AlwaysOn60       = 60|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn61       = 61|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn62       = 62|0x100U,   /**< DMAMUX Always Enabled slot. */
    kDmaRequestMux0AlwaysOn63       = 63|0x100U,   /**< DMAMUX Always Enabled slot. */
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
   -- BLE_RF_REGS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup BLE_RF_REGS_Peripheral_Access_Layer BLE_RF_REGS Peripheral Access Layer
 * @{
 */

/** BLE_RF_REGS - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[3328];
  __I  uint16_t BLE_PART_ID;                       /**< Bluetooth Low Energy Part ID, offset: 0xD00 */
       uint8_t RESERVED_1[2];
  __I  uint16_t DSM_STATUS;                        /**< DSM Status, offset: 0xD04 */
       uint8_t RESERVED_2[2];
  __IO uint16_t BLE_AFC;                           /**< Bluetooth Low Energy AFC, offset: 0xD08 */
       uint8_t RESERVED_3[2];
  __IO uint16_t BLE_BSM;                           /**< Bluetooth Low Energy BSM, offset: 0xD0C */
} BLE_RF_REGS_Type;

/* ----------------------------------------------------------------------------
   -- BLE_RF_REGS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup BLE_RF_REGS_Register_Masks BLE_RF_REGS Register Masks
 * @{
 */

/*! @name BLE_PART_ID - Bluetooth Low Energy Part ID */
#define BLE_RF_REGS_BLE_PART_ID_BLE_PART_ID_MASK (0xFFFFU)
#define BLE_RF_REGS_BLE_PART_ID_BLE_PART_ID_SHIFT (0U)
#define BLE_RF_REGS_BLE_PART_ID_BLE_PART_ID(x)   (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_BLE_PART_ID_BLE_PART_ID_SHIFT)) & BLE_RF_REGS_BLE_PART_ID_BLE_PART_ID_MASK)

/*! @name DSM_STATUS - DSM Status */
#define BLE_RF_REGS_DSM_STATUS_ORF_SYSCLK_REQ_MASK (0x1U)
#define BLE_RF_REGS_DSM_STATUS_ORF_SYSCLK_REQ_SHIFT (0U)
#define BLE_RF_REGS_DSM_STATUS_ORF_SYSCLK_REQ(x) (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_DSM_STATUS_ORF_SYSCLK_REQ_SHIFT)) & BLE_RF_REGS_DSM_STATUS_ORF_SYSCLK_REQ_MASK)
#define BLE_RF_REGS_DSM_STATUS_RIF_LL_ACTIVE_MASK (0x2U)
#define BLE_RF_REGS_DSM_STATUS_RIF_LL_ACTIVE_SHIFT (1U)
#define BLE_RF_REGS_DSM_STATUS_RIF_LL_ACTIVE(x)  (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_DSM_STATUS_RIF_LL_ACTIVE_SHIFT)) & BLE_RF_REGS_DSM_STATUS_RIF_LL_ACTIVE_MASK)

/*! @name BLE_AFC - Bluetooth Low Energy AFC */
#define BLE_RF_REGS_BLE_AFC_BLE_AFC_MASK         (0x3FFFU)
#define BLE_RF_REGS_BLE_AFC_BLE_AFC_SHIFT        (0U)
#define BLE_RF_REGS_BLE_AFC_BLE_AFC(x)           (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_BLE_AFC_BLE_AFC_SHIFT)) & BLE_RF_REGS_BLE_AFC_BLE_AFC_MASK)
#define BLE_RF_REGS_BLE_AFC_LATCH_AFC_ON_ACCESS_MATCH_MASK (0x8000U)
#define BLE_RF_REGS_BLE_AFC_LATCH_AFC_ON_ACCESS_MATCH_SHIFT (15U)
#define BLE_RF_REGS_BLE_AFC_LATCH_AFC_ON_ACCESS_MATCH(x) (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_BLE_AFC_LATCH_AFC_ON_ACCESS_MATCH_SHIFT)) & BLE_RF_REGS_BLE_AFC_LATCH_AFC_ON_ACCESS_MATCH_MASK)

/*! @name BLE_BSM - Bluetooth Low Energy BSM */
#define BLE_RF_REGS_BLE_BSM_BSM_EN_BLE_MASK      (0x1U)
#define BLE_RF_REGS_BLE_BSM_BSM_EN_BLE_SHIFT     (0U)
#define BLE_RF_REGS_BLE_BSM_BSM_EN_BLE(x)        (((uint16_t)(((uint16_t)(x)) << BLE_RF_REGS_BLE_BSM_BSM_EN_BLE_SHIFT)) & BLE_RF_REGS_BLE_BSM_BSM_EN_BLE_MASK)


/*!
 * @}
 */ /* end of group BLE_RF_REGS_Register_Masks */


/* BLE_RF_REGS - Peripheral instance base addresses */
/** Peripheral BLE_RF_REGS base address */
#define BLE_RF_REGS_BASE                         (0x4005B000u)
/** Peripheral BLE_RF_REGS base pointer */
#define BLE_RF_REGS                              ((BLE_RF_REGS_Type *)BLE_RF_REGS_BASE)
/** Array initializer of BLE_RF_REGS peripheral base addresses */
#define BLE_RF_REGS_BASE_ADDRS                   { BLE_RF_REGS_BASE }
/** Array initializer of BLE_RF_REGS peripheral base pointers */
#define BLE_RF_REGS_BASE_PTRS                    { BLE_RF_REGS }

/*!
 * @}
 */ /* end of group BLE_RF_REGS_Peripheral_Access_Layer */


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
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMP0_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMP0 }
/** Interrupt vectors for the CMP peripheral type */
#define CMP_IRQS                                 { CMP0_IRQn }

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
  } DAT[2];
       uint8_t RESERVED_0[28];
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
#define DAC_DATL_COUNT                           (2U)

/*! @name DATH - DAC Data High Register */
#define DAC_DATH_DATA1_MASK                      (0xFU)
#define DAC_DATH_DATA1_SHIFT                     (0U)
#define DAC_DATH_DATA1(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_DATH_DATA1_SHIFT)) & DAC_DATH_DATA1_MASK)

/* The count of DAC_DATH */
#define DAC_DATH_COUNT                           (2U)

/*! @name SR - DAC Status Register */
#define DAC_SR_DACBFRPBF_MASK                    (0x1U)
#define DAC_SR_DACBFRPBF_SHIFT                   (0U)
#define DAC_SR_DACBFRPBF(x)                      (((uint8_t)(((uint8_t)(x)) << DAC_SR_DACBFRPBF_SHIFT)) & DAC_SR_DACBFRPBF_MASK)
#define DAC_SR_DACBFRPTF_MASK                    (0x2U)
#define DAC_SR_DACBFRPTF_SHIFT                   (1U)
#define DAC_SR_DACBFRPTF(x)                      (((uint8_t)(((uint8_t)(x)) << DAC_SR_DACBFRPTF_SHIFT)) & DAC_SR_DACBFRPTF_MASK)

/*! @name C0 - DAC Control Register */
#define DAC_C0_DACBBIEN_MASK                     (0x1U)
#define DAC_C0_DACBBIEN_SHIFT                    (0U)
#define DAC_C0_DACBBIEN(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACBBIEN_SHIFT)) & DAC_C0_DACBBIEN_MASK)
#define DAC_C0_DACBTIEN_MASK                     (0x2U)
#define DAC_C0_DACBTIEN_SHIFT                    (1U)
#define DAC_C0_DACBTIEN(x)                       (((uint8_t)(((uint8_t)(x)) << DAC_C0_DACBTIEN_SHIFT)) & DAC_C0_DACBTIEN_MASK)
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
#define DAC_C1_DACBFMD_MASK                      (0x4U)
#define DAC_C1_DACBFMD_SHIFT                     (2U)
#define DAC_C1_DACBFMD(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C1_DACBFMD_SHIFT)) & DAC_C1_DACBFMD_MASK)
#define DAC_C1_DMAEN_MASK                        (0x80U)
#define DAC_C1_DMAEN_SHIFT                       (7U)
#define DAC_C1_DMAEN(x)                          (((uint8_t)(((uint8_t)(x)) << DAC_C1_DMAEN_SHIFT)) & DAC_C1_DMAEN_MASK)

/*! @name C2 - DAC Control Register 2 */
#define DAC_C2_DACBFUP_MASK                      (0x1U)
#define DAC_C2_DACBFUP_SHIFT                     (0U)
#define DAC_C2_DACBFUP(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C2_DACBFUP_SHIFT)) & DAC_C2_DACBFUP_MASK)
#define DAC_C2_DACBFRP_MASK                      (0x10U)
#define DAC_C2_DACBFRP_SHIFT                     (4U)
#define DAC_C2_DACBFRP(x)                        (((uint8_t)(((uint8_t)(x)) << DAC_C2_DACBFRP_SHIFT)) & DAC_C2_DACBFRP_MASK)


/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DAC0 base address */
#define DAC0_BASE                                (0x4003F000u)
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
   -- DCDC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DCDC_Peripheral_Access_Layer DCDC Peripheral Access Layer
 * @{
 */

/** DCDC - Register Layout Typedef */
typedef struct {
  __IO uint32_t REG0;                              /**< DCDC REGISTER 0, offset: 0x0 */
  __IO uint32_t REG1;                              /**< DCDC REGISTER 1, offset: 0x4 */
  __IO uint32_t REG2;                              /**< DCDC REGISTER 2, offset: 0x8 */
  __IO uint32_t REG3;                              /**< DCDC REGISTER 3, offset: 0xC */
  __IO uint32_t REG4;                              /**< DCDC REGISTER 4, offset: 0x10 */
       uint8_t RESERVED_0[4];
  __IO uint32_t REG6;                              /**< DCDC REGISTER 6, offset: 0x18 */
  __IO uint32_t REG7;                              /**< DCDC REGISTER 7, offset: 0x1C */
} DCDC_Type;

/* ----------------------------------------------------------------------------
   -- DCDC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DCDC_Register_Masks DCDC Register Masks
 * @{
 */

/*! @name REG0 - DCDC REGISTER 0 */
#define DCDC_REG0_DCDC_DISABLE_AUTO_CLK_SWITCH_MASK (0x2U)
#define DCDC_REG0_DCDC_DISABLE_AUTO_CLK_SWITCH_SHIFT (1U)
#define DCDC_REG0_DCDC_DISABLE_AUTO_CLK_SWITCH(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_DISABLE_AUTO_CLK_SWITCH_SHIFT)) & DCDC_REG0_DCDC_DISABLE_AUTO_CLK_SWITCH_MASK)
#define DCDC_REG0_DCDC_SEL_CLK_MASK              (0x4U)
#define DCDC_REG0_DCDC_SEL_CLK_SHIFT             (2U)
#define DCDC_REG0_DCDC_SEL_CLK(x)                (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_SEL_CLK_SHIFT)) & DCDC_REG0_DCDC_SEL_CLK_MASK)
#define DCDC_REG0_DCDC_PWD_OSC_INT_MASK          (0x8U)
#define DCDC_REG0_DCDC_PWD_OSC_INT_SHIFT         (3U)
#define DCDC_REG0_DCDC_PWD_OSC_INT(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_PWD_OSC_INT_SHIFT)) & DCDC_REG0_DCDC_PWD_OSC_INT_MASK)
#define DCDC_REG0_DCDC_LP_DF_CMP_ENABLE_MASK     (0x200U)
#define DCDC_REG0_DCDC_LP_DF_CMP_ENABLE_SHIFT    (9U)
#define DCDC_REG0_DCDC_LP_DF_CMP_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_LP_DF_CMP_ENABLE_SHIFT)) & DCDC_REG0_DCDC_LP_DF_CMP_ENABLE_MASK)
#define DCDC_REG0_DCDC_VBAT_DIV_CTRL_MASK        (0xC00U)
#define DCDC_REG0_DCDC_VBAT_DIV_CTRL_SHIFT       (10U)
#define DCDC_REG0_DCDC_VBAT_DIV_CTRL(x)          (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_VBAT_DIV_CTRL_SHIFT)) & DCDC_REG0_DCDC_VBAT_DIV_CTRL_MASK)
#define DCDC_REG0_DCDC_LP_STATE_HYS_L_MASK       (0x60000U)
#define DCDC_REG0_DCDC_LP_STATE_HYS_L_SHIFT      (17U)
#define DCDC_REG0_DCDC_LP_STATE_HYS_L(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_LP_STATE_HYS_L_SHIFT)) & DCDC_REG0_DCDC_LP_STATE_HYS_L_MASK)
#define DCDC_REG0_DCDC_LP_STATE_HYS_H_MASK       (0x180000U)
#define DCDC_REG0_DCDC_LP_STATE_HYS_H_SHIFT      (19U)
#define DCDC_REG0_DCDC_LP_STATE_HYS_H(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_LP_STATE_HYS_H_SHIFT)) & DCDC_REG0_DCDC_LP_STATE_HYS_H_MASK)
#define DCDC_REG0_HYST_LP_COMP_ADJ_MASK          (0x200000U)
#define DCDC_REG0_HYST_LP_COMP_ADJ_SHIFT         (21U)
#define DCDC_REG0_HYST_LP_COMP_ADJ(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_HYST_LP_COMP_ADJ_SHIFT)) & DCDC_REG0_HYST_LP_COMP_ADJ_MASK)
#define DCDC_REG0_HYST_LP_CMP_DISABLE_MASK       (0x400000U)
#define DCDC_REG0_HYST_LP_CMP_DISABLE_SHIFT      (22U)
#define DCDC_REG0_HYST_LP_CMP_DISABLE(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_HYST_LP_CMP_DISABLE_SHIFT)) & DCDC_REG0_HYST_LP_CMP_DISABLE_MASK)
#define DCDC_REG0_OFFSET_RSNS_LP_ADJ_MASK        (0x800000U)
#define DCDC_REG0_OFFSET_RSNS_LP_ADJ_SHIFT       (23U)
#define DCDC_REG0_OFFSET_RSNS_LP_ADJ(x)          (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_OFFSET_RSNS_LP_ADJ_SHIFT)) & DCDC_REG0_OFFSET_RSNS_LP_ADJ_MASK)
#define DCDC_REG0_OFFSET_RSNS_LP_DISABLE_MASK    (0x1000000U)
#define DCDC_REG0_OFFSET_RSNS_LP_DISABLE_SHIFT   (24U)
#define DCDC_REG0_OFFSET_RSNS_LP_DISABLE(x)      (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_OFFSET_RSNS_LP_DISABLE_SHIFT)) & DCDC_REG0_OFFSET_RSNS_LP_DISABLE_MASK)
#define DCDC_REG0_DCDC_LESS_I_MASK               (0x2000000U)
#define DCDC_REG0_DCDC_LESS_I_SHIFT              (25U)
#define DCDC_REG0_DCDC_LESS_I(x)                 (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_LESS_I_SHIFT)) & DCDC_REG0_DCDC_LESS_I_MASK)
#define DCDC_REG0_PWD_CMP_OFFSET_MASK            (0x4000000U)
#define DCDC_REG0_PWD_CMP_OFFSET_SHIFT           (26U)
#define DCDC_REG0_PWD_CMP_OFFSET(x)              (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_PWD_CMP_OFFSET_SHIFT)) & DCDC_REG0_PWD_CMP_OFFSET_MASK)
#define DCDC_REG0_DCDC_XTALOK_DISABLE_MASK       (0x8000000U)
#define DCDC_REG0_DCDC_XTALOK_DISABLE_SHIFT      (27U)
#define DCDC_REG0_DCDC_XTALOK_DISABLE(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_XTALOK_DISABLE_SHIFT)) & DCDC_REG0_DCDC_XTALOK_DISABLE_MASK)
#define DCDC_REG0_PSWITCH_STATUS_MASK            (0x10000000U)
#define DCDC_REG0_PSWITCH_STATUS_SHIFT           (28U)
#define DCDC_REG0_PSWITCH_STATUS(x)              (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_PSWITCH_STATUS_SHIFT)) & DCDC_REG0_PSWITCH_STATUS_MASK)
#define DCDC_REG0_VLPS_CONFIG_DCDC_HP_MASK       (0x20000000U)
#define DCDC_REG0_VLPS_CONFIG_DCDC_HP_SHIFT      (29U)
#define DCDC_REG0_VLPS_CONFIG_DCDC_HP(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_VLPS_CONFIG_DCDC_HP_SHIFT)) & DCDC_REG0_VLPS_CONFIG_DCDC_HP_MASK)
#define DCDC_REG0_VLPR_VLPW_CONFIG_DCDC_HP_MASK  (0x40000000U)
#define DCDC_REG0_VLPR_VLPW_CONFIG_DCDC_HP_SHIFT (30U)
#define DCDC_REG0_VLPR_VLPW_CONFIG_DCDC_HP(x)    (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_VLPR_VLPW_CONFIG_DCDC_HP_SHIFT)) & DCDC_REG0_VLPR_VLPW_CONFIG_DCDC_HP_MASK)
#define DCDC_REG0_DCDC_STS_DC_OK_MASK            (0x80000000U)
#define DCDC_REG0_DCDC_STS_DC_OK_SHIFT           (31U)
#define DCDC_REG0_DCDC_STS_DC_OK(x)              (((uint32_t)(((uint32_t)(x)) << DCDC_REG0_DCDC_STS_DC_OK_SHIFT)) & DCDC_REG0_DCDC_STS_DC_OK_MASK)

/*! @name REG1 - DCDC REGISTER 1 */
#define DCDC_REG1_POSLIMIT_BUCK_IN_MASK          (0x7FU)
#define DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT         (0U)
#define DCDC_REG1_POSLIMIT_BUCK_IN(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT)) & DCDC_REG1_POSLIMIT_BUCK_IN_MASK)
#define DCDC_REG1_POSLIMIT_BOOST_IN_MASK         (0x3F80U)
#define DCDC_REG1_POSLIMIT_BOOST_IN_SHIFT        (7U)
#define DCDC_REG1_POSLIMIT_BOOST_IN(x)           (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_POSLIMIT_BOOST_IN_SHIFT)) & DCDC_REG1_POSLIMIT_BOOST_IN_MASK)
#define DCDC_REG1_DCDC_LOOPCTRL_CM_HST_THRESH_MASK (0x200000U)
#define DCDC_REG1_DCDC_LOOPCTRL_CM_HST_THRESH_SHIFT (21U)
#define DCDC_REG1_DCDC_LOOPCTRL_CM_HST_THRESH(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_DCDC_LOOPCTRL_CM_HST_THRESH_SHIFT)) & DCDC_REG1_DCDC_LOOPCTRL_CM_HST_THRESH_MASK)
#define DCDC_REG1_DCDC_LOOPCTRL_DF_HST_THRESH_MASK (0x400000U)
#define DCDC_REG1_DCDC_LOOPCTRL_DF_HST_THRESH_SHIFT (22U)
#define DCDC_REG1_DCDC_LOOPCTRL_DF_HST_THRESH(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_DCDC_LOOPCTRL_DF_HST_THRESH_SHIFT)) & DCDC_REG1_DCDC_LOOPCTRL_DF_HST_THRESH_MASK)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST_MASK  (0x800000U)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST_SHIFT (23U)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST(x)    (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST_SHIFT)) & DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST_MASK)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST_MASK  (0x1000000U)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST_SHIFT (24U)
#define DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST(x)    (((uint32_t)(((uint32_t)(x)) << DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST_SHIFT)) & DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST_MASK)

/*! @name REG2 - DCDC REGISTER 2 */
#define DCDC_REG2_DCDC_LOOPCTRL_DC_C_MASK        (0x3U)
#define DCDC_REG2_DCDC_LOOPCTRL_DC_C_SHIFT       (0U)
#define DCDC_REG2_DCDC_LOOPCTRL_DC_C(x)          (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_LOOPCTRL_DC_C_SHIFT)) & DCDC_REG2_DCDC_LOOPCTRL_DC_C_MASK)
#define DCDC_REG2_DCDC_LOOPCTRL_DC_FF_MASK       (0x1C0U)
#define DCDC_REG2_DCDC_LOOPCTRL_DC_FF_SHIFT      (6U)
#define DCDC_REG2_DCDC_LOOPCTRL_DC_FF(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_LOOPCTRL_DC_FF_SHIFT)) & DCDC_REG2_DCDC_LOOPCTRL_DC_FF_MASK)
#define DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN_MASK   (0x2000U)
#define DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN_SHIFT  (13U)
#define DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN(x)     (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN_SHIFT)) & DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN_MASK)
#define DCDC_REG2_DCDC_LOOPCTRL_TOGGLE_DIF_MASK  (0x4000U)
#define DCDC_REG2_DCDC_LOOPCTRL_TOGGLE_DIF_SHIFT (14U)
#define DCDC_REG2_DCDC_LOOPCTRL_TOGGLE_DIF(x)    (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_LOOPCTRL_TOGGLE_DIF_SHIFT)) & DCDC_REG2_DCDC_LOOPCTRL_TOGGLE_DIF_MASK)
#define DCDC_REG2_DCDC_BATTMONITOR_EN_BATADJ_MASK (0x8000U)
#define DCDC_REG2_DCDC_BATTMONITOR_EN_BATADJ_SHIFT (15U)
#define DCDC_REG2_DCDC_BATTMONITOR_EN_BATADJ(x)  (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_BATTMONITOR_EN_BATADJ_SHIFT)) & DCDC_REG2_DCDC_BATTMONITOR_EN_BATADJ_MASK)
#define DCDC_REG2_DCDC_BATTMONITOR_BATT_VAL_MASK (0x3FF0000U)
#define DCDC_REG2_DCDC_BATTMONITOR_BATT_VAL_SHIFT (16U)
#define DCDC_REG2_DCDC_BATTMONITOR_BATT_VAL(x)   (((uint32_t)(((uint32_t)(x)) << DCDC_REG2_DCDC_BATTMONITOR_BATT_VAL_SHIFT)) & DCDC_REG2_DCDC_BATTMONITOR_BATT_VAL_MASK)

/*! @name REG3 - DCDC REGISTER 3 */
#define DCDC_REG3_DCDC_VDD1P8CTRL_TRG_MASK       (0x3FU)
#define DCDC_REG3_DCDC_VDD1P8CTRL_TRG_SHIFT      (0U)
#define DCDC_REG3_DCDC_VDD1P8CTRL_TRG(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P8CTRL_TRG_SHIFT)) & DCDC_REG3_DCDC_VDD1P8CTRL_TRG_MASK)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BUCK_MASK (0x7C0U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BUCK_SHIFT (6U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BUCK(x)   (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BUCK_SHIFT)) & DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BUCK_MASK)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BOOST_MASK (0xF800U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BOOST_SHIFT (11U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BOOST(x)  (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BOOST_SHIFT)) & DCDC_REG3_DCDC_VDD1P45CTRL_TRG_BOOST_MASK)
#define DCDC_REG3_DCDC_VDD1P45CTRL_ADJTN_MASK    (0x1E0000U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_ADJTN_SHIFT   (17U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_ADJTN(x)      (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P45CTRL_ADJTN_SHIFT)) & DCDC_REG3_DCDC_VDD1P45CTRL_ADJTN_MASK)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_LIMP_MASK (0x200000U)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_LIMP_SHIFT (21U)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_LIMP(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_LIMP_SHIFT)) & DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_LIMP_MASK)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_LIMP_MASK (0x400000U)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_LIMP_SHIFT (22U)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_LIMP(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_LIMP_SHIFT)) & DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_LIMP_MASK)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS_LIMP_MASK (0x800000U)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS_LIMP_SHIFT (23U)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS_LIMP(x)  (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_HALF_FETS_LIMP_SHIFT)) & DCDC_REG3_DCDC_MINPWR_HALF_FETS_LIMP_MASK)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_MASK    (0x1000000U)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_SHIFT   (24U)
#define DCDC_REG3_DCDC_MINPWR_DC_HALFCLK(x)      (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_SHIFT)) & DCDC_REG3_DCDC_MINPWR_DC_HALFCLK_MASK)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_MASK   (0x2000000U)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_SHIFT  (25U)
#define DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS(x)     (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_SHIFT)) & DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_MASK)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS_MASK     (0x4000000U)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS_SHIFT    (26U)
#define DCDC_REG3_DCDC_MINPWR_HALF_FETS(x)       (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_MINPWR_HALF_FETS_SHIFT)) & DCDC_REG3_DCDC_MINPWR_HALF_FETS_MASK)
#define DCDC_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP_MASK (0x20000000U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP_SHIFT (29U)
#define DCDC_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP_SHIFT)) & DCDC_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP_MASK)
#define DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP_MASK (0x40000000U)
#define DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP_SHIFT (30U)
#define DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP(x) (((uint32_t)(((uint32_t)(x)) << DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP_SHIFT)) & DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP_MASK)

/*! @name REG4 - DCDC REGISTER 4 */
#define DCDC_REG4_DCDC_SW_SHUTDOWN_MASK          (0x1U)
#define DCDC_REG4_DCDC_SW_SHUTDOWN_SHIFT         (0U)
#define DCDC_REG4_DCDC_SW_SHUTDOWN(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG4_DCDC_SW_SHUTDOWN_SHIFT)) & DCDC_REG4_DCDC_SW_SHUTDOWN_MASK)
#define DCDC_REG4_UNLOCK_MASK                    (0xFFFF0000U)
#define DCDC_REG4_UNLOCK_SHIFT                   (16U)
#define DCDC_REG4_UNLOCK(x)                      (((uint32_t)(((uint32_t)(x)) << DCDC_REG4_UNLOCK_SHIFT)) & DCDC_REG4_UNLOCK_MASK)

/*! @name REG6 - DCDC REGISTER 6 */
#define DCDC_REG6_PSWITCH_INT_RISE_EN_MASK       (0x1U)
#define DCDC_REG6_PSWITCH_INT_RISE_EN_SHIFT      (0U)
#define DCDC_REG6_PSWITCH_INT_RISE_EN(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG6_PSWITCH_INT_RISE_EN_SHIFT)) & DCDC_REG6_PSWITCH_INT_RISE_EN_MASK)
#define DCDC_REG6_PSWITCH_INT_FALL_EN_MASK       (0x2U)
#define DCDC_REG6_PSWITCH_INT_FALL_EN_SHIFT      (1U)
#define DCDC_REG6_PSWITCH_INT_FALL_EN(x)         (((uint32_t)(((uint32_t)(x)) << DCDC_REG6_PSWITCH_INT_FALL_EN_SHIFT)) & DCDC_REG6_PSWITCH_INT_FALL_EN_MASK)
#define DCDC_REG6_PSWITCH_INT_CLEAR_MASK         (0x4U)
#define DCDC_REG6_PSWITCH_INT_CLEAR_SHIFT        (2U)
#define DCDC_REG6_PSWITCH_INT_CLEAR(x)           (((uint32_t)(((uint32_t)(x)) << DCDC_REG6_PSWITCH_INT_CLEAR_SHIFT)) & DCDC_REG6_PSWITCH_INT_CLEAR_MASK)
#define DCDC_REG6_PSWITCH_INT_MUTE_MASK          (0x8U)
#define DCDC_REG6_PSWITCH_INT_MUTE_SHIFT         (3U)
#define DCDC_REG6_PSWITCH_INT_MUTE(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG6_PSWITCH_INT_MUTE_SHIFT)) & DCDC_REG6_PSWITCH_INT_MUTE_MASK)
#define DCDC_REG6_PSWITCH_INT_STS_MASK           (0x80000000U)
#define DCDC_REG6_PSWITCH_INT_STS_SHIFT          (31U)
#define DCDC_REG6_PSWITCH_INT_STS(x)             (((uint32_t)(((uint32_t)(x)) << DCDC_REG6_PSWITCH_INT_STS_SHIFT)) & DCDC_REG6_PSWITCH_INT_STS_MASK)

/*! @name REG7 - DCDC REGISTER 7 */
#define DCDC_REG7_INTEGRATOR_VALUE_MASK          (0x7FFFFU)
#define DCDC_REG7_INTEGRATOR_VALUE_SHIFT         (0U)
#define DCDC_REG7_INTEGRATOR_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << DCDC_REG7_INTEGRATOR_VALUE_SHIFT)) & DCDC_REG7_INTEGRATOR_VALUE_MASK)
#define DCDC_REG7_INTEGRATOR_VALUE_SEL_MASK      (0x80000U)
#define DCDC_REG7_INTEGRATOR_VALUE_SEL_SHIFT     (19U)
#define DCDC_REG7_INTEGRATOR_VALUE_SEL(x)        (((uint32_t)(((uint32_t)(x)) << DCDC_REG7_INTEGRATOR_VALUE_SEL_SHIFT)) & DCDC_REG7_INTEGRATOR_VALUE_SEL_MASK)
#define DCDC_REG7_PULSE_RUN_SPEEDUP_MASK         (0x100000U)
#define DCDC_REG7_PULSE_RUN_SPEEDUP_SHIFT        (20U)
#define DCDC_REG7_PULSE_RUN_SPEEDUP(x)           (((uint32_t)(((uint32_t)(x)) << DCDC_REG7_PULSE_RUN_SPEEDUP_SHIFT)) & DCDC_REG7_PULSE_RUN_SPEEDUP_MASK)


/*!
 * @}
 */ /* end of group DCDC_Register_Masks */


/* DCDC - Peripheral instance base addresses */
/** Peripheral DCDC base address */
#define DCDC_BASE                                (0x4005A000u)
/** Peripheral DCDC base pointer */
#define DCDC                                     ((DCDC_Type *)DCDC_BASE)
/** Array initializer of DCDC peripheral base addresses */
#define DCDC_BASE_ADDRS                          { DCDC_BASE }
/** Array initializer of DCDC peripheral base pointers */
#define DCDC_BASE_PTRS                           { DCDC }

/*!
 * @}
 */ /* end of group DCDC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[256];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __IO uint32_t SAR;                               /**< Source Address Register, array offset: 0x100, array step: 0x10 */
    __IO uint32_t DAR;                               /**< Destination Address Register, array offset: 0x104, array step: 0x10 */
    union {                                          /* offset: 0x108, array step: 0x10 */
      __IO uint32_t DSR_BCR;                           /**< DMA Status Register / Byte Count Register, array offset: 0x108, array step: 0x10 */
      struct {                                         /* offset: 0x108, array step: 0x10 */
             uint8_t RESERVED_0[3];
        __IO uint8_t DSR;                                /**< DMA_DSR0 register...DMA_DSR3 register., array offset: 0x10B, array step: 0x10 */
      } DMA_DSR_ACCESS8BIT;
    };
    __IO uint32_t DCR;                               /**< DMA Control Register, array offset: 0x10C, array step: 0x10 */
  } DMA[4];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name SAR - Source Address Register */
#define DMA_SAR_SAR_MASK                         (0xFFFFFFFFU)
#define DMA_SAR_SAR_SHIFT                        (0U)
#define DMA_SAR_SAR(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_SAR_SAR_SHIFT)) & DMA_SAR_SAR_MASK)

/* The count of DMA_SAR */
#define DMA_SAR_COUNT                            (4U)

/*! @name DAR - Destination Address Register */
#define DMA_DAR_DAR_MASK                         (0xFFFFFFFFU)
#define DMA_DAR_DAR_SHIFT                        (0U)
#define DMA_DAR_DAR(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_DAR_DAR_SHIFT)) & DMA_DAR_DAR_MASK)

/* The count of DMA_DAR */
#define DMA_DAR_COUNT                            (4U)

/*! @name DSR_BCR - DMA Status Register / Byte Count Register */
#define DMA_DSR_BCR_BCR_MASK                     (0xFFFFFFU)
#define DMA_DSR_BCR_BCR_SHIFT                    (0U)
#define DMA_DSR_BCR_BCR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_BCR_SHIFT)) & DMA_DSR_BCR_BCR_MASK)
#define DMA_DSR_BCR_DONE_MASK                    (0x1000000U)
#define DMA_DSR_BCR_DONE_SHIFT                   (24U)
#define DMA_DSR_BCR_DONE(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_DONE_SHIFT)) & DMA_DSR_BCR_DONE_MASK)
#define DMA_DSR_BCR_BSY_MASK                     (0x2000000U)
#define DMA_DSR_BCR_BSY_SHIFT                    (25U)
#define DMA_DSR_BCR_BSY(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_BSY_SHIFT)) & DMA_DSR_BCR_BSY_MASK)
#define DMA_DSR_BCR_REQ_MASK                     (0x4000000U)
#define DMA_DSR_BCR_REQ_SHIFT                    (26U)
#define DMA_DSR_BCR_REQ(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_REQ_SHIFT)) & DMA_DSR_BCR_REQ_MASK)
#define DMA_DSR_BCR_BED_MASK                     (0x10000000U)
#define DMA_DSR_BCR_BED_SHIFT                    (28U)
#define DMA_DSR_BCR_BED(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_BED_SHIFT)) & DMA_DSR_BCR_BED_MASK)
#define DMA_DSR_BCR_BES_MASK                     (0x20000000U)
#define DMA_DSR_BCR_BES_SHIFT                    (29U)
#define DMA_DSR_BCR_BES(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_BES_SHIFT)) & DMA_DSR_BCR_BES_MASK)
#define DMA_DSR_BCR_CE_MASK                      (0x40000000U)
#define DMA_DSR_BCR_CE_SHIFT                     (30U)
#define DMA_DSR_BCR_CE(x)                        (((uint32_t)(((uint32_t)(x)) << DMA_DSR_BCR_CE_SHIFT)) & DMA_DSR_BCR_CE_MASK)

/* The count of DMA_DSR_BCR */
#define DMA_DSR_BCR_COUNT                        (4U)

/* The count of DMA_DSR */
#define DMA_DSR_COUNT                            (4U)

/*! @name DCR - DMA Control Register */
#define DMA_DCR_LCH2_MASK                        (0x3U)
#define DMA_DCR_LCH2_SHIFT                       (0U)
#define DMA_DCR_LCH2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_LCH2_SHIFT)) & DMA_DCR_LCH2_MASK)
#define DMA_DCR_LCH1_MASK                        (0xCU)
#define DMA_DCR_LCH1_SHIFT                       (2U)
#define DMA_DCR_LCH1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_LCH1_SHIFT)) & DMA_DCR_LCH1_MASK)
#define DMA_DCR_LINKCC_MASK                      (0x30U)
#define DMA_DCR_LINKCC_SHIFT                     (4U)
#define DMA_DCR_LINKCC(x)                        (((uint32_t)(((uint32_t)(x)) << DMA_DCR_LINKCC_SHIFT)) & DMA_DCR_LINKCC_MASK)
#define DMA_DCR_D_REQ_MASK                       (0x80U)
#define DMA_DCR_D_REQ_SHIFT                      (7U)
#define DMA_DCR_D_REQ(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_DCR_D_REQ_SHIFT)) & DMA_DCR_D_REQ_MASK)
#define DMA_DCR_DMOD_MASK                        (0xF00U)
#define DMA_DCR_DMOD_SHIFT                       (8U)
#define DMA_DCR_DMOD(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_DMOD_SHIFT)) & DMA_DCR_DMOD_MASK)
#define DMA_DCR_SMOD_MASK                        (0xF000U)
#define DMA_DCR_SMOD_SHIFT                       (12U)
#define DMA_DCR_SMOD(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_SMOD_SHIFT)) & DMA_DCR_SMOD_MASK)
#define DMA_DCR_START_MASK                       (0x10000U)
#define DMA_DCR_START_SHIFT                      (16U)
#define DMA_DCR_START(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_DCR_START_SHIFT)) & DMA_DCR_START_MASK)
#define DMA_DCR_DSIZE_MASK                       (0x60000U)
#define DMA_DCR_DSIZE_SHIFT                      (17U)
#define DMA_DCR_DSIZE(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_DCR_DSIZE_SHIFT)) & DMA_DCR_DSIZE_MASK)
#define DMA_DCR_DINC_MASK                        (0x80000U)
#define DMA_DCR_DINC_SHIFT                       (19U)
#define DMA_DCR_DINC(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_DINC_SHIFT)) & DMA_DCR_DINC_MASK)
#define DMA_DCR_SSIZE_MASK                       (0x300000U)
#define DMA_DCR_SSIZE_SHIFT                      (20U)
#define DMA_DCR_SSIZE(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_DCR_SSIZE_SHIFT)) & DMA_DCR_SSIZE_MASK)
#define DMA_DCR_SINC_MASK                        (0x400000U)
#define DMA_DCR_SINC_SHIFT                       (22U)
#define DMA_DCR_SINC(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_SINC_SHIFT)) & DMA_DCR_SINC_MASK)
#define DMA_DCR_EADREQ_MASK                      (0x800000U)
#define DMA_DCR_EADREQ_SHIFT                     (23U)
#define DMA_DCR_EADREQ(x)                        (((uint32_t)(((uint32_t)(x)) << DMA_DCR_EADREQ_SHIFT)) & DMA_DCR_EADREQ_MASK)
#define DMA_DCR_AA_MASK                          (0x10000000U)
#define DMA_DCR_AA_SHIFT                         (28U)
#define DMA_DCR_AA(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_DCR_AA_SHIFT)) & DMA_DCR_AA_MASK)
#define DMA_DCR_CS_MASK                          (0x20000000U)
#define DMA_DCR_CS_SHIFT                         (29U)
#define DMA_DCR_CS(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_DCR_CS_SHIFT)) & DMA_DCR_CS_MASK)
#define DMA_DCR_ERQ_MASK                         (0x40000000U)
#define DMA_DCR_ERQ_SHIFT                        (30U)
#define DMA_DCR_ERQ(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_DCR_ERQ_SHIFT)) & DMA_DCR_ERQ_MASK)
#define DMA_DCR_EINT_MASK                        (0x80000000U)
#define DMA_DCR_EINT_SHIFT                       (31U)
#define DMA_DCR_EINT(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_DCR_EINT_SHIFT)) & DMA_DCR_EINT_MASK)

/* The count of DMA_DCR */
#define DMA_DCR_COUNT                            (4U)


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
#define DMA_CHN_IRQS                             { DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn }

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
  __IO uint8_t CHCFG[4];                           /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
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
#define DMAMUX_CHCFG_COUNT                       (4U)


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
/** Peripheral FGPIOB base address */
#define FGPIOB_BASE                              (0xF8000040u)
/** Peripheral FGPIOB base pointer */
#define FGPIOB                                   ((FGPIO_Type *)FGPIOB_BASE)
/** Peripheral FGPIOC base address */
#define FGPIOC_BASE                              (0xF8000080u)
/** Peripheral FGPIOC base pointer */
#define FGPIOC                                   ((FGPIO_Type *)FGPIOC_BASE)
/** Array initializer of FGPIO peripheral base addresses */
#define FGPIO_BASE_ADDRS                         { FGPIOA_BASE, FGPIOB_BASE, FGPIOC_BASE }
/** Array initializer of FGPIO peripheral base pointers */
#define FGPIO_BASE_PTRS                          { FGPIOA, FGPIOB, FGPIOC }

/*!
 * @}
 */ /* end of group FGPIO_Peripheral_Access_Layer */


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
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { GPIOA_BASE, GPIOB_BASE, GPIOC_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIOA, GPIOB, GPIOC }

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
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE, I2C1_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0, I2C1 }
/** Interrupt vectors for the I2C peripheral type */
#define I2C_IRQS                                 { I2C0_IRQn, I2C1_IRQn }

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


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
  __IO uint8_t ME;                                 /**< LLWU Module Enable register, offset: 0x4 */
  __IO uint8_t F1;                                 /**< LLWU Flag 1 register, offset: 0x5 */
  __IO uint8_t F2;                                 /**< LLWU Flag 2 register, offset: 0x6 */
  __I  uint8_t F3;                                 /**< LLWU Flag 3 register, offset: 0x7 */
  __IO uint8_t FILT1;                              /**< LLWU Pin Filter 1 register, offset: 0x8 */
  __IO uint8_t FILT2;                              /**< LLWU Pin Filter 2 register, offset: 0x9 */
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

/*! @name F1 - LLWU Flag 1 register */
#define LLWU_F1_WUF0_MASK                        (0x1U)
#define LLWU_F1_WUF0_SHIFT                       (0U)
#define LLWU_F1_WUF0(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF0_SHIFT)) & LLWU_F1_WUF0_MASK)
#define LLWU_F1_WUF1_MASK                        (0x2U)
#define LLWU_F1_WUF1_SHIFT                       (1U)
#define LLWU_F1_WUF1(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF1_SHIFT)) & LLWU_F1_WUF1_MASK)
#define LLWU_F1_WUF2_MASK                        (0x4U)
#define LLWU_F1_WUF2_SHIFT                       (2U)
#define LLWU_F1_WUF2(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF2_SHIFT)) & LLWU_F1_WUF2_MASK)
#define LLWU_F1_WUF3_MASK                        (0x8U)
#define LLWU_F1_WUF3_SHIFT                       (3U)
#define LLWU_F1_WUF3(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF3_SHIFT)) & LLWU_F1_WUF3_MASK)
#define LLWU_F1_WUF4_MASK                        (0x10U)
#define LLWU_F1_WUF4_SHIFT                       (4U)
#define LLWU_F1_WUF4(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF4_SHIFT)) & LLWU_F1_WUF4_MASK)
#define LLWU_F1_WUF5_MASK                        (0x20U)
#define LLWU_F1_WUF5_SHIFT                       (5U)
#define LLWU_F1_WUF5(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF5_SHIFT)) & LLWU_F1_WUF5_MASK)
#define LLWU_F1_WUF6_MASK                        (0x40U)
#define LLWU_F1_WUF6_SHIFT                       (6U)
#define LLWU_F1_WUF6(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF6_SHIFT)) & LLWU_F1_WUF6_MASK)
#define LLWU_F1_WUF7_MASK                        (0x80U)
#define LLWU_F1_WUF7_SHIFT                       (7U)
#define LLWU_F1_WUF7(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F1_WUF7_SHIFT)) & LLWU_F1_WUF7_MASK)

/*! @name F2 - LLWU Flag 2 register */
#define LLWU_F2_WUF8_MASK                        (0x1U)
#define LLWU_F2_WUF8_SHIFT                       (0U)
#define LLWU_F2_WUF8(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF8_SHIFT)) & LLWU_F2_WUF8_MASK)
#define LLWU_F2_WUF9_MASK                        (0x2U)
#define LLWU_F2_WUF9_SHIFT                       (1U)
#define LLWU_F2_WUF9(x)                          (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF9_SHIFT)) & LLWU_F2_WUF9_MASK)
#define LLWU_F2_WUF10_MASK                       (0x4U)
#define LLWU_F2_WUF10_SHIFT                      (2U)
#define LLWU_F2_WUF10(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF10_SHIFT)) & LLWU_F2_WUF10_MASK)
#define LLWU_F2_WUF11_MASK                       (0x8U)
#define LLWU_F2_WUF11_SHIFT                      (3U)
#define LLWU_F2_WUF11(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF11_SHIFT)) & LLWU_F2_WUF11_MASK)
#define LLWU_F2_WUF12_MASK                       (0x10U)
#define LLWU_F2_WUF12_SHIFT                      (4U)
#define LLWU_F2_WUF12(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF12_SHIFT)) & LLWU_F2_WUF12_MASK)
#define LLWU_F2_WUF13_MASK                       (0x20U)
#define LLWU_F2_WUF13_SHIFT                      (5U)
#define LLWU_F2_WUF13(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF13_SHIFT)) & LLWU_F2_WUF13_MASK)
#define LLWU_F2_WUF14_MASK                       (0x40U)
#define LLWU_F2_WUF14_SHIFT                      (6U)
#define LLWU_F2_WUF14(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF14_SHIFT)) & LLWU_F2_WUF14_MASK)
#define LLWU_F2_WUF15_MASK                       (0x80U)
#define LLWU_F2_WUF15_SHIFT                      (7U)
#define LLWU_F2_WUF15(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F2_WUF15_SHIFT)) & LLWU_F2_WUF15_MASK)

/*! @name F3 - LLWU Flag 3 register */
#define LLWU_F3_MWUF0_MASK                       (0x1U)
#define LLWU_F3_MWUF0_SHIFT                      (0U)
#define LLWU_F3_MWUF0(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF0_SHIFT)) & LLWU_F3_MWUF0_MASK)
#define LLWU_F3_MWUF1_MASK                       (0x2U)
#define LLWU_F3_MWUF1_SHIFT                      (1U)
#define LLWU_F3_MWUF1(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF1_SHIFT)) & LLWU_F3_MWUF1_MASK)
#define LLWU_F3_MWUF2_MASK                       (0x4U)
#define LLWU_F3_MWUF2_SHIFT                      (2U)
#define LLWU_F3_MWUF2(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF2_SHIFT)) & LLWU_F3_MWUF2_MASK)
#define LLWU_F3_MWUF3_MASK                       (0x8U)
#define LLWU_F3_MWUF3_SHIFT                      (3U)
#define LLWU_F3_MWUF3(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF3_SHIFT)) & LLWU_F3_MWUF3_MASK)
#define LLWU_F3_MWUF4_MASK                       (0x10U)
#define LLWU_F3_MWUF4_SHIFT                      (4U)
#define LLWU_F3_MWUF4(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF4_SHIFT)) & LLWU_F3_MWUF4_MASK)
#define LLWU_F3_MWUF5_MASK                       (0x20U)
#define LLWU_F3_MWUF5_SHIFT                      (5U)
#define LLWU_F3_MWUF5(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF5_SHIFT)) & LLWU_F3_MWUF5_MASK)
#define LLWU_F3_MWUF6_MASK                       (0x40U)
#define LLWU_F3_MWUF6_SHIFT                      (6U)
#define LLWU_F3_MWUF6(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF6_SHIFT)) & LLWU_F3_MWUF6_MASK)
#define LLWU_F3_MWUF7_MASK                       (0x80U)
#define LLWU_F3_MWUF7_SHIFT                      (7U)
#define LLWU_F3_MWUF7(x)                         (((uint8_t)(((uint8_t)(x)) << LLWU_F3_MWUF7_SHIFT)) & LLWU_F3_MWUF7_MASK)

/*! @name FILT1 - LLWU Pin Filter 1 register */
#define LLWU_FILT1_FILTSEL_MASK                  (0xFU)
#define LLWU_FILT1_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT1_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTSEL_SHIFT)) & LLWU_FILT1_FILTSEL_MASK)
#define LLWU_FILT1_FILTE_MASK                    (0x60U)
#define LLWU_FILT1_FILTE_SHIFT                   (5U)
#define LLWU_FILT1_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTE_SHIFT)) & LLWU_FILT1_FILTE_MASK)
#define LLWU_FILT1_FILTF_MASK                    (0x80U)
#define LLWU_FILT1_FILTF_SHIFT                   (7U)
#define LLWU_FILT1_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT1_FILTF_SHIFT)) & LLWU_FILT1_FILTF_MASK)

/*! @name FILT2 - LLWU Pin Filter 2 register */
#define LLWU_FILT2_FILTSEL_MASK                  (0xFU)
#define LLWU_FILT2_FILTSEL_SHIFT                 (0U)
#define LLWU_FILT2_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTSEL_SHIFT)) & LLWU_FILT2_FILTSEL_MASK)
#define LLWU_FILT2_FILTE_MASK                    (0x60U)
#define LLWU_FILT2_FILTE_SHIFT                   (5U)
#define LLWU_FILT2_FILTE(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTE_SHIFT)) & LLWU_FILT2_FILTE_MASK)
#define LLWU_FILT2_FILTF_MASK                    (0x80U)
#define LLWU_FILT2_FILTF_SHIFT                   (7U)
#define LLWU_FILT2_FILTF(x)                      (((uint8_t)(((uint8_t)(x)) << LLWU_FILT2_FILTF_SHIFT)) & LLWU_FILT2_FILTF_MASK)


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
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0 }
/** Interrupt vectors for the LPTMR peripheral type */
#define LPTMR_IRQS                               { LPTMR0_IRQn }

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
#define LPUART_MODIR_TNP_MASK                    (0x30000U)
#define LPUART_MODIR_TNP_SHIFT                   (16U)
#define LPUART_MODIR_TNP(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TNP_SHIFT)) & LPUART_MODIR_TNP_MASK)
#define LPUART_MODIR_IREN_MASK                   (0x40000U)
#define LPUART_MODIR_IREN_SHIFT                  (18U)
#define LPUART_MODIR_IREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_IREN_SHIFT)) & LPUART_MODIR_IREN_MASK)


/*!
 * @}
 */ /* end of group LPUART_Register_Masks */


/* LPUART - Peripheral instance base addresses */
/** Peripheral LPUART0 base address */
#define LPUART0_BASE                             (0x40054000u)
/** Peripheral LPUART0 base pointer */
#define LPUART0                                  ((LPUART_Type *)LPUART0_BASE)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                        { LPUART0_BASE }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                         { LPUART0 }
/** Interrupt vectors for the LPUART peripheral type */
#define LPUART_RX_TX_IRQS                        { LPUART0_IRQn }
#define LPUART_ERR_IRQS                          { LPUART0_IRQn }

/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LTC_Peripheral_Access_Layer LTC Peripheral Access Layer
 * @{
 */

/** LTC - Register Layout Typedef */
typedef struct {
  union {                                          /* offset: 0x0 */
    __IO uint32_t MD;                                /**< LTC Mode Register, offset: 0x0 */
  };
       uint8_t RESERVED_0[4];
  __IO uint32_t KS;                                /**< LTC Key Size Register, offset: 0x8 */
       uint8_t RESERVED_1[4];
  __IO uint32_t DS;                                /**< LTC Data Size Register, offset: 0x10 */
       uint8_t RESERVED_2[4];
  __IO uint32_t ICVS;                              /**< LTC ICV Size Register, offset: 0x18 */
       uint8_t RESERVED_3[20];
  __IO uint32_t COM;                               /**< LTC Command Register, offset: 0x30 */
  __IO uint32_t CTL;                               /**< LTC Control Register, offset: 0x34 */
       uint8_t RESERVED_4[8];
  __IO uint32_t CW;                                /**< LTC Clear Written Register, offset: 0x40 */
       uint8_t RESERVED_5[4];
  __IO uint32_t STA;                               /**< LTC Status Register, offset: 0x48 */
  __I  uint32_t ESTA;                              /**< LTC Error Status Register, offset: 0x4C */
       uint8_t RESERVED_6[8];
  __IO uint32_t AADSZ;                             /**< LTC AAD Size Register, offset: 0x58 */
       uint8_t RESERVED_7[164];
  __IO uint32_t CTX[16];                           /**< LTC Context Register, array offset: 0x100, array step: 0x4 */
       uint8_t RESERVED_8[192];
  __IO uint32_t KEY[4];                            /**< LTC Key Registers, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_9[740];
  __I  uint32_t VID2;                              /**< LTC Version ID 2 Register, offset: 0x4F4 */
       uint8_t RESERVED_10[712];
  __I  uint32_t FIFOSTA;                           /**< LTC FIFO Status Register, offset: 0x7C0 */
       uint8_t RESERVED_11[28];
  __O  uint32_t IFIFO;                             /**< LTC Input Data FIFO, offset: 0x7E0 */
       uint8_t RESERVED_12[12];
  __I  uint32_t OFIFO;                             /**< LTC Output Data FIFO, offset: 0x7F0 */
       uint8_t RESERVED_13[252];
  __I  uint32_t VID1;                              /**< LTC Version ID Register, offset: 0x8F0 */
       uint8_t RESERVED_14[4];
  __I  uint32_t CHAVID;                            /**< LTC CHA Version ID Register, offset: 0x8F8 */
} LTC_Type;

/* ----------------------------------------------------------------------------
   -- LTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LTC_Register_Masks LTC Register Masks
 * @{
 */

/*! @name MD - LTC Mode Register */
#define LTC_MD_ENC_MASK                          (0x1U)
#define LTC_MD_ENC_SHIFT                         (0U)
#define LTC_MD_ENC(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_MD_ENC_SHIFT)) & LTC_MD_ENC_MASK)
#define LTC_MD_ICV_TEST_MASK                     (0x2U)
#define LTC_MD_ICV_TEST_SHIFT                    (1U)
#define LTC_MD_ICV_TEST(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_MD_ICV_TEST_SHIFT)) & LTC_MD_ICV_TEST_MASK)
#define LTC_MD_AS_MASK                           (0xCU)
#define LTC_MD_AS_SHIFT                          (2U)
#define LTC_MD_AS(x)                             (((uint32_t)(((uint32_t)(x)) << LTC_MD_AS_SHIFT)) & LTC_MD_AS_MASK)
#define LTC_MD_AAI_MASK                          (0x1FF0U)
#define LTC_MD_AAI_SHIFT                         (4U)
#define LTC_MD_AAI(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_MD_AAI_SHIFT)) & LTC_MD_AAI_MASK)
#define LTC_MD_ALG_MASK                          (0xFF0000U)
#define LTC_MD_ALG_SHIFT                         (16U)
#define LTC_MD_ALG(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_MD_ALG_SHIFT)) & LTC_MD_ALG_MASK)

/*! @name KS - LTC Key Size Register */
#define LTC_KS_KS_MASK                           (0x1FU)
#define LTC_KS_KS_SHIFT                          (0U)
#define LTC_KS_KS(x)                             (((uint32_t)(((uint32_t)(x)) << LTC_KS_KS_SHIFT)) & LTC_KS_KS_MASK)

/*! @name DS - LTC Data Size Register */
#define LTC_DS_DS_MASK                           (0xFFFU)
#define LTC_DS_DS_SHIFT                          (0U)
#define LTC_DS_DS(x)                             (((uint32_t)(((uint32_t)(x)) << LTC_DS_DS_SHIFT)) & LTC_DS_DS_MASK)

/*! @name ICVS - LTC ICV Size Register */
#define LTC_ICVS_ICVS_MASK                       (0x1FU)
#define LTC_ICVS_ICVS_SHIFT                      (0U)
#define LTC_ICVS_ICVS(x)                         (((uint32_t)(((uint32_t)(x)) << LTC_ICVS_ICVS_SHIFT)) & LTC_ICVS_ICVS_MASK)

/*! @name COM - LTC Command Register */
#define LTC_COM_ALL_MASK                         (0x1U)
#define LTC_COM_ALL_SHIFT                        (0U)
#define LTC_COM_ALL(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_COM_ALL_SHIFT)) & LTC_COM_ALL_MASK)
#define LTC_COM_AES_MASK                         (0x2U)
#define LTC_COM_AES_SHIFT                        (1U)
#define LTC_COM_AES(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_COM_AES_SHIFT)) & LTC_COM_AES_MASK)

/*! @name CTL - LTC Control Register */
#define LTC_CTL_IM_MASK                          (0x1U)
#define LTC_CTL_IM_SHIFT                         (0U)
#define LTC_CTL_IM(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CTL_IM_SHIFT)) & LTC_CTL_IM_MASK)
#define LTC_CTL_IFE_MASK                         (0x100U)
#define LTC_CTL_IFE_SHIFT                        (8U)
#define LTC_CTL_IFE(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_IFE_SHIFT)) & LTC_CTL_IFE_MASK)
#define LTC_CTL_IFR_MASK                         (0x200U)
#define LTC_CTL_IFR_SHIFT                        (9U)
#define LTC_CTL_IFR(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_IFR_SHIFT)) & LTC_CTL_IFR_MASK)
#define LTC_CTL_OFE_MASK                         (0x1000U)
#define LTC_CTL_OFE_SHIFT                        (12U)
#define LTC_CTL_OFE(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_OFE_SHIFT)) & LTC_CTL_OFE_MASK)
#define LTC_CTL_OFR_MASK                         (0x2000U)
#define LTC_CTL_OFR_SHIFT                        (13U)
#define LTC_CTL_OFR(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_OFR_SHIFT)) & LTC_CTL_OFR_MASK)
#define LTC_CTL_IFS_MASK                         (0x10000U)
#define LTC_CTL_IFS_SHIFT                        (16U)
#define LTC_CTL_IFS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_IFS_SHIFT)) & LTC_CTL_IFS_MASK)
#define LTC_CTL_OFS_MASK                         (0x20000U)
#define LTC_CTL_OFS_SHIFT                        (17U)
#define LTC_CTL_OFS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_OFS_SHIFT)) & LTC_CTL_OFS_MASK)
#define LTC_CTL_KIS_MASK                         (0x100000U)
#define LTC_CTL_KIS_SHIFT                        (20U)
#define LTC_CTL_KIS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_KIS_SHIFT)) & LTC_CTL_KIS_MASK)
#define LTC_CTL_KOS_MASK                         (0x200000U)
#define LTC_CTL_KOS_SHIFT                        (21U)
#define LTC_CTL_KOS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_KOS_SHIFT)) & LTC_CTL_KOS_MASK)
#define LTC_CTL_CIS_MASK                         (0x400000U)
#define LTC_CTL_CIS_SHIFT                        (22U)
#define LTC_CTL_CIS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_CIS_SHIFT)) & LTC_CTL_CIS_MASK)
#define LTC_CTL_COS_MASK                         (0x800000U)
#define LTC_CTL_COS_SHIFT                        (23U)
#define LTC_CTL_COS(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_COS_SHIFT)) & LTC_CTL_COS_MASK)
#define LTC_CTL_KAL_MASK                         (0x80000000U)
#define LTC_CTL_KAL_SHIFT                        (31U)
#define LTC_CTL_KAL(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTL_KAL_SHIFT)) & LTC_CTL_KAL_MASK)

/*! @name CW - LTC Clear Written Register */
#define LTC_CW_CM_MASK                           (0x1U)
#define LTC_CW_CM_SHIFT                          (0U)
#define LTC_CW_CM(x)                             (((uint32_t)(((uint32_t)(x)) << LTC_CW_CM_SHIFT)) & LTC_CW_CM_MASK)
#define LTC_CW_CDS_MASK                          (0x4U)
#define LTC_CW_CDS_SHIFT                         (2U)
#define LTC_CW_CDS(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CW_CDS_SHIFT)) & LTC_CW_CDS_MASK)
#define LTC_CW_CICV_MASK                         (0x8U)
#define LTC_CW_CICV_SHIFT                        (3U)
#define LTC_CW_CICV(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CW_CICV_SHIFT)) & LTC_CW_CICV_MASK)
#define LTC_CW_CCR_MASK                          (0x20U)
#define LTC_CW_CCR_SHIFT                         (5U)
#define LTC_CW_CCR(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CW_CCR_SHIFT)) & LTC_CW_CCR_MASK)
#define LTC_CW_CKR_MASK                          (0x40U)
#define LTC_CW_CKR_SHIFT                         (6U)
#define LTC_CW_CKR(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CW_CKR_SHIFT)) & LTC_CW_CKR_MASK)
#define LTC_CW_COF_MASK                          (0x40000000U)
#define LTC_CW_COF_SHIFT                         (30U)
#define LTC_CW_COF(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CW_COF_SHIFT)) & LTC_CW_COF_MASK)
#define LTC_CW_CIF_MASK                          (0x80000000U)
#define LTC_CW_CIF_SHIFT                         (31U)
#define LTC_CW_CIF(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_CW_CIF_SHIFT)) & LTC_CW_CIF_MASK)

/*! @name STA - LTC Status Register */
#define LTC_STA_AB_MASK                          (0x2U)
#define LTC_STA_AB_SHIFT                         (1U)
#define LTC_STA_AB(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_STA_AB_SHIFT)) & LTC_STA_AB_MASK)
#define LTC_STA_DI_MASK                          (0x10000U)
#define LTC_STA_DI_SHIFT                         (16U)
#define LTC_STA_DI(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_STA_DI_SHIFT)) & LTC_STA_DI_MASK)
#define LTC_STA_EI_MASK                          (0x100000U)
#define LTC_STA_EI_SHIFT                         (20U)
#define LTC_STA_EI(x)                            (((uint32_t)(((uint32_t)(x)) << LTC_STA_EI_SHIFT)) & LTC_STA_EI_MASK)

/*! @name ESTA - LTC Error Status Register */
#define LTC_ESTA_ERRID1_MASK                     (0xFU)
#define LTC_ESTA_ERRID1_SHIFT                    (0U)
#define LTC_ESTA_ERRID1(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_ESTA_ERRID1_SHIFT)) & LTC_ESTA_ERRID1_MASK)
#define LTC_ESTA_CL1_MASK                        (0xF00U)
#define LTC_ESTA_CL1_SHIFT                       (8U)
#define LTC_ESTA_CL1(x)                          (((uint32_t)(((uint32_t)(x)) << LTC_ESTA_CL1_SHIFT)) & LTC_ESTA_CL1_MASK)

/*! @name AADSZ - LTC AAD Size Register */
#define LTC_AADSZ_AADSZ_MASK                     (0xFU)
#define LTC_AADSZ_AADSZ_SHIFT                    (0U)
#define LTC_AADSZ_AADSZ(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_AADSZ_AADSZ_SHIFT)) & LTC_AADSZ_AADSZ_MASK)
#define LTC_AADSZ_AL_MASK                        (0x80000000U)
#define LTC_AADSZ_AL_SHIFT                       (31U)
#define LTC_AADSZ_AL(x)                          (((uint32_t)(((uint32_t)(x)) << LTC_AADSZ_AL_SHIFT)) & LTC_AADSZ_AL_MASK)

/*! @name CTX - LTC Context Register */
#define LTC_CTX_CTX_MASK                         (0xFFFFFFFFU)
#define LTC_CTX_CTX_SHIFT                        (0U)
#define LTC_CTX_CTX(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_CTX_CTX_SHIFT)) & LTC_CTX_CTX_MASK)

/* The count of LTC_CTX */
#define LTC_CTX_COUNT                            (16U)

/*! @name KEY - LTC Key Registers */
#define LTC_KEY_KEY_MASK                         (0xFFFFFFFFU)
#define LTC_KEY_KEY_SHIFT                        (0U)
#define LTC_KEY_KEY(x)                           (((uint32_t)(((uint32_t)(x)) << LTC_KEY_KEY_SHIFT)) & LTC_KEY_KEY_MASK)

/* The count of LTC_KEY */
#define LTC_KEY_COUNT                            (4U)

/*! @name VID2 - LTC Version ID 2 Register */
#define LTC_VID2_ECO_REV_MASK                    (0xFFU)
#define LTC_VID2_ECO_REV_SHIFT                   (0U)
#define LTC_VID2_ECO_REV(x)                      (((uint32_t)(((uint32_t)(x)) << LTC_VID2_ECO_REV_SHIFT)) & LTC_VID2_ECO_REV_MASK)
#define LTC_VID2_ARCH_ERA_MASK                   (0xFF00U)
#define LTC_VID2_ARCH_ERA_SHIFT                  (8U)
#define LTC_VID2_ARCH_ERA(x)                     (((uint32_t)(((uint32_t)(x)) << LTC_VID2_ARCH_ERA_SHIFT)) & LTC_VID2_ARCH_ERA_MASK)

/*! @name FIFOSTA - LTC FIFO Status Register */
#define LTC_FIFOSTA_IFL_MASK                     (0x7FU)
#define LTC_FIFOSTA_IFL_SHIFT                    (0U)
#define LTC_FIFOSTA_IFL(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_FIFOSTA_IFL_SHIFT)) & LTC_FIFOSTA_IFL_MASK)
#define LTC_FIFOSTA_IFF_MASK                     (0x8000U)
#define LTC_FIFOSTA_IFF_SHIFT                    (15U)
#define LTC_FIFOSTA_IFF(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_FIFOSTA_IFF_SHIFT)) & LTC_FIFOSTA_IFF_MASK)
#define LTC_FIFOSTA_OFL_MASK                     (0x7F0000U)
#define LTC_FIFOSTA_OFL_SHIFT                    (16U)
#define LTC_FIFOSTA_OFL(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_FIFOSTA_OFL_SHIFT)) & LTC_FIFOSTA_OFL_MASK)
#define LTC_FIFOSTA_OFF_MASK                     (0x80000000U)
#define LTC_FIFOSTA_OFF_SHIFT                    (31U)
#define LTC_FIFOSTA_OFF(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_FIFOSTA_OFF_SHIFT)) & LTC_FIFOSTA_OFF_MASK)

/*! @name IFIFO - LTC Input Data FIFO */
#define LTC_IFIFO_IFIFO_MASK                     (0xFFFFFFFFU)
#define LTC_IFIFO_IFIFO_SHIFT                    (0U)
#define LTC_IFIFO_IFIFO(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_IFIFO_IFIFO_SHIFT)) & LTC_IFIFO_IFIFO_MASK)

/*! @name OFIFO - LTC Output Data FIFO */
#define LTC_OFIFO_OFIFO_MASK                     (0xFFFFFFFFU)
#define LTC_OFIFO_OFIFO_SHIFT                    (0U)
#define LTC_OFIFO_OFIFO(x)                       (((uint32_t)(((uint32_t)(x)) << LTC_OFIFO_OFIFO_SHIFT)) & LTC_OFIFO_OFIFO_MASK)

/*! @name VID1 - LTC Version ID Register */
#define LTC_VID1_MIN_REV_MASK                    (0xFFU)
#define LTC_VID1_MIN_REV_SHIFT                   (0U)
#define LTC_VID1_MIN_REV(x)                      (((uint32_t)(((uint32_t)(x)) << LTC_VID1_MIN_REV_SHIFT)) & LTC_VID1_MIN_REV_MASK)
#define LTC_VID1_MAJ_REV_MASK                    (0xFF00U)
#define LTC_VID1_MAJ_REV_SHIFT                   (8U)
#define LTC_VID1_MAJ_REV(x)                      (((uint32_t)(((uint32_t)(x)) << LTC_VID1_MAJ_REV_SHIFT)) & LTC_VID1_MAJ_REV_MASK)
#define LTC_VID1_IP_ID_MASK                      (0xFFFF0000U)
#define LTC_VID1_IP_ID_SHIFT                     (16U)
#define LTC_VID1_IP_ID(x)                        (((uint32_t)(((uint32_t)(x)) << LTC_VID1_IP_ID_SHIFT)) & LTC_VID1_IP_ID_MASK)

/*! @name CHAVID - LTC CHA Version ID Register */
#define LTC_CHAVID_AESREV_MASK                   (0xFU)
#define LTC_CHAVID_AESREV_SHIFT                  (0U)
#define LTC_CHAVID_AESREV(x)                     (((uint32_t)(((uint32_t)(x)) << LTC_CHAVID_AESREV_SHIFT)) & LTC_CHAVID_AESREV_MASK)
#define LTC_CHAVID_AESVID_MASK                   (0xF0U)
#define LTC_CHAVID_AESVID_SHIFT                  (4U)
#define LTC_CHAVID_AESVID(x)                     (((uint32_t)(((uint32_t)(x)) << LTC_CHAVID_AESVID_SHIFT)) & LTC_CHAVID_AESVID_MASK)


/*!
 * @}
 */ /* end of group LTC_Register_Masks */


/* LTC - Peripheral instance base addresses */
/** Peripheral LTC0 base address */
#define LTC0_BASE                                (0x40058000u)
/** Peripheral LTC0 base pointer */
#define LTC0                                     ((LTC_Type *)LTC0_BASE)
/** Array initializer of LTC peripheral base addresses */
#define LTC_BASE_ADDRS                           { LTC0_BASE }
/** Array initializer of LTC peripheral base pointers */
#define LTC_BASE_PTRS                            { LTC0 }
/** Interrupt vectors for the LTC peripheral type */
#define LTC_IRQS                                 { LTC0_IRQn }

/*!
 * @}
 */ /* end of group LTC_Peripheral_Access_Layer */


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
  __I  uint8_t C5;                                 /**< MCG Control 5 Register, offset: 0x4 */
  __IO uint8_t C6;                                 /**< MCG Control 6 Register, offset: 0x5 */
  __I  uint8_t S;                                  /**< MCG Status Register, offset: 0x6 */
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

/*! @name C6 - MCG Control 6 Register */
#define MCG_C6_CME_MASK                          (0x20U)
#define MCG_C6_CME_SHIFT                         (5U)
#define MCG_C6_CME(x)                            (((uint8_t)(((uint8_t)(x)) << MCG_C6_CME_SHIFT)) & MCG_C6_CME_MASK)

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
#define MCG_C7_OSCSEL_MASK                       (0x1U)
#define MCG_C7_OSCSEL_SHIFT                      (0U)
#define MCG_C7_OSCSEL(x)                         (((uint8_t)(((uint8_t)(x)) << MCG_C7_OSCSEL_SHIFT)) & MCG_C7_OSCSEL_MASK)

/*! @name C8 - MCG Control 8 Register */
#define MCG_C8_LOCS1_MASK                        (0x1U)
#define MCG_C8_LOCS1_SHIFT                       (0U)
#define MCG_C8_LOCS1(x)                          (((uint8_t)(((uint8_t)(x)) << MCG_C8_LOCS1_SHIFT)) & MCG_C8_LOCS1_MASK)
#define MCG_C8_CME1_MASK                         (0x20U)
#define MCG_C8_CME1_SHIFT                        (5U)
#define MCG_C8_CME1(x)                           (((uint8_t)(((uint8_t)(x)) << MCG_C8_CME1_SHIFT)) & MCG_C8_CME1_MASK)
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
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< PIT Module Control Register, offset: 0x0 */
       uint8_t RESERVED_0[220];
  __I  uint32_t LTMR64H;                           /**< PIT Upper Lifetime Timer Register, offset: 0xE0 */
  __I  uint32_t LTMR64L;                           /**< PIT Lower Lifetime Timer Register, offset: 0xE4 */
       uint8_t RESERVED_1[24];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __IO uint32_t LDVAL;                             /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    __IO uint32_t TFLG;                              /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[2];
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

/*! @name LTMR64H - PIT Upper Lifetime Timer Register */
#define PIT_LTMR64H_LTH_MASK                     (0xFFFFFFFFU)
#define PIT_LTMR64H_LTH_SHIFT                    (0U)
#define PIT_LTMR64H_LTH(x)                       (((uint32_t)(((uint32_t)(x)) << PIT_LTMR64H_LTH_SHIFT)) & PIT_LTMR64H_LTH_MASK)

/*! @name LTMR64L - PIT Lower Lifetime Timer Register */
#define PIT_LTMR64L_LTL_MASK                     (0xFFFFFFFFU)
#define PIT_LTMR64L_LTL_SHIFT                    (0U)
#define PIT_LTMR64L_LTL(x)                       (((uint32_t)(((uint32_t)(x)) << PIT_LTMR64L_LTL_SHIFT)) & PIT_LTMR64L_LTL_MASK)

/*! @name LDVAL - Timer Load Value Register */
#define PIT_LDVAL_TSV_MASK                       (0xFFFFFFFFU)
#define PIT_LDVAL_TSV_SHIFT                      (0U)
#define PIT_LDVAL_TSV(x)                         (((uint32_t)(((uint32_t)(x)) << PIT_LDVAL_TSV_SHIFT)) & PIT_LDVAL_TSV_MASK)

/* The count of PIT_LDVAL */
#define PIT_LDVAL_COUNT                          (2U)

/*! @name CVAL - Current Timer Value Register */
#define PIT_CVAL_TVL_MASK                        (0xFFFFFFFFU)
#define PIT_CVAL_TVL_SHIFT                       (0U)
#define PIT_CVAL_TVL(x)                          (((uint32_t)(((uint32_t)(x)) << PIT_CVAL_TVL_SHIFT)) & PIT_CVAL_TVL_MASK)

/* The count of PIT_CVAL */
#define PIT_CVAL_COUNT                           (2U)

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
#define PIT_TCTRL_COUNT                          (2U)

/*! @name TFLG - Timer Flag Register */
#define PIT_TFLG_TIF_MASK                        (0x1U)
#define PIT_TFLG_TIF_SHIFT                       (0U)
#define PIT_TFLG_TIF(x)                          (((uint32_t)(((uint32_t)(x)) << PIT_TFLG_TIF_SHIFT)) & PIT_TFLG_TIF_MASK)

/* The count of PIT_TFLG */
#define PIT_TFLG_COUNT                           (2U)


/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT base address */
#define PIT_BASE                                 (0x40037000u)
/** Peripheral PIT base pointer */
#define PIT                                      ((PIT_Type *)PIT_BASE)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS                           { PIT_BASE }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT }
/** Interrupt vectors for the PIT peripheral type */
#define PIT_IRQS                                 { PIT_IRQn, PIT_IRQn }

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
#define PMC_REGSC_VLPO_MASK                      (0x40U)
#define PMC_REGSC_VLPO_SHIFT                     (6U)
#define PMC_REGSC_VLPO(x)                        (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_VLPO_SHIFT)) & PMC_REGSC_VLPO_MASK)


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
#define PMC_IRQS                                 { LVD_LVW_DCDC_IRQn }

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
#define PORT_PCR_DSE_MASK                        (0x40U)
#define PORT_PCR_DSE_SHIFT                       (6U)
#define PORT_PCR_DSE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_DSE_SHIFT)) & PORT_PCR_DSE_MASK)
#define PORT_PCR_MUX_MASK                        (0x700U)
#define PORT_PCR_MUX_SHIFT                       (8U)
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_MUX_SHIFT)) & PORT_PCR_MUX_MASK)
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
/** Array initializer of PORT peripheral base addresses */
#define PORT_BASE_ADDRS                          { PORTA_BASE, PORTB_BASE, PORTC_BASE }
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORTA, PORTB, PORTC }
/** Interrupt vectors for the PORT peripheral type */
#define PORT_IRQS                                { PORTA_IRQn, PORTB_PORTC_IRQn, PORTB_PORTC_IRQn }

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
  __I  uint8_t SRS0;                               /**< System Reset Status Register 0, offset: 0x0 */
  __I  uint8_t SRS1;                               /**< System Reset Status Register 1, offset: 0x1 */
       uint8_t RESERVED_0[2];
  __IO uint8_t RPFC;                               /**< Reset Pin Filter Control register, offset: 0x4 */
  __IO uint8_t RPFW;                               /**< Reset Pin Filter Width register, offset: 0x5 */
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
   -- RSIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RSIM_Peripheral_Access_Layer RSIM Peripheral Access Layer
 * @{
 */

/** RSIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t CONTROL;                           /**< RSIM Control, offset: 0x0 */
  __IO uint32_t ACTIVE_DELAY;                      /**< RSIM BLE Active Delay, offset: 0x4 */
  __I  uint32_t MAC_MSB;                           /**< RSIM MAC MSB, offset: 0x8 */
  __I  uint32_t MAC_LSB;                           /**< RSIM MAC LSB, offset: 0xC */
  __IO uint32_t ANA_TEST;                          /**< RSIM Analog Test, offset: 0x10 */
} RSIM_Type;

/* ----------------------------------------------------------------------------
   -- RSIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RSIM_Register_Masks RSIM Register Masks
 * @{
 */

/*! @name CONTROL - RSIM Control */
#define RSIM_CONTROL_BLE_RF_OSC_REQ_EN_MASK      (0x1U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_EN_SHIFT     (0U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_EN(x)        (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_RF_OSC_REQ_EN_SHIFT)) & RSIM_CONTROL_BLE_RF_OSC_REQ_EN_MASK)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_STAT_MASK    (0x2U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_STAT_SHIFT   (1U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_STAT(x)      (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_RF_OSC_REQ_STAT_SHIFT)) & RSIM_CONTROL_BLE_RF_OSC_REQ_STAT_MASK)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT_EN_MASK  (0x10U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT_EN_SHIFT (4U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT_EN(x)    (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_RF_OSC_REQ_INT_EN_SHIFT)) & RSIM_CONTROL_BLE_RF_OSC_REQ_INT_EN_MASK)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT_MASK     (0x20U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT_SHIFT    (5U)
#define RSIM_CONTROL_BLE_RF_OSC_REQ_INT(x)       (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_RF_OSC_REQ_INT_SHIFT)) & RSIM_CONTROL_BLE_RF_OSC_REQ_INT_MASK)
#define RSIM_CONTROL_RF_OSC_EN_MASK              (0xF00U)
#define RSIM_CONTROL_RF_OSC_EN_SHIFT             (8U)
#define RSIM_CONTROL_RF_OSC_EN(x)                (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RF_OSC_EN_SHIFT)) & RSIM_CONTROL_RF_OSC_EN_MASK)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD_EN_MASK  (0x1000U)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD_EN_SHIFT (12U)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_GASKET_BYPASS_OVRD_EN_SHIFT)) & RSIM_CONTROL_GASKET_BYPASS_OVRD_EN_MASK)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD_MASK     (0x2000U)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD_SHIFT    (13U)
#define RSIM_CONTROL_GASKET_BYPASS_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_GASKET_BYPASS_OVRD_SHIFT)) & RSIM_CONTROL_GASKET_BYPASS_OVRD_MASK)
#define RSIM_CONTROL_RF_OSC_BYPASS_EN_MASK       (0x4000U)
#define RSIM_CONTROL_RF_OSC_BYPASS_EN_SHIFT      (14U)
#define RSIM_CONTROL_RF_OSC_BYPASS_EN(x)         (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RF_OSC_BYPASS_EN_SHIFT)) & RSIM_CONTROL_RF_OSC_BYPASS_EN_MASK)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_1_SEL_MASK  (0x10000U)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_1_SEL_SHIFT (16U)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_1_SEL(x)    (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_ACTIVE_PORT_1_SEL_SHIFT)) & RSIM_CONTROL_BLE_ACTIVE_PORT_1_SEL_MASK)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_2_SEL_MASK  (0x20000U)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_2_SEL_SHIFT (17U)
#define RSIM_CONTROL_BLE_ACTIVE_PORT_2_SEL(x)    (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_ACTIVE_PORT_2_SEL_SHIFT)) & RSIM_CONTROL_BLE_ACTIVE_PORT_2_SEL_MASK)
#define RSIM_CONTROL_BLE_DEEP_SLEEP_EXIT_MASK    (0x100000U)
#define RSIM_CONTROL_BLE_DEEP_SLEEP_EXIT_SHIFT   (20U)
#define RSIM_CONTROL_BLE_DEEP_SLEEP_EXIT(x)      (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLE_DEEP_SLEEP_EXIT_SHIFT)) & RSIM_CONTROL_BLE_DEEP_SLEEP_EXIT_MASK)
#define RSIM_CONTROL_STOP_ACK_OVRD_EN_MASK       (0x400000U)
#define RSIM_CONTROL_STOP_ACK_OVRD_EN_SHIFT      (22U)
#define RSIM_CONTROL_STOP_ACK_OVRD_EN(x)         (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_STOP_ACK_OVRD_EN_SHIFT)) & RSIM_CONTROL_STOP_ACK_OVRD_EN_MASK)
#define RSIM_CONTROL_STOP_ACK_OVRD_MASK          (0x800000U)
#define RSIM_CONTROL_STOP_ACK_OVRD_SHIFT         (23U)
#define RSIM_CONTROL_STOP_ACK_OVRD(x)            (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_STOP_ACK_OVRD_SHIFT)) & RSIM_CONTROL_STOP_ACK_OVRD_MASK)
#define RSIM_CONTROL_RF_OSC_READY_MASK           (0x1000000U)
#define RSIM_CONTROL_RF_OSC_READY_SHIFT          (24U)
#define RSIM_CONTROL_RF_OSC_READY(x)             (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RF_OSC_READY_SHIFT)) & RSIM_CONTROL_RF_OSC_READY_MASK)
#define RSIM_CONTROL_RF_OSC_READY_OVRD_EN_MASK   (0x2000000U)
#define RSIM_CONTROL_RF_OSC_READY_OVRD_EN_SHIFT  (25U)
#define RSIM_CONTROL_RF_OSC_READY_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RF_OSC_READY_OVRD_EN_SHIFT)) & RSIM_CONTROL_RF_OSC_READY_OVRD_EN_MASK)
#define RSIM_CONTROL_RF_OSC_READY_OVRD_MASK      (0x4000000U)
#define RSIM_CONTROL_RF_OSC_READY_OVRD_SHIFT     (26U)
#define RSIM_CONTROL_RF_OSC_READY_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RF_OSC_READY_OVRD_SHIFT)) & RSIM_CONTROL_RF_OSC_READY_OVRD_MASK)
#define RSIM_CONTROL_BLOCK_RADIO_RESETS_MASK     (0x10000000U)
#define RSIM_CONTROL_BLOCK_RADIO_RESETS_SHIFT    (28U)
#define RSIM_CONTROL_BLOCK_RADIO_RESETS(x)       (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLOCK_RADIO_RESETS_SHIFT)) & RSIM_CONTROL_BLOCK_RADIO_RESETS_MASK)
#define RSIM_CONTROL_BLOCK_RADIO_OUTPUTS_MASK    (0x20000000U)
#define RSIM_CONTROL_BLOCK_RADIO_OUTPUTS_SHIFT   (29U)
#define RSIM_CONTROL_BLOCK_RADIO_OUTPUTS(x)      (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_BLOCK_RADIO_OUTPUTS_SHIFT)) & RSIM_CONTROL_BLOCK_RADIO_OUTPUTS_MASK)
#define RSIM_CONTROL_RADIO_RESET_MASK            (0x80000000U)
#define RSIM_CONTROL_RADIO_RESET_SHIFT           (31U)
#define RSIM_CONTROL_RADIO_RESET(x)              (((uint32_t)(((uint32_t)(x)) << RSIM_CONTROL_RADIO_RESET_SHIFT)) & RSIM_CONTROL_RADIO_RESET_MASK)

/*! @name ACTIVE_DELAY - RSIM BLE Active Delay */
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_FINE_DELAY_MASK (0x3FU)
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_FINE_DELAY_SHIFT (0U)
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_FINE_DELAY(x) (((uint32_t)(((uint32_t)(x)) << RSIM_ACTIVE_DELAY_BLE_ACTIVE_FINE_DELAY_SHIFT)) & RSIM_ACTIVE_DELAY_BLE_ACTIVE_FINE_DELAY_MASK)
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_COARSE_DELAY_MASK (0xF0000U)
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_COARSE_DELAY_SHIFT (16U)
#define RSIM_ACTIVE_DELAY_BLE_ACTIVE_COARSE_DELAY(x) (((uint32_t)(((uint32_t)(x)) << RSIM_ACTIVE_DELAY_BLE_ACTIVE_COARSE_DELAY_SHIFT)) & RSIM_ACTIVE_DELAY_BLE_ACTIVE_COARSE_DELAY_MASK)

/*! @name MAC_MSB - RSIM MAC MSB */
#define RSIM_MAC_MSB_MAC_ADDR_MSB_MASK           (0xFFU)
#define RSIM_MAC_MSB_MAC_ADDR_MSB_SHIFT          (0U)
#define RSIM_MAC_MSB_MAC_ADDR_MSB(x)             (((uint32_t)(((uint32_t)(x)) << RSIM_MAC_MSB_MAC_ADDR_MSB_SHIFT)) & RSIM_MAC_MSB_MAC_ADDR_MSB_MASK)

/*! @name MAC_LSB - RSIM MAC LSB */
#define RSIM_MAC_LSB_MAC_ADDR_LSB_MASK           (0xFFFFFFFFU)
#define RSIM_MAC_LSB_MAC_ADDR_LSB_SHIFT          (0U)
#define RSIM_MAC_LSB_MAC_ADDR_LSB(x)             (((uint32_t)(((uint32_t)(x)) << RSIM_MAC_LSB_MAC_ADDR_LSB_SHIFT)) & RSIM_MAC_LSB_MAC_ADDR_LSB_MASK)

/*! @name ANA_TEST - RSIM Analog Test */
#define RSIM_ANA_TEST_ATST_GATE_EN_MASK          (0x1FU)
#define RSIM_ANA_TEST_ATST_GATE_EN_SHIFT         (0U)
#define RSIM_ANA_TEST_ATST_GATE_EN(x)            (((uint32_t)(((uint32_t)(x)) << RSIM_ANA_TEST_ATST_GATE_EN_SHIFT)) & RSIM_ANA_TEST_ATST_GATE_EN_MASK)
#define RSIM_ANA_TEST_RADIO_ID_MASK              (0xF000000U)
#define RSIM_ANA_TEST_RADIO_ID_SHIFT             (24U)
#define RSIM_ANA_TEST_RADIO_ID(x)                (((uint32_t)(((uint32_t)(x)) << RSIM_ANA_TEST_RADIO_ID_SHIFT)) & RSIM_ANA_TEST_RADIO_ID_MASK)


/*!
 * @}
 */ /* end of group RSIM_Register_Masks */


/* RSIM - Peripheral instance base addresses */
/** Peripheral RSIM base address */
#define RSIM_BASE                                (0x40059000u)
/** Peripheral RSIM base pointer */
#define RSIM                                     ((RSIM_Type *)RSIM_BASE)
/** Array initializer of RSIM peripheral base addresses */
#define RSIM_BASE_ADDRS                          { RSIM_BASE }
/** Array initializer of RSIM peripheral base pointers */
#define RSIM_BASE_PTRS                           { RSIM }

/*!
 * @}
 */ /* end of group RSIM_Peripheral_Access_Layer */


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
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SOPT1;                             /**< System Options Register 1, offset: 0x0 */
       uint8_t RESERVED_0[4096];
  __IO uint32_t SOPT2;                             /**< System Options Register 2, offset: 0x1004 */
       uint8_t RESERVED_1[4];
  __IO uint32_t SOPT4;                             /**< System Options Register 4, offset: 0x100C */
  __IO uint32_t SOPT5;                             /**< System Options Register 5, offset: 0x1010 */
       uint8_t RESERVED_2[4];
  __IO uint32_t SOPT7;                             /**< System Options Register 7, offset: 0x1018 */
       uint8_t RESERVED_3[8];
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x1024 */
       uint8_t RESERVED_4[12];
  __IO uint32_t SCGC4;                             /**< System Clock Gating Control Register 4, offset: 0x1034 */
  __IO uint32_t SCGC5;                             /**< System Clock Gating Control Register 5, offset: 0x1038 */
  __IO uint32_t SCGC6;                             /**< System Clock Gating Control Register 6, offset: 0x103C */
  __IO uint32_t SCGC7;                             /**< System Clock Gating Control Register 7, offset: 0x1040 */
  __IO uint32_t CLKDIV1;                           /**< System Clock Divider Register 1, offset: 0x1044 */
       uint8_t RESERVED_5[4];
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x104C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x1050 */
       uint8_t RESERVED_6[4];
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x1058 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x105C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x1060 */
       uint8_t RESERVED_7[156];
  __IO uint32_t COPC;                              /**< COP Control Register, offset: 0x1100 */
  __O  uint32_t SRVCOP;                            /**< Service COP, offset: 0x1104 */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name SOPT1 - System Options Register 1 */
#define SIM_SOPT1_OSC32KOUT_MASK                 (0x30000U)
#define SIM_SOPT1_OSC32KOUT_SHIFT                (16U)
#define SIM_SOPT1_OSC32KOUT(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_OSC32KOUT_SHIFT)) & SIM_SOPT1_OSC32KOUT_MASK)
#define SIM_SOPT1_OSC32KSEL_MASK                 (0xC0000U)
#define SIM_SOPT1_OSC32KSEL_SHIFT                (18U)
#define SIM_SOPT1_OSC32KSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT1_OSC32KSEL_SHIFT)) & SIM_SOPT1_OSC32KSEL_MASK)

/*! @name SOPT2 - System Options Register 2 */
#define SIM_SOPT2_CLKOUTSEL_MASK                 (0xE0U)
#define SIM_SOPT2_CLKOUTSEL_SHIFT                (5U)
#define SIM_SOPT2_CLKOUTSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_CLKOUTSEL_SHIFT)) & SIM_SOPT2_CLKOUTSEL_MASK)
#define SIM_SOPT2_TPMSRC_MASK                    (0x3000000U)
#define SIM_SOPT2_TPMSRC_SHIFT                   (24U)
#define SIM_SOPT2_TPMSRC(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_TPMSRC_SHIFT)) & SIM_SOPT2_TPMSRC_MASK)
#define SIM_SOPT2_LPUART0SRC_MASK                (0xC000000U)
#define SIM_SOPT2_LPUART0SRC_SHIFT               (26U)
#define SIM_SOPT2_LPUART0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT2_LPUART0SRC_SHIFT)) & SIM_SOPT2_LPUART0SRC_MASK)

/*! @name SOPT4 - System Options Register 4 */
#define SIM_SOPT4_TPM1CH0SRC_MASK                (0x40000U)
#define SIM_SOPT4_TPM1CH0SRC_SHIFT               (18U)
#define SIM_SOPT4_TPM1CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_TPM1CH0SRC_SHIFT)) & SIM_SOPT4_TPM1CH0SRC_MASK)
#define SIM_SOPT4_TPM2CH0SRC_MASK                (0x100000U)
#define SIM_SOPT4_TPM2CH0SRC_SHIFT               (20U)
#define SIM_SOPT4_TPM2CH0SRC(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_TPM2CH0SRC_SHIFT)) & SIM_SOPT4_TPM2CH0SRC_MASK)
#define SIM_SOPT4_TPM0CLKSEL_MASK                (0x1000000U)
#define SIM_SOPT4_TPM0CLKSEL_SHIFT               (24U)
#define SIM_SOPT4_TPM0CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_TPM0CLKSEL_SHIFT)) & SIM_SOPT4_TPM0CLKSEL_MASK)
#define SIM_SOPT4_TPM1CLKSEL_MASK                (0x2000000U)
#define SIM_SOPT4_TPM1CLKSEL_SHIFT               (25U)
#define SIM_SOPT4_TPM1CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_TPM1CLKSEL_SHIFT)) & SIM_SOPT4_TPM1CLKSEL_MASK)
#define SIM_SOPT4_TPM2CLKSEL_MASK                (0x4000000U)
#define SIM_SOPT4_TPM2CLKSEL_SHIFT               (26U)
#define SIM_SOPT4_TPM2CLKSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT4_TPM2CLKSEL_SHIFT)) & SIM_SOPT4_TPM2CLKSEL_MASK)

/*! @name SOPT5 - System Options Register 5 */
#define SIM_SOPT5_LPUART0TXSRC_MASK              (0x3U)
#define SIM_SOPT5_LPUART0TXSRC_SHIFT             (0U)
#define SIM_SOPT5_LPUART0TXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART0TXSRC_SHIFT)) & SIM_SOPT5_LPUART0TXSRC_MASK)
#define SIM_SOPT5_LPUART0RXSRC_MASK              (0x4U)
#define SIM_SOPT5_LPUART0RXSRC_SHIFT             (2U)
#define SIM_SOPT5_LPUART0RXSRC(x)                (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART0RXSRC_SHIFT)) & SIM_SOPT5_LPUART0RXSRC_MASK)
#define SIM_SOPT5_LPUART0ODE_MASK                (0x10000U)
#define SIM_SOPT5_LPUART0ODE_SHIFT               (16U)
#define SIM_SOPT5_LPUART0ODE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_SOPT5_LPUART0ODE_SHIFT)) & SIM_SOPT5_LPUART0ODE_MASK)

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

/*! @name SDID - System Device Identification Register */
#define SIM_SDID_PINID_MASK                      (0xFU)
#define SIM_SDID_PINID_SHIFT                     (0U)
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PINID_SHIFT)) & SIM_SDID_PINID_MASK)
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
#define SIM_SDID_SUBFAMID_MASK                   (0x3000000U)
#define SIM_SDID_SUBFAMID_SHIFT                  (24U)
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SUBFAMID_SHIFT)) & SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMID_MASK                      (0xF0000000U)
#define SIM_SDID_FAMID_SHIFT                     (28U)
#define SIM_SDID_FAMID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FAMID_SHIFT)) & SIM_SDID_FAMID_MASK)

/*! @name SCGC4 - System Clock Gating Control Register 4 */
#define SIM_SCGC4_CMT_MASK                       (0x4U)
#define SIM_SCGC4_CMT_SHIFT                      (2U)
#define SIM_SCGC4_CMT(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_CMT_SHIFT)) & SIM_SCGC4_CMT_MASK)
#define SIM_SCGC4_I2C0_MASK                      (0x40U)
#define SIM_SCGC4_I2C0_SHIFT                     (6U)
#define SIM_SCGC4_I2C0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_I2C0_SHIFT)) & SIM_SCGC4_I2C0_MASK)
#define SIM_SCGC4_I2C1_MASK                      (0x80U)
#define SIM_SCGC4_I2C1_SHIFT                     (7U)
#define SIM_SCGC4_I2C1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_I2C1_SHIFT)) & SIM_SCGC4_I2C1_MASK)
#define SIM_SCGC4_CMP_MASK                       (0x80000U)
#define SIM_SCGC4_CMP_SHIFT                      (19U)
#define SIM_SCGC4_CMP(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC4_CMP_SHIFT)) & SIM_SCGC4_CMP_MASK)

/*! @name SCGC5 - System Clock Gating Control Register 5 */
#define SIM_SCGC5_LPTMR_MASK                     (0x1U)
#define SIM_SCGC5_LPTMR_SHIFT                    (0U)
#define SIM_SCGC5_LPTMR(x)                       (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_LPTMR_SHIFT)) & SIM_SCGC5_LPTMR_MASK)
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
#define SIM_SCGC5_LPUART0_MASK                   (0x100000U)
#define SIM_SCGC5_LPUART0_SHIFT                  (20U)
#define SIM_SCGC5_LPUART0(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_LPUART0_SHIFT)) & SIM_SCGC5_LPUART0_MASK)
#define SIM_SCGC5_LTC_MASK                       (0x1000000U)
#define SIM_SCGC5_LTC_SHIFT                      (24U)
#define SIM_SCGC5_LTC(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_LTC_SHIFT)) & SIM_SCGC5_LTC_MASK)
#define SIM_SCGC5_RSIM_MASK                      (0x2000000U)
#define SIM_SCGC5_RSIM_SHIFT                     (25U)
#define SIM_SCGC5_RSIM(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_RSIM_SHIFT)) & SIM_SCGC5_RSIM_MASK)
#define SIM_SCGC5_DCDC_MASK                      (0x4000000U)
#define SIM_SCGC5_DCDC_SHIFT                     (26U)
#define SIM_SCGC5_DCDC(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_DCDC_SHIFT)) & SIM_SCGC5_DCDC_MASK)
#define SIM_SCGC5_BTLL_MASK                      (0x8000000U)
#define SIM_SCGC5_BTLL_SHIFT                     (27U)
#define SIM_SCGC5_BTLL(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_BTLL_SHIFT)) & SIM_SCGC5_BTLL_MASK)
#define SIM_SCGC5_PHYDIG_MASK                    (0x10000000U)
#define SIM_SCGC5_PHYDIG_SHIFT                   (28U)
#define SIM_SCGC5_PHYDIG(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_PHYDIG_SHIFT)) & SIM_SCGC5_PHYDIG_MASK)
#define SIM_SCGC5_ZigBee_MASK                    (0x20000000U)
#define SIM_SCGC5_ZigBee_SHIFT                   (29U)
#define SIM_SCGC5_ZigBee(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC5_ZigBee_SHIFT)) & SIM_SCGC5_ZigBee_MASK)

/*! @name SCGC6 - System Clock Gating Control Register 6 */
#define SIM_SCGC6_FTF_MASK                       (0x1U)
#define SIM_SCGC6_FTF_SHIFT                      (0U)
#define SIM_SCGC6_FTF(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_FTF_SHIFT)) & SIM_SCGC6_FTF_MASK)
#define SIM_SCGC6_DMAMUX_MASK                    (0x2U)
#define SIM_SCGC6_DMAMUX_SHIFT                   (1U)
#define SIM_SCGC6_DMAMUX(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_DMAMUX_SHIFT)) & SIM_SCGC6_DMAMUX_MASK)
#define SIM_SCGC6_TRNG_MASK                      (0x200U)
#define SIM_SCGC6_TRNG_SHIFT                     (9U)
#define SIM_SCGC6_TRNG(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_TRNG_SHIFT)) & SIM_SCGC6_TRNG_MASK)
#define SIM_SCGC6_SPI0_MASK                      (0x1000U)
#define SIM_SCGC6_SPI0_SHIFT                     (12U)
#define SIM_SCGC6_SPI0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_SPI0_SHIFT)) & SIM_SCGC6_SPI0_MASK)
#define SIM_SCGC6_SPI1_MASK                      (0x2000U)
#define SIM_SCGC6_SPI1_SHIFT                     (13U)
#define SIM_SCGC6_SPI1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_SPI1_SHIFT)) & SIM_SCGC6_SPI1_MASK)
#define SIM_SCGC6_PIT_MASK                       (0x800000U)
#define SIM_SCGC6_PIT_SHIFT                      (23U)
#define SIM_SCGC6_PIT(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_PIT_SHIFT)) & SIM_SCGC6_PIT_MASK)
#define SIM_SCGC6_TPM0_MASK                      (0x1000000U)
#define SIM_SCGC6_TPM0_SHIFT                     (24U)
#define SIM_SCGC6_TPM0(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_TPM0_SHIFT)) & SIM_SCGC6_TPM0_MASK)
#define SIM_SCGC6_TPM1_MASK                      (0x2000000U)
#define SIM_SCGC6_TPM1_SHIFT                     (25U)
#define SIM_SCGC6_TPM1(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_TPM1_SHIFT)) & SIM_SCGC6_TPM1_MASK)
#define SIM_SCGC6_TPM2_MASK                      (0x4000000U)
#define SIM_SCGC6_TPM2_SHIFT                     (26U)
#define SIM_SCGC6_TPM2(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SCGC6_TPM2_SHIFT)) & SIM_SCGC6_TPM2_MASK)
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
#define SIM_SCGC7_DMA_MASK                       (0x100U)
#define SIM_SCGC7_DMA_SHIFT                      (8U)
#define SIM_SCGC7_DMA(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_SCGC7_DMA_SHIFT)) & SIM_SCGC7_DMA_MASK)

/*! @name CLKDIV1 - System Clock Divider Register 1 */
#define SIM_CLKDIV1_OUTDIV4_MASK                 (0x70000U)
#define SIM_CLKDIV1_OUTDIV4_SHIFT                (16U)
#define SIM_CLKDIV1_OUTDIV4(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV4_SHIFT)) & SIM_CLKDIV1_OUTDIV4_MASK)
#define SIM_CLKDIV1_OUTDIV1_MASK                 (0xF0000000U)
#define SIM_CLKDIV1_OUTDIV1_SHIFT                (28U)
#define SIM_CLKDIV1_OUTDIV1(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV1_OUTDIV1_SHIFT)) & SIM_CLKDIV1_OUTDIV1_MASK)

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

/*! @name COPC - COP Control Register */
#define SIM_COPC_COPW_MASK                       (0x1U)
#define SIM_COPC_COPW_SHIFT                      (0U)
#define SIM_COPC_COPW(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPW_SHIFT)) & SIM_COPC_COPW_MASK)
#define SIM_COPC_COPCLKS_MASK                    (0x2U)
#define SIM_COPC_COPCLKS_SHIFT                   (1U)
#define SIM_COPC_COPCLKS(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPCLKS_SHIFT)) & SIM_COPC_COPCLKS_MASK)
#define SIM_COPC_COPT_MASK                       (0xCU)
#define SIM_COPC_COPT_SHIFT                      (2U)
#define SIM_COPC_COPT(x)                         (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPT_SHIFT)) & SIM_COPC_COPT_MASK)
#define SIM_COPC_COPSTPEN_MASK                   (0x10U)
#define SIM_COPC_COPSTPEN_SHIFT                  (4U)
#define SIM_COPC_COPSTPEN(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPSTPEN_SHIFT)) & SIM_COPC_COPSTPEN_MASK)
#define SIM_COPC_COPDBGEN_MASK                   (0x20U)
#define SIM_COPC_COPDBGEN_SHIFT                  (5U)
#define SIM_COPC_COPDBGEN(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPDBGEN_SHIFT)) & SIM_COPC_COPDBGEN_MASK)
#define SIM_COPC_COPCLKSEL_MASK                  (0xC0U)
#define SIM_COPC_COPCLKSEL_SHIFT                 (6U)
#define SIM_COPC_COPCLKSEL(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_COPC_COPCLKSEL_SHIFT)) & SIM_COPC_COPCLKSEL_MASK)

/*! @name SRVCOP - Service COP */
#define SIM_SRVCOP_SRVCOP_MASK                   (0xFFU)
#define SIM_SRVCOP_SRVCOP_SHIFT                  (0U)
#define SIM_SRVCOP_SRVCOP(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SRVCOP_SRVCOP_SHIFT)) & SIM_SRVCOP_SRVCOP_MASK)


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
#define SPI_MCR_PCSIS_MASK                       (0xF0000U)
#define SPI_MCR_PCSIS_SHIFT                      (16U)
#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_MCR_PCSIS_SHIFT)) & SPI_MCR_PCSIS_MASK)
#define SPI_MCR_ROOE_MASK                        (0x1000000U)
#define SPI_MCR_ROOE_SHIFT                       (24U)
#define SPI_MCR_ROOE(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_MCR_ROOE_SHIFT)) & SPI_MCR_ROOE_MASK)
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
#define SPI_PUSHR_PCS_MASK                       (0xF0000U)
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
#define SPI_PUSHR_SLAVE_TXDATA_MASK              (0xFFFFFFFFU)
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
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE }
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0, SPI1 }
/** Interrupt vectors for the SPI peripheral type */
#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn }

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
  } CONTROLS[4];
       uint8_t RESERVED_0[36];
  __IO uint32_t STATUS;                            /**< Capture and Compare Status, offset: 0x50 */
       uint8_t RESERVED_1[16];
  __IO uint32_t COMBINE;                           /**< Combine Channel Register, offset: 0x64 */
       uint8_t RESERVED_2[16];
  __IO uint32_t FILTER;                            /**< Filter Control, offset: 0x78 */
       uint8_t RESERVED_3[4];
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
#define TPM_CnSC_COUNT                           (4U)

/*! @name CnV - Channel (n) Value */
#define TPM_CnV_VAL_MASK                         (0xFFFFU)
#define TPM_CnV_VAL_SHIFT                        (0U)
#define TPM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << TPM_CnV_VAL_SHIFT)) & TPM_CnV_VAL_MASK)

/* The count of TPM_CnV */
#define TPM_CnV_COUNT                            (4U)

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
#define TPM_COMBINE_COMBINE1_MASK                (0x100U)
#define TPM_COMBINE_COMBINE1_SHIFT               (8U)
#define TPM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMBINE1_SHIFT)) & TPM_COMBINE_COMBINE1_MASK)
#define TPM_COMBINE_COMSWAP1_MASK                (0x200U)
#define TPM_COMBINE_COMSWAP1_SHIFT               (9U)
#define TPM_COMBINE_COMSWAP1(x)                  (((uint32_t)(((uint32_t)(x)) << TPM_COMBINE_COMSWAP1_SHIFT)) & TPM_COMBINE_COMSWAP1_MASK)

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
#define TPM_CONF_TRGSEL_MASK                     (0xF000000U)
#define TPM_CONF_TRGSEL_SHIFT                    (24U)
#define TPM_CONF_TRGSEL(x)                       (((uint32_t)(((uint32_t)(x)) << TPM_CONF_TRGSEL_SHIFT)) & TPM_CONF_TRGSEL_MASK)


/*!
 * @}
 */ /* end of group TPM_Register_Masks */


/* TPM - Peripheral instance base addresses */
/** Peripheral TPM0 base address */
#define TPM0_BASE                                (0x40038000u)
/** Peripheral TPM0 base pointer */
#define TPM0                                     ((TPM_Type *)TPM0_BASE)
/** Peripheral TPM1 base address */
#define TPM1_BASE                                (0x40039000u)
/** Peripheral TPM1 base pointer */
#define TPM1                                     ((TPM_Type *)TPM1_BASE)
/** Peripheral TPM2 base address */
#define TPM2_BASE                                (0x4003A000u)
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
#define TRNG0_BASE                               (0x40029000u)
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
   -- XCVR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XCVR_Peripheral_Access_Layer XCVR Peripheral Access Layer
 * @{
 */

/** XCVR - Register Layout Typedef */
typedef struct {
  __IO uint32_t RX_DIG_CTRL;                       /**< RX Digital Control, offset: 0x0 */
  __IO uint32_t AGC_CTRL_0;                        /**< AGC Control 0, offset: 0x4 */
  __IO uint32_t AGC_CTRL_1;                        /**< AGC Control 1, offset: 0x8 */
  __IO uint32_t AGC_CTRL_2;                        /**< AGC Control 2, offset: 0xC */
  __IO uint32_t AGC_CTRL_3;                        /**< AGC Control 3, offset: 0x10 */
  __I  uint32_t AGC_STAT;                          /**< AGC Status, offset: 0x14 */
  __IO uint32_t RSSI_CTRL_0;                       /**< RSSI Control 0, offset: 0x18 */
  __IO uint32_t RSSI_CTRL_1;                       /**< RSSI Control 1, offset: 0x1C */
  __IO uint32_t DCOC_CTRL_0;                       /**< DCOC Control 0, offset: 0x20 */
  __IO uint32_t DCOC_CTRL_1;                       /**< DCOC Control 1, offset: 0x24 */
  __IO uint32_t DCOC_CTRL_2;                       /**< DCOC Control 2, offset: 0x28 */
  __IO uint32_t DCOC_CTRL_3;                       /**< DCOC Control 3, offset: 0x2C */
  __IO uint32_t DCOC_CTRL_4;                       /**< DCOC Control 4, offset: 0x30 */
  __IO uint32_t DCOC_CAL_GAIN;                     /**< DCOC Calibration Gain, offset: 0x34 */
  __I  uint32_t DCOC_STAT;                         /**< DCOC Status, offset: 0x38 */
  __I  uint32_t DCOC_DC_EST;                       /**< DCOC DC Estimate, offset: 0x3C */
  __IO uint32_t DCOC_CAL_RCP;                      /**< DCOC Calibration Reciprocals, offset: 0x40 */
       uint8_t RESERVED_0[8];
  __IO uint32_t IQMC_CTRL;                         /**< IQMC Control, offset: 0x4C */
  __IO uint32_t IQMC_CAL;                          /**< IQMC Calibration, offset: 0x50 */
  __IO uint32_t TCA_AGC_VAL_3_0;                   /**< TCA AGC Step Values 3..0, offset: 0x54 */
  __IO uint32_t TCA_AGC_VAL_7_4;                   /**< TCA AGC Step Values 7..4, offset: 0x58 */
  __IO uint32_t TCA_AGC_VAL_8;                     /**< TCA AGC Step Values 8, offset: 0x5C */
  __IO uint32_t BBF_RES_TUNE_VAL_7_0;              /**< BBF Resistor Tune Values 7..0, offset: 0x60 */
  __IO uint32_t BBF_RES_TUNE_VAL_10_8;             /**< BBF Resistor Tune Values 10..8, offset: 0x64 */
  __IO uint32_t TCA_AGC_LIN_VAL_2_0;               /**< TCA AGC Linear Gain Values 2..0, offset: 0x68 */
  __IO uint32_t TCA_AGC_LIN_VAL_5_3;               /**< TCA AGC Linear Gain Values 5..3, offset: 0x6C */
  __IO uint32_t TCA_AGC_LIN_VAL_8_6;               /**< TCA AGC Linear Gain Values 8..6, offset: 0x70 */
  __IO uint32_t BBF_RES_TUNE_LIN_VAL_3_0;          /**< BBF Resistor Tune Values 3..0, offset: 0x74 */
  __IO uint32_t BBF_RES_TUNE_LIN_VAL_7_4;          /**< BBF Resistor Tune Values 7..4, offset: 0x78 */
  __IO uint32_t BBF_RES_TUNE_LIN_VAL_10_8;         /**< BBF Resistor Tune Values 10..8, offset: 0x7C */
  __IO uint32_t AGC_GAIN_TBL_03_00;                /**< AGC Gain Tables Step 03..00, offset: 0x80 */
  __IO uint32_t AGC_GAIN_TBL_07_04;                /**< AGC Gain Tables Step 07..04, offset: 0x84 */
  __IO uint32_t AGC_GAIN_TBL_11_08;                /**< AGC Gain Tables Step 11..08, offset: 0x88 */
  __IO uint32_t AGC_GAIN_TBL_15_12;                /**< AGC Gain Tables Step 15..12, offset: 0x8C */
  __IO uint32_t AGC_GAIN_TBL_19_16;                /**< AGC Gain Tables Step 19..16, offset: 0x90 */
  __IO uint32_t AGC_GAIN_TBL_23_20;                /**< AGC Gain Tables Step 23..20, offset: 0x94 */
  __IO uint32_t AGC_GAIN_TBL_26_24;                /**< AGC Gain Tables Step 26..24, offset: 0x98 */
       uint8_t RESERVED_1[4];
  __IO uint32_t DCOC_OFFSET_[27];                  /**< DCOC Offset, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_2[4];
  __IO uint32_t DCOC_TZA_STEP_[11];                /**< DCOC TZA DC step, array offset: 0x110, array step: 0x4 */
       uint8_t RESERVED_3[48];
  __I  uint32_t DCOC_CAL_ALPHA;                    /**< DCOC Calibration Alpha, offset: 0x16C */
  __I  uint32_t DCOC_CAL_BETA;                     /**< DCOC Calibration Beta, offset: 0x170 */
  __I  uint32_t DCOC_CAL_GAMMA;                    /**< DCOC Calibration Gamma, offset: 0x174 */
  __IO uint32_t DCOC_CAL_IIR;                      /**< DCOC Calibration IIR, offset: 0x178 */
       uint8_t RESERVED_4[4];
  __I  uint32_t DCOC_CAL[3];                       /**< DCOC Calibration Result, array offset: 0x180, array step: 0x4 */
       uint8_t RESERVED_5[20];
  __IO uint32_t RX_CHF_COEF[8];                    /**< Receive Channel Filter Coefficient, array offset: 0x1A0, array step: 0x4 */
       uint8_t RESERVED_6[64];
  __IO uint32_t TX_DIG_CTRL;                       /**< TX Digital Control, offset: 0x200 */
  __IO uint32_t TX_DATA_PAD_PAT;                   /**< TX Data Padding Pattern, offset: 0x204 */
  __IO uint32_t TX_GFSK_MOD_CTRL;                  /**< TX GFSK Modulation Control, offset: 0x208 */
  __IO uint32_t TX_GFSK_COEFF2;                    /**< TX GFSK Filter Coefficients 2, offset: 0x20C */
  __IO uint32_t TX_GFSK_COEFF1;                    /**< TX GFSK Filter Coefficients 1, offset: 0x210 */
  __IO uint32_t TX_FSK_MOD_SCALE;                  /**< TX FSK Modulation Scale, offset: 0x214 */
  __IO uint32_t TX_DFT_MOD_PAT;                    /**< TX DFT Modulation Pattern, offset: 0x218 */
  __IO uint32_t TX_DFT_TONE_0_1;                   /**< TX DFT Tones 0 and 1, offset: 0x21C */
  __IO uint32_t TX_DFT_TONE_2_3;                   /**< TX DFT Tones 2 and 3, offset: 0x220 */
       uint8_t RESERVED_7[4];
  __IO uint32_t PLL_MOD_OVRD;                      /**< PLL Modulation Overrides, offset: 0x228 */
  __IO uint32_t PLL_CHAN_MAP;                      /**< PLL Channel Mapping, offset: 0x22C */
  __IO uint32_t PLL_LOCK_DETECT;                   /**< PLL Lock Detect, offset: 0x230 */
  __IO uint32_t PLL_HP_MOD_CTRL;                   /**< PLL High Port Modulation Control, offset: 0x234 */
  __IO uint32_t PLL_HPM_CAL_CTRL;                  /**< PLL HPM Calibration Control, offset: 0x238 */
  __IO uint32_t PLL_LD_HPM_CAL1;                   /**< PLL Cycle Slip Lock Detect Configuration and HPM Calibration 1, offset: 0x23C */
  __IO uint32_t PLL_LD_HPM_CAL2;                   /**< PLL Cycle Slip Lock Detect Configuration and HPM Calibration 2, offset: 0x240 */
  __IO uint32_t PLL_HPM_SDM_FRACTION;              /**< PLL HPM SDM Fraction, offset: 0x244 */
  __IO uint32_t PLL_LP_MOD_CTRL;                   /**< PLL Low Port Modulation Control, offset: 0x248 */
  __IO uint32_t PLL_LP_SDM_CTRL1;                  /**< PLL Low Port SDM Control 1, offset: 0x24C */
  __IO uint32_t PLL_LP_SDM_CTRL2;                  /**< PLL Low Port SDM Control 2, offset: 0x250 */
  __IO uint32_t PLL_LP_SDM_CTRL3;                  /**< PLL Low Port SDM Control 3, offset: 0x254 */
  __I  uint32_t PLL_LP_SDM_NUM;                    /**< PLL Low Port SDM Numerator Applied, offset: 0x258 */
  __I  uint32_t PLL_LP_SDM_DENOM;                  /**< PLL Low Port SDM Denominator Applied, offset: 0x25C */
  __IO uint32_t PLL_DELAY_MATCH;                   /**< PLL Delay Matching, offset: 0x260 */
  __IO uint32_t PLL_CTUNE_CTRL;                    /**< PLL Coarse Tune Control, offset: 0x264 */
  __I  uint32_t PLL_CTUNE_CNT6;                    /**< PLL Coarse Tune Count 6, offset: 0x268 */
  __I  uint32_t PLL_CTUNE_CNT5_4;                  /**< PLL Coarse Tune Counts 5 and 4, offset: 0x26C */
  __I  uint32_t PLL_CTUNE_CNT3_2;                  /**< PLL Coarse Tune Counts 3 and 2, offset: 0x270 */
  __I  uint32_t PLL_CTUNE_CNT1_0;                  /**< PLL Coarse Tune Counts 1 and 0, offset: 0x274 */
  __I  uint32_t PLL_CTUNE_RESULTS;                 /**< PLL Coarse Tune Results, offset: 0x278 */
       uint8_t RESERVED_8[4];
  __IO uint32_t CTRL;                              /**< Transceiver Control, offset: 0x280 */
  __I  uint32_t STATUS;                            /**< Transceiver Status, offset: 0x284 */
  __I  uint32_t SOFT_RESET;                        /**< Soft Reset, offset: 0x288 */
       uint8_t RESERVED_9[4];
  __IO uint32_t OVERWRITE_VER;                     /**< Overwrite Version, offset: 0x290 */
  __IO uint32_t DMA_CTRL;                          /**< DMA Control, offset: 0x294 */
  __I  uint32_t DMA_DATA;                          /**< DMA Data, offset: 0x298 */
  __IO uint32_t DTEST_CTRL;                        /**< Digital Test Control, offset: 0x29C */
  __IO uint32_t PB_CTRL;                           /**< Packet Buffer Control Register, offset: 0x2A0 */
       uint8_t RESERVED_10[28];
  __IO uint32_t TSM_CTRL;                          /**< Transceiver Sequence Manager Control, offset: 0x2C0 */
  __IO uint32_t END_OF_SEQ;                        /**< End of Sequence Control, offset: 0x2C4 */
  __IO uint32_t TSM_OVRD0;                         /**< TSM Override 0, offset: 0x2C8 */
  __IO uint32_t TSM_OVRD1;                         /**< TSM Override 1, offset: 0x2CC */
  __IO uint32_t TSM_OVRD2;                         /**< TSM Override 2, offset: 0x2D0 */
  __IO uint32_t TSM_OVRD3;                         /**< TSM Override 3, offset: 0x2D4 */
  __IO uint32_t PA_POWER;                          /**< PA Power, offset: 0x2D8 */
  __IO uint32_t PA_BIAS_TBL0;                      /**< PA Bias Table 0, offset: 0x2DC */
  __IO uint32_t PA_BIAS_TBL1;                      /**< PA Bias Table 1, offset: 0x2E0 */
  __IO uint32_t RECYCLE_COUNT;                     /**< Recycle Count Register, offset: 0x2E4 */
  __IO uint32_t TSM_TIMING00;                      /**< TSM_TIMING00, offset: 0x2E8 */
  __IO uint32_t TSM_TIMING01;                      /**< TSM_TIMING01, offset: 0x2EC */
  __IO uint32_t TSM_TIMING02;                      /**< TSM_TIMING02, offset: 0x2F0 */
  __IO uint32_t TSM_TIMING03;                      /**< TSM_TIMING03, offset: 0x2F4 */
  __IO uint32_t TSM_TIMING04;                      /**< TSM_TIMING04, offset: 0x2F8 */
  __IO uint32_t TSM_TIMING05;                      /**< TSM_TIMING05, offset: 0x2FC */
  __IO uint32_t TSM_TIMING06;                      /**< TSM_TIMING06, offset: 0x300 */
  __IO uint32_t TSM_TIMING07;                      /**< TSM_TIMING07, offset: 0x304 */
  __IO uint32_t TSM_TIMING08;                      /**< TSM_TIMING08, offset: 0x308 */
  __IO uint32_t TSM_TIMING09;                      /**< TSM_TIMING09, offset: 0x30C */
  __IO uint32_t TSM_TIMING10;                      /**< TSM_TIMING10, offset: 0x310 */
  __IO uint32_t TSM_TIMING11;                      /**< TSM_TIMING11, offset: 0x314 */
  __IO uint32_t TSM_TIMING12;                      /**< TSM_TIMING12, offset: 0x318 */
  __IO uint32_t TSM_TIMING13;                      /**< TSM_TIMING13, offset: 0x31C */
  __IO uint32_t TSM_TIMING14;                      /**< TSM_TIMING14, offset: 0x320 */
  __IO uint32_t TSM_TIMING15;                      /**< TSM_TIMING15, offset: 0x324 */
  __IO uint32_t TSM_TIMING16;                      /**< TSM_TIMING16, offset: 0x328 */
  __IO uint32_t TSM_TIMING17;                      /**< TSM_TIMING17, offset: 0x32C */
  __IO uint32_t TSM_TIMING18;                      /**< TSM_TIMING18, offset: 0x330 */
  __IO uint32_t TSM_TIMING19;                      /**< TSM_TIMING19, offset: 0x334 */
  __IO uint32_t TSM_TIMING20;                      /**< TSM_TIMING20, offset: 0x338 */
  __IO uint32_t TSM_TIMING21;                      /**< TSM_TIMING21, offset: 0x33C */
  __IO uint32_t TSM_TIMING22;                      /**< TSM_TIMING22, offset: 0x340 */
  __IO uint32_t TSM_TIMING23;                      /**< TSM_TIMING23, offset: 0x344 */
  __IO uint32_t TSM_TIMING24;                      /**< TSM_TIMING24, offset: 0x348 */
  __IO uint32_t TSM_TIMING25;                      /**< TSM_TIMING25, offset: 0x34C */
  __IO uint32_t TSM_TIMING26;                      /**< TSM_TIMING26, offset: 0x350 */
  __IO uint32_t TSM_TIMING27;                      /**< TSM_TIMING27, offset: 0x354 */
  __IO uint32_t TSM_TIMING28;                      /**< TSM_TIMING28, offset: 0x358 */
  __IO uint32_t TSM_TIMING29;                      /**< TSM_TIMING29, offset: 0x35C */
  __IO uint32_t TSM_TIMING30;                      /**< TSM_TIMING30, offset: 0x360 */
  __IO uint32_t TSM_TIMING31;                      /**< TSM_TIMING31, offset: 0x364 */
  __IO uint32_t TSM_TIMING32;                      /**< TSM_TIMING32, offset: 0x368 */
  __IO uint32_t TSM_TIMING33;                      /**< TSM_TIMING33, offset: 0x36C */
  __IO uint32_t TSM_TIMING34;                      /**< TSM_TIMING34, offset: 0x370 */
  __IO uint32_t TSM_TIMING35;                      /**< TSM_TIMING35, offset: 0x374 */
  __IO uint32_t TSM_TIMING36;                      /**< TSM_TIMING36, offset: 0x378 */
  __IO uint32_t TSM_TIMING37;                      /**< TSM_TIMING37, offset: 0x37C */
  __IO uint32_t TSM_TIMING38;                      /**< TSM_TIMING38, offset: 0x380 */
  __IO uint32_t TSM_TIMING39;                      /**< TSM_TIMING39, offset: 0x384 */
  __IO uint32_t TSM_TIMING40;                      /**< TSM_TIMING40, offset: 0x388 */
  __IO uint32_t TSM_TIMING41;                      /**< TSM_TIMING41, offset: 0x38C */
  __IO uint32_t TSM_TIMING42;                      /**< TSM_TIMING42, offset: 0x390 */
  __IO uint32_t TSM_TIMING43;                      /**< TSM_TIMING43, offset: 0x394 */
       uint8_t RESERVED_11[40];
  __IO uint32_t CORR_CTRL;                         /**< CORR_CTRL, offset: 0x3C0 */
  __IO uint32_t PN_TYPE;                           /**< PN_TYPE, offset: 0x3C4 */
  __IO uint32_t PN_CODE;                           /**< PN_CODE, offset: 0x3C8 */
  __IO uint32_t SYNC_CTRL;                         /**< Sync Control, offset: 0x3CC */
  __IO uint32_t SNF_THR;                           /**< SNF_THR, offset: 0x3D0 */
  __IO uint32_t FAD_THR;                           /**< FAD_THR, offset: 0x3D4 */
  __IO uint32_t ZBDEM_AFC;                         /**< ZBDEM_AFC, offset: 0x3D8 */
  __IO uint32_t LPPS_CTRL;                         /**< LPPS Control Register, offset: 0x3DC */
       uint8_t RESERVED_12[32];
  __IO uint32_t ADC_CTRL;                          /**< ADC Control, offset: 0x400 */
  __IO uint32_t ADC_TUNE;                          /**< ADC Tuning, offset: 0x404 */
  __IO uint32_t ADC_ADJ;                           /**< ADC Adjustment, offset: 0x408 */
  __IO uint32_t ADC_REGS;                          /**< ADC Regulators, offset: 0x40C */
  __IO uint32_t ADC_TRIMS;                         /**< ADC Regulator Trims, offset: 0x410 */
  __IO uint32_t ADC_TEST_CTRL;                     /**< ADC Test Control, offset: 0x414 */
       uint8_t RESERVED_13[8];
  __IO uint32_t BBF_CTRL;                          /**< Baseband Filter Control, offset: 0x420 */
       uint8_t RESERVED_14[8];
  __IO uint32_t RX_ANA_CTRL;                       /**< RX Analog Control, offset: 0x42C */
       uint8_t RESERVED_15[4];
  __IO uint32_t XTAL_CTRL;                         /**< Crystal Oscillator Control Register 1, offset: 0x434 */
  __IO uint32_t XTAL_CTRL2;                        /**< Crystal Oscillator Control Register 2, offset: 0x438 */
  __IO uint32_t BGAP_CTRL;                         /**< Bandgap Control, offset: 0x43C */
       uint8_t RESERVED_16[4];
  __IO uint32_t PLL_CTRL;                          /**< PLL Control Register, offset: 0x444 */
  __IO uint32_t PLL_CTRL2;                         /**< PLL Control Register 2, offset: 0x448 */
  __IO uint32_t PLL_TEST_CTRL;                     /**< PLL Test Control, offset: 0x44C */
       uint8_t RESERVED_17[8];
  __IO uint32_t QGEN_CTRL;                         /**< QGEN Control, offset: 0x458 */
       uint8_t RESERVED_18[8];
  __IO uint32_t TCA_CTRL;                          /**< TCA Control, offset: 0x464 */
  __IO uint32_t TZA_CTRL;                          /**< TZA Control, offset: 0x468 */
       uint8_t RESERVED_19[8];
  __IO uint32_t TX_ANA_CTRL;                       /**< TX Analog Control, offset: 0x474 */
       uint8_t RESERVED_20[4];
  __IO uint32_t ANA_SPARE;                         /**< Analog Spare, offset: 0x47C */
} XCVR_Type;

/* ----------------------------------------------------------------------------
   -- XCVR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XCVR_Register_Masks XCVR Register Masks
 * @{
 */

/*! @name RX_DIG_CTRL - RX Digital Control */
#define XCVR_RX_DIG_CTRL_RX_ADC_NEGEDGE_MASK     (0x1U)
#define XCVR_RX_DIG_CTRL_RX_ADC_NEGEDGE_SHIFT    (0U)
#define XCVR_RX_DIG_CTRL_RX_ADC_NEGEDGE(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_ADC_NEGEDGE_SHIFT)) & XCVR_RX_DIG_CTRL_RX_ADC_NEGEDGE_MASK)
#define XCVR_RX_DIG_CTRL_RX_CH_FILT_BYPASS_MASK  (0x2U)
#define XCVR_RX_DIG_CTRL_RX_CH_FILT_BYPASS_SHIFT (1U)
#define XCVR_RX_DIG_CTRL_RX_CH_FILT_BYPASS(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_CH_FILT_BYPASS_SHIFT)) & XCVR_RX_DIG_CTRL_RX_CH_FILT_BYPASS_MASK)
#define XCVR_RX_DIG_CTRL_RX_ADC_RAW_EN_MASK      (0x4U)
#define XCVR_RX_DIG_CTRL_RX_ADC_RAW_EN_SHIFT     (2U)
#define XCVR_RX_DIG_CTRL_RX_ADC_RAW_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_ADC_RAW_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_ADC_RAW_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_DEC_FILT_OSR_MASK    (0x70U)
#define XCVR_RX_DIG_CTRL_RX_DEC_FILT_OSR_SHIFT   (4U)
#define XCVR_RX_DIG_CTRL_RX_DEC_FILT_OSR(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_DEC_FILT_OSR_SHIFT)) & XCVR_RX_DIG_CTRL_RX_DEC_FILT_OSR_MASK)
#define XCVR_RX_DIG_CTRL_RX_INTERP_EN_MASK       (0x100U)
#define XCVR_RX_DIG_CTRL_RX_INTERP_EN_SHIFT      (8U)
#define XCVR_RX_DIG_CTRL_RX_INTERP_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_INTERP_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_INTERP_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_NORM_EN_MASK         (0x200U)
#define XCVR_RX_DIG_CTRL_RX_NORM_EN_SHIFT        (9U)
#define XCVR_RX_DIG_CTRL_RX_NORM_EN(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_NORM_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_NORM_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_RSSI_EN_MASK         (0x400U)
#define XCVR_RX_DIG_CTRL_RX_RSSI_EN_SHIFT        (10U)
#define XCVR_RX_DIG_CTRL_RX_RSSI_EN(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_RSSI_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_RSSI_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_AGC_EN_MASK          (0x800U)
#define XCVR_RX_DIG_CTRL_RX_AGC_EN_SHIFT         (11U)
#define XCVR_RX_DIG_CTRL_RX_AGC_EN(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_AGC_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_AGC_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_DCOC_EN_MASK         (0x1000U)
#define XCVR_RX_DIG_CTRL_RX_DCOC_EN_SHIFT        (12U)
#define XCVR_RX_DIG_CTRL_RX_DCOC_EN(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_DCOC_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_DCOC_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_DCOC_CAL_EN_MASK     (0x2000U)
#define XCVR_RX_DIG_CTRL_RX_DCOC_CAL_EN_SHIFT    (13U)
#define XCVR_RX_DIG_CTRL_RX_DCOC_CAL_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_DCOC_CAL_EN_SHIFT)) & XCVR_RX_DIG_CTRL_RX_DCOC_CAL_EN_MASK)
#define XCVR_RX_DIG_CTRL_RX_IQ_SWAP_MASK         (0x4000U)
#define XCVR_RX_DIG_CTRL_RX_IQ_SWAP_SHIFT        (14U)
#define XCVR_RX_DIG_CTRL_RX_IQ_SWAP(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_CTRL_RX_IQ_SWAP_SHIFT)) & XCVR_RX_DIG_CTRL_RX_IQ_SWAP_MASK)

/*! @name AGC_CTRL_0 - AGC Control 0 */
#define XCVR_AGC_CTRL_0_SLOW_AGC_EN_MASK         (0x1U)
#define XCVR_AGC_CTRL_0_SLOW_AGC_EN_SHIFT        (0U)
#define XCVR_AGC_CTRL_0_SLOW_AGC_EN(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_SLOW_AGC_EN_SHIFT)) & XCVR_AGC_CTRL_0_SLOW_AGC_EN_MASK)
#define XCVR_AGC_CTRL_0_SLOW_AGC_SRC_MASK        (0x6U)
#define XCVR_AGC_CTRL_0_SLOW_AGC_SRC_SHIFT       (1U)
#define XCVR_AGC_CTRL_0_SLOW_AGC_SRC(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_SLOW_AGC_SRC_SHIFT)) & XCVR_AGC_CTRL_0_SLOW_AGC_SRC_MASK)
#define XCVR_AGC_CTRL_0_AGC_FREEZE_EN_MASK       (0x8U)
#define XCVR_AGC_CTRL_0_AGC_FREEZE_EN_SHIFT      (3U)
#define XCVR_AGC_CTRL_0_AGC_FREEZE_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_FREEZE_EN_SHIFT)) & XCVR_AGC_CTRL_0_AGC_FREEZE_EN_MASK)
#define XCVR_AGC_CTRL_0_FREEZE_AGC_SRC_MASK      (0x30U)
#define XCVR_AGC_CTRL_0_FREEZE_AGC_SRC_SHIFT     (4U)
#define XCVR_AGC_CTRL_0_FREEZE_AGC_SRC(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_FREEZE_AGC_SRC_SHIFT)) & XCVR_AGC_CTRL_0_FREEZE_AGC_SRC_MASK)
#define XCVR_AGC_CTRL_0_AGC_UP_EN_MASK           (0x40U)
#define XCVR_AGC_CTRL_0_AGC_UP_EN_SHIFT          (6U)
#define XCVR_AGC_CTRL_0_AGC_UP_EN(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_UP_EN_SHIFT)) & XCVR_AGC_CTRL_0_AGC_UP_EN_MASK)
#define XCVR_AGC_CTRL_0_AGC_UP_SRC_MASK          (0x80U)
#define XCVR_AGC_CTRL_0_AGC_UP_SRC_SHIFT         (7U)
#define XCVR_AGC_CTRL_0_AGC_UP_SRC(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_UP_SRC_SHIFT)) & XCVR_AGC_CTRL_0_AGC_UP_SRC_MASK)
#define XCVR_AGC_CTRL_0_AGC_DOWN_BBF_STEP_SZ_MASK (0xF00U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_BBF_STEP_SZ_SHIFT (8U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_BBF_STEP_SZ(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_DOWN_BBF_STEP_SZ_SHIFT)) & XCVR_AGC_CTRL_0_AGC_DOWN_BBF_STEP_SZ_MASK)
#define XCVR_AGC_CTRL_0_AGC_DOWN_TZA_STEP_SZ_MASK (0xF000U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_TZA_STEP_SZ_SHIFT (12U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_TZA_STEP_SZ(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_DOWN_TZA_STEP_SZ_SHIFT)) & XCVR_AGC_CTRL_0_AGC_DOWN_TZA_STEP_SZ_MASK)
#define XCVR_AGC_CTRL_0_AGC_UP_RSSI_THRESH_MASK  (0xFF0000U)
#define XCVR_AGC_CTRL_0_AGC_UP_RSSI_THRESH_SHIFT (16U)
#define XCVR_AGC_CTRL_0_AGC_UP_RSSI_THRESH(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_UP_RSSI_THRESH_SHIFT)) & XCVR_AGC_CTRL_0_AGC_UP_RSSI_THRESH_MASK)
#define XCVR_AGC_CTRL_0_AGC_DOWN_RSSI_THRESH_MASK (0xFF000000U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_RSSI_THRESH_SHIFT (24U)
#define XCVR_AGC_CTRL_0_AGC_DOWN_RSSI_THRESH(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_0_AGC_DOWN_RSSI_THRESH_SHIFT)) & XCVR_AGC_CTRL_0_AGC_DOWN_RSSI_THRESH_MASK)

/*! @name AGC_CTRL_1 - AGC Control 1 */
#define XCVR_AGC_CTRL_1_BBF_ALT_CODE_MASK        (0xFU)
#define XCVR_AGC_CTRL_1_BBF_ALT_CODE_SHIFT       (0U)
#define XCVR_AGC_CTRL_1_BBF_ALT_CODE(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_BBF_ALT_CODE_SHIFT)) & XCVR_AGC_CTRL_1_BBF_ALT_CODE_MASK)
#define XCVR_AGC_CTRL_1_LNM_ALT_CODE_MASK        (0xFF0U)
#define XCVR_AGC_CTRL_1_LNM_ALT_CODE_SHIFT       (4U)
#define XCVR_AGC_CTRL_1_LNM_ALT_CODE(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_LNM_ALT_CODE_SHIFT)) & XCVR_AGC_CTRL_1_LNM_ALT_CODE_MASK)
#define XCVR_AGC_CTRL_1_LNM_USER_GAIN_MASK       (0xF000U)
#define XCVR_AGC_CTRL_1_LNM_USER_GAIN_SHIFT      (12U)
#define XCVR_AGC_CTRL_1_LNM_USER_GAIN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_LNM_USER_GAIN_SHIFT)) & XCVR_AGC_CTRL_1_LNM_USER_GAIN_MASK)
#define XCVR_AGC_CTRL_1_BBF_USER_GAIN_MASK       (0xF0000U)
#define XCVR_AGC_CTRL_1_BBF_USER_GAIN_SHIFT      (16U)
#define XCVR_AGC_CTRL_1_BBF_USER_GAIN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_BBF_USER_GAIN_SHIFT)) & XCVR_AGC_CTRL_1_BBF_USER_GAIN_MASK)
#define XCVR_AGC_CTRL_1_USER_LNM_GAIN_EN_MASK    (0x100000U)
#define XCVR_AGC_CTRL_1_USER_LNM_GAIN_EN_SHIFT   (20U)
#define XCVR_AGC_CTRL_1_USER_LNM_GAIN_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_USER_LNM_GAIN_EN_SHIFT)) & XCVR_AGC_CTRL_1_USER_LNM_GAIN_EN_MASK)
#define XCVR_AGC_CTRL_1_USER_BBF_GAIN_EN_MASK    (0x200000U)
#define XCVR_AGC_CTRL_1_USER_BBF_GAIN_EN_SHIFT   (21U)
#define XCVR_AGC_CTRL_1_USER_BBF_GAIN_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_USER_BBF_GAIN_EN_SHIFT)) & XCVR_AGC_CTRL_1_USER_BBF_GAIN_EN_MASK)
#define XCVR_AGC_CTRL_1_PRESLOW_EN_MASK          (0x400000U)
#define XCVR_AGC_CTRL_1_PRESLOW_EN_SHIFT         (22U)
#define XCVR_AGC_CTRL_1_PRESLOW_EN(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_PRESLOW_EN_SHIFT)) & XCVR_AGC_CTRL_1_PRESLOW_EN_MASK)
#define XCVR_AGC_CTRL_1_TZA_GAIN_SETTLE_TIME_MASK (0xFF000000U)
#define XCVR_AGC_CTRL_1_TZA_GAIN_SETTLE_TIME_SHIFT (24U)
#define XCVR_AGC_CTRL_1_TZA_GAIN_SETTLE_TIME(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_1_TZA_GAIN_SETTLE_TIME_SHIFT)) & XCVR_AGC_CTRL_1_TZA_GAIN_SETTLE_TIME_MASK)

/*! @name AGC_CTRL_2 - AGC Control 2 */
#define XCVR_AGC_CTRL_2_BBF_PDET_RST_MASK        (0x1U)
#define XCVR_AGC_CTRL_2_BBF_PDET_RST_SHIFT       (0U)
#define XCVR_AGC_CTRL_2_BBF_PDET_RST(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_BBF_PDET_RST_SHIFT)) & XCVR_AGC_CTRL_2_BBF_PDET_RST_MASK)
#define XCVR_AGC_CTRL_2_TZA_PDET_RST_MASK        (0x2U)
#define XCVR_AGC_CTRL_2_TZA_PDET_RST_SHIFT       (1U)
#define XCVR_AGC_CTRL_2_TZA_PDET_RST(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_TZA_PDET_RST_SHIFT)) & XCVR_AGC_CTRL_2_TZA_PDET_RST_MASK)
#define XCVR_AGC_CTRL_2_BBF_GAIN_SETTLE_TIME_MASK (0xFF0U)
#define XCVR_AGC_CTRL_2_BBF_GAIN_SETTLE_TIME_SHIFT (4U)
#define XCVR_AGC_CTRL_2_BBF_GAIN_SETTLE_TIME(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_BBF_GAIN_SETTLE_TIME_SHIFT)) & XCVR_AGC_CTRL_2_BBF_GAIN_SETTLE_TIME_MASK)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_LO_MASK  (0x7000U)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_LO_SHIFT (12U)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_BBF_PDET_THRESH_LO_SHIFT)) & XCVR_AGC_CTRL_2_BBF_PDET_THRESH_LO_MASK)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_HI_MASK  (0x38000U)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_HI_SHIFT (15U)
#define XCVR_AGC_CTRL_2_BBF_PDET_THRESH_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_BBF_PDET_THRESH_HI_SHIFT)) & XCVR_AGC_CTRL_2_BBF_PDET_THRESH_HI_MASK)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_LO_MASK  (0x1C0000U)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_LO_SHIFT (18U)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_TZA_PDET_THRESH_LO_SHIFT)) & XCVR_AGC_CTRL_2_TZA_PDET_THRESH_LO_MASK)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_HI_MASK  (0xE00000U)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_HI_SHIFT (21U)
#define XCVR_AGC_CTRL_2_TZA_PDET_THRESH_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_TZA_PDET_THRESH_HI_SHIFT)) & XCVR_AGC_CTRL_2_TZA_PDET_THRESH_HI_MASK)
#define XCVR_AGC_CTRL_2_AGC_FAST_EXPIRE_MASK     (0x3F000000U)
#define XCVR_AGC_CTRL_2_AGC_FAST_EXPIRE_SHIFT    (24U)
#define XCVR_AGC_CTRL_2_AGC_FAST_EXPIRE(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_2_AGC_FAST_EXPIRE_SHIFT)) & XCVR_AGC_CTRL_2_AGC_FAST_EXPIRE_MASK)

/*! @name AGC_CTRL_3 - AGC Control 3 */
#define XCVR_AGC_CTRL_3_AGC_UNFREEZE_TIME_MASK   (0x1FFFU)
#define XCVR_AGC_CTRL_3_AGC_UNFREEZE_TIME_SHIFT  (0U)
#define XCVR_AGC_CTRL_3_AGC_UNFREEZE_TIME(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_3_AGC_UNFREEZE_TIME_SHIFT)) & XCVR_AGC_CTRL_3_AGC_UNFREEZE_TIME_MASK)
#define XCVR_AGC_CTRL_3_AGC_PDET_LO_DLY_MASK     (0xE000U)
#define XCVR_AGC_CTRL_3_AGC_PDET_LO_DLY_SHIFT    (13U)
#define XCVR_AGC_CTRL_3_AGC_PDET_LO_DLY(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_3_AGC_PDET_LO_DLY_SHIFT)) & XCVR_AGC_CTRL_3_AGC_PDET_LO_DLY_MASK)
#define XCVR_AGC_CTRL_3_AGC_RSSI_DELT_H2S_MASK   (0x7F0000U)
#define XCVR_AGC_CTRL_3_AGC_RSSI_DELT_H2S_SHIFT  (16U)
#define XCVR_AGC_CTRL_3_AGC_RSSI_DELT_H2S(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_3_AGC_RSSI_DELT_H2S_SHIFT)) & XCVR_AGC_CTRL_3_AGC_RSSI_DELT_H2S_MASK)
#define XCVR_AGC_CTRL_3_AGC_H2S_STEP_SZ_MASK     (0xF800000U)
#define XCVR_AGC_CTRL_3_AGC_H2S_STEP_SZ_SHIFT    (23U)
#define XCVR_AGC_CTRL_3_AGC_H2S_STEP_SZ(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_3_AGC_H2S_STEP_SZ_SHIFT)) & XCVR_AGC_CTRL_3_AGC_H2S_STEP_SZ_MASK)
#define XCVR_AGC_CTRL_3_AGC_UP_STEP_SZ_MASK      (0xF0000000U)
#define XCVR_AGC_CTRL_3_AGC_UP_STEP_SZ_SHIFT     (28U)
#define XCVR_AGC_CTRL_3_AGC_UP_STEP_SZ(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_CTRL_3_AGC_UP_STEP_SZ_SHIFT)) & XCVR_AGC_CTRL_3_AGC_UP_STEP_SZ_MASK)

/*! @name AGC_STAT - AGC Status */
#define XCVR_AGC_STAT_BBF_PDET_LO_STAT_MASK      (0x1U)
#define XCVR_AGC_STAT_BBF_PDET_LO_STAT_SHIFT     (0U)
#define XCVR_AGC_STAT_BBF_PDET_LO_STAT(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_BBF_PDET_LO_STAT_SHIFT)) & XCVR_AGC_STAT_BBF_PDET_LO_STAT_MASK)
#define XCVR_AGC_STAT_BBF_PDET_HI_STAT_MASK      (0x2U)
#define XCVR_AGC_STAT_BBF_PDET_HI_STAT_SHIFT     (1U)
#define XCVR_AGC_STAT_BBF_PDET_HI_STAT(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_BBF_PDET_HI_STAT_SHIFT)) & XCVR_AGC_STAT_BBF_PDET_HI_STAT_MASK)
#define XCVR_AGC_STAT_TZA_PDET_LO_STAT_MASK      (0x4U)
#define XCVR_AGC_STAT_TZA_PDET_LO_STAT_SHIFT     (2U)
#define XCVR_AGC_STAT_TZA_PDET_LO_STAT(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_TZA_PDET_LO_STAT_SHIFT)) & XCVR_AGC_STAT_TZA_PDET_LO_STAT_MASK)
#define XCVR_AGC_STAT_TZA_PDET_HI_STAT_MASK      (0x8U)
#define XCVR_AGC_STAT_TZA_PDET_HI_STAT_SHIFT     (3U)
#define XCVR_AGC_STAT_TZA_PDET_HI_STAT(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_TZA_PDET_HI_STAT_SHIFT)) & XCVR_AGC_STAT_TZA_PDET_HI_STAT_MASK)
#define XCVR_AGC_STAT_CURR_AGC_IDX_MASK          (0x1F0U)
#define XCVR_AGC_STAT_CURR_AGC_IDX_SHIFT         (4U)
#define XCVR_AGC_STAT_CURR_AGC_IDX(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_CURR_AGC_IDX_SHIFT)) & XCVR_AGC_STAT_CURR_AGC_IDX_MASK)
#define XCVR_AGC_STAT_AGC_FROZEN_MASK            (0x200U)
#define XCVR_AGC_STAT_AGC_FROZEN_SHIFT           (9U)
#define XCVR_AGC_STAT_AGC_FROZEN(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_AGC_FROZEN_SHIFT)) & XCVR_AGC_STAT_AGC_FROZEN_MASK)
#define XCVR_AGC_STAT_RSSI_ADC_RAW_MASK          (0xFF0000U)
#define XCVR_AGC_STAT_RSSI_ADC_RAW_SHIFT         (16U)
#define XCVR_AGC_STAT_RSSI_ADC_RAW(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_STAT_RSSI_ADC_RAW_SHIFT)) & XCVR_AGC_STAT_RSSI_ADC_RAW_MASK)

/*! @name RSSI_CTRL_0 - RSSI Control 0 */
#define XCVR_RSSI_CTRL_0_RSSI_USE_VALS_MASK      (0x1U)
#define XCVR_RSSI_CTRL_0_RSSI_USE_VALS_SHIFT     (0U)
#define XCVR_RSSI_CTRL_0_RSSI_USE_VALS(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_USE_VALS_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_USE_VALS_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_SRC_MASK      (0x6U)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_SRC_SHIFT     (1U)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_SRC(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_HOLD_SRC_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_HOLD_SRC_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_EN_MASK       (0x8U)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_EN_SHIFT      (3U)
#define XCVR_RSSI_CTRL_0_RSSI_HOLD_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_HOLD_EN_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_HOLD_EN_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_DEC_EN_MASK        (0x10U)
#define XCVR_RSSI_CTRL_0_RSSI_DEC_EN_SHIFT       (4U)
#define XCVR_RSSI_CTRL_0_RSSI_DEC_EN(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_DEC_EN_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_DEC_EN_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_CW_WEIGHT_MASK (0x60U)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_CW_WEIGHT_SHIFT (5U)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_CW_WEIGHT(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_IIR_CW_WEIGHT_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_IIR_CW_WEIGHT_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_WEIGHT_MASK    (0xF0000U)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_WEIGHT_SHIFT   (16U)
#define XCVR_RSSI_CTRL_0_RSSI_IIR_WEIGHT(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_IIR_WEIGHT_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_IIR_WEIGHT_MASK)
#define XCVR_RSSI_CTRL_0_RSSI_ADJ_MASK           (0xFF000000U)
#define XCVR_RSSI_CTRL_0_RSSI_ADJ_SHIFT          (24U)
#define XCVR_RSSI_CTRL_0_RSSI_ADJ(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_0_RSSI_ADJ_SHIFT)) & XCVR_RSSI_CTRL_0_RSSI_ADJ_MASK)

/*! @name RSSI_CTRL_1 - RSSI Control 1 */
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_MASK    (0xFFU)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_SHIFT   (0U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_SHIFT)) & XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_MASK)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_MASK    (0xFF00U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_SHIFT   (8U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_SHIFT)) & XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_MASK)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_H_MASK  (0xF0000U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_H_SHIFT (16U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_H(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_H_SHIFT)) & XCVR_RSSI_CTRL_1_RSSI_ED_THRESH0_H_MASK)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_H_MASK  (0xF00000U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_H_SHIFT (20U)
#define XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_H(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_H_SHIFT)) & XCVR_RSSI_CTRL_1_RSSI_ED_THRESH1_H_MASK)
#define XCVR_RSSI_CTRL_1_RSSI_OUT_MASK           (0xFF000000U)
#define XCVR_RSSI_CTRL_1_RSSI_OUT_SHIFT          (24U)
#define XCVR_RSSI_CTRL_1_RSSI_OUT(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_RSSI_CTRL_1_RSSI_OUT_SHIFT)) & XCVR_RSSI_CTRL_1_RSSI_OUT_MASK)

/*! @name DCOC_CTRL_0 - DCOC Control 0 */
#define XCVR_DCOC_CTRL_0_DCOC_MAN_MASK           (0x2U)
#define XCVR_DCOC_CTRL_0_DCOC_MAN_SHIFT          (1U)
#define XCVR_DCOC_CTRL_0_DCOC_MAN(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_MAN_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_MAN_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_TRACK_EN_MASK      (0x8U)
#define XCVR_DCOC_CTRL_0_DCOC_TRACK_EN_SHIFT     (3U)
#define XCVR_DCOC_CTRL_0_DCOC_TRACK_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_TRACK_EN_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_TRACK_EN_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_CORRECT_EN_MASK    (0x10U)
#define XCVR_DCOC_CTRL_0_DCOC_CORRECT_EN_SHIFT   (4U)
#define XCVR_DCOC_CTRL_0_DCOC_CORRECT_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_CORRECT_EN_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_CORRECT_EN_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_SIGN_SCALE_IDX_MASK (0x60U)
#define XCVR_DCOC_CTRL_0_DCOC_SIGN_SCALE_IDX_SHIFT (5U)
#define XCVR_DCOC_CTRL_0_DCOC_SIGN_SCALE_IDX(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_SIGN_SCALE_IDX_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_SIGN_SCALE_IDX_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHAC_SCALE_IDX_MASK (0x300U)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHAC_SCALE_IDX_SHIFT (8U)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHAC_SCALE_IDX(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_ALPHAC_SCALE_IDX_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_ALPHAC_SCALE_IDX_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHA_RADIUS_IDX_MASK (0x7000U)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHA_RADIUS_IDX_SHIFT (12U)
#define XCVR_DCOC_CTRL_0_DCOC_ALPHA_RADIUS_IDX(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_ALPHA_RADIUS_IDX_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_ALPHA_RADIUS_IDX_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_CAL_DURATION_MASK  (0xF8000U)
#define XCVR_DCOC_CTRL_0_DCOC_CAL_DURATION_SHIFT (15U)
#define XCVR_DCOC_CTRL_0_DCOC_CAL_DURATION(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_CAL_DURATION_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_CAL_DURATION_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_DLY_MASK      (0x1F00000U)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_DLY_SHIFT     (20U)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_DLY(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_CORR_DLY_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_CORR_DLY_MASK)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_HOLD_TIME_MASK (0xFE000000U)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_HOLD_TIME_SHIFT (25U)
#define XCVR_DCOC_CTRL_0_DCOC_CORR_HOLD_TIME(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_0_DCOC_CORR_HOLD_TIME_SHIFT)) & XCVR_DCOC_CTRL_0_DCOC_CORR_HOLD_TIME_MASK)

/*! @name DCOC_CTRL_1 - DCOC Control 1 */
#define XCVR_DCOC_CTRL_1_BBF_DCOC_STEP_MASK      (0x1FFU)
#define XCVR_DCOC_CTRL_1_BBF_DCOC_STEP_SHIFT     (0U)
#define XCVR_DCOC_CTRL_1_BBF_DCOC_STEP(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_1_BBF_DCOC_STEP_SHIFT)) & XCVR_DCOC_CTRL_1_BBF_DCOC_STEP_MASK)
#define XCVR_DCOC_CTRL_1_TRACK_FROM_ZERO_MASK    (0x1000000U)
#define XCVR_DCOC_CTRL_1_TRACK_FROM_ZERO_SHIFT   (24U)
#define XCVR_DCOC_CTRL_1_TRACK_FROM_ZERO(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_1_TRACK_FROM_ZERO_SHIFT)) & XCVR_DCOC_CTRL_1_TRACK_FROM_ZERO_MASK)
#define XCVR_DCOC_CTRL_1_BBA_CORR_POL_MASK       (0x2000000U)
#define XCVR_DCOC_CTRL_1_BBA_CORR_POL_SHIFT      (25U)
#define XCVR_DCOC_CTRL_1_BBA_CORR_POL(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_1_BBA_CORR_POL_SHIFT)) & XCVR_DCOC_CTRL_1_BBA_CORR_POL_MASK)
#define XCVR_DCOC_CTRL_1_TZA_CORR_POL_MASK       (0x4000000U)
#define XCVR_DCOC_CTRL_1_TZA_CORR_POL_SHIFT      (26U)
#define XCVR_DCOC_CTRL_1_TZA_CORR_POL(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_1_TZA_CORR_POL_SHIFT)) & XCVR_DCOC_CTRL_1_TZA_CORR_POL_MASK)

/*! @name DCOC_CTRL_2 - DCOC Control 2 */
#define XCVR_DCOC_CTRL_2_BBF_DCOC_STEP_RECIP_MASK (0x1FFFU)
#define XCVR_DCOC_CTRL_2_BBF_DCOC_STEP_RECIP_SHIFT (0U)
#define XCVR_DCOC_CTRL_2_BBF_DCOC_STEP_RECIP(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_2_BBF_DCOC_STEP_RECIP_SHIFT)) & XCVR_DCOC_CTRL_2_BBF_DCOC_STEP_RECIP_MASK)

/*! @name DCOC_CTRL_3 - DCOC Control 3 */
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_I_MASK    (0x3FU)
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_I_SHIFT   (0U)
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_I(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_I_SHIFT)) & XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_I_MASK)
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_Q_MASK    (0x3F00U)
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_Q_SHIFT   (8U)
#define XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_Q(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_Q_SHIFT)) & XCVR_DCOC_CTRL_3_BBF_DCOC_INIT_Q_MASK)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_I_MASK    (0xFF0000U)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_I_SHIFT   (16U)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_I(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_I_SHIFT)) & XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_I_MASK)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_Q_MASK    (0xFF000000U)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_Q_SHIFT   (24U)
#define XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_Q(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_Q_SHIFT)) & XCVR_DCOC_CTRL_3_TZA_DCOC_INIT_Q_MASK)

/*! @name DCOC_CTRL_4 - DCOC Control 4 */
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_I_MASK    (0xFFFU)
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_I_SHIFT   (0U)
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_I(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_I_SHIFT)) & XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_I_MASK)
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_Q_MASK    (0xFFF0000U)
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_Q_SHIFT   (16U)
#define XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_Q(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_Q_SHIFT)) & XCVR_DCOC_CTRL_4_DIG_DCOC_INIT_Q_MASK)

/*! @name DCOC_CAL_GAIN - DCOC Calibration Gain */
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN1_MASK (0xF00U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN1_SHIFT (8U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN1_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN1_MASK)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN1_MASK (0xF000U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN1_SHIFT (12U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN1_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN1_MASK)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN2_MASK (0xF0000U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN2_SHIFT (16U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN2_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN2_MASK)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN2_MASK (0xF00000U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN2_SHIFT (20U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN2_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN2_MASK)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN3_MASK (0xF000000U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN3_SHIFT (24U)
#define XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN3(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN3_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_BBF_CAL_GAIN3_MASK)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN3_MASK (0xF0000000U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN3_SHIFT (28U)
#define XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN3(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN3_SHIFT)) & XCVR_DCOC_CAL_GAIN_DCOC_TZA_CAL_GAIN3_MASK)

/*! @name DCOC_STAT - DCOC Status */
#define XCVR_DCOC_STAT_BBF_DCOC_I_MASK           (0x3FU)
#define XCVR_DCOC_STAT_BBF_DCOC_I_SHIFT          (0U)
#define XCVR_DCOC_STAT_BBF_DCOC_I(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_STAT_BBF_DCOC_I_SHIFT)) & XCVR_DCOC_STAT_BBF_DCOC_I_MASK)
#define XCVR_DCOC_STAT_BBF_DCOC_Q_MASK           (0x3F00U)
#define XCVR_DCOC_STAT_BBF_DCOC_Q_SHIFT          (8U)
#define XCVR_DCOC_STAT_BBF_DCOC_Q(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_STAT_BBF_DCOC_Q_SHIFT)) & XCVR_DCOC_STAT_BBF_DCOC_Q_MASK)
#define XCVR_DCOC_STAT_TZA_DCOC_I_MASK           (0xFF0000U)
#define XCVR_DCOC_STAT_TZA_DCOC_I_SHIFT          (16U)
#define XCVR_DCOC_STAT_TZA_DCOC_I(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_STAT_TZA_DCOC_I_SHIFT)) & XCVR_DCOC_STAT_TZA_DCOC_I_MASK)
#define XCVR_DCOC_STAT_TZA_DCOC_Q_MASK           (0xFF000000U)
#define XCVR_DCOC_STAT_TZA_DCOC_Q_SHIFT          (24U)
#define XCVR_DCOC_STAT_TZA_DCOC_Q(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_STAT_TZA_DCOC_Q_SHIFT)) & XCVR_DCOC_STAT_TZA_DCOC_Q_MASK)

/*! @name DCOC_DC_EST - DCOC DC Estimate */
#define XCVR_DCOC_DC_EST_DC_EST_I_MASK           (0xFFFU)
#define XCVR_DCOC_DC_EST_DC_EST_I_SHIFT          (0U)
#define XCVR_DCOC_DC_EST_DC_EST_I(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_DC_EST_DC_EST_I_SHIFT)) & XCVR_DCOC_DC_EST_DC_EST_I_MASK)
#define XCVR_DCOC_DC_EST_DC_EST_Q_MASK           (0xFFF0000U)
#define XCVR_DCOC_DC_EST_DC_EST_Q_SHIFT          (16U)
#define XCVR_DCOC_DC_EST_DC_EST_Q(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_DC_EST_DC_EST_Q_SHIFT)) & XCVR_DCOC_DC_EST_DC_EST_Q_MASK)

/*! @name DCOC_CAL_RCP - DCOC Calibration Reciprocals */
#define XCVR_DCOC_CAL_RCP_DCOC_TMP_CALC_RECIP_MASK (0x3FFU)
#define XCVR_DCOC_CAL_RCP_DCOC_TMP_CALC_RECIP_SHIFT (0U)
#define XCVR_DCOC_CAL_RCP_DCOC_TMP_CALC_RECIP(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_RCP_DCOC_TMP_CALC_RECIP_SHIFT)) & XCVR_DCOC_CAL_RCP_DCOC_TMP_CALC_RECIP_MASK)
#define XCVR_DCOC_CAL_RCP_ALPHA_CALC_RECIP_MASK  (0x1FFC00U)
#define XCVR_DCOC_CAL_RCP_ALPHA_CALC_RECIP_SHIFT (10U)
#define XCVR_DCOC_CAL_RCP_ALPHA_CALC_RECIP(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_RCP_ALPHA_CALC_RECIP_SHIFT)) & XCVR_DCOC_CAL_RCP_ALPHA_CALC_RECIP_MASK)

/*! @name IQMC_CTRL - IQMC Control */
#define XCVR_IQMC_CTRL_IQMC_CAL_EN_MASK          (0x1U)
#define XCVR_IQMC_CTRL_IQMC_CAL_EN_SHIFT         (0U)
#define XCVR_IQMC_CTRL_IQMC_CAL_EN(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_IQMC_CTRL_IQMC_CAL_EN_SHIFT)) & XCVR_IQMC_CTRL_IQMC_CAL_EN_MASK)
#define XCVR_IQMC_CTRL_IQMC_NUM_ITER_MASK        (0xFF00U)
#define XCVR_IQMC_CTRL_IQMC_NUM_ITER_SHIFT       (8U)
#define XCVR_IQMC_CTRL_IQMC_NUM_ITER(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_IQMC_CTRL_IQMC_NUM_ITER_SHIFT)) & XCVR_IQMC_CTRL_IQMC_NUM_ITER_MASK)

/*! @name IQMC_CAL - IQMC Calibration */
#define XCVR_IQMC_CAL_IQMC_GAIN_ADJ_MASK         (0x7FFU)
#define XCVR_IQMC_CAL_IQMC_GAIN_ADJ_SHIFT        (0U)
#define XCVR_IQMC_CAL_IQMC_GAIN_ADJ(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_IQMC_CAL_IQMC_GAIN_ADJ_SHIFT)) & XCVR_IQMC_CAL_IQMC_GAIN_ADJ_MASK)
#define XCVR_IQMC_CAL_IQMC_PHASE_ADJ_MASK        (0xFFF0000U)
#define XCVR_IQMC_CAL_IQMC_PHASE_ADJ_SHIFT       (16U)
#define XCVR_IQMC_CAL_IQMC_PHASE_ADJ(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_IQMC_CAL_IQMC_PHASE_ADJ_SHIFT)) & XCVR_IQMC_CAL_IQMC_PHASE_ADJ_MASK)

/*! @name TCA_AGC_VAL_3_0 - TCA AGC Step Values 3..0 */
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_0_MASK  (0xFFU)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_0_SHIFT (0U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_0(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_0_SHIFT)) & XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_0_MASK)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_1_MASK  (0xFF00U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_1_SHIFT (8U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_1(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_1_SHIFT)) & XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_1_MASK)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_2_MASK  (0xFF0000U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_2_SHIFT (16U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_2(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_2_SHIFT)) & XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_2_MASK)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_3_MASK  (0xFF000000U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_3_SHIFT (24U)
#define XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_3(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_3_SHIFT)) & XCVR_TCA_AGC_VAL_3_0_TCA_AGC_VAL_3_MASK)

/*! @name TCA_AGC_VAL_7_4 - TCA AGC Step Values 7..4 */
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_4_MASK  (0xFFU)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_4_SHIFT (0U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_4(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_4_SHIFT)) & XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_4_MASK)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_5_MASK  (0xFF00U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_5_SHIFT (8U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_5(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_5_SHIFT)) & XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_5_MASK)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_6_MASK  (0xFF0000U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_6_SHIFT (16U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_6(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_6_SHIFT)) & XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_6_MASK)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_7_MASK  (0xFF000000U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_7_SHIFT (24U)
#define XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_7(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_7_SHIFT)) & XCVR_TCA_AGC_VAL_7_4_TCA_AGC_VAL_7_MASK)

/*! @name TCA_AGC_VAL_8 - TCA AGC Step Values 8 */
#define XCVR_TCA_AGC_VAL_8_TCA_AGC_VAL_8_MASK    (0xFFU)
#define XCVR_TCA_AGC_VAL_8_TCA_AGC_VAL_8_SHIFT   (0U)
#define XCVR_TCA_AGC_VAL_8_TCA_AGC_VAL_8(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_VAL_8_TCA_AGC_VAL_8_SHIFT)) & XCVR_TCA_AGC_VAL_8_TCA_AGC_VAL_8_MASK)

/*! @name BBF_RES_TUNE_VAL_7_0 - BBF Resistor Tune Values 7..0 */
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_0_MASK (0xFU)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_0_SHIFT (0U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_0(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_0_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_0_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_1_MASK (0xF0U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_1_SHIFT (4U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_1_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_1_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_2_MASK (0xF00U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_2_SHIFT (8U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_2_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_2_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_3_MASK (0xF000U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_3_SHIFT (12U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_3(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_3_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_3_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_4_MASK (0xF0000U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_4_SHIFT (16U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_4(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_4_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_4_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_5_MASK (0xF00000U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_5_SHIFT (20U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_5(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_5_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_5_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_6_MASK (0xF000000U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_6_SHIFT (24U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_6(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_6_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_6_MASK)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_7_MASK (0xF0000000U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_7_SHIFT (28U)
#define XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_7(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_7_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_7_0_BBF_RES_TUNE_VAL_7_MASK)

/*! @name BBF_RES_TUNE_VAL_10_8 - BBF Resistor Tune Values 10..8 */
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_8_MASK (0xFU)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_8_SHIFT (0U)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_8(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_8_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_8_MASK)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_9_MASK (0xF0U)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_9_SHIFT (4U)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_9(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_9_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_9_MASK)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_10_MASK (0xF00U)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_10_SHIFT (8U)
#define XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_10(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_10_SHIFT)) & XCVR_BBF_RES_TUNE_VAL_10_8_BBF_RES_TUNE_VAL_10_MASK)

/*! @name TCA_AGC_LIN_VAL_2_0 - TCA AGC Linear Gain Values 2..0 */
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_0_MASK (0x3FFU)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_0_SHIFT (0U)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_0(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_0_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_0_MASK)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_1_MASK (0xFFC00U)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_1_SHIFT (10U)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_1_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_1_MASK)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_2_MASK (0x3FF00000U)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_2_SHIFT (20U)
#define XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_2_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_2_0_TCA_AGC_LIN_VAL_2_MASK)

/*! @name TCA_AGC_LIN_VAL_5_3 - TCA AGC Linear Gain Values 5..3 */
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_3_MASK (0x3FFU)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_3_SHIFT (0U)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_3(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_3_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_3_MASK)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_4_MASK (0xFFC00U)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_4_SHIFT (10U)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_4(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_4_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_4_MASK)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_5_MASK (0x3FF00000U)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_5_SHIFT (20U)
#define XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_5(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_5_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_5_3_TCA_AGC_LIN_VAL_5_MASK)

/*! @name TCA_AGC_LIN_VAL_8_6 - TCA AGC Linear Gain Values 8..6 */
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_6_MASK (0x3FFU)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_6_SHIFT (0U)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_6(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_6_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_6_MASK)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_7_MASK (0xFFC00U)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_7_SHIFT (10U)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_7(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_7_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_7_MASK)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_8_MASK (0x3FF00000U)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_8_SHIFT (20U)
#define XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_8(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_8_SHIFT)) & XCVR_TCA_AGC_LIN_VAL_8_6_TCA_AGC_LIN_VAL_8_MASK)

/*! @name BBF_RES_TUNE_LIN_VAL_3_0 - BBF Resistor Tune Values 3..0 */
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_0_MASK (0xFFU)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_0_SHIFT (0U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_0(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_0_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_0_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_1_MASK (0xFF00U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_1_SHIFT (8U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_1_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_1_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_2_MASK (0xFF0000U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_2_SHIFT (16U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_2_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_2_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_3_MASK (0xFF000000U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_3_SHIFT (24U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_3(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_3_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_3_0_BBF_RES_TUNE_LIN_VAL_3_MASK)

/*! @name BBF_RES_TUNE_LIN_VAL_7_4 - BBF Resistor Tune Values 7..4 */
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_4_MASK (0xFFU)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_4_SHIFT (0U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_4(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_4_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_4_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_5_MASK (0xFF00U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_5_SHIFT (8U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_5(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_5_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_5_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_6_MASK (0xFF0000U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_6_SHIFT (16U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_6(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_6_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_6_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_7_MASK (0xFF000000U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_7_SHIFT (24U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_7(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_7_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_7_4_BBF_RES_TUNE_LIN_VAL_7_MASK)

/*! @name BBF_RES_TUNE_LIN_VAL_10_8 - BBF Resistor Tune Values 10..8 */
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_8_MASK (0xFFU)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_8_SHIFT (0U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_8(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_8_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_8_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_9_MASK (0xFF00U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_9_SHIFT (8U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_9(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_9_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_9_MASK)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_10_MASK (0xFF0000U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_10_SHIFT (16U)
#define XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_10(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_10_SHIFT)) & XCVR_BBF_RES_TUNE_LIN_VAL_10_8_BBF_RES_TUNE_LIN_VAL_10_MASK)

/*! @name AGC_GAIN_TBL_03_00 - AGC Gain Tables Step 03..00 */
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_00_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_00_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_00(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_00_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_00_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_00_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_00_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_00(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_00_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_00_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_01_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_01_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_01(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_01_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_01_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_01_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_01_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_01(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_01_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_01_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_02_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_02_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_02(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_02_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_02_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_02_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_02_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_02(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_02_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_02_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_03_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_03_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_03(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_03_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_BBF_GAIN_03_MASK)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_03_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_03_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_03(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_03_SHIFT)) & XCVR_AGC_GAIN_TBL_03_00_LNM_GAIN_03_MASK)

/*! @name AGC_GAIN_TBL_07_04 - AGC Gain Tables Step 07..04 */
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_04_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_04_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_04(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_04_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_04_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_04_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_04_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_04(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_04_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_04_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_05_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_05_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_05(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_05_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_05_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_05_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_05_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_05(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_05_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_05_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_06_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_06_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_06(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_06_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_06_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_06_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_06_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_06(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_06_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_06_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_07_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_07_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_07(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_07_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_BBF_GAIN_07_MASK)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_07_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_07_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_07(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_07_SHIFT)) & XCVR_AGC_GAIN_TBL_07_04_LNM_GAIN_07_MASK)

/*! @name AGC_GAIN_TBL_11_08 - AGC Gain Tables Step 11..08 */
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_08_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_08_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_08(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_08_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_08_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_08_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_08_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_08(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_08_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_08_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_09_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_09_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_09(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_09_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_09_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_09_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_09_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_09(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_09_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_09_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_10_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_10_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_10(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_10_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_10_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_10_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_10_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_10(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_10_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_10_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_11_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_11_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_11(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_11_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_BBF_GAIN_11_MASK)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_11_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_11_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_11(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_11_SHIFT)) & XCVR_AGC_GAIN_TBL_11_08_LNM_GAIN_11_MASK)

/*! @name AGC_GAIN_TBL_15_12 - AGC Gain Tables Step 15..12 */
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_12_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_12_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_12(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_12_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_12_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_12_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_12_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_12(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_12_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_12_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_13_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_13_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_13(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_13_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_13_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_13_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_13_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_13(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_13_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_13_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_14_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_14_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_14(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_14_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_14_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_14_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_14_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_14(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_14_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_14_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_15_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_15_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_15(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_15_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_BBF_GAIN_15_MASK)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_15_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_15_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_15(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_15_SHIFT)) & XCVR_AGC_GAIN_TBL_15_12_LNM_GAIN_15_MASK)

/*! @name AGC_GAIN_TBL_19_16 - AGC Gain Tables Step 19..16 */
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_16_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_16_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_16(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_16_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_16_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_16_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_16_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_16(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_16_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_16_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_17_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_17_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_17(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_17_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_17_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_17_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_17_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_17(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_17_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_17_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_18_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_18_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_18(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_18_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_18_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_18_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_18_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_18(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_18_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_18_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_19_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_19_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_19(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_19_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_BBF_GAIN_19_MASK)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_19_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_19_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_19(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_19_SHIFT)) & XCVR_AGC_GAIN_TBL_19_16_LNM_GAIN_19_MASK)

/*! @name AGC_GAIN_TBL_23_20 - AGC Gain Tables Step 23..20 */
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_20_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_20_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_20(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_20_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_20_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_20_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_20_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_20(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_20_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_20_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_21_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_21_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_21(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_21_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_21_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_21_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_21_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_21(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_21_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_21_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_22_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_22_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_22(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_22_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_22_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_22_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_22_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_22(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_22_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_22_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_23_MASK (0xF000000U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_23_SHIFT (24U)
#define XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_23(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_23_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_BBF_GAIN_23_MASK)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_23_MASK (0xF0000000U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_23_SHIFT (28U)
#define XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_23(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_23_SHIFT)) & XCVR_AGC_GAIN_TBL_23_20_LNM_GAIN_23_MASK)

/*! @name AGC_GAIN_TBL_26_24 - AGC Gain Tables Step 26..24 */
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_24_MASK (0xFU)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_24_SHIFT (0U)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_24(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_24_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_24_MASK)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_24_MASK (0xF0U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_24_SHIFT (4U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_24(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_24_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_24_MASK)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_25_MASK (0xF00U)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_25_SHIFT (8U)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_25(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_25_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_25_MASK)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_25_MASK (0xF000U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_25_SHIFT (12U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_25(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_25_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_25_MASK)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_26_MASK (0xF0000U)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_26_SHIFT (16U)
#define XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_26(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_26_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_BBF_GAIN_26_MASK)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_26_MASK (0xF00000U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_26_SHIFT (20U)
#define XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_26(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_26_SHIFT)) & XCVR_AGC_GAIN_TBL_26_24_LNM_GAIN_26_MASK)

/*! @name DCOC_OFFSET_ - DCOC Offset */
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_I_MASK (0x3FU)
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_I_SHIFT (0U)
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_I(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_I_SHIFT)) & XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_I_MASK)
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_Q_MASK (0x3F00U)
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_Q_SHIFT (8U)
#define XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_Q(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_Q_SHIFT)) & XCVR_DCOC_OFFSET__DCOC_BBF_OFFSET_Q_MASK)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_I_MASK (0xFF0000U)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_I_SHIFT (16U)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_I(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_I_SHIFT)) & XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_I_MASK)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_Q_MASK (0xFF000000U)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_Q_SHIFT (24U)
#define XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_Q(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_Q_SHIFT)) & XCVR_DCOC_OFFSET__DCOC_TZA_OFFSET_Q_MASK)

/* The count of XCVR_DCOC_OFFSET_ */
#define XCVR_DCOC_OFFSET__COUNT                  (27U)

/*! @name DCOC_TZA_STEP_ - DCOC TZA DC step */
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_RCP_MASK (0x1FFFU)
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_RCP_SHIFT (0U)
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_RCP(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_RCP_SHIFT)) & XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_RCP_MASK)
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_GAIN_MASK (0xFFF0000U)
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_GAIN_SHIFT (16U)
#define XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_GAIN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_GAIN_SHIFT)) & XCVR_DCOC_TZA_STEP__DCOC_TZA_STEP_GAIN_MASK)

/* The count of XCVR_DCOC_TZA_STEP_ */
#define XCVR_DCOC_TZA_STEP__COUNT                (11U)

/*! @name DCOC_CAL_ALPHA - DCOC Calibration Alpha */
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_I_MASK (0xFFFFU)
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_I_SHIFT (0U)
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_I(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_I_SHIFT)) & XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_I_MASK)
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_Q_MASK (0xFFFF0000U)
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_Q_SHIFT (16U)
#define XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_Q(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_Q_SHIFT)) & XCVR_DCOC_CAL_ALPHA_DCOC_CAL_ALPHA_Q_MASK)

/*! @name DCOC_CAL_BETA - DCOC Calibration Beta */
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_I_MASK  (0xFFFFU)
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_I_SHIFT (0U)
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_I(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_I_SHIFT)) & XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_I_MASK)
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_Q_MASK  (0xFFFF0000U)
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_Q_SHIFT (16U)
#define XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_Q(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_Q_SHIFT)) & XCVR_DCOC_CAL_BETA_DCOC_CAL_BETA_Q_MASK)

/*! @name DCOC_CAL_GAMMA - DCOC Calibration Gamma */
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_I_MASK (0xFFFFU)
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_I_SHIFT (0U)
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_I(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_I_SHIFT)) & XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_I_MASK)
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_Q_MASK (0xFFFF0000U)
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_Q_SHIFT (16U)
#define XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_Q(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_Q_SHIFT)) & XCVR_DCOC_CAL_GAMMA_DCOC_CAL_GAMMA_Q_MASK)

/*! @name DCOC_CAL_IIR - DCOC Calibration IIR */
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR1A_IDX_MASK (0x3U)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR1A_IDX_SHIFT (0U)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR1A_IDX(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR1A_IDX_SHIFT)) & XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR1A_IDX_MASK)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR2A_IDX_MASK (0xCU)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR2A_IDX_SHIFT (2U)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR2A_IDX(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR2A_IDX_SHIFT)) & XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR2A_IDX_MASK)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR3A_IDX_MASK (0x30U)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR3A_IDX_SHIFT (4U)
#define XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR3A_IDX(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR3A_IDX_SHIFT)) & XCVR_DCOC_CAL_IIR_DCOC_CAL_IIR3A_IDX_MASK)

/*! @name DCOC_CAL - DCOC Calibration Result */
#define XCVR_DCOC_CAL_DCOC_CAL_RES_I_MASK        (0xFFFU)
#define XCVR_DCOC_CAL_DCOC_CAL_RES_I_SHIFT       (0U)
#define XCVR_DCOC_CAL_DCOC_CAL_RES_I(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_DCOC_CAL_RES_I_SHIFT)) & XCVR_DCOC_CAL_DCOC_CAL_RES_I_MASK)
#define XCVR_DCOC_CAL_DCOC_CAL_RES_Q_MASK        (0xFFF0000U)
#define XCVR_DCOC_CAL_DCOC_CAL_RES_Q_SHIFT       (16U)
#define XCVR_DCOC_CAL_DCOC_CAL_RES_Q(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_DCOC_CAL_DCOC_CAL_RES_Q_SHIFT)) & XCVR_DCOC_CAL_DCOC_CAL_RES_Q_MASK)

/* The count of XCVR_DCOC_CAL */
#define XCVR_DCOC_CAL_COUNT                      (3U)

/*! @name RX_CHF_COEF - Receive Channel Filter Coefficient */
#define XCVR_RX_CHF_COEF_RX_CH_FILT_HX_MASK      (0xFFU)
#define XCVR_RX_CHF_COEF_RX_CH_FILT_HX_SHIFT     (0U)
#define XCVR_RX_CHF_COEF_RX_CH_FILT_HX(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_RX_CHF_COEF_RX_CH_FILT_HX_SHIFT)) & XCVR_RX_CHF_COEF_RX_CH_FILT_HX_MASK)

/* The count of XCVR_RX_CHF_COEF */
#define XCVR_RX_CHF_COEF_COUNT                   (8U)

/*! @name TX_DIG_CTRL - TX Digital Control */
#define XCVR_TX_DIG_CTRL_DFT_MODE_MASK           (0x7U)
#define XCVR_TX_DIG_CTRL_DFT_MODE_SHIFT          (0U)
#define XCVR_TX_DIG_CTRL_DFT_MODE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_DFT_MODE_SHIFT)) & XCVR_TX_DIG_CTRL_DFT_MODE_MASK)
#define XCVR_TX_DIG_CTRL_DFT_EN_MASK             (0x8U)
#define XCVR_TX_DIG_CTRL_DFT_EN_SHIFT            (3U)
#define XCVR_TX_DIG_CTRL_DFT_EN(x)               (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_DFT_EN_SHIFT)) & XCVR_TX_DIG_CTRL_DFT_EN_MASK)
#define XCVR_TX_DIG_CTRL_DFT_LFSR_LEN_MASK       (0x70U)
#define XCVR_TX_DIG_CTRL_DFT_LFSR_LEN_SHIFT      (4U)
#define XCVR_TX_DIG_CTRL_DFT_LFSR_LEN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_DFT_LFSR_LEN_SHIFT)) & XCVR_TX_DIG_CTRL_DFT_LFSR_LEN_MASK)
#define XCVR_TX_DIG_CTRL_LFSR_EN_MASK            (0x80U)
#define XCVR_TX_DIG_CTRL_LFSR_EN_SHIFT           (7U)
#define XCVR_TX_DIG_CTRL_LFSR_EN(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_LFSR_EN_SHIFT)) & XCVR_TX_DIG_CTRL_LFSR_EN_MASK)
#define XCVR_TX_DIG_CTRL_DFT_CLK_SEL_MASK        (0x700U)
#define XCVR_TX_DIG_CTRL_DFT_CLK_SEL_SHIFT       (8U)
#define XCVR_TX_DIG_CTRL_DFT_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_DFT_CLK_SEL_SHIFT)) & XCVR_TX_DIG_CTRL_DFT_CLK_SEL_MASK)
#define XCVR_TX_DIG_CTRL_TONE_SEL_MASK           (0x3000U)
#define XCVR_TX_DIG_CTRL_TONE_SEL_SHIFT          (12U)
#define XCVR_TX_DIG_CTRL_TONE_SEL(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_TONE_SEL_SHIFT)) & XCVR_TX_DIG_CTRL_TONE_SEL_MASK)
#define XCVR_TX_DIG_CTRL_POL_MASK                (0x10000U)
#define XCVR_TX_DIG_CTRL_POL_SHIFT               (16U)
#define XCVR_TX_DIG_CTRL_POL(x)                  (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_POL_SHIFT)) & XCVR_TX_DIG_CTRL_POL_MASK)
#define XCVR_TX_DIG_CTRL_DP_SEL_MASK             (0x100000U)
#define XCVR_TX_DIG_CTRL_DP_SEL_SHIFT            (20U)
#define XCVR_TX_DIG_CTRL_DP_SEL(x)               (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_DP_SEL_SHIFT)) & XCVR_TX_DIG_CTRL_DP_SEL_MASK)
#define XCVR_TX_DIG_CTRL_FREQ_WORD_ADJ_MASK      (0xFFC00000U)
#define XCVR_TX_DIG_CTRL_FREQ_WORD_ADJ_SHIFT     (22U)
#define XCVR_TX_DIG_CTRL_FREQ_WORD_ADJ(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DIG_CTRL_FREQ_WORD_ADJ_SHIFT)) & XCVR_TX_DIG_CTRL_FREQ_WORD_ADJ_MASK)

/*! @name TX_DATA_PAD_PAT - TX Data Padding Pattern */
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_0_MASK (0xFFU)
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_0_SHIFT (0U)
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_0(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_0_SHIFT)) & XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_0_MASK)
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_1_MASK (0xFF00U)
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_1_SHIFT (8U)
#define XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_1_SHIFT)) & XCVR_TX_DATA_PAD_PAT_DATA_PADDING_PAT_1_MASK)
#define XCVR_TX_DATA_PAD_PAT_DFT_LFSR_OUT_MASK   (0x7FFF0000U)
#define XCVR_TX_DATA_PAD_PAT_DFT_LFSR_OUT_SHIFT  (16U)
#define XCVR_TX_DATA_PAD_PAT_DFT_LFSR_OUT(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DATA_PAD_PAT_DFT_LFSR_OUT_SHIFT)) & XCVR_TX_DATA_PAD_PAT_DFT_LFSR_OUT_MASK)
#define XCVR_TX_DATA_PAD_PAT_LRM_MASK            (0x80000000U)
#define XCVR_TX_DATA_PAD_PAT_LRM_SHIFT           (31U)
#define XCVR_TX_DATA_PAD_PAT_LRM(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DATA_PAD_PAT_LRM_SHIFT)) & XCVR_TX_DATA_PAD_PAT_LRM_MASK)

/*! @name TX_GFSK_MOD_CTRL - TX GFSK Modulation Control */
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MULTIPLY_TABLE_MANUAL_MASK (0xFFFFU)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MULTIPLY_TABLE_MANUAL_SHIFT (0U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MULTIPLY_TABLE_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_MOD_CTRL_GFSK_MULTIPLY_TABLE_MANUAL_SHIFT)) & XCVR_TX_GFSK_MOD_CTRL_GFSK_MULTIPLY_TABLE_MANUAL_MASK)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MI_MASK       (0x30000U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MI_SHIFT      (16U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MI(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_MOD_CTRL_GFSK_MI_SHIFT)) & XCVR_TX_GFSK_MOD_CTRL_GFSK_MI_MASK)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MLD_MASK      (0x100000U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MLD_SHIFT     (20U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_MLD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_MOD_CTRL_GFSK_MLD_SHIFT)) & XCVR_TX_GFSK_MOD_CTRL_GFSK_MLD_MASK)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_SYMBOL_RATE_MASK (0x7000000U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_SYMBOL_RATE_SHIFT (24U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_SYMBOL_RATE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_MOD_CTRL_GFSK_SYMBOL_RATE_SHIFT)) & XCVR_TX_GFSK_MOD_CTRL_GFSK_SYMBOL_RATE_MASK)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_FLD_MASK      (0x10000000U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_FLD_SHIFT     (28U)
#define XCVR_TX_GFSK_MOD_CTRL_GFSK_FLD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_MOD_CTRL_GFSK_FLD_SHIFT)) & XCVR_TX_GFSK_MOD_CTRL_GFSK_FLD_MASK)

/*! @name TX_GFSK_COEFF2 - TX GFSK Filter Coefficients 2 */
#define XCVR_TX_GFSK_COEFF2_GFSK_FILTER_COEFF_MANUAL2_MASK (0xFFFFFFFFU)
#define XCVR_TX_GFSK_COEFF2_GFSK_FILTER_COEFF_MANUAL2_SHIFT (0U)
#define XCVR_TX_GFSK_COEFF2_GFSK_FILTER_COEFF_MANUAL2(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_COEFF2_GFSK_FILTER_COEFF_MANUAL2_SHIFT)) & XCVR_TX_GFSK_COEFF2_GFSK_FILTER_COEFF_MANUAL2_MASK)

/*! @name TX_GFSK_COEFF1 - TX GFSK Filter Coefficients 1 */
#define XCVR_TX_GFSK_COEFF1_GFSK_FILTER_COEFF_MANUAL1_MASK (0xFFFFFFFFU)
#define XCVR_TX_GFSK_COEFF1_GFSK_FILTER_COEFF_MANUAL1_SHIFT (0U)
#define XCVR_TX_GFSK_COEFF1_GFSK_FILTER_COEFF_MANUAL1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_GFSK_COEFF1_GFSK_FILTER_COEFF_MANUAL1_SHIFT)) & XCVR_TX_GFSK_COEFF1_GFSK_FILTER_COEFF_MANUAL1_MASK)

/*! @name TX_FSK_MOD_SCALE - TX FSK Modulation Scale */
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_0_MASK (0x1FFFU)
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_0_SHIFT (0U)
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_0(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_0_SHIFT)) & XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_0_MASK)
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_1_MASK (0x1FFF0000U)
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_1_SHIFT (16U)
#define XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_1(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_1_SHIFT)) & XCVR_TX_FSK_MOD_SCALE_FSK_MODULATION_SCALE_1_MASK)

/*! @name TX_DFT_MOD_PAT - TX DFT Modulation Pattern */
#define XCVR_TX_DFT_MOD_PAT_DFT_MOD_PATTERN_MASK (0xFFFFFFFFU)
#define XCVR_TX_DFT_MOD_PAT_DFT_MOD_PATTERN_SHIFT (0U)
#define XCVR_TX_DFT_MOD_PAT_DFT_MOD_PATTERN(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DFT_MOD_PAT_DFT_MOD_PATTERN_SHIFT)) & XCVR_TX_DFT_MOD_PAT_DFT_MOD_PATTERN_MASK)

/*! @name TX_DFT_TONE_0_1 - TX DFT Tones 0 and 1 */
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_1_MASK     (0x1FFFU)
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_1_SHIFT    (0U)
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_1(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DFT_TONE_0_1_DFT_TONE_1_SHIFT)) & XCVR_TX_DFT_TONE_0_1_DFT_TONE_1_MASK)
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_0_MASK     (0x1FFF0000U)
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_0_SHIFT    (16U)
#define XCVR_TX_DFT_TONE_0_1_DFT_TONE_0(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DFT_TONE_0_1_DFT_TONE_0_SHIFT)) & XCVR_TX_DFT_TONE_0_1_DFT_TONE_0_MASK)

/*! @name TX_DFT_TONE_2_3 - TX DFT Tones 2 and 3 */
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_3_MASK     (0x1FFFU)
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_3_SHIFT    (0U)
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_3(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DFT_TONE_2_3_DFT_TONE_3_SHIFT)) & XCVR_TX_DFT_TONE_2_3_DFT_TONE_3_MASK)
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_2_MASK     (0x1FFF0000U)
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_2_SHIFT    (16U)
#define XCVR_TX_DFT_TONE_2_3_DFT_TONE_2(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TX_DFT_TONE_2_3_DFT_TONE_2_SHIFT)) & XCVR_TX_DFT_TONE_2_3_DFT_TONE_2_MASK)

/*! @name PLL_MOD_OVRD - PLL Modulation Overrides */
#define XCVR_PLL_MOD_OVRD_MODULATION_WORD_MANUAL_MASK (0x1FFFU)
#define XCVR_PLL_MOD_OVRD_MODULATION_WORD_MANUAL_SHIFT (0U)
#define XCVR_PLL_MOD_OVRD_MODULATION_WORD_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_MODULATION_WORD_MANUAL_SHIFT)) & XCVR_PLL_MOD_OVRD_MODULATION_WORD_MANUAL_MASK)
#define XCVR_PLL_MOD_OVRD_MOD_DIS_MASK           (0x8000U)
#define XCVR_PLL_MOD_OVRD_MOD_DIS_SHIFT          (15U)
#define XCVR_PLL_MOD_OVRD_MOD_DIS(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_MOD_DIS_SHIFT)) & XCVR_PLL_MOD_OVRD_MOD_DIS_MASK)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_MANUAL_MASK   (0xFF0000U)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_MANUAL_SHIFT  (16U)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_MANUAL(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_HPM_BANK_MANUAL_SHIFT)) & XCVR_PLL_MOD_OVRD_HPM_BANK_MANUAL_MASK)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_DIS_MASK      (0x8000000U)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_DIS_SHIFT     (27U)
#define XCVR_PLL_MOD_OVRD_HPM_BANK_DIS(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_HPM_BANK_DIS_SHIFT)) & XCVR_PLL_MOD_OVRD_HPM_BANK_DIS_MASK)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_MANUAL_MASK    (0x30000000U)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_MANUAL_SHIFT   (28U)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_MANUAL(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_HPM_LSB_MANUAL_SHIFT)) & XCVR_PLL_MOD_OVRD_HPM_LSB_MANUAL_MASK)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_DIS_MASK       (0x80000000U)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_DIS_SHIFT      (31U)
#define XCVR_PLL_MOD_OVRD_HPM_LSB_DIS(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_MOD_OVRD_HPM_LSB_DIS_SHIFT)) & XCVR_PLL_MOD_OVRD_HPM_LSB_DIS_MASK)

/*! @name PLL_CHAN_MAP - PLL Channel Mapping */
#define XCVR_PLL_CHAN_MAP_CHANNEL_NUM_MASK       (0x7FU)
#define XCVR_PLL_CHAN_MAP_CHANNEL_NUM_SHIFT      (0U)
#define XCVR_PLL_CHAN_MAP_CHANNEL_NUM(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CHAN_MAP_CHANNEL_NUM_SHIFT)) & XCVR_PLL_CHAN_MAP_CHANNEL_NUM_MASK)
#define XCVR_PLL_CHAN_MAP_BOC_MASK               (0x100U)
#define XCVR_PLL_CHAN_MAP_BOC_SHIFT              (8U)
#define XCVR_PLL_CHAN_MAP_BOC(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CHAN_MAP_BOC_SHIFT)) & XCVR_PLL_CHAN_MAP_BOC_MASK)
#define XCVR_PLL_CHAN_MAP_BMR_MASK               (0x200U)
#define XCVR_PLL_CHAN_MAP_BMR_SHIFT              (9U)
#define XCVR_PLL_CHAN_MAP_BMR(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CHAN_MAP_BMR_SHIFT)) & XCVR_PLL_CHAN_MAP_BMR_MASK)
#define XCVR_PLL_CHAN_MAP_ZOC_MASK               (0x400U)
#define XCVR_PLL_CHAN_MAP_ZOC_SHIFT              (10U)
#define XCVR_PLL_CHAN_MAP_ZOC(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CHAN_MAP_ZOC_SHIFT)) & XCVR_PLL_CHAN_MAP_ZOC_MASK)

/*! @name PLL_LOCK_DETECT - PLL Lock Detect */
#define XCVR_PLL_LOCK_DETECT_CT_FAIL_MASK        (0x1U)
#define XCVR_PLL_LOCK_DETECT_CT_FAIL_SHIFT       (0U)
#define XCVR_PLL_LOCK_DETECT_CT_FAIL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_CT_FAIL_SHIFT)) & XCVR_PLL_LOCK_DETECT_CT_FAIL_MASK)
#define XCVR_PLL_LOCK_DETECT_CTFF_MASK           (0x2U)
#define XCVR_PLL_LOCK_DETECT_CTFF_SHIFT          (1U)
#define XCVR_PLL_LOCK_DETECT_CTFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_CTFF_SHIFT)) & XCVR_PLL_LOCK_DETECT_CTFF_MASK)
#define XCVR_PLL_LOCK_DETECT_CS_FAIL_MASK        (0x4U)
#define XCVR_PLL_LOCK_DETECT_CS_FAIL_SHIFT       (2U)
#define XCVR_PLL_LOCK_DETECT_CS_FAIL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_CS_FAIL_SHIFT)) & XCVR_PLL_LOCK_DETECT_CS_FAIL_MASK)
#define XCVR_PLL_LOCK_DETECT_CSFF_MASK           (0x8U)
#define XCVR_PLL_LOCK_DETECT_CSFF_SHIFT          (3U)
#define XCVR_PLL_LOCK_DETECT_CSFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_CSFF_SHIFT)) & XCVR_PLL_LOCK_DETECT_CSFF_MASK)
#define XCVR_PLL_LOCK_DETECT_FT_FAIL_MASK        (0x10U)
#define XCVR_PLL_LOCK_DETECT_FT_FAIL_SHIFT       (4U)
#define XCVR_PLL_LOCK_DETECT_FT_FAIL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FT_FAIL_SHIFT)) & XCVR_PLL_LOCK_DETECT_FT_FAIL_MASK)
#define XCVR_PLL_LOCK_DETECT_FTFF_MASK           (0x20U)
#define XCVR_PLL_LOCK_DETECT_FTFF_SHIFT          (5U)
#define XCVR_PLL_LOCK_DETECT_FTFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FTFF_SHIFT)) & XCVR_PLL_LOCK_DETECT_FTFF_MASK)
#define XCVR_PLL_LOCK_DETECT_TAFF_MASK           (0x80U)
#define XCVR_PLL_LOCK_DETECT_TAFF_SHIFT          (7U)
#define XCVR_PLL_LOCK_DETECT_TAFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_TAFF_SHIFT)) & XCVR_PLL_LOCK_DETECT_TAFF_MASK)
#define XCVR_PLL_LOCK_DETECT_CTUNE_LDF_LEV_MASK  (0xF00U)
#define XCVR_PLL_LOCK_DETECT_CTUNE_LDF_LEV_SHIFT (8U)
#define XCVR_PLL_LOCK_DETECT_CTUNE_LDF_LEV(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_CTUNE_LDF_LEV_SHIFT)) & XCVR_PLL_LOCK_DETECT_CTUNE_LDF_LEV_MASK)
#define XCVR_PLL_LOCK_DETECT_FTF_RX_THRSH_MASK   (0x3F000U)
#define XCVR_PLL_LOCK_DETECT_FTF_RX_THRSH_SHIFT  (12U)
#define XCVR_PLL_LOCK_DETECT_FTF_RX_THRSH(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FTF_RX_THRSH_SHIFT)) & XCVR_PLL_LOCK_DETECT_FTF_RX_THRSH_MASK)
#define XCVR_PLL_LOCK_DETECT_FTW_RX_MASK         (0x80000U)
#define XCVR_PLL_LOCK_DETECT_FTW_RX_SHIFT        (19U)
#define XCVR_PLL_LOCK_DETECT_FTW_RX(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FTW_RX_SHIFT)) & XCVR_PLL_LOCK_DETECT_FTW_RX_MASK)
#define XCVR_PLL_LOCK_DETECT_FTF_TX_THRSH_MASK   (0x3F00000U)
#define XCVR_PLL_LOCK_DETECT_FTF_TX_THRSH_SHIFT  (20U)
#define XCVR_PLL_LOCK_DETECT_FTF_TX_THRSH(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FTF_TX_THRSH_SHIFT)) & XCVR_PLL_LOCK_DETECT_FTF_TX_THRSH_MASK)
#define XCVR_PLL_LOCK_DETECT_FTW_TX_MASK         (0x8000000U)
#define XCVR_PLL_LOCK_DETECT_FTW_TX_SHIFT        (27U)
#define XCVR_PLL_LOCK_DETECT_FTW_TX(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LOCK_DETECT_FTW_TX_SHIFT)) & XCVR_PLL_LOCK_DETECT_FTW_TX_MASK)

/*! @name PLL_HP_MOD_CTRL - PLL High Port Modulation Control */
#define XCVR_PLL_HP_MOD_CTRL_HPM_SDM_MANUAL_MASK (0x3FFU)
#define XCVR_PLL_HP_MOD_CTRL_HPM_SDM_MANUAL_SHIFT (0U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_SDM_MANUAL(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HPM_SDM_MANUAL_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HPM_SDM_MANUAL_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HPFF_MASK           (0x2000U)
#define XCVR_PLL_HP_MOD_CTRL_HPFF_SHIFT          (13U)
#define XCVR_PLL_HP_MOD_CTRL_HPFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HPFF_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HPFF_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_INV_MASK     (0x4000U)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_INV_SHIFT    (14U)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_INV(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HP_SDM_INV_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HP_SDM_INV_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_DIS_MASK     (0x8000U)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_DIS_SHIFT    (15U)
#define XCVR_PLL_HP_MOD_CTRL_HP_SDM_DIS(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HP_SDM_DIS_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HP_SDM_DIS_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HPM_LFSR_LEN_MASK   (0x70000U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_LFSR_LEN_SHIFT  (16U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_LFSR_LEN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HPM_LFSR_LEN_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HPM_LFSR_LEN_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HP_DTH_SCL_MASK     (0x100000U)
#define XCVR_PLL_HP_MOD_CTRL_HP_DTH_SCL_SHIFT    (20U)
#define XCVR_PLL_HP_MOD_CTRL_HP_DTH_SCL(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HP_DTH_SCL_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HP_DTH_SCL_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HPM_DTH_EN_MASK     (0x800000U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_DTH_EN_SHIFT    (23U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_DTH_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HPM_DTH_EN_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HPM_DTH_EN_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HPM_SCALE_MASK      (0x3000000U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_SCALE_SHIFT     (24U)
#define XCVR_PLL_HP_MOD_CTRL_HPM_SCALE(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HPM_SCALE_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HPM_SCALE_MASK)
#define XCVR_PLL_HP_MOD_CTRL_HP_MOD_INV_MASK     (0x80000000U)
#define XCVR_PLL_HP_MOD_CTRL_HP_MOD_INV_SHIFT    (31U)
#define XCVR_PLL_HP_MOD_CTRL_HP_MOD_INV(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HP_MOD_CTRL_HP_MOD_INV_SHIFT)) & XCVR_PLL_HP_MOD_CTRL_HP_MOD_INV_MASK)

/*! @name PLL_HPM_CAL_CTRL - PLL HPM Calibration Control */
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MASK (0x1FFFU)
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_SHIFT (0U)
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_SHIFT)) & XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MASK)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_DIS_MASK    (0x8000U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_DIS_SHIFT   (15U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_DIS(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_CAL_CTRL_HP_CAL_DIS_SHIFT)) & XCVR_PLL_HPM_CAL_CTRL_HP_CAL_DIS_MASK)
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MANUAL_MASK (0x1FFF0000U)
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MANUAL_SHIFT (16U)
#define XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MANUAL_SHIFT)) & XCVR_PLL_HPM_CAL_CTRL_HPM_CAL_FACTOR_MANUAL_MASK)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_ARY_MASK    (0x40000000U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_ARY_SHIFT   (30U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_ARY(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_CAL_CTRL_HP_CAL_ARY_SHIFT)) & XCVR_PLL_HPM_CAL_CTRL_HP_CAL_ARY_MASK)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_TIME_MASK   (0x80000000U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_TIME_SHIFT  (31U)
#define XCVR_PLL_HPM_CAL_CTRL_HP_CAL_TIME(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_CAL_CTRL_HP_CAL_TIME_SHIFT)) & XCVR_PLL_HPM_CAL_CTRL_HP_CAL_TIME_MASK)

/*! @name PLL_LD_HPM_CAL1 - PLL Cycle Slip Lock Detect Configuration and HPM Calibration 1 */
#define XCVR_PLL_LD_HPM_CAL1_CNT_1_MASK          (0x1FFFFU)
#define XCVR_PLL_LD_HPM_CAL1_CNT_1_SHIFT         (0U)
#define XCVR_PLL_LD_HPM_CAL1_CNT_1(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL1_CNT_1_SHIFT)) & XCVR_PLL_LD_HPM_CAL1_CNT_1_MASK)
#define XCVR_PLL_LD_HPM_CAL1_CS_WT_MASK          (0x700000U)
#define XCVR_PLL_LD_HPM_CAL1_CS_WT_SHIFT         (20U)
#define XCVR_PLL_LD_HPM_CAL1_CS_WT(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL1_CS_WT_SHIFT)) & XCVR_PLL_LD_HPM_CAL1_CS_WT_MASK)
#define XCVR_PLL_LD_HPM_CAL1_CS_FW_MASK          (0x7000000U)
#define XCVR_PLL_LD_HPM_CAL1_CS_FW_SHIFT         (24U)
#define XCVR_PLL_LD_HPM_CAL1_CS_FW(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL1_CS_FW_SHIFT)) & XCVR_PLL_LD_HPM_CAL1_CS_FW_MASK)
#define XCVR_PLL_LD_HPM_CAL1_CS_FCNT_MASK        (0xF0000000U)
#define XCVR_PLL_LD_HPM_CAL1_CS_FCNT_SHIFT       (28U)
#define XCVR_PLL_LD_HPM_CAL1_CS_FCNT(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL1_CS_FCNT_SHIFT)) & XCVR_PLL_LD_HPM_CAL1_CS_FCNT_MASK)

/*! @name PLL_LD_HPM_CAL2 - PLL Cycle Slip Lock Detect Configuration and HPM Calibration 2 */
#define XCVR_PLL_LD_HPM_CAL2_CNT_2_MASK          (0x1FFFFU)
#define XCVR_PLL_LD_HPM_CAL2_CNT_2_SHIFT         (0U)
#define XCVR_PLL_LD_HPM_CAL2_CNT_2(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL2_CNT_2_SHIFT)) & XCVR_PLL_LD_HPM_CAL2_CNT_2_MASK)
#define XCVR_PLL_LD_HPM_CAL2_CS_RC_MASK          (0x100000U)
#define XCVR_PLL_LD_HPM_CAL2_CS_RC_SHIFT         (20U)
#define XCVR_PLL_LD_HPM_CAL2_CS_RC(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL2_CS_RC_SHIFT)) & XCVR_PLL_LD_HPM_CAL2_CS_RC_MASK)
#define XCVR_PLL_LD_HPM_CAL2_CS_FT_MASK          (0x1F000000U)
#define XCVR_PLL_LD_HPM_CAL2_CS_FT_SHIFT         (24U)
#define XCVR_PLL_LD_HPM_CAL2_CS_FT(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LD_HPM_CAL2_CS_FT_SHIFT)) & XCVR_PLL_LD_HPM_CAL2_CS_FT_MASK)

/*! @name PLL_HPM_SDM_FRACTION - PLL HPM SDM Fraction */
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_NUM_SELECTED_MASK (0x3FFU)
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_NUM_SELECTED_SHIFT (0U)
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_NUM_SELECTED(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_SDM_FRACTION_HPM_NUM_SELECTED_SHIFT)) & XCVR_PLL_HPM_SDM_FRACTION_HPM_NUM_SELECTED_MASK)
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_DENOM_MASK (0x3FF0000U)
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_DENOM_SHIFT (16U)
#define XCVR_PLL_HPM_SDM_FRACTION_HPM_DENOM(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_HPM_SDM_FRACTION_HPM_DENOM_SHIFT)) & XCVR_PLL_HPM_SDM_FRACTION_HPM_DENOM_MASK)

/*! @name PLL_LP_MOD_CTRL - PLL Low Port Modulation Control */
#define XCVR_PLL_LP_MOD_CTRL_PLL_LOOP_DIVIDER_MANUAL_MASK (0x3FU)
#define XCVR_PLL_LP_MOD_CTRL_PLL_LOOP_DIVIDER_MANUAL_SHIFT (0U)
#define XCVR_PLL_LP_MOD_CTRL_PLL_LOOP_DIVIDER_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_PLL_LOOP_DIVIDER_MANUAL_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_PLL_LOOP_DIVIDER_MANUAL_MASK)
#define XCVR_PLL_LP_MOD_CTRL_PLL_LD_DIS_MASK     (0x800U)
#define XCVR_PLL_LP_MOD_CTRL_PLL_LD_DIS_SHIFT    (11U)
#define XCVR_PLL_LP_MOD_CTRL_PLL_LD_DIS(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_PLL_LD_DIS_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_PLL_LD_DIS_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPFF_MASK           (0x2000U)
#define XCVR_PLL_LP_MOD_CTRL_LPFF_SHIFT          (13U)
#define XCVR_PLL_LP_MOD_CTRL_LPFF(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPFF_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPFF_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_INV_MASK    (0x4000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_INV_SHIFT   (14U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_INV(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_SDM_INV_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_SDM_INV_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_DIS_MASK    (0x8000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_DIS_SHIFT   (15U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SDM_DIS(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_SDM_DIS_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_SDM_DIS_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_DTH_SCL_MASK    (0xF0000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_DTH_SCL_SHIFT   (16U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_DTH_SCL(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_DTH_SCL_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_DTH_SCL_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_CTRL_MASK     (0x400000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_CTRL_SHIFT    (22U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_CTRL(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_D_CTRL_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_D_CTRL_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_OVRD_MASK     (0x800000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_OVRD_SHIFT    (23U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_D_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_D_OVRD_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_D_OVRD_MASK)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SCALE_MASK      (0xF000000U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SCALE_SHIFT     (24U)
#define XCVR_PLL_LP_MOD_CTRL_LPM_SCALE(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_MOD_CTRL_LPM_SCALE_SHIFT)) & XCVR_PLL_LP_MOD_CTRL_LPM_SCALE_MASK)

/*! @name PLL_LP_SDM_CTRL1 - PLL Low Port SDM Control 1 */
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SELECTED_MASK (0x7FU)
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SELECTED_SHIFT (0U)
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SELECTED(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SELECTED_SHIFT)) & XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SELECTED_MASK)
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_MASK      (0x7F0000U)
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SHIFT     (16U)
#define XCVR_PLL_LP_SDM_CTRL1_LPM_INTG(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_SHIFT)) & XCVR_PLL_LP_SDM_CTRL1_LPM_INTG_MASK)
#define XCVR_PLL_LP_SDM_CTRL1_SDM_MAP_DIS_MASK   (0x80000000U)
#define XCVR_PLL_LP_SDM_CTRL1_SDM_MAP_DIS_SHIFT  (31U)
#define XCVR_PLL_LP_SDM_CTRL1_SDM_MAP_DIS(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_CTRL1_SDM_MAP_DIS_SHIFT)) & XCVR_PLL_LP_SDM_CTRL1_SDM_MAP_DIS_MASK)

/*! @name PLL_LP_SDM_CTRL2 - PLL Low Port SDM Control 2 */
#define XCVR_PLL_LP_SDM_CTRL2_LPM_NUM_MASK       (0xFFFFFFFU)
#define XCVR_PLL_LP_SDM_CTRL2_LPM_NUM_SHIFT      (0U)
#define XCVR_PLL_LP_SDM_CTRL2_LPM_NUM(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_CTRL2_LPM_NUM_SHIFT)) & XCVR_PLL_LP_SDM_CTRL2_LPM_NUM_MASK)

/*! @name PLL_LP_SDM_CTRL3 - PLL Low Port SDM Control 3 */
#define XCVR_PLL_LP_SDM_CTRL3_LPM_DENOM_MASK     (0xFFFFFFFU)
#define XCVR_PLL_LP_SDM_CTRL3_LPM_DENOM_SHIFT    (0U)
#define XCVR_PLL_LP_SDM_CTRL3_LPM_DENOM(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_CTRL3_LPM_DENOM_SHIFT)) & XCVR_PLL_LP_SDM_CTRL3_LPM_DENOM_MASK)

/*! @name PLL_LP_SDM_NUM - PLL Low Port SDM Numerator Applied */
#define XCVR_PLL_LP_SDM_NUM_LPM_NUM_SELECTED_MASK (0xFFFFFFFU)
#define XCVR_PLL_LP_SDM_NUM_LPM_NUM_SELECTED_SHIFT (0U)
#define XCVR_PLL_LP_SDM_NUM_LPM_NUM_SELECTED(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_NUM_LPM_NUM_SELECTED_SHIFT)) & XCVR_PLL_LP_SDM_NUM_LPM_NUM_SELECTED_MASK)

/*! @name PLL_LP_SDM_DENOM - PLL Low Port SDM Denominator Applied */
#define XCVR_PLL_LP_SDM_DENOM_LPM_DENOM_SELECTED_MASK (0xFFFFFFFU)
#define XCVR_PLL_LP_SDM_DENOM_LPM_DENOM_SELECTED_SHIFT (0U)
#define XCVR_PLL_LP_SDM_DENOM_LPM_DENOM_SELECTED(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_LP_SDM_DENOM_LPM_DENOM_SELECTED_SHIFT)) & XCVR_PLL_LP_SDM_DENOM_LPM_DENOM_SELECTED_MASK)

/*! @name PLL_DELAY_MATCH - PLL Delay Matching */
#define XCVR_PLL_DELAY_MATCH_LP_SDM_DELAY_MASK   (0xFU)
#define XCVR_PLL_DELAY_MATCH_LP_SDM_DELAY_SHIFT  (0U)
#define XCVR_PLL_DELAY_MATCH_LP_SDM_DELAY(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DELAY_MATCH_LP_SDM_DELAY_SHIFT)) & XCVR_PLL_DELAY_MATCH_LP_SDM_DELAY_MASK)
#define XCVR_PLL_DELAY_MATCH_HPM_SDM_DELAY_MASK  (0xF00U)
#define XCVR_PLL_DELAY_MATCH_HPM_SDM_DELAY_SHIFT (8U)
#define XCVR_PLL_DELAY_MATCH_HPM_SDM_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DELAY_MATCH_HPM_SDM_DELAY_SHIFT)) & XCVR_PLL_DELAY_MATCH_HPM_SDM_DELAY_MASK)
#define XCVR_PLL_DELAY_MATCH_HPM_BANK_DELAY_MASK (0xF0000U)
#define XCVR_PLL_DELAY_MATCH_HPM_BANK_DELAY_SHIFT (16U)
#define XCVR_PLL_DELAY_MATCH_HPM_BANK_DELAY(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DELAY_MATCH_HPM_BANK_DELAY_SHIFT)) & XCVR_PLL_DELAY_MATCH_HPM_BANK_DELAY_MASK)

/*! @name PLL_CTUNE_CTRL - PLL Coarse Tune Control */
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TARGET_MANUAL_MASK (0xFFFU)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TARGET_MANUAL_SHIFT (0U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TARGET_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CTRL_CTUNE_TARGET_MANUAL_SHIFT)) & XCVR_PLL_CTUNE_CTRL_CTUNE_TARGET_MANUAL_MASK)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TD_MASK        (0x8000U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TD_SHIFT       (15U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_TD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CTRL_CTUNE_TD_SHIFT)) & XCVR_PLL_CTUNE_CTRL_CTUNE_TD_MASK)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_ADJUST_MASK    (0xF0000U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_ADJUST_SHIFT   (16U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_ADJUST(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CTRL_CTUNE_ADJUST_SHIFT)) & XCVR_PLL_CTUNE_CTRL_CTUNE_ADJUST_MASK)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_MANUAL_MASK    (0x7F000000U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_MANUAL_SHIFT   (24U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_MANUAL(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CTRL_CTUNE_MANUAL_SHIFT)) & XCVR_PLL_CTUNE_CTRL_CTUNE_MANUAL_MASK)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_DIS_MASK       (0x80000000U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_DIS_SHIFT      (31U)
#define XCVR_PLL_CTUNE_CTRL_CTUNE_DIS(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CTRL_CTUNE_DIS_SHIFT)) & XCVR_PLL_CTUNE_CTRL_CTUNE_DIS_MASK)

/*! @name PLL_CTUNE_CNT6 - PLL Coarse Tune Count 6 */
#define XCVR_PLL_CTUNE_CNT6_CTUNE_COUNT_6_MASK   (0xFFFU)
#define XCVR_PLL_CTUNE_CNT6_CTUNE_COUNT_6_SHIFT  (0U)
#define XCVR_PLL_CTUNE_CNT6_CTUNE_COUNT_6(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT6_CTUNE_COUNT_6_SHIFT)) & XCVR_PLL_CTUNE_CNT6_CTUNE_COUNT_6_MASK)

/*! @name PLL_CTUNE_CNT5_4 - PLL Coarse Tune Counts 5 and 4 */
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_4_MASK (0xFFFU)
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_4_SHIFT (0U)
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_4(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_4_SHIFT)) & XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_4_MASK)
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_5_MASK (0xFFF0000U)
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_5_SHIFT (16U)
#define XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_5(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_5_SHIFT)) & XCVR_PLL_CTUNE_CNT5_4_CTUNE_COUNT_5_MASK)

/*! @name PLL_CTUNE_CNT3_2 - PLL Coarse Tune Counts 3 and 2 */
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_2_MASK (0xFFFU)
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_2_SHIFT (0U)
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_2(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_2_SHIFT)) & XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_2_MASK)
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_3_MASK (0xFFF0000U)
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_3_SHIFT (16U)
#define XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_3(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_3_SHIFT)) & XCVR_PLL_CTUNE_CNT3_2_CTUNE_COUNT_3_MASK)

/*! @name PLL_CTUNE_CNT1_0 - PLL Coarse Tune Counts 1 and 0 */
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_0_MASK (0xFFFU)
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_0_SHIFT (0U)
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_0(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_0_SHIFT)) & XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_0_MASK)
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_1_MASK (0xFFF0000U)
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_1_SHIFT (16U)
#define XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_1(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_1_SHIFT)) & XCVR_PLL_CTUNE_CNT1_0_CTUNE_COUNT_1_MASK)

/*! @name PLL_CTUNE_RESULTS - PLL Coarse Tune Results */
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_SELECTED_MASK (0x7FU)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_SELECTED_SHIFT (0U)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_SELECTED(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_RESULTS_CTUNE_SELECTED_SHIFT)) & XCVR_PLL_CTUNE_RESULTS_CTUNE_SELECTED_MASK)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_BEST_DIFF_MASK (0xFF00U)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_BEST_DIFF_SHIFT (8U)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_BEST_DIFF(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_RESULTS_CTUNE_BEST_DIFF_SHIFT)) & XCVR_PLL_CTUNE_RESULTS_CTUNE_BEST_DIFF_MASK)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_FREQ_TARGET_MASK (0xFFF0000U)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_FREQ_TARGET_SHIFT (16U)
#define XCVR_PLL_CTUNE_RESULTS_CTUNE_FREQ_TARGET(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTUNE_RESULTS_CTUNE_FREQ_TARGET_SHIFT)) & XCVR_PLL_CTUNE_RESULTS_CTUNE_FREQ_TARGET_MASK)

/*! @name CTRL - Transceiver Control */
#define XCVR_CTRL_PROTOCOL_MASK                  (0x7U)
#define XCVR_CTRL_PROTOCOL_SHIFT                 (0U)
#define XCVR_CTRL_PROTOCOL(x)                    (((uint32_t)(((uint32_t)(x)) << XCVR_CTRL_PROTOCOL_SHIFT)) & XCVR_CTRL_PROTOCOL_MASK)
#define XCVR_CTRL_TGT_PWR_SRC_MASK               (0x30U)
#define XCVR_CTRL_TGT_PWR_SRC_SHIFT              (4U)
#define XCVR_CTRL_TGT_PWR_SRC(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_CTRL_TGT_PWR_SRC_SHIFT)) & XCVR_CTRL_TGT_PWR_SRC_MASK)
#define XCVR_CTRL_REF_CLK_FREQ_MASK              (0xC0U)
#define XCVR_CTRL_REF_CLK_FREQ_SHIFT             (6U)
#define XCVR_CTRL_REF_CLK_FREQ(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_CTRL_REF_CLK_FREQ_SHIFT)) & XCVR_CTRL_REF_CLK_FREQ_MASK)

/*! @name STATUS - Transceiver Status */
#define XCVR_STATUS_TSM_COUNT_MASK               (0xFFU)
#define XCVR_STATUS_TSM_COUNT_SHIFT              (0U)
#define XCVR_STATUS_TSM_COUNT(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_TSM_COUNT_SHIFT)) & XCVR_STATUS_TSM_COUNT_MASK)
#define XCVR_STATUS_PLL_SEQ_STATE_MASK           (0xF00U)
#define XCVR_STATUS_PLL_SEQ_STATE_SHIFT          (8U)
#define XCVR_STATUS_PLL_SEQ_STATE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_PLL_SEQ_STATE_SHIFT)) & XCVR_STATUS_PLL_SEQ_STATE_MASK)
#define XCVR_STATUS_RX_MODE_MASK                 (0x1000U)
#define XCVR_STATUS_RX_MODE_SHIFT                (12U)
#define XCVR_STATUS_RX_MODE(x)                   (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_RX_MODE_SHIFT)) & XCVR_STATUS_RX_MODE_MASK)
#define XCVR_STATUS_TX_MODE_MASK                 (0x2000U)
#define XCVR_STATUS_TX_MODE_SHIFT                (13U)
#define XCVR_STATUS_TX_MODE(x)                   (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_TX_MODE_SHIFT)) & XCVR_STATUS_TX_MODE_MASK)
#define XCVR_STATUS_BTLE_SYSCLK_REQ_MASK         (0x10000U)
#define XCVR_STATUS_BTLE_SYSCLK_REQ_SHIFT        (16U)
#define XCVR_STATUS_BTLE_SYSCLK_REQ(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_BTLE_SYSCLK_REQ_SHIFT)) & XCVR_STATUS_BTLE_SYSCLK_REQ_MASK)
#define XCVR_STATUS_RIF_LL_ACTIVE_MASK           (0x20000U)
#define XCVR_STATUS_RIF_LL_ACTIVE_SHIFT          (17U)
#define XCVR_STATUS_RIF_LL_ACTIVE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_RIF_LL_ACTIVE_SHIFT)) & XCVR_STATUS_RIF_LL_ACTIVE_MASK)
#define XCVR_STATUS_XTAL_READY_MASK              (0x40000U)
#define XCVR_STATUS_XTAL_READY_SHIFT             (18U)
#define XCVR_STATUS_XTAL_READY(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_XTAL_READY_SHIFT)) & XCVR_STATUS_XTAL_READY_MASK)
#define XCVR_STATUS_SOC_USING_RF_OSC_CLK_MASK    (0x80000U)
#define XCVR_STATUS_SOC_USING_RF_OSC_CLK_SHIFT   (19U)
#define XCVR_STATUS_SOC_USING_RF_OSC_CLK(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_STATUS_SOC_USING_RF_OSC_CLK_SHIFT)) & XCVR_STATUS_SOC_USING_RF_OSC_CLK_MASK)

/*! @name OVERWRITE_VER - Overwrite Version */
#define XCVR_OVERWRITE_VER_OVERWRITE_VER_MASK    (0xFFU)
#define XCVR_OVERWRITE_VER_OVERWRITE_VER_SHIFT   (0U)
#define XCVR_OVERWRITE_VER_OVERWRITE_VER(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_OVERWRITE_VER_OVERWRITE_VER_SHIFT)) & XCVR_OVERWRITE_VER_OVERWRITE_VER_MASK)

/*! @name DMA_CTRL - DMA Control */
#define XCVR_DMA_CTRL_DMA_I_EN_MASK              (0x1U)
#define XCVR_DMA_CTRL_DMA_I_EN_SHIFT             (0U)
#define XCVR_DMA_CTRL_DMA_I_EN(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_DMA_CTRL_DMA_I_EN_SHIFT)) & XCVR_DMA_CTRL_DMA_I_EN_MASK)
#define XCVR_DMA_CTRL_DMA_Q_EN_MASK              (0x2U)
#define XCVR_DMA_CTRL_DMA_Q_EN_SHIFT             (1U)
#define XCVR_DMA_CTRL_DMA_Q_EN(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_DMA_CTRL_DMA_Q_EN_SHIFT)) & XCVR_DMA_CTRL_DMA_Q_EN_MASK)

/*! @name DMA_DATA - DMA Data */
#define XCVR_DMA_DATA_DMA_DATA_11_0_MASK         (0xFFFU)
#define XCVR_DMA_DATA_DMA_DATA_11_0_SHIFT        (0U)
#define XCVR_DMA_DATA_DMA_DATA_11_0(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_DMA_DATA_DMA_DATA_11_0_SHIFT)) & XCVR_DMA_DATA_DMA_DATA_11_0_MASK)
#define XCVR_DMA_DATA_DMA_DATA_27_16_MASK        (0xFFF0000U)
#define XCVR_DMA_DATA_DMA_DATA_27_16_SHIFT       (16U)
#define XCVR_DMA_DATA_DMA_DATA_27_16(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_DMA_DATA_DMA_DATA_27_16_SHIFT)) & XCVR_DMA_DATA_DMA_DATA_27_16_MASK)

/*! @name DTEST_CTRL - Digital Test Control */
#define XCVR_DTEST_CTRL_DTEST_PAGE_MASK          (0x3FU)
#define XCVR_DTEST_CTRL_DTEST_PAGE_SHIFT         (0U)
#define XCVR_DTEST_CTRL_DTEST_PAGE(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_DTEST_PAGE_SHIFT)) & XCVR_DTEST_CTRL_DTEST_PAGE_MASK)
#define XCVR_DTEST_CTRL_DTEST_EN_MASK            (0x80U)
#define XCVR_DTEST_CTRL_DTEST_EN_SHIFT           (7U)
#define XCVR_DTEST_CTRL_DTEST_EN(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_DTEST_EN_SHIFT)) & XCVR_DTEST_CTRL_DTEST_EN_MASK)
#define XCVR_DTEST_CTRL_GPIO0_OVLAY_PIN_MASK     (0xF00U)
#define XCVR_DTEST_CTRL_GPIO0_OVLAY_PIN_SHIFT    (8U)
#define XCVR_DTEST_CTRL_GPIO0_OVLAY_PIN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_GPIO0_OVLAY_PIN_SHIFT)) & XCVR_DTEST_CTRL_GPIO0_OVLAY_PIN_MASK)
#define XCVR_DTEST_CTRL_GPIO1_OVLAY_PIN_MASK     (0xF000U)
#define XCVR_DTEST_CTRL_GPIO1_OVLAY_PIN_SHIFT    (12U)
#define XCVR_DTEST_CTRL_GPIO1_OVLAY_PIN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_GPIO1_OVLAY_PIN_SHIFT)) & XCVR_DTEST_CTRL_GPIO1_OVLAY_PIN_MASK)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_0_MASK    (0x10000U)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_0_SHIFT   (16U)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_0(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_0_SHIFT)) & XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_0_MASK)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_1_MASK    (0x20000U)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_1_SHIFT   (17U)
#define XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_1(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_1_SHIFT)) & XCVR_DTEST_CTRL_TSM_GPIO_OVLAY_1_MASK)
#define XCVR_DTEST_CTRL_DTEST_SHFT_MASK          (0x7000000U)
#define XCVR_DTEST_CTRL_DTEST_SHFT_SHIFT         (24U)
#define XCVR_DTEST_CTRL_DTEST_SHFT(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_DTEST_SHFT_SHIFT)) & XCVR_DTEST_CTRL_DTEST_SHFT_MASK)
#define XCVR_DTEST_CTRL_RAW_MODE_I_MASK          (0x10000000U)
#define XCVR_DTEST_CTRL_RAW_MODE_I_SHIFT         (28U)
#define XCVR_DTEST_CTRL_RAW_MODE_I(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_RAW_MODE_I_SHIFT)) & XCVR_DTEST_CTRL_RAW_MODE_I_MASK)
#define XCVR_DTEST_CTRL_RAW_MODE_Q_MASK          (0x20000000U)
#define XCVR_DTEST_CTRL_RAW_MODE_Q_SHIFT         (29U)
#define XCVR_DTEST_CTRL_RAW_MODE_Q(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_DTEST_CTRL_RAW_MODE_Q_SHIFT)) & XCVR_DTEST_CTRL_RAW_MODE_Q_MASK)

/*! @name PB_CTRL - Packet Buffer Control Register */
#define XCVR_PB_CTRL_PB_PROTECT_MASK             (0x1U)
#define XCVR_PB_CTRL_PB_PROTECT_SHIFT            (0U)
#define XCVR_PB_CTRL_PB_PROTECT(x)               (((uint32_t)(((uint32_t)(x)) << XCVR_PB_CTRL_PB_PROTECT_SHIFT)) & XCVR_PB_CTRL_PB_PROTECT_MASK)

/*! @name TSM_CTRL - Transceiver Sequence Manager Control */
#define XCVR_TSM_CTRL_FORCE_TX_EN_MASK           (0x4U)
#define XCVR_TSM_CTRL_FORCE_TX_EN_SHIFT          (2U)
#define XCVR_TSM_CTRL_FORCE_TX_EN(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_FORCE_TX_EN_SHIFT)) & XCVR_TSM_CTRL_FORCE_TX_EN_MASK)
#define XCVR_TSM_CTRL_FORCE_RX_EN_MASK           (0x8U)
#define XCVR_TSM_CTRL_FORCE_RX_EN_SHIFT          (3U)
#define XCVR_TSM_CTRL_FORCE_RX_EN(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_FORCE_RX_EN_SHIFT)) & XCVR_TSM_CTRL_FORCE_RX_EN_MASK)
#define XCVR_TSM_CTRL_PA_RAMP_SEL_MASK           (0x30U)
#define XCVR_TSM_CTRL_PA_RAMP_SEL_SHIFT          (4U)
#define XCVR_TSM_CTRL_PA_RAMP_SEL(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_PA_RAMP_SEL_SHIFT)) & XCVR_TSM_CTRL_PA_RAMP_SEL_MASK)
#define XCVR_TSM_CTRL_DATA_PADDING_EN_MASK       (0x40U)
#define XCVR_TSM_CTRL_DATA_PADDING_EN_SHIFT      (6U)
#define XCVR_TSM_CTRL_DATA_PADDING_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_DATA_PADDING_EN_SHIFT)) & XCVR_TSM_CTRL_DATA_PADDING_EN_MASK)
#define XCVR_TSM_CTRL_TX_ABORT_DIS_MASK          (0x10000U)
#define XCVR_TSM_CTRL_TX_ABORT_DIS_SHIFT         (16U)
#define XCVR_TSM_CTRL_TX_ABORT_DIS(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_TX_ABORT_DIS_SHIFT)) & XCVR_TSM_CTRL_TX_ABORT_DIS_MASK)
#define XCVR_TSM_CTRL_RX_ABORT_DIS_MASK          (0x20000U)
#define XCVR_TSM_CTRL_RX_ABORT_DIS_SHIFT         (17U)
#define XCVR_TSM_CTRL_RX_ABORT_DIS(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_RX_ABORT_DIS_SHIFT)) & XCVR_TSM_CTRL_RX_ABORT_DIS_MASK)
#define XCVR_TSM_CTRL_ABORT_ON_CTUNE_MASK        (0x40000U)
#define XCVR_TSM_CTRL_ABORT_ON_CTUNE_SHIFT       (18U)
#define XCVR_TSM_CTRL_ABORT_ON_CTUNE(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_ABORT_ON_CTUNE_SHIFT)) & XCVR_TSM_CTRL_ABORT_ON_CTUNE_MASK)
#define XCVR_TSM_CTRL_ABORT_ON_CYCLE_SLIP_MASK   (0x80000U)
#define XCVR_TSM_CTRL_ABORT_ON_CYCLE_SLIP_SHIFT  (19U)
#define XCVR_TSM_CTRL_ABORT_ON_CYCLE_SLIP(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_ABORT_ON_CYCLE_SLIP_SHIFT)) & XCVR_TSM_CTRL_ABORT_ON_CYCLE_SLIP_MASK)
#define XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG_MASK    (0x100000U)
#define XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG_SHIFT   (20U)
#define XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG_SHIFT)) & XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG_MASK)
#define XCVR_TSM_CTRL_BKPT_MASK                  (0xFF000000U)
#define XCVR_TSM_CTRL_BKPT_SHIFT                 (24U)
#define XCVR_TSM_CTRL_BKPT(x)                    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_CTRL_BKPT_SHIFT)) & XCVR_TSM_CTRL_BKPT_MASK)

/*! @name END_OF_SEQ - End of Sequence Control */
#define XCVR_END_OF_SEQ_END_OF_TX_WU_MASK        (0xFFU)
#define XCVR_END_OF_SEQ_END_OF_TX_WU_SHIFT       (0U)
#define XCVR_END_OF_SEQ_END_OF_TX_WU(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_END_OF_SEQ_END_OF_TX_WU_SHIFT)) & XCVR_END_OF_SEQ_END_OF_TX_WU_MASK)
#define XCVR_END_OF_SEQ_END_OF_TX_WD_MASK        (0xFF00U)
#define XCVR_END_OF_SEQ_END_OF_TX_WD_SHIFT       (8U)
#define XCVR_END_OF_SEQ_END_OF_TX_WD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_END_OF_SEQ_END_OF_TX_WD_SHIFT)) & XCVR_END_OF_SEQ_END_OF_TX_WD_MASK)
#define XCVR_END_OF_SEQ_END_OF_RX_WU_MASK        (0xFF0000U)
#define XCVR_END_OF_SEQ_END_OF_RX_WU_SHIFT       (16U)
#define XCVR_END_OF_SEQ_END_OF_RX_WU(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_END_OF_SEQ_END_OF_RX_WU_SHIFT)) & XCVR_END_OF_SEQ_END_OF_RX_WU_MASK)
#define XCVR_END_OF_SEQ_END_OF_RX_WD_MASK        (0xFF000000U)
#define XCVR_END_OF_SEQ_END_OF_RX_WD_SHIFT       (24U)
#define XCVR_END_OF_SEQ_END_OF_RX_WD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_END_OF_SEQ_END_OF_RX_WD_SHIFT)) & XCVR_END_OF_SEQ_END_OF_RX_WD_MASK)

/*! @name TSM_OVRD0 - TSM Override 0 */
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_EN_MASK   (0x1U)
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_EN_SHIFT  (0U)
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_MASK      (0x2U)
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_SHIFT     (1U)
#define XCVR_TSM_OVRD0_PLL_REG_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_EN_MASK (0x4U)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_EN_SHIFT (2U)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_MASK  (0x8U)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_SHIFT (3U)
#define XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_EN_MASK  (0x10U)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_EN_SHIFT (4U)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_MASK     (0x20U)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_SHIFT    (5U)
#define XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_QGEN_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_EN_MASK (0x40U)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_EN_SHIFT (6U)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_MASK   (0x80U)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_SHIFT  (7U)
#define XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_TCA_TX_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_EN_MASK (0x100U)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_EN_SHIFT (8U)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_MASK  (0x200U)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_SHIFT (9U)
#define XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_ADC_ANA_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_EN_MASK (0x400U)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_EN_SHIFT (10U)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_MASK  (0x800U)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_SHIFT (11U)
#define XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_ADC_DIG_REG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_EN_MASK (0x1000U)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_EN_SHIFT (12U)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_MASK (0x2000U)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_SHIFT (13U)
#define XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_XTAL_PLL_REF_CLK_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_EN_MASK (0x4000U)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_EN_SHIFT (14U)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_MASK (0x8000U)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_SHIFT (15U)
#define XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_XTAL_ADC_REF_CLK_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_EN_MASK (0x10000U)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_EN_SHIFT (16U)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_MASK (0x20000U)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_SHIFT (17U)
#define XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_AUTOTUNE_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_EN_MASK (0x40000U)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_EN_SHIFT (18U)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_MASK (0x80000U)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_SHIFT (19U)
#define XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_CYCLE_SLIP_LD_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_EN_MASK   (0x100000U)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_EN_SHIFT  (20U)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_MASK      (0x200000U)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_SHIFT     (21U)
#define XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_EN_MASK (0x400000U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_EN_SHIFT (22U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_MASK (0x800000U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_SHIFT (23U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_BUF_RX_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_EN_MASK (0x1000000U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_EN_SHIFT (24U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_MASK (0x2000000U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_SHIFT (25U)
#define XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_VCO_BUF_TX_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_EN_MASK (0x4000000U)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_EN_SHIFT (26U)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_MASK   (0x8000000U)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_SHIFT  (27U)
#define XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_PA_BUF_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_EN_MASK   (0x10000000U)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_EN_SHIFT  (28U)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_MASK      (0x20000000U)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_SHIFT     (29U)
#define XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_LDV_EN_OVRD_MASK)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_EN_MASK (0x40000000U)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_EN_SHIFT (30U)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_MASK (0x80000000U)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_SHIFT (31U)
#define XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD0_PLL_RX_LDV_RIPPLE_MUX_EN_OVRD_MASK)

/*! @name TSM_OVRD1 - TSM Override 1 */
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_EN_MASK (0x1U)
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_EN_SHIFT (0U)
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_MASK (0x2U)
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_SHIFT (1U)
#define XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_PLL_TX_LDV_RIPPLE_MUX_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_EN_MASK (0x4U)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_EN_SHIFT (2U)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_MASK (0x8U)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_SHIFT (3U)
#define XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_PLL_FILTER_CHARGE_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_EN_MASK (0x10U)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_EN_SHIFT (4U)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_EN(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_MASK    (0x20U)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_SHIFT   (5U)
#define XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_PLL_PHDET_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD_EN_MASK    (0x40U)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD_EN_SHIFT   (6U)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_QGEN25_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_QGEN25_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD_MASK       (0x80U)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD_SHIFT      (7U)
#define XCVR_TSM_OVRD1_QGEN25_EN_OVRD(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_QGEN25_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_QGEN25_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_TX_EN_OVRD_EN_MASK        (0x100U)
#define XCVR_TSM_OVRD1_TX_EN_OVRD_EN_SHIFT       (8U)
#define XCVR_TSM_OVRD1_TX_EN_OVRD_EN(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_TX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_TX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_TX_EN_OVRD_MASK           (0x200U)
#define XCVR_TSM_OVRD1_TX_EN_OVRD_SHIFT          (9U)
#define XCVR_TSM_OVRD1_TX_EN_OVRD(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_TX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_TX_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD_EN_MASK       (0x400U)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD_EN_SHIFT      (10U)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD_MASK          (0x800U)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD_SHIFT         (11U)
#define XCVR_TSM_OVRD1_ADC_EN_OVRD(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_EN_MASK  (0x1000U)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_EN_SHIFT (12U)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_MASK     (0x2000U)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_SHIFT    (13U)
#define XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_BIAS_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_EN_MASK   (0x4000U)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_EN_SHIFT  (14U)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_MASK      (0x8000U)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_SHIFT     (15U)
#define XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_CLK_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_EN_MASK (0x10000U)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_EN_SHIFT (16U)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_EN(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_MASK    (0x20000U)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_SHIFT   (17U)
#define XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_I_ADC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_EN_MASK (0x40000U)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_EN_SHIFT (18U)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_EN(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_MASK    (0x80000U)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_SHIFT   (19U)
#define XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_Q_ADC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_EN_MASK  (0x100000U)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_EN_SHIFT (20U)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_MASK     (0x200000U)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_SHIFT    (21U)
#define XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_DAC1_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_EN_MASK  (0x400000U)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_EN_SHIFT (22U)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_MASK     (0x800000U)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_SHIFT    (23U)
#define XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_DAC2_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_EN_MASK   (0x1000000U)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_EN_SHIFT  (24U)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_MASK      (0x2000000U)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_SHIFT     (25U)
#define XCVR_TSM_OVRD1_ADC_RST_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_ADC_RST_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD_EN_MASK     (0x4000000U)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD_EN_SHIFT    (26U)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_I_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_BBF_I_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD_MASK        (0x8000000U)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD_SHIFT       (27U)
#define XCVR_TSM_OVRD1_BBF_I_EN_OVRD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_I_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_BBF_I_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_EN_MASK     (0x10000000U)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_EN_SHIFT    (28U)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_MASK        (0x20000000U)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_SHIFT       (29U)
#define XCVR_TSM_OVRD1_BBF_Q_EN_OVRD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_BBF_Q_EN_OVRD_MASK)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_EN_MASK  (0x40000000U)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_EN_SHIFT (30U)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_MASK     (0x80000000U)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_SHIFT    (31U)
#define XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD1_BBF_PDET_EN_OVRD_MASK)

/*! @name TSM_OVRD2 - TSM Override 2 */
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_EN_MASK  (0x1U)
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_EN_SHIFT (0U)
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_MASK     (0x2U)
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_SHIFT    (1U)
#define XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_BBF_DCOC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD_EN_MASK       (0x4U)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD_EN_SHIFT      (2U)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TCA_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TCA_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD_MASK          (0x8U)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD_SHIFT         (3U)
#define XCVR_TSM_OVRD2_TCA_EN_OVRD(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TCA_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TCA_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD_EN_MASK     (0x10U)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD_EN_SHIFT    (4U)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_I_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TZA_I_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD_MASK        (0x20U)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD_SHIFT       (5U)
#define XCVR_TSM_OVRD2_TZA_I_EN_OVRD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_I_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TZA_I_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_EN_MASK     (0x40U)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_EN_SHIFT    (6U)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_EN(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_MASK        (0x80U)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_SHIFT       (7U)
#define XCVR_TSM_OVRD2_TZA_Q_EN_OVRD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TZA_Q_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_EN_MASK  (0x100U)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_EN_SHIFT (8U)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_MASK     (0x200U)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_SHIFT    (9U)
#define XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TZA_PDET_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_EN_MASK  (0x400U)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_EN_SHIFT (10U)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_MASK     (0x800U)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_SHIFT    (11U)
#define XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TZA_DCOC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_EN_MASK   (0x1000U)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_EN_SHIFT  (12U)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_EN(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_MASK      (0x2000U)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_SHIFT     (13U)
#define XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_PLL_DIG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_EN_MASK    (0x4000U)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_EN_SHIFT   (14U)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_MASK       (0x8000U)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_SHIFT      (15U)
#define XCVR_TSM_OVRD2_TX_DIG_EN_OVRD(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_TX_DIG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_EN_MASK    (0x10000U)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_EN_SHIFT   (16U)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_MASK       (0x20000U)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_SHIFT      (17U)
#define XCVR_TSM_OVRD2_RX_DIG_EN_OVRD(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_RX_DIG_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD_EN_MASK      (0x40000U)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD_EN_SHIFT     (18U)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_RX_INIT_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_RX_INIT_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD_MASK         (0x80000U)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD_SHIFT        (19U)
#define XCVR_TSM_OVRD2_RX_INIT_OVRD(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_RX_INIT_OVRD_SHIFT)) & XCVR_TSM_OVRD2_RX_INIT_OVRD_MASK)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_EN_MASK (0x100000U)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_EN_SHIFT (20U)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_MASK  (0x200000U)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_SHIFT (21U)
#define XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_SIGMA_DELTA_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_EN_MASK  (0x400000U)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_EN_SHIFT (22U)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_EN(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_MASK     (0x800000U)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_SHIFT    (23U)
#define XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_ZBDEM_RX_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD_EN_MASK      (0x1000000U)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD_EN_SHIFT     (24U)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_DCOC_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_DCOC_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD_MASK         (0x2000000U)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD_SHIFT        (25U)
#define XCVR_TSM_OVRD2_DCOC_EN_OVRD(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_DCOC_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_DCOC_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD_EN_MASK    (0x4000000U)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD_EN_SHIFT   (26U)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD_EN(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_DCOC_INIT_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_DCOC_INIT_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD_MASK       (0x8000000U)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD_SHIFT      (27U)
#define XCVR_TSM_OVRD2_DCOC_INIT_OVRD(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_DCOC_INIT_OVRD_SHIFT)) & XCVR_TSM_OVRD2_DCOC_INIT_OVRD_MASK)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_EN_MASK (0x10000000U)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_EN_SHIFT (28U)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_MASK (0x20000000U)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_SHIFT (29U)
#define XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_FREQ_TARG_LD_EN_OVRD_MASK)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_EN_MASK (0x40000000U)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_EN_SHIFT (30U)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_EN(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_MASK (0x80000000U)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_SHIFT (31U)
#define XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD2_SAR_ADC_TRIG_EN_OVRD_MASK)

/*! @name TSM_OVRD3 - TSM Override 3 */
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_EN_MASK (0x1U)
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_EN_SHIFT (0U)
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_MASK   (0x2U)
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_SHIFT  (1U)
#define XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE0_EN_OVRD_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_EN_MASK (0x4U)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_EN_SHIFT (2U)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_MASK   (0x8U)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_SHIFT  (3U)
#define XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE1_EN_OVRD_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_EN_MASK (0x10U)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_EN_SHIFT (4U)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_MASK   (0x20U)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_SHIFT  (5U)
#define XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE2_EN_OVRD_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_EN_MASK (0x40U)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_EN_SHIFT (6U)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_MASK   (0x80U)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_SHIFT  (7U)
#define XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_SHIFT)) & XCVR_TSM_OVRD3_TSM_SPARE3_EN_OVRD_MASK)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD_EN_MASK      (0x100U)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD_EN_SHIFT     (8U)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TX_MODE_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_TX_MODE_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD_MASK         (0x200U)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD_SHIFT        (9U)
#define XCVR_TSM_OVRD3_TX_MODE_OVRD(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_TX_MODE_OVRD_SHIFT)) & XCVR_TSM_OVRD3_TX_MODE_OVRD_MASK)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD_EN_MASK      (0x400U)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD_EN_SHIFT     (10U)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD_EN(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_RX_MODE_OVRD_EN_SHIFT)) & XCVR_TSM_OVRD3_RX_MODE_OVRD_EN_MASK)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD_MASK         (0x800U)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD_SHIFT        (11U)
#define XCVR_TSM_OVRD3_RX_MODE_OVRD(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_OVRD3_RX_MODE_OVRD_SHIFT)) & XCVR_TSM_OVRD3_RX_MODE_OVRD_MASK)

/*! @name PA_POWER - PA Power */
#define XCVR_PA_POWER_PA_POWER_MASK              (0xFU)
#define XCVR_PA_POWER_PA_POWER_SHIFT             (0U)
#define XCVR_PA_POWER_PA_POWER(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_PA_POWER_PA_POWER_SHIFT)) & XCVR_PA_POWER_PA_POWER_MASK)

/*! @name PA_BIAS_TBL0 - PA Bias Table 0 */
#define XCVR_PA_BIAS_TBL0_PA_BIAS0_MASK          (0xFU)
#define XCVR_PA_BIAS_TBL0_PA_BIAS0_SHIFT         (0U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS0(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL0_PA_BIAS0_SHIFT)) & XCVR_PA_BIAS_TBL0_PA_BIAS0_MASK)
#define XCVR_PA_BIAS_TBL0_PA_BIAS1_MASK          (0xF00U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS1_SHIFT         (8U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS1(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL0_PA_BIAS1_SHIFT)) & XCVR_PA_BIAS_TBL0_PA_BIAS1_MASK)
#define XCVR_PA_BIAS_TBL0_PA_BIAS2_MASK          (0xF0000U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS2_SHIFT         (16U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS2(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL0_PA_BIAS2_SHIFT)) & XCVR_PA_BIAS_TBL0_PA_BIAS2_MASK)
#define XCVR_PA_BIAS_TBL0_PA_BIAS3_MASK          (0xF000000U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS3_SHIFT         (24U)
#define XCVR_PA_BIAS_TBL0_PA_BIAS3(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL0_PA_BIAS3_SHIFT)) & XCVR_PA_BIAS_TBL0_PA_BIAS3_MASK)

/*! @name PA_BIAS_TBL1 - PA Bias Table 1 */
#define XCVR_PA_BIAS_TBL1_PA_BIAS4_MASK          (0xFU)
#define XCVR_PA_BIAS_TBL1_PA_BIAS4_SHIFT         (0U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS4(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL1_PA_BIAS4_SHIFT)) & XCVR_PA_BIAS_TBL1_PA_BIAS4_MASK)
#define XCVR_PA_BIAS_TBL1_PA_BIAS5_MASK          (0xF00U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS5_SHIFT         (8U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS5(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL1_PA_BIAS5_SHIFT)) & XCVR_PA_BIAS_TBL1_PA_BIAS5_MASK)
#define XCVR_PA_BIAS_TBL1_PA_BIAS6_MASK          (0xF0000U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS6_SHIFT         (16U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS6(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL1_PA_BIAS6_SHIFT)) & XCVR_PA_BIAS_TBL1_PA_BIAS6_MASK)
#define XCVR_PA_BIAS_TBL1_PA_BIAS7_MASK          (0xF000000U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS7_SHIFT         (24U)
#define XCVR_PA_BIAS_TBL1_PA_BIAS7(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PA_BIAS_TBL1_PA_BIAS7_SHIFT)) & XCVR_PA_BIAS_TBL1_PA_BIAS7_MASK)

/*! @name RECYCLE_COUNT - Recycle Count Register */
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT0_MASK   (0xFFU)
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT0_SHIFT  (0U)
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT0(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_RECYCLE_COUNT_RECYCLE_COUNT0_SHIFT)) & XCVR_RECYCLE_COUNT_RECYCLE_COUNT0_MASK)
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT1_MASK   (0xFF00U)
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT1_SHIFT  (8U)
#define XCVR_RECYCLE_COUNT_RECYCLE_COUNT1(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_RECYCLE_COUNT_RECYCLE_COUNT1_SHIFT)) & XCVR_RECYCLE_COUNT_RECYCLE_COUNT1_MASK)

/*! @name TSM_TIMING00 - TSM_TIMING00 */
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_HI_MASK  (0xFFU)
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING00_PLL_REG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING00_PLL_REG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_LO_MASK  (0xFF00U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_TX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING00_PLL_REG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING00_PLL_REG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING00_PLL_REG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING00_PLL_REG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING00_PLL_REG_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING00_PLL_REG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING00_PLL_REG_EN_RX_LO_MASK)

/*! @name TSM_TIMING01 - TSM_TIMING01 */
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING01_PLL_VCO_REG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING01_PLL_VCO_REG_EN_RX_LO_MASK)

/*! @name TSM_TIMING02 - TSM_TIMING02 */
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_HI(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING02_QGEN_REG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING02_QGEN_REG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_TX_LO(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING02_QGEN_REG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING02_QGEN_REG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_HI(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING02_QGEN_REG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING02_QGEN_REG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING02_QGEN_REG_EN_RX_LO(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING02_QGEN_REG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING02_QGEN_REG_EN_RX_LO_MASK)

/*! @name TSM_TIMING03 - TSM_TIMING03 */
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING03_TCA_TX_REG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING03_TCA_TX_REG_EN_RX_LO_MASK)

/*! @name TSM_TIMING04 - TSM_TIMING04 */
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING04_ADC_REG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING04_ADC_REG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING04_ADC_REG_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING04_ADC_REG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING04_ADC_REG_EN_RX_LO_MASK)

/*! @name TSM_TIMING05 - TSM_TIMING05 */
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING05_PLL_REF_CLK_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING05_PLL_REF_CLK_EN_RX_LO_MASK)

/*! @name TSM_TIMING06 - TSM_TIMING06 */
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING06_ADC_CLK_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING06_ADC_CLK_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING06_ADC_CLK_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING06_ADC_CLK_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING06_ADC_CLK_EN_RX_LO_MASK)

/*! @name TSM_TIMING07 - TSM_TIMING07 */
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING07_PLL_VCO_AUTOTUNE_EN_RX_LO_MASK)

/*! @name TSM_TIMING08 - TSM_TIMING08 */
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING08_PLL_CYCLE_SLIP_LD_EN_RX_LO_MASK)

/*! @name TSM_TIMING09 - TSM_TIMING09 */
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_HI_MASK  (0xFFU)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING09_PLL_VCO_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING09_PLL_VCO_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_LO_MASK  (0xFF00U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_TX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING09_PLL_VCO_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING09_PLL_VCO_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING09_PLL_VCO_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING09_PLL_VCO_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING09_PLL_VCO_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING09_PLL_VCO_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING09_PLL_VCO_EN_RX_LO_MASK)

/*! @name TSM_TIMING10 - TSM_TIMING10 */
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING10_PLL_VCO_BUF_RX_EN_RX_LO_MASK)

/*! @name TSM_TIMING11 - TSM_TIMING11 */
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING11_PLL_VCO_BUF_TX_EN_TX_LO_MASK)

/*! @name TSM_TIMING12 - TSM_TIMING12 */
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING12_PLL_PA_BUF_EN_TX_LO_MASK)

/*! @name TSM_TIMING13 - TSM_TIMING13 */
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_HI_MASK  (0xFFU)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING13_PLL_LDV_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING13_PLL_LDV_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_LO_MASK  (0xFF00U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_TX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING13_PLL_LDV_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING13_PLL_LDV_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING13_PLL_LDV_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING13_PLL_LDV_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING13_PLL_LDV_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING13_PLL_LDV_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING13_PLL_LDV_EN_RX_LO_MASK)

/*! @name TSM_TIMING14 - TSM_TIMING14 */
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING14_PLL_RX_LDV_RIPPLE_MUX_EN_RX_LO_MASK)

/*! @name TSM_TIMING15 - TSM_TIMING15 */
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING15_PLL_TX_LDV_RIPPLE_MUX_EN_TX_LO_MASK)

/*! @name TSM_TIMING16 - TSM_TIMING16 */
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING16_PLL_FILTER_CHARGE_EN_RX_LO_MASK)

/*! @name TSM_TIMING17 - TSM_TIMING17 */
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_HI(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_LO(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING17_PLL_PHDET_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_HI(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_LO(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING17_PLL_PHDET_EN_RX_LO_MASK)

/*! @name TSM_TIMING18 - TSM_TIMING18 */
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_HI_MASK   (0xFF0000U)
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_HI_SHIFT  (16U)
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_HI(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING18_QGEN25_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING18_QGEN25_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_LO_MASK   (0xFF000000U)
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_LO_SHIFT  (24U)
#define XCVR_TSM_TIMING18_QGEN25_EN_RX_LO(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING18_QGEN25_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING18_QGEN25_EN_RX_LO_MASK)

/*! @name TSM_TIMING19 - TSM_TIMING19 */
#define XCVR_TSM_TIMING19_TX_EN_TX_HI_MASK       (0xFFU)
#define XCVR_TSM_TIMING19_TX_EN_TX_HI_SHIFT      (0U)
#define XCVR_TSM_TIMING19_TX_EN_TX_HI(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING19_TX_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING19_TX_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING19_TX_EN_TX_LO_MASK       (0xFF00U)
#define XCVR_TSM_TIMING19_TX_EN_TX_LO_SHIFT      (8U)
#define XCVR_TSM_TIMING19_TX_EN_TX_LO(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING19_TX_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING19_TX_EN_TX_LO_MASK)

/*! @name TSM_TIMING20 - TSM_TIMING20 */
#define XCVR_TSM_TIMING20_ADC_EN_RX_HI_MASK      (0xFF0000U)
#define XCVR_TSM_TIMING20_ADC_EN_RX_HI_SHIFT     (16U)
#define XCVR_TSM_TIMING20_ADC_EN_RX_HI(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING20_ADC_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING20_ADC_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING20_ADC_EN_RX_LO_MASK      (0xFF000000U)
#define XCVR_TSM_TIMING20_ADC_EN_RX_LO_SHIFT     (24U)
#define XCVR_TSM_TIMING20_ADC_EN_RX_LO(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING20_ADC_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING20_ADC_EN_RX_LO_MASK)

/*! @name TSM_TIMING21 - TSM_TIMING21 */
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING21_ADC_I_Q_EN_RX_LO_MASK)

/*! @name TSM_TIMING22 - TSM_TIMING22 */
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING22_ADC_DAC_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING22_ADC_DAC_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING22_ADC_DAC_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING22_ADC_DAC_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING22_ADC_DAC_EN_RX_LO_MASK)

/*! @name TSM_TIMING23 - TSM_TIMING23 */
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING23_ADC_RST_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING23_ADC_RST_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING23_ADC_RST_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING23_ADC_RST_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING23_ADC_RST_EN_RX_LO_MASK)

/*! @name TSM_TIMING24 - TSM_TIMING24 */
#define XCVR_TSM_TIMING24_BBF_EN_RX_HI_MASK      (0xFF0000U)
#define XCVR_TSM_TIMING24_BBF_EN_RX_HI_SHIFT     (16U)
#define XCVR_TSM_TIMING24_BBF_EN_RX_HI(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING24_BBF_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING24_BBF_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING24_BBF_EN_RX_LO_MASK      (0xFF000000U)
#define XCVR_TSM_TIMING24_BBF_EN_RX_LO_SHIFT     (24U)
#define XCVR_TSM_TIMING24_BBF_EN_RX_LO(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING24_BBF_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING24_BBF_EN_RX_LO_MASK)

/*! @name TSM_TIMING25 - TSM_TIMING25 */
#define XCVR_TSM_TIMING25_TCA_EN_RX_HI_MASK      (0xFF0000U)
#define XCVR_TSM_TIMING25_TCA_EN_RX_HI_SHIFT     (16U)
#define XCVR_TSM_TIMING25_TCA_EN_RX_HI(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING25_TCA_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING25_TCA_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING25_TCA_EN_RX_LO_MASK      (0xFF000000U)
#define XCVR_TSM_TIMING25_TCA_EN_RX_LO_SHIFT     (24U)
#define XCVR_TSM_TIMING25_TCA_EN_RX_LO(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING25_TCA_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING25_TCA_EN_RX_LO_MASK)

/*! @name TSM_TIMING26 - TSM_TIMING26 */
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_HI_MASK  (0xFFU)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING26_PLL_DIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING26_PLL_DIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_LO_MASK  (0xFF00U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_TX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING26_PLL_DIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING26_PLL_DIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_HI_MASK  (0xFF0000U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_HI(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING26_PLL_DIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING26_PLL_DIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_LO_MASK  (0xFF000000U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING26_PLL_DIG_EN_RX_LO(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING26_PLL_DIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING26_PLL_DIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING27 - TSM_TIMING27 */
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_HI_MASK   (0xFFU)
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_HI_SHIFT  (0U)
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_HI(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING27_TX_DIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING27_TX_DIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_LO_MASK   (0xFF00U)
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_LO_SHIFT  (8U)
#define XCVR_TSM_TIMING27_TX_DIG_EN_TX_LO(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING27_TX_DIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING27_TX_DIG_EN_TX_LO_MASK)

/*! @name TSM_TIMING28 - TSM_TIMING28 */
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_HI_MASK   (0xFF0000U)
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_HI_SHIFT  (16U)
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_HI(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING28_RX_DIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING28_RX_DIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_LO_MASK   (0xFF000000U)
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_LO_SHIFT  (24U)
#define XCVR_TSM_TIMING28_RX_DIG_EN_RX_LO(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING28_RX_DIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING28_RX_DIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING29 - TSM_TIMING29 */
#define XCVR_TSM_TIMING29_RX_INIT_RX_HI_MASK     (0xFF0000U)
#define XCVR_TSM_TIMING29_RX_INIT_RX_HI_SHIFT    (16U)
#define XCVR_TSM_TIMING29_RX_INIT_RX_HI(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING29_RX_INIT_RX_HI_SHIFT)) & XCVR_TSM_TIMING29_RX_INIT_RX_HI_MASK)
#define XCVR_TSM_TIMING29_RX_INIT_RX_LO_MASK     (0xFF000000U)
#define XCVR_TSM_TIMING29_RX_INIT_RX_LO_SHIFT    (24U)
#define XCVR_TSM_TIMING29_RX_INIT_RX_LO(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING29_RX_INIT_RX_LO_SHIFT)) & XCVR_TSM_TIMING29_RX_INIT_RX_LO_MASK)

/*! @name TSM_TIMING30 - TSM_TIMING30 */
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING30_SIGMA_DELTA_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING30_SIGMA_DELTA_EN_RX_LO_MASK)

/*! @name TSM_TIMING31 - TSM_TIMING31 */
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_HI(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_LO(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING31_ZBDEM_RX_EN_RX_LO_MASK)

/*! @name TSM_TIMING32 - TSM_TIMING32 */
#define XCVR_TSM_TIMING32_DCOC_EN_RX_HI_MASK     (0xFF0000U)
#define XCVR_TSM_TIMING32_DCOC_EN_RX_HI_SHIFT    (16U)
#define XCVR_TSM_TIMING32_DCOC_EN_RX_HI(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING32_DCOC_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING32_DCOC_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING32_DCOC_EN_RX_LO_MASK     (0xFF000000U)
#define XCVR_TSM_TIMING32_DCOC_EN_RX_LO_SHIFT    (24U)
#define XCVR_TSM_TIMING32_DCOC_EN_RX_LO(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING32_DCOC_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING32_DCOC_EN_RX_LO_MASK)

/*! @name TSM_TIMING33 - TSM_TIMING33 */
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_HI_MASK   (0xFF0000U)
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_HI_SHIFT  (16U)
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_HI(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING33_DCOC_INIT_RX_HI_SHIFT)) & XCVR_TSM_TIMING33_DCOC_INIT_RX_HI_MASK)
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_LO_MASK   (0xFF000000U)
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_LO_SHIFT  (24U)
#define XCVR_TSM_TIMING33_DCOC_INIT_RX_LO(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING33_DCOC_INIT_RX_LO_SHIFT)) & XCVR_TSM_TIMING33_DCOC_INIT_RX_LO_MASK)

/*! @name TSM_TIMING34 - TSM_TIMING34 */
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING34_FREQ_TARG_LD_EN_RX_LO_MASK)

/*! @name TSM_TIMING35 - TSM_TIMING35 */
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING35_SAR_ADC_TRIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING36 - TSM_TIMING36 */
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING36_TSM_SPARE0_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING36_TSM_SPARE0_EN_RX_LO_MASK)

/*! @name TSM_TIMING37 - TSM_TIMING37 */
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING37_TSM_SPARE1_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING37_TSM_SPARE1_EN_RX_LO_MASK)

/*! @name TSM_TIMING38 - TSM_TIMING38 */
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING38_TSM_SPARE2_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING38_TSM_SPARE2_EN_RX_LO_MASK)

/*! @name TSM_TIMING39 - TSM_TIMING39 */
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING39_TSM_SPARE3_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING39_TSM_SPARE3_EN_RX_LO_MASK)

/*! @name TSM_TIMING40 - TSM_TIMING40 */
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING40_GPIO0_TRIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING40_GPIO0_TRIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING41 - TSM_TIMING41 */
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING41_GPIO1_TRIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING41_GPIO1_TRIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING42 - TSM_TIMING42 */
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING42_GPIO2_TRIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING42_GPIO2_TRIG_EN_RX_LO_MASK)

/*! @name TSM_TIMING43 - TSM_TIMING43 */
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_HI_MASK (0xFFU)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_HI_SHIFT (0U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_HI_SHIFT)) & XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_HI_MASK)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_LO_MASK (0xFF00U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_LO_SHIFT (8U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_LO_SHIFT)) & XCVR_TSM_TIMING43_GPIO3_TRIG_EN_TX_LO_MASK)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_HI_MASK (0xFF0000U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_HI_SHIFT (16U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_HI_SHIFT)) & XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_HI_MASK)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_LO_MASK (0xFF000000U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_LO_SHIFT (24U)
#define XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_LO_SHIFT)) & XCVR_TSM_TIMING43_GPIO3_TRIG_EN_RX_LO_MASK)

/*! @name CORR_CTRL - CORR_CTRL */
#define XCVR_CORR_CTRL_CORR_VT_MASK              (0xFFU)
#define XCVR_CORR_CTRL_CORR_VT_SHIFT             (0U)
#define XCVR_CORR_CTRL_CORR_VT(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_CORR_CTRL_CORR_VT_SHIFT)) & XCVR_CORR_CTRL_CORR_VT_MASK)
#define XCVR_CORR_CTRL_CORR_NVAL_MASK            (0x700U)
#define XCVR_CORR_CTRL_CORR_NVAL_SHIFT           (8U)
#define XCVR_CORR_CTRL_CORR_NVAL(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_CORR_CTRL_CORR_NVAL_SHIFT)) & XCVR_CORR_CTRL_CORR_NVAL_MASK)
#define XCVR_CORR_CTRL_MAX_CORR_EN_MASK          (0x800U)
#define XCVR_CORR_CTRL_MAX_CORR_EN_SHIFT         (11U)
#define XCVR_CORR_CTRL_MAX_CORR_EN(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_CORR_CTRL_MAX_CORR_EN_SHIFT)) & XCVR_CORR_CTRL_MAX_CORR_EN_MASK)
#define XCVR_CORR_CTRL_RX_MAX_CORR_MASK          (0xFF0000U)
#define XCVR_CORR_CTRL_RX_MAX_CORR_SHIFT         (16U)
#define XCVR_CORR_CTRL_RX_MAX_CORR(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_CORR_CTRL_RX_MAX_CORR_SHIFT)) & XCVR_CORR_CTRL_RX_MAX_CORR_MASK)
#define XCVR_CORR_CTRL_RX_MAX_PREAMBLE_MASK      (0xFF000000U)
#define XCVR_CORR_CTRL_RX_MAX_PREAMBLE_SHIFT     (24U)
#define XCVR_CORR_CTRL_RX_MAX_PREAMBLE(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_CORR_CTRL_RX_MAX_PREAMBLE_SHIFT)) & XCVR_CORR_CTRL_RX_MAX_PREAMBLE_MASK)

/*! @name PN_TYPE - PN_TYPE */
#define XCVR_PN_TYPE_PN_TYPE_MASK                (0x1U)
#define XCVR_PN_TYPE_PN_TYPE_SHIFT               (0U)
#define XCVR_PN_TYPE_PN_TYPE(x)                  (((uint32_t)(((uint32_t)(x)) << XCVR_PN_TYPE_PN_TYPE_SHIFT)) & XCVR_PN_TYPE_PN_TYPE_MASK)
#define XCVR_PN_TYPE_TX_INV_MASK                 (0x2U)
#define XCVR_PN_TYPE_TX_INV_SHIFT                (1U)
#define XCVR_PN_TYPE_TX_INV(x)                   (((uint32_t)(((uint32_t)(x)) << XCVR_PN_TYPE_TX_INV_SHIFT)) & XCVR_PN_TYPE_TX_INV_MASK)

/*! @name PN_CODE - PN_CODE */
#define XCVR_PN_CODE_PN_LSB_MASK                 (0xFFFFU)
#define XCVR_PN_CODE_PN_LSB_SHIFT                (0U)
#define XCVR_PN_CODE_PN_LSB(x)                   (((uint32_t)(((uint32_t)(x)) << XCVR_PN_CODE_PN_LSB_SHIFT)) & XCVR_PN_CODE_PN_LSB_MASK)
#define XCVR_PN_CODE_PN_MSB_MASK                 (0xFFFF0000U)
#define XCVR_PN_CODE_PN_MSB_SHIFT                (16U)
#define XCVR_PN_CODE_PN_MSB(x)                   (((uint32_t)(((uint32_t)(x)) << XCVR_PN_CODE_PN_MSB_SHIFT)) & XCVR_PN_CODE_PN_MSB_MASK)

/*! @name SYNC_CTRL - Sync Control */
#define XCVR_SYNC_CTRL_SYNC_PER_MASK             (0x7U)
#define XCVR_SYNC_CTRL_SYNC_PER_SHIFT            (0U)
#define XCVR_SYNC_CTRL_SYNC_PER(x)               (((uint32_t)(((uint32_t)(x)) << XCVR_SYNC_CTRL_SYNC_PER_SHIFT)) & XCVR_SYNC_CTRL_SYNC_PER_MASK)
#define XCVR_SYNC_CTRL_TRACK_ENABLE_MASK         (0x8U)
#define XCVR_SYNC_CTRL_TRACK_ENABLE_SHIFT        (3U)
#define XCVR_SYNC_CTRL_TRACK_ENABLE(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_SYNC_CTRL_TRACK_ENABLE_SHIFT)) & XCVR_SYNC_CTRL_TRACK_ENABLE_MASK)

/*! @name SNF_THR - SNF_THR */
#define XCVR_SNF_THR_SNF_THR_MASK                (0xFFU)
#define XCVR_SNF_THR_SNF_THR_SHIFT               (0U)
#define XCVR_SNF_THR_SNF_THR(x)                  (((uint32_t)(((uint32_t)(x)) << XCVR_SNF_THR_SNF_THR_SHIFT)) & XCVR_SNF_THR_SNF_THR_MASK)

/*! @name FAD_THR - FAD_THR */
#define XCVR_FAD_THR_FAD_THR_MASK                (0xFFU)
#define XCVR_FAD_THR_FAD_THR_SHIFT               (0U)
#define XCVR_FAD_THR_FAD_THR(x)                  (((uint32_t)(((uint32_t)(x)) << XCVR_FAD_THR_FAD_THR_SHIFT)) & XCVR_FAD_THR_FAD_THR_MASK)

/*! @name ZBDEM_AFC - ZBDEM_AFC */
#define XCVR_ZBDEM_AFC_AFC_EN_MASK               (0x1U)
#define XCVR_ZBDEM_AFC_AFC_EN_SHIFT              (0U)
#define XCVR_ZBDEM_AFC_AFC_EN(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_ZBDEM_AFC_AFC_EN_SHIFT)) & XCVR_ZBDEM_AFC_AFC_EN_MASK)
#define XCVR_ZBDEM_AFC_DCD_EN_MASK               (0x2U)
#define XCVR_ZBDEM_AFC_DCD_EN_SHIFT              (1U)
#define XCVR_ZBDEM_AFC_DCD_EN(x)                 (((uint32_t)(((uint32_t)(x)) << XCVR_ZBDEM_AFC_DCD_EN_SHIFT)) & XCVR_ZBDEM_AFC_DCD_EN_MASK)
#define XCVR_ZBDEM_AFC_AFC_OUT_MASK              (0x1F00U)
#define XCVR_ZBDEM_AFC_AFC_OUT_SHIFT             (8U)
#define XCVR_ZBDEM_AFC_AFC_OUT(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_ZBDEM_AFC_AFC_OUT_SHIFT)) & XCVR_ZBDEM_AFC_AFC_OUT_MASK)

/*! @name LPPS_CTRL - LPPS Control Register */
#define XCVR_LPPS_CTRL_LPPS_ENABLE_MASK          (0x1U)
#define XCVR_LPPS_CTRL_LPPS_ENABLE_SHIFT         (0U)
#define XCVR_LPPS_CTRL_LPPS_ENABLE(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_ENABLE_SHIFT)) & XCVR_LPPS_CTRL_LPPS_ENABLE_MASK)
#define XCVR_LPPS_CTRL_LPPS_QGEN25_ALLOW_MASK    (0x2U)
#define XCVR_LPPS_CTRL_LPPS_QGEN25_ALLOW_SHIFT   (1U)
#define XCVR_LPPS_CTRL_LPPS_QGEN25_ALLOW(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_QGEN25_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_QGEN25_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_ADC_ALLOW_MASK       (0x4U)
#define XCVR_LPPS_CTRL_LPPS_ADC_ALLOW_SHIFT      (2U)
#define XCVR_LPPS_CTRL_LPPS_ADC_ALLOW(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_ADC_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_ADC_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_ADC_CLK_ALLOW_MASK   (0x8U)
#define XCVR_LPPS_CTRL_LPPS_ADC_CLK_ALLOW_SHIFT  (3U)
#define XCVR_LPPS_CTRL_LPPS_ADC_CLK_ALLOW(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_ADC_CLK_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_ADC_CLK_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_ADC_I_Q_ALLOW_MASK   (0x10U)
#define XCVR_LPPS_CTRL_LPPS_ADC_I_Q_ALLOW_SHIFT  (4U)
#define XCVR_LPPS_CTRL_LPPS_ADC_I_Q_ALLOW(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_ADC_I_Q_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_ADC_I_Q_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_ADC_DAC_ALLOW_MASK   (0x20U)
#define XCVR_LPPS_CTRL_LPPS_ADC_DAC_ALLOW_SHIFT  (5U)
#define XCVR_LPPS_CTRL_LPPS_ADC_DAC_ALLOW(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_ADC_DAC_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_ADC_DAC_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_BBF_ALLOW_MASK       (0x40U)
#define XCVR_LPPS_CTRL_LPPS_BBF_ALLOW_SHIFT      (6U)
#define XCVR_LPPS_CTRL_LPPS_BBF_ALLOW(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_BBF_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_BBF_ALLOW_MASK)
#define XCVR_LPPS_CTRL_LPPS_TCA_ALLOW_MASK       (0x80U)
#define XCVR_LPPS_CTRL_LPPS_TCA_ALLOW_SHIFT      (7U)
#define XCVR_LPPS_CTRL_LPPS_TCA_ALLOW(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_LPPS_CTRL_LPPS_TCA_ALLOW_SHIFT)) & XCVR_LPPS_CTRL_LPPS_TCA_ALLOW_MASK)

/*! @name ADC_CTRL - ADC Control */
#define XCVR_ADC_CTRL_ADC_32MHZ_SEL_MASK         (0x1U)
#define XCVR_ADC_CTRL_ADC_32MHZ_SEL_SHIFT        (0U)
#define XCVR_ADC_CTRL_ADC_32MHZ_SEL(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_CTRL_ADC_32MHZ_SEL_SHIFT)) & XCVR_ADC_CTRL_ADC_32MHZ_SEL_MASK)
#define XCVR_ADC_CTRL_ADC_2X_CLK_SEL_MASK        (0x4U)
#define XCVR_ADC_CTRL_ADC_2X_CLK_SEL_SHIFT       (2U)
#define XCVR_ADC_CTRL_ADC_2X_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_CTRL_ADC_2X_CLK_SEL_SHIFT)) & XCVR_ADC_CTRL_ADC_2X_CLK_SEL_MASK)
#define XCVR_ADC_CTRL_ADC_DITHER_ON_MASK         (0x200U)
#define XCVR_ADC_CTRL_ADC_DITHER_ON_SHIFT        (9U)
#define XCVR_ADC_CTRL_ADC_DITHER_ON(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_CTRL_ADC_DITHER_ON_SHIFT)) & XCVR_ADC_CTRL_ADC_DITHER_ON_MASK)
#define XCVR_ADC_CTRL_ADC_TEST_ON_MASK           (0x400U)
#define XCVR_ADC_CTRL_ADC_TEST_ON_SHIFT          (10U)
#define XCVR_ADC_CTRL_ADC_TEST_ON(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_CTRL_ADC_TEST_ON_SHIFT)) & XCVR_ADC_CTRL_ADC_TEST_ON_MASK)
#define XCVR_ADC_CTRL_ADC_COMP_ON_MASK           (0xFFFF0000U)
#define XCVR_ADC_CTRL_ADC_COMP_ON_SHIFT          (16U)
#define XCVR_ADC_CTRL_ADC_COMP_ON(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_CTRL_ADC_COMP_ON_SHIFT)) & XCVR_ADC_CTRL_ADC_COMP_ON_MASK)

/*! @name ADC_TUNE - ADC Tuning */
#define XCVR_ADC_TUNE_ADC_R1_TUNE_MASK           (0x7U)
#define XCVR_ADC_TUNE_ADC_R1_TUNE_SHIFT          (0U)
#define XCVR_ADC_TUNE_ADC_R1_TUNE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TUNE_ADC_R1_TUNE_SHIFT)) & XCVR_ADC_TUNE_ADC_R1_TUNE_MASK)
#define XCVR_ADC_TUNE_ADC_R2_TUNE_MASK           (0x70U)
#define XCVR_ADC_TUNE_ADC_R2_TUNE_SHIFT          (4U)
#define XCVR_ADC_TUNE_ADC_R2_TUNE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TUNE_ADC_R2_TUNE_SHIFT)) & XCVR_ADC_TUNE_ADC_R2_TUNE_MASK)
#define XCVR_ADC_TUNE_ADC_C1_TUNE_MASK           (0xF0000U)
#define XCVR_ADC_TUNE_ADC_C1_TUNE_SHIFT          (16U)
#define XCVR_ADC_TUNE_ADC_C1_TUNE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TUNE_ADC_C1_TUNE_SHIFT)) & XCVR_ADC_TUNE_ADC_C1_TUNE_MASK)
#define XCVR_ADC_TUNE_ADC_C2_TUNE_MASK           (0xF00000U)
#define XCVR_ADC_TUNE_ADC_C2_TUNE_SHIFT          (20U)
#define XCVR_ADC_TUNE_ADC_C2_TUNE(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TUNE_ADC_C2_TUNE_SHIFT)) & XCVR_ADC_TUNE_ADC_C2_TUNE_MASK)

/*! @name ADC_ADJ - ADC Adjustment */
#define XCVR_ADC_ADJ_ADC_IB_OPAMP1_ADJ_MASK      (0x7U)
#define XCVR_ADC_ADJ_ADC_IB_OPAMP1_ADJ_SHIFT     (0U)
#define XCVR_ADC_ADJ_ADC_IB_OPAMP1_ADJ(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_IB_OPAMP1_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_IB_OPAMP1_ADJ_MASK)
#define XCVR_ADC_ADJ_ADC_IB_OPAMP2_ADJ_MASK      (0x70U)
#define XCVR_ADC_ADJ_ADC_IB_OPAMP2_ADJ_SHIFT     (4U)
#define XCVR_ADC_ADJ_ADC_IB_OPAMP2_ADJ(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_IB_OPAMP2_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_IB_OPAMP2_ADJ_MASK)
#define XCVR_ADC_ADJ_ADC_IB_DAC1_ADJ_MASK        (0x7000U)
#define XCVR_ADC_ADJ_ADC_IB_DAC1_ADJ_SHIFT       (12U)
#define XCVR_ADC_ADJ_ADC_IB_DAC1_ADJ(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_IB_DAC1_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_IB_DAC1_ADJ_MASK)
#define XCVR_ADC_ADJ_ADC_IB_DAC2_ADJ_MASK        (0x70000U)
#define XCVR_ADC_ADJ_ADC_IB_DAC2_ADJ_SHIFT       (16U)
#define XCVR_ADC_ADJ_ADC_IB_DAC2_ADJ(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_IB_DAC2_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_IB_DAC2_ADJ_MASK)
#define XCVR_ADC_ADJ_ADC_IB_FLSH_ADJ_MASK        (0x7000000U)
#define XCVR_ADC_ADJ_ADC_IB_FLSH_ADJ_SHIFT       (24U)
#define XCVR_ADC_ADJ_ADC_IB_FLSH_ADJ(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_IB_FLSH_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_IB_FLSH_ADJ_MASK)
#define XCVR_ADC_ADJ_ADC_FLSH_RES_ADJ_MASK       (0x70000000U)
#define XCVR_ADC_ADJ_ADC_FLSH_RES_ADJ_SHIFT      (28U)
#define XCVR_ADC_ADJ_ADC_FLSH_RES_ADJ(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_ADJ_ADC_FLSH_RES_ADJ_SHIFT)) & XCVR_ADC_ADJ_ADC_FLSH_RES_ADJ_MASK)

/*! @name ADC_REGS - ADC Regulators */
#define XCVR_ADC_REGS_ADC_ANA_REG_SUPPLY_MASK    (0xFU)
#define XCVR_ADC_REGS_ADC_ANA_REG_SUPPLY_SHIFT   (0U)
#define XCVR_ADC_REGS_ADC_ANA_REG_SUPPLY(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_ANA_REG_SUPPLY_SHIFT)) & XCVR_ADC_REGS_ADC_ANA_REG_SUPPLY_MASK)
#define XCVR_ADC_REGS_ADC_REG_DIG_SUPPLY_MASK    (0xF0U)
#define XCVR_ADC_REGS_ADC_REG_DIG_SUPPLY_SHIFT   (4U)
#define XCVR_ADC_REGS_ADC_REG_DIG_SUPPLY(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_REG_DIG_SUPPLY_SHIFT)) & XCVR_ADC_REGS_ADC_REG_DIG_SUPPLY_MASK)
#define XCVR_ADC_REGS_ADC_ANA_REG_BYPASS_ON_MASK (0x100U)
#define XCVR_ADC_REGS_ADC_ANA_REG_BYPASS_ON_SHIFT (8U)
#define XCVR_ADC_REGS_ADC_ANA_REG_BYPASS_ON(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_ANA_REG_BYPASS_ON_SHIFT)) & XCVR_ADC_REGS_ADC_ANA_REG_BYPASS_ON_MASK)
#define XCVR_ADC_REGS_ADC_DIG_REG_BYPASS_ON_MASK (0x200U)
#define XCVR_ADC_REGS_ADC_DIG_REG_BYPASS_ON_SHIFT (9U)
#define XCVR_ADC_REGS_ADC_DIG_REG_BYPASS_ON(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_DIG_REG_BYPASS_ON_SHIFT)) & XCVR_ADC_REGS_ADC_DIG_REG_BYPASS_ON_MASK)
#define XCVR_ADC_REGS_ADC_VCMREF_BYPASS_ON_MASK  (0x8000U)
#define XCVR_ADC_REGS_ADC_VCMREF_BYPASS_ON_SHIFT (15U)
#define XCVR_ADC_REGS_ADC_VCMREF_BYPASS_ON(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_VCMREF_BYPASS_ON_SHIFT)) & XCVR_ADC_REGS_ADC_VCMREF_BYPASS_ON_MASK)
#define XCVR_ADC_REGS_ADC_INTERNAL_IREF_BYPASS_ON_MASK (0x20000U)
#define XCVR_ADC_REGS_ADC_INTERNAL_IREF_BYPASS_ON_SHIFT (17U)
#define XCVR_ADC_REGS_ADC_INTERNAL_IREF_BYPASS_ON(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_REGS_ADC_INTERNAL_IREF_BYPASS_ON_SHIFT)) & XCVR_ADC_REGS_ADC_INTERNAL_IREF_BYPASS_ON_MASK)

/*! @name ADC_TRIMS - ADC Regulator Trims */
#define XCVR_ADC_TRIMS_ADC_IREF_OPAMPS_RES_TRIM_MASK (0x7U)
#define XCVR_ADC_TRIMS_ADC_IREF_OPAMPS_RES_TRIM_SHIFT (0U)
#define XCVR_ADC_TRIMS_ADC_IREF_OPAMPS_RES_TRIM(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TRIMS_ADC_IREF_OPAMPS_RES_TRIM_SHIFT)) & XCVR_ADC_TRIMS_ADC_IREF_OPAMPS_RES_TRIM_MASK)
#define XCVR_ADC_TRIMS_ADC_IREF_FLSH_RES_TRIM_MASK (0x70U)
#define XCVR_ADC_TRIMS_ADC_IREF_FLSH_RES_TRIM_SHIFT (4U)
#define XCVR_ADC_TRIMS_ADC_IREF_FLSH_RES_TRIM(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TRIMS_ADC_IREF_FLSH_RES_TRIM_SHIFT)) & XCVR_ADC_TRIMS_ADC_IREF_FLSH_RES_TRIM_MASK)
#define XCVR_ADC_TRIMS_ADC_VCM_TRIM_MASK         (0x700U)
#define XCVR_ADC_TRIMS_ADC_VCM_TRIM_SHIFT        (8U)
#define XCVR_ADC_TRIMS_ADC_VCM_TRIM(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TRIMS_ADC_VCM_TRIM_SHIFT)) & XCVR_ADC_TRIMS_ADC_VCM_TRIM_MASK)

/*! @name ADC_TEST_CTRL - ADC Test Control */
#define XCVR_ADC_TEST_CTRL_ADC_ATST_SEL_MASK     (0x1FU)
#define XCVR_ADC_TEST_CTRL_ADC_ATST_SEL_SHIFT    (0U)
#define XCVR_ADC_TEST_CTRL_ADC_ATST_SEL(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TEST_CTRL_ADC_ATST_SEL_SHIFT)) & XCVR_ADC_TEST_CTRL_ADC_ATST_SEL_MASK)
#define XCVR_ADC_TEST_CTRL_ADC_DIG_REG_ATST_SEL_MASK (0x300U)
#define XCVR_ADC_TEST_CTRL_ADC_DIG_REG_ATST_SEL_SHIFT (8U)
#define XCVR_ADC_TEST_CTRL_ADC_DIG_REG_ATST_SEL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TEST_CTRL_ADC_DIG_REG_ATST_SEL_SHIFT)) & XCVR_ADC_TEST_CTRL_ADC_DIG_REG_ATST_SEL_MASK)
#define XCVR_ADC_TEST_CTRL_ADC_ANA_REG_ATST_SEL_MASK (0x3000U)
#define XCVR_ADC_TEST_CTRL_ADC_ANA_REG_ATST_SEL_SHIFT (12U)
#define XCVR_ADC_TEST_CTRL_ADC_ANA_REG_ATST_SEL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TEST_CTRL_ADC_ANA_REG_ATST_SEL_SHIFT)) & XCVR_ADC_TEST_CTRL_ADC_ANA_REG_ATST_SEL_MASK)
#define XCVR_ADC_TEST_CTRL_DCOC_ALPHA_RADIUS_GS_IDX_MASK (0x7000000U)
#define XCVR_ADC_TEST_CTRL_DCOC_ALPHA_RADIUS_GS_IDX_SHIFT (24U)
#define XCVR_ADC_TEST_CTRL_DCOC_ALPHA_RADIUS_GS_IDX(x) (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TEST_CTRL_DCOC_ALPHA_RADIUS_GS_IDX_SHIFT)) & XCVR_ADC_TEST_CTRL_DCOC_ALPHA_RADIUS_GS_IDX_MASK)
#define XCVR_ADC_TEST_CTRL_ADC_SPARE3_MASK       (0x8000000U)
#define XCVR_ADC_TEST_CTRL_ADC_SPARE3_SHIFT      (27U)
#define XCVR_ADC_TEST_CTRL_ADC_SPARE3(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_ADC_TEST_CTRL_ADC_SPARE3_SHIFT)) & XCVR_ADC_TEST_CTRL_ADC_SPARE3_MASK)

/*! @name BBF_CTRL - Baseband Filter Control */
#define XCVR_BBF_CTRL_BBF_CAP_TUNE_MASK          (0xFU)
#define XCVR_BBF_CTRL_BBF_CAP_TUNE_SHIFT         (0U)
#define XCVR_BBF_CTRL_BBF_CAP_TUNE(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_CAP_TUNE_SHIFT)) & XCVR_BBF_CTRL_BBF_CAP_TUNE_MASK)
#define XCVR_BBF_CTRL_BBF_RES_TUNE2_MASK         (0xF0U)
#define XCVR_BBF_CTRL_BBF_RES_TUNE2_SHIFT        (4U)
#define XCVR_BBF_CTRL_BBF_RES_TUNE2(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_RES_TUNE2_SHIFT)) & XCVR_BBF_CTRL_BBF_RES_TUNE2_MASK)
#define XCVR_BBF_CTRL_BBF_CUR_CNTL_MASK          (0x100U)
#define XCVR_BBF_CTRL_BBF_CUR_CNTL_SHIFT         (8U)
#define XCVR_BBF_CTRL_BBF_CUR_CNTL(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_CUR_CNTL_SHIFT)) & XCVR_BBF_CTRL_BBF_CUR_CNTL_MASK)
#define XCVR_BBF_CTRL_BBF_DCOC_ON_MASK           (0x200U)
#define XCVR_BBF_CTRL_BBF_DCOC_ON_SHIFT          (9U)
#define XCVR_BBF_CTRL_BBF_DCOC_ON(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_DCOC_ON_SHIFT)) & XCVR_BBF_CTRL_BBF_DCOC_ON_MASK)
#define XCVR_BBF_CTRL_BBF_TMUX_ON_MASK           (0x800U)
#define XCVR_BBF_CTRL_BBF_TMUX_ON_SHIFT          (11U)
#define XCVR_BBF_CTRL_BBF_TMUX_ON(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_TMUX_ON_SHIFT)) & XCVR_BBF_CTRL_BBF_TMUX_ON_MASK)
#define XCVR_BBF_CTRL_DCOC_ALPHAC_SCALE_GS_IDX_MASK (0x3000U)
#define XCVR_BBF_CTRL_DCOC_ALPHAC_SCALE_GS_IDX_SHIFT (12U)
#define XCVR_BBF_CTRL_DCOC_ALPHAC_SCALE_GS_IDX(x) (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_DCOC_ALPHAC_SCALE_GS_IDX_SHIFT)) & XCVR_BBF_CTRL_DCOC_ALPHAC_SCALE_GS_IDX_MASK)
#define XCVR_BBF_CTRL_BBF_SPARE_3_2_MASK         (0xC000U)
#define XCVR_BBF_CTRL_BBF_SPARE_3_2_SHIFT        (14U)
#define XCVR_BBF_CTRL_BBF_SPARE_3_2(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_BBF_CTRL_BBF_SPARE_3_2_SHIFT)) & XCVR_BBF_CTRL_BBF_SPARE_3_2_MASK)

/*! @name RX_ANA_CTRL - RX Analog Control */
#define XCVR_RX_ANA_CTRL_RX_ATST_SEL_MASK        (0xFU)
#define XCVR_RX_ANA_CTRL_RX_ATST_SEL_SHIFT       (0U)
#define XCVR_RX_ANA_CTRL_RX_ATST_SEL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_RX_ANA_CTRL_RX_ATST_SEL_SHIFT)) & XCVR_RX_ANA_CTRL_RX_ATST_SEL_MASK)
#define XCVR_RX_ANA_CTRL_IQMC_DC_GAIN_ADJ_EN_MASK (0x10U)
#define XCVR_RX_ANA_CTRL_IQMC_DC_GAIN_ADJ_EN_SHIFT (4U)
#define XCVR_RX_ANA_CTRL_IQMC_DC_GAIN_ADJ_EN(x)  (((uint32_t)(((uint32_t)(x)) << XCVR_RX_ANA_CTRL_IQMC_DC_GAIN_ADJ_EN_SHIFT)) & XCVR_RX_ANA_CTRL_IQMC_DC_GAIN_ADJ_EN_MASK)
#define XCVR_RX_ANA_CTRL_LNM_SPARE_3_2_1_MASK    (0xE0U)
#define XCVR_RX_ANA_CTRL_LNM_SPARE_3_2_1_SHIFT   (5U)
#define XCVR_RX_ANA_CTRL_LNM_SPARE_3_2_1(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RX_ANA_CTRL_LNM_SPARE_3_2_1_SHIFT)) & XCVR_RX_ANA_CTRL_LNM_SPARE_3_2_1_MASK)

/*! @name XTAL_CTRL - Crystal Oscillator Control Register 1 */
#define XCVR_XTAL_CTRL_XTAL_TRIM_MASK            (0xFFU)
#define XCVR_XTAL_CTRL_XTAL_TRIM_SHIFT           (0U)
#define XCVR_XTAL_CTRL_XTAL_TRIM(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_TRIM_SHIFT)) & XCVR_XTAL_CTRL_XTAL_TRIM_MASK)
#define XCVR_XTAL_CTRL_XTAL_GM_MASK              (0x1F00U)
#define XCVR_XTAL_CTRL_XTAL_GM_SHIFT             (8U)
#define XCVR_XTAL_CTRL_XTAL_GM(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_GM_SHIFT)) & XCVR_XTAL_CTRL_XTAL_GM_MASK)
#define XCVR_XTAL_CTRL_XTAL_BYPASS_MASK          (0x2000U)
#define XCVR_XTAL_CTRL_XTAL_BYPASS_SHIFT         (13U)
#define XCVR_XTAL_CTRL_XTAL_BYPASS(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_BYPASS_SHIFT)) & XCVR_XTAL_CTRL_XTAL_BYPASS_MASK)
#define XCVR_XTAL_CTRL_XTAL_READY_COUNT_SEL_MASK (0xC000U)
#define XCVR_XTAL_CTRL_XTAL_READY_COUNT_SEL_SHIFT (14U)
#define XCVR_XTAL_CTRL_XTAL_READY_COUNT_SEL(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_READY_COUNT_SEL_SHIFT)) & XCVR_XTAL_CTRL_XTAL_READY_COUNT_SEL_MASK)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_LO_MASK    (0x1F0000U)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_LO_SHIFT   (16U)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_LO(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_COMP_BIAS_LO_SHIFT)) & XCVR_XTAL_CTRL_XTAL_COMP_BIAS_LO_MASK)
#define XCVR_XTAL_CTRL_XTAL_ALC_START_512U_MASK  (0x400000U)
#define XCVR_XTAL_CTRL_XTAL_ALC_START_512U_SHIFT (22U)
#define XCVR_XTAL_CTRL_XTAL_ALC_START_512U(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_ALC_START_512U_SHIFT)) & XCVR_XTAL_CTRL_XTAL_ALC_START_512U_MASK)
#define XCVR_XTAL_CTRL_XTAL_ALC_ON_MASK          (0x800000U)
#define XCVR_XTAL_CTRL_XTAL_ALC_ON_SHIFT         (23U)
#define XCVR_XTAL_CTRL_XTAL_ALC_ON(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_ALC_ON_SHIFT)) & XCVR_XTAL_CTRL_XTAL_ALC_ON_MASK)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_HI_MASK    (0x1F000000U)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_HI_SHIFT   (24U)
#define XCVR_XTAL_CTRL_XTAL_COMP_BIAS_HI(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_COMP_BIAS_HI_SHIFT)) & XCVR_XTAL_CTRL_XTAL_COMP_BIAS_HI_MASK)
#define XCVR_XTAL_CTRL_XTAL_READY_MASK           (0x80000000U)
#define XCVR_XTAL_CTRL_XTAL_READY_SHIFT          (31U)
#define XCVR_XTAL_CTRL_XTAL_READY(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL_XTAL_READY_SHIFT)) & XCVR_XTAL_CTRL_XTAL_READY_MASK)

/*! @name XTAL_CTRL2 - Crystal Oscillator Control Register 2 */
#define XCVR_XTAL_CTRL2_XTAL_REG_SUPPLY_MASK     (0xFU)
#define XCVR_XTAL_CTRL2_XTAL_REG_SUPPLY_SHIFT    (0U)
#define XCVR_XTAL_CTRL2_XTAL_REG_SUPPLY(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_REG_SUPPLY_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_REG_SUPPLY_MASK)
#define XCVR_XTAL_CTRL2_XTAL_REG_BYPASS_ON_MASK  (0x10U)
#define XCVR_XTAL_CTRL2_XTAL_REG_BYPASS_ON_SHIFT (4U)
#define XCVR_XTAL_CTRL2_XTAL_REG_BYPASS_ON(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_REG_BYPASS_ON_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_REG_BYPASS_ON_MASK)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_ON_MASK (0x100U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_ON_SHIFT (8U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_ON(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_ON_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_ON_MASK)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_MASK    (0x200U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_SHIFT   (9U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_REG_ON_OVRD_MASK)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD_ON_MASK     (0x400U)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD_ON_SHIFT    (10U)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD_ON(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_ON_OVRD_ON_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_ON_OVRD_ON_MASK)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD_MASK        (0x800U)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD_SHIFT       (11U)
#define XCVR_XTAL_CTRL2_XTAL_ON_OVRD(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_ON_OVRD_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_ON_OVRD_MASK)
#define XCVR_XTAL_CTRL2_XTAL_DIG_CLK_OUT_ON_MASK (0x1000U)
#define XCVR_XTAL_CTRL2_XTAL_DIG_CLK_OUT_ON_SHIFT (12U)
#define XCVR_XTAL_CTRL2_XTAL_DIG_CLK_OUT_ON(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_DIG_CLK_OUT_ON_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_DIG_CLK_OUT_ON_MASK)
#define XCVR_XTAL_CTRL2_XTAL_REG_ATST_SEL_MASK   (0x30000U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ATST_SEL_SHIFT  (16U)
#define XCVR_XTAL_CTRL2_XTAL_REG_ATST_SEL(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_REG_ATST_SEL_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_REG_ATST_SEL_MASK)
#define XCVR_XTAL_CTRL2_XTAL_ATST_SEL_MASK       (0x3000000U)
#define XCVR_XTAL_CTRL2_XTAL_ATST_SEL_SHIFT      (24U)
#define XCVR_XTAL_CTRL2_XTAL_ATST_SEL(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_ATST_SEL_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_ATST_SEL_MASK)
#define XCVR_XTAL_CTRL2_XTAL_ATST_ON_MASK        (0x4000000U)
#define XCVR_XTAL_CTRL2_XTAL_ATST_ON_SHIFT       (26U)
#define XCVR_XTAL_CTRL2_XTAL_ATST_ON(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_ATST_ON_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_ATST_ON_MASK)
#define XCVR_XTAL_CTRL2_XTAL_SPARE_MASK          (0xF0000000U)
#define XCVR_XTAL_CTRL2_XTAL_SPARE_SHIFT         (28U)
#define XCVR_XTAL_CTRL2_XTAL_SPARE(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_XTAL_CTRL2_XTAL_SPARE_SHIFT)) & XCVR_XTAL_CTRL2_XTAL_SPARE_MASK)

/*! @name BGAP_CTRL - Bandgap Control */
#define XCVR_BGAP_CTRL_BGAP_CURRENT_TRIM_MASK    (0xFU)
#define XCVR_BGAP_CTRL_BGAP_CURRENT_TRIM_SHIFT   (0U)
#define XCVR_BGAP_CTRL_BGAP_CURRENT_TRIM(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_BGAP_CTRL_BGAP_CURRENT_TRIM_SHIFT)) & XCVR_BGAP_CTRL_BGAP_CURRENT_TRIM_MASK)
#define XCVR_BGAP_CTRL_BGAP_VOLTAGE_TRIM_MASK    (0xF0U)
#define XCVR_BGAP_CTRL_BGAP_VOLTAGE_TRIM_SHIFT   (4U)
#define XCVR_BGAP_CTRL_BGAP_VOLTAGE_TRIM(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_BGAP_CTRL_BGAP_VOLTAGE_TRIM_SHIFT)) & XCVR_BGAP_CTRL_BGAP_VOLTAGE_TRIM_MASK)
#define XCVR_BGAP_CTRL_BGAP_ATST_SEL_MASK        (0xF00U)
#define XCVR_BGAP_CTRL_BGAP_ATST_SEL_SHIFT       (8U)
#define XCVR_BGAP_CTRL_BGAP_ATST_SEL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_BGAP_CTRL_BGAP_ATST_SEL_SHIFT)) & XCVR_BGAP_CTRL_BGAP_ATST_SEL_MASK)
#define XCVR_BGAP_CTRL_BGAP_ATST_ON_MASK         (0x1000U)
#define XCVR_BGAP_CTRL_BGAP_ATST_ON_SHIFT        (12U)
#define XCVR_BGAP_CTRL_BGAP_ATST_ON(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_BGAP_CTRL_BGAP_ATST_ON_SHIFT)) & XCVR_BGAP_CTRL_BGAP_ATST_ON_MASK)

/*! @name PLL_CTRL - PLL Control Register */
#define XCVR_PLL_CTRL_PLL_VCO_BIAS_MASK          (0x7U)
#define XCVR_PLL_CTRL_PLL_VCO_BIAS_SHIFT         (0U)
#define XCVR_PLL_CTRL_PLL_VCO_BIAS(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_VCO_BIAS_SHIFT)) & XCVR_PLL_CTRL_PLL_VCO_BIAS_MASK)
#define XCVR_PLL_CTRL_PLL_LFILT_CNTL_MASK        (0x70U)
#define XCVR_PLL_CTRL_PLL_LFILT_CNTL_SHIFT       (4U)
#define XCVR_PLL_CTRL_PLL_LFILT_CNTL(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_LFILT_CNTL_SHIFT)) & XCVR_PLL_CTRL_PLL_LFILT_CNTL_MASK)
#define XCVR_PLL_CTRL_PLL_REG_SUPPLY_MASK        (0xF00U)
#define XCVR_PLL_CTRL_PLL_REG_SUPPLY_SHIFT       (8U)
#define XCVR_PLL_CTRL_PLL_REG_SUPPLY(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_REG_SUPPLY_SHIFT)) & XCVR_PLL_CTRL_PLL_REG_SUPPLY_MASK)
#define XCVR_PLL_CTRL_PLL_REG_BYPASS_ON_MASK     (0x10000U)
#define XCVR_PLL_CTRL_PLL_REG_BYPASS_ON_SHIFT    (16U)
#define XCVR_PLL_CTRL_PLL_REG_BYPASS_ON(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_REG_BYPASS_ON_SHIFT)) & XCVR_PLL_CTRL_PLL_REG_BYPASS_ON_MASK)
#define XCVR_PLL_CTRL_PLL_VCO_LDO_BYPASS_MASK    (0x20000U)
#define XCVR_PLL_CTRL_PLL_VCO_LDO_BYPASS_SHIFT   (17U)
#define XCVR_PLL_CTRL_PLL_VCO_LDO_BYPASS(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_VCO_LDO_BYPASS_SHIFT)) & XCVR_PLL_CTRL_PLL_VCO_LDO_BYPASS_MASK)
#define XCVR_PLL_CTRL_HPM_BIAS_MASK              (0x7F000000U)
#define XCVR_PLL_CTRL_HPM_BIAS_SHIFT             (24U)
#define XCVR_PLL_CTRL_HPM_BIAS(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_HPM_BIAS_SHIFT)) & XCVR_PLL_CTRL_HPM_BIAS_MASK)
#define XCVR_PLL_CTRL_PLL_VCO_SPARE7_MASK        (0x80000000U)
#define XCVR_PLL_CTRL_PLL_VCO_SPARE7_SHIFT       (31U)
#define XCVR_PLL_CTRL_PLL_VCO_SPARE7(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL_PLL_VCO_SPARE7_SHIFT)) & XCVR_PLL_CTRL_PLL_VCO_SPARE7_MASK)

/*! @name PLL_CTRL2 - PLL Control Register 2 */
#define XCVR_PLL_CTRL2_PLL_VCO_KV_MASK           (0x7U)
#define XCVR_PLL_CTRL2_PLL_VCO_KV_SHIFT          (0U)
#define XCVR_PLL_CTRL2_PLL_VCO_KV(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL2_PLL_VCO_KV_SHIFT)) & XCVR_PLL_CTRL2_PLL_VCO_KV_MASK)
#define XCVR_PLL_CTRL2_PLL_KMOD_SLOPE_MASK       (0x8U)
#define XCVR_PLL_CTRL2_PLL_KMOD_SLOPE_SHIFT      (3U)
#define XCVR_PLL_CTRL2_PLL_KMOD_SLOPE(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL2_PLL_KMOD_SLOPE_SHIFT)) & XCVR_PLL_CTRL2_PLL_KMOD_SLOPE_MASK)
#define XCVR_PLL_CTRL2_PLL_VCO_REG_SUPPLY_MASK   (0x30U)
#define XCVR_PLL_CTRL2_PLL_VCO_REG_SUPPLY_SHIFT  (4U)
#define XCVR_PLL_CTRL2_PLL_VCO_REG_SUPPLY(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL2_PLL_VCO_REG_SUPPLY_SHIFT)) & XCVR_PLL_CTRL2_PLL_VCO_REG_SUPPLY_MASK)
#define XCVR_PLL_CTRL2_PLL_TMUX_ON_MASK          (0x100U)
#define XCVR_PLL_CTRL2_PLL_TMUX_ON_SHIFT         (8U)
#define XCVR_PLL_CTRL2_PLL_TMUX_ON(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_CTRL2_PLL_TMUX_ON_SHIFT)) & XCVR_PLL_CTRL2_PLL_TMUX_ON_MASK)

/*! @name PLL_TEST_CTRL - PLL Test Control */
#define XCVR_PLL_TEST_CTRL_PLL_TMUX_SEL_MASK     (0x3U)
#define XCVR_PLL_TEST_CTRL_PLL_TMUX_SEL_SHIFT    (0U)
#define XCVR_PLL_TEST_CTRL_PLL_TMUX_SEL(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_TMUX_SEL_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_TMUX_SEL_MASK)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_REG_ATST_MASK (0x30U)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_REG_ATST_SHIFT (4U)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_REG_ATST(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_VCO_REG_ATST_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_VCO_REG_ATST_MASK)
#define XCVR_PLL_TEST_CTRL_PLL_REG_ATST_SEL_MASK (0x300U)
#define XCVR_PLL_TEST_CTRL_PLL_REG_ATST_SEL_SHIFT (8U)
#define XCVR_PLL_TEST_CTRL_PLL_REG_ATST_SEL(x)   (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_REG_ATST_SEL_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_REG_ATST_SEL_MASK)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_TEST_CLK_MODE_MASK (0x1000U)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_TEST_CLK_MODE_SHIFT (12U)
#define XCVR_PLL_TEST_CTRL_PLL_VCO_TEST_CLK_MODE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_VCO_TEST_CLK_MODE_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_VCO_TEST_CLK_MODE_MASK)
#define XCVR_PLL_TEST_CTRL_PLL_FORCE_VTUNE_EXTERNALLY_MASK (0x2000U)
#define XCVR_PLL_TEST_CTRL_PLL_FORCE_VTUNE_EXTERNALLY_SHIFT (13U)
#define XCVR_PLL_TEST_CTRL_PLL_FORCE_VTUNE_EXTERNALLY(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_FORCE_VTUNE_EXTERNALLY_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_FORCE_VTUNE_EXTERNALLY_MASK)
#define XCVR_PLL_TEST_CTRL_PLL_RIPPLE_COUNTER_TEST_MODE_MASK (0x4000U)
#define XCVR_PLL_TEST_CTRL_PLL_RIPPLE_COUNTER_TEST_MODE_SHIFT (14U)
#define XCVR_PLL_TEST_CTRL_PLL_RIPPLE_COUNTER_TEST_MODE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_TEST_CTRL_PLL_RIPPLE_COUNTER_TEST_MODE_SHIFT)) & XCVR_PLL_TEST_CTRL_PLL_RIPPLE_COUNTER_TEST_MODE_MASK)

/*! @name QGEN_CTRL - QGEN Control */
#define XCVR_QGEN_CTRL_QGEN_REG_SUPPLY_MASK      (0xFU)
#define XCVR_QGEN_CTRL_QGEN_REG_SUPPLY_SHIFT     (0U)
#define XCVR_QGEN_CTRL_QGEN_REG_SUPPLY(x)        (((uint32_t)(((uint32_t)(x)) << XCVR_QGEN_CTRL_QGEN_REG_SUPPLY_SHIFT)) & XCVR_QGEN_CTRL_QGEN_REG_SUPPLY_MASK)
#define XCVR_QGEN_CTRL_QGEN_REG_ATST_SEL_MASK    (0xF0U)
#define XCVR_QGEN_CTRL_QGEN_REG_ATST_SEL_SHIFT   (4U)
#define XCVR_QGEN_CTRL_QGEN_REG_ATST_SEL(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_QGEN_CTRL_QGEN_REG_ATST_SEL_SHIFT)) & XCVR_QGEN_CTRL_QGEN_REG_ATST_SEL_MASK)
#define XCVR_QGEN_CTRL_QGEN_REG_BYPASS_ON_MASK   (0x100U)
#define XCVR_QGEN_CTRL_QGEN_REG_BYPASS_ON_SHIFT  (8U)
#define XCVR_QGEN_CTRL_QGEN_REG_BYPASS_ON(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_QGEN_CTRL_QGEN_REG_BYPASS_ON_SHIFT)) & XCVR_QGEN_CTRL_QGEN_REG_BYPASS_ON_MASK)

/*! @name TCA_CTRL - TCA Control */
#define XCVR_TCA_CTRL_TCA_BIAS_CURR_MASK         (0x3U)
#define XCVR_TCA_CTRL_TCA_BIAS_CURR_SHIFT        (0U)
#define XCVR_TCA_CTRL_TCA_BIAS_CURR(x)           (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_CTRL_TCA_BIAS_CURR_SHIFT)) & XCVR_TCA_CTRL_TCA_BIAS_CURR_MASK)
#define XCVR_TCA_CTRL_TCA_LOW_PWR_ON_MASK        (0x4U)
#define XCVR_TCA_CTRL_TCA_LOW_PWR_ON_SHIFT       (2U)
#define XCVR_TCA_CTRL_TCA_LOW_PWR_ON(x)          (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_CTRL_TCA_LOW_PWR_ON_SHIFT)) & XCVR_TCA_CTRL_TCA_LOW_PWR_ON_MASK)
#define XCVR_TCA_CTRL_TCA_TX_REG_BYPASS_ON_MASK  (0x8U)
#define XCVR_TCA_CTRL_TCA_TX_REG_BYPASS_ON_SHIFT (3U)
#define XCVR_TCA_CTRL_TCA_TX_REG_BYPASS_ON(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_CTRL_TCA_TX_REG_BYPASS_ON_SHIFT)) & XCVR_TCA_CTRL_TCA_TX_REG_BYPASS_ON_MASK)
#define XCVR_TCA_CTRL_TCA_TX_REG_SUPPLY_MASK     (0xF0U)
#define XCVR_TCA_CTRL_TCA_TX_REG_SUPPLY_SHIFT    (4U)
#define XCVR_TCA_CTRL_TCA_TX_REG_SUPPLY(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_CTRL_TCA_TX_REG_SUPPLY_SHIFT)) & XCVR_TCA_CTRL_TCA_TX_REG_SUPPLY_MASK)
#define XCVR_TCA_CTRL_TCA_TX_REG_ATST_SEL_MASK   (0x300U)
#define XCVR_TCA_CTRL_TCA_TX_REG_ATST_SEL_SHIFT  (8U)
#define XCVR_TCA_CTRL_TCA_TX_REG_ATST_SEL(x)     (((uint32_t)(((uint32_t)(x)) << XCVR_TCA_CTRL_TCA_TX_REG_ATST_SEL_SHIFT)) & XCVR_TCA_CTRL_TCA_TX_REG_ATST_SEL_MASK)

/*! @name TZA_CTRL - TZA Control */
#define XCVR_TZA_CTRL_TZA_CAP_TUNE_MASK          (0xFU)
#define XCVR_TZA_CTRL_TZA_CAP_TUNE_SHIFT         (0U)
#define XCVR_TZA_CTRL_TZA_CAP_TUNE(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TZA_CTRL_TZA_CAP_TUNE_SHIFT)) & XCVR_TZA_CTRL_TZA_CAP_TUNE_MASK)
#define XCVR_TZA_CTRL_TZA_GAIN_MASK              (0x10U)
#define XCVR_TZA_CTRL_TZA_GAIN_SHIFT             (4U)
#define XCVR_TZA_CTRL_TZA_GAIN(x)                (((uint32_t)(((uint32_t)(x)) << XCVR_TZA_CTRL_TZA_GAIN_SHIFT)) & XCVR_TZA_CTRL_TZA_GAIN_MASK)
#define XCVR_TZA_CTRL_TZA_DCOC_ON_MASK           (0x20U)
#define XCVR_TZA_CTRL_TZA_DCOC_ON_SHIFT          (5U)
#define XCVR_TZA_CTRL_TZA_DCOC_ON(x)             (((uint32_t)(((uint32_t)(x)) << XCVR_TZA_CTRL_TZA_DCOC_ON_SHIFT)) & XCVR_TZA_CTRL_TZA_DCOC_ON_MASK)
#define XCVR_TZA_CTRL_TZA_CUR_CNTL_MASK          (0xC0U)
#define XCVR_TZA_CTRL_TZA_CUR_CNTL_SHIFT         (6U)
#define XCVR_TZA_CTRL_TZA_CUR_CNTL(x)            (((uint32_t)(((uint32_t)(x)) << XCVR_TZA_CTRL_TZA_CUR_CNTL_SHIFT)) & XCVR_TZA_CTRL_TZA_CUR_CNTL_MASK)
#define XCVR_TZA_CTRL_TZA_SPARE_MASK             (0xF00000U)
#define XCVR_TZA_CTRL_TZA_SPARE_SHIFT            (20U)
#define XCVR_TZA_CTRL_TZA_SPARE(x)               (((uint32_t)(((uint32_t)(x)) << XCVR_TZA_CTRL_TZA_SPARE_SHIFT)) & XCVR_TZA_CTRL_TZA_SPARE_MASK)

/*! @name TX_ANA_CTRL - TX Analog Control */
#define XCVR_TX_ANA_CTRL_HPM_CAL_ADJUST_MASK     (0xFU)
#define XCVR_TX_ANA_CTRL_HPM_CAL_ADJUST_SHIFT    (0U)
#define XCVR_TX_ANA_CTRL_HPM_CAL_ADJUST(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_TX_ANA_CTRL_HPM_CAL_ADJUST_SHIFT)) & XCVR_TX_ANA_CTRL_HPM_CAL_ADJUST_MASK)

/*! @name ANA_SPARE - Analog Spare */
#define XCVR_ANA_SPARE_IQMC_DC_GAIN_ADJ_MASK     (0x7FFU)
#define XCVR_ANA_SPARE_IQMC_DC_GAIN_ADJ_SHIFT    (0U)
#define XCVR_ANA_SPARE_IQMC_DC_GAIN_ADJ(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_ANA_SPARE_IQMC_DC_GAIN_ADJ_SHIFT)) & XCVR_ANA_SPARE_IQMC_DC_GAIN_ADJ_MASK)
#define XCVR_ANA_SPARE_DCOC_TRK_EST_GS_CNT_MASK  (0x3800U)
#define XCVR_ANA_SPARE_DCOC_TRK_EST_GS_CNT_SHIFT (11U)
#define XCVR_ANA_SPARE_DCOC_TRK_EST_GS_CNT(x)    (((uint32_t)(((uint32_t)(x)) << XCVR_ANA_SPARE_DCOC_TRK_EST_GS_CNT_SHIFT)) & XCVR_ANA_SPARE_DCOC_TRK_EST_GS_CNT_MASK)
#define XCVR_ANA_SPARE_HPM_LSB_INVERT_MASK       (0xC000U)
#define XCVR_ANA_SPARE_HPM_LSB_INVERT_SHIFT      (14U)
#define XCVR_ANA_SPARE_HPM_LSB_INVERT(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_ANA_SPARE_HPM_LSB_INVERT_SHIFT)) & XCVR_ANA_SPARE_HPM_LSB_INVERT_MASK)
#define XCVR_ANA_SPARE_ANA_DTEST_MASK            (0x3F0000U)
#define XCVR_ANA_SPARE_ANA_DTEST_SHIFT           (16U)
#define XCVR_ANA_SPARE_ANA_DTEST(x)              (((uint32_t)(((uint32_t)(x)) << XCVR_ANA_SPARE_ANA_DTEST_SHIFT)) & XCVR_ANA_SPARE_ANA_DTEST_MASK)


/*!
 * @}
 */ /* end of group XCVR_Register_Masks */


/* XCVR - Peripheral instance base addresses */
/** Peripheral XCVR base address */
#define XCVR_BASE                                (0x4005C000u)
/** Peripheral XCVR base pointer */
#define XCVR                                     ((XCVR_Type *)XCVR_BASE)
/** Array initializer of XCVR peripheral base addresses */
#define XCVR_BASE_ADDRS                          { XCVR_BASE }
/** Array initializer of XCVR peripheral base pointers */
#define XCVR_BASE_PTRS                           { XCVR }

/*!
 * @}
 */ /* end of group XCVR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- ZLL Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ZLL_Peripheral_Access_Layer ZLL Peripheral Access Layer
 * @{
 */

/** ZLL - Register Layout Typedef */
typedef struct {
  __IO uint32_t IRQSTS;                            /**< INTERRUPT REQUEST STATUS, offset: 0x0 */
  __IO uint32_t PHY_CTRL;                          /**< PHY CONTROL, offset: 0x4 */
  __I  uint32_t EVENT_TMR;                         /**< EVENT TIMER, offset: 0x8 */
  __I  uint32_t TIMESTAMP;                         /**< TIMESTAMP, offset: 0xC */
  __IO uint32_t T1CMP;                             /**< T1 COMPARE, offset: 0x10 */
  __IO uint32_t T2CMP;                             /**< T2 COMPARE, offset: 0x14 */
  __IO uint32_t T2PRIMECMP;                        /**< T2 PRIME COMPARE, offset: 0x18 */
  __IO uint32_t T3CMP;                             /**< T3 COMPARE, offset: 0x1C */
  __IO uint32_t T4CMP;                             /**< T4 COMPARE, offset: 0x20 */
  __IO uint32_t PA_PWR;                            /**< PA POWER, offset: 0x24 */
  __IO uint32_t CHANNEL_NUM0;                      /**< CHANNEL NUMBER 0, offset: 0x28 */
  __I  uint32_t LQI_AND_RSSI;                      /**< LQI AND RSSI, offset: 0x2C */
  __IO uint32_t MACSHORTADDRS0;                    /**< MAC SHORT ADDRESS 0, offset: 0x30 */
  __IO uint32_t MACLONGADDRS0_LSB;                 /**< MAC LONG ADDRESS 0 LSB, offset: 0x34 */
  __IO uint32_t MACLONGADDRS0_MSB;                 /**< MAC LONG ADDRESS 0 MSB, offset: 0x38 */
  __IO uint32_t RX_FRAME_FILTER;                   /**< RECEIVE FRAME FILTER, offset: 0x3C */
  __IO uint32_t CCA_LQI_CTRL;                      /**< CCA AND LQI CONTROL, offset: 0x40 */
  __IO uint32_t CCA2_CTRL;                         /**< CCA2 CONTROL, offset: 0x44 */
  __IO uint32_t FAD_CTRL;                          /**< FAD CONTROL, offset: 0x48 */
  __IO uint32_t SNF_CTRL;                          /**< SNF CONTROL, offset: 0x4C */
  __IO uint32_t BSM_CTRL;                          /**< BSM CONTROL, offset: 0x50 */
  __IO uint32_t MACSHORTADDRS1;                    /**< MAC SHORT ADDRESS 1, offset: 0x54 */
  __IO uint32_t MACLONGADDRS1_LSB;                 /**< MAC LONG ADDRESS 1 LSB, offset: 0x58 */
  __IO uint32_t MACLONGADDRS1_MSB;                 /**< MAC LONG ADDRESS 1 MSB, offset: 0x5C */
  __IO uint32_t DUAL_PAN_CTRL;                     /**< DUAL PAN CONTROL, offset: 0x60 */
  __IO uint32_t CHANNEL_NUM1;                      /**< CHANNEL NUMBER 1, offset: 0x64 */
  __IO uint32_t SAM_CTRL;                          /**< SAM CONTROL, offset: 0x68 */
  __IO uint32_t SAM_TABLE;                         /**< SOURCE ADDRESS MANAGEMENT TABLE, offset: 0x6C */
  __I  uint32_t SAM_MATCH;                         /**< SAM MATCH, offset: 0x70 */
  __I  uint32_t SAM_FREE_IDX;                      /**< SAM FREE INDEX, offset: 0x74 */
  __IO uint32_t SEQ_CTRL_STS;                      /**< SEQUENCE CONTROL AND STATUS, offset: 0x78 */
  __IO uint32_t ACKDELAY;                          /**< ACK DELAY, offset: 0x7C */
  __IO uint32_t FILTERFAIL_CODE;                   /**< FILTER FAIL CODE, offset: 0x80 */
  __IO uint32_t RX_WTR_MARK;                       /**< RECEIVE WATER MARK, offset: 0x84 */
       uint8_t RESERVED_0[4];
  __IO uint32_t SLOT_PRELOAD;                      /**< SLOT PRELOAD, offset: 0x8C */
  __I  uint32_t SEQ_STATE;                         /**< ZIGBEE SEQUENCE STATE, offset: 0x90 */
  __IO uint32_t TMR_PRESCALE;                      /**< TIMER PRESCALER, offset: 0x94 */
  __IO uint32_t LENIENCY_LSB;                      /**< LENIENCY LSB, offset: 0x98 */
  __IO uint32_t LENIENCY_MSB;                      /**< LENIENCY MSB, offset: 0x9C */
  __I  uint32_t PART_ID;                           /**< PART ID, offset: 0xA0 */
       uint8_t RESERVED_1[92];
  __IO uint32_t PKT_BUFFER[32];                    /**< PACKET BUFFER, array offset: 0x100, array step: 0x4 */
} ZLL_Type;

/* ----------------------------------------------------------------------------
   -- ZLL Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ZLL_Register_Masks ZLL Register Masks
 * @{
 */

/*! @name IRQSTS - INTERRUPT REQUEST STATUS */
#define ZLL_IRQSTS_SEQIRQ_MASK                   (0x1U)
#define ZLL_IRQSTS_SEQIRQ_SHIFT                  (0U)
#define ZLL_IRQSTS_SEQIRQ(x)                     (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_SEQIRQ_SHIFT)) & ZLL_IRQSTS_SEQIRQ_MASK)
#define ZLL_IRQSTS_TXIRQ_MASK                    (0x2U)
#define ZLL_IRQSTS_TXIRQ_SHIFT                   (1U)
#define ZLL_IRQSTS_TXIRQ(x)                      (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TXIRQ_SHIFT)) & ZLL_IRQSTS_TXIRQ_MASK)
#define ZLL_IRQSTS_RXIRQ_MASK                    (0x4U)
#define ZLL_IRQSTS_RXIRQ_SHIFT                   (2U)
#define ZLL_IRQSTS_RXIRQ(x)                      (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_RXIRQ_SHIFT)) & ZLL_IRQSTS_RXIRQ_MASK)
#define ZLL_IRQSTS_CCAIRQ_MASK                   (0x8U)
#define ZLL_IRQSTS_CCAIRQ_SHIFT                  (3U)
#define ZLL_IRQSTS_CCAIRQ(x)                     (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_CCAIRQ_SHIFT)) & ZLL_IRQSTS_CCAIRQ_MASK)
#define ZLL_IRQSTS_RXWTRMRKIRQ_MASK              (0x10U)
#define ZLL_IRQSTS_RXWTRMRKIRQ_SHIFT             (4U)
#define ZLL_IRQSTS_RXWTRMRKIRQ(x)                (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_RXWTRMRKIRQ_SHIFT)) & ZLL_IRQSTS_RXWTRMRKIRQ_MASK)
#define ZLL_IRQSTS_FILTERFAIL_IRQ_MASK           (0x20U)
#define ZLL_IRQSTS_FILTERFAIL_IRQ_SHIFT          (5U)
#define ZLL_IRQSTS_FILTERFAIL_IRQ(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_FILTERFAIL_IRQ_SHIFT)) & ZLL_IRQSTS_FILTERFAIL_IRQ_MASK)
#define ZLL_IRQSTS_PLL_UNLOCK_IRQ_MASK           (0x40U)
#define ZLL_IRQSTS_PLL_UNLOCK_IRQ_SHIFT          (6U)
#define ZLL_IRQSTS_PLL_UNLOCK_IRQ(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_PLL_UNLOCK_IRQ_SHIFT)) & ZLL_IRQSTS_PLL_UNLOCK_IRQ_MASK)
#define ZLL_IRQSTS_RX_FRM_PEND_MASK              (0x80U)
#define ZLL_IRQSTS_RX_FRM_PEND_SHIFT             (7U)
#define ZLL_IRQSTS_RX_FRM_PEND(x)                (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_RX_FRM_PEND_SHIFT)) & ZLL_IRQSTS_RX_FRM_PEND_MASK)
#define ZLL_IRQSTS_PB_ERR_IRQ_MASK               (0x200U)
#define ZLL_IRQSTS_PB_ERR_IRQ_SHIFT              (9U)
#define ZLL_IRQSTS_PB_ERR_IRQ(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_PB_ERR_IRQ_SHIFT)) & ZLL_IRQSTS_PB_ERR_IRQ_MASK)
#define ZLL_IRQSTS_TMRSTATUS_MASK                (0x800U)
#define ZLL_IRQSTS_TMRSTATUS_SHIFT               (11U)
#define ZLL_IRQSTS_TMRSTATUS(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMRSTATUS_SHIFT)) & ZLL_IRQSTS_TMRSTATUS_MASK)
#define ZLL_IRQSTS_PI_MASK                       (0x1000U)
#define ZLL_IRQSTS_PI_SHIFT                      (12U)
#define ZLL_IRQSTS_PI(x)                         (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_PI_SHIFT)) & ZLL_IRQSTS_PI_MASK)
#define ZLL_IRQSTS_SRCADDR_MASK                  (0x2000U)
#define ZLL_IRQSTS_SRCADDR_SHIFT                 (13U)
#define ZLL_IRQSTS_SRCADDR(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_SRCADDR_SHIFT)) & ZLL_IRQSTS_SRCADDR_MASK)
#define ZLL_IRQSTS_CCA_MASK                      (0x4000U)
#define ZLL_IRQSTS_CCA_SHIFT                     (14U)
#define ZLL_IRQSTS_CCA(x)                        (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_CCA_SHIFT)) & ZLL_IRQSTS_CCA_MASK)
#define ZLL_IRQSTS_CRCVALID_MASK                 (0x8000U)
#define ZLL_IRQSTS_CRCVALID_SHIFT                (15U)
#define ZLL_IRQSTS_CRCVALID(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_CRCVALID_SHIFT)) & ZLL_IRQSTS_CRCVALID_MASK)
#define ZLL_IRQSTS_TMR1IRQ_MASK                  (0x10000U)
#define ZLL_IRQSTS_TMR1IRQ_SHIFT                 (16U)
#define ZLL_IRQSTS_TMR1IRQ(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR1IRQ_SHIFT)) & ZLL_IRQSTS_TMR1IRQ_MASK)
#define ZLL_IRQSTS_TMR2IRQ_MASK                  (0x20000U)
#define ZLL_IRQSTS_TMR2IRQ_SHIFT                 (17U)
#define ZLL_IRQSTS_TMR2IRQ(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR2IRQ_SHIFT)) & ZLL_IRQSTS_TMR2IRQ_MASK)
#define ZLL_IRQSTS_TMR3IRQ_MASK                  (0x40000U)
#define ZLL_IRQSTS_TMR3IRQ_SHIFT                 (18U)
#define ZLL_IRQSTS_TMR3IRQ(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR3IRQ_SHIFT)) & ZLL_IRQSTS_TMR3IRQ_MASK)
#define ZLL_IRQSTS_TMR4IRQ_MASK                  (0x80000U)
#define ZLL_IRQSTS_TMR4IRQ_SHIFT                 (19U)
#define ZLL_IRQSTS_TMR4IRQ(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR4IRQ_SHIFT)) & ZLL_IRQSTS_TMR4IRQ_MASK)
#define ZLL_IRQSTS_TMR1MSK_MASK                  (0x100000U)
#define ZLL_IRQSTS_TMR1MSK_SHIFT                 (20U)
#define ZLL_IRQSTS_TMR1MSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR1MSK_SHIFT)) & ZLL_IRQSTS_TMR1MSK_MASK)
#define ZLL_IRQSTS_TMR2MSK_MASK                  (0x200000U)
#define ZLL_IRQSTS_TMR2MSK_SHIFT                 (21U)
#define ZLL_IRQSTS_TMR2MSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR2MSK_SHIFT)) & ZLL_IRQSTS_TMR2MSK_MASK)
#define ZLL_IRQSTS_TMR3MSK_MASK                  (0x400000U)
#define ZLL_IRQSTS_TMR3MSK_SHIFT                 (22U)
#define ZLL_IRQSTS_TMR3MSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR3MSK_SHIFT)) & ZLL_IRQSTS_TMR3MSK_MASK)
#define ZLL_IRQSTS_TMR4MSK_MASK                  (0x800000U)
#define ZLL_IRQSTS_TMR4MSK_SHIFT                 (23U)
#define ZLL_IRQSTS_TMR4MSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_TMR4MSK_SHIFT)) & ZLL_IRQSTS_TMR4MSK_MASK)
#define ZLL_IRQSTS_RX_FRAME_LENGTH_MASK          (0x7F000000U)
#define ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT         (24U)
#define ZLL_IRQSTS_RX_FRAME_LENGTH(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT)) & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK)

/*! @name PHY_CTRL - PHY CONTROL */
#define ZLL_PHY_CTRL_XCVSEQ_MASK                 (0x7U)
#define ZLL_PHY_CTRL_XCVSEQ_SHIFT                (0U)
#define ZLL_PHY_CTRL_XCVSEQ(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_XCVSEQ_SHIFT)) & ZLL_PHY_CTRL_XCVSEQ_MASK)
#define ZLL_PHY_CTRL_AUTOACK_MASK                (0x8U)
#define ZLL_PHY_CTRL_AUTOACK_SHIFT               (3U)
#define ZLL_PHY_CTRL_AUTOACK(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_AUTOACK_SHIFT)) & ZLL_PHY_CTRL_AUTOACK_MASK)
#define ZLL_PHY_CTRL_RXACKRQD_MASK               (0x10U)
#define ZLL_PHY_CTRL_RXACKRQD_SHIFT              (4U)
#define ZLL_PHY_CTRL_RXACKRQD(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_RXACKRQD_SHIFT)) & ZLL_PHY_CTRL_RXACKRQD_MASK)
#define ZLL_PHY_CTRL_CCABFRTX_MASK               (0x20U)
#define ZLL_PHY_CTRL_CCABFRTX_SHIFT              (5U)
#define ZLL_PHY_CTRL_CCABFRTX(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_CCABFRTX_SHIFT)) & ZLL_PHY_CTRL_CCABFRTX_MASK)
#define ZLL_PHY_CTRL_SLOTTED_MASK                (0x40U)
#define ZLL_PHY_CTRL_SLOTTED_SHIFT               (6U)
#define ZLL_PHY_CTRL_SLOTTED(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_SLOTTED_SHIFT)) & ZLL_PHY_CTRL_SLOTTED_MASK)
#define ZLL_PHY_CTRL_TMRTRIGEN_MASK              (0x80U)
#define ZLL_PHY_CTRL_TMRTRIGEN_SHIFT             (7U)
#define ZLL_PHY_CTRL_TMRTRIGEN(x)                (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMRTRIGEN_SHIFT)) & ZLL_PHY_CTRL_TMRTRIGEN_MASK)
#define ZLL_PHY_CTRL_SEQMSK_MASK                 (0x100U)
#define ZLL_PHY_CTRL_SEQMSK_SHIFT                (8U)
#define ZLL_PHY_CTRL_SEQMSK(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_SEQMSK_SHIFT)) & ZLL_PHY_CTRL_SEQMSK_MASK)
#define ZLL_PHY_CTRL_TXMSK_MASK                  (0x200U)
#define ZLL_PHY_CTRL_TXMSK_SHIFT                 (9U)
#define ZLL_PHY_CTRL_TXMSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TXMSK_SHIFT)) & ZLL_PHY_CTRL_TXMSK_MASK)
#define ZLL_PHY_CTRL_RXMSK_MASK                  (0x400U)
#define ZLL_PHY_CTRL_RXMSK_SHIFT                 (10U)
#define ZLL_PHY_CTRL_RXMSK(x)                    (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_RXMSK_SHIFT)) & ZLL_PHY_CTRL_RXMSK_MASK)
#define ZLL_PHY_CTRL_CCAMSK_MASK                 (0x800U)
#define ZLL_PHY_CTRL_CCAMSK_SHIFT                (11U)
#define ZLL_PHY_CTRL_CCAMSK(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_CCAMSK_SHIFT)) & ZLL_PHY_CTRL_CCAMSK_MASK)
#define ZLL_PHY_CTRL_RX_WMRK_MSK_MASK            (0x1000U)
#define ZLL_PHY_CTRL_RX_WMRK_MSK_SHIFT           (12U)
#define ZLL_PHY_CTRL_RX_WMRK_MSK(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_RX_WMRK_MSK_SHIFT)) & ZLL_PHY_CTRL_RX_WMRK_MSK_MASK)
#define ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK         (0x2000U)
#define ZLL_PHY_CTRL_FILTERFAIL_MSK_SHIFT        (13U)
#define ZLL_PHY_CTRL_FILTERFAIL_MSK(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_FILTERFAIL_MSK_SHIFT)) & ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK)
#define ZLL_PHY_CTRL_PLL_UNLOCK_MSK_MASK         (0x4000U)
#define ZLL_PHY_CTRL_PLL_UNLOCK_MSK_SHIFT        (14U)
#define ZLL_PHY_CTRL_PLL_UNLOCK_MSK(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_PLL_UNLOCK_MSK_SHIFT)) & ZLL_PHY_CTRL_PLL_UNLOCK_MSK_MASK)
#define ZLL_PHY_CTRL_CRC_MSK_MASK                (0x8000U)
#define ZLL_PHY_CTRL_CRC_MSK_SHIFT               (15U)
#define ZLL_PHY_CTRL_CRC_MSK(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_CRC_MSK_SHIFT)) & ZLL_PHY_CTRL_CRC_MSK_MASK)
#define ZLL_PHY_CTRL_PB_ERR_MSK_MASK             (0x20000U)
#define ZLL_PHY_CTRL_PB_ERR_MSK_SHIFT            (17U)
#define ZLL_PHY_CTRL_PB_ERR_MSK(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_PB_ERR_MSK_SHIFT)) & ZLL_PHY_CTRL_PB_ERR_MSK_MASK)
#define ZLL_PHY_CTRL_TMR1CMP_EN_MASK             (0x100000U)
#define ZLL_PHY_CTRL_TMR1CMP_EN_SHIFT            (20U)
#define ZLL_PHY_CTRL_TMR1CMP_EN(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMR1CMP_EN_SHIFT)) & ZLL_PHY_CTRL_TMR1CMP_EN_MASK)
#define ZLL_PHY_CTRL_TMR2CMP_EN_MASK             (0x200000U)
#define ZLL_PHY_CTRL_TMR2CMP_EN_SHIFT            (21U)
#define ZLL_PHY_CTRL_TMR2CMP_EN(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMR2CMP_EN_SHIFT)) & ZLL_PHY_CTRL_TMR2CMP_EN_MASK)
#define ZLL_PHY_CTRL_TMR3CMP_EN_MASK             (0x400000U)
#define ZLL_PHY_CTRL_TMR3CMP_EN_SHIFT            (22U)
#define ZLL_PHY_CTRL_TMR3CMP_EN(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMR3CMP_EN_SHIFT)) & ZLL_PHY_CTRL_TMR3CMP_EN_MASK)
#define ZLL_PHY_CTRL_TMR4CMP_EN_MASK             (0x800000U)
#define ZLL_PHY_CTRL_TMR4CMP_EN_SHIFT            (23U)
#define ZLL_PHY_CTRL_TMR4CMP_EN(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMR4CMP_EN_SHIFT)) & ZLL_PHY_CTRL_TMR4CMP_EN_MASK)
#define ZLL_PHY_CTRL_TC2PRIME_EN_MASK            (0x1000000U)
#define ZLL_PHY_CTRL_TC2PRIME_EN_SHIFT           (24U)
#define ZLL_PHY_CTRL_TC2PRIME_EN(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TC2PRIME_EN_SHIFT)) & ZLL_PHY_CTRL_TC2PRIME_EN_MASK)
#define ZLL_PHY_CTRL_PROMISCUOUS_MASK            (0x2000000U)
#define ZLL_PHY_CTRL_PROMISCUOUS_SHIFT           (25U)
#define ZLL_PHY_CTRL_PROMISCUOUS(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_PROMISCUOUS_SHIFT)) & ZLL_PHY_CTRL_PROMISCUOUS_MASK)
#define ZLL_PHY_CTRL_TMRLOAD_MASK                (0x4000000U)
#define ZLL_PHY_CTRL_TMRLOAD_SHIFT               (26U)
#define ZLL_PHY_CTRL_TMRLOAD(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TMRLOAD_SHIFT)) & ZLL_PHY_CTRL_TMRLOAD_MASK)
#define ZLL_PHY_CTRL_CCATYPE_MASK                (0x18000000U)
#define ZLL_PHY_CTRL_CCATYPE_SHIFT               (27U)
#define ZLL_PHY_CTRL_CCATYPE(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_CCATYPE_SHIFT)) & ZLL_PHY_CTRL_CCATYPE_MASK)
#define ZLL_PHY_CTRL_PANCORDNTR0_MASK            (0x20000000U)
#define ZLL_PHY_CTRL_PANCORDNTR0_SHIFT           (29U)
#define ZLL_PHY_CTRL_PANCORDNTR0(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_PANCORDNTR0_SHIFT)) & ZLL_PHY_CTRL_PANCORDNTR0_MASK)
#define ZLL_PHY_CTRL_TC3TMOUT_MASK               (0x40000000U)
#define ZLL_PHY_CTRL_TC3TMOUT_SHIFT              (30U)
#define ZLL_PHY_CTRL_TC3TMOUT(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TC3TMOUT_SHIFT)) & ZLL_PHY_CTRL_TC3TMOUT_MASK)
#define ZLL_PHY_CTRL_TRCV_MSK_MASK               (0x80000000U)
#define ZLL_PHY_CTRL_TRCV_MSK_SHIFT              (31U)
#define ZLL_PHY_CTRL_TRCV_MSK(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_PHY_CTRL_TRCV_MSK_SHIFT)) & ZLL_PHY_CTRL_TRCV_MSK_MASK)

/*! @name EVENT_TMR - EVENT TIMER */
#define ZLL_EVENT_TMR_EVENT_TMR_MASK             (0xFFFFFFU)
#define ZLL_EVENT_TMR_EVENT_TMR_SHIFT            (0U)
#define ZLL_EVENT_TMR_EVENT_TMR(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_EVENT_TMR_EVENT_TMR_SHIFT)) & ZLL_EVENT_TMR_EVENT_TMR_MASK)

/*! @name TIMESTAMP - TIMESTAMP */
#define ZLL_TIMESTAMP_TIMESTAMP_MASK             (0xFFFFFFU)
#define ZLL_TIMESTAMP_TIMESTAMP_SHIFT            (0U)
#define ZLL_TIMESTAMP_TIMESTAMP(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_TIMESTAMP_TIMESTAMP_SHIFT)) & ZLL_TIMESTAMP_TIMESTAMP_MASK)

/*! @name T1CMP - T1 COMPARE */
#define ZLL_T1CMP_T1CMP_MASK                     (0xFFFFFFU)
#define ZLL_T1CMP_T1CMP_SHIFT                    (0U)
#define ZLL_T1CMP_T1CMP(x)                       (((uint32_t)(((uint32_t)(x)) << ZLL_T1CMP_T1CMP_SHIFT)) & ZLL_T1CMP_T1CMP_MASK)

/*! @name T2CMP - T2 COMPARE */
#define ZLL_T2CMP_T2CMP_MASK                     (0xFFFFFFU)
#define ZLL_T2CMP_T2CMP_SHIFT                    (0U)
#define ZLL_T2CMP_T2CMP(x)                       (((uint32_t)(((uint32_t)(x)) << ZLL_T2CMP_T2CMP_SHIFT)) & ZLL_T2CMP_T2CMP_MASK)

/*! @name T2PRIMECMP - T2 PRIME COMPARE */
#define ZLL_T2PRIMECMP_T2PRIMECMP_MASK           (0xFFFFU)
#define ZLL_T2PRIMECMP_T2PRIMECMP_SHIFT          (0U)
#define ZLL_T2PRIMECMP_T2PRIMECMP(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_T2PRIMECMP_T2PRIMECMP_SHIFT)) & ZLL_T2PRIMECMP_T2PRIMECMP_MASK)

/*! @name T3CMP - T3 COMPARE */
#define ZLL_T3CMP_T3CMP_MASK                     (0xFFFFFFU)
#define ZLL_T3CMP_T3CMP_SHIFT                    (0U)
#define ZLL_T3CMP_T3CMP(x)                       (((uint32_t)(((uint32_t)(x)) << ZLL_T3CMP_T3CMP_SHIFT)) & ZLL_T3CMP_T3CMP_MASK)

/*! @name T4CMP - T4 COMPARE */
#define ZLL_T4CMP_T4CMP_MASK                     (0xFFFFFFU)
#define ZLL_T4CMP_T4CMP_SHIFT                    (0U)
#define ZLL_T4CMP_T4CMP(x)                       (((uint32_t)(((uint32_t)(x)) << ZLL_T4CMP_T4CMP_SHIFT)) & ZLL_T4CMP_T4CMP_MASK)

/*! @name PA_PWR - PA POWER */
#define ZLL_PA_PWR_PA_PWR_MASK                   (0xFU)
#define ZLL_PA_PWR_PA_PWR_SHIFT                  (0U)
#define ZLL_PA_PWR_PA_PWR(x)                     (((uint32_t)(((uint32_t)(x)) << ZLL_PA_PWR_PA_PWR_SHIFT)) & ZLL_PA_PWR_PA_PWR_MASK)

/*! @name CHANNEL_NUM0 - CHANNEL NUMBER 0 */
#define ZLL_CHANNEL_NUM0_CHANNEL_NUM0_MASK       (0x7FU)
#define ZLL_CHANNEL_NUM0_CHANNEL_NUM0_SHIFT      (0U)
#define ZLL_CHANNEL_NUM0_CHANNEL_NUM0(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_CHANNEL_NUM0_CHANNEL_NUM0_SHIFT)) & ZLL_CHANNEL_NUM0_CHANNEL_NUM0_MASK)

/*! @name LQI_AND_RSSI - LQI AND RSSI */
#define ZLL_LQI_AND_RSSI_LQI_VALUE_MASK          (0xFFU)
#define ZLL_LQI_AND_RSSI_LQI_VALUE_SHIFT         (0U)
#define ZLL_LQI_AND_RSSI_LQI_VALUE(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_LQI_AND_RSSI_LQI_VALUE_SHIFT)) & ZLL_LQI_AND_RSSI_LQI_VALUE_MASK)
#define ZLL_LQI_AND_RSSI_RSSI_MASK               (0xFF00U)
#define ZLL_LQI_AND_RSSI_RSSI_SHIFT              (8U)
#define ZLL_LQI_AND_RSSI_RSSI(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_LQI_AND_RSSI_RSSI_SHIFT)) & ZLL_LQI_AND_RSSI_RSSI_MASK)
#define ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK        (0xFF0000U)
#define ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT       (16U)
#define ZLL_LQI_AND_RSSI_CCA1_ED_FNL(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT)) & ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK)

/*! @name MACSHORTADDRS0 - MAC SHORT ADDRESS 0 */
#define ZLL_MACSHORTADDRS0_MACPANID0_MASK        (0xFFFFU)
#define ZLL_MACSHORTADDRS0_MACPANID0_SHIFT       (0U)
#define ZLL_MACSHORTADDRS0_MACPANID0(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_MACSHORTADDRS0_MACPANID0_SHIFT)) & ZLL_MACSHORTADDRS0_MACPANID0_MASK)
#define ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK   (0xFFFF0000U)
#define ZLL_MACSHORTADDRS0_MACSHORTADDRS0_SHIFT  (16U)
#define ZLL_MACSHORTADDRS0_MACSHORTADDRS0(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_MACSHORTADDRS0_MACSHORTADDRS0_SHIFT)) & ZLL_MACSHORTADDRS0_MACSHORTADDRS0_MASK)

/*! @name MACLONGADDRS0_LSB - MAC LONG ADDRESS 0 LSB */
#define ZLL_MACLONGADDRS0_LSB_MACLONGADDRS0_LSB_MASK (0xFFFFFFFFU)
#define ZLL_MACLONGADDRS0_LSB_MACLONGADDRS0_LSB_SHIFT (0U)
#define ZLL_MACLONGADDRS0_LSB_MACLONGADDRS0_LSB(x) (((uint32_t)(((uint32_t)(x)) << ZLL_MACLONGADDRS0_LSB_MACLONGADDRS0_LSB_SHIFT)) & ZLL_MACLONGADDRS0_LSB_MACLONGADDRS0_LSB_MASK)

/*! @name MACLONGADDRS0_MSB - MAC LONG ADDRESS 0 MSB */
#define ZLL_MACLONGADDRS0_MSB_MACLONGADDRS0_MSB_MASK (0xFFFFFFFFU)
#define ZLL_MACLONGADDRS0_MSB_MACLONGADDRS0_MSB_SHIFT (0U)
#define ZLL_MACLONGADDRS0_MSB_MACLONGADDRS0_MSB(x) (((uint32_t)(((uint32_t)(x)) << ZLL_MACLONGADDRS0_MSB_MACLONGADDRS0_MSB_SHIFT)) & ZLL_MACLONGADDRS0_MSB_MACLONGADDRS0_MSB_MASK)

/*! @name RX_FRAME_FILTER - RECEIVE FRAME FILTER */
#define ZLL_RX_FRAME_FILTER_BEACON_FT_MASK       (0x1U)
#define ZLL_RX_FRAME_FILTER_BEACON_FT_SHIFT      (0U)
#define ZLL_RX_FRAME_FILTER_BEACON_FT(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_BEACON_FT_SHIFT)) & ZLL_RX_FRAME_FILTER_BEACON_FT_MASK)
#define ZLL_RX_FRAME_FILTER_DATA_FT_MASK         (0x2U)
#define ZLL_RX_FRAME_FILTER_DATA_FT_SHIFT        (1U)
#define ZLL_RX_FRAME_FILTER_DATA_FT(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_DATA_FT_SHIFT)) & ZLL_RX_FRAME_FILTER_DATA_FT_MASK)
#define ZLL_RX_FRAME_FILTER_ACK_FT_MASK          (0x4U)
#define ZLL_RX_FRAME_FILTER_ACK_FT_SHIFT         (2U)
#define ZLL_RX_FRAME_FILTER_ACK_FT(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_ACK_FT_SHIFT)) & ZLL_RX_FRAME_FILTER_ACK_FT_MASK)
#define ZLL_RX_FRAME_FILTER_CMD_FT_MASK          (0x8U)
#define ZLL_RX_FRAME_FILTER_CMD_FT_SHIFT         (3U)
#define ZLL_RX_FRAME_FILTER_CMD_FT(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_CMD_FT_SHIFT)) & ZLL_RX_FRAME_FILTER_CMD_FT_MASK)
#define ZLL_RX_FRAME_FILTER_NS_FT_MASK           (0x10U)
#define ZLL_RX_FRAME_FILTER_NS_FT_SHIFT          (4U)
#define ZLL_RX_FRAME_FILTER_NS_FT(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_NS_FT_SHIFT)) & ZLL_RX_FRAME_FILTER_NS_FT_MASK)
#define ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK (0x20U)
#define ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_SHIFT (5U)
#define ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS(x) (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_SHIFT)) & ZLL_RX_FRAME_FILTER_ACTIVE_PROMISCUOUS_MASK)
#define ZLL_RX_FRAME_FILTER_FRM_VER_MASK         (0xC0U)
#define ZLL_RX_FRAME_FILTER_FRM_VER_SHIFT        (6U)
#define ZLL_RX_FRAME_FILTER_FRM_VER(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_RX_FRAME_FILTER_FRM_VER_SHIFT)) & ZLL_RX_FRAME_FILTER_FRM_VER_MASK)

/*! @name CCA_LQI_CTRL - CCA AND LQI CONTROL */
#define ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK        (0xFFU)
#define ZLL_CCA_LQI_CTRL_CCA1_THRESH_SHIFT       (0U)
#define ZLL_CCA_LQI_CTRL_CCA1_THRESH(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_CCA_LQI_CTRL_CCA1_THRESH_SHIFT)) & ZLL_CCA_LQI_CTRL_CCA1_THRESH_MASK)
#define ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_MASK    (0xFF0000U)
#define ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_SHIFT   (16U)
#define ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_SHIFT)) & ZLL_CCA_LQI_CTRL_LQI_OFFSET_COMP_MASK)
#define ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_MASK    (0x8000000U)
#define ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_SHIFT   (27U)
#define ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_SHIFT)) & ZLL_CCA_LQI_CTRL_CCA3_AND_NOT_OR_MASK)

/*! @name CCA2_CTRL - CCA2 CONTROL */
#define ZLL_CCA2_CTRL_CCA2_NUM_CORR_PEAKS_MASK   (0xFU)
#define ZLL_CCA2_CTRL_CCA2_NUM_CORR_PEAKS_SHIFT  (0U)
#define ZLL_CCA2_CTRL_CCA2_NUM_CORR_PEAKS(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_CCA2_CTRL_CCA2_NUM_CORR_PEAKS_SHIFT)) & ZLL_CCA2_CTRL_CCA2_NUM_CORR_PEAKS_MASK)
#define ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_MASK  (0x70U)
#define ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_SHIFT (4U)
#define ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_SHIFT)) & ZLL_CCA2_CTRL_CCA2_MIN_NUM_CORR_TH_MASK)
#define ZLL_CCA2_CTRL_CCA2_CORR_THRESH_MASK      (0xFF00U)
#define ZLL_CCA2_CTRL_CCA2_CORR_THRESH_SHIFT     (8U)
#define ZLL_CCA2_CTRL_CCA2_CORR_THRESH(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_CCA2_CTRL_CCA2_CORR_THRESH_SHIFT)) & ZLL_CCA2_CTRL_CCA2_CORR_THRESH_MASK)

/*! @name FAD_CTRL - FAD CONTROL */
#define ZLL_FAD_CTRL_FAD_EN_MASK                 (0x1U)
#define ZLL_FAD_CTRL_FAD_EN_SHIFT                (0U)
#define ZLL_FAD_CTRL_FAD_EN(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_FAD_EN_SHIFT)) & ZLL_FAD_CTRL_FAD_EN_MASK)
#define ZLL_FAD_CTRL_ANTX_MASK                   (0x2U)
#define ZLL_FAD_CTRL_ANTX_SHIFT                  (1U)
#define ZLL_FAD_CTRL_ANTX(x)                     (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_ANTX_SHIFT)) & ZLL_FAD_CTRL_ANTX_MASK)
#define ZLL_FAD_CTRL_FAD_NOT_GPIO_MASK           (0x4U)
#define ZLL_FAD_CTRL_FAD_NOT_GPIO_SHIFT          (2U)
#define ZLL_FAD_CTRL_FAD_NOT_GPIO(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_FAD_NOT_GPIO_SHIFT)) & ZLL_FAD_CTRL_FAD_NOT_GPIO_MASK)
#define ZLL_FAD_CTRL_ANTX_EN_MASK                (0x300U)
#define ZLL_FAD_CTRL_ANTX_EN_SHIFT               (8U)
#define ZLL_FAD_CTRL_ANTX_EN(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_ANTX_EN_SHIFT)) & ZLL_FAD_CTRL_ANTX_EN_MASK)
#define ZLL_FAD_CTRL_ANTX_HZ_MASK                (0x400U)
#define ZLL_FAD_CTRL_ANTX_HZ_SHIFT               (10U)
#define ZLL_FAD_CTRL_ANTX_HZ(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_ANTX_HZ_SHIFT)) & ZLL_FAD_CTRL_ANTX_HZ_MASK)
#define ZLL_FAD_CTRL_ANTX_CTRLMODE_MASK          (0x800U)
#define ZLL_FAD_CTRL_ANTX_CTRLMODE_SHIFT         (11U)
#define ZLL_FAD_CTRL_ANTX_CTRLMODE(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_ANTX_CTRLMODE_SHIFT)) & ZLL_FAD_CTRL_ANTX_CTRLMODE_MASK)
#define ZLL_FAD_CTRL_ANTX_POL_MASK               (0xF000U)
#define ZLL_FAD_CTRL_ANTX_POL_SHIFT              (12U)
#define ZLL_FAD_CTRL_ANTX_POL(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_FAD_CTRL_ANTX_POL_SHIFT)) & ZLL_FAD_CTRL_ANTX_POL_MASK)

/*! @name SNF_CTRL - SNF CONTROL */
#define ZLL_SNF_CTRL_SNF_EN_MASK                 (0x1U)
#define ZLL_SNF_CTRL_SNF_EN_SHIFT                (0U)
#define ZLL_SNF_CTRL_SNF_EN(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_SNF_CTRL_SNF_EN_SHIFT)) & ZLL_SNF_CTRL_SNF_EN_MASK)

/*! @name BSM_CTRL - BSM CONTROL */
#define ZLL_BSM_CTRL_BSM_EN_MASK                 (0x1U)
#define ZLL_BSM_CTRL_BSM_EN_SHIFT                (0U)
#define ZLL_BSM_CTRL_BSM_EN(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_BSM_CTRL_BSM_EN_SHIFT)) & ZLL_BSM_CTRL_BSM_EN_MASK)

/*! @name MACSHORTADDRS1 - MAC SHORT ADDRESS 1 */
#define ZLL_MACSHORTADDRS1_MACPANID1_MASK        (0xFFFFU)
#define ZLL_MACSHORTADDRS1_MACPANID1_SHIFT       (0U)
#define ZLL_MACSHORTADDRS1_MACPANID1(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_MACSHORTADDRS1_MACPANID1_SHIFT)) & ZLL_MACSHORTADDRS1_MACPANID1_MASK)
#define ZLL_MACSHORTADDRS1_MACSHORTADDRS1_MASK   (0xFFFF0000U)
#define ZLL_MACSHORTADDRS1_MACSHORTADDRS1_SHIFT  (16U)
#define ZLL_MACSHORTADDRS1_MACSHORTADDRS1(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_MACSHORTADDRS1_MACSHORTADDRS1_SHIFT)) & ZLL_MACSHORTADDRS1_MACSHORTADDRS1_MASK)

/*! @name MACLONGADDRS1_LSB - MAC LONG ADDRESS 1 LSB */
#define ZLL_MACLONGADDRS1_LSB_MACLONGADDRS1_LSB_MASK (0xFFFFFFFFU)
#define ZLL_MACLONGADDRS1_LSB_MACLONGADDRS1_LSB_SHIFT (0U)
#define ZLL_MACLONGADDRS1_LSB_MACLONGADDRS1_LSB(x) (((uint32_t)(((uint32_t)(x)) << ZLL_MACLONGADDRS1_LSB_MACLONGADDRS1_LSB_SHIFT)) & ZLL_MACLONGADDRS1_LSB_MACLONGADDRS1_LSB_MASK)

/*! @name MACLONGADDRS1_MSB - MAC LONG ADDRESS 1 MSB */
#define ZLL_MACLONGADDRS1_MSB_MACLONGADDRS1_MSB_MASK (0xFFFFFFFFU)
#define ZLL_MACLONGADDRS1_MSB_MACLONGADDRS1_MSB_SHIFT (0U)
#define ZLL_MACLONGADDRS1_MSB_MACLONGADDRS1_MSB(x) (((uint32_t)(((uint32_t)(x)) << ZLL_MACLONGADDRS1_MSB_MACLONGADDRS1_MSB_SHIFT)) & ZLL_MACLONGADDRS1_MSB_MACLONGADDRS1_MSB_MASK)

/*! @name DUAL_PAN_CTRL - DUAL PAN CONTROL */
#define ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_MASK    (0x1U)
#define ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_SHIFT   (0U)
#define ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_SHIFT)) & ZLL_DUAL_PAN_CTRL_ACTIVE_NETWORK_MASK)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_MASK     (0x2U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_SHIFT    (1U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO(x)       (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_SHIFT)) & ZLL_DUAL_PAN_CTRL_DUAL_PAN_AUTO_MASK)
#define ZLL_DUAL_PAN_CTRL_PANCORDNTR1_MASK       (0x4U)
#define ZLL_DUAL_PAN_CTRL_PANCORDNTR1_SHIFT      (2U)
#define ZLL_DUAL_PAN_CTRL_PANCORDNTR1(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_PANCORDNTR1_SHIFT)) & ZLL_DUAL_PAN_CTRL_PANCORDNTR1_MASK)
#define ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK_MASK   (0x8U)
#define ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK_SHIFT  (3U)
#define ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK_SHIFT)) & ZLL_DUAL_PAN_CTRL_CURRENT_NETWORK_MASK)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_EN_MASK (0x10U)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_EN_SHIFT (4U)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_EN(x)  (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_EN_SHIFT)) & ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_EN_MASK)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_SEL_MASK (0x20U)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_SEL_SHIFT (5U)
#define ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_SEL(x) (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_SEL_SHIFT)) & ZLL_DUAL_PAN_CTRL_ZB_DP_CHAN_OVRD_SEL_MASK)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_MASK    (0xFF00U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_SHIFT   (8U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_SHIFT)) & ZLL_DUAL_PAN_CTRL_DUAL_PAN_DWELL_MASK)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_MASK   (0x3F0000U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_SHIFT  (16U)
#define ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_SHIFT)) & ZLL_DUAL_PAN_CTRL_DUAL_PAN_REMAIN_MASK)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_MASK      (0x400000U)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_SHIFT     (22U)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_SHIFT)) & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN0_MASK)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_MASK      (0x800000U)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_SHIFT     (23U)
#define ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_SHIFT)) & ZLL_DUAL_PAN_CTRL_RECD_ON_PAN1_MASK)

/*! @name CHANNEL_NUM1 - CHANNEL NUMBER 1 */
#define ZLL_CHANNEL_NUM1_CHANNEL_NUM1_MASK       (0x7FU)
#define ZLL_CHANNEL_NUM1_CHANNEL_NUM1_SHIFT      (0U)
#define ZLL_CHANNEL_NUM1_CHANNEL_NUM1(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_CHANNEL_NUM1_CHANNEL_NUM1_SHIFT)) & ZLL_CHANNEL_NUM1_CHANNEL_NUM1_MASK)

/*! @name SAM_CTRL - SAM CONTROL */
#define ZLL_SAM_CTRL_SAP0_EN_MASK                (0x1U)
#define ZLL_SAM_CTRL_SAP0_EN_SHIFT               (0U)
#define ZLL_SAM_CTRL_SAP0_EN(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAP0_EN_SHIFT)) & ZLL_SAM_CTRL_SAP0_EN_MASK)
#define ZLL_SAM_CTRL_SAA0_EN_MASK                (0x2U)
#define ZLL_SAM_CTRL_SAA0_EN_SHIFT               (1U)
#define ZLL_SAM_CTRL_SAA0_EN(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAA0_EN_SHIFT)) & ZLL_SAM_CTRL_SAA0_EN_MASK)
#define ZLL_SAM_CTRL_SAP1_EN_MASK                (0x4U)
#define ZLL_SAM_CTRL_SAP1_EN_SHIFT               (2U)
#define ZLL_SAM_CTRL_SAP1_EN(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAP1_EN_SHIFT)) & ZLL_SAM_CTRL_SAP1_EN_MASK)
#define ZLL_SAM_CTRL_SAA1_EN_MASK                (0x8U)
#define ZLL_SAM_CTRL_SAA1_EN_SHIFT               (3U)
#define ZLL_SAM_CTRL_SAA1_EN(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAA1_EN_SHIFT)) & ZLL_SAM_CTRL_SAA1_EN_MASK)
#define ZLL_SAM_CTRL_SAA0_START_MASK             (0xFF00U)
#define ZLL_SAM_CTRL_SAA0_START_SHIFT            (8U)
#define ZLL_SAM_CTRL_SAA0_START(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAA0_START_SHIFT)) & ZLL_SAM_CTRL_SAA0_START_MASK)
#define ZLL_SAM_CTRL_SAP1_START_MASK             (0xFF0000U)
#define ZLL_SAM_CTRL_SAP1_START_SHIFT            (16U)
#define ZLL_SAM_CTRL_SAP1_START(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAP1_START_SHIFT)) & ZLL_SAM_CTRL_SAP1_START_MASK)
#define ZLL_SAM_CTRL_SAA1_START_MASK             (0xFF000000U)
#define ZLL_SAM_CTRL_SAA1_START_SHIFT            (24U)
#define ZLL_SAM_CTRL_SAA1_START(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_CTRL_SAA1_START_SHIFT)) & ZLL_SAM_CTRL_SAA1_START_MASK)

/*! @name SAM_TABLE - SOURCE ADDRESS MANAGEMENT TABLE */
#define ZLL_SAM_TABLE_SAM_INDEX_MASK             (0x7FU)
#define ZLL_SAM_TABLE_SAM_INDEX_SHIFT            (0U)
#define ZLL_SAM_TABLE_SAM_INDEX(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_INDEX_SHIFT)) & ZLL_SAM_TABLE_SAM_INDEX_MASK)
#define ZLL_SAM_TABLE_SAM_INDEX_WR_MASK          (0x80U)
#define ZLL_SAM_TABLE_SAM_INDEX_WR_SHIFT         (7U)
#define ZLL_SAM_TABLE_SAM_INDEX_WR(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_INDEX_WR_SHIFT)) & ZLL_SAM_TABLE_SAM_INDEX_WR_MASK)
#define ZLL_SAM_TABLE_SAM_CHECKSUM_MASK          (0xFFFF00U)
#define ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT         (8U)
#define ZLL_SAM_TABLE_SAM_CHECKSUM(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_CHECKSUM_SHIFT)) & ZLL_SAM_TABLE_SAM_CHECKSUM_MASK)
#define ZLL_SAM_TABLE_SAM_INDEX_INV_MASK         (0x1000000U)
#define ZLL_SAM_TABLE_SAM_INDEX_INV_SHIFT        (24U)
#define ZLL_SAM_TABLE_SAM_INDEX_INV(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_INDEX_INV_SHIFT)) & ZLL_SAM_TABLE_SAM_INDEX_INV_MASK)
#define ZLL_SAM_TABLE_SAM_INDEX_EN_MASK          (0x2000000U)
#define ZLL_SAM_TABLE_SAM_INDEX_EN_SHIFT         (25U)
#define ZLL_SAM_TABLE_SAM_INDEX_EN(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_INDEX_EN_SHIFT)) & ZLL_SAM_TABLE_SAM_INDEX_EN_MASK)
#define ZLL_SAM_TABLE_ACK_FRM_PND_MASK           (0x4000000U)
#define ZLL_SAM_TABLE_ACK_FRM_PND_SHIFT          (26U)
#define ZLL_SAM_TABLE_ACK_FRM_PND(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_ACK_FRM_PND_SHIFT)) & ZLL_SAM_TABLE_ACK_FRM_PND_MASK)
#define ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK      (0x8000000U)
#define ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_SHIFT     (27U)
#define ZLL_SAM_TABLE_ACK_FRM_PND_CTRL(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_SHIFT)) & ZLL_SAM_TABLE_ACK_FRM_PND_CTRL_MASK)
#define ZLL_SAM_TABLE_FIND_FREE_IDX_MASK         (0x10000000U)
#define ZLL_SAM_TABLE_FIND_FREE_IDX_SHIFT        (28U)
#define ZLL_SAM_TABLE_FIND_FREE_IDX(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_FIND_FREE_IDX_SHIFT)) & ZLL_SAM_TABLE_FIND_FREE_IDX_MASK)
#define ZLL_SAM_TABLE_INVALIDATE_ALL_MASK        (0x20000000U)
#define ZLL_SAM_TABLE_INVALIDATE_ALL_SHIFT       (29U)
#define ZLL_SAM_TABLE_INVALIDATE_ALL(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_INVALIDATE_ALL_SHIFT)) & ZLL_SAM_TABLE_INVALIDATE_ALL_MASK)
#define ZLL_SAM_TABLE_SAM_BUSY_MASK              (0x80000000U)
#define ZLL_SAM_TABLE_SAM_BUSY_SHIFT             (31U)
#define ZLL_SAM_TABLE_SAM_BUSY(x)                (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_TABLE_SAM_BUSY_SHIFT)) & ZLL_SAM_TABLE_SAM_BUSY_MASK)

/*! @name SAM_MATCH - SAM MATCH */
#define ZLL_SAM_MATCH_SAP0_MATCH_MASK            (0x7FU)
#define ZLL_SAM_MATCH_SAP0_MATCH_SHIFT           (0U)
#define ZLL_SAM_MATCH_SAP0_MATCH(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAP0_MATCH_SHIFT)) & ZLL_SAM_MATCH_SAP0_MATCH_MASK)
#define ZLL_SAM_MATCH_SAP0_ADDR_PRESENT_MASK     (0x80U)
#define ZLL_SAM_MATCH_SAP0_ADDR_PRESENT_SHIFT    (7U)
#define ZLL_SAM_MATCH_SAP0_ADDR_PRESENT(x)       (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAP0_ADDR_PRESENT_SHIFT)) & ZLL_SAM_MATCH_SAP0_ADDR_PRESENT_MASK)
#define ZLL_SAM_MATCH_SAA0_MATCH_MASK            (0x7F00U)
#define ZLL_SAM_MATCH_SAA0_MATCH_SHIFT           (8U)
#define ZLL_SAM_MATCH_SAA0_MATCH(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAA0_MATCH_SHIFT)) & ZLL_SAM_MATCH_SAA0_MATCH_MASK)
#define ZLL_SAM_MATCH_SAA0_ADDR_ABSENT_MASK      (0x8000U)
#define ZLL_SAM_MATCH_SAA0_ADDR_ABSENT_SHIFT     (15U)
#define ZLL_SAM_MATCH_SAA0_ADDR_ABSENT(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAA0_ADDR_ABSENT_SHIFT)) & ZLL_SAM_MATCH_SAA0_ADDR_ABSENT_MASK)
#define ZLL_SAM_MATCH_SAP1_MATCH_MASK            (0x7F0000U)
#define ZLL_SAM_MATCH_SAP1_MATCH_SHIFT           (16U)
#define ZLL_SAM_MATCH_SAP1_MATCH(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAP1_MATCH_SHIFT)) & ZLL_SAM_MATCH_SAP1_MATCH_MASK)
#define ZLL_SAM_MATCH_SAP1_ADDR_PRESENT_MASK     (0x800000U)
#define ZLL_SAM_MATCH_SAP1_ADDR_PRESENT_SHIFT    (23U)
#define ZLL_SAM_MATCH_SAP1_ADDR_PRESENT(x)       (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAP1_ADDR_PRESENT_SHIFT)) & ZLL_SAM_MATCH_SAP1_ADDR_PRESENT_MASK)
#define ZLL_SAM_MATCH_SAA1_MATCH_MASK            (0x7F000000U)
#define ZLL_SAM_MATCH_SAA1_MATCH_SHIFT           (24U)
#define ZLL_SAM_MATCH_SAA1_MATCH(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAA1_MATCH_SHIFT)) & ZLL_SAM_MATCH_SAA1_MATCH_MASK)
#define ZLL_SAM_MATCH_SAA1_ADDR_ABSENT_MASK      (0x80000000U)
#define ZLL_SAM_MATCH_SAA1_ADDR_ABSENT_SHIFT     (31U)
#define ZLL_SAM_MATCH_SAA1_ADDR_ABSENT(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_MATCH_SAA1_ADDR_ABSENT_SHIFT)) & ZLL_SAM_MATCH_SAA1_ADDR_ABSENT_MASK)

/*! @name SAM_FREE_IDX - SAM FREE INDEX */
#define ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_MASK  (0xFFU)
#define ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_SHIFT (0U)
#define ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_SHIFT)) & ZLL_SAM_FREE_IDX_SAP0_1ST_FREE_IDX_MASK)
#define ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_MASK  (0xFF00U)
#define ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_SHIFT (8U)
#define ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_SHIFT)) & ZLL_SAM_FREE_IDX_SAA0_1ST_FREE_IDX_MASK)
#define ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_MASK  (0xFF0000U)
#define ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_SHIFT (16U)
#define ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_SHIFT)) & ZLL_SAM_FREE_IDX_SAP1_1ST_FREE_IDX_MASK)
#define ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_MASK  (0xFF000000U)
#define ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_SHIFT (24U)
#define ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_SHIFT)) & ZLL_SAM_FREE_IDX_SAA1_1ST_FREE_IDX_MASK)

/*! @name SEQ_CTRL_STS - SEQUENCE CONTROL AND STATUS */
#define ZLL_SEQ_CTRL_STS_CLR_NEW_SEQ_INHIBIT_MASK (0x4U)
#define ZLL_SEQ_CTRL_STS_CLR_NEW_SEQ_INHIBIT_SHIFT (2U)
#define ZLL_SEQ_CTRL_STS_CLR_NEW_SEQ_INHIBIT(x)  (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_CLR_NEW_SEQ_INHIBIT_SHIFT)) & ZLL_SEQ_CTRL_STS_CLR_NEW_SEQ_INHIBIT_MASK)
#define ZLL_SEQ_CTRL_STS_EVENT_TMR_DO_NOT_LATCH_MASK (0x8U)
#define ZLL_SEQ_CTRL_STS_EVENT_TMR_DO_NOT_LATCH_SHIFT (3U)
#define ZLL_SEQ_CTRL_STS_EVENT_TMR_DO_NOT_LATCH(x) (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_EVENT_TMR_DO_NOT_LATCH_SHIFT)) & ZLL_SEQ_CTRL_STS_EVENT_TMR_DO_NOT_LATCH_MASK)
#define ZLL_SEQ_CTRL_STS_LATCH_PREAMBLE_MASK     (0x10U)
#define ZLL_SEQ_CTRL_STS_LATCH_PREAMBLE_SHIFT    (4U)
#define ZLL_SEQ_CTRL_STS_LATCH_PREAMBLE(x)       (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_LATCH_PREAMBLE_SHIFT)) & ZLL_SEQ_CTRL_STS_LATCH_PREAMBLE_MASK)
#define ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE_MASK      (0x20U)
#define ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE_SHIFT     (5U)
#define ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE_SHIFT)) & ZLL_SEQ_CTRL_STS_NO_RX_RECYCLE_MASK)
#define ZLL_SEQ_CTRL_STS_FORCE_CRC_ERROR_MASK    (0x40U)
#define ZLL_SEQ_CTRL_STS_FORCE_CRC_ERROR_SHIFT   (6U)
#define ZLL_SEQ_CTRL_STS_FORCE_CRC_ERROR(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_FORCE_CRC_ERROR_SHIFT)) & ZLL_SEQ_CTRL_STS_FORCE_CRC_ERROR_MASK)
#define ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK      (0x80U)
#define ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_SHIFT     (7U)
#define ZLL_SEQ_CTRL_STS_CONTINUOUS_EN(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_SHIFT)) & ZLL_SEQ_CTRL_STS_CONTINUOUS_EN_MASK)
#define ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_MASK      (0x700U)
#define ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_SHIFT     (8U)
#define ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL(x)        (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_SHIFT)) & ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_MASK)
#define ZLL_SEQ_CTRL_STS_SEQ_IDLE_MASK           (0x800U)
#define ZLL_SEQ_CTRL_STS_SEQ_IDLE_SHIFT          (11U)
#define ZLL_SEQ_CTRL_STS_SEQ_IDLE(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_SEQ_IDLE_SHIFT)) & ZLL_SEQ_CTRL_STS_SEQ_IDLE_MASK)
#define ZLL_SEQ_CTRL_STS_NEW_SEQ_INHIBIT_MASK    (0x1000U)
#define ZLL_SEQ_CTRL_STS_NEW_SEQ_INHIBIT_SHIFT   (12U)
#define ZLL_SEQ_CTRL_STS_NEW_SEQ_INHIBIT(x)      (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_NEW_SEQ_INHIBIT_SHIFT)) & ZLL_SEQ_CTRL_STS_NEW_SEQ_INHIBIT_MASK)
#define ZLL_SEQ_CTRL_STS_RX_TIMEOUT_PENDING_MASK (0x2000U)
#define ZLL_SEQ_CTRL_STS_RX_TIMEOUT_PENDING_SHIFT (13U)
#define ZLL_SEQ_CTRL_STS_RX_TIMEOUT_PENDING(x)   (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_RX_TIMEOUT_PENDING_SHIFT)) & ZLL_SEQ_CTRL_STS_RX_TIMEOUT_PENDING_MASK)
#define ZLL_SEQ_CTRL_STS_RX_MODE_MASK            (0x4000U)
#define ZLL_SEQ_CTRL_STS_RX_MODE_SHIFT           (14U)
#define ZLL_SEQ_CTRL_STS_RX_MODE(x)              (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_RX_MODE_SHIFT)) & ZLL_SEQ_CTRL_STS_RX_MODE_MASK)
#define ZLL_SEQ_CTRL_STS_TMR2_SEQ_TRIG_ARMED_MASK (0x8000U)
#define ZLL_SEQ_CTRL_STS_TMR2_SEQ_TRIG_ARMED_SHIFT (15U)
#define ZLL_SEQ_CTRL_STS_TMR2_SEQ_TRIG_ARMED(x)  (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_TMR2_SEQ_TRIG_ARMED_SHIFT)) & ZLL_SEQ_CTRL_STS_TMR2_SEQ_TRIG_ARMED_MASK)
#define ZLL_SEQ_CTRL_STS_SEQ_T_STATUS_MASK       (0x3F0000U)
#define ZLL_SEQ_CTRL_STS_SEQ_T_STATUS_SHIFT      (16U)
#define ZLL_SEQ_CTRL_STS_SEQ_T_STATUS(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_SEQ_T_STATUS_SHIFT)) & ZLL_SEQ_CTRL_STS_SEQ_T_STATUS_MASK)
#define ZLL_SEQ_CTRL_STS_SW_ABORTED_MASK         (0x1000000U)
#define ZLL_SEQ_CTRL_STS_SW_ABORTED_SHIFT        (24U)
#define ZLL_SEQ_CTRL_STS_SW_ABORTED(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_SW_ABORTED_SHIFT)) & ZLL_SEQ_CTRL_STS_SW_ABORTED_MASK)
#define ZLL_SEQ_CTRL_STS_TC3_ABORTED_MASK        (0x2000000U)
#define ZLL_SEQ_CTRL_STS_TC3_ABORTED_SHIFT       (25U)
#define ZLL_SEQ_CTRL_STS_TC3_ABORTED(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_TC3_ABORTED_SHIFT)) & ZLL_SEQ_CTRL_STS_TC3_ABORTED_MASK)
#define ZLL_SEQ_CTRL_STS_PLL_ABORTED_MASK        (0x4000000U)
#define ZLL_SEQ_CTRL_STS_PLL_ABORTED_SHIFT       (26U)
#define ZLL_SEQ_CTRL_STS_PLL_ABORTED(x)          (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_CTRL_STS_PLL_ABORTED_SHIFT)) & ZLL_SEQ_CTRL_STS_PLL_ABORTED_MASK)

/*! @name ACKDELAY - ACK DELAY */
#define ZLL_ACKDELAY_ACKDELAY_MASK               (0x3FU)
#define ZLL_ACKDELAY_ACKDELAY_SHIFT              (0U)
#define ZLL_ACKDELAY_ACKDELAY(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_ACKDELAY_ACKDELAY_SHIFT)) & ZLL_ACKDELAY_ACKDELAY_MASK)
#define ZLL_ACKDELAY_TXDELAY_MASK                (0x3F00U)
#define ZLL_ACKDELAY_TXDELAY_SHIFT               (8U)
#define ZLL_ACKDELAY_TXDELAY(x)                  (((uint32_t)(((uint32_t)(x)) << ZLL_ACKDELAY_TXDELAY_SHIFT)) & ZLL_ACKDELAY_TXDELAY_MASK)

/*! @name FILTERFAIL_CODE - FILTER FAIL CODE */
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_CODE_MASK (0x3FFU)
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_CODE_SHIFT (0U)
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_CODE(x)   (((uint32_t)(((uint32_t)(x)) << ZLL_FILTERFAIL_CODE_FILTERFAIL_CODE_SHIFT)) & ZLL_FILTERFAIL_CODE_FILTERFAIL_CODE_MASK)
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_PAN_SEL_MASK (0x8000U)
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_PAN_SEL_SHIFT (15U)
#define ZLL_FILTERFAIL_CODE_FILTERFAIL_PAN_SEL(x) (((uint32_t)(((uint32_t)(x)) << ZLL_FILTERFAIL_CODE_FILTERFAIL_PAN_SEL_SHIFT)) & ZLL_FILTERFAIL_CODE_FILTERFAIL_PAN_SEL_MASK)

/*! @name RX_WTR_MARK - RECEIVE WATER MARK */
#define ZLL_RX_WTR_MARK_RX_WTR_MARK_MASK         (0xFFU)
#define ZLL_RX_WTR_MARK_RX_WTR_MARK_SHIFT        (0U)
#define ZLL_RX_WTR_MARK_RX_WTR_MARK(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_RX_WTR_MARK_RX_WTR_MARK_SHIFT)) & ZLL_RX_WTR_MARK_RX_WTR_MARK_MASK)

/*! @name SLOT_PRELOAD - SLOT PRELOAD */
#define ZLL_SLOT_PRELOAD_SLOT_PRELOAD_MASK       (0xFFU)
#define ZLL_SLOT_PRELOAD_SLOT_PRELOAD_SHIFT      (0U)
#define ZLL_SLOT_PRELOAD_SLOT_PRELOAD(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_SLOT_PRELOAD_SLOT_PRELOAD_SHIFT)) & ZLL_SLOT_PRELOAD_SLOT_PRELOAD_MASK)

/*! @name SEQ_STATE - ZIGBEE SEQUENCE STATE */
#define ZLL_SEQ_STATE_SEQ_STATE_MASK             (0x1FU)
#define ZLL_SEQ_STATE_SEQ_STATE_SHIFT            (0U)
#define ZLL_SEQ_STATE_SEQ_STATE(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_SEQ_STATE_SHIFT)) & ZLL_SEQ_STATE_SEQ_STATE_MASK)
#define ZLL_SEQ_STATE_PREAMBLE_DET_MASK          (0x100U)
#define ZLL_SEQ_STATE_PREAMBLE_DET_SHIFT         (8U)
#define ZLL_SEQ_STATE_PREAMBLE_DET(x)            (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_PREAMBLE_DET_SHIFT)) & ZLL_SEQ_STATE_PREAMBLE_DET_MASK)
#define ZLL_SEQ_STATE_SFD_DET_MASK               (0x200U)
#define ZLL_SEQ_STATE_SFD_DET_SHIFT              (9U)
#define ZLL_SEQ_STATE_SFD_DET(x)                 (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_SFD_DET_SHIFT)) & ZLL_SEQ_STATE_SFD_DET_MASK)
#define ZLL_SEQ_STATE_FILTERFAIL_FLAG_SEL_MASK   (0x400U)
#define ZLL_SEQ_STATE_FILTERFAIL_FLAG_SEL_SHIFT  (10U)
#define ZLL_SEQ_STATE_FILTERFAIL_FLAG_SEL(x)     (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_FILTERFAIL_FLAG_SEL_SHIFT)) & ZLL_SEQ_STATE_FILTERFAIL_FLAG_SEL_MASK)
#define ZLL_SEQ_STATE_CRCVALID_MASK              (0x800U)
#define ZLL_SEQ_STATE_CRCVALID_SHIFT             (11U)
#define ZLL_SEQ_STATE_CRCVALID(x)                (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_CRCVALID_SHIFT)) & ZLL_SEQ_STATE_CRCVALID_MASK)
#define ZLL_SEQ_STATE_PLL_ABORT_MASK             (0x1000U)
#define ZLL_SEQ_STATE_PLL_ABORT_SHIFT            (12U)
#define ZLL_SEQ_STATE_PLL_ABORT(x)               (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_PLL_ABORT_SHIFT)) & ZLL_SEQ_STATE_PLL_ABORT_MASK)
#define ZLL_SEQ_STATE_PLL_ABORTED_MASK           (0x2000U)
#define ZLL_SEQ_STATE_PLL_ABORTED_SHIFT          (13U)
#define ZLL_SEQ_STATE_PLL_ABORTED(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_PLL_ABORTED_SHIFT)) & ZLL_SEQ_STATE_PLL_ABORTED_MASK)
#define ZLL_SEQ_STATE_RX_BYTE_COUNT_MASK         (0xFF0000U)
#define ZLL_SEQ_STATE_RX_BYTE_COUNT_SHIFT        (16U)
#define ZLL_SEQ_STATE_RX_BYTE_COUNT(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_RX_BYTE_COUNT_SHIFT)) & ZLL_SEQ_STATE_RX_BYTE_COUNT_MASK)
#define ZLL_SEQ_STATE_CCCA_BUSY_CNT_MASK         (0x3F000000U)
#define ZLL_SEQ_STATE_CCCA_BUSY_CNT_SHIFT        (24U)
#define ZLL_SEQ_STATE_CCCA_BUSY_CNT(x)           (((uint32_t)(((uint32_t)(x)) << ZLL_SEQ_STATE_CCCA_BUSY_CNT_SHIFT)) & ZLL_SEQ_STATE_CCCA_BUSY_CNT_MASK)

/*! @name TMR_PRESCALE - TIMER PRESCALER */
#define ZLL_TMR_PRESCALE_TMR_PRESCALE_MASK       (0x7U)
#define ZLL_TMR_PRESCALE_TMR_PRESCALE_SHIFT      (0U)
#define ZLL_TMR_PRESCALE_TMR_PRESCALE(x)         (((uint32_t)(((uint32_t)(x)) << ZLL_TMR_PRESCALE_TMR_PRESCALE_SHIFT)) & ZLL_TMR_PRESCALE_TMR_PRESCALE_MASK)

/*! @name LENIENCY_LSB - LENIENCY LSB */
#define ZLL_LENIENCY_LSB_LENIENCY_REGISTER_MASK  (0xFFFFFFFFU)
#define ZLL_LENIENCY_LSB_LENIENCY_REGISTER_SHIFT (0U)
#define ZLL_LENIENCY_LSB_LENIENCY_REGISTER(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_LENIENCY_LSB_LENIENCY_REGISTER_SHIFT)) & ZLL_LENIENCY_LSB_LENIENCY_REGISTER_MASK)

/*! @name LENIENCY_MSB - LENIENCY MSB */
#define ZLL_LENIENCY_MSB_LENIENCY_REGISTER_MASK  (0xFFU)
#define ZLL_LENIENCY_MSB_LENIENCY_REGISTER_SHIFT (0U)
#define ZLL_LENIENCY_MSB_LENIENCY_REGISTER(x)    (((uint32_t)(((uint32_t)(x)) << ZLL_LENIENCY_MSB_LENIENCY_REGISTER_SHIFT)) & ZLL_LENIENCY_MSB_LENIENCY_REGISTER_MASK)

/*! @name PART_ID - PART ID */
#define ZLL_PART_ID_PART_ID_MASK                 (0xFFU)
#define ZLL_PART_ID_PART_ID_SHIFT                (0U)
#define ZLL_PART_ID_PART_ID(x)                   (((uint32_t)(((uint32_t)(x)) << ZLL_PART_ID_PART_ID_SHIFT)) & ZLL_PART_ID_PART_ID_MASK)

/*! @name PKT_BUFFER - PACKET BUFFER */
#define ZLL_PKT_BUFFER_PKT_BUFFER_MASK           (0xFFFFFFFFU)
#define ZLL_PKT_BUFFER_PKT_BUFFER_SHIFT          (0U)
#define ZLL_PKT_BUFFER_PKT_BUFFER(x)             (((uint32_t)(((uint32_t)(x)) << ZLL_PKT_BUFFER_PKT_BUFFER_SHIFT)) & ZLL_PKT_BUFFER_PKT_BUFFER_MASK)

/* The count of ZLL_PKT_BUFFER */
#define ZLL_PKT_BUFFER_COUNT                     (32U)


/*!
 * @}
 */ /* end of group ZLL_Register_Masks */


/* ZLL - Peripheral instance base addresses */
/** Peripheral ZLL base address */
#define ZLL_BASE                                 (0x4005D000u)
/** Peripheral ZLL base pointer */
#define ZLL                                      ((ZLL_Type *)ZLL_BASE)
/** Array initializer of ZLL peripheral base addresses */
#define ZLL_BASE_ADDRS                           { ZLL_BASE }
/** Array initializer of ZLL peripheral base pointers */
#define ZLL_BASE_PTRS                            { ZLL }

/*!
 * @}
 */ /* end of group ZLL_Peripheral_Access_Layer */


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

#define DSPI0                     SPI0
#define DSPI1                     SPI1

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MKW30Z4_H_ */

