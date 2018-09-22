/**
******************************************************************************
* @file clock_map.h
* @brief CLOCK hw module register map
* @internal
* @author ON Semiconductor
* $Rev:  $
* $Date: 2014-11-26 $
******************************************************************************
* @copyright (c) 2012 ON Semiconductor. All rights reserved.
* ON Semiconductor is supplying this software for use with ON Semiconductor
* processor based microcontrollers only.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
* INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
* @endinternal
*
* @ingroup clock
*
* @details
*/

#ifndef CLOCK_MAP_H_
#define CLOCK_MAP_H_

/*************************************************************************************************
*                                                                                                *
*  Header files                                                                                  *
*                                                                                                *
*************************************************************************************************/


/**************************************************************************************************
*                                                                                                 *
*  Type definitions                                                                               *
*                                                                                                 *
**************************************************************************************************/

/** Clock control HW structure overlay */
typedef struct {
	union {
		struct {
			__IO uint32_t OSC_SEL:1;
			__IO uint32_t PAD0:1;
			__IO uint32_t CAL32K:1;
			__IO uint32_t CAL32M:1;
			__IO uint32_t RTCEN:1;
		} BITS;
		__IO uint32_t WORD;
	} CCR; 													/**< 0x4001B000 Clock control register */
	union {
		struct {
			__I uint32_t XTAL32M:1;
			__I uint32_t XTAL32K:1;
			__I uint32_t CAL32K:1;
			__I uint32_t DONE32K:1;
			__I uint32_t CAL32MFAIL:1;
			__I uint32_t CAL32MDONE:1;
		} BITS;
		__I uint32_t WORD;
	} CSR; 													/**< 0x4001B004 Clock status register */
	union {
		struct {
			__IO uint32_t IE32K:1;
			__IO uint32_t IE32M:1;
		} BITS;
		__IO uint32_t WORD;
	} IER;													/**< 0x4001B008 Interrup enable register */
	__IO uint32_t ICR; 										/**< 0x4001B00C Interrupt clear register */
	union {
		struct {
			__IO uint32_t TIMER0:1;
			__IO uint32_t TIMER1:1;
			__IO uint32_t TIMER2:1;
			__IO uint32_t PAD0:2;
			__IO uint32_t UART1:1;
			__IO uint32_t SPI:1;
			__IO uint32_t I2C:1;
			__IO uint32_t UART2:1;
			__IO uint32_t PAD1:1;
			__IO uint32_t WDOG:1;
			__IO uint32_t PWM:1;
			__IO uint32_t GPIO:1;
			__IO uint32_t PAD2:2;
			__IO uint32_t RTC:1;
			__IO uint32_t XBAR:1;
			__IO uint32_t RAND:1;
			__IO uint32_t PAD3:2;
			__IO uint32_t MACHW:1;
			__IO uint32_t ADC:1;
			__IO uint32_t AES:1;
			__IO uint32_t FLASH:1;
			__IO uint32_t PAD4:1;
			__IO uint32_t RFANA:1;
			__IO uint32_t IO:1;
			__IO uint32_t PAD5:1;
			__IO uint32_t PAD:1;
			__IO uint32_t PMU:1;
			__IO uint32_t PAD6:1;
			__IO uint32_t TEST:1;
		} BITS;
		__IO uint32_t WORD;
	} PDIS; 												/**< 0x4001B010 Periphery disable */
	__IO uint32_t FDIV; 									/**< 0x4001B014 FCLK divider */
	__IO uint32_t TDIV; 									/**< 0x4001B01C Traceclk divider */
	__IO uint32_t WDIV; 									/**< 0x4001B020 Watchdog clock divider */
	__IO uint32_t TRIM_32M_INT; 							/**< 0x4001B024 32Mhz internal trim */
	__IO uint32_t TRIM_32K_INT; 							/**< 0x4001B02C 32kHz internal trim */
	__IO uint32_t TRIM_32M_EXT; 							/**< 0x4001B030 32Mhz external trim */
	__IO uint32_t TRIM_32K_EXT; 							/**< 0x4001B034 32Khz external trim */
	union {
		struct {
			__IO uint32_t OV32M;
			__IO uint32_t EN32M;
			__IO uint32_t OV32K;
			__IO uint32_t EN32K;
		} BITS;
		__IO uint32_t WORD;
	} CER; 													/**< 0x4001B038 clock enable register*/
} ClockReg_t, *ClockReg_pt;

#endif /* CLOCK_MAP_H_ */
