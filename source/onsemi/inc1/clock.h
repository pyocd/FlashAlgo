/**
******************************************************************************
* @file clock.h
* @brief Header of clock hw module functions
* @internal
* @author ON Semiconductor
* $Rev:  $
* $Date: 2015-11-26 $
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
*/

#ifndef CLOCK_H_
#define CLOCK_H_

/*************************************************************************************************
*                                                                                                *
*  Header files                                                                                  *
*                                                                                                *
*************************************************************************************************/

#include "types.h"
#include "clock_map.h"
/*************************************************************************************************
*                                                                                                *
*  Symbolic Constants                                                                            *
*                                                                                                *
*************************************************************************************************/

/** Peripherals clock disable defines  /
 * @details
 */
#define CLOCK_TIMER0	        (0x0)  /**< <b> Timer 0 clock enable offset </b>*/
#define CLOCK_TIMER1	        (0x1)  /**< <b> Timer 1 clock enable offset </b>: */
#define CLOCK_TIMER2	        (0x2)  /**< <b> Timer 2 clock enable offset </b>: */
#define CLOCK_PAD0_0	        (0x3)  /**< <b> Unused offset </b> */
#define CLOCK_PAD0_1	        (0x4)  /**< <b> Unused offset </b> */
#define CLOCK_UART1				(0x5)  /**< <b> UART 1 clock enable offset </b> */
#define CLOCK_SPI				(0x6)  /**< <b> SPI clock enable offset </b> */
#define CLOCK_I2C				(0x7)  /**< <b> I2C clock enable offset </b> */
#define CLOCK_UART2				(0x8)  /**< <b> UART 2 clock enable offset </b> */
#define CLOCK_SPI2	        	(0x9)  /**< <b> Unused offset </b>: */
#define CLOCK_WDOG				(0xA)  /**< <b> Watchdog clock enable offset </b> */
#define CLOCK_PWM				(0xB)  /**< <b> PWM clock enable offset </b> */
#define CLOCK_GPIO				(0xC)  /**< <b> GPIO clock enable offset </b> */
#define CLOCK_I2C2	        	(0xD)  /**< <b> Unused offset </b> */
#define CLOCK_PAD2_1	        (0xE)  /**< <b> Unused offset </b> */
#define CLOCK_RTC				(0xF)  /**< <b> RTC clock enable offset </b> */
#define CLOCK_CROSSB	        (0x10) /**< <b> Crossbar clock enable offset </b> */
#define CLOCK_RAND				(0x11) /**< <b> Randomizer clock enable offset </b> */
#define CLOCK_PAD3_0	        (0x12) /**< <b> Unused offset </b> */
#define CLOCK_PAD3_1	        (0x13) /**< <b> Unused offset </b> */
#define CLOCK_MACHW				(0x14) /**< <b> macHw clock enable offset </b> */
#define CLOCK_ADC				(0x15) /**< <b> ADC clock enable offset </b> */
#define CLOCK_AES				(0x16) /**< <b> AES clock enable offset </b> */
#define CLOCK_FLASH				(0x17) /**< <b> Flash controller clock enable offset</b> */
#define CLOCK_PAD4_0	        (0x18) /**< <b> Unused offset </b> */
#define CLOCK_RFANA				(0x19) /**< <b> rfAna clock enable offset </b> */
#define CLOCK_IO				(0x1A) /**< <b> IO clock enable offset </b> */
#define CLOCK_PAD5_0	        (0x1B) /**< <b> Unused offset </b> */
#define CLOCK_PAD				(0x1C) /**< <b> Pad clock enable offset </b> */
#define CLOCK_PMU				(0x1D) /**< <b> Pmu clock enable offset </b> */
#define CLOCK_DMA	        	(0x1E) /**< <b> DMA clock enable offset </b> */
#define CLOCK_TEST				(0x1F) /**< <b> Test controller clock enable offset </b> */

/** Clock Control HW Registers Offset */ 
#define CLOCKREG_BASE		((uint32_t)0x4001B000)
/** Clock Control HW Structure Overlay */ 
#define CLOCKREG			((ClockReg_pt)CLOCKREG_BASE)

#define CLOCK_ENABLE(a)		CLOCKREG->PDIS.WORD &= ~(1 << a)
#define CLOCK_DISABLE(a)	CLOCKREG->PDIS.WORD |= (uint32_t)(1 << a)

/*************************************************************************************************
*                                                                                                *
*  Functions                                                                                     *
*                                                                                                *
*************************************************************************************************/

/** Function to initialize clocks
 * @details
 * The function initializes clocks.
 * This initialization includes:
 * - Enable of external 32mHz oscillator
 * - Disable of all peripheral clocks (to be turned on selectively when used later in the application)
 * - Setting core frequency
 */
void fClockInit(void);

/** Function to get peripheral clock frequency
 * @details
 * The function checks and returns peripheral clock frequency
 * @return Peripheral clock frequency
 */
uint32_t fClockGetPeriphClockfrequency(void);

/** Function to get peripheral clock frequency
 * @details
 * The function checks which input clock is sourcing 32kHz clock domain.
 * This domain can be either sourced by:
 * - Internal 32kHz oscillator
 * - External 32.768kHz oscillator
 * @return 32kHz clock domain frequency
 */
uint16_t fClockGet32kClockfrequency(void);

/** Function to enable peripheral clock
 * @details
 * The function enables clock of peripheral indicated by parameter
 * @param deviceId Peripheral ID whose clock must be enabled.
 */
void fClockEnablePeriph(uint8_t deviceId);

/** Function to disable peripheral clock
 * @details
 * The function disables clock of peripheral indicated by parameter
 * @param deviceId ID Peripheral whose clock must be disabled.
 */
void fClockDisablePeriph(uint8_t deviceId);

#endif /* CLOCK_H_ */
