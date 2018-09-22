/*
** ###################################################################
**     Processors:          MKL02Z16VFG4
**                          MKL02Z16VFK4
**                          MKL02Z16VFM4
**                          MKL02Z32CAF4
**                          MKL02Z32VFG4
**                          MKL02Z32VFK4
**                          MKL02Z32VFM4
**                          MKL02Z8VFG4
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manuals:   KL02P20M48SF0RM Rev2.1, July 2013
**                          KL02P32M48SF0RM Rev3.1, July 2013
**
**     Version:             rev. 1.6, 2015-07-29
**     Build:               b151217
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright (c) 2015 Freescale Semiconductor, Inc.
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
**     - rev. 1.0 (2012-10-04)
**         Initial version.
**     - rev. 1.1 (2013-04-05)
**         Changed start of doxygen comment.
**     - rev. 1.2 (2014-02-10)
**         Access restriction of some registers fixed.
**         Startup file for gcc has been updated according to CMSIS 3.2.
**         The declaration of clock configurations has been moved to separate header file system_MKL02Z4.h
**     - rev. 1.3 (2014-10-14)
**         Renamed interrupt vector LPTimer to LPTMR0
**     - rev. 1.4 (2015-01-22)
**         SystemInit() and  SystemCoreClockUpdate() implementation updated.
**     - rev. 1.5 (2015-01-23)
**         Add default value for DEFAULT_SYSTEM_CLOCK.
**     - rev. 1.6 (2015-07-29)
**         Correction of backward compatibility.
**
** ###################################################################
*/

/*!
 * @file MKL02Z4
 * @version 1.6
 * @date 2015-07-29
 * @brief Device specific configuration file for MKL02Z4 (header file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#ifndef _SYSTEM_MKL02Z4_H_
#define _SYSTEM_MKL02Z4_H_                       /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#ifndef DISABLE_WDOG
  #define DISABLE_WDOG                 1
#endif

/* Define clock source values */
#define CPU_XTAL_CLK_HZ                32768U              /* Value of the external crystal or oscillator clock frequency of the system oscillator (OSC) in Hz */
#define CPU_INT_SLOW_CLK_HZ            32768U              /* Value of the slow internal oscillator clock frequency in Hz */
#define CPU_INT_FAST_CLK_HZ            4000000U            /* Value of the fast internal oscillator clock frequency in Hz */

/* RTC oscillator setting */

/* Low power mode enable */
/* SMC_PMPROT: AVLP=1,AVLLS=0 */
#define SYSTEM_SMC_PMPROT_VALUE        0x22U               /* SMC_PMPROT */

#define DEFAULT_SYSTEM_CLOCK           20971520U           /* Default System clock value */


/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (void);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif  /* _SYSTEM_MKL02Z4_H_ */
