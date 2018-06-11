/**
 *
 * @file flash_map.h
 * @author Stef Servaes, Pierre Lebas
 * $Rev: 346 $
 * $Date: 2012-04-23 14:47:52 +0200 (Mon, 23 Apr 2012) $
 * @brief Flash controller HW register map
 ******************************************************************************
 * @copyright (c) 2012 ON Semiconductor. All rights reserved.
 * @internal
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
 * @ingroup FLASH
 *
 * @details
 * <p>
 * Flash controller HW register map description
 * </p>
 *
 */

//#ifdef CM3
#include "system_ARMCM3.h"
#include "core_cm3.h"
//#endif
#ifdef CM0
#include <system_ARMCM0.h>
#include <core_cm0.h>
#endif

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

#ifndef FLASH_MAP_H_
#define FLASH_MAP_H_

typedef struct {
  union {
    struct {
      __I uint32_t FLASH_A_BUSY:1;		/**< Busy A */
      __I uint32_t FLASH_B_BUSY:1;		/**< Busy B */
      __I uint32_t FLASH_A_UNLOCK:1;	/**< Unlock A */
      __I uint32_t FLASH_B_UNLOCK:1;	/**< Unlock B */
      __I uint32_t FLASH_ERROR:3;		/**< Attempt to erase bootloader */
    } BITS;
    __I uint32_t WORD;
  } STATUS;
  union {
	  struct {
		  __IO uint32_t FLASHB_PD:1;
		  __IO uint32_t FLASHA_PD:1;
		  __IO uint32_t REMAP:1;
		  __IO uint32_t WR_INT_EN:1;
		  __IO uint32_t ERASE_INT_EN:1;
		  __IO uint32_t ERROR_INT_EN:1;
		  __IO uint32_t WRITE_BLOCK:1;
	  } BITS;
	  __IO uint32_t WORD;
  } CONTROL;
  union {
    struct {
      __IO uint32_t PAGEERASE:1;   /**< Erase a single page */
      __IO uint32_t MASSERASE:1;   /**< MASS Erase */
    } BITS;
    __IO uint32_t WORD;
  } COMMAND;
  __IO uint32_t ADDR;
  __IO uint32_t UNLOCK1;
  __IO uint32_t UNLOCKA;
  __IO uint32_t UNLOCKB;
  union {
	  struct {
		  __I uint32_t INT_PEND:1; // Interrupt pending
		  __I uint32_t INT_TYPE:3; // Interrupt type
	  } BITS;
	  __I uint32_t WORD;
  } INT_STATUS;
} FlashReg_t, *FlashReg_pt;

#endif /* FLASH_MAP_H_ */
