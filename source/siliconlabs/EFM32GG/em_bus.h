/***************************************************************************//**
 * @file em_bus.h
 * @brief RAM and peripheral bit-field set and clear API
 * @version 3.20.2
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#ifndef __SILICON_LABS_EM_BUS__
#define __SILICON_LABS_EM_BUS__

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief
 *   Perform a single-bit write operation on a peripheral register
 *
 * @details
 *   This function uses Cortex-M bit-banding hardware to perform an atomic
 *   read-modify-write operation on a single register bit. Please refer to the
 *   reference manual for further details about bit-banding.
 *
 * @note
 *   This function is atomic on Cortex-M cores with bit-banding support. Bit-
 *   banding is a multicycle read-modify-write bus operation. Peripheral register
 *   bit-banding is performed using the memory alias region at BITBAND_PER_BASE.
 *
 * @param[in] addr Peripheral register address
 *
 * @param[in] bit Bit position to write, 0-31
 *
 * @param[in] val Value to set bit to, 0 or 1
 ******************************************************************************/
__STATIC_INLINE void BUS_RegBitWrite(volatile uint32_t *addr,
                                     unsigned int bit,
                                     unsigned int val)
{
#if defined( BITBAND_PER_BASE )
  uint32_t aliasAddr =
    BITBAND_PER_BASE + (((uint32_t)addr - PER_MEM_BASE) * 32) + (bit * 4);

  *(volatile uint32_t *)aliasAddr = (uint32_t)val;
#else
  uint32_t tmp = *addr;

  /* Make sure val is not more than 1, because we only want to set one bit. */
  *addr = (tmp & ~(1 << bit)) | ((val & 1) << bit);
#endif
}


/***************************************************************************//**
 * @brief
 *   Perform a single-bit read operation on a peripheral register
 *
 * @details
 *   This function uses Cortex-M bit-banding hardware to perform an atomic
 *   read operation on a single register bit. Please refer to the
 *   reference manual for further details about bit-banding.
 *
 * @note
 *   This function is atomic on Cortex-M cores with bit-banding support.
 *   Peripheral register bit-banding is performed using the memory alias
 *   region at BITBAND_PER_BASE.
 *
 * @param[in] addr Peripheral register address
 *
 * @param[in] bit Bit position to read, 0-31
 *
 * @return
 *     The requested bit shifted to bit position 0 in the return value
 ******************************************************************************/
__STATIC_INLINE unsigned int BUS_RegBitRead(volatile const uint32_t *addr,
                                            unsigned int bit)
{
#if defined( BITBAND_PER_BASE )
  uint32_t aliasAddr =
    BITBAND_PER_BASE + (((uint32_t)addr - PER_MEM_BASE) * 32) + (bit * 4);

  return *(volatile uint32_t *)aliasAddr;
#else
  return ((*addr) >> bit) & 1;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __SILICON_LABS_EM_BUS__ */
