/**
 ******************************************************************************
 * @file    types.h
 * @author  ON Semiconductor
 * $Rev:  $
 * $Date: 2015-11-15 $
 * @brief	(API) Public header of Flash driver
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
 *
 * <h1> General description </h1>
 * <p>
 * </p>
 */
#ifndef _types_h
#define _types_h

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>

typedef  unsigned       char   u8_t;
typedef    signed       char   i8_t;
typedef  unsigned      short  u16_t;
typedef    signed      short  i16_t;
typedef  unsigned       long  u32_t;
typedef    signed       long  i32_t;
typedef  unsigned  long long  u64_t;
typedef    signed  long long  i64_t;
typedef unsigned char boolean;

#define ASSERT(test)		((void)(test))

#define True		(1)
#define False		(0)
#define Null		(0)

#ifdef __cplusplus
  }
#endif

#endif
