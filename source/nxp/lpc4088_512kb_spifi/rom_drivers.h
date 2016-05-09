/* Definitions for ROM API for SPIFI in NXP MCUs
 * Copyright (c) 2010 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROM_DRIVERS_H_
#define ROM_DRIVERS_H_

#ifdef DIVROMD_PRESENT
#include "divfuncs.h"

typedef	struct _DIVD {
    int32_t (*sdiv) (int32_t, int32_t);
    uint32_t (*udiv) (int32_t, int32_t);
    sdiv_t (*sdivmod) (int32_t, int32_t);
    udiv_t (*udivmod) (uint32_t, uint32_t);
}  DIVD;
#endif /* DIVROMD_PRESENT */

#ifdef PWRROMD_PRESENT
#include "power_control.h"
#endif /* PWRROMD_PRESENT */

#ifdef USBROMD_PRESENT
#include "mw_usbd_rom_api.h"
#endif

#ifdef SPIFIROMD_PRESENT
#include "spifi_rom_api.h"
extern const SPIFI_RTNS spifi_table;
#endif

typedef	struct _ROM {
#ifdef USBROMD_PRESENT
   const USBD_API_T * pUSBD;
#else
   const unsigned p_usbd;
#endif /* USBROMD_PRESENT */

   const unsigned p_clib;
   const unsigned p_cand;

#ifdef PWRROMD_PRESENT
   const PWRD * pPWRD;
#else
   const unsigned p_pwrd;
#endif /* PWRROMD_PRESENT */

#ifdef DIVROMD_PRESENT
   const DIVD * pDIVD;
#else
   const unsigned p_promd;
#endif /* DIVROMD_PRESENT */

#ifdef SPIFIROMD_PRESENT
   const SPIFI_RTNS *pSPIFID;
#else
   const unsigned p_spifid;
#endif /*SPIFIROMD_PRESENT */

   const unsigned p_dev3;
   const unsigned p_dev4;
}  ROM;

#endif /*ROM_DRIVERS_H_*/
