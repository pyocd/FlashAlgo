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
