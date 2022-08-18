/* Flash OS Routines
 * Copyright (c) 2009-2015 ARM Limited
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

#include "FlashOS.h"

#define IAP_ENTRY   0x1FFF1001

#define IAP_ERAS        0x010
#define IAP_ERAS_DAT0   (IAP_ERAS + 0)
#define IAP_ERAS_DAT1   (IAP_ERAS + 1)
#define IAP_ERAS_SECT   (IAP_ERAS + 2)
#define IAP_ERAS_BLCK   (IAP_ERAS + 3)
#define IAP_ERAS_CHIP   (IAP_ERAS + 4)
#define IAP_ERAS_MASS   (IAP_ERAS + 5)

#define IAP_PROG        0x020
#define IAP_PROG_DAT0   (IAP_PROG + 0)
#define IAP_PROG_DAT1   (IAP_PROG + 1)
#define IAP_PROG_CODE   (IAP_PROG + 2)

void DO_IAP(unsigned long id, unsigned long dst_addr, unsigned char* src_addr, unsigned long size)
{
    (*((volatile uint32_t *)(0xE000ED04))) = 0x00000000; // ICSR(Interrupt Control and State Register) of SCB(SystemControlBlock)
    (*((volatile uint32_t *)(0xE000E180))) = 0xffffffff; // ICER(Interrupt Clear-enable Register) of NVIC(Nested Vectored Interrupt Controller)
    (*((volatile uint32_t *)(0xE000E010))) &= ~(0x01);   // SYST_CSR ( SystTick Control and Status Register)
    
    ((void(*)(unsigned long,unsigned long,unsigned char*,unsigned long))IAP_ENTRY)(id,dst_addr,src_addr,size);
}

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) 
{
	return(0);
}

int UnInit (unsigned long fnc) 
{
  return (0);
}

int EraseChip (void) 
{
    DO_IAP(IAP_ERAS_CHIP,0,0,0);
    return (0);                                    // Finished without Errors
}

int EraseSector (unsigned long adr) 
{
    DO_IAP(IAP_ERAS_SECT,adr,0,0);
    
    return (0);                                  // Finished without Errors
}

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) 
{
     DO_IAP(IAP_PROG_CODE,adr,(unsigned char*)buf,sz);

    return (0);                                  // Finished without Errors
}

