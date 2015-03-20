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
#include "FlashPrg.h"

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    // Called to configure the SoC. Should enable clocks
    //  watchdogs, peripherals and anything else needed to
    //  access or program memory. Fnc parameter has meaning
    //  but currently isnt used in MSC programming routines
    return 0;
}

uint32_t UnInit(uint32_t fnc)
{
    // When a session is complete this is called to powerdown
    //  communication channels and clocks that were enabled
    //  Fnc parameter has meaning but isnt used in MSC program
    //  routines
    return 0;
}

uint32_t BlankCheck(uint32_t adr, uint32_t sz, uint8_t pat)
{
    // Check that the memory at address adr for length sz is 
    //  empty or the same as pat
    return 0;
}

uint32_t EraseChip(void)
{
    // Execute a sequence that erases the entire of flash memory region 
    return 0;
}

uint32_t EraseSector(uint32_t adr)
{
    // Execute a sequence that erases the sector that adr resides in
    return 0;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    // Program the contents of buf starting at adr for length of sz
    return 0;
}

uint32_t Verify(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    // Given an adr and sz compare this against the content of buf
    return 0;
}
