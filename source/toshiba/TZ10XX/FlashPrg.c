/* Flash OS Routines
 * Copyright (c) 2016 Cerevo Inc.
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

/** @file FlashPrg.c */

#include "FlashOS.h"
#include "FlashPrg.h"

/* 
 * TZ10xx on chip NOR flash support functions. 
 */
#define SPIC_BASE_ADDR      (0x40004000)
#define GCNF_BASE_ADDR      (0x4004A000)
#define TIME_LIMIT          (8000)
#define TIME_LIMIT_MS       (10000)
#define CORE_CLOCK          (48000000)

#define REG_SPIC(offset)    (*((volatile uint32_t *)(SPIC_BASE_ADDR + (offset))))
#define REG_GCNF(offset)    (*((volatile uint32_t *)(GCNF_BASE_ADDR + (offset))))

/* local functions */

static void wait(uint32_t usec)
{
    uint32_t cnt;
    cnt = CORE_CLOCK / (1000000 * 4) * usec;
    do {
        __nop();
    } while (--cnt);
}

static int readCommand(uint32_t command, uint32_t *reg_val)
{
    uint32_t intr_stat;
    uint32_t cnt;
    // Read status command.
    REG_SPIC(0x028) = 0x00000100;
    REG_SPIC(0x02C) = 0x00000400;
    REG_SPIC(0x030) = 0x00000230;
    REG_SPIC(0x100) = command;
    REG_SPIC(0x034) = 0x00000001;
    // Wait for done.
    for (cnt = TIME_LIMIT; cnt > 0; --cnt) {
        intr_stat = REG_SPIC(0x0A0);
        if (intr_stat & 0x00000001) {
            // Detect PrgRdEnd flag.
            break;
        }
    }
    if (cnt == 0) {
        // Timeout
        return 1;
    }
    REG_SPIC(0x0A0) = 0x0000000F;   // Clear flags.
    *reg_val = REG_SPIC(0x200);     // Read buffer.
    return 0;

}

static inline int readStatus1(uint32_t *status)
{
    return readCommand(0x00000005, status);
}

static inline int readStatus2(uint32_t *status)
{
    return readCommand(0x00000035, status);
}

static int writeCommand(uint32_t reg_ioctrl, uint32_t reg_acctrl, uint32_t command)
{
    uint32_t intr_stat;
    uint32_t cnt;
    
    // Configuration of `PrgBufIOCtrl'
    REG_SPIC(0x028) = reg_ioctrl;
    // Configuration of `PrgOECtrl'
    REG_SPIC(0x02C) = 0x00000400;
    // Configuration of `PrgAccCtrl'
    REG_SPIC(0x030) = reg_acctrl;
    // Write `Write page program' command to SPIC PrimaryBuffer.
    REG_SPIC(0x100) = command;
    // Copy from SRAM to SPIC SecondaryBuffer.
    REG_SPIC(0x034) = 0x00000001;
    // Wait for done.
    for (cnt = TIME_LIMIT; cnt > 0; --cnt) {
        intr_stat = REG_SPIC(0x0A0);
        if (intr_stat & 0x00000002) {
            // Detect PrgWrEnd flag.
            break;
        }
    }
    if (cnt == 0) {
        // Timeout
        return 1;
    }
    REG_SPIC(0x0A0) = 0x0000000F;   // Clear flags.
    return 0;
}

static int prepareWrite(void)
{
    uint32_t intr_stat;
    uint32_t cnt;
    // Write Enable command.
    if (writeCommand(0x00000100, 0x00000310, 0x00000006) != 0) {
        return 1;
    }
    // Wait for change state.
    for (cnt = TIME_LIMIT; cnt > 0; --cnt) {
        if (readStatus1(&intr_stat) != 0) {
            return 1;
        }
        if (intr_stat & 0x00000002) {
            // Detect PrgWrEnd flag.
            break;
        }
    }
    return 0;
}

static int polling(void)
{
    uint32_t intr_stat;
    uint32_t cnt;

    for (cnt = TIME_LIMIT; cnt > 0; --cnt) {
        if (readStatus1(&intr_stat) != 0) {
            return 1;
        }
        if ((intr_stat & 0x00000001) == 0) {
            break;
        }
        for (int i = 0; i < 100; i++);
    }
    return 0;
}

/* FlashAlgo interface */

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    REG_GCNF(0x154) = 0;
    if (fnc == 3) {
        REG_SPIC(0x050) = 1;
    }
    return 0;
}

uint32_t UnInit(uint32_t fnc)
{
    return 0;
}

uint32_t EraseChip(void)
{
    uint32_t intr_stat;
    
    if (prepareWrite() != 0) {
        return 1;
    }
    wait(8);
    // Write chip erase command.
    if (writeCommand(0x00000100, 0x00000310, 0x00000C7) != 0) {
        return 1;
    }
    // Wait 'BUSY' bit cleard.
    for (int i = 0; i < TIME_LIMIT_MS; i++) {
        if (readStatus1(&intr_stat) != 0) {
            return 1;
        }
        if ((intr_stat & 0x00000001) == 0) {
            // BUSY bit cleard.
            break;
        }
        wait(1000); // 1ms;
    }
    
    return 0;
}

uint32_t EraseSector(uint32_t adr)
{
    if (prepareWrite() != 0) {
        return 1;
    }
    wait(8);
    // Write chip erase command.
    if (writeCommand(0x00000100, 0x00030310, (__rev(adr) | 0x20)) != 0) {
        return 1;
    }
    if (polling() != 0) {
        return 1;
    }
    return 0;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t stat;
    uint32_t offset;
    uint32_t cnt;

    // Write enable
    if (prepareWrite() != 0) {
        return 1;
    }
    
    if (readStatus2(&stat) != 0) {
        return 1;
    }
    if (stat & 0x00000002) {
        /* SPI quad access mode. */
        // Configuration of `PrgBufIOCtrl'
        REG_SPIC(0x028) = 0x00000102;
        // Configuration of `PrgOECtrl'
        REG_SPIC(0x02C) = 0x00000400;
        // Configuration of `PrgAccCtrl'
        REG_SPIC(0x030) = (0x00030330 | ((sz - 1) << 24));
        // Write `Write page program' command to SPIC PrimaryBuffer.
        REG_SPIC(0x100) = (__rev(adr) | 0x32);
    } else {
        /* SPI single access mode. */
        // Configuration of `PrgBufIOCtrl'
        REG_SPIC(0x028) = 0x00000100;
        // Configuration of `PrgOECtrl'
        REG_SPIC(0x02C) = 0x00000400;
        // Configuration of `PrgAccCtrl'
        REG_SPIC(0x030) = (0x00030330 | ((sz - 1) << 24));
        // Write `Write page program' command to SPIC PrimaryBuffer.
        REG_SPIC(0x100) = (__rev(adr) | 0x02);
    }
    // Copy from SRAM to SPIC SecondaryBuffer.
    offset = sz & 0xfffffffc;
    for (int i = 0; i < sz; i += 4) {
        REG_SPIC(0x200 + i) = buf[i >> 2];
    }
    if (offset != sz) {
        REG_SPIC(0x200 + offset) = buf[offset >> 2];
    }
    if (sz < 224) {
        wait(8 - (sz >> 5));
    }
    // Start
    REG_SPIC(0x034) = 0x00000001;
    // Wait for done.
    for (cnt = TIME_LIMIT; cnt > 0; --cnt) {
        if (REG_SPIC(0x0A0) & 0x00000002) {
            // Detect PrgWrEnd flag.
            break;
        }
    }
    if (cnt == 0) {
        // Timeout
        return 1;
    }
    // Wait for BUSY flag cleard.
    if (polling() != 0) {
        return 1;
    }

    return 0;
}
