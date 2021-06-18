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


/*
 * The PrgCode and PrgData output section names come from the CMSIS-Pack flash algo
 * templates and armlink. It is used here because several tools that work
 * with these flash algos expect this section name.
 */
SECTIONS
{
    PrgCode 0 : ALIGN_WITH_INPUT
    {
        *(.text*)
        EXCLUDE_FILE (*FlashDev.o) *(.rodata*)
        . = ALIGN(16);
    }

    PrgData : ALIGN_WITH_INPUT
    {
        *(.data*)
        *(.bss*)
        *(.got*)
        *(COMMON)
        . = ALIGN(16);
    }

    DevDscr :
    {
        *FlashDev.o(*)
    }

    /DISCARD/ : {
        /* Unused exception related info that only wastes space */
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}
