/*
 * Copyright (c) 2018-2019 Arm Limited
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

/**
 * \file sfn40ulp_eflash_drv.c
 *
 * \brief Process specifinc implementation of GFC100 flash controller for
 *        sfn40ulp128kx128m64p16i16_c_dw25_svt_110a flash macro.
 */

#include "gfc100_process_spec_api.h"

/**
 * Register map for gfc100 process specific part for the
 * sfn40ulp128kx128m64p16i16_c_dw25_svt_110a flash macro.
 */
struct sfn40ulp_reg_map_t {
    volatile uint32_t config0;            /* 0x000 RW Config register 0 */
    volatile uint32_t config1;            /* 0x004 RW Config register 1 */
    volatile uint32_t config2;            /* 0x008 RW Config register 2 */
    volatile uint32_t reserved0[3];       /* 0x00C Reserved */
    volatile uint32_t ecause;             /* 0x018 RW Error cause */
    volatile uint32_t reserved1[1005];    /* 0x01C Reserved */
    volatile uint32_t pidr4;              /* 0xFD0 RO Peripheral id register0 */
    volatile uint32_t reserved2[3];       /* 0xFD4 Reserved */
    volatile uint32_t pidr0;              /* 0xFE0 RO Peripheral id register4 */
    volatile uint32_t pidr1;              /* 0xFE4 RO Peripheral id register1 */
    volatile uint32_t pidr2;              /* 0xFE8 RO Peripheral id register2 */
    volatile uint32_t pidr3;              /* 0xFEC RO Peripheral id register3 */
    volatile uint32_t cidr0;              /* 0xFF0 RO Component id register0 */
    volatile uint32_t cidr1;              /* 0xFF4 RO Component id register1 */
    volatile uint32_t cidr2;              /* 0xFF8 RO Component id register2 */
    volatile uint32_t cidr3;              /* 0xFFC RO Component id register3 */
};

void gfc100_proc_spec_set_eflash_timing(uint32_t reg_map_base,
                                        uint32_t sys_clk)
{
    struct sfn40ulp_reg_map_t *reg_map =
                                    (struct sfn40ulp_reg_map_t *)reg_map_base;

    reg_map->config0 = 0x11082801;
    reg_map->config1 = 0x64050208;
    reg_map->config2 = 0xa0a0a08;
}

uint32_t gfc100_proc_spec_get_eflash_word_width(uint32_t reg_map_base)
{
    (void)reg_map_base;

    return 128U;
}

uint32_t gfc100_proc_spec_get_eflash_size(uint32_t reg_map_base)
{
    (void)reg_map_base;

    return 0x200000U;
}

uint32_t gfc100_proc_spec_get_eflash_page_size(uint32_t reg_map_base)
{
    (void)reg_map_base;

    return 0x4000;
}

uint32_t gfc100_proc_spec_get_num_of_info_pages(uint32_t reg_map_base)
{
    (void)reg_map_base;

    return 3U;
}

uint32_t gfc100_proc_spec_get_error_cause(uint32_t reg_map_base)
{
    struct sfn40ulp_reg_map_t *reg_map =
                                    (struct sfn40ulp_reg_map_t *)reg_map_base;

    return reg_map->ecause;
}

