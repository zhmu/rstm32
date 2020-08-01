/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include "rstm32/rcc.h"

namespace rcc
{
    namespace
    {
        uint32_t apb1_frequency = 8'000'000;
        uint32_t apb2_frequency = 8'000'000;
    } // namespace

    void Setup_8MHz_External_Crystal_Yielding_72MHz_PLL()
    {
        // Switch to internal high-speed (8MHz) clock
        Register(RCC_CR) |= cr::HSION;
        while ((Register(RCC_CR) & cr::HSIRDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_HSI;

        // Switch to external 8MHz crystal
        Register(RCC_CR) |= cr::HSEON;
        while ((Register(RCC_CR) & cr::HSERDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_HSE;

        // Prescalers
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::HPRE_MASK) | cfgr::HPRE_NO_DIV;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::ADCPRE_MASK) | cfgr::ADCPRE_DIV_8;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PPRE1_MASK) | cfgr::PPRE1_DIV_2;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PPRE2_MASK) | cfgr::PPRE2_NO_DIV;
        // Up flash WS to 2, as we are between 48..72MHz
        flash::Register(flash::FLASH_ACR) =
            (flash::Register(flash::FLASH_ACR) & ~flash::acr::LATENCY_MASK) |
            flash::acr::LATENCY_2_WS;

        // Setup PLL as HSE (8Mhz) * 9 = 72MHz
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLMUL_MASK) | cfgr::PLLMUL_MUL_9;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLSRC_MASK) | cfgr::PLLSRC_HSE;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLXTPRE_MASK) | cfgr::PLLXTPRE_NO_DIV;

        // Switch to PLL
        Register(RCC_CR) |= cr::PLLON;
        while ((Register(RCC_CR) & cr::PLLRDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_PLL;

        apb1_frequency = 36'000'000;
        apb2_frequency = 72'000'000;
    }

    uint32_t GetAPB1Frequency() { return apb1_frequency; }

    uint32_t GetAPB2Frequency() { return apb2_frequency; }
} // namespace rcc
