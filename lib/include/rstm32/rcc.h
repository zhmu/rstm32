/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include <cstdint>
#include "flash.h"

namespace rcc
{
    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4002'1000 + offset);
    }

    constexpr inline uint32_t RCC_CR = 0x00;
    namespace cr
    {
        constexpr inline uint32_t HSION = (1 << 0);
        constexpr inline uint32_t HSIRDY = (1 << 1);
        constexpr inline uint32_t HSEON = (1 << 16);
        constexpr inline uint32_t HSERDY = (1 << 17);
        constexpr inline uint32_t HSEBYP = (1 << 18);
        constexpr inline uint32_t CSSON = (1 << 19);
        constexpr inline uint32_t PLLON = (1 << 24);
        constexpr inline uint32_t PLLRDY = (1 << 25);
    } // namespace cr
    constexpr inline uint32_t RCC_CFGR = 0x04;
    namespace cfgr
    {
        constexpr inline uint32_t SW_MASK = (1 << 1) | (1 << 0);
        constexpr inline uint32_t SW_HSI = (0 << 1) | (0 << 0);
        constexpr inline uint32_t SW_HSE = (0 << 1) | (0 << 1);
        constexpr inline uint32_t SW_PLL = (1 << 1) | (0 << 1);

        constexpr inline uint32_t HPRE_MASK = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4);
        constexpr inline uint32_t HPRE_NO_DIV = (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4);
        constexpr inline uint32_t HPRE_DIV_2 = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4);

        constexpr inline uint32_t PPRE1_MASK = (1 << 10) | (1 << 9) | (1 << 8);
        constexpr inline uint32_t PPRE1_NO_DIV = (0 << 10) | (0 << 9) | (0 << 8);
        constexpr inline uint32_t PPRE1_DIV_2 = (1 << 10) | (0 << 9) | (0 << 8);

        constexpr inline uint32_t PPRE2_MASK = (1 << 13) | (1 << 12) | (1 << 11);
        constexpr inline uint32_t PPRE2_NO_DIV = (0 << 13) | (0 << 12) | (0 << 11);

        constexpr inline uint32_t ADCPRE_MASK = (1 << 15) | (1 << 14);
        constexpr inline uint32_t ADCPRE_DIV_8 = (1 << 15) | (1 << 14);

        constexpr inline uint32_t PLLSRC_MASK = (1 << 16);
        constexpr inline uint32_t PLLSRC_HSI = (0 << 16);
        constexpr inline uint32_t PLLSRC_HSE = (1 << 16);

        constexpr inline uint32_t PLLXTPRE_MASK = (1 << 17);
        constexpr inline uint32_t PLLXTPRE_NO_DIV = (0 << 17);
        constexpr inline uint32_t PLLXTPRE_DIV_2 = (1 << 17);

        constexpr inline uint32_t PLLMUL_MASK = (1 << 21) | (1 << 20) | (1 << 19) | (1 << 18);
        constexpr inline uint32_t PLLMUL_MUL_9 = (0 << 21) | (1 << 20) | (1 << 19) | (1 << 18);
    } // namespace cfgr
    constexpr inline uint32_t RCC_CIR = 0x08;
    constexpr inline uint32_t RCC_APB2RSTR = 0x0c;
    constexpr inline uint32_t RCC_APB1RSTR = 0x10;
    constexpr inline uint32_t RCC_AHBENR = 0x14;
    constexpr inline uint32_t RCC_APB2ENR = 0x18;
    namespace apb2enr
    {
        constexpr inline uint32_t AFIOEN = (1 << 0);
        constexpr inline uint32_t IOPAEN = (1 << 2);
        constexpr inline uint32_t IOPCEN = (1 << 4);
        constexpr inline uint32_t USART1EN = (1 << 14);
    } // namespace apb2enr
    constexpr inline uint32_t RCC_APB1ENR = 0x1c;
    constexpr inline uint32_t RCC_BDCR = 0x20;
    constexpr inline uint32_t RCC_CSR = 0x24;

    // Given a 8MHz external crystal oscillator, yields SYSCLK at 72MHz
    void Setup_8MHz_External_Crystal_Yielding_72MHz_PLL();

    uint32_t GetAPB1Frequency();
    uint32_t GetAPB2Frequency();
} // namespace rcc
