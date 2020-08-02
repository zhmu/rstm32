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

    namespace detail
    {
        constexpr uint32_t CombineRegisterAndBit(uint32_t reg, int bit) { return (reg << 5) | bit; }

        constexpr auto SplitRegisterAndBit(uint32_t v)
        {
            struct S {
                uint32_t bit;
                int reg;
            };
            return S{v >> 5, static_cast<int>(v & 31)};
        }
    } // namespace detail

    enum class PeripheralClock {
        DMA1 = detail::CombineRegisterAndBit(RCC_AHBENR, 0),
        DMA2 = detail::CombineRegisterAndBit(RCC_AHBENR, 1),
        SRAM = detail::CombineRegisterAndBit(RCC_AHBENR, 2),
        FLITF = detail::CombineRegisterAndBit(RCC_AHBENR, 4),
        CRC = detail::CombineRegisterAndBit(RCC_AHBENR, 6),
        FSMC = detail::CombineRegisterAndBit(RCC_AHBENR, 8),
        SDIO = detail::CombineRegisterAndBit(RCC_AHBENR, 10),

        AFIO = detail::CombineRegisterAndBit(RCC_APB2ENR, 0),
        IOPA = detail::CombineRegisterAndBit(RCC_APB2ENR, 2),
        IOPB = detail::CombineRegisterAndBit(RCC_APB2ENR, 3),
        IOPC = detail::CombineRegisterAndBit(RCC_APB2ENR, 4),
        IOPD = detail::CombineRegisterAndBit(RCC_APB2ENR, 5),
        IOPE = detail::CombineRegisterAndBit(RCC_APB2ENR, 6),
        IOPF = detail::CombineRegisterAndBit(RCC_APB2ENR, 7),
        IOPG = detail::CombineRegisterAndBit(RCC_APB2ENR, 8),
        ADC1 = detail::CombineRegisterAndBit(RCC_APB2ENR, 9),
        ADC2 = detail::CombineRegisterAndBit(RCC_APB2ENR, 10),
        TIM1 = detail::CombineRegisterAndBit(RCC_APB2ENR, 11),
        SPI1 = detail::CombineRegisterAndBit(RCC_APB2ENR, 12),
        TIM8 = detail::CombineRegisterAndBit(RCC_APB2ENR, 13),
        USART1 = detail::CombineRegisterAndBit(RCC_APB2ENR, 14),
        ADC3 = detail::CombineRegisterAndBit(RCC_APB2ENR, 15),
        TIM9 = detail::CombineRegisterAndBit(RCC_APB2ENR, 19),
        TIM10 = detail::CombineRegisterAndBit(RCC_APB2ENR, 20),
        TIM11 = detail::CombineRegisterAndBit(RCC_APB2ENR, 21),

        TIM2 = detail::CombineRegisterAndBit(RCC_APB1ENR, 0),
        TIM3 = detail::CombineRegisterAndBit(RCC_APB1ENR, 1),
        TIM4 = detail::CombineRegisterAndBit(RCC_APB1ENR, 2),
        TIM5 = detail::CombineRegisterAndBit(RCC_APB1ENR, 3),
        TIM6 = detail::CombineRegisterAndBit(RCC_APB1ENR, 4),
        TIM7 = detail::CombineRegisterAndBit(RCC_APB1ENR, 5),
        TIM12 = detail::CombineRegisterAndBit(RCC_APB1ENR, 6),
        TIM13 = detail::CombineRegisterAndBit(RCC_APB1ENR, 7),
        TIM14 = detail::CombineRegisterAndBit(RCC_APB1ENR, 8),
        WWD = detail::CombineRegisterAndBit(RCC_APB1ENR, 11),
        SPI2 = detail::CombineRegisterAndBit(RCC_APB1ENR, 14),
        SPI3 = detail::CombineRegisterAndBit(RCC_APB1ENR, 15),
        USART2 = detail::CombineRegisterAndBit(RCC_APB1ENR, 17),
        USART3 = detail::CombineRegisterAndBit(RCC_APB1ENR, 18),
        USART4 = detail::CombineRegisterAndBit(RCC_APB1ENR, 19),
        USART5 = detail::CombineRegisterAndBit(RCC_APB1ENR, 20),
        I2C1 = detail::CombineRegisterAndBit(RCC_APB1ENR, 21),
        I2C2 = detail::CombineRegisterAndBit(RCC_APB1ENR, 22),
        USB = detail::CombineRegisterAndBit(RCC_APB1ENR, 23),
        CAN = detail::CombineRegisterAndBit(RCC_APB1ENR, 25),
        BKP = detail::CombineRegisterAndBit(RCC_APB1ENR, 27),
        PWR = detail::CombineRegisterAndBit(RCC_APB1ENR, 28),
        DAC = detail::CombineRegisterAndBit(RCC_APB1ENR, 29),
    };

    // Given a 8MHz external crystal oscillator, yields SYSCLK at 72MHz
    void Setup_8MHz_External_Crystal_Yielding_72MHz_PLL();

    inline void EnableClock(const PeripheralClock clock)
    {
        const auto [clockRegister, clockBit] =
            detail::SplitRegisterAndBit(static_cast<uint32_t>(clock));
        Register(clockRegister) |= (1 << clockBit);
    }

    inline void DisableClock(const PeripheralClock clock)
    {
        const auto [clockRegister, clockBit] =
            detail::SplitRegisterAndBit(static_cast<uint32_t>(clock));
        Register(clockRegister) &= ~(1 << clockBit);
    }

    uint32_t GetAPB1Frequency();
    uint32_t GetAPB2Frequency();
} // namespace rcc
