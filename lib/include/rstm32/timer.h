/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include <cstdint>

namespace timer
{
    constexpr inline uint32_t TIM1 = 0x4001'2c00;
    constexpr inline uint32_t TIM2 = 0x4000'0000;
    constexpr inline uint32_t TIM3 = 0x4000'0400;
    constexpr inline uint32_t TIM4 = 0x4000'0800;

    inline volatile uint32_t& Register(uint32_t base, uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(base + offset);
    }

    namespace tim
    {
        constexpr inline uint32_t TIMx_CR1 = 0x00;
        namespace cr1
        {
            constexpr inline uint32_t CKD_MASK = (1 << 9) | (1 << 8);
            constexpr inline uint32_t CKD_DIV_1 = (0 << 9) | (0 << 8);
            constexpr inline uint32_t CKD_DIV_2 = (0 << 9) | (1 << 8);
            constexpr inline uint32_t CKD_DIV_4 = (1 << 9) | (0 << 8);
            constexpr inline uint32_t ARPE = (1 << 7);
            constexpr inline uint32_t CMS_MASK = (1 << 6) | (1 << 5);
            constexpr inline uint32_t CMS_EDGE = (0 << 6) | (0 << 5);
            constexpr inline uint32_t CMS_CENTER_1 = (0 << 6) | (1 << 5);
            constexpr inline uint32_t CMS_CENTER_2 = (1 << 6) | (0 << 5);
            constexpr inline uint32_t CMS_CENTER_3 = (1 << 6) | (1 << 5);
            constexpr inline uint32_t DIR = (1 << 4);
            constexpr inline uint32_t OPM = (1 << 3);
            constexpr inline uint32_t URS = (1 << 2);
            constexpr inline uint32_t UDIS = (1 << 1);
            constexpr inline uint32_t CEN = (1 << 0);
        } // namespace cr1

        constexpr inline uint32_t TIMx_CR2 = 0x04;
        namespace cr2
        {
            constexpr inline uint32_t TI1S = (1 << 7);
            constexpr inline uint32_t MMS_MASK = (1 << 6) | (1 << 5) | (1 << 4);
            constexpr inline uint32_t MMS_RESET = (0 << 6) | (0 << 5) | (0 << 4);
            constexpr inline uint32_t MMS_ENABLE = (0 << 6) | (0 << 5) | (1 << 4);
            constexpr inline uint32_t MMS_UPDATE = (0 << 6) | (1 << 5) | (0 << 4);
            constexpr inline uint32_t CCDS = (1 << 3);
        } // namespace cr2
        constexpr inline uint32_t TIMx_SMCR = 0x08;
        namespace smcr
        {
            constexpr inline uint32_t ETP = (1 << 15);
            constexpr inline uint32_t ECE = (1 << 14);
            constexpr inline uint32_t ETPS_MASK = (1 << 13) | (1 << 12);
            constexpr inline uint32_t ETF_MASK = (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8);
            constexpr inline uint32_t MSM = (1 << 7);
            constexpr inline uint32_t TS_MASK = (1 << 6) | (1 << 5) | (1 << 4);
            constexpr inline uint32_t SMS_MASK = (1 << 2) | (1 << 1) | (1 << 0);
        } // namespace smcr
        constexpr inline uint32_t TIMx_DIER = 0x0c;
        namespace dier
        {
            constexpr inline uint32_t TDE = (1 << 14);
            constexpr inline uint32_t CC4DE = (1 << 12);
            constexpr inline uint32_t CC3DE = (1 << 11);
            constexpr inline uint32_t CC2DE = (1 << 10);
            constexpr inline uint32_t CC1DE = (1 << 9);
            constexpr inline uint32_t UDE = (1 << 8);
            constexpr inline uint32_t TIE = (1 << 6);
            constexpr inline uint32_t CC4E = (1 << 4);
            constexpr inline uint32_t CC3E = (1 << 3);
            constexpr inline uint32_t CC2E = (1 << 2);
            constexpr inline uint32_t CC1E = (1 << 1);
            constexpr inline uint32_t UIE = (1 << 0);
        } // namespace dier
        constexpr inline uint32_t TIMx_SR = 0x10;
        namespace sr
        {
            constexpr inline uint32_t CC4OF = (1 << 12);
            constexpr inline uint32_t CC3OF = (1 << 11);
            constexpr inline uint32_t CC2OF = (1 << 10);
            constexpr inline uint32_t CC1OF = (1 << 9);
            constexpr inline uint32_t TIF = (1 << 6);
            constexpr inline uint32_t CC4F = (1 << 4);
            constexpr inline uint32_t CC3F = (1 << 3);
            constexpr inline uint32_t CC2F = (1 << 2);
            constexpr inline uint32_t CC1F = (1 << 1);
            constexpr inline uint32_t UIF = (1 << 0);
        } // namespace sr
        constexpr inline uint32_t TIMx_EGR = 0x14;
        namespace egr
        {
            constexpr inline uint32_t TG = (1 << 6);
            constexpr inline uint32_t CC4G = (1 << 4);
            constexpr inline uint32_t CC3G = (1 << 3);
            constexpr inline uint32_t CC2G = (1 << 2);
            constexpr inline uint32_t CC1G = (1 << 1);
            constexpr inline uint32_t UG = (1 << 0);
        } // namespace egr
        constexpr inline uint32_t TIMx_CNT = 0x24;
        constexpr inline uint32_t TIMx_PSC = 0x28;
        constexpr inline uint32_t TIMx_ARR = 0x2c;
    } // namespace tim

    template<typename Clock>
    void Delay(int ms)
    {
        constexpr auto pscReg = (Clock::apb1_frequency / 1000);
        // static_assert(pscReg <= 65535);

        Register(TIM2, tim::TIMx_SR) = 0;
        Register(TIM2, tim::TIMx_PSC) = pscReg;
        // If the APB1 prescaler is not 1, the clock is multiplied by 2 (refer to
        // Figure 11: Clock Tree of RM 0008 for details) so we need to wait twice as long
        if constexpr (Clock::apb1_frequency != Clock::cpu_frequency) {
            ms *= 2;
        }
        Register(TIM2, tim::TIMx_ARR) = ms;
        Register(TIM2, tim::TIMx_CR1) = tim::cr1::CEN;
        while ((Register(TIM2, tim::TIMx_SR) & tim::sr::UIF) == 0)
            ;
    }

} // namespace timer
