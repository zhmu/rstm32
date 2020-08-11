/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include <cstdint>

namespace systick
{
    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0xe000'e010 + offset);
    }

    constexpr inline uint32_t STK_CTRL = 0x00;
    namespace ctrl
    {
        constexpr inline uint32_t COUNTFLAG = (1 << 16);
        constexpr inline uint32_t CLKSOURCE = (1 << 2);
        constexpr inline uint32_t TICKINT = (1 << 1);
        constexpr inline uint32_t ENABLE = (1 << 0);
    } // namespace ctrl
    constexpr inline uint32_t STK_LOAD = 0x04;
    constexpr inline uint32_t STK_VAL = 0x08;

    template<typename Clock>
    void Delay_us(int us)
    {
        Register(STK_LOAD) = (Clock::cpu_frequency / 1'000'000) * us;
        Register(STK_VAL) = 0; // clears COUNTFLAG
        Register(STK_CTRL) = ctrl::CLKSOURCE | ctrl::ENABLE;
        while ((Register(STK_CTRL) & ctrl::COUNTFLAG) == 0)
            ;
        Register(STK_CTRL) = 0;
    }
} // namespace systick
