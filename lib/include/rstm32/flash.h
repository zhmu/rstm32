/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include <cstdint>

namespace flash
{
    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x40022000 + offset);
    }

    constexpr inline uint32_t FLASH_ACR = 0x00;
    namespace acr
    {
        constexpr inline uint32_t LATENCY_MASK = (1 << 2) | (1 << 1) | (1 << 0);
        constexpr inline uint32_t LATENCY_0_WS = (0 << 2) | (0 << 1) | (0 << 0);
        constexpr inline uint32_t LATENCY_1_WS = (0 << 2) | (0 << 1) | (1 << 0);
        constexpr inline uint32_t LATENCY_2_WS = (0 << 2) | (1 << 1) | (0 << 0);
    } // namespace acr
} // namespace flash
