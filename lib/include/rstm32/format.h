/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include "rstm32/usart.h"

namespace format
{
    void Print(const char* s)
    {
        for (; *s != '\0'; ++s)
            usart::Write(*s);
    }

    constexpr inline char hextab[] = "0123456789abcdef";

    template<typename T>
    void PrintNumber(const int base, const T value)
    {
        unsigned int divisor = 1, p = 0;
        for (T n = value; n >= static_cast<T>(base); n /= base, divisor *= base, p++)
            ;
        for (unsigned int i = 0; i <= p; i++, divisor /= base)
            usart::Write(hextab[(value / divisor) % base]);
    }

    void Print(unsigned int v) { PrintNumber(10, v); }

    void Print(uint32_t v)
    {
        Print("0x");
        PrintNumber(16, v);
    }

    template<typename... Args>
    void Print(Args... args)
    {
        (Print(args), ...);
    }

} // namespace format
