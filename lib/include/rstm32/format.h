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
    void Print(const usart::Port& port, const char* s)
    {
        for (; *s != '\0'; ++s)
            usart::Write(port, *s);
    }

    constexpr inline char hextab[] = "0123456789abcdef";

    template<typename T>
    void PrintNumber(const usart::Port& port, const int base, const T value)
    {
        unsigned int divisor = 1, p = 0;
        for (T n = value; n >= static_cast<T>(base); n /= base, divisor *= base, p++)
            ;
        for (unsigned int i = 0; i <= p; i++, divisor /= base)
            usart::Write(port, hextab[(value / divisor) % base]);
    }

    void Print(const usart::Port& port, unsigned int v) { PrintNumber(port, 10, v); }

    void Print(const usart::Port& port, uint32_t v)
    {
        Print(port, "0x");
        PrintNumber(port, 16, v);
    }

    template<typename... Args>
    void Print(const usart::Port& port, Args... args)
    {
        (Print(port, args), ...);
    }

} // namespace format
