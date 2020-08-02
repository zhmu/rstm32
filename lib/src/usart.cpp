/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include "rstm32/usart.h"
#include "rstm32/rcc.h"

namespace usart
{
    namespace
    {
        void SetBaudRate(const Port& port, const int baud)
        {
            const int fPCLK2 = port == Port::USART1 ? rcc::GetAPB2Frequency() : rcc::GetAPB1Frequency();
            const int value_mantissa = (fPCLK2 / 16) / baud;
            // Fraction is n/16-ths of the result; should we round up here?
            const int value_modulo = (fPCLK2 / 16) % baud;
            const int value_fraction = value_modulo / (baud / 16);
            Register(port, USART_BRR) = (value_mantissa << 4) | (value_fraction & 15);
        }
    } // namespace

    void Configure(
        const Port& port, const int baudRate, const DataBits db, const Parity p, const StopBits sb,
        const FlowControl fc)
    {
        SetBaudRate(port, baudRate);

        uint32_t cr1 = Register(port, USART_CR1);
        cr1 &= ~(cr1::M | cr1::PCE | cr1::PS);
        cr1 |= static_cast<uint32_t>(db);
        cr1 |= static_cast<uint32_t>(p);

        uint32_t cr2 = Register(port, USART_CR2);
        cr2 &= ~(cr2::STOP_MASK);
        cr2 |= static_cast<uint32_t>(sb);

        uint32_t cr3 = Register(port, USART_CR3);
        cr3 &= ~(cr3::CTSE | cr3::RTSE);
        cr3 |= static_cast<uint32_t>(fc);

        Register(port, USART_CR1) = cr1;
        Register(port, USART_CR2) = cr2;
        Register(port, USART_CR3) = cr3;
    }

    void SetMode(const Port& port, const Mode mode)
    {
        auto cr = Register(port, USART_CR1);
        cr &= ~(cr1::RE | cr1::TE | cr1::UE);
        cr |= static_cast<uint32_t>(mode);
        Register(port, USART_CR1) = cr;
    }
} // namespace usart
