/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include <cstdint>

namespace usart
{
    enum class Port : uint32_t {
        USART1 = 0x4001'3800,
        USART2 = 0x4000'4400,
        USART3 = 0x4000'4800,
        USART4 = 0x4000'4C00,
        USART5 = 0x4000'5000,
    };

    inline volatile uint32_t& Register(const Port& port, uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(static_cast<uint32_t>(port) + offset);
    }
    constexpr inline uint32_t USART_SR = 0x00;
    namespace sr
    {
        constexpr inline uint32_t TXE = (1 << 7);
        constexpr inline uint32_t TC = (1 << 6);
        constexpr inline uint32_t RXNE = (1 << 5);
    } // namespace sr
    constexpr inline uint32_t USART_DR = 0x04;
    constexpr inline uint32_t USART_BRR = 0x08;
    constexpr inline uint32_t USART_CR1 = 0x0c;

    namespace cr1
    {
        constexpr inline uint32_t UE = (1 << 13);
        constexpr inline uint32_t M = (1 << 12);
        constexpr inline uint32_t PCE = (1 << 10);
        constexpr inline uint32_t PS = (1 << 9);
        constexpr inline uint32_t TE = (1 << 3);
        constexpr inline uint32_t RE = (1 << 2);
    } // namespace cr1

    constexpr inline uint32_t USART_CR2 = 0x10;
    namespace cr2
    {
        constexpr inline uint32_t STOP_MASK = (1 << 13) | (1 << 12);
        constexpr inline uint32_t STOP_1 = (0 << 13) | (0 << 12);
        constexpr inline uint32_t STOP_0_5 = (0 << 13) | (1 << 12);
        constexpr inline uint32_t STOP_2 = (1 << 13) | (0 << 12);
        constexpr inline uint32_t STOP_1_5 = (1 << 13) | (1 << 12);
    } // namespace cr2

    constexpr inline uint32_t USART_CR3 = 0x14;
    namespace cr3
    {
        constexpr inline uint32_t CTSE = (1 << 9);
        constexpr inline uint32_t RTSE = (1 << 8);
    } // namespace cr3

    constexpr inline uint32_t USART_GTPR = 0x18;

    inline void Write(const Port& port, int ch)
    {
        while ((usart::Register(port, usart::USART_SR) & usart::sr::TXE) == 0)
            ;
        usart::Register(port, usart::USART_DR) = ch;
    }

    enum class DataBits { DB_8 = 0, DB_9 = cr1::M };
    enum class Parity { None = 0, Even = cr1::PCE, Odd = cr1::PCE | cr1::PS };
    enum class StopBits {
        SB_1 = cr2::STOP_1,
        SB_0_5 = cr2::STOP_0_5,
        SB_2 = cr2::STOP_2,
        SB_1_5 = cr2::STOP_1_5
    };
    enum class FlowControl {
        None = 0,
        RTS = cr3::RTSE,
        CTS = cr3::CTSE,
        RTS_CTS = cr3::RTSE | cr3::CTSE
    };

    enum class Mode {
        Disable = 0,
        Enable_TX = cr1::TE | cr1::UE,
        Enable_RX = cr1::RE | cr1::UE,
        Enable_RX_TX = cr1::RE | cr1::TE | cr1::UE
    };

    namespace detail
    {
        template<typename Clock>
        void SetBaudRate(const Port& port, const int baud)
        {
            const int fPCLK2 = port == Port::USART1 ? Clock::apb2_frequency : Clock::apb1_frequency;
            const int value_mantissa = (fPCLK2 / 16) / baud;
            // Fraction is n/16-ths of the result; should we round up here?
            const int value_modulo = (fPCLK2 / 16) % baud;
            const int value_fraction = value_modulo / (baud / 16);
            Register(port, USART_BRR) = (value_mantissa << 4) | (value_fraction & 15);
        }
    } // namespace detail

    template<typename Clock>
    void Configure(
        const Port& port, const int baudRate, const DataBits db, const Parity p, const StopBits sb,
        const FlowControl fc)
    {
        detail::SetBaudRate<Clock>(port, baudRate);

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
