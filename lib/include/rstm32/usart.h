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
    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x40013800 + offset);
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

    inline void Write(int ch)
    {
        while ((usart::Register(usart::USART_SR) & usart::sr::TXE) == 0)
            ;
        usart::Register(usart::USART_DR) = ch;
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

    void Configure(
        int baudRate, const DataBits db, const Parity p, const StopBits sb, const FlowControl fc);

    enum class Mode {
        Disable = 0,
        Enable_TX = cr1::TE | cr1::UE,
        Enable_RX = cr1::RE | cr1::UE,
        Enable_RX_TX = cr1::RE | cr1::TE | cr1::UE
    };
    void SetMode(const Mode mode);
} // namespace usart
