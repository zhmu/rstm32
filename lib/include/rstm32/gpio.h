/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#pragma once

#include <cstdint>

namespace gpio
{
    constexpr inline uint32_t GPIOA_ = 0x0800;
    constexpr inline uint32_t GPIOB_ = 0x0c00;
    constexpr inline uint32_t GPIOC_ = 0x1000;
    constexpr inline uint32_t GPIOD_ = 0x1400;
    constexpr inline uint32_t GPIOE_ = 0x1800;
    constexpr inline uint32_t GPIOF_ = 0x1c00;
    constexpr inline uint32_t GPIOG_ = 0x2000;

    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0000 + offset);
    }
    inline volatile uint32_t& RegisterG(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'2000 + offset);
    }
    inline volatile uint32_t& RegisterF(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1c00 + offset);
    }
    inline volatile uint32_t& RegisterE(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1800 + offset);
    }
    inline volatile uint32_t& RegisterD(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1400 + offset);
    }
    inline volatile uint32_t& RegisterC(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1000 + offset);
    }
    inline volatile uint32_t& RegisterB(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0c00 + offset);
    }
    inline volatile uint32_t& RegisterA(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0800 + offset);
    }

    constexpr inline uint32_t GPIOx_CRL = 0x00;
    constexpr inline uint32_t GPIOx_CRH = 0x04;
    namespace gpio_cr
    {
        constexpr inline uint32_t MODE_MASK = (1 << 1) | (1 << 0);
        constexpr inline uint32_t MODE_INPUT = (0 << 1) | (0 << 0);
        constexpr inline uint32_t MODE_OUTPUT_10MHZ = (0 << 1) | (1 << 0);
        constexpr inline uint32_t MODE_OUTPUT_2MHZ = (1 << 1) | (0 << 0);
        constexpr inline uint32_t MODE_OUTPUT_50MHZ = (1 << 1) | (1 << 0);

        constexpr inline uint32_t CNF_MASK = (1 << 1) | (1 << 0);
        constexpr inline uint32_t CNF_IN_ANALOG = (0 << 1) | (0 << 0);
        constexpr inline uint32_t CNF_IN_FLOATING = (0 << 1) | (0 << 0);
        constexpr inline uint32_t CNF_IN_PULL_UPDOWN = (1 << 1) | (0 << 0);

        constexpr inline uint32_t CNF_OUT_GENERAL_PUSH_PULL = (0 << 1) | (0 << 0);
        constexpr inline uint32_t CNF_OUT_GENERAL_OPEN_DRAIN = (0 << 1) | (1 << 0);
        constexpr inline uint32_t CNF_OUT_ALT_PUSH_PULL = (1 << 1) | (0 << 0);
        constexpr inline uint32_t CNF_OUT_ALT_OPEN_DRAIN = (1 << 1) | (1 << 0);
    } // namespace gpio_cr

    constexpr inline uint32_t GPIOx_IDR = 0x08;
    constexpr inline uint32_t GPIOx_ODR = 0x0c;
    constexpr inline uint32_t GPIOx_BSRR = 0x10;
    constexpr inline uint32_t GPIOx_BRR = 0x14;

    inline void TogglePin(int gpioBase, int pin)
    {
        gpio::Register(gpioBase + GPIOx_ODR) ^= (1 << pin);
    }

    inline void ClearPin(int gpioBase, int pin)
    {
        gpio::Register(gpioBase + GPIOx_ODR) &= ~(1 << pin);
    }

    inline void SetPin(int gpioBase, int pin)
    {
        gpio::Register(gpioBase + GPIOx_ODR) |= (1 << pin);
    }

    void SetupPin(int gpioBase, int pin, int mode, int cnf);
} // namespace gpio
