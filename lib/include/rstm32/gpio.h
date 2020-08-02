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
    constexpr inline uint32_t GPIOA = 0x0800;
    constexpr inline uint32_t GPIOB = 0x0c00;
    constexpr inline uint32_t GPIOC = 0x1000;
    constexpr inline uint32_t GPIOD = 0x1400;
    constexpr inline uint32_t GPIOE = 0x1800;
    constexpr inline uint32_t GPIOF = 0x1c00;
    constexpr inline uint32_t GPIOG = 0x2000;

    inline volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0000 + offset);
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

        constexpr inline uint32_t CNF_MASK = (1 << 3) | (1 << 2);
        constexpr inline uint32_t CNF_IN_ANALOG = (0 << 3) | (0 << 2);
        constexpr inline uint32_t CNF_IN_FLOATING = (0 << 3) | (0 << 2);
        constexpr inline uint32_t CNF_IN_PULL_UPDOWN = (1 << 3) | (0 << 2);

        constexpr inline uint32_t CNF_OUT_GENERAL_PUSH_PULL = (0 << 3) | (0 << 2);
        constexpr inline uint32_t CNF_OUT_GENERAL_OPEN_DRAIN = (0 << 3) | (1 << 2);
        constexpr inline uint32_t CNF_OUT_ALT_PUSH_PULL = (1 << 3) | (0 << 2);
        constexpr inline uint32_t CNF_OUT_ALT_OPEN_DRAIN = (1 << 3) | (1 << 2);
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

    enum class Config {
        Input_Analog = gpio_cr::MODE_INPUT | gpio_cr::CNF_IN_ANALOG,
        Input_Floating = gpio_cr::MODE_INPUT | gpio_cr::CNF_IN_FLOATING,
        Input_Pull_UpDown = gpio_cr::MODE_INPUT | gpio_cr::CNF_IN_PULL_UPDOWN,
        Output_2MHz_General_Push_Pull =
            gpio_cr::MODE_OUTPUT_2MHZ | gpio_cr::CNF_OUT_GENERAL_PUSH_PULL,
        Output_2MHz_General_Open_Drain =
            gpio_cr::MODE_OUTPUT_2MHZ | gpio_cr::CNF_OUT_GENERAL_OPEN_DRAIN,
        Output_2MHz_Alt_Push_Pull = gpio_cr::MODE_OUTPUT_2MHZ | gpio_cr::CNF_OUT_ALT_PUSH_PULL,
        Output_2MHz_Alt_Open_Drain = gpio_cr::MODE_OUTPUT_2MHZ | gpio_cr::CNF_OUT_ALT_OPEN_DRAIN,
        Output_10MHz_General_Push_Pull =
            gpio_cr::MODE_OUTPUT_10MHZ | gpio_cr::CNF_OUT_GENERAL_PUSH_PULL,
        Output_10MHz_General_Open_Drain =
            gpio_cr::MODE_OUTPUT_10MHZ | gpio_cr::CNF_OUT_GENERAL_OPEN_DRAIN,
        Output_10MHz_Alt_Push_Pull = gpio_cr::MODE_OUTPUT_10MHZ | gpio_cr::CNF_OUT_ALT_PUSH_PULL,
        Output_10MHz_Alt_Open_Drain = gpio_cr::MODE_OUTPUT_10MHZ | gpio_cr::CNF_OUT_ALT_OPEN_DRAIN,
        Output_50MHz_General_Push_Pull =
            gpio_cr::MODE_OUTPUT_50MHZ | gpio_cr::CNF_OUT_GENERAL_PUSH_PULL,
        Output_50MHz_General_Open_Drain =
            gpio_cr::MODE_OUTPUT_50MHZ | gpio_cr::CNF_OUT_GENERAL_OPEN_DRAIN,
        Output_50MHz_Alt_Push_Pull = gpio_cr::MODE_OUTPUT_50MHZ | gpio_cr::CNF_OUT_ALT_PUSH_PULL,
        Output_50MHz_Alt_Open_Drain = gpio_cr::MODE_OUTPUT_50MHZ | gpio_cr::CNF_OUT_ALT_OPEN_DRAIN,
    };

    void SetupPin(int gpioBase, int pin, const Config config);
} // namespace gpio
