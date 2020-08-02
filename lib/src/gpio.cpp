/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include "rstm32/gpio.h"

namespace gpio
{
    void SetupPin(int gpioBase, int pin, const Config config)
    {
        const int pinRegister = (pin >= 8) ? gpio::GPIOx_CRH : gpio::GPIOx_CRL;
        const int pinShift = (pin & 7) * 4;

        auto cr = gpio::Register(gpioBase + pinRegister);
        cr &= ~((gpio::gpio_cr::MODE_MASK | gpio::gpio_cr::CNF_MASK) << pinShift);
        cr |= static_cast<uint32_t>(config) << pinShift;
        gpio::Register(gpioBase + pinRegister) = cr;
    }
} // namespace gpio
