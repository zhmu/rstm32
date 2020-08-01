/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include "rstm32/gpio.h"

namespace gpio
{
    void SetupPin(int gpioBase, int pin, int mode, int cnf)
    {
        const int pinRegister = (pin >= 8) ? gpio::GPIOx_CRH : gpio::GPIOx_CRL;
        const int pinOffs = pin & 7;
        const int modeShift = pinOffs * 4;
        const int cnfShift = modeShift + 2;

        auto cr = gpio::Register(gpioBase + pinRegister);
        cr &= ~(gpio::gpio_cr::MODE_MASK << modeShift);
        cr &= ~(gpio::gpio_cr::CNF_MASK << cnfShift);
        cr |= (mode << modeShift);
        cr |= (cnf << cnfShift);
        gpio::Register(gpioBase + pinRegister) = cr;
    }
} // namespace gpio
