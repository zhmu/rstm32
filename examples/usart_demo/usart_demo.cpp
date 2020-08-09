/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include <cstdint>
#include "rstm32/rcc.h"
#include "rstm32/format.h"
#include "rstm32/gpio.h"
#include "rstm32/timer.h"
#include "rstm32/usart.h"

constexpr usart::Port usart1(usart::Port::USART1);
constexpr gpio::Pin usart1TX(gpio::Bank::A, 9);
constexpr gpio::Pin pinLed(gpio::Bank::C, 13);

using Clock = rcc::Clock8MHzOscillator72MHzPLL;

namespace
{
    void clock_init()
    {
        rcc::Initialize<Clock>();

        rcc::EnableClock(rcc::PeripheralClock::IOPC);
        rcc::EnableClock(rcc::PeripheralClock::IOPA);
        rcc::EnableClock(rcc::PeripheralClock::AFIO);
        rcc::EnableClock(rcc::PeripheralClock::USART1);
        rcc::EnableClock(rcc::PeripheralClock::TIM2);
    }

    void usart_init()
    {
        gpio::SetupPin(usart1TX, gpio::Config::Output_50MHz_Alt_Push_Pull);

        usart::Configure<Clock>(
            usart1, 115200, usart::DataBits::DB_8, usart::Parity::None, usart::StopBits::SB_1,
            usart::FlowControl::None);
        usart::SetMode(usart1, usart::Mode::Enable_TX);
    }

    void gpio_init()
    {
        gpio::ClearPin(pinLed);
        gpio::SetupPin(pinLed, gpio::Config::Output_50MHz_General_Push_Pull);
    }
} // namespace

void Default_Handler()
{
    while (1)
        ;
}

int main()
{
    clock_init();
    gpio_init();
    usart_init();

    format::Print(usart1, "hmm?\n");
    for (unsigned int counter = 0;; ++counter) {
        gpio::TogglePin(pinLed);
        //format::Print(usart1, "Hello world ", counter, " ", static_cast<uint32_t>(counter), "\r\n");

        timer::Delay<Clock>(10);
    }
}
