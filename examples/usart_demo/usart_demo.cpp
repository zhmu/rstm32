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
#include "rstm32/systick.h"
#include "rstm32/timer.h"
#include "rstm32/usart.h"

constexpr usart::Port usart1(usart::Port::USART1);
constexpr gpio::Pin usart1TX(gpio::Bank::A, 9);
constexpr gpio::Pin pinLed(gpio::Bank::C, 13);

constexpr gpio::Pin pinAM2303(gpio::Bank::A, 8);
constexpr gpio::Pin pinDebug(gpio::Bank::A, 10);

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

        rcc::EnableClock(rcc::PeripheralClock::IOPB);
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
        gpio::ClearPin(pinAM2303);
        gpio::SetupPin(pinAM2303, gpio::Config::Input_Floating);

        gpio::ClearPin(pinDebug);
        gpio::SetupPin(pinDebug, gpio::Config::Output_50MHz_General_Push_Pull);
    }

    void ReadAM2303()
    {
        // Set output, bring SDA down for at least 800uS, set input, wait for lo->hi
        gpio::SetupPin(pinAM2303, gpio::Config::Output_50MHz_General_Open_Drain);

        gpio::ClearPin(pinAM2303);
        timer::Delay_ms<Clock>(20);

        // Set input, wait for acknowledgement
        gpio::SetupPin(pinAM2303, gpio::Config::Input_Floating);
        gpio::ClearPin(pinAM2303);

        while (gpio::ReadPin(pinAM2303));
        while (!gpio::ReadPin(pinAM2303));
        while (gpio::ReadPin(pinAM2303));

        // Collect data bytes
        uint8_t response[5];
        for(int n = 0; n < 5; ++n) {
            uint8_t v = 0;
            for(int i = 0; i < 8; ++i) {
                while (!gpio::ReadPin(pinAM2303)); // should last 50uS
                systick::Delay_us<Clock>(30);
                v <<= 1;
                if (gpio::ReadPin(pinAM2303))
                    v |= 1;
                while(gpio::ReadPin(pinAM2303));
            }
            response[n] = v;
        }

        format::Print(usart1, "Read values ", (unsigned int)response[0], " ", (unsigned int)response[1], " ", (unsigned int)response[2], " ", (unsigned int)response[3], " ", (unsigned int)response[4], "\r\n");
        const uint8_t checksum = response[0] + response[1] + response[2] + response[3];

        const uint16_t rh = (response[0] << 8) | response[1];
        const uint16_t temp  = (response[2] << 8) | response[3];

        format::Print(usart1, "Decoded values: RH ", (unsigned int)rh, " T ", (unsigned int)temp, " checksum ", (unsigned int)checksum, "\r\n");
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
    // Wait for stable state
    format::Print(usart1, "hi?\r\n");
    timer::Delay_ms<Clock>(500);

    format::Print(usart1, "active\r\n");
    for (unsigned int counter = 0;; ++counter) {
        gpio::TogglePin(pinLed);
        format::Print(usart1, "Hello world ", counter, " ", static_cast<uint32_t>(counter), "\r\n");
        ReadAM2303();

        timer::Delay_ms<Clock>(5'000);
    }
}
