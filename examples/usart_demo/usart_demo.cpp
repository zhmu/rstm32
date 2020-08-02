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
#include "rstm32/usart.h"

constexpr gpio::Pin usart1TX(gpio::Bank::A, 9);
constexpr gpio::Pin pinLed(gpio::Bank::C, 13);

namespace
{
    void clock_init()
    {
        rcc::Setup_8MHz_External_Crystal_Yielding_72MHz_PLL();

        rcc::Register(rcc::RCC_APB2ENR) |= rcc::apb2enr::IOPCEN; // GPIOC
        rcc::Register(rcc::RCC_APB2ENR) |= rcc::apb2enr::IOPAEN; // GPIOA
        rcc::Register(rcc::RCC_APB2ENR) |= rcc::apb2enr::AFIOEN;
        rcc::Register(rcc::RCC_APB2ENR) |= rcc::apb2enr::USART1EN;
    }

    void usart_init()
    {
        gpio::SetupPin(usart1TX, gpio::Config::Output_50MHz_Alt_Push_Pull);

        usart::Configure(
            115200, usart::DataBits::DB_8, usart::Parity::None, usart::StopBits::SB_1,
            usart::FlowControl::None);
        usart::Register(usart::USART_CR1) |= usart::cr1::TE; // enable TX
        usart::Register(usart::USART_CR1) |= usart::cr1::UE; // enable UART
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

    for (unsigned int counter = 0;; ++counter) {
        gpio::TogglePin(pinLed);
        format::Print("Tadah! ", counter, " ", static_cast<uint32_t>(counter), "\r\n");

        for (int i = 0; i < 1000000; ++i)
            __asm __volatile("nop");
    }
}
