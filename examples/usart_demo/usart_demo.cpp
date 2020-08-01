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

constexpr int USART1_TX = 9; // PA9
constexpr int GPIO_LED = 13; // PC13

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
        gpio::SetupPin(
            gpio::GPIOA_, USART1_TX, gpio::gpio_cr::MODE_OUTPUT_50MHZ,
            gpio::gpio_cr::CNF_OUT_ALT_PUSH_PULL);

        usart::Configure(
            115200, usart::DataBits::DB_8, usart::Parity::None, usart::StopBits::SB_1,
            usart::FlowControl::None);
        usart::Register(usart::USART_CR1) |= usart::cr1::TE; // enable TX
        usart::Register(usart::USART_CR1) |= usart::cr1::UE; // enable UART
    }

    void gpio_init()
    {
        gpio::ClearPin(gpio::GPIOC_, GPIO_LED);
        gpio::SetupPin(
            gpio::GPIOC_, GPIO_LED, gpio::gpio_cr::MODE_OUTPUT_50MHZ,
            gpio::gpio_cr::CNF_OUT_GENERAL_PUSH_PULL);
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
        gpio::TogglePin(gpio::GPIOC_, GPIO_LED);
        format::Print("Tadah! ", counter, " ", static_cast<uint32_t>(counter), "\r\n");

        for (int i = 0; i < 1000000; ++i)
            __asm __volatile("nop");
    }
}
