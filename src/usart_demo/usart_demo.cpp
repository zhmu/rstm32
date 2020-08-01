/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include <cstdint>

constexpr int USART1_TX = 9; // PA9
constexpr int GPIO_LED = 13; // PC13

namespace flash
{
    volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x40022000 + offset);
    }

    constexpr uint32_t FLASH_ACR = 0x00;
    namespace acr
    {
        constexpr uint32_t LATENCY_MASK = (1 << 2) | (1 << 1) | (1 << 0);
        constexpr uint32_t LATENCY_0_WS = (0 << 2) | (0 << 1) | (0 << 0);
        constexpr uint32_t LATENCY_1_WS = (0 << 2) | (0 << 1) | (1 << 0);
        constexpr uint32_t LATENCY_2_WS = (0 << 2) | (1 << 1) | (0 << 0);
    } // namespace acr
} // namespace flash

namespace gpio
{
    constexpr uint32_t GPIOA_ = 0x0800;
    constexpr uint32_t GPIOB_ = 0x0c00;
    constexpr uint32_t GPIOC_ = 0x1000;
    constexpr uint32_t GPIOD_ = 0x1400;
    constexpr uint32_t GPIOE_ = 0x1800;
    constexpr uint32_t GPIOF_ = 0x1c00;
    constexpr uint32_t GPIOG_ = 0x2000;

    volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0000 + offset);
    }
    volatile uint32_t& RegisterG(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'2000 + offset);
    }
    volatile uint32_t& RegisterF(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1c00 + offset);
    }
    volatile uint32_t& RegisterE(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1800 + offset);
    }
    volatile uint32_t& RegisterD(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1400 + offset);
    }
    volatile uint32_t& RegisterC(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'1000 + offset);
    }
    volatile uint32_t& RegisterB(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0c00 + offset);
    }
    volatile uint32_t& RegisterA(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4001'0800 + offset);
    }

    constexpr uint32_t GPIOx_CRL = 0x00;
    constexpr uint32_t GPIOx_CRH = 0x04;
    namespace gpio_cr
    {
        constexpr uint32_t MODE_MASK = (1 << 1) | (1 << 0);
        constexpr uint32_t MODE_INPUT = (0 << 1) | (0 << 0);
        constexpr uint32_t MODE_OUTPUT_10MHZ = (0 << 1) | (1 << 0);
        constexpr uint32_t MODE_OUTPUT_2MHZ = (1 << 1) | (0 << 0);
        constexpr uint32_t MODE_OUTPUT_50MHZ = (1 << 1) | (1 << 0);

        constexpr uint32_t CNF_MASK = (1 << 1) | (1 << 0);
        constexpr uint32_t CNF_IN_ANALOG = (0 << 1) | (0 << 0);
        constexpr uint32_t CNF_IN_FLOATING = (0 << 1) | (0 << 0);
        constexpr uint32_t CNF_IN_PULL_UPDOWN = (1 << 1) | (0 << 0);

        constexpr uint32_t CNF_OUT_GENERAL_PUSH_PULL = (0 << 1) | (0 << 0);
        constexpr uint32_t CNF_OUT_GENERAL_OPEN_DRAIN = (0 << 1) | (1 << 0);
        constexpr uint32_t CNF_OUT_ALT_PUSH_PULL = (1 << 1) | (0 << 0);
        constexpr uint32_t CNF_OUT_ALT_OPEN_DRAIN = (1 << 1) | (1 << 0);
    } // namespace gpio_cr

    constexpr uint32_t GPIOx_IDR = 0x08;
    constexpr uint32_t GPIOx_ODR = 0x0c;
    constexpr uint32_t GPIOx_BSRR = 0x10;
    constexpr uint32_t GPIOx_BRR = 0x14;

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

    void TogglePin(int gpioBase, int pin) { gpio::Register(gpioBase + GPIOx_ODR) ^= (1 << pin); }

    void ClearPin(int gpioBase, int pin) { gpio::Register(gpioBase + GPIOx_ODR) &= ~(1 << pin); }

    void SetPin(int gpioBase, int pin) { gpio::Register(gpioBase + GPIOx_ODR) |= (1 << pin); }

} // namespace gpio

namespace rcc
{
    volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x4002'1000 + offset);
    }

    constexpr uint32_t RCC_CR = 0x00;
    namespace cr
    {
        constexpr uint32_t HSION = (1 << 0);
        constexpr uint32_t HSIRDY = (1 << 1);
        constexpr uint32_t HSEON = (1 << 16);
        constexpr uint32_t HSERDY = (1 << 17);
        constexpr uint32_t HSEBYP = (1 << 18);
        constexpr uint32_t CSSON = (1 << 19);
        constexpr uint32_t PLLON = (1 << 24);
        constexpr uint32_t PLLRDY = (1 << 25);
    } // namespace cr
    constexpr uint32_t RCC_CFGR = 0x04;
    namespace cfgr
    {
        constexpr uint32_t SW_MASK = (1 << 1) | (1 << 0);
        constexpr uint32_t SW_HSI = (0 << 1) | (0 << 0);
        constexpr uint32_t SW_HSE = (0 << 1) | (0 << 1);
        constexpr uint32_t SW_PLL = (1 << 1) | (0 << 1);

        constexpr uint32_t HPRE_MASK = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4);
        constexpr uint32_t HPRE_NO_DIV = (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4);
        constexpr uint32_t HPRE_DIV_2 = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4);

        constexpr uint32_t PPRE1_MASK = (1 << 10) | (1 << 9) | (1 << 8);
        constexpr uint32_t PPRE1_NO_DIV = (0 << 10) | (0 << 9) | (0 << 8);
        constexpr uint32_t PPRE1_DIV_2 = (1 << 10) | (0 << 9) | (0 << 8);

        constexpr uint32_t PPRE2_MASK = (1 << 13) | (1 << 12) | (1 << 11);
        constexpr uint32_t PPRE2_NO_DIV = (0 << 13) | (0 << 12) | (0 << 11);

        constexpr uint32_t ADCPRE_MASK = (1 << 15) | (1 << 14);
        constexpr uint32_t ADCPRE_DIV_8 = (1 << 15) | (1 << 14);

        constexpr uint32_t PLLSRC_MASK = (1 << 16);
        constexpr uint32_t PLLSRC_HSI = (0 << 16);
        constexpr uint32_t PLLSRC_HSE = (1 << 16);

        constexpr uint32_t PLLXTPRE_MASK = (1 << 17);
        constexpr uint32_t PLLXTPRE_NO_DIV = (0 << 17);
        constexpr uint32_t PLLXTPRE_DIV_2 = (1 << 17);

        constexpr uint32_t PLLMUL_MASK = (1 << 21) | (1 << 20) | (1 << 19) | (1 << 18);
        constexpr uint32_t PLLMUL_MUL_9 = (0 << 21) | (1 << 20) | (1 << 19) | (1 << 18);
    } // namespace cfgr
    constexpr uint32_t RCC_CIR = 0x08;
    constexpr uint32_t RCC_APB2RSTR = 0x0c;
    constexpr uint32_t RCC_APB1RSTR = 0x10;
    constexpr uint32_t RCC_AHBENR = 0x14;
    constexpr uint32_t RCC_APB2ENR = 0x18;
    namespace apb2enr
    {
        constexpr uint32_t AFIOEN = (1 << 0);
        constexpr uint32_t IOPAEN = (1 << 2);
        constexpr uint32_t IOPCEN = (1 << 4);
        constexpr uint32_t USART1EN = (1 << 14);
    } // namespace apb2enr
    constexpr uint32_t RCC_APB1ENR = 0x1c;
    constexpr uint32_t RCC_BDCR = 0x20;
    constexpr uint32_t RCC_CSR = 0x24;

    void setup_hse_8mhz_pll_72mhz()
    {
        // Switch to internal high-speed (8MHz) clock
        Register(RCC_CR) |= cr::HSION;
        while ((Register(RCC_CR) & cr::HSIRDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_HSI;

        // Switch to external 8MHz crystal
        Register(RCC_CR) |= cr::HSEON;
        while ((Register(RCC_CR) & cr::HSERDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_HSE;

        // Prescalers
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::HPRE_MASK) | cfgr::HPRE_NO_DIV;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::ADCPRE_MASK) | cfgr::ADCPRE_DIV_8;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PPRE1_MASK) | cfgr::PPRE1_DIV_2;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PPRE2_MASK) | cfgr::PPRE2_NO_DIV;
        // Up flash WS to 2, as we are between 48..72MHz
        flash::Register(flash::FLASH_ACR) =
            (flash::Register(flash::FLASH_ACR) & ~flash::acr::LATENCY_MASK) |
            flash::acr::LATENCY_2_WS;

        // Setup PLL as HSE (8Mhz) * 9 = 72MHz
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLMUL_MASK) | cfgr::PLLMUL_MUL_9;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLSRC_MASK) | cfgr::PLLSRC_HSE;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::PLLXTPRE_MASK) | cfgr::PLLXTPRE_NO_DIV;

        // Switch to PLL
        Register(RCC_CR) |= cr::PLLON;
        while ((Register(RCC_CR) & cr::PLLRDY) == 0)
            ;
        Register(RCC_CFGR) = (Register(RCC_CFGR) & ~cfgr::SW_MASK) | cfgr::SW_PLL;

        // APB1 = 36MHz, APB2 = 72MHz
    }
} // namespace rcc

namespace usart
{
    volatile uint32_t& Register(uint32_t offset)
    {
        return *reinterpret_cast<volatile uint32_t*>(0x40013800 + offset);
    }
    constexpr uint32_t USART_SR = 0x00;
    namespace sr
    {
        constexpr uint32_t TXE = (1 << 7);
        constexpr uint32_t TC = (1 << 6);
        constexpr uint32_t RXNE = (1 << 5);
    } // namespace sr
    constexpr uint32_t USART_DR = 0x04;
    constexpr uint32_t USART_BRR = 0x08;
    constexpr uint32_t USART_CR1 = 0x0c;

    namespace cr1
    {
        constexpr uint32_t UE = (1 << 13);
        constexpr uint32_t M = (1 << 12);
        constexpr uint32_t PCE = (1 << 10);
        constexpr uint32_t PS = (1 << 9);
        constexpr uint32_t TE = (1 << 3);
        constexpr uint32_t RE = (1 << 2);
    } // namespace cr1
    enum class DataBits : uint32_t { DB_8 = 0, DB_9 = cr1::M };
    enum class Parity : uint32_t { None = 0, Even = cr1::PCE, Odd = cr1::PCE | cr1::PS };

    constexpr uint32_t USART_CR2 = 0x10;
    namespace cr2
    {
        constexpr uint32_t STOP_MASK = (1 << 13) | (1 << 12);
        constexpr uint32_t STOP_1 = (0 << 13) | (0 << 12);
        constexpr uint32_t STOP_0_5 = (0 << 13) | (1 << 12);
        constexpr uint32_t STOP_2 = (1 << 13) | (0 << 12);
        constexpr uint32_t STOP_1_5 = (1 << 13) | (1 << 12);
    } // namespace cr2
    enum class StopBits : uint32_t {
        SB_1 = cr2::STOP_1,
        SB_0_5 = cr2::STOP_0_5,
        SB_2 = cr2::STOP_2,
        SB_1_5 = cr2::STOP_1_5
    };

    constexpr uint32_t USART_CR3 = 0x14;
    namespace cr3
    {
        constexpr uint32_t CTSE = (1 << 9);
        constexpr uint32_t RTSE = (1 << 8);
    } // namespace cr3
    enum class FlowControl : uint32_t {
        None = 0,
        RTS = cr3::RTSE,
        CTS = cr3::CTSE,
        RTS_CTS = cr3::RTSE | cr3::CTSE
    };
    constexpr uint32_t USART_GTPR = 0x18;

    void SetBaudRate(int baud)
    {
        constexpr int fPCLK2 = 72'000'000;
        const int value_mantissa = (fPCLK2 / 16) / baud;
        // Fraction is n/16-ths of the result; should we round up here?
        const int value_modulo = (fPCLK2 / 16) % baud;
        const int value_fraction = value_modulo / (baud / 16);
        Register(USART_BRR) = (value_mantissa << 4) | (value_fraction & 15);
    }

    void SetSettings(const DataBits db, const Parity p, const StopBits sb, const FlowControl fc)
    {
        uint32_t cr1 = Register(USART_CR1);
        cr1 &= ~(cr1::M | cr1::PCE | cr1::PS);
        cr1 |= static_cast<uint32_t>(db);
        cr1 |= static_cast<uint32_t>(p);

        uint32_t cr2 = Register(USART_CR2);
        cr2 &= ~(cr2::STOP_MASK);
        cr2 |= static_cast<uint32_t>(sb);

        uint32_t cr3 = Register(USART_CR3);
        cr3 &= ~(cr3::CTSE | cr3::RTSE);
        cr3 |= static_cast<uint32_t>(fc);

        Register(USART_CR1) = cr1;
        Register(USART_CR2) = cr2;
        Register(USART_CR3) = cr3;
    }

    void Write(int ch)
    {
        while ((usart::Register(usart::USART_SR) & usart::sr::TXE) == 0)
            ;
        usart::Register(usart::USART_DR) = ch;
    }
} // namespace usart

namespace
{
    void clock_init()
    {
        rcc::setup_hse_8mhz_pll_72mhz();

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

        usart::SetBaudRate(115200);
        usart::SetSettings(
            usart::DataBits::DB_8, usart::Parity::None, usart::StopBits::SB_1,
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

void Print(const char* s)
{
    for (; *s != '\0'; ++s)
        usart::Write(*s);
}

constexpr char hextab[] = "0123456789abcdef";

template<typename T>
void PrintNumber(const int base, const T value)
{
    unsigned int divisor = 1, p = 0;
    for (T n = value; n >= static_cast<T>(base); n /= base, divisor *= base, p++)
        ;
    for (unsigned int i = 0; i <= p; i++, divisor /= base)
        usart::Write(hextab[(value / divisor) % base]);
}

void Print(unsigned int v) { PrintNumber(10, v); }

void Print(uint32_t v)
{
    Print("0x");
    PrintNumber(16, v);
}

template<typename... Args>
void Print(Args... args)
{
    (Print(args), ...);
}

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
        Print("Tadah! ", counter, " ", static_cast<uint32_t>(counter), "\r\n");

        for (int i = 0; i < 1000000; ++i)
            __asm __volatile("nop");
    }
}
