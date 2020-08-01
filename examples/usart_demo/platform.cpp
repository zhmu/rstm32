/*-
 * SPDX-License-Identifier: Zlib
 *
 * Copyright (c) 2020 Rink Springer <rink@rink.nu>
 * For conditions of distribution and use, see LICENSE file
 */
#include <cstdint>

using FunctionPtr = void (*)();

extern "C" {

void Default_Handler() {}

void _data_source();
void _data_start();
void _data_end();

void _bss_start();
void _bss_end();

void _preinit_array_start();
void _preinit_array_end();

void _init_array_start();
void _init_array_end();

int main();

void Reset_Handler()
{
    // Copy data from flash to RAM
    if (auto dst = reinterpret_cast<uint32_t*>(&_data_start); true) {
        auto src = reinterpret_cast<uint32_t*>(&_data_source);
        while (dst < reinterpret_cast<uint32_t*>(&_data_end))
            *dst++ = *src++;

        // Zero-initialise BSS
        while (dst < reinterpret_cast<uint32_t*>(&_bss_end))
            *dst++ = 0;
    }

    constexpr auto runFunctionChain = []<typename T>(T start, T end)
    {
        for (auto initFn = reinterpret_cast<uint32_t*>(start);
             initFn < reinterpret_cast<uint32_t*>(end); ++initFn) {
            (reinterpret_cast<FunctionPtr>(initFn))();
        }
    };

    // Run C++ constructors
    runFunctionChain(&_preinit_array_start, &_preinit_array_end);
    runFunctionChain(&_init_array_start, &_init_array_end);

    main();

    // Destructors? Reboot?
    while (1)
        ;
}

void _stack();

// Default everything to Default_Handler()
void NMI_Handler() __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler() __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler() __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler() __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__((weak, alias("Default_Handler")));
void SVC_Handler() __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler() __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__((weak, alias("Default_Handler")));

// Cortex-M3, 2.3.4 Vector Table, Figure 12
__attribute__((used, section(".vector_table")))
const FunctionPtr vector_table[] = {reinterpret_cast<FunctionPtr>(&_stack),
                                    Reset_Handler,
                                    NMI_Handler,
                                    HardFault_Handler,
                                    MemManage_Handler,
                                    BusFault_Handler,
                                    UsageFault_Handler,
                                    0,
                                    0,
                                    0,
                                    0,
                                    SVC_Handler,
                                    0,
                                    0,
                                    PendSV_Handler,
                                    SysTick_Handler};
}
