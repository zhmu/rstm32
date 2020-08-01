/* STM32F103C8 */
MEMORY
{
    flash (rx)  : ORIGIN = 0x08000000, LENGTH = 64K /* try 128KB; most boards seem to have it */
    ram   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(Reset_Handler)

SECTIONS {
    /* Vector table, code and read-only data */
    .text : {
        FILL(0xff)
        KEEP(*(.vector_table))
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
    } >flash

    /* Constructors */
    .preinit_array : {
        . = ALIGN(4);
        _preinit_array_start = .;
        KEEP(*(.preinit_array))
        _preinit_array_end = .;
    } >flash
    .init_array : {
        . = ALIGN(4);
        _init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*.init_array)
        _init_array_end = .;
    } >flash

    /* Destructors */
    .fini_array : {
        . = ALIGN(4);
        _fini_array_start = .;
        KEEP(*.fini_array)
        KEEP(*(SORT(.fini_array.*)))
        _fini_array_end = .;
    } >flash

    /* Read/write data - initially located in flash, will be copied to RAM */
    .data : {
        _data_start = .;
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } >ram AT >flash
    PROVIDE(_data_source = LOADADDR(.data));

    /* Read/write zero-initialized data */
    .bss : {
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } >ram

    . = ALIGN(4);
    end = .;
}

/* Position initial stack at end of memory, growing downwards */
PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
