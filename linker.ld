ENTRY(_start)

SECTIONS
{
    . = 0x8000;

    .text.boot : {
        *(.text.boot)
    }

    .text : {
        *(.text)
        *(.text.*)
    }

    .rodata : {
        *(.rodata)
        *(.rodata.*)
    }

    .data : {
        *(.data)
        *(.data.*)
    }

    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        _bss_end = .;
    }

    .stack (NOLOAD) : {
        . = ALIGN(8);
        _stack_start = .;
        . += 0x4000;
        _stack_top = .;
    }

    .heap (NOLOAD) : {
        . = ALIGN(8);
        _heap_start = .;
        . += 0x100000;
        _heap_end = .;
    }
}
