ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
    DIAG (rw) : ORIGIN = 0x20000000, LENGTH = 256
    RAM (rwx) : ORIGIN = 0x20000100, LENGTH = 9984
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH
    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
    .diagnostic (NOLOAD) : { KEEP(*(.diagnostic)) } > DIAG
    /DISCARD/ : { *(.note.GNU-stack) *(.comment) }
    ASSERT(SIZEOF(.diagnostic) == 256, "Diagnostic ABI must be 256 bytes")
    ASSERT(_ebss <= _estack - 2048, "Keep at least 2 KiB for stack")
}
