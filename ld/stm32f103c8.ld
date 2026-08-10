/* STM32F103C8T6 - 64K flash, 20K ram
   Many Blue Pill clones actually carry 128K flash
   Bump the LENGTH below only after verifying with st-info --probe */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack_size = 0x400;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*)
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  ._user_stack :
  {
    . = ALIGN(8);
    . = . + _min_stack_size;
    . = ALIGN(8);
  } > RAM
}
