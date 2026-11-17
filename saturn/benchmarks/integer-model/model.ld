/* Link script for the executed-instruction model: everything lives in the
 * harness's 1 MiB flat RAM at 0x06000000; the harness provides the stack. */
SECTIONS
{
  . = 0x06004000;
  .text   : { *(.text .text.*) }
  .rodata : { *(.rodata .rodata.*) }
  .data   : { *(.data .data.*) }
  .bss    : { *(.bss .bss.*) *(COMMON) }
  /DISCARD/ : { *(.eh_frame) *(.comment) *(.stack) }
}
