# Section garbage-collection probe (sections-1)

A consumer that links SH2ZAM with `-ffunction-sections -fdata-sections` and
`-Wl,--gc-sections` should get only the code and tables it calls, transitively.
That works for C because the compiler gives every function and object its own
section. It works for the hand-written SH-2 entries only because each one is
assembled into its own `.text.<symbol>` section and each asm table into its
own `.rodata.<name>` section, and because every read-only C table is placed
in `.rodata.<name>` explicitly (`SH2ZI_RODATA`, `saturn/src/sh2zam_sections.h`).
This suite checks that, in the SH-2 cross workflow.

`ci.sh build`, inside the Yaul toolchain:

1. `sh2z_require_per_symbol_sections saturn/src/*.c` (`../ci/lib.sh`): each
   library source compiled with the consumer flags may leave nothing in the
   plain `.text`, `.data`, `.rodata` or `.bss` sections.
2. Compiles every library source with the consumer flags into
   `build/sh2/gc-probe/obj/` and archives them.
3. For each `probe_<name>` in `probe.c` (one call to one public entry point),
   links an ELF whose only root is that probe, with `--gc-sections`, once from
   the objects (sources compiled into the program) and once from the archive.
4. Lists the hand-written entries (the global functions that the CI objects
   hold outside plain `.text`) and the tables (their `.rodata.<name>`
   sections) that survive, and fails unless both link modes give exactly the
   set `ci.sh` expects for that probe.

The sizes go to `sh2-gc-probe.txt`: `.text + .rodata + .data + .bss` of the
linked ELF, libgcc helpers included, less the probe function itself. The
probes are never run.

## What a consumer does

Compile the library sources (directly, or into an archive) and the program
with `-ffunction-sections -fdata-sections`, and link with `-Wl,--gc-sections`.
Either way, an entry point that nothing calls, and a table that nothing
reachable reads, is not linked. Without `-ffunction-sections` the C functions
of one source share `.text` and are kept together, but the hand-written
entries and the tables still sit in sections of their own.

A linker script can also place a section by name, for example a cold table
in low work RAM:

```ld
.lwram_tables : { *(.rodata.sh2z_sin12_quarter_u30 .rodata.sh2zi_cot_q20) } > lwram
```
