# Experimental fused SH-2 batch kernels

Two explicitly selected candidates implement the same prepared transform:
`MAC.L` accumulation and `DMULS.L` with `ADDC` accumulation. Neither replaces
the normal prepared executor or the existing automatic batch dispatcher.
They are cross-compiled and inspected, but have **not been executed or timed
on a Saturn**. Static instruction counts do not establish a speedup.

## Verified generated code

The final inspected build is commit
[`8943e8297f41158796191527639ddc35510c6d4c`](https://github.com/lamb2k/SH2ZAM/commit/8943e8297f41158796191527639ddc35510c6d4c),
using `sh2eb-elf-gcc 13.2.0` with `-m2 -mb -O2 -ffreestanding -std=c11`.
The [successful SH-2 cross build](https://github.com/lamb2k/SH2ZAM/actions/runs/43486731579)
contains `sh2-fused-disassembly.txt` and `sh2-fused-audit.txt` in its
`sh2-compile-evidence` artifact. Both candidate kernels link into the diagnostic
Saturn benchmark image.

| Generated vertex loop | MAC.L candidate | DMULS.L candidate |
| --- | ---: | ---: |
| Signed 32 × 32 multiplications per vertex | 9 | 9 |
| `MAC.L` instructions | 9 | 0 |
| `DMULS.L` instructions | 0 | 9 |
| `STS MACH/MACL` transfers | 6 | 18 |
| `ADDC` instructions | 0 | 12 |
| `XTRCT` instructions | 3 | 3 |
| Coordinate word reads per vertex | 9 | 3 |
| Coefficient/translation word reads per vertex | 12 | 12 |
| Stack references inside vertex loop | 0 | 0 |
| Calls inside vertex loop | 0 | 0 |
| Saved registers per kernel call | r8–r11 | r8–r13 |
| Kernel save area | 16 bytes | 24 bytes |
| Additional local stack frame | 0 bytes | 0 bytes |

These frame sizes describe the two kernel functions, excluding the caller,
range-validation wrapper and plan construction. The MAC kernel is
`sh2zi_batch_mac_kernel`; the multiplication/carry kernel is
`sh2zi_batch_dmuls_kernel`.

The first DMULS version had a 44-byte total frame and reloaded a spilled row
pointer on each vertex. Inspection prompted two changes: model assembly reads
through the enclosing matrix, and store each completed row after capturing
all three input coordinates. The final build removes that spill and the local
frame. The numeric operations are unchanged.

MAC retains the accumulator across all three products of a row and extracts it
once. Its memory operands reread xyz for each row. DMULS loads xyz once, then
adds each subsequent product directly into a high/low register pair. The
extra MAC input traffic is a reason to measure both candidates with actual
memory placement and cache behavior before selecting one.

## Numeric and CPU contracts

Plan construction proves, for each matrix row, the raw-integer bound

```
sum(abs(coefficient[j]) * mesh.max_abs[j])
    + abs(translation) * 65536 <= 2^47 - 1
```

This bounds every partial sum and the final translated result. `XTRCT`
selects accumulator bits 47 through 16, giving the exact Q16.16 floor even for
negative fractional values. Translation then fits a signed 32-bit output.
There is no intermediate product rounding or approximation. Ineligible plans
use the ordinary prepared executor and its full-range fallback.

The [Renesas SH-1/SH-2/SH-DSP Software Manual](https://www.renesas.com/en/document/mas/sh-1sh-2sh-dsp-software-manual),
revision 5, documents `MAC.L` in section 6.1.31, `DMULS.L` in 6.1.20, `ADDC`
in 6.1.3 and `XTRCT` in 6.1.72. `MAC.L` has different accumulation semantics
depending on SR.S. This candidate explicitly requires **SR.S = 0**, checks it
on the executing CPU and falls back when S is set. It never writes SR to
change saturation or interrupt control. The audit checks for an SR read and
the absence of SR writes.

Interrupt handlers using MACH or MACL must preserve both registers. This is
also required by the existing native multiply backend; fused accumulation
keeps them live through several instructions. The prepared plan and borrowed
source mesh remain immutable. Range validation rejects output overlap with
the entire source mesh and with plan storage before any writes.

## Explicit use

Include [`sh2zam/batch_experimental.h`](../include/sh2zam/batch_experimental.h)
and initialize the normal mesh view and plan:

```c
sh2z_mesh_view_t mesh;
sh2z_transform_plan_t plan;
if (!sh2z_mesh_view_init(&mesh, positions, vertex_count)) return false;
if (!sh2z_transform_plan_init(&plan, &mesh, &matrix)) return false;

/* Record this on the CPU that will execute the batch. */
bool uses_native_mac = sh2z_transform_plan_mac_available(&plan);
if (!sh2z_transform_plan_execute_mac(output, &plan, 0, vertex_count))
    return false;
```

The alternative names are `sh2z_transform_plan_dmuls_available` and
`sh2z_transform_plan_execute_dmuls`. Output starts at `output[0]`; `first` and
`count` select the source range. Availability is false on host and
`SH2Z_FORCE_PORTABLE` builds, and for ineligible plans. A successful call can
therefore represent fallback execution: record availability instead of labeling
every call as native MAC or DMULS.

The implementation is in [`sh2zam_batch_sh2.c`](../src/sh2zam_batch_sh2.c).
[`audit-fused.py`](audit-fused.py) checks actual SH-2 disassembly and prints the
complete loops. [`test_batch_native.c`](../tests/test_batch_native.c) adds
2,651,515 host instruction-model and wrapper checks against an independent
wide-integer oracle. Those checks validate carry/extraction arithmetic and
fallback contracts; they do not execute the native instructions.

The benchmark must compare identical inputs and precision checks across
software float, the safe batch implementation, prepared C, native MAC and
native DMULS. Report per-matrix plan construction with the transformed batch;
record the one-time mesh scan separately. Hardware timing and native execution
results remain the evidence required to choose a production kernel.
