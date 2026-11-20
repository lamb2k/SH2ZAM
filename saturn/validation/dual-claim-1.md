# Validation record — 2026-11-19 (two-CPU try-claim, dual-claim-1)

`sh2zam/dual_claim_experimental.h` ([guide](../DUAL_CLAIM.md)) lets two processors share an
optional job so that exactly one runs it, with plain volatile words and no atomic instruction.
**No console run was done in this repository.** Evidence by tier:

- **HOST:** `dual_claim_exact` checks every interleaving of the two sides' operations (240 paths
  over fresh and left-over records, three tags including the largest): one owner and no
  deadlock on every path. The real functions pass in each sequential order over 2,000 tags.
  Negative controls: four broken protocols must each allow two owners, and they do. Built under
  GCC with `-Wall -Wextra -Werror` and under Clang with ASan/UBSan.
- **SH-2 build:** compiles with the CI flags (`-m2 -mb -O2 -ffreestanding -Werror`).
- **EMULATOR (consumer, not this repository):** a Saturn racing engine's cloud-preparation
  steal (the slave takes the job when idle) uses the same protocol in its glue. Ymir,
  timing modelled, 4,733 frames: the slave took the job 3,457 times. A chain hash of every
  frame's output is `08759b9d` in that run and in the runs forced to the master only and to the
  slave only.
- **Not done:** console execution and a bus-contention measurement.
