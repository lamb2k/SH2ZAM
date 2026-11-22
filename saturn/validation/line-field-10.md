# Validation record — 2026-11-22 (line field contact passes without repeated work, line-field-10)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds no entry point and no parameter. The
results are the line-field-9 step's, bit for bit; only the work changes.

A step runs up to three contact passes (a pass that pushes more than one pair is followed by another). Each pass
scanned the order from every vehicle until the first vehicle out of reach, and sampled the line under a vehicle
once for every pair and once for every push. In a pile (a pack bunched at a start, a field in a tube) that is
three scans of about a hundred gaps and some ninety samples in one step. A contact moves a vehicle sideways and
changes its speed and energy, never its distance or node, so:

- **One sample a vehicle.** The line sample under a vehicle (flags, half width) is the same for the whole step.
  It is taken on first use and kept for the first 32 vehicles; the others are sampled where used.
- **One list of pairs.** While no vehicle retires, the pairs within the contact reach along the line, in scan
  order, are the same in every pass. The first pass scans in full and lists them (up to 192), and the later passes
  walk the list.
- **Retirement.** The only change to the scan's rules during contact is a vehicle retiring (it leaves the scan).
  A list is walked only while the retirement count is the one it was scanned at. If a pass retires a vehicle
  part-way, it resumes as a full scan from the position after the pair it was on, and the next pass scans in full.
  A pile of more than 192 pairs is not listed and its passes scan in full.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,143,323 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`) and under GCC with ASan and UBSan. Every pin of the line-field-9 suite is unchanged (six plain
  races, six behaviour races, the scenes). Two new checks record the line-field-9 step's results:
  - five stacked frozen agents, with none, one, two or all retiring in the first pass (4 hashes);
  - 3000 random piles of 4 to 10 agents (every eleventh 28 to 32, which overflows the pair list; every fifth
    straddling the line's start; some external vehicles; low energy so that agents retire in the first and in
    later passes), one hash over every agent's lateral, speed, energy, flags and kicks and the field's counters.
    The same scene generator over 30000 piles gives the same hash on the line-field-9 and line-field-10 sources.
- **Controls:** a retirement changes the stacked scene's result, and 2999 piles hash differently from 3000.
- **Mutation controls (10 of 11 caught, the other equivalent):** the retirement check on the list removed; the list
  sample slot of the wrong vehicle; the list walked in reverse; no resume after a mid-pass retirement; the resume
  one column early; the resume re-checking the row's first vehicle; the kept sample off by one; the pair gap
  replaced by a constant; a fourth pass; and an overflowing list kept truncated. The caught mutants fail the pins,
  the stacked scenes or the piles. A mutant that drops an entry by treating the list as full is equivalent
  (the passes scan in full), and passes.
- **Consumer (not this repository):** a Saturn racing engine's native field, six courses, seeds 1 to 6, with
  behaviours on and off, built on the line-field-9 and line-field-10 sources: 72 of 72 reports equal (final state
  hash, contacts, falls, knock-outs and every counter). In an emulator run (EMULATOR, timing MODELLED) of the
  engine at a pile at a race start, the contact passes of the worst step took 2.1 ms of 4.0 ms (3 passes, 303
  gaps, 63 pairs in reach); the field step's 99th percentile on its tube course fell from 6.4 to 4.9 ms, and the
  frames with a step over 4.5 ms from 145 to 69 of 2900 (second tube course: 22 to 7 of 3900).
- **Not done:** an instruction-model capture and console execution.
