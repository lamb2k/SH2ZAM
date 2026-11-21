# Validation record — 2026-11-21 (line field step entry points and exact step cuts, line-field-7)

`sh2zam/line_field_experimental.h` ([guide](../LINE_FOLLOWER.md)) adds `sh2z_field_step_plain` and
`sh2z_field_step_behave`. Each gives `sh2z_field_step`'s state, step for step, for a field without or with a
behaviour block. It also adds `sh2z_line_locate_far`, a node lookup by halving. The same release removes
repeated work from the step, with results unchanged:
- the roll's exit look-ahead uses the halving lookup instead of walking node by node;
- the contact test samples the line for a tube's seam on tube nodes only;
- the rank sort carries each key instead of computing it again;
- an agent under lateral inertia takes speed and distance from the step's own sample;
- lap bookkeeping runs only when an agent's lap moved.

**No console run was done in this repository.** Evidence by tier:

- **HOST:** `line_field_exact` passes 26,150,715 checks under GCC and Clang (`-O2 -Wall -Wextra -Werror
  -pedantic`, ASan and UBSan):
  - each entry point gives `sh2z_field_step`'s hashes; with every behaviour on, the behaviour step gives the
    hashes line-field-5's single step gave (pinned from main 7f99eb7), and the plain step the pinned
    line-field-1 hashes;
  - `sh2z_line_locate_far` equals `sh2z_line_locate` from every hint, at every node start and either side of
    it, below 0 and past the length, for look-aheads crossing the start line and at random points, on an even
    line, a line with zero-length nodes and a one-node line;
  - a pair either side of a tube's seam touches;
  - an agent under lateral inertia keeps the line's speed cap after braking at its own rate.
- **Controls:** one behaviour off changes the race; a lookup that takes the first of equal starts differs on
  the zero-length nodes; without a tube the seam pair does not touch; without the cap the inertia agent runs
  faster.
- **Mutation controls (8 of 8 caught):** the rank key carried from the wrong place, the carried key never
  updated, the seam never wrapped, the lookup taking the first of equal starts, the lookup ignoring the last
  node, the brake replaced by the acceleration, the speed cap ignored, and lap bookkeeping skipped when the lap
  moved.
- **Consumer (not this repository):** a Saturn racing engine ran its native field on six courses, seeds 1 to
  6, with its behaviour block on and off: 72 race reports byte-equal before and after the cuts (wall clock
  excluded). Assertions held on every tick: each skipped seam sample had non-tube flags, each exit lookup
  equalled the old walk, and the rank order equalled the old sort's.
- **Not done:** an instruction-model capture, console execution. Frame cost was measured only in the
  consumer's emulator runs (MODELLED there).
