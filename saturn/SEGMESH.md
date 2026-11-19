# Segmented strip mesh: visibility pick (segmesh-1, experimental)

`sh2z_segmesh_pick` classifies a closed ring of mesh segments against a view
and returns the visible runs furthest first, ready for painter-order drawing.
It is the first component of a segmented strip mesh renderer: later components
cover level-of-detail selection, transform, projection, clipping and VDP1 quad
emission.

## Contract (`sh2zam/segmesh_experimental.h`)

- **Input:** one int16 anchor per segment, in ring order. Four fixed-point clip
  rows (x, y, w on one scale; depth on its own), each
  `r0*x + r1*y + r2*z + r3` with `|r0..r2| <= 16383` and `|r3| <= 2^29 - 1`, so
  every row fits int32. Thresholds: `far`, `slope`, `split` and `floor`, in depth
  units.
- **Visible:** `0 <= d <= far`, `|x| <= |w|` and `|y| <= |w|`. This is the
  divide-free form of a clip-square test.
- **Direction:** a segment is marked when it or its successor is visible. Its
  direction comes from the depth step to the successor: -1 below `-slope`, +1
  above `slope`, otherwise the previous marked direction.
- **Runs:** these start after the last unmarked segment. A run extends while
  depth stays within `split` of its first segment and the direction is
  unchanged. The breaking segment ends the run and starts the next one.
- **Order:** runs are returned by decreasing average depth. Ties keep creation
  order, and runs with an average at or below `floor` are dropped. When every
  segment is marked, the result is a single whole-ring run (start == end).
- **Arithmetic:** 32-bit, plus 64-bit depth differences and run sums. No float,
  no libc and no state. The caller provides n + 1 entries of depth and state
  scratch.

## Tests (`tests/test_segmesh.c`)

- **Oracle:** an independent implementation with the reference planner's
  structure (a sentinel slot, pointer walks, int64 arithmetic, a fixed run pool,
  and repeated furthest-first selection). It is compared field by field.
- **Coverage:**
  - argument and contract rejection;
  - the full-int16 contract corner;
  - nothing visible, and everything visible with and without the floor;
  - depth steps exactly at the slope and one past it;
  - 400 camera positions around a wobbling ring;
  - 20,000 random rings and rows;
  - pool overflow, which writes nothing past `max_runs`.
- **Sanitizers:** GCC and Clang, with ASan, UBSan, `-Wconversion` and
  `-Werror -Wpedantic`.
- **Mutation controls:** changes to tie order, the slope boundary or the far
  boundary all fail.
- **Equivalent mutant:** dropping the run loop's "reached the start" stop
  survives, and that is correct. The segment just before the start is always
  unmarked, so a run breaks on the direction change before it can reach the
  start. The stop is kept as a guard.

## Cost

- **SH-2 object:** 1,376 bytes (`-m2 -mb -O2`).
- **Per segment:** one row evaluation for depth. The x, y and w rows run only
  inside the depth range.
- **Division:** libgcc's 32-bit divide runs once per run for the average. The
  64-bit divide runs only when a run's depth sum leaves int32.
