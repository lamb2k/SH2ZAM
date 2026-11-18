# Strip rails in one call (`view-rails-1`, experimental)

Contract: [`view_experimental.h`](include/sh2zam/view_experimental.h), section
"Contract tag view-rails-1". Instruction model:
[`benchmarks/rails-model`](benchmarks/rails-model/README.md). Validation record:
[`validation/view-rails-1.md`](validation/view-rails-1.md). The centre view it
takes is view-center-1's ([VIEW_BATCH.md](VIEW_BATCH.md#fractional-centre-view-center-1)).

A strip renderer steps both edges ("rails") of a textured quad at shared
longitudinal boundaries: for each rail it copies the rail's start to the
boundaries at or below the rail's own low, its end to those at or above its
high, and steps the rest with one `sh2z_view_edge_steps_center_divu` call. On
SH-2 that call validates the view and does twelve 64-bit divisions before
its first point, about 2,000 modelled cycles per rail. `view-rails-1` folds
the whole loop, both rails and the copies, into one call, with the view
validated once per frame:

```c
typedef struct {
    sh2z_view_center_t view;
    uint32_t kernel[10];
    uint32_t check;
} sh2z_view_rails_t;                        /* 112 bytes, private fields */

bool sh2z_view_rails_prepare(sh2z_view_rails_t *out,const sh2z_view_center_t *v);
bool sh2z_view_edge_rails_center(sh2z_view_vertex_t *out_l,sh2z_view_vertex_t *out_r,
    const sh2z_view_rails_t *rails,
    const sh2z_view_vertex_t *l_from,const sh2z_view_vertex_t *l_to,
    const sh2z_view_vertex_t *r_from,const sh2z_view_vertex_t *r_to,
    const int32_t *boundary,unsigned count,
    int32_t l_low,int32_t l_high,int32_t r_low,int32_t r_high);
bool sh2z_view_edge_rails_center_divu(/* the same arguments */);
```

## Contract

For each rail (`out`, `from`, `to`, `low`, `high`): `f` is the number of
leading boundaries `<= low`; `e` is `count` minus the trailing boundaries at
or after `f` that are `>= high`. `out[j]` is `*from` for `j < f` and `*to`
for `j >= e` (all 20 bytes). The `n = e - f` entries between are **exactly**
what

```c
sh2z_view_edge_steps_center(out + f, &rails->view, from, to,
                            boundary[f] - low, boundary[f+1] - boundary[f], high - low, n)
```

writes (step 0 when `n == 1`): the same view-space points and the same `sx`,
`sy` and outcode bytes. So the error against `sh2z_view_edge_steps_center`
is 0 px, and 100% of points are equal; the Â±1 px the request allowed is not
used. A rail with `n > 0` needs `low < high`, `high - low <= 65535`,
`first < den` and, when `n >= 2`, `0 < step < den`, `first < last < den` and
`last - first == (n - 1) * step` (`last = boundary[e-1] - low`): true for
equally spaced boundaries. Only `boundary[f]`, `[f+1]` and `[e-1]` of a run
are read, so the call costs O(1) per rail beyond the scans that find `f` and
`e`, which the copies need anyway. The call returns false and writes nothing
on a null or unprepared handle, a null pointer with `count > 0`, `count >
65536`, a rail that breaks its rule, or overlapping or wrapping outputs.
`boundary` may lie inside an output; the four ends must not overlap the
outputs.

**Why exact rather than Â±1 px.** A view-space point one unit off can move
a pixel by more than one for valid views (near the near plane `focal * 1 /
z` exceeds one pixel), so a cheaper approximate stepper would need its own
depth-dependent bound. The exact stepper below costs no more per point than
the approximate one would, and its setup is a handful of multiplies.

**Why one reciprocal rather than a single DIVU below 2^15.** The request
suggested a single-division fraction path when the magnitude is below 2^15
and the general path otherwise. Every axis of every rail divides by the same
`den`, so one division per rail, `R = floor((2^32 - 1) / den)`, gives every
fraction of that rail exactly, for every magnitude below 2^32, with no
magnitude split and no second path to prove.

## Proof of exactness

The reference steps each axis with `F = floor(mag * first * 2^16 / den)`,
`S = floor(mag * step * 2^16 / den)` and `P(k) = from Â± floor((F + k*S) /
2^16)` modulo 2^32, where `mag = |to - from| < 2^32`.

1. **One multiplier per fraction.** Let `G = ceil(first * 2^64 / den)` (below
   2^64 since `first < den`). Then `mag * G / 2^48 = V + mag * e / 2^48`
   with `V = mag * first * 2^16 / den` and `0 <= e < 1`. `V`'s fractional
   part is `j / den` with `j <= den - 1`, and `mag * e / 2^48 < 2^-16 <
   1 / den`, so `floor(mag * G / 2^48) = floor(V) = F`. The 96-bit product
   `mag * (Gh * 2^32 + Gl)` takes two `DMULU.L`; bits 64..95 are
   `floor(F / 2^16)` and bits 48..63 are `F mod 2^16`. The same with `step`
   gives `S`.
2. **G without 64-bit division.** With `R = floor((2^32 - 1) / den)` and
   `rho = 2^32 - 1 - R * den`, `a * 2^32 / den = a * R + a * (rho + 1) / den`
   for `0 <= a < den`, and `a * (rho + 1) < den^2 < 2^32`. So `Gh = a * R +
   q`, with `q`, `p` the quotient and remainder of `a * (rho + 1) / den`,
   and `Gl = p * R + ceil(p * (rho + 1) / den)`. The small quotients use `R`
   again: for `n < 2^32`, `n / den - n * R / 2^32` lies in `[0, 1)`, so
   `mulhi(n, R)` is the quotient or one less, and one remainder test fixes
   it.
3. **Stepping.** The accumulator `F + k*S` is kept as a whole part `W =
   from Â± floor(./2^16)` and a fraction in the top 16 bits, so one `ADDC`
   pair per axis steps it. Towards smaller values the accumulator is held as
   `~(F + k*S)`, whose floor/2^16 is `-floor((F + k*S)/2^16) - 1`, so `W` is
   still `from - floor((F + k*S)/2^16)` and the step is the 64-bit negation
   of `(Sw, sf)`. Every step is exact modulo 2^32, as in the reference.
4. **Projection.** The stepped point goes through view-center-1's
   projection (the same C, `sh2zam_view_center_internal.h`, and on SH-2 the
   same assembly, `sh2zam_view_asm.h` with `rails=1`), so `sx`, `sy` and the
   outcode follow from the point.

## Implementation

- `sh2z_view_rails_prepare` checks the view once (`center_valid`, as every
  `_center` call does), copies it, and stores the ten camera words the SH-2
  loop reads, already in its block layout. The handle is read-only to the
  rails calls, so one handle serves both CPUs and any number of calls; a
  check word catches an unprepared handle. A handle was chosen over a
  per-call view pointer because the per-call validation alone is a large
  part of the 2,000-cycle setup it replaces, and over mutable scratch in the
  handle because the camera words cost only ten loads and stores per call.
- `sh2z_view_edge_rails_center_divu` on SH-2 is hand-written assembly
  (`sh2zam_view_rails.c`): the argument rules, both rails' scans and copies,
  one `0:0xFFFFFFFF / den` on the divider per rail (overlapped with the
  copies and with the left rail's setup; skipped for the right rail when it
  spans the same `low` and `high`), the two `G` per rail, the run states,
  and then one call of view-center-1's projection loop with the stepper in
  front of it, chained from the left run to the right one (`rails=1` in
  `sh2zam_view_asm.h`). Elsewhere the `_divu` form is the C path.
- The portable C path computes the same states in C and projects with
  view-center-1's C.
- No other object changed: every existing `saturn/src` object is
  byte-identical to main's under the CI flags (`cmp`), including
  `sh2zam_view_center.o`, whose static helpers moved to
  `sh2zam_view_center_internal.h`.

## Evidence

**HOST** (`tests/test_rails.c` against the independent 128-bit oracle in
`tests/rails_oracle.h`, written from the header texts; GCC strict, Clang
strict and GCC ASan/UBSan all pass): 7,296 hand cases (centres, 19
denominators, extreme ends), 300,000 road strips (2 to 65 boundaries, mean
5.26, on a 256, 512 or 1024 repetition length) and 300,000 general strips
(any spans, reversed spans, unsorted boundaries, unequal spacing, spans above
65,535; 119,104 rejected as the contract says): 4,242,122 stepped points and
250,753,743 checks. Every byte equals the oracle **and** the two-call loop
with `sh2z_view_edge_steps_center`, so 100% of points are exact. Argument
rules (each pointer, handle, count 0 and 65,537, overlap, `den` 65,535 and
65,536, a 65,534-point run, boundary inside an output) are checked
separately. Six mutation controls are each caught: an exact rational point
(208 records), floor instead of trunc on the way down (53,672), `from`
copied only below `low` (2,048), `to` only above `high` (4,144), the centre
fraction ignored (19,436) and the step rounded away from zero (220).

**MODELLED** (`benchmarks/rails-model`, Yabause SH-2 interpreter, flat RAM;
not Saturn timing; [results](benchmarks/rails-model/results.json) captured by
[CI run 44247175665](https://github.com/lamb2k/SH2ZAM/actions/runs/44247175665) on `769d1a4`). The model first checks every record of the assembly
entry, the C path and the loop against the oracle (128,000 calls, 25,158
rejected, 837,840 stepped points; the loop on 64,000 road strips) and the
argument rules through pointer tables (38 cases), then times them:

| Entry (road camera) | Cycles per quad (one call, both rails) | Per stepped point |
| --- | ---: | ---: |
| `sh2z_view_edge_rails_center_divu`, separate spans | 941 (471 per rail) | 130.6 |
| `sh2z_view_edge_rails_center_divu`, shared span | 867 (433 per rail) | 130.3 |
| The loop with `sh2z_view_edge_steps_center_divu` | 4,073 (2,037 per rail) | 154.5 |
| `sh2z_view_edge_rails_center` (C path) | 1,559 | 675.9 |

On 4,000 road strips the call takes 1,273 cycles per quad against the
loop's 3,536 (2.78 times faster). At a road renderer's measured edge load
(103 rail calls and 408 stepped points per frame, 809 at p99) the model's
lines give **3.55 ms** per frame against 9.53 ms for the loop (5.38 against
11.69 ms at p99), at 28.6364 MHz. The renderer measured 12.0 ms for its loop
on its own build; the model has no memory timing.

## Not met

The request's setup target was at most 150 cycles per rail. The model gives
471 per rail with separate spans: the scans and copies, the two `G` (about
80 cycles), the three axes (about 115) and the projection loop's own entry
are each near their instruction count, and no stall remains. Per point,
130.6 cycles meets the 150 target, and the frame estimate meets the 4 ms
target.

## Caller guidance

Prepare a handle whenever the centre view changes (once per frame), then
replace the per-rail loop with one call per quad:

```c
sh2z_view_rails_t rails;                         /* next to the frame's view */
if(!sh2z_view_rails_prepare(&rails,&view))return false;
...
if(!sh2z_view_edge_rails_center_divu(edge_l,edge_r,&rails,
        left_start,left_end,right_start,right_end,
        boundary,count,l_low,l_high,r_low,r_high))
    return false;                                /* den > 65535 and the like */
```

The arguments are those of the loop: the step is derived from the boundaries
instead of passed, and `den > 65535`, which the loop truncated to 16 bits,
is rejected.
