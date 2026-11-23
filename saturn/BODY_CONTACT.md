# Body contact (experimental, contract tags `body-contact-1`, `body-contact-2` and `vehicle-dynamics-3`)

[`body_contact_experimental.h`](include/sh2zam/body_contact_experimental.h) resolves contact
between bodies that touch as spheres of one reach. It is meant for racing games where vehicles
bump, shove and attack each other: the caller supplies each body's position, velocity, weight
and state, and gets back the push, the damage and who touched whom. What damage does, who may be
knocked out and who earns credit stay with the caller.

[`vehicle_dynamics_experimental.h`](include/sh2zam/vehicle_dynamics_experimental.h) gains
`sh2z_vd_step_contact` (`vehicle-dynamics-3`), which feeds one tick's contact into the
track-bound vehicle step at the place a contact force belongs.

Nothing selects either by default. The tags are experimental: their numeric definitions may
change until they are published.

## The rule

For a pair at separation `d <= reach`, with relative velocity `dv`, unit separation `n` and
weights `wA`, `wB` (a `WEAK` body counts `weak_share` of its strength), shares
`sA = wA / (wA + wB)` and `sB = 1 - sA`:

| When | What |
| --- | --- |
| First tick in reach | Impact `I = impact_gain * |n . dv|`. No attacker or two: A takes `sB * I`, B takes `sA * I`. One attacker: only the other takes `I`, times `heavy_gain` for a `HEAVY` attacker. A `CREDIT` body becomes the other's `toucher`. |
| Every tick in reach while closing | Push `P = (reach - d) n`, at least `attack_min` long with an attacker in the pair. A's acceleration gets `-sB P`, B's `+sA P`. An attacker against a non-attacker gets nothing; its victim gets `attack_push P` and the `hit` mark. |
| Separation below `near` | `n` is the unit of `dv` and the push is `-reach n`. |

Each body takes the other's share, so the lighter body moves more and is hurt more.

## Using it

```c
sh2z_bc_body_t body[N];
uint32_t last[N * (N - 1) / 2];
sh2z_bc_field_t field;
sh2z_bc_field_init(&field, body, last, N);

/* each tick */
sh2z_bc_field_begin(&field);                  /* clears outputs, ages every since counter */
/* fill pos, vel, strength, flags */
sh2z_bc_field_resolve(&field, order, along, length, &rules);
/* apply body[i].push, .damage, .hit; on a knock-out: */
int32_t who = sh2z_bc_credit(&field, victim, &rules);
```

`order` lists the bodies by `along` (distance along the course) descending. The walk tests only
pairs within `reach_long` along the line, each once, from the body ahead, so a 30-body field costs
about one comparison per body plus the pairs in reach. `sh2z_bc_pair` resolves one pair for callers
that keep their own history.

For a vehicle driven by `vehicle-dynamics-1`, pass the body's outputs to `sh2z_vd_step_contact`:

```c
sh2z_vd_contact_t c = { { b->push[0], b->push[1], b->push[2] }, b->damage, regain, b->hit };
sh2z_vd_step_contact(&vehicle, &track, &constants, &rules, &pad, &c);
```

On a track with tube shapes (`vehicle-dynamics-2`), `sh2z_vd_step2_contact` takes the same contact
with the `sh2z_vd_track2_t`; on road shapes it gives `sh2z_vd_step_contact`'s bits.

The contact lands after the tick's resets and before the pad is read: the push is the tick's first
acceleration (integrated with the vehicle's own forces), damage marks the tick (no grip) and, past
zero energy, starts the spin-out before the pad is read; `hit` takes grip away for
`rules.contact_grip` ticks; `regain` comes first, up to the maximum, and not while spinning out.

## The sweep and the pair list (`body-contact-2`)

A caller that wants less than the full walk each tick, because most bodies are far from any other or because a body's velocity costs
something to compute, builds the pair list itself in three steps. Nothing in `body-contact-1` changes; `sh2z_bc_field_resolve` also tests
the position box in 32 bits before calling the pair function, with the same results.

```c
uint32_t np = sh2z_bc_sweep(order, along, across, live, N, length, rules.reach_long, window_across, pa, pb, marked);
/* marked[i]: body i is in a listed pair, so it needs a position; fill pos for those, flags ACTIVE */
uint32_t nq = sh2z_bc_range(&field, pa, pb, np, &rules, qa, qb);   /* pairs of ACTIVE bodies within the reach on every axis */
/* bodies in qa/qb need their velocity now; nobody else does */
uint32_t touching = sh2z_bc_field_resolve_list(&field, qa, qb, nq, &rules);
```

- `sh2z_bc_sweep` visits pairs in the order `sh2z_bc_field_resolve` does (each pair once, from the body ahead, ties before the wrap) and
  lists those whose bodies are both `live` and whose `across` difference is at most `window_across`. Choosing the window is the caller's
  claim: it must cover every pair that can be in reach.
- `sh2z_bc_range` drops pairs without two ACTIVE bodies, counts the rest in `field->tested`, and keeps those whose positions are within the
  reach on every axis. The 32-bit test never rejects a pair that `sh2z_bc_pair` accepts; positions 2^32 apart in a coordinate pass it and
  `sh2z_bc_pair` rejects them with no effect.
- `sh2z_bc_field_resolve_list` calls the pair function for each listed pair with the field's history, as `sh2z_bc_field_resolve` does. With
  a window that covers every pair in reach, sweep, range and list give the field walk's outputs, history and counters bit for bit.

## Status

- **Integer only, host and SH-2 agree.** 64-bit products rounded to nearest, quotients from
  `sh2z_divu_div64`, lengths from `sh2z_isqrt64_q12`, unit vectors from `sh2z_vec3_normalize_u30`.
- **Error bound** against the exact rule on the same inputs: each damage and push component within
  2^-8 (256 Q16 LSB) for separations from `near` to `reach`, speeds below 2^10 per tick and shares
  above 2^-12. The decisions are exact on the integer inputs.
- **`sh2z_vd_step` is unchanged:** `sh2zam_vehicle_contact.c` builds the vehicle-dynamics source a
  second time with its contact hook set, so the step's model has one definition and the
  vehicle-dynamics-1 object is byte-identical. `sh2z_vd_step_contact` keeps the contact of the tick
  in progress in a file static: one tick at a time per CPU.
- Evidence tiers: HOST, plus the SH-2 build. See the validation records.
