# Sprite slot cache and cell4z (sprite-cache-1, experimental)

`sh2zam/sprite_cache_experimental.h` streams pre-rendered sprite cells into a fixed pool of
VDP1 VRAM slots, so a scene with many sprite sets keeps only the cells it is drawing resident.
It has two independent parts:

- a **slot cache**: a fixed pool of slots by size class, least-recently-used replacement that
  never replaces a slot VDP1 may still read, requests a frame ahead, and per-cell flip aliases;
- **cell4z**: a 4-bit cell codec built on vlc-2 symbols, which decodes straight into a slot.

The cache does not care how a slot is filled. A caller can copy raw cells into it, run SCU DMA,
or decode cell4z streams; the measurements below are what decide.

## Slot cache

```c
static sh2z_scache_slot_t slots[96];
static sh2z_scache_t cache;
const uint32_t sizes[4] = {512, 1024, 1536, 2304}, counts[4] = {40, 24, 20, 6};
uint32_t end;
uint32_t n = sh2z_scache_layout(slots, 96, region_base, region_bytes, sizes, counts, 4, &end);
sh2z_scache_init(&cache, slots, n, 2);          /* 2: double command banks (vdp1-banks-1) */

/* every frame, master CPU */
sh2z_scache_frame(&cache);
for each sprite drawn this frame:
    uint32_t flip, cell = sh2z_scache_alias(alias_map, wanted_cell, &flip), slot;
    switch (sh2z_scache_get(&cache, key_of(set, cell), cell_bytes, false, &slot)) {
    case SH2Z_SCACHE_HIT:     emit(slots[slot].addr, flip); break;
    case SH2Z_SCACHE_MISS:    queue_fill(slot);            /* fall through */
    case SH2Z_SCACHE_PENDING: emit_previous_cell_of(sprite); break;
    case SH2Z_SCACHE_REFUSED: break;                       /* pool too small this frame */
    }
for each sprite predicted for the next frame:
    if (sh2z_scache_get(&cache, key, bytes, true, &slot) == SH2Z_SCACHE_MISS) queue_fill(slot);

/* other CPU, fill job */
for each queued slot:
    if (!sh2z_scache_live(&cache, slot)) { sh2z_scache_abandon(&cache, slot); continue; }
    fill(slots[slot].addr, cell);                          /* copy, DMA or sh2z_cell4z_decode */
    sh2z_scache_ready(&cache, slot);
```

Rules (frozen at `sprite-cache-1`):

- **Key match:** `get` returns the slot holding `key` if it is at least `bytes` large. A HIT is
  a READY slot, PENDING a slot still filling.
- **Claim:** otherwise it claims the smallest size class that fits; within it a FREE slot
  first, else the least recently used slot that is neither live nor FILLING; ties go to the
  lowest index. The claimed slot is FILLING until `ready` or `abandon`.
- **Live protection:** a slot referenced in the frame being built, or in the `live - 1` frames
  before it, is live and never replaced. Use `live = 1` with one command list and `live = 2`
  with vdp1-banks-1 double banks, where VDP1 draws the previous frame's list while the next is
  built. A prefetch (`prefetch = true`) stamps the slot for the next frame, so it is protected
  from the moment it is claimed.
- **Aliases:** `sh2z_scache_alias(map, cell, &flip)` maps a cell to the cell it is drawn from
  and the horizontal flip bit for the VDP1 command (bit 15 of `map[cell]`). Cells that are
  exact mirrors of another need no slot of their own. The table is the caller's; it is
  built offline from per-cell flip permission.
- **Counters:** `hits`, `misses` (claims), `evictions` (claims that replaced a key) and
  `refused`.
- Deterministic: results depend only on the call sequence. No allocation, no libc, no float.
  Each call scans the slots once or twice (O(slots)); with a hundred slots that is a few
  hundred cycles per call.

Synchronisation is the caller's: the master calls `get`, the fill job calls `ready` or
`abandon`. A slot's state words are written by one side at a time (FILLING belongs to the fill
job until it publishes READY), so a cache-through view of the slot array is enough.

## cell4z

A cell of `w x h` 4-bit pixels (`w` a multiple of 8, up to 504; `h` up to 255; pixel 0
transparent) is coded row by row as vlc-2 symbols: 0..15 is a pixel, 16..29 a transparent
run of 2, 3, 4, 5, 6, 8, 10, 12, 16, 20, 24, 32, 48 or 64 pixels. A run never crosses a row
end. The table for each symbol is chosen by the previous pixel of the row (0 at a row start
and after a run): 16 context tables at window 6, built from code lengths by
`sh2z_cell4z_build` into one arena (2,828 bytes for the roster measured below).
`sh2z_cell4z_decode` writes the VDP1 4-bit layout (four pixels per 16-bit word, first pixel
in the high nibble) straight into the slot, using only constant shifts, since the SH-2 has no
variable shift.

Safety: before each row the decoder checks that the row cannot read past `readable`, so a
malformed stream never reads outside its buffer. A buffer's last stream needs
`SH2Z_CELL4Z_SLACK(w, lmax)` readable bytes after it. `bits_used` returns the bits consumed,
to compare with the cell's stored length.

## Measurements

On a local roster of 105 pre-rendered vehicle sprite sets (3,080 cells, 5.2 million pixels,
about 45% transparent; the data is not in this repository):

| Fill | Bytes vs raw 4-bit | Modelled SH-2 cycles per pixel | One 1,822-pixel cell |
|---|---:|---:|---:|
| raw copy (flat memory) | 1.00 | about 2 | about 0.13 ms |
| vlc-2, one table over the raw bytes | 0.557 | 15.1 | 0.93 ms |
| **cell4z** | **0.374** (near), 0.395, 0.441 (far) | **29.1** | **1.86 ms** |

- HOST: every cell decodes bit-exact against the source at 1.98 ns per pixel.
- MODELLED: the SH-2 cycles come from an SH7604 pipeline model with flat memory, running the
  `-O2` `sh2eb-elf-gcc` code on 308 sampled cells, every output checked. The model counts
  1 cycle per instruction, the branch, load-use and multiply-latency stalls of the hardware
  manual and no wait states. Streams or tables in the cartridge are slower (vlc-2 measured
  2.2 times slower with tables there); keep the arena in work RAM.

cell4z is 33% smaller than plain vlc-2 and twice as slow. Use it where bytes count and
time does not: data on disc inflated at load, or a cartridge that is short of space. In a
30-sprite racing replay (synthetic, 3 seeds, 3,600 frames), a 90 KB pool against 254 to 310 KB
of fully resident sets misses on 4.4% of draws. Filled by raw copy, the fill job costs
0.06 ms per frame on average (p99 0.9 ms), and 0.2% of draws show the previous cell for a
frame. Decoding cell4z on every miss with a 1 ms per frame budget costs 0.58 ms per frame
on average, and 10% of draws are late.

## Validation

See [validation/sprite-cache-1.md](validation/sprite-cache-1.md). No console or emulator run.
