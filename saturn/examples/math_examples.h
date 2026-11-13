/* Runnable example kernels: no SDK, stdio, allocation or hidden shared state.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#ifndef SH2Z_EXAMPLES_H
#define SH2Z_EXAMPLES_H
#include <sh2zam/sh2zam.h>
void sh2z_example_orbit(uint32_t frame,sh2z_screen_t *out);
bool sh2z_example_cube(sh2z_angle_t angle,sh2z_screen_t out[8],uint8_t *visible);
bool sh2z_example_batch(void);
/** Bitmask: 1 scalar, 2 tiny normalization, 4 large length, 8 dot cancellation,
 * 16 rotation, 32 projection, 64 partition/batch, 128 quaternion. Zero passes. */
uint32_t sh2z_saturn_smoke(void);
#endif
