/* Track-bound vehicle dynamics with tube shapes (contract tag
 * vehicle-dynamics-2): sh2z_vd_step2. The tick is vehicle-dynamics-1's,
 * built a second time with the pipe and cylinder shapes; the search, spline
 * and line-space view come from sh2zam_vehicle_dynamics.c.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#define SH2Z_VD_TUBES 1
#include "sh2zam_vehicle_dynamics.c"
