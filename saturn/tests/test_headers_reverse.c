/* Every public header in one translation unit, in reverse order (test_headers.c
 * includes them in the other order). Macros that two headers share, such
 * as SH2Z_U30_ONE (unit30_experimental.h and trig12_experimental.h), must
 * not clash in either order.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/vlc_experimental.h"
#include "sh2zam/vlc2_experimental.h"
#include "sh2zam/view_experimental.h"
#include "sh2zam/unit30_experimental.h"
#include "sh2zam/trig12_experimental.h"
#include "sh2zam/spatial.h"
#include "sh2zam/shade_experimental.h"
#include "sh2zam/rotation_batch.h"
#include "sh2zam/reuse_experimental.h"
#include "sh2zam/memo_experimental.h"
#include "sh2zam/m68k.h"
#include "sh2zam/divu_experimental.h"
#include "sh2zam/contracts/pose_1.h"
#include "sh2zam/cell16.h"
#include "sh2zam/cdiv_experimental.h"
#include "sh2zam/box2.h"
#include "sh2zam/batch_experimental.h"
#include "sh2zam/sh2zam.h"
#include <stdio.h>

int main(void) {
    if(SH2Z_U30_ONE!=INT32_C(0x40000000))return 1;
    if(sh2z_sin12_quarter_u30[1024]!=SH2Z_U30_ONE)return 2;
    puts("all public headers together (reverse order) passed");
    return 0;
}
