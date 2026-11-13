/* Original SH2ZAM diagnostic app using Yaul's public APIs.
 * SDK usage reference: yaul-org/libyaul-examples/cpu-wdt.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors.
 */
#include <yaul.h>
#include "../math_examples.h"

void user_init(void) {
    vdp2_tvmd_display_res_set(VDP2_TVMD_INTERLACE_NONE,VDP2_TVMD_HORZ_NORMAL_A,VDP2_TVMD_VERT_224);
    vdp2_scrn_back_color_set(VDP2_VRAM_ADDR(3,0x01FFFE),RGB1555(1,0,0,2));
    vdp2_tvmd_display_set();
}
void main(void) {
    cpu_intc_mask_set(0);
    dbgio_init();
    dbgio_dev_default_init(DBGIO_DEV_VDP2_ASYNC);
    dbgio_dev_font_load();
    uint32_t failures=sh2z_saturn_smoke(),frame=0;
    while(true) {
        sh2z_screen_t orbit,cube[8]={{0}}; uint8_t visible=0;
        sh2z_example_orbit(frame,&orbit);
        bool ok=sh2z_example_cube((sh2z_angle_t)(frame*128u),cube,&visible);
        dbgio_printf("\x1b[H\x1b[2JSH2ZAM " SH2Z_VERSION " Saturn smoke\nBackend: %s\nFailures: %08lx (expect 0)\nOrbit: %d %d\nCube: %s mask %02x\nVertex 0: %d %d\n",
            sh2z_backend(),(unsigned long)failures,(int)orbit.x,(int)orbit.y,ok?"OK":"FAIL",(unsigned)visible,(int)cube[0].x,(int)cube[0].y);
        dbgio_flush(); vdp2_sync(); vdp2_sync_wait(); ++frame;
    }
}
