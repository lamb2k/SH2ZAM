/* SH2ZAM / float32 workload capture. MIT; SH2ZAM Contributors.
 * FRT APIs: yaul-org/libyaul, libyaul/scu/bus/cpu/cpu/frt.h.
 * Timed kernels are the SAME bridge.c used by the WebAssembly exhibit.
 */
#include <yaul.h>
#include <string.h>
#include "../bridge.h"
#include "../fixtures.h"
#ifndef BENCH_COMMIT
#define BENCH_COMMIT "unrecorded"
#endif
#define CHUNK 32u
#define DIVISOR 128u
static struct {
    uint32_t ref[SAMPLE_COUNT],fixed[SAMPLE_COUNT],overhead[SAMPLE_COUNT];
    uint32_t checksum,max_error_lsb;
    bool invalid;
} results[CASE_COUNT];
static volatile uint32_t sink;
static void __attribute__((noinline)) empty(unsigned start,unsigned n) {
    (void)start;(void)n;__asm__ volatile("" ::: "memory");
}
/* Reset each chunk; reject ANY overflow instead of guessing wrap count.
 * Mask interrupts for both kernels. No display, conversion or checksum here.
 */
static uint32_t timed(void (*fn)(unsigned,unsigned),unsigned count,bool *invalid) {
    volatile cpu_ioregs_t *regs=(volatile cpu_ioregs_t *)CPU_IOREG_BASE;
    uint32_t ticks=0;
    cpu_intc_mask_set(15);
    for(unsigned i=0;i<count;i+=CHUNK) {
        unsigned n=count-i;if(n>CHUNK)n=CHUNK;
        cpu_frt_count_set(0);
        regs->ftcsr &= (uint8_t)~0x02;
        __asm__ volatile("" ::: "memory");
        fn(i,n);
        __asm__ volatile("" ::: "memory");
        uint16_t elapsed=cpu_frt_count_get();
        if(regs->ftcsr & 0x02) *invalid=true;
        ticks+=elapsed;
    }
    cpu_intc_mask_set(0);
    return ticks;
}
static void capture(unsigned c) {
    unsigned count=cases[c].count;
    if(count>capacity()){results[c].invalid=true;return;}
    memcpy(input_ptr(),cases[c].vertices,count*sizeof(sh2z_vec3_t));
    if(!prepare(count)){results[c].invalid=true;return;}
    for(unsigned s=0;s<SAMPLE_COUNT;s++) {
        setup(poses[s].ax,poses[s].ay,poses[s].sx,poses[s].cx,poses[s].sy,poses[s].cy);
        /* Warm both paths at each sampled pose. */
        before();after();before();after();
        results[c].overhead[s]=timed(empty,count,&results[c].invalid);
        if(s&1) {
            results[c].fixed[s]=timed(after_range,count,&results[c].invalid);
            results[c].ref[s]=timed(before_range,count,&results[c].invalid);
        } else {
            results[c].ref[s]=timed(before_range,count,&results[c].invalid);
            results[c].fixed[s]=timed(after_range,count,&results[c].invalid);
        }
        /* Consume every output outside timing; no LTO in this build. */
        for(unsigned i=0;i<count;i++) {
            sh2z_vec3_t v=after_ptr()[i];int32_t q[]={v.x,v.y,v.z};
            for(unsigned j=0;j<3;j++) {
                float f=before_ptr()[i*3+j];uint32_t bits;memcpy(&bits,&f,sizeof(bits));
                results[c].checksum=results[c].checksum*33u+bits+(uint32_t)q[j];
                double error=(double)f*65536.0-q[j];if(error<0)error=-error;
                uint32_t ceil_lsb=(uint32_t)error;if(error>ceil_lsb)ceil_lsb++;
                if(ceil_lsb>results[c].max_error_lsb)results[c].max_error_lsb=ceil_lsb;
            }
        }
        sink=results[c].checksum;
    }
}
void user_init(void) {
    vdp2_tvmd_display_res_set(VDP2_TVMD_INTERLACE_NONE,VDP2_TVMD_HORZ_NORMAL_A,VDP2_TVMD_VERT_224);
    vdp2_scrn_back_color_set(VDP2_VRAM_ADDR(3,0x01FFFE),RGB1555(1,0,0,2));
    vdp2_tvmd_display_set();
}
static void present(void){dbgio_flush();vdp2_sync();vdp2_sync_wait();}
void main(void) {
    cpu_intc_mask_set(0);dbgio_init();dbgio_dev_default_init(DBGIO_DEV_VDP2_ASYNC);dbgio_dev_font_load();
    cpu_frt_init(CPU_FRT_CLOCK_DIV_128);
    for(unsigned c=0;c<CASE_COUNT;c++) {
        dbgio_printf("\x1b[H\x1b[2JSH2ZAM hardware benchmark\nCapturing %u/15: %s detail %u\nDo not use emulator time as hardware.\n",c+1,cases[c].scene,cases[c].detail);present();
        capture(c);
    }
    unsigned page=0,frame=0;
    while(true) {
        if(frame==0) {
            dbgio_printf("\x1b[H\x1b[2JSH2ZAM capture %u/15\n%.8s / fixture %.8s\n%s d%u n%u %s\n%s FRT /128 chunk32\nchecksum %08lx err %lu LSB\nraw ref/fixed ticks (21 poses)\n",
                page+1,BENCH_COMMIT,FIXTURE_SHA,cases[page].scene,cases[page].detail,cases[page].count,
                results[page].invalid?"INVALID":"OK",sh2z_backend(),(unsigned long)results[page].checksum,(unsigned long)results[page].max_error_lsb);
            for(unsigned s=0;s<SAMPLE_COUNT;s++) {
                dbgio_printf("%02u %lu/%lu%s",s,(unsigned long)results[page].ref[s],(unsigned long)results[page].fixed[s],s%2||s==20?"\n":"  ");
            }
            dbgio_printf("overhead ticks per pose:\n");
            for(unsigned s=0;s<SAMPLE_COUNT;s++) dbgio_printf("%lu%s",(unsigned long)results[page].overhead[s],s%7==6?"\n":" ");
            dbgio_printf("\nPhoto all 15 pages; auto-advance.\nCPU cycles = ticks x 128\n");
        }
        present();if(++frame==600){frame=0;page=(page+1)%CASE_COUNT;}
    }
}
