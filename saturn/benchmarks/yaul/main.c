/* Five-kernel SH2ZAM / software-float capture. MIT; SH2ZAM Contributors.
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
#define PREPARE_ITERATIONS 64u
#define PAGE_SAMPLES 11u
#define PAGES_PER_CASE 4u
#define MAX_ERROR_LSB 66u
static struct {
    uint32_t ref[SAMPLE_COUNT],general[SAMPLE_COUNT],optimized[SAMPLE_COUNT];
    uint32_t mac[SAMPLE_COUNT],dmuls[SAMPLE_COUNT],overhead[SAMPLE_COUNT];
    uint32_t prepare[SAMPLE_COUNT],prepare_overhead[SAMPLE_COUNT];
    unsigned candidate_masks[SAMPLE_COUNT];
    uint32_t checksum,reference_checksum,general_checksum,optimized_checksum;
    uint32_t mac_checksum,dmuls_checksum,max_error_lsb,fixed_mismatches;
    bool invalid;
} results[CASE_COUNT];
static volatile uint32_t sink;

static void __attribute__((noinline)) empty(unsigned start,unsigned n) {
    (void)start;(void)n;__asm__ volatile("" ::: "memory");
}
static int __attribute__((noinline)) empty_prepare(void) {
    __asm__ volatile("" ::: "memory");return 1;
}
/* Reset each chunk and reject ANY overflow. All paths include their normal
 * range wrapper; the prepared plans are qualified once per pose separately.
 * No display, conversion, equality check or checksum is inside timing. */
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
        if(regs->ftcsr & 0x02)*invalid=true;
        ticks+=elapsed;
    }
    cpu_intc_mask_set(0);
    return ticks;
}
/* Amortize plan qualification over 64 calls so /128 FRT can resolve it. The
 * matching empty loop has the same call/result-consumption work. Publication
 * adds net preparation /64 ONCE to each whole-mesh prepared/candidate sample. */
static uint32_t timed_prepare(int (*fn)(void),bool *invalid) {
    volatile cpu_ioregs_t *regs=(volatile cpu_ioregs_t *)CPU_IOREG_BASE;
    int ok=1;
    cpu_intc_mask_set(15);
    cpu_frt_count_set(0);
    regs->ftcsr &= (uint8_t)~0x02;
    __asm__ volatile("" ::: "memory");
    for(unsigned i=0;i<PREPARE_ITERATIONS;i++)ok&=fn();
    __asm__ volatile("" ::: "memory");
    uint16_t ticks=cpu_frt_count_get();
    if((regs->ftcsr & 0x02)||!ok)*invalid=true;
    cpu_intc_mask_set(0);
    return ticks;
}

static void capture(unsigned c) {
    const unsigned count=cases[c].count;
    if(count>capacity()){results[c].invalid=true;return;}
    memcpy(input_ptr(),cases[c].vertices,count*sizeof(sh2z_vec3_t));
    if(!prepare(count)){results[c].invalid=true;return;}
    void (*const kernels[5])(unsigned,unsigned)={
        before_range,general_range,after_range,mac_range,dmuls_range
    };
    for(unsigned s=0;s<SAMPLE_COUNT;s++) {
        setup(poses[s].ax,poses[s].ay,poses[s].sx,poses[s].cx,poses[s].sy,poses[s].cy);
        /* With 21 poses and 5 kernels, every path occupies each timing
         * position four or five times; it is not perfectly balanced. */
        for(unsigned warm=0;warm<2;warm++)
            for(unsigned k=0;k<5;k++)kernels[(s+k)%5](0,count);
        if(s&1u) {
            results[c].prepare[s]=timed_prepare(plan_prepare,&results[c].invalid);
            results[c].prepare_overhead[s]=timed_prepare(empty_prepare,&results[c].invalid);
        } else {
            results[c].prepare_overhead[s]=timed_prepare(empty_prepare,&results[c].invalid);
            results[c].prepare[s]=timed_prepare(plan_prepare,&results[c].invalid);
        }
        if(results[c].prepare[s]<results[c].prepare_overhead[s])results[c].invalid=true;
        results[c].candidate_masks[s]=candidate_available();
        results[c].overhead[s]=timed(empty,count,&results[c].invalid);
        uint32_t ticks[5];
        for(unsigned k=0;k<5;k++) {
            const unsigned path=(s+k)%5;
            ticks[path]=timed(kernels[path],count,&results[c].invalid);
        }
        results[c].ref[s]=ticks[0];results[c].general[s]=ticks[1];
        results[c].optimized[s]=ticks[2];results[c].mac[s]=ticks[3];results[c].dmuls[s]=ticks[4];
        /* Consume all outputs, including candidate fallbacks, and require all
         * fixed-point components to agree. Availability is recorded separately
         * so a fallback cannot be mistaken for execution of native assembly. */
        for(unsigned i=0;i<count;i++) {
            const sh2z_vec3_t v=after_ptr()[i],g=general_ptr()[i];
            const sh2z_vec3_t a=mac_ptr()[i],b=dmuls_ptr()[i];
            const int32_t q[]={v.x,v.y,v.z},safe[]={g.x,g.y,g.z};
            const int32_t mac[]={a.x,a.y,a.z},dmuls[]={b.x,b.y,b.z};
            for(unsigned j=0;j<3;j++) {
                float f=before_ptr()[i*3+j];uint32_t bits;memcpy(&bits,&f,sizeof(bits));
                results[c].reference_checksum=results[c].reference_checksum*33u+bits;
                results[c].general_checksum=results[c].general_checksum*33u+(uint32_t)safe[j];
                results[c].optimized_checksum=results[c].optimized_checksum*33u+(uint32_t)q[j];
                results[c].mac_checksum=results[c].mac_checksum*33u+(uint32_t)mac[j];
                results[c].dmuls_checksum=results[c].dmuls_checksum*33u+(uint32_t)dmuls[j];
                results[c].checksum=results[c].checksum*33u+bits+(uint32_t)safe[j]+
                                    (uint32_t)q[j]+(uint32_t)mac[j]+(uint32_t)dmuls[j];
                if(q[j]!=safe[j]||mac[j]!=safe[j]||dmuls[j]!=safe[j]) {
                    results[c].fixed_mismatches++;results[c].invalid=true;
                }
                double error=(double)f*65536.0-q[j];if(error<0)error=-error;
                if(!(error>=0&&error<=MAX_ERROR_LSB)) {
                    results[c].invalid=true;results[c].max_error_lsb=UINT32_MAX;
                } else {
                    uint32_t ceil_lsb=(uint32_t)error;if(error>ceil_lsb)ceil_lsb++;
                    if(ceil_lsb>results[c].max_error_lsb)results[c].max_error_lsb=ceil_lsb;
                }
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
        dbgio_printf("\x1b[H\x1b[2JSH2ZAM five-kernel benchmark\nCapturing %u/%u: %s detail %u\n",c+1,CASE_COUNT,cases[c].scene,cases[c].detail);present();
        capture(c);
    }
    unsigned page=0,frame=0;
    while(true) {
        if(frame==0) {
            const unsigned c=page/PAGES_PER_CASE,group=(page%PAGES_PER_CASE)/2;
            const unsigned first=(page%2)*PAGE_SAMPLES;
            unsigned last=first+PAGE_SAMPLES;if(last>SAMPLE_COUNT)last=SAMPLE_COUNT;
            dbgio_printf("\x1b[H\x1b[2JSH2ZAM %u/%u schema3 %c\n%.8s / fixture %.8s\n%s d%u n%u %s\n%s FRT/128 chunk32\nerr %lu LSB mismatch %lu\nprepare64; A:1prep 2mac 4dmuls\n",
                page+1,CASE_COUNT*PAGES_PER_CASE,group?'B':'A',BENCH_COMMIT,FIXTURE_SHA,
                cases[c].scene,cases[c].detail,cases[c].count,results[c].invalid?"INVALID":"OK",
                sh2z_backend(),(unsigned long)results[c].max_error_lsb,(unsigned long)results[c].fixed_mismatches);
            if(group==0) {
                dbgio_printf("p    float    safe   empty prep64\n");
                for(unsigned s=first;s<last;s++)
                    dbgio_printf("%02u %7lu %7lu %7lu %5lu\n",s,(unsigned long)results[c].ref[s],
                        (unsigned long)results[c].general[s],(unsigned long)results[c].overhead[s],
                        (unsigned long)results[c].prepare[s]);
            } else {
                dbgio_printf("p prepared     mac   dmuls  pe64 A\n");
                for(unsigned s=first;s<last;s++)
                    dbgio_printf("%02u %7lu %7lu %7lu %5lu %x\n",s,(unsigned long)results[c].optimized[s],
                        (unsigned long)results[c].mac[s],(unsigned long)results[c].dmuls[s],
                        (unsigned long)results[c].prepare_overhead[s],results[c].candidate_masks[s]);
            }
            dbgio_printf("float %08lx safe %08lx\nprep  %08lx mac  %08lx\ndmuls %08lx all  %08lx\nPhoto all %u pages; auto-advance.\nprep64/pe64: setup/empty x64\n",
                (unsigned long)results[c].reference_checksum,(unsigned long)results[c].general_checksum,
                (unsigned long)results[c].optimized_checksum,(unsigned long)results[c].mac_checksum,
                (unsigned long)results[c].dmuls_checksum,(unsigned long)results[c].checksum,
                CASE_COUNT*PAGES_PER_CASE);
        }
        present();if(++frame==600){frame=0;page=(page+1)%(CASE_COUNT*PAGES_PER_CASE);}
    }
}
