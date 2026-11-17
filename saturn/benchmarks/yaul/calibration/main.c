/* SH2ZAM calibration image: per-operation SH-2 costs on a real console, one
 * screen, no game data. MIT; SH2ZAM Contributors.
 *
 * Every row is timed with the free-running timer at CPU clock / 8, with all
 * interrupts masked. A measurement never spans more than one 16-bit timer
 * period without being caught: the overflow flag (FTCSR.OVF) is cleared
 * before and read after each timed span, and any overflow marks the row
 * "OVF" instead of printing a number. Each number is the median of 7 runs.
 *
 * Arithmetic rows use the difference between 512 and 256 loop iterations,
 * which cancels the call, setup and timer-read overhead; the loop's own
 * dt/bf is included, amortized over the 16 unrolled operations (the nop row
 * shows its size). Memory rows time 64 KB of sequential 32-bit loads as four
 * 16 KB spans. View rows time 512 vertices as four 128-vertex calls.
 */
#include <yaul.h>
#include <string.h>
#include "sh2zam/sh2zam.h"
#include "sh2zam/view_experimental.h"
#include "sh2zam/divu_experimental.h"
#ifndef BENCH_COMMIT
#define BENCH_COMMIT "unrecorded"
#endif

#define REPS 7u
#define FRT_DIVISOR 8u
#define OVERFLOWED UINT32_MAX

static volatile uint32_t sink;

/* ---- timing ------------------------------------------------------------- */
typedef uint32_t (*timed_fn)(uint32_t arg);
/* Ticks for one call of fn(arg), or OVERFLOWED. */
static uint32_t time_once(timed_fn fn,uint32_t arg) {
    volatile cpu_ioregs_t *regs=(volatile cpu_ioregs_t *)CPU_IOREG_BASE;
    cpu_intc_mask_set(15);
    cpu_frt_count_set(0);
    regs->ftcsr&=(uint8_t)~0x02;
    __asm__ volatile("" ::: "memory");
    const uint32_t r=fn(arg);
    __asm__ volatile("" ::: "memory");
    const uint16_t ticks=cpu_frt_count_get();
    const bool overflow=(regs->ftcsr&0x02)!=0;
    cpu_intc_mask_set(0);
    sink+=r;
    return overflow?OVERFLOWED:ticks;
}
static uint32_t median(uint32_t *v,unsigned n) {
    for(unsigned i=1;i<n;++i)for(unsigned j=i;j>0 && v[j-1]>v[j];--j){uint32_t t=v[j];v[j]=v[j-1];v[j-1]=t;}
    return v[n/2];
}
/* Median over REPS of the sum of `parts` timed calls fn(arg + k). */
static uint32_t time_parts(timed_fn fn,uint32_t arg,unsigned parts) {
    uint32_t v[REPS];
    (void)fn(arg);                                  /* warm the cache */
    for(unsigned r=0;r<REPS;++r) {
        uint32_t sum=0;
        for(unsigned k=0;k<parts;++k) {
            const uint32_t t=time_once(fn,arg+k);
            if(t==OVERFLOWED)return OVERFLOWED;
            sum+=t;
        }
        v[r]=sum;
    }
    return median(v,REPS);
}

/* ---- arithmetic loops: 16 operations per iteration ---------------------- */
#define R2(x) x x
#define R16(x) R2(R2(R2(R2(x))))

static uint32_t __attribute__((noinline)) loop_nop(uint32_t n) {
    __asm__ volatile("1:\n\t" R16("nop\n\t") "dt %0\n\tbf 1b" : "+r"(n) :: "t");
    return n;
}
static uint32_t __attribute__((noinline)) loop_dmuls(uint32_t n) {
    const int32_t a=0x12345,b=-0x6789a;
    __asm__ volatile("1:\n\t" R16("dmuls.l %1,%2\n\t") "dt %0\n\tbf 1b"
        : "+r"(n) : "r"(a),"r"(b) : "mach","macl","t");
    return n;
}
/* dmuls.l followed by a read of the result: the multiply latency. */
static uint32_t __attribute__((noinline)) loop_dmuls_use(uint32_t n) {
    const int32_t a=0x12345,b=-0x6789a;
    uint32_t t;
    __asm__ volatile("1:\n\t" R16("dmuls.l %2,%3\n\tsts mach,%1\n\t") "dt %0\n\tbf 1b"
        : "+r"(n),"=&r"(t) : "r"(a),"r"(b) : "mach","macl","t");
    return n+t;
}
static int32_t mac_a[16],mac_b[16];
static uint32_t __attribute__((noinline)) loop_mac(uint32_t n) {
    const int32_t *p,*q;
    __asm__ volatile("clrmac\n1:\n\tmov %3,%1\n\tmov %4,%2\n\t" R16("mac.l @%1+,@%2+\n\t") "dt %0\n\tbf 1b"
        : "+r"(n),"=&r"(p),"=&r"(q) : "r"(mac_a),"r"(mac_b) : "mach","macl","t","memory");
    uint32_t lo;
    __asm__ volatile("sts macl,%0" : "=r"(lo));
    return n+lo;
}

/* ---- call loops: one call per iteration over a 64-entry table ----------- */
static int32_t num32[64],den32[64];
static int64_t num64[64];
static uint32_t __attribute__((noinline)) loop_sh2z_div(uint32_t n) {
    uint32_t acc=0;
    for(uint32_t i=0;i<n;++i)acc+=(uint32_t)sh2z_div(num32[i&63u],den32[i&63u]);
    return acc;
}
static uint32_t __attribute__((noinline)) loop_divu(uint32_t n) {
    uint32_t acc=0;
    for(uint32_t i=0;i<n;++i) {
        sh2z_divu_t t;
        const int64_t v=num64[i&63u];
        sh2z_divu_start64(&t,(int32_t)(v>>32),(uint32_t)v,den32[i&63u]);
        acc+=(uint32_t)sh2z_divu_finish(&t,NULL);
    }
    return acc;
}
static uint32_t __attribute__((noinline)) loop_mul64(uint32_t n) {
    uint64_t acc=0;
    for(uint32_t i=0;i<n;++i) {
        volatile int64_t a=num64[i&63u],b=num64[(i+1)&63u];
        acc+=(uint64_t)(a*b);
    }
    return (uint32_t)acc^(uint32_t)(acc>>32);
}
static uint32_t __attribute__((noinline)) loop_div64(uint32_t n) {
    uint64_t acc=0;
    for(uint32_t i=0;i<n;++i) {
        volatile int64_t a=num64[i&63u],b=den32[i&63u];
        acc+=(uint64_t)(a/b);
    }
    return (uint32_t)acc^(uint32_t)(acc>>32);
}

/* ---- memory: 16 KB of sequential 32-bit loads per call ------------------ */
static uintptr_t mem_base;
static uint32_t __attribute__((noinline)) read16k(uint32_t part) {
    const uint32_t *p=(const uint32_t *)(mem_base+part*16384u);
    uint32_t n=16384u/64u,t;
    __asm__ volatile("1:\n\t" R16("mov.l @%1+,%2\n\t") "dt %0\n\tbf 1b"
        : "+r"(n),"+r"(p),"=&r"(t) :: "t","memory");
    return t;
}
static uint32_t sum64k(uintptr_t base) {
    const volatile uint32_t *p=(const volatile uint32_t *)base;
    uint32_t s=0;
    for(unsigned i=0;i<16384u;++i)s=s*33u+p[i];
    return s;
}
static uint32_t high_buffer[16384];

/* ---- view batch: 512 vertices as four calls of 128 ----------------------- */
static sh2z_view_t view;
static sh2z_pos16_t positions[512];
static sh2z_view_vertex_t out_portable[512],out_divu[512];
static uint32_t __attribute__((noinline)) run_view(uint32_t part) {
    const unsigned k=(part&3u)*128u;
    return sh2z_view_batch(out_portable+k,&view,positions+k,sizeof(positions[0]),128);
}
static uint32_t __attribute__((noinline)) run_view_divu(uint32_t part) {
    const unsigned k=(part&3u)*128u;
    return sh2z_view_batch_divu(out_divu+k,&view,positions+k,sizeof(positions[0]),128);
}

/* ---- fixtures ------------------------------------------------------------ */
static uint32_t rng=UINT32_C(0x9e3779b9);
static uint32_t next(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static uint32_t fnv(const void *p,size_t n) {
    const uint8_t *b=p;uint32_t h=UINT32_C(2166136261);
    while(n--)h=(h^*b++)*UINT32_C(16777619);
    return h;
}
static void fixtures(void) {
    for(unsigned i=0;i<64;++i) {
        den32[i]=(int32_t)((next()>>8)|1u)*((next()&1u)?1:-1);
        num32[i]=(int32_t)next();
        /* |num64| < |den| * 2^30: always on the divider's fast path. */
        const int64_t d=den32[i]<0?-(int64_t)den32[i]:den32[i];
        num64[i]=(int64_t)(((uint64_t)next()<<32|next())%(uint64_t)(d<<30))*((next()&1u)?1:-1);
    }
    for(unsigned i=0;i<16;++i){mac_a[i]=(int32_t)next();mac_b[i]=(int32_t)next();}
    for(unsigned i=0;i<16384u;++i)high_buffer[i]=next();
    memset(&view,0,sizeof(view));
    view.r[0][0]=view.r[1][1]=view.r[2][2]=16384;
    view.focal_x=view.focal_y=160*65536;view.center_x=160;view.center_y=112;
    view.near_z=16384/4;view.far_z=4000*16384;
    view.clip[0]=0;view.clip[1]=0;view.clip[2]=319;view.clip[3]=223;
    (void)sh2z_view_init(&view,&view);
    (void)sh2z_view_place(&view,(sh2z_vec3_t){0,0,0});
    /* Every vertex in front, inside the far plane: two divisions each. */
    for(unsigned i=0;i<512;++i) {
        const int16_t z=(int16_t)(2+next()%3000u);
        positions[i].x=(int16_t)((int32_t)(next()%(2u*z+1u))-z);
        positions[i].y=(int16_t)((int32_t)(next()%(2u*z+1u))-z);
        positions[i].z=z;
    }
}

/* Hardware divider against C division: random fitting signed divisions,
 * both signs, quotients near the limits included. Returns mismatches. */
static uint32_t check_divider(uint32_t *checked) {
    uint32_t bad=0;
    for(uint32_t i=0;i<4096u;++i) {
        int32_t d=(int32_t)next();
        if(d==0 || d==INT32_MIN)d=7;
        const uint64_t ad=d<0?(uint64_t)-(int64_t)d:(uint64_t)d;
        const unsigned shift=next()%32u;
        int64_t n=(int64_t)((((uint64_t)next()<<32)|next())%((ad<<31)-1u)>>shift);
        if(next()&1u)n=-n;
        sh2z_divu_t t;
        sh2z_divu_start64(&t,(int32_t)((uint64_t)n>>32),(uint32_t)n,d);
        bool overflow=false;
        const int32_t q=sh2z_divu_finish(&t,&overflow);
        if(overflow || t.in_software || q!=(int32_t)(n/d))++bad;
        ++*checked;
    }
    return bad;
}

/* ---- screen -------------------------------------------------------------- */
/* cycles per unit with two decimals: ticks * 8 * 100 / units. */
static void row(const char *name,uint32_t ticks,uint32_t units,const char *note) {
    if(ticks==OVERFLOWED){dbgio_printf("%-15s     OVF %s\n",name,note);return;}
    const uint32_t c100=(uint32_t)(((uint64_t)ticks*FRT_DIVISOR*100u+units/2u)/units);
    dbgio_printf("%-15s %4lu.%02lu %6lu %s\n",name,(unsigned long)(c100/100u),
        (unsigned long)(c100%100u),(unsigned long)ticks,note);
}
static uint32_t delta(timed_fn fn) {
    const uint32_t a=time_parts(fn,256,1),b=time_parts(fn,512,1);
    return a==OVERFLOWED||b==OVERFLOWED||b<a?OVERFLOWED:b-a;
}

void user_init(void) {
    vdp2_tvmd_display_res_set(VDP2_TVMD_INTERLACE_NONE,VDP2_TVMD_HORZ_NORMAL_A,VDP2_TVMD_VERT_224);
    vdp2_scrn_back_color_set(VDP2_VRAM_ADDR(3,0x01FFFE),RGB1555(1,0,2,0));
    vdp2_tvmd_display_set();
}
static void present(void){dbgio_flush();vdp2_sync();vdp2_sync_wait();}

void main(void) {
    cpu_intc_mask_set(0);dbgio_init();dbgio_dev_default_init(DBGIO_DEV_VDP2_ASYNC);dbgio_dev_font_load();
    dbgio_printf("\x1b[H\x1b[2JSH2ZAM calibration: measuring...\n");present();
    cpu_frt_init(CPU_FRT_CLOCK_DIV_8);
    dram_cart_init();
    const dram_cart_id_t cart=dram_cart_id_get();
    fixtures();

    /* Arithmetic: 256 iterations x 16 ops of difference. */
    const uint32_t t_nop=delta(loop_nop),t_dmuls=delta(loop_dmuls),t_use=delta(loop_dmuls_use),t_mac=delta(loop_mac);
    /* Calls: 256 calls of difference. */
    const uint32_t t_div=delta(loop_sh2z_div),t_divu=delta(loop_divu),t_mul64=delta(loop_mul64),t_div64=delta(loop_div64);

    /* Memory: cached addresses are area 0, cache-through area 1 (+0x20000000). */
    const uintptr_t high=(uintptr_t)high_buffer&UINT32_C(0x07ffffff),low=UINT32_C(0x00200000);
    uint32_t t_mem[6]={OVERFLOWED,OVERFLOWED,OVERFLOWED,OVERFLOWED,OVERFLOWED,OVERFLOWED};
    uint32_t sum_mem[6]={0};
    const void *const cart_area=cart!=DRAM_CART_ID_INVALID?dram_cart_area_get():NULL;
    const uintptr_t cart_base=(uintptr_t)cart_area&UINT32_C(0x07ffffff);
    const uintptr_t bases[6]={high,high|UINT32_C(0x20000000),low,low|UINT32_C(0x20000000),cart_base,cart_base|UINT32_C(0x20000000)};
    for(unsigned i=0;i<6;++i) {
        if(i>=4 && !cart_base)break;
        mem_base=bases[i];t_mem[i]=time_parts(read16k,0,4);sum_mem[i]=sum64k(bases[i]);
    }

    /* View batch. */
    const uint32_t t_view=time_parts(run_view,0,4),t_view_divu=time_parts(run_view_divu,0,4);
    const uint32_t view_sum=fnv(out_portable,sizeof(out_portable));
    const bool view_equal=!memcmp(out_portable,out_divu,sizeof(out_portable));
    uint32_t checked=0;
    const uint32_t divider_bad=check_divider(&checked);

    dbgio_printf("\x1b[H\x1b[2JSH2ZAM calibration %.8s\n",BENCH_COMMIT);
    dbgio_printf("FRT/8 ticks; cycles = ticks x 8\n");
    dbgio_printf("%-15s %7s %6s\n","row","cyc/unit","ticks");
    row("nop x16 loop",t_nop,4096,"/op");
    row("dmuls.l",t_dmuls,4096,"/op");
    row("dmuls+sts mach",t_use,4096,"/pair");
    row("mac.l @+,@+",t_mac,4096,"/op");
    row("sh2z_div DIV1",t_div,256,"/call");
    row("DIVU 64/32",t_divu,256,"/div");
    row("int64 mul gcc",t_mul64,256,"/op");
    row("int64 div gcc",t_div64,256,"/op");
    static const char *const mem_names[6]={"HWRAM cached","HWRAM through","LWRAM cached","LWRAM through","cart cached","cart through"};
    for(unsigned i=0;i<6;++i) {
        if(i>=4 && !cart_base){dbgio_printf("%-15s  no RAM cart\n",mem_names[i]);continue;}
        row(mem_names[i],t_mem[i],16384,"/word");
    }
    row("view_batch",t_view,512,"/vtx");
    row("view_batch_divu",t_view_divu,512,"/vtx");
    const bool high_same=sum_mem[0]==sum_mem[1],low_same=sum_mem[2]==sum_mem[3];
    dbgio_printf("mem sums %08lx %s %08lx %s\n",(unsigned long)sum_mem[0],high_same?"=":"!",
        (unsigned long)sum_mem[2],low_same?"=":"!");
    dbgio_printf("cart id %02x  view fnv %08lx\n",(unsigned)cart,(unsigned long)view_sum);
    dbgio_printf("divu==portable %s\n",view_equal?"YES":"NO");
    dbgio_printf("DIVU vs C: %lu/%lu bad\n",(unsigned long)divider_bad,(unsigned long)checked);
    dbgio_printf("sink %08lx\n",(unsigned long)sink);
    dbgio_printf("Photograph this screen.\n");
    present();
    for(;;)vdp2_sync_wait();
}
