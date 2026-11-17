/* view-center-1 equivalence: the whole view-round-1 test suite
 * (test_round.c, unchanged) runs with every _trunc call routed through a
 * wrapper that also calls the _center form with center_x16 = 65536 * center_x
 * and center_y16 = 65536 * center_y on the same inputs and memory, and
 * requires the same return value and byte-identical memory. That covers
 * every input of that suite: its hand cases, relation sweep, path sweeps,
 * argument rules (failures included) and the sweeps of its five negative
 * controls. A final control perturbs the centre by 1/65536 and must be
 * detected. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t equiv_calls,equiv_compared_bytes,equiv_mismatches;
static int32_t equiv_perturb;   /* control: added to center_y16 */

static sh2z_view_center_t equiv_lift(const sh2z_view_t *v) {
    sh2z_view_center_t w;
    memset(&w,0,sizeof(w));
    if(v) {
        w.view=*v;
        w.center_x16=(int32_t)((int64_t)v->center_x*65536);
        w.center_y16=(int32_t)((int64_t)v->center_y*65536)+equiv_perturb;
    }
    return w;
}
/* One output region: snapshot before, the _trunc bytes after. */
#define EQUIV_MAX_BYTES (1024u*20u)
static unsigned char equiv_before[EQUIV_MAX_BYTES],equiv_after[EQUIV_MAX_BYTES];
static size_t equiv_begin(const void *out,size_t bytes) {
    if(!out || bytes>EQUIV_MAX_BYTES)return 0;
    /* The region is often an uninitialized local of the suite. Without this
     * barrier an optimizer may treat the copy of indeterminate bytes as dead
     * and drop it (clang -O3 does), so the restore would write stale bytes.
     * The empty asm declares the region possibly written, so its bytes are
     * whatever is in memory and the copy is kept. */
    __asm__ volatile("" : : "r"(out) : "memory");
    memcpy(equiv_before,out,bytes);
    return bytes;
}
/* After the _trunc call: keep its bytes and restore the snapshot. */
static void equiv_middle(void *out,size_t bytes) {
    if(!bytes)return;
    memcpy(equiv_after,out,bytes);
    memcpy(out,equiv_before,bytes);
}
/* After the _center call: compare, then leave the _trunc bytes in place. */
static void equiv_end(void *out,size_t bytes,bool same_result,const char *fn,bool divu) {
    ++equiv_calls;
    bool same=same_result;
    if(bytes) {
        same=same&&!memcmp(out,equiv_after,bytes);
        equiv_compared_bytes+=bytes;
        memcpy(out,equiv_after,bytes);
    }
    if(!same && !equiv_perturb)fprintf(stderr,"equivalence mismatch at paired call %llu: %s%s, %s\n",
        (unsigned long long)equiv_calls,fn,divu?" (divu)":"",same_result?"output bytes":"return value");
    equiv_mismatches+=!same;
}
static size_t equiv_count_bytes(size_t count,size_t size) {
    return count>EQUIV_MAX_BYTES/size?EQUIV_MAX_BYTES+1u:count*size;
}

static bool equiv_batch(bool divu,sh2z_view_vertex_t *out,const sh2z_view_t *v,const void *p,size_t stride,size_t count) {
    const size_t n=equiv_begin(out,equiv_count_bytes(count,sizeof(*out)));
    const bool a=divu?sh2z_view_batch_trunc_divu(out,v,p,stride,count):sh2z_view_batch_trunc(out,v,p,stride,count);
    equiv_middle(out,n);
    const sh2z_view_center_t w=equiv_lift(v);
    const bool b=divu?sh2z_view_batch_center_divu(out,v?&w:NULL,p,stride,count):sh2z_view_batch_center(out,v?&w:NULL,p,stride,count);
    equiv_end(out,n,a==b,__func__,divu);
    return a;
}
static bool equiv_project(bool divu,sh2z_view_vertex_t *out,const sh2z_view_t *v,const sh2z_view_vertex_t *in,size_t count) {
    /* in == out or disjoint (otherwise rejected, nothing written): restoring
     * out restores an in-place input too. */
    const size_t n=equiv_begin(out,equiv_count_bytes(count,sizeof(*out)));
    const bool a=divu?sh2z_view_project_trunc_divu(out,v,in,count):sh2z_view_project_trunc(out,v,in,count);
    equiv_middle(out,n);
    const sh2z_view_center_t w=equiv_lift(v);
    const bool b=divu?sh2z_view_project_center_divu(out,v?&w:NULL,in,count):sh2z_view_project_center(out,v?&w:NULL,in,count);
    equiv_end(out,n,a==b,__func__,divu);
    return a;
}
static bool equiv_edges(bool divu,sh2z_view_vertex_t *out,const sh2z_view_t *v,const sh2z_view_vertex_t *from,
    const sh2z_view_vertex_t *to,uint16_t first,uint16_t step,uint16_t den,size_t count) {
    const size_t n=equiv_begin(out,equiv_count_bytes(count,sizeof(*out)));
    const bool a=divu?sh2z_view_edge_steps_trunc_divu(out,v,from,to,first,step,den,count):
        sh2z_view_edge_steps_trunc(out,v,from,to,first,step,den,count);
    equiv_middle(out,n);
    const sh2z_view_center_t w=equiv_lift(v);
    const bool b=divu?sh2z_view_edge_steps_center_divu(out,v?&w:NULL,from,to,first,step,den,count):
        sh2z_view_edge_steps_center(out,v?&w:NULL,from,to,first,step,den,count);
    equiv_end(out,n,a==b,__func__,divu);
    return a;
}
static unsigned equiv_near(bool divu,sh2z_view_vertex_t *out,const sh2z_view_t *v,const sh2z_view_vertex_t *in) {
    const size_t n=equiv_begin(out,6u*sizeof(*out));
    const unsigned a=divu?sh2z_view_clip_near_trunc_divu(out,v,in):sh2z_view_clip_near_trunc(out,v,in);
    equiv_middle(out,n);
    const sh2z_view_center_t w=equiv_lift(v);
    const unsigned b=divu?sh2z_view_clip_near_center_divu(out,v?&w:NULL,in):sh2z_view_clip_near_center(out,v?&w:NULL,in);
    equiv_end(out,n,a==b,__func__,divu);
    return a;
}
static int equiv_guard(bool divu,sh2z_view_guard_vertex_t *out,const sh2z_view_t *v,const int16_t g[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs) {
    const size_t n=equiv_begin(out,SH2Z_VIEW_GUARD_MAX_OUT*sizeof(*out));
    const int a=divu?sh2z_view_clip_guard_trunc_divu(out,v,g,in,count,attrs):sh2z_view_clip_guard_trunc(out,v,g,in,count,attrs);
    equiv_middle(out,n);
    const sh2z_view_center_t w=equiv_lift(v);
    const int b=divu?sh2z_view_clip_guard_center_divu(out,v?&w:NULL,g,in,count,attrs):
        sh2z_view_clip_guard_center(out,v?&w:NULL,g,in,count,attrs);
    equiv_end(out,n,a==b,__func__,divu);
    return a;
}

#define sh2z_view_batch_trunc(o,v,p,s,c) equiv_batch(false,o,v,p,s,c)
#define sh2z_view_batch_trunc_divu(o,v,p,s,c) equiv_batch(true,o,v,p,s,c)
#define sh2z_view_project_trunc(o,v,i,c) equiv_project(false,o,v,i,c)
#define sh2z_view_project_trunc_divu(o,v,i,c) equiv_project(true,o,v,i,c)
#define sh2z_view_edge_steps_trunc(o,v,f,t,a,b,d,c) equiv_edges(false,o,v,f,t,a,b,d,c)
#define sh2z_view_edge_steps_trunc_divu(o,v,f,t,a,b,d,c) equiv_edges(true,o,v,f,t,a,b,d,c)
#define sh2z_view_clip_near_trunc(o,v,i) equiv_near(false,o,v,i)
#define sh2z_view_clip_near_trunc_divu(o,v,i) equiv_near(true,o,v,i)
#define sh2z_view_clip_guard_trunc(o,v,g,i,c,a) equiv_guard(false,o,v,g,i,c,a)
#define sh2z_view_clip_guard_trunc_divu(o,v,g,i,c,a) equiv_guard(true,o,v,g,i,c,a)
#define main round_suite_main
#include "test_round.c"
#undef main

int main(void) {
    if(round_suite_main()!=0)return EXIT_FAILURE;
    printf("view-center-1 equivalence: %llu paired calls over the whole view-round-1 suite, %llu output bytes compared, %llu mismatches\n",
        (unsigned long long)equiv_calls,(unsigned long long)equiv_compared_bytes,(unsigned long long)equiv_mismatches);
    if(equiv_mismatches||equiv_calls<10000u){fprintf(stderr,"view-center-1 equivalence FAILED\n");return EXIT_FAILURE;}
    /* Control: the centre one unit (1/65536 px) lower on screen, which keeps
     * the same floor (so the view stays valid), must be detected. */
    const uint64_t calls=equiv_calls;
    equiv_perturb=1;mutation=0;state=UINT32_C(0x70c0ffee);
    sweep_t s;memset(&s,0,sizeof(s));
    sweep(&s);
    printf("  control (centre y + 1/65536): %llu of %llu paired calls differ\n",
        (unsigned long long)equiv_mismatches,(unsigned long long)(equiv_calls-calls));
    if(!equiv_mismatches){fprintf(stderr,"view-center-1 equivalence control not detected\n");return EXIT_FAILURE;}
    printf("view-center-1 equivalence PASS\n");
    return 0;
}
