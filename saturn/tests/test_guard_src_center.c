/* view-center-1 forms of the view-guard-src-1 clip.
 * 1. Equivalence: the whole view-guard-src-1 suite (test_guard_src.c,
 *    unchanged) runs with every sh2z_view_clip_guard_src_trunc[_divu] call
 *    routed through a wrapper that also calls the _src_center form with
 *    center_x16 = 65536 * center_x and center_y16 = 65536 * center_y on the
 *    same inputs and memory; return values and bytes must match. A control
 *    moves the centre by 1/65536 and must be detected.
 * 2. Fractional centres: that suite's independent source-clipper oracle,
 *    whose planes, snap and projection take a Q16 centre, against both
 *    entry points on road quads and random polygons, hand cases at
 *    12648447 and 12648449, argument rules, and negative controls on the
 *    centre the oracle is given. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t equiv_calls,equiv_bytes,equiv_mismatches;
static int32_t equiv_perturb;   /* control: added to center_y16 */
static unsigned char equiv_before[SH2Z_VIEW_GUARD_MAX_OUT*sizeof(sh2z_view_guard_vertex_t)];
static unsigned char equiv_after[sizeof(equiv_before)];

static int equiv_src(bool divu,sh2z_view_guard_vertex_t *out,const sh2z_view_t *v,const int16_t g[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,unsigned max_out) {
    const size_t bytes=out?sizeof(equiv_before):0u;
    if(bytes) {
        /* out is often an uninitialized local of the suite: keep the copy
         * (see test_center_equiv.c). in == out restores with it. */
        __asm__ volatile("" : : "r"(out) : "memory");
        memcpy(equiv_before,out,bytes);
    }
    const int a=divu?sh2z_view_clip_guard_src_trunc_divu(out,v,g,in,count,attrs,max_out):
        sh2z_view_clip_guard_src_trunc(out,v,g,in,count,attrs,max_out);
    if(bytes){memcpy(equiv_after,out,bytes);memcpy(out,equiv_before,bytes);}
    sh2z_view_center_t w;
    memset(&w,0,sizeof(w));
    if(v) {
        w.view=*v;
        w.center_x16=(int32_t)((int64_t)v->center_x*65536);
        w.center_y16=(int32_t)((int64_t)v->center_y*65536)+equiv_perturb;
    }
    const int b=divu?sh2z_view_clip_guard_src_center_divu(out,v?&w:NULL,g,in,count,attrs,max_out):
        sh2z_view_clip_guard_src_center(out,v?&w:NULL,g,in,count,attrs,max_out);
    ++equiv_calls;
    bool same=a==b;
    if(bytes){same=same&&!memcmp(out,equiv_after,bytes);equiv_bytes+=bytes;memcpy(out,equiv_after,bytes);}
    if(!same&&!equiv_perturb)fprintf(stderr,"equivalence mismatch at paired call %llu%s\n",
        (unsigned long long)equiv_calls,divu?" (divu)":"");
    equiv_mismatches+=!same;
    return a;
}
#define sh2z_view_clip_guard_src_trunc(o,v,g,i,c,a,m) equiv_src(false,o,v,g,i,c,a,m)
#define sh2z_view_clip_guard_src_trunc_divu(o,v,g,i,c,a,m) equiv_src(true,o,v,g,i,c,a,m)
#define main guard_src_suite_main
#include "test_guard_src.c"
#undef main

/* ---- fractional centres ------------------------------------------------------ */
enum {
    C_FLOOR=1,      /* oracle given the whole-pixel centre (fraction dropped) */
    C_ROUND=2,      /* oracle given the centre rounded to the nearest pixel */
    C_Y_SIGN=4      /* oracle given the y fraction subtracted, not added */
};
typedef struct {
    uint64_t polygons,vertices,empty,overflow,snap,invalid,frac_x,frac_y,boundary,differ_trunc,bad,inplace;
} fstat_t;

static bool edge_ok(int16_t e,int32_t c16) {const wide_t d=(wide_t)e*65536-c16;return d>=-((wide_t)1<<30)&&d<=((wide_t)1<<30);}
/* A Q16 centre near pixel k. */
static int32_t pick16(int32_t k) {
    const wide_t b=(wide_t)k*65536;
    switch(random_word()%8u) {
    case 0: return (int32_t)(b+1);
    case 1: return (int32_t)(b-1);
    case 2: return (int32_t)(b+32768);
    case 3: return (int32_t)(b-32768);
    case 4: return (int32_t)b;
    case 5: return (random_word()&1u)?12648447:12648449;
    default: return (int32_t)(b+random_range(0,65535));
    }
}
static sh2z_view_center_t lift(const sh2z_view_t *v,int32_t cx16,int32_t cy16) {
    sh2z_view_center_t w;
    check(sh2z_view_center_init(&w,v,cx16,cy16),"centre view valid");
    return w;
}
/* A corner aimed at Q16 position (65536*px, 65536*py) through the Q16 centre:
 * exact when z equals the focal length. */
static sh2z_view_guard_vertex_t aim16(const sh2z_view_center_t *w,int32_t px,int32_t py,int32_t z,int nudge) {
    sh2z_view_guard_vertex_t p;
    memset(&p,0,sizeof(p));
    p.v.x=(int32_t)clampw(toward_zero(((wide_t)px*65536-w->center_x16)*z,w->view.focal_x)+nudge,INT32_MIN,INT32_MAX);
    p.v.y=(int32_t)clampw(toward_zero((w->center_y16-(wide_t)py*65536)*z,w->view.focal_y)-nudge,INT32_MIN,INT32_MAX);
    p.v.z=z;
    for(unsigned i=0;i<4;++i)p.attr[i]=(int32_t)random_word();
    return p;
}
static void aimed_polygon(const sh2z_view_center_t *w,const int16_t g[4],sh2z_view_guard_vertex_t *in,unsigned n) {
    const sh2z_view_t *v=&w->view;
    const int32_t zf=v->focal_x>=v->near_z&&v->focal_x<=v->far_z?v->focal_x:v->near_z;
    const bool flat=random_word()%2u!=0;
    for(unsigned k=0;k<n;++k) {
        const int16_t ex=g[random_word()%2u?0:2],ey=g[random_word()%2u?1:3];
        const int32_t px=random_word()%2u?ex:random_range(g[0]-500,g[2]+500);
        const int32_t py=random_word()%2u?ey:random_range(g[1]-500,g[3]+500);
        const int32_t z=flat?zf:random_word()%4u?random_depth(v):zf;
        in[k]=aim16(w,px,py,z,random_word()%4u?0:random_range(-2,2));
    }
}
/* The oracle's camera with the centre a control gives it. */
static ocam_t ocam16(const sh2z_view_center_t *w,const int16_t g[4],unsigned control) {
    ocam_t o=ocam(&w->view,g);
    o.cx=w->center_x16;o.cy=w->center_y16;
    if(control&C_FLOOR){o.cx=(wide_t)w->view.center_x*65536;o.cy=(wide_t)w->view.center_y*65536;}
    if(control&C_ROUND){o.cx=floor_wide(o.cx+32768,65536)*65536;o.cy=floor_wide(o.cy+32768,65536)*65536;}
    if(control&C_Y_SIGN){const wide_t f=o.cy-(wide_t)w->view.center_y*65536;o.cy=(wide_t)w->view.center_y*65536-f;}
    return o;
}
/* One polygon: the library against the oracle; with control 0 every other
 * property as well. Returns mismatches. */
static uint64_t frac_one(fstat_t *s,const sh2z_view_center_t *w,const int16_t g[4],const sh2z_view_guard_vertex_t *in,
    unsigned n,unsigned attrs,unsigned max_out,unsigned control) {
    sh2z_view_guard_vertex_t want[16],got[16],gd[16];
    memset(got,0x5A,sizeof(got));memset(gd,0x5A,sizeof(gd));
    const ocam_t o=ocam16(w,g,control);
    const bool ok=g[0]<=g[2]&&g[1]<=g[3]&&edge_ok(g[0],w->center_x16)&&edge_ok(g[2],w->center_x16)&&
        edge_ok(g[1],w->center_y16)&&edge_ok(g[3],w->center_y16);
    const int wn=ok?source_clip(want,&o,in,n,attrs,max_out,0):SH2Z_VIEW_GUARD_INVALID;
    const int rp=sh2z_view_clip_guard_src_center(got,w,g,in,n,attrs,max_out);
    const int rd=sh2z_view_clip_guard_src_center_divu(gd,w,g,in,n,attrs,max_out);
    uint64_t bad=rp!=wn;
    if(!bad)for(int k=0;k<wn;++k)bad+=!same_vertex(&got[k],&want[k]);
    if(control)return bad;
    ++s->polygons;s->bad+=bad;
    s->frac_x+=(w->center_x16&0xFFFF)!=0;s->frac_y+=(w->center_y16&0xFFFF)!=0;
    check(rp==rd,"centre divu count equals portable");
    if(rp>0)check(!memcmp(got,gd,(size_t)rp*sizeof(got[0])),"centre divu bytes equal portable");
    else {
        unsigned char pattern[sizeof(got)];memset(pattern,0x5A,sizeof(pattern));
        check(!memcmp(got,pattern,sizeof(got))&&!memcmp(gd,pattern,sizeof(gd)),"centre: nothing written on failure or empty");
        s->empty+=rp==0;s->overflow+=rp==SH2Z_VIEW_GUARD_OVERFLOW;s->snap+=rp==SH2Z_VIEW_GUARD_SNAP;s->invalid+=rp==SH2Z_VIEW_GUARD_INVALID;
        return bad;
    }
    s->vertices+=(uint64_t)rp;
    for(int k=0;k<rp;++k) {
        check(got[k].v.sx>=g[0]&&got[k].v.sx<=g[2]&&got[k].v.sy>=g[1]&&got[k].v.sy<=g[3],"centre output inside the guard rectangle");
        sh2z_view_vertex_t p=got[k].v;
        check(sh2z_view_project_center(&p,w,&p,1)&&p.sx==got[k].v.sx&&p.sy==got[k].v.sy&&p.outcode==got[k].v.outcode,
            "projection is sh2z_view_project_center");
        const wide_t qy=toward_zero((wide_t)w->view.focal_y*got[k].v.y,got[k].v.z);
        const wide_t qx=toward_zero((wide_t)w->view.focal_x*got[k].v.x,got[k].v.z);
        s->boundary+=((w->center_y16-qy)%65536==0)||((w->center_x16+qx)%65536==0);
    }
    sh2z_view_guard_vertex_t io[16];memset(io,0,sizeof(io));memcpy(io,in,n*sizeof(in[0]));
    check(sh2z_view_clip_guard_src_center(io,w,g,io,n,attrs,max_out)==rp&&!memcmp(io,got,(size_t)rp*sizeof(io[0])),"centre in place");
    ++s->inplace;
    /* The whole-pixel clip of the floor view, for the count of polygons the
     * fraction changes. */
    sh2z_view_guard_vertex_t t[16];
    const int rt=sh2z_view_clip_guard_src_trunc(t,&w->view,g,in,n,attrs,max_out);
    s->differ_trunc+=rt!=rp||!same_polygon(t,got,rp);
    return bad;
}
static uint64_t frac_road(fstat_t *s,unsigned trials,unsigned control) {
    uint64_t bad=0;
    for(unsigned t=0;t<trials;++t) {
        double pitch,yaw;
        const sh2z_view_t v=road_view(&pitch,&yaw);
        const int32_t cy16=t%4u==0?12648447:t%4u==1?12648449:pick16(112);
        sh2z_view_t fv=v;
        const sh2z_view_center_t w=lift(&fv,pick16(160),cy16);
        const int16_t g[4]={-900,-900,900,900};
        sh2z_view_guard_vertex_t q[4];
        unsigned tries=0;
        do road_quad(q,pitch,yaw);while(!road_unsafe(&w.view,q)&&++tries<64);
        bad+=frac_one(s,&w,g,q,4,random_word()%3u,10,control);
    }
    return bad;
}
static uint64_t frac_random(fstat_t *s,unsigned trials,unsigned control) {
    uint64_t bad=0;
    for(unsigned t=0;t<trials;++t) {
        const sh2z_view_t v=random_view();
        const sh2z_view_center_t w=lift(&v,pick16(v.center_x),pick16(v.center_y));
        int16_t g[4];random_rect(&w.view,g);
        if(g[0]<w.view.center_x-16385||g[2]>w.view.center_x+16385||g[1]<w.view.center_y-16385||g[3]>w.view.center_y+16385)continue;
        sh2z_view_guard_vertex_t in[8];
        const unsigned n=3u+random_word()%6u,attrs=random_word()%5u;
        if(random_word()%2u)aimed_polygon(&w,g,in,n);else random_polygon(&w.view,g,in,n);
        const unsigned max_out=random_word()%4u==0?(unsigned)random_range(3,16):random_word()%2u?10u:16u;
        bad+=frac_one(s,&w,g,in,n,attrs,max_out,control);
    }
    return bad;
}

static void frac_hand_cases(void) {
    const sh2z_view_t v=port_view();
    const int16_t g[4]={-900,-900,900,900};
    const int32_t z=v.focal_x;   /* quotients equal the aimed offsets */
    static const int32_t cy[2]={12648447,12648449};
    for(unsigned i=0;i<2;++i) {
        const sh2z_view_center_t w=lift(&v,160*65536,cy[i]);
        check(w.view.center_y==(i?193:192),"193 -/+ 1/65536 floors to 192 / 193");
        /* A quad across the left guard edge with a corner on the y axis
         * (qy = 0, so sy = trunc(cy16 / 65536)). */
        sh2z_view_guard_vertex_t in[4],out[16],want[16],t[16];
        in[0]=aim16(&w,100,150,z,0);in[1]=aim16(&w,300,150,z,0);in[2]=aim16(&w,300,60,z,0);in[3]=aim16(&w,-2000,60,z,0);
        in[1].v.y=0;
        const ocam_t o=ocam16(&w,g,0);
        const int n=sh2z_view_clip_guard_src_center(out,&w,g,in,4,0,10),wn=source_clip(want,&o,in,4,0,10,0);
        check(n==5&&wn==5&&same_polygon(out,want,5),"fractional centre: five corners, equal to the source");
        check(out[2].v.y==0&&out[2].v.sy==(i?193:192),"a corner at y = 0 lands on row floor(cy16 / 65536)");
        sh2z_view_t whole=v;whole.center_y=193;
        const int nt=sh2z_view_clip_guard_src_trunc(t,&whole,g,in,4,0,10);
        check(nt==5&&t[2].v.sy==193,"the whole-pixel clip at 193 puts it on row 193");
        printf("  centre y %ld: corner on the axis at row %d (whole-pixel 193: row %d), %d corners equal to the source\n",
            (long)cy[i],out[2].v.sy,t[2].v.sy,n);
    }
    /* The left plane and its snap bound move with the x fraction: corners at
     * z = focal_x make KL*z/focal_x = KL, so the left crossing is snapped to
     * x >= 65536*left - center_x16 exactly. */
    for(int f=0;f<3;++f) {
        const int32_t cx16=160*65536+(f==0?0:f==1?1:32768);
        const sh2z_view_center_t w=lift(&v,cx16,112*65536);
        sh2z_view_guard_vertex_t in[3],out[16],want[16];
        in[0]=aim16(&w,-2000,100,z,0);in[1]=aim16(&w,100,150,z,0);in[2]=aim16(&w,100,50,z,0);
        const ocam_t o=ocam16(&w,g,0);
        const int n=sh2z_view_clip_guard_src_center(out,&w,g,in,3,0,10),wn=source_clip(want,&o,in,3,0,10,0);
        check(n==wn&&n==4&&same_polygon(out,want,n),"left crossing equal to the source");
        int32_t minx=INT32_MAX;for(int k=0;k<n;++k)if(out[k].v.x<minx)minx=out[k].v.x;
        check(minx>=-900*65536-cx16&&minx<=-900*65536-cx16+2,"left crossings snapped to the Q16 plane");
    }
    printf("  left plane: crossings at x >= 65536*left - center_x16 for fractions 0, 1/65536, 1/2\n");
}
static void frac_argument_rules(void) {
    const sh2z_view_t v=port_view();
    sh2z_view_guard_vertex_t in[4],out[16],pattern[16];
    memset(in,0,sizeof(in));memset(pattern,0x3C,sizeof(pattern));
    for(unsigned k=0;k<4;++k){in[k].v.x=(int32_t)k*1000;in[k].v.y=(int32_t)(k&1u)*1000;in[k].v.z=100000;}
    sh2z_view_center_t stale=lift(&v,160*65536,12648447);stale.view.center_y=193;
    sh2z_view_center_t good=lift(&v,160*65536+1,112*65536);
    const int16_t rect[4]={-900,-900,900,900};
    const int16_t left_over[4]={160-16384,-900,900,900};    /* -2^30 - 1 from the centre */
    const int16_t right_edge[4]={-900,-900,160+16384,900};  /* 2^30 - 1 */
    struct { const sh2z_view_center_t *v; const int16_t *g; unsigned max; int want; } cases[]={
        {&stale,rect,10,SH2Z_VIEW_GUARD_INVALID},{NULL,rect,10,SH2Z_VIEW_GUARD_INVALID},{&good,left_over,10,SH2Z_VIEW_GUARD_INVALID},
        {&good,rect,2,SH2Z_VIEW_GUARD_INVALID},{&good,rect,17,SH2Z_VIEW_GUARD_INVALID}};
    for(unsigned c=0;c<sizeof(cases)/sizeof(cases[0]);++c)for(int d=0;d<2;++d) {
        memcpy(out,pattern,sizeof(out));
        const int r=d?sh2z_view_clip_guard_src_center_divu(out,cases[c].v,cases[c].g,in,4,0,cases[c].max):
            sh2z_view_clip_guard_src_center(out,cases[c].v,cases[c].g,in,4,0,cases[c].max);
        check(r==cases[c].want&&!memcmp(out,pattern,sizeof(out)),"centre: invalid arguments rejected, nothing written");
    }
    check(sh2z_view_clip_guard_src_center(NULL,&good,rect,in,4,0,10)==SH2Z_VIEW_GUARD_INVALID,"centre: null output");
    check(sh2z_view_clip_guard_src_center(out,&good,right_edge,in,4,0,16)>=0,"centre: 2^30 - 1 from the centre is fine");
    sh2z_view_center_t exact=lift(&v,160*65536,112*65536);
    check(sh2z_view_clip_guard_src_center(out,&exact,left_over,in,4,0,16)>=0,"centre: exactly 2^30 is fine");
    /* Extreme centres: the planes are exact for every int32 centre. */
    static const int32_t extremes[4][2]={{INT32_MIN,INT32_MIN},{INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MAX},{-1,0}};
    for(unsigned e=0;e<4;++e) {
        const sh2z_view_center_t w=lift(&v,extremes[e][0],extremes[e][1]);
        const int16_t g[4]={(int16_t)(w.view.center_x<0?w.view.center_x:w.view.center_x-900),(int16_t)(w.view.center_y<0?w.view.center_y:w.view.center_y-900),
            (int16_t)(w.view.center_x<0?w.view.center_x+900:w.view.center_x),(int16_t)(w.view.center_y<0?w.view.center_y+900:w.view.center_y)};
        sh2z_view_guard_vertex_t q[4],want[16];
        q[0]=aim16(&w,g[0]-50,g[1]+10,v.focal_x,0);q[1]=aim16(&w,g[2]-10,g[1]-40,v.focal_x,0);
        q[2]=aim16(&w,g[2]+70,g[3]-10,v.focal_x,0);q[3]=aim16(&w,g[0]+10,g[3]+30,v.focal_x,0);
        const ocam_t o=ocam16(&w,g,0);
        const int wn=source_clip(want,&o,q,4,0,16,0);
        const int n=sh2z_view_clip_guard_src_center(out,&w,g,q,4,0,16),nd=sh2z_view_clip_guard_src_center_divu(pattern,&w,g,q,4,0,16);
        check(n==wn&&nd==wn&&n>=3&&same_polygon(out,want,n)&&!memcmp(out,pattern,(size_t)n*sizeof(out[0])),"extreme centre equals the source");
    }
    printf("  argument rules: stale floor, null view, |65536*edge - centre| = 2^30 + 1 rejected (2^30 accepted), extreme centres exact\n");
}

int main(void) {
    if(guard_src_suite_main()!=0)return EXIT_FAILURE;
    printf("view-center-1 source-order equivalence: %llu paired calls over the whole view-guard-src-1 suite, %llu output bytes compared, %llu mismatches\n",
        (unsigned long long)equiv_calls,(unsigned long long)equiv_bytes,(unsigned long long)equiv_mismatches);
    check(equiv_mismatches==0&&equiv_calls>=1000000u,"_src_center equals _src_trunc at whole-pixel centres");
    frac_hand_cases();
    frac_argument_rules();
    fstat_t road,rnd;
    memset(&road,0,sizeof(road));memset(&rnd,0,sizeof(rnd));
    memset(&ostats,0,sizeof(ostats));
    state=UINT32_C(0x0c16ad01);
    const uint64_t rb=frac_road(&road,200000,0);
    state=UINT32_C(0x0c16a9d0);
    const uint64_t nb=frac_random(&rnd,400000,0);
    check(rb==0&&nb==0,"source-order clip with a fractional centre equals the source oracle");
    check(road.frac_y>0&&rnd.frac_x>0&&rnd.frac_y>0&&road.boundary>0&&rnd.boundary>0,"fractions and exact pixel boundaries exercised");
    check(rnd.overflow>0&&rnd.snap>0&&road.differ_trunc>0&&rnd.differ_trunc>0,"failures and fraction effects exercised");
    check(ostats.snapped>0&&ostats.snap_fail>0&&ostats.range_fail>0,"snaps and both snap failures exercised");
    const fstat_t *sets[2]={&road,&rnd};
    static const char *const names[2]={"road quads (centre y 12648447, 12648449 or near 112; +/-900; max_out 10)","random polygons"};
    for(unsigned i=0;i<2;++i) {
        const fstat_t *s=sets[i];
        printf("view-center-1 source-order %s: %llu polygons (x fraction %llu, y fraction %llu), %llu output vertices exact on both entry points "
            "(%llu on a pixel boundary), %llu empty, %llu overflow, %llu snap, %llu invalid; %llu in place; %llu differ from the whole-pixel clip of the floor view\n",
            names[i],(unsigned long long)s->polygons,(unsigned long long)s->frac_x,(unsigned long long)s->frac_y,(unsigned long long)s->vertices,
            (unsigned long long)s->boundary,(unsigned long long)s->empty,(unsigned long long)s->overflow,(unsigned long long)s->snap,
            (unsigned long long)s->invalid,(unsigned long long)s->inplace,(unsigned long long)s->differ_trunc);
    }
    static const struct { unsigned m; const char *name; } controls[]={
        {C_FLOOR,"oracle centre without its fraction"},{C_ROUND,"oracle centre rounded to the nearest pixel"},{C_Y_SIGN,"oracle y fraction subtracted"}};
    for(unsigned c=0;c<sizeof(controls)/sizeof(controls[0]);++c) {
        fstat_t m;memset(&m,0,sizeof(m));
        state=UINT32_C(0x0c16ad01);uint64_t mm=frac_road(&m,20000,controls[c].m);
        state=UINT32_C(0x0c16a9d0);mm+=frac_random(&m,40000,controls[c].m);
        printf("  control (%s): %llu mismatches\n",controls[c].name,(unsigned long long)mm);
        check(mm>0,"negative control detected");
    }
    /* Equivalence control: the centre 1/65536 lower (same floor). */
    const uint64_t calls=equiv_calls;
    equiv_perturb=1;mutation=0;
    study_t m;memset(&m,0,sizeof(m));
    state=UINT32_C(0x0ad0ad01);(void)run_road(&m,20000);
    state=UINT32_C(0x7a9d0b1e);(void)run_random(&m,40000);
    printf("  equivalence control (centre y + 1/65536): %llu of %llu paired calls differ\n",
        (unsigned long long)equiv_mismatches,(unsigned long long)(equiv_calls-calls));
    check(equiv_mismatches>0,"equivalence control detected");
    equiv_perturb=0;
    printf("view-center-1 source-order PASS (%llu checks)\n",(unsigned long long)checks);
    return 0;
}
