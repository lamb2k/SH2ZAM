/* Independent oracle, edge cases and negative controls for the view-edges-1
 * edge stepper and near-plane quad clipper. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

static uint64_t checks;
static uint32_t state=UINT32_C(0xed6e0001);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"edges check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t random_range(int32_t lo,int32_t hi) {
    /* In 64 bits: lo + offset can exceed int32 before it lands back in range. */
    return (int32_t)((int64_t)lo+(int64_t)(random_word()%(uint64_t)((int64_t)hi-lo+1)));
}
static int32_t random_coord(void) {
    switch(random_word()%4u) {
    case 0: return (int32_t)random_word();                  /* full int32 */
    case 1: return random_range(-100000,100000);
    case 2: return (random_word()&1u)?INT32_MAX-random_range(0,3):INT32_MIN+random_range(0,3);
    default: return random_range(-(1<<24),1<<24);
    }
}
static wide_t trunc_div(wide_t n,wide_t d) {   /* from magnitudes */
    const wide_t a=n<0?-n:n,b=d<0?-d:d,q=a/b;
    return ((n<0)!=(d<0))?-q:q;
}
static wide_t floor_div(wide_t n,wide_t d) {wide_t q=n/d;if(n%d!=0&&((n<0)!=(d<0)))--q;return q;}
static wide_t clampw(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}

/* Projection from the batch definition, written independently. */
static sh2z_view_vertex_t oracle_project(const sh2z_view_t *v,wide_t x,wide_t y,wide_t z) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    check(x>=INT32_MIN && x<=INT32_MAX && y>=INT32_MIN && y<=INT32_MAX && z>=INT32_MIN && z<=INT32_MAX,"oracle point fits int32");
    o.x=(int32_t)x;o.y=(int32_t)y;o.z=(int32_t)z;
    if(z<v->near_z){o.outcode=SH2Z_OUT_NEAR;return o;}
    const wide_t qx=clampw(trunc_div((wide_t)v->focal_x*x,z),INT32_MIN,INT32_MAX);
    const wide_t qy=clampw(trunc_div((wide_t)v->focal_y*y,z),INT32_MIN,INT32_MAX);
    o.sx=(int16_t)clampw(v->center_x+floor_div(qx,65536),INT16_MIN,INT16_MAX);
    o.sy=(int16_t)clampw(v->center_y-floor_div(qy,65536),INT16_MIN,INT16_MAX);
    o.outcode=(uint8_t)((z>v->far_z?SH2Z_OUT_FAR:0)|(o.sx<v->clip[0]?SH2Z_OUT_LEFT:0)|(o.sx>v->clip[2]?SH2Z_OUT_RIGHT:0)|
        (o.sy<v->clip[1]?SH2Z_OUT_TOP:0)|(o.sy>v->clip[3]?SH2Z_OUT_BOTTOM:0));
    return o;
}
static sh2z_view_t random_view(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=random_range(1,1024*65536);v.focal_y=random_range(1,1024*65536);
    v.center_x=(int16_t)random_range(0,352);v.center_y=(int16_t)random_range(0,256);
    v.near_z=random_range(1,1<<20);v.far_z=v.near_z+random_range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    check(sh2z_view_init(&v,&v),"random view valid");
    return v;
}
static sh2z_view_vertex_t vertex(int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o;memset(&o,0,sizeof(o));o.x=x;o.y=y;o.z=z;return o;
}

/* ---- edge steps --------------------------------------------------------- */
enum { MUT_NONE=0, MUT_DROP_FIRST=1, MUT_CLIP_FROM_CUR=2, MUT_NEAR_EXCLUSIVE=4 };
static sh2z_view_vertex_t oracle_step(const sh2z_view_t *v,const sh2z_view_vertex_t *a,const sh2z_view_vertex_t *b,
    unsigned first,unsigned step,unsigned den,size_t k,unsigned mutation) {
    const wide_t from[3]={a->x,a->y,a->z},to[3]={b->x,b->y,b->z};
    wide_t p[3];
    for(unsigned i=0;i<3;++i) {
        const wide_t delta=to[i]-from[i];
        const wide_t f=(mutation&MUT_DROP_FIRST)?0:trunc_div(delta*first*65536,den);
        const wide_t s=trunc_div(delta*step*65536,den);
        p[i]=from[i]+trunc_div(f+(wide_t)k*s,65536);
    }
    return oracle_project(v,p[0],p[1],p[2]);
}
static sh2z_view_vertex_t got[64],want[64],got_divu[64];
static uint64_t step_sweep(unsigned mutation,uint64_t *points) {
    state=UINT32_C(0xed6e0001);
    uint64_t bad=0;
    for(unsigned t=0;t<40000u;++t) {
        const sh2z_view_t v=random_view();
        const sh2z_view_vertex_t a=vertex(random_coord(),random_coord(),random_coord());
        const sh2z_view_vertex_t b=vertex(random_coord(),random_coord(),random_coord());
        const unsigned den=(unsigned)random_range(1,65535);
        const size_t count=t%4u==0?1u:t%4u==1?64u:(size_t)random_range(1,64);
        const unsigned first=t%8u==2?0u:(unsigned)random_range(0,(int32_t)den);
        const unsigned room=den-first;
        const unsigned step=count==1?(unsigned)random_range(0,65535):t%16u==3?0u:(unsigned)random_range(0,(int32_t)(room/(count-1)));
        check(sh2z_view_edge_steps(got,&v,&a,&b,(uint16_t)first,(uint16_t)step,(uint16_t)den,count),"edge steps succeed");
        check(sh2z_view_edge_steps_divu(got_divu,&v,&a,&b,(uint16_t)first,(uint16_t)step,(uint16_t)den,count) &&
            !memcmp(got,got_divu,count*sizeof(got[0])),"divu edge steps equal edge steps");
        for(size_t k=0;k<count;++k) {
            want[k]=oracle_step(&v,&a,&b,first,step,den,k,mutation);
            if(memcmp(&got[k],&want[k],sizeof(want[k])))++bad;
            if(mutation)continue;
            /* Distance from the exact rational point on the segment. */
            const wide_t from[3]={a.x,a.y,a.z},to[3]={b.x,b.y,b.z},pt[3]={got[k].x,got[k].y,got[k].z};
            for(unsigned i=0;i<3;++i) {
                const wide_t exact_num=from[i]*den+(to[i]-from[i])*(wide_t)(first+(wide_t)k*step);
                wide_t err=pt[i]*den-exact_num;if(err<0)err=-err;
                check(err*65536<(wide_t)den*(65536+(wide_t)k+1),"point within 1 + (k+1)/65536 units of the segment");
                const wide_t lo=from[i]<to[i]?from[i]:to[i],hi=from[i]<to[i]?to[i]:from[i];
                check(pt[i]>=lo && pt[i]<=hi,"point lies between from and to");
            }
            if(points)++*points;
        }
    }
    return bad;
}
static void step_cases(void) {
    sh2z_view_t v=random_view();
    sh2z_view_vertex_t a=vertex(0,0,100000),b=vertex(64000,-64000,200000),out[8],sentinel[8];
    memset(sentinel,0xa5,sizeof(sentinel));
    /* Exact quarters with no rounding. */
    check(sh2z_view_edge_steps(out,&v,&a,&b,1,1,4,4),"quarters");
    for(unsigned k=0;k<4;++k)check(out[k].x==(int32_t)(16000*(k+1)) && out[k].y==-(int32_t)(16000*(k+1)) &&
        out[k].z==(int32_t)(100000+25000*(k+1)),"quarter points exact");
    check(sh2z_view_edge_steps(out,&v,&a,&b,0,4,4,2) && !memcmp(&out[0],&(sh2z_view_vertex_t){0},0) &&
        out[0].x==0 && out[1].x==64000 && out[1].z==200000,"first_num 0 starts at from, ends at to");
    /* Aliasing: from and to inside out. */
    out[0]=a;out[3]=b;
    check(sh2z_view_edge_steps(out,&v,&out[0],&out[3],1,1,4,4),"aliased endpoints");
    check(out[3].x==64000 && out[0].x==16000,"aliased endpoints read before writing");
    /* Argument rules: nothing written. */
    memcpy(out,sentinel,sizeof(out));
    check(sh2z_view_edge_steps(NULL,&v,NULL,NULL,0,0,0,0),"count 0 succeeds with nulls");
    check(!sh2z_view_edge_steps(out,&v,&a,&b,0,1,0,1),"den 0");
    check(!sh2z_view_edge_steps(out,&v,&a,&b,5,0,4,1),"first > den");
    check(sh2z_view_edge_steps(out,&v,&a,&b,1,1,4,4) && !sh2z_view_edge_steps(sentinel,&v,&a,&b,1,1,4,5),"bound exact / exceeded by one");
    memcpy(out,sentinel,sizeof(out));
    check(!sh2z_view_edge_steps(out,&v,&a,&b,0,1,65535,(size_t)65536+1),"bound with large count");
    check(!sh2z_view_edge_steps(out,&v,NULL,&b,0,1,4,1) && !sh2z_view_edge_steps(out,&v,&a,NULL,0,1,4,1) &&
        !sh2z_view_edge_steps(NULL,&v,&a,&b,0,1,4,1),"null pointers");
    sh2z_view_t bad=v;bad.near_z=0;
    check(!sh2z_view_edge_steps(out,&bad,&a,&b,0,1,4,1) && !sh2z_view_edge_steps(out,NULL,&a,&b,0,1,4,1),"invalid view");
    check(!sh2z_view_edge_steps_divu(out,&v,&a,&b,0,1,0,1) && !sh2z_view_edge_steps_divu(out,&v,&a,&b,1,1,4,5) &&
        !sh2z_view_edge_steps_divu(out,&bad,&a,&b,0,1,4,1),"divu argument rules");
    check(!memcmp(out,sentinel,sizeof(out)),"failed calls write nothing");
    /* step 0 repeats one point; extremes of int32 in both directions. */
    a=vertex(INT32_MIN,INT32_MAX,INT32_MIN);b=vertex(INT32_MAX,INT32_MIN,INT32_MAX);
    check(sh2z_view_edge_steps(out,&v,&a,&b,65535,0,65535,8),"step 0 at to");
    for(unsigned k=0;k<8;++k)check(out[k].x==INT32_MAX && out[k].y==INT32_MIN && out[k].z==INT32_MAX,"full span reaches to exactly");
    check(sh2z_view_edge_steps(out,&v,&b,&a,0,1,65535,2) && out[0].x==INT32_MAX && out[1].x==INT32_MAX-65537,"full span, one step back: (2^32-1)/65535 = 65537");
}

/* ---- near clip ---------------------------------------------------------- */
static unsigned oracle_clip(sh2z_view_vertex_t out[6],const sh2z_view_t *v,const sh2z_view_vertex_t in[4],unsigned mutation) {
    unsigned n=0;
    for(unsigned i=0;i<4;++i) {
        const sh2z_view_vertex_t *c=&in[i],*d=&in[(i+1)%4];
        const bool ci=(mutation&MUT_NEAR_EXCLUSIVE)?c->z>v->near_z:c->z>=v->near_z;
        const bool di=(mutation&MUT_NEAR_EXCLUSIVE)?d->z>v->near_z:d->z>=v->near_z;
        if(ci)out[n++]=*c;
        if(ci!=di) {
            const sh2z_view_vertex_t *p=ci?c:d,*q=ci?d:c;
            if(mutation&MUT_CLIP_FROM_CUR){p=c;q=d;}
            const wide_t num=(wide_t)v->near_z-p->z,den=(wide_t)q->z-p->z;
            const wide_t x=p->x+trunc_div(((wide_t)q->x-p->x)*num,den);
            const wide_t y=p->y+trunc_div(((wide_t)q->y-p->y)*num,den);
            out[n++]=oracle_project(v,x,y,v->near_z);
        }
    }
    return n;
}
static uint64_t clip_sweep(unsigned mutation,unsigned *pattern_seen) {
    state=UINT32_C(0xc11b0002);
    uint64_t bad=0;
    for(unsigned t=0;t<200000u;++t) {
        const sh2z_view_t v=random_view();
        const unsigned pattern=t%16u;
        sh2z_view_vertex_t in[4],out[6],ref[6];
        for(unsigned i=0;i<4;++i) {
            int32_t z;
            const bool inside=(pattern>>i)&1u;
            if(t%7u==0)z=inside?v.near_z:v.near_z-1;               /* on the plane / just behind */
            else if(inside)z=random_range(v.near_z,(t%5u==0)?INT32_MAX:v.near_z+(1<<22));
            else z=random_range((t%5u==0)?INT32_MIN:v.near_z-(1<<22),v.near_z-1);
            in[i]=oracle_project(&v,random_coord(),random_coord(),z);
        }
        memset(out,0x5a,sizeof(out));
        const unsigned n=sh2z_view_clip_near(out,&v,in);
        sh2z_view_vertex_t out_divu[6];
        check(sh2z_view_clip_near_divu(out_divu,&v,in)==n && !memcmp(out,out_divu,n*sizeof(out[0])),"divu clip equals clip");
        const unsigned m=oracle_clip(ref,&v,in,mutation);
        if(n!=m || memcmp(out,ref,n*sizeof(ref[0])))++bad;
        if(mutation)continue;
        const unsigned inside=(unsigned)__builtin_popcount(pattern);
        const unsigned expect=inside==0?0u:inside==4?4u:inside==1?3u:inside==3?5u:(pattern==5u||pattern==10u)?6u:4u;
        check(n==expect,"vertex count by pattern");
        for(unsigned i=n;i<6;++i)check(out[i].x==0x5a5a5a5a,"nothing written past the count");
        for(unsigned i=0;i<n;++i)check(out[i].z>=v.near_z && !(out[i].outcode&SH2Z_OUT_NEAR),"clipped polygon in front");
        if(pattern_seen)pattern_seen[pattern]++;
        /* Reversed winding gives the same crossing points. */
        sh2z_view_vertex_t rev[4]={in[3],in[2],in[1],in[0]},rout[6];
        const unsigned rn=sh2z_view_clip_near(rout,&v,rev);
        check(rn==n,"reversed quad keeps count");
        for(unsigned i=0;i<n;++i) {
            bool found=false;
            for(unsigned j=0;j<rn;++j)found|=!memcmp(&out[i],&rout[j],sizeof(out[i]));
            check(found,"every vertex appears in the reversed clip");
        }
        /* Aliasing: clip in place. */
        sh2z_view_vertex_t inplace[6];
        memcpy(inplace,in,sizeof(in));
        check(sh2z_view_clip_near(inplace,&v,inplace)==n && !memcmp(inplace,out,n*sizeof(out[0])),"in-place clip");
    }
    return bad;
}
/* ---- view-space projection ---------------------------------------------- */
static void project_cases(uint64_t *points) {
    state=UINT32_C(0x9a0e0003);
    static sh2z_view_vertex_t in[256],out[256],out_divu[256],inplace[256];
    for(unsigned t=0;t<4000u;++t) {
        const sh2z_view_t v=random_view();
        for(unsigned k=0;k<256u;++k) {
            const int32_t z=k%3u==0?random_coord():random_range(v.near_z-(1<<20)<INT32_MIN/2?INT32_MIN/2:v.near_z-(1<<20),INT32_MAX/2);
            in[k]=vertex(random_coord(),random_coord(),z);
            in[k].sx=(int16_t)random_word();in[k].sy=(int16_t)random_word();in[k].outcode=(uint8_t)random_word();
            in[k].reserved[1]=(uint8_t)random_word();     /* junk: only x y z are read */
        }
        memcpy(inplace,in,sizeof(in));
        check(sh2z_view_project(out,&v,in,256) && sh2z_view_project_divu(out_divu,&v,in,256),"project succeeds");
        check(sh2z_view_project_divu(inplace,&v,inplace,256),"project in place succeeds");
        for(unsigned k=0;k<256u;++k) {
            const sh2z_view_vertex_t w=oracle_project(&v,in[k].x,in[k].y,in[k].z);
            check(!memcmp(&out[k],&w,sizeof(w)) && !memcmp(&out_divu[k],&w,sizeof(w)) && !memcmp(&inplace[k],&w,sizeof(w)),
                "projection equals the oracle");
        }
        *points+=256;
    }
    const sh2z_view_t v=random_view();
    sh2z_view_vertex_t a[4],sentinel[4];
    memset(sentinel,0xa5,sizeof(sentinel));memcpy(a,sentinel,sizeof(a));
    check(sh2z_view_project(NULL,&v,NULL,0) && sh2z_view_project_divu(NULL,&v,NULL,0),"count 0 succeeds");
    check(!sh2z_view_project(a+1,&v,a,3) && !sh2z_view_project_divu(a,&v,a+1,3),"partial overlap rejected");
    check(!sh2z_view_project(NULL,&v,a,1) && !sh2z_view_project_divu(a,&v,NULL,1),"null pointers");
    check(!sh2z_view_project(a,NULL,a,1) && !sh2z_view_project_divu(a,NULL,a,1),"invalid view");
    check(!memcmp(a,sentinel,sizeof(a)),"failed calls write nothing");
}

static void clip_cases(void) {
    sh2z_view_t v=random_view();
    sh2z_view_vertex_t in[4],out[6],sentinel[6];
    for(unsigned i=0;i<4;++i)in[i]=vertex(0,0,v.near_z+1);
    memset(sentinel,0xa5,sizeof(sentinel));memcpy(out,sentinel,sizeof(out));
    check(sh2z_view_clip_near(out,NULL,in)==0 && sh2z_view_clip_near(NULL,&v,in)==0 && sh2z_view_clip_near(out,&v,NULL)==0,"null arguments");
    sh2z_view_t bad=v;bad.focal_x=0;
    check(sh2z_view_clip_near(out,&bad,in)==0 && sh2z_view_clip_near_divu(out,&bad,in)==0 &&
        !memcmp(out,sentinel,sizeof(out)),"invalid view writes nothing");
    /* A simple crossing with an exact answer: (0,0,near+100) to (1000,-500,near-100). */
    in[0]=vertex(0,0,v.near_z+100);in[1]=vertex(1000,-500,v.near_z-100);
    in[2]=vertex(1000,-500,v.near_z-100);in[3]=vertex(0,0,v.near_z+100);
    const unsigned n=sh2z_view_clip_near(out,&v,in);
    check(n==4 && out[1].x==500 && out[1].y==-250 && out[1].z==v.near_z,"midpoint crossing exact");
}

int main(void) {
    uint64_t points=0;
    check(step_sweep(MUT_NONE,&points)==0,"edge steps equal the oracle");
    printf("edge steps: %llu points exact, all within 1 + (k+1)/65536 units of the segment\n",(unsigned long long)points);
    step_cases();
    unsigned seen[16]={0};
    check(clip_sweep(MUT_NONE,seen)==0,"near clip equals the oracle");
    for(unsigned p=0;p<16;++p)check(seen[p]>1000u,"every in/out pattern exercised");
    printf("near clip: 200000 quads exact over all 16 patterns, winding-independent crossings\n");
    clip_cases();
    uint64_t projected=0;
    project_cases(&projected);
    printf("project: %llu view-space points exact on both entry points, in place included\n",(unsigned long long)projected);
    const struct {unsigned mutation;const char *name;int clip;} controls[3]={
        {MUT_DROP_FIRST,"edge steps without the first offset (D0)",0},
        {MUT_CLIP_FROM_CUR,"crossing computed from the walk order, not the inside end",1},
        {MUT_NEAR_EXCLUSIVE,"near plane exclusive",1}};
    for(unsigned c=0;c<3;++c) {
        const uint64_t bad=controls[c].clip?clip_sweep(controls[c].mutation,NULL):step_sweep(controls[c].mutation,NULL);
        printf("control '%s': %llu mismatches\n",controls[c].name,(unsigned long long)bad);
        check(bad>0,"mutated oracle is detected");
    }
    printf("edges checks: %llu passed\n",(unsigned long long)checks);
    return 0;
}
