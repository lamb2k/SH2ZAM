/* Exact wide-oracle coverage for transform and collision arithmetic. */
#include "sh2zam/sh2zam.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined(__SIZEOF_INT128__)
#error "This host correctness test requires a 128-bit integer oracle."
#endif
__extension__ typedef __int128 wide_t;

static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) { \
    fprintf(stderr,"%s:%u: %s\n",__FILE__,(unsigned)__LINE__,#x); exit(1); \
} } while(0)

static uint32_t state=UINT32_C(0xa72c4f19);
static uint32_t random_u32(void) {
    state^=state<<13; state^=state>>17; state^=state<<5; return state;
}
static int32_t signed_bits(uint32_t v) {
    return v<=INT32_MAX?(int32_t)v:INT32_MIN+(int32_t)(v-UINT32_C(0x80000000));
}
static int32_t component(unsigned bounded) {
    uint32_t value=random_u32();
    return bounded?(int32_t)(value&UINT32_C(0x7ffff))-INT32_C(0x40000):signed_bits(value);
}
static wide_t oracle_dot_wide(const int32_t *a,const int32_t *b,unsigned n,unsigned minus) {
    wide_t sum=0;
    for(unsigned i=0;i<n;i++) {
        wide_t p=(wide_t)a[i]*b[i];
        sum+=(minus&(1u<<i))?-p:p;
    }
    wide_t value=sum/65536;
    if(sum<0&&sum%65536) --value;
    return value;
}
static int32_t oracle_dot(const int32_t *a,const int32_t *b,unsigned n,unsigned minus,int32_t offset) {
    wide_t value=oracle_dot_wide(a,b,n,minus)+offset;
    return value>INT32_MAX?INT32_MAX:value<INT32_MIN?INT32_MIN:(int32_t)value;
}

static void check_case(int32_t a[4][4],int32_t b[4][4],const int32_t v[4]) {
    sh2z_mat22_t a22,b22,w22,r22;
    sh2z_mat33_t a33,b33,w33,r33;
    sh2z_mat34_t a34,b34,w34,r34;
    sh2z_mat44_t a44,b44,w44,r44;
    for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
        a44.m[row][col]=a[row][col]; b44.m[row][col]=b[row][col];
        if(row<3) { a34.m[row][col]=a[row][col]; b34.m[row][col]=b[row][col]; }
        if(row<3&&col<3) { a33.m[row][col]=a[row][col]; b33.m[row][col]=b[row][col]; }
        if(row<2&&col<2) { a22.m[row][col]=a[row][col]; b22.m[row][col]=b[row][col]; }
        const int32_t column[]={b[0][col],b[1][col],b[2][col],b[3][col]};
        w44.m[row][col]=oracle_dot(a[row],column,4,0,0);
        if(row<3) w34.m[row][col]=oracle_dot(a[row],column,3,0,col==3?a[row][3]:0);
        if(row<3&&col<3) w33.m[row][col]=oracle_dot(a[row],column,3,0,0);
        if(row<2&&col<2) w22.m[row][col]=oracle_dot(a[row],column,2,0,0);
    }

    /* Independent output plus both legal alias directions use the same oracle. */
    for(unsigned alias=0;alias<3;alias++) {
        r22=alias==2?b22:a22;
        r33=alias==2?b33:a33;
        r34=alias==2?b34:a34;
        r44=alias==2?b44:a44;
        sh2z_mat22_multiply(&r22,alias==1?&r22:&a22,alias==2?&r22:&b22);
        sh2z_mat33_multiply(&r33,alias==1?&r33:&a33,alias==2?&r33:&b33);
        sh2z_mat34_multiply(&r34,alias==1?&r34:&a34,alias==2?&r34:&b34);
        sh2z_mat44_multiply(&r44,alias==1?&r44:&a44,alias==2?&r44:&b44);
        for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
            CHECK(r44.m[row][col]==w44.m[row][col]);
            if(row<3) CHECK(r34.m[row][col]==w34.m[row][col]);
            if(row<3&&col<3) CHECK(r33.m[row][col]==w33.m[row][col]);
            if(row<2&&col<2) CHECK(r22.m[row][col]==w22.m[row][col]);
        }
    }

    sh2z_vec2_t r2=sh2z_mat22_transform(&a22,(sh2z_vec2_t){v[0],v[1]});
    CHECK(r2.x==oracle_dot(a[0],v,2,0,0)); CHECK(r2.y==oracle_dot(a[1],v,2,0,0));
    sh2z_vec3_t input={v[0],v[1],v[2]};
    sh2z_vec3_t r3=sh2z_mat33_transform(&a33,input);
    CHECK(r3.x==oracle_dot(a[0],v,3,0,0)); CHECK(r3.y==oracle_dot(a[1],v,3,0,0)); CHECK(r3.z==oracle_dot(a[2],v,3,0,0));
    r3=sh2z_mat34_transform_direction(&a34,input);
    CHECK(r3.x==oracle_dot(a[0],v,3,0,0)); CHECK(r3.y==oracle_dot(a[1],v,3,0,0)); CHECK(r3.z==oracle_dot(a[2],v,3,0,0));
    r3=sh2z_mat34_transform_point(&a34,input);
    CHECK(r3.x==oracle_dot(a[0],v,3,0,a[0][3])); CHECK(r3.y==oracle_dot(a[1],v,3,0,a[1][3])); CHECK(r3.z==oracle_dot(a[2],v,3,0,a[2][3]));
    sh2z_vec4_t r4=sh2z_mat44_transform(&a44,(sh2z_vec4_t){v[0],v[1],v[2],v[3]});
    CHECK(r4.x==oracle_dot(a[0],v,4,0,0)); CHECK(r4.y==oracle_dot(a[1],v,4,0,0));
    CHECK(r4.z==oracle_dot(a[2],v,4,0,0)); CHECK(r4.w==oracle_dot(a[3],v,4,0,0));

    const sh2z_quat_t qa={a[0][1],a[0][2],a[0][3],a[0][0]};
    const sh2z_quat_t qb={v[0],v[1],v[2],v[3]};
    const int32_t bx[]={qb.x,qb.w,qb.z,qb.y},by[]={qb.y,qb.z,qb.w,qb.x};
    const int32_t bz[]={qb.z,qb.y,qb.x,qb.w},bw[]={qb.w,qb.x,qb.y,qb.z};
    sh2z_quat_t q=sh2z_quat_multiply(qa,qb);
    CHECK(q.x==oracle_dot(a[0],bx,4,8,0)); CHECK(q.y==oracle_dot(a[0],by,4,2,0));
    CHECK(q.z==oracle_dot(a[0],bz,4,4,0)); CHECK(q.w==oracle_dot(a[0],bw,4,14,0));

    /* The existing batch API's fallback calls the optimized scalar entry. */
    sh2z_vec3_t batch=input;
    CHECK(sh2z_mat34_transform_points_general(&batch,&a34,&batch,1));
    CHECK(batch.x==r3.x&&batch.y==r3.y&&batch.z==r3.z);
    batch=input;
    CHECK(sh2z_mat34_transform_points(&batch,&a34,&batch,1));
    CHECK(batch.x==r3.x&&batch.y==r3.y&&batch.z==r3.z);
}

static void transform_cases(void) {
    static const int32_t edges[]={0,1,-1,65535,-65535,65536,-65536,65537,-65537,INT32_MIN,INT32_MAX,INT32_MIN+1,INT32_MAX-1};
    int32_t a[4][4],b[4][4],v[4];
    for(unsigned e=0;e<sizeof(edges)/sizeof(edges[0]);e++) {
        for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
            a[row][col]=edges[(e+row+col)%13]; b[row][col]=edges[(e+2*row+3*col)%13];
        }
        for(unsigned i=0;i<4;i++) v[i]=edges[(e+i)%13];
        check_case(a,b,v);
    }
    /* Rows immediately on both sides of the proof boundary, plus full-width
     * cancellation. The zero/small terms expose rounding before saturation. */
    static const int32_t limits[][4]={
        {INT32_MIN,INT32_MAX,0,0},{INT32_MIN,INT32_MIN,0,0},
        {INT32_MAX,INT32_MAX,1,0},{INT32_MAX,INT32_MAX,2,0},
        {INT32_MAX,INT32_MAX,0,1},{INT32_MAX,INT32_MAX,0,2},
        {INT32_MIN,INT32_MIN,INT32_MAX,INT32_MAX},
        {INT32_MIN,INT32_MAX,INT32_MIN,INT32_MAX}
    };
    for(unsigned i=0;i<sizeof(limits)/sizeof(limits[0]);i++) {
        for(unsigned j=0;j<sizeof(limits)/sizeof(limits[0]);j++) {
            for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
                a[row][col]=limits[i][col]; b[row][col]=limits[j][row];
            }
            for(unsigned k=0;k<4;k++) v[k]=limits[j][k];
            check_case(a,b,v);
        }
    }
    /* Both bounded, both unrestricted, and one bounded side each. */
    for(unsigned mode=0;mode<4;mode++) for(unsigned sample=0;sample<4096;sample++) {
        for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
            a[row][col]=component(mode&1); b[row][col]=component(mode&2);
        }
        for(unsigned k=0;k<4;k++) v[k]=component(mode&2);
        check_case(a,b,v);
    }
}

static bool oracle_spheres(sh2z_sphere_t a,sh2z_sphere_t b) {
    if(a.radius<0||b.radius<0) return false;
    wide_t x=(int64_t)a.center.x-b.center.x,y=(int64_t)a.center.y-b.center.y,z=(int64_t)a.center.z-b.center.z;
    wide_t radius=(int64_t)a.radius+b.radius;
    return x*x+y*y+z*z<=radius*radius;
}

static bool oracle_inverse(sh2z_mat34_t *out,const sh2z_mat34_t *m) {
    /* Deliberately check all nine ordered row pairs, independently of the
     * optimized implementation's symmetry reduction. */
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<3;col++) {
        wide_t d=oracle_dot_wide(m->m[row],m->m[col],3,0)-(row==col?SH2Z_ONE:0);
        if(d>32||d< -32) return false;
    }
    sh2z_mat34_t result;
    const int32_t t[]={m->m[0][3],m->m[1][3],m->m[2][3]};
    for(unsigned row=0;row<3;row++) {
        for(unsigned col=0;col<3;col++) result.m[row][col]=m->m[col][row];
        wide_t value=-oracle_dot_wide(result.m[row],t,3,0);
        if(value<INT32_MIN||value>INT32_MAX) return false;
        result.m[row][3]=(int32_t)value;
    }
    *out=result; return true;
}
static void check_inverse(sh2z_mat34_t m) {
    sh2z_mat34_t want,actual;
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++)
        want.m[row][col]=(int32_t)(0x12345600u+row*4+col);
    actual=want;
    bool expected=oracle_inverse(&want,&m),result=sh2z_mat34_inverse_rigid(&actual,&m);
    CHECK(expected==result);
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) CHECK(actual.m[row][col]==want.m[row][col]);
    want=m; actual=m;
    expected=oracle_inverse(&want,&m); result=sh2z_mat34_inverse_rigid(&actual,&actual);
    CHECK(expected==result);
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) CHECK(actual.m[row][col]==want.m[row][col]);
}
static void inverse_cases(void) {
    static const int32_t tolerance_edges[]={-33,-32,-17,-16,-1,0,1,16,17,32,33};
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<3;col++)
        for(unsigned i=0;i<sizeof(tolerance_edges)/sizeof(tolerance_edges[0]);i++) {
            sh2z_mat34_t m=sh2z_mat34_identity();
            m.m[row][col]+=tolerance_edges[i]; check_inverse(m);
        }
    static const int32_t translations[]={INT32_MIN,INT32_MIN+1,-1,0,1,INT32_MAX};
    for(unsigned angle=0;angle<65536;angle+=1024) {
        sh2z_mat34_t m=sh2z_mat34_rotation_z((sh2z_angle_t)angle);
        for(unsigned i=0;i<sizeof(translations)/sizeof(translations[0]);i++) {
            m.m[0][3]=translations[i]; m.m[1][3]=translations[(i+2)%6]; m.m[2][3]=translations[(i+4)%6];
            check_inverse(m);
        }
    }
    for(unsigned i=0;i<4096;i++) {
        sh2z_mat34_t m;
        for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) m.m[row][col]=component(i&1);
        check_inverse(m);
    }
    sh2z_mat34_t sentinel=sh2z_mat34_identity(),out=sentinel;
    CHECK(!sh2z_mat34_inverse_rigid(NULL,&sentinel));
    CHECK(!sh2z_mat34_inverse_rigid(&out,NULL));
    for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) CHECK(out.m[row][col]==sentinel.m[row][col]);
}
static int32_t clamp_component(int32_t x,int32_t low,int32_t high) { return x<low?low:x>high?high:x; }
static bool oracle_box(sh2z_sphere_t s,sh2z_aabb_t b) {
    if(s.radius<0||b.min.x>b.max.x||b.min.y>b.max.y||b.min.z>b.max.z) return false;
    sh2z_sphere_t nearest={{clamp_component(s.center.x,b.min.x,b.max.x),
        clamp_component(s.center.y,b.min.y,b.max.y),clamp_component(s.center.z,b.min.z,b.max.z)},0};
    return oracle_spheres(s,nearest);
}
static void collision_cases(void) {
    static const int32_t coordinates[]={INT32_MIN,INT32_MIN+1,-65536,-1,0,1,65536,INT32_MAX-1,INT32_MAX};
    for(unsigned i=0;i<9;i++) for(unsigned j=0;j<9;j++) for(unsigned k=0;k<9;k++) {
        sh2z_sphere_t a={{coordinates[i],coordinates[j],coordinates[k]},INT32_MAX};
        sh2z_sphere_t b={{coordinates[8-i],coordinates[8-j],coordinates[8-k]},INT32_MAX};
        CHECK(sh2z_sphere_intersects(a,b)==oracle_spheres(a,b));
        b.radius=0;
        CHECK(sh2z_sphere_intersects(a,b)==oracle_spheres(a,b));
        sh2z_aabb_t box={{INT32_MIN,-1,0},{-1,1,INT32_MAX}};
        CHECK(sh2z_sphere_intersects_aabb(a,box)==oracle_box(a,box));
        a.radius=0;
        CHECK(sh2z_sphere_intersects_aabb(a,box)==oracle_box(a,box));
    }
    for(unsigned sample=0;sample<20000;sample++) {
        unsigned bounded=sample&1;
        sh2z_sphere_t a={{component(bounded),component(bounded),component(bounded)},component(bounded)};
        sh2z_sphere_t b={{component(bounded),component(bounded),component(bounded)},component(bounded)};
        if(sample&2) { a.radius=(int32_t)((uint32_t)a.radius&INT32_MAX); b.radius=(int32_t)((uint32_t)b.radius&INT32_MAX); }
        CHECK(sh2z_sphere_intersects(a,b)==oracle_spheres(a,b));
        sh2z_aabb_t box=sh2z_aabb(a.center,b.center);
        sh2z_sphere_t s={{component(bounded),component(bounded),component(bounded)},a.radius};
        CHECK(sh2z_sphere_intersects_aabb(s,box)==oracle_box(s,box));
        box.min.x=1; box.max.x=-1;
        CHECK(!sh2z_sphere_intersects_aabb(s,box));
    }
}

static bool oracle_project(sh2z_screen_t *out,sh2z_vec4_t v,uint16_t width,uint16_t height) {
    if(!width||!height||width>32767||height>32767||v.w<=0) return false;
    if(v.x< -v.w||v.x>v.w||v.y< -v.w||v.y>v.w||v.z< -v.w||v.z>v.w) return false;
    int64_t denominator=2*(int64_t)v.w;
    out->x=(int16_t)((((int64_t)v.x+v.w)*(width-1))/denominator);
    out->y=(int16_t)((((int64_t)v.w-v.y)*(height-1))/denominator);
    return true;
}
static void check_project(sh2z_vec4_t v,uint16_t width,uint16_t height) {
    sh2z_screen_t actual={1234,-2321},expected=actual;
    CHECK(sh2z_project_clip(&actual,v,width,height)==oracle_project(&expected,v,width,height));
    CHECK(actual.x==expected.x&&actual.y==expected.y);
}
static void projection_cases(void) {
    static const int32_t denominators[]={1,2,3,255,256,65535,65536,65537,1<<30,INT32_MAX};
    static const uint16_t screens[]={0,1,2,3,15,16,17,255,256,257,320,352,480,640,4096,32766,32767,32768,UINT16_MAX};
    for(unsigned i=0;i<sizeof(denominators)/sizeof(denominators[0]);i++) {
        int32_t w=denominators[i];
        const int32_t points[]={-w,-w+1,-1,0,1,w-1,w};
        for(unsigned x=0;x<7;x++) for(unsigned y=0;y<7;y++)
            for(unsigned j=0;j<sizeof(screens)/sizeof(screens[0]);j++)
                for(unsigned k=0;k<sizeof(screens)/sizeof(screens[0]);k++)
                    check_project((sh2z_vec4_t){points[x],points[y],0,w},screens[j],screens[k]);
        check_project((sh2z_vec4_t){INT32_MIN,0,0,w},320,240);
        check_project((sh2z_vec4_t){0,INT32_MIN,0,w},320,240);
        check_project((sh2z_vec4_t){0,0,INT32_MIN,w},320,240);
        check_project((sh2z_vec4_t){0,0,w,w},32767,32767);
        check_project((sh2z_vec4_t){0,0,-w,w},32767,32767);
        if(w<INT32_MAX) {
            check_project((sh2z_vec4_t){w+1,0,0,w},320,240);
            check_project((sh2z_vec4_t){0,w+1,0,w},320,240);
            check_project((sh2z_vec4_t){0,0,w+1,w},320,240);
        }
    }
    for(unsigned i=0;i<500000;i++) {
        int32_t w=(int32_t)(random_u32()&INT32_MAX);
        if(!w) w=1;
        uint32_t span=2u*(uint32_t)w;
        int32_t x=(int32_t)((int64_t)(random_u32()%(span+1u))-w);
        int32_t y=(int32_t)((int64_t)(random_u32()%(span+1u))-w);
        int32_t z=(int32_t)((int64_t)(random_u32()%(span+1u))-w);
        check_project((sh2z_vec4_t){x,y,z,w},(uint16_t)(random_u32()&32767u),(uint16_t)(random_u32()&32767u));
    }
    check_project((sh2z_vec4_t){0,0,0,0},320,240);
    check_project((sh2z_vec4_t){0,0,0,-1},320,240);
    check_project((sh2z_vec4_t){0,0,0,INT32_MIN},320,240);
    CHECK(!sh2z_project_clip(NULL,(sh2z_vec4_t){0,0,0,1},320,240));
}

int main(void) {
    transform_cases(); collision_cases(); inverse_cases(); projection_cases();
    printf("transform/collision wide-oracle checks: %u passed\n",checks);
    return 0;
}
