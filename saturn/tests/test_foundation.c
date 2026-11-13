/* Host oracle/regression suite; no assert/NDEBUG dependency.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/sh2zam.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned long checks=0;
#define CHECK(e) do { ++checks; if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(1); } } while(0)
static void near64(int64_t a,int64_t b,int64_t tolerance) {
    if(a-b>tolerance||b-a>tolerance) fprintf(stderr,"actual=%lld expected=%lld tolerance=%lld\n",(long long)a,(long long)b,(long long)tolerance);
    CHECK(a-b<=tolerance&&b-a<=tolerance);
}
#if !defined(__SIZEOF_INT128__)
#error "The host oracle suite requires GCC/Clang int128; the library does not."
#endif
__extension__ typedef __int128 wide_t;
static int32_t sat(wide_t n) { return n>INT32_MAX?INT32_MAX:(n<INT32_MIN?INT32_MIN:(int32_t)n); }
static wide_t floor16(wide_t n) { return n>=0?n/65536:-((-n+65535)/65536); }
static uint32_t rng=UINT32_C(0x51a7b00b);
static uint32_t random_u32(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static int32_t random_i32(void) { uint32_t x=random_u32(); return x<=INT32_MAX?(int32_t)x:(int32_t)((int64_t)x-INT64_C(4294967296)); }
static int32_t fx(int32_t x) { return sh2z_from_int(x); }

static void scalars(void) {
    const int32_t edge[]={INT32_MIN,INT32_MIN+1,-65537,-65536,-32768,-1,0,1,32768,65535,65536,INT32_MAX-1,INT32_MAX};
    for(size_t i=0;i<sizeof(edge)/sizeof(edge[0]);i++) for(size_t j=0;j<sizeof(edge)/sizeof(edge[0]);j++) {
        int32_t a=edge[i],b=edge[j];
        CHECK(sh2z_add(a,b)==sat((wide_t)a+b)); CHECK(sh2z_sub(a,b)==sat((wide_t)a-b));
        CHECK(sh2z_mul(a,b)==sat(floor16((wide_t)a*b)));
        CHECK(sh2z_div(a,b)==(b?sat(((wide_t)a*65536)/b):(a>0?INT32_MAX:(a<0?INT32_MIN:0))));
    }
    CHECK(sh2z_abs(INT32_MIN)==INT32_MAX); CHECK(sh2z_neg(INT32_MIN)==INT32_MAX);
    CHECK(sh2z_floor(-1)==-65536); CHECK(sh2z_ceil(-1)==0);
    CHECK(sh2z_round(-32768)==-65536); CHECK(sh2z_round(32768)==65536);
    CHECK(sh2z_floor(INT32_MIN)==INT32_MIN); CHECK(sh2z_ceil(INT32_MAX)==INT32_MAX);
    CHECK(sh2z_round(INT32_MAX)==INT32_MAX); CHECK(sh2z_clamp(7,9,2)==7);
    CHECK(sh2z_min(1,2)==1); CHECK(sh2z_max(1,2)==2);
    CHECK(sh2z_smoothstep(-1)==0); CHECK(sh2z_smoothstep(INT32_MAX)==SH2Z_ONE);
    CHECK(sh2z_smoothstep(SH2Z_HALF)==SH2Z_HALF);
    CHECK(sh2z_rsqrt(0)==0); CHECK(sh2z_rsqrt(-1)==0); CHECK(sh2z_recip(0)==INT32_MAX);
    CHECK(sh2z_rsqrt(fx(4))==SH2Z_HALF);
    CHECK(sh2z_lerp(INT32_MIN,INT32_MAX,SH2Z_HALF)==-1);
    for(unsigned i=0;i<50000;i++) {
        int32_t a=random_i32(),b=random_i32(),c=random_i32();
        CHECK(sh2z_mul(a,b)==sat(floor16((wide_t)a*b)));
        CHECK(sh2z_div(a,b)==(b?sat(((wide_t)a*65536)/b):(a>0?INT32_MAX:(a<0?INT32_MIN:0))));
        wide_t p=(wide_t)a*b;
        CHECK(sh2z_muldiv(a,b,c)==(c?sat(p/c):(p>0?INT32_MAX:(p<0?INT32_MIN:0))));
        CHECK(sh2z_lerp(a,b,c)==sat((wide_t)a+floor16(((wide_t)b-a)*c)));
        CHECK(sh2z_floor(a)==sat(floor16(a)*65536));
        CHECK(sh2z_ceil(a)==sat(-floor16(-(wide_t)a)*65536));
        int32_t positive=(int32_t)(random_u32()&INT32_MAX),root=sh2z_sqrt(positive);
        uint64_t n=(uint64_t)(uint32_t)positive*65536u;
        CHECK((uint64_t)(uint32_t)root*(uint32_t)root<=n);
        CHECK((uint64_t)((uint32_t)root+1)*((uint32_t)root+1)>n);
    }
}
static void vectors(void) {
    sh2z_vec3_t big={fx(300),fx(400),0};
    CHECK(sh2z_vec3_length(big)==fx(500));
    sh2z_vec3_t unit=sh2z_vec3_normalize(big);
    near64(unit.x,39321,1); near64(unit.y,52428,1);
    unit=sh2z_vec3_normalize((sh2z_vec3_t){1,1,0});
    near64(unit.x,46340,1); near64(unit.y,46340,1);
    unit=sh2z_vec3_normalize((sh2z_vec3_t){INT32_MIN,0,0}); CHECK(unit.x==-SH2Z_ONE);
    sh2z_vec4_t huge={INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN};
    sh2z_vec4_t h=sh2z_vec4_normalize(huge);
    CHECK(sh2z_vec4_length(huge)==INT32_MAX); CHECK(h.x==-SH2Z_HALF&&h.w==-SH2Z_HALF);
    CHECK(sh2z_vec4_dot((sh2z_vec4_t){INT32_MAX,INT32_MAX,INT32_MAX,INT32_MAX},
                      (sh2z_vec4_t){INT32_MAX,-INT32_MAX,INT32_MAX,-INT32_MAX})==0);
    CHECK(sh2z_vec3_dot((sh2z_vec3_t){1,1,1},(sh2z_vec3_t){32768,32768,32768})==1);
    CHECK(sh2z_vec2_length((sh2z_vec2_t){1,0})==1);
    CHECK(sh2z_vec2_cross((sh2z_vec2_t){SH2Z_ONE,0},(sh2z_vec2_t){0,SH2Z_ONE})==SH2Z_ONE);
    sh2z_vec2_t v2=sh2z_vec2_add((sh2z_vec2_t){1,2},(sh2z_vec2_t){3,4}); CHECK(v2.x==4&&v2.y==6);
    v2=sh2z_vec2_sub(v2,(sh2z_vec2_t){3,4}); CHECK(v2.x==1&&v2.y==2);
    v2=sh2z_vec2_scale(v2,fx(2)); CHECK(v2.x==2&&v2.y==4);
    v2=sh2z_vec2_lerp(v2,(sh2z_vec2_t){0,0},SH2Z_HALF); CHECK(v2.x==1&&v2.y==2);
    sh2z_vec4_t v4=sh2z_vec4_sub(sh2z_vec4_add(h,h),h); CHECK(v4.w==h.w);
    sh2z_vec3_t v3=sh2z_vec3_lerp(big,big,SH2Z_HALF); CHECK(v3.x==big.x);
    CHECK(sh2z_vec3_sub(big,big).x==0);
    CHECK(sh2z_vec4_normalize((sh2z_vec4_t){0,0,0,0}).w==0);
    for(unsigned i=0;i<20000;i++) {
        int32_t a[4],b[4]; wide_t total=0;
        for(unsigned j=0;j<4;j++) { a[j]=random_i32(); b[j]=random_i32(); if(i%3) {a[j]/=256; b[j]/=256;} total+=(wide_t)a[j]*b[j]; }
        sh2z_vec4_t av={a[0],a[1],a[2],a[3]},bv={b[0],b[1],b[2],b[3]};
        CHECK(sh2z_vec4_dot(av,bv)==sat(floor16(total)));
        CHECK(sh2z_vec3_dot((sh2z_vec3_t){a[0],a[1],a[2]},(sh2z_vec3_t){b[0],b[1],b[2]})==sat(floor16(total-(wide_t)a[3]*b[3])));
        CHECK(sh2z_vec2_dot((sh2z_vec2_t){a[0],a[1]},(sh2z_vec2_t){b[0],b[1]})==sat(floor16((wide_t)a[0]*b[0]+(wide_t)a[1]*b[1])));
        long double sum=0; for(unsigned j=0;j<4;j++) sum+=(long double)a[j]*a[j];
        long double len=sqrtl(sum);
        near64(sh2z_vec4_length(av),len>INT32_MAX?INT32_MAX:(int64_t)len,1);
        sh2z_vec4_t normalized=sh2z_vec4_normalize(av);
        int32_t nv[]={normalized.x,normalized.y,normalized.z,normalized.w};
        for(unsigned j=0;j<4;j++) near64(nv[j],len?(int64_t)((long double)a[j]*65536/len):0,2);
        sh2z_vec3_t cross=sh2z_vec3_cross((sh2z_vec3_t){a[0],a[1],a[2]},(sh2z_vec3_t){b[0],b[1],b[2]});
        CHECK(cross.x==sat(floor16((wide_t)a[1]*b[2]-(wide_t)a[2]*b[1])));
    }
}
static void trigonometry(void) {
    const long double tau=6.283185307179586476925286766559L;
    int64_t max_sin_error=0,max_angle_error=0;
    CHECK(sh2z_angle_delta(65000,100)==636); CHECK(sh2z_angle_delta(0,32768)==-32768);
    CHECK(sh2z_angle_lerp(65000,100,SH2Z_ONE)==100);
    CHECK(sh2z_angle_lerp(65000,100,-1)==65000);
    CHECK(sh2z_tan(16384)==INT32_MAX); CHECK(sh2z_tan(49152)==INT32_MIN);
    CHECK(sh2z_atan2(0,0)==0); CHECK(sh2z_atan(0)==0);
    CHECK(sh2z_asin(INT32_MAX)==16384); CHECK(sh2z_acos(INT32_MIN)==32768);
    for(unsigned a=0;a<65536;a++) {
        sh2z_fx_t s,c; sh2z_sincos((sh2z_angle_t)a,&s,&c);
        int64_t ref=llroundl(sinl((long double)a*tau/65536)*65536);
        int64_t e=llabs((long long)s-ref); if(e>max_sin_error) max_sin_error=e;
        CHECK(e<=2); CHECK(c==sh2z_cos((sh2z_angle_t)a));
        int64_t ae=llabs(sh2z_angle_delta((sh2z_angle_t)a,sh2z_atan2(s,c)));
        if(ae>max_angle_error) max_angle_error=ae;
        CHECK(ae<=3);
    }
    for(int32_t x=-65536;x<=65536;x++) {
        sh2z_angle_t ref=(sh2z_angle_t)(int32_t)llroundl(asinl((long double)x/65536)*65536/tau);
        CHECK(abs(sh2z_angle_delta(ref,sh2z_asin(x)))<=3);
        CHECK(sh2z_acos(x)==(sh2z_angle_t)(16384u-sh2z_asin(x)));
    }
    sh2z_sincos(0,NULL,NULL);
    for(unsigned i=0;i<10000;i++) {
        int32_t x=random_i32(),y=random_i32();
        sh2z_angle_t ref=(sh2z_angle_t)(int32_t)llroundl(atan2l(y,x)*65536/tau);
        CHECK(abs(sh2z_angle_delta(ref,sh2z_atan2(y,x)))<=3);
    }
    printf("Full-turn max sine error: %lld Q16 LSB; atan2 roundtrip: %lld angle units\n",(long long)max_sin_error,(long long)max_angle_error);
}
static void transforms(void) {
    sh2z_mat22_t a2=sh2z_mat22_identity(); sh2z_mat22_multiply(&a2,&a2,&a2);
    CHECK(sh2z_mat22_transform(&a2,(sh2z_vec2_t){1,2}).y==2);
    sh2z_mat33_t a3=sh2z_mat33_identity(); sh2z_mat33_multiply(&a3,&a3,&a3);
    CHECK(sh2z_mat33_transform(&a3,(sh2z_vec3_t){1,2,3}).z==3);
    a3.m[0][1]=17; sh2z_mat33_transpose(&a3,&a3); CHECK(a3.m[1][0]==17);
    sh2z_mat44_t a4=sh2z_mat44_identity(); a4.m[0][1]=13; sh2z_mat44_transpose(&a4,&a4); CHECK(a4.m[1][0]==13);
    for(unsigned i=0;i<1000;i++) {
        sh2z_mat44_t a,b,result;
        for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) { a.m[row][col]=random_i32()/1024; b.m[row][col]=random_i32()/1024; }
        sh2z_mat44_multiply(&result,&a,&b);
        for(unsigned row=0;row<4;row++) for(unsigned col=0;col<4;col++) {
            wide_t sum=0; for(unsigned k=0;k<4;k++) sum+=(wide_t)a.m[row][k]*b.m[k][col];
            CHECK(result.m[row][col]==sat(floor16(sum)));
        }
        sh2z_mat44_multiply(&a,&a,&b); CHECK(!memcmp(&a,&result,sizeof(a)));
    }
    for(unsigned angle=0;angle<65536;angle+=254) {
        sh2z_mat34_t x=sh2z_mat34_rotation_x((sh2z_angle_t)angle),y=sh2z_mat34_rotation_y((sh2z_angle_t)angle);
        sh2z_mat34_t z=sh2z_mat34_rotation_z((sh2z_angle_t)angle),m,inverse,id;
        sh2z_mat34_multiply(&m,&x,&y); sh2z_mat34_multiply(&m,&m,&z);
        m.m[0][3]=fx(3);m.m[1][3]=fx(-4);m.m[2][3]=fx(5);
        CHECK(sh2z_mat34_inverse_rigid(&inverse,&m)); sh2z_mat34_multiply(&id,&inverse,&m);
        for(unsigned row=0;row<3;row++) for(unsigned col=0;col<4;col++) near64(id.m[row][col],row==col?SH2Z_ONE:0,40);
        sh2z_quat_t q=sh2z_quat_axis_angle((sh2z_vec3_t){0,SH2Z_ONE,0},(sh2z_angle_t)angle);
        sh2z_mat34_t qm=sh2z_quat_to_mat34(q);
        for(unsigned row=0;row<3;row++) for(unsigned col=0;col<3;col++) near64(qm.m[row][col],y.m[row][col],12);
        sh2z_quat_t invq=sh2z_quat_conjugate(q),ident=sh2z_quat_multiply(q,invq);
        near64(ident.w,SH2Z_ONE,5); near64(ident.x,0,2);
        sh2z_quat_t neg={-q.x,-q.y,-q.z,-q.w}; sh2z_quat_t blend=sh2z_quat_nlerp(q,neg,SH2Z_HALF);
        near64(llabs(blend.y),llabs(q.y),2); near64(llabs(blend.w),llabs(q.w),2);
    }
    sh2z_mat34_t view,original=sh2z_mat34_identity(); view=original;
    CHECK(!sh2z_mat34_look_at(&view,(sh2z_vec3_t){0,0,0},(sh2z_vec3_t){0,0,0},(sh2z_vec3_t){0,SH2Z_ONE,0}));
    CHECK(!memcmp(&view,&original,sizeof(view)));
    CHECK(sh2z_mat34_look_at(&view,(sh2z_vec3_t){0,0,fx(5)},(sh2z_vec3_t){0,0,0},(sh2z_vec3_t){0,SH2Z_ONE,0}));
    CHECK(sh2z_mat34_transform_point(&view,(sh2z_vec3_t){0,0,0}).z==fx(-5));
    CHECK(sh2z_mat34_transform_point(&view,(sh2z_vec3_t){0,0,fx(5)}).z==0);
    sh2z_mat34_t scale=sh2z_mat34_scale((sh2z_vec3_t){fx(2),fx(2),fx(2)});
    CHECK(!sh2z_mat34_inverse_rigid(&view,&scale));
    sh2z_mat44_t p; CHECK(sh2z_mat44_perspective(&p,16384,SH2Z_ONE,SH2Z_ONE,fx(10)));
    CHECK(p.m[0][0]==SH2Z_ONE&&p.m[1][1]==SH2Z_ONE);
    sh2z_mat44_t saved=p; CHECK(!sh2z_mat44_perspective(&p,0,SH2Z_ONE,SH2Z_ONE,fx(10))); CHECK(!memcmp(&saved,&p,sizeof(p)));
    CHECK(!sh2z_mat44_perspective(&p,16384,0,SH2Z_ONE,fx(10)));
    CHECK(!sh2z_mat44_perspective(&p,16384,SH2Z_ONE,fx(10),fx(1)));
    sh2z_screen_t screen={7,9};
    CHECK(sh2z_project_clip(&screen,(sh2z_vec4_t){0,0,0,SH2Z_ONE},320,224)); CHECK(screen.x==159&&screen.y==111);
    CHECK(sh2z_project_clip(&screen,(sh2z_vec4_t){SH2Z_ONE,-SH2Z_ONE,0,SH2Z_ONE},320,224)); CHECK(screen.x==319&&screen.y==223);
    CHECK(!sh2z_project_clip(&screen,(sh2z_vec4_t){0,0,0,0},320,224)); CHECK(screen.x==319);
    CHECK(!sh2z_project_clip(&screen,(sh2z_vec4_t){0,0,SH2Z_ONE+1,SH2Z_ONE},320,224));
    CHECK(!sh2z_project_clip(&screen,(sh2z_vec4_t){0,0,0,SH2Z_ONE},0,224));
    CHECK(sh2z_project_clip(&screen,(sh2z_vec4_t){INT32_MAX,INT32_MAX,0,INT32_MAX},32767,32767)); CHECK(screen.x==32766&&screen.y==0);
    sh2z_vec3_t rotated=sh2z_quat_rotate(sh2z_quat_axis_angle((sh2z_vec3_t){0,0,SH2Z_ONE},16384),(sh2z_vec3_t){SH2Z_ONE,0,0});
    near64(rotated.x,0,8); near64(rotated.y,SH2Z_ONE,8);
}
static void bounds_jobs_stack(void) {
    sh2z_aabb_t b=sh2z_aabb((sh2z_vec3_t){fx(2),fx(2),fx(2)},(sh2z_vec3_t){0,0,0});
    CHECK(sh2z_aabb_contains(b,(sh2z_vec3_t){0,0,0})); CHECK(!sh2z_aabb_contains(b,(sh2z_vec3_t){-1,0,0}));
    CHECK(sh2z_aabb_intersects(b,b));
    sh2z_sphere_t s={{fx(3),SH2Z_ONE,SH2Z_ONE},SH2Z_ONE}; CHECK(sh2z_sphere_intersects_aabb(s,b)); ++s.center.x; CHECK(!sh2z_sphere_intersects_aabb(s,b));
    sh2z_sphere_t a={{INT32_MIN,0,0},INT32_MAX},c={{INT32_MAX,0,0},INT32_MAX};
    CHECK(!sh2z_sphere_intersects(a,c)); --c.center.x; CHECK(sh2z_sphere_intersects(a,c)); c.center.y=INT32_MAX; CHECK(!sh2z_sphere_intersects(a,c));
    s.radius=-1; CHECK(!sh2z_sphere_intersects(s,s)); CHECK(!sh2z_sphere_intersects_aabb(s,b));
    sh2z_mat34_t storage[2],t=sh2z_mat34_translation((sh2z_vec3_t){SH2Z_ONE,0,0}); sh2z_stack_t stack;
    CHECK(sh2z_stack_init(&stack,storage,2)); CHECK(!sh2z_stack_pop(&stack)); CHECK(sh2z_stack_push(&stack)); CHECK(!sh2z_stack_push(&stack));
    CHECK(sh2z_stack_multiply(&stack,&t)); CHECK(sh2z_stack_top(&stack)->m[0][3]==SH2Z_ONE);
    CHECK(sh2z_stack_pop(&stack)); CHECK(sh2z_stack_top(&stack)->m[0][3]==0);
    CHECK(!sh2z_stack_init(&stack,NULL,0)); CHECK(!sh2z_stack_top(&stack)); CHECK(!sh2z_stack_push(&stack));
    const size_t counts[]={0,1,5,17,SIZE_MAX};
    for(unsigned i=0;i<5;i++) for(size_t workers=1;workers<8;workers++) {
        size_t previous=0;
        for(size_t worker=0;worker<workers;worker++) {
            sh2z_range_t range; CHECK(sh2z_partition(counts[i],worker,workers,&range));
            CHECK(range.begin==previous); CHECK(range.end>=range.begin&&range.end<=counts[i]); previous=range.end;
        }
        CHECK(previous==counts[i]);
    }
    sh2z_range_t range; CHECK(!sh2z_partition(5,0,0,&range)); CHECK(!sh2z_partition(5,2,2,&range));
    sh2z_vec3_t points[17],reference[17];
    for(unsigned i=0;i<17;i++) { points[i]=(sh2z_vec3_t){fx((int32_t)i),0,0}; reference[i]=sh2z_mat34_transform_point(&t,points[i]); }
    for(size_t worker=0;worker<2;worker++) {
        CHECK(sh2z_partition(17,worker,2,&range)); CHECK(sh2z_mat34_transform_points(points+range.begin,&t,points+range.begin,range.end-range.begin));
    }
    CHECK(!memcmp(points,reference,sizeof(points))); CHECK(sh2z_mat34_transform_points(NULL,NULL,NULL,0)); CHECK(!sh2z_mat34_transform_points(NULL,NULL,NULL,1));
}
int main(void) {
    scalars();vectors();trigonometry();transforms();bounds_jobs_stack();
    printf("%lu checks passed; backend=%s; full-range integer oracle and trigonometric sweeps\n",checks,sh2z_backend());
    return 0;
}
