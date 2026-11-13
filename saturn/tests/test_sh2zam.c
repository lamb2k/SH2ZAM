/* Host-side SH2ZAM behavior tests; SH-2 instruction path needs console tests. */
#include "sh2zam/sh2zam.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

static int32_t abs32(int32_t a) { return a < 0 ? -a : a; }
static void near(sh2z_fx_t actual, sh2z_fx_t expected, int32_t tolerance) {
    assert(abs32(actual-expected) <= tolerance);
}

int main(void) {
    assert(sh2z_from_int(1) == SH2Z_ONE);
    assert(sh2z_from_int(-2) == -2*SH2Z_ONE);
    assert(sh2z_from_int(50000) == INT32_MAX);
    assert(sh2z_from_int(-50000) == INT32_MIN);
    assert(sh2z_to_int(-SH2Z_ONE-SH2Z_HALF) == -1);
    assert(sh2z_add(INT32_MAX, 1) == INT32_MAX);
    assert(sh2z_sub(INT32_MIN, 1) == INT32_MIN);
    assert(sh2z_add(SH2Z_ONE, -SH2Z_ONE) == 0);
    assert(sh2z_mul(SH2Z_HALF, SH2Z_HALF) == SH2Z_ONE/4);
    assert(sh2z_mul(-SH2Z_HALF, SH2Z_HALF) == -SH2Z_ONE/4);
    assert(sh2z_mul(INT32_MAX, INT32_MAX) == INT32_MAX);
    assert(sh2z_div(SH2Z_ONE, sh2z_from_int(2)) == SH2Z_HALF);
    assert(sh2z_div(-SH2Z_ONE, sh2z_from_int(2)) == -SH2Z_HALF);
    assert(sh2z_div(SH2Z_ONE, 0) == INT32_MAX);
    assert(sh2z_div(-SH2Z_ONE, 0) == INT32_MIN);
    assert(sh2z_div(0, 0) == 0);
    assert(sh2z_sqrt(0) == 0);
    assert(sh2z_sqrt(-SH2Z_ONE) == 0);
    assert(sh2z_sqrt(sh2z_from_int(4)) == sh2z_from_int(2));
    near(sh2z_sqrt(sh2z_from_int(2)), 92681, 2);

    assert(sh2z_angle_from_degrees(90) == SH2Z_ANGLE_QUARTER);
    assert(sh2z_angle_from_degrees(-90) == 49152u);
    assert(sh2z_angle_from_degrees(450) == SH2Z_ANGLE_QUARTER);
    assert(sh2z_sin(0) == 0);
    assert(sh2z_cos(0) == SH2Z_ONE);
    assert(sh2z_sin(SH2Z_ANGLE_QUARTER) == SH2Z_ONE);
    assert(sh2z_sin(SH2Z_ANGLE_HALF) == 0);
    assert(sh2z_sin((sh2z_angle_t)49152u) == -SH2Z_ONE);
    assert(sh2z_cos(SH2Z_ANGLE_HALF) == -SH2Z_ONE);
    near(sh2z_sin(sh2z_angle_from_degrees(45)), 46341, 12);
    for (unsigned a=0; a<65536u; a+=73u) {
        const sh2z_angle_t angle = (sh2z_angle_t)a;
        assert(sh2z_sin((sh2z_angle_t)(angle+SH2Z_ANGLE_HALF)) == -sh2z_sin(angle));
        assert(sh2z_cos(angle) == sh2z_sin((sh2z_angle_t)(angle+SH2Z_ANGLE_QUARTER)));
    }
    const sh2z_vec3_t a={sh2z_from_int(3),sh2z_from_int(4),0};
    assert(sh2z_vec3_dot(a,a) == sh2z_from_int(25));
    assert(sh2z_vec3_length(a) == sh2z_from_int(5));
    sh2z_vec3_t unit = sh2z_vec3_normalize(a);
    near(unit.x, (sh2z_fx_t)39321, 2);
    near(unit.y, (sh2z_fx_t)52428, 2);
    const sh2z_vec3_t x={SH2Z_ONE,0,0}, y={0,SH2Z_ONE,0};
    sh2z_vec3_t z = sh2z_vec3_cross(x,y);
    assert(z.z == SH2Z_ONE && z.x == 0 && z.y == 0);
    assert(sh2z_vec3_normalize((sh2z_vec3_t){0,0,0}).x == 0);

    sh2z_mat34_t identity = sh2z_mat34_identity();
    sh2z_mat34_t translation = sh2z_mat34_identity();
    translation.m[0][3]=sh2z_from_int(4);
    translation.m[1][3]=sh2z_from_int(-2);
    sh2z_mat34_t combined;
    sh2z_mat34_multiply(&combined,&identity,&translation);
    sh2z_vec3_t point={SH2Z_ONE,SH2Z_ONE,SH2Z_ONE};
    sh2z_vec3_t moved=sh2z_mat34_transform_point(&combined,point);
    assert(moved.x==sh2z_from_int(5) && moved.y==-SH2Z_ONE && moved.z==SH2Z_ONE);
    sh2z_vec3_t direction=sh2z_mat34_transform_direction(&combined,point);
    assert(direction.x==SH2Z_ONE && direction.y==SH2Z_ONE);
    sh2z_mat34_multiply(&translation,&translation,&identity);
    assert(translation.m[0][3]==sh2z_from_int(4));

    sh2z_mat44_t m=sh2z_mat44_identity();
    sh2z_mat44_t n=sh2z_mat44_identity();
    n.m[0][3]=sh2z_from_int(10);
    sh2z_mat44_multiply(&n,&m,&n); /* alias-safe output */
    const sh2z_vec4_t v=sh2z_mat44_transform(&n,(sh2z_vec4_t){SH2Z_ONE,0,0,SH2Z_ONE});
    assert(v.x==sh2z_from_int(11) && v.w==SH2Z_ONE);

    sh2z_quat_t qi=sh2z_quat_identity();
    sh2z_quat_t q={SH2Z_HALF, 0, 0, SH2Z_HALF};
    sh2z_quat_t qr=sh2z_quat_multiply(qi,q);
    assert(qr.x==q.x && qr.w==q.w);
    sh2z_quat_t normalized=sh2z_quat_normalize(q);
    near(normalized.x, 46341, 10);
    near(normalized.w, 46341, 10);
    assert(sh2z_quat_normalize((sh2z_quat_t){0,0,0,0}).w==SH2Z_ONE);

    puts("SH2ZAM host tests passed (portable semantics only)");
    return 0;
}
