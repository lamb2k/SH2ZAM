/* Private WASM experiments. Not linked into the library or exhibit.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
#include "sh2zam/sh2zam.h"
#include <wasm_simd128.h>
#include <limits.h>

#define API(name) __attribute__((export_name(name)))
#define CAPACITY 16384u
static sh2z_vec3_t input[CAPACITY], general_output[CAPACITY];
static sh2z_vec3_t current_output[CAPACITY], prepared_output[CAPACITY];
static sh2z_vec3_t wide_output[CAPACITY], packed_output[CAPACITY];
static sh2z_vec3_t rows_output[CAPACITY], split_output[CAPACITY];
static sh2z_vec3_t narrow_output[CAPACITY];
static float float_input[CAPACITY][3], float_output[CAPACITY][3];
static int32_t packed_input[CAPACITY*3u];
static int16_t split_input[CAPACITY*8u];
static sh2z_mat34_t fixed_matrix;
static float float_matrix[3][4];
static sh2z_mesh_view_t mesh_view;
static sh2z_transform_plan_t plan;
static unsigned count;
static bool split_input_fits;

/* Freestanding compiler-generated copies. No libc or host imports. */
void *memcpy(void *d,const void *s,size_t n) {
    unsigned char *a=d;const unsigned char *b=s;
    for(size_t i=0;i<n;i++)a[i]=b[i];
    return d;
}
void *memset(void *d,int v,size_t n) {
    unsigned char *a=d;for(size_t i=0;i<n;i++)a[i]=(unsigned char)v;
    return d;
}
API("capacity") unsigned capacity(void){return CAPACITY;}
API("input_ptr") sh2z_vec3_t *input_ptr(void){return input;}
API("matrix_ptr") sh2z_mat34_t *matrix_ptr(void){return &fixed_matrix;}
API("before_ptr") float *before_ptr(void){return &float_output[0][0];}
API("general_ptr") sh2z_vec3_t *general_ptr(void){return general_output;}
API("after_ptr") sh2z_vec3_t *after_ptr(void){return current_output;}
API("prepared_ptr") sh2z_vec3_t *prepared_ptr(void){return prepared_output;}
API("prepared_inclusive_ptr") sh2z_vec3_t *prepared_inclusive_ptr(void){return prepared_output;}
API("wide_ptr") sh2z_vec3_t *wide_ptr(void){return wide_output;}
API("packed_ptr") sh2z_vec3_t *packed_ptr(void){return packed_output;}
API("rows_ptr") sh2z_vec3_t *rows_ptr(void){return rows_output;}
API("split_ptr") sh2z_vec3_t *split_ptr(void){return split_output;}
API("narrow_ptr") sh2z_vec3_t *narrow_ptr(void){return narrow_output;}

API("prepare_float") void prepare_float(void) {
    for(unsigned i=0;i<count;i++) {
        float_input[i][0]=(float)input[i].x/65536.0f;
        float_input[i][1]=(float)input[i].y/65536.0f;
        float_input[i][2]=(float)input[i].z/65536.0f;
    }
}
API("prepare_view") int prepare_view(void) {
    return sh2z_mesh_view_init(&mesh_view,input,count);
}
API("pack_wide") void pack_wide(void) {
    for(unsigned i=0;i+4<=count;i+=4)for(unsigned j=0;j<4;j++) {
        packed_input[i*3+j]=input[i+j].x;
        packed_input[i*3+4+j]=input[i+j].y;
        packed_input[i*3+8+j]=input[i+j].z;
    }
}
static int32_t low_signed16(int32_t x) {
    const uint32_t low=(uint32_t)x&UINT32_C(65535);
    return low<UINT32_C(32768)?(int32_t)low:(int32_t)low-65536;
}
API("pack_split") void pack_split(void) {
    split_input_fits=true;
    for(unsigned i=0;i+4<=count;i+=4)for(unsigned j=0;j<4;j++) {
        const int32_t x=input[i+j].x,y=input[i+j].y,z=input[i+j].z;
        const int32_t xs=low_signed16(x),ys=low_signed16(y),zs=low_signed16(z);
        const int32_t xh=(int32_t)(((int64_t)x-xs)/65536);
        const int32_t yh=(int32_t)(((int64_t)y-ys)/65536);
        const int32_t zh=(int32_t)(((int64_t)z-zs)/65536);
        if(xh<INT16_MIN||xh>INT16_MAX||yh<INT16_MIN||yh>INT16_MAX||
           zh<INT16_MIN||zh>INT16_MAX)split_input_fits=false;
        /* Conversion outside int16_t range is avoided even for an ineligible
         * mesh. Its packed values are ignored by the guarded executors. */
        split_input[i*8+j*2]=(int16_t)xs;
        split_input[i*8+j*2+1]=(int16_t)ys;
        split_input[i*8+8+j*2]=(int16_t)zs;
        split_input[i*8+8+j*2+1]=0;
        split_input[i*8+16+j*2]=(int16_t)(xh>=INT16_MIN&&xh<=INT16_MAX?xh:0);
        split_input[i*8+16+j*2+1]=(int16_t)(yh>=INT16_MIN&&yh<=INT16_MAX?yh:0);
        split_input[i*8+24+j*2]=(int16_t)(zh>=INT16_MIN&&zh<=INT16_MAX?zh:0);
        split_input[i*8+24+j*2+1]=0;
    }
}
API("prepare") int prepare(unsigned n) {
    if(n>CAPACITY)return 0;
    count=n;
    prepare_float();pack_wide();pack_split();
    return prepare_view();
}
API("prepare_float_matrix") void prepare_float_matrix(void) {
    for(unsigned r=0;r<3;r++)for(unsigned c=0;c<4;c++)
        float_matrix[r][c]=(float)fixed_matrix.m[r][c]/65536.0f;
}
API("prepare_plan") int prepare_plan(void) {
    return sh2z_transform_plan_init(&plan,&mesh_view,&fixed_matrix);
}
API("apply_matrix") int apply_matrix(void) {
    prepare_float_matrix();return prepare_plan();
}
API("setup") int setup(unsigned ax,unsigned ay) {
    const sh2z_mat34_t x=sh2z_mat34_rotation_x((sh2z_angle_t)ax);
    const sh2z_mat34_t y=sh2z_mat34_rotation_y((sh2z_angle_t)ay);
    sh2z_mat34_multiply(&fixed_matrix,&y,&x);
    return apply_matrix();
}
API("fast_path") int fast_path(void){return plan.fast_path;}

/* Identical ordinary float affine loop and quantized inputs/coefficients to
 * exhibits/bridge.c. The same SIMD/optimization/FP flags apply to every TU. */
API("before") void before(void) {
    for(unsigned i=0;i<count;i++)for(unsigned r=0;r<3;r++)
        float_output[i][r]=float_matrix[r][0]*float_input[i][0]
            +float_matrix[r][1]*float_input[i][1]
            +float_matrix[r][2]*float_input[i][2]+float_matrix[r][3];
}
API("general") void general(void) {
    (void)sh2z_mat34_transform_points_general(general_output,&fixed_matrix,input,count);
}
API("after") void after(void) {
    (void)sh2z_mat34_transform_points(current_output,&fixed_matrix,input,count);
}
API("prepared") void prepared(void) {
    (void)sh2z_transform_plan_execute(prepared_output,&plan);
}
API("prepared_inclusive") void prepared_inclusive(void) {
    (void)prepare_plan();prepared();
}

#include "kernels.h"
