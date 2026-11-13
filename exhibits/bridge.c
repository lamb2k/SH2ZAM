/* Shared browser/Saturn benchmark bridge. MIT, Copyright (c) 2026 SH2ZAM Contributors.
 * Every kernel uses identical quantized inputs and matrix coefficients.
 * Matrix construction and rendering are outside measured transforms. The
 * browser's after() includes plan qualification once per complete batch.
 * The browser uses the portable backend, not an SH-2 emulator. */
#include "sh2zam/sh2zam.h"
#include "sh2zam/batch_experimental.h"
#ifdef __wasm__
#include "sh2zam/reuse_experimental.h"
#define API(name) __attribute__((export_name(name)))
#else
#define API(name)
#endif
#ifndef SH2Z_EXHIBIT_CAPACITY
#define SH2Z_EXHIBIT_CAPACITY 16384
#endif
#define CAPACITY SH2Z_EXHIBIT_CAPACITY
static sh2z_vec3_t input[CAPACITY], general_output[CAPACITY], fixed_output[CAPACITY];
static sh2z_vec3_t mac_output[CAPACITY],dmuls_output[CAPACITY];
static float float_input[CAPACITY][3], float_output[CAPACITY][3];
static sh2z_mat34_t fixed_matrix;
static sh2z_mesh_view_t mesh_view;
static sh2z_transform_plan_t transform_plan;
#ifdef __wasm__
/* An optional immutable-mesh strategy. Its generic reuse gate selects from
 * actual coordinate counts, never the scene name. Native candidate captures
 * keep their original prepared executor so their kernel identity is stable. */
#define REUSE_KEYS 256u
static sh2z_reuse_mesh_t reuse_mesh;
static sh2z_reuse_plan_t reuse_plan;
static sh2z_reuse_index_t reuse_indices[CAPACITY];
static int32_t reuse_x_keys[REUSE_KEYS];
static sh2z_reuse_yz_t reuse_yz_keys[REUSE_KEYS];
static sh2z_reuse_term_t reuse_x_terms[REUSE_KEYS],reuse_yz_terms[REUSE_KEYS];
#endif
static float float_matrix[3][4];
static unsigned count;
/* Freestanding compiler-generated structure copies. No host/libc imports. */
#ifdef __wasm__
void *memcpy(void *d,const void *s,size_t n) { unsigned char *a=d; const unsigned char *b=s; for(size_t i=0;i<n;i++) a[i]=b[i]; return d; }
void *memset(void *d,int v,size_t n) { unsigned char *a=d; for(size_t i=0;i<n;i++) a[i]=(unsigned char)v; return d; }
#endif
API("capacity") unsigned capacity(void) { return CAPACITY; }
API("input_ptr") sh2z_vec3_t *input_ptr(void) { return input; }
API("before_ptr") float *before_ptr(void) { return &float_output[0][0]; }
API("general_ptr") sh2z_vec3_t *general_ptr(void) { return general_output; }
API("after_ptr") sh2z_vec3_t *after_ptr(void) { return fixed_output; }
sh2z_vec3_t *mac_ptr(void) { return mac_output; }
sh2z_vec3_t *dmuls_ptr(void) { return dmuls_output; }
API("plan_prepare") int plan_prepare(void) {
#ifdef __wasm__
    if(reuse_mesh.reusable) {
        const bool ok=sh2z_reuse_plan_init(&reuse_plan,&reuse_mesh,&fixed_matrix,
            reuse_x_terms,REUSE_KEYS,reuse_yz_terms,REUSE_KEYS);
        if(ok)transform_plan=reuse_plan.base;
        return ok;
    }
    reuse_plan.enabled=false;
#endif
    return sh2z_transform_plan_init(&transform_plan,&mesh_view,&fixed_matrix);
}
/* Browser labels and exports identify the actual strategy. */
API("optimized_path") unsigned optimized_path(void) {
#ifdef __wasm__
    if(reuse_plan.enabled)return 2;
#endif
    return transform_plan.fast_path?1u:0u;
}
API("candidate_available") unsigned candidate_available(void) {
    return (transform_plan.fast_path?1u:0u)
        |(sh2z_transform_plan_mac_available(&transform_plan)?2u:0u)
        |(sh2z_transform_plan_dmuls_available(&transform_plan)?4u:0u);
}
API("prepare") int prepare(unsigned n) {
    if(n>CAPACITY) return 0;
#ifdef __wasm__
    if(!sh2z_reuse_mesh_init(&reuse_mesh,input,n,reuse_indices,CAPACITY,
        reuse_x_keys,REUSE_KEYS,reuse_yz_keys,REUSE_KEYS))return 0;
    mesh_view=reuse_mesh.view;
    reuse_plan.enabled=false;
#else
    if(!sh2z_mesh_view_init(&mesh_view,input,n)) return 0;
#endif
    count=n;
    for(unsigned i=0;i<n;i++) {
        float_input[i][0]=(float)input[i].x/65536.0f;
        float_input[i][1]=(float)input[i].y/65536.0f;
        float_input[i][2]=(float)input[i].z/65536.0f;
    }
    return 1;
}
/* Ry * Rx, row-major with column vectors. Quantize the matrix once, then use
 * its exact coefficients in every kernel. Reported float/fixed error isolates
 * transform rounding; it does not measure matrix/trig quantization error.
 * The sin/cos arguments remain for compatibility with recorded fixture poses. */
API("setup") void setup(unsigned ax,unsigned ay,float sx,float cx,float sy,float cy) {
    (void)sx; (void)cx; (void)sy; (void)cy;
    sh2z_mat34_t x=sh2z_mat34_rotation_x((sh2z_angle_t)ax);
    sh2z_mat34_t y=sh2z_mat34_rotation_y((sh2z_angle_t)ay);
    sh2z_mat34_multiply(&fixed_matrix,&y,&x);
    for(unsigned row=0;row<3;row++)
        for(unsigned col=0;col<4;col++)
            float_matrix[row][col]=(float)fixed_matrix.m[row][col]/65536.0f;
    (void)plan_prepare();
}
void before_range(unsigned start,unsigned n) {
    for(unsigned i=start;i<start+n;i++)
        for(unsigned j=0;j<3;j++)
            float_output[i][j]=float_matrix[j][0]*float_input[i][0]
                +float_matrix[j][1]*float_input[i][1]+float_matrix[j][2]*float_input[i][2]
                +float_matrix[j][3];
}
void general_range(unsigned start,unsigned n) {
    (void)sh2z_mat34_transform_points(general_output+start,&fixed_matrix,input+start,n);
}
void after_range(unsigned start,unsigned n) {
#ifdef __wasm__
    if(reuse_plan.enabled) {
        (void)sh2z_reuse_plan_execute_range(fixed_output+start,&reuse_plan,start,n);
        return;
    }
#endif
    (void)sh2z_transform_plan_execute_range(fixed_output+start,&transform_plan,start,n);
}
void mac_range(unsigned start,unsigned n) {
    (void)sh2z_transform_plan_execute_mac(mac_output+start,&transform_plan,start,n);
}
void dmuls_range(unsigned start,unsigned n) {
    (void)sh2z_transform_plan_execute_dmuls(dmuls_output+start,&transform_plan,start,n);
}
API("before") void before(void) { before_range(0,count); }
API("general") void general(void) { general_range(0,count); }
API("after_kernel") void after_kernel(void) { after_range(0,count); }
API("after") void after(void) {
    (void)plan_prepare();
    after_kernel();
}

