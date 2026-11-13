/* Browser exhibit bridge. MIT, Copyright (c) 2026 SH2ZAM Contributors.
 * Both kernels are C -> WebAssembly, same inputs, one batch/call.
 * Matrix setup and rendering are outside the measured transform kernels.
 * SH2ZAM's PORTABLE backend is used: this is not an SH-2 emulator. */
#include "sh2zam/sh2zam.h"
#ifdef __wasm__
#define API(name) __attribute__((export_name(name)))
#else
#define API(name)
#endif
#ifndef SH2Z_EXHIBIT_CAPACITY
#define SH2Z_EXHIBIT_CAPACITY 16384
#endif
#define CAPACITY SH2Z_EXHIBIT_CAPACITY
static sh2z_vec3_t input[CAPACITY], fixed_output[CAPACITY];
static float float_input[CAPACITY][3], float_output[CAPACITY][3];
static sh2z_mat34_t fixed_matrix;
static float float_matrix[3][3];
static unsigned count;
/* Freestanding compiler-generated structure copies. No host/libc imports. */
#ifdef __wasm__
void *memcpy(void *d,const void *s,size_t n) { unsigned char *a=d; const unsigned char *b=s; for(size_t i=0;i<n;i++) a[i]=b[i]; return d; }
void *memset(void *d,int v,size_t n) { unsigned char *a=d; for(size_t i=0;i<n;i++) a[i]=(unsigned char)v; return d; }
#endif
API("capacity") unsigned capacity(void) { return CAPACITY; }
API("input_ptr") sh2z_vec3_t *input_ptr(void) { return input; }
API("before_ptr") float *before_ptr(void) { return &float_output[0][0]; }
API("after_ptr") sh2z_vec3_t *after_ptr(void) { return fixed_output; }
API("prepare") int prepare(unsigned n) {
    if(n>CAPACITY) return 0;
    count=n;
    for(unsigned i=0;i<n;i++) {
        float_input[i][0]=(float)input[i].x/65536.0f;
        float_input[i][1]=(float)input[i].y/65536.0f;
        float_input[i][2]=(float)input[i].z/65536.0f;
    }
    return 1;
}
/* Same quantized turn angles are supplied to the float reference's sin/cos.
 * Both models are Ry * Rx, row-major with column vectors. */
API("setup") void setup(unsigned ax,unsigned ay,float sx,float cx,float sy,float cy) {
    float_matrix[0][0]=cy; float_matrix[0][1]=sy*sx; float_matrix[0][2]=sy*cx;
    float_matrix[1][0]=0;  float_matrix[1][1]=cx;    float_matrix[1][2]=-sx;
    float_matrix[2][0]=-sy;float_matrix[2][1]=cy*sx; float_matrix[2][2]=cy*cx;
    sh2z_mat34_t x=sh2z_mat34_rotation_x((sh2z_angle_t)ax);
    sh2z_mat34_t y=sh2z_mat34_rotation_y((sh2z_angle_t)ay);
    sh2z_mat34_multiply(&fixed_matrix,&y,&x);
}
void before_range(unsigned start,unsigned n) {
    for(unsigned i=start;i<start+n;i++)
        for(unsigned j=0;j<3;j++)
            float_output[i][j]=float_matrix[j][0]*float_input[i][0]
                +float_matrix[j][1]*float_input[i][1]+float_matrix[j][2]*float_input[i][2];
}
void after_range(unsigned start,unsigned n) {
    (void)sh2z_mat34_transform_points(fixed_output+start,&fixed_matrix,input+start,n);
}
API("before") void before(void) { before_range(0,count); }
API("after") void after(void) { after_range(0,count); }

