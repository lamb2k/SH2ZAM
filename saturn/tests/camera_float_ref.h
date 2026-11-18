/* Reference float camera pipeline for the view-camera-1 accuracy study and
 * the model's cost control. MIT; SH2ZAM Contributors.
 *
 * This is the float path a view from a camera replaces, written out from its
 * rules in binary32 with every operation in source order:
 *   - a 4096-entry binary32 sine table built from a 13-term Taylor series
 *     with pi ~ 3.1415926535 (in binary64), tan(a) = sin(a) / cos(a);
 *   - an orthonormalisation around a forward vector (the camera basis);
 *   - a row-vector look-at matrix (eye, at, up) and an off-axis frustum
 *     (fov in degrees, half angle rounded to the table, fov scale and
 *     frustum centre), each also packed to a 16.16 integer matrix, and their
 *     product;
 *   - a float-to-fixed view adapter (Q2.30 rotation, Q16.16 focal and centre,
 *     a 224-line crop of 8 lines) and the Q1.14 centre view built from it
 *     with a Q16.16 eye, eye * 65536 truncated.
 * Freestanding: no libc or libm (sqrtf is computed here, correctly rounded,
 * with 32-bit integer steps), so the SH-2 bridge compiles the same code. */
#ifndef SH2ZAM_CAMERA_FLOAT_REF_H
#define SH2ZAM_CAMERA_FLOAT_REF_H

#include "sh2zam/view_experimental.h"

typedef struct { float m[4][4]; } cf_mtxf_t;
typedef struct { int16_t i[4][4]; uint16_t f[4][4]; } cf_mtx_t;

static inline uint32_t cf_bits(float x) { union { float f; uint32_t u; } v; v.f=x; return v.u; }
static inline float cf_float(uint32_t u) { union { float f; uint32_t u; } v; v.u=u; return v.f; }

/* Correctly rounded square root for finite x >= 0 (others returned as is). */
static inline float cf_sqrtf(float x) {
    const uint32_t b=cf_bits(x);
    if((b&0x7fffffffu)==0u || (b>>31) || (b&0x7f800000u)==0x7f800000u)return x;
    uint32_t m=b&0x7fffffu;
    int32_t e=(int32_t)(b>>23);
    if(e==0) { e=1; while(!(m&0x800000u)) { m<<=1; --e; } }
    else m|=0x800000u;
    int32_t E=e-150;                       /* x = m * 2^E, 2^23 <= m < 2^24 */
    if(E&1) { m<<=1; --E; }                /* now 2^23 <= m < 2^25 */
    const unsigned k=m<(1u<<24)?12u:11u;   /* N = m * 2^(2k), 2^46 <= N < 2^48 */
    /* N as 24 base-4 digits, high first: hi holds bits 47..32, lo bits 31..0. */
    const uint32_t hi=k==12u?(m>>8):(m>>10),lo=k==12u?(m<<24):(m<<22);
    uint32_t root=0,rem=0;
    for(unsigned d=0;d<24;++d) {
        const unsigned shift=46u-2u*d;     /* bit position of this digit */
        const uint32_t digit=shift>=32u?(hi>>(shift-32u))&3u:(lo>>shift)&3u;
        rem=(rem<<2)|digit;
        const uint32_t trial=(root<<2)|1u;
        if(rem>=trial) { rem-=trial; root=(root<<1)|1u; } else root<<=1;
    }
    /* sqrt(N) is never exactly halfway; round up when N > root^2 + root. */
    if(rem>root)++root;
    int32_t exp=E/2-(int32_t)k+23+127;
    if(root==(1u<<24)) { root>>=1; ++exp; }
    return cf_float(((uint32_t)exp<<23)|(root&0x7fffffu));
}

static inline int32_t cf_round(float x) { return x<0.0f?(int32_t)(x-0.5f):(int32_t)(x+0.5f); }

/* The start-up sine table (binary64 series, stored as binary32). */
static inline void cf_sin_table_init(float t[4096]) {
    for(int i=0;i<4096;++i) {
        double x=(i*(2*3.1415926535))/4096;
        double numerator=-x*x*x,minus_square=-x*x,denominator=1*2*3;
        for(int j=2;j<14;++j) {
            x+=numerator/denominator;
            numerator*=minus_square;
            denominator*=j*((j<<2)+2);
        }
        t[i]=(float)x;
    }
}

/* Basis rows x (forward on input, unnormalised), y (up), z (output). */
static inline int cf_orthonormalize_forward(float b[3][3]) {
    float s=b[0][0]*b[0][0]+b[0][1]*b[0][1]+b[0][2]*b[0][2];
    if(s==0.0f)return -1;
    s=1.0f/cf_sqrtf(s);
    b[0][0]*=s;b[0][1]*=s;b[0][2]*=s;
    b[2][0]=b[1][1]*b[0][2]-b[1][2]*b[0][1];
    b[2][1]=b[1][2]*b[0][0]-b[1][0]*b[0][2];
    b[2][2]=b[1][0]*b[0][1]-b[1][1]*b[0][0];
    s=b[2][0]*b[2][0]+b[2][1]*b[2][1]+b[2][2]*b[2][2];
    if(s==0.0f)return -1;
    s=1.0f/cf_sqrtf(s);
    b[2][0]*=s;b[2][1]*=s;b[2][2]*=s;
    b[1][0]=b[0][1]*b[2][2]-b[0][2]*b[2][1];
    b[1][1]=b[0][2]*b[2][0]-b[0][0]*b[2][2];
    b[1][2]=b[0][0]*b[2][1]-b[0][1]*b[2][0];
    return 0;
}

/* Pack to 16.16 integer halves, rounding half away from zero. A packed
 * element holds less than 32768 in magnitude: a look-at translation beyond
 * that (an eye at the edge of an int16 world) does not fit, and the source's
 * conversion is then undefined. Here it saturates; nothing reads the packed
 * matrices, they are built for cost only. */
static inline void cf_to_mtx(const cf_mtxf_t *src,cf_mtx_t *dst) {
    for(int i=3;i>=0;--i)for(int j=3;j>=0;--j) {
        const float v=src->m[i][j]*65536.0f;
        const uint32_t b=cf_bits(v);   /* |v| < 2^30 (and not NaN) by its bits: no float compare */
        const int32_t n=(b&0x7fffffffu)>=0x4e800000u?((b>>31)?INT32_MIN:INT32_MAX):
            v<0.0f?(int32_t)(v-0.5f):(int32_t)(v+0.5f);
        dst->i[i][j]=(int16_t)(n>>16);
        dst->f[i][j]=(uint16_t)n;
    }
}

static inline void cf_look_at(cf_mtx_t *mtx,cf_mtxf_t *m,float eyeX,float eyeY,float eyeZ,
    float atX,float atY,float atZ,float upX,float upY,float upZ) {
    const float lookX=eyeX-atX,lookY=eyeY-atY,lookZ=eyeZ-atZ;
    float f=lookX*lookX+lookY*lookY+lookZ*lookZ;
    if(f<=0.0f)return;
    f=1.0f/cf_sqrtf(f);
    m->m[3][2]=-eyeX*(m->m[0][2]=lookX*f)-eyeY*(m->m[1][2]=lookY*f)-eyeZ*(m->m[2][2]=lookZ*f);
    const float nX=(upY*lookZ)-(upZ*lookY),nY=(upZ*lookX)-(upX*lookZ),nZ=(upX*lookY)-(upY*lookX);
    f=nX*nX+nY*nY+nZ*nZ;
    if(f<=0.0f)return;
    f=1.0f/cf_sqrtf(f);
    m->m[3][0]=-eyeX*(m->m[0][0]=nX*f)-eyeY*(m->m[1][0]=nY*f)-eyeZ*(m->m[2][0]=nZ*f);
    const float uX=(lookY*nZ)-(lookZ*nY),uY=(lookZ*nX)-(lookX*nZ),uZ=(lookX*nY)-(lookY*nX);
    f=uX*uX+uY*uY+uZ*uZ;
    if(f<=0.0f)return;
    f=1.0f/cf_sqrtf(f);
    m->m[3][1]=-eyeX*(m->m[0][1]=uX*f)-eyeY*(m->m[1][1]=uY*f)-eyeZ*(m->m[2][1]=uZ*f);
    m->m[0][3]=0.0f;m->m[1][3]=0.0f;m->m[2][3]=0.0f;m->m[3][3]=1.0f;
    cf_to_mtx(m,mtx);
}

static inline void cf_frustum(cf_mtx_t *mtx,cf_mtxf_t *m,const float *sin_table,float fov,float near,float far,
    float fovScaleX,float centerX,float fovScaleY,float centerY,uint16_t *perspective) {
    int32_t half=cf_round(fov*((0x1000/360.0f)/2));
    float radius=(sin_table[half&0xFFF]/sin_table[(half+0x400)&0xFFF])*near;
    centerX*=(2.0f*radius)/fovScaleX;
    const float right=centerX+radius,left=centerX-radius;
    half=cf_round(fov*((0x1000/360.0f)/2));
    radius*=fovScaleY/fovScaleX;
    centerY*=(2.0f*radius)/fovScaleY;
    const float top=centerY+radius,bottom=centerY-radius;
    m->m[0][0]=2.0f*near*(1.0f/(right-left));
    m->m[2][0]=(right+left)*(1.0f/(right-left));
    radius*=fovScaleY/fovScaleX;
    m->m[1][1]=2.0f*near*(1.0f/(top-bottom));
    m->m[2][1]=(top+bottom)*(1.0f/(top-bottom));
    m->m[3][2]=2.0f*near*far*(1.0f/(near-far));
    m->m[2][2]=(near+far)*(1.0f/(near-far));
    if((near+far)>2.0f) {
        const uint16_t scale=(uint16_t)(int32_t)(0x20000/(near+far));
        *perspective=(uint16_t)(scale?scale:1u);
    } else *perspective=0xFFFFu;
    m->m[0][1]=m->m[0][2]=m->m[0][3]=m->m[1][0]=m->m[1][2]=m->m[1][3]=m->m[3][0]=m->m[3][1]=m->m[3][3]=0.0f;
    m->m[2][3]=-1.0f;
    cf_to_mtx(m,mtx);
    (void)radius;
}

/* Projection-view product of the row-vector matrices (sparse frustum). */
static inline void cf_projection_view(cf_mtxf_t *pv,const cf_mtxf_t *p,const cf_mtxf_t *v) {
    for(int i=0;i<4;++i) {
        pv->m[i][0]=(p->m[0][0]*v->m[i][0])+(p->m[2][0]*v->m[i][2]);
        pv->m[i][1]=(p->m[1][1]*v->m[i][1])+(p->m[2][1]*v->m[i][2]);
        pv->m[i][2]=p->m[2][2]*v->m[i][2];
        pv->m[i][3]=-v->m[i][2];
    }
    pv->m[3][2]=(p->m[2][2]*v->m[3][2])+p->m[3][2];
}

/* Float-to-fixed adapter output: Q2.30 rotation, Q20.12 translation, Q16.16
 * focal and centre, Q20.12 planes, half-open crop rectangle. */
typedef struct {
    int32_t rotation[3][3],translation[3];
    int32_t focal[2],center[2],near_plane,far_plane;
    int16_t clip[4];
} cf_view_t;

static inline int cf_finite(float x) { return (cf_bits(x)&0x7f800000u)!=0x7f800000u; }
static inline int cf_fixed(float x,float scale,float bound,int32_t *out) {
    if(!cf_finite(x) || x<-bound || x>bound)return 0;
    *out=(int32_t)(x*scale);
    return 1;
}
static inline int cf_view_from(cf_view_t *out,const float v[4][4],const float p[4][4],const float vp[4],
    const float sc[4],float near_plane,float far_plane) {
    cf_view_t a;
    for(unsigned i=0;i<4;++i)for(unsigned j=0;j<4;++j)
        if(!cf_finite(v[i][j]) || !cf_finite(p[i][j]))return 0;
    if(v[0][3]!=0 || v[1][3]!=0 || v[2][3]!=0 || v[3][3]!=1 ||
       p[0][1]!=0 || p[0][2]!=0 || p[0][3]!=0 || p[1][0]!=0 ||
       p[1][2]!=0 || p[1][3]!=0 || p[2][3]!=-1 || p[3][0]!=0 ||
       p[3][1]!=0 || p[3][3]!=0 || p[0][0]<=0 || p[1][1]<=0)return 0;
    for(unsigned row=0;row<3;++row) {
        const float sign=row==2?-1.0f:1.0f;
        for(unsigned col=0;col<3;++col)
            if(!cf_fixed(sign*v[col][row],1073741824.0f,1.01f,&a.rotation[row][col]))return 0;
        if(!cf_fixed(sign*v[3][row],4096.0f,131072.0f,&a.translation[row]))return 0;
    }
    for(unsigned i=0;i<4;++i)if(!cf_finite(vp[i]) || !cf_finite(sc[i]))return 0;
    if(vp[0]<=0 || vp[1]<=0 || vp[0]>320 || vp[1]>240)return 0;
    if(!cf_fixed(vp[0]*p[0][0],65536.0f,4096.0f,&a.focal[0]) ||
       !cf_fixed(vp[1]*p[1][1],65536.0f,4096.0f,&a.focal[1]) ||
       !cf_fixed(vp[2]-vp[0]*p[2][0],65536.0f,4096.0f,&a.center[0]) ||
       !cf_fixed(vp[3]-8.0f+vp[1]*p[2][1],65536.0f,4096.0f,&a.center[1]) ||
       !cf_fixed(near_plane,4096.0f,131072.0f,&a.near_plane) ||
       !cf_fixed(far_plane,4096.0f,131072.0f,&a.far_plane))return 0;
    if(a.near_plane<=0 || a.far_plane<=a.near_plane)return 0;
    const float crop[4]={sc[0],sc[1]-8.0f,sc[2],sc[3]-8.0f};
    for(unsigned i=0;i<4;++i) {
        if(crop[i]<-240 || crop[i]>640 || (float)(int)crop[i]!=crop[i])return 0;
        int n=(int)crop[i],limit=(i&1u)?224:320;
        if(n<0)n=0;
        if(n>limit)n=limit;
        a.clip[i]=(int16_t)n;
    }
    if(a.clip[0]>=a.clip[2] || a.clip[1]>=a.clip[3])return 0;
    *out=a;
    return 1;
}

/* The centre view from the adapter output and a Q16.16 eye, through the
 * library as a caller does today: rotation rounded half away from zero to
 * Q1.14, inclusive clip, sh2z_view_place, sh2z_view_center_init. */
static inline int cf_view16_center(sh2z_view_center_t *out,const cf_view_t *view,const int32_t eye[3]) {
    if(view->near_plane<=0 || view->far_plane<=view->near_plane || view->far_plane>INT32_MAX/4 ||
       view->clip[0]>=view->clip[2] || view->clip[1]>=view->clip[3])return 0;
    sh2z_view_t next={{{0}},{0},0,0,0,0,0,0,{0}};
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j) {
        const int32_t n=view->rotation[i][j];
        const int32_t q=n>=0?(int32_t)(((uint32_t)n+32768u)>>16):-(int32_t)((((uint32_t)0-(uint32_t)n)+32768u)>>16);
        if(q<INT16_MIN || q>INT16_MAX)return 0;
        next.r[i][j]=(int16_t)q;
    }
    next.focal_x=view->focal[0];next.focal_y=view->focal[1];
    next.near_z=view->near_plane*4;next.far_z=view->far_plane*4;
    for(unsigned i=0;i<4;++i)next.clip[i]=(int16_t)(view->clip[i]-(i>=2));
    const sh2z_vec3_t position={eye[0],eye[1],eye[2]};
    if(!sh2z_view_place(&next,position))return 0;
    return sh2z_view_center_init(out,&next,view->center[0],view->center[1]);
}

/* Camera inputs of the float chain, in source units (floats). */
typedef struct {
    float eye[3],at[3],up[3];
    float fov,near_plane,far_plane;
    float fov_scale_x,fov_scale_y,frustum_x,frustum_y;
    float viewport[4];   /* scale x, y, translate x, y */
    float scissor[4];    /* left, top, right, bottom */
} cf_camera_t;

/* Matrices of one frame, as the float chain leaves them. */
typedef struct {
    float basis[3][3];
    cf_mtxf_t view,projection,projection_view;
    cf_mtx_t view_fixed,projection_fixed;
    uint16_t perspective;
} cf_frame_t;

/* The per-frame chain a view from a camera replaces: frustum, look-at with
 * the basis's up row, their product, the float-to-fixed adapter and the
 * centre view with eye * 65536 truncated. The basis (orthonormalised from
 * at - eye and up) is camera state and is computed by the caller.
 * Returns 1 and writes *out on success. */
static inline int cf_chain(sh2z_view_center_t *out,cf_frame_t *f,const cf_camera_t *c,const float *sin_table) {
    cf_frustum(&f->projection_fixed,&f->projection,sin_table,c->fov,c->near_plane,c->far_plane,
        c->fov_scale_x,c->frustum_x,c->fov_scale_y,c->frustum_y,&f->perspective);
    cf_look_at(&f->view_fixed,&f->view,c->eye[0],c->eye[1],c->eye[2],c->at[0],c->at[1],c->at[2],
        f->basis[1][0],f->basis[1][1],f->basis[1][2]);
    cf_projection_view(&f->projection_view,&f->projection,&f->view);
    cf_view_t v;
    if(!cf_view_from(&v,(const float(*)[4])f->view.m,(const float(*)[4])f->projection.m,
        c->viewport,c->scissor,c->near_plane,c->far_plane))return 0;
    int32_t eye[3];
    for(unsigned i=0;i<3;++i) {
        const float e=c->eye[i]*65536.0f;
        if(!(e>-2147483648.0f && e<2147483648.0f))return 0;   /* the Q16.16 eye overflows */
        eye[i]=(int32_t)e;
    }
    return cf_view16_center(out,&v,eye);
}

#endif
