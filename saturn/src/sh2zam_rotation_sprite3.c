/* Rotation sprite sets, contract tag rotation-sprites-3: the yaw climb, the
 * world pose from camera-space terms and the emission of one cell, each with
 * output identical to its rotation-sprites-2 counterpart. See
 * sh2zam/rotation_sprite_experimental.h. A separate unit, so the
 * rotation-sprites-2 objects are unchanged.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/rotation_sprite_experimental.h"

static int rspr3_half(const int16_t v[2]) {
    return (v[1]>0||(v[1]==0&&v[0]>0))?0:1;
}

static int64_t rspr3_cross(const int16_t a[2],const int16_t b[2]) {
    return (int64_t)a[0]*b[1]-(int64_t)a[1]*b[0];
}

/* a before b in polar angle, counter-clockwise from +sin, in [0, 2 pi). */
static bool rspr3_before(const int16_t a[2],const int16_t b[2]) {
    const int ha=rspr3_half(a),hb=rspr3_half(b);
    return ha<hb||(ha==hb&&rspr3_cross(a,b)>0);
}

bool sh2z_rspr_yaw_convex(const sh2z_rspr_set_t *set) {
    if(!set||(set->flags&SH2Z_RSPR_MIRROR))return false;
    const unsigned n=set->yaw_count;
    if(n<3u||n>SH2Z_RSPR_MAX_YAW)return false;
    int sign=0;
    unsigned wraps=0;
    for(unsigned k=0;k<n;++k) {
        const unsigned k1=k+1u<n?k+1u:k+1u-n,k2=k+2u<n?k+2u:k+2u-n;
        const int16_t *a=set->yaw[k],*b=set->yaw[k1],*c=set->yaw[k2];
        const int64_t turn=(int64_t)(b[0]-a[0])*(c[1]-b[1])-(int64_t)(b[1]-a[1])*(c[0]-b[0]);
        const int64_t step=rspr3_cross(a,b);
        const int st=turn>0?1:turn<0?-1:0,ss=step>0?1:step<0?-1:0;
        if(!st||ss!=st||(sign&&st!=sign))return false;
        sign=st;
        if(sign>0?!rspr3_before(a,b):!rspr3_before(b,a))++wraps;
    }
    return wraps==1u;
}

unsigned sh2z_rspr_yaw_climb(const sh2z_rspr_set_t *set,unsigned start,int32_t x,int32_t z,int32_t *best) {
    const unsigned n=set->yaw_count;
    unsigned k=start<n?start:0u;
    int32_t fk=set->yaw[k][0]*x+set->yaw[k][1]*z;
    for(;;) {
        const unsigned kn=k+1u==n?0u:k+1u,kp=k==0u?n-1u:k-1u;
        const int32_t fn=set->yaw[kn][0]*x+set->yaw[kn][1]*z;
        if(fn>fk){k=kn;fk=fn;continue;}
        const int32_t fp=set->yaw[kp][0]*x+set->yaw[kp][1]*z;
        if(fp>fk){k=kp;fk=fp;continue;}
        if(fn==fk&&kn<k)k=kn;
        if(fp==fk&&kp<k)k=kp;
        break;
    }
    *best=fk;
    return k;
}

static uint32_t rspr3_abs(int32_t v) {
    return v<0?0u-(uint32_t)v:(uint32_t)v;
}

static int32_t rspr3_dot16(const int16_t row[3],const int32_t d[3]) {
    const int64_t sum=(int64_t)row[0]*d[0]+(int64_t)row[1]*d[1]+(int64_t)row[2]*d[2];
    return (int32_t)(sum>>16);
}

/* floor(sqrt(v)), bit by bit (as sh2z_rspr_pose). */
static uint32_t rspr3_isqrt32(uint32_t v) {
    uint32_t root=0,bit=UINT32_C(1)<<30;
    while(bit>v)bit>>=2;
    while(bit) {
        if(v>=root+bit){v-=root+bit;root=(root>>1)+bit;}
        else root>>=1;
        bit>>=2;
    }
    return root;
}

static void rspr3_div_start(sh2z_divu_t *t,int64_t n,int32_t d) {
    const uint64_t raw=(uint64_t)n;
    sh2z_divu_start64(t,(int32_t)(uint32_t)(raw>>32),(uint32_t)raw,d);
}

bool sh2z_rspr_pose_terms(sh2z_rspr_pick_t *pick,sh2z_rspr_place_t *place,int32_t *depth,
                          const sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,
                          const sh2z_rspr_terms_t *t,const int16_t basis[9],uint8_t *yaw_hint) {
    if(!pick||!place||!depth||!c||!set||!t||!basis)return false;
    const int32_t tz=t->tz;
    if(tz<=c->near_z||tz<=0)return false;
    sh2z_divu_t dv;
    rspr3_div_start(&dv,c->scale_num,tz);
    /* The selection (as rspr_select_core) while the divider works. */
    int32_t x=-rspr3_dot16(basis+6,t->d),y=-rspr3_dot16(basis+3,t->d),z=-rspr3_dot16(basis,t->d);
    if(x==0&&y==0&&z==0){(void)sh2z_divu_finish(&dv,NULL);return false;}
    uint32_t largest=rspr3_abs(x);
    if(rspr3_abs(y)>largest)largest=rspr3_abs(y);
    if(rspr3_abs(z)>largest)largest=rspr3_abs(z);
    while(largest>=32768u){x>>=1;y>>=1;z>>=1;largest>>=1;}
    uint8_t mirror=0;
    if((set->flags&SH2Z_RSPR_MIRROR)&&x<0){x=-x;mirror=1;}
    unsigned yaw=0,pitch=0;int32_t best=0,top=0;
    if(yaw_hint) {
        yaw=sh2z_rspr_yaw_climb(set,*yaw_hint,x,z,&best);
        *yaw_hint=(uint8_t)yaw;
    } else {
        for(unsigned k=0;k<set->yaw_count;++k) {
            const int32_t v=set->yaw[k][0]*x+set->yaw[k][1]*z;
            if(k==0||v>best){best=v;yaw=k;}
        }
    }
    const int32_t h=best>>14;
    for(unsigned k=0;k<set->pitch_count;++k) {
        const int32_t v=set->pitch[k][0]*y+set->pitch[k][1]*h;
        if(k==0||v>top){top=v;pitch=k;}
    }
    const int32_t scale=sh2z_divu_finish(&dv,NULL);
    rspr3_div_start(&dv,t->nx,tz);
    const int16_t *r0=c->r[0],*r1=c->r[1];
    const int64_t ux64=(int64_t)r0[0]*basis[3]+(int64_t)r0[1]*basis[4]+(int64_t)r0[2]*basis[5];
    const int64_t uy64=(int64_t)r1[0]*basis[3]+(int64_t)r1[1]*basis[4]+(int64_t)r1[2]*basis[5];
    const int32_t ux=(int32_t)(ux64>>14),uy=(int32_t)(uy64>>14);
    const int32_t qx=sh2z_divu_finish(&dv,NULL);
    rspr3_div_start(&dv,t->ny,tz);
    const uint32_t len=rspr3_isqrt32((uint32_t)(ux*ux)+(uint32_t)(uy*uy));
    const int32_t qy=sh2z_divu_finish(&dv,NULL);
    const int64_t ax=(int64_t)c->center_x*65536+qx,ay=(int64_t)c->center_y*65536-qy;
    const int32_t sx=(int32_t)(ax>>16),sy=(int32_t)(ay>>16);
    if(sx< -1023||sx>1023||sy< -1023||sy>1023)return false;
    if(scale<1||scale>(1<<24)||len==0)return false;
    rspr3_div_start(&dv,(int64_t)16384*ux,(int32_t)len);
    const int32_t u0=sh2z_divu_finish(&dv,NULL);
    rspr3_div_start(&dv,-(int64_t)16384*uy,(int32_t)len);
    const int32_t u1=sh2z_divu_finish(&dv,NULL);
    pick->cell=(uint16_t)(pitch*set->yaw_count+yaw);pick->mirror=mirror;pick->reserved=0;
    place->x=(int16_t)sx;place->y=(int16_t)sy;place->scale=scale;
    place->frac[0]=(uint16_t)(ax&0xffff);place->frac[1]=(uint16_t)(ay&0xffff);
    place->up[0]=(int16_t)u0;place->up[1]=(int16_t)u1;
    *depth=tz;
    return true;
}

/* frac * 2^14 + scale * e + 2^29, shifted right 30: |e| < 2^26 and
 * scale <= 2^24, so the product is one widening multiply and the result fits
 * 21 bits; the shift is taken from the two halves (no 64-bit shift call). */
static int32_t rspr3_axis(int32_t e,int32_t scale,uint32_t frac) {
    const int64_t v=(int64_t)e*scale+(int64_t)(frac<<14)+((int64_t)1<<29);
    const uint64_t u=(uint64_t)v;
    return (int32_t)(((uint32_t)(u>>32)<<2)|((uint32_t)u>>30));
}

bool sh2z_rspr_emit_cell(uint16_t command[16],const sh2z_rspr_cell_t *cell,uint32_t mirror,
                         const sh2z_rspr_place_t *place,uint16_t pmod,uint16_t colr,uint16_t srca) {
    if(!command||!cell||!place||mirror>1u)return false;
    /* Lookup-table mode: the table address must be a multiple of 32 bytes. */
    if(((pmod>>3)&7u)==1u&&(colr&3u))return false;
    const int32_t scale=place->scale;
    if(scale<1||scale>(1<<24))return false;
    int32_t l=-(int32_t)cell->origin_x,r=(int32_t)cell->width-cell->origin_x;
    if(mirror){l=-l;r=-r;}
    const int32_t tp=-(int32_t)cell->origin_y,bt=(int32_t)cell->height-cell->origin_y;
    const int32_t u0=place->up[0],u1=place->up[1];
    /* Corner (px, py): x offset px*(-up.y) - py*up.x, y offset px*up.x - py*up.y. */
    const int32_t lx=-l*u1,rx=-r*u1,ly=l*u0,ry=r*u0;
    const int32_t tx=tp*u0,bx=bt*u0,ty=tp*u1,by=bt*u1;
    const uint32_t f0=place->frac[0],f1=place->frac[1];
    const int32_t x0=place->x,y0=place->y;
    const int32_t xa=x0+rspr3_axis(lx-tx,scale,f0),ya=y0+rspr3_axis(ly-ty,scale,f1);
    const int32_t xb=x0+rspr3_axis(rx-tx,scale,f0),yb=y0+rspr3_axis(ry-ty,scale,f1);
    const int32_t xc=x0+rspr3_axis(rx-bx,scale,f0),yc=y0+rspr3_axis(ry-by,scale,f1);
    const int32_t xd=x0+rspr3_axis(lx-bx,scale,f0),yd=y0+rspr3_axis(ly-by,scale,f1);
    /* -1024..1023 on every coordinate: one unsigned compare each after the bias. */
    if((uint32_t)(xa+1024)>2047u||(uint32_t)(ya+1024)>2047u||(uint32_t)(xb+1024)>2047u||
       (uint32_t)(yb+1024)>2047u||(uint32_t)(xc+1024)>2047u||(uint32_t)(yc+1024)>2047u||
       (uint32_t)(xd+1024)>2047u||(uint32_t)(yd+1024)>2047u)return false;
    command[0]=0x0002u;command[1]=0;command[2]=pmod;command[3]=colr;command[4]=srca;
    command[5]=(uint16_t)(((uint32_t)cell->width/8u)<<8|cell->height);
    command[6]=(uint16_t)xa;command[7]=(uint16_t)ya;command[8]=(uint16_t)xb;command[9]=(uint16_t)yb;
    command[10]=(uint16_t)xc;command[11]=(uint16_t)yc;command[12]=(uint16_t)xd;command[13]=(uint16_t)yd;
    command[14]=0;command[15]=0;
    return true;
}
