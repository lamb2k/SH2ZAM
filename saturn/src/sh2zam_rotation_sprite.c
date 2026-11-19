/* Rotation sprite sets: validation, cell selection, placement and VDP1
 * distorted sprite emission. See sh2zam/rotation_sprite_experimental.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/rotation_sprite_experimental.h"

/* floor(sqrt(v)) for any uint64_t, bit by bit (no divide, no libc). */
static uint32_t rspr_isqrt64(uint64_t v) {
    uint64_t root=0,bit=(uint64_t)1<<62;
    while(bit>v)bit>>=2;
    while(bit) {
        if(v>=root+bit){v-=root+bit;root=(root>>1)+bit;}
        else root>>=1;
        bit>>=2;
    }
    return (uint32_t)root;
}

static uint32_t rspr_abs32(int32_t v) {
    return v<0?0u-(uint32_t)v:(uint32_t)v;
}

/* Truncating signed 64-bit division by a positive divisor (the compiler's
 * 64-bit division; a few calls per placed sprite). */
static int64_t rspr_div_trunc(int64_t n,int64_t d) {
    return n/d;
}

bool sh2z_rspr_valid(const sh2z_rspr_set_t *set) {
    if(!set||set->magic!=SH2Z_RSPR_MAGIC)return false;
    if(set->yaw_count<1||set->yaw_count>SH2Z_RSPR_MAX_YAW||
       set->pitch_count<1||set->pitch_count>SH2Z_RSPR_MAX_PITCH)return false;
    if(set->flags&~SH2Z_RSPR_MIRROR)return false;
    if(set->clut_count<1||set->clut_count>1024||(set->clut_offset&7u)||set->pixels_per_unit<=0)return false;
    const uint32_t cells=(uint32_t)set->yaw_count*set->pitch_count;
    const uint32_t head=(uint32_t)sizeof(*set)+cells*(uint32_t)sizeof(sh2z_rspr_cell_t);
    if(set->bytes<head||set->clut_offset<head||set->clut_offset>set->bytes||
       (uint32_t)set->clut_count*32u>set->bytes-set->clut_offset)return false;
    const sh2z_rspr_cell_t *cell=sh2z_rspr_cells(set);
    for(uint32_t i=0;i<cells;++i) {
        const sh2z_rspr_cell_t *c=cell+i;
        if(c->width<8||c->width>504||(c->width&7u)||c->height<1||c->height>255)return false;
        if((c->pixel_offset&7u)||c->pixel_offset<head||c->clut>=set->clut_count)return false;
        const uint32_t size=(uint32_t)c->width*c->height/2u;
        if(c->pixel_offset>set->bytes||size>set->bytes-c->pixel_offset)return false;
    }
    return true;
}

bool sh2z_rspr_select(sh2z_rspr_pick_t *out,const sh2z_rspr_set_t *set,
                      int32_t x,int32_t y,int32_t z) {
    if(!out||!sh2z_rspr_valid(set)||(x==0&&y==0&&z==0))return false;
    uint32_t largest=rspr_abs32(x);
    if(rspr_abs32(y)>largest)largest=rspr_abs32(y);
    if(rspr_abs32(z)>largest)largest=rspr_abs32(z);
    while(largest>=32768u){x>>=1;y>>=1;z>>=1;largest>>=1;}
    uint8_t mirror=0;
    if((set->flags&SH2Z_RSPR_MIRROR)&&x<0){x=-x;mirror=1;}
    /* |components| < 2^15 and |bins| <= 2^14: every product fits in int32. */
    unsigned yaw=0;int32_t best=0;
    for(unsigned k=0;k<set->yaw_count;++k) {
        const int32_t d=set->yaw[k][0]*x+set->yaw[k][1]*z;
        if(k==0||d>best){best=d;yaw=k;}
    }
    const int32_t h=best>>14;
    unsigned pitch=0;int32_t top=0;
    for(unsigned k=0;k<set->pitch_count;++k) {
        const int32_t d=set->pitch[k][0]*y+set->pitch[k][1]*h;
        if(k==0||d>top){top=d;pitch=k;}
    }
    out->cell=(uint16_t)(pitch*set->yaw_count+yaw);out->mirror=mirror;out->reserved=0;
    return true;
}

bool sh2z_rspr_place(sh2z_rspr_place_t *out,const sh2z_view_t *v,const sh2z_rspr_set_t *set) {
    if(!out||!v||!sh2z_rspr_valid(set)||v->t[2]<=v->near_z||v->t[2]<=0)return false;
    const int64_t qx=rspr_div_trunc((int64_t)v->focal_x*v->t[0],v->t[2]);
    const int64_t qy=rspr_div_trunc((int64_t)v->focal_y*v->t[1],v->t[2]);
    const int64_t x=(int64_t)v->center_x+(qx>>16),y=(int64_t)v->center_y-(qy>>16);
    if(x< -1023||x>1023||y< -1023||y>1023)return false;
    const int32_t ux=v->r[0][1],uy=v->r[1][1],uz=v->r[2][1];
    const uint32_t screen=rspr_isqrt64((uint64_t)((int64_t)ux*ux+(int64_t)uy*uy));
    if(screen==0)return false;
    const uint32_t full=rspr_isqrt64((uint64_t)((int64_t)ux*ux+(int64_t)uy*uy+(int64_t)uz*uz));
    /* Screen pixels per model unit at the origin, Q16.16: focal_y * full / t.z
     * (full / 16384 view units per model unit; t.z in Q18.14 view units). */
    const int64_t pixels=rspr_div_trunc((int64_t)v->focal_y*full,v->t[2]);
    const int64_t scale=rspr_div_trunc(pixels*65536,set->pixels_per_unit);
    if(scale<1||scale>((int64_t)1<<24))return false;
    out->x=(int16_t)x;out->y=(int16_t)y;out->scale=(int32_t)scale;
    out->up[0]=(int16_t)rspr_div_trunc((int64_t)16384*ux,screen);
    out->up[1]=(int16_t)rspr_div_trunc(-(int64_t)16384*uy,screen);
    return true;
}

static bool rspr_corner(int16_t *sx,int16_t *sy,const sh2z_rspr_place_t *p,int32_t px,int32_t py) {
    /* R = {-up.y, up.x}; point = scale * (px*R - py*up), Q16 * Q14 = Q30. */
    const int64_t ex=(int64_t)px*(-p->up[1])-(int64_t)py*p->up[0];
    const int64_t ey=(int64_t)px*p->up[0]-(int64_t)py*p->up[1];
    const int64_t half=(int64_t)1<<29;
    const int64_t dx=(ex*p->scale+half)>>30,dy=(ey*p->scale+half)>>30;
    const int64_t x=p->x+dx,y=p->y+dy;
    if(x< -1024||x>1023||y< -1024||y>1023)return false;
    *sx=(int16_t)x;*sy=(int16_t)y;return true;
}

bool sh2z_rspr_emit(uint16_t command[16],const sh2z_rspr_set_t *set,const sh2z_rspr_pick_t *pick,
                    const sh2z_rspr_place_t *place,uint32_t vram,uint16_t pmod) {
    if(!command||!pick||!place||!sh2z_rspr_valid(set)||(vram&7u)||vram>0x7ffffu)return false;
    if(pick->cell>=(uint32_t)set->yaw_count*set->pitch_count||pick->mirror>1)return false;
    if(place->scale<1||place->scale>(1<<24))return false;
    if(set->bytes>0x80000u-vram)return false;
    const sh2z_rspr_cell_t *c=sh2z_rspr_cells(set)+pick->cell;
    const int32_t left=-c->origin_x,right=(int32_t)c->width-c->origin_x;
    const int32_t top=-c->origin_y,bottom=(int32_t)c->height-c->origin_y;
    const int32_t sign=pick->mirror?-1:1;
    int16_t xy[8];
    if(!rspr_corner(&xy[0],&xy[1],place,sign*left,top)||
       !rspr_corner(&xy[2],&xy[3],place,sign*right,top)||
       !rspr_corner(&xy[4],&xy[5],place,sign*right,bottom)||
       !rspr_corner(&xy[6],&xy[7],place,sign*left,bottom))return false;
    command[0]=0x0002u;command[1]=0;command[2]=pmod;
    command[3]=(uint16_t)((vram+set->clut_offset+(uint32_t)c->clut*32u)>>3);
    command[4]=(uint16_t)((vram+c->pixel_offset)>>3);
    command[5]=(uint16_t)(((uint32_t)c->width/8u)<<8|c->height);
    for(unsigned i=0;i<8;++i)command[6+i]=(uint16_t)xy[i];
    command[14]=0;command[15]=0;
    return true;
}

/* floor(sqrt(v)) for v < 2^32, bit by bit in 32-bit arithmetic. */
static uint32_t rspr_isqrt32(uint32_t v) {
    uint32_t root=0,bit=UINT32_C(1)<<30;
    while(bit>v)bit>>=2;
    while(bit) {
        if(v>=root+bit){v-=root+bit;root=(root>>1)+bit;}
        else root>>=1;
        bit>>=2;
    }
    return root;
}

static int32_t rspr_dot16(const int16_t row[3],const int32_t d[3]) {
    const int64_t sum=(int64_t)row[0]*d[0]+(int64_t)row[1]*d[1]+(int64_t)row[2]*d[2];
    return (int32_t)(sum>>16);
}

bool sh2z_rspr_camera_scale(sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,int32_t unit_q14) {
    if(!c||!sh2z_rspr_valid(set)||unit_q14<=0||c->focal_y<=0)return false;
    c->scale_num=(int64_t)c->focal_y*unit_q14*65536/set->pixels_per_unit;
    return true;
}

bool sh2z_rspr_pose(sh2z_rspr_pick_t *pick,sh2z_rspr_place_t *place,int32_t *depth,
                    const sh2z_rspr_camera_t *c,const sh2z_rspr_set_t *set,
                    const int32_t pos[3],const int16_t basis[9]) {
    if(!pick||!place||!depth||!c||!set||!pos||!basis)return false;
    const int32_t d[3]={(int32_t)((uint32_t)pos[0]-(uint32_t)c->eye[0]),
                        (int32_t)((uint32_t)pos[1]-(uint32_t)c->eye[1]),
                        (int32_t)((uint32_t)pos[2]-(uint32_t)c->eye[2])};
    const int32_t tz=rspr_dot16(c->r[2],d);
    if(tz<=c->near_z||tz<=0)return false;
    sh2z_rspr_pick_t chosen;
    if(!sh2z_rspr_select(&chosen,set,-rspr_dot16(basis+6,d),-rspr_dot16(basis+3,d),-rspr_dot16(basis,d)))return false;
    const int32_t tx=rspr_dot16(c->r[0],d),ty=rspr_dot16(c->r[1],d);
    const int32_t qx=sh2z_divu_div64((int64_t)c->focal_x*tx,tz,NULL);
    const int32_t qy=sh2z_divu_div64((int64_t)c->focal_y*ty,tz,NULL);
    const int32_t x=(int32_t)c->center_x+(qx>>16),y=(int32_t)c->center_y-(qy>>16);
    if(x< -1023||x>1023||y< -1023||y>1023)return false;
    const int32_t scale=sh2z_divu_div64(c->scale_num,tz,NULL);
    if(scale<1||scale>(1<<24))return false;
    const int64_t ux64=(int64_t)c->r[0][0]*basis[3]+(int64_t)c->r[0][1]*basis[4]+(int64_t)c->r[0][2]*basis[5];
    const int64_t uy64=(int64_t)c->r[1][0]*basis[3]+(int64_t)c->r[1][1]*basis[4]+(int64_t)c->r[1][2]*basis[5];
    const int32_t ux=(int32_t)(ux64>>14),uy=(int32_t)(uy64>>14);
    const uint32_t len=rspr_isqrt32((uint32_t)(ux*ux)+(uint32_t)(uy*uy));
    if(len==0)return false;
    *pick=chosen;
    place->x=(int16_t)x;place->y=(int16_t)y;place->scale=scale;
    place->up[0]=(int16_t)sh2z_divu_div64((int64_t)16384*ux,(int32_t)len,NULL);
    place->up[1]=(int16_t)sh2z_divu_div64(-(int64_t)16384*uy,(int32_t)len,NULL);
    *depth=tz;
    return true;
}
