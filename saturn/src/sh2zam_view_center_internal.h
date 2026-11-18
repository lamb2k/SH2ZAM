/* Private to sh2zam_view_center.c and sh2zam_view_rails.c: the view-center-1
 * projection rule (Q16 centre, final-coordinate truncation) and its validity
 * check. Include after sh2zam_view_internal.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#ifndef SH2ZAM_VIEW_CENTER_INTERNAL_H
#define SH2ZAM_VIEW_CENTER_INTERNAL_H

/* Clamp to +/-(2^31 - 1), view-round-1's symmetric quotient range. */
static int32_t sat31(int64_t a) {
    return a<-INT32_MAX?-INT32_MAX:a>INT32_MAX?INT32_MAX:(int32_t)a;
}
/* trunc((c + q) / 65536) for the centre c = 65536 * whole + frac (whole is
 * view.center_x of a valid centre view) and any int32 q, in 32-bit
 * arithmetic: the low halves are added apart, so the carry joins the floor
 * of the high halves and the 33-bit sum is never formed. The floor is in
 * -65536 .. 65536. */
static SH2ZI_ALWAYS_INLINE int16_t center_sx(int32_t whole,int32_t c,int32_t q) {
    const uint32_t low=((uint32_t)c&0xFFFFu)+((uint32_t)q&0xFFFFu);
    int32_t f=whole+shift16(q)+(int32_t)(low>>16);
    if(f<0 && (low&0xFFFFu))++f;
    return sat16(f);
}
/* trunc((c - q) / 65536) likewise: the borrow of the low halves leaves the
 * floor one lower, and the fraction is non-zero when the halves differ. */
static SH2ZI_ALWAYS_INLINE int16_t center_sy(int32_t whole,int32_t c,int32_t q) {
    const uint32_t cl=(uint32_t)c&0xFFFFu,ql=(uint32_t)q&0xFFFFu;
    int32_t f=whole-shift16(q)-(cl<ql?1:0);
    if(f<0 && cl!=ql)++f;
    return sat16(f);
}
static bool center_valid(const sh2z_view_center_t *w) {
    return w && view_valid(&w->view) && w->view.center_x==shift16(w->center_x16) &&
        w->view.center_y==shift16(w->center_y16);
}
static SH2ZI_ALWAYS_INLINE uint8_t classify(const sh2z_view_t *c,int16_t sx,int16_t sy,int32_t z) {
    unsigned code=0;
    if(z>c->far_z)code|=SH2Z_OUT_FAR;
    if(sx<c->clip[0])code|=SH2Z_OUT_LEFT;
    if(sx>c->clip[2])code|=SH2Z_OUT_RIGHT;
    if(sy<c->clip[1])code|=SH2Z_OUT_TOP;
    if(sy>c->clip[3])code|=SH2Z_OUT_BOTTOM;
    return (uint8_t)code;
}
static SH2ZI_ALWAYS_INLINE sh2z_view_vertex_t project_point(const sh2z_view_center_t *c,int32_t x,int32_t y,int32_t z) {
    sh2z_view_vertex_t o={x,y,z,0,0,0,{0,0,0}};
    if(z<c->view.near_z) {
        o.outcode=SH2Z_OUT_NEAR;
        return o;
    }
    o.sx=center_sx(c->view.center_x,c->center_x16,sat31((int64_t)c->view.focal_x*x/z));
    o.sy=center_sy(c->view.center_y,c->center_y16,sat31((int64_t)c->view.focal_y*y/z));
    o.outcode=classify(&c->view,o.sx,o.sy,z);
    return o;
}

#endif
