/* SH2ZAM bounds and caller-owned matrix stack. MIT; (c) 2026 SH2ZAM Contributors. */
#include "sh2zam_internal.h"

sh2z_aabb_t sh2z_aabb(sh2z_vec3_t a,sh2z_vec3_t b) {
    return (sh2z_aabb_t){{sh2z_min(a.x,b.x),sh2z_min(a.y,b.y),sh2z_min(a.z,b.z)},
                        {sh2z_max(a.x,b.x),sh2z_max(a.y,b.y),sh2z_max(a.z,b.z)}};
}
static bool valid_box(sh2z_aabb_t b) { return b.min.x<=b.max.x&&b.min.y<=b.max.y&&b.min.z<=b.max.z; }
bool sh2z_aabb_contains(sh2z_aabb_t b,sh2z_vec3_t p) {
    return valid_box(b)&&p.x>=b.min.x&&p.x<=b.max.x&&p.y>=b.min.y&&p.y<=b.max.y&&p.z>=b.min.z&&p.z<=b.max.z;
}
bool sh2z_aabb_intersects(sh2z_aabb_t a,sh2z_aabb_t b) {
    return valid_box(a)&&valid_box(b)&&a.min.x<=b.max.x&&b.min.x<=a.max.x&&
        a.min.y<=b.max.y&&b.min.y<=a.max.y&&a.min.z<=b.max.z&&b.min.z<=a.max.z;
}
static bool within_radius(const int64_t delta[3],uint64_t radius) {
    uint64_t remaining=radius*radius;
    for(unsigned i=0;i<3;i++) {
        uint64_t d=(uint64_t)(delta[i]<0?-delta[i]:delta[i]);
        if(d>radius) return false;
        uint64_t square=d*d;
        if(square>remaining) return false;
        remaining-=square;
    }
    return true;
}
bool sh2z_sphere_intersects(sh2z_sphere_t a,sh2z_sphere_t b) {
    if(a.radius<0||b.radius<0) return false;
    int64_t d[]={(int64_t)a.center.x-b.center.x,(int64_t)a.center.y-b.center.y,(int64_t)a.center.z-b.center.z};
    return within_radius(d,(uint64_t)(uint32_t)a.radius+(uint32_t)b.radius);
}
bool sh2z_sphere_intersects_aabb(sh2z_sphere_t s,sh2z_aabb_t b) {
    if(s.radius<0||!valid_box(b)) return false;
    int64_t d[]={(int64_t)s.center.x-sh2z_clamp(s.center.x,b.min.x,b.max.x),
                 (int64_t)s.center.y-sh2z_clamp(s.center.y,b.min.y,b.max.y),
                 (int64_t)s.center.z-sh2z_clamp(s.center.z,b.min.z,b.max.z)};
    return within_radius(d,(uint32_t)s.radius);
}
const sh2z_mat34_t *sh2z_stack_top(const sh2z_stack_t *s) {
    if(!s||!s->storage||!s->size||s->size>s->capacity) return NULL;
    return &s->storage[s->size-1];
}
bool sh2z_stack_init(sh2z_stack_t *s,sh2z_mat34_t *storage,size_t capacity) {
    if(!s) return false;
    *s=(sh2z_stack_t){NULL,0,0};
    if(!storage||!capacity) return false;
    storage[0]=sh2z_mat34_identity();
    *s=(sh2z_stack_t){storage,1,capacity}; return true;
}
bool sh2z_stack_push(sh2z_stack_t *s) {
    if(!sh2z_stack_top(s)||s->size==s->capacity) return false;
    s->storage[s->size]=s->storage[s->size-1]; ++s->size; return true;
}
bool sh2z_stack_pop(sh2z_stack_t *s) {
    if(!sh2z_stack_top(s)||s->size<=1) return false;
    --s->size; return true;
}
bool sh2z_stack_multiply(sh2z_stack_t *s,const sh2z_mat34_t *m) {
    if(!sh2z_stack_top(s)||!m) return false;
    sh2z_mat34_multiply(&s->storage[s->size-1],&s->storage[s->size-1],m); return true;
}
