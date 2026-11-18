/* Private to sh2zam_view.c and sh2zam_view_round.c: the edge stepper and the
 * guard clip, instantiated once per projection rule. Before including, define
 *     SH2ZI_VIEW_POINT       one-point projector (project_fn)
 *     SH2ZI_VIEW_POINT_DIVU  the same through the hardware divider
 *     SH2ZI_VIEW_RANGE       in-place range projector (out, v, in, count)
 *     SH2ZI_VIEW_RANGE_DIVU  the same through the hardware divider
 * (the _DIVU forms are used only when SH2Z_DIVU_NATIVE). Include once per
 * file, after sh2zam_view_internal.h.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */

static bool edge_steps(sh2z_view_vertex_t *out,const sh2z_view_t *v,
    const sh2z_view_vertex_t *from,const sh2z_view_vertex_t *to,
    uint16_t first_num,uint16_t step_num,uint16_t den,size_t count,bool divu) {
    if(!view_valid(v))return false;
    if(!count)return true;
    if(!out || !from || !to || !den || first_num>den)return false;
    if(step_num && (count-1)>(size_t)((uint32_t)(den-first_num)/step_num))return false;
    if(count>SIZE_MAX/sizeof(*out))return false;
    /* from and to are fully read here, before out is written. */
    edge_axis_t ex=edge_axis(from->x,to->x,first_num,step_num,den,divu);
    edge_axis_t ey=edge_axis(from->y,to->y,first_num,step_num,den,divu);
    edge_axis_t ez=edge_axis(from->z,to->z,first_num,step_num,den,divu);
    for(size_t k=0;k<count;++k) {
        out[k].x=edge_next(&ex);out[k].y=edge_next(&ey);out[k].z=edge_next(&ez);
    }
#if SH2Z_DIVU_NATIVE
    if(divu) {
        SH2ZI_VIEW_RANGE_DIVU(out,v,out,count);
        return true;
    }
#else
    (void)divu;
#endif
    SH2ZI_VIEW_RANGE(out,v,out,count);
    return true;
}

/* ---- view-guard-1: guard-band polygon clip ------------------------------ */
static int clip_guard(sh2z_view_guard_vertex_t *out,const sh2z_view_t *v,const int16_t guard[4],
    const sh2z_view_guard_vertex_t *in,unsigned count,unsigned attrs,bool divu) {
    if(!view_valid(v) || !out || !guard || !in || count<3u || count>SH2Z_VIEW_GUARD_MAX_IN ||
       attrs>SH2Z_VIEW_GUARD_ATTRS)return SH2Z_VIEW_GUARD_INVALID;
    const int16_t gl=guard[0],gt=guard[1],gr=guard[2],gb=guard[3];
    if(gl>gr || gt>gb || !guard_edge_ok(gl,v->center_x) || !guard_edge_ok(gr,v->center_x) ||
       !guard_edge_ok(gt,v->center_y) || !guard_edge_ok(gb,v->center_y))return SH2Z_VIEW_GUARD_INVALID;
    const guard_planes_t g={v->near_z,v->far_z,v->focal_x,v->focal_y,
        (gl-v->center_x)*65536,(gr-v->center_x)*65536,(v->center_y-gt)*65536,(v->center_y-gb)*65536};
    const unsigned coords=3u+attrs;
    guard_point_t buf[2][SH2Z_VIEW_GUARD_MAX_OUT];
    for(unsigned k=0;k<count;++k) {
        buf[0][k].c[0]=in[k].v.x;buf[0][k].c[1]=in[k].v.y;buf[0][k].c[2]=in[k].v.z;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)buf[0][k].c[3+i]=i<attrs?in[k].attr[i]:0;
    }
    int n=(int)count;
    unsigned cur=0;
    for(unsigned plane=0;plane<6u;++plane) {
        /* A pass with every corner inside is the identity, and one with none
         * inside empties the polygon: neither needs the copy. */
        int64_t d[SH2Z_VIEW_GUARD_MAX_OUT];
        const unsigned inside=guard_distances(d,buf[cur],(unsigned)n,&g,plane);
        if(inside==(unsigned)n)continue;
        if(!inside){n=0;break;}
        n=guard_pass(buf[cur^1u],buf[cur],(unsigned)n,d,plane,coords,divu);
        if(n<0)return SH2Z_VIEW_GUARD_OVERFLOW;
        cur^=1u;
    }
    const sh2z_view_t c=*v;
    for(int k=0;k<n;++k) {
        const guard_point_t *p=&buf[cur][k];
        /* Out-of-line projection: a few calls per polygon, and it keeps the
         * clipper small enough for the benchmark image's RAM region. */
#if SH2Z_DIVU_NATIVE
        sh2z_view_vertex_t q=divu?SH2ZI_VIEW_POINT_DIVU(&c,p->c[0],p->c[1],p->c[2]):SH2ZI_VIEW_POINT(&c,p->c[0],p->c[1],p->c[2]);
#else
        sh2z_view_vertex_t q=SH2ZI_VIEW_POINT(&c,p->c[0],p->c[1],p->c[2]);
#endif
        if(q.sx<gl)q.sx=gl;
        if(q.sx>gr)q.sx=gr;
        if(q.sy<gt)q.sy=gt;
        if(q.sy>gb)q.sy=gb;
        q.outcode=(uint8_t)((q.sx<c.clip[0]?SH2Z_OUT_LEFT:0u)|(q.sx>c.clip[2]?SH2Z_OUT_RIGHT:0u)|
            (q.sy<c.clip[1]?SH2Z_OUT_TOP:0u)|(q.sy>c.clip[3]?SH2Z_OUT_BOTTOM:0u));
        out[k].v=q;
        for(unsigned i=0;i<SH2Z_VIEW_GUARD_ATTRS;++i)out[k].attr[i]=p->c[3+i];
    }
    return n;
}
