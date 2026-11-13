/* Exact coordinate-reuse transforms. MIT; SH2ZAM Contributors. */
#include "sh2zam/reuse_experimental.h"
#include "sh2zam_prepared_internal.h"

typedef struct {const void *pointer;size_t bytes;} span_t;
static bool valid_span(const void *pointer,size_t count,size_t stride) {
    return count<=SIZE_MAX/stride&&(count==0||pointer!=NULL);
}
static bool all_disjoint(const span_t *spans,size_t count) {
    for(size_t i=0;i<count;i++)for(size_t j=0;j<i;j++)
        if(sh2zi_prepared_overlap(spans[i].pointer,spans[i].bytes,
                                 spans[j].pointer,spans[j].bytes))return false;
    return true;
}
static bool reuse_gate(size_t count,size_t x_count,size_t yz_count) {
    if(count<128||x_count>count/2)return false;
    return yz_count<=(count/2-x_count)/2;
}

bool sh2z_reuse_mesh_init(sh2z_reuse_mesh_t *mesh,
    const sh2z_vec3_t *positions,size_t count,
    sh2z_reuse_index_t *indices,size_t index_capacity,
    sh2z_fx_t *x_keys,size_t x_capacity,
    sh2z_reuse_yz_t *yz_keys,size_t yz_capacity) {
    if(!mesh||index_capacity<count||
       !valid_span(positions,count,sizeof(*positions))||
       !valid_span(indices,index_capacity,sizeof(*indices))||
       !valid_span(x_keys,x_capacity,sizeof(*x_keys))||
       !valid_span(yz_keys,yz_capacity,sizeof(*yz_keys))||
       x_capacity>UINT32_MAX/sizeof(sh2z_reuse_term_t)||
       yz_capacity>UINT32_MAX/sizeof(sh2z_reuse_term_t))return false;
    const span_t spans[]={
        {mesh,sizeof(*mesh)},{positions,count*sizeof(*positions)},
        {indices,index_capacity*sizeof(*indices)},
        {x_keys,x_capacity*sizeof(*x_keys)},
        {yz_keys,yz_capacity*sizeof(*yz_keys)}
    };
    if(!all_disjoint(spans,sizeof(spans)/sizeof(spans[0])))return false;
    sh2z_reuse_mesh_t prepared={0};
    if(!sh2z_mesh_view_init(&prepared.view,positions,count))return false;
    prepared.indices=indices;prepared.x_keys=x_keys;prepared.yz_keys=yz_keys;
    prepared.index_capacity=index_capacity;
    prepared.x_capacity=x_capacity;prepared.yz_capacity=yz_capacity;
    prepared.reusable=count>=128&&x_capacity!=0&&yz_capacity!=0;
    if(prepared.reusable)for(size_t i=0;i<count;i++) {
        size_t x=0,yz=0;
        while(x<prepared.x_count&&x_keys[x]!=positions[i].x)x++;
        if(x==prepared.x_count) {
            if(x==x_capacity){prepared.reusable=false;break;}
            x_keys[x]=positions[i].x;prepared.x_count++;
        }
        while(yz<prepared.yz_count&&
              (yz_keys[yz].y!=positions[i].y||yz_keys[yz].z!=positions[i].z))yz++;
        if(yz==prepared.yz_count) {
            if(yz==yz_capacity){prepared.reusable=false;break;}
            yz_keys[yz]=(sh2z_reuse_yz_t){positions[i].y,positions[i].z};
            prepared.yz_count++;
        }
        indices[i].x_offset=(uint32_t)(x*sizeof(sh2z_reuse_term_t));
        indices[i].yz_offset=(uint32_t)(yz*sizeof(sh2z_reuse_term_t));
        /* Unique counts only grow; a rejected lower bound cannot recover. */
        if(!reuse_gate(count,prepared.x_count,prepared.yz_count)) {
            prepared.reusable=false;break;
        }
    }
    *mesh=prepared;
    return true;
}

static bool mesh_spans_valid(const sh2z_reuse_mesh_t *mesh) {
    return mesh&&mesh->index_capacity>=mesh->view.count&&
        valid_span(mesh->view.positions,mesh->view.count,sizeof(sh2z_vec3_t))&&
        valid_span(mesh->indices,mesh->index_capacity,sizeof(*mesh->indices))&&
        valid_span(mesh->x_keys,mesh->x_capacity,sizeof(*mesh->x_keys))&&
        valid_span(mesh->yz_keys,mesh->yz_capacity,sizeof(*mesh->yz_keys))&&
        mesh->x_count<=mesh->x_capacity&&mesh->yz_count<=mesh->yz_capacity;
}

bool sh2z_reuse_plan_init(sh2z_reuse_plan_t *plan,
    const sh2z_reuse_mesh_t *mesh,const sh2z_mat34_t *matrix,
    sh2z_reuse_term_t *x_terms,size_t x_term_capacity,
    sh2z_reuse_term_t *yz_terms,size_t yz_term_capacity) {
    if(!plan||!matrix||!mesh_spans_valid(mesh)||
       !valid_span(x_terms,x_term_capacity,sizeof(*x_terms))||
       !valid_span(yz_terms,yz_term_capacity,sizeof(*yz_terms)))return false;
    const span_t reads[]={
        {mesh,sizeof(*mesh)},
        {matrix,sizeof(*matrix)},
        {mesh->view.positions,mesh->view.count*sizeof(sh2z_vec3_t)},
        {mesh->indices,mesh->index_capacity*sizeof(*mesh->indices)},
        {mesh->x_keys,mesh->x_capacity*sizeof(*mesh->x_keys)},
        {mesh->yz_keys,mesh->yz_capacity*sizeof(*mesh->yz_keys)}
    };
    const span_t writes[]={
        {plan,sizeof(*plan)},
        {x_terms,x_term_capacity*sizeof(*x_terms)},
        {yz_terms,yz_term_capacity*sizeof(*yz_terms)}
    };
    if(!all_disjoint(writes,sizeof(writes)/sizeof(writes[0])))return false;
    for(size_t i=0;i<sizeof(writes)/sizeof(writes[0]);i++)
        for(size_t j=0;j<sizeof(reads)/sizeof(reads[0]);j++)
            if(sh2zi_prepared_overlap(writes[i].pointer,writes[i].bytes,
                                     reads[j].pointer,reads[j].bytes))return false;
    sh2z_reuse_plan_t prepared={0};
    if(!sh2z_transform_plan_init(&prepared.base,&mesh->view,matrix))return false;
    prepared.mesh=mesh;prepared.x_terms=x_terms;prepared.yz_terms=yz_terms;
    prepared.x_term_capacity=x_term_capacity;
    prepared.yz_term_capacity=yz_term_capacity;
    prepared.enabled=mesh->reusable&&prepared.base.fast_path&&
        x_term_capacity>=mesh->x_count&&yz_term_capacity>=mesh->yz_count;
    if(prepared.enabled) {
        const int32_t a=matrix->m[0][0],b=matrix->m[1][0],c=matrix->m[2][0];
        const int64_t tx=(int64_t)matrix->m[0][3]*INT64_C(65536);
        const int64_t ty=(int64_t)matrix->m[1][3]*INT64_C(65536);
        const int64_t tz=(int64_t)matrix->m[2][3]*INT64_C(65536);
        for(size_t i=0;i<mesh->x_count;i++) {
            const int32_t x=mesh->x_keys[i];
            x_terms[i]=(sh2z_reuse_term_t){
                (int64_t)a*x+tx,(int64_t)b*x+ty,(int64_t)c*x+tz};
        }
        const int32_t ay=matrix->m[0][1],az=matrix->m[0][2];
        const int32_t by=matrix->m[1][1],bz=matrix->m[1][2];
        const int32_t cy=matrix->m[2][1],cz=matrix->m[2][2];
        for(size_t i=0;i<mesh->yz_count;i++) {
            const int32_t y=mesh->yz_keys[i].y,z=mesh->yz_keys[i].z;
            yz_terms[i]=(sh2z_reuse_term_t){
                (int64_t)ay*y+(int64_t)az*z,
                (int64_t)by*y+(int64_t)bz*z,
                (int64_t)cy*y+(int64_t)cz*z};
        }
    }
    *plan=prepared;
    return true;
}

static bool output_valid(sh2z_vec3_t *out,const sh2z_reuse_plan_t *plan,
                         size_t first,size_t count) {
    if(!plan||!plan->mesh||
       !sh2zi_transform_plan_range_valid(out,&plan->base,first,count))return false;
    if(count==0)return true;
    const sh2z_reuse_mesh_t *mesh=plan->mesh;
    const span_t protected[]={
        {plan,sizeof(*plan)},{mesh,sizeof(*mesh)},
        {mesh->indices,mesh->index_capacity*sizeof(*mesh->indices)},
        {mesh->x_keys,mesh->x_capacity*sizeof(*mesh->x_keys)},
        {mesh->yz_keys,mesh->yz_capacity*sizeof(*mesh->yz_keys)},
        {plan->x_terms,plan->x_term_capacity*sizeof(*plan->x_terms)},
        {plan->yz_terms,plan->yz_term_capacity*sizeof(*plan->yz_terms)}
    };
    for(size_t i=0;i<sizeof(protected)/sizeof(protected[0]);i++)
        if(sh2zi_prepared_overlap(out,count*sizeof(*out),
                                  protected[i].pointer,protected[i].bytes))return false;
    return true;
}

bool sh2z_reuse_plan_execute_range(sh2z_vec3_t *out,
    const sh2z_reuse_plan_t *plan,size_t first,size_t count) {
    if(!output_valid(out,plan,first,count))return false;
    if(count==0)return true;
    if(!plan->enabled)
        return sh2z_transform_plan_execute_range(out,&plan->base,first,count);
    const sh2z_reuse_index_t *indices=plan->mesh->indices+first;
    const unsigned char *x_bytes=(const unsigned char *)plan->x_terms;
    const unsigned char *yz_bytes=(const unsigned char *)plan->yz_terms;
    for(size_t i=0;i<count;i++) {
        const sh2z_reuse_term_t *a=(const sh2z_reuse_term_t *)(x_bytes+indices[i].x_offset);
        const sh2z_reuse_term_t *b=(const sh2z_reuse_term_t *)(yz_bytes+indices[i].yz_offset);
        out[i].x=(int32_t)sh2zi_prepared_floor16(a->x+b->x);
        out[i].y=(int32_t)sh2zi_prepared_floor16(a->y+b->y);
        out[i].z=(int32_t)sh2zi_prepared_floor16(a->z+b->z);
    }
    return true;
}

bool sh2z_reuse_plan_execute(sh2z_vec3_t *out,const sh2z_reuse_plan_t *plan) {
    if(!plan)return false;
    return sh2z_reuse_plan_execute_range(out,plan,0,plan->base.count);
}
