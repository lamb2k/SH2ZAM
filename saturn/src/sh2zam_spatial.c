/* Exact static AABB index and moving-sphere queries. MIT; SH2ZAM Contributors. */
#include "sh2zam/spatial.h"
#include <limits.h>

static bool spatial_valid_box(const sh2z_aabb_t *b) {
    return b->min.x<=b->max.x&&b->min.y<=b->max.y&&b->min.z<=b->max.z;
}
static bool spatial_buffer(const void *p,size_t count,size_t size,size_t alignment) {
    if(count>SIZE_MAX/size) return false;
    if(!count) return true;
    if(!p||(uintptr_t)p%alignment) return false;
    const size_t bytes=count*size;
    return (uintptr_t)p<=UINTPTR_MAX-(bytes-1);
}
static bool spatial_overlap(const void *a,size_t an,const void *b,size_t bn) {
    if(!an||!bn) return false;
    const uintptr_t aa=(uintptr_t)a,bb=(uintptr_t)b;
    return aa>=bb?aa-bb<bn:bb-aa<an;
}
static bool spatial_hits_box(const sh2z_aabb_t *a,const sh2z_aabb_t *b) {
    return a->min.x<=b->max.x&&b->min.x<=a->max.x&&
           a->min.y<=b->max.y&&b->min.y<=a->max.y&&
           a->min.z<=b->max.z&&b->min.z<=a->max.z;
}
static bool spatial_contains_box(const sh2z_aabb_t *outer,const sh2z_aabb_t *inner) {
    return outer->min.x<=inner->min.x&&outer->max.x>=inner->max.x&&
           outer->min.y<=inner->min.y&&outer->max.y>=inner->max.y&&
           outer->min.z<=inner->min.z&&outer->max.z>=inner->max.z;
}
static uint32_t spatial_distance(int32_t a,int32_t b) {
    return a>=b?(uint32_t)a-(uint32_t)b:(uint32_t)b-(uint32_t)a;
}
/* The farthest AABB corner maximizes squared distance from the center.
 * Reject each oversized component before squaring, then subtract squares
 * from the radius square so their sum cannot overflow. */
static bool spatial_sphere_contains_box(const sh2z_sphere_t *s,const sh2z_aabb_t *b) {
    const uint32_t radius=(uint32_t)s->radius;
    uint32_t a=spatial_distance(s->center.x,b->min.x),c=spatial_distance(s->center.x,b->max.x);
    const uint32_t x=a>c?a:c;
    if(x>radius) return false;
    a=spatial_distance(s->center.y,b->min.y); c=spatial_distance(s->center.y,b->max.y);
    const uint32_t y=a>c?a:c;
    if(y>radius) return false;
    a=spatial_distance(s->center.z,b->min.z); c=spatial_distance(s->center.z,b->max.z);
    const uint32_t z=a>c?a:c;
    if(z>radius) return false;
    uint64_t remaining=(uint64_t)radius*radius;
    uint64_t square=(uint64_t)x*x;
    if(square>remaining) return false;
    remaining-=square; square=(uint64_t)y*y;
    if(square>remaining) return false;
    remaining-=square; square=(uint64_t)z*z;
    return square<=remaining;
}
static int32_t spatial_low(int32_t center,int32_t radius) {
    return center<INT32_MIN+radius?INT32_MIN:center-radius;
}
static int32_t spatial_high(int32_t center,int32_t radius) {
    return center>INT32_MAX-radius?INT32_MAX:center+radius;
}
static sh2z_aabb_t spatial_sphere_bounds(sh2z_sphere_t s) {
    return (sh2z_aabb_t){
        {spatial_low(s.center.x,s.radius),spatial_low(s.center.y,s.radius),spatial_low(s.center.z,s.radius)},
        {spatial_high(s.center.x,s.radius),spatial_high(s.center.y,s.radius),spatial_high(s.center.z,s.radius)}};
}

/* Unsigned order keys preserve signed coordinate order. Their midpoint never
 * overflows, including the full INT32_MIN..INT32_MAX interval. */
static uint32_t spatial_center(const sh2z_aabb_t *b,unsigned axis) {
    int32_t lo,hi;
    if(axis==0) {lo=b->min.x;hi=b->max.x;}
    else if(axis==1) {lo=b->min.y;hi=b->max.y;}
    else {lo=b->min.z;hi=b->max.z;}
    const uint32_t key=(uint32_t)lo^UINT32_C(0x80000000);
    return key+(((uint32_t)hi^UINT32_C(0x80000000))-key)/2u;
}
static void spatial_swap(uint32_t *a,uint32_t *b) {
    const uint32_t t=*a;*a=*b;*b=t;
}
static uint32_t spatial_median(uint32_t a,uint32_t b,uint32_t c) {
    if(a>b) {const uint32_t t=a;a=b;b=t;}
    if(b>c) b=c;
    return a>b?a:b;
}
/* Three-way median selection keeps duplicate centers from degenerating into
 * repeated single-element partitions. Every recursive tree split has exactly
 * floor(n/2) and ceil(n/2) leaves, independent of the spatial distribution. */
static void spatial_select(uint32_t *ids,size_t count,size_t middle,
                           const sh2z_aabb_t *boxes,unsigned axis) {
    size_t first=0,last=count;
    while(last-first>1) {
        const uint32_t pivot=spatial_median(spatial_center(&boxes[ids[first]],axis),
            spatial_center(&boxes[ids[first+(last-first)/2]],axis),
            spatial_center(&boxes[ids[last-1]],axis));
        size_t low=first,scan=first,high=last;
        while(scan<high) {
            const uint32_t key=spatial_center(&boxes[ids[scan]],axis);
            if(key<pivot) {spatial_swap(&ids[low],&ids[scan]);++low;++scan;}
            else if(key>pivot) {--high;spatial_swap(&ids[scan],&ids[high]);}
            else ++scan;
        }
        if(middle<low) last=low;
        else if(middle>=high) first=high;
        else return;
    }
}
static void spatial_build_node(sh2z_aabb_node_t *nodes,size_t node,
                               uint32_t *ids,size_t count,const sh2z_aabb_t *boxes) {
    sh2z_aabb_t bounds=boxes[ids[0]];
    for(size_t i=1;i<count;i++) {
        const sh2z_aabb_t *b=&boxes[ids[i]];
        if(b->min.x<bounds.min.x) bounds.min.x=b->min.x;
        if(b->min.y<bounds.min.y) bounds.min.y=b->min.y;
        if(b->min.z<bounds.min.z) bounds.min.z=b->min.z;
        if(b->max.x>bounds.max.x) bounds.max.x=b->max.x;
        if(b->max.y>bounds.max.y) bounds.max.y=b->max.y;
        if(b->max.z>bounds.max.z) bounds.max.z=b->max.z;
    }
    const size_t end=node+count+(count-1);
    nodes[node]=(sh2z_aabb_node_t){bounds,(uint32_t)end,UINT32_MAX};
    if(count==1) {nodes[node].object=ids[0];return;}
    const uint32_t x=(uint32_t)bounds.max.x-(uint32_t)bounds.min.x;
    const uint32_t y=(uint32_t)bounds.max.y-(uint32_t)bounds.min.y;
    const uint32_t z=(uint32_t)bounds.max.z-(uint32_t)bounds.min.z;
    const unsigned axis=y>x?(z>y?2u:1u):(z>x?2u:0u);
    const size_t middle=count/2;
    spatial_select(ids,count,middle,boxes,axis);
    spatial_build_node(nodes,node+1,ids,middle,boxes);
    spatial_build_node(nodes,node+middle+middle,ids+middle,count-middle,boxes);
}

bool sh2z_aabb_index_build(sh2z_aabb_index_t *index,
    const sh2z_aabb_t *boxes,size_t count,
    sh2z_aabb_node_t *nodes,size_t node_capacity,
    uint32_t *scratch,size_t scratch_capacity) {
    if(count>UINT32_C(0x80000000)||
       !spatial_buffer(index,1,sizeof(*index),_Alignof(sh2z_aabb_index_t))||
       !spatial_buffer(boxes,count,sizeof(*boxes),_Alignof(sh2z_aabb_t))||
       !spatial_buffer(nodes,node_capacity,sizeof(*nodes),_Alignof(sh2z_aabb_node_t))||
       !spatial_buffer(scratch,scratch_capacity,sizeof(*scratch),_Alignof(uint32_t))) return false;
    const size_t box_bytes=count*sizeof(*boxes),node_bytes=node_capacity*sizeof(*nodes);
    const size_t scratch_bytes=scratch_capacity*sizeof(*scratch);
    if(spatial_overlap(index,sizeof(*index),boxes,box_bytes)||
       spatial_overlap(index,sizeof(*index),nodes,node_bytes)||
       spatial_overlap(index,sizeof(*index),scratch,scratch_bytes)||
       spatial_overlap(boxes,box_bytes,nodes,node_bytes)||
       spatial_overlap(boxes,box_bytes,scratch,scratch_bytes)||
       spatial_overlap(nodes,node_bytes,scratch,scratch_bytes)) return false;
    size_t valid=0;
    const size_t maximum=count?count+(count-1):0;
    if(node_capacity>=maximum&&scratch_capacity>=count) {
        /* With worst-case storage already checked, no later input can cause
         * failure. Compact IDs in the validation pass instead of scanning
         * immutable input twice. */
        for(size_t i=0;i<count;i++) if(spatial_valid_box(&boxes[i])) scratch[valid++]=(uint32_t)i;
    } else {
        for(size_t i=0;i<count;i++) if(spatial_valid_box(&boxes[i])) ++valid;
        const size_t needed=valid?valid+(valid-1):0;
        if(node_capacity<needed||scratch_capacity<valid) return false;
        size_t at=0;
        for(size_t i=0;i<count;i++) if(spatial_valid_box(&boxes[i])) scratch[at++]=(uint32_t)i;
    }
    const size_t needed=valid?valid+(valid-1):0;
    if(valid) spatial_build_node(nodes,0,scratch,valid,boxes);
    *index=(sh2z_aabb_index_t){boxes,valid?nodes:NULL,count,needed};
    return true;
}

static bool spatial_query_valid(uint32_t *hits,size_t capacity,size_t *hit_count,
                                const sh2z_aabb_index_t *index) {
    if(!spatial_buffer(index,1,sizeof(*index),_Alignof(sh2z_aabb_index_t))) return false;
    if(index->count>UINT32_C(0x80000000)||
       index->node_count>(index->count?index->count+(index->count-1):0)||
       (index->node_count&&!(index->node_count&1u))||
       !spatial_buffer(index->boxes,index->count,sizeof(*index->boxes),_Alignof(sh2z_aabb_t))||
       !spatial_buffer(index->nodes,index->node_count,sizeof(*index->nodes),_Alignof(sh2z_aabb_node_t))||
       !spatial_buffer(hits,capacity,sizeof(*hits),_Alignof(uint32_t))||
       (hit_count&&!spatial_buffer(hit_count,1,sizeof(*hit_count),_Alignof(size_t)))) return false;
    const size_t words=index->count/32+(index->count%32!=0);
    if(capacity<words) return false;
    const size_t bytes=capacity*sizeof(*hits),boxes=index->count*sizeof(*index->boxes);
    const size_t nodes=index->node_count*sizeof(*index->nodes);
    if(spatial_overlap(hits,bytes,index,sizeof(*index))||
       spatial_overlap(hits,bytes,index->boxes,boxes)||
       spatial_overlap(hits,bytes,index->nodes,nodes)) return false;
    if(hit_count&&(spatial_overlap(hit_count,sizeof(*hit_count),hits,bytes)||
       spatial_overlap(hit_count,sizeof(*hit_count),index,sizeof(*index))||
       spatial_overlap(hit_count,sizeof(*hit_count),index->boxes,boxes)||
       spatial_overlap(hit_count,sizeof(*hit_count),index->nodes,nodes))) return false;
    return true;
}
static void spatial_set_hit(uint32_t *hits,uint32_t object) {
    hits[object>>5]|=UINT32_C(1)<<(object&31u);
}
static size_t spatial_accept_subtree(uint32_t *hits,const sh2z_aabb_node_t *nodes,
                                     size_t first,size_t last) {
    size_t count=0;
    for(size_t i=first;i<last;i++) if(nodes[i].object!=UINT32_MAX) {
        spatial_set_hit(hits,nodes[i].object);++count;
    }
    return count;
}
bool sh2z_aabb_index_query_sphere(uint32_t *hits,size_t hit_word_capacity,
    size_t *hit_count,const sh2z_aabb_index_t *index,sh2z_sphere_t sphere) {
    if(!spatial_query_valid(hits,hit_word_capacity,hit_count,index)) return false;
    const size_t words=index->count/32+(index->count%32!=0);
    size_t count=0;
    /* Exact whole-tree acceptance is especially useful for dense queries.
     * If every original box is valid, original-index output is a packed mask. */
    bool all=false;
    if(sphere.radius>=0&&index->node_count)
        all=spatial_sphere_contains_box(&sphere,&index->nodes[0].bounds);
    if(all&&index->node_count==index->count+(index->count-1)) {
        for(size_t i=0;i<words;i++) hits[i]=UINT32_MAX;
        const unsigned remainder=(unsigned)(index->count%32);
        if(remainder) hits[words-1]=(UINT32_C(1)<<remainder)-1u;
        if(hit_count) *hit_count=index->count;
        return true;
    }
    for(size_t i=0;i<words;i++) hits[i]=0;
    if(sphere.radius<0||!index->node_count) {
        if(hit_count) *hit_count=0;
        return true;
    }
    if(all) count=spatial_accept_subtree(hits,index->nodes,0,index->node_count);
    else {
        const sh2z_aabb_t query=spatial_sphere_bounds(sphere);
        /* If even the root's AABB is contained by the query AABB, none of the
         * child AABBs can prune a candidate; avoid traversing that tree. */
        if(spatial_contains_box(&query,&index->nodes[0].bounds)) {
            for(size_t i=0;i<index->count;i++) if(sh2z_sphere_intersects_aabb(sphere,index->boxes[i])) {
                spatial_set_hit(hits,(uint32_t)i);++count;
            }
        } else {
            size_t node=0;
            while(node<index->node_count) {
                const sh2z_aabb_node_t *n=&index->nodes[node];
                if(!spatial_hits_box(&query,&n->bounds)) {node=n->escape;continue;}
                if(n->object!=UINT32_MAX) {
                    if(sh2z_sphere_intersects_aabb(sphere,index->boxes[n->object])) {
                        spatial_set_hit(hits,n->object);++count;
                    }
                    ++node;
                } else if(n->escape-node>=15&&spatial_contains_box(&query,&n->bounds)&&
                          spatial_sphere_contains_box(&sphere,&n->bounds)) {
                    count+=spatial_accept_subtree(hits,index->nodes,node,n->escape);
                    node=n->escape;
                } else ++node;
            }
        }
    }
    if(hit_count) *hit_count=count;
    return true;
}
