/* Independent ordinary-C full-range sphere/AABB predicate. MIT.
 * No SH2ZAM arithmetic, bounds, traversal or intersection routines are called.
 * Included directly in the workload bridge so -O2 can inline the brute loop. */
static inline uint32_t plain_distance(int32_t a,int32_t b) {
 return a>=b?(uint32_t)a-(uint32_t)b:(uint32_t)b-(uint32_t)a;
}
struct plain_sphere {int32_t x,y,z;uint32_t radius;uint64_t square;};
static inline struct plain_sphere plain_prepare(sh2z_sphere_t s) {
 uint32_t r=(uint32_t)s.radius;
 return (struct plain_sphere){s.center.x,s.center.y,s.center.z,r,(uint64_t)r*r};
}
static inline bool plain_sphere_aabb(const struct plain_sphere *sphere,const sh2z_aabb_t *box) {
 if(box->min.x>box->max.x || box->min.y>box->max.y || box->min.z>box->max.z) return false;
 uint64_t remaining=sphere->square;
#define PLAIN_AXIS(axis) do { \
 int32_t nearest=sphere->axis<box->min.axis?box->min.axis:sphere->axis>box->max.axis?box->max.axis:sphere->axis; \
 uint32_t distance=plain_distance(sphere->axis,nearest); \
 if(distance>sphere->radius) return false; \
 uint64_t square=(uint64_t)distance*distance; \
 if(square>remaining) return false; \
 remaining-=square; \
} while(0)
 PLAIN_AXIS(x);PLAIN_AXIS(y);PLAIN_AXIS(z);
#undef PLAIN_AXIS
 return true;
}
