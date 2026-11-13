/* Exact workload comparison fixtures. MIT; SH2ZAM Contributors. */
#ifndef SH2Z_WORKLOAD_SUITE_H
#define SH2Z_WORKLOAD_SUITE_H
#include "sh2zam/sh2zam.h"
#include "sh2zam/rotation_batch.h"
#include "sh2zam/spatial.h"
#define WORKLOAD_SAMPLES 7u
#define WORKLOAD_CASES 44u
#define WORKLOAD_VECTORS 512u
#define WORKLOAD_BOXES 512u
#define WORKLOAD_QUERIES 128u
#define WORKLOAD_PATHS 5u
#define WORKLOAD_WORDS 2176u
enum { W_ROTATION,W_NORMALIZE,W_ATAN2,W_ATAN,W_SPATIAL };
struct workload_case { const char *id; uint32_t group,profile,size,queries,words; };
struct workload_input {
 sh2z_quat_t rotation;
 sh2z_vec3_t vectors[WORKLOAD_VECTORS];
 sh2z_aabb_t boxes[WORKLOAD_BOXES];
 sh2z_sphere_t queries[WORKLOAD_QUERIES];
};
union workload_output { uint32_t words[WORKLOAD_WORDS]; sh2z_vec3_t vectors[WORKLOAD_VECTORS]; };
extern const struct workload_case workload_cases[WORKLOAD_CASES];
extern const uint32_t workload_case_count,workload_input_bytes,workload_max_words;
extern struct workload_input workload_inputs;
extern union workload_output workload_outputs[WORKLOAD_PATHS];
uint32_t workload_prepare(uint32_t case_id,uint32_t sample);
uint32_t workload_run(uint32_t path);
bool workload_plain_sphere_aabb(const sh2z_sphere_t *sphere,const sh2z_aabb_t *box);
#endif
