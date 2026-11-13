/* Shared with the browser's actual C bridge. MIT. */
#ifndef SH2Z_BENCH_BRIDGE_H
#define SH2Z_BENCH_BRIDGE_H
#include "sh2zam/sh2zam.h"
unsigned capacity(void);
sh2z_vec3_t *input_ptr(void);
float *before_ptr(void);
sh2z_vec3_t *general_ptr(void);
sh2z_vec3_t *after_ptr(void);
sh2z_vec3_t *mac_ptr(void);
sh2z_vec3_t *dmuls_ptr(void);
sh2z_vec3_t *plain_ptr(void);
int plain_prepare(void);
unsigned plain_available(void);
int plain_range(unsigned start,unsigned n);
int plain(void);
int prepare(unsigned n);
int plan_prepare(void);
/* bit 0: prepared bound proved; bit 1: MAC.L active; bit 2: DMULS.L active. */
unsigned candidate_available(void);
/* 0=general fallback, 1=prepared scalar, 2=reused coordinate products. */
unsigned optimized_path(void);
unsigned reuse_block_size(void);
void setup(unsigned ax,unsigned ay,float sx,float cx,float sy,float cy);
void before_range(unsigned start,unsigned n);
void general_range(unsigned start,unsigned n);
void after_range(unsigned start,unsigned n);
void mac_range(unsigned start,unsigned n);
void dmuls_range(unsigned start,unsigned n);
void before(void);
void general(void);
void after(void);
void after_kernel(void);
#endif
