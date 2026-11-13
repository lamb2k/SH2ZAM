/* Shared with the browser's actual C bridge. MIT. */
#ifndef SH2Z_BENCH_BRIDGE_H
#define SH2Z_BENCH_BRIDGE_H
#include "sh2zam/sh2zam.h"
unsigned capacity(void);
sh2z_vec3_t *input_ptr(void);
float *before_ptr(void);
sh2z_vec3_t *general_ptr(void);
sh2z_vec3_t *after_ptr(void);
int prepare(unsigned n);
void setup(unsigned ax,unsigned ay,float sx,float cx,float sy,float cy);
void before_range(unsigned start,unsigned n);
void general_range(unsigned start,unsigned n);
void after_range(unsigned start,unsigned n);
void before(void);
void general(void);
void after(void);
#endif
