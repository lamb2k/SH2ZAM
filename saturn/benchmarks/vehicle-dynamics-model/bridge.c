/* SH-2 bridge for the executed-instruction vehicle-dynamics model. MIT;
 * Copyright (c) 2026 SH2ZAM Contributors. Every entry point is called
 * directly by the model harness with the SH-2 calling convention; the harness
 * writes the track records, the constants, the body and the pad into target
 * RAM before the calls; nothing here uses libc.
 *
 *   set_track     points the track at the records below and sets its count,
 *                 line length and laps (the harness writes the records)
 *   step          sh2z_vd_step on body with the current pad: one tick
 *   locate        sh2z_vd_locate on probe toward target (iterations,
 *                 tolerance in Q32 as two words)
 *   lap_distance  sh2z_vd_lap_distance on probe
 *   set_track2    vehicle-dynamics-2: points track2 at the same records plus
 *                 the joins (or none) and sets its count, length and laps (the
 *                 harness writes the joins and the tube rules)
 *   step2         sh2z_vd_step2 on body with the current pad: one tick
 * sizes[] lets the harness check that each record has the host layout; its
 * last entry is the target offset of track2.tube (track2 holds pointers, so
 * its layout is the target's own). */
#include "sh2zam/vehicle_dynamics_experimental.h"
#include <stddef.h>

#define NOINLINE __attribute__((noinline))

enum {NODES=16,ZONES=4,RAMPS=2,MINES=2};
sh2z_vd_node_t nodes[NODES];
sh2z_vd_zone_t zones[ZONES];
sh2z_vd_ramp_t ramps[RAMPS];
sh2z_vd_mine_t mines[MINES];
sh2z_vd_track_t track;
sh2z_vd_vehicle_t vehicle;
sh2z_vd_rules_t rules;
sh2z_vd_body_t body,probe;
sh2z_vd_input_t input;
int32_t target[3];
sh2z_vd_join_t joins[NODES];
sh2z_vd_track2_t track2;

const uint32_t sizes[10]={
    sizeof(sh2z_vd_node_t),sizeof(sh2z_vd_zone_t),sizeof(sh2z_vd_ramp_t),sizeof(sh2z_vd_mine_t),
    sizeof(sh2z_vd_vehicle_t),sizeof(sh2z_vd_rules_t),sizeof(sh2z_vd_body_t),sizeof(sh2z_vd_input_t),
    sizeof(sh2z_vd_join_t),offsetof(sh2z_vd_track2_t,tube)};

NOINLINE uint32_t set_track(uint32_t count,uint32_t length,uint32_t laps) {
    if(count<4u||count>NODES)return 0;
    track.nodes=nodes;track.zones=zones;track.ramps=ramps;track.mines=mines;
    track.count=count;track.length=(int32_t)length;track.laps=(int32_t)laps;
    return 1;
}

NOINLINE uint32_t step(void) {
    sh2z_vd_step(&body,&track,&vehicle,&rules,&input);
    return body.flags;
}

NOINLINE uint32_t locate(uint32_t iterations,uint32_t tolerance_hi,uint32_t tolerance_lo) {
    const int64_t tolerance=(int64_t)(((uint64_t)tolerance_hi<<32)|tolerance_lo);
    return (uint32_t)sh2z_vd_locate(&probe,&track,target,(int)iterations,tolerance);
}

NOINLINE uint32_t lap_distance(void) {
    return (uint32_t)sh2z_vd_lap_distance(&probe,&track);
}

NOINLINE uint32_t set_track2(uint32_t count,uint32_t length,uint32_t laps,uint32_t with_joins) {
    if(count<4u||count>NODES)return 0;
    track2.base.nodes=nodes;track2.base.zones=zones;track2.base.ramps=ramps;track2.base.mines=mines;
    track2.base.count=count;track2.base.length=(int32_t)length;track2.base.laps=(int32_t)laps;
    track2.joins=with_joins?joins:0;
    return 1;
}

NOINLINE uint32_t step2(void) {
    sh2z_vd_step2(&body,&track2,&vehicle,&rules,&input);
    return body.flags;
}
