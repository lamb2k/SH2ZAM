/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 SH2ZAM Contributors.
 * Link probes for section garbage collection (ci.sh). Each probe_<name> calls
 * one library entry point and nothing else; ci.sh links an ELF per probe with
 * that probe as its only root and --gc-sections, and checks which hand-written
 * entries and tables survive. The probes are linked, never run: the null and
 * zero arguments only make the calls well formed. */
#include "sh2zam/memo_experimental.h"
#include "sh2zam/shade_experimental.h"
#include "sh2zam/trig12_experimental.h"
#include "sh2zam/unit30_experimental.h"
#include "sh2zam/view_experimental.h"
#include "sh2zam/vlc2_experimental.h"
#include "sh2zam/vlc_experimental.h"

void probe_fog_gouraud_refs(void);
void probe_fog_gouraud(void);
void probe_memo_put4(void);
void probe_memo_sync16w(void);
void probe_memo_diff8_runs(void);
void probe_view_batch_divu(void);
void probe_view_batch_trunc_divu(void);
void probe_view_batch_center_divu(void);
void probe_view_batch_divu_indexed(void);
void probe_view_merge_tier(void);
void probe_view_edge_rails_center_divu(void);
void probe_view_from_camera30(void);
void probe_shade_gouraud_approx(void);
void probe_shade_light_normals_indexed(void);
void probe_shade_screen_facing(void);
void probe_vlc8_decode(void);
void probe_vlc2_build(void);
void probe_vec3_normalize_u30(void);
void probe_tan12_q16(void);
void probe_pose_from_f32(void);

void probe_fog_gouraud_refs(void) {(void)sh2z_shade_fog_gouraud_refs(0,0,0,0,0,0,0);}
void probe_fog_gouraud(void) {(void)sh2z_shade_fog_gouraud(0,0,0,0,0,0,0);}
void probe_memo_put4(void) {(void)sh2z_memo_put4(0,0,0,0,0);}
void probe_memo_sync16w(void) {(void)sh2z_memo_sync16w(0,0);}
void probe_memo_diff8_runs(void) {(void)sh2z_memo_diff8_runs(0,0,0,0,0);}
void probe_view_batch_divu(void) {(void)sh2z_view_batch_divu(0,0,0,0,0);}
void probe_view_batch_trunc_divu(void) {(void)sh2z_view_batch_trunc_divu(0,0,0,0,0);}
void probe_view_batch_center_divu(void) {(void)sh2z_view_batch_center_divu(0,0,0,0,0);}
void probe_view_batch_divu_indexed(void) {(void)sh2z_view_batch_divu_indexed(0,0,0,0,0,0,0);}
void probe_view_merge_tier(void) {(void)sh2z_view_merge_tier(0,0,0,0,0,0,0);}
void probe_view_edge_rails_center_divu(void) {(void)sh2z_view_edge_rails_center_divu(0,0,0,0,0,0,0,0,0,0,0,0,0);}
void probe_view_from_camera30(void) {(void)sh2z_view_from_camera30(0,0,0,0);}
void probe_shade_gouraud_approx(void) {(void)sh2z_shade_gouraud_approx(0,0,0,0,0,0,0,0,0);}
void probe_shade_light_normals_indexed(void) {(void)sh2z_shade_light_normals_indexed(0,0,0,0,0,0,0,0,0);}
void probe_shade_screen_facing(void) {(void)sh2z_shade_screen_facing(0,0,0,0,0,0,0,0);}
void probe_vlc8_decode(void) {(void)sh2z_vlc8_decode(0,0,0,0);}
void probe_vlc2_build(void) {(void)sh2z_vlc2_build(0,0,0,0,0,0);}
void probe_vec3_normalize_u30(void) {(void)sh2z_vec3_normalize_u30(0,0);}
void probe_tan12_q16(void) {(void)sh2z_tan12_q16(0);}
void probe_pose_from_f32(void) {(void)sh2z_pose_from_f32(0,0,0,0,0,0,0,0);}
