/* Experimental vehicle dynamics with contact (contract tag vehicle-dynamics-2).
 *
 * The tick is vehicle-dynamics-1's own source, built a second time here with
 * its contact hook set, so the two entry points share one definition of the
 * model and sh2z_vd_step keeps its object code. This file's copies of the
 * model's functions are private to it.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vehicle_dynamics_experimental.h"

#define SH2Z_VDC_UNUSED __attribute__((unused))

static void vdc_step(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const sh2z_vd_vehicle_t *v,
                     const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in);
static int vdc_locate(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const int32_t target[3], int iterations,
                      int64_t tolerance_q32);
static int32_t vdc_lap_distance(sh2z_vd_body_t *b, const sh2z_vd_track_t *track);
static SH2Z_VDC_UNUSED int32_t vdc_body_lateral(const sh2z_vd_body_t *b);
static SH2Z_VDC_UNUSED void vdc_line_view(const sh2z_vd_body_t *b, const sh2z_vd_track_t *track,
                                          sh2z_vd_line_view_t *out);
static SH2Z_VDC_UNUSED void vdc_apply_kick(sh2z_vd_body_t *b, int32_t kick_lat, int32_t kick_speed);
static void vdc_spline(const sh2z_vd_track_t *track, uint32_t n, int32_t t, int32_t pos[3], int32_t fwd[3]);
static void vdc_contact(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r);

#define sh2z_vd_step vdc_step
#define sh2z_vd_locate vdc_locate
#define sh2z_vd_lap_distance vdc_lap_distance
#define sh2z_vd_body_lateral vdc_body_lateral
#define sh2z_vd_line_view vdc_line_view
#define sh2z_vd_apply_kick vdc_apply_kick
#define sh2z_vd_spline vdc_spline
#define VD_CONTACT(b, v, r) vdc_contact((b), (v), (r))
#include "sh2zam_vehicle_dynamics.c"
#undef sh2z_vd_step

/* The contact of the tick in progress (one tick at a time per CPU). */
static const sh2z_vd_contact_t *vdc_now;

/* Contact at the start of the tick: the regain, the push as the first
 * acceleration, the damage (which may start the spin-out before the pad is
 * read) and the grip-off timer. */
static void vdc_contact(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r) {
    const sh2z_vd_contact_t *c = vdc_now;
    if(c->regain != 0 && !(b->flags & (SH2Z_VD_CRASHED | SH2Z_VD_SPINNING))) {
        b->energy += c->regain;
        if(b->energy > v->max_energy) b->energy = v->max_energy;
    }
    b->acc[0] = c->push[0];
    b->acc[1] = c->push[1];
    b->acc[2] = c->push[2];
    if(c->damage > 0) damage(b, r, c->damage);
    if(c->hit) b->hit_timer = r->contact_grip;
}

void sh2z_vd_step_contact(sh2z_vd_body_t *b, const sh2z_vd_track_t *track, const sh2z_vd_vehicle_t *v,
                          const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in, const sh2z_vd_contact_t *c) {
    vdc_now = c;
    vdc_step(b, track, v, r, in);
}
