/* Experimental vehicle dynamics with contact on tube shapes (contract tag
 * vehicle-dynamics-3): sh2z_vd_step2_contact.
 *
 * The vehicle-dynamics-2 tick (sh2zam_vehicle_dynamics.c built with
 * SH2Z_VD_TUBES, as sh2zam_vehicle_dynamics2.c does) built a third time with
 * the contact hook set, as sh2zam_vehicle_contact.c does for the road tick.
 * The search, spline and line-space view come from sh2zam_vehicle_dynamics.c.
 *
 * Copyright (c) 2026 SH2ZAM Contributors. MIT License.
 * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.
 */
#include "sh2zam/vehicle_dynamics_experimental.h"

static void vdc2_step2(sh2z_vd_body_t *b, const sh2z_vd_track2_t *track2, const sh2z_vd_vehicle_t *v,
                       const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in);
static void vdc2_contact(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r);

#define SH2Z_VD_TUBES 1
#define sh2z_vd_step2 vdc2_step2
#define VD_CONTACT(b, v, r) vdc2_contact((b), (v), (r));
#include "sh2zam_vehicle_dynamics.c"
#undef sh2z_vd_step2

/* The contact of the tick in progress (one tick at a time per CPU). */
static const sh2z_vd_contact_t *vdc2_now;

/* As sh2zam_vehicle_contact.c: regain, the push as the first acceleration,
 * the damage (before the pad is read), the grip-off timer. */
static void vdc2_contact(sh2z_vd_body_t *b, const sh2z_vd_vehicle_t *v, const sh2z_vd_rules_t *r) {
    const sh2z_vd_contact_t *c = vdc2_now;
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

void sh2z_vd_step2_contact(sh2z_vd_body_t *b, const sh2z_vd_track2_t *track, const sh2z_vd_vehicle_t *v,
                           const sh2z_vd_rules_t *r, const sh2z_vd_input_t *in, const sh2z_vd_contact_t *c) {
    vdc2_now = c;
    vdc2_step2(b, track, v, r, in);
}
