/**
 * @file    state_machine.h
 * @brief   Gait-aware state machine for the prosthetic knee.
 *
 * State diagram (text form):
 *
 *                  +-----------+
 *                  |   BOOT    |
 *                  +-----+-----+
 *                        |
 *                        v
 *      +--<--------+----------+--<--------+
 *      |           |   IDLE   |           |
 *      |           +----+-----+           |
 *      |                |                 |
 *      |  high force    |  zero force     |  heel-strike pattern
 *      v                v                 v
 *  +-------+       +---------+        +---------+
 *  |STANDING|<---->| WALKING |        |SITTING  |
 *  +---+---+       +----+----+        +----+----+
 *      |                |                 |
 *      | stair-up       | stair-down      |
 *      v                v                 |
 *  +------------+  +---------------+      |
 *  |STAIR_ASCENT|  |STAIR_DESCENT  |      |
 *  +-----+------+  +-------+-------+      |
 *        |               |               |
 *        +-------+--------+---------------+
 *                |
 *                v
 *           +---------+         any fault
 *           | CALIB   |---------------+
 *           +---------+               |
 *                                     v
 *                                +---------+
 *                                |  FAULT  |
 *                                +---------+
 *
 * Transitions
 * -----------
 *   1. IDLE → STANDING        when force > 30 N for > 200 ms
 *   2. STANDING → WALKING     when 3 heel-strikes detected in 1 s
 *   3. STANDING → SITTING     when knee angle > 80 deg and force < 10 N
 *   4. STANDING → STAIR_ASCENT when knee flexion < 5 deg and vertical rise
 *   5. WALKING → STAIR_DESCENT when knee flexion > 60 deg with descent
 *   6. any → FAULT            on safety violation
 *   7. FAULT → IDLE           only on operator reset
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "project_types.h"

typedef struct {
    pk_state_t  current;
    pk_state_t  previous;
    pk_fault_t  last_fault;
    uint32_t    state_enter_tick;
    uint32_t    force_accum_ms;
    uint16_t    heel_strike_count;
    pk_spool_angle_t current_spool;
} pk_sm_ctx_t;

void        pk_sm_init(pk_sm_ctx_t *ctx);
void        pk_sm_step(pk_sm_ctx_t *ctx,
                       pk_adc_data_t *adc,
                       pk_knee_angle_t knee_deg_x10);
pk_state_t  pk_sm_get(const pk_sm_ctx_t *ctx);

/* Force a transition — used by operator commands */
void        pk_sm_force_state(pk_sm_ctx_t *ctx, pk_state_t s);

/* Trigger a fault; never returns until cleared */
void        pk_sm_raise_fault(pk_sm_ctx_t *ctx, pk_fault_t f);

#endif /* STATE_MACHINE_H */
