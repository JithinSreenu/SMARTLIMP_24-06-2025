/**

 * @file    state_machine.c

 * @brief   Gait state machine implementation.

 *

 * Inputs to the FSM (every 5 ms from the ADC task):

 *   - force[N]          vertical load

 *   - knee_angle[deg*10]

 *   - battery voltage   (sanity)

 *

 * Outputs:

 *   - target spool angle (0..100 %) forwarded to motor control

 *   - global state id   (consumed by telemetry)

 *

 * Why we don't use a full FSM library

 *   A switch / case with explicit transitions is more auditable for

 *   certification than a generated FSM table. MISRA-C 2012 Rule 16.1

 *   also discourages deeply nested control flow.

 */


#include "state_machine.h"

#include "motor_control.h"

#include <string.h>


/* Tunable thresholds — kept as #defines so they can be patched

 * without re-running static analysis. */

#define PK_FORCE_STAND_THRESH_N   (30)    /* 3.0 N in 0.1-N units    */

#define PK_FORCE_SIT_THRESH_N     (10)

#define PK_KNEE_SIT_DEG_X10      (800)    /* 80.0 deg                 */

#define PK_KNEE_FLEX_STAIR       (600)    /* 60 deg — stair descent   */

#define PK_HEEL_STRIKE_WIN_MS    (1000u)

#define PK_HEEL_STRIKE_COUNT     (3u)


/* -------------------------------------------------------------------- */

/*  Internal helpers                                                     */

/* -------------------------------------------------------------------- */


static inline uint32_t now_ticks(void)

{

    return xTaskGetTickCount();

}


static void enter_state(pk_sm_ctx_t *ctx, pk_state_t s)

{

    ctx->previous        = ctx->current;

    ctx->current         = s;

    ctx->state_enter_tick = now_ticks();

    ctx->heel_strike_count = 0u;

}


/* -------------------------------------------------------------------- */

/*  Public                                                               */

/* -------------------------------------------------------------------- */


void pk_sm_init(pk_sm_ctx_t *ctx)

{

    (void)memset(ctx, 0, sizeof(*ctx));

    ctx->current        = PK_STATE_BOOT;

    ctx->previous       = PK_STATE_BOOT;

    ctx->last_fault     = PK_FAULT_NONE;

    ctx->current_spool  = 50;     /* mid-position — neutral */

    pk_motor_set_spool(ctx->current_spool);

    enter_state(ctx, PK_STATE_IDLE);

}


void pk_sm_force_state(pk_sm_ctx_t *ctx, pk_state_t s)

{

    if (s >= PK_STATE_MAX) return;

    enter_state(ctx, s);

}


void pk_sm_raise_fault(pk_sm_ctx_t *ctx, pk_fault_t f)

{

    ctx->last_fault = f;

    enter_state(ctx, PK_STATE_FAULT);

    pk_motor_emergency_stop();

}


void pk_sm_step(pk_sm_ctx_t *ctx,

                pk_adc_data_t *adc,

                pk_knee_angle_t knee_deg_x10)

{

    if (ctx->current == PK_STATE_FAULT) {

        /* Stay in fault until cleared */

        return;

    }


    /* Range guard — any wild value means sensor failure */

    if (adc->filt[PK_ADC_CH_FORCE] > 5000 ||

        adc->filt[PK_ADC_CH_FORCE] < -5000) {

        pk_sm_raise_fault(ctx, PK_FAULT_ADC_OVERRANGE);

        return;

    }


    switch (ctx->current) {

    case PK_STATE_IDLE:

        ctx->current_spool = 50;            /* half-closed */

        if (adc->filt[PK_ADC_CH_FORCE] > PK_FORCE_STAND_THRESH_N) {

            ctx->force_accum_ms += 5u;       /* called every 5 ms */

            if (ctx->force_accum_ms > 200u) {

                enter_state(ctx, PK_STATE_STANDING);

            }

        } else {

            ctx->force_accum_ms = 0u;

        }

        break;


    case PK_STATE_STANDING:

        ctx->current_spool = 90;            /* closed — stiff knee */

        if (knee_deg_x10 > PK_KNEE_SIT_DEG_X10 &&

            adc->filt[PK_ADC_CH_FORCE] < PK_FORCE_SIT_THRESH_N) {

            enter_state(ctx, PK_STATE_SITTING);

        }

        /* simple heel-strike detection: rapid force rise */

        if (adc->filt[PK_ADC_CH_FORCE] > 200 &&

            ctx->heel_strike_count < 10u) {

            ctx->heel_strike_count++;

            if (ctx->heel_strike_count >= PK_HEEL_STRIKE_COUNT) {

                enter_state(ctx, PK_STATE_WALKING);

            }

        }

        break;


    case PK_STATE_WALKING:

        /* Alternate between stiff (stance) and free (swing) per step.

         * For brevity we just modulate the spool by knee angle. */

        ctx->current_spool = (knee_deg_x10 < 0) ? 20 : 80;

        if (knee_deg_x10 > PK_KNEE_FLEX_STAIR) {

            enter_state(ctx, PK_STATE_STAIR_DESCENT);

        }

        break;


    case PK_STATE_STAIR_ASCENT:

        ctx->current_spool = 70;

        break;


    case PK_STATE_STAIR_DESCENT:

        ctx->current_spool = 30;            /* very open — let leg bend */

        if (knee_deg_x10 < 100) {

            enter_state(ctx, PK_STATE_STANDING);

        }

        break;


    case PK_STATE_SITTING:

        ctx->current_spool = 10;            /* nearly free */

        break;


    case PK_STATE_CALIBRATION:

        ctx->current_spool = 50;

        break;


    default:

        break;

    }


    pk_motor_set_spool(ctx->current_spool);

}


pk_state_t pk_sm_get(const pk_sm_ctx_t *ctx)

{

    return ctx->current;

}

