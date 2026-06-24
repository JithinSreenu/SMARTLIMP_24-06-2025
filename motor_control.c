/**

 * @file    motor_control.c

 * @brief   Hydraulic valve motor controller implementation.

 *

 * The control law is a single integrator per motor with slew-rate

 * limiting. We deliberately keep it simple:

 *

 *   target[m]  — desired PWM ticks (set by set_target)

 *   current[m] — actual PWM ticks  (ramped each 1 ms tick)

 *

 *   if current < target : current += min(PK_PWM_RAMP_PER_MS, target - current)

 *   if current > target : current -= min(PK_PWM_RAMP_PER_MS, current - target)

 *

 * Dead-time enforcement: when both targets are positive we clamp the

 * smaller one to zero. This guarantees that the H-bridge can never

 * shoot through and short the supply.

 */


#include "motor_control.h"

#include "stm32u5xx_hal.h"


extern TIM_HandleTypeDef htim3;


/* -------------------------------------------------------------------- */

/*  Module state                                                         */

/* -------------------------------------------------------------------- */


static struct {

    uint16_t target[2];

    uint16_t current[2];

    bool     enabled;

    uint32_t stall_ms[2];

} s_mot = {

    .target   = { 0u, 0u },

    .current  = { 0u, 0u },

    .enabled  = false

};


/* -------------------------------------------------------------------- */

/*  Helpers                                                              */

/* -------------------------------------------------------------------- */


static inline uint16_t pk_min(uint16_t a, uint16_t b) { return (a < b) ? a : b; }


static void pk_apply_hw(pk_motor_id_t m, uint16_t duty)

{

    if (m == PK_MOTOR_A) {

        /* H-bridge: PWM on CH1 enables forward, CH2 enables reverse.

         * Always force the opposite channel low. */

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0u);

    } else {

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0u);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);

    }

}


/* -------------------------------------------------------------------- */

/*  Public API                                                           */

/* -------------------------------------------------------------------- */


HAL_StatusTypeDef pk_motor_init(void)

{

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) return HAL_ERROR;


    /* Disable H-bridge enable pin at reset */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    pk_apply_hw(PK_MOTOR_A, 0u);

    pk_apply_hw(PK_MOTOR_B, 0u);

    return HAL_OK;

}


void pk_motor_emergency_stop(void)

{

    s_mot.target[0]  = 0u;

    s_mot.target[1]  = 0u;

    s_mot.enabled    = false;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  /* H-bridge disable */

    pk_apply_hw(PK_MOTOR_A, 0u);

    pk_apply_hw(PK_MOTOR_B, 0u);

}


void pk_motor_set_target(pk_motor_id_t motor, uint16_t duty_ticks)

{

    if (motor > PK_MOTOR_B) return;

    if (duty_ticks > PK_PWM_PERIOD_TICKS) {

        duty_ticks = PK_PWM_PERIOD_TICKS;

    }

    s_mot.target[motor] = duty_ticks;

}


void pk_motor_set_spool(pk_spool_angle_t pct)

{

    /* Map 0..100 % to PK_PWM_PERIOD_TICKS, but never actuate

     * both motors simultaneously — pick the closest. */

    uint32_t duty = ((uint32_t)pct * PK_PWM_PERIOD_TICKS) / 100u;

    if (pct <= 50u) {

        pk_motor_set_target(PK_MOTOR_B, (uint16_t)duty);   /* closing */

        pk_motor_set_target(PK_MOTOR_A, 0u);

    } else {

        pk_motor_set_target(PK_MOTOR_A, (uint16_t)duty);   /* opening */

        pk_motor_set_target(PK_MOTOR_B, 0u);

    }

    s_mot.enabled = true;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

}


void pk_motor_tick_1ms(void)

{

    if (!s_mot.enabled) {

        pk_apply_hw(PK_MOTOR_A, 0u);

        pk_apply_hw(PK_MOTOR_B, 0u);

        return;

    }


    /* Ramp each motor towards its target. */

    for (uint32_t m = 0u; m < 2u; m++) {

        uint16_t tgt = s_mot.target[m];

        uint16_t cur = s_mot.current[m];


        if (cur == tgt) {

            s_mot.stall_ms[m] = 0u;

            continue;

        }


        /* Apply dead-time — if both targets > 0, force lower motor to 0 */

        if (s_mot.target[0] > 0u && s_mot.target[1] > 0u) {

            uint32_t idx_min = (s_mot.target[0] < s_mot.target[1]) ? 0u : 1u;

            s_mot.target[idx_min] = 0u;

        }


        uint16_t step = PK_PWM_RAMP_PER_MS;

        if (cur < tgt) {

            cur = pk_min((uint16_t)(cur + step), tgt);

        } else {

            cur = pk_min((uint16_t)(cur - step), tgt);

            /* underflow guard */

            if (cur > tgt) cur = tgt;

        }


        if (cur == s_mot.current[m]) {

            s_mot.stall_ms[m]++;

            if (s_mot.stall_ms[m] > 100u) {

                pk_motor_emergency_stop();   /* stalled */

            }

        } else {

            s_mot.stall_ms[m] = 0u;

        }


        s_mot.current[m] = cur;

        pk_apply_hw((pk_motor_id_t)m, cur);

    }

}


uint16_t pk_motor_get_duty(pk_motor_id_t motor)

{

    if (motor > PK_MOTOR_B) return 0u;

    return s_mot.current[motor];

}

