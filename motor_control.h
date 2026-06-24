/**
 * @file    motor_control.h
 * @brief   Dual-DC-motor hydraulic valve controller.
 *
 * Two brushed DC motors drive a hydraulic spool valve in opposite
 * directions to control knee flexion / extension damping.
 *
 *   Motor A  → opens valve (swing / flexion)   TIM3 CH1 (PA6)
 *   Motor B  → closes valve (stance / support) TIM3 CH2 (PA7)
 *
 * Drive H-bridge enable pin: PB0
 * Direction selection is done by setting one PWM channel active
 * and forcing the other channel to 0% duty.
 *
 * Safety notes
 * ------------
 *   - Motors MUST be disabled immediately on FAULT state.
 *   - Duty cycle is ramped linearly to avoid current spikes that
 *     could brown-out the Li-ion battery.
 *   - A 100 ms hard timeout disables the motors if the ramp
 *     never reaches its target (stalled motor detection).
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "project_types.h"
#include <stdint.h>

/* PWM period is 1 kHz (period register value 16000 - 1 with 16 MHz
 * clock). This is inaudible and gives smooth torque. */
#define PK_PWM_PERIOD_TICKS      (16000u)
#define PK_PWM_DEADTIME_TICKS    (400u)   /* 25 µs dead-band between motors */
#define PK_PWM_RAMP_PER_MS       (1u)     /* duty change per ms */

typedef enum {
    PK_MOTOR_A = 0,   /* open valve  — flexion */
    PK_MOTOR_B = 1,   /* close valve — stance  */
} pk_motor_id_t;

typedef enum {
    PK_MOTOR_STOP = 0,
    PK_MOTOR_FWD,
    PK_MOTOR_REV
} pk_motor_dir_t;

HAL_StatusTypeDef pk_motor_init(void);
void              pk_motor_emergency_stop(void);

/**
 * @brief  Set target duty cycle, automatically ramps.
 * @param  motor      motor id
 * @param  duty_ticks target duty in PWM ticks (0..PK_PWM_PERIOD_TICKS)
 */
void              pk_motor_set_target(pk_motor_id_t motor, uint16_t duty_ticks);

/**
 * @brief  Set valve spool angle (0..100 %) — translates to PWM
 *         duty via pk_motor_set_target.
 */
void              pk_motor_set_spool(pk_spool_angle_t pct);

/**
 * @brief  1 kHz periodic tick (called from Motor Control task).
 */
void              pk_motor_tick_1ms(void);

/**
 * @brief  Read back current applied duty (for telemetry).
 */
uint16_t          pk_motor_get_duty(pk_motor_id_t motor);

#endif /* MOTOR_CONTROL_H */
