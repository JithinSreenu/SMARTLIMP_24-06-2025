/**
 * @file    safety.h
 * @brief   Safety / fault monitor + IWDG refresh.
 *
 * The safety task runs at the highest priority. Every 1 ms it:
 *
 *   1. Validates sensor ranges (force / angle / battery).
 *   2. Checks that the most recent ADC DMA completed within 5 ms.
 *   3. Checks the FreeRTOS heartbeat of every other task
 *      (each task updates g_task_heartbeat[ID] before delaying).
 *   4. If anything is wrong, transitions to FAULT and disables motors.
 *   5. If everything is fine, refreshes the IWDG.
 *
 * The IWDG (independent watchdog) is a separate LSI-oscillator-driven
 * counter that resets the MCU if not refreshed. We refresh it only
 * from the safety task — a single, audited source.
 */

#ifndef SAFETY_H
#define SAFETY_H

#include "project_types.h"

typedef enum {
    PK_TASK_SAFETY  = 0,
    PK_TASK_MOTOR   = 1,
    PK_TASK_ADC     = 2,
    PK_TASK_COMMS   = 3,
    PK_TASK_BT      = 4,
    PK_TASK_WDT     = 5,
    PK_TASK__COUNT
} pk_task_id_t;

void                pk_safety_init(void);
void                pk_safety_task(void *pv);

/* Each task calls this just before blocking on a queue / delay. */
void                pk_safety_heartbeat(pk_task_id_t id);

#endif /* SAFETY_H */
