/**
 * @file    app_tasks.h
 * @brief   FreeRTOS task entry points and shared globals.
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "project_types.h"

/* Task priorities — numerically higher = more important.
 * configMAX_SYSCALL_INTERRUPT_PRIORITY must be respected, so we
 * keep all task priorities <= configMAX_SYSCALL_INTERRUPT_PRIORITY. */
#define PK_PRIO_SAFETY   (configMAX_PRIORITIES - 1)
#define PK_PRIO_MOTOR    (configMAX_PRIORITIES - 2)
#define PK_PRIO_ADC      (configMAX_PRIORITIES - 3)
#define PK_PRIO_COMMS    (configMAX_PRIORITIES - 4)
#define PK_PRIO_BT       (configMAX_PRIORITIES - 5)
#define PK_PRIO_WDT      (tskIDLE_PRIORITY + 1)

/* Shared telemetry snapshot — written by ADC task, read by BT. */
extern volatile pk_telemetry_t g_telemetry;

/* Global state machine context */
extern pk_sm_ctx_t g_sm;

/* Task prototypes */
void vSafetyTask (void *pv);   /* defined in safety.c */
void vMotorTask  (void *pv);
void vAdcTask    (void *pv);
void vCommsTask  (void *pv);
void vBluetoothTask(void *pv); /* defined in bluetooth.c */
void vWatchdogTask(void *pv);

/* Boot sequence — defined in main.c */
void pk_boot_start_tasks(void);

#endif /* APP_TASKS_H */
