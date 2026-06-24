/**

 * @file    safety.c

 * @brief   Safety monitor + watchdog.

 */


#include "safety.h"

#include "state_machine.h"

#include "motor_control.h"

#include "stm32u5xx_hal.h"

#include "task.h"


extern IWDG_HandleTypeDef hiwdg1;

extern pk_sm_ctx_t        g_sm;

extern volatile pk_telemetry_t g_telemetry;


/* -------------------------------------------------------------------- */

/*  Per-task heartbeat counters                                          */

/* -------------------------------------------------------------------- */


static volatile uint32_t g_task_heartbeat[PK_TASK__COUNT] = {0};

static volatile uint32_t g_task_last_seen[PK_TASK__COUNT]  = {0};


void pk_safety_heartbeat(pk_task_id_t id)

{

    if (id >= PK_TASK__COUNT) return;

    g_task_heartbeat[id] = xTaskGetTickCount();

}


/* -------------------------------------------------------------------- */

/*  Init                                                                 */

/* -------------------------------------------------------------------- */


void pk_safety_init(void)

{

    /* HAL_IWDG_Init is done in main.c. We just seed the baselines. */

    uint32_t now = xTaskGetTickCount();

    for (uint32_t i = 0u; i < PK_TASK__COUNT; i++) {

        g_task_heartbeat[i] = now;

        g_task_last_seen[i]  = now;

    }

}


/* -------------------------------------------------------------------- */

/*  Task                                                                 */

/* -------------------------------------------------------------------- */


void pk_safety_task(void *pv)

{

    (void)pv;

    const TickType_t period = pdMS_TO_TICKS(1u);

    TickType_t last = xTaskGetTickCount();


    for (;;) {

        vTaskDelayUntil(&last, period);


        /* ----- (1) Heartbeat check ----- */

        uint32_t now = xTaskGetTickCount();

        for (uint32_t i = 0u; i < PK_TASK__COUNT; i++) {

            if (i == PK_TASK_SAFETY) continue;       /* we are this task */

            uint32_t since = now - g_task_heartbeat[i];

            /* Each task has a maximum allowed silence. Conservative

             * 5x the nominal period — anything beyond = hung task. */

            uint32_t max_silence = 0u;

            switch (i) {

            case PK_TASK_MOTOR:  max_silence = 10u;  break;

            case PK_TASK_ADC:    max_silence = 25u;  break;

            case PK_TASK_COMMS:  max_silence = 50u;  break;

            case PK_TASK_BT:     max_silence = 100u; break;

            case PK_TASK_WDT:    max_silence = 200u; break;

            default:             max_silence = 100u; break;

            }

            if (since > max_silence) {

                pk_sm_raise_fault(&g_sm, PK_FAULT_WATCHDOG);

            }

        }


        /* ----- (2) Battery sanity ----- */

        if (g_telemetry.battery_mv < 3000u) {        /* 3.0 V cutoff */

            pk_sm_raise_fault(&g_sm, PK_FAULT_BATTERY_CRITICAL);

        } else if (g_telemetry.battery_mv < 3400u) {

            pk_sm_raise_fault(&g_sm, PK_FAULT_BATTERY_LOW);

        }


        /* ----- (3) Refresh IWDG ----- */

        if (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK) {

            /* Cannot refresh — fault and stop */

            pk_motor_emergency_stop();

            NVIC_SystemReset();

        }


        pk_safety_heartbeat(PK_TASK_SAFETY);

    }

}

