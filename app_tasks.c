/**

 * @file    app_tasks.c

 * @brief   Implementation of all FreeRTOS tasks (except BT, defined in bluetooth.c

 *          and Safety, defined in safety.c).

 *

 * Educational note — task creation strategy

 * -----------------------------------------

 * Tasks are created in two phases:

 *

 *   1. Static allocation at compile time (configSUPPORT_STATIC_ALLOCATION

 *      and configKERNEL_PROVIDED_STATIC_MEMORY are kept 0 for simplicity).

 *   2. Dynamic creation in pk_boot_start_tasks() called from main()

 *      AFTER the kernel scheduler is started.

 *

 * The reason we don't call xTaskCreate() before vTaskStartScheduler()

 * is that the scheduler doesn't exist yet — calling vTaskStartScheduler()

 * after xTaskCreate() is fine, but it makes the boot order harder

 * to reason about. We instead create everything from the very first

 * idle hook (configKERNEL_PROVIDED is enabled at idle priority).

 *

 * For this firmware we use the simpler approach: create all tasks

 * in main() before starting the scheduler, which works because

 * xTaskCreate() itself doesn't require the scheduler to be running.

 */


#include "app_tasks.h"

#include "adc_manager.h"

#include "motor_control.h"

#include "state_machine.h"

#include "safety.h"

#include "uart_dma.h"

#include "packet_protocol.h"

#include "bluetooth.h"

#include "task.h"

#include "queue.h"

#include <string.h>


/* -------------------------------------------------------------------- */

/*  Globals                                                              */

/* -------------------------------------------------------------------- */


volatile pk_telemetry_t g_telemetry;

pk_sm_ctx_t             g_sm;


/* Conversion constants — set during calibration */

#define PK_BATTERY_VDIV_RATIO_MV   (5000u)   /* full-scale at 5 V */


/* -------------------------------------------------------------------- */

/*  Helper — convert ADC counts to physical units                       */

/* -------------------------------------------------------------------- */


static pk_force_t       raw_to_force       (uint16_t raw) {

    /* 14-bit effective after oversampling, 0.1 N / count,

     * centred at 8192 (half-scale = no load). */

    int32_t c = (int32_t)raw - 8192;

    return (pk_force_t)(c / 8);   /* tune per load cell */

}

static pk_knee_angle_t  raw_to_knee_angle  (uint16_t raw) {

    /* ±90 deg at ±4096 counts → 0.1 deg per 4.55 counts */

    int32_t c = (int32_t)raw - 8192;

    return (pk_knee_angle_t)((c * 100) / 455);   /* 0.1 deg */

}

static pk_battery_mv_t  raw_to_battery_mv  (uint16_t raw) {

    /* 14-bit, full-scale = 5 V = 5000 mV */

    return (pk_battery_mv_t)(((uint32_t)raw * 5000u) / 16384u);

}


/* -------------------------------------------------------------------- */

/*  ADC task — runs at 5 ms (200 Hz), consumes ADC data, fills telemetry */

/* -------------------------------------------------------------------- */


void vAdcTask(void *pv)

{

    (void)pv;

    const TickType_t period = pdMS_TO_TICKS(5u);

    TickType_t last = xTaskGetTickCount();


    for (;;) {

        vTaskDelayUntil(&last, period);


        pk_adc_data_t d;

        if (pk_adc_get(&d)) {

            pk_telemetry_t t;

            t.spool_angle = g_sm.current_spool;

            t.force       = raw_to_force     ((uint16_t)d.filt[PK_ADC_CH_FORCE]);

            t.knee_angle  = raw_to_knee_angle((uint16_t)d.filt[PK_ADC_CH_ANGLE]);

            /* Moment is approximated as force * arm-length (placeholder). */

            t.moment      = (pk_moment_t)(t.force * 15);  /* 0.01 Nm */

            t.state_id    = (pk_state_id_t)g_sm.current;

            t.battery_mv  = raw_to_battery_mv((uint16_t)d.filt[PK_ADC_CH_BATTERY]);

            g_telemetry = t;

        }


        /* Drive the state machine */

        pk_sm_step(&g_sm,

                   &d,

                   raw_to_knee_angle((uint16_t)d.filt[PK_ADC_CH_ANGLE]));


        pk_safety_heartbeat(PK_TASK_ADC);

    }

}


/* -------------------------------------------------------------------- */

/*  Motor task — 1 ms tick                                              */

/* -------------------------------------------------------------------- */


void vMotorTask(void *pv)

{

    (void)pv;

    const TickType_t period = pdMS_TO_TICKS(1u);

    TickType_t last = xTaskGetTickCount();


    for (;;) {

        vTaskDelayUntil(&last, period);

        pk_motor_tick_1ms();

        pk_safety_heartbeat(PK_TASK_MOTOR);

    }

}


/* -------------------------------------------------------------------- */

/*  DMC Comms task — drains USART2 RX queue                             */

/* -------------------------------------------------------------------- */


void vCommsTask(void *pv)

{

    (void)pv;

    QueueHandle_t q = (QueueHandle_t)pk_uart_rx_queue(PK_UART_DMC);


    for (;;) {

        pk_uart_pkt_t pkt;

        if (xQueueReceive(q, &pkt, portMAX_DELAY) == pdTRUE) {

            /* Stub — the DMC protocol is application-specific.

             * Typical actions: relay state, log event, etc. */

            switch (pkt.cmd) {

            case PK_CMD_GET_STATE: {

                uint8_t buf[1] = { (uint8_t)g_sm.current };

                uint8_t frame[PK_PROTO_MAX_FRAME];

                uint32_t n = pk_packet_encode(PK_CMD_GET_STATE,

                                              buf, 1u, frame);

                (void)pk_uart_send(PK_UART_DMC, frame, n);

                break;

            }

            default:

                break;

            }

        }

        pk_safety_heartbeat(PK_TASK_COMMS);

    }

}


/* -------------------------------------------------------------------- */

/*  Watchdog task — very low priority, just toggles a debug LED         */

/* -------------------------------------------------------------------- */


void vWatchdogTask(void *pv)

{

    (void)pv;

    for (;;) {

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);     /* LD1 heartbeat */

        vTaskDelay(pdMS_TO_TICKS(500u));

        pk_safety_heartbeat(PK_TASK_WDT);

    }

}


/* -------------------------------------------------------------------- */

/*  Boot — create all tasks before scheduler starts                     */

/* -------------------------------------------------------------------- */


void pk_boot_start_tasks(void)

{

    /* Module init */

    (void)pk_adc_init();

    (void)pk_motor_init();

    (void)pk_uart_init();

    pk_packet_decoder_reset();

    pk_sm_init(&g_sm);

    pk_safety_init();


    /* Calibrate ADC */

    pk_adc_calibrate();

    (void)pk_adc_start();


    /* Tasks */

    xTaskCreate(vSafetyTask,   "SAFETY", 256, NULL, PK_PRIO_SAFETY, NULL);

    xTaskCreate(vMotorTask,    "MOTOR",  256, NULL, PK_PRIO_MOTOR,  NULL);

    xTaskCreate(vAdcTask,      "ADC",    256, NULL, PK_PRIO_ADC,    NULL);

    xTaskCreate(vCommsTask,    "COMMS",  256, NULL, PK_PRIO_COMMS,  NULL);

    xTaskCreate(vBluetoothTask,"BT",     256, NULL, PK_PRIO_BT,     NULL);

    xTaskCreate(vWatchdogTask, "WDT",    128, NULL, PK_PRIO_WDT,    NULL);

}

