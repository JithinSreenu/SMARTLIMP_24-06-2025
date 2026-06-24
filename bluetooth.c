/**

 * @file    bluetooth.c

 * @brief   Bluetooth telemetry driver.

 *

 * The BT task does two things:

 *

 *   1. Every 20 ms it pulls the latest pk_telemetry_t from the

 *      shared telemetry structure and pushes it through the packet

 *      encoder to the BT UART (DMA, non-blocking).

 *

 *   2. It drains the BT RX queue for incoming calibration / config

 *      commands.

 *

 * Why 50 Hz and not 100 Hz?

 *   50 Hz covers the bandwidth of human gait dynamics (swing phase

 *   is ~0.4 s, so Nyquist needs ~5 Hz, 50 Hz leaves 5x oversampling

 *   for the filter rolloff). Going higher just wastes BT bandwidth.

 */


#include "bluetooth.h"

#include "packet_protocol.h"

#include "uart_dma.h"

#include "task.h"

#include <string.h>


/* External shared telemetry snapshot — written by the ADC task,

 * read by this task. Single-writer / single-reader so no mutex

 * is needed. We mark it volatile because the writer task may

 * preempt us. */

extern volatile pk_telemetry_t g_telemetry;


/* -------------------------------------------------------------------- */

/*  Encode a telemetry payload                                          */

/* -------------------------------------------------------------------- */


static uint8_t s_tx_buf[PK_PROTO_MAX_FRAME];


static uint32_t encode_telemetry(const pk_telemetry_t *t)

{

    uint8_t payload[sizeof(pk_telemetry_t)];

    uint32_t i = 0u;


    /* Little-endian explicit byte packing — keeps the code portable

     * across GCC arm-none-eabi endian configurations and avoids

     * any struct-endianness pitfall. */

    payload[i++] = t->spool_angle;

    payload[i++] = (uint8_t)(t->force & 0xFFu);

    payload[i++] = (uint8_t)((t->force >> 8) & 0xFFu);

    payload[i++] = (uint8_t)(t->knee_angle & 0xFFu);

    payload[i++] = (uint8_t)((t->knee_angle >> 8) & 0xFFu);

    payload[i++] = (uint8_t)(t->moment & 0xFFu);

    payload[i++] = (uint8_t)((t->moment >> 8) & 0xFFu);

    payload[i++] = t->state_id;

    payload[i++] = (uint8_t)(t->battery_mv & 0xFFu);

    payload[i++] = (uint8_t)((t->battery_mv >> 8) & 0xFFu);


    return pk_packet_encode(PK_CMD_TELEMETRY_PUSH, payload, (uint8_t)i, s_tx_buf);

}


/* -------------------------------------------------------------------- */

/*  Inbound command handler                                              */

/* -------------------------------------------------------------------- */


static void handle_command(pk_cmd_t cmd, const uint8_t *pld, uint8_t plen)

{

    switch (cmd) {

    case PK_CMD_GET_STATE:

        /* Reply with current state via a TELEMETRY push */

        (void)cmd; (void)pld; (void)plen;

        break;


    case PK_CMD_CALIBRATE_ZERO:

        /* Hook for zero-load calibration of the load cell.

         * Implementation lives in state_machine.c. */

        break;


    case PK_CMD_CALIBRATE_SCALE:

        /* Hook for full-scale calibration. */

        break;


    case PK_CMD_BATTERY_QUERY:

        /* Reply is the next telemetry frame, which is automatically

         * pushed every 20 ms — no extra reply needed. */

        break;


    default:

        break;

    }

}


/* -------------------------------------------------------------------- */

/*  Task                                                                 */

/* -------------------------------------------------------------------- */


void vBluetoothTask(void *pv)

{

    (void)pv;

    QueueHandle_t q = (QueueHandle_t)pk_uart_rx_queue(PK_UART_BT);


    const TickType_t period = pdMS_TO_TICKS(20u);   /* 50 Hz */

    TickType_t last = xTaskGetTickCount();


    for (;;) {

        vTaskDelayUntil(&last, period);


        /* Push telemetry */

        uint32_t n = encode_telemetry((const pk_telemetry_t *)&g_telemetry);

        (void)pk_uart_send(PK_UART_BT, s_tx_buf, n);


        /* Drain any incoming commands without blocking */

        pk_uart_pkt_t pkt;

        while (xQueueReceive(q, &pkt, 0) == pdTRUE) {

            handle_command(pkt.cmd, pkt.payload, pkt.plen);

        }

    }

}

