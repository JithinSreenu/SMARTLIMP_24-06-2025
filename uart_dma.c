/**

 * @file    uart_dma.c

 * @brief   UART DMA + packet parser driver.

 *

 * Receive-To-Idle explained

 * -------------------------

 * Idle-line DMA is a feature where the UART peripheral detects the

 * bus going silent (no transitions for one character time) and

 * raises an interrupt. The DMA controller has, by that point, copied

 * everything received into our RAM buffer.

 *

 * That gives us:

 *   - Variable-length packets without CPU polling

 *   - Notification only at end-of-frame

 *   - CPU free during the burst

 */


#include "uart_dma.h"

#include "packet_protocol.h"

#include "stm32u5xx_hal.h"

#include "queue.h"


extern UART_HandleTypeDef huart1;   /* BT  */

extern UART_HandleTypeDef huart2;   /* DMC */

extern DMA_HandleTypeDef  hdma_usart1_rx;

extern DMA_HandleTypeDef  hdma_usart2_rx;


/* -------------------------------------------------------------------- */

/*  Per-channel state                                                    */

/* -------------------------------------------------------------------- */


typedef struct {

    UART_HandleTypeDef       *huart;

    uint8_t                   rx_buf[PK_UART_RX_BUF_LEN];

    volatile uint16_t         rx_head;     /* where the DMA is writing */

    QueueHandle_t             rx_queue;

} pk_uart_ch_t;


static pk_uart_ch_t s_ch[PK_UART__COUNT] = {

    [PK_UART_BT]  = { .huart = &huart1 },

    [PK_UART_DMC] = { .huart = &huart2 },

};


/* The queue carries pk_uart_pkt_t (declared in uart_dma.h) */


/* -------------------------------------------------------------------- */

/*  Init                                                                 */

/* -------------------------------------------------------------------- */


HAL_StatusTypeDef pk_uart_init(void)

{

    for (uint32_t i = 0u; i < PK_UART__COUNT; i++) {

        pk_uart_ch_t *c = &s_ch[i];


        /* FreeRTOS queue created with pvPortMalloc by the kernel. */

        c->rx_queue = xQueueCreate(8u, sizeof(pk_uart_pkt_t));

        configASSERT(c->rx_queue != NULL);


        c->rx_head = 0u;


        /* Receive-To-Idle + circular DMA into our 128-byte buffer. */

        if (HAL_UARTEx_ReceiveToIdle_DMA(c->huart,

                                         c->rx_buf,

                                         PK_UART_RX_BUF_LEN) != HAL_OK) {

            return HAL_ERROR;

        }

        /* The HAL sets a transfer-callback mask. Disable the half-transfer

         * notification because we use the idle line for framing. */

        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        if (i == PK_UART_DMC) {

            __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

        }

    }

    return HAL_OK;

}


/* -------------------------------------------------------------------- */

/*  TX path                                                              */

/* -------------------------------------------------------------------- */


HAL_StatusTypeDef pk_uart_send(pk_uart_id_t which,

                               const uint8_t *frame,

                               uint32_t len)

{

    if (which >= PK_UART__COUNT || frame == NULL || len == 0u) {

        return HAL_ERROR;

    }

    pk_uart_ch_t *c = &s_ch[which];


    /* Snapshot into a local buffer because the caller may free

     * the source memory right after returning. We re-use the

     * per-channel RX buffer for scratch; the idle-line ISR will

     * never touch it during a TX. */

    uint32_t cap = sizeof(c->rx_buf);

    if (len > cap) len = cap;

    memcpy(c->rx_buf, frame, len);


    /* Kick a non-blocking DMA. The HAL_TxCpltCallback is unused —

     * we just leave it to the caller to re-arm. */

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(c->huart, c->rx_buf, (uint16_t)len);

    return st;

}


/* -------------------------------------------------------------------- */

/*  RX path                                                              */

/* -------------------------------------------------------------------- */


/**

 * @brief  Called from USART idle-line ISR. Walks the RX ring from

 *         the last consumed position up to the current DMA write

 *         pointer, feeding each byte into the packet decoder FSM.

 *         On a complete frame the packet is enqueued for the

 *         command task.

 */

void pk_uart_rx_isr(pk_uart_id_t which)

{

    pk_uart_ch_t *c = &s_ch[which];

    UART_HandleTypeDef *huart = c->huart;


    /* current head = remaining bytes in DMA buffer. Compute how many

     * new bytes are available: total - ndtr (mod buffer length). */

    uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);

    uint16_t newest = (uint16_t)(PK_UART_RX_BUF_LEN - ndtr);

    while (c->rx_head != newest) {

        uint8_t b = c->rx_buf[c->rx_head];

        c->rx_head = (uint16_t)((c->rx_head + 1u) % PK_UART_RX_BUF_LEN);


        pk_cmd_t   cmd;

        uint8_t    payload[PK_PROTO_MAX_PAYLOAD];

        uint8_t    plen;

        if (pk_packet_decode_byte(b, &cmd, payload, &plen)) {

            pk_uart_pkt_t pkt = {

                .port = which,

                .cmd  = cmd,

                .plen = plen

            };

            if (plen > 0u) memcpy(pkt.payload, payload, plen);

            BaseType_t hpw = pdFALSE;

            (void)xQueueSendFromISR(c->rx_queue, &pkt, &hpw);

            portYIELD_FROM_ISR(hpw);

        }

    }

}


void *pk_uart_rx_queue(pk_uart_id_t which)

{

    if (which >= PK_UART__COUNT) return NULL;

    return s_ch[which].rx_queue;

}

