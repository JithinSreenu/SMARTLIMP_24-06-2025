/**
 * @file    uart_dma.h
 * @brief   Dual UART with DMA TX and Receive-To-Idle DMA RX.
 *
 *   USART1 — Bluetooth telemetry (PA9 TX, PA10 RX)
 *   USART2 — DMC external controller (PA2 TX, PA3 RX)
 *
 * Both channels use DMA in circular mode for RX and a double-buffer
 * scheme for TX (one buffer being filled by CPU while the other is
 * being drained by DMA). This is critical on a Cortex-M33: a naive
 * HAL_UART_Transmit_DMA() call blocks if a previous DMA is still
 * in flight, so we queue frames in a FreeRTOS queue and a small
 * dispatcher task drains them in order.
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#include "project_types.h"
#include <stdint.h>

#define PK_UART_TX_QUEUE_LEN   (8u)
#define PK_UART_RX_BUF_LEN     (128u)

typedef enum {
    PK_UART_BT  = 0,   /* USART1 — Bluetooth                  */
    PK_UART_DMC = 1,   /* USART2 — external controller         */
    PK_UART__COUNT
} pk_uart_id_t;

/* Public packet descriptor — pushed by the ISR into the RX queue,
 * consumed by the command tasks. */
typedef struct {
    pk_uart_id_t  port;
    pk_cmd_t      cmd;
    uint8_t       payload[PK_PROTO_MAX_PAYLOAD];
    uint8_t       plen;
} pk_uart_pkt_t;

HAL_StatusTypeDef pk_uart_init(void);

/**
 * @brief  Send a packet via DMA. Non-blocking — frames are queued.
 * @param  which    UART channel
 * @param  frame    byte buffer with full packet (must include
 *                  start/stop/crc — call pk_packet_encode first)
 * @param  len      length in bytes
 * @return HAL_OK on successful enqueue, HAL_BUSY if queue full
 */
HAL_StatusTypeDef pk_uart_send(pk_uart_id_t which,
                               const uint8_t *frame,
                               uint32_t len);

/**
 * @brief  Service routine called by the UART idle-line ISR. Pulls
 *         bytes from the DMA RX buffer into the packet decoder.
 */
void pk_uart_rx_isr(pk_uart_id_t which);

/**
 * @brief  Get the FreeRTOS queue handle used by the dispatcher.
 */
void *pk_uart_rx_queue(pk_uart_id_t which);

#endif /* UART_DMA_H */
