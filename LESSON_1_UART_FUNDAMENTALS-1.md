# Lesson 1 — UART Fundamentals (lecture notes)

> These notes are written for embedding inside NotebookLM or any
> RAG-style AI tutor. They go from absolute zero to UART + DMA +
> FreeRTOS so an engineer with no STM32 background can follow.

---

## 0. Why this lesson matters

In the **artificial-knee** project the STM32U585 is the brain.
It does three things:

1. **Acquires** sensor data (load cell, knee angle, battery).
2. **Runs the gait algorithm** — a state machine that decides whether
   the user is standing, walking, sitting, climbing stairs, etc.
3. **Communicates** with the outside world: telemetry to a GUI,
   commands from a clinician, control packets to the damper
   controller.

Communication is via **UART** — Universal Asynchronous
Receiver/Transmitter. UART is the lowest-overhead serial bus that
still gives us full duplex, hardware flow control and the ability to
talk to Bluetooth modules, GPS receivers, GSM modems, and other
controllers.

---

## 1. What is UART?

UART is a hardware peripheral inside the STM32. It exchanges data
**serially** (one bit at a time) using just two wires plus ground:

```
   STM32                            External device
   ┌────────┐                        ┌────────┐
   │     TX ├───────────────────────►│ RX     │
   │     RX │◄───────────────────────┤ TX     │
   │    GND ├───────────────────────┤ GND    │
   └────────┘                        └────────┘
```

* **TX** = Transmit
* **RX** = Receive
* **GND** = common reference

Because there is no shared clock, both ends must agree in advance on:

| Parameter    | Value        |
|--------------|--------------|
| Baud rate    | 115 200      |
| Data bits    | 8            |
| Parity       | None         |
| Stop bits    | 1            |

Written shorthand: **115200 8N1**.

---

## 2. Why asynchronous?

The "A" in UART means **no clock line**. Both sides run their own
baud-rate generator, and the receiver resynchronises on every
**start bit**.

Sending the ASCII letter `A` (0x41) at 115 200 8N1 looks like this
on the wire (idle high, start low, 8 data bits LSB first, stop high):

```
 idle ─┐  ┌──┬──┬──┬──┬──┬──┬──┬──┬──┐   idle
       └──┘  │  │  │  │  │  │  │  │  │
       start 0  1 0 0 0 0 0 1 0   stop
              (LSB first of 0x41)
```

The receiver detects the high-to-low transition, waits 1.5 bit
periods, then samples every bit period. With 115 200 baud one bit is
8.68 µs — easy on a 16 MHz Cortex-M33.

---

## 3. UART peripherals in the STM32U585

The U585 has:

* USART1, USART2, USART3 — full feature set
* UART4, UART5, LPUART1 — basic / low power

For the prosthetic knee we use:

| Peripheral | Purpose              | Pins (default) |
|------------|----------------------|----------------|
| USART1     | Bluetooth telemetry  | PA9 (TX) / PA10 (RX) |
| USART2     | DMC external link    | PA2 (TX) / PA3 (RX)  |

USART2 overlaps with ADC1 IN7 (PA2). On the production PCB you move
USART2 to PD5/PD6.

---

## 4. STM32CubeMX configuration (offline equivalent)

Because we cannot open the .ioc GUI on the offline workstation we
manually initialise the peripheral in C. The configuration is:

```c
huart1.Instance        = USART1;
huart1.Init.BaudRate   = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits   = UART_STOPBITS_1;
huart1.Init.Parity     = UART_PARITY_NONE;
huart1.Init.Mode       = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
HAL_UART_Init(&huart1);
```

This corresponds to the CubeMX panel **USART1 → Mode = Asynchronous,
Baud Rate = 115200, Word Length = 8, Parity = None, Stop Bits = 1**.

---

## 5. HAL transmission — polling style

```c
char msg[] = "Hello World\r\n";
HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
```

`HAL_UART_Transmit` **blocks** the CPU until every byte has been
clocked out or the timeout (100 ms) expires. In a real-time system
this is unacceptable.

---

## 6. HAL transmission — DMA style

```c
HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx, strlen(tx));
```

The DMA controller streams the bytes from RAM to the USART TX
register. The CPU is free the entire time. When the transfer ends an
interrupt fires and you can send the next frame.

---

## 7. HAL reception — polling vs idle-DMA

Polling:

```c
uint8_t rx;
HAL_UART_Receive(&huart1, &rx, 1, HAL_MAX_DELAY);
```

Idle-line DMA — the canonical embedded pattern:

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, sizeof(rx_buf));
```

This:

1. Configures DMA in **circular** mode — DMA continuously fills the
   buffer.
2. Arms the **IDLE** interrupt — fires after one character time of
   silence.
3. Inside the ISR you copy `DMA_get_counter() → newest` bytes into
   your packet decoder.

No CPU cycles wasted while waiting for a packet. CPU only wakes on
packet boundary.

---

## 8. Why FreeRTOS on top of DMA

Even with DMA you still need to:

* send telemetry periodically (50 Hz is too fast to do in a single
  loop)
* react to inbound commands
* monitor safety sensors

FreeRTOS gives us **tasks** for these jobs. A task is just a C
function that runs "concurrently" — the kernel switches between them
every 1 ms tick. Each task can block on a queue or a delay without
busy-waiting.

---

## 9. UART + DMA + FreeRTOS architecture

```
              ┌──────────────────┐
              │  Bluetooth Task  │ (50 Hz)
              └─────┬────────────┘
                    │ xQueueReceive()  ←  pk_uart_pkt_t
                    ▼
              ┌──────────────────┐
              │  USART1 ISR      │ idle-line + DMA circular
              └──────────────────┘
```

The ISR is short. It walks the DMA ring buffer, feeds each byte into
the packet decoder, and on a complete frame it pushes the struct
into a FreeRTOS queue. The Bluetooth task blocks on the queue and
runs only when a real packet arrives.

---

## 10. Telemetry packet (what we actually send)

Every 20 ms we send one frame:

```
 0xAA   0x0A  <10 bytes>   CMD  CRC8  0x55
 start   len   payload     cmd  crc  stop
```

* Start = 0xAA, Stop = 0x55
* Len = 10 (always)
* Cmd = 0x01 (TELEMETRY_PUSH)
* CRC8 covers len + payload + cmd
* Payload layout (little-endian):

| Off | Size | Field         | Type     | Range       |
|-----|------|---------------|----------|-------------|
|  0  |  1   | spool_angle   | uint8_t  | 0..100 %    |
|  1  |  2   | force         | int16_t  | ±200 N      |
|  3  |  2   | knee_angle    | int16_t  | ±200°       |
|  5  |  2   | moment        | int16_t  | ±3000 Nm    |
|  7  |  1   | state_id      | uint8_t  | 0..16       |
|  8  |  2   | battery_mv    | uint16_t | 0..5000 mV  |

On the wire: 1 + 1 + 10 + 1 + 1 + 1 = **15 bytes**.
At 115 200 baud: 1.13 ms — well below the 20 ms cycle.

---

## 11. Summary

* UART is a 2-wire async bus used everywhere in embedded.
* STM32CubeMX configures it via a graphical panel; we replicate
  this by hand in `main.c`.
* `HAL_UART_Transmit_DMA()` and `HAL_UARTEx_ReceiveToIdle_DMA()`
  free the CPU.
* FreeRTOS tasks drain the DMA queues and act on complete packets.
* Our protocol is **Start | Len | Payload | Cmd | CRC8 | Stop** with
  a 10-byte telemetry payload.

---

### Quiz yourself

1. Why do we need a start bit if the line is idle-high?
2. What is the smallest baud rate that gives < 1 ms latency for a
   13-byte frame?
3. Why is idle-DMA better than a fixed-length RX DMA?
4. In our packet format, what happens if the CRC byte is corrupted?

(Answers at the end of Lesson 2.)

